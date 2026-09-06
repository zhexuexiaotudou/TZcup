from pathlib import Path

from sanitation_formal_campus_integration.dynamic_footprint_core import (
    ARM_JOINTS,
    ARM_STOWED,
    load_footprints,
    normalize_exact_polygon,
    polygons_exactly_equal,
    profile_decision,
    run_with_fail_closed_cleanup,
    select_profile,
)

ROOT = Path(__file__).resolve().parents[4]


def _stowed() -> dict[str, float]:
    return dict(zip(ARM_JOINTS, ARM_STOWED, strict=True))


def test_fail_closed_until_all_arm_joints_are_known() -> None:
    assert select_profile({}, False) == "arm_deployed"


def test_arm_inhibit_or_joint_motion_selects_arm_envelope() -> None:
    joints = _stowed()
    assert select_profile(joints, True) == "arm_deployed"
    joints["elbow_joint"] += 0.2
    assert select_profile(joints, False) == "arm_deployed"


def test_profile_decision_keeps_the_fail_closed_trigger_reason() -> None:
    assert profile_decision({}, False) == (
        "arm_deployed",
        "arm_state_unknown_or_not_stowed",
    )
    assert profile_decision(_stowed(), True) == (
        "arm_deployed",
        "base_motion_inhibited",
    )


def test_cleaning_and_transport_profiles_follow_lift_position() -> None:
    joints = _stowed()
    joints["cleaning_lift_joint"] = 0.100
    assert select_profile(joints, False) == "cleaning_deployed"
    joints["cleaning_lift_joint"] = 0.00
    assert select_profile(joints, False) == "transport_stowed"


def test_missing_or_nonfinite_cleaning_lift_never_shrinks_to_transport() -> None:
    joints = _stowed()
    assert profile_decision(joints, False) == (
        "cleaning_deployed",
        "cleaning_lift_unknown_or_nonfinite",
    )
    joints["cleaning_lift_joint"] = float("nan")
    assert profile_decision(joints, False) == (
        "cleaning_deployed",
        "cleaning_lift_unknown_or_nonfinite",
    )


def test_fail_closed_cleanup_preserves_primary_failure_and_adds_cleanup_note() -> None:
    def fail_operation() -> None:
        raise RuntimeError("primary readback failure")

    def fail_cleanup() -> None:
        raise RuntimeError("override clear failure")

    try:
        run_with_fail_closed_cleanup(fail_operation, fail_cleanup)
    except RuntimeError as error:
        assert str(error) == "primary readback failure"
        assert error.__notes__ == [
            "fail-closed cleanup also failed: RuntimeError: override clear failure"
        ]
    else:  # pragma: no cover - assertion guard
        raise AssertionError("primary failure must remain visible")


def test_fail_closed_cleanup_failure_is_fatal_after_success() -> None:
    def fail_cleanup() -> None:
        raise RuntimeError("override clear failure")

    try:
        run_with_fail_closed_cleanup(lambda: "PASS", fail_cleanup)
    except RuntimeError as error:
        assert str(error) == "override clear failure"
    else:  # pragma: no cover - assertion guard
        raise AssertionError("cleanup failure must fail the gate")


def test_formal_profile_defines_all_runtime_envelopes() -> None:
    profile = load_footprints(
        ROOT / "config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml"
    )
    assert set(profile) == {"transport_stowed", "cleaning_deployed", "arm_deployed"}
    assert max(y for _, y in profile["cleaning_deployed"]) > max(
        y for _, y in profile["transport_stowed"]
    )
    assert profile["transport_stowed"] == [
        [0.620, 0.675],
        [0.620, -0.675],
        [-0.540, -0.675],
        [-0.540, 0.675],
    ]
    assert profile["cleaning_deployed"] == [
        [0.620, 0.695],
        [0.620, -0.695],
        [-0.540, -0.695],
        [-0.540, 0.695],
    ]
    assert profile["arm_deployed"] == [
        [1.300, 1.050],
        [1.300, -1.280],
        [-1.100, -1.280],
        [-1.100, 1.050],
    ]


def test_exact_polygon_comparison_uses_shared_point32_normalization() -> None:
    yaml_polygon = [[0.620, 0.675], [0.620, -0.675], [-0.540, -0.675]]
    wire_polygon = normalize_exact_polygon(yaml_polygon)
    assert polygons_exactly_equal(wire_polygon, yaml_polygon)
    assert not polygons_exactly_equal(
        wire_polygon,
        [[0.621, 0.675], [0.620, -0.675], [-0.540, -0.675]],
    )
    assert not polygons_exactly_equal(list(reversed(wire_polygon)), yaml_polygon)


def test_static_message_contract_and_runtime_gate_are_explicit() -> None:
    package = Path(__file__).resolve().parents[1]
    manager_source = (
        package / "sanitation_formal_campus_integration/dynamic_footprint_manager.py"
    ).read_text(encoding="utf-8")
    gate_source = (
        package / "sanitation_formal_campus_integration/dynamic_footprint_runtime_gate.py"
    ).read_text(encoding="utf-8")
    launch_source = (package / "launch/formal_campus.launch.py").read_text(
        encoding="utf-8"
    )
    lifecycle_source = (
        package / "launch/formal_campus_map_lifecycle.launch.py"
    ).read_text(encoding="utf-8")
    safety_source = (
        package.parent
        / "sanitation_safety/sanitation_safety/whole_vehicle_safety_manager.py"
    ).read_text(encoding="utf-8")
    assert "from geometry_msgs.msg import Point32, Polygon" in manager_source
    assert "PolygonStamped" not in manager_source
    assert "Polygon, \"/local_costmap/footprint\"" in manager_source
    assert "Polygon, \"/global_costmap/footprint\"" in manager_source
    assert '"reason": reason' in manager_source
    assert 'declare_parameter("enable_runtime_test_override", False)' in manager_source
    assert "if self._runtime_test_override_enabled:" in manager_source
    assert "runtime_test_override" in manager_source
    assert '"motion_authorized": motion_authorized' in manager_source
    assert "get_publishers_info_by_topic" in gate_source
    assert "get_subscriptions_info_by_topic" in gate_source
    assert "def _only_node" in gate_source
    assert "exclusive_manager_publisher" in gate_source
    assert "polygons_exactly_equal" in gate_source
    assert "independent_safety_subscriber" in gate_source
    assert "PolygonStamped" in gate_source
    assert "runtime_test_override:opt_in_manager_subscriber" in gate_source
    assert "published_topic}:{costmap_node}_publisher" in gate_source
    assert "runtime_test_nonce" in gate_source
    assert "fresh exact Nav2 published_footprint" in gate_source
    assert 'SAFETY_STATUS_TOPIC = "/safety/status_json"' in gate_source
    assert 'ROOT_NAMESPACE = "/"' in gate_source
    assert 'costmap_namespace = f"/{costmap_node}"' in gate_source
    assert 'NO_PUBLISH_THREAD_ERROR = "none"' in gate_source
    assert "node_namespace == node_namespace" in gate_source
    assert "exclusive_safety_manager_publisher" in gate_source
    assert "status_publish_count" in gate_source
    assert '"BASE_COMMAND_STOPPED"' in gate_source
    assert '"manipulator_base_inhibit"' in gate_source
    assert '"publish_thread_error"' in gate_source
    assert "!= NO_PUBLISH_THREAD_ERROR" in gate_source
    assert '"publish_thread_error": (' in safety_source
    assert '"none"\n                if self._publish_thread_error is None' in safety_source
    assert '"/joint_states"' not in gate_source
    assert "JointState" not in gate_source
    assert "Bool(data=False)" not in gate_source
    assert "cmd_vel" not in gate_source
    assert '"enable_dynamic_footprint_runtime_test_override"' in launch_source
    assert 'default_value="false"' in launch_source
    assert '"enable_runtime_test_override": ParameterValue(' in launch_source
    assert "value_type=bool" in launch_source
    assert '"enable_dynamic_footprint_runtime_test_override"' in lifecycle_source
    assert 'default_value="false"' in lifecycle_source
    assert lifecycle_source.count("enable_dynamic_footprint_runtime_test_override") >= 3
