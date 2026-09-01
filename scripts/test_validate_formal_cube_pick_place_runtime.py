from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VALIDATOR = ROOT / "scripts" / "validate_formal_cube_pick_place_runtime.py"
RUNNER = ROOT / "scripts" / "run_formal_cube_pick_place_runtime.sh"
LAUNCH = ROOT / "starter_ws" / "src" / "sanitation_manipulation" / "launch" / "formal_cube_pick_place.launch.py"
WRAPPER = ROOT / "starter_ws" / "src" / "sanitation_manipulation" / "urdf" / "formal_manipulation_acceptance.urdf.xacro"
CONTACT_GATE = ROOT / "starter_ws" / "src" / "sanitation_gazebo_control" / "src" / "GripperContactGateSystem.cc"
CHUTE = ROOT / "starter_ws" / "src" / "sanitation_vehicle_description" / "urdf" / "high_fidelity" / "storage_system.xacro"
BODYWORK = ROOT / "starter_ws" / "src" / "sanitation_vehicle_description" / "urdf" / "high_fidelity" / "bodywork.xacro"


def test_product_launch_has_no_payload_or_delete_command_bridge() -> None:
    source = LAUNCH.read_text(encoding="utf-8")
    assert "payload/dry_mass_kg" not in source
    assert "/remove@" not in source
    assert "/set_pose@ros_gz_interfaces/srv/SetEntityPose" in source
    # preserveFixedJoint keeps the evaluator floor sensor on dry_bin_link;
    # fail closed if the bridge drifts back to the pre-preservation base lump.
    assert "/link/dry_bin_link/sensor/dry_bin_floor_contact/contact" in source
    assert "/link/base_footprint/sensor/dry_bin_floor_contact/contact" not in source


def test_attachment_is_fingertip_contact_gated_in_validator() -> None:
    source = VALIDATOR.read_text(encoding="utf-8")
    assert "left_cube_contacts <= 0" in source
    assert "right_cube_contacts <= 0" in source
    assert "self.latest.get(GRIPPER_JOINT, 0.0) < 0.20" in source
    assert "horizontal_offset_m > 0.050" in source
    assert "not 0.12 < vertical_offset_m < 0.21" in source
    assert "self.attach.publish(Empty())" in source
    assert "handle.cancel_goal_async()" in source
    assert '"passed": False' in source
    assert '"passed": True' in source
    assert source.index("left_cube_contacts <= 0") < source.index("self.attach.publish(Empty())")
    assert "SetEntityPose is evaluator-only and forbidden after task start" in source
    assert "DeleteEntity" not in source
    assert "payload/dry_mass_kg" not in source


def test_physical_cube_is_not_removed_or_double_counted() -> None:
    source = VALIDATOR.read_text(encoding="utf-8")
    assert '"present_after_deposit": True' in source
    assert '"dynamic_dry_payload_added_kg": 0.0' in source
    assert '"physical_material_mass_kg": material["mass_kg"]' in source
    assert '"generated_sdf_inertial": sdf_inertial' in source
    assert "node.bin_support_contacts <= 0" in source
    assert '"/storage/dry_bin/floor_contact"' in source
    assert "node.spin_sim_for(3.0" in source
    assert "node.spin_sim_for(1.0" in source
    assert '"support_contact_count": node.bin_support_contacts' in source
    assert '"bin_floor_support_z_m": BIN_FLOOR_SUPPORT_Z_M' in source
    assert '"delete_entity_calls": 0' in source
    assert '"physical_cube_deleted_after_deposit": False' in source
    assert '"aggregation_or_reserve_payload_substitution": False' in source


def test_released_pet_cube_is_hard_gated_by_bridged_dry_bin_instrumentation() -> None:
    source = VALIDATOR.read_text(encoding="utf-8")
    runner = RUNNER.read_text(encoding="utf-8")
    topic = "/model/tzcup_formal_sanitation_vehicle/dry_bin/status_json"
    assert f'DRY_BIN_STATUS_TOPIC = "{topic}"' in source
    assert "self.create_subscription(String, DRY_BIN_STATUS_TOPIC" in source
    assert 'status.get("sensor_ready") is not True' in source
    assert 'status["contained_object_count"]' in source
    assert 'status["contained_mass_kg"]' in source
    assert "DRY_BIN_MASS_TOLERANCE_KG = 1e-5" in source
    assert 'status.get("full") is not False' in source
    assert 'expected_count=1' in source
    assert 'expected_mass_kg=material["mass_kg"]' in source
    assert '"post_release_sample_observed": True' in source
    assert topic in runner
    assert "std_msgs/msg/String[gz.msgs.StringMsg" in runner
    assert "dry_bin_bridge_pid" in runner


def test_stale_single_cube_canonical_artifact_is_superseded_before_preflight() -> None:
    runner = RUNNER.read_text(encoding="utf-8")
    supersede = 'mv -- "${output}" "${output}.superseded.$(date -u +%Y%m%dT%H%M%SZ).$$"'
    assert supersede in runner
    assert runner.index(supersede) < runner.index("source /opt/ros/jazzy/setup.bash")
    assert runner.index(supersede) < runner.index(
        'if [[ ! -f "${runtime_ws}/install/setup.bash" ]]; then'
    )


def test_acceptance_wrapper_uses_real_contact_and_detachable_joint() -> None:
    source = WRAPPER.read_text(encoding="utf-8")
    assert "left_finger_tip_contact" in source
    assert "right_finger_tip_contact" in source
    assert "dry_bin_floor_contact" in source
    assert "<preserveFixedJoint>true</preserveFixedJoint>" in source
    assert "<collision>dry_bin_link_fixed_joint_lump__dry_floor_collision_collision</collision>" in source
    assert "sanitation_gazebo_control::GripperContactGateSystem" in source
    assert "<parent_link>ur5e_wrist_3_link</parent_link>" in source
    assert "<child_model>material_cube</child_model>" not in source
    assert "<attach_topic>/manipulation/grasp/attach</attach_topic>" in source

    gate = CONTACT_GATE.read_text(encoding="utf-8")
    assert "components::DetachableJointInfo" in gate
    assert "contactedCollision" in gate
    assert "leftExternal" in gate and "rightExternal" in gate
    assert '"material_cube"' not in gate and '"object_"' not in gate
    assert "PublishAttachmentState(true)" in gate
    assert "RequestRemoveEntity" in gate


def test_dry_chute_has_four_walls_and_open_bore() -> None:
    source = CHUTE.read_text(encoding="utf-8")
    for wall in ("front", "rear", "left", "right"):
        assert f"dry_deposit_chute_{wall}_wall_collision" in source
    assert "leave both ends open" in source
    assert "dry_deposit_chute_collision\"><geometry><box size=\"0.130 0.105 0.350\"" not in source


def test_rear_shell_lower_frame_does_not_create_a_false_bin_floor() -> None:
    source = BODYWORK.read_text(encoding="utf-8")
    assert 'name="rear_shell_lower_collision"' not in source
    assert 'name="rear_shell_lower_left_skirt_collision"' in source
    assert 'name="rear_shell_lower_right_skirt_collision"' in source
    assert 'name="rear_shell_lower_rear_rail_collision"' in source
    assert '<box size="0.53 0.68 0.23"' not in source
