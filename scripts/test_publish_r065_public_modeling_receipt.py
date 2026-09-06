import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/publish_r065_public_modeling_receipt.py"
SPEC = importlib.util.spec_from_file_location("r065_receipt", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

WALKER_IDS = [f"walker_{index}" for index in range(8)]
WALKER_RADII = {identity: 0.25 for identity in WALKER_IDS}


def _write(path: Path, value: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")
    return path


def _w2() -> dict:
    return {
        "runtime_gate": "moveit_ground_collision", "passed": True,
        "executor_or_controller_commands_sent": False, "truth_used_for_control": False,
        "normal_collision_checked_ik": {"pick": 1},
        "normal_anchor_state_valid": {"pick": True},
        "normal_pick_cartesian_fraction": 1.0,
        "below_ground_ground_contacts": [{"body_1": "ground", "body_2": "ur5e_forearm_link"}],
        "scene_revisions": {"initial": 1, "after_ground_restore": 2,
                            "after_perceived_cube_add": 3, "after_perceived_cube_remove": 4},
        "ground_removal_preserved_non_ground_world_and_acm": True,
        "ground_removal_used_robot_state_diff_only": True,
    }


def _w1() -> dict:
    live_padding = {
        "/local_costmap/footprint": 0.009999999776482582,
        "/global_costmap/footprint": 0.009999999776482582,
    }
    return {
        "result": "PASS", "passed": True, "runtime_only": True,
        "input_type": "geometry_msgs/msg/Polygon",
        "published_type": "geometry_msgs/msg/PolygonStamped",
        "profiles_read_back": ["transport_stowed", "cleaning_deployed", "arm_deployed"],
        "base_motion_inhibit_independent_safety_subscriber": True,
        "safety_status_fresh_per_override": True,
        "safety_manager_state": "BASE_COMMAND_STOPPED",
        "safety_manager_reason": "manipulator_base_inhibit",
        "test_override_preserves_base_inhibit": True,
        "test_override_never_authorizes_motion": True,
        "fresh_readback_required_per_override": True,
        "raw_input_exact_per_override": True,
        "published_frame_by_topic": {
            "/local_costmap/published_footprint": "odom",
            "/global_costmap/published_footprint": "map",
        },
        "footprint_padding_m": live_padding,
        "declared_footprint_padding_m": 0.01,
        "live_footprint_padding_m": live_padding,
        "footprint_padding_quantization_bound_m": 1.862645149230957e-09,
        "point32_quantization_bound_m": 1e-6,
        "published_readback_contract": "ordered_frame_aware_padded_rigid_point32",
        "published_stamp_max_age_sec": 2.0,
        "profile_base_frame": "base_footprint",
        "profile_to_robot_base_planar_equivalence": {
            "/local_costmap/footprint": {
                "profile_base_frame": "base_footprint", "robot_base_frame": "base_link",
                "translation_x_m": 0.0, "translation_y_m": 0.0,
                "translation_z_m": 0.1651, "yaw_rad": 0.0,
                "planar_zero_ulp_bound_m": 1e-323,
                "planar_zero_ulp_bound_rad": 1e-323,
            },
            "/global_costmap/footprint": {
                "profile_base_frame": "base_footprint", "robot_base_frame": "base_link",
                "translation_x_m": 0.0, "translation_y_m": 0.0,
                "translation_z_m": 0.1651, "yaw_rad": 0.0,
                "planar_zero_ulp_bound_m": 1e-323,
                "planar_zero_ulp_bound_rad": 1e-323,
            },
        },
    }


def _w2_request() -> dict:
    return {
        "schema_version": 2,
        "target_id": "pc-track-01",
        "frame_id": "map",
        "pose": {
            "x_m": 0.3, "y_m": -0.95, "z_m": 0.015,
            "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
        },
        "size_m": [0.03, 0.03, 0.03],
        "material": "unknown",
        "confidence": 0.8,
        "truth_used": False,
    }


def _w2_target() -> dict:
    return {
        "uuid": "pc-track-01",
        "frame_id": "map",
        "source_stamp_ns": 900_000_000,
        "header_stamp_ns": 900_000_000,
        "source_backend": "dosod_edgesam_pc",
        "target_type": "discrete",
        "track_state": "CONFIRMED",
        "confidence": 0.8,
        "pose": dict(_w2_request()["pose"]),
        "size_m": list(_w2_request()["size_m"]),
    }


def _w3(schedule_sha: str) -> dict:
    pairs = {"|".join((left, right)): 0.51 for index, left in enumerate(WALKER_IDS) for right in WALKER_IDS[index + 1:]}
    return {
        "status": "FORMAL_DYNAMIC_OBSTACLE_AVOIDANCE_ACCEPTANCE_PASSED", "passed": True,
        "metrics": {"environment_truth_collector": {
            "collector_role": "evaluator_only_no_robot_control", "control_topics_published": [],
            "product_actions_created": [], "pedestrian_schedule_sha256": schedule_sha,
            "pose_source_topic": "/world/campus_formal/pose/info",
            "gazebo_native_pose_topic": "/world/campus_formal/pose/info",
            "pose_source_native_gazebo_read": True,
            "evaluator_native_gazebo_topics_read": ["/world/campus_formal/pose/info"],
            "pose_source_schedule_bound_walker_ids": WALKER_IDS, "walker_radius_m_by_id": WALKER_RADII,
            "pose_source_is_live_gazebo_truth": True, "walker_pose_source_fresh_at_window_end": True,
            "walker_pose_sampling_sufficient": True, "walker_peer_gate_passed": True,
            "native_pose_transport_error_count": 0, "native_pose_transport_timeout_count": 0,
            "native_pose_transport_timeout_policy": "count_and_fail_closed",
            "walker_center_distance_violation_count": 0, "walker_center_distance_violations_lte_0_50_m": [],
            "minimum_walker_center_distance_m_by_pair": pairs,
            "walker_pair_clearance_threshold_m_by_pair": {key: 0.50 for key in pairs},
        }},
    }


def _mapping() -> dict:
    return {
        "passed": True, "truth_used_for_control": False,
        "command_topic_publishers": {
            "/cmd_vel_nav": ["/controller_server"], "/cmd_vel_smoothed": ["/velocity_smoother"],
            "/cmd_vel_gate": ["/collision_monitor"], "/base_controller/cmd_vel": ["/whole_vehicle_safety_manager"],
        },
        "command_chain_publishers_attributed": True,
        "active_command_chain_command_timeout_count": 0, "odom_displacement_m": 0.10,
    }


def _w5(mapping: Path) -> dict:
    return {
        "status": "FORMAL_FIRST_MAP_THEN_SAVED_MAP_CLEANING_PASSED", "passed": True,
        "checks": {"quality_gated_map_manifest": True, "mapping_runtime_passed": True,
                   "mapping_safe_profile_retains_0_45_m_s": True,
                   "saved_map_cleaning_runtime_passed": True},
        "evidence": {"mapping_runtime": str(mapping.resolve())},
    }


def test_regular_in_rejects_escape_and_symlink(tmp_path):
    root = tmp_path / "run"; root.mkdir()
    outside = _write(tmp_path / "outside.json", {})
    with pytest.raises(MODULE.ReceiptError, match="escapes"):
        MODULE._regular_in(outside, root, "outside")
    link = root / "link.json"
    try:
        link.symlink_to(outside)
    except OSError:
        pytest.skip("symlinks unavailable")
    with pytest.raises(MODULE.ReceiptError, match="regular"):
        MODULE._regular_in(link, root, "link")


def test_child_pass_requires_real_w2_w3_and_w5_contracts():
    assert not MODULE._child_passed("w1", {"result": "PASS"})
    assert MODULE._child_passed("w1", _w1())
    assert not MODULE._child_passed("w2", {"passed": True, "runtime_gate": "wrong"})
    assert not MODULE._child_passed("w3_public_audit", {"scope": "hidden"})
    assert not MODULE._child_passed("w5", {"status": "FORMAL_FIRST_MAP_THEN_SAVED_MAP_CLEANING_PASSED", "passed": True})
    assert MODULE._child_passed("w2", _w2())
    assert MODULE._child_passed("w3_live_dynamic", _w3("a" * 64), runtime_schedule_sha256="a" * 64, walker_ids=WALKER_IDS, walker_radii=WALKER_RADII, world_name="campus_formal")


@pytest.mark.parametrize(
    ("key", "replacement"),
    [
        ("passed", False),
        ("raw_input_exact_per_override", False),
        ("published_frame_by_topic", {}),
        ("declared_footprint_padding_m", 0.02),
        ("live_footprint_padding_m", {}),
        ("point32_quantization_bound_m", 0.0),
        ("published_readback_contract", "base_coordinates_exact"),
        ("profile_base_frame", "base_link"),
    ],
)
def test_w1_receipt_rejects_each_new_frame_aware_contract_field(key, replacement):
    payload = _w1()
    payload[key] = replacement
    assert not MODULE._child_passed("w1", payload)


@pytest.mark.parametrize(
    ("field", "replacement"),
    [
        ("translation_x_m", 1e-6),
        ("translation_y_m", -1e-6),
        ("yaw_rad", 1e-6),
        ("planar_zero_ulp_bound_m", float("inf")),
        ("planar_zero_ulp_bound_rad", 0.0),
    ],
)
def test_w1_receipt_rejects_unverifiable_profile_to_robot_planar_evidence(field, replacement):
    payload = _w1()
    payload["profile_to_robot_base_planar_equivalence"]["/local_costmap/footprint"][field] = replacement
    assert not MODULE._w1_passed(payload)


@pytest.mark.parametrize(
    ("path", "replacement"),
    [
        (("uuid",), "other-target"),
        (("frame_id",), "odom"),
        (("header_stamp_ns",), 900_000_001),
        (("source_backend",), "synthetic_truth"),
        (("target_type",), "continuous"),
        (("track_state",), "LOST"),
        (("confidence",), 0.7),
        (("pose", "x_m"), 0.31),
        (("size_m", 0), 0.04),
    ],
)
def test_w2_receipt_rejects_any_matched_target_field_tamper(path, replacement):
    request = _w2_request()
    target = _w2_target()
    cursor = target
    for key in path[:-1]:
        cursor = cursor[key]
    cursor[path[-1]] = replacement
    assert not MODULE._w2_target_matches_request(target, request)


def test_live_dynamic_rejects_wrong_schedule_sha_and_tampered_pair():
    payload = _w3("a" * 64)
    assert not MODULE._child_passed("w3_live_dynamic", payload, runtime_schedule_sha256="b" * 64, walker_ids=WALKER_IDS, walker_radii=WALKER_RADII, world_name="campus_formal")
    payload["metrics"]["environment_truth_collector"]["minimum_walker_center_distance_m_by_pair"]["walker_0|walker_1"] = 0.50
    assert not MODULE._child_passed("w3_live_dynamic", payload, runtime_schedule_sha256="a" * 64, walker_ids=WALKER_IDS, walker_radii=WALKER_RADII, world_name="campus_formal")


def test_stdout_receipt_is_atomic_and_refuses_overwrite(tmp_path):
    source = _write(tmp_path / "w2.stdout", _w2())
    output = tmp_path / "w2.json"
    MODULE.seal_stdout(source, output)
    assert json.loads(output.read_text(encoding="utf-8"))["runtime_gate"] == "moveit_ground_collision"
    assert not list(tmp_path.glob("*.pending.*"))
    with pytest.raises(MODULE.ReceiptError, match="unsafe"):
        MODULE.seal_stdout(source, output)


def test_publish_binds_fresh_w5_mapping_and_snapshot_identity(tmp_path, monkeypatch):
    repository = tmp_path / "repo"
    snapshot = _write(repository / "reports/engineering/formal_vehicle_snapshot_manifest.json", {})
    identity = {"snapshot_manifest_sha256": "s" * 64, "source_inventory_sha256": "i" * 64, "expanded_urdf_sha256": "u" * 64}
    monkeypatch.setattr(MODULE, "_snapshot_identity", lambda _: identity)
    run = tmp_path / "run"; run.mkdir(); started = 1
    session = _write(run / "session.json", {"status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING", "started_epoch_ns": started, "snapshot": identity})
    binding = _write(run / "binding.json", {})
    _write(run / "w1.runtime_binding.json", {})
    w2_binding = _write(run / "w2.runtime_binding.json", {})
    _write(run / "w3.runtime_binding.json", {})
    perception_artifacts = tmp_path / "perception_artifacts"; perception_artifacts.mkdir()
    onnx_pythonpath = tmp_path / "onnx_pythonpath"
    (onnx_pythonpath / "onnxruntime").mkdir(parents=True)
    (onnx_pythonpath / "onnxruntime" / "__init__.py").write_text("", encoding="utf-8")
    closure = _write(tmp_path / "closure.json", {
        "status": "frozen",
        "closure": {
            "perception_artifact_root": str(perception_artifacts.resolve()),
            "onnx_pythonpath": str(onnx_pythonpath.resolve()),
        },
    })
    monkeypatch.setattr(MODULE, "load_binding", lambda _: {"status": "FORMAL_RUNTIME_GATE_BOUND", "acceptance_session_binding": {
        "session_manifest": str(session.resolve()), "session_manifest_sha256": MODULE._hash(session),
        "session_started_epoch_ns": started, "snapshot": identity,
    }, "runtime_closure_binding": {
        "manifest": str(closure.resolve()), "manifest_sha256": MODULE._hash(closure),
    }})
    world = run / "world.sdf"; world.write_text("world", encoding="utf-8")
    episode = _write(run / "episode.json", {"split": "train"})
    schedule = _write(run / "schedule.json", {"access": "environment_driver_only_not_robot_control"})
    runtime_schedule = _write(run / "runtime_schedule.json", {
        "world_name": "campus_formal",
        "acceptance_environment": {"product_control_access_prohibited": True},
        "pedestrians": [{"object_id": walker, "radius_m": 0.25} for walker in WALKER_IDS],
    })
    mapping = _write(run / "mapping.json", _mapping())
    payloads = {
        "w1": _w1(),
        "w2": _w2(),
        "w3_public_audit": {"scope": "public_train_val_only", "hidden_accessed": False, "map_count": 40, "episode_count": 800, "pedestrian_path_count": 6400, "pedestrian_pair_count": 22400, "pedestrian_static_collision_path_count": 0, "pedestrian_cube_collision_path_count": 0, "pedestrian_pair_violation_count": 0},
        "w3_live_dynamic": _w3(hashlib.sha256(runtime_schedule.read_bytes()).hexdigest()), "w5": _w5(mapping),
    }
    children = {name: _write(run / f"{name}.json", payload) for name, payload in payloads.items()}
    request = _write(run / "w2_request.json", _w2_request())
    provenance_path = _write(run / "w2_request_provenance.json", {
        "schema_version": 1,
        "report_id": "r065_w2_live_grasp_request_provenance",
        "passed": True,
        "capture_epoch_ns": 2,
        "capture_ros_time_ns": 1_000_000_000,
        "source_age_s": 0.1,
        "raw_request_sha256": MODULE._hash(request),
        "request": {"path": str(request.resolve()), "size_bytes": request.stat().st_size},
        "product_topics": {
            "targets": {"topic": "/perception/garbage/targets", "type": "sanitation_perception_interfaces/msg/GarbageTargetArray", "publisher": "/pc_open_vocab_product_adapter"},
            "wrist_recheck": {"topic": "/perception/wrist/grasp_recheck", "type": "std_msgs/msg/String", "publisher": "/pc_open_vocab_product_adapter"},
        },
        "target": _w2_target(),
        "acceptance_session": {"path": str(session.resolve()), "sha256": MODULE._hash(session)},
        "runtime_binding": {"path": str(w2_binding.resolve()), "sha256": MODULE._hash(w2_binding)},
        "closure_manifest": {"path": str(closure.resolve()), "sha256": MODULE._hash(closure)},
        "perception_artifact_root": str(perception_artifacts.resolve()),
        "onnx_pythonpath": str(onnx_pythonpath.resolve()),
    })
    receipt = MODULE.publish(repository_root=repository, run_root=run, session=session, runtime_binding=binding, episode_manifest=episode, world=world, environment_schedule=schedule, runtime_schedule=runtime_schedule, children=children, output=run / "r065_public_modeling_receipt.json")
    assert receipt["status"] == "R065_PUBLIC_MODELING_PASSED"
    assert receipt["children"]["w5"]["mapping_runtime_evidence"]["sha256"] == MODULE._hash(mapping)
    with pytest.raises(MODULE.ReceiptError, match="overwrite"):
        MODULE.publish(repository_root=repository, run_root=run, session=session, runtime_binding=binding, episode_manifest=episode, world=world, environment_schedule=schedule, runtime_schedule=runtime_schedule, children=children, output=run / "r065_public_modeling_receipt.json")
    (run / "r065_public_modeling_receipt.json").unlink()
    tampered = json.loads(provenance_path.read_text(encoding="utf-8"))
    tampered["source_age_s"] = 0.2
    provenance_path.write_text(json.dumps(tampered), encoding="utf-8")
    with pytest.raises(MODULE.ReceiptError, match="product contract"):
        MODULE.publish(repository_root=repository, run_root=run, session=session, runtime_binding=binding, episode_manifest=episode, world=world, environment_schedule=schedule, runtime_schedule=runtime_schedule, children=children, output=run / "r065_public_modeling_receipt.json")
    tampered["source_age_s"] = 0.1
    tampered["target"]["pose"]["x_m"] = 99.0
    provenance_path.write_text(json.dumps(tampered), encoding="utf-8")
    with pytest.raises(MODULE.ReceiptError, match="product contract"):
        MODULE.publish(repository_root=repository, run_root=run, session=session, runtime_binding=binding, episode_manifest=episode, world=world, environment_schedule=schedule, runtime_schedule=runtime_schedule, children=children, output=run / "r065_public_modeling_receipt.json")
    tampered["target"] = _w2_target()
    tampered["onnx_pythonpath"] = str(perception_artifacts.resolve())
    provenance_path.write_text(json.dumps(tampered), encoding="utf-8")
    with pytest.raises(MODULE.ReceiptError, match="product contract"):
        MODULE.publish(repository_root=repository, run_root=run, session=session, runtime_binding=binding, episode_manifest=episode, world=world, environment_schedule=schedule, runtime_schedule=runtime_schedule, children=children, output=run / "r065_public_modeling_receipt.json")


def test_w5_missing_mapping_evidence_is_blocked(tmp_path):
    payload = _w5(tmp_path / "missing_mapping.json")
    with pytest.raises(MODULE.ReceiptError, match="regular"):
        MODULE._w5_mapping_evidence_row(payload, tmp_path, 1)
    payload["evidence"] = {}
    with pytest.raises(MODULE.ReceiptError, match="lacks"):
        MODULE._w5_mapping_evidence_row(payload, tmp_path, 1)


def test_w1_w3_bindings_must_be_fresh_and_equivalent_to_root(tmp_path, monkeypatch):
    source = SCRIPT.read_text(encoding="utf-8")
    assert 'run_root / "w1.runtime_binding.json"' in source
    assert 'run_root / "w3.runtime_binding.json"' in source
    root = tmp_path / "run"
    root.mkdir()
    binding_file = _write(root / "w3.runtime_binding.json", {})
    root_binding = {
        "acceptance_session_binding": {"session": "root"},
        "runtime_closure_binding": {"closure": "root"},
    }
    monkeypatch.setattr(MODULE, "load_binding", lambda _: dict(root_binding))
    for gate in ("w1", "w3"):
        gate_binding = _write(root / f"{gate}.runtime_binding.json", {})
        row = MODULE._matching_gate_binding_row(
            gate_binding, root, 1, root_binding, f"{gate} runtime gate binding"
        )
        assert row["path"] == str(gate_binding.resolve())

    monkeypatch.setattr(
        MODULE,
        "load_binding",
        lambda _: {
            "acceptance_session_binding": {"session": "other"},
            "runtime_closure_binding": {"closure": "root"},
        },
    )
    with pytest.raises(MODULE.ReceiptError, match="differs from the root"):
        MODULE._matching_gate_binding_row(
            binding_file, root, 1, root_binding, "w3 runtime gate binding"
        )


def test_wrapper_creates_root_and_runner_local_bindings_after_session_start():
    wrapper = (ROOT / "scripts/run_r065_public_modeling_session.sh").read_text(encoding="utf-8")
    assert 'set +u\nsource /opt/ros/jazzy/setup.bash\nsource "$R065_INSTALL_ROOT/setup.bash"\nset -u' in wrapper
    assert "formal_acceptance_session.py\" start" in wrapper
    assert "formal_source_bound_preflight \"$ROOT\" \"$R065_RUNTIME_WS\"" in wrapper
    assert wrapper.count('formal_source_bound_preflight "$ROOT"') == 1
    assert 'run_r065_w1_dynamic_footprint_live.sh" "$RUN_ROOT"' in wrapper
    assert 'run_r065_w2_moveit_ground_live.sh" "$RUN_ROOT"' in wrapper
    assert "R065_W2_REQUEST_JSON" not in wrapper
    assert "FORMAL_DYNAMIC_SAVED_MAP_ROOT=\"$RUN_ROOT/first_map\"" in wrapper
    assert "FORMAL_MAP_LIFECYCLE_OUTPUT=\"$RUN_ROOT/w5.json\"" in wrapper
    assert "FORMAL_DYNAMIC_OUTPUT=\"$RUN_ROOT/w3_live_dynamic.json\"" in wrapper
    assert 'FORMAL_DYNAMIC_RUNTIME_BINDING="$RUN_ROOT/w3.runtime_binding.json"' in wrapper
    assert 'RAW_RUN_ROOT="$(realpath --no-symlinks -e "$R065_RUN_ROOT")"' in wrapper
    assert '[[ "$RAW_RUN_ROOT" == "$RUN_ROOT" ]]' in wrapper
    assert wrapper.index("trap on_exit EXIT") < wrapper.index("formal_acceptance_session.py\" start")
    assert wrapper.index("trap on_exit EXIT") < wrapper.index("formal_source_bound_preflight")
    assert wrapper.index("trap on_exit EXIT") < wrapper.index("sanitation-campus-scenario generate")
    assert wrapper.index("run_formal_first_map_dynamic_prerequisite.sh") < wrapper.index("run_formal_dynamic_obstacle_avoidance.sh")
    assert wrapper.index("run_formal_dynamic_obstacle_avoidance.sh") < wrapper.index("run_formal_saved_map_cleaning_lifecycle.sh")
    assert "materialize-hidden" not in wrapper
    assert "--r065-public-run-root" not in wrapper
