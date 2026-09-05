import hashlib
import importlib.util
import json
from pathlib import Path
import copy
import sys

import pytest


SCRIPT = Path(__file__).with_name("validate_formal_map_lifecycle_runtime.py")
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("map_lifecycle_validator", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _write(path: Path, value: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")
    return path


def test_validator_passes_only_complete_real_runtime_contract(tmp_path):
    root = tmp_path / "map"
    root.mkdir()
    (root / "occupancy.yaml").write_text("image: occupancy.pgm\n", encoding="utf-8")
    files = (
        "occupancy.yaml",
        "occupancy.pgm",
        "mission_geometry.yaml",
        "materialization_contract.yaml",
        "geofence_keepout.yaml",
        "geofence_keepout.pgm",
        "neutral_speed.yaml",
        "neutral_speed.pgm",
    )
    for name in files[1:]:
        (root / name).write_bytes(name.encode("utf-8"))
    hashes = {
        name: hashlib.sha256((root / name).read_bytes()).hexdigest()
        for name in files
    }
    _write(root / "map_lifecycle_manifest.json", {
        "schema_version": 1,
        "status": "ready_for_localization_cleaning",
        "occupancy_map": "occupancy.yaml",
        "observed_fraction": 0.95,
        "quality_threshold": 0.95,
        "stable_gate_samples": 3,
        "fixed_start_verified": True,
        "gnss_mapping_reference_observed": True,
        "mapping_pose_source": (
            "wheel_imu_ekf_lidar_scan_matching_gnss_consistency"
        ),
        "world_truth_used_for_control": False,
        "mapping_ignored_dirt": True,
        "sha256": hashes,
    })
    mapping = _write(tmp_path / "mapping.json", {
        "passed": True,
        "truth_used_for_control": False,
        "collision_monitor_node_count": 1,
        "cmd_vel_gate_publisher_count": 1,
        "odom_tf_publisher_count": 1,
        "odom_tf_min_rate_hz": 20.0,
        "slam_map_observed": True,
        "slam_odom_failures_after_ready": 0,
    })
    cleaning = _write(tmp_path / "cleaning.json", {
        "passed": True,
        "truth_used_for_control": False,
        "localization_backend": "amcl",
        "saved_map_sha256_verified": True,
        "world_derived_map_fallback": False,
        "collision_monitor_node_count": 1,
        "cmd_vel_gate_publisher_count": 1,
        "cleaning_stack_ready": True,
        "coverage_server_ready": True,
        "hard_restart_verified": True,
        "coverage_action_terminal_passed": True,
        "coverage_state": {"state": "COMPLETED"},
        "coverage_execution_report": {
            "success": True,
            "terminal_state": "COMPLETED",
            "ground_truth_used_for_control": False,
            "operation_width_m": 1.32,
            "operation_speed_profile": "dry_cleaning_competition_candidate",
            "maximum_linear_speed_mps": 1.0,
            "planned_swath_count": 3,
            "completed_swath_count": 3,
        },
        "trajectory_total_distance_m": 100.0,
        "brush_enabled_distance_m": 95.0,
        "brush_state_sample_count": 4,
        "brush_state_transitions": 4,
        "brush_state_source": "/brush_enabled_product_runtime",
        "brush_disabled_on_exit": True,
        "estimated_coverage_fraction": 0.95,
    })
    result = MODULE.validate(root, mapping, cleaning)
    assert result["passed"] is True
    assert result["status"] == MODULE.PASS_STATUS
    assert result["operation_speed_profiles"] == {
        "mapping_safe": pytest.approx(0.45),
        "dry_cleaning": pytest.approx(1.0),
    }

    valid_cleaning = json.loads(cleaning.read_text(encoding="utf-8"))
    for profile_name, speed in (
        ("dry_cleaning_competition_candidate", 0.45),
        ("mapping_safe", 0.45),
    ):
        invalid_cleaning = copy.deepcopy(valid_cleaning)
        invalid_cleaning["coverage_execution_report"].update(
            {
                "operation_speed_profile": profile_name,
                "maximum_linear_speed_mps": speed,
            }
        )
        failed = MODULE.validate(
            root,
            mapping,
            _write(tmp_path / f"cleaning-{profile_name}.json", invalid_cleaning),
        )
        assert failed["passed"] is False
        assert failed["checks"]["saved_map_cleaning_runtime_passed"] is False


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("coverage_action_terminal_passed", False),
        ("trajectory_total_distance_m", 0.0),
        ("brush_enabled_distance_m", 0.0),
        ("brush_state_sample_count", 0),
        ("brush_state_transitions", 0),
        ("brush_disabled_on_exit", False),
        ("estimated_coverage_fraction", 0.949999),
    ),
)
def test_cleaning_aggregate_fails_closed_without_real_coverage_evidence(
    tmp_path, field, value
):
    root = tmp_path / "map"
    root.mkdir()
    files = (
        "occupancy.yaml", "occupancy.pgm", "mission_geometry.yaml",
        "materialization_contract.yaml", "geofence_keepout.yaml",
        "geofence_keepout.pgm", "neutral_speed.yaml", "neutral_speed.pgm",
    )
    (root / "occupancy.yaml").write_text("image: occupancy.pgm\n", encoding="utf-8")
    for name in files[1:]:
        (root / name).write_bytes(name.encode("utf-8"))
    hashes = {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in files}
    _write(root / "map_lifecycle_manifest.json", {
        "schema_version": 1, "status": "ready_for_localization_cleaning",
        "occupancy_map": "occupancy.yaml", "observed_fraction": 0.95,
        "quality_threshold": 0.95, "stable_gate_samples": 3,
        "fixed_start_verified": True, "gnss_mapping_reference_observed": True,
        "mapping_pose_source": "wheel_imu_ekf_lidar_scan_matching_gnss_consistency",
        "world_truth_used_for_control": False, "mapping_ignored_dirt": True,
        "sha256": hashes,
    })
    mapping_value = {
        "passed": True, "truth_used_for_control": False,
        "collision_monitor_node_count": 1, "cmd_vel_gate_publisher_count": 1,
        "odom_tf_publisher_count": 1, "odom_tf_min_rate_hz": 20.0,
        "slam_map_observed": True, "slam_odom_failures_after_ready": 0,
    }
    cleaning_value = {
        "passed": True, "truth_used_for_control": False,
        "localization_backend": "amcl", "saved_map_sha256_verified": True,
        "world_derived_map_fallback": False, "collision_monitor_node_count": 1,
        "cmd_vel_gate_publisher_count": 1, "cleaning_stack_ready": True,
        "coverage_server_ready": True, "hard_restart_verified": True,
        "coverage_action_terminal_passed": True,
        "coverage_state": {"state": "COMPLETED"},
        "coverage_execution_report": {
            "success": True, "terminal_state": "COMPLETED",
            "ground_truth_used_for_control": False,
            "operation_width_m": 1.32,
            "operation_speed_profile": "dry_cleaning_competition_candidate",
            "maximum_linear_speed_mps": 1.0,
            "planned_swath_count": 3, "completed_swath_count": 3,
        },
        "trajectory_total_distance_m": 100.0,
        "brush_enabled_distance_m": 95.0,
        "brush_state_sample_count": 4,
        "brush_state_transitions": 4,
        "brush_state_source": "/brush_enabled_product_runtime",
        "brush_disabled_on_exit": True,
        "estimated_coverage_fraction": 0.95,
    }
    cleaning_value = copy.deepcopy(cleaning_value)
    cleaning_value[field] = value
    result = MODULE.validate(
        root,
        _write(tmp_path / "mapping.json", mapping_value),
        _write(tmp_path / "cleaning.json", cleaning_value),
    )
    assert result["passed"] is False
    assert result["checks"]["saved_map_cleaning_runtime_passed"] is False


def test_validator_blocks_missing_or_tampered_evidence(tmp_path):
    result = MODULE.validate(tmp_path / "map", tmp_path / "m.json", tmp_path / "c.json")
    assert result["passed"] is False
    assert result["status"] == MODULE.BLOCKED_STATUS
    assert len(result["blockers"]) == 3


def test_bound_report_preserves_existing_binding_and_writes_canonical_sidecar(
    tmp_path, monkeypatch
):
    binding = {
        "schema_version": 1,
        "status": "FORMAL_RUNTIME_GATE_BOUND",
        "acceptance_session_binding": {
            "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        },
        "runtime_closure_binding": {
            "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
        },
    }
    monkeypatch.setattr(MODULE, "load_binding", lambda _: binding)
    output = tmp_path / "formal_map_lifecycle_acceptance.json"
    report = {"status": MODULE.PASS_STATUS, "passed": True}

    MODULE.write_bound_report(output, report, tmp_path / "existing-binding.json")

    sidecar = output.with_name(output.name + ".runtime_binding.json")
    assert json.loads(sidecar.read_text(encoding="utf-8")) == binding
    persisted = json.loads(output.read_text(encoding="utf-8"))
    assert persisted["runtime_gate_binding"] == binding
    assert persisted["acceptance_session_binding"] == binding["acceptance_session_binding"]
    assert persisted["runtime_closure_binding"] == binding["runtime_closure_binding"]


def test_bound_report_fails_closed_when_existing_binding_is_invalid(tmp_path, monkeypatch):
    monkeypatch.setattr(
        MODULE,
        "load_binding",
        lambda _: (_ for _ in ()).throw(MODULE.RuntimeGateError("invalid binding")),
    )
    output = tmp_path / "formal_map_lifecycle_acceptance.json"
    with pytest.raises(MODULE.RuntimeGateError, match="invalid binding"):
        MODULE.write_bound_report(output, {"passed": True}, tmp_path / "missing.json")
    assert not output.exists()
    assert not output.with_name(output.name + ".runtime_binding.json").exists()


def test_runtime_collectors_preserve_safety_and_hard_restart_contract():
    collector = SCRIPT.with_name("collect_formal_map_lifecycle_runtime.py").read_text(
        encoding="utf-8"
    )
    mapping_runner = SCRIPT.with_name(
        "run_formal_first_map_dynamic_prerequisite.sh"
    ).read_text(encoding="utf-8")
    cleaning_runner = SCRIPT.with_name(
        "run_formal_saved_map_cleaning_lifecycle.sh"
    ).read_text(encoding="utf-8")
    assert '"/formal_vehicle/simulation/command/emergency_stop"' in collector
    assert '"/formal_vehicle/simulation/command/main_power"' in collector
    assert "create_publisher(Twist" not in collector
    assert '"/cmd_vel_gate"' in collector
    assert '"odom"' in collector and '"base_footprint"' in collector
    assert "ExternalShutdownException" in collector
    assert "interrupted_fail_closed" in collector
    assert "self.safety_base_enabled_samples > 0" in collector
    assert '"odom_displacement_m"' in collector
    assert '"collision_points_inside_transport_max"' in collector
    assert '"command_topic_sample_counts"' in collector
    assert '"/scan/navigation"' in collector
    assert '"filtered_scan_sample_count"' in collector
    assert "qos_profile_sensor_data" in collector
    assert '"/robot_description"' in collector
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in collector
    assert "self.robot_description_samples > 0" in collector
    assert '"robot_description_sha256"' in collector
    assert '"safety_active_reason_counts"' in collector
    assert '"safety_latest_values"' in collector
    assert "collect_formal_map_lifecycle_runtime.py" in mapping_runner
    assert "validate_formal_map_lifecycle_runtime.py" in mapping_runner
    assert 'FORMAL_MAPPING_TIMEOUT_S:-21600' in mapping_runner
    assert '--timeout "${mapping_timeout_sec}"' in mapping_runner
    assert "mapping_timeout_sec + mapping_poll_period_sec - 1" in mapping_runner
    assert 'FORMAL_MAPPING_POLLS:-40' not in mapping_runner
    for runner in (mapping_runner, cleaning_runner):
        assert "formal_source_bound_preflight.sh" in runner
        assert "formal_source_bound_preflight" in runner
        assert "formal_source_bound_verify_overlay" in runner
        assert "FORMAL_VEHICLE_RUNTIME_WS" in runner
        assert "FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST" in runner
        assert "FORMAL_ACCEPTANCE_SESSION" in runner
        assert "FORMAL_MAP_RUNTIME_OVERLAY" not in runner
        assert "Legacy developer fallback" not in runner
        assert "/.work/stage1_" not in runner
    assert "mission_mode:=cleaning" in cleaning_runner
    assert "start_pedestrians:=true" in cleaning_runner
    assert "start_coverage:=true" in cleaning_runner
    assert 'FORMAL_CLEANING_PLANNER:-full_coverage' in cleaning_runner
    assert 'FORMAL_PERCEPTION_ARTIFACT_ROOT' in cleaning_runner
    assert 'FORMAL_POLICY_CHECKPOINT' in cleaning_runner
    assert 'FORMAL_FULL_COVERAGE_DISTANCE_M' in cleaning_runner
    assert "sanitation_product_demo_integration product_demo.launch.py" in cleaning_runner
    assert 'cleaning_planner:=full_coverage' in cleaning_runner
    assert 'perception_artifact_root:="${perception_artifact_root}"' in cleaning_runner
    assert 'policy_checkpoint:="${policy_checkpoint}"' in cleaning_runner
    assert 'maximum_task_distance_m:="${maximum_task_distance_m}"' in cleaning_runner
    assert "mapping_process_count_before_cleaning" in cleaning_runner
    assert "mapping_pid_alive_count_before_cleaning" in cleaning_runner
    assert 'os.kill(pid, 0)' in cleaning_runner
    assert 'handoff.get("mapping_runner_exit_code") != 0' in cleaning_runner
    assert '"mapping_handoff_record_sha256"' in cleaning_runner
    assert '"map_lifecycle_manifest_sha256"' in cleaning_runner
    assert '"mapping_runtime_sha256"' in cleaning_runner
    assert '--mission-geometry "${map_root}/mission_geometry.yaml"' in cleaning_runner
    assert '--coverage-report "${coverage_execution}"' in cleaning_runner
    assert "--restart-record" in cleaning_runner
    assert '--runtime-binding "${runtime_binding}"' in cleaning_runner
