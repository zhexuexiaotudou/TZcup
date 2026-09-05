from __future__ import annotations

import copy
import hashlib
import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/validate_formal_dry_speed_requalification.py"
SPEC = importlib.util.spec_from_file_location("dry_speed_requalification", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _reports() -> tuple[dict, dict, dict, dict, dict, dict]:
    binding = {"session_started_epoch_ns": 9, "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING", "snapshot": {"source_inventory_sha256": "a" * 64}}
    closure = {"status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED", "runtime_install_root": "/tmp/runtime"}
    runtime_binding = {"status": "FORMAL_RUNTIME_GATE_BOUND", "acceptance_session_binding": binding, "runtime_closure_binding": closure}
    token = {
        "kind": "TZCUP_DRY_SPEED_REQUALIFICATION_RUN_SCOPED_OPT_IN_MARKER",
        "profile_id": "formal_dry_cleaning_speed_requalification_v1",
        "profile_sha256": hashlib.sha256((ROOT / "config/high_fidelity_vehicle/formal_dry_speed_requalification.yaml").read_bytes()).hexdigest(),
        "test_only_whole_vehicle_safety_cap_mps": 1.0,
        "run_root": str(ROOT.resolve()),
        "nonce": "n" * 32,
    }
    mobility = {
        "status": "FORMAL_A300_DRIVETRAIN_FORWARD_STOP_RUNTIME_PASSED",
        "acceptance_session_binding": binding,
        "command": {
            "forward_speed_mps": 1.0,
            "safety_max_linear_velocity_mps": 1.0,
            "estop": {"exercise_estop": True},
        },
        "checks": {name: True for name in (
            "ground_truth_forward_motion", "plant_odometry_forward_motion",
            "vehicle_stopped_after_zero_command", "plant_odometry_stopped_after_zero_command",
            "wheel_joints_stopped_after_zero_command",
            "estop_asserted_during_physical_motion", "final_safety_command_zero_after_estop",
            "final_safety_command_reached_one_mps_before_estop",
            "final_safety_command_has_one_expected_writer_and_input_subscriber",
            "estop_feedback_or_manual_estop_status_observed",
            "gazebo_estop_braking_distance_bounded",
            "plant_odom_estop_braking_matches_ground_truth",
            "physical_vehicle_stopped_after_estop",
        )},
        "runtime_gate_binding": runtime_binding,
    }
    interlock = {
        "status": "WHOLE_VEHICLE_ACTUATOR_INTERLOCK_PASSED",
        "acceptance_session_binding": binding,
        "safety_max_linear_velocity_mps": 1.0,
        "base_command_input_mps": 1.0,
        "checks": {name: True for name in (
            "relock_base_zero", "relock_velocity_controllers_inactive",
            "final_physical_estop_attributed_to_manual_estop", "relay_false_zeros_base_brush_and_pump",
        )},
        "runtime_gate_binding": runtime_binding,
    }
    dynamic = {
        "status": "FORMAL_DYNAMIC_OBSTACLE_AVOIDANCE_ACCEPTANCE_PASSED",
        "runtime_gate_binding": runtime_binding,
        "metrics": {
            "operation_speed_profile": "dry_cleaning_competition_candidate",
            "safety_max_linear_velocity_mps": 1.0,
            "maximum_command_speeds_mps": {"base": 1.0},
        },
        "checks": {"collision_monitor_intervened": True, "zero_physical_collisions": True},
    }
    ground = {
        "status": "FORMAL_GROUND_DIRT_PHYSICAL_CLEANING_PASSED",
        "acceptance_session_binding": binding,
        "metrics": {"drive_speed_mps": 1.0, "safety_max_linear_velocity_mps": 1.0},
        "checks": {name: True for name in (
            "physical_sweep_reaches_95_percent", "real_joint_and_world_pose_ready_samples_seen",
            "all_rigid_litter_models_remain",
        )},
        "runtime_gate_binding": runtime_binding,
    }
    return mobility, interlock, dynamic, ground, runtime_binding, token


def test_requalification_only_passes_all_source_bound_subgates() -> None:
    reports = _reports()
    result = MODULE.validate(mobility=reports[0], interlock=reports[1], dynamic=reports[2], ground_dirt=reports[3], current_runtime_binding=reports[4], opt_in_token=reports[5], run_root=ROOT)
    assert result["passed"] is True
    assert result["dry_speed_safety_requalified"] is True
    assert result["competition_efficiency_eligible"] is False


def test_dynamic_collision_or_speed_evidence_cannot_be_skipped() -> None:
    mobility, interlock, dynamic, ground, current, token = _reports()
    dynamic = copy.deepcopy(dynamic)
    dynamic["checks"]["zero_physical_collisions"] = False
    result = MODULE.validate(mobility=mobility, interlock=interlock, dynamic=dynamic, ground_dirt=ground, current_runtime_binding=current, opt_in_token=token, run_root=ROOT)
    assert result["passed"] is False
    assert "dynamic_ttc_intervention_zero_collision_at_one_mps" in result["blockers"]


def test_mismatched_session_binding_fails_closed() -> None:
    mobility, interlock, dynamic, ground, current, token = _reports()
    ground = copy.deepcopy(ground)
    ground["acceptance_session_binding"]["session_started_epoch_ns"] = 10
    result = MODULE.validate(mobility=mobility, interlock=interlock, dynamic=dynamic, ground_dirt=ground, current_runtime_binding=current, opt_in_token=token, run_root=ROOT)
    assert result["passed"] is False
    assert "all_subgates_bind_one_running_session" in result["blockers"]


def test_stale_runtime_recheck_or_token_cannot_authorize_speed() -> None:
    mobility, interlock, dynamic, ground, current, token = _reports()
    current = copy.deepcopy(current)
    current["acceptance_session_binding"]["session_status_at_gate"] = "FORMAL_FINAL_ACCEPTANCE_SESSION_CLOSED"
    result = MODULE.validate(mobility=mobility, interlock=interlock, dynamic=dynamic, ground_dirt=ground, current_runtime_binding=current, opt_in_token=token, run_root=ROOT)
    assert result["passed"] is False
    assert "current_session_and_runtime_closure_reverified" in result["blockers"]

    _, _, _, _, current, token = _reports()
    token = copy.deepcopy(token)
    token["profile_sha256"] = "0" * 64
    result = MODULE.validate(mobility=mobility, interlock=interlock, dynamic=dynamic, ground_dirt=ground, current_runtime_binding=current, opt_in_token=token, run_root=ROOT)
    assert result["passed"] is False
    assert "run_scoped_explicit_opt_in_marker_binds_profile_and_run" in result["blockers"]


def test_malformed_subgate_evidence_is_never_accepted() -> None:
    try:
        MODULE.validate(mobility={}, interlock={}, dynamic={}, ground_dirt={}, current_runtime_binding={}, opt_in_token={}, run_root=ROOT)
    except (KeyError, ValueError):
        pass
    else:
        raise AssertionError("malformed subgate evidence unexpectedly passed")
