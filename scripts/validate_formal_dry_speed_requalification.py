#!/usr/bin/env python3
"""Fail-closed aggregation for the isolated 1.0 m/s dry-cleaning test lane."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import yaml

from formal_runtime_gate_binding import load_binding
from formal_dry_speed_requalification_token import validate as validate_opt_in_token


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "config/high_fidelity_vehicle/formal_dry_speed_requalification.yaml"
ENVELOPES = ROOT / "starter_ws/src/sanitation_safety/config/operational_envelopes.yaml"
PASS_STATUS = "FORMAL_DRY_SPEED_SAFETY_REQUALIFIED"
BLOCKED_STATUS = "FORMAL_DRY_SPEED_SAFETY_REQUALIFICATION_BLOCKED"


def _read(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _number(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("expected finite numeric value")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("expected finite numeric value")
    return value


def _binding(report: dict[str, Any]) -> dict[str, Any] | None:
    direct = report.get("acceptance_session_binding")
    if isinstance(direct, dict):
        return direct
    runtime = report.get("runtime_gate_binding")
    if isinstance(runtime, dict) and isinstance(runtime.get("acceptance_session_binding"), dict):
        return runtime["acceptance_session_binding"]
    return None


def _checks(report: dict[str, Any]) -> dict[str, bool]:
    value = report.get("checks")
    return value if isinstance(value, dict) else {}


def _runtime_binding(report: dict[str, Any]) -> dict[str, Any] | None:
    value = report.get("runtime_gate_binding")
    return value if isinstance(value, dict) else None


def _stage(profile: dict[str, Any], stage_id: str) -> tuple[int, dict[str, Any]]:
    stages = profile.get("qualification_stages")
    if not isinstance(stages, list):
        raise ValueError("qualification_stages must be a list")
    for index, item in enumerate(stages):
        if isinstance(item, dict) and item.get("id") == stage_id:
            return index, item
    raise ValueError("requested qualification stage is not declared")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _require_regular_file(path: Path, description: str) -> None:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{description} must be a regular, non-symlink file")


def _require_non_symlink_path_under(path: Path, root: Path, description: str) -> None:
    """Reject lexical escape and every linked directory in retained evidence."""
    root = root.absolute()
    path = path.absolute()
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"{description} is outside the retained .work evidence root") from exc
    current = path.parent
    while True:
        if current.is_symlink():
            raise ValueError(f"{description} has a symlinked ancestor: {current}")
        if current == root:
            return
        if current == current.parent:
            raise ValueError(f"{description} escaped the retained .work evidence root")
        current = current.parent


def _runtime_binding_identity(binding: dict[str, Any]) -> dict[str, Any] | None:
    """Compare the stable gate identity, not the per-verification timestamp."""
    session = binding.get("acceptance_session_binding")
    closure = binding.get("runtime_closure_binding")
    if not isinstance(session, dict) or not isinstance(closure, dict):
        return None
    return {
        "status": binding.get("status"),
        "acceptance_session_binding": session,
        "runtime_closure_binding": closure,
    }


def verify_final_receipt(
    *,
    receipt_path: Path,
    current_runtime_binding: dict[str, Any],
    profile_path: Path = PROFILE,
    evidence_root: Path | None = None,
) -> dict[str, Any]:
    """Verify the retained, current-session four-stage receipt before 1.0 m/s use.

    This is deliberately a verifier, not an enablement mechanism.  Its caller
    still has to opt in explicitly and passes the resulting cap only to the
    isolated formal dry-cleaning launch.
    """
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    if not isinstance(profile, dict):
        raise ValueError("requalification profile must be a mapping")
    stages = profile.get("qualification_stages")
    if not isinstance(stages, list) or not stages:
        raise ValueError("requalification profile stages are missing")
    stage_ids = [item.get("id") if isinstance(item, dict) else None for item in stages]
    targets = [
        _number(item.get("target_linear_speed_mps")) if isinstance(item, dict) else math.nan
        for item in stages
    ]
    if stage_ids != ["speed_0_25_mps", "speed_0_45_mps", "speed_0_70_mps", "speed_1_00_mps"]:
        raise ValueError("requalification profile does not declare the required four stages")
    if not math.isclose(targets[-1], 1.0, abs_tol=1e-12):
        raise ValueError("requalification profile final stage is not 1.0 m/s")

    _require_regular_file(receipt_path, "final speed receipt")
    if receipt_path.name != "dry_speed_requalification.json" or receipt_path.parent.name != stage_ids[-1]:
        raise ValueError("final receipt must be the retained speed_1_00_mps stage report")
    permitted_root = (evidence_root or ROOT / ".work").absolute()
    _require_non_symlink_path_under(receipt_path, permitted_root, "final receipt")
    run_root = receipt_path.parent.parent.resolve()
    try:
        run_root.relative_to(permitted_root.resolve())
    except ValueError as exc:
        raise ValueError("requalification receipt is outside the retained .work evidence root") from exc

    current_session = current_runtime_binding.get("acceptance_session_binding")
    current_closure = current_runtime_binding.get("runtime_closure_binding")
    if (
        current_runtime_binding.get("status") != "FORMAL_RUNTIME_GATE_BOUND"
        or not isinstance(current_session, dict)
        or current_session.get("session_status_at_gate")
        != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
        or not isinstance(current_closure, dict)
    ):
        raise ValueError("current formal session/runtime binding is not RUNNING and closed")
    current_identity = _runtime_binding_identity(current_runtime_binding)

    previous_receipt_sha256: str | None = None
    final_receipt: dict[str, Any] | None = None
    for index, (stage_id, target) in enumerate(zip(stage_ids, targets, strict=True)):
        stage_root = run_root / stage_id
        stage_receipt = stage_root / "dry_speed_requalification.json"
        marker = stage_root / "requalification.opt_in.marker.json"
        _require_regular_file(stage_receipt, f"{stage_id} receipt")
        _require_regular_file(marker, f"{stage_id} opt-in marker")
        report = _read(stage_receipt)
        token = validate_opt_in_token(
            profile_path=profile_path,
            run_root=stage_root,
            token_path=marker,
            requested_cap=target,
        )
        checks = _checks(report)
        evidence_marker = report.get("requalification_evidence_marker")
        if not isinstance(evidence_marker, dict):
            raise ValueError(f"{stage_id} receipt has no evidence-only marker")
        if (
            report.get("status") != PASS_STATUS
            or report.get("passed") is not True
            or report.get("dry_speed_safety_requalified") is not True
            or report.get("qualification_stage_id") != stage_id
            or not math.isclose(_number(report.get("qualification_target_speed_mps")), target, abs_tol=1e-12)
            or report.get("competition_efficiency_eligible") is not False
            or not checks
            or not all(value is True for value in checks.values())
            or report.get("source_binding") != current_session
            or _runtime_binding_identity(report.get("runtime_gate_binding", {})) != current_identity
            or report.get("requalification_run_root") != str(run_root)
            or evidence_marker.get("kind") != "TZCUP_DRY_SPEED_REQUALIFICATION_EVIDENCE_ONLY"
            or evidence_marker.get("run_scoped_opt_in_marker_nonce") != token.get("nonce")
            or evidence_marker.get("authorizes_product_default_change") is not False
            or evidence_marker.get("authorizes_real_hardware_operation") is not False
        ):
            raise ValueError(f"{stage_id} receipt is incomplete, stale, or not evidence-only")
        if bool(evidence_marker.get("future_speed_enablement_must_fail_closed_consume_this_report")) != (index == len(stage_ids) - 1):
            raise ValueError(f"{stage_id} receipt has an invalid enablement boundary")
        if index:
            if report.get("preceding_stage_receipt_sha256") != previous_receipt_sha256:
                raise ValueError(f"{stage_id} receipt does not directly follow the preceding stage")
        elif report.get("preceding_stage_receipt_sha256") is not None:
            raise ValueError("first speed receipt unexpectedly has a predecessor")
        previous_receipt_sha256 = _sha256(stage_receipt)
        final_receipt = report

    if final_receipt is None:
        raise ValueError("final speed receipt is missing")
    return {
        "schema_version": 1,
        "status": "FORMAL_DRY_SPEED_ENABLEMENT_RECEIPT_VERIFIED",
        "final_stage": stage_ids[-1],
        "authorized_isolated_dry_cleaning_cap_mps": targets[-1],
        "requalification_run_root": str(run_root),
        "source_binding": current_session,
        "claim_boundary": (
            "Receipt permits only an explicit isolated formal dry-cleaning speed cap; "
            "it does not change product defaults, certify measured efficiency, or authorize hardware."
        ),
    }


def validate(
    *,
    mobility: dict[str, Any],
    interlock: dict[str, Any],
    dynamic: dict[str, Any],
    ground_dirt: dict[str, Any],
    current_runtime_binding: dict[str, Any],
    opt_in_token: dict[str, Any],
    run_root: Path,
    stage_id: str,
    predecessor: dict[str, Any] | None,
    predecessor_receipt_sha256: str | None = None,
    profile_path: Path = PROFILE,
    envelopes_path: Path = ENVELOPES,
) -> dict[str, Any]:
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    envelopes = yaml.safe_load(envelopes_path.read_text(encoding="utf-8"))
    if not isinstance(profile, dict) or not isinstance(envelopes, dict):
        raise ValueError("profile and envelopes must be mappings")
    activation = profile["activation"]
    stage_index, stage = _stage(profile, stage_id)
    target = _number(stage["target_linear_speed_mps"])
    observed_minimum = _number(stage["minimum_observed_command_speed_mps"])
    default_cap = _number(activation["product_default_whole_vehicle_safety_cap_mps"])
    test_maximum = _number(activation["test_only_maximum_whole_vehicle_safety_cap_mps"])
    product_cap = _number(envelopes["profiles"]["localization_coverage"]["max_linear_velocity"])
    bindings = [_binding(report) for report in (mobility, interlock, dynamic, ground_dirt)]
    common_binding = bindings[0]
    runtime_bindings = [_runtime_binding(report) for report in (mobility, interlock, dynamic, ground_dirt)]
    current_session = current_runtime_binding.get("acceptance_session_binding")
    current_closure = current_runtime_binding.get("runtime_closure_binding")
    mobility_command = mobility.get("command", {})
    dynamic_speeds = dynamic.get("metrics", {}).get("maximum_command_speeds_mps", {})
    ground_metrics = ground_dirt.get("metrics", {})
    interlock_checks = _checks(interlock)
    dynamic_checks = _checks(dynamic)
    dirt_checks = _checks(ground_dirt)
    previous_stage_id = None
    if stage_index:
        stages = profile["qualification_stages"]
        previous_stage_id = stages[stage_index - 1]["id"]
    predecessor_passed = (
        stage_index == 0
        or (
            isinstance(predecessor, dict)
            and predecessor.get("passed") is True
            and predecessor.get("qualification_stage_id") == previous_stage_id
            and predecessor.get("source_binding") == common_binding
        )
    )
    mobility_metrics = mobility.get("metrics", {})
    mobility_command = mobility.get("command", {})
    mobility_duration = _number(mobility_command.get("forward_duration_s"))
    observed_mobility_speed = _number(mobility_metrics.get("ground_truth_forward_delta_m")) / mobility_duration
    checks = {
        "profile_requires_explicit_test_opt_in": activation.get("required_environment") == "FORMAL_DRY_SPEED_REQUALIFICATION=1",
        "profile_disables_automatic_product_enablement": activation.get("automatic_product_enablement") is False,
        "product_default_safety_cap_remains_0_45_mps": math.isclose(product_cap, default_cap, abs_tol=1e-12),
        "stage_is_at_or_below_test_only_maximum": target <= test_maximum,
        "all_subgates_bind_one_running_session": common_binding is not None and all(item == common_binding for item in bindings) and common_binding == current_session,
        "current_session_and_runtime_closure_reverified": current_runtime_binding.get("status") == "FORMAL_RUNTIME_GATE_BOUND" and isinstance(current_session, dict) and current_session.get("session_status_at_gate") == "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING" and all(isinstance(item, dict) and item.get("runtime_closure_binding") == current_closure for item in runtime_bindings),
        "stage_follows_the_previous_qualified_speed": predecessor_passed,
        "run_scoped_explicit_opt_in_marker_binds_profile_stage_and_run": opt_in_token.get("kind") == "TZCUP_DRY_SPEED_REQUALIFICATION_RUN_SCOPED_OPT_IN_MARKER" and opt_in_token.get("profile_id") == profile.get("profile_id") and opt_in_token.get("profile_sha256") == hashlib.sha256(profile_path.read_bytes()).hexdigest() and opt_in_token.get("qualification_stage_id") == stage_id and opt_in_token.get("run_root") == str(run_root.resolve()) and math.isclose(_number(opt_in_token.get("test_only_whole_vehicle_safety_cap_mps")), target, abs_tol=1e-12) and isinstance(opt_in_token.get("nonce"), str) and len(opt_in_token["nonce"]) >= 32,
        "mobility_report_passed": mobility.get("status") == "FORMAL_A300_DRIVETRAIN_FORWARD_STOP_RUNTIME_PASSED",
        "mobility_requested_stage_speed_and_test_cap": math.isclose(_number(mobility_command.get("forward_speed_mps")), target, abs_tol=1e-12) and math.isclose(_number(mobility_command.get("safety_max_linear_velocity_mps")), target, abs_tol=1e-12) and observed_mobility_speed >= observed_minimum,
        "mobility_physical_braking_and_final_zero": all(_checks(mobility).get(name) is True for name in ("ground_truth_forward_motion", "plant_odometry_forward_motion", "vehicle_stopped_after_zero_command", "plant_odometry_stopped_after_zero_command", "wheel_joints_stopped_after_zero_command", "estop_asserted_during_physical_motion", "final_safety_command_reached_requested_speed_before_estop", "final_safety_command_has_one_expected_writer_and_input_subscriber", "estop_feedback_or_manual_estop_status_observed", "final_safety_command_zero_after_estop", "gazebo_estop_braking_distance_bounded", "plant_odom_estop_braking_matches_ground_truth", "physical_vehicle_stopped_after_estop")) and mobility_command.get("estop", {}).get("exercise_estop") is True,
        "interlock_estop_or_fault_zeroes_outputs": interlock.get("status") == "WHOLE_VEHICLE_ACTUATOR_INTERLOCK_PASSED" and math.isclose(_number(interlock.get("safety_max_linear_velocity_mps")), target, abs_tol=1e-12) and math.isclose(_number(interlock.get("base_command_input_mps")), target, abs_tol=1e-12) and all(interlock_checks.get(name) is True for name in ("relock_base_zero", "relock_velocity_controllers_inactive", "final_physical_estop_attributed_to_manual_estop", "relay_false_zeros_base_brush_and_pump")),
        "dynamic_ttc_intervention_zero_collision_at_stage_speed": dynamic.get("status") == "FORMAL_DYNAMIC_OBSTACLE_AVOIDANCE_ACCEPTANCE_PASSED" and dynamic.get("metrics", {}).get("operation_speed_profile") == "dry_cleaning_competition_candidate" and math.isclose(_number(dynamic.get("metrics", {}).get("safety_max_linear_velocity_mps")), target, abs_tol=1e-12) and _number(dynamic_speeds.get("base")) >= observed_minimum and dynamic_checks.get("collision_monitor_intervened") is True and dynamic_checks.get("zero_physical_collisions") is True,
        "ground_dirt_contact_cleaning_at_stage_speed": ground_dirt.get("status") == "FORMAL_GROUND_DIRT_PHYSICAL_CLEANING_PASSED" and math.isclose(_number(ground_metrics.get("drive_speed_mps")), target, abs_tol=1e-12) and math.isclose(_number(ground_metrics.get("safety_max_linear_velocity_mps")), target, abs_tol=1e-12) and all(dirt_checks.get(name) is True for name in ("physical_sweep_reaches_95_percent", "real_joint_and_world_pose_ready_samples_seen", "all_rigid_litter_models_remain")),
    }
    passed = all(checks.values())
    return {
        "schema_version": 1,
        "report_id": profile["result"]["report_id"],
        "status": PASS_STATUS if passed else BLOCKED_STATUS,
        "passed": passed,
        "dry_speed_safety_requalified": passed,
        "qualification_stage_id": stage_id,
        "qualification_target_speed_mps": target,
        "qualification_observed_mobility_speed_mps": observed_mobility_speed,
        # A requalification is evidence only. The product profile remains
        # intentionally ineligible until the broader measured-efficiency gate.
        "competition_efficiency_eligible": False,
        "checks": checks,
        "blockers": [name for name, value in checks.items() if not value],
        "source_binding": common_binding,
        # Preserve the complete binding so a later formal consumer can reject
        # an otherwise plausible receipt from another session, source tree, or
        # frozen runtime closure.
        "runtime_gate_binding": current_runtime_binding,
        "requalification_run_root": str(run_root.resolve().parent),
        "preceding_stage_receipt_sha256": (
            predecessor_receipt_sha256 if predecessor is not None else None
        ),
        "requalification_evidence_marker": {
            "kind": "TZCUP_DRY_SPEED_REQUALIFICATION_EVIDENCE_ONLY",
            "run_scoped_opt_in_marker_nonce": opt_in_token.get("nonce"),
            "future_speed_enablement_must_fail_closed_consume_this_report": stage_index == len(profile["qualification_stages"]) - 1,
            "authorizes_product_default_change": False,
            "authorizes_real_hardware_operation": False,
        },
        "claim_boundary": "Isolated Gazebo evidence only; this report does not modify product eligibility or authorize real-hardware operation.",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-final-receipt", action="store_true")
    parser.add_argument("--receipt", type=Path)
    parser.add_argument("--mobility", type=Path)
    parser.add_argument("--interlock", type=Path)
    parser.add_argument("--dynamic", type=Path)
    parser.add_argument("--ground-dirt", type=Path)
    parser.add_argument("--current-runtime-binding", type=Path, required=True)
    parser.add_argument("--token", type=Path)
    parser.add_argument("--stage")
    parser.add_argument("--predecessor", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.verify_final_receipt:
        if args.receipt is None:
            parser.error("--receipt is required with --verify-final-receipt")
        try:
            result = verify_final_receipt(
                receipt_path=args.receipt,
                current_runtime_binding=load_binding(args.current_runtime_binding),
            )
        except (OSError, UnicodeError, ValueError, TypeError, KeyError) as exc:
            print(f"final speed receipt rejected: {exc}")
            return 2
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    for name in ("mobility", "interlock", "dynamic", "ground_dirt", "token", "stage", "output"):
        if getattr(args, name) is None:
            parser.error(f"--{name.replace('_', '-')} is required when aggregating a stage")
    try:
        result = validate(
            mobility=_read(args.mobility), interlock=_read(args.interlock),
            dynamic=_read(args.dynamic), ground_dirt=_read(args.ground_dirt),
            current_runtime_binding=load_binding(args.current_runtime_binding),
            opt_in_token=_read(args.token), run_root=args.output.parent,
            stage_id=args.stage,
            predecessor=_read(args.predecessor) if args.predecessor else None,
            predecessor_receipt_sha256=_sha256(args.predecessor) if args.predecessor else None,
        )
    except (OSError, UnicodeError, ValueError, TypeError, KeyError) as exc:
        result = {
            "schema_version": 1,
            "report_id": "tzcup_formal_dry_speed_requalification_v1",
            "status": BLOCKED_STATUS,
            "passed": False,
            "dry_speed_safety_requalified": False,
            "competition_efficiency_eligible": False,
            "checks": {},
            "blockers": [f"invalid_or_missing_subgate_evidence:{exc}"],
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
