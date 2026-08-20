#!/usr/bin/env python3
"""Create and evaluate the bounded formal Ackermann runtime matrix.

The collector that runs ROS/Gazebo writes one JSON result per scenario. This
tool defines the complete matrix and evaluates only measured results; absent
or malformed results fail closed and retain the first failure.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


SCENARIOS = {
    "straight_5m": {"runs": 1, "gate": "ACKERMANN_PHYSICS_PASS"},
    "circle_matrix": {"runs": 12, "gate": "ACKERMANN_PHYSICS_PASS", "steer_deg": [-28, -25, -15, 15, 25, 28], "directions": ["forward", "reverse"], "laps": 2},
    "steering_step_slalom": {"runs": 1, "gate": "ACKERMANN_PHYSICS_PASS"},
    "zero_speed_steering": {"runs": 1, "gate": "ACKERMANN_PHYSICS_PASS"},
    "three_point_turn": {"runs": 1, "gate": "ACKERMANN_NAV2_PASS"},
    "wheel_odometry": {"runs": 1, "gate": "ACKERMANN_ODOMETRY_PASS"},
    "localization_30_seed": {"runs": 30, "gate": "ACKERMANN_LOCALIZATION_PASS"},
    "coverage_30_seed": {"runs": 30, "gate": "ACKERMANN_COVERAGE_PASS"},
    "dynamic_30_seed": {"runs": 30, "gate": "ACKERMANN_DYNAMIC_PASS"},
    "estop_30": {"runs": 30, "gate": "ACKERMANN_ESTOP_PASS"},
    "mcap_replay": {"runs": 1, "gate": "ACKERMANN_REPLAY_PASS"},
}


def manifest() -> dict:
    return {
        "schema_version": 1,
        "profile": {"drive_model": "ackermann", "coverage_profile": "ackermann"},
        "scenarios": SCENARIOS,
        "global_invariants": {
            "in_place_rotation_event_count": 0,
            "collision_count": 0,
            "keepout_violation_count": 0,
            "curvature_violation_count": 0,
            "steering_limit_violation_count": 0,
            "ground_truth_control_violation_count": 0,
        },
        "thresholds": {
            "straight_cross_track_error_max_m": 0.05,
            "straight_yaw_drift_max_deg": 1.0,
            "radius_error_max_fraction": 0.05,
            "steering_relation_error_max_deg": 2.0,
            "zero_speed_body_yaw_change_max_deg": 0.5,
            "zero_speed_translation_max_m": 0.02,
            "wheel_odom_straight_error_max_fraction": 0.01,
            "wheel_odom_circle_closure_max_m": 0.10,
            "wheel_odom_yaw_error_max_deg": 2.0,
            "localization_xy_rmse_p95_max_m": 0.05,
            "coverage_min_fraction": 0.95,
            "repeat_coverage_max_fraction": 0.20,
            "effective_cleaning_efficiency_min_m2h": 3500.0,
            "targets_cleaned_required": 10,
            "dynamic_replan_resume_min_fraction": 0.95,
            "estop_p95_max_sec": 0.2,
            "replay_metric_delta_max_fraction": 0.01,
        },
    }


def evaluate(directory: Path) -> dict:
    results = {}
    first_failure = None
    for name, spec in SCENARIOS.items():
        path = directory / f"{name}.json"
        passed = False
        reason = "evidence_missing"
        if path.is_file():
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
                passed = payload.get("passed") is True and int(payload.get("runs", 0)) >= spec["runs"]
                reason = payload.get("first_failure") or ("passed" if passed else "run_count_or_threshold_failed")
            except (OSError, ValueError, json.JSONDecodeError) as error:
                reason = f"evidence_unreadable:{type(error).__name__}"
        results[name] = {"passed": passed, "required_runs": spec["runs"], "reason": reason}
        if not passed and first_failure is None:
            first_failure = name
    return {
        "schema_version": 1,
        "all_pass": all(item["passed"] for item in results.values()),
        "first_failure": first_failure,
        "results": results,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    manifest_parser = sub.add_parser("manifest")
    manifest_parser.add_argument("--output", type=Path, required=True)
    evaluate_parser = sub.add_parser("evaluate")
    evaluate_parser.add_argument("--evidence-dir", type=Path, required=True)
    evaluate_parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = manifest() if args.command == "manifest" else evaluate(args.evidence_dir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0 if args.command == "manifest" or payload["all_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
