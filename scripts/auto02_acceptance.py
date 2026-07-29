#!/usr/bin/env python3
"""Assemble and enforce the AUTO-02 full navigation regression gate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def build_report(root: Path) -> dict:
    static = load(root / "static" / "stage4w_static_matrix_report.json")
    dynamic = load(root / "dynamic" / "stage4w_dynamic_report.json")
    dynamic_replay = load(root / "dynamic" / "auto02_replay_audit.json")
    cold = [
        load(root / "cold_start" / f"trial_{index}" / "cold_start_report.json")
        for index in range(5)
    ]
    trials = static["trials"]
    runtime_audits = [
        load(root / "static" / f"seed_{index}" / "auto02_runtime_geometry_audit.json")
        for index in range(5)
    ]
    replay_audits = [
        load(root / "static" / f"seed_{index}" / "auto02_replay_audit.json")
        for index in range(5)
    ]
    dynamic_gates = dynamic["gates"]
    dynamic_obstacles = dynamic["dynamic_obstacles"]
    safety = dynamic["emergency_stop"]
    filters = dynamic["filters"]
    speed_zone = filters["speed_zone"]
    speed_inside = speed_zone["mean_speed_m_s"]["inside"]
    speed_ceiling = speed_zone["allowed_maximum_mean_speed_m_s"]
    dynamic_trials = dynamic_obstacles["trials"]

    checks = {
        "static_seed_count_5": len(trials) == 5,
        "static_mission_complete_5_of_5": all(
            item["static_gate_pass"]
            and item["full_execution_success"]
            and item["component_terminal_success_count"] == 17
            and item["component_terminal_total"] == 17
            for item in trials
        ),
        "static_empirical_coverage_at_least_0_90": all(
            float(item["empirical_coverage_rate"]) >= 0.90 for item in trials
        ),
        "static_planned_coverage_at_least_0_95": all(
            float(item["planned_coverage_rate"]) >= 0.95 for item in trials
        ),
        "static_collision_keepout_zero": all(
            item["collision_count"] == 0
            and item["keepout_violation_sample_count"] == 0
            for item in trials
        ),
        "static_brush_safe_terminal_state": all(
            item["brush_state_violation_sample_count"] == 0
            and item["brush_disabled_on_exit"] is True
            for item in trials
        ),
        "static_trajectory_rmse_at_most_0_05_m": all(
            float(item["localization"]["rmse_m"]) <= 0.05 for item in trials
        ),
        "frozen_profile_runtime_geometry_5_of_5": (
            len(runtime_audits) == 5
            and all(item["runtime_geometry_gate_pass"] for item in runtime_audits)
        ),
        "dynamic_valid_interactions_at_least_20": (
            dynamic_obstacles["dynamic_obstacle_valid_trials"] >= 20
            and dynamic_gates["dynamic_obstacle_gate_pass"] is True
        ),
        "dynamic_obstacle_really_moved": all(
            len(trial["moving_obstacle_trajectory"]) >= 2
            and len(
                {
                    tuple(sample["target_map_xy"])
                    for sample in trial["moving_obstacle_trajectory"]
                }
            )
            >= 2
            for trial in dynamic_trials
        ),
        "dynamic_collision_zero_and_mission_complete": (
            dynamic_gates["collision_count_zero"] is True
            and dynamic_gates["full_execution_success"] is True
        ),
        "dynamic_minimum_separation_pass": (
            dynamic_obstacles["minimum_separation_gate_pass"] is True
            and float(dynamic_obstacles["minimum_observed_separation_m"])
            >= float(
                dynamic_obstacles["configured_hard_minimum_separation_m"]
            )
        ),
        "dynamic_coverage_resume_all": (
            dynamic_obstacles["mission_progress_resumed_all"] is True
            and all(trial["mission_progress_resumed"] for trial in dynamic_trials)
        ),
        "filter_keepout_zero": (
            dynamic_gates["keepout_violations_zero"] is True
            and filters["keepout"]["violation_sample_count"] == 0
        ),
        "filter_speed_mean_within_limit_plus_0_03_m_s": (
            dynamic_gates["speed_zone_pass"] is True
            and filters["success"] is True
            and speed_inside is not None
            and float(speed_inside) <= float(speed_ceiling)
            and abs(float(speed_zone["allowed_tolerance_m_s"]) - 0.03) <= 1.0e-9
        ),
        "estop_30_of_30": (
            dynamic_gates["emergency_stop_30_trials"] is True
            and safety["completed_trial_count"] == 30
            and safety["trial_count"] == 30
            and safety["emergency_stop_zeroed"] is True
        ),
        "estop_latency_p95_and_max_pass": (
            dynamic_gates["emergency_stop_p95_at_most_1s"] is True
            and float(safety["latency_sec"]["p95"]) <= 1.0
            and float(safety["latency_sec"]["max"]) <= 1.5
        ),
        "estop_post_stop_output_zero": (
            safety["post_stop_command_output_zero"] is True
            and all(
                trial["post_stop_command_output_zero"]
                for trial in safety["trials"]
            )
        ),
        "dynamic_brush_final_false": (
            dynamic_gates["brush_final_state_false"] is True
            and dynamic["coverage"]["brush_disabled_on_exit"] is True
        ),
        "cold_start_5_of_5": (
            len(cold) == 5
            and all(
                item["interfaces_ready"]
                and item["nav2_parameter_services_ready_within_60_seconds"]
                and item["full_lifecycle_active"]
                and item["lifecycle_active_count"]
                == item["lifecycle_node_count"]
                and item["tf_tree_complete"]
                for item in cold
            )
        ),
        "replay_metadata_topics_state_and_delta_pass": (
            dynamic_gates["complete_rosbag_replay"] is True
            and dynamic_replay["replay_gate_pass"] is True
            and all(item["rosbag_replay"] for item in trials)
            and all(item["replay_gate_pass"] for item in replay_audits)
        ),
    }
    return {
        "schema_version": 1,
        "stage": "AUTO-02",
        "profile": "autonomous_navigation_profile_v1",
        "source_profile": "auto01_g2_v5_retracted",
        "checks": checks,
        "machine_gate_pass": all(checks.values()),
        "static_trials": trials,
        "runtime_geometry_audits": runtime_audits,
        "static_replay_audits": replay_audits,
        "dynamic_summary": {
            "valid_interactions": dynamic_obstacles[
                "dynamic_obstacle_valid_trials"
            ],
            "minimum_observed_separation_m": dynamic_obstacles[
                "minimum_observed_separation_m"
            ],
            "configured_hard_minimum_separation_m": dynamic_obstacles[
                "configured_hard_minimum_separation_m"
            ],
            "gates": dynamic_gates,
            "filters": filters,
            "emergency_stop": safety,
            "replay": dynamic_replay,
        },
        "cold_start_ready_seconds": [
            item["nav2_parameter_services_ready_seconds"] for item in cold
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = build_report(args.root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["machine_gate_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
