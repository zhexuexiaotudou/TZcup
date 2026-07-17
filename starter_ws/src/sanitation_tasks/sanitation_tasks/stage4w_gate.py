"""Pure Stage4W dynamic gate aggregation shared by runtime and tests."""

import json
from pathlib import Path


def load(root: Path, name: str):
    return json.loads((root / name).read_text(encoding="utf-8"))


def assemble(root: Path, exit_codes: dict[str, int] | None = None):
    coverage = load(root, "coverage_report.json")
    dynamic = load(root, "dynamic_obstacle_report.json")
    filters = load(root, "filter_report.json")
    safety = load(root, "safety_latency_report.json")
    exit_codes = exit_codes or {}
    replay_pass = bool(
        (root / "dynamic_coverage_bag" / "metadata.yaml").is_file()
        and (root / "replay_coverage_state.txt").is_file()
        and (root / "replay_coverage_state.txt").stat().st_size > 0
    )
    gates = {
        "full_execution_success": bool(coverage.get("full_execution_success")),
        "empirical_coverage_at_least_90pct": float(
            coverage.get("empirical_metrics", {}).get("coverage_rate", 0.0)
        ) >= 0.90,
        "localization_regression_pass": bool(
            coverage.get("localization_regression_during_coverage", {}).get(
                "pass_rmse_at_most_0_05m"
            )
        ),
        "swath_exclusion_intersections_zero": int(
            coverage.get("swath_exclusion_intersection_count", -1)
        ) == 0,
        "collision_count_zero": (
            int(coverage.get("collision_count", -1)) == 0
            and int(dynamic.get("collision_count", -1)) == 0
        ),
        "coverage_keepout_violations_zero": int(
            coverage.get("keepout_violation_sample_count", -1)
        ) == 0,
        "brush_only_during_swaths": int(
            coverage.get("brush_state_violation_sample_count", -1)
        ) == 0,
        "brush_final_state_false": bool(coverage.get("brush_disabled_on_exit")),
        "dynamic_obstacle_valid_trials_at_least_20": int(
            dynamic.get("dynamic_obstacle_valid_trials", 0)
        ) >= 20,
        "dynamic_obstacle_gate_pass": bool(dynamic.get("success")),
        "keepout_violations_zero": int(
            filters.get("keepout", {}).get("violation_sample_count", -1)
        ) == 0,
        "speed_zone_pass": bool(
            filters.get("speed_zone", {}).get("speed_compliance_pass")
        ),
        "emergency_stop_30_trials": int(safety.get("trial_count", 0)) == 30,
        "emergency_stop_p95_at_most_1s": float(
            safety.get("latency_sec", {}).get("p95") or 999.0
        ) <= 1.0,
        "complete_rosbag_replay": replay_pass,
        "all_processes_exit_zero": not any(exit_codes.values()),
    }
    return {
        "schema_version": 1,
        "stage": "Stage4W",
        "lane": "hybrid_rtk_scan_imu_wheel",
        "competition_evidence": True,
        "gates": gates,
        "exit_codes": exit_codes,
        "coverage": coverage,
        "dynamic_obstacles": dynamic,
        "filters": filters,
        "emergency_stop": safety,
        "efficiency": {
            "operation_width_m": 0.65,
            "max_linear_velocity_m_s": 0.45,
            "theoretical_efficiency_m2_h": 0.65 * 0.45 * 3600.0,
            "target_efficiency_m2_h": 3500.0,
            "competition_efficiency_pass": False,
            "minimum_speed_at_current_width_m_s": 3500.0 / 3600.0 / 0.65,
            "minimum_width_at_current_speed_m": 3500.0 / 3600.0 / 0.45,
        },
        "success": all(gates.values()),
    }
