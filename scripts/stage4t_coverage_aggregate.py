#!/usr/bin/env python3
"""Assemble the realistic Stage4T coverage and safety gate."""

import argparse
import json
from pathlib import Path


def load(root, name): return json.loads((root / name).read_text(encoding="utf-8"))


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("output_dir", type=Path)
    args = parser.parse_args(); root = args.output_dir
    coverage = load(root, "coverage_report.json"); dynamic = load(root, "dynamic_obstacle_report.json"); safety = load(root, "safety_latency_report.json"); filters = load(root, "filter_report.json")
    empirical = coverage.get("empirical_metrics", {})
    gates = {
        "full_execution_success": bool(coverage.get("full_execution_success")),
        "empirical_coverage_at_least_90pct": float(empirical.get("coverage_rate", 0.0)) >= 0.90,
        "collision_count_zero": int(coverage.get("collision_count", -1)) == 0 and int(dynamic.get("collision_count", -1)) == 0,
        "brush_final_state_false": bool(coverage.get("brush_disabled_on_exit")),
        "keepout_violations_zero": int(filters.get("keepout", {}).get("violation_sample_count", -1)) == 0,
        "speed_zone_pass": bool(filters.get("speed_zone", {}).get("speed_compliance_pass")),
        "dynamic_obstacle_valid_trials_at_least_20": int(dynamic.get("dynamic_obstacle_valid_trials", 0)) >= 20,
        "emergency_stop_p95_at_most_1s": float(safety.get("latency_sec", {}).get("p95") or 999.0) <= 1.0,
    }
    report = {
        "schema_version": 1, "lane": "realistic", "competition_evidence": True,
        "gates": gates, "coverage": coverage, "dynamic_obstacles": dynamic,
        "filters": filters, "emergency_stop": safety,
        "efficiency": {
            "operation_width_m": 0.65, "max_linear_velocity_m_s": 0.45,
            "theoretical_efficiency_m2_h": 0.65 * 0.45 * 3600.0,
            "target_efficiency_m2_h": 3500.0, "competition_efficiency_pass": False,
            "minimum_speed_at_current_width_m_s": 3500.0 / 3600.0 / 0.65,
            "minimum_width_at_current_speed_m": 3500.0 / 3600.0 / 0.45,
        },
        "success": all(gates.values()),
    }
    (root / "stage4t_coverage_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__": main()
