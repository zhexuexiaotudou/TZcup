#!/usr/bin/env python3
"""Run the AUTO-11 large-map and scheduled-route formal matrix."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_tasks"))
from sanitation_tasks.large_map import (
    MapSpec,
    reload_map,
    run_coverage_mission,
    schedule_routes,
    serialize_map,
    simulate_localization,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--implementation-commit", required=True)
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    map_root = output / "map"
    spec = MapSpec()
    serialized = serialize_map(spec, map_root)
    reloaded = reload_map(map_root)
    localization = [simulate_localization(spec, seed) for seed in range(10)]
    coverage = [run_coverage_mission(spec, seed) for seed in range(5)]
    schedules = schedule_routes(spec, 20)
    lost = sum(row["lost_localization_events"] for row in localization)
    recovered = sum(row["recovered_events"] for row in localization)
    tf_samples = sum(row["tf_sample_count"] for row in localization)
    tf_discontinuities = sum(
        row["tf_discontinuity_count"] for row in localization
    )
    resume_trials = coverage + schedules
    checks = {
        "mapped_area_at_least_20000_m2": serialized["area_m2"] >= 20_000,
        "map_resolution_documented": serialized["resolution"] > 0,
        "map_serialization_reload_pass": reloaded["area_m2"]
        == serialized["area_m2"],
        "submap_zone_indexing_pass": len(serialized["zones"]) == 20
        and len({item["submap_id"] for item in serialized["zones"]}) == 20,
        "formal_trajectories_at_least_10": len(localization) >= 10,
        "trajectory_xy_rmse_each_at_most_0_05_m": all(
            row["rmse_m"] <= 0.05 for row in localization
        ),
        "truth_estimate_separation_pass": all(
            row["truth_source"] != row["estimate_source"]
            and not row["self_comparison_used"]
            for row in localization
        ),
        "lost_localization_recovery_at_least_0_95": recovered / lost >= 0.95,
        "tf_continuity_at_least_0_999": (
            tf_samples - tf_discontinuities
        )
        / tf_samples
        >= 0.999,
        "full_coverage_missions_at_least_5": len(coverage) >= 5
        and all(row["coverage_complete"] for row in coverage),
        "scheduled_routes_at_least_20": len(schedules) >= 20
        and all(row["route_completed"] for row in schedules),
        "zone_selection_accuracy_100_percent": all(
            row["requested_zone_id"] == row["selected_zone_id"]
            for row in schedules
        ),
        "boundary_violation_zero": sum(
            row["boundary_violation_count"] for row in resume_trials
        )
        == 0,
        "dynamic_collision_zero": sum(
            row["dynamic_collision_count"] for row in resume_trials
        )
        == 0,
        "resume_after_interruption_at_least_0_95": sum(
            row["resume_success"] for row in resume_trials
        )
        / len(resume_trials)
        >= 0.95,
    }
    report = {
        "schema_version": 1,
        "stage": "AUTO-11",
        "attempt_id": "AUTO-11-LARGE-MAP-V1",
        "implementation_commit": args.implementation_commit,
        "source_level": "OFFLINE_LARGE_MAP_SIMULATION",
        "map": serialized,
        "localization": localization,
        "coverage_missions": coverage,
        "scheduled_routes": schedules,
        "aggregate": {
            "lost_localization_recovery_rate": recovered / lost,
            "tf_continuity": (tf_samples - tf_discontinuities) / tf_samples,
            "zone_selection_accuracy": sum(
                row["requested_zone_id"] == row["selected_zone_id"]
                for row in schedules
            )
            / len(schedules),
            "resume_after_interruption_rate": sum(
                row["resume_success"] for row in resume_trials
            )
            / len(resume_trials),
        },
        "checks": checks,
        "auto11_gate_pass": all(checks.values()),
    }
    (output / "formal_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({"aggregate": report["aggregate"], "checks": checks}, indent=2))
    return 0 if report["auto11_gate_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
