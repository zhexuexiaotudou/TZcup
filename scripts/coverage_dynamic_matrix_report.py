#!/usr/bin/env python3
"""Aggregate multiple bounded dynamic-obstacle missions into one formal gate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def _read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def build_report(root: Path) -> dict:
    runs = []
    all_trials = []
    for run_dir in sorted(root.glob("run_seed_*")):
        dynamic_path = run_dir / "dynamic_obstacle_report.json"
        coverage_path = run_dir / "coverage_report.json"
        if not dynamic_path.exists() or not coverage_path.exists():
            runs.append({
                "run": run_dir.name,
                "evidence": str(run_dir),
                "complete": False,
            })
            continue
        dynamic = _read(dynamic_path)
        coverage = _read(coverage_path)
        trials = dynamic.get("trials", [])
        all_trials.extend(trials)
        runs.append({
            "run": run_dir.name,
            "evidence": str(run_dir),
            "complete": True,
            "requested_trial_count": int(dynamic.get("requested_trial_count", 0)),
            "completed_trial_count": int(dynamic.get("completed_trial_count", 0)),
            "valid_trial_count": int(dynamic.get("dynamic_obstacle_valid_trials", 0)),
            "collision_count": int(dynamic.get("collision_count", 0)),
            "minimum_observed_separation_m": dynamic.get("minimum_observed_separation_m"),
            "repeated_oscillation_count": int(dynamic.get("repeated_oscillation_count", 0)),
            "probe_success": bool(dynamic.get("success")),
            "mission_full_execution_success": bool(coverage.get("full_execution_success")),
            "mission_coverage_quality_success": bool(
                coverage.get("coverage_quality_success")
            ),
            "mission_collision_count": int(coverage.get("collision_count", 0)),
            "mission_keepout_violation_count": int(
                coverage.get("keepout_violation_sample_count", 0)
            ),
            "brush_disabled_on_exit": bool(coverage.get("brush_disabled_on_exit")),
        })

    completed = sum(int(item.get("completed_trial_count", 0)) for item in runs)
    valid = sum(int(item.get("valid_trial_count", 0)) for item in runs)
    collisions = sum(int(item.get("collision_count", 0)) for item in runs)
    resumed = sum(bool(trial.get("mission_progress_resumed")) for trial in all_trials)
    oscillations = sum(int(item.get("repeated_oscillation_count", 0)) for item in runs)
    separations = [
        float(item["minimum_observed_separation_m"])
        for item in runs
        if item.get("minimum_observed_separation_m") is not None
    ]
    valid_rate = valid / completed if completed else 0.0
    resume_rate = resumed / completed if completed else 0.0
    gates = {
        "at_least_three_independent_missions": len(runs) >= 3,
        "all_run_evidence_complete": bool(runs) and all(item.get("complete") for item in runs),
        "valid_dynamic_interactions_at_least_20": valid >= 20,
        "dynamic_recovery_rate_at_least_0_95": valid_rate >= 0.95,
        "mission_resume_rate_at_least_0_95": resume_rate >= 0.95,
        "collision_count_zero": collisions == 0,
        "minimum_separation_pass": bool(all_trials) and all(
            trial.get("minimum_separation_gate_pass") for trial in all_trials
        ),
        "repeated_oscillation_zero": oscillations == 0,
        "all_missions_completed": bool(runs) and all(
            item.get("mission_full_execution_success") for item in runs
        ),
        "all_missions_meet_coverage_quality": bool(runs) and all(
            item.get("mission_coverage_quality_success") for item in runs
        ),
        "mission_safety_counts_zero": bool(runs) and all(
            item.get("mission_collision_count") == 0
            and item.get("mission_keepout_violation_count") == 0
            for item in runs
        ),
        "brush_disabled_on_every_exit": bool(runs) and all(
            item.get("brush_disabled_on_exit") for item in runs
        ),
        "all_configured_subruns_pass": bool(runs) and all(
            item.get("probe_success") for item in runs
        ),
    }
    passed = all(gates.values())
    return {
        "schema": "tzcup.coverage_dynamic_matrix.v1",
        "root": str(root),
        "run_count": len(runs),
        "requested_trial_count": sum(
            int(item.get("requested_trial_count", 0)) for item in runs
        ),
        "completed_trial_count": completed,
        "dynamic_obstacle_valid_trials": valid,
        "dynamic_recovery_rate": valid_rate,
        "mission_resume_rate": resume_rate,
        "collision_count": collisions,
        "minimum_observed_separation_m": min(separations) if separations else None,
        "repeated_oscillation_count": oscillations,
        "runs": runs,
        "gates": gates,
        "success": passed,
        "pass": passed,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    output = root / "dynamic_obstacle_report.json"
    if output.exists():
        raise SystemExit(f"refusing to overwrite {output}")
    report = build_report(root)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"pass": report["pass"], "valid": report["dynamic_obstacle_valid_trials"]}))
    raise SystemExit(0 if report["pass"] else 1)


if __name__ == "__main__":
    main()
