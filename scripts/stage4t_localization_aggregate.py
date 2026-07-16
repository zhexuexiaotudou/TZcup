#!/usr/bin/env python3
"""Aggregate all localization seeds; never retain only the best run."""

import argparse
import json
import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_tasks"))
from sanitation_tasks.localization_metrics import trial_completion_reasons


def percentile(values, probability):
    if not values: return None
    ordered = sorted(values); position = (len(ordered) - 1) * probability
    lower = int(math.floor(position)); upper = int(math.ceil(position))
    if lower == upper: return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("trial_dir", type=Path); parser.add_argument("output", type=Path); parser.add_argument("--lane", choices=("realistic", "oracle"), required=True); parser.add_argument("--required-seeds", type=int, default=10)
    args = parser.parse_args(); trials = []
    for path in sorted(args.trial_dir.glob("seed_*/localization_report.json")):
        report = json.loads(path.read_text(encoding="utf-8")); report["trial_path"] = path.relative_to(args.trial_dir).as_posix(); trials.append(report)
    def metric(trial, section, axis, fallback=None):
        value = trial.get(section, {}).get(axis, {}).get("rmse")
        if value is None and fallback:
            value = trial.get(fallback, {}).get("rmse")
        return value

    completion = [
        {
            "trial_path": trial["trial_path"],
            "complete": not (reasons := trial_completion_reasons(trial)),
            "reasons": reasons,
        }
        for trial in trials
    ]
    completed_paths = {item["trial_path"] for item in completion if item["complete"]}
    completed_trials = [trial for trial in trials if trial["trial_path"] in completed_paths]
    rmse_trials = [
        (float(value), trial)
        for trial in completed_trials
        if (value := metric(trial, "map_relative_localization_error", "xy_m", "xy_error_m")) is not None
    ]
    rmses = [item[0] for item in rmse_trials]
    yaw_rmses = [
        float(value)
        for trial in completed_trials
        if (value := metric(trial, "map_relative_localization_error", "yaw_rad", "yaw_error_rad")) is not None
    ]
    absolute_rmses = [
        float(value)
        for trial in completed_trials
        if (value := metric(trial, "absolute_world_error", "xy_m")) is not None
    ]
    calibrated_absolute_rmses = [
        float(value)
        for trial in completed_trials
        if (value := metric(trial, "calibrated_absolute_world_error", "xy_m")) is not None
    ]
    particle_applicable_trials = [
        trial
        for trial in completed_trials
        if trial.get("particle_filter", {}).get("particle_instrumentation_required", True)
    ]
    particle_gate_pass = bool(
        all(
            trial.get("particle_filter", {}).get("particle_instrumentation_pass")
            for trial in particle_applicable_trials
        )
    )
    worst_trial = max(rmse_trials, key=lambda item: item[0])[1] if rmse_trials else None
    report = {
        "schema_version": 2,
        "lane": args.lane,
        "oracle_only": args.lane == "oracle",
        "competition_evidence": args.lane == "realistic",
        "required_seed_count": args.required_seeds,
        "discovered_seed_count": len(trials),
        "completed_seed_count": len(completed_trials),
        "incomplete_seed_count": len(trials) - len(completed_trials),
        "trial_completion": completion,
        "map_relative_xy_rmse_m": {"p50": percentile(rmses, 0.50), "p95": percentile(rmses, 0.95), "max": max(rmses) if rmses else None},
        "map_relative_yaw_rmse_rad": {"p50": percentile(yaw_rmses, 0.50), "p95": percentile(yaw_rmses, 0.95), "max": max(yaw_rmses) if yaw_rmses else None},
        "absolute_world_xy_rmse_m": {"p50": percentile(absolute_rmses, 0.50), "p95": percentile(absolute_rmses, 0.95), "max": max(absolute_rmses) if absolute_rmses else None},
        "calibrated_absolute_world_xy_rmse_m": {"p50": percentile(calibrated_absolute_rmses, 0.50), "p95": percentile(calibrated_absolute_rmses, 0.95), "max": max(calibrated_absolute_rmses) if calibrated_absolute_rmses else None},
        # Compatibility aliases now explicitly mirror map-relative metrics.
        "xy_rmse_m": {"p50": percentile(rmses, 0.50), "p95": percentile(rmses, 0.95), "max": max(rmses) if rmses else None},
        "yaw_rmse_rad": {"p50": percentile(yaw_rmses, 0.50), "p95": percentile(yaw_rmses, 0.95), "max": max(yaw_rmses) if yaw_rmses else None},
        "map_georeferencing_error": completed_trials[0].get("map_georeferencing_error") if completed_trials else None,
        "particle_instrumentation_applicable_trial_count": len(particle_applicable_trials),
        "particle_instrumentation_pass_all_trials": particle_gate_pass,
        "particle_degenerate_updates": sum(int(trial.get("particle_filter", {}).get("degenerate_update_count", 0)) for trial in trials),
        "tf_continuous_all_trials": bool(completed_trials and all(trial.get("tf_continuity", {}).get("continuous") for trial in completed_trials)),
        "navigation_success_all_trials": bool(completed_trials and all(trial.get("navigation", {}).get("success") for trial in completed_trials)),
        "recovery_count": sum(int(trial.get("navigation", {}).get("recoveries_max", 0)) for trial in completed_trials),
        "worst_seed_trial": worst_trial["trial_path"] if worst_trial is not None else None,
        "trials": trials,
    }
    localization_pass = bool(
        len(trials) == args.required_seeds
        and len(completed_trials) == args.required_seeds
        and rmses
        and max(rmses) <= 0.05
        and all(trial.get("competition_localization_pass") for trial in completed_trials)
        and particle_gate_pass
        and report["tf_continuous_all_trials"]
        and report["navigation_success_all_trials"]
    )
    report["localization_pass"] = localization_pass
    report["oracle_localization_pass"] = bool(args.lane == "oracle" and localization_pass)
    report["competition_localization_pass"] = bool(args.lane == "realistic" and localization_pass)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__": main()
