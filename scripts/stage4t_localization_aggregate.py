#!/usr/bin/env python3
"""Aggregate all localization seeds; never retain only the best run."""

import argparse
import json
import math
from pathlib import Path


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
    rmse_trials = [(float(trial["xy_error_m"]["rmse"]), trial) for trial in trials if trial.get("xy_error_m", {}).get("rmse") is not None]
    rmses = [item[0] for item in rmse_trials]
    yaw_rmses = [float(trial["yaw_error_rad"]["rmse"]) for trial in trials if trial.get("yaw_error_rad", {}).get("rmse") is not None]
    worst_trial = max(rmse_trials, key=lambda item: item[0])[1] if rmse_trials else None
    report = {
        "schema_version": 1,
        "lane": args.lane,
        "oracle_only": args.lane == "oracle",
        "competition_evidence": args.lane == "realistic",
        "required_seed_count": args.required_seeds,
        "completed_seed_count": len(trials),
        "xy_rmse_m": {"p50": percentile(rmses, 0.50), "p95": percentile(rmses, 0.95), "max": max(rmses) if rmses else None},
        "yaw_rmse_rad": {"p50": percentile(yaw_rmses, 0.50), "p95": percentile(yaw_rmses, 0.95), "max": max(yaw_rmses) if yaw_rmses else None},
        "particle_degenerate_updates": sum(int(trial.get("particle_filter", {}).get("degenerate_update_count", 0)) for trial in trials),
        "tf_continuous_all_trials": bool(trials and all(trial.get("tf_continuity", {}).get("continuous") for trial in trials)),
        "recovery_count": sum(int(trial.get("navigation", {}).get("recoveries_max", 0)) for trial in trials),
        "worst_seed_trial": worst_trial["trial_path"] if worst_trial is not None else None,
        "trials": trials,
    }
    localization_pass = bool(
        len(trials) >= args.required_seeds
        and rmses
        and max(rmses) <= 0.05
        and all(trial.get("competition_localization_pass") for trial in trials)
    )
    report["localization_pass"] = localization_pass
    report["oracle_localization_pass"] = bool(args.lane == "oracle" and localization_pass)
    report["competition_localization_pass"] = bool(args.lane == "realistic" and localization_pass)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__": main()
