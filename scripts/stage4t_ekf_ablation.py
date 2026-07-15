#!/usr/bin/env python3
"""Aggregate comparable Stage4T EKF A/B/C/D trial reports."""

from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path


def percentile(values, probability):
    if not values: return None
    ordered = sorted(values); position = (len(ordered) - 1) * probability
    lower = int(math.floor(position)); upper = int(math.ceil(position))
    if lower == upper: return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def summary(values):
    if not values: return {"rmse": None, "p95": None, "max": None}
    return {
        "rmse": math.sqrt(sum(value * value for value in values) / len(values)),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def candidate_metrics(reports):
    xy = [] ; yaw = [] ; closures = [] ; multirate = [] ; symmetry = []
    for report in reports:
        segments = {item["segment"]: item for item in report.get("segments", [])}
        for segment in segments.values():
            value = segment.get("ekf_xy_error", {}).get("rmse")
            if value is not None: xy.append(float(value))
            if segment.get("ekf_yaw_delta_rad") is not None:
                yaw.append(abs(float(segment["ekf_yaw_delta_rad"]) - float(segment["gt_yaw_delta_rad"])))
        for name in ("rectangle_2x1", "figure_eight_r1"):
            value = segments.get(name, {}).get("ekf_closure_vector_error_m")
            if value is not None: closures.append(float(value))
        for rate in ("0p35", "0p45", "0p6"):
            positive = segments.get(f"turn_positive_step_{rate}")
            negative = segments.get(f"turn_negative_step_{rate}")
            for item in (positive, negative):
                if item:
                    multirate.append(abs(float(item["ekf_yaw_delta_rad"]) - float(item["gt_yaw_delta_rad"])))
            if positive and negative:
                symmetry.append(abs(abs(float(positive["ekf_yaw_delta_rad"])) - abs(float(negative["ekf_yaw_delta_rad"]))))
    return {
        "trial_count": len(reports),
        "xy_error_m": summary(xy),
        "yaw_error_rad": summary(yaw),
        "closure_error_m": summary(closures),
        "multi_speed_yaw_stability_rad": summary(multirate),
        "left_right_symmetry_rad": summary(symmetry),
        "all_trials_complete": bool(reports and all(report.get("experiment_completed") and report.get("all_segments_complete") for report in reports)),
    }


def select_with_equivalence(candidates, eligible):
    """Apply the required ordering, using declared engineering equivalence bands."""
    remaining = list(eligible)
    criteria = (
        ("xy_error_m", 0.010),
        ("closure_error_m", 0.010),
        ("multi_speed_yaw_stability_rad", 0.005),
        ("left_right_symmetry_rad", 0.030),
    )
    trace = []
    for field, tolerance in criteria:
        values = {name: candidates[name][field]["rmse"] for name in remaining}
        finite = {name: value for name, value in values.items() if value is not None}
        if not finite: continue
        best = min(finite.values())
        remaining = [name for name in remaining if values.get(name) is not None and values[name] <= best + tolerance]
        trace.append({"criterion": field, "best": best, "equivalence_tolerance": tolerance, "remaining": remaining})
    portability_order = ("A", "B", "C", "D")
    selected = next((name for name in portability_order if name in remaining), None)
    return selected, trace


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("output_dir", type=Path); parser.add_argument("--required-repeats", type=int, default=5); parser.add_argument("--config-dir", type=Path, required=True)
    args = parser.parse_args(); candidates = {}
    configs = {
        "A": "ekf_a_wheel_vx_imu_vyaw.yaml",
        "B": "ekf_b_wheel_vx_imu_yaw_vyaw.yaml",
        "C": "ekf_c_current_wheel_twist_imu_vyaw.yaml",
        "D": "ekf_d_wheel_vx_vyaw_no_imu.yaml",
    }
    for candidate in configs:
        reports = [json.loads(path.read_text(encoding="utf-8")) for path in sorted((args.output_dir / "ekf_trials" / candidate).glob("seed_*/motion_calibration_report.json"))]
        candidates[candidate] = candidate_metrics(reports)
    eligible = [name for name, metrics in candidates.items() if metrics["all_trials_complete"]]
    selected, selection_trace = select_with_equivalence(candidates, eligible) if eligible else (None, [])
    report = {
        "schema_version": 1,
        "comparison_protocol": "same calibrated vehicle, Stage4T action set, seeds 0..N-1",
        "required_repeats_per_candidate": args.required_repeats,
        "candidate_definitions": {
            "A": "wheel vx + IMU vyaw; no wheel vy/vyaw",
            "B": "wheel vx + validated IMU yaw/vyaw",
            "C": "wheel vx/vy/vyaw + IMU vyaw (Stage4S behavior)",
            "D": "wheel vx/vyaw without IMU",
        },
        "candidates": candidates,
        "selected_candidate": selected,
        "selection_order": ["realistic ground-truth XY error", "closure residual", "multi-speed stability", "left-right symmetry", "real-vehicle portability"],
        "selection_trace": selection_trace,
        "portability_preference": ["A", "B", "C", "D"],
        "ablation_complete": bool(selected and all(metrics["trial_count"] >= args.required_repeats and metrics["all_trials_complete"] for metrics in candidates.values())),
    }
    (args.output_dir / "ekf_ablation_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if selected:
        shutil.copyfile(args.config_dir / configs[selected], args.output_dir / "selected_ekf.yaml")


if __name__ == "__main__": main()
