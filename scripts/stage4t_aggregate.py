#!/usr/bin/env python3
"""Aggregate Stage4T trials while making incomplete matrices explicit."""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path


RATES = (-0.60, -0.45, -0.35, -0.25, -0.10, 0.10, 0.25, 0.35, 0.45, 0.60)
HEADINGS = (-6.283185307179586, -3.141592653589793, -1.5707963267948966, 1.5707963267948966, 3.141592653589793, 6.283185307179586)
THERMAL_STATES = ("cold", "hot")


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path, payload):
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def repeatability(values):
    if not values:
        return {"count": 0, "mean": None, "standard_deviation": None}
    mean = sum(values) / len(values)
    deviation = None
    if len(values) >= 2:
        deviation = (sum((value - mean) ** 2 for value in values) / (len(values) - 1)) ** 0.5
    return {"count": len(values), "mean": mean, "standard_deviation": deviation}


def numeric_summary(values):
    values = sorted(float(value) for value in values if value is not None)
    if not values: return {"count": 0, "mean": None, "p95": None, "max": None}
    position = (len(values) - 1) * 0.95; lower = int(position); upper = min(len(values) - 1, lower + 1)
    p95 = values[lower] + (values[upper] - values[lower]) * (position - lower)
    return {"count": len(values), "mean": sum(values) / len(values), "p95": p95, "max": max(values)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--required-repeats", type=int, default=10)
    args = parser.parse_args()
    output = args.output_dir
    trial_dir = output / "angular_rate_trials"
    trials = [read_json(path) for path in sorted(trial_dir.glob("*.json"))]
    fixed = [trial for trial in trials if trial.get("trial_type") == "fixed_time"]
    heading = [trial for trial in trials if trial.get("trial_type") == "closed_loop_heading"]

    fixed_groups = defaultdict(list)
    for trial in fixed:
        fixed_groups[(round(float(trial["target_yaw_rate_rad_s"]), 2), trial["thermal_state"])].append(trial)
    matrix = []
    for rate in RATES:
        for thermal in THERMAL_STATES:
            group = fixed_groups[(rate, thermal)]
            body_errors = [float(trial["body_yaw_error_deg"]) for trial in group if trial.get("complete") and trial.get("body_yaw_error_deg") is not None]
            metric_fields = (
                "requested_command_integral_rad", "actual_output_command_integral_rad",
                "ground_truth_yaw_delta_rad", "raw_odom_yaw_delta_rad",
                "imu_integrated_yaw_rad", "ekf_yaw_delta_rad",
                "request_to_output_delay_sec", "output_to_body_delay_sec",
                "rise_time_sec", "settling_time_sec", "overshoot_ratio",
                "steady_state_yaw_rate_gain", "integral_tracking_error_rad",
            )
            matrix.append({
                "angular_rate_rad_s": rate,
                "thermal_state": thermal,
                "required_trials": args.required_repeats,
                "completed_trials": sum(bool(trial.get("runner_complete") and trial.get("complete")) for trial in group),
                "repeatability": repeatability([float(trial["ground_truth_yaw_delta_rad"]) for trial in group if trial.get("complete")]),
                "body_yaw_error_deg": {"mean": sum(body_errors) / len(body_errors) if body_errors else None, "max": max(body_errors) if body_errors else None},
                "metrics": {field: numeric_summary([trial.get(field) for trial in group if trial.get("complete")]) for field in metric_fields},
            })
    def tracking_pass_for(rates):
        selected = [item for item in matrix if abs(item["angular_rate_rad_s"]) in rates]
        return bool(selected and all(item["completed_trials"] >= item["required_trials"] and item["body_yaw_error_deg"]["max"] is not None and item["body_yaw_error_deg"]["max"] <= 18.0 for item in selected))
    transient = {
        "schema_version": 1,
        "stage4s_high_speed_failure_preserved": True,
        "stage4s_high_speed_body_yaw_error_deg": 19.1825,
        "stage4s_body_yaw_gate_deg": 18.0,
        "trial_count": len(fixed),
        "required_trial_count": len(RATES) * len(THERMAL_STATES) * args.required_repeats,
        "all_trials_retained": True,
        "matrix": matrix,
        "matrix_complete": all(item["completed_trials"] >= item["required_trials"] for item in matrix),
        "precision_0p25_tracking_pass": tracking_pass_for({0.25}),
        "coverage_0p35_tracking_pass": tracking_pass_for({0.35}),
        "high_speed_open_loop_stress_pass": tracking_pass_for({0.60}),
    }
    write_json(output / "transient_response_report.json", transient)

    heading_groups = defaultdict(list)
    for trial in heading:
        heading_groups[(round(float(trial.get("target_heading_rad", 0.0)), 6), trial["thermal_state"])].append(trial)
    heading_matrix = []
    for target in HEADINGS:
        for thermal in THERMAL_STATES:
            group = heading_groups[(round(target, 6), thermal)]
            errors = [float(trial["ground_truth_heading_error_deg"]) for trial in group if trial.get("ground_truth_heading_error_deg") is not None]
            heading_matrix.append({
                "target_heading_rad": target,
                "thermal_state": thermal,
                "required_trials": args.required_repeats,
                "completed_trials": sum(bool(trial.get("runner_complete") and trial.get("complete")) for trial in group),
                "ground_truth_used_for_control": any(trial.get("ground_truth_used_for_control") for trial in group),
                "feedback_sources": sorted({trial.get("feedback_source") for trial in group if trial.get("feedback_source")}),
                "ground_truth_heading_error_deg": {"mean": sum(errors) / len(errors) if errors else None, "max": max(errors) if errors else None},
                "tracking_pass": bool(len(errors) >= args.required_repeats and max(errors) <= 2.0),
            })
    closed_loop = {
        "schema_version": 1,
        "trial_count": len(heading),
        "required_trial_count": len(HEADINGS) * len(THERMAL_STATES) * args.required_repeats,
        "matrix": heading_matrix,
        "matrix_complete": all(item["completed_trials"] >= item["required_trials"] for item in heading_matrix),
        "ground_truth_control_violation_count": sum(bool(trial.get("ground_truth_used_for_control")) for trial in heading),
        "closed_loop_tracking_pass": bool(heading_matrix and all(item["tracking_pass"] for item in heading_matrix)),
    }
    write_json(output / "closed_loop_heading_report.json", closed_loop)

    profiles = {}
    for name in ("precision", "coverage"):
        path = output / f"{name}_envelope_trial.json"
        if path.exists(): profiles[name] = read_json(path)
    violations = sum(int(report.get("actual_cmd_limit_violations", 0)) for report in profiles.values())
    envelope = {
        "schema_version": 1,
        "high_speed_open_loop_stress_pass": False,
        "stress_profile_default_enabled": False,
        "precision_profile_pass": bool(profiles.get("precision", {}).get("profile_pass")),
        "coverage_profile_pass": bool(profiles.get("coverage", {}).get("profile_pass")),
        "actual_cmd_limit_violations": violations,
        "profiles": profiles,
        "pass": bool(len(profiles) == 2 and violations == 0 and all(report.get("profile_pass") for report in profiles.values())),
    }
    if profiles:
        write_json(output / "operational_envelope_report.json", envelope)


if __name__ == "__main__":
    main()
