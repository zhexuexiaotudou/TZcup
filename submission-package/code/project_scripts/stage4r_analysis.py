#!/usr/bin/env python3
"""Create Stage4R efficiency scan and evidence plots from machine data."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--artifact-dir", required=True)
    args = parser.parse_args(); root = Path(args.artifact_dir)
    widths = [0.65, 0.8, 1.0, 1.2]; speeds = [0.45, 0.8, 1.0, 1.2, 1.5]
    cases = []
    for width in widths:
        for speed in speeds:
            theory = width * speed * 3600.0
            cases.append({"operation_width_m": width, "speed_m_s": speed, "theoretical_efficiency_m2_h": theory, "competition_theory_pass": theory >= 3500.0, "simulation_config_supported": speed <= 0.50 and width == 0.65, "mechanical_feasibility": "unverified_by_URDF"})
    coverage_path = root / "coverage_report.json"
    actual = None
    if coverage_path.is_file():
        actual = json.loads(coverage_path.read_text(encoding="utf-8")).get("empirical_metrics", {}).get("net_efficiency_m2_h")
    minimum_speed_at_current_width = 3500.0 / 3600.0 / 0.65
    report = {
        "formula": "operation_width_m * speed_m_s * 3600",
        "current_config": {"operation_width_m": 0.65, "desired_speed_m_s": 0.45, "theoretical_efficiency_m2_h": 0.65 * 0.45 * 3600.0},
        "actual_net_efficiency_m2_h": actual,
        "target_efficiency_m2_h": 3500.0,
        "minimum_speed_at_0_65m_m_s": minimum_speed_at_current_width,
        "minimum_width_at_0_45m_m": 3500.0 / 3600.0 / 0.45,
        "gap_statement": "The current 0.65 m / 0.45 m/s Nav2 configuration cannot reach 3500 m2/h even theoretically; wider hardware or >=1.496 m/s is required before turn/overlap losses.",
        "mechanical_feasibility_boundary": "URDF and simulation parameter scans do not prove brush, drivetrain, braking, stability, or competition-rule feasibility.",
        "cases": cases,
    }
    report["competition_efficiency_pass"] = bool(actual is not None and actual >= 3500.0)
    (root / "efficiency_scan.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    trajectory = root / "coverage_trajectory.csv"
    if trajectory.is_file():
        rows = list(csv.DictReader(trajectory.open(encoding="utf-8")))
        if rows:
            figure, axis = plt.subplots(figsize=(11, 6), constrained_layout=True)
            on = [row for row in rows if row["brush_enabled"] == "True"]
            off = [row for row in rows if row["brush_enabled"] != "True"]
            if off: axis.plot([float(row["base_x_m"]) for row in off], [float(row["base_y_m"]) for row in off], color="#94a3b8", linewidth=1, label="brush off")
            if on: axis.plot([float(row["base_x_m"]) for row in on], [float(row["base_y_m"]) for row in on], color="#0f766e", linewidth=2, label="brush on actual GT")
            axis.set_aspect("equal"); axis.set_title("Stage4R empirical coverage trajectory"); axis.legend(); axis.grid(alpha=0.2)
            figure.savefig(root / "empirical_coverage_overlay.png", dpi=160); plt.close(figure)


if __name__ == "__main__": main()
