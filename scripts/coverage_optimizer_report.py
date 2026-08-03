#!/usr/bin/env python3
"""Aggregate retained coverage matrix evidence without rewriting raw runs."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics

import yaml


CONNECTOR_KINDS = {
    "ROTATE", "SHIFT", "BACKUP", "OBSTACLE_BYPASS", "TRANSIT",
    "LEGACY_DUBINS_TURN",
}


def _length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def _read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def _target_summary(run_dir):
    path = run_dir / "gazebo_cleaning_telemetry.json"
    if not path.exists():
        return None, None
    payload = _read_json(path)
    return payload.get("targets_cleaned"), payload.get("targets_total")


def _mcap_summary(run_dir):
    metadata = run_dir / "visual_demo_bag" / "metadata.yaml"
    if not metadata.exists():
        return {"present": False, "messages": 0, "duration_ns": 0}
    payload = yaml.safe_load(metadata.read_text(encoding="utf-8"))
    info = payload.get("rosbag2_bagfile_information", payload)
    return {
        "present": True,
        "messages": int(info.get("message_count", 0)),
        "duration_ns": int(info.get("duration", {}).get("nanoseconds", 0)),
    }


def _geometry(path_report):
    semantic = path_report.get("semantic_plan") or {}
    components = semantic.get("ordered_components") or semantic.get("components") or []
    if components:
        primary = [item for item in components if item.get("kind") == "SWATH"]
        connectors = [item for item in components if item.get("kind") in CONNECTOR_KINDS]
        repairs = [item for item in components if item.get("kind") == "REPAIR_SWATH"]
        return {
            "swath_count": len(primary),
            "connector_count": len(connectors),
            "repair_component_count": len(repairs),
            "primary_path_length_m": sum(float(item.get("length_m", _length(item.get("points", [])))) for item in primary),
            "connector_path_length_m": sum(float(item.get("length_m", _length(item.get("points", [])))) for item in connectors),
        }
    swaths = path_report.get("execution_swaths", [])
    turns = path_report.get("turns", [])
    return {
        "swath_count": len(swaths),
        "connector_count": len(turns),
        "repair_component_count": 0,
        "primary_path_length_m": sum(_length(item) for item in swaths),
        "connector_path_length_m": sum(_length(item) for item in turns),
    }


def collect_run(profile, run_dir):
    report_path = run_dir / "coverage_report.json"
    path_path = run_dir / "coverage_path.json"
    if not report_path.exists() or not path_path.exists():
        return {
            "profile": profile,
            "seed": int(run_dir.name.split("_")[-1]),
            "evidence": str(run_dir),
            "report_present": False,
            "success": False,
        }
    report = _read_json(report_path)
    path_report = _read_json(path_path)
    empirical = report.get("empirical_metrics", {})
    localization = report.get("localization_regression_during_coverage", {})
    repair = report.get("coverage_repair", {})
    straight = empirical.get("primary_swath_straightness_error", {})
    cleaned, total = _target_summary(run_dir)
    geometry = _geometry(path_report)
    repair_passes = repair.get("passes", [])
    return {
        "profile": profile,
        "seed": int(run_dir.name.split("_")[-1]),
        "evidence": str(run_dir),
        "report_present": True,
        "success": bool(report.get("success")),
        "swath_angle_deg": path_report.get("selected_swath_angle_deg"),
        "swath_spacing_m": report.get("planning_swath_spacing_m"),
        **geometry,
        "repair_count": sum(int(item.get("segment_count", 0)) for item in repair_passes),
        "repair_passes": len(repair_passes),
        "repair_path_length_m": sum(float(item.get("planned_repair_length_m", 0.0)) for item in repair_passes),
        "brush_on_distance_m": empirical.get("brush_on_distance_m"),
        "brush_off_distance_m": empirical.get("brush_off_distance_m"),
        "actual_total_distance_m": empirical.get("total_distance_m"),
        "actual_duration_sec": empirical.get("actual_duration_sec"),
        "coverage_rate": empirical.get("coverage_rate"),
        "repeat_rate": empirical.get("repeat_rate"),
        "miss_rate": empirical.get("miss_rate"),
        "targets_cleaned": cleaned,
        "targets_total": total,
        "collision_count": report.get("collision_count"),
        "keepout_violation": report.get("keepout_violation_sample_count"),
        "brush_state_violation": report.get("brush_state_violation_sample_count"),
        "localization_rmse_m": localization.get("rmse_m"),
        "lateral_error_p95_m": straight.get("p95_m"),
        "brush_disabled_on_exit": report.get("brush_disabled_on_exit"),
        "mcap": _mcap_summary(run_dir),
    }


def _median(rows, key):
    values = [float(row[key]) for row in rows if row.get(key) is not None]
    return statistics.median(values) if values else None


def _reduction(baseline, selected, key):
    old = _median(baseline, key)
    new = _median(selected, key)
    if old in (None, 0.0) or new is None:
        return None
    return (old - new) / old


def build_report(root):
    baseline = [collect_run("legacy", path) for path in sorted((root / "baseline").glob("seed_*"))]
    selected = [collect_run("optimized", path) for path in sorted((root / "selected").glob("seed_*"))]
    replay_path = root / "mcap_replay_report.json"
    dynamic_path = root / "dynamic" / "dynamic_obstacle_report.json"
    repair_path = root / "repair" / "repair_matrix_report.json"
    comparison = {
        "actual_total_distance_reduction": _reduction(baseline, selected, "actual_total_distance_m"),
        "brush_off_distance_reduction": _reduction(baseline, selected, "brush_off_distance_m"),
        "connector_distance_reduction": _reduction(baseline, selected, "connector_path_length_m"),
        "duration_reduction": _reduction(baseline, selected, "actual_duration_sec"),
    }
    gates = {
        "five_legacy_seeds": len(baseline) >= 5,
        "five_optimized_seeds": len(selected) >= 5,
        "optimized_all_success": bool(selected) and all(row.get("success") for row in selected),
        "coverage_at_least_0_995": bool(selected) and all((row.get("coverage_rate") or 0) >= 0.995 for row in selected),
        "repeat_at_most_0_20": bool(selected) and all((row.get("repeat_rate") or 1) <= 0.20 for row in selected),
        "targets_10_of_10": bool(selected) and all(row.get("targets_cleaned") == 10 and row.get("targets_total") == 10 for row in selected),
        "zero_collision_keepout": bool(selected) and all(row.get("collision_count") == 0 and row.get("keepout_violation") == 0 for row in selected),
        "localization_rmse_at_most_0_05": bool(selected) and all((row.get("localization_rmse_m") or 1) <= 0.05 for row in selected),
        "lateral_p95_at_most_0_08": bool(selected) and all((row.get("lateral_error_p95_m") or 1) <= 0.08 for row in selected),
        "distance_reduction_at_least_0_25": (comparison["actual_total_distance_reduction"] or -1) >= 0.25,
        "brush_off_reduction_at_least_0_40": (comparison["brush_off_distance_reduction"] or -1) >= 0.40,
        "connector_reduction_at_least_0_50": (comparison["connector_distance_reduction"] or -1) >= 0.50,
        "mcap_replay": replay_path.exists() and bool(_read_json(replay_path).get("pass")),
        "dynamic_matrix": dynamic_path.exists() and bool(_read_json(dynamic_path).get("pass")),
        "repair_matrix": repair_path.exists() and bool(_read_json(repair_path).get("pass")),
    }
    return {
        "schema": "tzcup.coverage_optimizer_comparison.v1",
        "root": str(root),
        "baseline": baseline,
        "selected": selected,
        "comparison": comparison,
        "gates": gates,
        "pass": all(gates.values()),
    }


def write_markdown(report, output):
    lines = [
        "# Coverage path optimization comparison", "",
        f"Overall gate: **{'PASS' if report['pass'] else 'FAIL'}**", "",
        "| Profile | Seed | Success | Coverage | Repeat | Distance m | Brush-off m | Duration s | Targets | RMSE m | Straight P95 m |",
        "|---|---:|:---:|---:|---:|---:|---:|---:|:---:|---:|---:|",
    ]
    for row in report["baseline"] + report["selected"]:
        def fmt(key, digits=3):
            value = row.get(key)
            return "n/a" if value is None else f"{value:.{digits}f}"
        lines.append(
            f"| {row['profile']} | {row['seed']} | {'yes' if row.get('success') else 'no'} | "
            f"{fmt('coverage_rate')} | {fmt('repeat_rate')} | {fmt('actual_total_distance_m')} | "
            f"{fmt('brush_off_distance_m')} | {fmt('actual_duration_sec', 1)} | "
            f"{row.get('targets_cleaned', 'n/a')}/{row.get('targets_total', 'n/a')} | "
            f"{fmt('localization_rmse_m', 4)} | {fmt('lateral_error_p95_m', 4)} |"
        )
    lines.extend(["", "## Hard gates", ""])
    lines.extend(f"- [{'x' if passed else ' '}] {name}" for name, passed in report["gates"].items())
    lines.extend(["", "## Median reductions", ""])
    for key, value in report["comparison"].items():
        lines.append(f"- {key}: {'n/a' if value is None else f'{value:.2%}'}")
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_manifest(root, output):
    files = []
    for path in sorted(item for item in root.rglob("*") if item.is_file() and item != output):
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        files.append({
            "path": path.relative_to(root).as_posix(),
            "bytes": path.stat().st_size,
            "sha256": digest.hexdigest(),
        })
    output.write_text(json.dumps({
        "schema": "tzcup.artifact_manifest.v1", "files": files,
    }, indent=2) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    report = build_report(root)
    json_path = root / "comparison_report.json"
    markdown_path = root / "comparison_report.md"
    manifest_path = root / "artifact_manifest.json"
    json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    write_markdown(report, markdown_path)
    write_manifest(root, manifest_path)
    print(json.dumps({"pass": report["pass"], "runs": len(report["baseline"]) + len(report["selected"])}))


if __name__ == "__main__":
    main()
