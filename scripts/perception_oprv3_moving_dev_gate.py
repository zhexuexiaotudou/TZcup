#!/usr/bin/env python3
"""Aggregate raw OPRV3 moving-camera reports into the OPRV3-02 gate."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re


DISCRETE_CLASSES = ("plastic_bottle", "metal_can", "paper_litter")
REQUIRED_COVERAGE = (
    "far_first_appearance",
    "vehicle_gradually_approaches",
    "small_paper_and_can",
    "multiple_world_material_light",
    "negative_regions",
    "behind_vehicle_fov_entry",
    "turning",
    "occlusion",
    "reflection",
)
THRESHOLDS = {
    "minimum_missions": 20,
    "eventual_discrete_recall": 0.95,
    "eventual_correct_class_recall": 0.95,
    "wrong_actionable_target_rate": 0.01,
    "clean_opportunity_miss_rate": 0.02,
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _ratio(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def _eligible(encounter: dict) -> bool:
    return bool(encounter.get("entered_actionable_window")) and not bool(
        encounter.get("insufficient_sampled_actionable_frames")
    )


def _is_small_discrete(encounter: dict) -> bool:
    if encounter.get("class_name") not in DISCRETE_CLASSES:
        return False
    sizes = [
        float(frame.get("visible_bbox_short_side_px", 0.0))
        for frame in encounter.get("frames", [])
        if frame.get("actionable_window")
        and float(frame.get("visible_bbox_short_side_px", 0.0)) > 0.0
    ]
    return bool(sizes) and min(sizes) < 18.0


def build_report(benchmarks: list[dict], *, route: str = "MRV2-A") -> dict:
    if not benchmarks:
        raise ValueError("at least one moving benchmark is required")
    encounters: list[dict] = []
    coverage = {key: False for key in REQUIRED_COVERAGE}
    mission_count = 0
    actionable_predictions = 0
    wrong_actionable_predictions = 0
    negative_frame_actionable_predictions = 0
    source_commits = set()
    for benchmark in benchmarks:
        if benchmark.get("G5_SEALED_FINAL_read") is not False:
            raise ValueError("moving evidence violates the sealed-final boundary")
        route_payload = benchmark.get("routes", {}).get(route)
        if not route_payload:
            raise ValueError(f"moving benchmark has no {route} route")
        source_commits.add(benchmark.get("source_commit", "unavailable"))
        route_metrics = route_payload["metrics"]
        encounters.extend(route_payload.get("encounters", []))
        actionable_predictions += int(route_metrics["actionable_predictions"])
        wrong_actionable_predictions += int(
            route_metrics["wrong_actionable_predictions"]
        )
        negative_frame_actionable_predictions += int(
            route_metrics.get("negative_frame_actionable_predictions", 0)
        )
        audit = benchmark.get("capture_audit", {})
        mission_count += int(audit.get("mission_count", 0))
        required = benchmark.get("required_coverage", {})
        for key in coverage:
            coverage[key] = coverage[key] or bool(required.get(key, False))

    eligible = [item for item in encounters if _eligible(item)]
    discrete = [
        item for item in eligible if item.get("class_name") in DISCRETE_CLASSES
    ]
    small = [item for item in discrete if _is_small_discrete(item)]
    per_class = {}
    for class_name in ("plastic_bottle", "metal_can", "paper_litter", "leaf_pile", "puddle"):
        members = [item for item in eligible if item.get("class_name") == class_name]
        per_class[class_name] = _ratio(
            sum(bool(item.get("eventual_detection")) for item in members),
            len(members),
        )
    aggregate = {
        "eligible_targets": len(eligible),
        "eligible_discrete_targets": len(discrete),
        "eventual_detection_recall": _ratio(
            sum(bool(item.get("eventual_detection")) for item in eligible),
            len(eligible),
        ),
        "eventual_discrete_recall": _ratio(
            sum(bool(item.get("eventual_detection")) for item in discrete),
            len(discrete),
        ),
        "eventual_correct_class_recall": _ratio(
            sum(bool(item.get("eventual_correct_class")) for item in eligible),
            len(eligible),
        ),
        "eventual_track_confirmation_recall": _ratio(
            sum(bool(item.get("eventual_track_confirmation")) for item in eligible),
            len(eligible),
        ),
        "clean_opportunity_miss_rate": _ratio(
            sum(bool(item.get("clean_opportunity_missed")) for item in eligible),
            len(eligible),
        ),
        "actionable_predictions": actionable_predictions,
        "wrong_actionable_predictions": wrong_actionable_predictions,
        "wrong_actionable_target_rate": _ratio(
            wrong_actionable_predictions, actionable_predictions
        ),
        "negative_frame_actionable_predictions": (
            negative_frame_actionable_predictions
        ),
    }
    breakdown = {
        "per_class_eventual_detection_recall": per_class,
        "small_object_definition": (
            "discrete target with at least one actionable GT frame below 18 px short side"
        ),
        "small_object_eligible_targets": len(small),
        "small_object_eventual_recall": _ratio(
            sum(bool(item.get("eventual_detection")) for item in small),
            len(small),
        ),
    }
    gates = {
        "minimum_missions": mission_count >= THRESHOLDS["minimum_missions"],
        "required_coverage": all(coverage.values()),
        "nonempty_discrete_set": bool(discrete),
        "nonempty_small_object_set": bool(small),
        "single_full_source_commit": (
            len(source_commits) == 1
            and bool(re.fullmatch(r"[0-9a-f]{40}", next(iter(source_commits))))
        ),
        "eventual_discrete_recall": (
            aggregate["eventual_discrete_recall"] is not None
            and aggregate["eventual_discrete_recall"]
            >= THRESHOLDS["eventual_discrete_recall"]
        ),
        "eventual_correct_class_recall": (
            aggregate["eventual_correct_class_recall"] is not None
            and aggregate["eventual_correct_class_recall"]
            >= THRESHOLDS["eventual_correct_class_recall"]
        ),
        "wrong_actionable_target_rate": (
            aggregate["wrong_actionable_target_rate"] is not None
            and aggregate["wrong_actionable_target_rate"]
            <= THRESHOLDS["wrong_actionable_target_rate"]
        ),
        "clean_opportunity_miss_rate": (
            aggregate["clean_opportunity_miss_rate"] is not None
            and aggregate["clean_opportunity_miss_rate"]
            <= THRESHOLDS["clean_opportunity_miss_rate"]
        ),
    }
    return {
        "schema_version": 1,
        "protocol": "OPRV3-02",
        "candidate": route,
        "thresholds": THRESHOLDS,
        "mission_count": mission_count,
        "required_coverage": coverage,
        "aggregate_mrv2_a": aggregate,
        "development_breakdown": breakdown,
        "gates": gates,
        "OPRV3_02_pass": all(gates.values()),
        "source_commits": sorted(source_commits),
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", type=Path, action="append", required=True)
    parser.add_argument("--route", default="MRV2-A")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    benchmarks = [
        json.loads(path.read_text(encoding="utf-8")) for path in args.benchmark
    ]
    report = build_report(benchmarks, route=args.route)
    report["inputs"] = [
        {"path": path.as_posix(), "sha256": sha256(path)}
        for path in args.benchmark
    ]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
