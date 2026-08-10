#!/usr/bin/env python3
"""Fail-closed OPRV3-07 product development gate aggregator."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


THRESHOLDS = {
    "eventual_detection_recall": 0.95,
    "eventual_correct_class_recall": 0.95,
    "small_object_eventual_recall": 0.90,
    "metal_can_eventual_recall": 0.90,
    "paper_eventual_recall": 0.95,
    "actionable_target_precision": 0.95,
    "false_actionable_target_rate": 0.01,
    "map_rmse_m": 0.10,
    "id_consistency": 0.95,
    "duplicate_target_rate": 0.01,
    "track_fragmentation": 0.05,
    "leaf_iou": 0.80,
    "puddle_iou": 0.80,
    "macro_miou": 0.80,
    "boundary_f1": 0.75,
    "negative_area_fp_per_frame": 0.05,
    "minimum_effective_hz": 10.0,
    "maximum_end_to_end_p95_ms": 200.0,
    "maximum_drop_rate": 0.01,
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _at_least(value, threshold: float) -> bool:
    return value is not None and float(value) >= threshold


def _at_most(value, threshold: float) -> bool:
    return value is not None and float(value) <= threshold


def build_report(moving: dict, area_static: dict) -> dict:
    if not moving.get("OPRV3_02_pass"):
        raise ValueError("OPRV3-02 moving core has not passed")
    if moving.get("G5_SEALED_FINAL_read") is not False:
        raise ValueError("moving evidence violates the sealed-final boundary")
    if area_static.get("G5_SEALED_FINAL_read") is not False:
        raise ValueError("area evidence violates the sealed-final boundary")

    aggregate = moving["aggregate_mrv2_a"]
    breakdown = moving["development_breakdown"]
    per_class = breakdown["per_class_eventual_detection_recall"]
    actions = int(aggregate["actionable_predictions"])
    wrong = int(aggregate["wrong_actionable_predictions"])
    actionable_precision = (actions - wrong) / max(actions, 1)
    area = area_static["cross_world_aggregate"]["area"]

    object_metrics = {
        "eventual_detection_recall": aggregate["eventual_detection_recall"],
        "eventual_correct_class_recall": aggregate[
            "eventual_correct_class_recall"
        ],
        "small_object_eventual_recall": breakdown[
            "small_object_eventual_recall"
        ],
        "metal_can_eventual_recall": per_class["metal_can"],
        "paper_eventual_recall": per_class["paper_litter"],
    }
    object_gates = {
        key: _at_least(value, THRESHOLDS[key])
        for key, value in object_metrics.items()
    }
    behavior_metrics = {
        "actionable_target_precision": actionable_precision,
        "false_actionable_target_rate": aggregate[
            "wrong_actionable_target_rate"
        ],
        "wrong_class_leading_to_wrong_clean_action": None,
        "pre_fov_target_creation": None,
        "GT_control_violation": 0,
    }
    behavior_gates = {
        "actionable_target_precision": _at_least(
            actionable_precision, THRESHOLDS["actionable_target_precision"]
        ),
        "false_actionable_target_rate": _at_most(
            behavior_metrics["false_actionable_target_rate"],
            THRESHOLDS["false_actionable_target_rate"],
        ),
        "wrong_class_leading_to_wrong_clean_action": False,
        "pre_fov_target_creation": False,
        "GT_control_violation": behavior_metrics["GT_control_violation"] == 0,
    }
    map_metrics = {
        "map_rmse_m": None,
        "id_consistency": None,
        "duplicate_target_rate": None,
        "track_fragmentation": None,
        "removed_target_stale_action": None,
    }
    map_gates = {key: False for key in map_metrics}
    area_metrics = {
        "leaf_iou": area["iou_by_class"]["leaf_pile"],
        "puddle_iou": area["iou_by_class"]["puddle"],
        "macro_miou": area["macro_miou"],
        "boundary_f1": area["boundary_f1"],
        "negative_area_fp_per_frame": area["negative_area_fp_per_frame"],
    }
    area_gates = {
        "leaf_iou": _at_least(area_metrics["leaf_iou"], THRESHOLDS["leaf_iou"]),
        "puddle_iou": _at_least(
            area_metrics["puddle_iou"], THRESHOLDS["puddle_iou"]
        ),
        "macro_miou": _at_least(
            area_metrics["macro_miou"], THRESHOLDS["macro_miou"]
        ),
        "boundary_f1": _at_least(
            area_metrics["boundary_f1"], THRESHOLDS["boundary_f1"]
        ),
        "negative_area_fp_per_frame": _at_most(
            area_metrics["negative_area_fp_per_frame"],
            THRESHOLDS["negative_area_fp_per_frame"],
        ),
    }
    performance_metrics = {
        "effective_hz": None,
        "end_to_end_p95_ms": None,
        "drop_rate": None,
        "formal_product_pipeline_executed": False,
    }
    performance_gates = {
        "effective_hz": False,
        "end_to_end_p95_ms": False,
        "drop_rate": False,
    }
    sections = {
        "object_level_online_discovery": {
            "metrics": object_metrics,
            "gates": object_gates,
            "pass": all(object_gates.values()),
        },
        "precision_and_wrong_behavior": {
            "metrics": behavior_metrics,
            "gates": behavior_gates,
            "pass": all(behavior_gates.values()),
        },
        "map_and_track": {
            "metrics": map_metrics,
            "gates": map_gates,
            "pass": False,
            "reason": "current MRV2-A moving observations have not run through the product DynamicTrashMap evaluator",
        },
        "area": {
            "metrics": area_metrics,
            "gates": area_gates,
            "pass": all(area_gates.values()),
        },
        "performance": {
            "metrics": performance_metrics,
            "gates": performance_gates,
            "pass": False,
            "reason": "no formal MRV2-A product pipeline latency/drop profile exists",
        },
    }
    failed = [name for name, section in sections.items() if not section["pass"]]
    return {
        "schema_version": 1,
        "protocol": "OPRV3-07",
        "candidate": "MRV2-A",
        "thresholds": THRESHOLDS,
        "sections": sections,
        "failed_sections": failed,
        "OPRV3_X86_DEV_PASS": not failed,
        "MODEL_BLOCKED_INTERNAL": bool(failed),
        "next_action": (
            "OPRV3-06 area recovery, then current-candidate map/track and performance integration"
            if "area" in failed
            else "complete remaining OPRV3-07 sections"
        ),
        "freeze_allowed": False,
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--moving", type=Path, required=True)
    parser.add_argument("--area-static", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    moving = json.loads(args.moving.read_text(encoding="utf-8"))
    area = json.loads(args.area_static.read_text(encoding="utf-8"))
    report = build_report(moving, area)
    report["inputs"] = {
        "moving": {"path": args.moving.as_posix(), "sha256": sha256(args.moving)},
        "area_static": {
            "path": args.area_static.as_posix(),
            "sha256": sha256(args.area_static),
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["OPRV3_X86_DEV_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
