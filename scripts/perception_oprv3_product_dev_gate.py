#!/usr/bin/env python3
"""Fail-closed OPRV3-07 product development gate aggregator."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re


THRESHOLDS = {
    "eventual_detection_recall": 0.95,
    "eventual_correct_class_recall": 0.95,
    "small_object_eventual_recall": 0.90,
    "metal_can_eventual_recall": 0.90,
    "paper_eventual_recall": 0.95,
    "actionable_target_precision": 0.95,
    "false_actionable_target_rate": 0.01,
    "map_localization_coverage": 0.95,
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


def build_report(
    moving: dict,
    area_static: dict,
    product_map: dict | None = None,
    performance: dict | None = None,
) -> dict:
    if not moving.get("OPRV3_02_pass"):
        raise ValueError("OPRV3-02 moving core has not passed")
    source_commits = moving.get("source_commits", [])
    if (
        len(source_commits) != 1
        or not re.fullmatch(r"[0-9a-f]{40}", str(source_commits[0]))
    ):
        raise ValueError("OPRV3-02 moving evidence lacks one full source commit")
    if moving.get("G5_SEALED_FINAL_read") is not False:
        raise ValueError("moving evidence violates the sealed-final boundary")
    if area_static.get("G5_SEALED_FINAL_read") is not False:
        raise ValueError("area evidence violates the sealed-final boundary")
    if product_map is not None:
        if product_map.get("G5_SEALED_FINAL_read") is not False:
            raise ValueError("product map evidence violates the sealed-final boundary")
        if product_map.get("GT_used_by_product_pipeline") is not False:
            raise ValueError("product map pipeline consumed GT")
    if performance is not None and performance.get(
        "G5_SEALED_FINAL_read"
    ) is not False:
        raise ValueError("performance evidence violates the sealed-final boundary")

    aggregate = moving["aggregate_mrv2_a"]
    breakdown = moving["development_breakdown"]
    per_class = breakdown["per_class_eventual_detection_recall"]
    actions = int(aggregate["actionable_predictions"])
    wrong = int(aggregate["wrong_actionable_predictions"])
    detector_actionable_precision = (actions - wrong) / max(actions, 1)
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
    product_metrics = (product_map or {}).get("metrics", {})
    actionable_precision = product_metrics.get("product_target_precision")
    false_actionable_rate = product_metrics.get("false_confirmed_target_rate")
    official_object_recall = product_metrics.get("map_localization_coverage")
    official_object_f1 = (
        2.0 * actionable_precision * official_object_recall
        / (actionable_precision + official_object_recall)
        if actionable_precision is not None
        and official_object_recall is not None
        and actionable_precision + official_object_recall > 0.0
        else None
    )
    behavior_metrics = {
        "actionable_target_precision": actionable_precision,
        "false_actionable_target_rate": false_actionable_rate,
        "detector_actionable_precision": detector_actionable_precision,
        "detector_wrong_actionable_target_rate": aggregate[
            "wrong_actionable_target_rate"
        ],
        "wrong_class_leading_to_wrong_clean_action": product_metrics.get(
            "wrong_class_leading_to_wrong_clean_action"
        ),
        "pre_fov_target_creation": product_metrics.get(
            "pre_fov_target_creation"
        ),
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
        "wrong_class_leading_to_wrong_clean_action": (
            behavior_metrics["wrong_class_leading_to_wrong_clean_action"] == 0
        ),
        "pre_fov_target_creation": (
            behavior_metrics["pre_fov_target_creation"] == 0
        ),
        "GT_control_violation": behavior_metrics["GT_control_violation"] == 0,
    }
    map_metrics = {
        "map_localization_coverage": product_metrics.get(
            "map_localization_coverage"
        ),
        "map_rmse_m": product_metrics.get("map_rmse_m"),
        "id_consistency": product_metrics.get("id_consistency"),
        "duplicate_target_rate": product_metrics.get("duplicate_target_rate"),
        "track_fragmentation": product_metrics.get("track_fragmentation"),
        "removed_target_stale_action": product_metrics.get(
            "removed_target_stale_action"
        ),
    }
    map_gates = {
        "map_localization_coverage": _at_least(
            map_metrics["map_localization_coverage"],
            THRESHOLDS["map_localization_coverage"],
        ),
        "map_rmse_m": _at_most(
            map_metrics["map_rmse_m"], THRESHOLDS["map_rmse_m"]
        ),
        "id_consistency": _at_least(
            map_metrics["id_consistency"], THRESHOLDS["id_consistency"]
        ),
        "duplicate_target_rate": _at_most(
            map_metrics["duplicate_target_rate"],
            THRESHOLDS["duplicate_target_rate"],
        ),
        "track_fragmentation": _at_most(
            map_metrics["track_fragmentation"],
            THRESHOLDS["track_fragmentation"],
        ),
        "removed_target_stale_action": (
            map_metrics["removed_target_stale_action"] == 0
        ),
    }
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
    performance_source = (performance or {}).get("metrics", performance or {})
    performance_metrics = {
        "effective_hz": performance_source.get("effective_hz"),
        "end_to_end_p95_ms": performance_source.get("end_to_end_p95_ms"),
        "drop_rate": performance_source.get("drop_rate"),
        "formal_product_pipeline_executed": bool(
            performance_source.get("formal_product_pipeline_executed", False)
        ),
    }
    performance_gates = {
        "effective_hz": _at_least(
            performance_metrics["effective_hz"],
            THRESHOLDS["minimum_effective_hz"],
        ),
        "end_to_end_p95_ms": _at_most(
            performance_metrics["end_to_end_p95_ms"],
            THRESHOLDS["maximum_end_to_end_p95_ms"],
        ),
        "drop_rate": _at_most(
            performance_metrics["drop_rate"],
            THRESHOLDS["maximum_drop_rate"],
        ),
        "formal_product_pipeline_executed": performance_metrics[
            "formal_product_pipeline_executed"
        ],
    }
    sections = {
        "official_object_recognition_mapping": {
            "source": "https://developer.horizon.auto/competition/848127658035142656",
            "interpretation": (
                "undefined official accuracy is conservatively mapped to "
                "full-set product-target precision, recall, and F1"
            ),
            "metrics": {
                "object_level_precision": actionable_precision,
                "object_level_recall": official_object_recall,
                "object_level_f1": official_object_f1,
            },
            "gates": {
                "object_level_precision": _at_least(actionable_precision, 0.95),
                "object_level_recall": _at_least(official_object_recall, 0.95),
                "object_level_f1": _at_least(official_object_f1, 0.95),
            },
        },
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
            "pass": all(map_gates.values()),
            "reason": (
                None
                if product_map is not None
                else "current MRV2-A moving observations have not run through the product DynamicTrashMap evaluator"
            ),
        },
        "area": {
            "metrics": area_metrics,
            "gates": area_gates,
            "pass": all(area_gates.values()),
        },
        "performance": {
            "metrics": performance_metrics,
            "gates": performance_gates,
            "pass": all(performance_gates.values()),
            "reason": (
                None
                if performance is not None
                else "no formal MRV2-A product pipeline latency/drop profile exists"
            ),
        },
    }
    sections["official_object_recognition_mapping"]["pass"] = all(
        sections["official_object_recognition_mapping"]["gates"].values()
    )
    failed = [name for name, section in sections.items() if not section["pass"]]
    return {
        "schema_version": 1,
        "protocol": "OPRV3-07",
        "source_commit": source_commits[0],
        "candidate": "MRV2-A",
        "thresholds": THRESHOLDS,
        "sections": sections,
        "failed_sections": failed,
        "OPRV3_X86_DEV_PASS": not failed,
        "MODEL_BLOCKED_INTERNAL": bool(failed),
        "next_action": (
            "OPRV3-06 area recovery, then current-candidate map/track and performance integration"
            if "area" in failed
            else (
                "complete remaining OPRV3-07 sections"
                if failed
                else "create OPRV3-08 x86 freeze"
            )
        ),
        "freeze_allowed": not failed,
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--moving", type=Path, required=True)
    parser.add_argument("--area-static", type=Path, required=True)
    parser.add_argument("--product-map", type=Path)
    parser.add_argument("--performance", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    moving = json.loads(args.moving.read_text(encoding="utf-8"))
    area = json.loads(args.area_static.read_text(encoding="utf-8"))
    product_map = (
        json.loads(args.product_map.read_text(encoding="utf-8"))
        if args.product_map
        else None
    )
    performance = (
        json.loads(args.performance.read_text(encoding="utf-8"))
        if args.performance
        else None
    )
    report = build_report(moving, area, product_map, performance)
    report["inputs"] = {
        "moving": {"path": args.moving.as_posix(), "sha256": sha256(args.moving)},
        "area_static": {
            "path": args.area_static.as_posix(),
            "sha256": sha256(args.area_static),
        },
    }
    if args.product_map:
        report["inputs"]["product_map"] = {
            "path": args.product_map.as_posix(),
            "sha256": sha256(args.product_map),
        }
    if args.performance:
        report["inputs"]["performance"] = {
            "path": args.performance.as_posix(),
            "sha256": sha256(args.performance),
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["OPRV3_X86_DEV_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
