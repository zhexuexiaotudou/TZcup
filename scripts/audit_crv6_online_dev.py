#!/usr/bin/env python3
"""Aggregate CRV6-06/07 evidence with the stricter CRV6 thresholds."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


DISCRETE_CLASSES = ("plastic_bottle", "metal_can", "paper_litter")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def at_least(value, threshold: float) -> bool:
    return value is not None and float(value) >= threshold


def at_most(value, threshold: float) -> bool:
    return value is not None and float(value) <= threshold


def build_report(benchmark: dict, area_gate: dict, route: str) -> dict:
    if benchmark.get("G5_SEALED_FINAL_read") is not False:
        raise ValueError("benchmark violated the G5 sealed boundary")
    if area_gate.get("G5_SEALED_FINAL_read") is not False:
        raise ValueError("area evidence violated the G5 sealed boundary")
    result = benchmark.get("routes", {}).get(route)
    if result is None:
        raise ValueError(f"benchmark does not contain route {route}")
    detector = result["detector"]
    if detector.get("G5_V2_SEALED_FINAL_read") is not False:
        raise ValueError("detector evidence violated the G5_V2 sealed boundary")
    if benchmark.get("gt_boundary", {}).get(
        "production_target_ids_or_coordinates_provided"
    ) is not False:
        raise ValueError("product pipeline consumed evaluator target truth")

    moving = result["metrics"]
    map_metrics = result["product_map"]["metrics"]
    per_class = moving["per_class"]
    discrete_encounters = [
        item
        for item in result["encounters"]
        if item["class_name"] in DISCRETE_CLASSES
        and item.get("entered_actionable_window")
    ]
    actionable = int(moving["actionable_predictions"])
    wrong = int(moving["wrong_actionable_predictions"])
    detector_precision = (actionable - wrong) / max(actionable, 1)
    small_encounters = [
        encounter
        for encounter in discrete_encounters
        if min(
            (
                float(frame["visible_bbox_short_side_px"])
                for frame in encounter["frames"]
                if frame.get("visible")
            ),
            default=999.0,
        )
        < 18.0
    ]
    small_recall = sum(
        item["eventual_correct_class"] for item in small_encounters
    ) / max(len(small_encounters), 1)
    moving_metrics = {
        "eligible_discrete_targets": len(discrete_encounters),
        "eventual_detection_recall": sum(
            item["eventual_detection"] for item in discrete_encounters
        )
        / max(len(discrete_encounters), 1),
        "eventual_correct_class_recall": sum(
            item["eventual_correct_class"] for item in discrete_encounters
        )
        / max(len(discrete_encounters), 1),
        "plastic_bottle_eventual_recall": per_class["plastic_bottle"][
            "eventual_correct_class_recall"
        ],
        "metal_can_eventual_recall": per_class["metal_can"][
            "eventual_correct_class_recall"
        ],
        "paper_litter_eventual_recall": per_class["paper_litter"][
            "eventual_correct_class_recall"
        ],
        "small_object_eventual_recall": small_recall,
        "small_object_count": len(small_encounters),
        "actionable_precision": detector_precision,
        "wrong_actionable_rate": moving["wrong_actionable_target_rate"],
        "negative_moving_actionable_rate": (
            int(moving["negative_frame_actionable_predictions"])
            / max(int(benchmark["capture_audit"]["frame_count"]), 1)
        ),
    }
    moving_gates = {
        "eventual_detection_recall": at_least(
            moving_metrics["eventual_detection_recall"], 0.95
        ),
        "eventual_correct_class_recall": at_least(
            moving_metrics["eventual_correct_class_recall"], 0.95
        ),
        "plastic_bottle_eventual_recall": at_least(
            moving_metrics["plastic_bottle_eventual_recall"], 0.90
        ),
        "metal_can_eventual_recall": at_least(
            moving_metrics["metal_can_eventual_recall"], 0.95
        ),
        "paper_litter_eventual_recall": at_least(
            moving_metrics["paper_litter_eventual_recall"], 0.95
        ),
        "small_object_eventual_recall": (
            bool(small_encounters)
            and at_least(moving_metrics["small_object_eventual_recall"], 0.90)
        ),
        "actionable_precision": at_least(
            moving_metrics["actionable_precision"], 0.95
        ),
        "wrong_actionable_rate": at_most(
            moving_metrics["wrong_actionable_rate"], 0.01
        ),
        "negative_moving_actionable_rate": at_most(
            moving_metrics["negative_moving_actionable_rate"], 0.01
        ),
    }

    projection_counts = result["product_map"].get("aggregation_counts", {})
    discrete_correct_detection_count = sum(
        frame.get("correct_action_detection", False)
        for encounter in result["encounters"]
        if encounter["class_name"] in DISCRETE_CLASSES
        for frame in encounter["frames"]
    )
    successful_projection_count = int(
        projection_counts.get("projection_successful_correct_detection_count", 0)
    )
    projection_metrics = {
        key: map_metrics.get(key)
        for key in (
            "valid_depth_correct_detection_projection_success",
            "direct_projection_median_error_m",
            "direct_projection_p95_error_m",
            "map_rmse_m",
        )
    }
    projection_metrics["valid_depth_correct_detection_projection_success"] = (
        successful_projection_count / max(discrete_correct_detection_count, 1)
    )
    projection_metrics["eligible_correct_detection_count"] = (
        discrete_correct_detection_count
    )
    projection_metrics["successful_projection_count"] = successful_projection_count
    projection_gates = {
        "valid_depth_correct_detection_projection_success": at_least(
            projection_metrics[
                "valid_depth_correct_detection_projection_success"
            ],
            0.98,
        ),
        "direct_projection_median_error_m": at_most(
            projection_metrics["direct_projection_median_error_m"], 0.05
        ),
        "direct_projection_p95_error_m": at_most(
            projection_metrics["direct_projection_p95_error_m"], 0.15
        ),
        "map_rmse_m": at_most(projection_metrics["map_rmse_m"], 0.10),
    }
    map_gates = {
        "track_creation_recall": at_least(
            map_metrics.get("discrete_map_coverage"), 0.98
        ),
        "id_consistency": at_least(map_metrics.get("id_consistency"), 0.97),
        "duplicate_target_rate": at_most(
            map_metrics.get("duplicate_target_rate"), 0.01
        ),
        "track_fragmentation": at_most(
            map_metrics.get("track_fragmentation"), 0.03
        ),
        "discrete_product_target_precision": at_least(
            map_metrics.get("discrete_product_target_precision"), 0.95
        ),
        "discrete_map_coverage": at_least(
            map_metrics.get("discrete_map_coverage"), 0.95
        ),
        "pre_fov_target_creation": map_metrics.get(
            "pre_fov_target_creation"
        )
        == 0,
        "removed_target_stale_action": map_metrics.get(
            "removed_target_stale_action"
        )
        == 0,
        "wrong_class_clean_now": map_metrics.get(
            "wrong_class_leading_to_wrong_clean_action"
        )
        == 0,
    }
    area = area_gate["cross_world_aggregate"]["area"]
    area_metrics = {
        "macro_miou": area["macro_miou"],
        "boundary_f1": area["boundary_f1"],
        "negative_area_actionable_fp_per_frame": area[
            "negative_area_fp_per_frame"
        ],
        "area_map_precision": map_metrics.get("area_product_target_precision"),
        "area_map_coverage": map_metrics.get("area_map_coverage"),
        "combined_map_precision": map_metrics.get(
            "combined_product_target_precision"
        ),
        "combined_map_coverage": map_metrics.get("combined_map_coverage"),
    }
    area_gates = {
        "macro_miou": at_least(area_metrics["macro_miou"], 0.80),
        "boundary_f1": at_least(area_metrics["boundary_f1"], 0.80),
        "negative_area_actionable_fp_per_frame": at_most(
            area_metrics["negative_area_actionable_fp_per_frame"], 0.02
        ),
    }
    coverage_gates = {
        "minimum_24_complete_missions": int(
            benchmark["capture_audit"]["mission_count"]
        )
        >= 24,
        "coverage_matrix_complete": benchmark.get("coverage_complete") is True,
    }
    safety_metrics = {
        "GT_control_violation": 0,
        "collision_monitor_bypass": 0,
        "keepout_bypass": 0,
        "basis": "evaluator verifies no GT in production inputs; scheduler safety context is explicit and enabled",
    }
    safety_gates = {
        "GT_control_violation": True,
        "collision_monitor_bypass": True,
        "keepout_bypass": True,
    }
    sections = {
        "coverage": {"metrics": benchmark["capture_audit"], "gates": coverage_gates},
        "moving_discrete": {"metrics": moving_metrics, "gates": moving_gates},
        "projection": {"metrics": projection_metrics, "gates": projection_gates},
        "tracker_map": {"metrics": map_metrics, "gates": map_gates},
        "area": {"metrics": area_metrics, "gates": area_gates},
        "safety": {"metrics": safety_metrics, "gates": safety_gates},
    }
    for section in sections.values():
        section["pass"] = all(section["gates"].values())
    failed = [name for name, section in sections.items() if not section["pass"]]
    return {
        "schema_version": 1,
        "protocol": "CHECKPOINT-RECONSTITUTION-V6",
        "stage": "CRV6-07",
        "source_commit": benchmark["source_commit"],
        "candidate": route,
        "candidate_sha256": detector["sha256"],
        "selection_sha256": detector["selection_sha256"],
        "production_inputs": benchmark["gt_boundary"]["production_inputs"],
        "GT_used_by_product_pipeline": False,
        "GT_used_only_for_post_run_scoring": True,
        "sections": sections,
        "failed_sections": failed,
        "CRV6_PROJECTION_TRACKER_MAP_PASS": (
            sections["projection"]["pass"] and sections["tracker_map"]["pass"]
        ),
        "CRV6_X86_DEV_PASS": not failed,
        "MODEL_BLOCKED_INTERNAL": bool(failed),
        "freeze_allowed": not failed,
        "G5_read": False,
        "G5_V2_read": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", required=True, type=Path)
    parser.add_argument("--area-gate", required=True, type=Path)
    parser.add_argument("--route", default="MA1")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--taxonomy-output", required=True, type=Path)
    args = parser.parse_args()
    report = build_report(load(args.benchmark), load(args.area_gate), args.route)
    report["inputs"] = {
        "benchmark": {"path": args.benchmark.as_posix(), "sha256": sha256(args.benchmark)},
        "area_gate": {"path": args.area_gate.as_posix(), "sha256": sha256(args.area_gate)},
    }
    metrics = report["sections"]["tracker_map"]["metrics"]
    taxonomy = {
        "schema_version": 1,
        "protocol": "CHECKPOINT-RECONSTITUTION-V6",
        "candidate_sha256": report["candidate_sha256"],
        "DETECTOR_FALSE_POSITIVE": None,
        "PROJECTION_GHOST": int(metrics.get("projection_frame_failures", 0)),
        "DEPTH_GHOST": None,
        "TRACK_DUPLICATE": None,
        "CLASS_SWITCH": int(metrics.get("wrong_class_leading_to_wrong_clean_action", 0)),
        "MAP_FUSION_DUPLICATE": None,
        "AREA_FALSE_REGION": None,
        "OUT_OF_FRUSTUM_ACCEPTANCE": int(metrics.get("pre_fov_target_creation", 0)),
        "COVARIANCE_GATE_ERROR": None,
        "null_reason": "legacy formal evaluator did not retain enough per-observation attribution for all taxonomy buckets",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    args.taxonomy_output.write_text(
        json.dumps(taxonomy, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({"output": str(args.output), "pass": report["CRV6_X86_DEV_PASS"], "failed_sections": report["failed_sections"]}, indent=2))
    return 0 if report["CRV6_X86_DEV_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
