#!/usr/bin/env python3
"""Truth-isolated offline re-score of saved formal perception evidence.

This diagnostic never feeds evaluator truth back into product inference.  It
reuses saved postprocessed DOSOD output, reconstructs only the evaluator's
staging-time 2D cube boxes, and verifies stored ground-dirt confusion counts.
The result is deliberately ineligible for formal product acceptance.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
from types import SimpleNamespace

import numpy as np

from sanitation_perception.formal_random_scene_evaluator import _project_cube
from sanitation_perception.formal_random_scene_evaluator_core import (
    BoxObservation,
    TruthBox,
    box_iou,
    match_boxes,
)


CUBE_SCORE_THRESHOLD = 0.005
CUBE_IOU_THRESHOLD = 0.50
CUBE_GATE = 0.80
BASE_FOOTPRINT_TO_BASE_LINK_Z_M = 0.1651
CUBE_SLOTS = tuple(
    (forward, lateral)
    for forward in (1.15, 1.35, 1.55, 1.75, 1.95)
    for lateral in (-0.54, -0.18, 0.18, 0.54)
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _stats(values) -> dict:
    values = np.asarray(list(values), dtype=np.float64)
    if not values.size:
        return {"count": 0, "min": None, "p50": None, "p90": None, "max": None}
    return {
        "count": int(values.size),
        "min": float(values.min()),
        "p50": float(np.percentile(values, 50)),
        "p90": float(np.percentile(values, 90)),
        "max": float(values.max()),
    }


def _matrix_from_tf2_echo(path: Path) -> np.ndarray:
    """Parse the first saved 3x4 matrix from one bounded tf2_echo log."""

    lines = path.read_text(encoding="utf-8").splitlines()
    try:
        start = next(index for index, line in enumerate(lines) if line.strip() == "- Matrix:")
    except StopIteration as exc:
        raise ValueError(f"saved TF evidence has no matrix: {path}") from exc
    rows = []
    for line in lines[start + 1 : start + 4]:
        values = [float(value) for value in re.findall(r"[-+]?\d+(?:\.\d+)?", line)]
        if len(values) != 4:
            raise ValueError(f"invalid saved TF matrix row: {line}")
        rows.append(values)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :] = np.asarray(rows, dtype=np.float64)
    if not np.isfinite(matrix).all() or abs(np.linalg.det(matrix[:3, :3])) < 1e-6:
        raise ValueError("saved TF matrix is non-finite or singular")
    return matrix


def _frozen_truth_boxes(metadata: dict, base_from_camera: np.ndarray) -> list[TruthBox]:
    """Rebuild the evaluator-only fixed ROI in the staging-time base frame."""

    truth_rows = metadata.get("truth_boxes_xyxy", [])
    if len(truth_rows) != len(CUBE_SLOTS):
        raise ValueError("saved diagnostic metadata must contain all 20 staged cube ids")
    shape = metadata.get("image_shape_hwc")
    if shape != [480, 848, 3]:
        raise ValueError(f"unexpected saved D435 image shape: {shape}")
    camera_from_base = np.linalg.inv(np.asarray(base_from_camera, dtype=np.float64))
    info = SimpleNamespace(
        k=[447.2, 0.0, 424.0, 0.0, 433.0, 240.0, 0.0, 0.0, 1.0],
        width=848,
        height=480,
    )
    result = []
    for row, (forward, lateral) in zip(truth_rows, CUBE_SLOTS):
        cube = {
            "object_id": str(row["object_id"]),
            "edge_m": 0.03,
            "pose": {
                "x_m": forward,
                "y_m": lateral,
                # The slot coordinates are relative to base_footprint, while
                # the saved camera matrix is relative to base_link.
                "z_m": -BASE_FOOTPRINT_TO_BASE_LINK_Z_M + 0.015,
            },
        }
        projected = _project_cube(cube, camera_from_base, info)
        if projected is None:
            raise ValueError(f"staged cube did not project into the real D435 frame: {row['object_id']}")
        result.append(projected[0])
    return result


def _detection_distributions(detections: list[dict], truth: list[TruthBox]) -> dict:
    image_area = 848.0 * 480.0
    cube = [item for item in detections if item["class_id"] == "litter_cube"]
    cube_areas = [
        max(0.0, float(item["xyxy"][2]) - float(item["xyxy"][0]))
        * max(0.0, float(item["xyxy"][3]) - float(item["xyxy"][1]))
        for item in cube
    ]
    truth_areas = [
        (item.xyxy[2] - item.xyxy[0]) * (item.xyxy[3] - item.xyxy[1])
        for item in truth
    ]
    nearest_ious = [
        max((box_iou(item["xyxy"], target.xyxy) for target in truth), default=0.0)
        for item in cube
    ]
    same_class_overlaps = [
        box_iou(left["xyxy"], right["xyxy"])
        for index, left in enumerate(cube)
        for right in cube[index + 1 :]
    ]
    cross_class = []
    for index, left in enumerate(detections):
        for right in detections[index + 1 :]:
            if left["class_id"] == right["class_id"]:
                continue
            overlap = box_iou(left["xyxy"], right["xyxy"])
            if overlap >= 0.80:
                cross_class.append(
                    {
                        "classes": sorted((left["class_id"], right["class_id"])),
                        "iou": overlap,
                        "scores": [float(left["confidence"]), float(right["confidence"])],
                    }
                )
    return {
        "postprocess_count_by_class": {
            class_id: sum(item["class_id"] == class_id for item in detections)
            for class_id in ("litter_cube", "fallen_leaves", "dust_or_soil", "puddle")
        },
        "cube_confidence": _stats(item["confidence"] for item in cube),
        "cube_box_area_px2": _stats(cube_areas),
        "cube_box_area_fraction": _stats(area / image_area for area in cube_areas),
        "corrected_truth_box_area_px2": _stats(truth_areas),
        "cube_nearest_corrected_truth_iou": _stats(nearest_ious),
        "same_class_pair_iou": _stats(same_class_overlaps),
        "same_class_pair_count_ge_0_50": sum(value >= 0.50 for value in same_class_overlaps),
        "cross_class_postprocess_pairs_iou_ge_0_80": cross_class,
    }


def _raw_anchor_overlap(raw: dict) -> dict:
    anchors: dict[int, list[dict]] = {}
    for class_id, rows in raw.get("top_raw_candidates_by_class", {}).items():
        for row in rows:
            anchors.setdefault(int(row["anchor_index"]), []).append(
                {"class_id": class_id, "score": float(row["score"])}
            )
    overlaps = [
        {"anchor_index": anchor, "classes": sorted(rows, key=lambda row: row["class_id"])}
        for anchor, rows in sorted(anchors.items())
        if len({row["class_id"] for row in rows}) >= 2
    ]
    return {
        "top10_anchor_count": len(anchors),
        "anchors_shared_by_multiple_classes": len(overlaps),
        "shared_anchors": overlaps,
    }


def rescore_episode(episode_root: Path) -> dict:
    raw_path = episode_root / "dosod_raw_diagnostic.json"
    metadata_path = episode_root / "best_front_frame.json"
    image_path = episode_root / "best_front_frame.png"
    acceptance_path = episode_root / "perception_acceptance.json"
    tf_path = episode_root / "tf2_echo_base_link_front_rgbd_depth_optical_frame.txt"
    for required in (raw_path, metadata_path, image_path, acceptance_path, tf_path):
        if not required.is_file():
            raise FileNotFoundError(f"saved r7 evidence is missing: {required}")
    raw = json.loads(raw_path.read_text(encoding="utf-8"))
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    acceptance = json.loads(acceptance_path.read_text(encoding="utf-8"))
    if raw.get("claim_boundary", {}).get("evaluator_only_offline_diagnostic") is not True:
        raise ValueError("raw input is not evaluator-only diagnostic evidence")
    if raw.get("input", {}).get("real_gazebo_camera_frame") is not True:
        raise ValueError("raw input is not a saved real Gazebo camera frame")
    if raw.get("input", {}).get("image_sha256") != _sha256(image_path):
        raise ValueError("saved diagnostic image hash does not match raw output")

    threshold_key = f"{CUBE_SCORE_THRESHOLD:.3f}"
    detections = raw.get("postprocess_threshold_sweep", {}).get(threshold_key, {}).get(
        "detections", []
    )
    predictions = [
        BoxObservation(
            class_id=str(item["class_id"]),
            confidence=float(item["confidence"]),
            xyxy=tuple(float(value) for value in item["xyxy"]),
        )
        for item in detections
        if item["class_id"] == "litter_cube"
    ]
    truth = _frozen_truth_boxes(metadata, _matrix_from_tf2_echo(tf_path))
    matching = match_boxes(predictions, truth, iou_threshold=CUBE_IOU_THRESHOLD)
    tp = matching["true_positive_count"]
    fp = matching["false_positive_count"]
    fn = matching["false_negative_count"]
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    f1 = 2.0 * precision * recall / (precision + recall) if precision + recall else 0.0

    ground = acceptance["ground_dirt_segmentation"]
    intersection = int(ground["intersection_cell_count"])
    union = int(ground["union_cell_count"])
    predicted = int(ground["predicted_cell_count"])
    truth_cells = int(ground["truth_cell_count"])
    ground_recomputed = {
        "intersection_cell_count": intersection,
        "union_cell_count": union,
        "predicted_cell_count": predicted,
        "truth_cell_count": truth_cells,
        "iou": intersection / union if union else 0.0,
        "precision": intersection / predicted if predicted else 0.0,
        "recall": intersection / truth_cells if truth_cells else 0.0,
        "basis": "stored_episode_aggregate_confusion_counts_unaffected_by_2d_truth_frame_fix",
    }
    return {
        "episode_id": acceptance["episode_id"],
        "inputs": {
            "episode_root": str(episode_root),
            "image_sha256": _sha256(image_path),
            "raw_diagnostic_sha256": _sha256(raw_path),
            "acceptance_report_sha256": _sha256(acceptance_path),
            "front_camera_tf_evidence_sha256": _sha256(tf_path),
        },
        "cube_best_saved_frame_rescore": {
            "scope": "one_saved_real_gazebo_front_frame_not_episode_acceptance",
            "score_threshold": CUBE_SCORE_THRESHOLD,
            "iou_threshold": CUBE_IOU_THRESHOLD,
            "truth_count": len(truth),
            "prediction_count": len(predictions),
            "true_positive_count": tp,
            "false_positive_count": fp,
            "false_negative_count": fn,
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "matched_ious": [float(item["iou"]) for item in matching["matches"]],
        },
        "ground_dirt_episode_rescore": ground_recomputed,
        "postprocess_distributions_at_common_0_005": _detection_distributions(
            detections, truth
        ),
        "raw_top10_cross_class_anchor_overlap": _raw_anchor_overlap(raw),
    }


def build_report(episode_roots: list[Path]) -> dict:
    episodes = [rescore_episode(path) for path in episode_roots]
    cube_rows = [item["cube_best_saved_frame_rescore"] for item in episodes]
    ground_rows = [item["ground_dirt_episode_rescore"] for item in episodes]
    cube_tp = sum(row["true_positive_count"] for row in cube_rows)
    cube_fp = sum(row["false_positive_count"] for row in cube_rows)
    cube_fn = sum(row["false_negative_count"] for row in cube_rows)
    cube_precision = cube_tp / (cube_tp + cube_fp) if cube_tp + cube_fp else 0.0
    cube_recall = cube_tp / (cube_tp + cube_fn) if cube_tp + cube_fn else 0.0
    cube_f1 = (
        2.0 * cube_precision * cube_recall / (cube_precision + cube_recall)
        if cube_precision + cube_recall
        else 0.0
    )
    ground_intersection = sum(row["intersection_cell_count"] for row in ground_rows)
    ground_union = sum(row["union_cell_count"] for row in ground_rows)
    ground_predicted = sum(row["predicted_cell_count"] for row in ground_rows)
    ground_truth = sum(row["truth_cell_count"] for row in ground_rows)
    all_cube_meet = bool(cube_rows) and all(
        row["precision"] >= CUBE_GATE
        and row["recall"] >= CUBE_GATE
        and row["f1"] >= CUBE_GATE
        for row in cube_rows
    )
    return {
        "schema_version": 1,
        "report_id": "tzcup_formal_r7_perception_truth_frame_offline_rescore_v1",
        "claim_boundary": {
            "evaluator_only_offline_diagnostic": True,
            "eligible_as_formal_product_acceptance": False,
            "truth_used_to_modify_product_output": False,
            "threshold_prompt_or_weight_changed": False,
            "single_best_frame_cube_metrics_are_not_episode_metrics": True,
        },
        "frozen_contract": {
            "cube_score_threshold": CUBE_SCORE_THRESHOLD,
            "cube_iou_threshold": CUBE_IOU_THRESHOLD,
            "cube_precision_recall_f1_gate": CUBE_GATE,
            "base_footprint_to_base_link_z_m": BASE_FOOTPRINT_TO_BASE_LINK_Z_M,
            "staged_cube_slot_count": len(CUBE_SLOTS),
        },
        "episodes": episodes,
        "saved_evidence_aggregate": {
            "cube_two_best_frames": {
                "scope": "two_saved_best_frames_not_formal_episode_aggregate",
                "true_positive_count": cube_tp,
                "false_positive_count": cube_fp,
                "false_negative_count": cube_fn,
                "precision": cube_precision,
                "recall": cube_recall,
                "f1": cube_f1,
            },
            "ground_two_episode_confusion_counts": {
                "scope": "stored_formal_r7_episode_aggregate_counts",
                "intersection_cell_count": ground_intersection,
                "union_cell_count": ground_union,
                "iou": ground_intersection / ground_union if ground_union else 0.0,
                "precision": (
                    ground_intersection / ground_predicted if ground_predicted else 0.0
                ),
                "recall": ground_intersection / ground_truth if ground_truth else 0.0,
            },
        },
        "capability_assessment": {
            "cube_best_saved_frames_both_meet_0_8": all_cube_meet,
            "cube_existing_weights_may_reach_gate": all_cube_meet,
            "cube_formal_episode_gate_demonstrated": False,
            "ground_iou_range": [
                min((row["iou"] for row in ground_rows), default=None),
                max((row["iou"] for row in ground_rows), default=None),
            ],
            "ground_existing_path_demonstrates_gate": all(
                row["iou"] >= 0.65 for row in ground_rows
            ),
            "interpretation": (
                "Corrected single-frame geometry can show whether the frozen DOSOD weights "
                "localize the staged cubes, but only a fresh full episode can prove the 0.8 "
                "gate. Stored ground counts remain the authoritative r7 evidence for EdgeSAM."
            ),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode-root", action="append", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    report = build_report(args.episode_root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"output": str(args.output), "report_id": report["report_id"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
