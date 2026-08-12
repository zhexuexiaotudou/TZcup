#!/usr/bin/env python3
"""Audit GA1 HOLDOUT detector failures for RGDRV8 without model selection.

The audit replays the already-consumed development HOLDOUT at its frozen GA1
threshold.  It records proposal-level facts and visual hard-negative heuristics;
it does not tune thresholds and never opens VAL, G5/G5_V2, or formal seeds.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception"))
from sanitation_learning.opr_c_rtmdet import patch_mmdet_cuda_nms  # noqa: E402
from sanitation_perception.rtmdet_product_runtime import decode_rtmdet_result  # noqa: E402


CLASSES = ("plastic_bottle", "metal_can", "paper_litter")
MISS_TAXONOMY = (
    "NO_PROPOSAL", "LOW_SCORE_CORRECT_CLASS", "WRONG_CLASS_HIGH_SCORE",
    "BOX_IOU_FAIL", "SMALL_OBJECT", "OCCLUSION", "REFLECTION",
    "DARK_OBJECT", "BACKGROUND_BLEND", "MOTION_BLUR", "OTHER",
)
FALSE_TAXONOMY = (
    "ROAD_PAINT", "SPECULAR_HIGHLIGHT", "WET_ROAD", "SHADOW",
    "LEAF_ORGANIC_CLUTTER", "STONE", "METAL_LIKE_BACKGROUND",
    "PLASTIC_LIKE_BACKGROUND", "PAPER_LIKE_BACKGROUND", "EDGE_OR_SEAM",
    "UNKNOWN_HARD_NEGATIVE",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def repository_commit() -> str:
    injected = os.environ.get("TZCUP_SOURCE_COMMIT", "").strip()
    if injected:
        if not re.fullmatch(r"[0-9a-fA-F]{40}", injected):
            raise RuntimeError("TZCUP_SOURCE_COMMIT must be a full git SHA")
        return injected.lower()
    return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()


def iou(first: list[float], second: list[float]) -> float:
    x1, y1 = max(first[0], second[0]), max(first[1], second[1])
    x2, y2 = min(first[2], second[2]), min(first[3], second[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area_a = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    area_b = max(0.0, second[2] - second[0]) * max(0.0, second[3] - second[1])
    return intersection / max(area_a + area_b - intersection, 1e-12)


def _crop(image: np.ndarray, bbox: list[float]) -> np.ndarray:
    height, width = image.shape[:2]
    x1, y1, x2, y2 = bbox
    xa, ya = max(0, int(math.floor(x1))), max(0, int(math.floor(y1)))
    xb, yb = min(width, int(math.ceil(x2))), min(height, int(math.ceil(y2)))
    return image[ya:yb, xa:xb]


def visual_false_taxonomy(image: np.ndarray, prediction: dict, image_meta: dict) -> tuple[str, dict]:
    """Return a deterministic visual heuristic, explicitly not background GT."""
    crop = _crop(image, prediction["bbox_xyxy"])
    if crop.size == 0:
        return "UNKNOWN_HARD_NEGATIVE", {"method": "empty_crop"}
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mean = float(gray.mean())
    std = float(gray.std())
    bright = float((gray >= 235).mean())
    dark = float((gray <= 45).mean())
    saturation = float(hsv[..., 1].mean())
    edges = cv2.Canny(gray, 60, 150)
    edge_fraction = float((edges > 0).mean())
    x1, y1, x2, y2 = prediction["bbox_xyxy"]
    aspect = max((x2 - x1) / max(y2 - y1, 1.0), (y2 - y1) / max(x2 - x1, 1.0))
    if bright >= 0.15 and std >= 35:
        label = "SPECULAR_HIGHLIGHT"
    elif dark >= 0.65:
        label = "SHADOW"
    elif edge_fraction >= 0.24 and aspect >= 3.0:
        label = "EDGE_OR_SEAM"
    elif mean >= 175 and saturation <= 45 and prediction["class_name"] == "paper_litter":
        label = "ROAD_PAINT"
    elif prediction["class_name"] == "metal_can" and std >= 32:
        label = "METAL_LIKE_BACKGROUND"
    elif prediction["class_name"] == "plastic_bottle" and bright >= 0.04:
        label = "PLASTIC_LIKE_BACKGROUND"
    elif prediction["class_name"] == "paper_litter" and mean >= 130:
        label = "PAPER_LIKE_BACKGROUND"
    elif "wet" in str(image_meta.get("surface", "")).lower():
        label = "WET_ROAD"
    else:
        label = "UNKNOWN_HARD_NEGATIVE"
    return label, {
        "method": "deterministic_visual_heuristic_not_background_ground_truth",
        "mean_gray": mean, "std_gray": std, "bright_fraction": bright,
        "dark_fraction": dark, "mean_saturation": saturation,
        "edge_fraction": edge_fraction, "aspect_ratio": aspect,
    }


def classify_miss(*, truth: dict, all_predictions: list[dict], threshold: float) -> tuple[str, dict]:
    overlaps = sorted(
        [(iou(pred["bbox_xyxy"], truth["bbox_xyxy"]), pred) for pred in all_predictions],
        key=lambda item: (item[0], float(item[1]["score"])), reverse=True,
    )
    correct = [item for item in overlaps if item[1]["class_name"] == truth["class_name"]]
    best_iou, best_pred = overlaps[0] if overlaps else (0.0, None)
    best_correct_iou, best_correct = correct[0] if correct else (0.0, None)
    facts = {
        "best_iou": best_iou,
        "best_predicted_class": best_pred["class_name"] if best_pred else None,
        "best_score": float(best_pred["score"]) if best_pred else 0.0,
        "best_correct_iou": best_correct_iou,
        "best_correct_score": float(best_correct["score"]) if best_correct else 0.0,
    }
    if not overlaps or max(item[0] for item in overlaps) < 0.10:
        primary = "NO_PROPOSAL"
    elif best_correct and best_correct_iou >= 0.5 and float(best_correct["score"]) < threshold:
        primary = "LOW_SCORE_CORRECT_CLASS"
    elif best_pred and best_iou >= 0.5 and best_pred["class_name"] != truth["class_name"] and float(best_pred["score"]) >= threshold:
        primary = "WRONG_CLASS_HIGH_SCORE"
    elif best_iou < 0.5:
        primary = "BOX_IOU_FAIL"
    else:
        primary = "OTHER"
    secondary = []
    if float(truth["bbox_short_side_px"]) < 18:
        secondary.append("SMALL_OBJECT")
    if truth.get("occlusion") not in (None, "none", "full"):
        secondary.append("OCCLUSION")
    return primary, {**facts, "secondary_taxonomy": secondary}


def audit(payload: dict, frame_predictions: dict[int, list[dict]], images_bgr: dict[int, np.ndarray], threshold: float) -> dict[str, dict]:
    images = {int(item["id"]): item for item in payload["images"]}
    categories = {int(item["id"]): item["name"] for item in payload["categories"]}
    annotations: dict[int, list[dict]] = defaultdict(list)
    targets: dict[str, list[dict]] = defaultdict(list)
    for raw in payload["annotations"]:
        x, y, width, height = raw["bbox"]
        image = images[int(raw["image_id"])]
        item = {
            **raw,
            "class_name": categories[int(raw["category_id"])],
            "bbox_xyxy": [x, y, x + width, y + height],
            "frame_index": image["frame_index"],
            "surface": image.get("surface", "unknown"),
            "lighting": image.get("lighting", "unknown"),
            "camera_motion": image.get("camera_motion", "unknown"),
            "occlusion": image.get("occlusion", "unknown"),
            "orientation": image.get("orientation", "unknown"),
            "material_variant": image.get("material_variant", "unknown"),
            "distance": image.get("distance_m"),
        }
        annotations[int(raw["image_id"])].append(item)
        targets[f"{raw['mission_id']}:{raw['target_id']}"] .append(item)

    target_rows = []
    miss_counts: Counter[str] = Counter()
    class_failure: Counter[str] = Counter()
    small_causes: Counter[str] = Counter()
    for target_key, records in sorted(targets.items()):
        actionable = [item for item in records if item.get("actionable")]
        if len(actionable) < 3:
            continue
        correct = False
        detected = False
        best_fact = None
        for truth in actionable:
            predictions = frame_predictions[int(truth["image_id"])]
            primary, facts = classify_miss(truth=truth, all_predictions=predictions, threshold=threshold)
            overlaps = [(iou(pred["bbox_xyxy"], truth["bbox_xyxy"]), pred) for pred in predictions]
            detected = detected or any(overlap >= 0.5 and float(pred["score"]) >= threshold for overlap, pred in overlaps)
            correct = correct or any(overlap >= 0.5 and float(pred["score"]) >= threshold and pred["class_name"] == truth["class_name"] for overlap, pred in overlaps)
            candidate = {**truth, **facts, "miss_taxonomy": primary}
            if best_fact is None or candidate["best_correct_score"] > best_fact["best_correct_score"]:
                best_fact = candidate
        small = min(float(item["bbox_short_side_px"]) for item in records) < 18
        row = {
            "target_key": target_key, "class": records[0]["class_name"],
            "small": small, "eventual_detected": detected,
            "eventual_correct_class": correct, "representative": best_fact,
        }
        if not correct:
            cause = best_fact["miss_taxonomy"]
            miss_counts[cause] += 1
            class_failure[records[0]["class_name"]] += 1
            if small:
                small_causes[cause] += 1
        target_rows.append(row)

    false_rows = []
    false_counts: Counter[str] = Counter()
    wrong_predicted_class: Counter[str] = Counter()
    actionable_count = correct_count = 0
    for image_id, predictions in sorted(frame_predictions.items()):
        truths = annotations[image_id]
        for prediction in predictions:
            if float(prediction["score"]) < threshold:
                continue
            matches = sorted(
                [(iou(prediction["bbox_xyxy"], truth["bbox_xyxy"]), truth) for truth in truths],
                key=lambda item: item[0], reverse=True,
            )
            best = matches[0] if matches else None
            if best and best[0] >= 0.5 and not best[1].get("actionable"):
                continue
            actionable_count += 1
            if best and best[0] >= 0.5 and best[1].get("actionable") and best[1]["class_name"] == prediction["class_name"]:
                correct_count += 1
                continue
            taxonomy, visual = visual_false_taxonomy(images_bgr[image_id], prediction, images[image_id])
            false_counts[taxonomy] += 1
            wrong_predicted_class[prediction["class_name"]] += 1
            false_rows.append({
                "image_id": image_id, "mission_id": images[image_id]["mission_id"],
                "frame_index": images[image_id]["frame_index"],
                "negative_only": images[image_id].get("negative_only", False),
                "predicted_class": prediction["class_name"], "score": float(prediction["score"]),
                "bbox_xyxy": prediction["bbox_xyxy"],
                "best_truth_iou": best[0] if best else 0.0,
                "best_truth_class": best[1]["class_name"] if best else None,
                "taxonomy": taxonomy, "taxonomy_evidence": visual,
            })
    return {
        "targets": target_rows,
        "summary": {
            "eligible_target_count": len(target_rows),
            "correct_target_count": sum(row["eventual_correct_class"] for row in target_rows),
            "small_target_count": sum(row["small"] for row in target_rows),
            "small_correct_count": sum(row["small"] and row["eventual_correct_class"] for row in target_rows),
            "miss_taxonomy": dict(sorted(miss_counts.items())),
            "small_miss_primary_taxonomy": dict(sorted(small_causes.items())),
            "failed_targets_by_class": dict(sorted(class_failure.items())),
        },
        "false_actionable": {
            "actionable_predictions": actionable_count,
            "correct_actionable_predictions": correct_count,
            "false_actionable_count": actionable_count - correct_count,
            "taxonomy": dict(sorted(false_counts.items())),
            "predicted_class_contribution": dict(sorted(wrong_predicted_class.items())),
            "records": false_rows,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--prepared", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--threshold", type=float, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--container-digest", required=True)
    args = parser.parse_args()
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise FileExistsError(args.output_dir)
    if sha256(args.checkpoint) != args.expected_sha256:
        raise RuntimeError("GA1 checkpoint hash mismatch")
    payload = json.loads((args.prepared / "holdout.json").read_text(encoding="utf-8"))
    patch_mmdet_cuda_nms()
    from mmdet.apis import inference_detector, init_detector
    model = init_detector(str(args.config), str(args.checkpoint), device="cuda:0")
    predictions, images_bgr = {}, {}
    for offset in range(0, len(payload["images"]), 8):
        batch = payload["images"][offset:offset + 8]
        batch_images = [cv2.imread(str(args.data_root / item["file_name"]), cv2.IMREAD_COLOR) for item in batch]
        outputs = inference_detector(model, batch_images)
        for item, image, result in zip(batch, batch_images, outputs):
            image_id = int(item["id"])
            images_bgr[image_id] = image
            predictions[image_id] = decode_rtmdet_result(result, observation_threshold=0.01, action_threshold=0.01)
    result = audit(payload, predictions, images_bgr, args.threshold)
    summary = result["summary"]
    false = result["false_actionable"]
    report = {
        "schema_version": 1, "protocol": "REAL-GAZEBO-DETECTOR-RECOVERY-V8",
        "stage": "RGDRV8-00-GA1-FAILURE-FORENSICS",
        "repository_commit": repository_commit(), "container_digest": args.container_digest,
        "checkpoint_sha256": args.expected_sha256, "config_sha256": sha256(args.config),
        "selection_threshold_frozen_from_gocv7": args.threshold,
        "data_role": "LEGACY_GA1_HOLDOUT_DEVELOPMENT_FORENSICS_ONLY",
        "threshold_tuning_performed": False, "VAL_NEW_read": False,
        "G5_read": False, "G5_V2_read": False, "formal_30seed_read": False,
        "answers": {
            "Q1_small_recall_root_cause": max(summary["small_miss_primary_taxonomy"], key=summary["small_miss_primary_taxonomy"].get) if summary["small_miss_primary_taxonomy"] else "NO_SMALL_MISS",
            "Q2_false_actionable_primary_taxonomy": max(false["taxonomy"], key=false["taxonomy"].get) if false["taxonomy"] else "NO_FALSE_ACTIONABLE",
            "Q3_wrong_actionable_largest_predicted_class": max(false["predicted_class_contribution"], key=false["predicted_class_contribution"].get) if false["predicted_class_contribution"] else "NO_FALSE_ACTIONABLE",
        },
        "summary": summary,
        "false_actionable_summary": {key: value for key, value in false.items() if key != "records"},
        "RGDRV8_GA1_FORENSICS_PASS": True,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "RGDRV8_GA1_FAILURE_TAXONOMY.json").write_text(json.dumps({**report, "target_records": result["targets"], "false_actionable_records": false["records"]}, indent=2) + "\n", encoding="utf-8")
    score_distribution = {"threshold": args.threshold, "target_best_correct_scores": [row["representative"]["best_correct_score"] for row in result["targets"]], "false_actionable_scores": [row["score"] for row in false["records"]]}
    (args.output_dir / "RGDRV8_GA1_SCORE_DISTRIBUTION.json").write_text(json.dumps(score_distribution, indent=2) + "\n", encoding="utf-8")
    size_domain = {"scale_buckets": dict(Counter("<8" if row["representative"]["bbox_short_side_px"] < 8 else "8-12" if row["representative"]["bbox_short_side_px"] < 12 else "12-18" if row["representative"]["bbox_short_side_px"] < 18 else ">=18" for row in result["targets"])), "summary": summary}
    (args.output_dir / "RGDRV8_GA1_SIZE_DOMAIN_MATRIX.json").write_text(json.dumps(size_domain, indent=2) + "\n", encoding="utf-8")
    confusion = {"wrong_actionable_predicted_class_contribution": false["predicted_class_contribution"], "failed_targets_by_class": summary["failed_targets_by_class"]}
    (args.output_dir / "RGDRV8_GA1_CONFUSION_MATRIX.json").write_text(json.dumps(confusion, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
