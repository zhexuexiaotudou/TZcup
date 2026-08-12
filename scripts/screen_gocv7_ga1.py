#!/usr/bin/env python3
"""Select the GA1 action threshold on world-isolated HOLDOUT only."""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

import cv2


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception"))
from sanitation_learning.opr_c_rtmdet import patch_mmdet_cuda_nms  # noqa: E402
from sanitation_perception.rtmdet_product_runtime import decode_rtmdet_result  # noqa: E402


CLASSES = ("plastic_bottle", "metal_can", "paper_litter")
THRESHOLDS = tuple(round(0.05 + index * 0.02, 2) for index in range(21))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def repository_commit() -> str:
    injected = os.environ.get("TZCUP_SOURCE_COMMIT", "").strip()
    if injected:
        if not re.fullmatch(r"[0-9a-fA-F]{40}", injected):
            raise RuntimeError("TZCUP_SOURCE_COMMIT must be a full git SHA")
        return injected.lower()
    return subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
    ).strip()


def iou(first, second) -> float:
    x1, y1 = max(first[0], second[0]), max(first[1], second[1])
    x2, y2 = min(first[2], second[2]), min(first[3], second[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    second_area = max(0.0, second[2] - second[0]) * max(0.0, second[3] - second[1])
    return intersection / max(first_area + second_area - intersection, 1e-12)


def metrics(payload: dict, frames: dict[int, list[dict]], threshold: float) -> dict:
    images = {int(item["id"]): item for item in payload["images"]}
    categories = {int(item["id"]): item["name"] for item in payload["categories"]}
    annotations = defaultdict(list)
    targets = defaultdict(list)
    for item in payload["annotations"]:
        x, y, width, height = item["bbox"]
        record = {
            **item,
            "class_name": categories[int(item["category_id"])],
            "bbox_xyxy": [x, y, x + width, y + height],
            "frame_index": images[int(item["image_id"])]["frame_index"],
        }
        annotations[int(item["image_id"])].append(record)
        targets[f"{item['mission_id']}:{item['target_id']}"].append(record)
    selected = {
        image_id: [item for item in rows if float(item["score"]) >= threshold]
        for image_id, rows in frames.items()
    }
    eligible = []
    for records in targets.values():
        actionable_rows = [item for item in records if item.get("actionable")]
        if len(actionable_rows) < 3:
            continue
        correct = False
        detected = False
        for truth in actionable_rows:
            rows = selected[int(truth["image_id"])]
            overlaps = [item for item in rows if iou(item["bbox_xyxy"], truth["bbox_xyxy"]) >= 0.5]
            detected = detected or bool(overlaps)
            correct = correct or any(item["class_name"] == truth["class_name"] for item in overlaps)
        eligible.append(
            {
                "class_name": records[0]["class_name"],
                "detected": detected,
                "correct": correct,
                "small": min(item["bbox_short_side_px"] for item in records) < 18,
            }
        )
    actionable_predictions = 0
    correct_predictions = 0
    negative_predictions = 0
    for image_id, rows in selected.items():
        truths = annotations[image_id]
        actionable_truths = [truth for truth in truths if truth.get("actionable")]
        actionable_predictions += len(rows)
        used = set()
        for prediction in rows:
            candidates = [
                (iou(prediction["bbox_xyxy"], truth["bbox_xyxy"]), index, truth)
                for index, truth in enumerate(actionable_truths)
                if index not in used and prediction["class_name"] == truth["class_name"]
            ]
            if candidates:
                overlap, index, _truth = max(candidates, key=lambda item: item[0])
                if overlap >= 0.5:
                    used.add(index)
                    correct_predictions += 1
                    continue
            if images[image_id].get("negative_only"):
                negative_predictions += 1
    per_class = {}
    for class_name in CLASSES:
        rows = [item for item in eligible if item["class_name"] == class_name]
        per_class[class_name] = {
            "eligible": len(rows),
            "eventual_detection_recall": sum(item["detected"] for item in rows) / max(len(rows), 1),
            "eventual_correct_class_recall": sum(item["correct"] for item in rows) / max(len(rows), 1),
        }
    small = [item for item in eligible if item["small"]]
    result = {
        "threshold": threshold,
        "eligible_targets": len(eligible),
        "eventual_detection_recall": sum(item["detected"] for item in eligible) / max(len(eligible), 1),
        "eventual_correct_class_recall": sum(item["correct"] for item in eligible) / max(len(eligible), 1),
        "small_target_count": len(small),
        "small_target_correct_recall": sum(item["correct"] for item in small) / max(len(small), 1),
        "actionable_predictions": actionable_predictions,
        "correct_actionable_predictions": correct_predictions,
        "actionable_precision": correct_predictions / max(actionable_predictions, 1),
        "wrong_actionable_rate": (actionable_predictions - correct_predictions) / max(actionable_predictions, 1),
        "negative_only_actionable_predictions": negative_predictions,
        "per_class": per_class,
    }
    result["all_required_gates_pass"] = all(
        (
            result["eventual_detection_recall"] >= 0.95,
            result["eventual_correct_class_recall"] >= 0.95,
            per_class["metal_can"]["eventual_correct_class_recall"] >= 0.95,
            per_class["paper_litter"]["eventual_correct_class_recall"] >= 0.95,
            result["small_target_correct_recall"] >= 0.90,
            result["actionable_precision"] >= 0.95,
            result["wrong_actionable_rate"] <= 0.01,
        )
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--prepared", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--container-digest", required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    if sha256(args.checkpoint) != args.expected_sha256:
        raise RuntimeError("GA1 checkpoint hash mismatch")
    prep = json.loads((args.prepared / "GOCV7_GA1_DATA_PREP.json").read_text(encoding="utf-8"))
    if prep.get("GA1_PREP_PASS") is not True:
        raise RuntimeError("GA1 prep did not pass")
    payload = json.loads((args.prepared / "holdout.json").read_text(encoding="utf-8"))
    patch_mmdet_cuda_nms()
    from mmdet.apis import inference_detector, init_detector

    model = init_detector(str(args.config), str(args.checkpoint), device="cuda:0")
    frames = {}
    for offset in range(0, len(payload["images"]), 8):
        batch = payload["images"][offset : offset + 8]
        images = [cv2.imread(str(args.data_root / item["file_name"]), cv2.IMREAD_COLOR) for item in batch]
        outputs = inference_detector(model, images)
        for item, result in zip(batch, outputs):
            frames[int(item["id"])] = decode_rtmdet_result(
                result, observation_threshold=0.01, action_threshold=0.01
            )
    sweep = [metrics(payload, frames, threshold) for threshold in THRESHOLDS]
    selected = max(
        sweep,
        key=lambda item: (
            item["all_required_gates_pass"],
            min(item["eventual_correct_class_recall"], item["actionable_precision"]),
            item["eventual_correct_class_recall"],
            item["actionable_precision"],
            item["threshold"],
        ),
    )
    report = {
        "schema_version": 1,
        "protocol": "GAZEBO-ONLINE-CLOSURE-V7",
        "stage": "GOCV7-01-GA1-SELECTION",
        "repository_commit": repository_commit(),
        "container_digest": args.container_digest,
        "selection_data": "GOCV7_GA1_HOLDOUT_ONLY",
        "checkpoint_sha256": args.expected_sha256,
        "config_sha256": sha256(args.config),
        "selected_threshold": selected["threshold"],
        "selected_metrics": selected,
        "threshold_sweep": sweep,
        "existing_24_mission_read_before_selection_freeze": False,
        "G5_read": False,
        "G5_V2_read": False,
        "formal_30seed_read": False,
        "GOCV7_GA1_HOLDOUT_PASS": selected["all_required_gates_pass"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["GOCV7_GA1_HOLDOUT_PASS"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
