#!/usr/bin/env python3
"""Screen OPR-A specialist proposals through the frozen closed-set classifier."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g4_data import CLASSIFIER_CLASSES, CLASSIFIER_MODEL_SIZE, DISCRETE_NAMES, square_crop  # noqa: E402
from sanitation_learning.g4_models import build_g4_model  # noqa: E402
from sanitation_learning.g6_small_specialist import (  # noqa: E402
    SmallSpecialistDataset,
    build_small_specialist,
    class_agnostic_nms,
    ground_roi_tiles,
    load_g6_rows,
    map_tile_box_to_native,
    small_specialist_collate,
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def iou(left, right) -> float:
    x0, y0 = max(left[0], right[0]), max(left[1], right[1])
    x1, y1 = min(left[2], right[2]), min(left[3], right[3])
    intersection = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    a = max(0.0, left[2] - left[0]) * max(0.0, left[3] - left[1])
    b = max(0.0, right[2] - right[0]) * max(0.0, right[3] - right[1])
    return intersection / (a + b - intersection) if a + b > intersection else 0.0


def all_tile_samples(rows: list[dict]) -> list[dict]:
    return [
        {
            "rgb_path": row["rgb_path"],
            "scene_seed": int(row["scene_seed"]),
            "frame_index": int(row["frame_index"]),
            "split": row["split"],
            "tile_index": index,
            "tile": tile,
            "targets": [],
            "hard_negative": False,
        }
        for row in rows
        for index, tile in enumerate(ground_roi_tiles())
    ]


def specialist_native_candidates(model, rows, threshold, device, batch_size):
    samples = all_tile_samples(rows)
    loader = DataLoader(
        SmallSpecialistDataset(samples),
        batch_size=batch_size,
        shuffle=False,
        num_workers=0,
        collate_fn=small_specialist_collate,
    )
    by_key: dict[tuple[int, int], list[dict]] = {}
    model.eval()
    with torch.no_grad():
        for images, _targets, batch_samples in loader:
            outputs = model([image.to(device) for image in images])
            for output, sample in zip(outputs, batch_samples):
                key = (sample["scene_seed"], sample["frame_index"])
                for box, score in zip(output["boxes"].cpu().tolist(), output["scores"].cpu().tolist()):
                    if float(score) < threshold:
                        continue
                    by_key.setdefault(key, []).append(
                        {
                            "bbox_xyxy": map_tile_box_to_native(box, sample["tile"]),
                            "objectness": float(score),
                            "tile_index": sample["tile_index"],
                        }
                    )
    return {key: class_agnostic_nms(items, 0.5)[:16] for key, items in by_key.items()}


def classify_candidates(classifier, rows, candidates, threshold, device):
    classified: dict[tuple[int, int], list[dict]] = {}
    classifier.eval()
    with torch.no_grad():
        for row in rows:
            key = (int(row["scene_seed"]), int(row["frame_index"]))
            items = candidates.get(key, [])
            if not items:
                continue
            rgb = cv2.cvtColor(cv2.imread(str(row["rgb_path"])), cv2.COLOR_BGR2RGB)
            crops = []
            for item in items:
                crop_box = square_crop(640, 480, tuple(item["bbox_xyxy"]), scale=6.0, minimum_side=64)
                x0, y0, x1, y1 = crop_box
                crop = cv2.resize(rgb[y0:y1, x0:x1], CLASSIFIER_MODEL_SIZE, interpolation=cv2.INTER_AREA)
                crops.append(np.ascontiguousarray(crop.transpose(2, 0, 1), dtype=np.float32) / 255.0)
            logits = classifier(torch.from_numpy(np.stack(crops)).to(device))
            probabilities = torch.softmax(logits, dim=1).cpu().numpy()
            for item, probs in zip(items, probabilities):
                class_index = int(np.argmax(probs))
                class_score = float(probs[class_index])
                if class_index == 0 or class_score < threshold or class_score <= float(probs[0]):
                    continue
                classified.setdefault(key, []).append(
                    {
                        **item,
                        "class_name": CLASSIFIER_CLASSES[class_index],
                        "class_index": class_index,
                        "class_score": class_score,
                        "score": float(item["objectness"]) * class_score,
                    }
                )
    return classified


def evaluate(rows, instances, classified):
    truth_count = matched = false_predictions = 0
    per_class = {name: {"truth": 0, "matched": 0} for name in DISCRETE_NAMES}
    for row in rows:
        key = (int(row["scene_seed"]), int(row["frame_index"]))
        truth = [record for record in instances.get(key, []) if record["class_id"] in DISCRETE_NAMES and int(record["bbox_short_side_px"]) < 18]
        predictions = list(classified.get(key, []))
        truth_count += len(truth)
        unused = set(range(len(predictions)))
        for record in truth:
            class_name = record["class_id"]
            per_class[class_name]["truth"] += 1
            choices = sorted(
                (
                    (iou(record["bbox_xyxy"], predictions[index]["bbox_xyxy"]), index)
                    for index in unused
                    if predictions[index]["class_name"] == class_name
                ),
                reverse=True,
            )
            if choices and choices[0][0] >= 0.5:
                unused.remove(choices[0][1])
                matched += 1
                per_class[class_name]["matched"] += 1
        false_predictions += len(unused)
    precision = matched / (matched + false_predictions) if matched + false_predictions else 1.0
    recall = matched / truth_count if truth_count else 0.0
    return {
        "truth_count": truth_count,
        "matched_correct_class": matched,
        "false_actionable_predictions": false_predictions,
        "correct_class_recall": recall,
        "actionable_precision": precision,
        "false_actionable_rate": false_predictions / max(matched + false_predictions, 1),
        "per_class_recall": {
            name: value["matched"] / value["truth"] if value["truth"] else None
            for name, value in per_class.items()
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--g6-root", type=Path, required=True)
    parser.add_argument("--base-checkpoint", type=Path, required=True)
    parser.add_argument("--specialist-checkpoint", type=Path, required=True)
    parser.add_argument("--specialist-report", type=Path, required=True)
    parser.add_argument("--classifier-checkpoint", type=Path, required=True)
    parser.add_argument("--classifier-threshold", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--batch-size", type=int, default=4)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = json.loads(args.specialist_report.read_text())
    if not report["OPR_A_SPECIALIST_PASS"] or report["data_policy"]["G5_SEALED_FINAL_read"]:
        raise RuntimeError("classifier screen requires passed development-only specialist")
    rows, instances = load_g6_rows(args.g6_root, ("val",))
    if args.max_frames > 0:
        rows = rows[: args.max_frames]
    keys = {(int(row["scene_seed"]), int(row["frame_index"])) for row in rows}
    instances = {key: value for key, value in instances.items() if key in keys}
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("formal OPR-A classifier screening requires CUDA")
    specialist = build_small_specialist(args.base_checkpoint)
    payload = torch.load(args.specialist_checkpoint, map_location="cpu", weights_only=False)
    specialist.load_state_dict(payload["state_dict"], strict=True)
    specialist.to(device)
    classifier = build_g4_model("classifier", from_scratch_control=True)
    classifier_payload = torch.load(args.classifier_checkpoint, map_location="cpu", weights_only=False)
    classifier.load_state_dict(classifier_payload["state_dict"], strict=True)
    classifier.to(device)
    candidates = specialist_native_candidates(
        specialist, rows, float(payload["objectness_threshold"]), device, args.batch_size
    )
    classified = classify_candidates(classifier, rows, candidates, args.classifier_threshold, device)
    result = evaluate(rows, instances, classified)
    gates = {
        "correct_class_recall_at_least_0_95": result["correct_class_recall"] >= 0.95,
        "actionable_precision_at_least_0_95": result["actionable_precision"] >= 0.95,
        "false_actionable_rate_at_most_0_01": result["false_actionable_rate"] <= 0.01,
        "each_class_recall_at_least_0_90": all(value is not None and value >= 0.90 for value in result["per_class_recall"].values()),
    }
    output = {
        "schema_version": 1,
        "stage": "OPRV3-05-OPR-A-CLASSIFIER-SCREEN",
        "route": "OPR-A",
        "data_policy": {"split": "G6_VAL", "frame_count": len(rows), "threshold_selection": "frozen_development_operating_points", "G5_SEALED_FINAL_read": False},
        "thresholds": {"objectness": payload["objectness_threshold"], "classifier": args.classifier_threshold},
        "metrics": result,
        "gates": gates,
        "OPR_A_CLASSIFIER_PASS": all(gates.values()),
        "artifacts": {
            "specialist_sha256": sha256(args.specialist_checkpoint),
            "classifier_sha256": sha256(args.classifier_checkpoint),
        },
        "next_action": "integrate_general_and_run_online_dev_gate" if all(gates.values()) else "retrain_classifier_on_G6_TRAIN_only",
    }
    args.output.write_text(json.dumps(output, indent=2) + "\n")
    print(json.dumps({"metrics": result, "gates": gates}, indent=2), flush=True)
    return 0 if all(gates.values()) else 4


if __name__ == "__main__":
    raise SystemExit(main())
