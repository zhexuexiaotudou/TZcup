#!/usr/bin/env python3
"""Calibrate on G7-MOVING HOLDOUT and open MOVING VAL once for CRV6-04."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception"))
from sanitation_learning.opr_c_rtmdet import patch_mmdet_cuda_nms  # noqa: E402
from sanitation_perception.rtmdet_product_runtime import RTMDetProductRuntime, file_sha256  # noqa: E402

THRESHOLDS = tuple(round(value / 100, 2) for value in range(5, 96))
CLASSES = ("plastic_bottle", "metal_can", "paper_litter")


def jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def iou(a, b) -> float:
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    return intersection / max(area_a + area_b - intersection, 1e-12)


def match_frame(truths: list[dict], predictions: list[dict], threshold: float, iou_threshold: float = 0.5) -> dict:
    selected = [row for row in predictions if row["score"] >= threshold]
    remaining = set(range(len(truths)))
    matches, wrong = [], []
    for prediction in sorted(selected, key=lambda row: row["score"], reverse=True):
        ranked = sorted(((iou(prediction["bbox_xyxy"], truths[index]["bbox_xyxy"]), index) for index in remaining), reverse=True)
        if not ranked or ranked[0][0] < iou_threshold:
            continue
        overlap, index = ranked[0]
        remaining.remove(index)
        item = {"prediction": prediction, "truth": truths[index], "iou": overlap}
        (matches if prediction["class_name"] == truths[index]["class_name"] else wrong).append(item)
    return {"predictions": selected, "matches": matches, "wrong_class_matches": wrong, "unmatched_truth": [truths[i] for i in remaining]}


def average_precision(frames: list[dict], iou_threshold: float) -> float:
    values = []
    for class_name in CLASSES:
        truth_count = sum(sum(t["class_name"] == class_name for t in frame["truths"]) for frame in frames)
        ranked = sorted(
            ((p["score"], frame_index, p) for frame_index, frame in enumerate(frames) for p in frame["predictions"] if p["class_name"] == class_name),
            reverse=True, key=lambda row: row[0],
        )
        used: dict[int, set[int]] = {}
        tp, fp, precisions, recalls = 0, 0, [], []
        for _, frame_index, prediction in ranked:
            truths = frames[frame_index]["truths"]
            candidates = sorted(((iou(prediction["bbox_xyxy"], truth["bbox_xyxy"]), idx) for idx, truth in enumerate(truths) if truth["class_name"] == class_name and idx not in used.setdefault(frame_index, set())), reverse=True)
            if candidates and candidates[0][0] >= iou_threshold:
                tp += 1; used[frame_index].add(candidates[0][1])
            else:
                fp += 1
            precisions.append(tp / (tp + fp)); recalls.append(tp / max(truth_count, 1))
        values.append(sum(max((p for p, r in zip(precisions, recalls) if r >= level), default=0.0) for level in [i / 100 for i in range(101)]) / 101)
    return sum(values) / len(values)


def evaluate(frames: list[dict], encounters: list[dict], threshold: float) -> dict:
    matched_targets, detected_targets, wrong_target_matches = set(), set(), set()
    actionable = correct = wrong = negative_actionable = 0
    frame_truth = frame_correct = 0
    per_domain: dict[str, list[int]] = {}
    first: dict[str, dict] = {}
    scores = {name: [] for name in CLASSES}
    for frame in frames:
        result = match_frame(frame["truths"], frame["predictions"], threshold)
        actionable += len(result["predictions"])
        correct += len(result["matches"]); wrong += len(result["wrong_class_matches"])
        frame_truth += len(frame["truths"]); frame_correct += len(result["matches"])
        if frame["negative_only"]:
            negative_actionable += len(result["predictions"])
        for prediction in result["predictions"]:
            scores[prediction["class_name"]].append(prediction["score"])
        for item in result["matches"]:
            target = item["truth"]["target_id"]; matched_targets.add(target); detected_targets.add(target)
            if target not in first:
                first[target] = {"frame_index": frame["frame_index"], "distance_m": item["truth"]["distance_m"], "time_s": frame["frame_index"] / 15.0}
        for item in result["wrong_class_matches"]:
            target = item["truth"]["target_id"]; detected_targets.add(target); wrong_target_matches.add(target)
    eligible = {row["target_id"]: row for row in encounters if row["actionable"]}
    per_class = {name: sum(target in matched_targets for target, row in eligible.items() if row["class_name"] == name) / max(sum(row["class_name"] == name for row in eligible.values()), 1) for name in CLASSES}
    small = [target for target, row in eligible.items() if row["first_visible_small"]]
    domains = sorted(set(frame["surface_domain"] for frame in frames))
    for domain in domains:
        domain_targets = {t["target_id"] for frame in frames if frame["surface_domain"] == domain for t in frame["truths"] if t["target_id"] in eligible}
        per_domain[domain] = [sum(t in matched_targets for t in domain_targets), len(domain_targets)]
    ap50 = average_precision(frames, 0.5)
    ap5095 = sum(average_precision(frames, value / 100) for value in range(50, 96, 5)) / 10
    metrics = {
        "threshold": threshold, "eligible_encounters": len(eligible),
        "eventual_detection_recall": len(detected_targets & set(eligible)) / max(len(eligible), 1),
        "eventual_correct_class_recall": len(matched_targets & set(eligible)) / max(len(eligible), 1),
        "per_class_eventual_recall": per_class,
        "small_eventual_recall": sum(t in matched_targets for t in small) / max(len(small), 1),
        "actionable_precision": correct / max(actionable, 1),
        "wrong_actionable_rate": wrong / max(actionable, 1),
        "negative_moving_actionable_rate": negative_actionable / max(sum(frame["negative_only"] for frame in frames), 1),
        "frame_recall": frame_correct / max(frame_truth, 1), "frame_precision": correct / max(actionable, 1),
        "AP50": ap50, "AP50_95": ap5095,
        "per_domain_recall": {name: value[0] / max(value[1], 1) for name, value in per_domain.items()},
        "first_detection": first,
        "score_histograms": {name: {"count": len(values), "minimum": min(values) if values else None, "median": sorted(values)[len(values)//2] if values else None, "maximum": max(values) if values else None} for name, values in scores.items()},
    }
    gates = {
        "eventual_detection_recall": metrics["eventual_detection_recall"] >= 0.95,
        "eventual_correct_class_recall": metrics["eventual_correct_class_recall"] >= 0.95,
        "plastic_bottle_eventual": per_class["plastic_bottle"] >= 0.90,
        "metal_can_eventual": per_class["metal_can"] >= 0.95,
        "paper_litter_eventual": per_class["paper_litter"] >= 0.95,
        "small_eventual": metrics["small_eventual_recall"] >= 0.90,
        "actionable_precision": metrics["actionable_precision"] >= 0.95,
        "wrong_actionable_rate": metrics["wrong_actionable_rate"] <= 0.01,
        "negative_moving_actionable_rate": metrics["negative_moving_actionable_rate"] <= 0.01,
    }
    metrics["gates"] = gates; metrics["all_required_gates_pass"] = all(gates.values())
    metrics["gate_distance"] = sum((0 if value else 1) for value in gates.values())
    return metrics


def load_split(root: Path, split: str, runtime) -> tuple[list[dict], list[dict]]:
    rows = [row for row in jsonl(root / "G7_MOVING_FRAME_MANIFEST.jsonl") if row["split"] == split]
    encounters = [row for row in jsonl(root / "G7_MOVING_EVALUATOR_ENCOUNTERS.jsonl") if row["split"] == split]
    frames = []
    for row in rows:
        gt = json.loads((root / row["evaluator_gt_path"]).read_text(encoding="utf-8"))
        image = cv2.imread(str(root / row["rgb_path"]), cv2.IMREAD_COLOR)
        frames.append({**row, "truths": gt["objects"], "predictions": runtime.infer_bgr(image)})
    return frames, encounters


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True, type=Path); parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--checkpoint", required=True, type=Path); parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    patch_mmdet_cuda_nms(); started = time.perf_counter()
    runtime = RTMDetProductRuntime(args.config, args.checkpoint, expected_sha256=args.expected_sha256, observation_threshold=0.05, action_threshold=0.05)
    holdout_frames, holdout_encounters = load_split(args.data_root, "MOVING_HOLDOUT", runtime)
    sweep = [evaluate(holdout_frames, holdout_encounters, threshold) for threshold in THRESHOLDS]
    selected = min(sweep, key=lambda row: (not row["all_required_gates_pass"], row["gate_distance"], -row["eventual_correct_class_recall"], -row["actionable_precision"], row["threshold"]))
    selection = {"stage": "CRV6-04-SELECTION", "selection_data": "G7_MOVING_HOLDOUT_ONLY", "selected_threshold": selected["threshold"], "selected_metrics": selected, "threshold_sweep": sweep, "MOVING_VAL_read_before_selection_freeze": False, "MOVING_VAL_used_for_selection": False}
    selection_path = args.output / "CRV6_MOVING_SELECTION.json"; selection_path.write_text(json.dumps(selection, indent=2) + "\n", encoding="utf-8")
    val_frames, val_encounters = load_split(args.data_root, "MOVING_VAL", runtime)
    val = evaluate(val_frames, val_encounters, selected["threshold"])
    report = {"schema_version": 1, "protocol": "CHECKPOINT-RECONSTITUTION-V6", "stage": "CRV6-04", "candidate_sha256": args.expected_sha256, "selection_sha256": file_sha256(selection_path), "threshold_selected_on": "G7_MOVING_HOLDOUT_ONLY", "MOVING_VAL_evaluation_count": 1, "MOVING_VAL_used_for_selection": False, "VAL": val, "MOVING_NATIVE_DETECTOR_PASS": val["all_required_gates_pass"], "duration_s": time.perf_counter() - started, "G5_read": False, "G5_V2_read": False}
    (args.output / "CRV6_NATIVE_MOVING_REPORT.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0 if report["MOVING_NATIVE_DETECTOR_PASS"] else 4


if __name__ == "__main__": raise SystemExit(main())
