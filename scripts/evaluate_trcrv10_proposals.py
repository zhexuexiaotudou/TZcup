#!/usr/bin/env python3
"""Evaluate a frozen class-agnostic proposal operating point on G10 sequences."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np


TARGET_LABELS = {1, 2, 3}


def iou(first: list[float], second: list[float]) -> float:
    x1, y1 = max(first[0], second[0]), max(first[1], second[1])
    x2, y2 = min(first[2], second[2]), min(first[3], second[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    second_area = max(0.0, second[2] - second[0]) * max(0.0, second[3] - second[1])
    return intersection / max(first_area + second_area - intersection, 1e-12)


def truth_bbox(mask: np.ndarray) -> dict | None:
    ys, xs = np.where(np.isin(mask, tuple(TARGET_LABELS)))
    if not len(xs):
        return None
    bbox = [int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1)]
    return {"bbox_xyxy": bbox, "short_side_px": min(bbox[2] - bbox[0], bbox[3] - bbox[1])}


def longest_consecutive(values: list[int]) -> int:
    best = current = 0
    prior = None
    for value in sorted(set(values)):
        current = current + 1 if prior is not None and value == prior + 1 else 1
        best = max(best, current)
        prior = value
    return best


def evaluate_records(truth: dict[tuple[str, int], dict | None], predictions: dict[tuple[str, int], list[dict]],
                     threshold: float, persistence: int) -> dict:
    scenes = sorted({scene for scene, _ in truth})
    positives, small, detected = [], [], []
    false_proposals = total_frames = 0
    per_scene = []
    for scene in scenes:
        keys = sorted((key for key in truth if key[0] == scene), key=lambda key: key[1])
        has_target = any(truth[key] is not None for key in keys)
        matched_frames = []
        starts_small = False
        for key in keys:
            gt = truth[key]
            if gt is not None and not any(truth[earlier] is not None for earlier in keys if earlier[1] < key[1]):
                starts_small = gt["short_side_px"] < 18
            selected = [row for row in predictions.get(key, []) if row["score"] >= threshold]
            frame_matched = False
            for row in selected:
                matched = gt is not None and iou(row["bbox_xyxy"], gt["bbox_xyxy"]) >= 0.5
                frame_matched |= matched
                false_proposals += int(not matched)
            if frame_matched:
                matched_frames.append(key[1])
            total_frames += 1
        longest = longest_consecutive(matched_frames)
        persistent = longest >= persistence
        if has_target:
            positives.append(scene)
            if starts_small:
                small.append(scene)
            if persistent:
                detected.append(scene)
        per_scene.append({
            "scene": scene,
            "positive": has_target,
            "starts_small": starts_small,
            "matched_frames": matched_frames,
            "longest_consecutive_matches": longest,
            "persistent_candidate": persistent,
        })
    detected_set = set(detected)
    small_detected = sum(scene in detected_set for scene in small)
    metrics = {
        "threshold": threshold,
        "persistence_frames": persistence,
        "positive_missions": len(positives),
        "small_start_missions": len(small),
        "eventual_proposal_recall": len(detected) / max(len(positives), 1),
        "small_eventual_proposal_recall": small_detected / max(len(small), 1),
        "false_proposals": false_proposals,
        "frames": total_frames,
        "proposal_fp_per_frame": false_proposals / max(total_frames, 1),
    }
    gates = {
        "eventual_proposal_recall": metrics["eventual_proposal_recall"] >= 0.98,
        "small_eventual_proposal_recall": metrics["small_eventual_proposal_recall"] >= 0.95,
        "proposal_flood_hard_limit": metrics["proposal_fp_per_frame"] <= 1.0,
        "persistence_in_authorized_range": 2 <= persistence <= 5,
    }
    return {"metrics": metrics, "gates": gates, "missions": per_scene, "pass": bool(positives) and all(gates.values())}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-scenes", type=Path, required=True)
    parser.add_argument("--raw-inference", type=Path, required=True)
    parser.add_argument("--candidate-id", required=True)
    parser.add_argument("--threshold", type=float, required=True)
    parser.add_argument("--persistence", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    raw = json.loads(args.raw_inference.read_text(encoding="utf-8"))
    predictions: dict[tuple[str, int], list[dict]] = defaultdict(list)
    for row in raw["frames"]:
        predictions[(row["scene"], int(row["frame_index"]))].extend(row["detections"])
    truth = {}
    for scene in sorted(path for path in args.capture_scenes.glob("scene_*") if path.is_dir()):
        report = json.loads((scene / "capture_report.json").read_text(encoding="utf-8"))
        for index in range(int(report["captured_frames"])):
            truth[(scene.name, index)] = truth_bbox(np.load(scene / "semantic" / f"frame_{index:02d}.npy"))
    result = evaluate_records(truth, predictions, args.threshold, args.persistence)
    payload = {
        "schema_version": 1,
        "protocol": "TRCRV10",
        "stage": "TRCRV10-03-PROPOSAL-HOLDOUT",
        "candidate_id": args.candidate_id,
        "score_semantics": "class_agnostic_objectness",
        "semantic_gt_role": "offline_evaluator_only",
        "production_runtime_gt_used": False,
        **result,
        "G10_DEV_VAL_SEALED_read": False,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
        "TRCRV10_PROPOSAL_PASS": result["pass"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"metrics": payload["metrics"], "gates": payload["gates"], "pass": payload["pass"]}, indent=2))
    return 0 if payload["pass"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
