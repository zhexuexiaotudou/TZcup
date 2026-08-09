#!/usr/bin/env python3
"""Development-only root-cause diagnostics for a failed AUTO-05R P4 run.

The script reconstructs the exact train-world holdout and cross-world VAL
partitions used by :mod:`auto05r_screening`, then decomposes small-object
failures before and after crop classification.  It intentionally asks the
manifest loader for development roles only; legacy D6 and sealed G5 are never
resolved or read.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np
import torch


ROOT = Path(__file__).resolve().parents[1]
LEARNING_PACKAGE = ROOT / "starter_ws" / "src" / "sanitation_learning"
sys.path.insert(0, str(LEARNING_PACKAGE))
sys.path.insert(0, str(ROOT / "scripts"))

from sanitation_learning.auto04_contract import box_iou  # noqa: E402
from sanitation_learning.g4_data import (  # noqa: E402
    index_instance_records,
    load_frame_rows,
    load_instance_records,
    load_scene_manifests,
)
from sanitation_learning.g4_evaluation import (  # noqa: E402
    classify_detections,
    discrete_metrics,
    discovery_metrics,
    discovery_predictions,
    match_discrete_predictions,
)
from sanitation_learning.g4_split_policy import (  # noqa: E402
    DEVELOPMENT_ROLES,
    partition_rows,
)
from auto05r_screening import (  # noqa: E402
    _holdout_rows,
    _load_reused_model,
    _tag_rows_with_scene_metadata,
)


SMALL_NATIVE_SHORT_SIDE_PX = 18.0


def _filter_frames(frames: list[dict], threshold: float) -> list[dict]:
    return [
        {
            **frame,
            "detections": [
                item
                for item in frame["detections"]
                if float(item["score"]) >= threshold
            ],
        }
        for frame in frames
    ]


def _candidate_match_summary(frames: list[dict]) -> dict:
    totals = {"small": 0, "non_small": 0}
    matched = {"small": 0, "non_small": 0}
    best_iou_by_size = {"small": [], "non_small": []}
    for frame in frames:
        for truth in frame["truth"]:
            size_group = (
                "small"
                if float(truth.get("native_short_side_px", 0.0))
                < SMALL_NATIVE_SHORT_SIDE_PX
                else "non_small"
            )
            best_iou = max(
                (
                    box_iou(
                        tuple(float(value) for value in truth["bbox_xyxy"]),
                        tuple(float(value) for value in detection["bbox_xyxy"]),
                    )
                    for detection in frame["detections"]
                ),
                default=0.0,
            )
            totals[size_group] += 1
            matched[size_group] += int(best_iou >= 0.5)
            best_iou_by_size[size_group].append(best_iou)
    return {
        group: {
            "total": totals[group],
            "matched_iou50": matched[group],
            "recall_iou50": matched[group] / max(totals[group], 1),
            "mean_best_iou": float(np.mean(best_iou_by_size[group]))
            if best_iou_by_size[group]
            else 0.0,
        }
        for group in ("small", "non_small")
    }


def _small_failure_reasons(classified_frames: list[dict]) -> dict:
    reasons = {
        "matched_correct_class": 0,
        "no_candidate_iou50": 0,
        "candidate_rejected_as_background": 0,
        "candidate_wrong_class": 0,
        "candidate_correct_class_but_unmatched": 0,
    }
    examples: list[dict] = []
    for frame in classified_frames:
        for truth in frame["truth"]:
            if (
                float(truth.get("native_short_side_px", 0.0))
                >= SMALL_NATIVE_SHORT_SIDE_PX
            ):
                continue
            candidates = []
            for detection, prediction in zip(
                frame["detections"], frame.get("predictions", [])
            ):
                iou = box_iou(
                    tuple(float(value) for value in truth["bbox_xyxy"]),
                    tuple(float(value) for value in detection["bbox_xyxy"]),
                )
                if iou >= 0.5:
                    candidates.append((iou, detection, prediction))
            expected_class = str(truth["semantic_class"])
            if not candidates:
                reason = "no_candidate_iou50"
                best = None
            else:
                best = max(candidates, key=lambda item: item[0])
                predictions = [item[2] for item in candidates]
                if any(
                    item.get("class_name") == expected_class
                    and int(item.get("class_index", 0)) > 0
                    for item in predictions
                ):
                    reason = "matched_correct_class"
                elif all(int(item.get("class_index", 0)) == 0 for item in predictions):
                    reason = "candidate_rejected_as_background"
                elif any(int(item.get("class_index", 0)) > 0 for item in predictions):
                    reason = "candidate_wrong_class"
                else:
                    reason = "candidate_correct_class_but_unmatched"
            reasons[reason] += 1
            if reason != "matched_correct_class" and len(examples) < 80:
                examples.append(
                    {
                        "world_id": frame["world_id"],
                        "scene_seed": frame["scene_seed"],
                        "frame_index": frame["frame_index"],
                        "semantic_class": expected_class,
                        "native_short_side_px": truth.get("native_short_side_px"),
                        "truth_bbox_xyxy": truth["bbox_xyxy"],
                        "reason": reason,
                        "best_candidate_iou": best[0] if best else 0.0,
                        "best_candidate_score": (
                            float(best[1]["score"]) if best else None
                        ),
                        "classifier_result": best[2] if best else None,
                    }
                )
    return {"counts": reasons, "examples": examples}


def _false_candidate_examples(frames: list[dict]) -> list[dict]:
    records = []
    for frame in frames:
        used_truth: set[int] = set()
        for detection in sorted(
            frame["detections"],
            key=lambda item: float(item["score"]),
            reverse=True,
        ):
            best_iou = 0.0
            best_index = -1
            for index, truth in enumerate(frame["truth"]):
                if index in used_truth:
                    continue
                iou = box_iou(
                    tuple(float(value) for value in truth["bbox_xyxy"]),
                    tuple(float(value) for value in detection["bbox_xyxy"]),
                )
                if iou > best_iou:
                    best_iou = iou
                    best_index = index
            if best_index >= 0 and best_iou >= 0.5:
                used_truth.add(best_index)
                continue
            x1, y1, x2, y2 = (float(v) for v in detection["bbox_xyxy"])
            records.append(
                {
                    "score": float(detection["score"]),
                    "best_truth_iou": best_iou,
                    "bbox_xyxy": detection["bbox_xyxy"],
                    "bbox_width": x2 - x1,
                    "bbox_height": y2 - y1,
                    "world_id": frame["world_id"],
                    "scene_seed": frame["scene_seed"],
                    "frame_index": frame["frame_index"],
                    "negative_only": frame["negative_only"],
                    "taxonomies": list(frame["row"].get("taxonomies", ())),
                }
            )
    return sorted(records, key=lambda item: item["score"], reverse=True)


def _split_diagnostics(
    discovery,
    classifier,
    rows: list[dict],
    instances_by_key: dict,
    *,
    device,
    discovery_threshold: float,
    classifier_threshold: float,
) -> dict:
    minimum_threshold = 0.50
    all_frames = discovery_predictions(
        discovery,
        rows,
        instances_by_key,
        device=device,
        threshold=minimum_threshold,
    )
    sweep = []
    for threshold in (0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95):
        frames = _filter_frames(all_frames, threshold)
        metrics = discovery_metrics(frames)
        sweep.append(
            {
                "threshold": threshold,
                "candidate_recall": metrics["all_gt_candidate_recall"],
                "precision": metrics["precision"],
                "false_candidates_per_min": metrics[
                    "false_candidates_per_min"
                ],
                "total_false_positives": metrics["total_false_positives"],
                "size_groups": _candidate_match_summary(frames),
            }
        )
    selected_frames = _filter_frames(all_frames, discovery_threshold)
    classified = classify_detections(
        classifier,
        selected_frames,
        device=device,
        class_threshold=classifier_threshold,
    )
    matched = match_discrete_predictions(classified)
    return {
        "rows": len(rows),
        "threshold_sweep": sweep,
        "selected_candidate_metrics": discovery_metrics(selected_frames),
        "selected_candidate_size_groups": _candidate_match_summary(
            selected_frames
        ),
        "selected_final_metrics": discrete_metrics(matched),
        "selected_small_failure_reasons": _small_failure_reasons(classified),
        "selected_false_candidate_examples": _false_candidate_examples(
            selected_frames
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--evidence-dir", required=True, type=Path)
    parser.add_argument("--model-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    report = json.loads(
        (args.model_dir / "auto05r_screening_report.json").read_text(
            encoding="utf-8"
        )
    )
    architecture = report["student_route"]["discovery_architecture"]
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    # Use an isolated scratch copy destination because the audited loader never
    # mutates the selected source checkpoints.
    scratch = args.output.parent / f".{args.output.stem}-checkpoint-audit"
    scratch.mkdir(parents=True, exist_ok=True)
    discovery, _ = _load_reused_model(
        "discovery",
        args.model_dir,
        scratch,
        device,
        discovery_architecture=architecture,
    )
    classifier, _ = _load_reused_model(
        "classifier", args.model_dir, scratch, device
    )

    rows = load_frame_rows(
        args.evidence_dir / "g4_frame_manifest.jsonl",
        args.data_root,
        allowed_splits=DEVELOPMENT_ROLES,
    )
    allowed_keys = {
        (int(row["scene_seed"]), int(row["frame_index"])) for row in rows
    }
    records = load_instance_records(
        args.evidence_dir / "g4_instance_records.jsonl",
        allowed_frame_keys=allowed_keys,
    )
    instances_by_key = index_instance_records(records)
    rows = _tag_rows_with_scene_metadata(
        rows, load_scene_manifests(args.data_root, rows)
    )
    by_role = partition_rows(rows)
    holdout_raw = _holdout_rows(by_role["train"])
    holdout = [
        {**row, "split": "train_world_holdout"} for row in holdout_raw
    ]
    val_rows = by_role["val"]

    thresholds = report["thresholds"]
    result = {
        "schema_version": 1,
        "task": "AUTO-05R-P4 discovery root-cause diagnostic",
        "decision_role": "diagnostic_only_not_a_gate",
        "source_report": str(
            args.model_dir / "auto05r_screening_report.json"
        ),
        "source_p4_pass": report.get("P4_SCREENING_PASS"),
        "discovery_architecture": architecture,
        "sealed_G5_accessed": False,
        "legacy_G4_D6_accessed": False,
        "thresholds": {
            "discovery": float(thresholds["discovery"]),
            "classifier": float(thresholds["classifier"]),
        },
        "in_domain": _split_diagnostics(
            discovery,
            classifier,
            holdout,
            instances_by_key,
            device=device,
            discovery_threshold=float(thresholds["discovery"]),
            classifier_threshold=float(thresholds["classifier"]),
        ),
        "cross_world": _split_diagnostics(
            discovery,
            classifier,
            val_rows,
            instances_by_key,
            device=device,
            discovery_threshold=float(thresholds["discovery"]),
            classifier_threshold=float(thresholds["classifier"]),
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
