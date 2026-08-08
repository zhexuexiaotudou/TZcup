#!/usr/bin/env python3
"""Auditable P1 control rerun for the retired CenterNet-like discovery head.

The command reads only TRAIN and VAL rows. It creates a deterministic
TRAIN_WORLD_HOLDOUT, compares the three required objectness losses, selects a
threshold on the holdout, evaluates that frozen threshold on cross-world VAL,
and emits L1/L2/L3 loss, gradient, score-histogram and visual-decode evidence.
It never reads legacy G4 test annotations or G5 sealed data.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import random
import sys

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))

from sanitation_learning.g4_data import (  # noqa: E402
    G4DiscoveryDataset,
    index_instance_records,
    load_frame_rows,
    load_instance_records,
)
from sanitation_learning.g4_evaluation import (  # noqa: E402
    discovery_metrics,
    discovery_predictions,
)
from sanitation_learning.g4_losses import (  # noqa: E402
    OBJECTNESS_LOSS_VARIANTS,
    discovery_loss,
)
from sanitation_learning.g4_models import build_g4_model  # noqa: E402
from sanitation_learning.g4_split_policy import (  # noqa: E402
    stratified_row_sample,
)
from sanitation_learning.g4_train import train_discovery  # noqa: E402


SEED = 20260809
CONTROL_GATES = {
    "all_gt_candidate_recall": (">=", 0.75),
    "false_candidates_per_min": ("<=", 20.0),
    "negative_only_fp_per_frame": ("<=", 0.20),
}
THRESHOLDS = tuple(round(value, 2) for value in np.arange(0.05, 0.96, 0.05))
HISTOGRAM_EDGES = tuple(round(value, 1) for value in np.arange(0.0, 1.01, 0.1))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _holdout_rows(rows: list[dict], fraction: float) -> list[dict]:
    if not 0.0 < fraction < 1.0:
        raise ValueError("holdout fraction must be in (0, 1)")
    return [
        row
        for row in rows
        if hashlib.sha256(
            f"{row['world_id']}:{int(row['scene_seed'])}".encode("utf-8")
        ).digest()[0]
        % 100
        < int(fraction * 100)
    ]


def _filtered_frames(frames: list[dict], threshold: float) -> list[dict]:
    return [
        {
            **frame,
            "detections": [
                detection
                for detection in frame["detections"]
                if float(detection["score"]) >= threshold
            ],
        }
        for frame in frames
    ]


def _constraint_distance(metrics: dict) -> float:
    return (
        max(0.0, 0.75 - float(metrics["all_gt_candidate_recall"])) / 0.75
        + max(0.0, float(metrics["false_candidates_per_min"]) - 20.0) / 20.0
        + max(0.0, float(metrics["negative_only_fp_per_frame"]) - 0.20) / 0.20
    )


def _gate_verdict(metrics: dict) -> dict[str, bool]:
    return {
        "recall_at_least_0_75": metrics["all_gt_candidate_recall"] >= 0.75,
        "false_candidates_per_min_at_most_20": (
            metrics["false_candidates_per_min"] <= 20.0
        ),
        "negative_only_fp_per_frame_at_most_0_20": (
            metrics["negative_only_fp_per_frame"] <= 0.20
        ),
    }


def _select_holdout_threshold(frames: list[dict]) -> tuple[float, dict, list[dict]]:
    sweep = []
    for threshold in THRESHOLDS:
        metrics = discovery_metrics(_filtered_frames(frames, threshold))
        gates = _gate_verdict(metrics)
        sweep.append(
            {
                "threshold": threshold,
                "metrics": metrics,
                "gates": gates,
                "all_gates_pass": all(gates.values()),
                "constraint_distance": _constraint_distance(metrics),
            }
        )
    selected = min(
        sweep,
        key=lambda item: (
            not item["all_gates_pass"],
            item["constraint_distance"],
            -item["metrics"]["all_gt_candidate_recall"],
            item["metrics"]["false_candidates_per_min"],
            -item["threshold"],
        ),
    )
    return float(selected["threshold"]), selected, sweep


def _head_gradient_norms(model) -> dict[str, float]:
    groups = {
        "backbone": model.backbone,
        "shared_head": model.head,
        "objectness_head": model.objectness,
        "offset_head": model.offset,
        "bbox_size_head": model.bbox_size,
    }
    result = {}
    for name, module in groups.items():
        squared = 0.0
        for parameter in module.parameters():
            if parameter.grad is not None:
                squared += float(parameter.grad.detach().float().pow(2).sum().cpu())
        result[name] = squared ** 0.5
    return result


def _audit_model(model, rows, instances_by_key, device, variant: str) -> dict:
    dataset = G4DiscoveryDataset(
        rows[: min(64, len(rows))], instances_by_key, augment=False, seed=SEED
    )
    loader = DataLoader(dataset, batch_size=4, shuffle=False, num_workers=0)
    component_sums: dict[str, float] = {}
    positive_scores: list[np.ndarray] = []
    negative_scores: list[np.ndarray] = []
    model.eval()
    model.zero_grad(set_to_none=True)
    first_loss = None
    batches = 0
    for images, targets in loader:
        images = images.to(device)
        targets = {key: value.to(device) for key, value in targets.items()}
        outputs = model(images)
        losses = discovery_loss(
            outputs, targets, objectness_variant=variant
        )
        if first_loss is None:
            first_loss = losses["total"]
        for name in (
            "total",
            "objectness",
            "objectness_positive",
            "objectness_negative",
            "objectness_hard_negative",
            "offset",
            "giou",
            "negative_penalty",
        ):
            component_sums[name] = component_sums.get(name, 0.0) + float(
                losses[name].detach().cpu()
            )
        scores = torch.sigmoid(outputs["objectness_logits"]).detach()
        positive_mask = targets["heatmap"].eq(1.0)
        negative_mask = targets["heatmap"].lt(1.0)
        positive_scores.append(scores[positive_mask].float().cpu().numpy())
        negative_values = scores[negative_mask].flatten()
        if negative_values.numel() > 20000:
            negative_values = negative_values[:20000]
        negative_scores.append(negative_values.float().cpu().numpy())
        batches += 1
    if first_loss is None:
        raise RuntimeError("P1 audit received no batches")
    first_loss.backward()
    gradient_norms = _head_gradient_norms(model)
    model.zero_grad(set_to_none=True)
    positive = np.concatenate(positive_scores) if positive_scores else np.array([])
    negative = np.concatenate(negative_scores) if negative_scores else np.array([])
    positive_hist, _ = np.histogram(positive, bins=HISTOGRAM_EDGES)
    negative_hist, _ = np.histogram(negative, bins=HISTOGRAM_EDGES)
    return {
        "batch_count": batches,
        "frame_count": len(dataset),
        "loss_contributions_mean": {
            key: value / max(batches, 1) for key, value in component_sums.items()
        },
        "gradient_norm_by_head": gradient_norms,
        "score_histogram": {
            "edges": list(HISTOGRAM_EDGES),
            "positive_counts": positive_hist.tolist(),
            "negative_counts": negative_hist.tolist(),
            "positive_samples": int(positive.size),
            "negative_samples": int(negative.size),
            "positive_mean": float(positive.mean()) if positive.size else None,
            "negative_mean": float(negative.mean()) if negative.size else None,
        },
    }


def _write_visual_decode(frames: list[dict], output: Path, count: int = 24) -> list[dict]:
    output.mkdir(parents=True, exist_ok=True)
    positives = [frame for frame in frames if frame["truth"]]
    negatives = [frame for frame in frames if not frame["truth"]]
    selected = (positives[: count // 2] + negatives[: count - count // 2])[:count]
    manifest = []
    for index, frame in enumerate(selected):
        image = cv2.imread(str(frame["row"]["rgb_path"]))
        if image is None:
            raise RuntimeError(f"cannot read {frame['row']['rgb_path']}")
        for truth in frame["truth"]:
            x1, y1, x2, y2 = [int(round(value)) for value in truth["bbox_xyxy"]]
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 220, 0), 2)
        for detection in frame["detections"]:
            x1, y1, x2, y2 = [
                int(round(value)) for value in detection["bbox_xyxy"]
            ]
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(
                image,
                f"{detection['score']:.2f}",
                (x1, max(12, y1 - 3)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0, 0, 255),
                1,
            )
        path = output / f"decode_{index:02d}.png"
        cv2.imwrite(str(path), image)
        manifest.append(
            {
                "path": path.name,
                "sha256": _sha256(path),
                "scene_seed": frame["scene_seed"],
                "frame_index": frame["frame_index"],
                "truth_count": len(frame["truth"]),
                "detection_count": len(frame["detections"]),
            }
        )
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--evidence-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--learning-rate", type=float, default=5e-4)
    parser.add_argument("--holdout-fraction", type=float, default=0.2)
    parser.add_argument("--max-train-frames", type=int, default=0)
    parser.add_argument("--max-eval-frames", type=int, default=0)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    rows = load_frame_rows(
        args.evidence_dir / "g4_frame_manifest.jsonl",
        args.data_root,
        allowed_splits=("train", "val"),
    )
    train_all = [row for row in rows if row["split"] == "train"]
    val_rows = [row for row in rows if row["split"] == "val"]
    holdout_raw = _holdout_rows(train_all, args.holdout_fraction)
    holdout_scenes = {
        (str(row["world_id"]), int(row["scene_seed"])) for row in holdout_raw
    }
    train_rows = [
        row
        for row in train_all
        if (str(row["world_id"]), int(row["scene_seed"])) not in holdout_scenes
    ]
    holdout_rows = [
        {**row, "split": "train_world_holdout"} for row in holdout_raw
    ]
    if args.max_train_frames > 0:
        train_rows = stratified_row_sample(
            train_rows, args.max_train_frames, seed=SEED
        )
    if args.max_eval_frames > 0:
        holdout_rows = stratified_row_sample(
            holdout_rows, args.max_eval_frames, seed=SEED + 1
        )
        val_rows = stratified_row_sample(
            val_rows, args.max_eval_frames, seed=SEED + 2
        )
    if not train_rows or not holdout_rows or not val_rows:
        raise RuntimeError(
            "P1 requires non-empty train/holdout/val, got "
            f"{len(train_rows)}/{len(holdout_rows)}/{len(val_rows)}"
        )
    allowed_keys = {
        (int(row["scene_seed"]), int(row["frame_index"])) for row in rows
    }
    records = load_instance_records(
        args.evidence_dir / "g4_instance_records.jsonl",
        allowed_frame_keys=allowed_keys,
    )
    instances_by_key = index_instance_records(records)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(SEED)
    np.random.seed(SEED)
    random.seed(SEED)
    print(
        f"[P1] train={len(train_rows)} holdout={len(holdout_rows)} "
        f"val={len(val_rows)} legacy_test=not_read device={device}",
        flush=True,
    )

    variants: dict[str, dict] = {}
    live_models = {}
    for variant in OBJECTNESS_LOSS_VARIANTS:
        print(f"[P1] training {variant}", flush=True)
        model = build_g4_model("discovery", legacy_fpn_control=True)
        checkpoint = args.output / f"legacy_{variant}.pt"
        model, training = train_discovery(
            train_rows,
            instances_by_key,
            device=device,
            epochs=args.epochs,
            batch_size=args.batch_size,
            learning_rate=args.learning_rate,
            seed=SEED,
            val_rows=holdout_rows,
            checkpoint_path=checkpoint,
            early_stopping_patience=8,
            load_best=True,
            model=model,
            objectness_variant=variant,
        )
        print(f"[P1] evaluating {variant}", flush=True)
        holdout_raw_frames = discovery_predictions(
            model,
            holdout_rows,
            instances_by_key,
            device=device,
            threshold=min(THRESHOLDS),
            max_detections=100,
            pre_nms_topk=1000,
        )
        threshold, selected_holdout, sweep = _select_holdout_threshold(
            holdout_raw_frames
        )
        val_raw_frames = discovery_predictions(
            model,
            val_rows,
            instances_by_key,
            device=device,
            threshold=min(THRESHOLDS),
            max_detections=100,
            pre_nms_topk=1000,
        )
        val_frames = _filtered_frames(val_raw_frames, threshold)
        val_metrics = discovery_metrics(val_frames)
        val_gates = _gate_verdict(val_metrics)
        audit = _audit_model(
            model, holdout_rows, instances_by_key, device, variant
        )
        variants[variant] = {
            "training": training,
            "checkpoint": {
                "path": checkpoint.name,
                "sha256": _sha256(checkpoint),
                "bytes": checkpoint.stat().st_size,
                "diagnostic_only": True,
            },
            "selected_threshold_from_train_world_holdout": threshold,
            "holdout_selection": selected_holdout,
            "holdout_threshold_sweep": sweep,
            "cross_world_val_metrics": val_metrics,
            "cross_world_val_gates": val_gates,
            "cross_world_val_all_gates_pass": all(val_gates.values()),
            "cross_world_val_constraint_distance": _constraint_distance(
                val_metrics
            ),
            "audit": audit,
        }
        live_models[variant] = (model, val_frames)

    selected_variant = min(
        variants,
        key=lambda name: (
            not variants[name]["cross_world_val_all_gates_pass"],
            variants[name]["cross_world_val_constraint_distance"],
            -variants[name]["cross_world_val_metrics"]["all_gt_candidate_recall"],
            variants[name]["cross_world_val_metrics"]["false_candidates_per_min"],
        ),
    )
    selected = variants[selected_variant]
    architecture_retired = not selected["cross_world_val_all_gates_pass"]
    visual_frames = live_models[selected_variant][1]
    visual_manifest = _write_visual_decode(
        visual_frames, args.output / "visual_decode", count=24
    )
    report = {
        "schema_version": 1,
        "stage": "PERCEPTION-P1",
        "architecture": "legacy_small_fpn_centernet_like_full_frame",
        "architecture_role": "historical_control_diagnostic_only",
        "data_policy": {
            "read_splits": ["train", "val"],
            "derived_split": "train_world_holdout",
            "legacy_G4_D6_diagnostic_read": False,
            "G5_SEALED_FINAL_read": False,
            "train_frames": len(train_rows),
            "train_world_holdout_frames": len(holdout_rows),
            "cross_world_val_frames": len(val_rows),
            "bounded_subset_sampling": "deterministic_world_x_polarity_round_robin",
            "train_world_count": len({row["world_id"] for row in train_rows}),
            "holdout_world_count": len({row["world_id"] for row in holdout_rows}),
            "val_world_count": len({row["world_id"] for row in val_rows}),
            "train_negative_only_frames": sum(
                bool(row.get("negative_only")) for row in train_rows
            ),
            "holdout_negative_only_frames": sum(
                bool(row.get("negative_only")) for row in holdout_rows
            ),
            "val_negative_only_frames": sum(
                bool(row.get("negative_only")) for row in val_rows
            ),
        },
        "control_gates": CONTROL_GATES,
        "decoder_policy": {
            "postprocess_location": "outside_model_graph",
            "pre_nms_topk": 1000,
            "max_detections": 100,
            "bounded_before_nms": True,
        },
        "variants": variants,
        "selected_variant": selected_variant,
        "selected_threshold": selected[
            "selected_threshold_from_train_world_holdout"
        ],
        "selected_cross_world_val_metrics": selected["cross_world_val_metrics"],
        "selected_cross_world_val_gates": selected["cross_world_val_gates"],
        "legacy_architecture_retired": architecture_retired,
        "additional_epoch_tuning_allowed": False,
        "next_action": (
            "proceed_to_P2_teacher_then_FCOS_lite"
            if architecture_retired
            else "retain_control_only_and_compare_against_P2"
        ),
        "visual_decode": {
            "frame_count": len(visual_manifest),
            "legend": "green=ground_truth, red=decoded_detection",
            "files": visual_manifest,
        },
    }
    report_path = args.output / "P1_CONTROL_REPORT.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "report": str(report_path),
                "selected_variant": selected_variant,
                "metrics": selected["cross_world_val_metrics"],
                "gates": selected["cross_world_val_gates"],
                "legacy_architecture_retired": architecture_retired,
            },
            indent=2,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
