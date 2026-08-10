#!/usr/bin/env python3
"""Benchmark the verified official Grounding DINO checkpoint on MRV2 dev sets."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import statistics
import sys
import time

import numpy as np
import torch
from torchvision.ops import box_convert


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
sys.path.insert(0, str(ROOT / "scripts"))

from perception_mrv2_a_train import holdout_rows  # noqa: E402
from perception_prod_x1_full_pipeline import candidate_size_metrics, load_partition  # noqa: E402
from sanitation_learning.auto04_contract import box_iou  # noqa: E402
from sanitation_learning.g4_data import (  # noqa: E402
    discrete_boxes_for_frame,
    index_instance_records,
    load_frame_rows,
    load_instance_records,
)
from sanitation_learning.g4_evaluation import discovery_metrics  # noqa: E402
from sanitation_learning.g4_split_policy import stratified_row_sample  # noqa: E402


PROMPT = "plastic bottle . metal can . paper litter ."
THRESHOLDS = tuple(round(value / 100.0, 2) for value in range(10, 61, 5))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def infer_rows(model, rows, instances, *, device: str, load_image, predict):
    by_key = index_instance_records(instances)
    frames = []
    preprocess_ms = []
    inference_ms = []
    for index, row in enumerate(rows, start=1):
        started = time.perf_counter()
        image_source, image = load_image(row["rgb_path"])
        preprocess_ms.append((time.perf_counter() - started) * 1000.0)
        if device == "cuda":
            torch.cuda.synchronize()
        started = time.perf_counter()
        boxes, logits, _ = predict(
            model=model,
            image=image,
            caption=PROMPT,
            box_threshold=0.05,
            text_threshold=0.05,
            device=device,
        )
        if device == "cuda":
            torch.cuda.synchronize()
        inference_ms.append((time.perf_counter() - started) * 1000.0)
        height, width = image_source.shape[:2]
        native = box_convert(boxes, in_fmt="cxcywh", out_fmt="xyxy")
        native = native * torch.tensor([width, height, width, height])
        detections = sorted(
            [
                {
                    "class_index": 0,
                    "score": float(score),
                    "bbox_xyxy": [float(value) for value in box],
                }
                for box, score in zip(native, logits)
            ],
            key=lambda item: item["score"],
            reverse=True,
        )[:100]
        truth = discrete_boxes_for_frame(
            row,
            by_key,
            native_size=(width, height),
            model_size=(width, height),
        )
        frames.append(
            {
                "row": row,
                "scene_seed": int(row["scene_seed"]),
                "frame_index": int(row["frame_index"]),
                "split": row["split"],
                "world_id": row["world_id"],
                "negative_only": bool(row.get("negative_only", False)),
                "detections": detections,
                "truth": truth,
            }
        )
        if index % 100 == 0:
            print(f"[Grounding DINO] completed {index}/{len(rows)} frames", flush=True)
    return frames, preprocess_ms, inference_ms


def filter_frames(frames, threshold):
    return [
        {
            **frame,
            "detections": [
                item for item in frame["detections"] if item["score"] >= threshold
            ][:16],
        }
        for frame in frames
    ]


def metal_candidate_recall(frames):
    total = matched = 0
    for frame in frames:
        for truth in frame["truth"]:
            if truth.get("semantic_class") != "metal_can":
                continue
            total += 1
            matched += int(
                any(
                    box_iou(
                        tuple(float(value) for value in truth["bbox_xyxy"]),
                        tuple(float(value) for value in detection["bbox_xyxy"]),
                    ) >= 0.5
                    for detection in frame["detections"]
                )
            )
    return {"total": total, "matched": matched, "recall": matched / max(total, 1)}


def metrics(frames):
    result = discovery_metrics(frames)
    result.update(candidate_size_metrics(frames))
    result["metal_can_candidate_recall"] = metal_candidate_recall(frames)
    return result


def select_threshold(raw_frames):
    sweep = []
    for threshold in THRESHOLDS:
        current = metrics(filter_frames(raw_frames, threshold))
        gates = {
            "candidate_recall_at_least_0_80": current["all_gt_candidate_recall"] >= 0.80,
            "small_recall_at_least_0_70": current["small_object_candidate_recall"] >= 0.70,
            "false_candidates_per_min_at_most_2": current["false_candidates_per_min"] <= 2.0,
            "negative_fp_per_frame_at_most_0_05": current["negative_only_fp_per_frame"] <= 0.05,
        }
        distance = (
            max(0.0, 0.80 - current["all_gt_candidate_recall"])
            + max(0.0, 0.70 - current["small_object_candidate_recall"])
            + max(0.0, current["false_candidates_per_min"] - 2.0)
            + max(0.0, current["negative_only_fp_per_frame"] - 0.05)
        )
        sweep.append(
            {
                "threshold": threshold,
                "metrics": current,
                "gates": gates,
                "all_pass": all(gates.values()),
                "constraint_distance": distance,
            }
        )
    return min(
        sweep,
        key=lambda item: (
            not item["all_pass"], item["constraint_distance"],
            -item["metrics"]["all_gt_candidate_recall"], -item["threshold"],
        ),
    ), sweep


def latency_summary(values):
    return {
        "count": len(values),
        "mean_ms": statistics.fmean(values),
        "p50_ms": float(np.percentile(values, 50)),
        "p95_ms": float(np.percentile(values, 95)),
        "max_ms": max(values),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--factorized-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(args.source_root))
    from groundingdino.util.inference import load_image, load_model, predict

    device = "cuda" if torch.cuda.is_available() else "cpu"
    if device != "cuda":
        raise RuntimeError("formal Grounding DINO benchmark requires CUDA")
    config = args.source_root / "groundingdino/config/GroundingDINO_SwinT_OGC.py"
    model = load_model(str(config), str(args.checkpoint), device=device)

    all_rows = load_frame_rows(
        args.evidence_dir / "g4_frame_manifest.jsonl",
        args.data_root,
        allowed_splits=("train", "val"),
    )
    train_rows = [row for row in all_rows if row["split"] == "train"]
    holdout = stratified_row_sample(
        [{**row, "split": "train_world_holdout"} for row in holdout_rows(train_rows)],
        100,
        seed=20260811,
    )
    all_keys = {(int(row["scene_seed"]), int(row["frame_index"])) for row in all_rows}
    all_instances = load_instance_records(
        args.evidence_dir / "g4_instance_records.jsonl", allowed_frame_keys=all_keys
    )
    holdout_raw, holdout_pre, holdout_infer = infer_rows(
        model, holdout, all_instances, device=device, load_image=load_image, predict=predict
    )
    selected, sweep = select_threshold(holdout_raw)
    threshold = float(selected["threshold"])

    val_rows, val_instances = load_partition(
        args.data_root, args.evidence_dir, allowed_splits={"val"}
    )
    split_inputs = {"VAL": (val_rows, val_instances)}
    for index in range(1, 6):
        root = args.factorized_root / f"D{index}"
        split_inputs[f"D{index}"] = load_partition(
            root / "g4_screening_native", root / "evidence/raw_g4_qa"
        )
    splits = {}
    all_pre = list(holdout_pre)
    all_infer = list(holdout_infer)
    for name, (rows, instances) in split_inputs.items():
        raw, pre, infer = infer_rows(
            model, rows, instances, device=device, load_image=load_image, predict=predict
        )
        fixed = filter_frames(raw, threshold)
        splits[name] = metrics(fixed)
        all_pre.extend(pre)
        all_infer.extend(infer)
        print(
            f"[{name}] recall={splits[name]['all_gt_candidate_recall']:.4f} "
            f"small={splits[name]['small_object_candidate_recall']:.4f} "
            f"fp_min={splits[name]['false_candidates_per_min']:.2f}",
            flush=True,
        )
    val = splits["VAL"]
    gates = {
        "VAL_candidate_recall_at_least_0_80": val["all_gt_candidate_recall"] >= 0.80,
        "VAL_small_recall_at_least_0_70": val["small_object_candidate_recall"] >= 0.70,
        "VAL_false_candidates_per_min_at_most_2": val["false_candidates_per_min"] <= 2.0,
        "D1_D5_each_small_recall_at_least_0_70": all(
            splits[f"D{index}"]["small_object_candidate_recall"] >= 0.70
            for index in range(1, 5)
        ),
        "D1_D5_each_metal_candidate_recall_at_least_0_70": all(
            splits[f"D{index}"]["metal_can_candidate_recall"]["recall"] >= 0.70
            for index in range(1, 5)
        ),
        "inference_p95_at_most_200ms": float(np.percentile(all_infer, 95)) <= 200.0,
    }
    patch_file = (
        args.source_root / "groundingdino/models/GroundingDINO/ms_deform_attn.py"
    )
    report = {
        "schema_version": 1,
        "stage": "MRV2-05-GROUNDING-DINO-BENCHMARK",
        "official_checkpoint": {
            "path": args.checkpoint.as_posix(),
            "bytes": args.checkpoint.stat().st_size,
            "sha256": sha256(args.checkpoint),
        },
        "official_source": {
            "path": args.source_root.as_posix(),
            "upstream_commit": "856dde20aee659246248e20734ef9ba5214f5e44",
            "local_cuda_fallback_patch_file": patch_file.relative_to(args.source_root).as_posix(),
            "local_cuda_fallback_patch_sha256": sha256(patch_file),
            "compiled_custom_ops": False,
        },
        "environment": {
            "platform": platform.platform(),
            "python": platform.python_version(),
            "torch": torch.__version__,
            "gpu": torch.cuda.get_device_name(0),
            "device": device,
        },
        "prompt": PROMPT,
        "selection_policy": "train_world_holdout_only",
        "selection": selected,
        "threshold_sweep": sweep,
        "splits": splits,
        "performance": {
            "preprocess": latency_summary(all_pre),
            "inference": latency_summary(all_infer),
            "implementation": "official_pytorch_deformable_attention_fallback_on_cuda",
        },
        "gates": gates,
        "GROUNDING_DINO_REFERENCE_STATIC_PASS": all(gates.values()),
        "moving_camera_development_benchmark": (
            "not_run_reference_failed_static_gate"
            if not all(gates.values()) else "required_before_product_selection"
        ),
        "closed_set_classifier": "not_run_reference_proposal_gate_first",
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
        "claim_boundary": (
            "Reference candidate-proposal benchmark only; it cannot freeze or overwrite "
            "historical X2 and does not prove closed-set or product readiness."
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0 if all(gates.values()) else 4


if __name__ == "__main__":
    raise SystemExit(main())
