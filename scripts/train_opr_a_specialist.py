#!/usr/bin/env python3
"""Train and screen the OPR-A class-agnostic small-object specialist."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import random
import sys
import time

import numpy as np
import torch
from torch.utils.data import DataLoader


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g6_small_specialist import (  # noqa: E402
    SmallSpecialistDataset,
    build_small_specialist,
    build_small_specialist_samples,
    load_g6_rows,
    small_specialist_collate,
)


SEED = 20260811
THRESHOLDS = tuple(round(value / 100, 2) for value in range(5, 96, 5))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def iou(left, right) -> float:
    x0 = max(float(left[0]), float(right[0]))
    y0 = max(float(left[1]), float(right[1]))
    x1 = min(float(left[2]), float(right[2]))
    y1 = min(float(left[3]), float(right[3]))
    intersection = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    left_area = max(0.0, float(left[2]) - float(left[0])) * max(
        0.0, float(left[3]) - float(left[1])
    )
    right_area = max(0.0, float(right[2]) - float(right[0])) * max(
        0.0, float(right[3]) - float(right[1])
    )
    union = left_area + right_area - intersection
    return intersection / union if union else 0.0


def move_targets(targets, device):
    return [{key: value.to(device) for key, value in target.items()} for target in targets]


def bounded(samples: list[dict], maximum: int, seed: int) -> list[dict]:
    if maximum <= 0 or len(samples) <= maximum:
        return list(samples)
    rng = random.Random(seed)
    positives = [sample for sample in samples if sample["targets"]]
    negatives = [sample for sample in samples if not sample["targets"]]
    rng.shuffle(positives)
    rng.shuffle(negatives)
    negative_count = min(len(negatives), max(1, maximum // 5))
    selected = positives[: maximum - negative_count] + negatives[:negative_count]
    rng.shuffle(selected)
    return selected


def raw_predictions(model, samples, device, batch_size: int) -> list[dict]:
    loader = DataLoader(
        SmallSpecialistDataset(samples),
        batch_size=batch_size,
        shuffle=False,
        num_workers=0,
        collate_fn=small_specialist_collate,
    )
    output = []
    model.eval()
    with torch.no_grad():
        for images, targets, batch_samples in loader:
            predictions = model([image.to(device) for image in images])
            for prediction, target, sample in zip(predictions, targets, batch_samples):
                items = [
                    {
                        "bbox_xyxy": [float(value) for value in box],
                        "score": float(score),
                    }
                    for box, score in zip(
                        prediction["boxes"].cpu().tolist(),
                        prediction["scores"].cpu().tolist(),
                    )
                ]
                output.append(
                    {
                        "sample": sample,
                        "truth": target["boxes"].tolist(),
                        "predictions": items,
                    }
                )
    return output


def metrics(raw: list[dict], threshold: float) -> dict:
    truth_count = 0
    matched_count = 0
    false_positive_count = 0
    negative_fp = 0
    negative_tiles = 0
    per_class = {name: {"truth": 0, "matched": 0} for name in ("plastic_bottle", "metal_can", "paper_litter")}
    per_bucket = {name: {"truth": 0, "matched": 0} for name in ("lt8", "8_12", "12_18")}
    for frame in raw:
        predictions = [item for item in frame["predictions"] if item["score"] >= threshold]
        truth = frame["truth"]
        records = frame["sample"]["targets"]
        truth_count += len(truth)
        unmatched_predictions = set(range(len(predictions)))
        for truth_index, truth_box in enumerate(truth):
            record = records[truth_index]
            class_id = record["class_id"]
            bucket = record["short_side_bucket"]
            per_class[class_id]["truth"] += 1
            per_bucket[bucket]["truth"] += 1
            candidates = sorted(
                (
                    (iou(truth_box, predictions[index]["bbox_xyxy"]), index)
                    for index in unmatched_predictions
                ),
                reverse=True,
            )
            if candidates and candidates[0][0] >= 0.5:
                unmatched_predictions.remove(candidates[0][1])
                matched_count += 1
                per_class[class_id]["matched"] += 1
                per_bucket[bucket]["matched"] += 1
        false_positive_count += len(unmatched_predictions)
        if not truth:
            negative_tiles += 1
            negative_fp += len(predictions)
    recall = matched_count / truth_count if truth_count else 0.0
    precision = matched_count / (matched_count + false_positive_count) if matched_count + false_positive_count else 1.0
    return {
        "threshold": threshold,
        "truth_count": truth_count,
        "matched_count": matched_count,
        "recall": recall,
        "precision": precision,
        "false_positive_per_tile": false_positive_count / max(len(raw), 1),
        "negative_false_positive_per_tile": negative_fp / max(negative_tiles, 1),
        "per_class_recall": {
            name: value["matched"] / value["truth"] if value["truth"] else None
            for name, value in per_class.items()
        },
        "per_bucket_recall": {
            name: value["matched"] / value["truth"] if value["truth"] else None
            for name, value in per_bucket.items()
        },
    }


def select_threshold(raw: list[dict]) -> tuple[dict, list[dict]]:
    sweep = []
    for threshold in THRESHOLDS:
        result = metrics(raw, threshold)
        result["gates"] = {
            "recall_at_least_0_95": result["recall"] >= 0.95,
            "candidate_fp_per_tile_at_most_2": result["false_positive_per_tile"] <= 2.0,
            "negative_candidate_fp_per_tile_at_most_2": result["negative_false_positive_per_tile"] <= 2.0,
        }
        result["all_pass"] = all(result["gates"].values())
        result["constraint_distance"] = (
            max(0.0, 0.95 - result["recall"])
            + max(0.0, result["false_positive_per_tile"] - 2.0)
            + max(0.0, result["negative_false_positive_per_tile"] - 2.0)
        )
        sweep.append(result)
    selected = min(
        sweep,
        key=lambda item: (
            not item["all_pass"],
            item["constraint_distance"],
            -item["recall"],
            -item["precision"],
            -item["threshold"],
        ),
    )
    return selected, sweep


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--g6-root", type=Path, required=True)
    parser.add_argument("--base-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=3)
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--max-train-samples", type=int, default=4000)
    parser.add_argument("--max-holdout-samples", type=int, default=600)
    parser.add_argument("--max-val-samples", type=int, default=1000)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    audit = json.loads((args.g6_root / "G6_INDEPENDENT_AUDIT.json").read_text())
    if not audit["G6_INDEPENDENT_AUDIT_PASS"] or audit["sealed_final_read"]:
        raise RuntimeError("formal OPR-A training requires passed development-only G6 audit")
    train_rows, train_instances = load_g6_rows(args.g6_root, ("train",))
    train_worlds = sorted({row["world_id"] for row in train_rows})
    holdout_world = train_worlds[-1]
    fit_rows = [row for row in train_rows if row["world_id"] != holdout_world]
    holdout_rows = [row for row in train_rows if row["world_id"] == holdout_world]
    fit = bounded(build_small_specialist_samples(fit_rows, train_instances), args.max_train_samples, SEED)
    holdout = bounded(build_small_specialist_samples(holdout_rows, train_instances), args.max_holdout_samples, SEED + 1)
    val_rows, val_instances = load_g6_rows(args.g6_root, ("val",))
    val = bounded(build_small_specialist_samples(val_rows, val_instances), args.max_val_samples, SEED + 2)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("formal OPR-A training requires CUDA")
    random.seed(SEED)
    np.random.seed(SEED)
    torch.manual_seed(SEED)
    torch.cuda.manual_seed_all(SEED)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    model = build_small_specialist(args.base_checkpoint).to(device)
    loader = DataLoader(
        SmallSpecialistDataset(fit),
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=0,
        collate_fn=small_specialist_collate,
        generator=torch.Generator().manual_seed(SEED),
    )
    optimizer = torch.optim.AdamW(model.parameters(), lr=5e-5, weight_decay=1e-4)
    scaler = torch.amp.GradScaler("cuda", enabled=True)
    curves = []
    best = None
    best_state = None
    started = time.perf_counter()
    for epoch in range(1, args.epochs + 1):
        model.train()
        total_loss = 0.0
        steps = 0
        for images, targets, _samples in loader:
            optimizer.zero_grad(set_to_none=True)
            with torch.autocast("cuda", dtype=torch.float16):
                losses = model([image.to(device) for image in images], move_targets(targets, device))
                loss = sum(losses.values())
            scaler.scale(loss).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            scaler.step(optimizer)
            scaler.update()
            total_loss += float(loss.detach().cpu())
            steps += 1
        raw = raw_predictions(model, holdout, device, args.batch_size)
        selected, sweep = select_threshold(raw)
        curve = {"epoch": epoch, "loss": total_loss / max(steps, 1), "selection": selected, "threshold_sweep": sweep}
        curves.append(curve)
        rank = (
            not selected["all_pass"],
            selected["constraint_distance"],
            -selected["recall"],
            selected["false_positive_per_tile"],
            selected["negative_false_positive_per_tile"],
            -selected["precision"],
        )
        if best is None or rank < best[0]:
            best = (rank, selected)
            best_state = {name: value.detach().cpu().clone() for name, value in model.state_dict().items()}
        print(f"[OPR-A] epoch={epoch} loss={curve['loss']:.4f} recall={selected['recall']:.4f} precision={selected['precision']:.4f} threshold={selected['threshold']:.2f}", flush=True)
    if best_state is None or best is None:
        raise RuntimeError("OPR-A training produced no selected state")
    model.load_state_dict(best_state)
    model.to(device)
    selected = best[1]
    val_result = metrics(raw_predictions(model, val, device, args.batch_size), selected["threshold"])
    gates = {
        "VAL_small_objectness_recall_at_least_0_95": val_result["recall"] >= 0.95,
        "VAL_candidate_fp_per_tile_at_most_2": val_result["false_positive_per_tile"] <= 2.0,
        "VAL_negative_candidate_fp_per_tile_at_most_2": val_result["negative_false_positive_per_tile"] <= 2.0,
        "each_class_recall_at_least_0_90": all(value is not None and value >= 0.90 for value in val_result["per_class_recall"].values()),
    }
    checkpoint = args.output / "opr_a_small_specialist.pt"
    torch.save(
        {
            "state_dict": best_state,
            "model_id": "opr_a_p2_fcos_small_objectness_v1",
            "input_size": [640, 480],
            "tile_contract": "ground_roi_6x_overlap_native_320x240_to_640x480",
            "objectness_threshold": selected["threshold"],
            "base_checkpoint_sha256": sha256(args.base_checkpoint),
            "G5_SEALED_FINAL_read": False,
        },
        checkpoint,
    )
    report = {
        "schema_version": 1,
        "stage": "OPRV3-05-OPR-A-SMALL-SPECIALIST",
        "route": "OPR-A",
        "architecture": "P2_FCOS_R50_CLASS_AGNOSTIC_GROUND_ROI_TILES",
        "data_policy": {
            "dataset": "G6_DEVELOPMENT_OPRV3_V1",
            "fit_samples": len(fit),
            "holdout_samples": len(holdout),
            "holdout_world": holdout_world,
            "val_samples": len(val),
            "VAL_used_for_selection": False,
            "G5_SEALED_FINAL_read": False,
        },
        "training": {"duration_s": time.perf_counter() - started, "curves": curves},
        "selected_holdout_operating_point": selected,
        "VAL": val_result,
        "gates": gates,
        "OPR_A_SPECIALIST_PASS": all(gates.values()),
        "checkpoint": {"path": checkpoint.name, "sha256": sha256(checkpoint)},
        "candidate_precision_is_diagnostic_until_closed_set_classifier": True,
        "next_action": "integrate_classifier_and_general_detector" if all(gates.values()) else "run_OPR_B_two_stage_fallback",
    }
    (args.output / "OPR_A_SPECIALIST_REPORT.json").write_text(json.dumps(report, indent=2) + "\n")
    return 0 if all(gates.values()) else 4


if __name__ == "__main__":
    raise SystemExit(main())
