#!/usr/bin/env python3
"""Train and screen the bounded OPR-B two-stage small-object detector."""

from __future__ import annotations

import argparse
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
sys.path.insert(0, str(ROOT / "scripts"))

from sanitation_learning.g4_data import DISCRETE_NAMES  # noqa: E402
from sanitation_learning.g6_opr_b import ANCHOR_SIZES, build_opr_b  # noqa: E402
from sanitation_learning.g6_small_specialist import (  # noqa: E402
    SmallSpecialistDataset,
    build_small_specialist_samples,
    load_g6_rows,
    small_specialist_collate,
)
from train_opr_a_specialist import bounded, iou, move_targets, sha256  # noqa: E402


SEED = 20260812
THRESHOLDS = tuple(round(value / 100, 2) for value in range(10, 96, 5))


def raw_predictions(model, samples, device, batch_size):
    loader = DataLoader(
        SmallSpecialistDataset(samples, class_agnostic=False),
        batch_size=batch_size,
        shuffle=False,
        num_workers=0,
        collate_fn=small_specialist_collate,
    )
    frames = []
    model.eval()
    with torch.no_grad():
        for images, targets, batch_samples in loader:
            outputs = model([image.to(device) for image in images])
            for output, target, sample in zip(outputs, targets, batch_samples):
                frames.append(
                    {
                        "sample": sample,
                        "truth_boxes": target["boxes"].tolist(),
                        "truth_labels": target["labels"].tolist(),
                        "predictions": [
                            {
                                "bbox_xyxy": box,
                                "score": float(score),
                                "label": int(label),
                            }
                            for box, score, label in zip(
                                output["boxes"].cpu().tolist(),
                                output["scores"].cpu().tolist(),
                                output["labels"].cpu().tolist(),
                            )
                        ],
                    }
                )
    return frames


def metrics(raw, threshold):
    truth_count = matched = false_positive = negative_fp = negative_tiles = 0
    per_class = {name: {"truth": 0, "matched": 0} for name in DISCRETE_NAMES}
    for frame in raw:
        predictions = [item for item in frame["predictions"] if item["score"] >= threshold]
        unused = set(range(len(predictions)))
        truth_count += len(frame["truth_boxes"])
        for box, label in zip(frame["truth_boxes"], frame["truth_labels"]):
            class_name = DISCRETE_NAMES[int(label) - 1]
            per_class[class_name]["truth"] += 1
            ranked = sorted(
                (
                    (iou(box, predictions[index]["bbox_xyxy"]), index)
                    for index in unused
                    if predictions[index]["label"] == int(label)
                ),
                reverse=True,
            )
            if ranked and ranked[0][0] >= 0.5:
                unused.remove(ranked[0][1])
                matched += 1
                per_class[class_name]["matched"] += 1
        false_positive += len(unused)
        if not frame["truth_boxes"]:
            negative_tiles += 1
            negative_fp += len(predictions)
    precision = matched / max(matched + false_positive, 1)
    recall = matched / max(truth_count, 1)
    return {
        "threshold": threshold,
        "truth_count": truth_count,
        "matched_correct_class": matched,
        "recall": recall,
        "precision": precision,
        "false_positive_per_tile": false_positive / max(len(raw), 1),
        "negative_false_positive_per_tile": negative_fp / max(negative_tiles, 1),
        "per_class_recall": {
            name: value["matched"] / value["truth"] if value["truth"] else None
            for name, value in per_class.items()
        },
    }


def select(raw):
    sweep = []
    for threshold in THRESHOLDS:
        item = metrics(raw, threshold)
        item["gates"] = {
            "recall_at_least_0_95": item["recall"] >= 0.95,
            "precision_at_least_0_95": item["precision"] >= 0.95,
            "negative_fp_per_tile_at_most_0_05": item["negative_false_positive_per_tile"] <= 0.05,
        }
        item["all_pass"] = all(item["gates"].values())
        item["distance"] = max(0, 0.95 - item["recall"]) + max(0, 0.95 - item["precision"]) + max(0, item["negative_false_positive_per_tile"] - 0.05)
        sweep.append(item)
    return min(sweep, key=lambda item: (not item["all_pass"], item["distance"], -item["recall"], -item["precision"])), sweep


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--g6-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=3)
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--max-train-samples", type=int, default=1600)
    parser.add_argument("--max-holdout-samples", type=int, default=500)
    parser.add_argument("--max-val-samples", type=int, default=800)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    audit = json.loads((args.g6_root / "G6_INDEPENDENT_AUDIT.json").read_text())
    if not audit["G6_INDEPENDENT_AUDIT_PASS"]:
        raise RuntimeError("OPR-B requires passed G6 development audit")
    rows, instances = load_g6_rows(args.g6_root, ("train",))
    holdout_world = sorted({row["world_id"] for row in rows})[-1]
    fit_rows = [row for row in rows if row["world_id"] != holdout_world]
    holdout_rows = [row for row in rows if row["world_id"] == holdout_world]
    fit = bounded(build_small_specialist_samples(fit_rows, instances), args.max_train_samples, SEED)
    holdout = bounded(build_small_specialist_samples(holdout_rows, instances), args.max_holdout_samples, SEED + 1)
    val_rows, val_instances = load_g6_rows(args.g6_root, ("val",))
    val = bounded(build_small_specialist_samples(val_rows, val_instances), args.max_val_samples, SEED + 2)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("formal OPR-B training requires CUDA")
    random.seed(SEED)
    np.random.seed(SEED)
    torch.manual_seed(SEED)
    torch.cuda.manual_seed_all(SEED)
    model = build_opr_b(weights_required=True).to(device)
    loader = DataLoader(
        SmallSpecialistDataset(fit, class_agnostic=False),
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=0,
        collate_fn=small_specialist_collate,
        generator=torch.Generator().manual_seed(SEED),
    )
    optimizer = torch.optim.AdamW(model.parameters(), lr=5e-5, weight_decay=1e-4)
    best = None
    best_state = None
    curves = []
    started = time.perf_counter()
    for epoch in range(1, args.epochs + 1):
        model.train()
        losses = []
        for images, targets, _ in loader:
            optimizer.zero_grad(set_to_none=True)
            loss_dict = model([image.to(device) for image in images], move_targets(targets, device))
            loss = sum(loss_dict.values())
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            optimizer.step()
            losses.append(float(loss.detach().cpu()))
        selected, sweep = select(raw_predictions(model, holdout, device, args.batch_size))
        curve = {"epoch": epoch, "loss": float(np.mean(losses)), "selection": selected, "threshold_sweep": sweep}
        curves.append(curve)
        rank = (not selected["all_pass"], selected["distance"], -selected["recall"], -selected["precision"])
        if best is None or rank < best[0]:
            best = (rank, selected)
            best_state = {name: value.detach().cpu().clone() for name, value in model.state_dict().items()}
        print(f"[OPR-B] epoch={epoch} loss={curve['loss']:.4f} recall={selected['recall']:.4f} precision={selected['precision']:.4f} threshold={selected['threshold']:.2f}", flush=True)
    model.load_state_dict(best_state)
    model.to(device)
    val_result = metrics(raw_predictions(model, val, device, args.batch_size), best[1]["threshold"])
    gates = {
        "VAL_recall_at_least_0_95": val_result["recall"] >= 0.95,
        "VAL_precision_at_least_0_95": val_result["precision"] >= 0.95,
        "VAL_negative_fp_per_tile_at_most_0_05": val_result["negative_false_positive_per_tile"] <= 0.05,
        "each_class_recall_at_least_0_90": all(value is not None and value >= 0.90 for value in val_result["per_class_recall"].values()),
    }
    checkpoint = args.output / "opr_b_fasterrcnn.pt"
    torch.save({"state_dict": best_state, "threshold": best[1]["threshold"], "model_id": model.model_id, "anchor_sizes": ANCHOR_SIZES, "G5_SEALED_FINAL_read": False}, checkpoint)
    report = {
        "schema_version": 1,
        "stage": "OPRV3-05-OPR-B-TWO-STAGE",
        "route": "OPR-B",
        "architecture": model.model_id,
        "provenance": model.opr_b_provenance,
        "data_policy": {"dataset": "G6_DEVELOPMENT_OPRV3_V1", "fit_samples": len(fit), "holdout_samples": len(holdout), "holdout_world": holdout_world, "val_samples": len(val), "VAL_used_for_selection": False, "G5_SEALED_FINAL_read": False},
        "training": {"duration_s": time.perf_counter() - started, "curves": curves},
        "selected_holdout_operating_point": best[1],
        "VAL": val_result,
        "gates": gates,
        "OPR_B_PASS": all(gates.values()),
        "checkpoint": {"path": checkpoint.name, "sha256": sha256(checkpoint)},
        "next_action": "integrate_general_and_run_online_dev_gate" if all(gates.values()) else "execute_OPR_C_official_small_detector",
    }
    (args.output / "OPR_B_REPORT.json").write_text(json.dumps(report, indent=2) + "\n")
    return 0 if all(gates.values()) else 4


if __name__ == "__main__":
    raise SystemExit(main())
