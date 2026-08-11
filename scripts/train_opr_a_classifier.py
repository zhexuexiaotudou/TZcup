#!/usr/bin/env python3
"""Adapt the frozen crop classifier on G6 TRAIN only for OPR-A proposals."""

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

from sanitation_learning.g4_data import (  # noqa: E402
    CLASSIFIER_CLASSES,
    DISCRETE_NAMES,
    G4ClassifierDataset,
    square_crop,
)
from sanitation_learning.g4_losses import classifier_loss  # noqa: E402
from sanitation_learning.g4_models import build_g4_model  # noqa: E402
from sanitation_learning.g6_small_specialist import load_g6_rows  # noqa: E402


SEED = 20260811
THRESHOLDS = (
    *tuple(round(value / 100, 2) for value in range(40, 91, 5)),
    0.92,
    0.94,
    0.95,
    0.96,
    0.97,
    0.98,
    0.99,
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def overlap(left, right) -> float:
    x0, y0 = max(left[0], right[0]), max(left[1], right[1])
    x1, y1 = min(left[2], right[2]), min(left[3], right[3])
    intersection = max(0, x1 - x0) * max(0, y1 - y0)
    area = max(1, (left[2] - left[0]) * (left[3] - left[1]))
    return intersection / area


def build_samples(rows, instances, per_class: int, backgrounds: int, seed: int):
    rng = random.Random(seed)
    candidates = {name: [] for name in DISCRETE_NAMES}
    for row in rows:
        key = (int(row["scene_seed"]), int(row["frame_index"]))
        for record in instances.get(key, []):
            if record["class_id"] not in DISCRETE_NAMES:
                continue
            candidates[record["class_id"]].append((row, record))
    samples = []
    for class_name, items in candidates.items():
        rng.shuffle(items)
        for row, record in items[:per_class]:
            crop = square_crop(
                640,
                480,
                tuple(record["bbox_xyxy"]),
                scale=6.0 if int(record["bbox_short_side_px"]) < 18 else 4.0,
                minimum_side=64,
            )
            samples.append(
                {
                    "rgb_path": row["rgb_path"],
                    "crop": crop,
                    "label": CLASSIFIER_CLASSES.index(class_name),
                    "class_name": class_name,
                    "split": row["split"],
                    "scene_seed": int(row["scene_seed"]),
                    "frame_index": int(row["frame_index"]),
                    "hard_negative": False,
                }
            )
    shuffled_rows = list(rows)
    rng.shuffle(shuffled_rows)
    attempts = 0
    while sum(sample["label"] == 0 for sample in samples) < backgrounds and attempts < backgrounds * 20:
        row = shuffled_rows[attempts % len(shuffled_rows)]
        key = (int(row["scene_seed"]), int(row["frame_index"]))
        boxes = [record["bbox_xyxy"] for record in instances.get(key, [])]
        side = rng.choice((64, 80, 96, 128))
        x0 = rng.randint(0, 640 - side)
        y0 = rng.randint(120, 480 - side)
        crop = (x0, y0, x0 + side, y0 + side)
        attempts += 1
        if any(overlap(crop, box) >= 0.05 for box in boxes):
            continue
        samples.append(
            {
                "rgb_path": row["rgb_path"],
                "crop": crop,
                "label": 0,
                "class_name": "background",
                "split": row["split"],
                "scene_seed": int(row["scene_seed"]),
                "frame_index": int(row["frame_index"]),
                "hard_negative": True,
            }
        )
    rng.shuffle(samples)
    return samples


def score(model, samples, device, batch_size):
    loader = DataLoader(G4ClassifierDataset(samples, augment=False), batch_size=batch_size, shuffle=False, num_workers=0)
    probabilities = []
    model.eval()
    with torch.no_grad():
        for images, labels in loader:
            probs = torch.softmax(model(images.to(device)), dim=1).cpu().numpy()
            probabilities.extend((int(label), row.tolist()) for label, row in zip(labels, probs))
    return probabilities


def metrics(scored, threshold):
    matrix = np.zeros((4, 4), dtype=np.int64)
    for truth, probs in scored:
        predicted = int(np.argmax(probs))
        if predicted != 0 and float(probs[predicted]) < threshold:
            predicted = 0
        matrix[truth, predicted] += 1
    recalls = []
    precisions = []
    for index in range(1, 4):
        tp = int(matrix[index, index])
        recalls.append(tp / max(int(matrix[index].sum()), 1))
        precisions.append(tp / max(int(matrix[:, index].sum()), 1))
    macro_recall = float(np.mean(recalls))
    macro_precision = float(np.mean(precisions))
    macro_f1 = 2 * macro_recall * macro_precision / max(macro_recall + macro_precision, 1e-9)
    return {
        "threshold": threshold,
        "confusion_matrix": matrix.tolist(),
        "per_class_recall": dict(zip(DISCRETE_NAMES, recalls)),
        "per_class_precision": dict(zip(DISCRETE_NAMES, precisions)),
        "macro_recall": macro_recall,
        "macro_precision": macro_precision,
        "macro_f1": macro_f1,
        "background_specificity": float(matrix[0, 0] / max(matrix[0].sum(), 1)),
    }


def select(scored):
    sweep = []
    for threshold in THRESHOLDS:
        item = metrics(scored, threshold)
        item["gates"] = {
            "macro_recall_at_least_0_95": item["macro_recall"] >= 0.95,
            "macro_precision_at_least_0_95": item["macro_precision"] >= 0.95,
            "background_specificity_at_least_0_99": item["background_specificity"] >= 0.99,
        }
        item["all_pass"] = all(item["gates"].values())
        item["distance"] = max(0, 0.95 - item["macro_recall"]) + max(0, 0.95 - item["macro_precision"]) + max(0, 0.99 - item["background_specificity"])
        sweep.append(item)
    return min(sweep, key=lambda item: (not item["all_pass"], item["distance"], -item["macro_f1"])), sweep


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--g6-root", type=Path, required=True)
    parser.add_argument("--base-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=5)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--train-per-class", type=int, default=600)
    parser.add_argument("--train-backgrounds", type=int, default=1200)
    parser.add_argument("--holdout-per-class", type=int, default=120)
    parser.add_argument("--holdout-backgrounds", type=int, default=240)
    parser.add_argument("--proposal-samples", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    audit = json.loads((args.g6_root / "G6_INDEPENDENT_AUDIT.json").read_text())
    if not audit["G6_INDEPENDENT_AUDIT_PASS"]:
        raise RuntimeError("OPR-A classifier adaptation requires passed G6 audit")
    rows, instances = load_g6_rows(args.g6_root, ("train",))
    holdout_world = sorted({row["world_id"] for row in rows})[-1]
    fit_rows = [row for row in rows if row["world_id"] != holdout_world]
    holdout_rows = [row for row in rows if row["world_id"] == holdout_world]
    fit = build_samples(fit_rows, instances, args.train_per_class, args.train_backgrounds, SEED)
    holdout = build_samples(holdout_rows, instances, args.holdout_per_class, args.holdout_backgrounds, SEED + 1)
    proposal_policy = None
    if args.proposal_samples is not None:
        proposal = json.loads(args.proposal_samples.read_text())
        if proposal["data_policy"]["VAL_read"] or proposal["data_policy"]["G5_SEALED_FINAL_read"]:
            raise RuntimeError("proposal mining must remain G6 TRAIN only")
        fit.extend(proposal["fit_samples"])
        holdout = proposal["holdout_samples"]
        proposal_policy = {
            "path": args.proposal_samples.as_posix(),
            "fit_counts": proposal["fit_counts"],
            "holdout_counts": proposal["holdout_counts"],
        }
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("formal classifier adaptation requires CUDA")
    torch.manual_seed(SEED)
    model = build_g4_model("classifier", from_scratch_control=True)
    base = torch.load(args.base_checkpoint, map_location="cpu", weights_only=False)
    model.load_state_dict(base["state_dict"], strict=True)
    model.to(device)
    loader = DataLoader(G4ClassifierDataset(fit, augment=True, seed=SEED, cache_crops=True), batch_size=args.batch_size, shuffle=True, num_workers=0, generator=torch.Generator().manual_seed(SEED))
    optimizer = torch.optim.AdamW(model.parameters(), lr=2e-4, weight_decay=1e-4)
    best = None
    best_state = None
    curves = []
    started = time.perf_counter()
    for epoch in range(1, args.epochs + 1):
        model.train()
        losses = []
        for images, labels in loader:
            optimizer.zero_grad(set_to_none=True)
            loss = classifier_loss(
                model(images.to(device)),
                labels.to(device),
                weights=(1.5, 1.0, 1.0, 1.2),
            )
            loss.backward()
            optimizer.step()
            losses.append(float(loss.detach().cpu()))
        selected, sweep = select(score(model, holdout, device, args.batch_size))
        curve = {"epoch": epoch, "loss": float(np.mean(losses)), "selection": selected, "threshold_sweep": sweep}
        curves.append(curve)
        rank = (not selected["all_pass"], selected["distance"], -selected["macro_f1"])
        if best is None or rank < best[0]:
            best = (rank, selected)
            best_state = {name: value.detach().cpu().clone() for name, value in model.state_dict().items()}
        print(f"[OPR-A classifier] epoch={epoch} loss={curve['loss']:.4f} recall={selected['macro_recall']:.4f} precision={selected['macro_precision']:.4f} background={selected['background_specificity']:.4f}", flush=True)
    checkpoint = args.output / "opr_a_classifier.pt"
    torch.save({"state_dict": best_state, "threshold": best[1]["threshold"], "base_checkpoint_sha256": sha256(args.base_checkpoint), "G5_SEALED_FINAL_read": False}, checkpoint)
    report = {
        "schema_version": 1,
        "stage": "OPRV3-05-OPR-A-CLASSIFIER-ADAPT",
        "data_policy": {"dataset": "G6_TRAIN", "fit_samples": len(fit), "holdout_samples": len(holdout), "holdout_world": holdout_world, "proposal_mining": proposal_policy, "VAL_read": False, "G5_SEALED_FINAL_read": False},
        "training": {"duration_s": time.perf_counter() - started, "curves": curves},
        "selected": best[1],
        "gates": best[1]["gates"],
        "OPR_A_CLASSIFIER_ADAPT_PASS": best[1]["all_pass"],
        "checkpoint": {"path": checkpoint.name, "sha256": sha256(checkpoint)},
    }
    (args.output / "OPR_A_CLASSIFIER_ADAPT_REPORT.json").write_text(json.dumps(report, indent=2) + "\n")
    return 0 if best[1]["all_pass"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
