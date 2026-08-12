#!/usr/bin/env python3
"""Train Route C's one authorized hard-negative enhanced verifier."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import random
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
from sanitation_learning.g4_models import (  # noqa: E402
    CandidateCropClassifier,
    CLASSIFIER_CLASSES,
)


SEED = 20260813
THRESHOLDS = tuple(round(value / 100, 2) for value in range(20, 100, 2))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_images(rows: list[dict], root: Path) -> np.ndarray:
    images = []
    for row in rows:
        image = cv2.imread(str(root / row["crop_path"]), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"unable to read {row['crop_path']}")
        images.append(
            cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), (192, 192), interpolation=cv2.INTER_AREA)
        )
    return np.stack(images)


def infer(model, rows, images, device, batch_size):
    import torch
    result = []
    model.eval()
    with torch.inference_mode():
        for offset in range(0, len(rows), batch_size):
            batch = torch.from_numpy(
                images[offset : offset + batch_size].transpose(0, 3, 1, 2).astype(np.float32) / 255.0
            ).to(device)
            probabilities = torch.softmax(model(batch), dim=1).cpu().numpy()
            result.extend(
                {**row, "probabilities": probability.tolist()}
                for row, probability in zip(rows[offset : offset + batch_size], probabilities)
            )
    return result


def evaluate(rows: list[dict], threshold: float) -> dict:
    confusion = [[0] * 4 for _ in range(4)]
    for row in rows:
        probabilities = row["probabilities"]
        target = max(range(1, 4), key=lambda index: probabilities[index])
        predicted = target if probabilities[target] >= threshold and probabilities[target] > probabilities[0] else 0
        confusion[int(row["class_id"])][predicted] += 1
    per_class = {}
    f1_values = []
    for index, name in enumerate(CLASSIFIER_CLASSES):
        tp = confusion[index][index]
        truth = sum(confusion[index])
        predicted = sum(row[index] for row in confusion)
        recall = tp / max(truth, 1)
        precision = tp / max(predicted, 1)
        f1 = 2 * recall * precision / max(recall + precision, 1e-12)
        per_class[name] = {"truth": truth, "recall": recall, "precision": precision, "f1": f1}
        f1_values.append(f1)
    result = {
        "threshold": threshold,
        "confusion_matrix": confusion,
        "macro_f1": sum(f1_values) / 4,
        "per_class": per_class,
        "background_specificity": per_class["background"]["recall"],
        "metal_recall": per_class["metal_can"]["recall"],
        "paper_precision": per_class["paper_litter"]["precision"],
    }
    result["gate_pass"] = all(
        (
            result["macro_f1"] >= 0.97,
            all(per_class[name]["recall"] >= 0.95 for name in CLASSIFIER_CLASSES[1:]),
            result["background_specificity"] >= 0.98,
            result["metal_recall"] >= 0.95,
            result["paper_precision"] >= 0.95,
        )
    )
    result["constraint_distance"] = (
        max(0.0, 0.97 - result["macro_f1"])
        + sum(max(0.0, 0.95 - per_class[name]["recall"]) for name in CLASSIFIER_CLASSES[1:])
        + max(0.0, 0.98 - result["background_specificity"])
        + max(0.0, 0.95 - result["paper_precision"])
    )
    return result


def main() -> int:
    import torch
    parser = argparse.ArgumentParser()
    parser.add_argument("--crops-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--per-class", type=int, default=3577)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    crop_report = json.loads((args.crops_root / "ROUTE_C_CONTEXT_CROP_REPORT.json").read_text())
    if not crop_report["ROUTE_C_CONTEXT_CROPS_PASS"] or crop_report["VAL_NEW_read"]:
        raise RuntimeError("Route C crop provenance failed")
    train = json.loads((args.crops_root / "train_crops.json").read_text())
    holdout = json.loads((args.crops_root / "holdout_new_crops.json").read_text())
    pools = {index: [row for row in train if int(row["class_id"]) == index] for index in range(4)}
    sample_count = min(args.per_class, *(len(rows) for rows in pools.values()))
    if sample_count < 3000:
        raise RuntimeError(f"insufficient unique samples per class: {sample_count}")
    rng = random.Random(SEED)
    balanced = []
    for rows in pools.values():
        rng.shuffle(rows)
        balanced.extend(rows[:sample_count])
    rng.shuffle(balanced)
    train_images = load_images(balanced, args.crops_root)
    holdout_images = load_images(holdout, args.crops_root)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("formal Route C verifier training requires CUDA")
    torch.manual_seed(SEED)
    torch.cuda.manual_seed_all(SEED)
    model = CandidateCropClassifier().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=2e-4, weight_decay=2e-4)
    loss_fn = torch.nn.CrossEntropyLoss(label_smoothing=0.02)
    args.output.mkdir(parents=True)
    best = None
    history = []
    started = time.perf_counter()
    for epoch in range(1, args.epochs + 1):
        model.train()
        order = list(range(len(balanced)))
        rng.shuffle(order)
        total = 0.0
        for offset in range(0, len(order), args.batch_size):
            indexes = order[offset : offset + args.batch_size]
            rows = [balanced[index] for index in indexes]
            batch = train_images[indexes]
            # Deterministic train-only photometric jitter preserves geometry.
            if epoch % 2 == 0:
                batch = np.clip(batch.astype(np.float32) * 0.92 + 6.0, 0, 255).astype(np.uint8)
            x = torch.from_numpy(batch.transpose(0, 3, 1, 2).astype(np.float32) / 255.0).to(device)
            y = torch.tensor([int(row["class_id"]) for row in rows], device=device)
            optimizer.zero_grad(set_to_none=True)
            loss = loss_fn(model(x), y)
            loss.backward()
            optimizer.step()
            total += float(loss.detach()) * len(rows)
        scored = infer(model, holdout, holdout_images, device, args.batch_size)
        sweep = [evaluate(scored, threshold) for threshold in THRESHOLDS]
        selected = min(
            sweep,
            key=lambda row: (
                not row["gate_pass"],
                row["constraint_distance"],
                -row["macro_f1"],
                -row["background_specificity"],
                -row["threshold"],
            ),
        )
        record = {"epoch": epoch, "loss": total / len(balanced), "selected": selected}
        history.append(record)
        rank = (not selected["gate_pass"], selected["constraint_distance"], -selected["macro_f1"])
        if best is None or rank < best[0]:
            best = (rank, record)
            torch.save(
                {"state_dict": model.state_dict(), "epoch": epoch, "threshold": selected["threshold"]},
                args.output / "verifier.pt",
            )
        print(
            f"[Route C verifier] epoch={epoch} loss={record['loss']:.4f} "
            f"f1={selected['macro_f1']:.4f} bg={selected['background_specificity']:.4f} "
            f"threshold={selected['threshold']:.2f}",
            flush=True,
        )
    checkpoint = torch.load(args.output / "verifier.pt", map_location=device, weights_only=True)
    model.load_state_dict(checkpoint["state_dict"])
    final_scores = infer(model, holdout, holdout_images, device, args.batch_size)
    selected = evaluate(final_scores, float(checkpoint["threshold"]))
    (args.output / "holdout_scores.json").write_text(json.dumps(final_scores, indent=2) + "\n")
    report = {
        "schema_version": 1,
        "stage": "RGDRV8-04-ROUTE-C-VERIFIER",
        "architecture": "torchvision_mobilenet_v3_small_imagenet1k_v1",
        "hard_negative_enhancement": "fixed_proposals_square_context_scale_6",
        "classes": CLASSIFIER_CLASSES,
        "unique_train_counts": dict(Counter(row["class_name"] for row in train)),
        "balanced_unique_train_per_class": sample_count,
        "epochs": args.epochs,
        "duration_s": time.perf_counter() - started,
        "history": history,
        "selected_epoch": int(checkpoint["epoch"]),
        "selected_threshold": float(checkpoint["threshold"]),
        "holdout_metrics": selected,
        "checkpoint_sha256": sha256(args.output / "verifier.pt"),
        "HOLDOUT_proposals_fixed_once": True,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
        "ROUTE_C_VERIFIER_HOLDOUT_PASS": selected["gate_pass"],
    }
    (args.output / "ROUTE_C_VERIFIER_REPORT.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: report[key] for key in ("selected_epoch", "selected_threshold", "holdout_metrics", "ROUTE_C_VERIFIER_HOLDOUT_PASS")}, indent=2))
    return 0 if report["ROUTE_C_VERIFIER_HOLDOUT_PASS"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
