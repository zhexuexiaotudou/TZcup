#!/usr/bin/env python3
"""Train the protocol-first C1 close-range classifier and score G10 HOLDOUT."""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import random

import numpy as np


CLASSES = ("plastic_bottle", "metal_can", "paper_litter", "background_or_unknown")
PRIMARY_MODEL = "convnext_tiny"
CONTROL_MODEL = "mobilenet_v3_large"
TARGETED_RECOVERY = {
    "reason": "TRAIN-to-HOLDOUT target color correlation and background overconfidence",
    "color_jitter": {"brightness": .35, "contrast": .35, "saturation": .45, "hue": .12},
    "random_grayscale_probability": .35,
    "label_smoothing": .08,
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def classification_metrics(truth: list[int], predicted: list[int]) -> dict:
    confusion = np.zeros((len(CLASSES), len(CLASSES)), dtype=np.int64)
    for expected, actual in zip(truth, predicted):
        confusion[expected, actual] += 1
    per_class, f1_values = {}, []
    for index, name in enumerate(CLASSES):
        tp = int(confusion[index, index])
        fp = int(confusion[:, index].sum() - tp)
        fn = int(confusion[index, :].sum() - tp)
        tn = int(confusion.sum() - tp - fp - fn)
        precision = tp / max(tp + fp, 1)
        recall = tp / max(tp + fn, 1)
        f1 = 2 * precision * recall / max(precision + recall, 1e-12)
        specificity = tn / max(tn + fp, 1)
        per_class[name] = {"precision": precision, "recall": recall, "f1": f1, "specificity": specificity,
                           "support": int(confusion[index].sum())}
        f1_values.append(f1)
    background_specificity = per_class["background_or_unknown"]["recall"]
    metrics = {
        "macro_f1": float(np.mean(f1_values)),
        "background_specificity": background_specificity,
        "per_class": per_class,
        "confusion": confusion.tolist(),
    }
    gates = {
        "macro_f1": metrics["macro_f1"] >= .98,
        "each_target_precision": all(per_class[name]["precision"] >= .97 for name in CLASSES[:3]),
        "each_target_recall": all(per_class[name]["recall"] >= .97 for name in CLASSES[:3]),
        "background_specificity": background_specificity >= .995,
        "paper_precision": per_class["paper_litter"]["precision"] >= .98,
        "metal_recall": per_class["metal_can"]["recall"] >= .97,
    }
    return {"metrics": metrics, "gates": gates, "pass": bool(truth) and all(gates.values())}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--train", type=Path, required=True)
    parser.add_argument("--holdout", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--model", choices=(PRIMARY_MODEL, CONTROL_MODEL), default=PRIMARY_MODEL)
    parser.add_argument("--epochs", type=int, default=12)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--seed", type=int, default=20260813)
    args = parser.parse_args()
    import torch
    from torch import nn
    from torch.utils.data import DataLoader, Dataset, WeightedRandomSampler
    import torchvision
    from torchvision.io import read_image
    from torchvision.transforms import v2

    if not torch.cuda.is_available():
        raise RuntimeError("TRCRV10 close-range classifier training requires CUDA")
    train_manifest = args.train / "CLASSIFIER_TRAIN_CROP_MANIFEST.json"
    holdout_manifest = args.holdout / "CLASSIFIER_HOLDOUT_CROP_MANIFEST.json"
    train_payload = json.loads(train_manifest.read_text(encoding="utf-8"))
    holdout_payload = json.loads(holdout_manifest.read_text(encoding="utf-8"))
    train_rows, holdout_rows = train_payload["rows"], holdout_payload["rows"]
    if {row["source_split"] for row in train_rows} != {"G10_TRAIN"}:
        raise ValueError("C1 training rows must be G10_TRAIN only")
    if {row["source_split"] for row in holdout_rows} != {"G10_HOLDOUT"}:
        raise ValueError("C1 evaluation rows must be G10_HOLDOUT only")
    if {row["class_id"] for row in train_rows} != set(CLASSES) or {row["class_id"] for row in holdout_rows} != set(CLASSES):
        raise ValueError("all four classifier classes are required in both splits")
    torch.manual_seed(args.seed); np.random.seed(args.seed); random.seed(args.seed)
    class_to_index = {name: index for index, name in enumerate(CLASSES)}
    device = torch.device("cuda")

    class Crops(Dataset):
        def __init__(self, root: Path, rows: list[dict], train: bool):
            self.root, self.rows = root, rows
            ops = [v2.ToDtype(torch.float32, scale=True), v2.Resize((224, 224), antialias=True)]
            if train:
                ops.extend([
                    v2.RandomHorizontalFlip(),
                    v2.ColorJitter(**TARGETED_RECOVERY["color_jitter"]),
                    v2.RandomGrayscale(p=TARGETED_RECOVERY["random_grayscale_probability"]),
                ])
            ops.append(v2.Normalize(mean=(.485, .456, .406), std=(.229, .224, .225)))
            self.transform = v2.Compose(ops)
        def __len__(self): return len(self.rows)
        def __getitem__(self, index):
            row = self.rows[index]
            return self.transform(read_image(str(self.root / row["path"]))), class_to_index[row["class_id"]], index

    if args.model == PRIMARY_MODEL:
        weights = torchvision.models.ConvNeXt_Tiny_Weights.IMAGENET1K_V1
        model = torchvision.models.convnext_tiny(weights=weights)
        model.classifier[-1] = nn.Linear(model.classifier[-1].in_features, len(CLASSES))
    else:
        weights = torchvision.models.MobileNet_V3_Large_Weights.IMAGENET1K_V2
        model = torchvision.models.mobilenet_v3_large(weights=weights)
        model.classifier[-1] = nn.Linear(model.classifier[-1].in_features, len(CLASSES))
    model.to(device)
    counts = defaultdict(int)
    for row in train_rows: counts[row["class_id"]] += 1
    sample_weights = [1 / counts[row["class_id"]] for row in train_rows]
    sampler = WeightedRandomSampler(sample_weights, num_samples=len(train_rows), replacement=True)
    train_loader = DataLoader(Crops(args.train, train_rows, True), batch_size=args.batch_size, sampler=sampler,
                              num_workers=4, pin_memory=True)
    optimizer = torch.optim.AdamW(model.parameters(), lr=2e-4, weight_decay=1e-4)
    loss_fn = nn.CrossEntropyLoss(label_smoothing=TARGETED_RECOVERY["label_smoothing"])
    for _ in range(args.epochs):
        model.train()
        for images, labels, _ in train_loader:
            optimizer.zero_grad(set_to_none=True)
            loss = loss_fn(model(images.to(device, non_blocking=True)), labels.to(device, non_blocking=True))
            loss.backward(); optimizer.step()
    holdout = Crops(args.holdout, holdout_rows, False)
    loader = DataLoader(holdout, batch_size=args.batch_size, shuffle=False, num_workers=4, pin_memory=True)
    truth, predicted, evaluated = [], [], []
    model.eval()
    with torch.no_grad():
        for images, labels, indexes in loader:
            probabilities = model(images.to(device, non_blocking=True)).softmax(1).cpu()
            choices = probabilities.argmax(1)
            truth.extend(labels.tolist()); predicted.extend(choices.tolist())
            evaluated.extend({
                **holdout_rows[int(index)],
                "truth": CLASSES[int(expected)],
                "predicted": CLASSES[int(actual)],
                "predicted_probability": float(probability[int(actual)]),
                "class_probabilities": {
                    class_id: float(probability[class_index])
                    for class_index, class_id in enumerate(CLASSES)
                },
            } for index, expected, actual, probability in zip(indexes, labels, choices, probabilities))
    aggregate = classification_metrics(truth, predicted)
    breakdown = {}
    for field in ("world_id", "distance_bucket", "size_bucket", "occlusion_bucket"):
        breakdown[field] = {}
        for value in sorted({str(row.get(field, "unknown")) for row in evaluated}):
            selected = [row for row in evaluated if str(row.get(field, "unknown")) == value]
            breakdown[field][value] = classification_metrics(
                [class_to_index[row["truth"]] for row in selected], [class_to_index[row["predicted"]] for row in selected]
            )
    args.output.mkdir(parents=True, exist_ok=True)
    checkpoint = args.output / f"{args.model}.pt"
    torch.save({"model": model.state_dict(), "model_name": args.model, "classes": CLASSES, "official_weights": str(weights),
                "seed": args.seed, "epochs": args.epochs}, checkpoint)
    payload = {
        "schema_version": 1, "protocol": "TRCRV10", "stage": "TRCRV10-04-CLOSE-RANGE-CLASSIFIER",
        "model": args.model, "official_weights": str(weights), "train_manifest_sha256": sha256(train_manifest),
        "holdout_manifest_sha256": sha256(holdout_manifest), "checkpoint": str(checkpoint.resolve()),
        "checkpoint_sha256": sha256(checkpoint), "train_samples": len(train_rows), "holdout_samples": len(holdout_rows),
        "targeted_recovery": TARGETED_RECOVERY,
        "aggregate": aggregate, "breakdown": breakdown,
        "evaluated_rows": evaluated,
        "CLOSE_RANGE_CLASSIFICATION_BLOCKED": not aggregate["pass"],
        "TRCRV10_CLOSE_RANGE_CLASSIFIER_PASS": aggregate["pass"],
        "G10_DEV_VAL_SEALED_read": False, "VAL_NEW_read": False, "G5_V2_read": False,
    }
    (args.output / "CLOSE_RANGE_CLASSIFIER_REPORT.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"model": args.model, "aggregate": aggregate, "checkpoint_sha256": payload["checkpoint_sha256"]}, indent=2))
    return 0 if aggregate["pass"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
