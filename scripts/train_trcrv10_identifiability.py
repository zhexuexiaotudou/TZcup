#!/usr/bin/env python3
"""Train the two protocol-bounded torchvision identifiability classifiers."""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import random

import numpy as np


MODELS = ("convnext_tiny", "resnet18")
CLASSES = ("metal_can", "paper_litter", "plastic_bottle")
VIEWS = ("tight", "context")


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def metrics(truth: list[int], predicted: list[int]) -> dict:
    confusion = np.zeros((len(CLASSES), len(CLASSES)), dtype=np.int64)
    for expected, actual in zip(truth, predicted):
        confusion[expected, actual] += 1
    per_class = {}
    f1_values = []
    for index, class_id in enumerate(CLASSES):
        tp = int(confusion[index, index])
        fp = int(confusion[:, index].sum() - tp)
        fn = int(confusion[index, :].sum() - tp)
        precision = tp / max(tp + fp, 1)
        recall = tp / max(tp + fn, 1)
        f1 = 2 * precision * recall / max(precision + recall, 1e-12)
        f1_values.append(f1)
        per_class[class_id] = {"precision": precision, "recall": recall, "f1": f1, "support": int(confusion[index].sum())}
    return {"macro_f1": float(np.mean(f1_values)), "per_class": per_class, "confusion": confusion.tolist()}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=10)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--seed", type=int, default=20260813)
    args = parser.parse_args()

    import torch
    from torch import nn
    from torch.utils.data import DataLoader, Dataset
    import torchvision
    from torchvision.io import read_image
    from torchvision.transforms import v2

    if not torch.cuda.is_available():
        raise RuntimeError("TRCRV10 identifiability training requires CUDA")
    manifest_path = args.dataset / "IDENTIFIABILITY_CROP_MANIFEST.json"
    manifest_sha256 = sha256(manifest_path)
    rows = read(manifest_path)["rows"]
    if not rows or {row["split"] for row in rows} != {"TRAIN_DIAG", "HOLDOUT_DIAG"}:
        raise ValueError("both isolated diagnostic splits are required")
    if any(row.get("production_runtime_eligible") is not False for row in rows):
        raise ValueError("diagnostic GT crop leaked into a runtime-eligible row")
    torch.manual_seed(args.seed); np.random.seed(args.seed); random.seed(args.seed)
    device = torch.device("cuda")
    class_to_index = {name: index for index, name in enumerate(CLASSES)}

    class Crops(Dataset):
        def __init__(self, selected: list[dict], train: bool):
            self.rows = selected
            ops = [v2.ToDtype(torch.float32, scale=True), v2.Resize((224, 224), antialias=True)]
            if train:
                ops.extend([v2.RandomHorizontalFlip(), v2.ColorJitter(brightness=.12, contrast=.12, saturation=.08)])
            ops.append(v2.Normalize(mean=(.485, .456, .406), std=(.229, .224, .225)))
            self.transform = v2.Compose(ops)
        def __len__(self): return len(self.rows)
        def __getitem__(self, index):
            row = self.rows[index]
            image = read_image(str(args.dataset / row["path"]))
            return self.transform(image), class_to_index[row["class_id"]], index

    results = []
    args.output.mkdir(parents=True, exist_ok=True)
    for model_name in MODELS:
        for view in VIEWS:
            train_rows = [row for row in rows if row["split"] == "TRAIN_DIAG" and row["view"] == view]
            holdout_rows = [row for row in rows if row["split"] == "HOLDOUT_DIAG" and row["view"] == view]
            if model_name == "convnext_tiny":
                weights = torchvision.models.ConvNeXt_Tiny_Weights.IMAGENET1K_V1
                model = torchvision.models.convnext_tiny(weights=weights)
                model.classifier[-1] = nn.Linear(model.classifier[-1].in_features, len(CLASSES))
            else:
                weights = torchvision.models.ResNet18_Weights.IMAGENET1K_V1
                model = torchvision.models.resnet18(weights=weights)
                model.fc = nn.Linear(model.fc.in_features, len(CLASSES))
            model.to(device)
            optimizer = torch.optim.AdamW(model.parameters(), lr=2e-4, weight_decay=1e-4)
            loss_fn = nn.CrossEntropyLoss()
            train_loader = DataLoader(Crops(train_rows, True), batch_size=args.batch_size, shuffle=True, num_workers=4, pin_memory=True)
            for _ in range(args.epochs):
                model.train()
                for images, labels, _ in train_loader:
                    optimizer.zero_grad(set_to_none=True)
                    loss = loss_fn(model(images.to(device, non_blocking=True)), labels.to(device, non_blocking=True))
                    loss.backward(); optimizer.step()
            model.eval(); truth, predicted, evaluated = [], [], []
            holdout = Crops(holdout_rows, False)
            loader = DataLoader(holdout, batch_size=args.batch_size, shuffle=False, num_workers=4, pin_memory=True)
            with torch.no_grad():
                for images, labels, indexes in loader:
                    logits = model(images.to(device, non_blocking=True)).cpu()
                    choices = logits.argmax(1)
                    truth.extend(labels.tolist()); predicted.extend(choices.tolist())
                    evaluated.extend({**holdout_rows[int(i)], "truth": CLASSES[int(y)], "predicted": CLASSES[int(p)]}
                                     for i, y, p in zip(indexes, labels, choices))
            checkpoint = args.output / f"{model_name}_{view}.pt"
            torch.save({"model": model.state_dict(), "classes": CLASSES, "model_name": model_name, "view": view,
                        "official_weights": str(weights), "epochs": args.epochs, "seed": args.seed}, checkpoint)
            aggregate = metrics(truth, predicted)
            by_size, by_domain = {}, {}
            for size in sorted({row["size_bucket"] for row in evaluated}):
                selected = [row for row in evaluated if row["size_bucket"] == size]
                by_size[size] = metrics([class_to_index[row["truth"]] for row in selected], [class_to_index[row["predicted"]] for row in selected])
            for world in sorted({row["world_id"] for row in evaluated}):
                selected = [row for row in evaluated if row["world_id"] == world]
                by_domain[world] = metrics([class_to_index[row["truth"]] for row in selected], [class_to_index[row["predicted"]] for row in selected])
            results.append({"model": model_name, "view": view, "official_weights": str(weights),
                            "train_samples": len(train_rows), "holdout_samples": len(holdout_rows),
                            "checkpoint": str(checkpoint.resolve()), "checkpoint_sha256": sha256(checkpoint),
                            "aggregate": aggregate, "by_size": by_size, "by_domain": by_domain})
    write(args.output / "IDENTIFIABILITY_RAW_RESULTS.json", {
        "schema_version": 1,
        "protocol": "TRCRV10",
        "stage": "TRCRV10-01-IDENTIFIABILITY-RAW-RESULTS",
        "dataset_manifest_sha256": manifest_sha256,
        "models": list(MODELS),
        "views": list(VIEWS),
        "seed": args.seed,
        "epochs": args.epochs,
        "results": results,
        "production_runtime_eligible": False,
        "G10_DEV_VAL_SEALED_read": False,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
    })
    print(json.dumps({"runs": len(results), "output": str(args.output.resolve())}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
