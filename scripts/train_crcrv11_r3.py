#!/usr/bin/env python3
"""Train the final authorized CRCRV11 R3 paired tight/context classifier."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import random

import numpy as np

from train_crcrv11_r1 import CLASSES, classification_metrics, rank_metrics


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--five-view", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--epochs", type=int, default=12)
    parser.add_argument("--patience", type=int, default=3)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--seed", type=int, default=20260814)
    args = parser.parse_args()

    import torch
    from torch import nn
    from torch.utils.data import DataLoader, Dataset
    import torchvision
    from torchvision.io import read_image
    from torchvision.transforms import v2

    if not torch.cuda.is_available():
        raise RuntimeError("CRCRV11 R3 requires CUDA")
    five_view = load_json(args.five_view)
    evidence = five_view.get("disagreement", {})
    if evidence.get("R3_COMPLEMENTARY_EVIDENCE") is not True:
        raise RuntimeError("R3 is unauthorized without frozen tight/context complementary evidence")
    train_path, holdout_path = args.data / "C11_TRAIN_PAIR_MANIFEST.json", args.data / "C11_HOLDOUT_PAIR_MANIFEST.json"
    qa_path = args.data / "C11_DATA_QA.json"
    train_payload, holdout_payload, qa = load_json(train_path), load_json(holdout_path), load_json(qa_path)
    if qa.get("C11_DATA_PASS") is not True:
        raise RuntimeError("R3 requires C11_DATA_PASS=true")
    fit_pairs = [row for row in train_payload["pairs"] if row["development_partition"] == "fit"]
    dev_pairs = [row for row in train_payload["pairs"] if row["development_partition"] == "dev"]
    holdout_pairs = holdout_payload["pairs"]
    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    class_to_index = {name: index for index, name in enumerate(CLASSES)}

    train_transform = v2.Compose([
        v2.ToDtype(torch.float32, scale=True), v2.Resize((224, 224), antialias=True),
        v2.RandomHorizontalFlip(), v2.ColorJitter(brightness=.10, contrast=.10, saturation=.08, hue=.02),
        v2.RandomApply([v2.GaussianBlur(kernel_size=3, sigma=(.1, .7))], p=.15),
        v2.Normalize(mean=(.485, .456, .406), std=(.229, .224, .225)),
    ])
    eval_transform = v2.Compose([
        v2.ToDtype(torch.float32, scale=True), v2.Resize((224, 224), antialias=True),
        v2.Normalize(mean=(.485, .456, .406), std=(.229, .224, .225)),
    ])

    class Pairs(Dataset):
        def __init__(self, rows: list[dict], transform):
            self.rows, self.transform = rows, transform
        def __len__(self): return len(self.rows)
        def __getitem__(self, index):
            row = self.rows[index]
            tight = self.transform(read_image(str(args.data / row["tight_path"])))
            context = self.transform(read_image(str(args.data / row["context_path"])))
            return tight, context, class_to_index[row["class"]], index

    class DualViewConvNeXt(nn.Module):
        def __init__(self, weights):
            super().__init__()
            base = torchvision.models.convnext_tiny(weights=weights)
            self.features, self.avgpool = base.features, base.avgpool
            self.norm, self.flatten = base.classifier[0], base.classifier[1]
            embedding = base.classifier[-1].in_features
            self.fusion_head = nn.Sequential(
                nn.Linear(embedding * 2, embedding), nn.GELU(), nn.Dropout(.10),
                nn.Linear(embedding, len(CLASSES)),
            )
        def encode(self, image):
            return self.flatten(self.norm(self.avgpool(self.features(image))))
        def forward(self, tight, context):
            return self.fusion_head(torch.cat((self.encode(tight), self.encode(context)), dim=1))

    def evaluate(model, rows: list[dict]) -> dict:
        loader = DataLoader(Pairs(rows, eval_transform), batch_size=args.batch_size, shuffle=False,
                            num_workers=4, pin_memory=True, persistent_workers=True)
        truth, predicted, evaluated = [], [], []
        model.eval()
        with torch.inference_mode():
            for tight, context, labels, indexes in loader:
                probabilities = model(tight.cuda(non_blocking=True), context.cuda(non_blocking=True)).softmax(1).cpu()
                choices = probabilities.argmax(1)
                truth.extend(labels.tolist()); predicted.extend(choices.tolist())
                for index, expected, actual, probability in zip(indexes.tolist(), labels.tolist(), choices.tolist(), probabilities):
                    evaluated.append({
                        "pair_id": rows[index]["pair_id"], "truth": CLASSES[expected], "predicted": CLASSES[actual],
                        "class_probabilities": {name: float(probability[position]) for position, name in enumerate(CLASSES)},
                    })
        return {"metrics": classification_metrics(truth, predicted), "evaluated_candidates": evaluated}

    weights = torchvision.models.ConvNeXt_Tiny_Weights.IMAGENET1K_V1
    model = DualViewConvNeXt(weights).cuda()
    fit_counts = Counter(row["class"] for row in fit_pairs); maximum = max(fit_counts.values())
    loss_weights = torch.tensor([np.sqrt(maximum / fit_counts[name]) for name in CLASSES], dtype=torch.float32, device="cuda")
    loader = DataLoader(Pairs(fit_pairs, train_transform), batch_size=args.batch_size, shuffle=True,
                        num_workers=4, pin_memory=True, persistent_workers=True)
    optimizer = torch.optim.AdamW(model.parameters(), lr=2e-4, weight_decay=1e-4)
    loss_fn = nn.CrossEntropyLoss(weight=loss_weights)
    scaler = torch.cuda.amp.GradScaler(enabled=True)
    best = None; stale = 0; history = []
    for epoch in range(1, args.epochs + 1):
        model.train(); losses = []
        for tight, context, labels, _ in loader:
            optimizer.zero_grad(set_to_none=True)
            with torch.cuda.amp.autocast(enabled=True):
                loss = loss_fn(model(tight.cuda(non_blocking=True), context.cuda(non_blocking=True)), labels.cuda(non_blocking=True))
            scaler.scale(loss).backward(); scaler.step(optimizer); scaler.update(); losses.append(float(loss.detach().cpu()))
        dev = evaluate(model, dev_pairs)["metrics"]
        history.append({"epoch": epoch, "train_loss": float(np.mean(losses)), "dev": dev})
        if best is None or rank_metrics(dev) > rank_metrics(best["dev"]):
            best = {"epoch": epoch, "dev": dev,
                    "state": {name: tensor.detach().cpu().clone() for name, tensor in model.state_dict().items()}}
            stale = 0
        else:
            stale += 1
        if stale >= args.patience:
            break
    model.load_state_dict(best["state"])
    holdout = evaluate(model, holdout_pairs)
    args.output.mkdir(parents=True, exist_ok=True)
    checkpoint_path = args.output / "r3_dual_view_convnext_tiny.pt"
    torch.save({
        "model": best["state"], "model_name": "dual_view_convnext_tiny", "classes": CLASSES,
        "official_weights": str(weights), "best_epoch": best["epoch"], "dev_metrics": best["dev"],
    }, checkpoint_path)
    payload = {
        "schema_version": 1, "protocol": "CRCRV11", "stage": "CRCRV11-06-R3-PAIRED-DUAL-VIEW",
        "source_commit": args.source_commit, "architecture": "shared ConvNeXt-Tiny tight/context embeddings plus fusion head",
        "five_view_sha256": sha256(args.five_view), "complementary_evidence": evidence,
        "train_manifest_sha256": sha256(train_path), "holdout_manifest_sha256": sha256(holdout_path),
        "c11_qa_sha256": sha256(qa_path), "official_weights": str(weights),
        "fit_pairs": len(fit_pairs), "dev_pairs": len(dev_pairs), "holdout_pairs": len(holdout_pairs),
        "unique_sample_coverage_per_epoch": 1.0, "augmentation": "AUG1 bounded blur and mild color jitter",
        "loss_weights": loss_weights.cpu().tolist(), "history": history, "best_epoch": best["epoch"],
        "dev": best["dev"], "checkpoint": str(checkpoint_path.resolve()),
        "checkpoint_sha256": sha256(checkpoint_path), "holdout": holdout,
        "CRCRV11_R3_PASS": holdout["metrics"]["formal_pass"],
        "G10_DEV_VAL_SEALED_read": False, "VAL_NEW_read": False, "G5_V2_read": False,
    }
    write_json(args.output / "CRCRV11_R3_REPORT.json", payload)
    print(json.dumps({"dev": best["dev"], "holdout": holdout["metrics"],
                      "checkpoint_sha256": payload["checkpoint_sha256"], "CRCRV11_R3_PASS": payload["CRCRV11_R3_PASS"]}, indent=2))
    return 0 if payload["CRCRV11_R3_PASS"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
