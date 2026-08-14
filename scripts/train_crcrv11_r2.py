#!/usr/bin/env python3
"""Train CRCRV11 R2: binary background rejector plus three-class classifier."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import random

import numpy as np

from train_crcrv11_r1 import CLASSES, TARGETS, classification_metrics


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def binary_metrics(truth_litter: list[bool], litter_probability: list[float], threshold: float) -> dict:
    predicted = [probability >= threshold for probability in litter_probability]
    litter_total = sum(truth_litter)
    background_total = len(truth_litter) - litter_total
    true_litter = sum(expected and actual for expected, actual in zip(truth_litter, predicted))
    true_background = sum(not expected and not actual for expected, actual in zip(truth_litter, predicted))
    false_reject = sum(expected and not actual for expected, actual in zip(truth_litter, predicted))
    false_accept = sum(not expected and actual for expected, actual in zip(truth_litter, predicted))
    result = {
        "threshold": threshold, "litter_recall": true_litter / max(litter_total, 1),
        "background_specificity": true_background / max(background_total, 1),
        "false_reject": false_reject, "false_accept": false_accept,
        "litter_support": litter_total, "background_support": background_total,
    }
    result["gates"] = {
        "litter_recall": result["litter_recall"] >= .98,
        "background_specificity": result["background_specificity"] >= .995,
    }
    result["pass"] = all(result["gates"].values())
    return result


def select_binary_threshold(truth_litter: list[bool], probabilities: list[float]) -> dict:
    candidates = [binary_metrics(truth_litter, probabilities, round(value / 100, 2)) for value in range(5, 96)]
    return max(candidates, key=lambda row: (
        row["pass"], min(row["litter_recall"] / .98, row["background_specificity"] / .995),
        row["background_specificity"], row["litter_recall"], row["threshold"],
    ))


def target_metrics(truth: list[int], predicted: list[int]) -> dict:
    confusion = np.zeros((3, 3), dtype=np.int64)
    for expected, actual in zip(truth, predicted):
        confusion[expected, actual] += 1
    per_class, f1s = {}, []
    for index, name in enumerate(TARGETS):
        tp = int(confusion[index, index])
        fp = int(confusion[:, index].sum() - tp)
        fn = int(confusion[index, :].sum() - tp)
        precision = tp / max(tp + fp, 1)
        recall = tp / max(tp + fn, 1)
        f1 = 2 * precision * recall / max(precision + recall, 1e-12)
        per_class[name] = {"precision": precision, "recall": recall, "f1": f1,
                           "support": int(confusion[index].sum())}
        f1s.append(f1)
    result = {"macro_f1": float(np.mean(f1s)), "per_class": per_class, "confusion": confusion.tolist()}
    result["gates"] = {
        "macro_f1": result["macro_f1"] >= .98,
        "each_precision": all(per_class[name]["precision"] >= .97 for name in TARGETS),
        "each_recall": all(per_class[name]["recall"] >= .97 for name in TARGETS),
    }
    result["pass"] = all(result["gates"].values())
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--epochs", type=int, default=12)
    parser.add_argument("--patience", type=int, default=3)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--seed", type=int, default=20260814)
    args = parser.parse_args()

    import torch
    from torch import nn
    from torch.utils.data import DataLoader, Dataset
    import torchvision
    from torchvision.io import read_image
    from torchvision.transforms import v2

    if not torch.cuda.is_available():
        raise RuntimeError("CRCRV11 R2 requires CUDA")
    train_path = args.data / "C11_TRAIN_PAIR_MANIFEST.json"
    holdout_path = args.data / "C11_HOLDOUT_PAIR_MANIFEST.json"
    qa_path = args.data / "C11_DATA_QA.json"
    train_payload, holdout_payload, qa = load_json(train_path), load_json(holdout_path), load_json(qa_path)
    if qa.get("C11_DATA_PASS") is not True:
        raise RuntimeError("R2 requires C11_DATA_PASS=true")
    train_pairs, holdout_pairs = train_payload["pairs"], holdout_payload["pairs"]
    fit_pairs = [row for row in train_pairs if row["development_partition"] == "fit"]
    dev_pairs = [row for row in train_pairs if row["development_partition"] == "dev"]
    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    args.output.mkdir(parents=True, exist_ok=True)

    def rows_for(pairs: list[dict], target_only: bool) -> list[dict]:
        selected = [row for row in pairs if not target_only or row["class"] in TARGETS]
        return [{"pair_id": row["pair_id"], "path": row[path], "view": view, "class": row["class"]}
                for row in selected for path, view in (("tight_path", "tight"), ("context_path", "context"))]

    transform_train = v2.Compose([
        v2.ToDtype(torch.float32, scale=True), v2.Resize((224, 224), antialias=True),
        v2.RandomHorizontalFlip(), v2.ColorJitter(brightness=.10, contrast=.10, saturation=.08, hue=.02),
        v2.RandomApply([v2.GaussianBlur(kernel_size=3, sigma=(.1, .7))], p=.15),
        v2.Normalize(mean=(.485, .456, .406), std=(.229, .224, .225)),
    ])
    transform_eval = v2.Compose([
        v2.ToDtype(torch.float32, scale=True), v2.Resize((224, 224), antialias=True),
        v2.Normalize(mean=(.485, .456, .406), std=(.229, .224, .225)),
    ])

    class Crops(Dataset):
        def __init__(self, rows: list[dict], transform, stage: str):
            self.rows, self.transform, self.stage = rows, transform, stage
        def __len__(self): return len(self.rows)
        def __getitem__(self, index):
            row = self.rows[index]
            if self.stage == "binary":
                label = int(row["class"] != CLASSES[-1])
            else:
                label = TARGETS.index(row["class"]) if row["class"] in TARGETS else 0
            return self.transform(read_image(str(args.data / row["path"]))), label, index

    def pair_probabilities(model, rows: list[dict], stage: str) -> list[dict]:
        loader = DataLoader(Crops(rows, transform_eval, stage), batch_size=args.batch_size, shuffle=False,
                            num_workers=4, pin_memory=True, persistent_workers=True)
        records = []
        model.eval()
        with torch.inference_mode():
            for images, labels, indexes in loader:
                probabilities = model(images.cuda(non_blocking=True)).softmax(1).cpu().numpy()
                for label, index, probability in zip(labels.tolist(), indexes.tolist(), probabilities):
                    records.append({**rows[index], "truth_index": label, "probabilities": probability.tolist()})
        pairs = {}
        for row in records:
            pairs.setdefault(row["pair_id"], {})[row["view"]] = row
        fused = []
        for pair_id, pair in sorted(pairs.items()):
            probability = (np.asarray(pair["tight"]["probabilities"]) + np.asarray(pair["context"]["probabilities"])) / 2
            fused.append({"pair_id": pair_id, "class": pair["tight"]["class"],
                          "truth_index": pair["tight"]["truth_index"], "probabilities": probability.tolist()})
        return fused

    def new_model(outputs: int):
        weights = torchvision.models.ConvNeXt_Tiny_Weights.IMAGENET1K_V1
        model = torchvision.models.convnext_tiny(weights=weights)
        model.classifier[-1] = nn.Linear(model.classifier[-1].in_features, outputs)
        return model.cuda(), weights

    def train_stage(stage: str, outputs: int, fit_rows: list[dict], dev_rows: list[dict], seed_offset: int):
        random.seed(args.seed + seed_offset); np.random.seed(args.seed + seed_offset); torch.manual_seed(args.seed + seed_offset)
        model, weights = new_model(outputs)
        loader = DataLoader(Crops(fit_rows, transform_train, stage), batch_size=args.batch_size, shuffle=True,
                            num_workers=4, pin_memory=True, persistent_workers=True)
        labels = [int(row["class"] != CLASSES[-1]) if stage == "binary" else TARGETS.index(row["class"]) for row in fit_rows]
        counts = Counter(labels); maximum = max(counts.values())
        class_weights = torch.tensor(
            [np.sqrt(maximum / counts[index]) for index in range(outputs)],
            dtype=torch.float32, device="cuda",
        )
        optimizer = torch.optim.AdamW(model.parameters(), lr=2e-4, weight_decay=1e-4)
        loss_fn = nn.CrossEntropyLoss(weight=class_weights)
        scaler = torch.cuda.amp.GradScaler(enabled=True)
        best = None; stale = 0; history = []
        for epoch in range(1, args.epochs + 1):
            model.train(); losses = []
            for images, labels_batch, _ in loader:
                optimizer.zero_grad(set_to_none=True)
                with torch.cuda.amp.autocast(enabled=True):
                    loss = loss_fn(model(images.cuda(non_blocking=True)), labels_batch.cuda(non_blocking=True))
                scaler.scale(loss).backward(); scaler.step(optimizer); scaler.update(); losses.append(float(loss.detach().cpu()))
            fused = pair_probabilities(model, dev_rows, stage)
            if stage == "binary":
                metric = select_binary_threshold(
                    [bool(row["truth_index"]) for row in fused], [row["probabilities"][1] for row in fused]
                )
                rank = (metric["pass"], min(metric["litter_recall"] / .98, metric["background_specificity"] / .995))
            else:
                metric = target_metrics([row["truth_index"] for row in fused], [int(np.argmax(row["probabilities"])) for row in fused])
                rank = (metric["pass"], metric["macro_f1"])
            history.append({"epoch": epoch, "train_loss": float(np.mean(losses)), "dev": metric})
            if best is None or rank > best["rank"]:
                best = {"rank": rank, "epoch": epoch, "dev": metric,
                        "state": {name: tensor.detach().cpu().clone() for name, tensor in model.state_dict().items()}}
                stale = 0
            else:
                stale += 1
            if stale >= args.patience:
                break
        model.load_state_dict(best["state"])
        return model, weights, best, history, class_weights.cpu().tolist()

    binary_fit, binary_dev, binary_holdout = rows_for(fit_pairs, False), rows_for(dev_pairs, False), rows_for(holdout_pairs, False)
    target_fit, target_dev = rows_for(fit_pairs, True), rows_for(dev_pairs, True)
    target_holdout_all = rows_for(holdout_pairs, False)
    binary_model, binary_weights, binary_best, binary_history, binary_loss_weights = train_stage(
        "binary", 2, binary_fit, binary_dev, 0
    )
    stage2_model, stage2_weights, stage2_best, stage2_history, stage2_loss_weights = train_stage(
        "target", 3, target_fit, target_dev, 1
    )
    binary_path, stage2_path = args.output / "r2_binary_rejector.pt", args.output / "r2_three_class.pt"
    torch.save({"model": binary_best["state"], "model_name": "convnext_tiny", "outputs": ["background", "litter"],
                "threshold": binary_best["dev"]["threshold"]}, binary_path)
    torch.save({"model": stage2_best["state"], "model_name": "convnext_tiny", "classes": TARGETS}, stage2_path)

    binary_fused = pair_probabilities(binary_model, binary_holdout, "binary")
    stage2_fused = pair_probabilities(stage2_model, target_holdout_all, "target")
    binary_by_pair = {row["pair_id"]: row for row in binary_fused}
    stage2_by_pair = {row["pair_id"]: row for row in stage2_fused}
    threshold = binary_best["dev"]["threshold"]
    binary_holdout_metrics = binary_metrics(
        [bool(row["truth_index"]) for row in binary_fused], [row["probabilities"][1] for row in binary_fused], threshold
    )
    stage2_target_rows = [row for row in stage2_fused if row["class"] in TARGETS]
    stage2_holdout_metrics = target_metrics(
        [TARGETS.index(row["class"]) for row in stage2_target_rows],
        [int(np.argmax(row["probabilities"])) for row in stage2_target_rows]
    )
    truth, predicted, evaluated = [], [], []
    for pair in holdout_pairs:
        binary_row = binary_by_pair[pair["pair_id"]]
        expected = CLASSES.index(pair["class"])
        if binary_row["probabilities"][1] < threshold:
            actual = 3
        else:
            stage2_row = stage2_by_pair.get(pair["pair_id"])
            actual = int(np.argmax(stage2_row["probabilities"])) if stage2_row else 0
        truth.append(expected); predicted.append(actual)
        evaluated.append({"pair_id": pair["pair_id"], "truth": CLASSES[expected], "predicted": CLASSES[actual],
                          "litter_probability": binary_row["probabilities"][1]})
    combined = classification_metrics(truth, predicted)
    payload = {
        "schema_version": 1, "protocol": "CRCRV11", "stage": "CRCRV11-05-R2-TWO-STAGE",
        "source_commit": args.source_commit, "train_manifest_sha256": sha256(train_path),
        "holdout_manifest_sha256": sha256(holdout_path), "c11_qa_sha256": sha256(qa_path),
        "augmentation": "AUG1 bounded blur and mild color jitter", "unique_sample_coverage_per_epoch": 1.0,
        "binary": {"official_weights": str(binary_weights), "best_epoch": binary_best["epoch"],
                   "dev": binary_best["dev"], "history": binary_history, "loss_weights": binary_loss_weights,
                   "checkpoint": str(binary_path.resolve()), "checkpoint_sha256": sha256(binary_path),
                   "holdout": binary_holdout_metrics},
        "stage2": {"official_weights": str(stage2_weights), "best_epoch": stage2_best["epoch"],
                   "dev": stage2_best["dev"], "history": stage2_history, "loss_weights": stage2_loss_weights,
                   "checkpoint": str(stage2_path.resolve()), "checkpoint_sha256": sha256(stage2_path),
                   "holdout": stage2_holdout_metrics},
        "combined_holdout": combined, "evaluated_candidates": evaluated,
        "CRCRV11_R2_PASS": combined["formal_pass"],
        "G10_DEV_VAL_SEALED_read": False, "VAL_NEW_read": False, "G5_V2_read": False,
    }
    write_json(args.output / "CRCRV11_R2_REPORT.json", payload)
    print(json.dumps({"binary": binary_holdout_metrics, "stage2": stage2_holdout_metrics,
                      "combined": combined, "CRCRV11_R2_PASS": payload["CRCRV11_R2_PASS"]}, indent=2))
    return 0 if payload["CRCRV11_R2_PASS"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
