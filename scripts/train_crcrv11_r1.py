#!/usr/bin/env python3
"""Train and evaluate CRCRV11 R1 on the runtime-faithful C11 dataset."""

from __future__ import annotations

import argparse
from collections import Counter
import copy
import hashlib
import json
from pathlib import Path
import random

import numpy as np


CLASSES = ("plastic_bottle", "metal_can", "paper_litter", "background_or_unknown")
TARGETS = CLASSES[:3]
AUGMENTATIONS = {
    "AUG0": {"color_jitter": {"brightness": .10, "contrast": .10, "saturation": .08, "hue": .02}},
    "AUG1": {"color_jitter": {"brightness": .10, "contrast": .10, "saturation": .08, "hue": .02}, "blur": True},
    "AUG2": {"color_jitter": {"brightness": .10, "contrast": .10, "saturation": .08, "hue": .02}, "modest_crop_scale": [.90, 1.0]},
}


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def classification_metrics(truth: list[int], predicted: list[int]) -> dict:
    confusion = np.zeros((len(CLASSES), len(CLASSES)), dtype=np.int64)
    for expected, actual in zip(truth, predicted):
        confusion[expected, actual] += 1
    per_class, f1_values = {}, []
    for index, name in enumerate(CLASSES):
        tp = int(confusion[index, index])
        fp = int(confusion[:, index].sum() - tp)
        fn = int(confusion[index, :].sum() - tp)
        precision = tp / max(tp + fp, 1)
        recall = tp / max(tp + fn, 1)
        f1 = 2 * precision * recall / max(precision + recall, 1e-12)
        per_class[name] = {"precision": precision, "recall": recall, "f1": f1,
                           "support": int(confusion[index].sum())}
        f1_values.append(f1)
    target_macro_f1 = float(np.mean([per_class[name]["f1"] for name in TARGETS]))
    result = {
        "macro_f1": float(np.mean(f1_values)), "target_macro_f1": target_macro_f1,
        "background_specificity": per_class[CLASSES[-1]]["recall"],
        "per_class": per_class, "confusion": confusion.tolist(),
        "accuracy": float(np.trace(confusion) / max(confusion.sum(), 1)),
    }
    result["internal_gates"] = {
        "background_specificity": result["background_specificity"] >= .98,
        "target_macro_f1": target_macro_f1 >= .95,
    }
    result["internal_pass"] = all(result["internal_gates"].values())
    result["formal_gates"] = {
        "macro_f1": result["macro_f1"] >= .98,
        "each_target_precision": all(per_class[name]["precision"] >= .97 for name in TARGETS),
        "each_target_recall": all(per_class[name]["recall"] >= .97 for name in TARGETS),
        "background_specificity": result["background_specificity"] >= .995,
        "paper_precision": per_class["paper_litter"]["precision"] >= .98,
        "metal_recall": per_class["metal_can"]["recall"] >= .97,
    }
    result["formal_pass"] = all(result["formal_gates"].values())
    return result


def rank_metrics(metrics: dict) -> tuple:
    return (
        metrics["internal_pass"],
        min(metrics["background_specificity"] / .98, metrics["target_macro_f1"] / .95),
        metrics["macro_f1"],
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--augmentations", nargs="+", choices=tuple(AUGMENTATIONS), default=list(AUGMENTATIONS))
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
        raise RuntimeError("CRCRV11 R1 training requires CUDA")
    train_manifest = args.data / "C11_TRAIN_PAIR_MANIFEST.json"
    holdout_manifest = args.data / "C11_HOLDOUT_PAIR_MANIFEST.json"
    qa_path = args.data / "C11_DATA_QA.json"
    train_payload, holdout_payload, qa = load_json(train_manifest), load_json(holdout_manifest), load_json(qa_path)
    if qa.get("C11_DATA_PASS") is not True:
        raise RuntimeError("R1 requires C11_DATA_PASS=true")
    if any(train_payload.get(name) is True or holdout_payload.get(name) is True for name in (
        "G10_DEV_VAL_SEALED_read", "VAL_NEW_read", "G5_V2_read"
    )):
        raise RuntimeError("sealed boundary is already consumed")
    train_pairs, holdout_pairs = train_payload["pairs"], holdout_payload["pairs"]
    fit_pairs = [row for row in train_pairs if row["development_partition"] == "fit"]
    dev_pairs = [row for row in train_pairs if row["development_partition"] == "dev"]
    if not fit_pairs or not dev_pairs or {row["class"] for row in fit_pairs} != set(CLASSES) or {row["class"] for row in dev_pairs} != set(CLASSES):
        raise ValueError("R1 fit/dev partitions must both contain all four classes")

    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    class_to_index = {name: index for index, name in enumerate(CLASSES)}

    def expand_pairs(pairs: list[dict]) -> list[dict]:
        return [
            {"pair_id": row["pair_id"], "path": row[path_name], "view": view,
             "class": row["class"], "source_world": row["source_world"]}
            for row in pairs for path_name, view in (("tight_path", "tight"), ("context_path", "context"))
        ]

    fit_rows, dev_rows, holdout_rows = expand_pairs(fit_pairs), expand_pairs(dev_pairs), expand_pairs(holdout_pairs)

    def transform_for(augmentation: str | None):
        ops = [v2.ToDtype(torch.float32, scale=True), v2.Resize((224, 224), antialias=True)]
        if augmentation:
            config = AUGMENTATIONS[augmentation]
            ops.extend([v2.RandomHorizontalFlip(), v2.ColorJitter(**config["color_jitter"])])
            if config.get("blur"):
                ops.append(v2.RandomApply([v2.GaussianBlur(kernel_size=3, sigma=(.1, .7))], p=.15))
            if config.get("modest_crop_scale"):
                ops.append(v2.RandomResizedCrop((224, 224), scale=tuple(config["modest_crop_scale"]), ratio=(.92, 1.08), antialias=True))
        ops.append(v2.Normalize(mean=(.485, .456, .406), std=(.229, .224, .225)))
        return v2.Compose(ops)

    class Crops(Dataset):
        def __init__(self, rows: list[dict], transform):
            self.rows, self.transform = rows, transform
        def __len__(self): return len(self.rows)
        def __getitem__(self, index):
            row = self.rows[index]
            return self.transform(read_image(str(args.data / row["path"]))), class_to_index[row["class"]], index

    eval_transform = transform_for(None)

    def evaluate(model, rows: list[dict]) -> dict:
        loader = DataLoader(Crops(rows, eval_transform), batch_size=args.batch_size, shuffle=False,
                            num_workers=4, pin_memory=True, persistent_workers=True)
        evaluated = []
        model.eval()
        with torch.inference_mode():
            for images, labels, indexes in loader:
                probabilities = model(images.cuda(non_blocking=True)).softmax(1).cpu()
                for label, index, probability in zip(labels.tolist(), indexes.tolist(), probabilities):
                    row = rows[index]
                    evaluated.append({**row, "truth_index": label, "probabilities": probability.tolist()})
        by_pair = {}
        for row in evaluated:
            by_pair.setdefault(row["pair_id"], {})[row["view"]] = row
        fused_truth, fused_predicted, fused_rows = [], [], []
        for pair_id, pair in sorted(by_pair.items()):
            if set(pair) != {"tight", "context"}:
                raise ValueError(f"incomplete pair: {pair_id}")
            probability = (np.asarray(pair["tight"]["probabilities"]) + np.asarray(pair["context"]["probabilities"])) / 2
            expected, actual = int(pair["tight"]["truth_index"]), int(np.argmax(probability))
            fused_truth.append(expected); fused_predicted.append(actual)
            fused_rows.append({
                "pair_id": pair_id, "truth": CLASSES[expected], "predicted": CLASSES[actual],
                "class_probabilities": {name: float(probability[index]) for index, name in enumerate(CLASSES)},
            })
        view_metrics = {}
        for view in ("tight", "context"):
            selected = [row for row in evaluated if row["view"] == view]
            view_metrics[view] = classification_metrics(
                [row["truth_index"] for row in selected], [int(np.argmax(row["probabilities"])) for row in selected]
            )
        return {"candidate_fused": classification_metrics(fused_truth, fused_predicted),
                "per_view": view_metrics, "evaluated_candidates": fused_rows}

    args.output.mkdir(parents=True, exist_ok=True)
    fit_counts = Counter(row["class"] for row in fit_rows)
    max_count = max(fit_counts.values())
    loss_weights = torch.tensor([
        np.sqrt(max_count / fit_counts[name]) for name in CLASSES
    ], dtype=torch.float32, device="cuda")
    trials = []
    for trial_index, augmentation in enumerate(args.augmentations):
        trial_seed = args.seed + trial_index
        random.seed(trial_seed); np.random.seed(trial_seed); torch.manual_seed(trial_seed)
        weights = torchvision.models.ConvNeXt_Tiny_Weights.IMAGENET1K_V1
        model = torchvision.models.convnext_tiny(weights=weights)
        model.classifier[-1] = nn.Linear(model.classifier[-1].in_features, len(CLASSES))
        model.cuda()
        loader = DataLoader(Crops(fit_rows, transform_for(augmentation)), batch_size=args.batch_size,
                            shuffle=True, num_workers=4, pin_memory=True, persistent_workers=True)
        optimizer = torch.optim.AdamW(model.parameters(), lr=2e-4, weight_decay=1e-4)
        loss_fn = nn.CrossEntropyLoss(weight=loss_weights)
        scaler = torch.cuda.amp.GradScaler(enabled=True)
        history, best = [], None
        stale = 0
        for epoch in range(1, args.epochs + 1):
            model.train(); losses = []
            for images, labels, _ in loader:
                optimizer.zero_grad(set_to_none=True)
                with torch.cuda.amp.autocast(enabled=True):
                    loss = loss_fn(model(images.cuda(non_blocking=True)), labels.cuda(non_blocking=True))
                scaler.scale(loss).backward(); scaler.step(optimizer); scaler.update()
                losses.append(float(loss.detach().cpu()))
            dev = evaluate(model, dev_rows)
            row = {"epoch": epoch, "train_loss": float(np.mean(losses)), "dev": dev["candidate_fused"]}
            history.append(row)
            if best is None or rank_metrics(row["dev"]) > rank_metrics(best["dev"]):
                best = {"epoch": epoch, "dev": row["dev"],
                        "state": {name: tensor.detach().cpu().clone() for name, tensor in model.state_dict().items()}}
                stale = 0
            else:
                stale += 1
            if stale >= args.patience:
                break
        checkpoint_path = args.output / f"r1_{augmentation.lower()}_best.pt"
        torch.save({
            "model": best["state"], "model_name": "convnext_tiny", "classes": CLASSES,
            "official_weights": str(weights), "augmentation": augmentation, "seed": trial_seed,
            "best_epoch": best["epoch"], "dev_metrics": best["dev"],
        }, checkpoint_path)
        trials.append({
            "augmentation": augmentation, "seed": trial_seed, "history": history,
            "best_epoch": best["epoch"], "dev": best["dev"],
            "checkpoint": str(checkpoint_path.resolve()), "checkpoint_sha256": sha256(checkpoint_path),
        })
        del model
        torch.cuda.empty_cache()

    selected = max(trials, key=lambda row: rank_metrics(row["dev"]))
    selected_checkpoint = torch.load(selected["checkpoint"], map_location="cpu", weights_only=False)
    model = torchvision.models.convnext_tiny(weights=None)
    model.classifier[-1] = nn.Linear(model.classifier[-1].in_features, len(CLASSES))
    model.load_state_dict(selected_checkpoint["model"]); model.cuda().eval()
    holdout = evaluate(model, holdout_rows)
    selected_path = args.output / "r1_convnext_tiny_selected.pt"
    torch.save(selected_checkpoint, selected_path)
    aggregate = holdout["candidate_fused"]
    payload = {
        "schema_version": 1, "protocol": "CRCRV11", "stage": "CRCRV11-04-R1-RUNTIME-FAITHFUL-CONVNEXT",
        "source_commit": args.source_commit,
        "initialization": "official ImageNet ConvNeXt-Tiny",
        "initialization_candidates_evaluated": ["official ImageNet ConvNeXt-Tiny"],
        "train_manifest_sha256": sha256(train_manifest), "holdout_manifest_sha256": sha256(holdout_manifest),
        "c11_qa_sha256": sha256(qa_path), "fit_pairs": len(fit_pairs), "dev_pairs": len(dev_pairs),
        "holdout_pairs": len(holdout_pairs), "unique_sample_coverage_per_epoch": 1.0,
        "sampler": "shuffle_without_replacement", "loss_class_weights": loss_weights.cpu().tolist(),
        "augmentations": AUGMENTATIONS, "trials": trials,
        "selected_augmentation": selected["augmentation"], "selected_checkpoint": str(selected_path.resolve()),
        "selected_checkpoint_sha256": sha256(selected_path), "selected_dev": selected["dev"],
        "holdout": holdout, "CRCRV11_R1_PASS": aggregate["formal_pass"],
        "G10_DEV_VAL_SEALED_read": False, "VAL_NEW_read": False, "G5_V2_read": False,
    }
    write_json(args.output / "CRCRV11_R1_REPORT.json", payload)
    print(json.dumps({
        "selected_augmentation": selected["augmentation"], "selected_dev": selected["dev"],
        "holdout_candidate_fused": aggregate, "per_view": holdout["per_view"],
        "checkpoint_sha256": payload["selected_checkpoint_sha256"], "CRCRV11_R1_PASS": payload["CRCRV11_R1_PASS"],
    }, indent=2))
    return 0 if payload["CRCRV11_R1_PASS"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
