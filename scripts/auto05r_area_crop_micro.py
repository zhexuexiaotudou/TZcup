#!/usr/bin/env python3
"""AUTO-05R area square-crop micro gate, mirroring the AUTO-04 pattern."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import random
import sys
import time

import cv2
import numpy as np
import torch
import torch.nn.functional as functional
from torch.utils.data import DataLoader, Dataset


ROOT = Path(__file__).resolve().parents[1]
LEARNING_PACKAGE = ROOT / "starter_ws" / "src" / "sanitation_learning"
sys.path.insert(0, str(LEARNING_PACKAGE))

from sanitation_learning.g4_data import (  # noqa: E402
    build_area_input,
    index_instance_records,
    load_camera_info,
    load_frame_rows,
    load_instance_records,
    mask_boundary,
    read_frame,
    read_rgb,
    square_crop,
)
from sanitation_learning.g4_losses import area_loss  # noqa: E402
from sanitation_learning.g4_models import build_g4_models  # noqa: E402
from sanitation_learning.g4_train import fit_model  # noqa: E402


SEED = 20260807
CROP_SIZE = 256
POSITIVE_LIMIT = 40
NEGATIVE_LIMIT = 40


def _resize_features(
    features: np.ndarray,
    crop: tuple[int, int, int, int],
    size: int,
) -> np.ndarray:
    left, top, right, bottom = crop
    return np.stack(
        [
            cv2.resize(
                features[top:bottom, left:right, channel],
                (size, size),
                interpolation=cv2.INTER_AREA,
            )
            for channel in range(features.shape[2])
        ],
        axis=-1,
    ).astype(np.float32)


class SimpleAreaUNet(torch.nn.Module):
    def __init__(self, width: int = 48):
        super().__init__()
        self.enc1 = torch.nn.Sequential(
            torch.nn.Conv2d(3, width, 3, padding=1),
            torch.nn.SiLU(),
            torch.nn.Conv2d(width, width, 3, padding=1),
            torch.nn.SiLU(),
        )
        self.enc2 = torch.nn.Sequential(
            torch.nn.Conv2d(width, width * 2, 3, padding=1),
            torch.nn.SiLU(),
            torch.nn.Conv2d(width * 2, width * 2, 3, padding=1),
            torch.nn.SiLU(),
        )
        self.bottleneck = torch.nn.Sequential(
            torch.nn.Conv2d(width * 2, width * 3, 3, padding=1),
            torch.nn.SiLU(),
            torch.nn.Conv2d(width * 3, width * 2, 3, padding=1),
            torch.nn.SiLU(),
        )
        self.decode = torch.nn.Sequential(
            torch.nn.Conv2d(width * 3, width, 3, padding=1),
            torch.nn.SiLU(),
            torch.nn.Conv2d(width, 2, 1),
        )

    def forward(self, image):
        first = self.enc1(image)
        second = self.enc2(torch.nn.functional.max_pool2d(first, 2))
        second = self.bottleneck(second)
        upsampled = torch.nn.functional.interpolate(
            second,
            size=first.shape[-2:],
            mode="bilinear",
            align_corners=False,
        )
        return self.decode(torch.cat((first, upsampled), dim=1))


def _build_samples(
    rows: list[dict],
    instances_by_key: dict[tuple[int, int], list[dict]],
    task: str,
) -> list[dict]:
    semantic_id = 4 if task == "leaf" else 5
    semantic_name = "leaf_pile" if task == "leaf" else "puddle"
    positives: list[dict] = []
    negatives: list[dict] = []
    for row in rows:
        records = instances_by_key.get(
            (int(row["scene_seed"]), int(row["frame_index"])), []
        )
        labels = {item.get("semantic_class") for item in records}
        if row.get("negative_only"):
            negatives.append({**row, "_crop": (192, 112, 448, 368)})
        elif semantic_name in labels:
            _, _, semantic, _ = read_frame(row)
            ys, xs = np.nonzero(semantic == semantic_id)
            if xs.size == 0:
                continue
            bbox = (
                float(xs.min()),
                float(ys.min()),
                float(xs.max() + 1),
                float(ys.max() + 1),
            )
            crop = square_crop(
                semantic.shape[1],
                semantic.shape[0],
                bbox,
                scale=1.35,
                minimum_side=64,
            )
            positives.append({**row, "_crop": crop})
    rng = random.Random(SEED)
    rng.shuffle(positives)
    rng.shuffle(negatives)
    return positives[:POSITIVE_LIMIT] + negatives[:NEGATIVE_LIMIT]


class AreaCropDataset(Dataset):
    def __init__(
        self,
        samples: list[dict],
        task: str,
        *,
        cache: bool = True,
        mode: str = "deep",
    ):
        self.samples = samples
        self.task = task
        self.semantic_id = 4 if task == "leaf" else 5
        self.cache = cache
        self.mode = mode
        self._cache: dict[int, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, index: int):
        if self.cache and index in self._cache:
            features, target, boundary = self._cache[index]
        else:
            row = self.samples[index]
            left, top, right, bottom = row["_crop"]
            if self.mode == "rgb":
                rgb = read_rgb(row)
                features = (
                    cv2.resize(
                        rgb[top:bottom, left:right],
                        (CROP_SIZE, CROP_SIZE),
                        interpolation=cv2.INTER_AREA,
                    ).astype(np.float32)
                    / 255.0
                )
                semantic = np.load(row["semantic_path"], allow_pickle=False)
            else:
                rgb, depth, semantic, _ = read_frame(row)
                camera_info = load_camera_info(row)
                full_features = build_area_input(
                    rgb,
                    depth,
                    (640, 480),
                    task=self.task,
                    camera_info=camera_info,
                )
                features = _resize_features(
                    full_features, (left, top, right, bottom), CROP_SIZE
                )
            semantic_crop = cv2.resize(
                semantic[top:bottom, left:right],
                (CROP_SIZE, CROP_SIZE),
                interpolation=cv2.INTER_NEAREST,
            )
            if self.mode == "rgb":
                target = np.stack(
                    (semantic_crop == 4, semantic_crop == 5)
                ).astype(np.float32)
                boundary = np.stack(
                    (
                        mask_boundary(semantic_crop == 4),
                        mask_boundary(semantic_crop == 5),
                    )
                ).astype(np.float32)
            else:
                target_2d = (semantic_crop == self.semantic_id).astype(np.float32)
                target = target_2d[None]
                boundary = mask_boundary(target_2d)[None]
            if row.get("negative_only"):
                target = np.zeros_like(target)
                boundary = np.zeros_like(boundary)
            if self.cache:
                self._cache[index] = (features, target, boundary)
        tensor = torch.from_numpy(
            np.ascontiguousarray(features.transpose(2, 0, 1), dtype=np.float32)
        )
        if self.mode == "rgb":
            target_tensor = torch.from_numpy(target).float()
            boundary_tensor = torch.from_numpy(boundary).float()
        else:
            target_tensor = torch.from_numpy(target[None]).float()
            boundary_tensor = torch.from_numpy(boundary[None]).float()
        return (
            tensor,
            target_tensor,
            boundary_tensor,
        )


def _evaluate(model, dataset: AreaCropDataset, device) -> dict:
    model.eval()
    probabilities = []
    truths = []
    negatives = []
    with torch.no_grad():
        for index, (tensor, target, _) in enumerate(
            DataLoader(dataset, batch_size=1, shuffle=False)
        ):
            outputs = model(tensor.to(device))
            if isinstance(outputs, dict):
                probability = torch.sigmoid(outputs["logits"])
            else:
                probability = torch.sigmoid(outputs)
            channel = 0 if dataset.task == "leaf" else 1
            truth = target.numpy()
            if truth.ndim == 2:
                truth = truth[None]
            probabilities.append(probability[0, channel].cpu().numpy())
            truths.append(truth[0, channel].astype(bool))
            negatives.append(bool(dataset.samples[index].get("negative_only")))
    best = None
    best_threshold = 0.5
    for threshold in (0.25, 0.35, 0.45, 0.5, 0.55, 0.65, 0.75):
        intersections = 0
        unions = 0
        negative_frames = 0
        negative_fp_frames = 0
        for probability, truth, negative in zip(
            probabilities, truths, negatives
        ):
            predicted = (probability >= threshold).astype(bool)
            intersections += int((predicted & truth).sum())
            unions += int((predicted | truth).sum())
            if negative:
                negative_frames += 1
                count, _, stats, _ = cv2.connectedComponentsWithStats(
                    predicted.astype(np.uint8), 8
                )
                false = count > 1 and int(stats[1:, cv2.CC_STAT_AREA].max()) >= 20
                negative_fp_frames += int(false)
        iou = intersections / max(unions, 1)
        neg_fp = negative_fp_frames / max(negative_frames, 1)
        if iou >= 0.95 and neg_fp <= 0.05:
            if best is None or iou > best["iou"]:
                best = {
                    "iou": iou,
                    "intersection_pixels": intersections,
                    "union_pixels": unions,
                    "negative_only_frames": negative_frames,
                    "negative_only_fp_frames": negative_fp_frames,
                    "negative_fp_per_frame": neg_fp,
                }
                best_threshold = threshold
    if best is None:
        threshold = 0.5
        intersections = 0
        unions = 0
        negative_frames = 0
        negative_fp_frames = 0
        for probability, truth, negative in zip(
            probabilities, truths, negatives
        ):
            predicted = (probability >= threshold).astype(bool)
            intersections += int((predicted & truth).sum())
            unions += int((predicted | truth).sum())
            if negative:
                negative_frames += 1
                count, _, stats, _ = cv2.connectedComponentsWithStats(
                    predicted.astype(np.uint8), 8
                )
                false = count > 1 and int(stats[1:, cv2.CC_STAT_AREA].max()) >= 20
                negative_fp_frames += int(false)
        best = {
            "iou": intersections / max(unions, 1),
            "intersection_pixels": intersections,
            "union_pixels": unions,
            "negative_only_frames": negative_frames,
            "negative_only_fp_frames": negative_fp_frames,
            "negative_fp_per_frame": negative_fp_frames / max(negative_frames, 1),
        }
    best["selected_threshold"] = best_threshold
    return best


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--evidence-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--task", required=True, choices=("leaf", "puddle"))
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--arch", choices=("deep", "simple"), default="deep")
    args = parser.parse_args()

    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    rows = load_frame_rows(args.evidence_dir / "g4_frame_manifest.jsonl", args.data_root)
    records = load_instance_records(args.evidence_dir / "g4_instance_records.jsonl")
    instances_by_key = index_instance_records(records)
    train_rows = [row for row in rows if row["split"] == "train"]
    samples = _build_samples(train_rows, instances_by_key, args.task)
    if not samples:
        raise RuntimeError("no area crop samples")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(SEED)
    np.random.seed(SEED)
    random.seed(SEED)
    dataset = AreaCropDataset(
        samples,
        args.task,
        mode="rgb" if args.arch == "simple" else "deep",
    )
    loader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=True,
        generator=torch.Generator().manual_seed(SEED),
    )
    model = (
        SimpleAreaUNet()
        if args.arch == "simple"
        else build_g4_models()[args.task]
    )
    def crop_area_loss(logits, targets, boundaries):
        positive_counts = targets.sum(dim=(0, 2, 3))
        pixel_count = targets.shape[0] * targets.shape[2] * targets.shape[3]
        pos_weight = torch.clamp(
            (pixel_count - positive_counts) / positive_counts.clamp(min=1.0),
            min=1.0,
            max=12.0,
        )
        binary = functional.binary_cross_entropy_with_logits(
            logits,
            targets,
            pos_weight=pos_weight.view(1, -1, 1, 1),
        )
        probability = torch.sigmoid(logits)
        intersection = (probability * targets).sum(dim=(0, 2, 3))
        denominator = probability.sum(dim=(0, 2, 3)) + targets.sum(dim=(0, 2, 3))
        dice = 1.0 - ((2.0 * intersection + 1.0) / (denominator + 1.0)).mean()
        negative_probability = probability[targets == 0]
        negative_penalty = negative_probability.pow(2).mean()
        return binary + dice + 0.5 * negative_penalty

    model, training = fit_model(
        model,
        loader,
        None,
        loss_fn=lambda outputs, targets, boundaries: crop_area_loss(
            outputs["logits"] if isinstance(outputs, dict) else outputs,
            targets,
            boundaries,
        ),
        device=device,
        epochs=args.epochs,
        learning_rate=args.learning_rate,
        seed=SEED,
        checkpoint_path=output / f"{args.task}_crop_micro.pt",
        early_stopping_patience=0,
        load_best=False,
    )
    metrics = _evaluate(model, dataset, device)
    gates = {
        f"{args.task}_iou": metrics["iou"] >= 0.95,
        "negative_fp_per_frame": metrics["negative_fp_per_frame"] <= 0.05,
    }
    report = {
        "schema_version": 1,
        "stage": "AUTO-05R",
        "task": "AUTO-05R-3",
        "model_type": args.task,
        "micro_mode": "square_crop_256",
        "executed": True,
        "micro_overfit_pass": bool(gates and all(gates.values())),
        "metrics": metrics,
        "gates": gates,
        "positive_samples": sum(
            not sample.get("negative_only") for sample in samples
        ),
        "negative_samples": sum(
            bool(sample.get("negative_only")) for sample in samples
        ),
        "training": training,
    }
    report_path = output / "micro_overfit_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["micro_overfit_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
