#!/usr/bin/env python3
"""Train one bounded gray-background calibration model on a CUDA GPU.

The calibration corpus is generated from the frozen Stage5A synthetic scene
generator.  It never reads the retained Gazebo holdout frames or their labels.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import platform
import random
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_dataset"))
from sanitation_dataset.synthetic import (  # noqa: E402
    CLASS_COLORS_RGB,
    CLASS_ORDER,
    HEIGHT,
    WIDTH,
    generate_scene,
)


RECIPE_ID = "stage5a_gray_background_calibration_v1"
TRAIN_SEED_START = 100_000
VAL_SEED_START = 200_000


class ColorCalibrationNet(nn.Module):
    def __init__(self, hidden: int = 32):
        super().__init__()
        self.layers = nn.Sequential(
            nn.Conv2d(3, hidden, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden, hidden, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(hidden, len(CLASS_ORDER), 1),
        )

    def forward(self, images):
        return self.layers(images)


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_commit() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=ROOT,
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.SubprocessError):
        return os.environ.get("TZcup_SOURCE_COMMIT", "UNKNOWN")


def _background_style(seed: int) -> tuple[np.ndarray, float, float]:
    rng = np.random.default_rng(seed ^ 0x5A17)
    family = int(seed % 4)
    if family == 0:
        value = float(rng.uniform(0.38, 0.55))
        return np.full(3, value, np.float32), 0.94, 1.06
    if family == 1:
        value = float(rng.uniform(0.04, 0.14))
        return np.full(3, value, np.float32), 0.92, 1.08
    if family == 2:
        value = float(rng.uniform(0.18, 0.70))
        tint = rng.normal(0.0, 0.012, 3)
        return np.clip(np.full(3, value) + tint, 0, 1).astype(np.float32), 0.90, 1.10
    value = float(rng.uniform(0.24, 0.62))
    tint = rng.uniform(-0.05, 0.05, 3)
    return np.clip(np.full(3, value) + tint, 0, 1).astype(np.float32), 0.88, 1.12


def synthetic_sample(seed: int) -> tuple[np.ndarray, np.ndarray]:
    """Return an RGB uint8 image and six-class semantic labels."""
    scene = generate_scene(int(seed))
    image = scene.image_rgb.astype(np.float32) / 255.0
    labels = scene.labels.astype(np.uint8, copy=True)
    rng = np.random.default_rng(seed ^ 0xC0FFEE)

    background_color, low_gain, high_gain = _background_style(seed)
    background_mask = labels == 0
    background = np.broadcast_to(background_color, image.shape).copy()
    background += rng.normal(0.0, 0.018, background.shape).astype(np.float32)
    image[background_mask] = background[background_mask]

    object_mask = labels > 0
    per_channel_gain = rng.uniform(low_gain, high_gain, 3).astype(np.float32)
    object_gain = float(rng.uniform(0.72, 1.18))
    image[object_mask] = image[object_mask] * per_channel_gain * object_gain
    image = np.clip(image, 0.0, 1.0)
    image = cv2.GaussianBlur(image, (3, 3), sigmaX=float(rng.uniform(0.25, 0.75)))

    # Neutral hard negatives remain background labels and include colors close
    # to the gray failure mode rather than any generated positive class.
    if seed % 5 == 0:
        x0 = int(rng.integers(2, WIDTH - 24))
        y0 = int(rng.integers(2, HEIGHT - 18))
        value = float(rng.uniform(0.20, 0.70))
        tint = rng.uniform(-0.03, 0.03, 3)
        image[y0:y0 + 16, x0:x0 + 20] = np.clip(value + tint, 0.0, 1.0)
        labels[y0:y0 + 16, x0:x0 + 20] = 0

    return np.clip(image * 255.0, 0, 255).astype(np.uint8), labels


def build_split(seed_start: int, count: int, progress: str) -> tuple[torch.Tensor, torch.Tensor]:
    images = np.empty((count, 3, HEIGHT, WIDTH), dtype=np.uint8)
    labels = np.empty((count, HEIGHT, WIDTH), dtype=np.uint8)
    for index in range(count):
        image, target = synthetic_sample(seed_start + index)
        images[index] = np.transpose(image, (2, 0, 1))
        labels[index] = target
        if (index + 1) % max(1, count // 5) == 0:
            print(f"{progress}: {index + 1}/{count}", flush=True)
    return torch.from_numpy(images), torch.from_numpy(labels)


def prototype_logits(images: torch.Tensor) -> torch.Tensor:
    if images.dtype == torch.uint8:
        images = images.float().div_(255.0)
    prototypes = torch.from_numpy(CLASS_COLORS_RGB.astype(np.float32) / 255.0).to(images.device)
    flat = images.permute(0, 2, 3, 1).reshape(-1, 3)
    logits = 2.0 * flat @ prototypes.T - (prototypes.square().sum(dim=1))[None]
    return logits.reshape(images.shape[0], HEIGHT, WIDTH, -1).permute(0, 3, 1, 2)


def metrics(logits: torch.Tensor, truth: torch.Tensor) -> dict:
    prediction = logits.argmax(dim=1)
    classes = len(CLASS_ORDER)
    flat_truth = truth.reshape(-1).to(torch.int64)
    flat_prediction = prediction.reshape(-1).to(torch.int64)
    confusion = torch.zeros((classes, classes), dtype=torch.int64, device=truth.device)
    confusion.view(-1).index_add_(
        0,
        flat_truth * classes + flat_prediction,
        torch.ones_like(flat_truth),
    )
    per_class = {}
    for class_index, class_id in enumerate(CLASS_ORDER):
        tp = int(confusion[class_index, class_index])
        fp = int(confusion[:, class_index].sum()) - tp
        fn = int(confusion[class_index, :].sum()) - tp
        precision = tp / max(tp + fp, 1)
        recall = tp / max(tp + fn, 1)
        f1 = 2.0 * precision * recall / max(precision + recall, 1e-12)
        iou = tp / max(tp + fp + fn, 1)
        per_class[class_id] = {
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "iou": iou,
            "support": int(tp + fn),
        }
    background_leaf_fp = int(((truth == 0) & (prediction == 4)).sum())
    background_count = int((truth == 0).sum())
    foreground = [row["f1"] for name, row in per_class.items() if name != "background"]
    foreground_iou = [row["iou"] for name, row in per_class.items() if name != "background"]
    return {
        "macro_f1_foreground": float(np.mean(foreground)),
        "macro_iou_foreground": float(np.mean(foreground_iou)),
        "background_specificity": per_class["background"]["recall"],
        "background_leaf_false_positive_rate": background_leaf_fp / max(background_count, 1),
        "background_leaf_false_positive_pixels": background_leaf_fp,
        "pixel_count": background_count,
        "per_class": per_class,
        "confusion_matrix": confusion.tolist(),
    }


def gpu_sampler(stop: threading.Event, samples: list[int]) -> None:
    while not stop.is_set():
        try:
            value = subprocess.check_output(
                [
                    "nvidia-smi",
                    "--query-gpu=utilization.gpu",
                    "--format=csv,noheader,nounits",
                ],
                text=True,
                timeout=5,
            ).strip().splitlines()[0]
            samples.append(int(value))
        except (OSError, subprocess.SubprocessError, ValueError, IndexError):
            pass
        stop.wait(0.5)


def evaluate(model: nn.Module, images: torch.Tensor, labels: torch.Tensor, device: torch.device) -> dict:
    model.eval()
    with torch.no_grad():
        images = images.to(device)
        if images.dtype == torch.uint8:
            images = images.float().div_(255.0)
        logits = model(images)
        return metrics(logits, labels.to(device))


def train(args) -> dict:
    if not torch.cuda.is_available() and not args.allow_cpu:
        raise SystemExit("CUDA is required unless --allow-cpu is explicit")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    output = Path(args.output)
    if output.exists():
        raise SystemExit(f"fresh output directory required: {output}")
    output.mkdir(parents=True)
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    if device.type == "cuda":
        torch.cuda.manual_seed_all(args.seed)
        torch.cuda.reset_peak_memory_stats()

    train_images, train_labels = build_split(TRAIN_SEED_START, args.train_scenes, "train")
    val_images, val_labels = build_split(VAL_SEED_START, args.val_scenes, "val")
    loader = DataLoader(
        TensorDataset(train_images, train_labels),
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.workers,
        pin_memory=device.type == "cuda",
        persistent_workers=args.workers > 0,
        generator=torch.Generator().manual_seed(args.seed),
    )

    model = ColorCalibrationNet(args.hidden).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.learning_rate, weight_decay=1e-4)
    class_weights = torch.tensor([0.45, 4.0, 4.0, 4.0, 4.0, 4.0], device=device)
    scaler = torch.cuda.amp.GradScaler(enabled=device.type == "cuda")
    gpu_samples: list[int] = []
    stop_sampler = threading.Event()
    sampler = threading.Thread(target=gpu_sampler, args=(stop_sampler, gpu_samples), daemon=True)
    sampler.start()
    started = time.perf_counter()
    curves = []
    for epoch in range(1, args.epochs + 1):
        model.train()
        losses = []
        for images, labels in loader:
            images = images.to(device, non_blocking=True).float().div_(255.0)
            labels = labels.to(device, non_blocking=True).long()
            image_gain = torch.empty((images.shape[0], 1, 1, 1), device=device).uniform_(0.88, 1.12)
            images = (images * image_gain).clamp_(0.0, 1.0)
            optimizer.zero_grad(set_to_none=True)
            with torch.cuda.amp.autocast(enabled=device.type == "cuda"):
                logits = model(images)
                cross_entropy = nn.functional.cross_entropy(logits, labels, weight=class_weights)
                probability = torch.softmax(logits, dim=1)
                one_hot = nn.functional.one_hot(labels, len(CLASS_ORDER)).permute(0, 3, 1, 2).float()
                intersection = (probability[:, 1:] * one_hot[:, 1:]).sum(dim=(0, 2, 3))
                denominator = probability[:, 1:].sum(dim=(0, 2, 3)) + one_hot[:, 1:].sum(dim=(0, 2, 3))
                loss = cross_entropy + 1.0 - ((2.0 * intersection + 1.0) / (denominator + 1.0)).mean()
            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()
            losses.append(float(loss.detach()))
        curves.append({"epoch": epoch, "loss": float(np.mean(losses))})
        print(f"epoch {epoch}/{args.epochs}: loss={curves[-1]['loss']:.6f}", flush=True)
    stop_sampler.set()
    sampler.join(timeout=5)
    duration_s = time.perf_counter() - started

    baseline_logits = prototype_logits(val_images.to(device))
    baseline_metrics = metrics(baseline_logits, val_labels.to(device))
    calibrated_metrics = evaluate(model, val_images, val_labels, device)
    model = model.cpu().eval()
    onnx_path = output / "stage5a_gray_calibrated_perception.onnx"
    example = torch.zeros((1, 3, HEIGHT, WIDTH), dtype=torch.float32)
    torch.onnx.export(
        model,
        example,
        onnx_path,
        input_names=["images"],
        output_names=["logits"],
        opset_version=13,
    )
    import onnx

    graph = onnx.load(onnx_path)
    onnx.checker.check_model(graph)
    operators = Counter(node.op_type for node in graph.graph.node)
    model_sha256 = sha256_file(onnx_path)
    report = {
        "schema_version": 1,
        "recipe_id": RECIPE_ID,
        "recipe_sha256": sha256_file(Path(__file__)),
        "source_commit": source_commit(),
        "device": str(device),
        "gpu_name": torch.cuda.get_device_name(0) if device.type == "cuda" else None,
        "gpu_driver_samples_percent": {
            "count": len(gpu_samples),
            "mean": float(np.mean(gpu_samples)) if gpu_samples else None,
            "max": max(gpu_samples) if gpu_samples else None,
        },
        "peak_training_vram_mib": (
            torch.cuda.max_memory_allocated() / 1048576.0 if device.type == "cuda" else None
        ),
        "duration_s": duration_s,
        "train_scene_seeds": [TRAIN_SEED_START, TRAIN_SEED_START + args.train_scenes - 1],
        "val_scene_seeds": [VAL_SEED_START, VAL_SEED_START + args.val_scenes - 1],
        "train_scene_count": int(train_images.shape[0]),
        "val_scene_count": int(val_images.shape[0]),
        "frozen_holdout_used": False,
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "hidden_channels": args.hidden,
        "learning_rate": args.learning_rate,
        "class_weights": class_weights.cpu().tolist(),
        "parameter_count": sum(parameter.numel() for parameter in model.parameters()),
        "curves": curves,
        "synthetic_validation_prototype": baseline_metrics,
        "synthetic_validation_calibrated": calibrated_metrics,
        "onnx": {
            "path": onnx_path.name,
            "sha256": model_sha256,
            "bytes": onnx_path.stat().st_size,
            "input_shape": [1, 3, HEIGHT, WIDTH],
            "output_shape": [1, len(CLASS_ORDER), HEIGHT, WIDTH],
            "operators": dict(sorted(operators.items())),
        },
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "torch": torch.__version__,
            "numpy": np.__version__,
            "cuda_runtime": torch.version.cuda,
            "cudnn": torch.backends.cudnn.version(),
        },
    }
    (output / "training_receipt.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--train-scenes", type=int, default=4096)
    parser.add_argument("--val-scenes", type=int, default=512)
    parser.add_argument("--epochs", type=int, default=24)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--hidden", type=int, default=32)
    parser.add_argument("--learning-rate", type=float, default=3e-3)
    parser.add_argument("--workers", type=int, default=min(8, os.cpu_count() or 1))
    parser.add_argument("--seed", type=int, default=20260914)
    parser.add_argument("--allow-cpu", action="store_true")
    args = parser.parse_args()
    report = train(args)
    print(json.dumps({
        "status": "CALIBRATION_COMPLETE",
        "onnx": report["onnx"],
        "synthetic_validation_prototype": {
            "macro_f1_foreground": report["synthetic_validation_prototype"]["macro_f1_foreground"],
            "background_leaf_false_positive_rate": report["synthetic_validation_prototype"]["background_leaf_false_positive_rate"],
        },
        "synthetic_validation_calibrated": {
            "macro_f1_foreground": report["synthetic_validation_calibrated"]["macro_f1_foreground"],
            "background_leaf_false_positive_rate": report["synthetic_validation_calibrated"]["background_leaf_false_positive_rate"],
        },
        "gpu_driver_samples_percent": report["gpu_driver_samples_percent"],
        "duration_s": report["duration_s"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
