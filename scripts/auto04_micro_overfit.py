#!/usr/bin/env python3
"""AUTO-04 task-specific micro-overfit for direct detection and area segmentation."""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import random
import shutil
import time

import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset


ROOT = Path(__file__).resolve().parents[1]
LEARNING_PACKAGE = ROOT / "starter_ws" / "src" / "sanitation_learning"
import sys

sys.path.insert(0, str(LEARNING_PACKAGE))
from sanitation_learning.auto04_contract import (  # noqa: E402
    Detection,
    box_iou,
    decode_centernet_outputs,
    encode_centernet_targets,
)


DISCRETE_NAMES = ("plastic_bottle", "metal_can", "paper_litter")
AREA_NAMES = ("leaf_pile", "puddle")
SEMANTIC_TO_DISCRETE = {1: 0, 2: 1, 3: 2}
SEMANTIC_TO_AREA = {4: 1, 5: 2}
DETECTOR_SIZE = 192
AREA_SIZE = 128
STRIDE = 4
SEED = 20260730


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_records(root: Path) -> list[dict]:
    rows: list[dict] = []
    for scene in sorted((root / "scenes").glob("scene_*")):
        manifest = json.loads((scene / "scene_manifest.json").read_text())
        capture = json.loads((scene / "capture_report.json").read_text())
        if manifest["split"] != "train":
            continue
        for record in capture["records"]:
            rows.append(
                {
                    "scene": int(manifest["scene_seed"]),
                    "frame": int(record["frame_index"]),
                    "world": manifest["world_id"],
                    "negative_only": bool(manifest["negative_only"]),
                    "rgb": scene / record["paths"]["rgb"],
                    "semantic": scene / record["paths"]["semantic"],
                    "instance": scene / record["paths"]["instance"],
                    "depth": scene / record["paths"]["depth"],
                }
            )
    return rows


def instance_boxes(semantic: np.ndarray, instance: np.ndarray) -> list[dict]:
    boxes: list[dict] = []
    for instance_id in (int(value) for value in np.unique(instance) if int(value) != 0):
        mask = instance == instance_id
        labels = semantic[mask].astype(np.int64)
        label = int(np.bincount(labels, minlength=6).argmax())
        if label not in SEMANTIC_TO_DISCRETE:
            continue
        ys, xs = np.nonzero(mask)
        if xs.size == 0:
            continue
        boxes.append(
            {
                "semantic_label": label,
                "class_index": SEMANTIC_TO_DISCRETE[label],
                "bbox_xyxy": [
                    float(xs.min()),
                    float(ys.min()),
                    float(xs.max() + 1),
                    float(ys.max() + 1),
                ],
                "mask_area": int(mask.sum()),
            }
        )
    return boxes


def square_crop(
    width: int, height: int, bbox: tuple[float, float, float, float], scale: float
) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = bbox
    center_x = (x1 + x2) * 0.5
    center_y = (y1 + y2) * 0.5
    side = max(64.0, max(x2 - x1, y2 - y1) * scale)
    side = min(side, float(min(width, height)))
    left = int(round(center_x - side * 0.5))
    top = int(round(center_y - side * 0.5))
    left = max(0, min(width - int(side), left))
    top = max(0, min(height - int(side), top))
    return left, top, left + int(side), top + int(side)


def transform_boxes(
    boxes: list[dict],
    crop: tuple[int, int, int, int],
    output_size: int,
) -> list[dict]:
    left, top, right, bottom = crop
    scale_x = output_size / (right - left)
    scale_y = output_size / (bottom - top)
    transformed = []
    for item in boxes:
        x1, y1, x2, y2 = item["bbox_xyxy"]
        x1, x2 = max(left, x1), min(right, x2)
        y1, y2 = max(top, y1), min(bottom, y2)
        if x2 <= x1 or y2 <= y1:
            continue
        visible_fraction = (x2 - x1) * (y2 - y1) / max(
            (item["bbox_xyxy"][2] - item["bbox_xyxy"][0])
            * (item["bbox_xyxy"][3] - item["bbox_xyxy"][1]),
            1.0,
        )
        if visible_fraction < 0.7:
            continue
        transformed.append(
            {
                "class_index": item["class_index"],
                "bbox_xyxy": [
                    (x1 - left) * scale_x,
                    (y1 - top) * scale_y,
                    (x2 - left) * scale_x,
                    (y2 - top) * scale_y,
                ],
            }
        )
    return transformed


def build_detector_samples(rows: list[dict]) -> list[dict]:
    candidates: dict[int, list[dict]] = defaultdict(list)
    negatives: list[dict] = []
    for row in rows:
        semantic = np.load(row["semantic"], allow_pickle=False)
        instance = np.load(row["instance"], allow_pickle=False)
        boxes = instance_boxes(semantic, instance)
        if not boxes:
            if row["negative_only"]:
                negatives.append({**row, "crop": (192, 112, 448, 368), "boxes": []})
            continue
        for primary in boxes:
            x1, y1, x2, y2 = primary["bbox_xyxy"]
            shortest = min(x2 - x1, y2 - y1)
            bucket = 0 if shortest < 8 else 1 if shortest < 18 else 2
            crop = square_crop(
                semantic.shape[1],
                semantic.shape[0],
                tuple(primary["bbox_xyxy"]),
                scale=(9.0, 6.0, 4.0)[bucket],
            )
            converted = transform_boxes(boxes, crop, DETECTOR_SIZE)
            if any(item["class_index"] == primary["class_index"] for item in converted):
                candidates[primary["class_index"]].append(
                    {**row, "crop": crop, "boxes": converted, "size_bucket": bucket}
                )
    selected: list[dict] = []
    for class_index in range(3):
        pool = sorted(
            candidates[class_index],
            key=lambda item: (
                item["size_bucket"],
                item["scene"],
                item["frame"],
                str(item["rgb"]),
            ),
        )
        per_bucket = {0: 0, 1: 0, 2: 0}
        used: set[tuple[int, int, tuple[int, ...]]] = set()
        while len([item for item in selected if item["primary_class"] == class_index]) < 12:
            progress = False
            for item in pool:
                key = (item["scene"], item["frame"], tuple(item["crop"]))
                bucket = item["size_bucket"]
                if key in used or per_bucket[bucket] >= 4:
                    continue
                selected.append({**item, "primary_class": class_index})
                used.add(key)
                per_bucket[bucket] += 1
                progress = True
                break
            if not progress:
                raise RuntimeError(f"insufficient detector samples for class {class_index}")
    for row in negatives[:10]:
        selected.append({**row, "primary_class": None, "size_bucket": None})
    return selected


def build_area_samples(rows: list[dict]) -> list[dict]:
    candidates: dict[int, list[dict]] = defaultdict(list)
    negatives: list[dict] = []
    for row in rows:
        semantic = np.load(row["semantic"], allow_pickle=False)
        labels = set(int(value) for value in np.unique(semantic))
        if not labels.intersection({4, 5}):
            if row["negative_only"]:
                negatives.append({**row, "crop": (192, 112, 448, 368)})
            continue
        for label in (4, 5):
            ys, xs = np.nonzero(semantic == label)
            if xs.size == 0:
                continue
            bbox = (float(xs.min()), float(ys.min()), float(xs.max() + 1), float(ys.max() + 1))
            crop = square_crop(
                semantic.shape[1], semantic.shape[0], bbox, scale=1.35
            )
            candidates[label].append({**row, "crop": crop, "primary_label": label})
    selected: list[dict] = []
    for label in (4, 5):
        distinct = {}
        for item in candidates[label]:
            key = (item["scene"], item["frame"], tuple(item["crop"]))
            distinct.setdefault(key, item)
        pool = sorted(distinct.values(), key=lambda item: (item["scene"], item["frame"]))
        if len(pool) < 20:
            raise RuntimeError(f"insufficient area samples for label {label}")
        step = max(1, len(pool) // 20)
        selected.extend(pool[::step][:20])
    selected.extend({**item, "primary_label": None} for item in negatives[:20])
    return selected


def load_crop(row: dict, output_size: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    image = cv2.cvtColor(cv2.imread(str(row["rgb"])), cv2.COLOR_BGR2RGB)
    semantic = np.load(row["semantic"], allow_pickle=False)
    instance = np.load(row["instance"], allow_pickle=False)
    left, top, right, bottom = row["crop"]
    image = cv2.resize(
        image[top:bottom, left:right],
        (output_size, output_size),
        interpolation=cv2.INTER_AREA,
    )
    semantic = cv2.resize(
        semantic[top:bottom, left:right],
        (output_size, output_size),
        interpolation=cv2.INTER_NEAREST,
    )
    instance = cv2.resize(
        instance[top:bottom, left:right].astype(np.int32),
        (output_size, output_size),
        interpolation=cv2.INTER_NEAREST,
    )
    return image, semantic, instance


class DetectorDataset(Dataset):
    def __init__(self, rows: list[dict]):
        self.rows = rows

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int):
        row = self.rows[index]
        image, _, _ = load_crop(row, DETECTOR_SIZE)
        targets = encode_centernet_targets(
            row["boxes"],
            input_width=DETECTOR_SIZE,
            input_height=DETECTOR_SIZE,
            stride=STRIDE,
            class_count=3,
        )
        tensor = torch.from_numpy(
            np.ascontiguousarray(image.transpose(2, 0, 1), dtype=np.float32)
            / 255.0
        )
        return (
            tensor,
            torch.from_numpy(targets["heatmap"]),
            torch.from_numpy(targets["offset"]),
            torch.from_numpy(targets["size"]),
            torch.from_numpy(targets["regression_mask"]),
        )


class AreaDataset(Dataset):
    def __init__(self, rows: list[dict]):
        self.rows = rows

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int):
        image, semantic, _ = load_crop(self.rows[index], AREA_SIZE)
        target = np.zeros_like(semantic, dtype=np.int64)
        target = np.stack((semantic == 4, semantic == 5)).astype(np.float32)
        tensor = torch.from_numpy(
            np.ascontiguousarray(image.transpose(2, 0, 1), dtype=np.float32)
            / 255.0
        )
        return tensor, torch.from_numpy(target)


class DirectDetector(nn.Module):
    def __init__(self, width: int = 48):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, width, 5, stride=2, padding=2),
            nn.BatchNorm2d(width),
            nn.SiLU(),
            nn.Conv2d(width, width * 2, 3, stride=2, padding=1),
            nn.BatchNorm2d(width * 2),
            nn.SiLU(),
            nn.Conv2d(width * 2, width * 2, 3, padding=1),
            nn.SiLU(),
            nn.Conv2d(width * 2, width * 2, 3, padding=1),
            nn.SiLU(),
        )
        self.heatmap = nn.Conv2d(width * 2, 3, 1)
        self.offset = nn.Conv2d(width * 2, 2, 1)
        self.size = nn.Sequential(nn.Conv2d(width * 2, 2, 1), nn.Softplus())
        nn.init.constant_(self.heatmap.bias, -2.19)

    def forward(self, image):
        features = self.features(image)
        return self.heatmap(features), self.offset(features), self.size(features)


class AreaUNet(nn.Module):
    def __init__(self, width: int = 48):
        super().__init__()
        self.enc1 = nn.Sequential(
            nn.Conv2d(3, width, 3, padding=1),
            nn.SiLU(),
            nn.Conv2d(width, width, 3, padding=1),
            nn.SiLU(),
        )
        self.enc2 = nn.Sequential(
            nn.Conv2d(width, width * 2, 3, padding=1),
            nn.SiLU(),
            nn.Conv2d(width * 2, width * 2, 3, padding=1),
            nn.SiLU(),
        )
        self.bottleneck = nn.Sequential(
            nn.Conv2d(width * 2, width * 3, 3, padding=1),
            nn.SiLU(),
            nn.Conv2d(width * 3, width * 2, 3, padding=1),
            nn.SiLU(),
        )
        self.decode = nn.Sequential(
            nn.Conv2d(width * 3, width, 3, padding=1),
            nn.SiLU(),
            nn.Conv2d(width, 2, 1),
        )

    def forward(self, image):
        import torch.nn.functional as functional

        first = self.enc1(image)
        second = self.enc2(functional.max_pool2d(first, 2))
        second = self.bottleneck(second)
        upsampled = functional.interpolate(
            second, size=first.shape[-2:], mode="bilinear", align_corners=False
        )
        return self.decode(torch.cat((first, upsampled), dim=1))


def focal_heatmap_loss(logits, target):
    prediction = torch.sigmoid(logits).clamp(1e-5, 1.0 - 1e-5)
    positive = target.eq(1.0).float()
    negative = target.lt(1.0).float()
    negative_weight = (1.0 - target).pow(4)
    positive_loss = -(prediction.log()) * (1.0 - prediction).pow(2) * positive
    negative_loss = (
        -((1.0 - prediction).log())
        * prediction.pow(2)
        * negative_weight
        * negative
    )
    count = positive.sum().clamp(min=1.0)
    return (positive_loss.sum() + negative_loss.sum()) / count


def train_detector(samples: list[dict], output: Path, device, epochs: int) -> tuple[nn.Module, dict]:
    dataset = DetectorDataset(samples)
    loader = DataLoader(
        dataset,
        batch_size=8,
        shuffle=True,
        generator=torch.Generator().manual_seed(SEED),
    )
    model = DirectDetector().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=2e-3, weight_decay=1e-6)
    curve = []
    started = time.perf_counter()
    for epoch in range(1, epochs + 1):
        model.train()
        losses = []
        for image, heatmap, offset, size, mask in loader:
            image, heatmap = image.to(device), heatmap.to(device)
            offset, size, mask = offset.to(device), size.to(device), mask.to(device)
            optimizer.zero_grad(set_to_none=True)
            predicted_heatmap, predicted_offset, predicted_size = model(image)
            heatmap_loss = focal_heatmap_loss(predicted_heatmap, heatmap)
            denominator = mask.sum().clamp(min=1.0)
            offset_loss = (torch.abs(predicted_offset - offset) * mask).sum() / denominator
            size_loss = (torch.abs(predicted_size - size) * mask).sum() / denominator
            loss = heatmap_loss + offset_loss + 0.2 * size_loss
            loss.backward()
            optimizer.step()
            losses.append(float(loss.detach()))
        if epoch == 1 or epoch % 10 == 0 or epoch == epochs:
            curve.append({"epoch": epoch, "loss": float(np.mean(losses))})
    return model, {
        "epochs": epochs,
        "duration_s": time.perf_counter() - started,
        "curve": curve,
    }


def detector_predictions(model, samples: list[dict], device) -> tuple[list[dict], dict]:
    model.eval()
    predictions: list[dict] = []
    truth: dict[tuple[int, int], list[tuple[float, ...]]] = defaultdict(list)
    with torch.no_grad():
        for frame_index, row in enumerate(samples):
            image, _, _ = load_crop(row, DETECTOR_SIZE)
            tensor = torch.from_numpy(
                np.ascontiguousarray(image.transpose(2, 0, 1)[None], dtype=np.float32)
                / 255.0
            ).to(device)
            heatmap, offset, size = model(tensor)
            decoded = decode_centernet_outputs(
                torch.sigmoid(heatmap)[0].cpu().numpy(),
                offset[0].cpu().numpy(),
                size[0].cpu().numpy(),
                stride=STRIDE,
                score_threshold=0.01,
                nms_iou_threshold=0.5,
            )
            for item in decoded:
                predictions.append(
                    {
                        "frame": frame_index,
                        "class_index": item.class_index,
                        "score": item.score,
                        "bbox_xyxy": item.bbox_xyxy,
                    }
                )
            for item in row["boxes"]:
                truth[(frame_index, item["class_index"])].append(
                    tuple(item["bbox_xyxy"])
                )

    def average_precision(iou_threshold: float) -> tuple[float, list[float]]:
        values = []
        for class_index in range(3):
            ranked = sorted(
                (
                    item
                    for item in predictions
                    if item["class_index"] == class_index
                ),
                key=lambda item: item["score"],
                reverse=True,
            )
            total = sum(
                len(items)
                for (frame, label), items in truth.items()
                if label == class_index
            )
            used: dict[int, set[int]] = defaultdict(set)
            true_positive, false_positive = [], []
            for item in ranked:
                choices = truth[(item["frame"], class_index)]
                overlaps = [
                    box_iou(item["bbox_xyxy"], box)
                    if index not in used[item["frame"]]
                    else -1.0
                    for index, box in enumerate(choices)
                ]
                best = int(np.argmax(overlaps)) if overlaps else -1
                matched = best >= 0 and overlaps[best] >= iou_threshold
                true_positive.append(int(matched))
                false_positive.append(int(not matched))
                if matched:
                    used[item["frame"]].add(best)
            cumulative_true = np.cumsum(true_positive)
            cumulative_false = np.cumsum(false_positive)
            recall = cumulative_true / max(total, 1)
            precision = cumulative_true / np.maximum(
                cumulative_true + cumulative_false, 1
            )
            values.append(
                float(
                    np.mean(
                        [
                            max(precision[recall >= level], default=0.0)
                            for level in np.linspace(0.0, 1.0, 101)
                        ]
                    )
                )
            )
        return float(np.mean(values)), values

    ap50, ap50_classes = average_precision(0.5)
    threshold = 0.2
    matched_by_class = [0, 0, 0]
    total_by_class = [0, 0, 0]
    threshold_predictions = [
        item for item in predictions if item["score"] >= threshold
    ]
    for class_index in range(3):
        for (frame, label), boxes in truth.items():
            if label != class_index:
                continue
            total_by_class[class_index] += len(boxes)
            candidates = sorted(
                (
                    item
                    for item in threshold_predictions
                    if item["frame"] == frame and item["class_index"] == class_index
                ),
                key=lambda item: item["score"],
                reverse=True,
            )
            used: set[int] = set()
            for candidate in candidates:
                overlaps = [
                    box_iou(candidate["bbox_xyxy"], box)
                    if index not in used
                    else -1.0
                    for index, box in enumerate(boxes)
                ]
                best = int(np.argmax(overlaps)) if overlaps else -1
                if best >= 0 and overlaps[best] >= 0.5:
                    used.add(best)
            matched_by_class[class_index] += len(used)
    recalls = [
        matched / max(total, 1)
        for matched, total in zip(matched_by_class, total_by_class)
    ]
    true_positive = sum(matched_by_class)
    precision = true_positive / max(len(threshold_predictions), 1)
    recall = true_positive / max(sum(total_by_class), 1)
    negative_frames = {
        index for index, row in enumerate(samples) if not row["boxes"]
    }
    negative_false_positive = sum(
        item["frame"] in negative_frames for item in threshold_predictions
    )
    return predictions, {
        "confidence_ranked": True,
        "confidence_threshold": threshold,
        "nms_tested": True,
        "ap50": ap50,
        "ap50_by_class": ap50_classes,
        "macro_recall": float(np.mean(recalls)),
        "recall_by_class": dict(zip(DISCRETE_NAMES, recalls)),
        "precision_at_threshold": precision,
        "recall_at_threshold": recall,
        "negative_only_frames": len(negative_frames),
        "negative_only_false_positive_per_frame": negative_false_positive
        / max(len(negative_frames), 1),
    }


def train_area(samples: list[dict], output: Path, device, epochs: int) -> tuple[nn.Module, dict]:
    dataset = AreaDataset(samples)
    loader = DataLoader(
        dataset,
        batch_size=10,
        shuffle=True,
        generator=torch.Generator().manual_seed(SEED),
    )
    model = AreaUNet().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-3, weight_decay=1e-6)
    positive_counts = torch.zeros(2)
    pixel_count = 0
    for _, target in DataLoader(dataset, batch_size=1):
        positive_counts += target.sum((0, 2, 3))
        pixel_count += int(target.shape[0] * target.shape[2] * target.shape[3])
    positive_weight = torch.clamp(
        (pixel_count - positive_counts) / torch.clamp(positive_counts, min=1),
        min=1.0,
        max=12.0,
    ).to(device)
    curve = []
    started = time.perf_counter()
    for epoch in range(1, epochs + 1):
        model.train()
        losses = []
        for image, target in loader:
            image, target = image.to(device), target.to(device)
            optimizer.zero_grad(set_to_none=True)
            logits = model(image)
            binary = torch.nn.functional.binary_cross_entropy_with_logits(
                logits,
                target,
                pos_weight=positive_weight.view(1, 2, 1, 1),
            )
            probability = torch.sigmoid(logits)
            intersection = (probability * target).sum((0, 2, 3))
            denominator = probability.sum((0, 2, 3)) + target.sum((0, 2, 3))
            dice_loss = 1.0 - ((2.0 * intersection + 1.0) / (denominator + 1.0)).mean()
            negative_probability = probability[target == 0]
            negative_penalty = negative_probability.pow(2).mean()
            loss = binary + dice_loss + 0.5 * negative_penalty
            loss.backward()
            optimizer.step()
            losses.append(float(loss.detach()))
        if epoch == 1 or epoch % 10 == 0 or epoch == epochs:
            curve.append({"epoch": epoch, "loss": float(np.mean(losses))})
    return model, {
        "epochs": epochs,
        "duration_s": time.perf_counter() - started,
        "curve": curve,
        "positive_weights": positive_weight.cpu().tolist(),
    }


def area_metrics(model, samples: list[dict], device) -> dict:
    intersections = np.zeros(2, dtype=np.int64)
    unions = np.zeros(2, dtype=np.int64)
    negative_false_frames = 0
    minimum_component_area = 32
    model.eval()
    with torch.no_grad():
        for index, (image, target) in enumerate(
            DataLoader(AreaDataset(samples), batch_size=1, shuffle=False)
        ):
            probability = torch.sigmoid(model(image.to(device)))[0].cpu().numpy()
            predicted = probability >= 0.5
            truth = target.numpy()[0].astype(bool)
            intersections += (predicted & truth).sum((1, 2))
            unions += (predicted | truth).sum((1, 2))
            if samples[index]["primary_label"] is None:
                false_candidate = False
                for channel in range(2):
                    count, _, stats, _ = cv2.connectedComponentsWithStats(
                        predicted[channel].astype(np.uint8), 8
                    )
                    if count > 1 and int(stats[1:, cv2.CC_STAT_AREA].max()) >= minimum_component_area:
                        false_candidate = True
                negative_false_frames += int(false_candidate)
    ious = [
        float(intersection / max(union, 1))
        for intersection, union in zip(intersections, unions)
    ]
    negative_count = sum(item["primary_label"] is None for item in samples)
    return {
        "iou_by_class": dict(zip(AREA_NAMES, ious)),
        "macro_miou": float(np.mean(ious)),
        "negative_only_frames": negative_count,
        "negative_area_false_positive_per_frame": negative_false_frames
        / max(negative_count, 1),
        "candidate_minimum_component_area_px": minimum_component_area,
        "intersection_pixels": intersections.tolist(),
        "union_pixels": unions.tolist(),
    }


def export_and_compare(
    model, shape: tuple[int, ...], path: Path, device, *, output_mode: str
) -> dict:
    import onnx
    import onnxruntime as ort

    # ONNX Runtime is the deployment CPU reference for this gate. Compare it
    # against the same PyTorch CPU graph so GPU kernel reduction differences do
    # not get mislabeled as export drift.
    model = model.cpu().eval()
    sample = torch.rand(shape, generator=torch.Generator().manual_seed(SEED))
    with torch.no_grad():
        torch_outputs = model(sample)
    if not isinstance(torch_outputs, tuple):
        torch_outputs = (torch_outputs,)
    output_names = [f"output_{index}" for index in range(len(torch_outputs))]
    torch.onnx.export(
        model,
        sample,
        path,
        input_names=["images"],
        output_names=output_names,
        opset_version=17,
    )
    graph = onnx.load(path)
    onnx.checker.check_model(graph)
    operators: dict[str, int] = defaultdict(int)
    for node in graph.graph.node:
        operators[node.op_type] += 1
    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    ort_outputs = session.run(None, {"images": sample.cpu().numpy()})
    max_error = max(
        float(
            np.max(
                np.abs(
                    expected.detach().cpu().numpy().astype(np.float64)
                    - actual.astype(np.float64)
                )
            )
        )
        for expected, actual in zip(torch_outputs, ort_outputs)
    )
    if output_mode == "detector":
        torch_detections = decode_centernet_outputs(
            torch.sigmoid(torch_outputs[0])[0].numpy(),
            torch_outputs[1][0].numpy(),
            torch_outputs[2][0].numpy(),
            stride=STRIDE,
            score_threshold=0.5,
        )
        ort_detections = decode_centernet_outputs(
            1.0 / (1.0 + np.exp(-ort_outputs[0][0])),
            ort_outputs[1][0],
            ort_outputs[2][0],
            stride=STRIDE,
            score_threshold=0.5,
        )
        matching = len(torch_detections) == len(ort_detections) and all(
            left.class_index == right.class_index
            and abs(left.score - right.score) <= 1e-4
            and max(
                abs(a - b)
                for a, b in zip(left.bbox_xyxy, right.bbox_xyxy)
            )
            <= 1e-3
            for left, right in zip(torch_detections, ort_detections)
        )
        agreement = 1.0 if matching else 0.0
    elif output_mode == "area":
        torch_scores = torch.sigmoid(torch_outputs[0]).numpy()
        ort_scores = 1.0 / (1.0 + np.exp(-ort_outputs[0]))
        torch_background = np.full(
            (shape[0], 1, shape[2], shape[3]), 0.5, dtype=np.float32
        )
        ort_background = torch_background.copy()
        agreement = float(
            np.mean(
                np.concatenate((torch_background, torch_scores), axis=1).argmax(1)
                == np.concatenate((ort_background, ort_scores), axis=1).argmax(1)
            )
        )
    else:
        raise ValueError(f"unknown output mode: {output_mode}")
    return {
        "path": path.name,
        "sha256": sha256(path),
        "bytes": path.stat().st_size,
        "batch": 1,
        "fixed_input": list(shape),
        "custom_ops": 0,
        "operator_inventory": dict(sorted(operators.items())),
        "max_numeric_output_error": max_error,
        "decoded_or_argmax_agreement": agreement,
    }


def write_manifest(output: Path) -> None:
    files = []
    for path in sorted(output.rglob("*")):
        if not path.is_file() or path.name == "artifact_manifest.json":
            continue
        files.append(
            {
                "path": path.relative_to(output).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
        )
    payload = {
        "schema_version": 1,
        "stage": "AUTO-04",
        "coverage": 1.0,
        "file_count": len(files),
        "files": files,
    }
    (output / "artifact_manifest.json").write_text(
        json.dumps(payload, indent=2) + "\n"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--implementation-commit", required=True)
    parser.add_argument("--attempt", type=int, default=1)
    parser.add_argument("--detector-epochs", type=int, default=180)
    parser.add_argument("--area-epochs", type=int, default=160)
    args = parser.parse_args()

    random.seed(SEED)
    np.random.seed(SEED)
    torch.manual_seed(SEED)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(SEED)
        torch.backends.cudnn.deterministic = True
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    data_root = Path(args.data_root)
    rows = load_records(data_root)
    detector_samples = build_detector_samples(rows)
    area_samples = build_area_samples(rows)

    detector, detector_training = train_detector(
        detector_samples, output, device, args.detector_epochs
    )
    _, detector_metrics = detector_predictions(detector, detector_samples, device)
    detector_export = export_and_compare(
        detector,
        (1, 3, DETECTOR_SIZE, DETECTOR_SIZE),
        output / "auto04_direct_detector.onnx",
        device,
        output_mode="detector",
    )

    area, area_training = train_area(
        area_samples, output, device, args.area_epochs
    )
    area_result = area_metrics(area, area_samples, device)
    area_export = export_and_compare(
        area,
        (1, 3, AREA_SIZE, AREA_SIZE),
        output / "auto04_area_segmenter.onnx",
        device,
        output_mode="area",
    )

    detector_positive_count = sum(bool(row["boxes"]) for row in detector_samples)
    detector_classes = sorted(
        {
            item["class_index"]
            for row in detector_samples
            for item in row["boxes"]
        }
    )
    detector_multi_instance = sum(len(row["boxes"]) >= 2 for row in detector_samples)
    size_buckets = sorted(
        {
            row["size_bucket"]
            for row in detector_samples
            if row["size_bucket"] is not None
        }
    )
    area_counts = {
        name: sum(
            row["primary_label"] == label
            for row in area_samples
        )
        for name, label in zip(AREA_NAMES, (4, 5))
    }
    gates = {
        "detector_positive_frames_20_to_40": 20 <= detector_positive_count <= 40,
        "detector_all_three_classes": detector_classes == [0, 1, 2],
        "detector_negative_frames_at_least_10": sum(
            not row["boxes"] for row in detector_samples
        )
        >= 10,
        "detector_multi_instance_present": detector_multi_instance > 0,
        "detector_small_medium_large_present": size_buckets == [0, 1, 2],
        "detector_train_ap50_at_least_0_95": detector_metrics["ap50"] >= 0.95,
        "detector_train_macro_recall_at_least_0_95": detector_metrics[
            "macro_recall"
        ]
        >= 0.95,
        "detector_each_class_recall_at_least_0_95": min(
            detector_metrics["recall_by_class"].values()
        )
        >= 0.95,
        "detector_negative_fp_at_most_0_05": detector_metrics[
            "negative_only_false_positive_per_frame"
        ]
        <= 0.05,
        "detector_onnx_agreement_at_least_0_9999": detector_export[
            "decoded_or_argmax_agreement"
        ]
        >= 0.9999,
        "detector_onnx_error_at_most_1e_4": detector_export[
            "max_numeric_output_error"
        ]
        <= 1e-4,
        "area_leaf_frames_at_least_20": area_counts["leaf_pile"] >= 20,
        "area_puddle_frames_at_least_20": area_counts["puddle"] >= 20,
        "area_negative_frames_at_least_10": sum(
            row["primary_label"] is None for row in area_samples
        )
        >= 10,
        "area_leaf_iou_at_least_0_95": area_result["iou_by_class"]["leaf_pile"]
        >= 0.95,
        "area_puddle_iou_at_least_0_95": area_result["iou_by_class"]["puddle"]
        >= 0.95,
        "area_macro_miou_at_least_0_95": area_result["macro_miou"] >= 0.95,
        "area_negative_fp_at_most_0_05": area_result[
            "negative_area_false_positive_per_frame"
        ]
        <= 0.05,
        "area_onnx_agreement_at_least_0_9999": area_export[
            "decoded_or_argmax_agreement"
        ]
        >= 0.9999,
    }
    report = {
        "schema_version": 1,
        "stage": "AUTO-04",
        "attempt_id": f"AUTO-04-DIRECT-DETECTOR-AREA-V{args.attempt}",
        "attempt_limit": {
            "detector_architectures": 3,
            "area_architectures": 3,
            "configs_per_architecture": 3,
        },
        "hypothesis": (
            "direct center/offset/box supervision plus task-specific crops can prove object-level detector capacity while an independent area head proves leaf/puddle mask capacity before any data expansion"
            if args.attempt == 1
            else "the ranked detector already has AP50 above 0.99, so a frozen 0.20 threshold should recover recall without negative FP; independent binary area heads and tighter target crops should remove three-class background competition"
        ),
        "changed_variables": [
            "direct anchor-free center detector",
            "task-specific real-Gazebo micro crops",
            "independent leaf/puddle binary area heads",
        ],
        "fixed_variables": [
            "source RGB/semantic/instance frames",
            "class registry",
            "frozen input shapes",
            "test sample identities",
        ],
        "stop_condition": "stop and preserve failure if any fixed AUTO-04 gate fails",
        "implementation_commit": args.implementation_commit,
        "source_data": {
            "root": str(data_root),
            "source": "Gazebo Harmonic synchronized RGB/semantic/instance G2 train split",
            "training_rows_available": len(rows),
            "ground_truth_control_input": False,
        },
        "device": str(device),
        "detector": {
            "architecture": "direct anchor-free stride-4 center heatmap + offset + bbox regression",
            "segmentation_connected_components_used_as_detector": False,
            "input_shape": [1, 3, DETECTOR_SIZE, DETECTOR_SIZE],
            "positive_frames": detector_positive_count,
            "negative_frames": sum(not row["boxes"] for row in detector_samples),
            "multi_instance_frames": detector_multi_instance,
            "size_buckets": size_buckets,
            "training": detector_training,
            "metrics": detector_metrics,
            "onnx": detector_export,
            "parameter_count": sum(parameter.numel() for parameter in detector.parameters()),
        },
        "area_segmenter": {
            "architecture": "independent RGB U-Net-style area segmentation head",
            "input_shape": [1, 3, AREA_SIZE, AREA_SIZE],
            "positive_frames": area_counts,
            "negative_frames": sum(
                row["primary_label"] is None for row in area_samples
            ),
            "partial_or_occluded_boundaries_present": True,
            "training": area_training,
            "metrics": area_result,
            "onnx": area_export,
            "parameter_count": sum(parameter.numel() for parameter in area.parameters()),
        },
        "gates": gates,
        "auto04_gate_pass": all(gates.values()),
    }
    prior = (
        output.parent
        / "autonomous_auto04_attempt1_raw"
        / "auto04_acceptance_report.json"
    )
    if args.attempt > 1 and prior.is_file():
        prior_dir = output / "prior_attempts"
        prior_dir.mkdir(parents=True, exist_ok=True)
        copied = prior_dir / "attempt1_acceptance_report.json"
        shutil.copy2(prior, copied)
        report["prior_attempts"] = [
            {
                "attempt_id": "AUTO-04-DIRECT-DETECTOR-AREA-V1",
                "report": copied.relative_to(output).as_posix(),
                "sha256": sha256(copied),
                "gate_pass": False,
            }
        ]
    (output / "auto04_acceptance_report.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    (output / "environment.json").write_text(
        json.dumps(
            {
                "python": platform.python_version(),
                "platform": platform.platform(),
                "torch": torch.__version__,
                "cuda_available": torch.cuda.is_available(),
                "cuda": torch.version.cuda,
                "gpu": torch.cuda.get_device_name(0)
                if torch.cuda.is_available()
                else None,
                "hostname": platform.node(),
            },
            indent=2,
        )
        + "\n"
    )
    (output / "attempt_ledger.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "stage": "AUTO-04",
                "attempts": [
                    *report.get("prior_attempts", []),
                    {
                        "attempt_id": report["attempt_id"],
                        "hypothesis": report["hypothesis"],
                        "changed_variables": report["changed_variables"],
                        "fixed_variables": report["fixed_variables"],
                        "stop_condition": report["stop_condition"],
                        "selected": report["auto04_gate_pass"],
                        "gate_pass": report["auto04_gate_pass"],
                    }
                ],
            },
            indent=2,
        )
        + "\n"
    )
    (output / "README.md").write_text(
        "# AUTO-04 micro-overfit evidence\n\n"
        "Direct object detector and independent leaf/puddle area segmenter evidence. "
        "This proves only task-specific train-set capacity on synchronized Gazebo frames; "
        "it is not AUTO-05 cross-world screening or competition perception evidence.\n"
    )
    write_manifest(output)
    print(
        json.dumps(
            {
                "auto04_gate_pass": report["auto04_gate_pass"],
                "detector_ap50": detector_metrics["ap50"],
                "detector_macro_recall": detector_metrics["macro_recall"],
                "area_miou": area_result["macro_miou"],
                "failed_gates": [name for name, value in gates.items() if not value],
            },
            indent=2,
        )
    )
    return 0 if report["auto04_gate_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
