#!/usr/bin/env python3
"""AUTO-05 G3 cross-world detector and area-model screening."""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import platform
import random
import re
import subprocess
import time

import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset, WeightedRandomSampler


ROOT = Path(__file__).resolve().parents[1]
LEARNING_PACKAGE = ROOT / "starter_ws" / "src" / "sanitation_learning"
import sys

sys.path.insert(0, str(LEARNING_PACKAGE))
from sanitation_learning.auto04_contract import (  # noqa: E402
    box_iou,
    decode_centernet_outputs,
    encode_centernet_targets,
)


DISCRETE_NAMES = ("plastic_bottle", "metal_can", "paper_litter")
AREA_NAMES = ("leaf_pile", "puddle")
SEMANTIC_TO_DISCRETE = {1: 0, 2: 1, 3: 2}
INPUT_WIDTH = 384
INPUT_HEIGHT = 288
STRIDE = 4
SEED = 20260730
G4_CONTRACT_NAME = "auto05_g4_screening.yaml"
# Populated only after the checked-in G4 contract has been parsed.  Keeping
# this separate from CLI arguments makes an accidental per-run override fail
# closed rather than silently changing the preregistered experiment.
G4_FROZEN_TRAINING: dict[str, int | float | bool] | None = None


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_rows(data_root: Path, manifest_path: Path) -> list[dict]:
    rows = []
    for line in manifest_path.read_text(encoding="utf-8").splitlines():
        row = json.loads(line)
        row["rgb"] = data_root / row["rgb_path"]
        row["depth"] = data_root / row["depth_path"]
        row["semantic"] = data_root / row["semantic_path"]
        row["instance"] = data_root / row["instance_path"]
        rows.append(row)
    return rows


def instance_boxes(semantic: np.ndarray, instance: np.ndarray) -> list[dict]:
    boxes = []
    for instance_id in (
        int(value) for value in np.unique(instance) if int(value) != 0
    ):
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
                "class_index": SEMANTIC_TO_DISCRETE[label],
                "bbox_xyxy": [
                    float(xs.min()),
                    float(ys.min()),
                    float(xs.max() + 1),
                    float(ys.max() + 1),
                ],
                "short_side": float(
                    min(xs.max() - xs.min() + 1, ys.max() - ys.min() + 1)
                ),
                "mask_area": int(mask.sum()),
            }
        )
    return boxes


def read_inputs(row: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    rgb = cv2.cvtColor(cv2.imread(str(row["rgb"])), cv2.COLOR_BGR2RGB)
    depth = np.load(row["depth"], allow_pickle=False).astype(np.float32)
    semantic = np.load(row["semantic"], allow_pickle=False)
    instance = np.load(row["instance"], allow_pickle=False)
    return rgb, depth, semantic, instance


def normalize_depth(depth: np.ndarray) -> np.ndarray:
    valid = np.isfinite(depth) & (depth > 0.0)
    normalized = np.zeros_like(depth, dtype=np.float32)
    normalized[valid] = np.clip(np.log1p(depth[valid]) / math.log(11.0), 0.0, 1.0)
    return normalized


def build_model_input(
    image: np.ndarray,
    depth: np.ndarray,
    mode: str,
    attempt: int,
) -> np.ndarray:
    if attempt < 3:
        if mode == "detector":
            return image
        return np.concatenate(
            (image, normalize_depth(depth)[:, :, None]), axis=2
        )
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    gradient_x = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gradient_y = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    edge = np.clip(
        np.sqrt(gradient_x * gradient_x + gradient_y * gradient_y) * 2.5,
        0.0,
        1.0,
    )
    local_contrast = np.clip(
        np.abs(gray - cv2.GaussianBlur(gray, (11, 11), 0)) * 4.0,
        0.0,
        1.0,
    )
    depth_channel = normalize_depth(depth)
    shared = np.concatenate(
        (
            image,
            depth_channel[:, :, None],
            edge[:, :, None],
            local_contrast[:, :, None],
        ),
        axis=2,
    )
    if mode == "detector":
        return shared
    saturation = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)[:, :, 1]
    return np.concatenate((shared, saturation[:, :, None]), axis=2)


def training_row_weights(rows: list[dict], mode: str) -> list[float]:
    weights = []
    for row in rows:
        labels = set(
            int(value)
            for value in np.unique(np.load(row["semantic"], allow_pickle=False))
        )
        if mode == "detector":
            weight = 4.0 if row["negative_only"] else 1.0
            if 3 in labels:
                weight += 2.5
            if not labels.intersection({1, 2, 3}):
                weight += 2.0
        else:
            weight = 2.0 if row["negative_only"] else 1.0
            if 4 in labels:
                weight += 3.0
            if 5 in labels:
                weight += 2.0
            if not labels.intersection({4, 5}):
                weight += 1.0
        weights.append(weight)
    return weights


def require_split(rows: list[dict], expected: str, purpose: str) -> None:
    """Reject accidental validation/test reuse before any model selection."""
    unexpected = sorted({str(row.get("split")) for row in rows} - {expected})
    if unexpected:
        raise ValueError(
            f"{purpose} requires only {expected} rows; found {unexpected}"
        )


def g4_train_only_hard_negative_manifest(rows: list[dict]) -> list[str]:
    """Return deterministic train-only hard-negative identities, never predictions."""
    require_split(rows, "train", "G4 hard-negative manifest")
    selected = []
    for row in rows:
        labels = set(
            int(value)
            for value in np.unique(np.load(row["semantic"], allow_pickle=False))
        )
        if row["negative_only"] or 3 in labels or not labels.intersection({1, 2, 3}):
            selected.append(str(row["rgb_path"]))
    return sorted(selected)


def g4_detector_training_weights(rows: list[dict]) -> list[float]:
    """Fixed, train-only hard-negative rebalance declared by the G4 contract."""
    selected = set(g4_train_only_hard_negative_manifest(rows))
    return [
        4.0 if str(row["rgb_path"]) in selected else 1.0
        for row in rows
    ]


def g4_file_sha256(path: Path) -> str:
    return sha256(path)


def g4_validate_dataset(data_root: Path, dataset_evidence: Path) -> dict:
    required = ("g3_dataset_qa.json", "split_manifest.json", "leakage_report.json", "g3_frame_manifest.jsonl")
    missing = [name for name in required if not (dataset_evidence / name).is_file()]
    if missing:
        raise ValueError(f"G4 dataset evidence is incomplete: {missing}")
    if any((dataset_evidence / name).is_symlink() for name in required):
        raise ValueError("G4 dataset evidence may not contain symbolic links")
    qa = json.loads((dataset_evidence / "g3_dataset_qa.json").read_text(encoding="utf-8"))
    split = json.loads((dataset_evidence / "split_manifest.json").read_text(encoding="utf-8"))
    leakage = json.loads((dataset_evidence / "leakage_report.json").read_text(encoding="utf-8"))
    if qa.get("dataset_gate_pass") is not True or split.get("test_used_for_model_selection") is not False:
        raise ValueError("G4 requires a passed QA gate and frozen test split")
    if any(value for value in leakage.values()):
        raise ValueError("G4 rejects non-empty dataset leakage evidence")
    root = data_root.resolve()
    rows = load_rows(data_root, dataset_evidence / "g3_frame_manifest.jsonl")
    if not rows or any(
        not path.is_file() or path.is_symlink() or root not in path.resolve().parents
        for row in rows for path in (row["rgb"], row["depth"], row["semantic"], row["instance"])
    ):
        raise ValueError("G4 frame manifest is missing files or escapes the bound data root")
    return {"data_root": str(root), "files": {name: g4_file_sha256(dataset_evidence / name) for name in required}}


def g4_contract_parameters(contract: Path) -> dict[str, int | float | bool]:
    text = contract.read_text(encoding="utf-8")
    result: dict[str, int | float | bool] = {}
    integer_names = ("detector_epochs", "area_epochs", "detector_batch_size", "area_batch_size", "num_workers")
    float_names = (
        "detector_learning_rate", "area_learning_rate", "weight_decay",
        "scheduler_eta_min_fraction", "detector_giou_weight",
        "detector_quality_weight", "area_tversky_alpha", "area_tversky_beta",
        "area_tversky_gamma",
    )
    for name in integer_names:
        match = re.search(rf"^\s*{name}:\s*(\d+)\s*$", text, re.MULTILINE)
        if not match:
            raise ValueError(f"G4 contract lacks frozen {name}")
        result[name] = int(match.group(1))
    for name in float_names:
        match = re.search(rf"^\s*{name}:\s*([0-9]+(?:\.[0-9]+)?)\s*$", text, re.MULTILINE)
        if not match:
            raise ValueError(f"G4 contract lacks frozen {name}")
        result[name] = float(match.group(1))
    if not re.search(r"^\s*train_only_hard_negative_manifest:\s*true\s*$", text, re.MULTILINE):
        raise ValueError("G4 contract must require the train-only hard-negative manifest")
    result["train_only_hard_negative_manifest"] = True
    return result


def g4_require_runtime_binding(path: Path, implementation_commit: str, contract: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if value.get("status") != "AUTO05_G4_RUNTIME_GATE_BOUND":
        raise ValueError("G4 runtime binding is not bound")
    if value.get("git", {}).get("head") != implementation_commit:
        raise ValueError("G4 implementation commit differs from capture runtime binding")
    if value.get("contract", {}).get("sha256") != g4_file_sha256(contract):
        raise ValueError("G4 contract differs from capture runtime binding")
    formal = value.get("formal_runtime_gate", {})
    session = formal.get("acceptance_session_binding", {})
    closure = formal.get("runtime_closure_binding", {})
    capture = value.get("capture", {})
    if (
        formal.get("status") != "FORMAL_RUNTIME_GATE_BOUND"
        or session.get("session_status_at_gate") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
        or closure.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"
        or not isinstance(capture.get("single_gazebo_lock"), str)
        or not capture.get("single_gazebo_lock")
    ):
        raise ValueError("G4 runtime binding lacks formal closure/session/lock proof")
    return value


def g4_consume_test_lock(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open("x", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    except FileExistsError as exc:
        raise ValueError("G4 frozen test was already consumed") from exc


def g4_reserve_attempt_ledger(path: Path, payload: dict) -> None:
    """Reserve the single preregistered configuration before training starts."""
    try:
        with path.open("x", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    except FileExistsError as exc:
        raise ValueError("G4 preregistered attempt was already reserved") from exc


class G3Dataset(Dataset):
    def __init__(
        self,
        rows: list[dict],
        mode: str,
        augment: bool = False,
        attempt: int = 1,
    ):
        self.rows, self.mode, self.augment = rows, mode, augment
        self.attempt = attempt
        self.epoch = 0

    def set_epoch(self, epoch: int) -> None:
        self.epoch = epoch

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int):
        row = self.rows[index]
        rgb, depth, semantic, instance = read_inputs(row)
        rgb = cv2.resize(rgb, (INPUT_WIDTH, INPUT_HEIGHT), interpolation=cv2.INTER_AREA)
        depth = cv2.resize(
            depth, (INPUT_WIDTH, INPUT_HEIGHT), interpolation=cv2.INTER_NEAREST
        )
        semantic = cv2.resize(
            semantic, (INPUT_WIDTH, INPUT_HEIGHT), interpolation=cv2.INTER_NEAREST
        )
        instance = cv2.resize(
            instance.astype(np.float32),
            (INPUT_WIDTH, INPUT_HEIGHT),
            interpolation=cv2.INTER_NEAREST,
        )
        if self.augment:
            rng = np.random.default_rng(
                SEED + index * 1009 + self.epoch * 7919 + self.attempt * 104729
            )
            if rng.random() < 0.5:
                rgb = np.ascontiguousarray(rgb[:, ::-1])
                depth = np.ascontiguousarray(depth[:, ::-1])
                semantic = np.ascontiguousarray(semantic[:, ::-1])
                instance = np.ascontiguousarray(instance[:, ::-1])
            if self.attempt >= 2:
                angle = float(rng.uniform(-8.0, 8.0))
                scale = float(rng.uniform(0.85, 1.15))
                matrix = cv2.getRotationMatrix2D(
                    (INPUT_WIDTH / 2.0, INPUT_HEIGHT / 2.0), angle, scale
                )
                matrix[:, 2] += [
                    float(rng.uniform(-0.05, 0.05) * INPUT_WIDTH),
                    float(rng.uniform(-0.05, 0.05) * INPUT_HEIGHT),
                ]
                rgb = cv2.warpAffine(
                    rgb,
                    matrix,
                    (INPUT_WIDTH, INPUT_HEIGHT),
                    flags=cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_REFLECT_101,
                )
                depth = cv2.warpAffine(
                    depth,
                    matrix,
                    (INPUT_WIDTH, INPUT_HEIGHT),
                    flags=cv2.INTER_NEAREST,
                    borderMode=cv2.BORDER_CONSTANT,
                    borderValue=0,
                )
                semantic = cv2.warpAffine(
                    semantic,
                    matrix,
                    (INPUT_WIDTH, INPUT_HEIGHT),
                    flags=cv2.INTER_NEAREST,
                    borderMode=cv2.BORDER_CONSTANT,
                    borderValue=0,
                )
                instance = cv2.warpAffine(
                    instance,
                    matrix,
                    (INPUT_WIDTH, INPUT_HEIGHT),
                    flags=cv2.INTER_NEAREST,
                    borderMode=cv2.BORDER_CONSTANT,
                    borderValue=0,
                )
            image = rgb.astype(np.float32) / 255.0
            if self.attempt >= 2:
                hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
                hsv[:, :, 0] = np.mod(
                    hsv[:, :, 0] + float(rng.uniform(-90.0, 90.0)), 360.0
                )
                hsv[:, :, 1] = np.clip(
                    hsv[:, :, 1] * float(rng.uniform(0.25, 1.75)), 0.0, 1.0
                )
                hsv[:, :, 2] = np.clip(
                    hsv[:, :, 2] ** float(rng.uniform(0.60, 1.60)), 0.0, 1.0
                )
                image = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
                image = np.clip(
                    image * rng.uniform(0.55, 1.45, size=(1, 1, 3))
                    + float(rng.uniform(-0.12, 0.12)),
                    0.0,
                    1.0,
                ).astype(np.float32)
                if rng.random() < 0.20:
                    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
                    image = np.repeat(gray[:, :, None], 3, axis=2)
                if rng.random() < 0.35:
                    image = cv2.GaussianBlur(image, (3, 3), 0)
                image = np.clip(
                    image
                    + rng.normal(0.0, rng.uniform(0.0, 0.035), image.shape),
                    0.0,
                    1.0,
                ).astype(np.float32)
            else:
                gain = float(rng.uniform(0.72, 1.28))
                bias = float(rng.uniform(-0.08, 0.08))
                image = np.clip(image * gain + bias, 0.0, 1.0)
                image = np.clip(
                    image * rng.uniform(0.85, 1.15, size=(1, 1, 3)),
                    0.0,
                    1.0,
                )
        else:
            image = rgb.astype(np.float32) / 255.0
        image = np.ascontiguousarray(image, dtype=np.float32)
        if self.mode == "detector":
            boxes = instance_boxes(semantic, instance)
            converted = [
                {
                    "class_index": item["class_index"],
                    "bbox_xyxy": item["bbox_xyxy"],
                }
                for item in boxes
            ]
            target = encode_centernet_targets(
                converted,
                input_width=INPUT_WIDTH,
                input_height=INPUT_HEIGHT,
                stride=STRIDE,
                class_count=3,
            )
            return (
                torch.from_numpy(
                    np.ascontiguousarray(
                        build_model_input(
                            image, depth, self.mode, self.attempt
                        ).transpose(2, 0, 1),
                        dtype=np.float32,
                    )
                ),
                torch.from_numpy(target["heatmap"]),
                torch.from_numpy(target["offset"]),
                torch.from_numpy(target["size"]),
                torch.from_numpy(target["regression_mask"]),
            )
        if self.mode == "area":
            inputs = build_model_input(image, depth, self.mode, self.attempt)
            target = np.stack((semantic == 4, semantic == 5)).astype(np.float32)
            return (
                torch.from_numpy(
                    np.ascontiguousarray(inputs.transpose(2, 0, 1), dtype=np.float32)
                ),
                torch.from_numpy(target),
            )
        raise ValueError(self.mode)


class DirectDetector(nn.Module):
    def __init__(self, width: int = 48, input_channels: int = 3):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(input_channels, width, 5, stride=2, padding=2),
            nn.BatchNorm2d(width),
            nn.SiLU(),
            nn.Conv2d(width, width * 2, 3, stride=2, padding=1),
            nn.BatchNorm2d(width * 2),
            nn.SiLU(),
            nn.Conv2d(width * 2, width * 2, 3, padding=1),
            nn.SiLU(),
            nn.Conv2d(width * 2, width * 2, 3, padding=2, dilation=2),
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


class G4DirectDetector(nn.Module):
    """Small stride-four FPN which still emits direct center/offset/size boxes."""

    def __init__(self, width: int = 64, input_channels: int = 6):
        super().__init__()
        self.p2 = nn.Sequential(
            nn.Conv2d(input_channels, width, 5, stride=2, padding=2),
            nn.BatchNorm2d(width),
            nn.SiLU(),
        )
        self.p3 = nn.Sequential(
            nn.Conv2d(width, width * 2, 3, stride=2, padding=1),
            nn.BatchNorm2d(width * 2),
            nn.SiLU(),
            nn.Conv2d(width * 2, width * 2, 3, padding=1),
            nn.SiLU(),
        )
        self.lateral = nn.Conv2d(width, width * 2, 1)
        self.fuse = nn.Sequential(
            nn.Conv2d(width * 2, width * 2, 3, padding=1), nn.SiLU(),
            nn.Conv2d(width * 2, width * 2, 3, padding=2, dilation=2), nn.SiLU(),
        )
        self.heatmap = nn.Conv2d(width * 2, 3, 1)
        self.offset = nn.Conv2d(width * 2, 2, 1)
        self.size = nn.Sequential(nn.Conv2d(width * 2, 2, 1), nn.Softplus())
        self.quality = nn.Conv2d(width * 2, 1, 1)
        nn.init.constant_(self.heatmap.bias, -2.19)

    def forward(self, image):
        import torch.nn.functional as functional

        p2 = self.p2(image)
        p3 = self.p3(p2)
        fused = self.fuse(p3 + functional.interpolate(
            self.lateral(p2), size=p3.shape[-2:], mode="bilinear", align_corners=False
        ))
        return self.heatmap(fused), self.offset(fused), self.size(fused), self.quality(fused)


class RGBDAreaUNet(nn.Module):
    def __init__(self, width: int = 32, input_channels: int = 4):
        super().__init__()
        self.enc1 = nn.Sequential(
            nn.Conv2d(input_channels, width, 3, padding=1),
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
        self.enc3 = nn.Sequential(
            nn.Conv2d(width * 2, width * 3, 3, padding=1),
            nn.SiLU(),
            nn.Conv2d(width * 3, width * 3, 3, padding=1),
            nn.SiLU(),
        )
        self.decode2 = nn.Sequential(
            nn.Conv2d(width * 5, width * 2, 3, padding=1), nn.SiLU()
        )
        self.decode1 = nn.Sequential(
            nn.Conv2d(width * 3, width, 3, padding=1),
            nn.SiLU(),
            nn.Conv2d(width, 2, 1),
        )

    def forward(self, image):
        import torch.nn.functional as functional

        first = self.enc1(image)
        second = self.enc2(functional.max_pool2d(first, 2))
        third = self.enc3(functional.max_pool2d(second, 2))
        up_second = functional.interpolate(
            third, size=second.shape[-2:], mode="bilinear", align_corners=False
        )
        decoded_second = self.decode2(torch.cat((second, up_second), dim=1))
        up_first = functional.interpolate(
            decoded_second, size=first.shape[-2:], mode="bilinear", align_corners=False
        )
        return self.decode1(torch.cat((first, up_first), dim=1))


class RGBDAreaBinaryUNet(RGBDAreaUNet):
    """One independently parameterized area head; no leaf/puddle sharing."""

    def __init__(self, width: int = 40, input_channels: int = 7):
        super().__init__(width=width, input_channels=input_channels)
        self.decode1[-1] = nn.Conv2d(width, 1, 1)


class G4IndependentAreaHeads(nn.Module):
    def __init__(self, width: int = 40, input_channels: int = 7):
        super().__init__()
        self.leaf = RGBDAreaBinaryUNet(width=width, input_channels=input_channels)
        self.puddle = RGBDAreaBinaryUNet(width=width, input_channels=input_channels)

    def forward(self, image):
        return torch.cat((self.leaf(image), self.puddle(image)), dim=1)


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
    return (positive_loss.sum() + negative_loss.sum()) / positive.sum().clamp(min=1.0)


def g4_giou_loss(predicted_offset, predicted_size, offset, size, mask):
    """GIoU on direct-head cells; masked empty frames are finite by construction."""
    height, width = predicted_offset.shape[-2:]
    yy, xx = torch.meshgrid(
        torch.arange(height, device=predicted_offset.device),
        torch.arange(width, device=predicted_offset.device),
        indexing="ij",
    )
    center = torch.stack((xx, yy), dim=0).float()[None]
    def corners(delta, extent):
        middle = center + delta
        half = extent.clamp(min=1e-6) * 0.5
        return middle - half, middle + half
    p0, p1 = corners(predicted_offset, predicted_size)
    t0, t1 = corners(offset, size)
    intersection = (torch.minimum(p1, t1) - torch.maximum(p0, t0)).clamp(min=0)
    inter = intersection[:, 0] * intersection[:, 1]
    p_area = ((p1 - p0).clamp(min=0)).prod(dim=1)
    t_area = ((t1 - t0).clamp(min=0)).prod(dim=1)
    union = p_area + t_area - inter
    enclosing = (torch.maximum(p1, t1) - torch.minimum(p0, t0)).clamp(min=0)
    enclosing_area = enclosing.prod(dim=1).clamp(min=1e-6)
    iou = inter / union.clamp(min=1e-6)
    giou = iou - (enclosing_area - union) / enclosing_area
    return ((1.0 - giou) * mask[:, 0]).sum() / mask.sum().clamp(min=1.0)


def g4_area_loss(logits, target):
    """Fixed BCE plus focal-Tversky, separately evaluated for each binary head."""
    if G4_FROZEN_TRAINING is None:
        raise ValueError("G4 loss requires a parsed frozen training contract")
    probability = torch.sigmoid(logits)
    bce = torch.nn.functional.binary_cross_entropy_with_logits(logits, target)
    tp = (probability * target).sum((0, 2, 3))
    fp = (probability * (1.0 - target)).sum((0, 2, 3))
    fn = ((1.0 - probability) * target).sum((0, 2, 3))
    tversky = (tp + 1.0) / (
        tp + G4_FROZEN_TRAINING["area_tversky_alpha"] * fp
        + G4_FROZEN_TRAINING["area_tversky_beta"] * fn + 1.0
    )
    return bce + (1.0 - tversky).pow(G4_FROZEN_TRAINING["area_tversky_gamma"]).mean()


def train_detector(rows: list[dict], device, epochs: int, attempt: int):
    require_split(rows, "train", "detector training")
    dataset = G3Dataset(rows, "detector", augment=True, attempt=attempt)
    sampler = None
    if attempt >= 3:
        sampler = WeightedRandomSampler(
            g4_detector_training_weights(rows) if attempt >= 4 else training_row_weights(rows, "detector"),
            num_samples=len(rows) * 2,
            replacement=True,
            generator=torch.Generator().manual_seed(SEED + 3),
        )
    loader = DataLoader(
        dataset,
        batch_size=(G4_FROZEN_TRAINING["detector_batch_size"] if attempt >= 4 else 8),
        shuffle=sampler is None,
        sampler=sampler,
        num_workers=(G4_FROZEN_TRAINING["num_workers"] if attempt >= 4 else 2),
        pin_memory=True,
        generator=torch.Generator().manual_seed(SEED),
    )
    model = (
        G4DirectDetector(width=64, input_channels=6)
        if attempt >= 4
        else DirectDetector(
            width=64 if attempt >= 3 else 48,
            input_channels=6 if attempt >= 3 else 3,
        )
    ).to(device)
    learning_rate = (
        G4_FROZEN_TRAINING["detector_learning_rate"] if attempt >= 4
        else 5e-4 if attempt >= 2 else 1.5e-3
    )
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=learning_rate,
        weight_decay=(G4_FROZEN_TRAINING["weight_decay"] if attempt >= 4 else 1e-5),
    )
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=max(epochs, 1), eta_min=learning_rate * (
            G4_FROZEN_TRAINING["scheduler_eta_min_fraction"] if attempt >= 4 else 0.05
        )
    )
    curve, started = [], time.perf_counter()
    for epoch in range(1, epochs + 1):
        dataset.set_epoch(epoch)
        model.train()
        losses = []
        for image, heatmap, offset, size, mask in loader:
            image, heatmap = image.to(device), heatmap.to(device)
            offset, size, mask = offset.to(device), size.to(device), mask.to(device)
            optimizer.zero_grad(set_to_none=True)
            outputs = model(image)
            predicted_heatmap, predicted_offset, predicted_size = outputs[:3]
            denominator = mask.sum().clamp(min=1.0)
            loss = (
                focal_heatmap_loss(predicted_heatmap, heatmap)
                + (torch.abs(predicted_offset - offset) * mask).sum() / denominator
                + 0.2
                * (torch.abs(predicted_size - size) * mask).sum()
                / denominator
            )
            if attempt >= 4:
                quality = outputs[3]
                quality_target = mask
                loss = loss + G4_FROZEN_TRAINING["detector_giou_weight"] * g4_giou_loss(
                    predicted_offset, predicted_size, offset, size, mask
                ) + G4_FROZEN_TRAINING["detector_quality_weight"] * torch.nn.functional.binary_cross_entropy_with_logits(
                    quality, quality_target
                )
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=5.0)
            optimizer.step()
            losses.append(float(loss.detach()))
        scheduler.step()
        if epoch == 1 or epoch % 5 == 0 or epoch == epochs:
            curve.append(
                {
                    "epoch": epoch,
                    "loss": float(np.mean(losses)),
                    "learning_rate": optimizer.param_groups[0]["lr"],
                }
            )
    return model, {
        "epochs": epochs,
        "curve": curve,
        "augmentation_profile": (
            "g4_train_only_hard_negative_rebalance_plus_quality_giou"
            if attempt >= 4
            else
            "epoch_varying_color_geometry_plus_hard_negative_rebalance"
            if attempt >= 3
            else "epoch_varying_color_geometry_distribution_repair"
            if attempt == 2
            else "baseline_fixed_light_augmentation"
        ),
        "weighted_samples_per_epoch": len(loader.sampler)
        if sampler is not None
        else len(rows),
        "duration_s": time.perf_counter() - started,
        "g4_train_only_hard_negative_manifest": (
            g4_train_only_hard_negative_manifest(rows) if attempt >= 4 else None
        ),
    }


def detector_raw_predictions(
    model, rows: list[dict], device, attempt: int
) -> tuple[list[list], list[list]]:
    model.eval()
    predictions, truths = [], []
    with torch.no_grad():
        for row in rows:
            rgb, depth, semantic, instance = read_inputs(row)
            truth = instance_boxes(semantic, instance)
            image = cv2.resize(
                rgb, (INPUT_WIDTH, INPUT_HEIGHT), interpolation=cv2.INTER_AREA
            ).astype(np.float32) / 255.0
            resized_depth = cv2.resize(
                depth,
                (INPUT_WIDTH, INPUT_HEIGHT),
                interpolation=cv2.INTER_NEAREST,
            )
            inputs = build_model_input(
                image, resized_depth, "detector", attempt
            )
            tensor = torch.from_numpy(
                np.ascontiguousarray(
                    inputs.transpose(2, 0, 1)[None], dtype=np.float32
                )
            ).to(device)
            outputs = model(tensor)
            heatmap, offset, size = outputs[:3]
            scores = torch.sigmoid(heatmap)[0]
            if attempt >= 4:
                scores = scores * torch.sigmoid(outputs[3])[0]
            decoded = decode_centernet_outputs(
                scores.cpu().numpy(),
                offset[0].cpu().numpy(),
                size[0].cpu().numpy(),
                stride=STRIDE,
                score_threshold=0.01,
                nms_iou_threshold=0.35 if attempt >= 3 else 0.5,
                local_maximum_radius=2 if attempt >= 3 else 1,
                max_detections=60 if attempt >= 3 else 100,
            )
            scaled_truth = [
                {
                    **item,
                    "bbox_xyxy": [
                        item["bbox_xyxy"][0] * INPUT_WIDTH / 640.0,
                        item["bbox_xyxy"][1] * INPUT_HEIGHT / 480.0,
                        item["bbox_xyxy"][2] * INPUT_WIDTH / 640.0,
                        item["bbox_xyxy"][3] * INPUT_HEIGHT / 480.0,
                    ],
                }
                for item in truth
            ]
            predictions.append(decoded)
            truths.append(scaled_truth)
    return predictions, truths


def detector_metrics(
    rows: list[dict],
    predictions: list[list],
    truths: list[list],
    threshold: float,
) -> dict:
    tp = [0, 0, 0]
    fp = [0, 0, 0]
    fn = [0, 0, 0]
    small_tp = 0
    small_total = 0
    discovery_matched = 0
    discovery_total = 0
    discovery_false = 0
    negative_false = 0
    evaluable_count = 0
    for row, frame_predictions, frame_truth in zip(rows, predictions, truths):
        selected = [item for item in frame_predictions if item.score >= threshold]
        ready_truth = [
            item
            for item in frame_truth
            if item["short_side"] >= 8 and item["mask_area"] >= 20
        ]
        evaluable_count += len(ready_truth)
        used_predictions = set()
        for class_index in range(3):
            class_truth = [
                item for item in ready_truth if item["class_index"] == class_index
            ]
            class_predictions = [
                (index, item)
                for index, item in enumerate(selected)
                if item.class_index == class_index
            ]
            matched_truth = set()
            for prediction_index, prediction in class_predictions:
                overlaps = [
                    box_iou(tuple(prediction.bbox_xyxy), tuple(item["bbox_xyxy"]))
                    if index not in matched_truth
                    else -1.0
                    for index, item in enumerate(class_truth)
                ]
                best = int(np.argmax(overlaps)) if overlaps else -1
                if best >= 0 and overlaps[best] >= 0.5:
                    tp[class_index] += 1
                    matched_truth.add(best)
                    used_predictions.add(prediction_index)
                    if class_truth[best]["short_side"] < 18:
                        small_tp += 1
                else:
                    fp[class_index] += 1
            fn[class_index] += len(class_truth) - len(matched_truth)
            small_total += sum(item["short_side"] < 18 for item in class_truth)
        visible_truth = [item for item in frame_truth if item["mask_area"] >= 4]
        discovery_total += len(visible_truth)
        matched_visible = set()
        matched_candidates = set()
        for prediction_index, prediction in enumerate(selected):
            overlaps = [
                box_iou(tuple(prediction.bbox_xyxy), tuple(item["bbox_xyxy"]))
                if index not in matched_visible
                else -1.0
                for index, item in enumerate(visible_truth)
            ]
            best = int(np.argmax(overlaps)) if overlaps else -1
            if best >= 0 and overlaps[best] >= 0.3:
                matched_visible.add(best)
                matched_candidates.add(prediction_index)
        discovery_matched += len(matched_visible)
        discovery_false += len(selected) - len(matched_candidates)
        if row["negative_only"]:
            negative_false += len(selected)
    precision = [tp[i] / max(tp[i] + fp[i], 1) for i in range(3)]
    recall = [tp[i] / max(tp[i] + fn[i], 1) for i in range(3)]
    f1 = [
        2 * precision[i] * recall[i] / max(precision[i] + recall[i], 1e-12)
        for i in range(3)
    ]
    duration_minutes = len(rows) * 8.0 / 60.0
    negative_frames = sum(row["negative_only"] for row in rows)
    return {
        "threshold": threshold,
        "machine_evaluable_object_count": evaluable_count,
        "precision_by_class": dict(zip(DISCRETE_NAMES, precision)),
        "recall_by_class": dict(zip(DISCRETE_NAMES, recall)),
        "f1_by_class": dict(zip(DISCRETE_NAMES, f1)),
        "macro_precision": float(np.mean(precision)),
        "macro_recall": float(np.mean(recall)),
        "macro_f1": float(np.mean(f1)),
        "small_object_recall": small_tp / max(small_total, 1),
        "small_object_count": small_total,
        "negative_only_frames": negative_frames,
        "negative_only_false_positive_per_frame": negative_false
        / max(negative_frames, 1),
        "all_visible_candidate_recall": discovery_matched
        / max(discovery_total, 1),
        "all_visible_object_count": discovery_total,
        "false_candidates_per_min": discovery_false / max(duration_minutes, 1e-12),
    }


def select_detector_threshold(rows, predictions, truths) -> tuple[float, list[dict]]:
    require_split(rows, "val", "detector threshold selection")
    candidates = [
        detector_metrics(rows, predictions, truths, threshold)
        for threshold in (
            0.05,
            0.08,
            0.10,
            0.12,
            0.15,
            0.18,
            0.20,
            0.25,
            0.30,
            0.35,
            0.40,
            0.50,
            0.60,
            0.70,
            0.80,
            0.90,
        )
    ]
    feasible = [
        item
        for item in candidates
        if item["negative_only_false_positive_per_frame"] <= 0.05
        and item["false_candidates_per_min"] <= 2.0
    ]
    selected = max(
        feasible or candidates,
        key=lambda item: (
            item["macro_f1"],
            item["all_visible_candidate_recall"],
            item["threshold"],
        ),
    )
    return selected["threshold"], candidates


def train_area(rows: list[dict], device, epochs: int, attempt: int):
    require_split(rows, "train", "area training")
    dataset = G3Dataset(rows, "area", augment=True, attempt=attempt)
    sampler = None
    if attempt >= 3:
        sampler = WeightedRandomSampler(
            training_row_weights(rows, "area"),
            num_samples=len(rows) * 2,
            replacement=True,
            generator=torch.Generator().manual_seed(SEED + 5),
        )
    loader = DataLoader(
        dataset,
        batch_size=(G4_FROZEN_TRAINING["area_batch_size"] if attempt >= 4 else 2 if attempt >= 3 else 4),
        shuffle=sampler is None,
        sampler=sampler,
        num_workers=(G4_FROZEN_TRAINING["num_workers"] if attempt >= 4 else 2),
        pin_memory=True,
        generator=torch.Generator().manual_seed(SEED),
    )
    model = (
        G4IndependentAreaHeads(width=40, input_channels=7)
        if attempt >= 4
        else RGBDAreaUNet(
            width=40 if attempt >= 3 else 32,
            input_channels=7 if attempt >= 3 else 4,
        )
    ).to(device)
    learning_rate = (
        G4_FROZEN_TRAINING["area_learning_rate"] if attempt >= 4
        else 4e-4 if attempt >= 2 else 1e-3
    )
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=learning_rate,
        weight_decay=(G4_FROZEN_TRAINING["weight_decay"] if attempt >= 4 else 1e-5),
    )
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=max(epochs, 1), eta_min=learning_rate * (
            G4_FROZEN_TRAINING["scheduler_eta_min_fraction"] if attempt >= 4 else 0.05
        )
    )
    positive_counts = torch.zeros(2)
    pixel_count = 0
    for _, target in DataLoader(
        G3Dataset(rows, "area", augment=False),
        batch_size=(G4_FROZEN_TRAINING["area_batch_size"] if attempt >= 4 else 2),
        num_workers=(G4_FROZEN_TRAINING["num_workers"] if attempt >= 4 else 2),
    ):
        positive_counts += target.sum((0, 2, 3))
        pixel_count += int(target.shape[0] * target.shape[2] * target.shape[3])
    positive_weight = torch.clamp(
        (pixel_count - positive_counts) / positive_counts.clamp(min=1),
        min=1.0,
        max=80.0,
    ).to(device)
    curve, started = [], time.perf_counter()
    for epoch in range(1, epochs + 1):
        dataset.set_epoch(epoch)
        model.train()
        losses = []
        for image, target in loader:
            image, target = image.to(device), target.to(device)
            optimizer.zero_grad(set_to_none=True)
            logits = model(image)
            if attempt >= 4:
                loss = g4_area_loss(logits, target)
            else:
                binary = torch.nn.functional.binary_cross_entropy_with_logits(
                    logits, target, pos_weight=positive_weight.view(1, 2, 1, 1)
                )
                probability = torch.sigmoid(logits)
                intersection = (probability * target).sum((0, 2, 3))
                denominator = probability.sum((0, 2, 3)) + target.sum((0, 2, 3))
                dice = 1.0 - (
                    (2.0 * intersection + 1.0) / (denominator + 1.0)
                ).mean()
                loss = binary + dice + 0.2 * probability[target == 0].pow(2).mean()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=5.0)
            optimizer.step()
            losses.append(float(loss.detach()))
        scheduler.step()
        if epoch == 1 or epoch % 5 == 0 or epoch == epochs:
            curve.append(
                {
                    "epoch": epoch,
                    "loss": float(np.mean(losses)),
                    "learning_rate": optimizer.param_groups[0]["lr"],
                }
            )
    return model, {
        "epochs": epochs,
        "curve": curve,
        "positive_weights": positive_weight.cpu().tolist(),
        "augmentation_profile": (
            "g4_independent_binary_heads_plus_focal_tversky"
            if attempt >= 4
            else
            "epoch_varying_color_geometry_plus_class_rebalance"
            if attempt >= 3
            else "epoch_varying_color_geometry_distribution_repair"
            if attempt == 2
            else "baseline_fixed_light_augmentation"
        ),
        "weighted_samples_per_epoch": len(loader.sampler)
        if sampler is not None
        else len(rows),
        "duration_s": time.perf_counter() - started,
    }


def area_probabilities(
    model, rows: list[dict], device, attempt: int
) -> list[np.ndarray]:
    model.eval()
    probabilities = []
    with torch.no_grad():
        for image, _ in DataLoader(
            G3Dataset(rows, "area", augment=False, attempt=attempt),
            batch_size=1,
            num_workers=2,
        ):
            probabilities.append(torch.sigmoid(model(image.to(device)))[0].cpu().numpy())
    return probabilities


def area_metrics(rows, probabilities, thresholds) -> dict:
    intersections = np.zeros(2, dtype=np.int64)
    unions = np.zeros(2, dtype=np.int64)
    negative_false = 0
    minimum_area = 20
    for row, probability in zip(rows, probabilities):
        semantic = cv2.resize(
            np.load(row["semantic"], allow_pickle=False),
            (INPUT_WIDTH, INPUT_HEIGHT),
            interpolation=cv2.INTER_NEAREST,
        )
        truth = np.stack((semantic == 4, semantic == 5))
        predicted = probability >= np.asarray(thresholds)[:, None, None]
        intersections += (predicted & truth).sum((1, 2))
        unions += (predicted | truth).sum((1, 2))
        if row["negative_only"]:
            false_candidate = False
            for channel in range(2):
                count, _, stats, _ = cv2.connectedComponentsWithStats(
                    predicted[channel].astype(np.uint8), 8
                )
                if count > 1 and int(stats[1:, cv2.CC_STAT_AREA].max()) >= minimum_area:
                    false_candidate = True
            negative_false += int(false_candidate)
    iou = [
        float(intersection / max(union, 1))
        for intersection, union in zip(intersections, unions)
    ]
    negative_count = sum(row["negative_only"] for row in rows)
    return {
        "thresholds": dict(zip(AREA_NAMES, thresholds)),
        "iou_by_class": dict(zip(AREA_NAMES, iou)),
        "macro_miou": float(np.mean(iou)),
        "negative_only_frames": negative_count,
        "negative_area_false_positive_per_frame": negative_false
        / max(negative_count, 1),
        "intersection_pixels": intersections.tolist(),
        "union_pixels": unions.tolist(),
        "minimum_candidate_area_px": minimum_area,
    }


def select_area_thresholds(rows, probabilities):
    require_split(rows, "val", "area threshold selection")
    traces = {}
    selected = []
    for channel, name in enumerate(AREA_NAMES):
        candidates = []
        for threshold in (
            0.30,
            0.35,
            0.40,
            0.45,
            0.50,
            0.55,
            0.60,
            0.65,
            0.70,
            0.75,
            0.80,
            0.85,
            0.90,
        ):
            intersections = unions = 0
            for row, probability in zip(rows, probabilities):
                semantic = cv2.resize(
                    np.load(row["semantic"], allow_pickle=False),
                    (INPUT_WIDTH, INPUT_HEIGHT),
                    interpolation=cv2.INTER_NEAREST,
                )
                truth = semantic == (4 + channel)
                predicted = probability[channel] >= threshold
                intersections += int((predicted & truth).sum())
                unions += int((predicted | truth).sum())
            candidates.append(
                {"threshold": threshold, "iou": intersections / max(unions, 1)}
            )
        best = max(candidates, key=lambda item: (item["iou"], item["threshold"]))
        selected.append(best["threshold"])
        traces[name] = candidates
    return selected, traces


def export_and_compare(model, shape, path: Path) -> dict:
    import onnx
    import onnxruntime as ort

    model = model.cpu().eval()
    sample = torch.rand(shape, generator=torch.Generator().manual_seed(SEED))
    with torch.no_grad():
        expected = model(sample)
    outputs = expected if isinstance(expected, tuple) else (expected,)
    output_names = [f"output_{index}" for index in range(len(outputs))]
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
    operators = defaultdict(int)
    for node in graph.graph.node:
        operators[node.op_type] += 1
    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    actual = session.run(None, {"images": sample.numpy()})
    max_error = max(
        float(np.max(np.abs(left.detach().numpy() - right)))
        for left, right in zip(outputs, actual)
    )
    return {
        "path": path.name,
        "sha256": sha256(path),
        "bytes": path.stat().st_size,
        "fixed_input": list(shape),
        "custom_ops": 0,
        "operator_inventory": dict(sorted(operators.items())),
        "max_numeric_output_error": max_error,
    }


def main() -> int:
    global INPUT_WIDTH, INPUT_HEIGHT, G4_FROZEN_TRAINING
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True)
    parser.add_argument("--dataset-evidence", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--implementation-commit", required=True)
    parser.add_argument("--attempt", type=int, default=1)
    parser.add_argument("--detector-epochs", type=int)
    parser.add_argument("--area-epochs", type=int)
    parser.add_argument("--g4-contract")
    parser.add_argument("--g4-runtime-binding")
    parser.add_argument("--g4-attempt-ledger")
    parser.add_argument("--g4-test-lock")
    args = parser.parse_args()
    contract = None
    g4_parameters = None
    if args.attempt >= 4:
        if not all((args.g4_contract, args.g4_runtime_binding, args.g4_attempt_ledger, args.g4_test_lock)):
            parser.error("G4 requires --g4-contract; blind attempt 4 is forbidden")
        contract = Path(args.g4_contract)
        required = (
            "direct_anchor_free_center_offset_bbox",
            "test_used_for_model_selection: false",
            "maximum_configs_per_architecture: 1",
        )
        if contract.name != G4_CONTRACT_NAME or not contract.is_file() or any(
            item not in contract.read_text(encoding="utf-8") for item in required
        ):
            parser.error("G4 contract is incomplete or permits test-driven selection")
        g4_parameters = g4_contract_parameters(contract)
        if args.detector_epochs not in (None, g4_parameters["detector_epochs"]):
            parser.error("G4 detector epochs are frozen by its contract")
        if args.area_epochs not in (None, g4_parameters["area_epochs"]):
            parser.error("G4 area epochs are frozen by its contract")
        args.detector_epochs = g4_parameters["detector_epochs"]
        args.area_epochs = g4_parameters["area_epochs"]
        G4_FROZEN_TRAINING = g4_parameters
    else:
        args.detector_epochs = args.detector_epochs or 45
        args.area_epochs = args.area_epochs or 35
    if args.attempt >= 3:
        INPUT_WIDTH, INPUT_HEIGHT = 512, 384
    random.seed(SEED)
    np.random.seed(SEED)
    torch.manual_seed(SEED)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(SEED)
        torch.backends.cudnn.deterministic = True
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    data_root, dataset_evidence, output = (
        Path(args.data_root),
        Path(args.dataset_evidence),
        Path(args.output),
    )
    if args.attempt >= 4:
        work_root = ROOT / ".work" / "auto05-g4"
        expected_paths = {
            "data root": work_root / "data" / "g3_screening_native",
            "dataset evidence": work_root / "evidence" / "dataset",
            "output": work_root / "evidence" / "screening",
            "runtime binding": work_root / "evidence" / "runtime_gate_binding.json",
            "attempt ledger": work_root / "evidence" / "g4_attempt_ledger.json",
            "test lock": work_root / "evidence" / "g4_test_consumed_lock.json",
        }
        actual_paths = {
            "data root": data_root, "dataset evidence": dataset_evidence,
            "output": output, "runtime binding": Path(args.g4_runtime_binding),
            "attempt ledger": Path(args.g4_attempt_ledger),
            "test lock": Path(args.g4_test_lock),
        }
        if output.exists() or any(
            actual_paths[name].resolve() != expected.resolve()
            for name, expected in expected_paths.items()
        ):
            parser.error("G4 requires fresh, canonical paths below TZcup/.work/auto05-g4")
        if Path(args.g4_runtime_binding).is_symlink() or contract.is_symlink():
            parser.error("G4 refuses symbolic-link contract or runtime binding")
        try:
            head = subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip()
        except subprocess.CalledProcessError as exc:
            parser.error(f"cannot resolve G4 implementation commit: {exc}")
        if args.implementation_commit != head:
            parser.error("implementation_commit must equal the current Git HEAD")
        try:
            g4_runtime_binding = g4_require_runtime_binding(
                Path(args.g4_runtime_binding), head, contract
            )
            g4_dataset_binding = g4_validate_dataset(data_root, dataset_evidence)
        except (OSError, UnicodeError, ValueError, json.JSONDecodeError) as exc:
            parser.error(str(exc))
    else:
        g4_runtime_binding = None
        g4_dataset_binding = None
    output.mkdir(parents=True, exist_ok=True)
    if args.attempt >= 4:
        tree = subprocess.check_output(
            ["git", "-C", str(ROOT), "rev-parse", "HEAD^{tree}"], text=True
        ).strip()
        g4_reserve_attempt_ledger(
            Path(args.g4_attempt_ledger),
            {
                "schema_version": 1, "status": "G4_ATTEMPT_RESERVED",
                "implementation_commit": args.implementation_commit,
                "implementation_tree": tree,
                "contract_sha256": g4_file_sha256(contract),
                "dataset_binding": g4_dataset_binding,
                "runtime_binding_sha256": g4_file_sha256(Path(args.g4_runtime_binding)),
                "configuration_count": 1,
                "test_runs_allowed": 1,
            },
        )
    rows = load_rows(data_root, dataset_evidence / "g3_frame_manifest.jsonl")
    train_rows = [
        row
        for row in rows
        if row["split"] == "train" and row["scene_seed"] % 15 < 12
    ]
    in_domain_rows = [
        row
        for row in rows
        if row["split"] == "train" and row["scene_seed"] % 15 >= 12
    ]
    val_rows = [row for row in rows if row["split"] == "val"]
    test_rows = [row for row in rows if row["split"] == "test"]

    detector, detector_training = train_detector(
        train_rows, device, args.detector_epochs, args.attempt
    )
    detector_checkpoint = output / "auto05_direct_detector.pt"
    torch.save(detector.state_dict(), detector_checkpoint)
    val_predictions, val_truth = detector_raw_predictions(
        detector, val_rows, device, args.attempt
    )
    threshold, detector_calibration = select_detector_threshold(
        val_rows, val_predictions, val_truth
    )
    if args.attempt >= 4:
        g4_consume_test_lock(
            Path(args.g4_test_lock),
            {
                "schema_version": 1,
                "status": "G4_TEST_CONSUMED",
                "implementation_commit": args.implementation_commit,
                "contract_sha256": g4_file_sha256(contract),
                "dataset_binding": g4_dataset_binding,
                "runtime_binding_sha256": g4_file_sha256(Path(args.g4_runtime_binding)),
                "attempt_ledger_sha256": g4_file_sha256(Path(args.g4_attempt_ledger)),
                "output": str(output.resolve()),
            },
        )
    detector_results = {}
    for name, split_rows in (
        ("in_domain", in_domain_rows),
        ("validation_cross_world", val_rows),
        ("test_cross_world", test_rows),
    ):
        predictions, truths = (
            (val_predictions, val_truth)
            if name == "validation_cross_world"
            else detector_raw_predictions(
                detector, split_rows, device, args.attempt
            )
        )
        detector_results[name] = detector_metrics(
            split_rows, predictions, truths, threshold
        )
    detector_export = export_and_compare(
        detector,
        (1, 6 if args.attempt >= 3 else 3, INPUT_HEIGHT, INPUT_WIDTH),
        output / "auto05_direct_detector.onnx",
    )

    area, area_training = train_area(
        train_rows, device, args.area_epochs, args.attempt
    )
    area_checkpoint = output / "auto05_rgbd_area_segmenter.pt"
    torch.save(area.state_dict(), area_checkpoint)
    val_area_probabilities = area_probabilities(
        area, val_rows, device, args.attempt
    )
    area_thresholds, area_calibration = select_area_thresholds(
        val_rows, val_area_probabilities
    )
    area_results = {
        "validation_cross_world": area_metrics(
            val_rows, val_area_probabilities, area_thresholds
        ),
        "test_cross_world": area_metrics(
            test_rows,
            area_probabilities(area, test_rows, device, args.attempt),
            area_thresholds,
        ),
    }
    area_export = export_and_compare(
        area,
        (1, 7 if args.attempt >= 3 else 4, INPUT_HEIGHT, INPUT_WIDTH),
        output / "auto05_rgbd_area_segmenter.onnx",
    )
    val_detector = detector_results["validation_cross_world"]
    test_detector = detector_results["test_cross_world"]
    val_area = area_results["validation_cross_world"]
    test_area = area_results["test_cross_world"]
    gates = {
        "discovery_all_visible_candidate_recall_at_least_0_80": min(
            val_detector["all_visible_candidate_recall"],
            test_detector["all_visible_candidate_recall"],
        )
        >= 0.80,
        "discovery_false_candidates_per_min_at_most_2": max(
            val_detector["false_candidates_per_min"],
            test_detector["false_candidates_per_min"],
        )
        <= 2.0,
        "in_domain_macro_f1_at_least_0_90": detector_results["in_domain"][
            "macro_f1"
        ]
        >= 0.90,
        "cross_world_macro_f1_at_least_0_70": min(
            val_detector["macro_f1"], test_detector["macro_f1"]
        )
        >= 0.70,
        "small_object_recall_at_least_0_70": min(
            val_detector["small_object_recall"],
            test_detector["small_object_recall"],
        )
        >= 0.70,
        "negative_only_fp_per_frame_at_most_0_05": max(
            val_detector["negative_only_false_positive_per_frame"],
            test_detector["negative_only_false_positive_per_frame"],
        )
        <= 0.05,
        "cross_world_leaf_iou_at_least_0_75": min(
            val_area["iou_by_class"]["leaf_pile"],
            test_area["iou_by_class"]["leaf_pile"],
        )
        >= 0.75,
        "cross_world_puddle_iou_at_least_0_75": min(
            val_area["iou_by_class"]["puddle"],
            test_area["iou_by_class"]["puddle"],
        )
        >= 0.75,
        "cross_world_macro_miou_at_least_0_75": min(
            val_area["macro_miou"], test_area["macro_miou"]
        )
        >= 0.75,
        "color_material_stress_macro_f1_at_least_0_60": test_detector[
            "macro_f1"
        ]
        >= 0.60,
        "same_color_negative_specificity_at_least_0_95": (
            1.0 - test_detector["negative_only_false_positive_per_frame"]
        )
        >= 0.95,
        "onnx_detector_error_at_most_1e_4": detector_export[
            "max_numeric_output_error"
        ]
        <= 1e-4,
        "onnx_area_error_at_most_1e_4": area_export["max_numeric_output_error"]
        <= 1e-4,
    }
    report = {
        "schema_version": 1,
        "stage": "AUTO-05",
        "attempt_id": f"AUTO-05-G3-SCREENING-V{args.attempt}",
        "hypothesis": (
            "pre-registered G4 direct anchor-free FPN with quality/GIoU and "
            "independent RGB-D area heads; no test-driven selection"
            if args.attempt >= 4
            else
            "hard-negative and class-balanced sampling plus 512x384 RGB-D, "
            "edge, and local-contrast inputs can improve precision and "
            "leaf/generalization gates left unresolved by attempt 2"
            if args.attempt >= 3
            else
            "epoch-varying color, lighting, and affine distribution repair plus "
            "stabilized optimization can reduce the asset/world shift observed "
            "in attempt 1 without using test data for selection"
            if args.attempt == 2
            else "deployment-aligned G3 world isolation plus direct center detector and RGB-D binary area heads can generalize beyond the micro train set"
        ),
        "implementation_commit": args.implementation_commit,
        "implementation_tree": (
            subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD^{tree}"], text=True).strip()
            if args.attempt >= 4 else None
        ),
        "device": str(device),
        "selection_policy": {
            "threshold_selected_on": "validation worlds only",
            "test_used_for_model_selection": False,
            "g4_contract": Path(args.g4_contract).name if args.g4_contract else None,
            "g4_contract_sha256": g4_file_sha256(contract) if contract else None,
            "g4_frozen_training": g4_parameters,
            "g4_dataset_binding": g4_dataset_binding,
            "g4_runtime_binding": g4_runtime_binding,
            "g4_attempt_ledger_sha256": g4_file_sha256(Path(args.g4_attempt_ledger)) if args.attempt >= 4 else None,
            "g4_test_lock_sha256": g4_file_sha256(Path(args.g4_test_lock)) if args.attempt >= 4 else None,
            "detector_threshold": threshold,
            "area_thresholds": dict(zip(AREA_NAMES, area_thresholds)),
        },
        "row_counts": {
            "train": len(train_rows),
            "in_domain_holdout": len(in_domain_rows),
            "validation": len(val_rows),
            "test": len(test_rows),
        },
        "detector": {
            "architecture": "direct anchor-free center/offset/bbox detector",
            "segmentation_connected_components_used_as_detector": False,
            "g4_quality_head_used": args.attempt >= 4,
            "input_shape": [
                1,
                6 if args.attempt >= 3 else 3,
                INPUT_HEIGHT,
                INPUT_WIDTH,
            ],
            "input_features": (
                ["rgb", "normalized_depth", "edge", "local_contrast"]
                if args.attempt >= 3
                else ["rgb"]
            ),
            "training": detector_training,
            "calibration_trace": detector_calibration,
            "results": detector_results,
            "onnx": detector_export,
            "parameter_count": sum(
                parameter.numel() for parameter in detector.parameters()
            ),
        },
        "area_segmenter": {
            "architecture": (
                "G4 independently parameterized RGB-D binary leaf/puddle U-Net heads"
                if args.attempt >= 4
                else "RGB-D independent binary leaf/puddle U-Net heads"
            ),
            "input_shape": [
                1,
                7 if args.attempt >= 3 else 4,
                INPUT_HEIGHT,
                INPUT_WIDTH,
            ],
            "input_features": (
                [
                    "rgb",
                    "normalized_depth",
                    "edge",
                    "local_contrast",
                    "saturation",
                ]
                if args.attempt >= 3
                else ["rgb", "normalized_depth"]
            ),
            "training": area_training,
            "calibration_trace": area_calibration,
            "results": area_results,
            "onnx": area_export,
            "parameter_count": sum(parameter.numel() for parameter in area.parameters()),
        },
        "gates": gates,
        "auto05_screening_gate_pass": all(gates.values()),
        "claim_boundary": "offline native Gazebo G3 screening only; not AUTO-06 formal, live, real-domain, J6, or competition evidence",
    }
    (output / "auto05_screening_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
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
                "random_seed": SEED,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "auto05_screening_gate_pass": report["auto05_screening_gate_pass"],
                "failed_gates": [
                    name for name, passed in gates.items() if not passed
                ],
                "detector": detector_results,
                "area": area_results,
            },
            indent=2,
        )
    )
    return 0 if report["auto05_screening_gate_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
