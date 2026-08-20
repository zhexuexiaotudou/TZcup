"""Boundary-aware G6 Area recovery primitives.

The G6 corpus is development-only.  This module deliberately has no sealed-set
loader and keeps the model contract small enough for x86 development and later
fixed-shape ONNX export.  Temporal masks passed to :class:`AreaTemporalFilter`
must already be registered into the current image frame by the caller.
"""

from __future__ import annotations

from collections import deque
from pathlib import Path

import cv2
import numpy as np
import torch
from torch import nn
from torch.nn import functional


AREA_SIZE = (512, 384)
AREA_CLASSES = ("leaf_pile", "puddle")
SEMANTIC_IDS = {"leaf_pile": 4, "puddle": 5}
NEGATIVE_AREA_COLORS = {
    "wet_asphalt_not_puddle": (55, 82, 94),
    "specular_dry_road": (174, 180, 180),
    "dark_shadow": (32, 34, 38),
    "bright_reflection": (216, 219, 213),
    "road_paint": (224, 218, 170),
    "tile_seam": (63, 65, 68),
    "crack": (40, 39, 42),
    "oil_like_visual_decoy": (54, 62, 76),
    "curb_wet_edge": (70, 97, 103),
    "vehicle_shadow_body_reflection": (46, 51, 63),
}


def mask_boundary(mask: np.ndarray) -> np.ndarray:
    """Return a one-pixel, interior morphological boundary."""

    value = np.asarray(mask, dtype=np.uint8)
    eroded = cv2.erode(value, np.ones((3, 3), np.uint8), iterations=1)
    return (value > eroded).astype(np.uint8)


def negative_area_mask(rgb: np.ndarray, taxonomy: str | None) -> np.ndarray:
    """Recover the exact G6 hard-negative patch from its registered RGB color."""

    if not taxonomy:
        return np.zeros(np.asarray(rgb).shape[:2], dtype=np.uint8)
    if taxonomy not in NEGATIVE_AREA_COLORS:
        raise ValueError(f"unknown G6 negative-area taxonomy: {taxonomy}")
    color = np.asarray(NEGATIVE_AREA_COLORS[taxonomy], dtype=np.uint8)
    return np.all(np.asarray(rgb, dtype=np.uint8) == color, axis=2).astype(np.uint8)


def preprocess_g6_area(
    rgb: np.ndarray,
    depth_mm: np.ndarray,
    *,
    size: tuple[int, int] = AREA_SIZE,
) -> np.ndarray:
    """Create the fixed 10-channel RGB/appearance/geometry Area input."""

    rgb_u8 = np.asarray(rgb, dtype=np.uint8)
    depth_m = np.asarray(depth_mm, dtype=np.float32) / 1000.0
    resized_rgb = cv2.resize(rgb_u8, size, interpolation=cv2.INTER_AREA)
    resized_depth = cv2.resize(depth_m, size, interpolation=cv2.INTER_NEAREST)
    rgb_f = resized_rgb.astype(np.float32) / 255.0
    hsv = cv2.cvtColor(resized_rgb, cv2.COLOR_RGB2HSV).astype(np.float32)
    hsv[:, :, 0] /= 180.0
    hsv[:, :, 1:] /= 255.0
    valid = np.isfinite(resized_depth) & (resized_depth > 0.0)
    normalized_depth = np.zeros_like(resized_depth, dtype=np.float32)
    normalized_depth[valid] = np.clip(
        np.log1p(resized_depth[valid]) / np.log(11.0), 0.0, 1.0
    )
    ground_depth = float(np.median(resized_depth[valid])) if valid.any() else 0.0
    height_proxy = np.zeros_like(resized_depth, dtype=np.float32)
    height_proxy[valid] = np.clip(
        (ground_depth - resized_depth[valid]) / 3.0, -1.0, 1.0
    )
    gray = cv2.cvtColor(resized_rgb, cv2.COLOR_RGB2GRAY).astype(np.float32)
    texture = np.clip(np.abs(cv2.Laplacian(gray, cv2.CV_32F)) / 80.0, 0.0, 1.0)
    features = np.concatenate(
        [
            rgb_f,
            hsv,
            normalized_depth[:, :, None],
            valid.astype(np.float32)[:, :, None],
            height_proxy[:, :, None],
            texture[:, :, None],
        ],
        axis=2,
    ).astype(np.float32)
    if features.shape != (size[1], size[0], 10):
        raise AssertionError(f"G6 Area feature contract mismatch: {features.shape}")
    return np.ascontiguousarray(features.transpose(2, 0, 1))


def load_g6_area_sample(
    root: Path,
    row: dict,
    *,
    size: tuple[int, int] = AREA_SIZE,
) -> dict[str, np.ndarray]:
    """Load one audited G6 frame without consulting any sealed corpus."""

    rgb_bgr = cv2.imread(str(root / row["rgb_path"]), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(root / row["depth_path"]), cv2.IMREAD_UNCHANGED)
    semantic = cv2.imread(str(root / row["semantic_path"]), cv2.IMREAD_UNCHANGED)
    if rgb_bgr is None or depth is None or semantic is None:
        raise RuntimeError(f"failed to read G6 Area frame {row.get('rgb_path')}")
    rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)
    targets = np.stack(
        [(semantic == SEMANTIC_IDS[name]).astype(np.uint8) for name in AREA_CLASSES]
    )
    targets = np.stack(
        [cv2.resize(mask, size, interpolation=cv2.INTER_NEAREST) for mask in targets]
    ).astype(np.float32)
    boundaries = np.stack([mask_boundary(mask) for mask in targets]).astype(np.float32)
    taxonomy = (row.get("negative_area_taxonomies") or [None])[0]
    negative = cv2.resize(
        negative_area_mask(rgb, taxonomy), size, interpolation=cv2.INTER_NEAREST
    ).astype(np.float32)
    resized_depth = cv2.resize(
        depth.astype(np.float32) / 1000.0, size, interpolation=cv2.INTER_NEAREST
    )
    return {
        "features": preprocess_g6_area(rgb, depth, size=size),
        "targets": targets,
        "boundaries": boundaries,
        "negative": negative[None],
        "depth_m": resized_depth[None],
    }


class G6AreaDataset(torch.utils.data.Dataset):
    def __init__(
        self,
        root: Path,
        rows: list[dict],
        *,
        augment: bool = False,
        seed: int = 20260811,
    ):
        self.root = Path(root)
        self.rows = list(rows)
        self.augment = augment
        self.seed = int(seed)

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int):
        sample = load_g6_area_sample(self.root, self.rows[index])
        if self.augment:
            rng = np.random.default_rng(self.seed + index * 104729)
            if rng.random() < 0.5:
                for key in sample:
                    sample[key] = np.ascontiguousarray(sample[key][..., ::-1])
            gain = float(rng.uniform(0.88, 1.12))
            bias = float(rng.uniform(-0.035, 0.035))
            sample["features"][:3] = np.clip(
                sample["features"][:3] * gain + bias, 0.0, 1.0
            )
        return {key: torch.from_numpy(value.copy()) for key, value in sample.items()}


class _ConvBlock(nn.Sequential):
    def __init__(self, in_channels: int, out_channels: int):
        super().__init__(
            nn.Conv2d(in_channels, out_channels, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.SiLU(inplace=True),
            nn.Conv2d(out_channels, out_channels, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.SiLU(inplace=True),
        )


class G6BoundaryAwareAreaNet(nn.Module):
    """Shared encoder with high-resolution semantic and boundary-aware heads."""

    def __init__(self, base_channels: int = 16):
        super().__init__()
        base = int(base_channels)
        self.encoder_1 = _ConvBlock(10, base)
        self.encoder_2 = _ConvBlock(base, base * 2)
        self.encoder_3 = _ConvBlock(base * 2, base * 4)
        self.bottleneck = _ConvBlock(base * 4, base * 6)
        self.decoder_2 = _ConvBlock(base * 8, base * 2)
        self.decoder_1 = _ConvBlock(base * 3, base * 2)
        self.semantic_head = nn.Conv2d(base * 2, 2, 1)
        self.boundary_head = nn.Sequential(
            nn.Conv2d(base * 2, base, 3, padding=1),
            nn.SiLU(inplace=True),
            nn.Conv2d(base, 2, 1),
        )
        self.boundary_refiner = nn.Sequential(
            nn.Conv2d(base * 2 + 2, base, 3, padding=1),
            nn.SiLU(inplace=True),
            nn.Conv2d(base, 2, 1),
        )

    def forward(self, value: torch.Tensor) -> dict[str, torch.Tensor]:
        high = self.encoder_1(value)
        middle = self.encoder_2(functional.max_pool2d(high, 2))
        low = self.encoder_3(functional.max_pool2d(middle, 2))
        bottleneck = self.bottleneck(functional.max_pool2d(low, 2))
        up_middle = functional.interpolate(
            bottleneck, size=middle.shape[-2:], mode="bilinear", align_corners=False
        )
        decoded_middle = self.decoder_2(torch.cat([up_middle, middle], dim=1))
        up_high = functional.interpolate(
            decoded_middle, size=high.shape[-2:], mode="bilinear", align_corners=False
        )
        decoded = self.decoder_1(torch.cat([up_high, high], dim=1))
        boundary_logits = self.boundary_head(decoded)
        semantic_logits = self.semantic_head(decoded)
        semantic_logits = semantic_logits + self.boundary_refiner(
            torch.cat([decoded, torch.sigmoid(boundary_logits)], dim=1)
        )
        return {
            "semantic_logits": semantic_logits,
            "boundary_logits": boundary_logits,
        }


class G6AreaTaskExport(nn.Module):
    def __init__(self, model: G6BoundaryAwareAreaNet, task_index: int):
        super().__init__()
        self.model = model
        self.task_index = int(task_index)

    def forward(self, value: torch.Tensor) -> torch.Tensor:
        output = self.model(value)
        return torch.cat(
            [
                output["semantic_logits"][:, self.task_index : self.task_index + 1],
                output["boundary_logits"][:, self.task_index : self.task_index + 1],
            ],
            dim=1,
        )


def g6_area_loss(
    output: dict[str, torch.Tensor],
    targets: torch.Tensor,
    boundaries: torch.Tensor,
    negative_mask: torch.Tensor,
) -> dict[str, torch.Tensor]:
    semantic_logits = output["semantic_logits"]
    boundary_logits = output["boundary_logits"]
    pixel_weights = 1.0 + boundaries * 5.0 + negative_mask * 2.0
    semantic_bce = functional.binary_cross_entropy_with_logits(
        semantic_logits, targets, reduction="none"
    )
    semantic_bce = (semantic_bce * pixel_weights).mean()
    probabilities = torch.sigmoid(semantic_logits)
    intersection = (probabilities * targets).sum(dim=(0, 2, 3))
    denominator = (probabilities + targets).sum(dim=(0, 2, 3))
    dice = (1.0 - (2.0 * intersection + 1.0) / (denominator + 1.0)).mean()
    boundary_bce = functional.binary_cross_entropy_with_logits(
        boundary_logits, boundaries, reduction="none"
    )
    boundary_bce = (boundary_bce * (1.0 + boundaries * 7.0)).mean()
    negative_penalty = (probabilities * negative_mask).mean()
    total = semantic_bce + dice + 0.55 * boundary_bce + 1.5 * negative_penalty
    return {
        "total": total,
        "semantic_bce": semantic_bce,
        "dice": dice,
        "boundary_bce": boundary_bce,
        "negative_penalty": negative_penalty,
    }


def physical_component_filter(
    mask: np.ndarray,
    depth_m: np.ndarray,
    *,
    fx: float,
    fy: float,
    minimum_area_m2: float,
    minimum_valid_depth_ratio: float = 0.8,
) -> np.ndarray:
    """Reject components without valid ground geometry or minimum physical area."""

    value = np.asarray(mask, dtype=np.uint8)
    depth = np.asarray(depth_m, dtype=np.float32)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(value, connectivity=8)
    kept = np.zeros_like(value)
    for label in range(1, count):
        component = labels == label
        valid = component & np.isfinite(depth) & (depth > 0.0)
        pixels = int(component.sum())
        if pixels == 0 or valid.sum() / pixels < minimum_valid_depth_ratio:
            continue
        median_depth = float(np.median(depth[valid]))
        physical_area = pixels * (median_depth / fx) * (median_depth / fy)
        if physical_area >= minimum_area_m2:
            kept[component] = 1
    return kept.astype(bool)


class AreaTemporalFilter:
    """Require registered per-pixel support across a bounded frame window."""

    def __init__(self, *, window: int = 3, minimum_hits: int = 2):
        if not 1 <= minimum_hits <= window:
            raise ValueError("minimum_hits must be within the temporal window")
        self.window = int(window)
        self.minimum_hits = int(minimum_hits)
        self._history: deque[np.ndarray] = deque(maxlen=self.window)

    def reset(self) -> None:
        self._history.clear()

    def update(self, registered_mask: np.ndarray) -> np.ndarray:
        current = np.asarray(registered_mask, dtype=bool)
        if self._history and current.shape != self._history[0].shape:
            raise ValueError("registered temporal masks must keep a fixed shape")
        self._history.append(current.copy())
        hits = np.stack(tuple(self._history), axis=0).sum(axis=0)
        return current & (hits >= self.minimum_hits)


__all__ = [
    "AREA_CLASSES",
    "AREA_SIZE",
    "AreaTemporalFilter",
    "G6AreaDataset",
    "G6AreaTaskExport",
    "G6BoundaryAwareAreaNet",
    "NEGATIVE_AREA_COLORS",
    "g6_area_loss",
    "load_g6_area_sample",
    "mask_boundary",
    "negative_area_mask",
    "physical_component_filter",
    "preprocess_g6_area",
]
