"""Exact development-only input/output contract for the recovered AUTO-05 model."""

from __future__ import annotations

import math

import cv2
import numpy as np


AREA_INPUT_SIZE = (512, 384)
AREA_CLASS_ORDER = ("leaf_pile", "puddle")


def preprocess_legacy_area(rgb: np.ndarray, depth_m: np.ndarray) -> np.ndarray:
    """Reproduce AUTO-05 attempt-3's seven-channel inference transform."""
    image = np.asarray(rgb)
    depth = np.asarray(depth_m, dtype=np.float32)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError("legacy area RGB must be HxWx3 uint8")
    if depth.shape != image.shape[:2]:
        raise ValueError("legacy area RGB/depth dimensions differ")
    image = cv2.resize(image, AREA_INPUT_SIZE, interpolation=cv2.INTER_AREA).astype(
        np.float32
    ) / 255.0
    depth = cv2.resize(depth, AREA_INPUT_SIZE, interpolation=cv2.INTER_NEAREST)
    valid = np.isfinite(depth) & (depth > 0.0)
    normalized_depth = np.zeros_like(depth, dtype=np.float32)
    normalized_depth[valid] = np.clip(
        np.log1p(depth[valid]) / math.log(11.0), 0.0, 1.0
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
    saturation = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)[:, :, 1]
    features = np.concatenate(
        (
            image,
            normalized_depth[:, :, None],
            edge[:, :, None],
            local_contrast[:, :, None],
            saturation[:, :, None],
        ),
        axis=2,
    ).astype(np.float32)
    if features.shape != (384, 512, 7):
        raise AssertionError(f"legacy area feature contract mismatch: {features.shape}")
    return np.ascontiguousarray(features.transpose(2, 0, 1)[None])


def decode_legacy_area(
    logits: np.ndarray,
    *,
    leaf_threshold: float = 0.9,
    puddle_threshold: float = 0.3,
) -> dict[str, dict[str, np.ndarray]]:
    """Decode the two independent heads; channel one is not a boundary map."""
    value = np.asarray(logits, dtype=np.float32)
    if value.shape != (1, 2, 384, 512):
        raise ValueError(f"legacy area output shape mismatch: {value.shape}")
    if not np.isfinite(value).all():
        raise ValueError("legacy area output contains non-finite logits")
    thresholds = (float(leaf_threshold), float(puddle_threshold))
    if any(not 0.0 <= threshold <= 1.0 for threshold in thresholds):
        raise ValueError("legacy area thresholds must be in [0, 1]")
    probabilities = 1.0 / (1.0 + np.exp(-np.clip(value[0], -80.0, 80.0)))
    return {
        class_name: {
            "probability": probabilities[index],
            "mask": probabilities[index] >= thresholds[index],
        }
        for index, class_name in enumerate(AREA_CLASS_ORDER)
    }


__all__ = [
    "AREA_CLASS_ORDER",
    "AREA_INPUT_SIZE",
    "decode_legacy_area",
    "preprocess_legacy_area",
]
