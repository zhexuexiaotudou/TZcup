"""Frozen FCOS-lite discovery preprocessing and graph-external decoding."""

from __future__ import annotations

import cv2
import numpy as np


MODEL_SIZE = (640, 480)
PYRAMID_STRIDES = (4, 8, 16)


def preprocess_discovery(rgb: np.ndarray) -> np.ndarray:
    if np.asarray(rgb).ndim != 3 or np.asarray(rgb).shape[2] != 3:
        raise ValueError("discovery RGB must be HxWx3")
    resized = cv2.resize(rgb, MODEL_SIZE, interpolation=cv2.INTER_AREA)
    return np.ascontiguousarray(
        resized.transpose(2, 0, 1)[None], dtype=np.float32
    ) / 255.0


def _iou(first, second) -> float:
    x1, y1 = max(first[0], second[0]), max(first[1], second[1])
    x2, y2 = min(first[2], second[2]), min(first[3], second[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area1 = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    area2 = max(0.0, second[2] - second[0]) * max(0.0, second[3] - second[1])
    union = area1 + area2 - intersection
    return intersection / union if union > 0.0 else 0.0


def _decode_level(
    probability: np.ndarray,
    offset: np.ndarray,
    size: np.ndarray,
    *,
    stride: int,
    threshold: float,
) -> list[dict]:
    local_max = cv2.dilate(probability.astype(np.float32), np.ones((3, 3), np.uint8))
    ys, xs = np.where((probability >= threshold) & (probability >= local_max))
    detections = []
    for y, x in zip(ys.tolist(), xs.tolist()):
        width = max(0.0, float(size[0, y, x]) * stride)
        height = max(0.0, float(size[1, y, x]) * stride)
        center_x = (x + float(offset[0, y, x])) * stride
        center_y = (y + float(offset[1, y, x])) * stride
        bbox = (
            max(0.0, center_x - width * 0.5),
            max(0.0, center_y - height * 0.5),
            min(float(MODEL_SIZE[0]), center_x + width * 0.5),
            min(float(MODEL_SIZE[1]), center_y + height * 0.5),
        )
        if bbox[2] > bbox[0] and bbox[3] > bbox[1]:
            detections.append(
                {"score": float(probability[y, x]), "bbox_xyxy": bbox}
            )
    return detections


def decode_discovery(
    flat_output: np.ndarray,
    *,
    score_threshold: float,
    nms_iou_threshold: float,
    maximum_candidates: int = 100,
) -> list[dict]:
    flat = np.asarray(flat_output, dtype=np.float32)
    if flat.shape != (1, 15, 120, 160):
        raise ValueError(f"discovery output shape mismatch: {flat.shape}")
    logits, offsets, sizes = flat[0, :3], flat[0, 3:9], flat[0, 9:15]
    probability = 1.0 / (1.0 + np.exp(-np.clip(logits, -80.0, 80.0)))
    candidates = []
    for level, stride in enumerate(PYRAMID_STRIDES):
        factor = stride // 4
        candidates.extend(
            _decode_level(
                probability[level, ::factor, ::factor],
                offsets[level * 2 : level * 2 + 2, ::factor, ::factor],
                sizes[level * 2 : level * 2 + 2, ::factor, ::factor],
                stride=stride,
                threshold=float(score_threshold),
            )
        )
    kept = []
    for candidate in sorted(candidates, key=lambda item: item["score"], reverse=True):
        if any(
            _iou(candidate["bbox_xyxy"], prior["bbox_xyxy"])
            >= nms_iou_threshold
            for prior in kept
        ):
            continue
        kept.append(candidate)
        if len(kept) >= maximum_candidates:
            break
    return kept


__all__ = ["MODEL_SIZE", "decode_discovery", "preprocess_discovery"]
