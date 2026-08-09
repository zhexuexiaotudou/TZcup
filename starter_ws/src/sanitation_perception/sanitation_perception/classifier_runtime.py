"""Frozen crop-classifier preprocessing and acceptance policy."""

from __future__ import annotations

import cv2
import numpy as np

from sanitation_perception.detector_runtime import MODEL_SIZE


CLASS_NAMES = ("background", "plastic_bottle", "metal_can", "paper_litter")
CLASSIFIER_SIZE = (192, 192)


def _square_crop(width: int, height: int, bbox, scale: float = 3.0):
    x1, y1, x2, y2 = (float(value) for value in bbox)
    center_x, center_y = (x1 + x2) * 0.5, (y1 + y2) * 0.5
    side = min(float(min(width, height)), max(48.0, max(x2 - x1, y2 - y1) * scale))
    side_int = max(1, int(side))
    left = max(0, min(width - side_int, int(round(center_x - side * 0.5))))
    top = max(0, min(height - side_int, int(round(center_y - side * 0.5))))
    return left, top, left + side_int, top + side_int


def classifier_input(rgb: np.ndarray, bbox_model) -> np.ndarray:
    height, width = rgb.shape[:2]
    scale_x, scale_y = width / MODEL_SIZE[0], height / MODEL_SIZE[1]
    native_bbox = (
        float(bbox_model[0]) * scale_x,
        float(bbox_model[1]) * scale_y,
        float(bbox_model[2]) * scale_x,
        float(bbox_model[3]) * scale_y,
    )
    left, top, right, bottom = _square_crop(width, height, native_bbox)
    crop = cv2.resize(
        rgb[top:bottom, left:right], CLASSIFIER_SIZE, interpolation=cv2.INTER_AREA
    )
    return np.ascontiguousarray(
        crop.transpose(2, 0, 1)[None], dtype=np.float32
    ) / 255.0


def classify_candidate(
    logits: np.ndarray, candidate: dict, *, score_threshold: float
) -> dict:
    flat = np.asarray(logits, dtype=np.float32)
    if flat.shape != (1, 4):
        raise ValueError(f"classifier output shape mismatch: {flat.shape}")
    shifted = flat[0] - float(flat[0].max())
    probabilities = np.exp(shifted)
    probabilities /= probabilities.sum()
    class_index = int(np.argmax(probabilities[1:])) + 1
    accepted = (
        float(probabilities[class_index]) >= score_threshold
        and float(probabilities[class_index]) > float(probabilities[0])
    )
    return {
        **candidate,
        "class_index": class_index if accepted else 0,
        "class_id": CLASS_NAMES[class_index] if accepted else "background",
        "confidence": float(
            probabilities[class_index] if accepted else probabilities[0]
        ),
        "class_probabilities": {
            name: float(probabilities[index])
            for index, name in enumerate(CLASS_NAMES)
        },
        "accepted": bool(accepted),
    }


__all__ = ["CLASS_NAMES", "classifier_input", "classify_candidate"]
