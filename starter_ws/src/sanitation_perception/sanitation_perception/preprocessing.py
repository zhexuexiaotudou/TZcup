from __future__ import annotations

import cv2
import numpy as np


MODEL_HEIGHT = 96
MODEL_WIDTH = 128
CLASS_ORDER = (
    "background",
    "plastic_bottle",
    "metal_can",
    "paper_litter",
    "leaf_pile",
    "puddle",
)


def preprocess_rgb(image_rgb: np.ndarray) -> np.ndarray:
    """Apply the deployed ROS preprocessing contract.

    The caller must provide RGB channel order.  Keeping this function free of
    ROS types lets the training audit compare the exact deployed transform with
    its PyTorch and ONNX inputs.
    """
    image = np.asarray(image_rgb)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("expected an HxWx3 RGB image")
    if image.dtype != np.uint8:
        raise ValueError("expected uint8 RGB input")
    resized = cv2.resize(
        image,
        (MODEL_WIDTH, MODEL_HEIGHT),
        interpolation=cv2.INTER_AREA,
    )
    return np.ascontiguousarray(
        np.transpose(resized.astype(np.float32) / 255.0, (2, 0, 1))[None]
    )


def resize_labels(labels: np.ndarray, width: int, height: int) -> np.ndarray:
    """Resize class indices without inventing intermediate label values."""
    return cv2.resize(
        np.asarray(labels, dtype=np.uint8),
        (int(width), int(height)),
        interpolation=cv2.INTER_NEAREST,
    )
