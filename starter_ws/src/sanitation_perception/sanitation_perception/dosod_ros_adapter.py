"""CPU ONNX adapter for the frozen four-class DOSOD product detector.

This module is ROS-independent on purpose.  The product node imports it, while
unit tests can exercise preprocessing and postprocessing without ROS or model
weights.  It accepts only camera pixels and never exposes evaluator inputs.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np


CLASS_IDS = ("litter_cube", "fallen_leaves", "dust_or_soil", "puddle")


@dataclass(frozen=True)
class DosodDetection:
    """One detector result in original-image pixel coordinates."""

    class_id: str
    class_index: int
    confidence: float
    xyxy: tuple[float, float, float, float]


def _nms(boxes: np.ndarray, scores: np.ndarray, threshold: float) -> np.ndarray:
    if boxes.size == 0:
        return np.empty((0,), dtype=np.int64)
    x1, y1, x2, y2 = boxes.T
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]
    keep: list[int] = []
    while order.size:
        index = int(order[0])
        keep.append(index)
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(x1[index], x1[rest])
        yy1 = np.maximum(y1[index], y1[rest])
        xx2 = np.minimum(x2[index], x2[rest])
        yy2 = np.minimum(y2[index], y2[rest])
        intersection = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        union = areas[index] + areas[rest] - intersection
        iou = np.divide(intersection, union, out=np.zeros_like(intersection), where=union > 0)
        order = rest[iou <= threshold]
    return np.asarray(keep, dtype=np.int64)


def preprocess_rgb(image: np.ndarray, size: int = 640) -> tuple[np.ndarray, float, tuple[int, int]]:
    """Match the locked upstream DOSOD square-pad preprocessing."""

    if image.ndim != 3 or image.shape[2] != 3 or image.size == 0:
        raise ValueError("DOSOD input must be a non-empty HxWx3 RGB image")
    import cv2

    height, width = image.shape[:2]
    max_size = max(height, width)
    pad_y = (max_size - height) // 2
    pad_x = (max_size - width) // 2
    padded = np.zeros((max_size, max_size, 3), dtype=image.dtype)
    padded[pad_y : pad_y + height, pad_x : pad_x + width] = image
    resized = cv2.resize(padded, (size, size), interpolation=cv2.INTER_LINEAR)
    tensor = resized.astype(np.float32) / 255.0
    tensor = tensor.transpose(2, 0, 1)[None]
    return tensor, size / float(max_size), (pad_y, pad_x)


class DosodOnnxDetector:
    """Execute the reparameterized no-NMS DOSOD ONNX model."""

    def __init__(
        self,
        model_path: str | Path | None = None,
        *,
        session=None,
        class_ids: Sequence[str] = CLASS_IDS,
        score_threshold: float = 0.25,
        class_score_thresholds: Mapping[str, float] | None = None,
        nms_threshold: float = 0.65,
        max_detections: int = 300,
    ) -> None:
        if tuple(class_ids) != CLASS_IDS:
            raise ValueError("DOSOD class order must match the frozen project vocabulary")
        if session is None:
            if model_path is None or not Path(model_path).is_file():
                raise FileNotFoundError(f"DOSOD ONNX artifact missing: {model_path}")
            import onnxruntime as ort

            session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        self.session = session
        self.class_ids = tuple(class_ids)
        self.score_threshold = float(score_threshold)
        class_score_thresholds = dict(class_score_thresholds or {})
        unknown_thresholds = sorted(set(class_score_thresholds) - set(self.class_ids))
        if unknown_thresholds:
            raise ValueError(f"unknown DOSOD class thresholds: {unknown_thresholds}")
        if any(
            not np.isfinite(float(value)) or not 0.0 <= float(value) <= 1.0
            for value in class_score_thresholds.values()
        ):
            raise ValueError("DOSOD class thresholds must be finite values in [0, 1]")
        self.class_score_thresholds = {
            class_id: float(value) for class_id, value in class_score_thresholds.items()
        }
        self.nms_threshold = float(nms_threshold)
        self.max_detections = int(max_detections)
        inputs = self.session.get_inputs()
        outputs = {item.name for item in self.session.get_outputs()}
        if len(inputs) != 1 or not {"scores", "boxes"} <= outputs:
            raise ValueError("DOSOD ONNX must expose images -> scores, boxes")
        self.input_name = inputs[0].name

    def infer(self, rgb_image: np.ndarray) -> list[DosodDetection]:
        tensor, scale, (pad_y, pad_x) = preprocess_rgb(rgb_image)
        scores, boxes = self.session.run(["scores", "boxes"], {self.input_name: tensor})
        scores = np.asarray(scores, dtype=np.float32)[0]
        boxes = np.asarray(boxes, dtype=np.float32)[0]
        if scores.ndim != 2 or scores.shape[1] != len(self.class_ids):
            raise RuntimeError("DOSOD output class count does not match frozen vocabulary")
        if boxes.shape != (scores.shape[0], 4) or not (
            np.isfinite(scores).all() and np.isfinite(boxes).all()
        ):
            raise RuntimeError("DOSOD returned invalid or non-finite outputs")

        height, width = rgb_image.shape[:2]
        results: list[DosodDetection] = []
        for class_index, class_id in enumerate(self.class_ids):
            class_scores = scores[:, class_index]
            threshold = self.class_score_thresholds.get(class_id, self.score_threshold)
            candidates = np.flatnonzero(class_scores >= threshold)
            if not candidates.size:
                continue
            kept = candidates[_nms(boxes[candidates], class_scores[candidates], self.nms_threshold)]
            for index in kept:
                x1, y1, x2, y2 = boxes[index].astype(float)
                x1 = np.clip(x1 / scale - pad_x, 0.0, float(width))
                x2 = np.clip(x2 / scale - pad_x, 0.0, float(width))
                y1 = np.clip(y1 / scale - pad_y, 0.0, float(height))
                y2 = np.clip(y2 / scale - pad_y, 0.0, float(height))
                if x2 <= x1 or y2 <= y1:
                    continue
                results.append(
                    DosodDetection(
                        class_id=class_id,
                        class_index=class_index,
                        confidence=float(class_scores[index]),
                        xyxy=(float(x1), float(y1), float(x2), float(y2)),
                    )
                )
        results.sort(key=lambda result: result.confidence, reverse=True)
        return results[: self.max_detections]
