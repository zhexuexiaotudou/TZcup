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

from .dosod_geometry import inverse_model_roi, square_pad_geometry


CLASS_IDS = ("litter_cube", "fallen_leaves", "dust_or_soil", "puddle")
OFFICIAL_PREFILTER_SCORE = 0.002
OFFICIAL_NMS_IOU = 0.65
OFFICIAL_CANDIDATE_CAP = 400
OFFICIAL_TOP_K = 300
PROJECT_CLASS_THRESHOLDS = {
    "litter_cube": 0.005,
    "fallen_leaves": 0.0025,
    "dust_or_soil": 0.002,
    "puddle": 0.003,
}


@dataclass(frozen=True)
class DosodDetection:
    """One detector result in original-image pixel coordinates."""

    class_id: str
    class_index: int
    confidence: float
    xyxy: tuple[float, float, float, float]


def _nms(
    boxes: np.ndarray, scores: np.ndarray, threshold: float, original_indices: np.ndarray
) -> np.ndarray:
    if boxes.size == 0:
        return np.empty((0,), dtype=np.int64)
    x1, y1, x2, y2 = boxes.T
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = np.asarray(
        sorted(range(len(scores)), key=lambda index: (-float(scores[index]), int(original_indices[index]))),
        dtype=np.int64,
    )
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


def postprocess_anchors(
    scores: np.ndarray,
    boxes: np.ndarray,
    *,
    prefilter_score: float = OFFICIAL_PREFILTER_SCORE,
    nms_iou: float = OFFICIAL_NMS_IOU,
    candidate_cap: int = OFFICIAL_CANDIDATE_CAP,
    top_k: int = OFFICIAL_TOP_K,
    class_thresholds: Mapping[str, float] = PROJECT_CLASS_THRESHOLDS,
    counters: dict[str, int] | None = None,
) -> list[tuple[int, int, float]]:
    """Frozen DOSOD anchor ordering shared by PC, ONNX, and HBM evaluation."""
    scores = np.asarray(scores, dtype=np.float32)
    boxes = np.asarray(boxes, dtype=np.float32)
    if scores.ndim != 2 or scores.shape[1] != len(CLASS_IDS) or boxes.shape != (scores.shape[0], 4):
        raise ValueError("DOSOD anchor output shape invalid")
    if not np.isfinite(scores).all():
        raise ValueError("DOSOD anchor scores nonfinite")
    if (
        prefilter_score != OFFICIAL_PREFILTER_SCORE
        or nms_iou != OFFICIAL_NMS_IOU
        or candidate_cap != OFFICIAL_CANDIDATE_CAP
        or top_k != OFFICIAL_TOP_K
        or dict(class_thresholds) != PROJECT_CLASS_THRESHOLDS
    ):
        raise ValueError("DOSOD postprocess contract drift")
    class_indices = np.argmax(scores, axis=1)
    chosen_scores = scores[np.arange(scores.shape[0]), class_indices]
    # The board contract is strictly greater than 0.002, not greater-or-equal.
    candidates = [index for index, value in enumerate(chosen_scores) if float(value) > prefilter_score]
    candidates.sort(key=lambda index: (-float(chosen_scores[index]), index))
    candidate_indices = np.asarray(candidates[:candidate_cap], dtype=np.int64)
    finite_ordered = np.isfinite(boxes[candidate_indices]).all(axis=1) & (boxes[candidate_indices, 2] > boxes[candidate_indices, 0]) & (boxes[candidate_indices, 3] > boxes[candidate_indices, 1])
    # The normal board domain contains finite boxes.  Reject malformed boxes
    # as a project fail-safe extension *after* cap400 and before NMS/top300, so
    # one bad anchor neither aborts the frame nor consumes a valid top-K slot.
    valid_indices = candidate_indices[finite_ordered]
    if counters is not None:
        counters["invalid_box"] = counters.get("invalid_box", 0) + int((~finite_ordered).sum())
    kept_positions = _nms(boxes[valid_indices], chosen_scores[valid_indices], nms_iou, valid_indices)
    kept_indices = valid_indices[kept_positions][:top_k]
    output: list[tuple[int, int, float]] = []
    for index in kept_indices:
        class_index = int(class_indices[index])
        score = float(chosen_scores[index])
        if score >= class_thresholds[CLASS_IDS[class_index]]:
            output.append((int(index), class_index, score))
    return output


def preprocess_rgb(image: np.ndarray, size: int = 640) -> tuple[np.ndarray, float, tuple[int, int]]:
    """Match the locked upstream DOSOD square-pad preprocessing."""

    if image.ndim != 3 or image.shape[2] != 3 or image.size == 0:
        raise ValueError("DOSOD input must be a non-empty HxWx3 RGB image")
    import cv2

    height, width = image.shape[:2]
    geometry = square_pad_geometry(width, height, size)
    max_size = max(height, width)
    padded = np.zeros((max_size, max_size, 3), dtype=image.dtype)
    padded[geometry.pad_y : geometry.pad_y + height, geometry.pad_x : geometry.pad_x + width] = image
    resized = cv2.resize(padded, (size, size), interpolation=cv2.INTER_LINEAR)
    tensor = resized.astype(np.float32) / 255.0
    tensor = tensor.transpose(2, 0, 1)[None]
    return tensor, geometry.scale, (geometry.pad_y, geometry.pad_x)


class DosodOnnxDetector:
    """Execute the reparameterized no-NMS DOSOD ONNX model."""

    def __init__(
        self,
        model_path: str | Path | None = None,
        *,
        session=None,
        providers: Sequence[str] | None = None,
        class_ids: Sequence[str] = CLASS_IDS,
        score_threshold: float = OFFICIAL_PREFILTER_SCORE,
        class_score_thresholds: Mapping[str, float] | None = None,
        nms_threshold: float = OFFICIAL_NMS_IOU,
        max_detections: int = OFFICIAL_TOP_K,
    ) -> None:
        if tuple(class_ids) != CLASS_IDS:
            raise ValueError("DOSOD class order must match the frozen project vocabulary")
        if session is None:
            if model_path is None or not Path(model_path).is_file():
                raise FileNotFoundError(f"DOSOD ONNX artifact missing: {model_path}")
            import onnxruntime as ort

            requested_providers = list(providers or ["CPUExecutionProvider"])
            if not requested_providers:
                raise ValueError("DOSOD ONNX provider list must not be empty")
            session = ort.InferenceSession(str(model_path), providers=requested_providers)
        self.session = session
        self.class_ids = tuple(class_ids)
        self.score_threshold = float(score_threshold)
        overrides = dict(class_score_thresholds or {})
        unknown_thresholds = sorted(set(overrides) - set(self.class_ids))
        if unknown_thresholds:
            raise ValueError(f"unknown DOSOD class thresholds: {unknown_thresholds}")
        if any(
            not np.isfinite(float(value)) or not 0.0 <= float(value) <= 1.0
            for value in overrides.values()
        ):
            raise ValueError("DOSOD class thresholds must be finite values in [0, 1]")
        self.class_score_thresholds = dict(PROJECT_CLASS_THRESHOLDS)
        self.class_score_thresholds.update({class_id: float(value) for class_id, value in overrides.items()})
        self.nms_threshold = float(nms_threshold)
        self.max_detections = int(max_detections)
        self.last_geometry_counters = {"kept": 0, "partial_clip": 0, "padding_only_drop": 0, "invalid_box": 0}
        inputs = self.session.get_inputs()
        outputs = {item.name for item in self.session.get_outputs()}
        if len(inputs) != 1 or not {"scores", "boxes"} <= outputs:
            raise ValueError("DOSOD ONNX must expose images -> scores, boxes")
        self.input_name = inputs[0].name

    def infer(self, rgb_image: np.ndarray) -> list[DosodDetection]:
        tensor, scale, (pad_y, pad_x) = preprocess_rgb(rgb_image)
        raw_scores, raw_boxes = self.session.run(["scores", "boxes"], {self.input_name: tensor})
        scores = np.asarray(raw_scores, dtype=np.float32)
        boxes = np.asarray(raw_boxes, dtype=np.float32)
        if scores.shape != (1, 8400, len(self.class_ids)) or boxes.shape != (1, 8400, 4):
            raise RuntimeError("DOSOD output shape does not match frozen 8400-anchor contract")
        scores = scores[0]
        boxes = boxes[0]
        if not np.isfinite(scores).all():
            raise RuntimeError("DOSOD scores are non-finite")

        height, width = rgb_image.shape[:2]
        geometry = square_pad_geometry(width, height)
        self.last_geometry_counters = {"kept": 0, "partial_clip": 0, "padding_only_drop": 0, "invalid_box": 0}
        results: list[DosodDetection] = []
        thresholds = {class_id: self.class_score_thresholds.get(class_id, self.score_threshold) for class_id in self.class_ids}
        for index, class_index, score in postprocess_anchors(
            scores, boxes, prefilter_score=self.score_threshold, nms_iou=self.nms_threshold,
            candidate_cap=OFFICIAL_CANDIDATE_CAP, top_k=self.max_detections,
            class_thresholds=thresholds, counters=self.last_geometry_counters,
        ):
            roi, status = inverse_model_roi(boxes[index], geometry)
            if roi is None:
                self.last_geometry_counters[status] += 1
                continue
            self.last_geometry_counters[status] += 1
            x1, y1, x2, y2 = roi
            results.append(DosodDetection(
                class_id=self.class_ids[class_index], class_index=class_index,
                confidence=score, xyxy=(float(x1), float(y1), float(x2), float(y2)),
            ))
        return results
