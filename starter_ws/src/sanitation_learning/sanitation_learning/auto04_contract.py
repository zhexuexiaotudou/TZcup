"""Pure-numpy contracts shared by the AUTO-04 detector and its tests.

The detector predicts an object-centre heatmap plus centre offsets and box size.
It is intentionally not a semantic-segmentation/connected-components detector.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class Detection:
    class_index: int
    score: float
    bbox_xyxy: tuple[float, float, float, float]


def box_iou(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    iw = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    ih = max(0.0, min(ay2, by2) - max(ay1, by1))
    intersection = iw * ih
    union = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    union += max(0.0, bx2 - bx1) * max(0.0, by2 - by1) - intersection
    return intersection / union if union > 0.0 else 0.0


def classwise_nms(
    detections: list[Detection], iou_threshold: float = 0.5
) -> list[Detection]:
    kept: list[Detection] = []
    for class_index in sorted({item.class_index for item in detections}):
        candidates = sorted(
            (item for item in detections if item.class_index == class_index),
            key=lambda item: item.score,
            reverse=True,
        )
        while candidates:
            selected = candidates.pop(0)
            kept.append(selected)
            candidates = [
                item
                for item in candidates
                if box_iou(selected.bbox_xyxy, item.bbox_xyxy) < iou_threshold
            ]
    return sorted(kept, key=lambda item: item.score, reverse=True)


def encode_centernet_targets(
    boxes: list[dict],
    *,
    input_width: int,
    input_height: int,
    stride: int,
    class_count: int,
) -> dict[str, np.ndarray]:
    if input_width % stride or input_height % stride:
        raise ValueError("input dimensions must be divisible by stride")
    output_width = input_width // stride
    output_height = input_height // stride
    heatmap = np.zeros((class_count, output_height, output_width), np.float32)
    offset = np.zeros((2, output_height, output_width), np.float32)
    size = np.zeros((2, output_height, output_width), np.float32)
    regression_mask = np.zeros((1, output_height, output_width), np.float32)

    for item in boxes:
        class_index = int(item["class_index"])
        if not 0 <= class_index < class_count:
            raise ValueError(f"invalid class index: {class_index}")
        x1, y1, x2, y2 = (float(value) for value in item["bbox_xyxy"])
        if x2 <= x1 or y2 <= y1:
            raise ValueError("box must have positive area")
        center_x = (x1 + x2) * 0.5 / stride
        center_y = (y1 + y2) * 0.5 / stride
        cell_x = min(output_width - 1, max(0, int(math.floor(center_x))))
        cell_y = min(output_height - 1, max(0, int(math.floor(center_y))))
        radius = max(1, min(3, int(round(min(x2 - x1, y2 - y1) / stride / 4))))
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                xx, yy = cell_x + dx, cell_y + dy
                if 0 <= xx < output_width and 0 <= yy < output_height:
                    value = math.exp(-(dx * dx + dy * dy) / max(2.0 * radius, 1.0))
                    heatmap[class_index, yy, xx] = max(
                        heatmap[class_index, yy, xx], value
                    )
        heatmap[class_index, cell_y, cell_x] = 1.0
        offset[:, cell_y, cell_x] = (center_x - cell_x, center_y - cell_y)
        size[:, cell_y, cell_x] = ((x2 - x1) / stride, (y2 - y1) / stride)
        regression_mask[0, cell_y, cell_x] = 1.0

    return {
        "heatmap": heatmap,
        "offset": offset,
        "size": size,
        "regression_mask": regression_mask,
    }


def decode_centernet_outputs(
    heatmap_probability: np.ndarray,
    offset: np.ndarray,
    size: np.ndarray,
    *,
    stride: int,
    score_threshold: float,
    nms_iou_threshold: float = 0.5,
    local_maximum_radius: int = 1,
    max_detections: int | None = None,
    pre_nms_topk: int | None = None,
) -> list[Detection]:
    if heatmap_probability.ndim != 3:
        raise ValueError("heatmap must be CxHxW")
    if offset.shape != (2, *heatmap_probability.shape[1:]):
        raise ValueError("offset shape mismatch")
    if size.shape != (2, *heatmap_probability.shape[1:]):
        raise ValueError("size shape mismatch")
    _, height, width = heatmap_probability.shape
    detections: list[Detection] = []
    for class_index in range(heatmap_probability.shape[0]):
        for y in range(height):
            for x in range(width):
                score = float(heatmap_probability[class_index, y, x])
                if score < score_threshold:
                    continue
                x0 = max(0, x - local_maximum_radius)
                x1 = min(width, x + local_maximum_radius + 1)
                y0 = max(0, y - local_maximum_radius)
                y1 = min(height, y + local_maximum_radius + 1)
                if score < float(
                    heatmap_probability[class_index, y0:y1, x0:x1].max()
                ):
                    continue
                box_width = max(0.0, float(size[0, y, x]) * stride)
                box_height = max(0.0, float(size[1, y, x]) * stride)
                center_x = (x + float(offset[0, y, x])) * stride
                center_y = (y + float(offset[1, y, x])) * stride
                detections.append(
                    Detection(
                        class_index=class_index,
                        score=score,
                        bbox_xyxy=(
                            center_x - box_width * 0.5,
                            center_y - box_height * 0.5,
                            center_x + box_width * 0.5,
                            center_y + box_height * 0.5,
                        ),
                    )
                )
    if pre_nms_topk is not None:
        if pre_nms_topk < 1:
            raise ValueError("pre_nms_topk must be positive")
        detections = sorted(
            detections, key=lambda item: item.score, reverse=True
        )[:pre_nms_topk]
    decoded = classwise_nms(detections, nms_iou_threshold)
    if max_detections is not None:
        if max_detections < 1:
            raise ValueError("max_detections must be positive")
        decoded = decoded[:max_detections]
    return decoded
