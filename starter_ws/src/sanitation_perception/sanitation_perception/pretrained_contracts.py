"""Graph-external detector/classifier contracts shared by PC and Journey 6."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Detection:
    bbox_xyxy: tuple[float, float, float, float]
    score: float
    source_class: str
    product_class: str


def _iou(first: Detection, second: Detection) -> float:
    ax1, ay1, ax2, ay2 = first.bbox_xyxy
    bx1, by1, bx2, by2 = second.bbox_xyxy
    intersection = max(0.0, min(ax2, bx2) - max(ax1, bx1)) * max(
        0.0, min(ay2, by2) - max(ay1, by1)
    )
    first_area = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    second_area = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def decode_yolo_detect(
    output: np.ndarray,
    *,
    class_order: list[str],
    class_mapping: dict[str, str],
    score_threshold: float,
    nms_iou_threshold: float,
    input_size: tuple[int, int],
    has_objectness: bool = False,
    maximum_detections: int = 100,
) -> list[Detection]:
    """Decode YOLOv5/v9/v11-style raw xywh + class score output.

    The manifest owns class order and output layout; dynamic NMS stays outside
    the model graph for HBDK compatibility.
    """
    tensor = np.asarray(output, dtype=np.float32)
    if tensor.ndim != 3 or tensor.shape[0] != 1:
        raise ValueError(f"YOLO output must be rank-3 batch=1: {tensor.shape}")
    class_offset = 5 if has_objectness else 4
    attribute_count = class_offset + len(class_order)
    if tensor.shape[1] == attribute_count:
        rows = tensor[0].T
    elif tensor.shape[2] == attribute_count:
        rows = tensor[0]
    else:
        raise ValueError("YOLO output attribute dimension does not match class_order")
    width, height = input_size
    candidates: list[Detection] = []
    for row in rows:
        class_index = int(np.argmax(row[class_offset:]))
        class_score = float(row[class_offset + class_index])
        score = class_score * float(row[4]) if has_objectness else class_score
        if score < score_threshold:
            continue
        center_x, center_y, box_width, box_height = (float(value) for value in row[:4])
        x1 = max(0.0, min(float(width), center_x - box_width * 0.5))
        y1 = max(0.0, min(float(height), center_y - box_height * 0.5))
        x2 = max(0.0, min(float(width), center_x + box_width * 0.5))
        y2 = max(0.0, min(float(height), center_y + box_height * 0.5))
        if x2 <= x1 or y2 <= y1:
            continue
        source_class = class_order[class_index]
        candidates.append(
            Detection(
                bbox_xyxy=(x1, y1, x2, y2),
                score=score,
                source_class=source_class,
                product_class=class_mapping.get(source_class, "background_or_unknown"),
            )
        )
    kept: list[Detection] = []
    for candidate in sorted(candidates, key=lambda item: item.score, reverse=True):
        if any(
            prior.product_class == candidate.product_class
            and _iou(candidate, prior) >= nms_iou_threshold
            for prior in kept
        ):
            continue
        kept.append(candidate)
        if len(kept) >= maximum_detections:
            break
    return kept


def decode_material_classifier(
    logits: np.ndarray,
    *,
    class_order: list[str],
    class_mapping: dict[str, str],
    minimum_confidence: float,
) -> dict:
    values = np.asarray(logits, dtype=np.float32)
    if values.ndim == 2 and values.shape[0] == 1:
        values = values[0]
    if values.ndim != 1 or values.shape[0] != len(class_order):
        raise ValueError("classifier output does not match class_order")
    shifted = values - float(values.max())
    probabilities = np.exp(shifted)
    probabilities /= probabilities.sum()
    index = int(np.argmax(probabilities))
    source_class = class_order[index]
    confidence = float(probabilities[index])
    product_evidence = class_mapping.get(source_class, "background_or_unknown")
    accepted = confidence >= minimum_confidence and product_evidence != "background_or_unknown"
    return {
        "source_class": source_class,
        "product_evidence": product_evidence if accepted else "background_or_unknown",
        "confidence": confidence,
        "accepted": accepted,
        "probabilities": {
            name: float(probabilities[position])
            for position, name in enumerate(class_order)
        },
    }


def fuse_detector_classifier(
    detection: Detection,
    classification: dict,
    *,
    stable_track: bool,
    valid_depth: bool,
    reobserve_count: int,
) -> dict:
    """Return verifier input; this function never emits CLEAN_NOW/CONFIRMED."""
    detector_class = detection.product_class
    material_class = classification.get("product_evidence", "background_or_unknown")
    agreement = detector_class == material_class
    actionable = detector_class in {"plastic_bottle", "metal_can", "paper_litter"}
    ready = bool(
        actionable
        and agreement
        and classification.get("accepted") is True
        and stable_track
        and valid_depth
    )
    if ready:
        status = "READY_FOR_ACTION_VERIFIER"
    elif reobserve_count < 2 and actionable:
        status = "OBSERVE_AGAIN"
    else:
        status = "DEFER"
    return {
        "status": status,
        "detector_class": detector_class,
        "material_evidence": material_class,
        "detector_classifier_agreement": agreement,
        "stable_track": bool(stable_track),
        "valid_depth": bool(valid_depth),
        "reobserve_count": int(reobserve_count),
        "action_verifier_required": True,
        "confirmed": False,
        "clean_now": False,
    }


__all__ = [
    "Detection",
    "decode_material_classifier",
    "decode_yolo_detect",
    "fuse_detector_classifier",
]
