"""TRAIN-only teacher pseudo-label helpers for MODEL-RECOVERY-V2 route C."""

from __future__ import annotations

from .auto04_contract import box_iou
from .g4_data import DISCRETE_NAMES


def select_small_teacher_pseudo_labels(
    teacher_frames: list[dict],
    truth_by_key: dict[tuple[str, int, int], list[dict]],
    *,
    score_threshold: float = 0.70,
    iou_threshold: float = 0.50,
    small_short_side_px: float = 18.0,
) -> tuple[dict[tuple[str, int, int], list[dict]], dict]:
    """Assign high-confidence class-agnostic teacher boxes to TRAIN small GT.

    Class identity comes only from the existing TRAIN annotation.  The teacher
    contributes refined box geometry and confidence; it never invents a class
    and is never run on VAL or sealed-final data for training/selection.
    """
    selected: dict[tuple[str, int, int], list[dict]] = {}
    small_truth = matched_truth = 0
    for frame in teacher_frames:
        key = (
            str(frame["world_id"]),
            int(frame["scene_seed"]),
            int(frame["frame_index"]),
        )
        truths = [
            item for item in truth_by_key.get(key, ())
            if item.get("semantic_class") in DISCRETE_NAMES
            and min(
                float(item["bbox_xyxy"][2]) - float(item["bbox_xyxy"][0]),
                float(item["bbox_xyxy"][3]) - float(item["bbox_xyxy"][1]),
            ) < small_short_side_px
        ]
        detections = [
            item for item in frame.get("detections", ())
            if float(item.get("score", 0.0)) >= score_threshold
        ]
        small_truth += len(truths)
        used: set[int] = set()
        for truth in truths:
            candidates = []
            for index, detection in enumerate(detections):
                if index in used:
                    continue
                overlap = box_iou(
                    tuple(float(value) for value in truth["bbox_xyxy"]),
                    tuple(float(value) for value in detection["bbox_xyxy"]),
                )
                if overlap >= iou_threshold:
                    candidates.append((overlap, float(detection["score"]), index, detection))
            if not candidates:
                continue
            overlap, score, index, detection = max(candidates)
            used.add(index)
            matched_truth += 1
            selected.setdefault(key, []).append(
                {
                    "semantic_class": truth["semantic_class"],
                    "bbox_xyxy": [float(value) for value in detection["bbox_xyxy"]],
                    "teacher_score": score,
                    "teacher_iou_to_train_truth": overlap,
                    "pseudo_label_role": "train_only_small_geometry_refinement",
                }
            )
    return selected, {
        "teacher_score_threshold": score_threshold,
        "teacher_iou_threshold": iou_threshold,
        "small_short_side_px": small_short_side_px,
        "small_train_truth": small_truth,
        "matched_small_train_truth": matched_truth,
        "coverage": matched_truth / max(small_truth, 1),
        "frames_with_pseudo_labels": len(selected),
        "pseudo_label_count": sum(len(items) for items in selected.values()),
    }


def replace_small_truth_with_teacher(
    truths: list[dict], pseudo_labels: list[dict], *, match_iou: float = 0.30
) -> list[dict]:
    """Replace only matching small TRAIN geometry and avoid duplicate targets."""
    output = [dict(item) for item in truths]
    used: set[int] = set()
    for pseudo in pseudo_labels:
        candidates = []
        for index, truth in enumerate(output):
            if index in used or truth.get("semantic_class") != pseudo.get("semantic_class"):
                continue
            overlap = box_iou(
                tuple(float(value) for value in truth["bbox_xyxy"]),
                tuple(float(value) for value in pseudo["bbox_xyxy"]),
            )
            if overlap >= match_iou:
                candidates.append((overlap, index))
        if candidates:
            _, index = max(candidates)
            used.add(index)
            output[index] = dict(pseudo)
    return output


__all__ = ["replace_small_truth_with_teacher", "select_small_teacher_pseudo_labels"]
