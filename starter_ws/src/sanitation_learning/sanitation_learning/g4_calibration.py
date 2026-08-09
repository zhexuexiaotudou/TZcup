"""Validation-only operating-point selection for AUTO-05R screening.

The threshold grids are fixed before sealed-final access.  Selection is
constraint-first and deterministic; an infeasible result remains explicitly
diagnostic-only instead of being relabelled as a product operating point.
"""

from __future__ import annotations

from collections.abc import Iterable

from .g4_evaluation import area_metrics, discovery_metrics


DISCOVERY_THRESHOLD_GRID = tuple(
    round(value / 1000.0, 3) for value in range(50, 976, 25)
) + (0.99,)
AREA_THRESHOLD_GRID = tuple(
    round(value / 100.0, 2) for value in range(10, 91, 5)
)


def filter_discovery_frames(frames: Iterable[dict], threshold: float) -> list[dict]:
    """Return a score-filtered copy without rerunning model inference."""
    return [
        {
            **frame,
            "detections": [
                item
                for item in frame.get("detections", ())
                if float(item["score"]) >= float(threshold)
            ],
        }
        for frame in frames
    ]


def _discovery_violation(
    metrics: dict,
    *,
    false_candidates_per_min_max: float,
    negative_fp_per_frame_max: float,
) -> float:
    return (
        max(
            0.0,
            float(metrics["false_candidates_per_min"])
            - false_candidates_per_min_max,
        )
        / max(false_candidates_per_min_max, 1e-12)
        + max(
            0.0,
            float(metrics["negative_only_fp_per_frame"])
            - negative_fp_per_frame_max,
        )
        / max(negative_fp_per_frame_max, 1e-12)
    )


def select_discovery_threshold(
    frames: list[dict],
    *,
    thresholds: Iterable[float] = DISCOVERY_THRESHOLD_GRID,
    false_candidates_per_min_max: float = 2.0,
    negative_fp_per_frame_max: float = 0.05,
) -> dict:
    """Select a VAL operating point: FP constraints, then recall and AP50."""
    sweep = []
    for threshold in thresholds:
        metrics = discovery_metrics(filter_discovery_frames(frames, threshold))
        violation = _discovery_violation(
            metrics,
            false_candidates_per_min_max=false_candidates_per_min_max,
            negative_fp_per_frame_max=negative_fp_per_frame_max,
        )
        sweep.append(
            {
                "threshold": float(threshold),
                "metrics": metrics,
                "product_eligible": violation == 0.0,
                "constraint_violation": float(violation),
            }
        )
    if not sweep:
        raise ValueError("discovery threshold grid must not be empty")
    selected = min(
        sweep,
        key=lambda item: (
            not item["product_eligible"],
            item["constraint_violation"],
            -float(item["metrics"]["all_gt_candidate_recall"]),
            -float(item["metrics"]["ap50"]),
            -float(item["metrics"]["precision"]),
            -float(item["threshold"]),
        ),
    )
    return {
        **selected,
        "selection_split": "val",
        "selection_rule": "fp_constraints_then_recall_ap50_precision",
        "sweep": sweep,
    }


def _area_frames_at_threshold(
    predictions: list[dict], task: str, threshold: float
) -> list[dict]:
    if task not in ("leaf", "puddle"):
        raise ValueError(f"unknown area task {task!r}")
    thresholds = (float(threshold), 1.1) if task == "leaf" else (1.1, float(threshold))
    return [{**frame, "thresholds": thresholds} for frame in predictions]


def select_area_threshold(
    predictions: list[dict],
    task: str,
    *,
    thresholds: Iterable[float] = AREA_THRESHOLD_GRID,
    negative_fp_per_frame_max: float = 0.05,
    boundary_f1_min: float = 0.70,
) -> dict:
    """Select one task threshold using its own IoU and boundary channel."""
    key = "leaf_pile" if task == "leaf" else "puddle"
    sweep = []
    for threshold in thresholds:
        metrics = area_metrics(
            _area_frames_at_threshold(predictions, task, float(threshold))
        )
        boundary_f1 = float(metrics["boundary_f1_by_class"][key])
        iou = float(metrics["iou_by_class"][key])
        negative_fp = float(metrics["negative_area_fp_per_frame"])
        violation = (
            max(0.0, boundary_f1_min - boundary_f1)
            / max(boundary_f1_min, 1e-12)
            + max(0.0, negative_fp - negative_fp_per_frame_max)
            / max(negative_fp_per_frame_max, 1e-12)
        )
        sweep.append(
            {
                "threshold": float(threshold),
                "iou": iou,
                "boundary_f1": boundary_f1,
                "negative_area_fp_per_frame": negative_fp,
                "metrics": metrics,
                "product_eligible": violation == 0.0,
                "constraint_violation": float(violation),
            }
        )
    if not sweep:
        raise ValueError("area threshold grid must not be empty")
    selected = min(
        sweep,
        key=lambda item: (
            not item["product_eligible"],
            item["constraint_violation"],
            -item["iou"],
            -item["boundary_f1"],
            -item["threshold"],
        ),
    )
    return {
        **selected,
        "selection_split": "val",
        "selection_rule": "negative_fp_boundary_constraints_then_iou",
        "task": task,
        "sweep": sweep,
    }


__all__ = [
    "AREA_THRESHOLD_GRID",
    "DISCOVERY_THRESHOLD_GRID",
    "filter_discovery_frames",
    "select_area_threshold",
    "select_discovery_threshold",
]
