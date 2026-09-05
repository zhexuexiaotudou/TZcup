"""Truth-isolated metrics for formal random-scene perception acceptance.

This module is deliberately ROS independent.  The live evaluator is the only
consumer of generated scenario truth; product perception never imports it and
never receives evaluator topics.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import numpy as np


class EvaluationContractError(RuntimeError):
    """Raised when an evaluator input could weaken the truth boundary."""


@dataclass(frozen=True)
class BoxObservation:
    class_id: str
    confidence: float
    xyxy: tuple[float, float, float, float]


@dataclass(frozen=True)
class TruthBox:
    object_id: str
    class_id: str
    xyxy: tuple[float, float, float, float]


DEFAULT_THRESHOLDS = {
    "minimum_episode_count": 30,
    "cube_box_iou_match": 0.50,
    "cube_precision_min": 0.80,
    "cube_recall_min": 0.80,
    "cube_f1_min": 0.80,
    "false_positives_per_evaluated_frame_max": 0.20,
    "ground_dirt_iou_min": 0.65,
    "ground_dirt_recall_min": 0.85,
    "map_projection_rmse_m_max": 0.20,
    "map_projection_p95_m_max": 0.35,
    "depth_rgb_skew_s_max": 0.50,
    "required_rgb_topics": 4,
    "required_depth_topics": 2,
    "required_camera_info_topics": 4,
    "tf_success_ratio_min": 0.95,
    "tf_max_age_s": 0.50,
}


def load_evaluator_truth(path: str | Path) -> dict:
    """Load generated truth only when its evaluator-only contract is intact."""

    source = Path(path)
    try:
        truth = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise EvaluationContractError(f"unable to load evaluator truth: {source}") from exc
    if truth.get("schema_version") != 1:
        raise EvaluationContractError("evaluator truth schema_version must equal 1")
    namespace = str(truth.get("namespace", ""))
    if not namespace.startswith("/evaluation/"):
        raise EvaluationContractError("truth namespace must remain under /evaluation")
    if truth.get("control_use_prohibited") is not True:
        raise EvaluationContractError("truth must explicitly prohibit control use")
    if not isinstance(truth.get("discrete_cubes"), list) or not isinstance(
        truth.get("dirt_patches"), list
    ):
        raise EvaluationContractError("truth lacks cubes or dirt patches")
    return truth


def box_iou(a: Sequence[float], b: Sequence[float]) -> float:
    ax1, ay1, ax2, ay2 = map(float, a)
    bx1, by1, bx2, by2 = map(float, b)
    iw = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    ih = max(0.0, min(ay2, by2) - max(ay1, by1))
    intersection = iw * ih
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - intersection
    return intersection / union if union > 0.0 else 0.0


def match_boxes(
    predictions: Iterable[BoxObservation],
    truth: Iterable[TruthBox],
    *,
    iou_threshold: float,
) -> dict:
    """Greedy confidence-ordered, class-aware one-to-one box matching."""

    predictions = sorted(predictions, key=lambda item: item.confidence, reverse=True)
    truth = list(truth)
    unused = set(range(len(truth)))
    matches: list[dict] = []
    false_positive_indices: list[int] = []
    for prediction_index, prediction in enumerate(predictions):
        candidates = [
            (box_iou(prediction.xyxy, truth[index].xyxy), index)
            for index in unused
            if prediction.class_id == truth[index].class_id
        ]
        score, match_index = max(candidates, default=(0.0, -1))
        if match_index < 0 or score < iou_threshold:
            false_positive_indices.append(prediction_index)
            continue
        unused.remove(match_index)
        matches.append(
            {
                "prediction_index": prediction_index,
                "truth_index": match_index,
                "truth_object_id": truth[match_index].object_id,
                "iou": score,
            }
        )
    return {
        "matches": matches,
        "true_positive_count": len(matches),
        "false_positive_count": len(false_positive_indices),
        "false_negative_count": len(unused),
        "false_positive_indices": false_positive_indices,
        "unmatched_truth_object_ids": [truth[index].object_id for index in sorted(unused)],
    }


def rasterize_dirt_truth(
    dirt_patches: Iterable[Mapping],
    *,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> np.ndarray:
    """Rasterize rotated fixed-area rectangles in OccupancyGrid row order."""

    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise EvaluationContractError("invalid public grid geometry")
    result = np.zeros((height, width), dtype=bool)
    for patch in dirt_patches:
        pose = patch["pose"]
        size_x, size_y = map(float, patch["size_m"])
        cx, cy, yaw = float(pose["x_m"]), float(pose["y_m"]), float(pose["yaw_rad"])
        radius = math.hypot(size_x, size_y) / 2.0
        x0 = max(0, int(math.floor((cx - radius - origin_x) / resolution)))
        x1 = min(width, int(math.ceil((cx + radius - origin_x) / resolution)))
        y0 = max(0, int(math.floor((cy - radius - origin_y) / resolution)))
        y1 = min(height, int(math.ceil((cy + radius - origin_y) / resolution)))
        if x1 <= x0 or y1 <= y0:
            continue
        xs = origin_x + (np.arange(x0, x1) + 0.5) * resolution - cx
        ys = origin_y + (np.arange(y0, y1) + 0.5) * resolution - cy
        dx, dy = np.meshgrid(xs, ys)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        inside = (np.abs(local_x) <= size_x / 2.0) & (np.abs(local_y) <= size_y / 2.0)
        result[y0:y1, x0:x1] |= inside
    return result


def segmentation_metrics(predicted_dirty: np.ndarray, truth_dirty: np.ndarray) -> dict:
    if predicted_dirty.shape != truth_dirty.shape:
        raise EvaluationContractError("product and truth dirt rasters differ in shape")
    predicted = predicted_dirty.astype(bool)
    truth = truth_dirty.astype(bool)
    intersection = int(np.count_nonzero(predicted & truth))
    union = int(np.count_nonzero(predicted | truth))
    truth_count = int(np.count_nonzero(truth))
    predicted_count = int(np.count_nonzero(predicted))
    return {
        "intersection_cell_count": intersection,
        "union_cell_count": union,
        "truth_cell_count": truth_count,
        "predicted_cell_count": predicted_count,
        "iou": intersection / union if union else 1.0,
        "recall": intersection / truth_count if truth_count else 1.0,
        "precision": intersection / predicted_count if predicted_count else (1.0 if not truth_count else 0.0),
    }


def projection_error_metrics(errors_m: Iterable[float]) -> dict:
    values = np.asarray([float(value) for value in errors_m], dtype=np.float64)
    if values.size == 0:
        return {"sample_count": 0, "rmse_m": None, "p95_m": None, "max_m": None}
    if not np.isfinite(values).all() or np.any(values < 0.0):
        raise EvaluationContractError("map projection errors must be finite and non-negative")
    return {
        "sample_count": int(values.size),
        "rmse_m": float(np.sqrt(np.mean(values * values))),
        "p95_m": float(np.percentile(values, 95)),
        "max_m": float(np.max(values)),
    }


def finalize_acceptance(
    *,
    episode_id: str,
    detection: Mapping,
    segmentation: Mapping,
    projection: Mapping,
    freshness: Mapping,
    thresholds: Mapping | None = None,
) -> dict:
    """Create a fail-closed per-episode acceptance report."""

    limits = {**DEFAULT_THRESHOLDS, **dict(thresholds or {})}
    tp = int(detection.get("true_positive_count", 0))
    fp = int(detection.get("false_positive_count", 0))
    visible = int(detection.get("visible_unique_truth_count", 0))
    matched_unique = int(detection.get("matched_unique_truth_count", 0))
    frames = int(detection.get("evaluated_frame_count", 0))
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = matched_unique / visible if visible else 0.0
    f1 = 2.0 * precision * recall / (precision + recall) if precision + recall else 0.0
    fp_per_frame = fp / frames if frames else math.inf
    metric_checks = {
        "visible_cube_truth_present": visible > 0 and frames > 0
        and int(freshness.get("real_camera_message_count", 0)) > 0,
        "cube_precision": precision >= float(limits["cube_precision_min"]),
        "cube_recall": recall >= float(limits["cube_recall_min"]),
        "cube_f1": f1 >= float(limits["cube_f1_min"]),
        "false_positives_per_frame": fp_per_frame
        <= float(limits["false_positives_per_evaluated_frame_max"]),
        "ground_dirt_iou": float(segmentation.get("iou", 0.0))
        >= float(limits["ground_dirt_iou_min"]),
        "ground_dirt_recall": float(segmentation.get("recall", 0.0))
        >= float(limits["ground_dirt_recall_min"]),
        "map_projection_samples_present": int(projection.get("sample_count", 0)) > 0,
        "map_projection_rmse": projection.get("rmse_m") is not None
        and float(projection["rmse_m"]) <= float(limits["map_projection_rmse_m_max"]),
        "map_projection_p95": projection.get("p95_m") is not None
        and float(projection["p95_m"]) <= float(limits["map_projection_p95_m_max"]),
        "rgb_topics": int(freshness.get("rgb_topic_count", 0))
        >= int(limits["required_rgb_topics"]),
        "depth_topics": int(freshness.get("depth_topic_count", 0))
        >= int(limits["required_depth_topics"]),
        "camera_info_topics": int(freshness.get("camera_info_topic_count", 0))
        >= int(limits["required_camera_info_topics"]),
        "depth_rgb_skew": freshness.get("depth_rgb_skew_max_s") is not None
        and float(freshness["depth_rgb_skew_max_s"]) <= float(limits["depth_rgb_skew_s_max"]),
        "map_tf": float(freshness.get("tf_success_ratio", 0.0))
        >= float(limits["tf_success_ratio_min"]),
        "map_tf_age": freshness.get("tf_age_max_s") is not None
        and float(freshness["tf_age_max_s"]) <= float(limits["tf_max_age_s"]),
        "product_diagnostics_truth_free": freshness.get("diagnostic_ground_truth_input_used") is False,
        "product_detection_messages_seen": int(
            freshness.get("product_detection_message_count", 0)
        ) > 0,
        "product_mask_messages_seen": int(
            freshness.get("product_mask_message_count", 0)
        ) > 0,
        "product_target_messages_seen": int(
            freshness.get("product_target_message_count", 0)
        ) > 0,
        "real_camera_messages_seen": int(freshness.get("real_camera_message_count", 0)) > 0,
    }
    runtime_check_names = {
        "visible_cube_truth_present",
        "rgb_topics",
        "depth_topics",
        "camera_info_topics",
        "depth_rgb_skew",
        "map_tf",
        "map_tf_age",
        "product_diagnostics_truth_free",
        "product_detection_messages_seen",
        "product_mask_messages_seen",
        "product_target_messages_seen",
        "real_camera_messages_seen",
    }
    return {
        "schema_version": 1,
        "report_id": "tzcup_formal_random_scene_perception_episode_v1",
        "episode_id": episode_id,
        "status": "PASSED" if all(metric_checks.values()) else "BLOCKED_ACCURACY_OR_RUNTIME",
        "thresholds": limits,
        "litter_cube_detection": {
            **dict(detection),
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "false_positives_per_evaluated_frame": fp_per_frame if math.isfinite(fp_per_frame) else None,
        },
        "ground_dirt_segmentation": dict(segmentation),
        "map_projection": dict(projection),
        "sensor_runtime": dict(freshness),
        "metric_checks": metric_checks,
        "blocked_checks": {
            "runtime": sorted(name for name in runtime_check_names if not metric_checks[name]),
            "accuracy": sorted(
                name for name, passed in metric_checks.items() if name not in runtime_check_names and not passed
            ),
        },
        "truth_isolation": {
            "truth_reader": "evaluator_process_only",
            "truth_published_to_ros": False,
            "truth_used_by_product_perception": False,
            "truth_used_by_product_control": False,
            "synthetic_offline_image_used": False,
            "input_evidence_required": "Gazebo camera messages through formal vehicle ROS topics",
        },
        "claim_boundary": {
            "pc_product_perception_accepted": all(metric_checks.values()),
            "s100_board_accepted": False,
            "real_vehicle_accepted": False,
        },
    }
