"""Development-only close-range evidence without direct action authority.

The provider deliberately stops at evidence generation.  ProductTrackerV2 and
ProductActionVerifier remain the only path towards a CONFIRMED target.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Protocol, Sequence

import numpy as np

from sanitation_perception.pretrained_contracts import Detection


D1_CLASS_MAPPING = {
    "plastic_bottle": "plastic_bottle",
    "drinks_can": "metal_can",
    "paper_waste": "paper_litter",
    "cigarette_butt": "background_or_unknown",
    "fast_food_packaging": "background_or_unknown",
    "plastic_bag": "background_or_unknown",
    "coffee_cup": "background_or_unknown",
    "glass_bottle": "background_or_unknown",
    "food_wrapper": "background_or_unknown",
    "general_litter": "background_or_unknown",
}
ACTIONABLE_CLASSES = frozenset({"plastic_bottle", "metal_can", "paper_litter"})


@dataclass(frozen=True)
class CameraInfoContract:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def validate(self) -> None:
        values = (self.fx, self.fy, self.cx, self.cy)
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera dimensions must be positive")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("camera intrinsics must be finite")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("camera focal lengths must be positive")


@dataclass(frozen=True)
class Observation:
    product_class: str
    score: float
    stamp_ns: int
    view_direction_rad: float


@dataclass(frozen=True)
class CloseRangeEvidence:
    status: str
    product_class: str
    class_posterior: dict[str, float]
    score_ema: float
    agreeing_views: int
    view_separation_rad: float
    distance_m: float | None
    bbox_short_side_px: float
    unknown_probability: float
    reasons: tuple[str, ...]
    native_match_bbox_xyxy: tuple[float, float, float, float] | None
    action_verifier_required: bool = True
    confirmed: bool = False
    clean_now: bool = False

    def as_tracker_detection(
        self,
        *,
        x_m: float,
        y_m: float,
        covariance_trace: float,
        bbox_xyxy: tuple[float, float, float, float],
    ) -> dict:
        """Build tracker input while preserving independent verification."""
        if self.status != "READY_FOR_ACTION_VERIFIER":
            raise ValueError("only ready evidence may be forwarded to the tracker")
        probabilities = dict(self.class_posterior)
        probabilities["background"] = probabilities.pop(
            "background_or_unknown", self.unknown_probability
        )
        return {
            "x_m": float(x_m),
            "y_m": float(y_m),
            "covariance_trace": float(covariance_trace),
            "bbox_xyxy": tuple(float(value) for value in bbox_xyxy),
            "confidence": float(self.score_ema),
            "class_id": self.product_class,
            "class_probabilities": probabilities,
            "source_backend": "development_d1_second_pass_onnx",
            "target_type": "DISCRETE",
        }


class CloseRangeEvidenceProvider(Protocol):
    def evaluate(
        self,
        rgb: np.ndarray,
        candidate_box_xyxy: tuple[float, float, float, float],
        track_history: Sequence[Observation],
        depth: np.ndarray,
        camera_info: CameraInfoContract,
        *,
        stamp_ns: int,
        view_direction_rad: float,
        reobserve_count: int = 0,
    ) -> CloseRangeEvidence:
        ...


@dataclass(frozen=True)
class D1SecondPassConfig:
    context_expansion: float = 0.25
    minimum_bbox_short_side_px: float = 48.0
    maximum_range_m: float = 2.5
    maximum_bbox_only_range_m: float = 6.0
    minimum_valid_depth_pixels: int = 20
    minimum_valid_depth_ratio: float = 0.20
    minimum_overlap_iou: float = 0.30
    minimum_score: float = 0.25
    minimum_agreeing_views: int = 2
    minimum_view_separation_rad: float = 0.05
    score_ema_alpha: float = 0.5
    maximum_reobserve_count: int = 2

    def validate(self) -> None:
        if self.context_expansion < 0.0:
            raise ValueError("context expansion must be non-negative")
        if (
            self.minimum_bbox_short_side_px <= 0.0
            or self.maximum_range_m <= 0.0
            or self.maximum_bbox_only_range_m < self.maximum_range_m
        ):
            raise ValueError("close-range gates must be positive")
        if self.minimum_valid_depth_pixels < 1:
            raise ValueError("minimum valid depth pixels must be positive")
        if not 0.0 < self.minimum_valid_depth_ratio <= 1.0:
            raise ValueError("minimum valid depth ratio must be in (0, 1]")
        if not 0.0 <= self.minimum_overlap_iou <= 1.0:
            raise ValueError("minimum overlap IoU must be in [0, 1]")
        if not 0.0 <= self.minimum_score <= 1.0:
            raise ValueError("minimum score must be in [0, 1]")
        if self.minimum_agreeing_views < 2:
            raise ValueError("second pass requires multi-frame evidence")
        if not 0.0 < self.minimum_view_separation_rad <= math.pi:
            raise ValueError("view separation must be in (0, pi]")
        if not 0.0 < self.score_ema_alpha <= 1.0:
            raise ValueError("score EMA alpha must be in (0, 1]")
        if not 0 <= self.maximum_reobserve_count <= 2:
            raise ValueError("at most two re-observations are allowed")


def _box_iou(first, second) -> float:
    x1, y1 = max(first[0], second[0]), max(first[1], second[1])
    x2, y2 = min(first[2], second[2]), min(first[3], second[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    second_area = max(0.0, second[2] - second[0]) * max(0.0, second[3] - second[1])
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def _angular_separation(values: Sequence[float]) -> float:
    normalized = [float(value) % math.tau for value in values]
    return max(
        (
            abs((first - second + math.pi) % math.tau - math.pi)
            for index, first in enumerate(normalized)
            for second in normalized[index + 1 :]
        ),
        default=0.0,
    )


class D1SecondPassEvidenceProvider:
    """Run D1 on an expanded native ROI and accumulate fail-closed evidence."""

    def __init__(
        self,
        infer_crop: Callable[[np.ndarray], Sequence[Detection]],
        config: D1SecondPassConfig = D1SecondPassConfig(),
    ) -> None:
        config.validate()
        self.infer_crop = infer_crop
        self.config = config

    @staticmethod
    def _result(
        *,
        status: str,
        bbox_short_side_px: float,
        distance_m: float | None,
        reasons: tuple[str, ...],
        product_class: str = "background_or_unknown",
        score: float = 0.0,
        agreeing_views: int = 0,
        view_separation_rad: float = 0.0,
        native_match_bbox_xyxy=None,
    ) -> CloseRangeEvidence:
        score = min(1.0, max(0.0, float(score)))
        posterior = {name: 0.0 for name in sorted(ACTIONABLE_CLASSES)}
        posterior["background_or_unknown"] = 1.0
        if product_class in ACTIONABLE_CLASSES:
            posterior[product_class] = score
            posterior["background_or_unknown"] = 1.0 - score
        return CloseRangeEvidence(
            status=status,
            product_class=product_class,
            class_posterior=posterior,
            score_ema=score,
            agreeing_views=agreeing_views,
            view_separation_rad=view_separation_rad,
            distance_m=distance_m,
            bbox_short_side_px=bbox_short_side_px,
            unknown_probability=posterior["background_or_unknown"],
            reasons=reasons,
            native_match_bbox_xyxy=native_match_bbox_xyxy,
        )

    def evaluate(
        self,
        rgb: np.ndarray,
        candidate_box_xyxy: tuple[float, float, float, float],
        track_history: Sequence[Observation],
        depth: np.ndarray,
        camera_info: CameraInfoContract,
        *,
        stamp_ns: int,
        view_direction_rad: float,
        reobserve_count: int = 0,
    ) -> CloseRangeEvidence:
        camera_info.validate()
        image = np.asarray(rgb)
        depth_values = np.asarray(depth, dtype=np.float32)
        if image.shape != (camera_info.height, camera_info.width, 3):
            raise ValueError("RGB shape does not match CameraInfo")
        if depth_values.shape != (camera_info.height, camera_info.width):
            raise ValueError("depth shape does not match CameraInfo")
        if image.dtype != np.uint8:
            raise ValueError("RGB input must be uint8")
        if not 0 <= reobserve_count <= self.config.maximum_reobserve_count:
            raise ValueError("invalid reobserve count")
        if stamp_ns < 0 or not math.isfinite(float(view_direction_rad)):
            raise ValueError("observation stamp and view direction must be valid")

        x1, y1, x2, y2 = (float(value) for value in candidate_box_xyxy)
        if not all(math.isfinite(value) for value in (x1, y1, x2, y2)) or x2 <= x1 or y2 <= y1:
            raise ValueError("candidate bbox must be a finite positive rectangle")
        if x1 < 0.0 or y1 < 0.0 or x2 > camera_info.width or y2 > camera_info.height:
            raise ValueError("candidate bbox must lie inside the native image")
        short_side = min(x2 - x1, y2 - y1)
        ix1, iy1 = int(math.floor(x1)), int(math.floor(y1))
        ix2, iy2 = int(math.ceil(x2)), int(math.ceil(y2))
        candidate_depth = depth_values[iy1:iy2, ix1:ix2]
        valid_depth = candidate_depth[
            np.isfinite(candidate_depth) & (candidate_depth > 0.0)
        ]
        valid_depth_ratio = (
            float(valid_depth.size) / float(candidate_depth.size)
            if candidate_depth.size
            else 0.0
        )
        distance_m = float(np.median(valid_depth)) if valid_depth.size else None
        if (
            distance_m is None
            or valid_depth.size < self.config.minimum_valid_depth_pixels
            or valid_depth_ratio < self.config.minimum_valid_depth_ratio
        ):
            return self._result(
                status="DEFER",
                bbox_short_side_px=short_side,
                distance_m=distance_m,
                reasons=("insufficient_valid_depth_coverage",),
            )
        if (
            short_side < self.config.minimum_bbox_short_side_px
            and distance_m > self.config.maximum_range_m
        ):
            return self._result(
                status="WAIT_CLOSE_RANGE",
                bbox_short_side_px=short_side,
                distance_m=distance_m,
                reasons=("close_range_gate_not_met",),
            )
        if distance_m > self.config.maximum_bbox_only_range_m:
            return self._result(
                status="DEFER",
                bbox_short_side_px=short_side,
                distance_m=distance_m,
                reasons=("bbox_depth_geometry_conflict",),
            )

        expand_x = (x2 - x1) * self.config.context_expansion
        expand_y = (y2 - y1) * self.config.context_expansion
        crop_x1 = max(0, int(math.floor(x1 - expand_x)))
        crop_y1 = max(0, int(math.floor(y1 - expand_y)))
        crop_x2 = min(camera_info.width, int(math.ceil(x2 + expand_x)))
        crop_y2 = min(camera_info.height, int(math.ceil(y2 + expand_y)))
        crop = np.ascontiguousarray(image[crop_y1:crop_y2, crop_x1:crop_x2])
        candidate_in_crop = (
            x1 - crop_x1,
            y1 - crop_y1,
            x2 - crop_x1,
            y2 - crop_y1,
        )
        matches = [
            detection
            for detection in self.infer_crop(crop)
            if math.isfinite(float(detection.score))
            and self.config.minimum_score <= float(detection.score) <= 1.0
            and len(detection.bbox_xyxy) == 4
            and all(math.isfinite(float(value)) for value in detection.bbox_xyxy)
            and _box_iou(detection.bbox_xyxy, candidate_in_crop)
            >= self.config.minimum_overlap_iou
        ]
        if not matches:
            status = (
                "OBSERVE_AGAIN"
                if reobserve_count < self.config.maximum_reobserve_count
                else "DEFER"
            )
            return self._result(
                status=status,
                bbox_short_side_px=short_side,
                distance_m=distance_m,
                reasons=("no_overlapping_second_pass_match",),
            )
        product_classes = {
            D1_CLASS_MAPPING.get(item.source_class, "background_or_unknown")
            for item in matches
        }
        actionable_matches = [
            item
            for item in matches
            if D1_CLASS_MAPPING.get(item.source_class, "background_or_unknown")
            in ACTIONABLE_CLASSES
        ]
        if len(product_classes) > 1:
            status = (
                "OBSERVE_AGAIN"
                if reobserve_count < self.config.maximum_reobserve_count
                else "DEFER"
            )
            return self._result(
                status=status,
                bbox_short_side_px=short_side,
                distance_m=distance_m,
                reasons=("conflicting_second_pass_matches",),
            )
        if not actionable_matches:
            return self._result(
                status="DEFER",
                bbox_short_side_px=short_side,
                distance_m=distance_m,
                reasons=("non_target_second_pass_class",),
            )

        best = max(actionable_matches, key=lambda item: item.score)
        product_class = D1_CLASS_MAPPING[best.source_class]
        observations_by_stamp = {
            int(item.stamp_ns): item
            for item in track_history
            if item.product_class == product_class
            and int(item.stamp_ns) >= 0
            and int(item.stamp_ns) < int(stamp_ns)
            and math.isfinite(float(item.score))
            and 0.0 <= float(item.score) <= 1.0
            and math.isfinite(float(item.view_direction_rad))
        }
        observations_by_stamp[int(stamp_ns)] = Observation(
            product_class=product_class,
            score=float(best.score),
            stamp_ns=int(stamp_ns),
            view_direction_rad=float(view_direction_rad),
        )
        observations = [
            observations_by_stamp[key] for key in sorted(observations_by_stamp)
        ]
        scores = [float(item.score) for item in observations]
        score_ema = scores[0]
        for score in scores[1:]:
            alpha = self.config.score_ema_alpha
            score_ema = alpha * score + (1.0 - alpha) * score_ema
        separation = _angular_separation(
            [item.view_direction_rad for item in observations]
        )
        enough_views = len(observations) >= self.config.minimum_agreeing_views
        separated = separation >= self.config.minimum_view_separation_rad
        if not enough_views or not separated:
            status = (
                "OBSERVE_AGAIN"
                if reobserve_count < self.config.maximum_reobserve_count
                else "DEFER"
            )
            reasons = tuple(
                name
                for name, passed in (
                    ("insufficient_agreeing_views", enough_views),
                    ("insufficient_view_separation", separated),
                )
                if not passed
            )
        else:
            status = "READY_FOR_ACTION_VERIFIER"
            reasons = ()
        native_box = (
            best.bbox_xyxy[0] + crop_x1,
            best.bbox_xyxy[1] + crop_y1,
            best.bbox_xyxy[2] + crop_x1,
            best.bbox_xyxy[3] + crop_y1,
        )
        return self._result(
            status=status,
            product_class=product_class,
            score=score_ema,
            agreeing_views=len(observations),
            view_separation_rad=separation,
            bbox_short_side_px=short_side,
            distance_m=distance_m,
            reasons=reasons,
            native_match_bbox_xyxy=native_box,
        )


__all__ = [
    "ACTIONABLE_CLASSES",
    "CameraInfoContract",
    "CloseRangeEvidence",
    "CloseRangeEvidenceProvider",
    "D1_CLASS_MAPPING",
    "D1SecondPassConfig",
    "D1SecondPassEvidenceProvider",
    "Observation",
]
