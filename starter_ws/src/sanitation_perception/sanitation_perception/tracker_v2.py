"""Product tracker with class-agnostic association and accumulated evidence."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import uuid


TERMINAL_STATES = {"CLEANED", "REJECTED"}


@dataclass(frozen=True)
class TrackerV2Config:
    association_distance_m: float
    close_recovery_distance_m: float
    minimum_image_iou: float
    maximum_observation_gap_s: float
    occlusion_recovery_s: float
    duplicate_distance_m: float
    confirmation_observations: int
    confirmation_class_posterior: float
    confirmation_score_ema: float
    score_ema_alpha: float
    defer_after_observations: int

    @classmethod
    def from_pipeline_manifest(cls, manifest: dict) -> "TrackerV2Config":
        try:
            values = manifest["runtime"]["tracker_v2"]
            config = cls(**values)
        except (KeyError, TypeError) as exc:
            raise ValueError("pipeline manifest tracker_v2 thresholds are incomplete") from exc
        if not 0.0 < config.score_ema_alpha <= 1.0:
            raise ValueError("score_ema_alpha must be in (0, 1]")
        for name in (
            "minimum_image_iou",
            "confirmation_class_posterior",
            "confirmation_score_ema",
        ):
            if not 0.0 <= float(getattr(config, name)) <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        if config.confirmation_observations < 2 or config.defer_after_observations < 1:
            raise ValueError("tracker observation thresholds are invalid")
        return config


@dataclass
class ProductTrack:
    uuid: str
    x_m: float
    y_m: float
    covariance_trace: float
    class_log_evidence: dict[str, float]
    score_ema: float
    observation_count: int
    first_seen_s: float
    last_seen_s: float
    bbox_xyxy: tuple[float, float, float, float] | None = None
    state: str = "TENTATIVE"
    state_before_occlusion: str = "TENTATIVE"
    source_backend: str = "onnxruntime"
    z_m: float = 0.0
    target_type: str = "DISCRETE"
    polygon_xy_m: tuple[tuple[float, float], ...] = ()
    physical_area_m2: float = 0.0

    @property
    def class_posterior(self) -> dict[str, float]:
        if not self.class_log_evidence:
            return {}
        peak = max(self.class_log_evidence.values())
        weights = {
            name: math.exp(max(-60.0, value - peak))
            for name, value in self.class_log_evidence.items()
        }
        total = sum(weights.values())
        return {name: value / total for name, value in weights.items()}

    @property
    def class_id(self) -> str:
        posterior = self.class_posterior
        eligible = {name: value for name, value in posterior.items() if name != "background"}
        return max(eligible or posterior, key=(eligible or posterior).get, default="background")

    @property
    def class_confidence(self) -> float:
        return float(self.class_posterior.get(self.class_id, 0.0))


def _bbox_iou(
    first: tuple[float, float, float, float] | None,
    second: tuple[float, float, float, float] | None,
) -> float | None:
    if first is None or second is None:
        return None
    x1, y1 = max(first[0], second[0]), max(first[1], second[1])
    x2, y2 = min(first[2], second[2]), min(first[3], second[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    second_area = max(0.0, second[2] - second[0]) * max(0.0, second[3] - second[1])
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


class ProductTrackerV2:
    """Class-agnostic map tracker; GT detections are rejected at ingress."""

    def __init__(self, config: TrackerV2Config):
        self.config = config
        self.namespace = uuid.UUID("37c781c8-4e87-5e18-819c-c0b8bd21f72d")
        self.tracks: dict[str, ProductTrack] = {}
        self._sequence = 0

    @staticmethod
    def _probabilities(detection: dict) -> dict[str, float]:
        supplied = detection.get("class_probabilities")
        if supplied:
            values = {str(name): max(1e-6, float(value)) for name, value in supplied.items()}
        else:
            confidence = min(1.0 - 1e-6, max(1e-6, float(detection["confidence"])))
            values = {
                str(detection["class_id"]): confidence,
                "background": 1.0 - confidence,
            }
        total = sum(values.values())
        if not math.isfinite(total) or total <= 0.0:
            raise ValueError("class probabilities must have a finite positive sum")
        return {name: value / total for name, value in values.items()}

    def _new_track(self, detection: dict, stamp: float) -> ProductTrack:
        probabilities = self._probabilities(detection)
        key = f"{round(float(detection['x_m']), 2)}:{round(float(detection['y_m']), 2)}:{self._sequence}"
        self._sequence += 1
        track = ProductTrack(
            uuid=str(uuid.uuid5(self.namespace, key)),
            x_m=float(detection["x_m"]),
            y_m=float(detection["y_m"]),
            covariance_trace=float(detection["covariance_trace"]),
            class_log_evidence={name: math.log(value) for name, value in probabilities.items()},
            score_ema=float(detection.get("confidence", max(probabilities.values()))),
            observation_count=1,
            first_seen_s=stamp,
            last_seen_s=stamp,
            bbox_xyxy=(
                tuple(float(value) for value in detection["bbox_xyxy"])
                if detection.get("bbox_xyxy") is not None
                else None
            ),
            source_backend=str(detection.get("source_backend", "onnxruntime")),
            z_m=float(detection.get("z_m", 0.0)),
            target_type=str(detection.get("target_type", "DISCRETE")),
            polygon_xy_m=tuple(
                tuple(float(coordinate) for coordinate in point)
                for point in detection.get("polygon_xy_m", ())
            ),
            physical_area_m2=float(detection.get("physical_area_m2", 0.0)),
        )
        self.tracks[track.uuid] = track
        return track

    def _association_cost(self, track: ProductTrack, detection: dict, stamp: float):
        age = stamp - track.last_seen_s
        if age < 0.0 or age > self.config.occlusion_recovery_s:
            return None
        distance = math.hypot(
            track.x_m - float(detection["x_m"]),
            track.y_m - float(detection["y_m"]),
        )
        if distance > self.config.association_distance_m:
            return None
        incoming_bbox = detection.get("bbox_xyxy")
        iou = _bbox_iou(
            track.bbox_xyxy,
            tuple(float(value) for value in incoming_bbox) if incoming_bbox is not None else None,
        )
        if (
            iou is not None
            and iou < self.config.minimum_image_iou
            and distance > self.config.close_recovery_distance_m
        ):
            return None
        return distance + age * 0.01 + (0.0 if iou is None else 0.05 * (1.0 - iou))

    def _update_track(self, track: ProductTrack, detection: dict, stamp: float) -> None:
        probabilities = self._probabilities(detection)
        count = track.observation_count + 1
        track.x_m = (track.x_m * track.observation_count + float(detection["x_m"])) / count
        track.y_m = (track.y_m * track.observation_count + float(detection["y_m"])) / count
        track.covariance_trace = (
            track.covariance_trace * track.observation_count
            + float(detection["covariance_trace"])
        ) / count
        for name in set(track.class_log_evidence) | set(probabilities):
            track.class_log_evidence[name] = max(
                -60.0,
                track.class_log_evidence.get(name, math.log(1e-6))
                + math.log(probabilities.get(name, 1e-6)),
            )
        score = float(detection.get("confidence", max(probabilities.values())))
        alpha = self.config.score_ema_alpha
        track.score_ema = alpha * score + (1.0 - alpha) * track.score_ema
        track.observation_count = count
        track.last_seen_s = stamp
        if detection.get("bbox_xyxy") is not None:
            track.bbox_xyxy = tuple(float(value) for value in detection["bbox_xyxy"])
        track.z_m = float(detection.get("z_m", track.z_m))
        track.target_type = str(detection.get("target_type", track.target_type))
        if detection.get("polygon_xy_m"):
            track.polygon_xy_m = tuple(
                tuple(float(coordinate) for coordinate in point)
                for point in detection["polygon_xy_m"]
            )
        track.physical_area_m2 = float(
            detection.get("physical_area_m2", track.physical_area_m2)
        )
        if track.state == "LOST":
            track.state = track.state_before_occlusion
        if (
            track.state in {"TENTATIVE", "DEFERRED"}
            and track.observation_count >= self.config.confirmation_observations
            and track.class_confidence >= self.config.confirmation_class_posterior
            and track.score_ema >= self.config.confirmation_score_ema
        ):
            track.state = "CONFIRMED"
        elif (
            track.state == "TENTATIVE"
            and track.observation_count >= self.config.defer_after_observations
        ):
            track.state = "DEFERRED"

    def _suppress_duplicates(self) -> None:
        active = sorted(
            (track for track in self.tracks.values() if track.state not in TERMINAL_STATES),
            key=lambda item: (item.first_seen_s, item.uuid),
        )
        removed: set[str] = set()
        for index, keeper in enumerate(active):
            if keeper.uuid in removed:
                continue
            for duplicate in active[index + 1 :]:
                if duplicate.uuid in removed:
                    continue
                if math.hypot(keeper.x_m - duplicate.x_m, keeper.y_m - duplicate.y_m) > self.config.duplicate_distance_m:
                    continue
                total = keeper.observation_count + duplicate.observation_count
                keeper.x_m = (keeper.x_m * keeper.observation_count + duplicate.x_m * duplicate.observation_count) / total
                keeper.y_m = (keeper.y_m * keeper.observation_count + duplicate.y_m * duplicate.observation_count) / total
                keeper.observation_count = total
                keeper.last_seen_s = max(keeper.last_seen_s, duplicate.last_seen_s)
                keeper.score_ema = max(keeper.score_ema, duplicate.score_ema)
                for name, value in duplicate.class_log_evidence.items():
                    keeper.class_log_evidence[name] = keeper.class_log_evidence.get(name, 0.0) + value
                removed.add(duplicate.uuid)
        for track_uuid in removed:
            del self.tracks[track_uuid]

    def update(self, detections: list[dict], stamp_s: float) -> list[ProductTrack]:
        matched: set[str] = set()
        for detection in detections:
            if detection.get("source_backend") == "ground_truth":
                raise ValueError("ground-truth detections are forbidden in the product tracker")
            candidates = [
                (cost, track)
                for track in self.tracks.values()
                if track.uuid not in matched and track.state not in TERMINAL_STATES
                for cost in [self._association_cost(track, detection, float(stamp_s))]
                if cost is not None
            ]
            if candidates:
                track = min(candidates, key=lambda item: item[0])[1]
                self._update_track(track, detection, float(stamp_s))
            else:
                track = self._new_track(detection, float(stamp_s))
            matched.add(track.uuid)
        for track in self.tracks.values():
            if track.uuid in matched or track.state in TERMINAL_STATES:
                continue
            if float(stamp_s) - track.last_seen_s > self.config.maximum_observation_gap_s:
                if track.state != "LOST":
                    track.state_before_occlusion = track.state
                track.state = "LOST"
        self._suppress_duplicates()
        return list(self.tracks.values())
