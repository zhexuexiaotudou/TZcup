"""Observation records accepted by the product dynamic-map boundary."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Mapping


FORBIDDEN_SOURCE_NAMES = {"ground_truth", "gazebo_registry", "evaluation_registry"}


@dataclass(frozen=True)
class MapPoseMeasurement:
    x_m: float
    y_m: float
    z_m: float = 0.0
    covariance_xx: float = 0.01
    covariance_xy: float = 0.0
    covariance_yy: float = 0.01

    @property
    def covariance_trace(self) -> float:
        return self.covariance_xx + self.covariance_yy

    def validate(self) -> None:
        values = (
            self.x_m,
            self.y_m,
            self.z_m,
            self.covariance_xx,
            self.covariance_xy,
            self.covariance_yy,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("map measurement values must be finite")
        if self.covariance_xx <= 0.0 or self.covariance_yy <= 0.0:
            raise ValueError("map covariance diagonal must be positive")
        if abs(self.covariance_xy) ** 2 >= self.covariance_xx * self.covariance_yy:
            raise ValueError("map covariance must be positive definite")


@dataclass(frozen=True)
class TargetObservation:
    observation_id: str
    mission_id: str
    stamp_ns: int
    camera_frame_id: str
    image_frame_id: str
    source_model: str
    source_backend: str
    target_type: str
    class_probabilities: Mapping[str, float]
    confidence: float
    map_pose: MapPoseMeasurement
    bbox_xyxy: tuple[float, float, float, float] | None = None
    mask_reference: str | None = None
    polygon_xy_m: tuple[tuple[float, float], ...] = ()
    estimated_size_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    view_direction_rad: float = 0.0
    in_current_fov: bool = False

    def validate(self) -> None:
        if not self.observation_id or not self.mission_id:
            raise ValueError("observation_id and mission_id are required")
        if self.stamp_ns < 0:
            raise ValueError("stamp_ns must be non-negative")
        if not self.camera_frame_id or not self.image_frame_id:
            raise ValueError("camera and image frame identifiers are required")
        if self.source_backend.lower() in FORBIDDEN_SOURCE_NAMES:
            raise ValueError("ground-truth or registry observations are forbidden")
        if self.source_model.lower() in FORBIDDEN_SOURCE_NAMES:
            raise ValueError("ground-truth or registry source models are forbidden")
        if self.target_type not in {"DISCRETE", "AREA"}:
            raise ValueError("target_type must be DISCRETE or AREA")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in [0, 1]")
        probabilities = {str(key): float(value) for key, value in self.class_probabilities.items()}
        if not probabilities or any(not math.isfinite(value) or value < 0.0 for value in probabilities.values()):
            raise ValueError("class probabilities must be finite and non-negative")
        if sum(probabilities.values()) <= 0.0:
            raise ValueError("class probabilities must have positive mass")
        if self.bbox_xyxy is not None:
            x1, y1, x2, y2 = self.bbox_xyxy
            if not all(math.isfinite(value) for value in self.bbox_xyxy) or x2 <= x1 or y2 <= y1:
                raise ValueError("bbox_xyxy must be a finite positive rectangle")
        if self.target_type == "AREA" and len(self.polygon_xy_m) < 3:
            raise ValueError("AREA observations require a map polygon")
        if not all(math.isfinite(value) and value >= 0.0 for value in self.estimated_size_m):
            raise ValueError("estimated physical size must be finite and non-negative")
        self.map_pose.validate()

    def normalized_probabilities(self) -> dict[str, float]:
        total = sum(float(value) for value in self.class_probabilities.values())
        return {str(key): float(value) / total for key, value in self.class_probabilities.items()}

    def to_record(self) -> dict:
        payload = asdict(self)
        payload["class_probabilities"] = dict(self.class_probabilities)
        return payload

    @classmethod
    def from_record(cls, payload: dict) -> "TargetObservation":
        record = dict(payload)
        record["map_pose"] = MapPoseMeasurement(**record["map_pose"])
        for key in ("bbox_xyxy", "estimated_size_m"):
            if record.get(key) is not None:
                record[key] = tuple(record[key])
        record["polygon_xy_m"] = tuple(tuple(point) for point in record.get("polygon_xy_m", ()))
        return cls(**record)
