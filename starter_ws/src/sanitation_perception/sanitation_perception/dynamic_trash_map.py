"""Mission-scoped dynamic target map built only from online camera observations."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path
import uuid

from sanitation_perception.observation_model import TargetObservation
from sanitation_perception.observed_region_map import ObservedRegionMap
from sanitation_perception.trash_map_messages import (
    PostCleanState,
    TERMINAL_STATES,
    TargetState,
    require_transition,
)


@dataclass(frozen=True)
class DynamicTrashMapConfig:
    association_distance_m: float = 0.30
    confirmation_observations: int = 3
    confirmation_class_posterior: float = 0.70
    confirmation_confidence: float = 0.60
    maximum_covariance_trace: float = 0.03
    lost_after_s: float = 1.0
    reject_after_s: float = 5.0
    maximum_observation_history: int = 64

    def validate(self) -> None:
        if self.association_distance_m <= 0.0:
            raise ValueError("association_distance_m must be positive")
        if self.confirmation_observations < 2:
            raise ValueError("confirmation_observations must be at least 2")
        for value in (self.confirmation_class_posterior, self.confirmation_confidence):
            if not 0.0 <= value <= 1.0:
                raise ValueError("confirmation thresholds must be in [0, 1]")
        if self.maximum_covariance_trace <= 0.0:
            raise ValueError("maximum_covariance_trace must be positive")
        if not 0.0 < self.lost_after_s < self.reject_after_s:
            raise ValueError("expiry thresholds must be positive and ordered")
        if self.maximum_observation_history < self.confirmation_observations:
            raise ValueError("observation history is smaller than confirmation window")


@dataclass
class DynamicTrashTarget:
    uuid: str
    mission_id: str
    target_type: str
    class_log_evidence: dict[str, float]
    confidence_ema: float
    source_models: list[str]
    first_seen_stamp_ns: int
    last_seen_stamp_ns: int
    observation_count: int
    camera_frame_ids: list[str]
    image_history: list[dict]
    map_x_m: float
    map_y_m: float
    map_z_m: float
    covariance_xx: float
    covariance_xy: float
    covariance_yy: float
    information_weight: float
    polygon_xy_m: list[list[float]]
    estimated_size_m: list[float]
    view_directions_rad: list[float]
    track_state: TargetState = TargetState.CANDIDATE
    task_state: TargetState = TargetState.CANDIDATE
    clean_attempts: int = 0
    post_clean_verification_state: PostCleanState = PostCleanState.NOT_STARTED
    transitions: list[dict] = field(default_factory=list)

    @property
    def class_posterior(self) -> dict[str, float]:
        peak = max(self.class_log_evidence.values())
        weights = {
            name: math.exp(max(-60.0, value - peak))
            for name, value in self.class_log_evidence.items()
        }
        total = sum(weights.values())
        return {name: value / total for name, value in weights.items()}

    @property
    def current_class(self) -> str:
        posterior = self.class_posterior
        foreground = {name: value for name, value in posterior.items() if name != "background"}
        return max(foreground or posterior, key=(foreground or posterior).get)

    @property
    def confidence(self) -> float:
        return min(self.confidence_ema, self.class_posterior.get(self.current_class, 0.0))

    @property
    def covariance_trace(self) -> float:
        return self.covariance_xx + self.covariance_yy

    def to_record(self) -> dict:
        payload = asdict(self)
        payload["track_state"] = self.track_state.value
        payload["task_state"] = self.task_state.value
        payload["post_clean_verification_state"] = self.post_clean_verification_state.value
        payload["class_posterior"] = self.class_posterior
        payload["current_class"] = self.current_class
        payload["confidence"] = self.confidence
        return payload

    @classmethod
    def from_record(cls, record: dict) -> "DynamicTrashTarget":
        payload = dict(record)
        payload.pop("class_posterior", None)
        payload.pop("current_class", None)
        payload.pop("confidence", None)
        payload["track_state"] = TargetState(payload["track_state"])
        payload["task_state"] = TargetState(payload["task_state"])
        payload["post_clean_verification_state"] = PostCleanState(
            payload["post_clean_verification_state"]
        )
        return cls(**payload)


class DynamicTrashMap:
    """Online-only map. A new mission is always empty unless explicitly resumed."""

    SCHEMA_VERSION = 1
    UUID_NAMESPACE = uuid.UUID("3b077391-b0b2-5b63-a772-3b1805386c1a")

    def __init__(
        self,
        mission_id: str,
        *,
        observed_regions: ObservedRegionMap,
        config: DynamicTrashMapConfig = DynamicTrashMapConfig(),
    ):
        if not mission_id:
            raise ValueError("mission_id is required")
        if observed_regions.mission_id != mission_id:
            raise ValueError("observed-region map belongs to another mission")
        config.validate()
        self.mission_id = mission_id
        self.observed_regions = observed_regions
        self.config = config
        self.targets: dict[str, DynamicTrashTarget] = {}
        self.observation_log: list[dict] = []
        self.rejected_observations: list[dict] = []
        self._sequence = 0

    @classmethod
    def start_new(
        cls,
        mission_id: str,
        *,
        config: DynamicTrashMapConfig = DynamicTrashMapConfig(),
    ) -> "DynamicTrashMap":
        instance = cls(
            mission_id,
            observed_regions=ObservedRegionMap(mission_id),
            config=config,
        )
        if instance.count != 0:
            raise RuntimeError("new mission dynamic trash map must be empty")
        return instance

    @classmethod
    def resume_same_mission(cls, path: str | Path, mission_id: str) -> "DynamicTrashMap":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if payload.get("mission_id") != mission_id:
            raise ValueError("persisted map belongs to a different mission")
        config = DynamicTrashMapConfig(**payload["config"])
        observed = ObservedRegionMap.from_records(mission_id, payload["observed_regions"])
        instance = cls(mission_id, observed_regions=observed, config=config)
        instance.targets = {
            item["uuid"]: DynamicTrashTarget.from_record(item)
            for item in payload["targets"]
        }
        instance.observation_log = list(payload.get("observation_log", []))
        instance.rejected_observations = list(payload.get("rejected_observations", []))
        instance._sequence = int(payload.get("sequence", len(instance.targets)))
        return instance

    @property
    def count(self) -> int:
        return sum(target.track_state not in TERMINAL_STATES for target in self.targets.values())

    def _reject_observation(self, observation: TargetObservation, reason: str) -> None:
        self.rejected_observations.append(
            {
                "observation_id": observation.observation_id,
                "stamp_ns": observation.stamp_ns,
                "reason": reason,
            }
        )

    def _visibility_proven(self, observation: TargetObservation) -> bool:
        return observation.in_current_fov and self.observed_regions.visible_at(
            x_m=observation.map_pose.x_m,
            y_m=observation.map_pose.y_m,
            stamp_ns=observation.stamp_ns,
            camera_frame_id=observation.camera_frame_id,
            image_frame_id=observation.image_frame_id,
        )

    def _association(self, observation: TargetObservation) -> DynamicTrashTarget | None:
        candidates = [
            target
            for target in self.targets.values()
            if target.track_state not in TERMINAL_STATES
            and target.target_type == observation.target_type
            and math.hypot(
                target.map_x_m - observation.map_pose.x_m,
                target.map_y_m - observation.map_pose.y_m,
            ) <= self.config.association_distance_m
        ]
        return min(
            candidates,
            key=lambda target: math.hypot(
                target.map_x_m - observation.map_pose.x_m,
                target.map_y_m - observation.map_pose.y_m,
            ),
            default=None,
        )

    def _new_target(self, observation: TargetObservation) -> DynamicTrashTarget:
        probabilities = observation.normalized_probabilities()
        weight = 1.0 / observation.map_pose.covariance_trace
        key = f"{self.mission_id}:{observation.stamp_ns}:{observation.observation_id}:{self._sequence}"
        self._sequence += 1
        return DynamicTrashTarget(
            uuid=str(uuid.uuid5(self.UUID_NAMESPACE, key)),
            mission_id=self.mission_id,
            target_type=observation.target_type,
            class_log_evidence={name: math.log(max(value, 1e-9)) for name, value in probabilities.items()},
            confidence_ema=observation.confidence,
            source_models=[observation.source_model],
            first_seen_stamp_ns=observation.stamp_ns,
            last_seen_stamp_ns=observation.stamp_ns,
            observation_count=1,
            camera_frame_ids=[observation.camera_frame_id],
            image_history=[self._image_record(observation)],
            map_x_m=observation.map_pose.x_m,
            map_y_m=observation.map_pose.y_m,
            map_z_m=observation.map_pose.z_m,
            covariance_xx=observation.map_pose.covariance_xx,
            covariance_xy=observation.map_pose.covariance_xy,
            covariance_yy=observation.map_pose.covariance_yy,
            information_weight=weight,
            polygon_xy_m=[list(point) for point in observation.polygon_xy_m],
            estimated_size_m=list(observation.estimated_size_m),
            view_directions_rad=[observation.view_direction_rad],
            transitions=[
                {
                    "stamp_ns": observation.stamp_ns,
                    "from": None,
                    "to": TargetState.CANDIDATE.value,
                    "reason": "first_visible_online_observation",
                }
            ],
        )

    @staticmethod
    def _image_record(observation: TargetObservation) -> dict:
        return {
            "observation_id": observation.observation_id,
            "stamp_ns": observation.stamp_ns,
            "image_frame_id": observation.image_frame_id,
            "bbox_xyxy": list(observation.bbox_xyxy) if observation.bbox_xyxy else None,
            "mask_reference": observation.mask_reference,
        }

    def _fuse(self, target: DynamicTrashTarget, observation: TargetObservation) -> None:
        probabilities = observation.normalized_probabilities()
        incoming_weight = 1.0 / observation.map_pose.covariance_trace
        combined_weight = target.information_weight + incoming_weight
        target.map_x_m = (
            target.map_x_m * target.information_weight
            + observation.map_pose.x_m * incoming_weight
        ) / combined_weight
        target.map_y_m = (
            target.map_y_m * target.information_weight
            + observation.map_pose.y_m * incoming_weight
        ) / combined_weight
        target.map_z_m = (
            target.map_z_m * target.information_weight
            + observation.map_pose.z_m * incoming_weight
        ) / combined_weight
        target.information_weight = combined_weight
        target.covariance_xx = 1.0 / combined_weight * 0.5
        target.covariance_yy = 1.0 / combined_weight * 0.5
        target.covariance_xy = 0.0
        for name in set(target.class_log_evidence) | set(probabilities):
            target.class_log_evidence[name] = max(
                -60.0,
                target.class_log_evidence.get(name, math.log(1e-9))
                + math.log(max(probabilities.get(name, 1e-9), 1e-9)),
            )
        target.confidence_ema = 0.35 * observation.confidence + 0.65 * target.confidence_ema
        target.last_seen_stamp_ns = observation.stamp_ns
        target.observation_count += 1
        if observation.camera_frame_id not in target.camera_frame_ids:
            target.camera_frame_ids.append(observation.camera_frame_id)
        if observation.source_model not in target.source_models:
            target.source_models.append(observation.source_model)
        target.image_history.append(self._image_record(observation))
        del target.image_history[: max(0, len(target.image_history) - self.config.maximum_observation_history)]
        target.view_directions_rad.append(observation.view_direction_rad)
        del target.view_directions_rad[: max(0, len(target.view_directions_rad) - self.config.maximum_observation_history)]
        if observation.polygon_xy_m:
            target.polygon_xy_m = [list(point) for point in observation.polygon_xy_m]
        if any(observation.estimated_size_m):
            target.estimated_size_m = [
                (current * (target.observation_count - 1) + incoming) / target.observation_count
                for current, incoming in zip(target.estimated_size_m, observation.estimated_size_m)
            ]

    def _auto_state(self, target: DynamicTrashTarget, stamp_ns: int) -> None:
        requested = None
        reason = None
        if target.track_state in {TargetState.CANDIDATE, TargetState.LOST} and target.observation_count >= 2:
            requested, reason = TargetState.TRACKED, "multi_frame_track"
        if (
            target.observation_count >= self.config.confirmation_observations
            and target.class_posterior.get(target.current_class, 0.0) >= self.config.confirmation_class_posterior
            and target.confidence_ema >= self.config.confirmation_confidence
            and target.covariance_trace <= self.config.maximum_covariance_trace
        ):
            requested, reason = TargetState.CONFIRMED, "temporal_class_pose_confirmation"
        if requested is not None and requested != target.track_state:
            self.transition(target.uuid, requested, stamp_ns, reason)

    def ingest(self, observation: TargetObservation) -> DynamicTrashTarget | None:
        observation.validate()
        if observation.mission_id != self.mission_id:
            raise ValueError("observation belongs to another mission")
        if not self._visibility_proven(observation):
            self._reject_observation(observation, "no_matching_current_fov_proof")
            return None
        target = self._association(observation)
        if target is None:
            target = self._new_target(observation)
            self.targets[target.uuid] = target
        else:
            self._fuse(target, observation)
        self.observation_log.append(observation.to_record())
        self._auto_state(target, observation.stamp_ns)
        return target

    def transition(
        self,
        target_uuid: str,
        state: TargetState,
        stamp_ns: int,
        reason: str,
    ) -> DynamicTrashTarget:
        target = self.targets[target_uuid]
        require_transition(target.track_state, state)
        previous = target.track_state
        target.track_state = state
        target.task_state = state
        target.transitions.append(
            {"stamp_ns": stamp_ns, "from": previous.value, "to": state.value, "reason": reason}
        )
        return target

    def expire(self, now_ns: int) -> list[str]:
        changed = []
        for target in self.targets.values():
            if target.track_state in TERMINAL_STATES:
                continue
            age_s = (now_ns - target.last_seen_stamp_ns) / 1_000_000_000.0
            if age_s >= self.config.reject_after_s and target.track_state == TargetState.LOST:
                self.transition(target.uuid, TargetState.REJECTED, now_ns, "expired_after_removal")
                changed.append(target.uuid)
            elif age_s >= self.config.lost_after_s and target.track_state not in {
                TargetState.CLEANING,
                TargetState.POST_VERIFY,
                TargetState.LOST,
            }:
                self.transition(target.uuid, TargetState.LOST, now_ns, "not_reobserved_in_time")
                changed.append(target.uuid)
        return changed

    def snapshot(self) -> dict:
        return {
            "schema_version": self.SCHEMA_VERSION,
            "mission_id": self.mission_id,
            "active_target_count": self.count,
            "preknown_target_coordinates_used": False,
            "ground_truth_control_allowed": False,
            "config": asdict(self.config),
            "targets": [target.to_record() for target in sorted(self.targets.values(), key=lambda item: item.uuid)],
            "observed_regions": self.observed_regions.to_records(),
            "observation_log": list(self.observation_log),
            "rejected_observations": list(self.rejected_observations),
            "sequence": self._sequence,
        }

    def persist(self, path: str | Path) -> None:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            json.dumps(self.snapshot(), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    @classmethod
    def replay(cls, records: list[dict], observed_regions: list[dict], mission_id: str) -> "DynamicTrashMap":
        instance = cls(
            mission_id,
            observed_regions=ObservedRegionMap.from_records(mission_id, observed_regions),
        )
        for record in records:
            instance.ingest(TargetObservation.from_record(record))
        return instance
