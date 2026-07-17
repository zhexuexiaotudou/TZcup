from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
import uuid


TRACK_STATES = {
    "TENTATIVE",
    "CONFIRMED",
    "QUEUED",
    "APPROACHING",
    "CLEANING",
    "CLEANED",
    "LOST",
    "REJECTED",
}


@dataclass
class Track:
    uuid: str
    class_id: str
    target_type: str
    cleaning_policy: str
    x_m: float
    y_m: float
    confidence: float
    covariance_trace: float
    observation_count: int = 1
    first_seen: float = field(default_factory=time.monotonic)
    last_seen: float = field(default_factory=time.monotonic)
    state: str = "TENTATIVE"
    source_backend: str = "onnxruntime"


class TargetTracker:
    def __init__(
        self,
        confirmation_observations: int = 3,
        association_distance_m: float = 0.25,
        maximum_covariance_trace: float = 0.03,
        lost_timeout_s: float = 2.0,
    ):
        self.confirmation_observations = confirmation_observations
        self.association_distance_m = association_distance_m
        self.maximum_covariance_trace = maximum_covariance_trace
        self.lost_timeout_s = lost_timeout_s
        self.namespace = uuid.UUID("75fcf776-d5cc-5b2c-9335-3b923c69d74b")
        self.tracks: dict[str, Track] = {}

    def _new_uuid(self, detection: dict) -> str:
        key = f"{detection['class_id']}:{detection['x_m']:.2f}:{detection['y_m']:.2f}"
        return str(uuid.uuid5(self.namespace, key))

    def update(self, detections: list[dict], now: float | None = None) -> list[Track]:
        stamp = time.monotonic() if now is None else float(now)
        matched: set[str] = set()
        for detection in detections:
            if detection.get("source_backend") == "ground_truth":
                raise ValueError("ground-truth detections are forbidden in the decision tracker")
            candidates = [
                track
                for track in self.tracks.values()
                if track.class_id == detection["class_id"]
                and track.state not in {"CLEANED", "REJECTED"}
                and math.hypot(track.x_m - detection["x_m"], track.y_m - detection["y_m"])
                <= self.association_distance_m
            ]
            track = min(
                candidates,
                key=lambda item: math.hypot(item.x_m - detection["x_m"], item.y_m - detection["y_m"]),
                default=None,
            )
            if track is None:
                track = Track(
                    uuid=self._new_uuid(detection),
                    class_id=detection["class_id"],
                    target_type=detection["target_type"],
                    cleaning_policy=detection["cleaning_policy"],
                    x_m=float(detection["x_m"]),
                    y_m=float(detection["y_m"]),
                    confidence=float(detection["confidence"]),
                    covariance_trace=float(detection["covariance_trace"]),
                    first_seen=stamp,
                    last_seen=stamp,
                    source_backend=str(detection.get("source_backend", "onnxruntime")),
                )
                self.tracks[track.uuid] = track
            else:
                count = track.observation_count + 1
                track.x_m = (track.x_m * track.observation_count + float(detection["x_m"])) / count
                track.y_m = (track.y_m * track.observation_count + float(detection["y_m"])) / count
                track.observation_count = count
                track.confidence = min(track.confidence, float(detection["confidence"]))
                track.covariance_trace = min(track.covariance_trace, float(detection["covariance_trace"]))
                track.last_seen = stamp
            if (
                track.state in {"TENTATIVE", "LOST"}
                and
                track.observation_count >= self.confirmation_observations
                and track.confidence >= 0.80
                and track.covariance_trace <= self.maximum_covariance_trace
            ):
                track.state = "CONFIRMED"
            matched.add(track.uuid)
        for track in self.tracks.values():
            if track.uuid not in matched and track.state not in {"CLEANED", "REJECTED"}:
                if stamp - track.last_seen > self.lost_timeout_s:
                    track.state = "LOST"
        return list(self.tracks.values())

    def transition(self, track_uuid: str, next_state: str) -> Track:
        if next_state not in TRACK_STATES:
            raise ValueError(f"unknown track state {next_state}")
        track = self.tracks[track_uuid]
        allowed = {
            "CONFIRMED": {"QUEUED", "REJECTED"},
            "QUEUED": {"APPROACHING", "REJECTED"},
            "APPROACHING": {"CLEANING", "QUEUED", "REJECTED"},
            "CLEANING": {"CLEANED", "QUEUED", "REJECTED"},
        }
        if next_state not in allowed.get(track.state, set()):
            raise ValueError(f"invalid transition {track.state}->{next_state}")
        track.state = next_state
        return track
