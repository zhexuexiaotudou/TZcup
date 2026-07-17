from __future__ import annotations

from dataclasses import dataclass

from sanitation_perception.tracking import TargetTracker, Track


@dataclass(frozen=True)
class Preflight:
    at_component_boundary: bool
    path_available: bool
    keepout_clear: bool
    footprint_clearance_m: float
    covariance_trace: float
    observation_age_s: float


class SpotCleaningCoordinator:
    def __init__(self, tracker: TargetTracker, mode: str = "deferred", maximum_retries: int = 2):
        if mode not in {"deferred", "immediate", "post_coverage"}:
            raise ValueError("invalid spot-cleaning mode")
        self.tracker = tracker
        self.mode = mode
        self.maximum_retries = maximum_retries
        self.retries: dict[str, int] = {}
        self.events: list[dict] = []
        self.coverage_paused = False
        self.coverage_resumed = False
        self.brush_enabled = False

    def queue_confirmed(self) -> list[Track]:
        queued = []
        for track in self.tracker.tracks.values():
            if track.state == "CONFIRMED":
                self.tracker.transition(track.uuid, "QUEUED")
                queued.append(track)
        return queued

    @staticmethod
    def preflight_ok(preflight: Preflight) -> tuple[bool, str]:
        checks = {
            "not_component_boundary": preflight.at_component_boundary,
            "no_path": preflight.path_available,
            "keepout_or_obstacle": preflight.keepout_clear,
            "insufficient_clearance": preflight.footprint_clearance_m >= 0.15,
            "covariance_too_large": preflight.covariance_trace <= 0.03,
            "observation_stale": preflight.observation_age_s <= 1.0,
        }
        for reason, passed in checks.items():
            if not passed:
                return False, reason
        return True, "accepted"

    def clean(self, track_uuid: str, preflight: Preflight, cleaned_fraction: float = 1.0) -> dict:
        track = self.tracker.tracks[track_uuid]
        accepted, reason = self.preflight_ok(preflight)
        if not accepted:
            self.retries[track_uuid] = self.retries.get(track_uuid, 0) + 1
            if self.retries[track_uuid] > self.maximum_retries:
                self.tracker.transition(track_uuid, "REJECTED")
            return {"target_uuid": track_uuid, "result": "deferred", "reason": reason}
        if track.source_backend == "ground_truth":
            raise ValueError("GT control violation: ground-truth track reached cleaning coordinator")
        self.coverage_paused = True
        self.coverage_resumed = False
        self.tracker.transition(track_uuid, "APPROACHING")
        self.tracker.transition(track_uuid, "CLEANING")
        self.brush_enabled = True
        threshold = 0.90 if track.target_type == "area" else 1.0
        result = "cleaned" if cleaned_fraction >= threshold else "incomplete"
        if result == "cleaned":
            self.tracker.transition(track_uuid, "CLEANED")
        else:
            self.tracker.transition(track_uuid, "QUEUED")
        self.brush_enabled = False
        self.coverage_paused = False
        self.coverage_resumed = True
        event = {
            "target_uuid": track_uuid,
            "class_id": track.class_id,
            "cleaning_policy": track.cleaning_policy,
            "result": result,
            "cleaned_fraction": cleaned_fraction,
            "brush_enabled_during_event": True,
            "brush_final": self.brush_enabled,
            "in_keepout": False,
            "source_backend": track.source_backend,
        }
        self.events.append(event)
        return event
