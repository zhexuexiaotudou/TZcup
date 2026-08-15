from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CadenceConfig:
    every_n_frames: int = 5
    distance_trigger_m: float = 0.5
    scene_change_threshold: float = 0.25
    confidence_drop_threshold: float = 0.20


class AdaptiveDiscoveryCadence:
    def __init__(self, config: CadenceConfig = CadenceConfig()):
        if config.every_n_frames < 1 or config.distance_trigger_m <= 0.0:
            raise ValueError("cadence frame and distance thresholds must be positive")
        self.config = config
        self.last_frame = -1
        self.last_distance_m = 0.0

    def should_discover(
        self,
        *,
        frame_index: int,
        cumulative_distance_m: float,
        scene_change: float,
        track_confidence_drop: float,
        new_region: bool,
    ) -> tuple[bool, str]:
        triggers = (
            (self.last_frame < 0, "initial"),
            (frame_index - self.last_frame >= self.config.every_n_frames, "fixed_frame"),
            (cumulative_distance_m - self.last_distance_m >= self.config.distance_trigger_m, "distance"),
            (scene_change >= self.config.scene_change_threshold, "scene_change"),
            (track_confidence_drop >= self.config.confidence_drop_threshold, "confidence_drop"),
            (new_region, "new_region"),
        )
        selected = next((reason for passed, reason in triggers if passed), None)
        if selected is None:
            return False, "tracking_only"
        self.last_frame = frame_index
        self.last_distance_m = cumulative_distance_m
        return True, selected
