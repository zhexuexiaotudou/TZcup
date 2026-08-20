"""Prediction-only temporal, geometric, and re-observation evidence policy."""

from __future__ import annotations

from dataclasses import dataclass, field
import math


CLASSES = ("plastic_bottle", "metal_can", "paper_litter", "background")


@dataclass(frozen=True)
class TemporalGeometryConfig:
    observation_threshold: float = 0.05
    confirmation_probability: float = 0.95
    minimum_observations: int = 3
    maximum_map_scatter_m: float = 0.20
    maximum_reobserve_count: int = 2
    maximum_action_distance_m: float = 4.0
    minimum_depth_valid_ratio: float = 0.80
    minimum_short_side_for_action_px: int = 12
    log_evidence_floor: float = 1e-6
    policy: str = "weighted_log_probability"


@dataclass
class TemporalGeometryTrack:
    config: TemporalGeometryConfig
    log_evidence: dict[str, float] = field(default_factory=lambda: {name: 0.0 for name in CLASSES})
    positions_xy: list[tuple[float, float]] = field(default_factory=list)
    observation_count: int = 0
    reobserve_count: int = 0
    state: str = "UNKNOWN"
    final_class: str = "UNKNOWN"

    def posterior(self) -> dict[str, float]:
        peak = max(self.log_evidence.values())
        weights = {name: math.exp(max(-60.0, value - peak)) for name, value in self.log_evidence.items()}
        total = sum(weights.values())
        return {name: value / total for name, value in weights.items()}

    def map_scatter_m(self) -> float:
        if len(self.positions_xy) < 2:
            return 0.0
        mean_x = sum(point[0] for point in self.positions_xy) / len(self.positions_xy)
        mean_y = sum(point[1] for point in self.positions_xy) / len(self.positions_xy)
        return max(math.hypot(x - mean_x, y - mean_y) for x, y in self.positions_xy)

    def update(self, observation: dict) -> str:
        probabilities = {name: max(self.config.log_evidence_floor, float(observation.get("class_probabilities", {}).get(name, self.config.log_evidence_floor))) for name in CLASSES}
        total = sum(probabilities.values())
        probabilities = {name: value / total for name, value in probabilities.items()}
        # Far/tiny observations contribute discovery evidence but are not
        # allowed to dominate the eventual action class.
        distance = float(observation.get("distance_m", math.inf))
        short_side = int(observation.get("short_side_px", 0))
        weight = 0.35 if distance > self.config.maximum_action_distance_m or short_side < self.config.minimum_short_side_for_action_px else 1.0
        for name, value in probabilities.items():
            self.log_evidence[name] += weight * math.log(value)
        if observation.get("map_xy_m") is not None:
            self.positions_xy.append(tuple(float(value) for value in observation["map_xy_m"]))
        self.observation_count += 1
        posterior = self.posterior()
        candidate = max((name for name in CLASSES if name != "background"), key=posterior.get)
        geometry_healthy = (
            float(observation.get("depth_valid_ratio", 0.0)) >= self.config.minimum_depth_valid_ratio
            and distance <= self.config.maximum_action_distance_m
            and short_side >= self.config.minimum_short_side_for_action_px
            and self.map_scatter_m() <= self.config.maximum_map_scatter_m
            and bool(observation.get("physical_plausible", True))
        )
        if self.observation_count >= self.config.minimum_observations and posterior[candidate] >= self.config.confirmation_probability and geometry_healthy:
            self.state, self.final_class = "CONFIRMED", candidate
        elif self.observation_count >= self.config.minimum_observations and posterior.get("background", 0.0) >= self.config.confirmation_probability:
            self.state, self.final_class = "REJECTED", "UNKNOWN"
        elif bool(observation.get("candidate_observed")) and self.reobserve_count < self.config.maximum_reobserve_count:
            self.reobserve_count += 1
            self.state, self.final_class = "OBSERVE_AGAIN", "UNKNOWN"
        else:
            self.state, self.final_class = "UNKNOWN", "UNKNOWN"
        return self.state

    @property
    def clean_action_allowed(self) -> bool:
        return self.state == "CONFIRMED" and self.final_class != "UNKNOWN"
