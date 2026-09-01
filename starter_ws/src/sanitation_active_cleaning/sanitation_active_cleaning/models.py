"""Immutable task, observation, and report models."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping


Point2D = tuple[float, float]
Polygon2D = tuple[Point2D, ...]


_ROLE_CONSTANTS = {
    "layout": 0x9E3779B97F4A7C15,
    "dynamics": 0xD1B54A32D192ED03,
    "grasp": 0x94D049BB133111EB,
    "policy": 0xBF58476D1CE4E5B9,
    "policy_rng": 0x632BE59BD9B4E019,
}


def derive_role_seed(master_seed: int, role: str) -> int:
    """Stable SplitMix64 derivation so stochastic roles never share a stream."""
    if role not in _ROLE_CONSTANTS:
        raise ValueError(f"unknown seed role: {role}")
    value = (int(master_seed) ^ _ROLE_CONSTANTS[role]) & 0xFFFFFFFFFFFFFFFF
    value = (value ^ (value >> 30)) * 0xBF58476D1CE4E5B9 & 0xFFFFFFFFFFFFFFFF
    value = (value ^ (value >> 27)) * 0x94D049BB133111EB & 0xFFFFFFFFFFFFFFFF
    return (value ^ (value >> 31)) & 0x7FFFFFFFFFFFFFFF


@dataclass(frozen=True)
class RoleSeeds:
    layout: int
    dynamics: int
    grasp: int
    policy: int

    @classmethod
    def from_master(cls, master_seed: int) -> "RoleSeeds":
        return cls(
            layout=derive_role_seed(master_seed, "layout"),
            dynamics=derive_role_seed(master_seed, "dynamics"),
            grasp=derive_role_seed(master_seed, "grasp"),
            policy=derive_role_seed(master_seed, "policy"),
        )

    def as_mapping(self) -> Mapping[str, int]:
        return {
            "layout": self.layout,
            "dynamics": self.dynamics,
            "grasp": self.grasp,
            "policy": self.policy,
        }


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class PedestrianSpec:
    radius: float = 0.35
    step_distance: float = 0.20


@dataclass(frozen=True)
class TaskLayout:
    """Evaluation-harness task truth, deliberately absent from observations."""

    ground_dirt_regions: tuple[tuple[float, float, float], ...] = ()
    ground_dirt_polygons: tuple[Polygon2D, ...] = ()
    discrete_targets: tuple[tuple[str, float, float], ...] = ()
    pedestrians: tuple[tuple[float, float, float], ...] = ()

    def validate(self) -> None:
        if any(radius <= 0.0 for _, _, radius in self.ground_dirt_regions):
            raise ValueError("explicit ground dirt radii must be positive")
        if any(len(polygon) < 3 for polygon in self.ground_dirt_polygons):
            raise ValueError("explicit ground dirt polygons need at least three points")
        identifiers = [target_id for target_id, _, _ in self.discrete_targets]
        if any(not target_id for target_id in identifiers) or len(set(identifiers)) != len(identifiers):
            raise ValueError("explicit target identifiers must be non-empty and unique")


@dataclass(frozen=True)
class TaskConfig:
    geofence: Polygon2D
    static_obstacles: tuple[Polygon2D, ...]
    start: Pose2D
    grid_resolution: float
    sensing_radius: float
    sensing_fov_rad: float
    cleaning_width: float
    vehicle_radius: float
    grasp_radius: float
    min_turn_radius: float
    path_sample_spacing: float
    observation_threshold: float
    ground_clear_threshold: float
    discrete_clear_threshold: float
    ground_dirt_count: int
    ground_dirt_radius_range: tuple[float, float]
    discrete_target_count: int
    pedestrian_count: int
    pedestrian: PedestrianSpec
    max_grasp_attempts: int
    grasp_success_probability: float
    max_steps: int

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "TaskConfig":
        def polygon(raw: Any) -> Polygon2D:
            return tuple((float(point[0]), float(point[1])) for point in raw)

        start = data.get("start", {})
        dirt_range = data.get("ground_dirt_radius_range", [0.3, 0.8])
        pedestrian = data.get("pedestrian", {})
        config = cls(
            geofence=polygon(data["geofence"]),
            static_obstacles=tuple(polygon(item) for item in data.get("static_obstacles", [])),
            start=Pose2D(
                float(start.get("x", 0.0)),
                float(start.get("y", 0.0)),
                float(start.get("yaw", 0.0)),
            ),
            grid_resolution=float(data.get("grid_resolution", 0.25)),
            sensing_radius=float(data.get("sensing_radius", 3.0)),
            sensing_fov_rad=float(data.get("sensing_fov_rad", math.pi)),
            cleaning_width=float(data.get("cleaning_width", 0.70)),
            vehicle_radius=float(data.get("vehicle_radius", 0.30)),
            grasp_radius=float(data.get("grasp_radius", 0.45)),
            min_turn_radius=float(data.get("min_turn_radius", 0.30)),
            path_sample_spacing=float(data.get("path_sample_spacing", 0.10)),
            observation_threshold=float(data.get("observation_threshold", 0.95)),
            ground_clear_threshold=float(data.get("ground_clear_threshold", 0.95)),
            discrete_clear_threshold=float(data.get("discrete_clear_threshold", 0.95)),
            ground_dirt_count=int(data.get("ground_dirt_count", 12)),
            ground_dirt_radius_range=(float(dirt_range[0]), float(dirt_range[1])),
            discrete_target_count=int(data.get("discrete_target_count", 10)),
            pedestrian_count=int(data.get("pedestrian_count", 3)),
            pedestrian=PedestrianSpec(
                radius=float(pedestrian.get("radius", 0.35)),
                step_distance=float(pedestrian.get("step_distance", 0.20)),
            ),
            max_grasp_attempts=int(data.get("max_grasp_attempts", 2)),
            grasp_success_probability=float(data.get("grasp_success_probability", 1.0)),
            max_steps=int(data.get("max_steps", 2000)),
        )
        config.validate()
        return config

    @classmethod
    def from_json(cls, path: str | Path) -> "TaskConfig":
        return cls.from_mapping(json.loads(Path(path).read_text(encoding="utf-8")))

    def validate(self) -> None:
        if len(self.geofence) < 3:
            raise ValueError("geofence must contain at least three points")
        if any(len(item) < 3 for item in self.static_obstacles):
            raise ValueError("each static obstacle must contain at least three points")
        positive = {
            "grid_resolution": self.grid_resolution,
            "sensing_radius": self.sensing_radius,
            "cleaning_width": self.cleaning_width,
            "vehicle_radius": self.vehicle_radius,
            "grasp_radius": self.grasp_radius,
            "min_turn_radius": self.min_turn_radius,
            "path_sample_spacing": self.path_sample_spacing,
        }
        for name, value in positive.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if not (0.0 < self.sensing_fov_rad <= 2.0 * math.pi):
            raise ValueError("sensing_fov_rad must be in (0, 2*pi]")
        for name, value in (
            ("observation_threshold", self.observation_threshold),
            ("ground_clear_threshold", self.ground_clear_threshold),
            ("discrete_clear_threshold", self.discrete_clear_threshold),
            ("grasp_success_probability", self.grasp_success_probability),
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        if self.ground_dirt_radius_range[0] <= 0.0 or self.ground_dirt_radius_range[1] < self.ground_dirt_radius_range[0]:
            raise ValueError("ground_dirt_radius_range is invalid")
        if min(self.ground_dirt_count, self.discrete_target_count, self.pedestrian_count) < 0:
            raise ValueError("random object counts cannot be negative")
        if self.max_grasp_attempts <= 0 or self.max_steps <= 0:
            raise ValueError("attempt and step limits must be positive")


@dataclass(frozen=True)
class KnownTarget:
    target_id: str
    x: float
    y: float
    cleared: bool
    attempts: int


@dataclass(frozen=True)
class BeliefSnapshot:
    width: int
    height: int
    origin: Point2D
    resolution: float
    traversable: tuple[bool, ...]
    observed: tuple[bool, ...]
    known_ground_dirt: tuple[float, ...]
    known_targets: tuple[KnownTarget, ...]


@dataclass(frozen=True)
class AgentObservation:
    step_index: int
    pose: Pose2D
    observed_ratio: float
    belief: BeliefSnapshot
    static_obstacles: tuple[Polygon2D, ...]
    current_pedestrians: tuple[tuple[float, float, float], ...]
    task_distance: float
    remaining_distance_budget: float | None


@dataclass(frozen=True)
class StepResult:
    observation: AgentObservation
    reward: float
    terminated: bool
    truncated: bool
    info: Mapping[str, Any]


@dataclass(frozen=True)
class EvaluationSnapshot:
    seed: int
    role_seeds: RoleSeeds
    step_index: int
    initial_ground_dirt_cells: frozenset[int]
    remaining_ground_dirt_cells: frozenset[int]
    initial_targets: tuple[tuple[str, float, float], ...]
    cleared_target_ids: frozenset[str]
    observed: tuple[bool, ...]
    task_distance: float
    collisions: int
    boundary_violations: int
    invalid_actions: int
    terminated: bool
    truncated: bool
    grasp_verification_mode: str
