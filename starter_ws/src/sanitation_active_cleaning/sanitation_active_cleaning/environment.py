"""RL-ready, URDF-independent active-cleaning environment.

The public observation contains only sensed/belief state. Hidden task truth is
available exclusively through an identity-checked evaluation token.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import Iterable, Protocol, Sequence

from .geometry import (
    distance,
    distance_segment_to_polygon_boundary,
    distance_to_segment,
    distance_to_polygon_boundary,
    distance_to_polygon,
    point_in_polygon,
    polyline_length,
    validate_ackermann_path,
    wrap_angle,
)
from .models import (
    AgentObservation,
    BeliefSnapshot,
    EvaluationSnapshot,
    KnownTarget,
    Point2D,
    Pose2D,
    RoleSeeds,
    StepResult,
    TaskConfig,
    TaskLayout,
)


class EvaluationToken:
    """Opaque capability for evaluation-only ground truth access."""

    __slots__ = ("_marker",)

    def __init__(self, marker: object):
        self._marker = marker


def create_evaluation_token() -> EvaluationToken:
    """Create a per-harness capability; policies never receive this object."""
    return EvaluationToken(object())


@dataclass(frozen=True)
class GraspVerificationResult:
    verified_in_bin: bool
    source: str = "external"


class GraspVerifier(Protocol):
    def __call__(
        self,
        target_id: str,
        target_position: Point2D,
        observation: AgentObservation,
    ) -> GraspVerificationResult: ...


@dataclass(frozen=True)
class TrajectoryAction:
    """High-level reference trajectory, never wheel or steering commands."""

    points: tuple[Pose2D, ...]
    clean_ground: bool = True
    grasp_target_ids: tuple[str, ...] = ()


@dataclass
class _TargetState:
    target_id: str
    x: float
    y: float
    cleared: bool = False
    attempts: int = 0


@dataclass
class _PedestrianState:
    x: float
    y: float
    yaw: float


class GridModel:
    def __init__(self, config: TaskConfig):
        xs = [point[0] for point in config.geofence]
        ys = [point[1] for point in config.geofence]
        self.origin = (min(xs), min(ys))
        self.resolution = config.grid_resolution
        self.width = max(1, int(math.ceil((max(xs) - min(xs)) / self.resolution)))
        self.height = max(1, int(math.ceil((max(ys) - min(ys)) / self.resolution)))
        self.centers: tuple[Point2D, ...] = tuple(
            (
                self.origin[0] + (column + 0.5) * self.resolution,
                self.origin[1] + (row + 0.5) * self.resolution,
            )
            for row in range(self.height)
            for column in range(self.width)
        )
        # Formal campuses contain more than one hundred static assets.  Testing
        # every grid centre against every polygon made a 200 m x 100 m map
        # needlessly quadratic.  The expanded AABB is an exact rejection test:
        # a polygon outside it cannot be within one vehicle radius of the point.
        obstacle_bounds = tuple(
            (
                min(point[0] for point in obstacle) - config.vehicle_radius,
                min(point[1] for point in obstacle) - config.vehicle_radius,
                max(point[0] for point in obstacle) + config.vehicle_radius,
                max(point[1] for point in obstacle) + config.vehicle_radius,
                obstacle,
            )
            for obstacle in config.static_obstacles
        )
        self.traversable = tuple(
            point_in_polygon(center, config.geofence)
            and distance_to_polygon_boundary(center, config.geofence)
            >= config.vehicle_radius
            and all(
                distance_to_polygon(center, obstacle) > config.vehicle_radius
                for minimum_x, minimum_y, maximum_x, maximum_y, obstacle in obstacle_bounds
                if minimum_x <= center[0] <= maximum_x
                and minimum_y <= center[1] <= maximum_y
            )
            for center in self.centers
        )
        if not any(self.traversable):
            raise ValueError("task has no traversable grid cells")
        self.free_indices = tuple(
            index for index, free in enumerate(self.traversable) if free
        )
        self._free_index_set = frozenset(self.free_indices)

    def nearest_index(self, point: Point2D) -> int:
        column = min(
            self.width - 1,
            max(0, int(math.floor((point[0] - self.origin[0]) / self.resolution))),
        )
        row = min(
            self.height - 1,
            max(0, int(math.floor((point[1] - self.origin[1]) / self.resolution))),
        )
        direct = row * self.width + column
        # Dirt and litter are generated on traversable cell centres, so this
        # constant-time path serves the overwhelmingly common product case.
        # Retain the exact global fallback for arbitrary external coordinates.
        if direct in self._free_index_set:
            return direct
        return min(
            self.free_indices,
            key=lambda index: distance(self.centers[index], point),
        )

    def indices_in_aabb(
        self, minimum_x: float, minimum_y: float, maximum_x: float, maximum_y: float
    ) -> Iterable[int]:
        first_column = max(0, int(math.floor((minimum_x - self.origin[0]) / self.resolution)))
        last_column = min(
            self.width - 1,
            int(math.floor((maximum_x - self.origin[0]) / self.resolution)),
        )
        first_row = max(0, int(math.floor((minimum_y - self.origin[1]) / self.resolution)))
        last_row = min(
            self.height - 1,
            int(math.floor((maximum_y - self.origin[1]) / self.resolution)),
        )
        if first_column > last_column or first_row > last_row:
            return ()
        return (
            row * self.width + column
            for row in range(first_row, last_row + 1)
            for column in range(first_column, last_column + 1)
        )

    def cells_within_path(self, path: Sequence[Pose2D], radius: float) -> set[int]:
        if not path:
            return set()
        segments = tuple(zip(path, path[1:])) or ((path[0], path[0]),)
        result: set[int] = set()
        for start, end in segments:
            candidates = self.indices_in_aabb(
                min(start.x, end.x) - radius,
                min(start.y, end.y) - radius,
                max(start.x, end.x) + radius,
                max(start.y, end.y) + radius,
            )
            for index in candidates:
                if self.traversable[index] and distance_to_segment(
                    self.centers[index], (start.x, start.y), (end.x, end.y)
                ) <= radius:
                    result.add(index)
        return result


class ActiveCleaningEnv:
    """A minimal reset/step API that intentionally does not depend on Gym."""

    def __init__(
        self,
        config: TaskConfig,
        *,
        evaluation_token: EvaluationToken | None = None,
        max_task_distance: float | None = None,
        task_layout: TaskLayout | None = None,
        grasp_verifier: GraspVerifier | None = None,
    ):
        config.validate()
        if task_layout is not None:
            task_layout.validate()
        self.config = config
        self.grid = GridModel(config)
        self._obstacle_bounds = tuple(
            (
                min(point[0] for point in obstacle),
                min(point[1] for point in obstacle),
                max(point[0] for point in obstacle),
                max(point[1] for point in obstacle),
                obstacle,
            )
            for obstacle in config.static_obstacles
        )
        self._evaluation_token = evaluation_token
        self._max_task_distance = max_task_distance
        self._task_layout = task_layout
        self._grasp_verifier = grasp_verifier
        self._layout_rng = random.Random()
        self._dynamics_rng = random.Random()
        self._grasp_rng = random.Random()
        self._seed = 0
        self._role_seeds = RoleSeeds.from_master(0)
        self._pose = config.start
        self._step_index = 0
        self._task_distance = 0.0
        self._observed = [False] * len(self.grid.centers)
        self._known_ground = [-1.0] * len(self.grid.centers)
        self._initial_ground: set[int] = set()
        self._remaining_ground: set[int] = set()
        self._targets: dict[str, _TargetState] = {}
        self._pedestrians: list[_PedestrianState] = []
        self._collisions = 0
        self._boundary_violations = 0
        self._invalid_actions = 0
        self._terminated = False
        self._truncated = False

    def reset(self, *, seed: int) -> AgentObservation:
        self._seed = int(seed)
        self._role_seeds = RoleSeeds.from_master(self._seed)
        self._layout_rng.seed(self._role_seeds.layout)
        self._dynamics_rng.seed(self._role_seeds.dynamics)
        self._grasp_rng.seed(self._role_seeds.grasp)
        self._pose = self.config.start
        if not self._point_is_free((self._pose.x, self._pose.y), include_pedestrians=False):
            raise ValueError("start pose is outside the free geofence")
        self._step_index = 0
        self._task_distance = 0.0
        self._observed = [False] * len(self.grid.centers)
        self._known_ground = [-1.0] * len(self.grid.centers)
        self._initial_ground = self._generate_ground_dirt()
        self._remaining_ground = set(self._initial_ground)
        self._targets = self._generate_targets()
        self._pedestrians = self._generate_pedestrians()
        self._collisions = 0
        self._boundary_violations = 0
        self._invalid_actions = 0
        self._terminated = False
        self._truncated = False
        self._observe_along((self._pose,))
        self._terminated = self._belief_terminal()
        return self.observation()

    def _random_free_index(self, *, minimum_start_distance: float = 0.0) -> int:
        candidates = [
            index
            for index, free in enumerate(self.grid.traversable)
            if free
            and distance(self.grid.centers[index], (self.config.start.x, self.config.start.y))
            >= minimum_start_distance
        ]
        if not candidates:
            raise ValueError("no free placement candidate satisfies the task geometry")
        return self._layout_rng.choice(candidates)

    def _generate_ground_dirt(self) -> set[int]:
        if self._task_layout is not None:
            dirty = {
                index
                for x, y, radius in self._task_layout.ground_dirt_regions
                for index in self.grid.indices_in_aabb(
                    x - radius, y - radius, x + radius, y + radius
                )
                if self.grid.traversable[index]
                and distance((x, y), self.grid.centers[index]) <= radius
            }
            dirty.update(
                index
                for polygon in self._task_layout.ground_dirt_polygons
                for index in self.grid.indices_in_aabb(
                    min(point[0] for point in polygon),
                    min(point[1] for point in polygon),
                    max(point[0] for point in polygon),
                    max(point[1] for point in polygon),
                )
                if self.grid.traversable[index]
                and point_in_polygon(self.grid.centers[index], polygon)
            )
            if (
                self._task_layout.ground_dirt_regions
                or self._task_layout.ground_dirt_polygons
            ) and not dirty:
                raise ValueError("explicit ground dirt does not intersect a traversable cell")
            return dirty
        dirty: set[int] = set()
        for _ in range(self.config.ground_dirt_count):
            center = self.grid.centers[self._random_free_index(minimum_start_distance=0.5)]
            radius = self._layout_rng.uniform(*self.config.ground_dirt_radius_range)
            dirty.update(
                index
                for index in self.grid.indices_in_aabb(
                    center[0] - radius,
                    center[1] - radius,
                    center[0] + radius,
                    center[1] + radius,
                )
                if self.grid.traversable[index]
                and distance(center, self.grid.centers[index]) <= radius
            )
        return dirty

    def _generate_targets(self) -> dict[str, _TargetState]:
        if self._task_layout is not None:
            result = {}
            for target_id, x, y in self._task_layout.discrete_targets:
                if not self._point_is_free((x, y), include_pedestrians=False):
                    raise ValueError(f"explicit target {target_id!r} is outside free space")
                result[target_id] = _TargetState(target_id, x, y)
            return result
        result: dict[str, _TargetState] = {}
        used: set[int] = set()
        for index in range(self.config.discrete_target_count):
            for _ in range(1000):
                cell = self._random_free_index(minimum_start_distance=0.8)
                if cell not in used:
                    used.add(cell)
                    x, y = self.grid.centers[cell]
                    result[f"trash_{index:03d}"] = _TargetState(f"trash_{index:03d}", x, y)
                    break
            else:
                raise ValueError("not enough distinct free cells for discrete targets")
        return result

    def _generate_pedestrians(self) -> list[_PedestrianState]:
        if self._task_layout is not None:
            result = []
            for x, y, yaw in self._task_layout.pedestrians:
                if not self._point_is_free((x, y), include_pedestrians=False):
                    raise ValueError("explicit pedestrian is outside free space")
                result.append(_PedestrianState(x, y, yaw))
            return result
        result = []
        for _ in range(self.config.pedestrian_count):
            index = self._random_free_index(minimum_start_distance=1.0)
            x, y = self.grid.centers[index]
            result.append(
                _PedestrianState(
                    x, y, self._dynamics_rng.uniform(-math.pi, math.pi)
                )
            )
        return result

    def observation(self) -> AgentObservation:
        known_targets = tuple(
            KnownTarget(target.target_id, target.x, target.y, target.cleared, target.attempts)
            for target in sorted(self._targets.values(), key=lambda item: item.target_id)
            if self._observed[self.grid.nearest_index((target.x, target.y))]
        )
        traversable_count = sum(self.grid.traversable)
        observed_count = sum(
            observed and free for observed, free in zip(self._observed, self.grid.traversable)
        )
        remaining_budget = None
        if self._max_task_distance is not None:
            remaining_budget = max(0.0, self._max_task_distance - self._task_distance)
        belief = BeliefSnapshot(
            width=self.grid.width,
            height=self.grid.height,
            origin=self.grid.origin,
            resolution=self.grid.resolution,
            traversable=self.grid.traversable,
            observed=tuple(self._observed),
            known_ground_dirt=tuple(self._known_ground),
            known_targets=known_targets,
        )
        return AgentObservation(
            step_index=self._step_index,
            pose=self._pose,
            observed_ratio=observed_count / traversable_count,
            belief=belief,
            static_obstacles=self.config.static_obstacles,
            current_pedestrians=tuple(
                (pedestrian.x, pedestrian.y, self.config.pedestrian.radius)
                for pedestrian in self._pedestrians
            ),
            task_distance=self._task_distance,
            remaining_distance_budget=remaining_budget,
        )

    def _line_of_sight(self, start: Point2D, end: Point2D) -> bool:
        # Exact segment/edge intersection is substantially cheaper than
        # sampling every ray at half-grid spacing, and it cannot skip a thin
        # pole or wall between samples.
        ray_min_x, ray_max_x = sorted((start[0], end[0]))
        ray_min_y, ray_max_y = sorted((start[1], end[1]))
        candidates = (
            obstacle
            for min_x, min_y, max_x, max_y, obstacle in self._obstacle_bounds
            if max_x >= ray_min_x
            and min_x <= ray_max_x
            and max_y >= ray_min_y
            and min_y <= ray_max_y
        )
        return not any(
            point_in_polygon(start, obstacle)
            or point_in_polygon(end, obstacle)
            or distance_segment_to_polygon_boundary(start, end, obstacle)
            <= 1.0e-9
            for obstacle in candidates
        )

    def _observe_along(self, path: Sequence[Pose2D]) -> int:
        before = sum(self._observed)
        # Trajectories are sampled densely for collision checking. Repeating a
        # full ray-cast at every 0.1-0.2 m control sample is both redundant and
        # prohibitively expensive on the 106 x 53 m / 200 x 100 m fields. A
        # sensing sample every half radius still overlaps adjacent 360-degree
        # footprints; the final pose is always retained.
        sensing_spacing = max(
            self.grid.resolution * 0.5,
            self.config.sensing_radius * 0.5,
        )
        sensor_path: list[Pose2D] = []
        accumulated = sensing_spacing
        previous: Pose2D | None = None
        for pose in path:
            if previous is not None:
                accumulated += distance(
                    (previous.x, previous.y), (pose.x, pose.y)
                )
            if not sensor_path or accumulated >= sensing_spacing:
                sensor_path.append(pose)
                accumulated = 0.0
            previous = pose
        if path and sensor_path[-1] != path[-1]:
            sensor_path.append(path[-1])
        for pose in sensor_path:
            for index in self.grid.indices_in_aabb(
                pose.x - self.config.sensing_radius,
                pose.y - self.config.sensing_radius,
                pose.x + self.config.sensing_radius,
                pose.y + self.config.sensing_radius,
            ):
                center = self.grid.centers[index]
                if not self.grid.traversable[index] or self._observed[index]:
                    continue
                target_distance = distance((pose.x, pose.y), center)
                if target_distance > self.config.sensing_radius:
                    continue
                bearing = math.atan2(center[1] - pose.y, center[0] - pose.x)
                if abs(wrap_angle(bearing - pose.yaw)) > self.config.sensing_fov_rad * 0.5:
                    continue
                if self._line_of_sight((pose.x, pose.y), center):
                    self._observed[index] = True
        for index, observed in enumerate(self._observed):
            if observed:
                self._known_ground[index] = 1.0 if index in self._remaining_ground else 0.0
        return sum(self._observed) - before

    def _point_is_free(self, point: Point2D, *, include_pedestrians: bool = True) -> bool:
        if not point_in_polygon(point, self.config.geofence):
            return False
        if distance_to_polygon_boundary(point, self.config.geofence) < self.config.vehicle_radius:
            return False
        if any(
            distance_to_polygon(point, obstacle) <= self.config.vehicle_radius
            for obstacle in self.config.static_obstacles
        ):
            return False
        if include_pedestrians and any(
            distance(point, (pedestrian.x, pedestrian.y))
            <= self.config.vehicle_radius + self.config.pedestrian.radius
            for pedestrian in self._pedestrians
        ):
            return False
        return True

    def _validate_action(self, action: TrajectoryAction) -> tuple[bool, str, float]:
        path = action.points
        if len(action.grasp_target_ids) > 1:
            return False, "multiple_grasp_targets_forbidden", 0.0
        if path:
            if distance((path[0].x, path[0].y), (self._pose.x, self._pose.y)) > self.config.path_sample_spacing:
                return False, "trajectory_does_not_start_at_current_pose", 0.0
            if abs(wrap_angle(path[0].yaw - self._pose.yaw)) > 0.18:
                return False, "trajectory_start_heading_mismatch", 0.0
        valid, reason = validate_ackermann_path(
            path,
            max_curvature=1.0 / self.config.min_turn_radius,
        )
        if not valid:
            return False, reason, 0.0
        maximum_segment = self.config.path_sample_spacing * (1.0 + 1.0e-6)
        if any(
            distance((previous.x, previous.y), (current.x, current.y))
            > maximum_segment
            for previous, current in zip(path, path[1:])
        ):
            return False, "trajectory_segment_spacing_exceeded", 0.0
        length = polyline_length(path)
        if self._max_task_distance is not None and self._task_distance + length > self._max_task_distance + 1.0e-9:
            return False, "full_coverage_distance_budget_exceeded", length
        checked_path = path or (self._pose,)
        segments = tuple(zip(checked_path, checked_path[1:])) or (
            (checked_path[0], checked_path[0]),
        )
        for start_pose, end_pose in segments:
            start = (start_pose.x, start_pose.y)
            end = (end_pose.x, end_pose.y)
            if (
                not point_in_polygon(start, self.config.geofence)
                or not point_in_polygon(end, self.config.geofence)
                or distance_segment_to_polygon_boundary(
                    start, end, self.config.geofence
                )
                < self.config.vehicle_radius
            ):
                return False, "trajectory_intersects_geofence_boundary", length
            padded = self.config.vehicle_radius
            segment_min_x, segment_max_x = sorted((start[0], end[0]))
            segment_min_y, segment_max_y = sorted((start[1], end[1]))
            candidate_obstacles = (
                obstacle
                for min_x, min_y, max_x, max_y, obstacle in self._obstacle_bounds
                if max_x + padded >= segment_min_x
                and min_x - padded <= segment_max_x
                and max_y + padded >= segment_min_y
                and min_y - padded <= segment_max_y
            )
            for obstacle in candidate_obstacles:
                if (
                    point_in_polygon(start, obstacle)
                    or point_in_polygon(end, obstacle)
                    or distance_segment_to_polygon_boundary(start, end, obstacle)
                    <= self.config.vehicle_radius
                ):
                    return False, "trajectory_intersects_static_obstacle", length
            if any(
                distance_to_segment(
                    (pedestrian.x, pedestrian.y), start, end
                )
                <= self.config.vehicle_radius + self.config.pedestrian.radius
                for pedestrian in self._pedestrians
            ):
                return False, "trajectory_intersects_current_pedestrian", length
        return True, "ok", length

    def _advance_pedestrians(self) -> None:
        for pedestrian in self._pedestrians:
            step = self.config.pedestrian.step_distance
            proposal = (
                pedestrian.x + step * math.cos(pedestrian.yaw),
                pedestrian.y + step * math.sin(pedestrian.yaw),
            )
            # Dynamic actors must not be advanced into the robot after an
            # otherwise valid action.  The old update checked the geofence and
            # static assets only; a pedestrian could therefore enter the
            # stationary vehicle footprint between decisions and make the
            # next trajectory invalid at its mandatory start point.  Treat
            # the live chassis as an obstacle for pedestrian motion.
            clears_vehicle = distance(
                proposal,
                (self._pose.x, self._pose.y),
            ) > self.config.vehicle_radius + self.config.pedestrian.radius
            if clears_vehicle and self._point_is_free(
                proposal,
                include_pedestrians=False,
            ):
                pedestrian.x, pedestrian.y = proposal
            else:
                pedestrian.yaw = wrap_angle(pedestrian.yaw + math.pi)

    def _belief_terminal(self) -> bool:
        observation = self.observation()
        observed_ground = {
            index for index in self._initial_ground if self._observed[index]
        }
        cleared_ground = observed_ground.difference(self._remaining_ground)
        ground_ratio = (
            1.0 if not observed_ground else len(cleared_ground) / len(observed_ground)
        )
        observed_targets = tuple(
            target
            for target in self._targets.values()
            if self._observed[self.grid.nearest_index((target.x, target.y))]
        )
        cleared_targets = sum(target.cleared for target in observed_targets)
        target_ratio = (
            1.0 if not observed_targets else cleared_targets / len(observed_targets)
        )
        return (
            observation.observed_ratio >= self.config.observation_threshold
            and ground_ratio >= self.config.ground_clear_threshold
            and target_ratio >= self.config.discrete_clear_threshold
        )

    def _belief_unrecoverable(self) -> bool:
        observation = self.observation()
        if observation.observed_ratio < self.config.observation_threshold:
            return False
        observed_targets = tuple(
            target
            for target in self._targets.values()
            if self._observed[self.grid.nearest_index((target.x, target.y))]
        )
        if not observed_targets:
            return False
        maximum_clearable = sum(
            target.cleared or target.attempts < self.config.max_grasp_attempts
            for target in observed_targets
        )
        return (
            maximum_clearable / len(observed_targets)
            < self.config.discrete_clear_threshold
        )

    def step(self, action: TrajectoryAction) -> StepResult:
        if self._terminated or self._truncated:
            raise RuntimeError("step called after terminal state; call reset")
        valid, reason, length = self._validate_action(action)
        self._step_index += 1
        if not valid:
            self._invalid_actions += 1
            if reason == "trajectory_intersects_geofence_boundary":
                self._boundary_violations += 1
            elif reason in {
                "trajectory_intersects_static_obstacle",
                "trajectory_intersects_current_pedestrian",
            }:
                self._collisions += 1
            self._advance_pedestrians()
            if reason == "full_coverage_distance_budget_exceeded":
                self._truncated = True
            elif self._step_index >= self.config.max_steps:
                self._truncated = True
            return StepResult(
                self.observation(),
                -10.0,
                self._terminated,
                self._truncated,
                {"accepted": False, "reason": reason, "executed_distance": 0.0},
            )

        path = action.points or (self._pose,)
        newly_observed = self._observe_along(path)
        cleared_ground = 0
        if action.clean_ground:
            swept = self.grid.cells_within_path(
                path, self.config.cleaning_width * 0.5
            )
            cleared = self._remaining_ground.intersection(swept)
            cleared_ground = len(cleared)
            self._remaining_ground.difference_update(cleared)
            for index in cleared:
                if self._observed[index]:
                    self._known_ground[index] = 0.0

        cleared_targets = 0
        final_pose = path[-1]
        self._pose = final_pose
        known_ids = {target.target_id for target in self.observation().belief.known_targets}
        grasp_verification: GraspVerificationResult | None = None
        for target_id in action.grasp_target_ids:
            target = self._targets.get(target_id)
            if target is None or target.cleared or target_id not in known_ids:
                continue
            if target.attempts >= self.config.max_grasp_attempts:
                continue
            if distance((final_pose.x, final_pose.y), (target.x, target.y)) > self.config.grasp_radius:
                continue
            target.attempts += 1
            if self._grasp_verifier is None:
                grasp_verification = GraspVerificationResult(
                    verified_in_bin=(
                        self._grasp_rng.random()
                        <= self.config.grasp_success_probability
                    ),
                    source="simulated_probability",
                )
            else:
                grasp_verification = self._grasp_verifier(
                    target.target_id,
                    (target.x, target.y),
                    self.observation(),
                )
                if not isinstance(grasp_verification, GraspVerificationResult):
                    raise TypeError(
                        "grasp_verifier must return GraspVerificationResult"
                    )
            if grasp_verification.verified_in_bin:
                target.cleared = True
                cleared_targets += 1

        self._task_distance += length
        self._advance_pedestrians()
        self._terminated = self._belief_terminal()
        if not self._terminated and self._belief_unrecoverable():
            self._truncated = True
        elif self._step_index >= self.config.max_steps and not self._terminated:
            self._truncated = True
        reward = (
            newly_observed / max(1, sum(self.grid.traversable))
            + cleared_ground / max(1, len(self._initial_ground))
            + float(cleared_targets)
            - 0.01 * length
        )
        return StepResult(
            self.observation(),
            reward,
            self._terminated,
            self._truncated,
            {
                "accepted": True,
                "reason": "ok",
                "executed_distance": length,
                "newly_observed_cells": newly_observed,
                "cleared_ground_cells": cleared_ground,
                "cleared_target_count": cleared_targets,
                "grasp_verification_source": (
                    None if grasp_verification is None else grasp_verification.source
                ),
                "grasp_verified_in_bin": (
                    None
                    if grasp_verification is None
                    else grasp_verification.verified_in_bin
                ),
            },
        )

    def evaluation_snapshot(self, token: EvaluationToken) -> EvaluationSnapshot:
        if self._evaluation_token is None or token is not self._evaluation_token:
            raise PermissionError("ground truth is evaluation-only")
        return EvaluationSnapshot(
            seed=self._seed,
            role_seeds=self._role_seeds,
            step_index=self._step_index,
            initial_ground_dirt_cells=frozenset(self._initial_ground),
            remaining_ground_dirt_cells=frozenset(self._remaining_ground),
            initial_targets=tuple(
                (target.target_id, target.x, target.y)
                for target in sorted(self._targets.values(), key=lambda item: item.target_id)
            ),
            cleared_target_ids=frozenset(
                target.target_id for target in self._targets.values() if target.cleared
            ),
            observed=tuple(self._observed),
            task_distance=self._task_distance,
            collisions=self._collisions,
            boundary_violations=self._boundary_violations,
            invalid_actions=self._invalid_actions,
            terminated=self._terminated,
            truncated=self._truncated,
            grasp_verification_mode=(
                "simulated_probability"
                if self._grasp_verifier is None
                else "external_callback"
            ),
        )
