"""Deterministic high-level trajectory policies used as RL baselines."""

from __future__ import annotations

import heapq
import itertools
import math
from typing import Iterable, Sequence

from .environment import TrajectoryAction
from .geometry import (
    ackermann_path_to_point,
    distance,
    distance_segment_to_polygon_boundary,
    distance_to_segment,
    point_in_polygon,
    wrap_angle,
)
from .models import AgentObservation, EvaluationSnapshot, Point2D, Pose2D, TaskConfig


def cell_center(observation: AgentObservation, index: int) -> Point2D:
    belief = observation.belief
    row, column = divmod(index, belief.width)
    return (
        belief.origin[0] + (column + 0.5) * belief.resolution,
        belief.origin[1] + (row + 0.5) * belief.resolution,
    )


class TrajectoryPolicy:
    name = "base"
    evaluation_only = False

    def reset(self, *, episode_seed: int | None = None) -> None:
        pass

    def act(self, observation: AgentObservation) -> TrajectoryAction:
        raise NotImplementedError


class _SafeTrajectoryMixin:
    _MAX_GOAL_CANDIDATES = 24

    def __init__(self, config: TaskConfig):
        self.config = config
        self._inspection_route: list[Point2D] = []
        self._inspection_cursor = 0
        padded = config.vehicle_radius
        self._obstacle_bounds = tuple(
            (
                min(point[0] for point in obstacle) - padded,
                min(point[1] for point in obstacle) - padded,
                max(point[0] for point in obstacle) + padded,
                max(point[1] for point in obstacle) + padded,
                obstacle,
            )
            for obstacle in config.static_obstacles
        )

    def _reset_inspection_route(self) -> None:
        self._inspection_route = []
        self._inspection_cursor = 0

    def _build_inspection_route(self, observation: AgentObservation) -> None:
        """Build a sensor-footprint sweep, distinct from brush coverage.

        The task requires at least 95% of traversable area to be observed, not
        brushed blindly.  Rows are therefore spaced from the declared sensing
        radius; detected dirt is serviced separately with the cleaning width.
        """
        belief = observation.belief
        row_stride = max(
            1,
            int(math.floor(self.config.sensing_radius * 1.20 / belief.resolution)),
        )
        column_stride = max(
            1,
            int(math.floor(self.config.sensing_radius * 0.80 / belief.resolution)),
        )
        rows: list[list[Point2D]] = []
        for row in range(0, belief.height, row_stride):
            runs: list[list[Point2D]] = []
            current: list[Point2D] = []
            for column in range(belief.width):
                index = row * belief.width + column
                if belief.traversable[index]:
                    current.append(cell_center(observation, index))
                elif current:
                    runs.append(current)
                    current = []
            if current:
                runs.append(current)
            points: list[Point2D] = []
            for run in runs:
                # Include local waypoints within a long free run instead of
                # only its two endpoints.  Endpoint-only segments crossed
                # distant furniture/pedestrians and forced an expensive
                # Hybrid-A* search almost every Q step.  Consecutive sensing-
                # scale waypoints remain global reference trajectories while
                # making the belief sweep directly executable.
                sampled = run[::column_stride]
                points.extend(sampled)
                if sampled[-1] != run[-1]:
                    points.append(run[-1])
            if points:
                rows.append(points)
        route: list[Point2D] = []
        for row_index, points in enumerate(rows):
            route.extend(points if row_index % 2 == 0 else reversed(points))
        if route:
            nearest = min(
                range(len(route)),
                key=lambda index: distance(
                    (observation.pose.x, observation.pose.y), route[index]
                ),
            )
            route = route[nearest:] + route[:nearest]
        self._inspection_route = route
        self._inspection_cursor = 0

    def _inspection_action(
        self,
        observation: AgentObservation,
        *,
        allow_hybrid: bool = True,
    ) -> TrajectoryAction | None:
        if not self._inspection_route:
            self._build_inspection_route(observation)
        attempts_remaining = len(self._inspection_route) - self._inspection_cursor
        while (
            self._inspection_cursor < len(self._inspection_route)
            and attempts_remaining > 0
        ):
            goal = self._inspection_route[self._inspection_cursor]
            if distance(
                (observation.pose.x, observation.pose.y), goal
            ) <= observation.belief.resolution:
                self._inspection_cursor += 1
                continue
            action = self._trajectory_to(
                observation,
                goal,
                clean=False,
                allow_hybrid=allow_hybrid,
            )
            if action is not None:
                self._inspection_cursor += 1
                return action
            self._inspection_route.append(
                self._inspection_route.pop(self._inspection_cursor)
            )
            attempts_remaining -= 1
        return None

    def _path_is_safe(self, path: Sequence[Pose2D], observation: AgentObservation) -> bool:
        if not path:
            return True
        segments = tuple(zip(path, path[1:])) or ((path[0], path[0]),)
        for start_pose, end_pose in segments:
            start = (start_pose.x, start_pose.y)
            end = (end_pose.x, end_pose.y)
            if not point_in_polygon(start, self.config.geofence) or not point_in_polygon(
                end, self.config.geofence
            ):
                return False
            if distance_segment_to_polygon_boundary(
                start, end, self.config.geofence
            ) < self.config.vehicle_radius:
                return False
            segment_min_x, segment_max_x = sorted((start[0], end[0]))
            segment_min_y, segment_max_y = sorted((start[1], end[1]))
            candidate_obstacles = (
                obstacle
                for minimum_x, minimum_y, maximum_x, maximum_y, obstacle in self._obstacle_bounds
                if maximum_x >= segment_min_x
                and minimum_x <= segment_max_x
                and maximum_y >= segment_min_y
                and minimum_y <= segment_max_y
            )
            if any(
                point_in_polygon(start, obstacle)
                or point_in_polygon(end, obstacle)
                or distance_segment_to_polygon_boundary(start, end, obstacle)
                <= self.config.vehicle_radius
                for obstacle in candidate_obstacles
            ):
                return False
            if any(
                distance_to_segment((x, y), start, end)
                <= self.config.vehicle_radius + radius
                for x, y, radius in observation.current_pedestrians
            ):
                return False
        return True

    def _trajectory_to(
        self,
        observation: AgentObservation,
        goal: Point2D,
        *,
        clean: bool = True,
        allow_hybrid: bool = True,
    ) -> TrajectoryAction | None:
        candidates: list[tuple[Pose2D, ...]] = []
        try:
            candidates.append(
                ackermann_path_to_point(
                    observation.pose,
                    goal,
                    min_turn_radius=self.config.min_turn_radius,
                    spacing=self.config.path_sample_spacing,
                )
            )
        except ValueError:
            pass

        # A curvature-limited reference path parked close to the geofence
        # cannot always make the forward U-turn produced above, even though
        # the same goal is immediately reachable by reversing.  Build the exact reverse
        # counterpart by solving from the opposite velocity heading and then
        # restoring the chassis heading.  ``validate_ackermann_path`` already
        # checks reverse chord alignment and curvature, so this is not an
        # in-place rotation or holonomic shortcut.
        reverse_start = Pose2D(
            observation.pose.x,
            observation.pose.y,
            wrap_angle(observation.pose.yaw + math.pi),
        )
        try:
            reverse_velocity_path = ackermann_path_to_point(
                reverse_start,
                goal,
                min_turn_radius=self.config.min_turn_radius,
                spacing=self.config.path_sample_spacing,
            )
            candidates.append(
                tuple(
                    Pose2D(pose.x, pose.y, wrap_angle(pose.yaw - math.pi))
                    for pose in reverse_velocity_path
                )
            )
        except ValueError:
            pass

        safe_candidates = [
            path for path in candidates if path and self._path_is_safe(path, observation)
        ]
        if safe_candidates:
            path = min(
                safe_candidates,
                key=lambda candidate: sum(
                    distance((a.x, a.y), (b.x, b.y))
                    for a, b in zip(candidate, candidate[1:])
                ),
            )
            return TrajectoryAction(path, clean_ground=clean)
        if not allow_hybrid:
            return None

        # A single turn-plus-line primitive is intentionally the fast path, but
        # it cannot pass a building or a row of street furniture.  The formal
        # campus therefore needs a bounded forward-only Hybrid-A* fallback.
        # Its states include heading and its edges are real constant-curvature
        # primitives, so the returned global trajectory still satisfies the
        # curvature-limited skid-steer reference-path contract rather than
        # teleporting laterally.
        routed = self._hybrid_path_to(observation, goal)
        if routed is None:
            return None
        return TrajectoryAction(routed, clean_ground=clean)

    def _forward_primitive(
        self,
        start: Pose2D,
        *,
        steering: int,
        direction: int,
        length: float,
    ) -> tuple[Pose2D, ...]:
        count = max(1, int(math.ceil(length / self.config.path_sample_spacing)))
        if steering == 0:
            return tuple(
                Pose2D(
                    start.x + direction * length * index / count * math.cos(start.yaw),
                    start.y + direction * length * index / count * math.sin(start.yaw),
                    start.yaw,
                )
                for index in range(count + 1)
            )
        turn_sign = -1 if steering < 0 else 1
        # A lattice edge should change heading by 15 or 30 degrees, not spin a
        # full minimum-radius semicircle at every map cell.  The resulting
        # radius is never smaller than the vehicle's declared minimum.
        requested_sweep = abs(steering) * math.pi / 12.0
        radius = max(self.config.min_turn_radius, length / requested_sweep)
        sweep = direction * length / radius
        center_x = start.x - turn_sign * radius * math.sin(start.yaw)
        center_y = start.y + turn_sign * radius * math.cos(start.yaw)
        start_radial = start.yaw - turn_sign * math.pi / 2.0
        return tuple(
            Pose2D(
                center_x
                + radius
                * math.cos(start_radial + turn_sign * sweep * index / count),
                center_y
                + radius
                * math.sin(start_radial + turn_sign * sweep * index / count),
                wrap_angle(start.yaw + turn_sign * sweep * index / count),
            )
            for index in range(count + 1)
        )

    def _hybrid_path_to(
        self,
        observation: AgentObservation,
        goal: Point2D,
    ) -> tuple[Pose2D, ...] | None:
        belief = observation.belief
        step = max(belief.resolution, self.config.min_turn_radius * 1.25)
        position_bin = max(belief.resolution * 0.5, step * 0.5)
        heading_bins = 24
        goal_tolerance = max(belief.resolution * 0.75, step * 0.55)
        # Keep formal 200 x 100 m rollouts computationally bounded.  The old
        # 40k-state cap made one blocked inspection waypoint take more than a
        # minute and prevented multi-map training from completing.  Eight
        # thousand heading-aware states still span the local obstacle detour;
        # unreachable goals are rotated and retried from a later pose.
        maximum_expansions = min(
            8000,
            max(2000, belief.width * belief.height),
        )

        def key(pose: Pose2D) -> tuple[int, int, int]:
            return (
                int(round((pose.x - belief.origin[0]) / position_bin)),
                int(round((pose.y - belief.origin[1]) / position_bin)),
                int(
                    round(
                        (wrap_angle(pose.yaw) + math.pi)
                        / (2.0 * math.pi)
                        * heading_bins
                    )
                )
                % heading_bins,
            )

        start = observation.pose
        start_key = key(start)
        counter = itertools.count()
        queue: list[tuple[float, int, tuple[int, int, int]]] = [
            (distance((start.x, start.y), goal), next(counter), start_key)
        ]
        poses = {start_key: start}
        costs = {start_key: 0.0}
        parents: dict[
            tuple[int, int, int],
            tuple[tuple[int, int, int], tuple[Pose2D, ...]],
        ] = {}
        closed: set[tuple[int, int, int]] = set()

        for _ in range(maximum_expansions):
            while queue:
                _, _, current_key = heapq.heappop(queue)
                if current_key not in closed:
                    break
            else:
                return None
            closed.add(current_key)
            current = poses[current_key]

            terminal: tuple[Pose2D, ...] | None = None
            if distance((current.x, current.y), goal) <= goal_tolerance:
                try:
                    proposal = ackermann_path_to_point(
                        current,
                        goal,
                        min_turn_radius=self.config.min_turn_radius,
                        spacing=self.config.path_sample_spacing,
                    )
                except ValueError:
                    proposal = (current,)
                if self._path_is_safe(proposal, observation):
                    terminal = proposal

            if terminal is not None:
                segments: list[tuple[Pose2D, ...]] = [terminal]
                trace_key = current_key
                while trace_key != start_key:
                    parent_key, primitive = parents[trace_key]
                    segments.append(primitive)
                    trace_key = parent_key
                segments.reverse()
                path: list[Pose2D] = []
                for segment in segments:
                    path.extend(segment if not path else segment[1:])
                return tuple(path)

            for direction in (1, -1):
                for steering in (0, -1, 1, -2, 2):
                    primitive = self._forward_primitive(
                        current,
                        steering=steering,
                        direction=direction,
                        length=step,
                    )
                    if not self._path_is_safe(primitive, observation):
                        continue
                    candidate = primitive[-1]
                    candidate_key = key(candidate)
                    if candidate_key in closed:
                        continue
                    turn_penalty = 0.03 * step if steering else 0.0
                    reverse_penalty = 0.15 * step if direction < 0 else 0.0
                    candidate_cost = (
                        costs[current_key] + step + turn_penalty + reverse_penalty
                    )
                    if candidate_cost + 1.0e-9 >= costs.get(candidate_key, math.inf):
                        continue
                    costs[candidate_key] = candidate_cost
                    poses[candidate_key] = candidate
                    parents[candidate_key] = (current_key, primitive)
                    heuristic = distance((candidate.x, candidate.y), goal)
                    heapq.heappush(
                        queue,
                        (candidate_cost + heuristic, next(counter), candidate_key),
                    )
        return None

    @staticmethod
    def _wait(observation: AgentObservation) -> TrajectoryAction:
        return TrajectoryAction((observation.pose,), clean_ground=False)

    def _grasp_if_reached(self, observation: AgentObservation) -> TrajectoryAction | None:
        reachable = [
            target.target_id
            for target in observation.belief.known_targets
            if not target.cleared
            and target.attempts < self.config.max_grasp_attempts
            and distance((observation.pose.x, observation.pose.y), (target.x, target.y)) <= self.config.grasp_radius
        ]
        if reachable:
            return TrajectoryAction(
                (observation.pose,),
                clean_ground=False,
                grasp_target_ids=(min(reachable),),
            )
        return None

    def _try_goals(
        self,
        observation: AgentObservation,
        goals: Iterable[Point2D],
        *,
        clean: bool = True,
        allow_hybrid: bool = True,
    ) -> TrajectoryAction | None:
        goal_list = list(goals)
        ordered = heapq.nsmallest(
            self._MAX_GOAL_CANDIDATES,
            goal_list,
            key=lambda goal: distance((observation.pose.x, observation.pose.y), goal),
        )
        if len(goal_list) > self._MAX_GOAL_CANDIDATES:
            nearby_count = self._MAX_GOAL_CANDIDATES // 2
            nearby = ordered[:nearby_count]
            stride = max(1, len(goal_list) // (self._MAX_GOAL_CANDIDATES - nearby_count))
            distributed = [
                goal
                for goal in goal_list[::stride]
                if goal not in nearby
            ][: self._MAX_GOAL_CANDIDATES - nearby_count]
            ordered = nearby + distributed
        for goal in ordered:
            action = self._trajectory_to(
                observation,
                goal,
                clean=clean,
                allow_hybrid=False,
            )
            if action is not None:
                return action
        if not allow_hybrid:
            return None
        # Hybrid-A* is intentionally reserved for the single closest blocked
        # goal. Launching an independent lattice search for several dirt or
        # target candidates dominated formal multi-map training time; a failed
        # goal is reconsidered after the next observation/pose update.
        for goal in ordered[:1]:
            action = self._trajectory_to(
                observation,
                goal,
                clean=clean,
                allow_hybrid=True,
            )
            if action is not None:
                return action
        return None

    def _reorientation_arc(self, observation: AgentObservation) -> TrajectoryAction | None:
        """Move through a short forward arc when every direct goal is blocked.

        This preserves the curvature-limited reference-path contract while
        avoiding a repeated zero-distance wait near geofence edges or obstacle
        corners.
        """
        start = observation.pose
        radius = self.config.min_turn_radius
        for sweep in (math.pi / 2.0, math.pi / 3.0, math.pi / 4.0):
            for turn_sign in (-1, 1):
                center_x = start.x - turn_sign * radius * math.sin(start.yaw)
                center_y = start.y + turn_sign * radius * math.cos(start.yaw)
                start_radial = start.yaw - turn_sign * math.pi / 2.0
                count = max(2, int(math.ceil(radius * sweep / self.config.path_sample_spacing)))
                path = tuple(
                    Pose2D(
                        center_x + radius * math.cos(start_radial + turn_sign * sweep * index / count),
                        center_y + radius * math.sin(start_radial + turn_sign * sweep * index / count),
                        start.yaw + turn_sign * sweep * index / count,
                    )
                    for index in range(count + 1)
                )
                if self._path_is_safe(path, observation):
                    return TrajectoryAction(path, clean_ground=False)
        return None


class FullCoveragePolicy(_SafeTrajectoryMixin, TrajectoryPolicy):
    """Serpentine free-grid baseline; no hidden dirt information is used."""

    name = "full_coverage"

    def __init__(self, config: TaskConfig):
        super().__init__(config)
        self._route: list[Point2D] = []
        self._cursor = 0

    def reset(self, *, episode_seed: int | None = None) -> None:
        self._route = []
        self._cursor = 0

    def _build_route(self, observation: AgentObservation) -> None:
        belief = observation.belief
        row_stride = max(1, int(math.floor(self.config.cleaning_width * 0.80 / belief.resolution)))
        rows: list[list[Point2D]] = []
        for row in range(0, belief.height, row_stride):
            runs: list[list[Point2D]] = []
            current: list[Point2D] = []
            for column in range(belief.width):
                index = row * belief.width + column
                if belief.traversable[index]:
                    current.append(cell_center(observation, index))
                elif current:
                    runs.append(current)
                    current = []
            if current:
                runs.append(current)
            row_points: list[Point2D] = []
            for run in runs:
                if len(run) == 1:
                    row_points.append(run[0])
                else:
                    row_points.extend((run[0], run[-1]))
            if row_points:
                rows.append(row_points)
        self._route = []
        for row_index, points in enumerate(rows):
            self._route.extend(points if row_index % 2 == 0 else reversed(points))
        if self._route:
            nearest = min(
                range(len(self._route)),
                key=lambda index: distance(
                    (observation.pose.x, observation.pose.y), self._route[index]
                ),
            )
            self._route = self._route[nearest:] + self._route[:nearest]

    def act(self, observation: AgentObservation) -> TrajectoryAction:
        grasp = self._grasp_if_reached(observation)
        if grasp is not None:
            return grasp
        # Full traversal is a distance baseline, not permission to ignore
        # objects that the public sensor belief has already exposed.  Service
        # those belief-visible tasks before resuming the deterministic sweep;
        # this keeps the baseline truth-free while making its completion
        # distance a usable (and conservative) upper bound for active policy.
        target_goals = [
            (target.x, target.y)
            for target in observation.belief.known_targets
            if not target.cleared and target.attempts < self.config.max_grasp_attempts
        ]
        target_action = self._try_goals(observation, target_goals, clean=False)
        if target_action is not None:
            return target_action
        dirt_goals = [
            cell_center(observation, index)
            for index, amount in enumerate(observation.belief.known_ground_dirt)
            if amount > 0.0
        ]
        dirt_action = self._try_goals(observation, dirt_goals, clean=True)
        if dirt_action is not None:
            return dirt_action
        if not self._route:
            self._build_route(observation)
        # A temporarily blocked waypoint must not be discarded forever.  On a
        # campus with people and many islands that old behaviour consumed the
        # entire 700-point coverage route in a handful of calls and then
        # waited at 12-17% observed area.  Rotate blocked goals to the tail and
        # revisit them after the chassis (or pedestrian) has moved.
        attempts_remaining = len(self._route) - self._cursor
        while self._cursor < len(self._route) and attempts_remaining > 0:
            goal = self._route[self._cursor]
            if distance((observation.pose.x, observation.pose.y), goal) <= observation.belief.resolution:
                self._cursor += 1
                continue
            action = self._trajectory_to(
                observation,
                goal,
                clean=True,
                allow_hybrid=False,
            )
            if action is not None:
                self._cursor += 1
                return action
            self._route.append(self._route.pop(self._cursor))
            attempts_remaining -= 1
        if observation.belief.width * observation.belief.height <= 2000:
            # Retain the exact small-map baseline behaviour used by unit and
            # research demos, where the Hybrid-A* fallback is inexpensive.
            return SensingGreedyPolicy(self.config).act(observation)
        # Do not construct a fresh SensingGreedy/Hybrid-A* search after every
        # blocked formal-campus sweep.  That discarded route state and made
        # one 240-step baseline take unbounded minutes.  A bounded reorientation
        # changes the reachable direct-waypoint set; blocked goals remain in
        # the route and are retried on the next step.
        reorientation = self._reorientation_arc(observation)
        return reorientation if reorientation is not None else self._wait(observation)


class SensingGreedyPolicy(_SafeTrajectoryMixin, TrajectoryPolicy):
    """Nearest known dirt/target, otherwise nearest unobserved free cell."""

    name = "sensing_greedy"

    def reset(self, *, episode_seed: int | None = None) -> None:
        self._reset_inspection_route()

    def act(self, observation: AgentObservation) -> TrajectoryAction:
        grasp = self._grasp_if_reached(observation)
        if grasp is not None:
            return grasp
        target_goals = [
            (target.x, target.y)
            for target in observation.belief.known_targets
            if not target.cleared and target.attempts < self.config.max_grasp_attempts
        ]
        action = self._try_goals(observation, target_goals, clean=False)
        if action is not None:
            return action
        dirt_goals = [
            cell_center(observation, index)
            for index, amount in enumerate(observation.belief.known_ground_dirt)
            if amount > 0.0
        ]
        action = self._try_goals(observation, dirt_goals, clean=True)
        if action is not None:
            return action
        action = self._inspection_action(observation)
        if action is not None:
            return action
        reorientation = self._reorientation_arc(observation)
        return reorientation if reorientation is not None else self._wait(observation)


class OraclePolicy(SensingGreedyPolicy):
    """Evaluation-only upper bound that may prioritize hidden dirt."""

    name = "oracle"
    evaluation_only = True

    def act_with_truth(
        self,
        observation: AgentObservation,
        truth: EvaluationSnapshot,
    ) -> TrajectoryAction:
        grasp = self._grasp_if_reached(observation)
        if grasp is not None:
            return grasp
        uncleared_targets = [
            (x, y)
            for target_id, x, y in truth.initial_targets
            if target_id not in truth.cleared_target_ids
            and all(
                known.target_id != target_id
                or known.attempts < self.config.max_grasp_attempts
                for known in observation.belief.known_targets
            )
        ]
        action = self._try_goals(observation, uncleared_targets, clean=False)
        if action is not None:
            return action
        hidden_dirt = [
            cell_center(observation, index)
            for index in truth.remaining_ground_dirt_cells
        ]
        action = self._try_goals(observation, hidden_dirt, clean=True)
        if action is not None:
            return action
        return super().act(observation)
