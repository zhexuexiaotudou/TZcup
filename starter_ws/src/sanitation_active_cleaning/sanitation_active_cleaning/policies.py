"""Deterministic high-level trajectory policies used as RL baselines."""

from __future__ import annotations

import heapq
import math
from typing import Iterable, Sequence

from .environment import TrajectoryAction
from .geometry import (
    ackermann_path_to_point,
    distance,
    distance_segment_to_polygon_boundary,
    distance_to_segment,
    point_in_polygon,
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
            if any(
                point_in_polygon(start, obstacle)
                or point_in_polygon(end, obstacle)
                or distance_segment_to_polygon_boundary(start, end, obstacle)
                <= self.config.vehicle_radius
                for obstacle in observation.static_obstacles
            ):
                return False
            if any(
                distance_to_segment((x, y), start, end)
                <= self.config.vehicle_radius + radius
                for x, y, radius in observation.current_pedestrians
            ):
                return False
        return True

    def _trajectory_to(self, observation: AgentObservation, goal: Point2D, *, clean: bool = True) -> TrajectoryAction | None:
        try:
            path = ackermann_path_to_point(
                observation.pose,
                goal,
                min_turn_radius=self.config.min_turn_radius,
                spacing=self.config.path_sample_spacing,
            )
        except ValueError:
            return None
        if not self._path_is_safe(path, observation):
            return None
        return TrajectoryAction(path, clean_ground=clean)

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

    def _try_goals(self, observation: AgentObservation, goals: Iterable[Point2D], *, clean: bool = True) -> TrajectoryAction | None:
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
            action = self._trajectory_to(observation, goal, clean=clean)
            if action is not None:
                return action
        return None

    def _reorientation_arc(self, observation: AgentObservation) -> TrajectoryAction | None:
        """Move through a short forward arc when every direct goal is blocked.

        This preserves the virtual Ackermann contract while avoiding a repeated
        zero-distance wait near geofence edges or obstacle corners.
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
        if not self._route:
            self._build_route(observation)
        while self._cursor < len(self._route):
            goal = self._route[self._cursor]
            self._cursor += 1
            if distance((observation.pose.x, observation.pose.y), goal) <= observation.belief.resolution:
                continue
            action = self._trajectory_to(observation, goal, clean=True)
            if action is not None:
                return action
        fallback = SensingGreedyPolicy(self.config)
        return fallback.act(observation)


class SensingGreedyPolicy(_SafeTrajectoryMixin, TrajectoryPolicy):
    """Nearest known dirt/target, otherwise nearest unobserved free cell."""

    name = "sensing_greedy"

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
        frontier = [
            cell_center(observation, index)
            for index, (free, observed) in enumerate(
                zip(observation.belief.traversable, observation.belief.observed)
            )
            if free and not observed
        ]
        action = self._try_goals(observation, frontier, clean=False)
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
