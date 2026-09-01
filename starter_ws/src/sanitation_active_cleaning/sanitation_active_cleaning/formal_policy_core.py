"""ROS-independent runtime adapter from product belief to an RL trajectory."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable, Sequence

import yaml

from .environment import GridModel
from .formal_observation_core import PublicPlanningMap
from .geometry import curvature_limited_reference_path_for_skid_steer, distance, wrap_angle
from .models import AgentObservation, BeliefSnapshot, KnownTarget, Pose2D, TaskConfig
from .rl import ACTION_TARGET, QLearningPolicy


# The final high-fidelity arm geometry accepts a cube only through this
# base_link-relative right-side window.  Navigation must park the vehicle
# around the detected target; driving the vehicle centre onto the cube is both
# physically wrong and unsafe.
GRASP_WINDOW_X_M = 0.300
GRASP_WINDOW_Y_M = -0.950
GRASP_WINDOW_POSITION_TOLERANCE_M = 0.10


@dataclass(frozen=True)
class RuntimePolicyDecision:
    kind: str
    reason: str
    trajectory: tuple[Pose2D, ...] = ()
    clean_ground: bool = False
    grasp_target_id: str | None = None
    observed_ratio: float = 0.0


def runtime_task_config(
    planning_map: PublicPlanningMap,
    mission_geometry: str | Path,
    *,
    planning_resolution_m: float,
    sensing_radius_m: float,
    sensing_fov_rad: float,
) -> TaskConfig:
    value = yaml.safe_load(Path(mission_geometry).read_text(encoding="utf-8"))
    if not isinstance(value, dict) or value.get("frame_id") != "map":
        raise ValueError("formal mission geometry must be a map-frame mapping")
    start = value.get("vehicle_start_pose_map", {})
    return TaskConfig.from_mapping(
        {
            "geofence": planning_map.outer_polygon,
            # These polygons are already inflated for the formal transport
            # footprint.  A tiny positive radius avoids double inflation while
            # retaining TaskConfig's strict-positive geometry contract.
            "static_obstacles": planning_map.keepout_polygons,
            "start": {
                "x": start["x_m"],
                "y": start["y_m"],
                "yaw": start.get("yaw_rad", 0.0),
            },
            "grid_resolution": planning_resolution_m,
            "sensing_radius": sensing_radius_m,
            "sensing_fov_rad": sensing_fov_rad,
            "cleaning_width": float(value["operation_width_m"]),
            "vehicle_radius": 1.0e-4,
            "grasp_radius": 0.75,
            "min_turn_radius": 0.70,
            "path_sample_spacing": min(0.20, planning_resolution_m / 2.0),
            "observation_threshold": 0.95,
            "ground_clear_threshold": 0.95,
            "discrete_clear_threshold": 0.95,
            "ground_dirt_count": 0,
            "discrete_target_count": 0,
            "pedestrian_count": 0,
            "max_grasp_attempts": 2,
            "max_steps": 1200,
        }
    )


class FormalRuntimePolicyCore:
    """Downsample a 0.1 m product belief and invoke a frozen truth-free policy."""

    def __init__(
        self,
        planning_map: PublicPlanningMap,
        config: TaskConfig,
        checkpoint: str | Path,
        *,
        maximum_task_distance_m: float,
    ):
        if not math.isfinite(maximum_task_distance_m) or maximum_task_distance_m <= 0:
            raise ValueError("maximum_task_distance_m must be finite and positive")
        self.public_map = planning_map
        self.config = config
        self.maximum_task_distance_m = float(maximum_task_distance_m)
        self.grid = GridModel(config)
        self.policy = QLearningPolicy.load(config, checkpoint)
        self.policy.epsilon = 0.0
        self._attempts: dict[str, int] = {}
        self._cleared: set[str] = set()

    def reset(self, *, episode_seed: int) -> None:
        self._attempts.clear()
        self._cleared.clear()
        self.policy.reset(episode_seed=episode_seed)

    def mark_grasp_result(self, target_id: str, *, verified_in_bin: bool) -> None:
        if not target_id:
            raise ValueError("grasp result target_id cannot be empty")
        self._attempts[target_id] = self._attempts.get(target_id, 0) + 1
        if verified_in_bin:
            self._cleared.add(target_id)

    def _product_window(self, center: tuple[float, float]) -> tuple[int, int, int, int]:
        half = self.config.grid_resolution * 0.5
        first_column = max(
            0,
            int(math.floor((center[0] - half - self.public_map.origin_x) / self.public_map.resolution)),
        )
        last_column = min(
            self.public_map.width - 1,
            int(math.floor((center[0] + half - self.public_map.origin_x) / self.public_map.resolution)),
        )
        first_row = max(
            0,
            int(math.floor((center[1] - half - self.public_map.origin_y) / self.public_map.resolution)),
        )
        last_row = min(
            self.public_map.height - 1,
            int(math.floor((center[1] + half - self.public_map.origin_y) / self.public_map.resolution)),
        )
        return first_column, last_column, first_row, last_row

    def observation(
        self,
        *,
        belief_values: Sequence[int],
        pose: Pose2D,
        targets: Iterable[KnownTarget],
        step_index: int,
        task_distance: float,
    ) -> AgentObservation:
        expected = self.public_map.width * self.public_map.height
        if len(belief_values) != expected:
            raise ValueError("product belief size mismatch")
        product_free = sum(self.public_map.traversable)
        product_observed = sum(
            free and int(value) >= 0
            for free, value in zip(self.public_map.traversable, belief_values)
        )
        coarse_observed = []
        coarse_dirt = []
        for free, center in zip(self.grid.traversable, self.grid.centers):
            if not free:
                coarse_observed.append(False)
                coarse_dirt.append(0.0)
                continue
            first_column, last_column, first_row, last_row = self._product_window(center)
            free_count = 0
            observed_count = 0
            maximum_dirt = 0
            for row in range(first_row, last_row + 1):
                offset = row * self.public_map.width
                for column in range(first_column, last_column + 1):
                    index = offset + column
                    if not self.public_map.traversable[index]:
                        continue
                    free_count += 1
                    value = int(belief_values[index])
                    if value >= 0:
                        observed_count += 1
                    if value > maximum_dirt:
                        maximum_dirt = value
            coarse_observed.append(
                free_count > 0 and observed_count / free_count >= 0.50
            )
            coarse_dirt.append(maximum_dirt / 100.0 if maximum_dirt > 0 else 0.0)

        known_targets = tuple(
            KnownTarget(
                target_id=item.target_id,
                x=item.x,
                y=item.y,
                cleared=item.cleared or item.target_id in self._cleared,
                attempts=max(item.attempts, self._attempts.get(item.target_id, 0)),
            )
            for item in targets
        )
        snapshot = BeliefSnapshot(
            width=self.grid.width,
            height=self.grid.height,
            origin=self.grid.origin,
            resolution=self.grid.resolution,
            traversable=self.grid.traversable,
            observed=tuple(coarse_observed),
            known_ground_dirt=tuple(coarse_dirt),
            known_targets=known_targets,
        )
        return AgentObservation(
            step_index=int(step_index),
            pose=pose,
            observed_ratio=(product_observed / product_free if product_free else 0.0),
            belief=snapshot,
            static_obstacles=self.config.static_obstacles,
            current_pedestrians=(),
            task_distance=float(task_distance),
            remaining_distance_budget=max(
                0.0, self.maximum_task_distance_m - float(task_distance)
            ),
        )

    def decide(self, observation: AgentObservation) -> RuntimePolicyDecision:
        _, label, action = self.policy.act_with_label(observation, explore=False)
        if label == ACTION_TARGET:
            target_decision = self._service_target(observation)
            if target_decision is not None:
                return self._budget_gate(target_decision, observation)
        if action.grasp_target_ids:
            return RuntimePolicyDecision(
                kind="grasp",
                reason="policy_requested_verified_pick_and_store",
                grasp_target_id=action.grasp_target_ids[0],
                observed_ratio=observation.observed_ratio,
            )
        if len(action.points) < 2:
            return RuntimePolicyDecision(
                kind="wait",
                reason="no_safe_progress_trajectory",
                observed_ratio=observation.observed_ratio,
            )
        return self._budget_gate(RuntimePolicyDecision(
            kind="trajectory",
            reason="truth_free_policy_trajectory",
            trajectory=action.points,
            clean_ground=action.clean_ground,
            observed_ratio=observation.observed_ratio,
        ), observation)

    @staticmethod
    def _trajectory_distance(trajectory: Sequence[Pose2D]) -> float:
        return sum(
            math.dist((first.x, first.y), (second.x, second.y))
            for first, second in zip(trajectory, trajectory[1:])
        )

    def _budget_gate(
        self,
        decision: RuntimePolicyDecision,
        observation: AgentObservation,
    ) -> RuntimePolicyDecision:
        if decision.kind != "trajectory":
            return decision
        remaining = observation.remaining_distance_budget
        distance_m = self._trajectory_distance(decision.trajectory)
        if remaining is None or distance_m <= remaining + 1.0e-9:
            return decision
        return RuntimePolicyDecision(
            kind="wait",
            reason="full_coverage_distance_budget_exceeded",
            observed_ratio=observation.observed_ratio,
        )

    def return_home(self, observation: AgentObservation) -> RuntimePolicyDecision:
        home = (self.config.start.x, self.config.start.y)
        if distance((observation.pose.x, observation.pose.y), home) <= max(
            0.25, self.config.grid_resolution * 0.5
        ):
            return RuntimePolicyDecision(
                kind="home_reached",
                reason="fixed_start_pose_reached_after_task",
                observed_ratio=observation.observed_ratio,
            )
        action = self.policy._trajectory_to(observation, home, clean=False)
        if action is None or len(action.points) < 2:
            return RuntimePolicyDecision(
                kind="wait",
                reason="no_safe_return_home_trajectory",
                observed_ratio=observation.observed_ratio,
            )
        return RuntimePolicyDecision(
            kind="trajectory",
            reason="return_to_fixed_start_after_task",
            trajectory=action.points,
            clean_ground=False,
            observed_ratio=observation.observed_ratio,
        )

    @staticmethod
    def _target_in_grasp_window(pose: Pose2D, target: KnownTarget) -> bool:
        delta_x = target.x - pose.x
        delta_y = target.y - pose.y
        cosine = math.cos(pose.yaw)
        sine = math.sin(pose.yaw)
        body_x = cosine * delta_x + sine * delta_y
        body_y = -sine * delta_x + cosine * delta_y
        return (
            abs(body_x - GRASP_WINDOW_X_M)
            <= GRASP_WINDOW_POSITION_TOLERANCE_M
            and abs(body_y - GRASP_WINDOW_Y_M)
            <= GRASP_WINDOW_POSITION_TOLERANCE_M
        )

    def _service_target(
        self, observation: AgentObservation
    ) -> RuntimePolicyDecision | None:
        targets = [
            item
            for item in observation.belief.known_targets
            if not item.cleared and item.attempts < self.config.max_grasp_attempts
        ]
        if not targets:
            return None
        target = min(
            targets,
            key=lambda item: distance(
                (observation.pose.x, observation.pose.y), (item.x, item.y)
            ),
        )
        if self._target_in_grasp_window(observation.pose, target):
            return RuntimePolicyDecision(
                kind="grasp",
                reason="detected_target_in_physical_grasp_window",
                grasp_target_id=target.target_id,
                observed_ratio=observation.observed_ratio,
            )

        action = self._parking_action(observation, target)
        if action is None or len(action.points) < 2:
            return RuntimePolicyDecision(
                kind="wait",
                reason="no_safe_grasp_parking_trajectory",
                observed_ratio=observation.observed_ratio,
            )
        return RuntimePolicyDecision(
            kind="trajectory",
            reason="navigate_to_physical_grasp_window",
            trajectory=action.points,
            clean_ground=False,
            observed_ratio=observation.observed_ratio,
        )

    def _parking_action(self, observation: AgentObservation, target: KnownTarget):
        """Find a safe approach whose endpoint places the target in the arm window."""

        offset_angle = math.atan2(GRASP_WINDOW_Y_M, GRASP_WINDOW_X_M)
        approach_angle = math.atan2(
            target.y - observation.pose.y,
            target.x - observation.pose.x,
        )
        initial_yaw = wrap_angle(approach_angle - offset_angle)
        best = None
        best_error = math.inf
        # Fixed-point refinement couples the curvature-limited reference-path
        # terminal heading to the parking position implied by the arm's side
        # window.  It does not model or command physical steering joints.
        # Multiple deterministic initial headings avoid a single bad branch.
        for branch in range(16):
            yaw = wrap_angle(initial_yaw + branch * 2.0 * math.pi / 16.0)
            for _ in range(12):
                cosine = math.cos(yaw)
                sine = math.sin(yaw)
                goal = (
                    target.x
                    - (cosine * GRASP_WINDOW_X_M - sine * GRASP_WINDOW_Y_M),
                    target.y
                    - (sine * GRASP_WINDOW_X_M + cosine * GRASP_WINDOW_Y_M),
                )
                try:
                    path = curvature_limited_reference_path_for_skid_steer(
                        observation.pose,
                        goal,
                        min_turn_radius=self.config.min_turn_radius,
                        spacing=self.config.path_sample_spacing,
                    )
                except ValueError:
                    break
                if not path or not self.policy._path_is_safe(path, observation):
                    break
                endpoint = path[-1]
                delta_x = target.x - endpoint.x
                delta_y = target.y - endpoint.y
                end_cosine = math.cos(endpoint.yaw)
                end_sine = math.sin(endpoint.yaw)
                body_x = end_cosine * delta_x + end_sine * delta_y
                body_y = -end_sine * delta_x + end_cosine * delta_y
                error = math.hypot(
                    body_x - GRASP_WINDOW_X_M,
                    body_y - GRASP_WINDOW_Y_M,
                )
                if error < best_error:
                    best_error = error
                    best = path
                if error <= GRASP_WINDOW_POSITION_TOLERANCE_M * 0.5:
                    from .environment import TrajectoryAction

                    return TrajectoryAction(tuple(path), clean_ground=False)
                yaw = endpoint.yaw

        if best is not None and best_error <= GRASP_WINDOW_POSITION_TOLERANCE_M:
            from .environment import TrajectoryAction

            return TrajectoryAction(tuple(best), clean_ground=False)

        # When the exact final approach is blocked, first move safely toward a
        # candidate parking point.  A later cycle recomputes the final approach
        # from the new pose; it never emits a grasp until the TF window check
        # above passes.
        candidates = []
        for index in range(16):
            yaw = index * 2.0 * math.pi / 16.0
            cosine = math.cos(yaw)
            sine = math.sin(yaw)
            candidates.append(
                (
                    target.x
                    - (cosine * GRASP_WINDOW_X_M - sine * GRASP_WINDOW_Y_M),
                    target.y
                    - (sine * GRASP_WINDOW_X_M + cosine * GRASP_WINDOW_Y_M),
                )
            )
        return self.policy._try_goals(observation, candidates, clean=False)
