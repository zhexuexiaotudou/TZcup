"""Autonomous frontier exploration for formal large-map SLAM runs."""

from __future__ import annotations

from dataclasses import asdict
import json
import math
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import BackUp, ComputePathToPose, FollowPath, NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from sanitation_coverage.ackermann_connector import (
    plan_forward_dubins_path,
    split_hybrid_path_by_direction,
)
from sanitation_coverage.metrics import split_path_at_curvature_reversals

from .frontier_core import (
    frontier_sweep_targets,
    frontier_sweep_target_axis,
    GridGeometry,
    lane_shift_connector_goals,
    map_extent_metrics,
    mapping_completion_reached,
    next_adaptive_goal_distance,
    prune_timed_exclusions,
    rank_frontiers,
    reverse_escape_goal,
    vertical_sweep_anchor_reached,
    world_disk_has_known_cell,
    world_disk_is_traversable,
)


def _yaw_from_quaternion(quaternion) -> float:
    siny = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny, cosy)


class FrontierExplorer(Node):
    """Select map frontiers and send collision-checked Nav2 goals.

    This node has no ground-truth subscription and never reads the SDF. Its
    only exploration input is the online occupancy grid plus the production
    map-to-base TF used by Nav2.
    """

    def __init__(self) -> None:
        super().__init__("sanitation_frontier_explorer")
        self.declare_parameter("output_path", "/tmp/frontier_exploration.json")
        self.declare_parameter("required_bounds_xyxy_m", [-100.0, -50.0, 100.0, 50.0])
        self.declare_parameter("required_bounds_coverage_ratio", 1.0)
        self.declare_parameter("minimum_frontier_cells", 8)
        self.declare_parameter("frontier_connection_radius_cells", 3)
        self.declare_parameter("minimum_goal_distance_m", 1.5)
        self.declare_parameter("failed_goal_exclusion_radius_m", 3.0)
        self.declare_parameter("timed_out_goal_exclusion_radius_m", 1.5)
        self.declare_parameter("required_bounds_goal_margin_m", 1.5)
        self.declare_parameter("frontier_goal_backoff_m", 1.5)
        self.declare_parameter("maximum_frontier_goal_distance_m", 4.0)
        self.declare_parameter("initial_frontier_goal_distance_m", 2.0)
        self.declare_parameter("goal_distance_growth_success_count", 3)
        self.declare_parameter("goal_distance_growth_step_m", 1.0)
        self.declare_parameter("maximum_frontier_goal_yaw_change_rad", 0.35)
        self.declare_parameter("minimum_frontier_arc_yaw_change_rad", 0.15)
        self.declare_parameter("minimum_turning_radius_m", 1.429)
        self.declare_parameter("boundary_turn_buffer_m", 1.429)
        self.declare_parameter("maximum_goal_count", 160)
        self.declare_parameter("goal_timeout_sec", 60.0)
        self.declare_parameter("failed_goal_cooldown_sec", 10.0)
        self.declare_parameter("failed_goal_exclusion_ttl_sec", 180.0)
        self.declare_parameter("reverse_escape_distance_m", 2.0)
        self.declare_parameter("reverse_escape_speed_mps", 0.15)
        self.declare_parameter("frontier_sweep_enabled", False)
        self.declare_parameter("frontier_sweep_initial_target_index", 0)
        # A millimetre north of the centre removes the otherwise platform-
        # dependent tie between the -10 m and +10 m first sweep lanes.
        self.declare_parameter("frontier_sweep_reference_pose_xyyaw_m_rad", [0.0, 0.001, 0.0])
        self.declare_parameter("mapping_sensor_range_m", 12.0)
        self.declare_parameter("frontier_sweep_lane_overlap_m", 2.0)
        self.declare_parameter("frontier_sweep_target_tolerance_m", 2.0)
        self.declare_parameter("frontier_sweep_mapped_target_radius_m", 5.0)
        self.declare_parameter("frontier_sweep_lane_shift_backup_distance_m", 4.0)
        self.declare_parameter("frontier_sweep_lane_shift_backup_max_attempts", 2)
        self.declare_parameter(
            "frontier_sweep_lane_shift_connector_distances_m", [6.0, 4.0, 2.0]
        )
        self.declare_parameter("lane_shift_connector_timeout_sec", 180.0)
        self.declare_parameter("timeout_sec", 7200.0)
        self.declare_parameter("completion_stable_map_updates", 3)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("global_costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("goal_clearance_radius_m", 0.70)
        self.declare_parameter("maximum_goal_cost", 99)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("positioning_source", "wheel_imu_scan_matching")
        self.declare_parameter("behavior_tree", "")
        self.output_path = Path(str(self.get_parameter("output_path").value))
        self.required_bounds = tuple(
            float(value)
            for value in self.get_parameter("required_bounds_xyxy_m").value
        )
        if len(self.required_bounds) != 4:
            raise ValueError("required_bounds_xyxy_m must contain four values")

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            map_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("global_costmap_topic").value),
            self._on_costmap,
            map_qos,
        )
        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.backup_client = ActionClient(self, BackUp, "/backup")
        self.compute_path_client = ActionClient(
            self, ComputePathToPose, "/compute_path_to_pose"
        )
        self.follow_path_client = ActionClient(self, FollowPath, "/follow_path")
        self.nav_manager_client = self.create_client(
            ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes"
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0, self._tick)

        self.started_monotonic = time.monotonic()
        self.latest_data = None
        self.latest_geometry = None
        self.latest_metrics = None
        self.latest_costmap_data = None
        self.latest_costmap_geometry = None
        self.costmap_rejected_goal_count = 0
        self.frontier_exclusion_wait_count = 0
        self.reverse_escape_goal_count = 0
        self.sweep_targets = []
        self.sweep_target_index = int(
            self.get_parameter("frontier_sweep_initial_target_index").value
        )
        if self.sweep_target_index < 0:
            raise ValueError("frontier_sweep_initial_target_index must be non-negative")
        self.sweep_completed = False
        self.sweep_active_anchor = None
        self.sweep_active_preference = None
        self.sweep_active_axis = None
        self.sweep_lane_shift_backup_completed = set()
        self.sweep_lane_shift_backup_skipped = set()
        self.sweep_lane_shift_backup_pending = None
        self.sweep_lane_shift_backup_count = 0
        self.sweep_lane_shift_backup_attempts = {}
        self.sweep_lane_shift_locked_x = {}
        self.sweep_lane_shift_connector_completed = set()
        self.sweep_lane_shift_connector_pending = None
        self.sweep_lane_shift_connector_attempts = {}
        self.sweep_lane_shift_connector_sections = []
        self.sweep_lane_shift_connector_section_index = 0
        self.sweep_lane_shift_connector_goal = None
        self.sweep_lane_shift_connector_row = None
        maximum_goal_distance = float(
            self.get_parameter("maximum_frontier_goal_distance_m").value
        )
        self.adaptive_goal_distance_m = min(
            maximum_goal_distance,
            float(self.get_parameter("initial_frontier_goal_distance_m").value),
        )
        self.goal_distance_success_streak = 0
        self.map_update_count = 0
        self.stable_pass_updates = 0
        self.goal_history = []
        self.excluded_goals = []
        self.active_goal = None
        self.active_goal_handle = None
        self.active_goal_started_monotonic = None
        self.active_goal_timeout_sec = float(
            self.get_parameter("goal_timeout_sec").value
        )
        self.active_goal_cancel_requested = False
        self.next_goal_not_before_monotonic = self.started_monotonic
        self.nav_recovery_in_progress = False
        self.nav_recovery_count = 0
        self.nav_recovery_status = "not_required"
        self.terminal = False
        self.success = False
        self.terminal_reason = None
        self.last_pose = None
        self.last_error = None

    def _on_map(self, message: OccupancyGrid) -> None:
        origin = message.info.origin
        geometry = GridGeometry(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution_m=float(message.info.resolution),
            origin_x_m=float(origin.position.x),
            origin_y_m=float(origin.position.y),
            origin_yaw_rad=_yaw_from_quaternion(origin.orientation),
        )
        data = tuple(int(value) for value in message.data)
        metrics = map_extent_metrics(
            data,
            geometry,
            required_bounds_xyxy_m=self.required_bounds,
        )
        self.latest_data = data
        self.latest_geometry = geometry
        self.latest_metrics = metrics
        self.map_update_count += 1
        required = float(
            self.get_parameter("required_bounds_coverage_ratio").value
        )
        if mapping_completion_reached(
            metrics, required_envelope_coverage_ratio=required
        ):
            self.stable_pass_updates += 1
        else:
            self.stable_pass_updates = 0
        self._write_report()

    def _robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("base_frame").value),
                rclpy.time.Time(),
            )
        except TransformException as error:
            self.last_error = f"tf_unavailable: {error}"
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        pose = (
            float(translation.x),
            float(translation.y),
            _yaw_from_quaternion(rotation),
        )
        self.last_pose = pose
        return pose

    def _on_costmap(self, message: OccupancyGrid) -> None:
        origin = message.info.origin
        self.latest_costmap_geometry = GridGeometry(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution_m=float(message.info.resolution),
            origin_x_m=float(origin.position.x),
            origin_y_m=float(origin.position.y),
            origin_yaw_rad=_yaw_from_quaternion(origin.orientation),
        )
        self.latest_costmap_data = tuple(int(value) for value in message.data)

    def _goal_is_costmap_clear(self, goal) -> bool:
        if self.latest_costmap_data is None or self.latest_costmap_geometry is None:
            return False
        return world_disk_is_traversable(
            self.latest_costmap_data,
            self.latest_costmap_geometry,
            (goal.world_x_m, goal.world_y_m),
            radius_m=float(self.get_parameter("goal_clearance_radius_m").value),
            maximum_cost=int(self.get_parameter("maximum_goal_cost").value),
        )

    def _tick(self) -> None:
        if self.terminal:
            return
        if self.nav_recovery_in_progress:
            return
        elapsed = time.monotonic() - self.started_monotonic
        if elapsed >= float(self.get_parameter("timeout_sec").value):
            self._finish(False, "exploration_timeout")
            return
        stable_required = int(
            self.get_parameter("completion_stable_map_updates").value
        )
        if self.stable_pass_updates >= stable_required:
            if self.active_goal_handle is not None:
                self.active_goal_handle.cancel_goal_async()
            self._finish(True, "required_mapping_bounds_and_area_mapped")
            return
        if self.active_goal is not None:
            goal_elapsed = time.monotonic() - float(
                self.active_goal_started_monotonic or time.monotonic()
            )
            if (
                goal_elapsed >= self.active_goal_timeout_sec
                and self.active_goal_handle is not None
                and not self.active_goal_cancel_requested
            ):
                self.active_goal_cancel_requested = True
                self.last_error = f"frontier_goal_timeout:{goal_elapsed:.3f}s"
                self.active_goal_handle.cancel_goal_async()
                self._write_report()
            return
        if len(self.goal_history) >= int(
            self.get_parameter("maximum_goal_count").value
        ):
            self._finish(False, "maximum_goal_count_exhausted")
            return
        if time.monotonic() < self.next_goal_not_before_monotonic:
            return
        if (
            self.latest_data is None
            or self.latest_costmap_data is None
            or not self.action_client.server_is_ready()
        ):
            return
        robot_pose = self._robot_pose()
        if robot_pose is None:
            return
        self._sweep_preference(robot_pose)
        if self._start_sweep_lane_shift_backup(robot_pose):
            return
        if self._start_sweep_lane_shift_connector(robot_pose):
            return
        goal = None
        now = time.monotonic()
        self.excluded_goals, temporary_exclusions = prune_timed_exclusions(
            self.excluded_goals,
            now_monotonic=now,
        )
        for _ in range(32):
            goals = self._rank_goals(robot_pose, temporary_exclusions)
            if not goals:
                break
            candidate = goals[0]
            if self._goal_is_costmap_clear(candidate):
                goal = candidate
                break
            temporary_exclusions.append((candidate.world_x_m, candidate.world_y_m))
            self._add_excluded_goal(candidate.world_x_m, candidate.world_y_m)
            self.costmap_rejected_goal_count += 1
        if goal is None:
            escape = reverse_escape_goal(
                robot_pose[:2],
                robot_pose[2],
                distance_m=float(
                    self.get_parameter("reverse_escape_distance_m").value
                ),
                allowed_bounds_xyxy_m=self.required_bounds,
                boundary_margin_m=float(
                    self.get_parameter("required_bounds_goal_margin_m").value
                ),
            )
            # A long mission must not permanently lose its only reachable
            # frontier because of one historical controller failure. If a
            # candidate exists without the active timed exclusions, first
            # reverse into verified free space so the next forward arc does
            # not collapse onto the same dead end. Wait only if that escape
            # endpoint is itself unavailable.
            if self.excluded_goals and self._rank_goals(robot_pose, []):
                if escape is not None and self._goal_is_costmap_clear(escape):
                    self.last_error = "frontier_dead_end_reverse_escape"
                    if self._send_backup(escape):
                        self.reverse_escape_goal_count += 1
                        return
                    self.last_error = "backup_action_server_unavailable"
                    self.next_goal_not_before_monotonic = now + 1.0
                    self._write_report()
                    return
                self.frontier_exclusion_wait_count += 1
                self.last_error = "frontier_candidates_temporarily_excluded"
                earliest_expiry = min(row[2] for row in self.excluded_goals)
                self.next_goal_not_before_monotonic = max(
                    now + 1.0, earliest_expiry
                )
                self._write_report()
                return
            # Reaching an envelope edge can temporarily leave every online
            # frontier outside the permitted goal margin even though the
            # required envelope is incomplete. Reverse into verified free
            # space and recompute instead of declaring false exhaustion.
            if escape is not None and self._goal_is_costmap_clear(escape):
                self.last_error = "frontier_exhaustion_reverse_escape"
                if self._send_backup(escape):
                    self.reverse_escape_goal_count += 1
                    return
                self.last_error = "backup_action_server_unavailable"
                self.next_goal_not_before_monotonic = now + 1.0
                self._write_report()
                return
            self._finish(False, "frontiers_exhausted_before_required_bounds")
            return
        self._send_goal(goal)

    def _rank_goals(self, robot_pose, excluded_world_xy):
        sweep_preference = self._sweep_preference(robot_pose)
        return rank_frontiers(
            self.latest_data,
            self.latest_geometry,
            robot_pose[:2],
            robot_yaw_rad=robot_pose[2],
            excluded_world_xy=excluded_world_xy,
            exclusion_radius_m=float(
                self.get_parameter("failed_goal_exclusion_radius_m").value
            ),
            minimum_goal_distance_m=float(
                self.get_parameter("minimum_goal_distance_m").value
            ),
            minimum_cells=int(
                self.get_parameter("minimum_frontier_cells").value
            ),
            connection_radius_cells=int(
                self.get_parameter("frontier_connection_radius_cells").value
            ),
            allowed_bounds_xyxy_m=self.required_bounds,
            boundary_margin_m=float(
                self.get_parameter("required_bounds_goal_margin_m").value
            ),
            goal_backoff_m=float(
                self.get_parameter("frontier_goal_backoff_m").value
            ),
            maximum_goal_distance_m=self.adaptive_goal_distance_m,
            maximum_goal_yaw_change_rad=float(
                self.get_parameter("maximum_frontier_goal_yaw_change_rad").value
            ),
            minimum_goal_arc_yaw_change_rad=float(
                self.get_parameter("minimum_frontier_arc_yaw_change_rad").value
            ),
            minimum_turning_radius_m=float(
                self.get_parameter("minimum_turning_radius_m").value
            ),
            boundary_turn_buffer_m=float(
                self.get_parameter("boundary_turn_buffer_m").value
            ),
            preferred_world_xy=sweep_preference,
        )

    def _start_sweep_lane_shift_backup(self, robot_pose) -> bool:
        """Back inward once before an Ackermann lane-shift turn at an edge."""
        if self.sweep_active_axis != "vertical":
            return False
        index = self.sweep_target_index
        if index in self.sweep_lane_shift_backup_completed:
            return False
        if self.sweep_lane_shift_backup_pending == index:
            return True
        target = self.sweep_targets[index]
        candidates = lane_shift_connector_goals(
            robot_pose,
            target[1],
            candidate_distances_m=tuple(
                float(value) for value in self.get_parameter(
                    "frontier_sweep_lane_shift_connector_distances_m"
                ).value
            ),
            allowed_bounds_xyxy_m=self.required_bounds,
            boundary_margin_m=float(
                self.get_parameter("required_bounds_goal_margin_m").value
            ),
        )
        # BackUp is a safety-checked fallback, not a mandatory maneuver.  At a
        # boundary with a non-tangent heading the rear collision envelope may
        # correctly stop reverse motion even though a forward Dubins turn is
        # fully known and clear.  Prefer that forward path and never bypass the
        # Collision Monitor merely to force a backup through.
        if any(
            self._forward_sweep_lane_shift_path(robot_pose, candidate)
            for candidate in candidates
            if self._goal_is_costmap_clear(candidate)
        ):
            self.sweep_lane_shift_backup_completed.add(index)
            self.sweep_lane_shift_backup_skipped.add(index)
            self.sweep_lane_shift_locked_x.setdefault(index, robot_pose[0])
            return False
        attempts = self.sweep_lane_shift_backup_attempts.get(index, 0)
        if attempts >= int(
            self.get_parameter(
                "frontier_sweep_lane_shift_backup_max_attempts"
            ).value
        ):
            self._finish(False, "sweep_lane_shift_backup_exhausted")
            return True
        escape = reverse_escape_goal(
            robot_pose[:2],
            robot_pose[2],
            distance_m=float(
                self.get_parameter(
                    "frontier_sweep_lane_shift_backup_distance_m"
                ).value
            ),
            allowed_bounds_xyxy_m=self.required_bounds,
            boundary_margin_m=float(
                self.get_parameter("required_bounds_goal_margin_m").value
            ),
        )
        if escape is None or not self._goal_is_costmap_clear(escape):
            self.last_error = "sweep_lane_shift_backup_unavailable"
            return False
        if not self._send_backup(
            escape,
            goal_kind="lane_shift_backup",
            sweep_target_index=index,
        ):
            self.last_error = "backup_action_server_unavailable"
            return False
        self.sweep_lane_shift_backup_pending = index
        self.sweep_lane_shift_backup_count += 1
        self.sweep_lane_shift_backup_attempts[index] = attempts + 1
        self.last_error = "sweep_lane_shift_backup"
        return True

    def _sweep_preference(self, robot_pose):
        """Keep one bounds-derived sweep target until the chassis reaches it."""
        if not bool(self.get_parameter("frontier_sweep_enabled").value):
            self.sweep_active_anchor = None
            self.sweep_active_preference = None
            self.sweep_active_axis = None
            return None
        if not self.sweep_targets:
            sweep_reference = tuple(
                float(value)
                for value in self.get_parameter(
                    "frontier_sweep_reference_pose_xyyaw_m_rad"
                ).value
            )
            if len(sweep_reference) != 3:
                raise ValueError(
                    "frontier_sweep_reference_pose_xyyaw_m_rad must contain three values"
                )
            self.sweep_targets = frontier_sweep_targets(
                self.required_bounds,
                sweep_reference[:2],
                sweep_reference[2],
                sensor_range_m=float(
                    self.get_parameter("mapping_sensor_range_m").value
                ),
                lane_overlap_m=float(
                    self.get_parameter("frontier_sweep_lane_overlap_m").value
                ),
                boundary_margin_m=float(
                    self.get_parameter("required_bounds_goal_margin_m").value
                ),
            )
        tolerance = max(0.0, float(
            self.get_parameter("frontier_sweep_target_tolerance_m").value
        ))
        while self.sweep_target_index < len(self.sweep_targets):
            target = self.sweep_targets[self.sweep_target_index]
            axis = frontier_sweep_target_axis(
                self.sweep_targets, self.sweep_target_index
            )
            pose_reached = math.hypot(
                target[0] - robot_pose[0], target[1] - robot_pose[1]
            ) <= tolerance
            mapped_radius = float(
                self.get_parameter(
                    "frontier_sweep_mapped_target_radius_m"
                ).value
            )
            preference = target
            if axis == "vertical":
                previous_y = self.sweep_targets[
                    self.sweep_target_index - 1
                ][1]
                envelope = (self.latest_metrics or {}).get(
                    "mapped_envelope_bounds_xyxy_m"
                )
                mapped_reached = vertical_sweep_anchor_reached(
                    envelope,
                    previous_y_m=previous_y,
                    target_y_m=target[1],
                    radius_m=mapped_radius,
                )
                pose_reached = abs(target[1] - robot_pose[1]) <= tolerance
                locked_x = robot_pose[0]
                if self.sweep_target_index in self.sweep_lane_shift_backup_completed:
                    locked_x = self.sweep_lane_shift_locked_x.setdefault(
                        self.sweep_target_index, robot_pose[0]
                    )
                preference = (locked_x, target[1])
            else:
                mapped_reached = world_disk_has_known_cell(
                    self.latest_data,
                    self.latest_geometry,
                    target,
                    radius_m=mapped_radius,
                )
            if not pose_reached and not mapped_reached:
                self.sweep_active_anchor = target
                self.sweep_active_preference = preference
                self.sweep_active_axis = axis
                return preference
            self.sweep_target_index += 1
        self.sweep_completed = True
        self.sweep_active_anchor = None
        self.sweep_active_preference = None
        self.sweep_active_axis = None
        return None

    def _start_sweep_lane_shift_connector(self, robot_pose) -> bool:
        """Turn onto the next sweep lane through a frozen feasible path.

        Frontier waypoints intentionally ignore terminal yaw.  That is correct
        for short observation hops, but it means a sequence of synthesized arc
        endpoints cannot change the vehicle's actual heading at a lane edge.
        After collision-checked BackUp creates room, prefer a forward-only
        Dubins path derived from the fused pose and verify every sample against
        the online costmap.  If that path is unavailable, ask Hybrid-A*/
        Reeds-Shepp once, split the frozen result at every direction cusp, and
        execute each section through a direction-constrained FollowPath
        controller.  No BT replanning may change the first gear in place.
        """
        if self.sweep_active_axis != "vertical":
            return False
        index = self.sweep_target_index
        if index not in self.sweep_lane_shift_backup_completed:
            return False
        if index in self.sweep_lane_shift_connector_completed:
            return False
        if self.sweep_lane_shift_connector_pending == index:
            if (
                self.active_goal is None
                and self.sweep_lane_shift_connector_sections
            ):
                self._send_next_sweep_lane_shift_section()
            return True
        target = self.sweep_targets[index]
        candidates = lane_shift_connector_goals(
            robot_pose,
            target[1],
            candidate_distances_m=tuple(
                float(value) for value in self.get_parameter(
                    "frontier_sweep_lane_shift_connector_distances_m"
                ).value
            ),
            allowed_bounds_xyxy_m=self.required_bounds,
            boundary_margin_m=float(
                self.get_parameter("required_bounds_goal_margin_m").value
            ),
        )
        attempt = self.sweep_lane_shift_connector_attempts.get(index, 0)
        clear_candidates = [
            candidate for candidate in candidates
            if self._goal_is_costmap_clear(candidate)
        ]
        if attempt >= len(clear_candidates):
            self._finish(False, "sweep_lane_shift_connector_exhausted")
            return True
        goal = clear_candidates[attempt]
        self.sweep_lane_shift_connector_attempts[index] = attempt + 1
        self.sweep_lane_shift_connector_pending = index
        self.last_error = "sweep_lane_shift_connector"
        self._send_sweep_lane_shift_plan(goal, index, robot_pose)
        return True

    def _set_lane_shift_active_goal(self, goal, handle=None) -> None:
        self.active_goal = goal
        self.active_goal_handle = handle
        self.active_goal_started_monotonic = time.monotonic()
        self.active_goal_timeout_sec = float(
            self.get_parameter("lane_shift_connector_timeout_sec").value
        )
        self.active_goal_cancel_requested = False

    def _clear_active_goal(self) -> None:
        self.active_goal = None
        self.active_goal_handle = None
        self.active_goal_started_monotonic = None
        self.active_goal_timeout_sec = float(
            self.get_parameter("goal_timeout_sec").value
        )
        self.active_goal_cancel_requested = False

    def _path_poses_are_costmap_clear(self, poses) -> bool:
        return all(
            world_disk_is_traversable(
                self.latest_costmap_data,
                self.latest_costmap_geometry,
                (pose[0], pose[1]),
                radius_m=float(
                    self.get_parameter("goal_clearance_radius_m").value
                ),
                maximum_cost=int(self.get_parameter("maximum_goal_cost").value),
            )
            for pose in poses
        )

    def _forward_sweep_lane_shift_path(self, robot_pose, goal):
        margin = float(
            self.get_parameter("required_bounds_goal_margin_m").value
        )
        min_x, min_y, max_x, max_y = self.required_bounds
        apron = [
            (min_x + margin, min_y + margin),
            (max_x - margin, min_y + margin),
            (max_x - margin, max_y - margin),
            (min_x + margin, max_y - margin),
        ]
        path = plan_forward_dubins_path(
            robot_pose,
            (goal.world_x_m, goal.world_y_m, goal.yaw_rad),
            apron,
            [],
        )
        return path if path and self._path_poses_are_costmap_clear(path) else None

    @staticmethod
    def _forward_dubins_sections(path):
        points = [(pose[0], pose[1]) for pose in path]
        headings = [pose[2] for pose in path]
        primitives = split_path_at_curvature_reversals(points, headings)
        sections = []
        for index, (primitive_points, primitive_headings) in enumerate(primitives):
            sections.append({
                "direction": "FORWARD",
                "poses": [
                    (point[0], point[1], heading)
                    for point, heading in zip(
                        primitive_points, primitive_headings
                    )
                ],
                "cusp_before": False,
                "controller_id": "DubinsPath",
                "goal_checker_id": (
                    "connector_goal_checker"
                    if index == len(primitives) - 1
                    else "primitive_goal_checker"
                ),
            })
        return sections

    def _accept_sweep_lane_shift_sections(
        self,
        sections,
        *,
        planned_path_pose_count: int,
        planner_id: str,
    ) -> None:
        row = self.sweep_lane_shift_connector_row
        if row is None:
            return
        row["accepted"] = True
        row["planner_id"] = planner_id
        row["planned_path_pose_count"] = int(planned_path_pose_count)
        row["planned_section_directions"] = [
            section["direction"] for section in sections
        ]
        row["planned_sections"] = [
            {
                "direction": section["direction"],
                "controller_id": section.get("controller_id", "ConnectorPath"),
                "goal_checker_id": section.get(
                    "goal_checker_id", "connector_goal_checker"
                ),
                "pose_count": len(section["poses"]),
                "start_pose": list(section["poses"][0]),
                "end_pose": list(section["poses"][-1]),
            }
            for section in sections
        ]
        row["path_costmap_clearance_checked"] = True
        self.sweep_lane_shift_connector_sections = sections
        self.sweep_lane_shift_connector_section_index = 0
        self._clear_active_goal()
        self._send_next_sweep_lane_shift_section()

    def _send_sweep_lane_shift_plan(
        self, goal, sweep_target_index: int, robot_pose
    ) -> None:
        if not self.compute_path_client.server_is_ready():
            self._fail_sweep_lane_shift_connector(
                GoalStatus.STATUS_ABORTED, "compute_path_server_unavailable"
            )
            return
        row = {
            **asdict(goal),
            "goal_kind": "lane_shift_connector",
            "execution": "analytic_forward_dubins_then_smac_fallback",
            "sweep_target_index": sweep_target_index,
            "sequence": len(self.goal_history) + 1,
            "pre_connector_backup": (
                "skipped_online_costmap_clear_forward_dubins"
                if sweep_target_index in self.sweep_lane_shift_backup_skipped
                else "nav2_behaviors_BackUp_collision_checked"
            ),
            "accepted": None,
            "terminal_status": None,
            "planned_path_pose_count": None,
            "planned_section_directions": [],
            "completed_section_count": 0,
        }
        self.goal_history.append(row)
        self.sweep_lane_shift_connector_row = row
        self.sweep_lane_shift_connector_goal = goal
        self.sweep_lane_shift_connector_sections = []
        self.sweep_lane_shift_connector_section_index = 0
        self._set_lane_shift_active_goal(goal)

        forward_path = self._forward_sweep_lane_shift_path(robot_pose, goal)
        if forward_path:
            row["execution"] = "online_costmap_checked_forward_dubins"
            sections = self._forward_dubins_sections(forward_path)
            self._accept_sweep_lane_shift_sections(
                sections,
                planned_path_pose_count=len(forward_path),
                planner_id="curvature_segmented_analytic_forward_dubins",
            )
            return
        row["analytic_forward_dubins_fallback_reason"] = (
            "costmap_clearance_failed" if forward_path else "no_feasible_path"
        )
        message = ComputePathToPose.Goal()
        message.goal = self._pose_stamped(goal)
        message.planner_id = "GridBased"
        message.use_start = False
        future = self.compute_path_client.send_goal_async(message)
        future.add_done_callback(self._on_sweep_lane_shift_plan_response)
        self._write_report()

    def _on_sweep_lane_shift_plan_response(self, future) -> None:
        handle = future.result()
        row = self.sweep_lane_shift_connector_row
        if row is None:
            return
        row["accepted"] = bool(handle and handle.accepted)
        if handle is None or not handle.accepted:
            self._fail_sweep_lane_shift_connector(
                GoalStatus.STATUS_ABORTED, "compute_path_rejected"
            )
            return
        self.active_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_sweep_lane_shift_plan_result)

    def _on_sweep_lane_shift_plan_result(self, future) -> None:
        wrapped = future.result()
        status = int(wrapped.status)
        result = wrapped.result
        error_code = int(getattr(result, "error_code", 0))
        if status != GoalStatus.STATUS_SUCCEEDED or error_code != 0:
            self._fail_sweep_lane_shift_connector(
                status, f"compute_path_failed:{error_code}"
            )
            return
        path_poses = [
            (
                float(item.pose.position.x),
                float(item.pose.position.y),
                _yaw_from_quaternion(item.pose.orientation),
            )
            for item in result.path.poses
        ]
        sections = split_hybrid_path_by_direction(path_poses)
        if not sections:
            self._fail_sweep_lane_shift_connector(
                GoalStatus.STATUS_ABORTED, "compute_path_has_no_motion_sections"
            )
            return
        for section in sections:
            section["controller_id"] = (
                "ReversePath"
                if section["direction"] == "REVERSE"
                else "ConnectorPath"
            )
        if not self._path_poses_are_costmap_clear(
            pose for section in sections for pose in section["poses"]
        ):
            self._fail_sweep_lane_shift_connector(
                GoalStatus.STATUS_ABORTED, "planned_path_costmap_clearance_failed"
            )
            return
        self._accept_sweep_lane_shift_sections(
            sections,
            planned_path_pose_count=len(path_poses),
            planner_id="smac_hybrid_reeds_shepp",
        )

    def _send_next_sweep_lane_shift_section(self) -> None:
        index = self.sweep_lane_shift_connector_section_index
        if index >= len(self.sweep_lane_shift_connector_sections):
            self._complete_sweep_lane_shift_connector()
            return
        if not self.follow_path_client.server_is_ready():
            self._fail_sweep_lane_shift_connector(
                GoalStatus.STATUS_ABORTED, "follow_path_server_unavailable"
            )
            return
        section = self.sweep_lane_shift_connector_sections[index]
        message = FollowPath.Goal()
        message.path = self._path_message(section["poses"])
        message.controller_id = section.get("controller_id", "ConnectorPath")
        message.goal_checker_id = section.get(
            "goal_checker_id",
            "cusp_goal_checker"
            if index < len(self.sweep_lane_shift_connector_sections) - 1
            else "connector_goal_checker",
        )
        message.progress_checker_id = "progress_checker"
        self._set_lane_shift_active_goal(self.sweep_lane_shift_connector_goal)
        future = self.follow_path_client.send_goal_async(message)
        future.add_done_callback(self._on_sweep_lane_shift_section_response)
        self._write_report()

    def _on_sweep_lane_shift_section_response(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self._fail_sweep_lane_shift_connector(
                GoalStatus.STATUS_ABORTED, "follow_path_rejected"
            )
            return
        self.active_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_sweep_lane_shift_section_result)

    def _on_sweep_lane_shift_section_result(self, future) -> None:
        wrapped = future.result()
        status = int(wrapped.status)
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._fail_sweep_lane_shift_connector(status, "follow_path_failed")
            return
        self.sweep_lane_shift_connector_section_index += 1
        row = self.sweep_lane_shift_connector_row
        if row is not None:
            row["completed_section_count"] = (
                self.sweep_lane_shift_connector_section_index
            )
        self._clear_active_goal()
        self.next_goal_not_before_monotonic = time.monotonic() + 1.0
        self._write_report()

    def _complete_sweep_lane_shift_connector(self) -> None:
        index = self.sweep_lane_shift_connector_pending
        row = self.sweep_lane_shift_connector_row
        if row is not None:
            row["terminal_status"] = GoalStatus.STATUS_SUCCEEDED
            row["succeeded"] = True
        if index is not None:
            self.sweep_lane_shift_connector_completed.add(int(index))
        self.sweep_lane_shift_connector_pending = None
        self.sweep_lane_shift_connector_sections = []
        self.sweep_lane_shift_connector_section_index = 0
        self.sweep_lane_shift_connector_goal = None
        self.sweep_lane_shift_connector_row = None
        self._clear_active_goal()
        self._update_adaptive_goal_distance(True)
        self._write_report()

    def _fail_sweep_lane_shift_connector(self, status: int, error: str) -> None:
        row = self.sweep_lane_shift_connector_row
        if row is not None:
            row["terminal_status"] = int(status)
            row["succeeded"] = False
            row["error"] = error
        self.last_error = error
        self.sweep_lane_shift_connector_pending = None
        self.sweep_lane_shift_connector_sections = []
        self.sweep_lane_shift_connector_section_index = 0
        self.sweep_lane_shift_connector_goal = None
        self.sweep_lane_shift_connector_row = None
        self._clear_active_goal()
        self._update_adaptive_goal_distance(False)
        self.next_goal_not_before_monotonic = time.monotonic() + float(
            self.get_parameter("failed_goal_cooldown_sec").value
        )
        self._write_report()

    def _pose_stamped(self, goal) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = str(self.get_parameter("map_frame").value)
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = goal.world_x_m
        message.pose.position.y = goal.world_y_m
        message.pose.orientation.z = math.sin(goal.yaw_rad / 2.0)
        message.pose.orientation.w = math.cos(goal.yaw_rad / 2.0)
        return message

    def _path_message(self, poses) -> NavPath:
        message = NavPath()
        message.header.frame_id = str(self.get_parameter("map_frame").value)
        message.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in poses:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
            pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
            message.poses.append(pose)
        return message

    def _add_excluded_goal(self, world_x_m: float, world_y_m: float) -> None:
        ttl = max(0.0, float(
            self.get_parameter("failed_goal_exclusion_ttl_sec").value
        ))
        self.excluded_goals.append((
            float(world_x_m),
            float(world_y_m),
            time.monotonic() + ttl,
        ))

    def _send_goal(self, goal, *, goal_kind: str = "frontier") -> None:
        message = NavigateToPose.Goal()
        message.pose = PoseStamped()
        message.pose.header.frame_id = str(self.get_parameter("map_frame").value)
        message.pose.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = goal.world_x_m
        message.pose.pose.position.y = goal.world_y_m
        message.pose.pose.orientation.z = math.sin(goal.yaw_rad / 2.0)
        message.pose.pose.orientation.w = math.cos(goal.yaw_rad / 2.0)
        message.behavior_tree = str(self.get_parameter("behavior_tree").value)
        row = {
            **asdict(goal),
            "goal_kind": goal_kind,
            "behavior_tree": message.behavior_tree,
            "sequence": len(self.goal_history) + 1,
            "accepted": None,
            "terminal_status": None,
        }
        self.goal_history.append(row)
        self.active_goal = goal
        self.active_goal_started_monotonic = time.monotonic()
        self.active_goal_cancel_requested = False
        future = self.action_client.send_goal_async(message)
        future.add_done_callback(self._on_goal_response)
        self._write_report()

    def _send_backup(
        self,
        goal,
        *,
        goal_kind: str = "reverse_escape",
        sweep_target_index: int | None = None,
    ) -> bool:
        """Execute a collision-checked straight reverse without global replanning."""
        if not self.backup_client.server_is_ready():
            return False
        message = BackUp.Goal()
        message.target.x = -goal.distance_m
        message.speed = -abs(float(
            self.get_parameter("reverse_escape_speed_mps").value
        ))
        message.time_allowance.sec = max(1, int(math.ceil(float(
            self.get_parameter("goal_timeout_sec").value
        ))))
        row = {
            **asdict(goal),
            "goal_kind": goal_kind,
            "execution": "nav2_behaviors_BackUp_collision_checked",
            "sweep_target_index": sweep_target_index,
            "sequence": len(self.goal_history) + 1,
            "accepted": None,
            "terminal_status": None,
        }
        self.goal_history.append(row)
        self.active_goal = goal
        self.active_goal_started_monotonic = time.monotonic()
        self.active_goal_cancel_requested = False
        future = self.backup_client.send_goal_async(message)
        future.add_done_callback(self._on_goal_response)
        self._write_report()
        return True

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        self.goal_history[-1]["accepted"] = bool(handle and handle.accepted)
        if handle is None or not handle.accepted:
            if self.goal_history[-1].get("goal_kind") == "lane_shift_backup":
                self.sweep_lane_shift_backup_pending = None
            self._update_adaptive_goal_distance(False)
            self._add_excluded_goal(
                self.active_goal.world_x_m, self.active_goal.world_y_m
            )
            self.active_goal = None
            self.active_goal_handle = None
            self.active_goal_started_monotonic = None
            self.active_goal_cancel_requested = False
            self.next_goal_not_before_monotonic = time.monotonic() + float(
                self.get_parameter("failed_goal_cooldown_sec").value
            )
            self._write_report()
            return
        self.active_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        wrapped = future.result()
        status = int(wrapped.status)
        timed_out = self.active_goal_cancel_requested
        self.goal_history[-1]["terminal_status"] = status
        self.goal_history[-1]["succeeded"] = (
            status == GoalStatus.STATUS_SUCCEEDED
        )
        if self.goal_history[-1].get("goal_kind") == "lane_shift_backup":
            index = self.goal_history[-1].get("sweep_target_index")
            if status == GoalStatus.STATUS_SUCCEEDED and index is not None:
                self.sweep_lane_shift_backup_completed.add(int(index))
            self.sweep_lane_shift_backup_pending = None
        self._update_adaptive_goal_distance(
            status == GoalStatus.STATUS_SUCCEEDED
        )
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._exclude_failed_goal(
                self.active_goal,
                wide_exclusion=(
                    timed_out or status == GoalStatus.STATUS_ABORTED
                ),
            )
            self.next_goal_not_before_monotonic = time.monotonic() + float(
                self.get_parameter("failed_goal_cooldown_sec").value
            )
        self.active_goal = None
        self.active_goal_handle = None
        self.active_goal_started_monotonic = None
        self.active_goal_cancel_requested = False
        if timed_out:
            self._begin_nav_recovery()
        self._write_report()

    def _update_adaptive_goal_distance(self, succeeded: bool) -> None:
        (
            self.adaptive_goal_distance_m,
            self.goal_distance_success_streak,
        ) = next_adaptive_goal_distance(
            self.adaptive_goal_distance_m,
            self.goal_distance_success_streak,
            succeeded=succeeded,
            minimum_distance_m=float(
                self.get_parameter("initial_frontier_goal_distance_m").value
            ),
            maximum_distance_m=float(
                self.get_parameter("maximum_frontier_goal_distance_m").value
            ),
            successes_per_growth=int(
                self.get_parameter("goal_distance_growth_success_count").value
            ),
            growth_step_m=float(
                self.get_parameter("goal_distance_growth_step_m").value
            ),
        )

    def _exclude_failed_goal(self, goal, *, wide_exclusion: bool) -> None:
        center = (goal.world_x_m, goal.world_y_m)
        self._add_excluded_goal(*center)
        if not wide_exclusion:
            return
        base_radius = float(
            self.get_parameter("failed_goal_exclusion_radius_m").value
        )
        timeout_radius = float(
            self.get_parameter("timed_out_goal_exclusion_radius_m").value
        )
        ring_radius = max(0.0, timeout_radius - base_radius)
        if ring_radius <= 1.0e-9:
            return
        for index in range(8):
            angle = index * math.pi / 4.0
            self._add_excluded_goal(
                center[0] + ring_radius * math.cos(angle),
                center[1] + ring_radius * math.sin(angle),
            )

    def _begin_nav_recovery(self) -> None:
        self.nav_recovery_count += 1
        if not self.nav_manager_client.service_is_ready():
            self.nav_recovery_status = "manager_service_unavailable"
            self.next_goal_not_before_monotonic = time.monotonic() + 30.0
            return
        self.nav_recovery_in_progress = True
        self.nav_recovery_status = "reset_requested"
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.RESET
        future = self.nav_manager_client.call_async(request)
        future.add_done_callback(self._on_nav_reset)

    def _on_nav_reset(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - middleware failure path
            self.nav_recovery_in_progress = False
            self.nav_recovery_status = f"reset_exception:{error}"
            self.next_goal_not_before_monotonic = time.monotonic() + 30.0
            self._write_report()
            return
        if response is None or not response.success:
            self.nav_recovery_in_progress = False
            self.nav_recovery_status = "reset_failed"
            self.next_goal_not_before_monotonic = time.monotonic() + 30.0
            self._write_report()
            return
        self.nav_recovery_status = "startup_requested"
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        startup = self.nav_manager_client.call_async(request)
        startup.add_done_callback(self._on_nav_startup)

    def _on_nav_startup(self, future) -> None:
        try:
            response = future.result()
            success = bool(response and response.success)
        except Exception as error:  # pragma: no cover - middleware failure path
            success = False
            self.nav_recovery_status = f"startup_exception:{error}"
        if success:
            self.nav_recovery_status = "recovered"
            self.next_goal_not_before_monotonic = time.monotonic() + 5.0
        elif not self.nav_recovery_status.startswith("startup_exception:"):
            self.nav_recovery_status = "startup_failed"
            self.next_goal_not_before_monotonic = time.monotonic() + 30.0
        self.nav_recovery_in_progress = False
        self._write_report()

    def _finish(self, success: bool, reason: str) -> None:
        self.terminal = True
        self.success = success
        self.terminal_reason = reason
        self._write_report()
        self.get_logger().info(
            json.dumps(
                {"success": success, "terminal_reason": reason},
                ensure_ascii=False,
            )
        )

    def _report(self) -> dict:
        metrics = self.latest_metrics or {}
        required_area = metrics.get("required_bounds_area_m2")
        coverage = metrics.get("required_bounds_known_coverage_ratio")
        mapping_area = metrics.get("known_area_m2") or 0.0
        return {
            "schema_version": 1,
            "stage": "PRODUCT-MAPPING-EXPLORATION",
            "success": self.success,
            "terminal": self.terminal,
            "terminal_reason": self.terminal_reason,
            "elapsed_wall_sec": time.monotonic() - self.started_monotonic,
            "mapping_area_m2": mapping_area,
            "required_bounds_xyxy_m": list(self.required_bounds),
            "required_bounds_coverage_threshold": float(
                self.get_parameter("required_bounds_coverage_ratio").value
            ),
            "completion_basis": (
                "required_bounds_envelope_coverage_and_total_known_area"
            ),
            "required_bounds_goal_margin_m": float(
                self.get_parameter("required_bounds_goal_margin_m").value
            ),
            "frontier_connection_radius_cells": int(
                self.get_parameter("frontier_connection_radius_cells").value
            ),
            "frontier_goal_backoff_m": float(
                self.get_parameter("frontier_goal_backoff_m").value
            ),
            "maximum_frontier_goal_distance_m": float(
                self.get_parameter("maximum_frontier_goal_distance_m").value
            ),
            "adaptive_frontier_goal_distance_m": self.adaptive_goal_distance_m,
            "goal_distance_success_streak": self.goal_distance_success_streak,
            "maximum_frontier_goal_yaw_change_rad": float(
                self.get_parameter("maximum_frontier_goal_yaw_change_rad").value
            ),
            "minimum_frontier_arc_yaw_change_rad": float(
                self.get_parameter("minimum_frontier_arc_yaw_change_rad").value
            ),
            "goal_timeout_sec": float(
                self.get_parameter("goal_timeout_sec").value
            ),
            "failed_goal_cooldown_sec": float(
                self.get_parameter("failed_goal_cooldown_sec").value
            ),
            "failed_goal_exclusion_ttl_sec": float(
                self.get_parameter("failed_goal_exclusion_ttl_sec").value
            ),
            "active_failed_goal_exclusion_count": len(self.excluded_goals),
            "frontier_exclusion_wait_count": self.frontier_exclusion_wait_count,
            "reverse_escape_goal_count": self.reverse_escape_goal_count,
            "reverse_escape_distance_m": float(
                self.get_parameter("reverse_escape_distance_m").value
            ),
            "reverse_escape_speed_mps": float(
                self.get_parameter("reverse_escape_speed_mps").value
            ),
            "reverse_escape_execution": (
                "nav2_behaviors_BackUp_collision_checked"
            ),
            "frontier_sweep_enabled": bool(
                self.get_parameter("frontier_sweep_enabled").value
            ),
            "frontier_sweep_source": (
                "required_bounds_and_configured_lidar_range"
            ),
            "mapping_sensor_range_m": float(
                self.get_parameter("mapping_sensor_range_m").value
            ),
            "frontier_sweep_lane_overlap_m": float(
                self.get_parameter("frontier_sweep_lane_overlap_m").value
            ),
            "frontier_sweep_target_tolerance_m": float(
                self.get_parameter("frontier_sweep_target_tolerance_m").value
            ),
            "frontier_sweep_mapped_target_radius_m": float(
                self.get_parameter(
                    "frontier_sweep_mapped_target_radius_m"
                ).value
            ),
            "frontier_sweep_lane_shift_backup_distance_m": float(
                self.get_parameter(
                    "frontier_sweep_lane_shift_backup_distance_m"
                ).value
            ),
            "frontier_sweep_lane_shift_backup_count": (
                self.sweep_lane_shift_backup_count
            ),
            "frontier_sweep_lane_shift_backup_max_attempts": int(
                self.get_parameter(
                    "frontier_sweep_lane_shift_backup_max_attempts"
                ).value
            ),
            "frontier_sweep_lane_shift_backup_attempts_by_index": {
                str(index): attempts
                for index, attempts in sorted(
                    self.sweep_lane_shift_backup_attempts.items()
                )
            },
            "frontier_sweep_lane_shift_backup_completed_indices": sorted(
                self.sweep_lane_shift_backup_completed
            ),
            "frontier_sweep_lane_shift_backup_skipped_indices": sorted(
                self.sweep_lane_shift_backup_skipped
            ),
            "frontier_sweep_lane_shift_locked_x_by_index": {
                str(index): value
                for index, value in sorted(self.sweep_lane_shift_locked_x.items())
            },
            "frontier_sweep_lane_shift_connector_distances_m": [
                float(value) for value in self.get_parameter(
                    "frontier_sweep_lane_shift_connector_distances_m"
                ).value
            ],
            "frontier_sweep_lane_shift_connector_execution": (
                "analytic_forward_dubins_then_smac_fallback"
            ),
            "frontier_sweep_lane_shift_connector_timeout_sec": float(
                self.get_parameter("lane_shift_connector_timeout_sec").value
            ),
            "frontier_sweep_lane_shift_connector_completed_indices": sorted(
                self.sweep_lane_shift_connector_completed
            ),
            "frontier_sweep_lane_shift_connector_attempts_by_index": {
                str(index): value for index, value in sorted(
                    self.sweep_lane_shift_connector_attempts.items()
                )
            },
            "frontier_sweep_target_index": self.sweep_target_index,
            "frontier_sweep_target_count": len(self.sweep_targets),
            "frontier_sweep_targets_xy_m": [
                list(target) for target in self.sweep_targets
            ],
            "frontier_sweep_active_target_xy_m": (
                list(self.sweep_active_anchor)
                if self.sweep_active_anchor is not None else None
            ),
            "frontier_sweep_active_preference_xy_m": (
                list(self.sweep_active_preference)
                if self.sweep_active_preference is not None else None
            ),
            "frontier_sweep_active_axis": self.sweep_active_axis,
            "frontier_sweep_completed": self.sweep_completed,
            "timed_out_goal_exclusion_radius_m": float(
                self.get_parameter("timed_out_goal_exclusion_radius_m").value
            ),
            "nav_recovery_count": self.nav_recovery_count,
            "nav_recovery_status": self.nav_recovery_status,
            "minimum_turning_radius_m": float(
                self.get_parameter("minimum_turning_radius_m").value
            ),
            "boundary_turn_buffer_m": float(
                self.get_parameter("boundary_turn_buffer_m").value
            ),
            "map_update_count": self.map_update_count,
            "costmap_rejected_goal_count": self.costmap_rejected_goal_count,
            "map_metrics": metrics,
            "goal_count": len(self.goal_history),
            "goal_success_count": sum(
                row.get("succeeded") is True for row in self.goal_history
            ),
            "goal_failure_count": sum(
                row.get("succeeded") is False for row in self.goal_history
            ),
            "goals": self.goal_history,
            "last_pose_map": list(self.last_pose) if self.last_pose else None,
            "last_error": self.last_error,
            "ground_truth_used_for_control": False,
            "ground_truth_subscription_count": 0,
            "preknown_target_coordinates": False,
            "control_input": (
                "online_occupancy_grid_frontiers_map_to_base_tf_and_"
                "bounds_derived_lidar_sweep_preference"
                if bool(self.get_parameter("frontier_sweep_enabled").value)
                else "online_occupancy_grid_frontiers_and_map_to_base_tf"
            ),
            "positioning_source": str(
                self.get_parameter("positioning_source").value
            ),
            "behavior_tree": str(self.get_parameter("behavior_tree").value),
        }

    def _write_report(self) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output_path.with_suffix(self.output_path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(self._report(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.output_path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        while rclpy.ok() and not node.terminal:
            rclpy.spin_once(node, timeout_sec=0.2)
        code = 0 if node.success else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
