"""ROS action-level regression for route-aware frontier detours."""

from __future__ import annotations

import math
from pathlib import Path
import threading
import time

import pytest


rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav2_msgs.action import ComputePathToPose, NavigateToPose  # noqa: E402
from nav_msgs.msg import Path as NavPath  # noqa: E402
from rclpy.action import ActionServer  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from sanitation_tasks.frontier_core import (  # noqa: E402
    FrontierGoal,
    GridGeometry,
)
from sanitation_tasks.frontier_explorer import FrontierExplorer  # noqa: E402


class _PlannerHarness(Node):
    def __init__(self, *, abort_first_navigation: bool = False) -> None:
        super().__init__("frontier_detour_pipeline_harness")
        self.navigation_goal = None
        self.navigation_completed = threading.Event()
        self.first_navigation_completed = threading.Event()
        self.navigation_count = 0
        self.abort_first_navigation = abort_first_navigation
        self.compute_server = ActionServer(
            self,
            ComputePathToPose,
            "/compute_path_to_pose",
            self._compute_path,
        )
        self.navigation_server = ActionServer(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            self._navigate,
        )

    def _compute_path(self, goal_handle):
        request = goal_handle.request
        result = ComputePathToPose.Result()
        result.error_code = 0
        result.path = NavPath()
        result.path.header.frame_id = "map"
        route = [
            (70.2, 47.6, math.pi),
            (72.0, 48.0, math.atan2(0.4, 1.8)),
            (72.0, 35.0, -math.pi / 2.0),
            (
                float(request.goal.pose.position.x),
                float(request.goal.pose.position.y),
                math.pi,
            ),
        ]
        for x, y, yaw in route:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            result.path.poses.append(pose)
        goal_handle.succeed()
        return result

    def _navigate(self, goal_handle):
        self.navigation_count += 1
        pose = goal_handle.request.pose.pose
        self.navigation_goal = (
            float(pose.position.x),
            float(pose.position.y),
        )
        if self.abort_first_navigation and self.navigation_count == 1:
            goal_handle.abort()
            self.first_navigation_completed.set()
        else:
            goal_handle.succeed()
            self.navigation_completed.set()
        return NavigateToPose.Result()


def _obstacle_costmap():
    geometry = GridGeometry(
        width=800,
        height=480,
        resolution_m=0.25,
        origin_x_m=-120.0,
        origin_y_m=-70.0,
    )
    data = [0] * (geometry.width * geometry.height)
    for grid_y in range(geometry.height):
        y = geometry.origin_y_m + (grid_y + 0.5) * geometry.resolution_m
        if not 38.5 <= y <= 47.5:
            continue
        for grid_x in range(geometry.width):
            x = geometry.origin_x_m + (grid_x + 0.5) * geometry.resolution_m
            if 51.0 <= x <= 69.0:
                data[grid_y * geometry.width + grid_x] = 100
    return tuple(data), geometry


def _unknown_route_band_costmap(geometry):
    data = [0] * (geometry.width * geometry.height)
    for grid_x in range(geometry.width):
        x = geometry.origin_x_m + (grid_x + 0.5) * geometry.resolution_m
        if not 58.0 <= x <= 60.0:
            continue
        for grid_y in range(geometry.height):
            data[grid_y * geometry.width + grid_x] = -1
    return tuple(data)


def test_blocked_projection_uses_global_route_lookahead(tmp_path: Path):
    rclpy.init()
    explorer = FrontierExplorer()
    harness = _PlannerHarness()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(explorer)
    executor.add_node(harness)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        assert explorer.compute_path_client.wait_for_server(timeout_sec=3.0)
        assert explorer.action_client.wait_for_server(timeout_sec=3.0)
        explorer.terminal = True
        explorer.output_path = tmp_path / "frontier_detour_pipeline.json"
        obstacle_map, geometry = _obstacle_costmap()
        explorer.latest_data = obstacle_map
        explorer.latest_geometry = geometry
        explorer.latest_costmap_data = tuple(0 for _ in obstacle_map)
        explorer.latest_costmap_geometry = geometry
        explorer.latest_metrics = {"known_area_m2": 9705.93}
        explorer.map_update_count = 22190
        source_goal = FrontierGoal(
            grid_x=17,
            grid_y=23,
            world_x_m=67.2,
            world_y_m=47.4,
            yaw_rad=math.pi,
            frontier_cell_count=100,
            information_gain_m=10.0,
            distance_m=3.0,
            score=9.25,
            preference_distance_m=12.0,
            raw_world_x_m=-94.0,
            raw_world_y_m=39.0,
        )
        assert explorer._goal_clearance_sources(source_goal) == (False, True)
        assert not explorer._goal_is_costmap_clear(source_goal)
        edge_route_goal = FrontierGoal(
            grid_x=0,
            grid_y=0,
            world_x_m=60.0,
            world_y_m=48.33,
            yaw_rad=math.pi,
            frontier_cell_count=0,
            information_gain_m=0.0,
            distance_m=10.0,
            score=10.0,
        )
        reserved_route_goal = FrontierGoal(
            grid_x=0,
            grid_y=0,
            world_x_m=60.0,
            world_y_m=48.95,
            yaw_rad=math.pi,
            frontier_cell_count=0,
            information_gain_m=0.0,
            distance_m=10.0,
            score=10.0,
        )
        assert explorer._goal_clearance_sources(edge_route_goal) == (False, True)
        assert explorer._goal_clearance_sources(reserved_route_goal) == (True, True)
        assert not explorer._path_poses_are_costmap_clear([
            (70.2, 47.6, math.pi),
            (-92.5, 39.0, math.pi),
        ])
        assert explorer._start_frontier_detour_plan(
            source_goal, (70.2, 47.6, math.pi)
        ) == "started"
        assert harness.navigation_completed.wait(timeout=5.0)
        deadline = time.monotonic() + 5.0
        while (
            not explorer.goal_history[-1].get("succeeded")
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        row = explorer.goal_history[-1]
        assert row["planner_accepted"] is True
        assert row["planned_path_endpoints_match"] is True
        assert row["path_costmap_clearance_checked"] is True
        assert row["detour_lookahead_distance_m"] == pytest.approx(30.0)
        assert row["succeeded"] is True
        assert row["phase"] == "completed"
        assert harness.navigation_goal is not None
        assert harness.navigation_goal[0] < 60.0
        assert harness.navigation_goal[1] < 38.5
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        explorer.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()


def test_global_route_advances_only_through_continuous_safe_prefix(
    tmp_path: Path,
):
    rclpy.init()
    explorer = FrontierExplorer()
    harness = _PlannerHarness()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(explorer)
    executor.add_node(harness)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        assert explorer.compute_path_client.wait_for_server(timeout_sec=3.0)
        assert explorer.action_client.wait_for_server(timeout_sec=3.0)
        explorer.terminal = True
        explorer.output_path = tmp_path / "frontier_detour_safe_prefix.json"
        obstacle_map, geometry = _obstacle_costmap()
        explorer.latest_data = obstacle_map
        explorer.latest_geometry = geometry
        explorer.latest_costmap_data = _unknown_route_band_costmap(geometry)
        explorer.latest_costmap_geometry = geometry
        explorer.latest_metrics = {"known_area_m2": 9014.25}
        explorer.map_update_count = 1017
        source_goal = FrontierGoal(
            grid_x=17,
            grid_y=23,
            world_x_m=67.2,
            world_y_m=47.4,
            yaw_rad=math.pi,
            frontier_cell_count=100,
            information_gain_m=10.0,
            distance_m=3.0,
            score=9.25,
            preference_distance_m=147.0,
            raw_world_x_m=-94.0,
            raw_world_y_m=39.0,
        )
        assert explorer._start_frontier_detour_plan(
            source_goal, (70.2, 47.6, math.pi)
        ) == "started"
        assert harness.navigation_completed.wait(timeout=5.0)
        deadline = time.monotonic() + 5.0
        while (
            not explorer.goal_history[-1].get("succeeded")
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        row = explorer.goal_history[-1]
        assert row["planner_accepted"] is True
        assert row["planned_path_endpoints_match"] is True
        assert row["path_costmap_clearance_checked"] is True
        assert row["planned_path_fully_costmap_clear"] is False
        assert row["costmap_clear_prefix_pose_count"] > 1
        assert 0.8 <= row["costmap_clear_prefix_length_m"] < row[
            "planned_path_length_m"
        ]
        assert row["succeeded"] is True
        assert harness.navigation_goal is not None
        assert harness.navigation_goal[0] > 60.0
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        explorer.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()


def test_global_timeout_preempts_navigation_recovery(tmp_path: Path):
    rclpy.init()
    explorer = FrontierExplorer()
    try:
        explorer.output_path = tmp_path / "global_timeout.json"
        explorer.set_parameters([
            Parameter("timeout_sec", value=0.01),
        ])
        explorer.started_monotonic = time.monotonic() - 1.0
        explorer.nav_recovery_in_progress = True
        explorer.nav_recovery_status = "reset_requested"
        explorer._tick()
        assert explorer.terminal is True
        assert explorer.success is False
        assert explorer.terminal_reason == "exploration_timeout"
    finally:
        explorer.destroy_node()
        rclpy.shutdown()


def test_recent_progress_defers_idle_timeout_but_not_indefinitely(
    tmp_path: Path,
):
    class _CancelHandle:
        def __init__(self):
            self.cancel_count = 0

        def cancel_goal_async(self):
            self.cancel_count += 1

    rclpy.init()
    explorer = FrontierExplorer()
    try:
        explorer.output_path = tmp_path / "progress_watchdog.json"
        explorer.active_goal = FrontierGoal(
            1, 2, 1.0, 0.0, 0.0, 10, 1.0, 1.0, 0.75
        )
        handle = _CancelHandle()
        explorer.active_goal_handle = handle
        explorer.goal_history = [{"goal_kind": "frontier", "sequence": 1}]
        explorer._start_active_goal_watchdog(0.05)
        now = time.monotonic()
        explorer.active_goal_started_monotonic = now - 0.10
        explorer.active_goal_last_progress_monotonic = now
        explorer.active_goal_progress_reference_pose = (0.0, 0.0, 0.0)
        explorer._robot_pose = lambda: (0.0, 0.0, 0.0)

        explorer._tick()
        assert handle.cancel_count == 0
        assert explorer.active_goal_cancel_requested is False

        explorer.active_goal_last_progress_monotonic = time.monotonic() - 0.06
        explorer._tick()
        assert handle.cancel_count == 1
        assert explorer.active_goal_cancel_requested is True
        assert "frontier_goal_no_progress_timeout" in explorer.last_error
    finally:
        explorer.destroy_node()
        rclpy.shutdown()


def test_failed_short_frontier_queues_one_route_aware_fallback(
    tmp_path: Path,
):
    rclpy.init()
    explorer = FrontierExplorer()
    harness = _PlannerHarness(abort_first_navigation=True)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(explorer)
    executor.add_node(harness)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        assert explorer.compute_path_client.wait_for_server(timeout_sec=3.0)
        assert explorer.action_client.wait_for_server(timeout_sec=3.0)
        explorer.terminal = True
        explorer.output_path = tmp_path / "frontier_detour_fallback.json"
        obstacle_map, geometry = _obstacle_costmap()
        explorer.latest_data = obstacle_map
        explorer.latest_geometry = geometry
        explorer.latest_costmap_data = tuple(0 for _ in obstacle_map)
        explorer.latest_costmap_geometry = geometry
        explorer.latest_metrics = {"known_area_m2": 9705.93}
        explorer.map_update_count = 22190
        source_goal = FrontierGoal(
            grid_x=17,
            grid_y=23,
            world_x_m=67.2,
            world_y_m=47.4,
            yaw_rad=math.pi,
            frontier_cell_count=100,
            information_gain_m=10.0,
            distance_m=3.0,
            score=9.25,
            preference_distance_m=12.0,
            raw_world_x_m=-94.0,
            raw_world_y_m=39.0,
        )
        explorer._send_goal(source_goal)
        assert harness.first_navigation_completed.wait(timeout=5.0)
        deadline = time.monotonic() + 5.0
        while (
            explorer.pending_frontier_detour_source_goal is None
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        assert explorer.pending_frontier_detour_source_goal == source_goal
        while explorer.active_goal is not None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert explorer.active_goal is None
        assert explorer.goal_history[0]["succeeded"] is False
        assert explorer.goal_history[0]["detour_fallback_queued"] is True
        assert explorer.frontier_detour_fallback_queued_count == 1
        assert explorer._start_frontier_detour_plan(
            explorer.pending_frontier_detour_source_goal,
            (70.2, 47.6, math.pi),
        ) == "started"
        assert harness.navigation_completed.wait(timeout=5.0)
        deadline = time.monotonic() + 5.0
        while (
            not explorer.goal_history[-1].get("succeeded")
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        assert len(explorer.goal_history) == 2
        assert explorer.goal_history[-1]["goal_kind"] == "frontier_detour"
        assert explorer.goal_history[-1]["succeeded"] is True
        assert harness.navigation_count == 2
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        explorer.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()
