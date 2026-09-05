"""Execute saved-map coverage through real Coverage and Nav2 actions."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Path as NavPath
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .saved_map_coverage_core import (
    FORMAL_MAX_LINEAR_SPEED_MPS,
    FORMAL_OPERATION_WIDTH_M,
    load_product_mission_geometry,
    polygon_area,
    validate_execution_parameters,
)


class FormalSavedMapCoverageExecutor(Node):
    """Product executor: public geometry and action results only, never truth."""

    def __init__(self) -> None:
        super().__init__("formal_saved_map_coverage_executor")
        self.declare_parameter("mission_geometry_path", "")
        self.declare_parameter("output_path", "coverage_execution.json")
        self.declare_parameter("operation_width_m", FORMAL_OPERATION_WIDTH_M)
        self.declare_parameter("maximum_linear_speed_mps", FORMAL_MAX_LINEAR_SPEED_MPS)
        self.declare_parameter("planning_timeout_sec", 300.0)
        self.declare_parameter("component_timeout_margin_sec", 120.0)
        self._state = self.create_publisher(
            String, "/formal_saved_map_coverage/state", 10
        )
        self._brush = self.create_publisher(Bool, "/brush_enabled", 10)
        self._coverage = ActionClient(
            self, ComputeCoveragePath, "/compute_coverage_path"
        )
        self._navigate = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._follow = ActionClient(self, FollowPath, "/follow_path")
        self._brush_state = False

    def _publish_state(self, state: str, **details) -> None:  # type: ignore[no-untyped-def]
        message = String()
        message.data = json.dumps({
            "schema_version": 1,
            "state": state,
            "ground_truth_used_for_control": False,
            **details,
        }, sort_keys=True)
        self._state.publish(message)

    def _set_brush(self, enabled: bool) -> None:
        self._brush_state = bool(enabled)
        self._brush.publish(Bool(data=self._brush_state))

    @staticmethod
    def _pose(x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    @classmethod
    def _path(cls, start: tuple[float, float], end: tuple[float, float]) -> NavPath:
        path = NavPath()
        path.header.frame_id = "map"
        length = math.dist(start, end)
        count = max(2, math.ceil(length / 0.20) + 1)
        yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        path.poses = [
            cls._pose(
                start[0] + (end[0] - start[0]) * index / (count - 1),
                start[1] + (end[1] - start[1]) * index / (count - 1),
                yaw,
            )
            for index in range(count)
        ]
        return path

    def _run_action(self, client, goal, timeout: float, label: str) -> dict:  # type: ignore[no-untyped-def]
        if not client.wait_for_server(timeout_sec=60.0):
            return {"success": False, "error": f"{label}_server_unavailable"}
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=30.0)
        handle = sent.result() if sent.done() else None
        if handle is None or not handle.accepted:
            return {"success": False, "error": f"{label}_goal_rejected"}
        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not result_future.done():
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
            return {"success": False, "error": f"{label}_result_timeout"}
        wrapped = result_future.result()
        return {
            "success": int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED,
            "terminal_status": int(wrapped.status),
        }

    def _plan(self, polygon: tuple[tuple[float, float], ...]) -> dict:
        goal = ComputeCoveragePath.Goal()
        goal.generate_headland = True
        goal.generate_route = True
        goal.generate_path = True
        goal.frame_id = "map"
        goal.swath_mode.objective = "LENGTH"
        goal.swath_mode.mode = "SET_ANGLE"
        goal.swath_mode.best_angle = 0.0
        goal.route_mode.mode = "BOUSTROPHEDON"
        goal.path_mode.mode = "DUBIN"
        goal.path_mode.continuity_mode = "DISCONTINUOUS"
        closed = (*polygon, polygon[0])
        coordinates = Coordinates()
        coordinates.coordinates = [
            Coordinate(axis1=x, axis2=y) for x, y in closed
        ]
        goal.polygons = [coordinates]
        timeout = float(self.get_parameter("planning_timeout_sec").value)
        if not math.isfinite(timeout) or timeout <= 0.0:
            return {"success": False, "error": "invalid_planning_timeout"}
        if not self._coverage.wait_for_server(timeout_sec=60.0):
            return {"success": False, "error": "coverage_action_unavailable"}
        sent = self._coverage.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=30.0)
        handle = sent.result() if sent.done() else None
        if handle is None or not handle.accepted:
            return {"success": False, "error": "coverage_goal_rejected"}
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if not result_future.done():
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
            return {"success": False, "error": "coverage_result_timeout"}
        wrapped = result_future.result()
        result = wrapped.result
        if (
            int(wrapped.status) != GoalStatus.STATUS_SUCCEEDED
            or int(result.error_code) != 0
        ):
            return {
                "success": False,
                "error": "coverage_planning_failed",
                "terminal_status": int(wrapped.status),
                "error_code": int(result.error_code),
            }
        swaths = [
            (
                (float(item.start.x), float(item.start.y)),
                (float(item.end.x), float(item.end.y)),
            )
            for item in result.coverage_path.swaths
            if math.dist(
                (float(item.start.x), float(item.start.y)),
                (float(item.end.x), float(item.end.y)),
            ) > 0.01
        ]
        return {"success": bool(swaths), "swaths": swaths}

    def execute(self) -> dict:
        width = float(self.get_parameter("operation_width_m").value)
        speed = float(self.get_parameter("maximum_linear_speed_mps").value)
        validate_execution_parameters(width, speed)
        polygon = load_product_mission_geometry(
            str(self.get_parameter("mission_geometry_path").value)
        )
        self._set_brush(False)
        self._publish_state("PLANNING", operation_width_m=width)
        planning = self._plan(polygon)
        if not planning.get("success"):
            return self._finish(False, "FAILED", planning)
        swaths = planning["swaths"]
        results = []
        margin = float(self.get_parameter("component_timeout_margin_sec").value)
        if not math.isfinite(margin) or margin <= 0.0:
            return self._finish(
                False, "FAILED", {"error": "invalid_component_timeout_margin"}
            )
        maximum_transit_timeout = (
            math.hypot(
                max(point[0] for point in polygon) - min(point[0] for point in polygon),
                max(point[1] for point in polygon) - min(point[1] for point in polygon),
            )
            / speed
            + margin
        )
        for index, (start, end) in enumerate(swaths):
            yaw = math.atan2(end[1] - start[1], end[0] - start[0])
            self._publish_state(
                "TRANSIT", swath_index=index, planned_swath_count=len(swaths)
            )
            transit = NavigateToPose.Goal()
            transit.pose = self._pose(start[0], start[1], yaw)
            transit_result = self._run_action(
                self._navigate,
                transit,
                max(300.0, maximum_transit_timeout),
                "navigate_to_swath",
            )
            if not transit_result["success"]:
                results.append({"index": index, "transit": transit_result})
                return self._finish(False, "FAILED", {
                    "planned_swath_count": len(swaths),
                    "completed_swath_count": index,
                    "component_results": results,
                })
            path = self._path(start, end)
            follow = FollowPath.Goal()
            follow.path = path
            follow.controller_id = "CleanPath"
            follow.goal_checker_id = "goal_checker"
            follow.progress_checker_id = "progress_checker"
            self._set_brush(True)
            self._publish_state(
                "CLEANING", swath_index=index, planned_swath_count=len(swaths)
            )
            follow_result = self._run_action(
                self._follow,
                follow,
                max(300.0, math.dist(start, end) / speed + margin),
                "follow_swath",
            )
            self._set_brush(False)
            results.append({
                "index": index,
                "planned_length_m": math.dist(start, end),
                "transit": transit_result,
                "follow": follow_result,
            })
            if not follow_result["success"]:
                return self._finish(False, "FAILED", {
                    "planned_swath_count": len(swaths),
                    "completed_swath_count": index,
                    "component_results": results,
                })
        length = sum(math.dist(start, end) for start, end in swaths)
        return self._finish(True, "COMPLETED", {
            "planned_swath_count": len(swaths),
            "completed_swath_count": len(swaths),
            "planned_swath_length_m": length,
            "planned_coverage_fraction": min(
                1.0, length * width / polygon_area(polygon)
            ),
            "component_results": results,
        })

    def _finish(self, success: bool, state: str, details: dict) -> dict:
        self._set_brush(False)
        report = {
            "schema_version": 1,
            "success": success,
            "terminal_state": state,
            "ground_truth_used_for_control": False,
            "truth_topics_subscribed": [],
            "operation_width_m": FORMAL_OPERATION_WIDTH_M,
            "maximum_linear_speed_mps": FORMAL_MAX_LINEAR_SPEED_MPS,
            "brush_disabled_on_exit": True,
            **details,
        }
        output = Path(str(self.get_parameter("output_path").value))
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_suffix(output.suffix + ".tmp")
        temporary.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        temporary.replace(output)
        self._publish_state(
            state,
            success=success,
            terminal_state=state,
            operation_width_m=FORMAL_OPERATION_WIDTH_M,
            maximum_linear_speed_mps=FORMAL_MAX_LINEAR_SPEED_MPS,
            planned_swath_count=int(report.get("planned_swath_count", 0)),
            completed_swath_count=int(report.get("completed_swath_count", 0)),
            brush_disabled_on_exit=True,
            error=report.get("error"),
        )
        return report


def main(args=None) -> None:  # type: ignore[no-untyped-def]
    rclpy.init(args=args)
    node = FormalSavedMapCoverageExecutor()
    report: dict = {"success": False, "terminal_state": "FAILED"}
    try:
        try:
            report = node.execute()
        except Exception as exc:  # Preserve a durable fail-closed terminal artifact.
            node.get_logger().error(f"saved-map coverage failed: {exc}")
            report = node._finish(
                False,
                "FAILED",
                {"error": "executor_exception", "detail": str(exc)},
            )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if report.get("success") else 2)


if __name__ == "__main__":
    main()
