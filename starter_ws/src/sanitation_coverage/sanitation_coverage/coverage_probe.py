import json
import math
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path as NavPath
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
import yaml

from sanitation_coverage.metrics import (
    path_length,
    raster_coverage_metrics,
    repair_degenerate_swaths,
)


class CoverageProbe(Node):
    def __init__(self) -> None:
        super().__init__("sanitation_coverage_probe")
        default_config = Path(
            get_package_share_directory("sanitation_tasks")
        ) / "config" / "demo_area.yaml"
        self.declare_parameter("config_path", str(default_config))
        self.declare_parameter("output_path", "coverage_metrics.json")
        self.declare_parameter("path_output_path", "coverage_path.json")
        self.declare_parameter("handoff_duration_sec", 20.0)
        self.coverage_client = ActionClient(
            self, ComputeCoveragePath, "/compute_coverage_path"
        )
        self.follow_client = ActionClient(self, FollowPath, "/follow_path")
        self.controller_state_client = self.create_client(
            GetState, "/controller_server/get_state"
        )
        self.brush_publisher = self.create_publisher(Bool, "/brush_enabled", 10)
        self.last_odom = None
        self.last_amcl_pose = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10
        )

    def _on_odom(self, message) -> None:
        self.last_odom = message

    def _on_amcl_pose(self, message) -> None:
        self.last_amcl_pose = message

    def run(self) -> int:
        config_path = Path(str(self.get_parameter("config_path").value))
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        polygon = [[float(x), float(y)] for x, y in config["polygon"]]
        operation_width = float(config["operation_width_m"])

        if not self.coverage_client.wait_for_server(timeout_sec=60.0):
            return self._write_error("coverage_action_timeout")

        goal = ComputeCoveragePath.Goal()
        goal.generate_headland = False
        goal.generate_route = True
        goal.generate_path = True
        goal.frame_id = str(config["frame_id"])
        coordinates = Coordinates()
        closed_polygon = polygon if polygon[0] == polygon[-1] else polygon + [polygon[0]]
        coordinates.coordinates = [
            Coordinate(axis1=float(x), axis2=float(y)) for x, y in closed_polygon
        ]
        goal.polygons = [coordinates]

        send_future = self.coverage_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=30.0)
        goal_handle = send_future.result() if send_future.done() else None
        if goal_handle is None or not goal_handle.accepted:
            return self._write_error("coverage_goal_rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        if not result_future.done():
            return self._write_error("coverage_result_timeout")
        wrapped = result_future.result()
        result = wrapped.result
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or result.error_code != 0:
            return self._write_error(
                "coverage_planning_failed",
                {"status": int(wrapped.status), "error_code": int(result.error_code)},
            )

        raw_swaths = [
            (
                (float(swath.start.x), float(swath.start.y)),
                (float(swath.end.x), float(swath.end.y)),
            )
            for swath in result.coverage_path.swaths
        ]
        turns = [
            [
                (float(pose.pose.position.x), float(pose.pose.position.y))
                for pose in turn.poses
            ]
            for turn in result.coverage_path.turns
        ]
        nav_points = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in result.nav_path.poses
        ]
        swaths, swath_endpoints_repaired = repair_degenerate_swaths(
            raw_swaths, turns, nav_points
        )
        metrics = raster_coverage_metrics(
            polygon, swaths, operation_width, resolution=0.10
        )
        total_path_length = path_length(nav_points)
        task_time = float(result.task_time)
        estimated_time = task_time if task_time > 0.0 else total_path_length / 0.35
        metrics.update(
            {
                "metric_basis": "planned_fields2cover_swaths_rasterized",
                "swath_endpoint_compatibility_repair": swath_endpoints_repaired,
                "path_length_m": total_path_length,
                "estimated_total_time_sec": estimated_time,
                "effective_cleaning_efficiency_m2_s": (
                    metrics["covered_area_m2"] / estimated_time
                    if estimated_time > 0.0
                    else 0.0
                ),
                "recovery_count": 0,
            }
        )

        schedule = []
        for index, swath in enumerate(swaths):
            schedule.append(
                {
                    "component": "swath",
                    "index": index,
                    "brush_enabled": True,
                    "length_m": math.dist(*swath),
                }
            )
            if index < len(turns):
                schedule.append(
                    {
                        "component": "turn",
                        "index": index,
                        "brush_enabled": False,
                        "length_m": path_length(turns[index]),
                    }
                )

        handoff = self._handoff(result.nav_path)
        path_report = {
            "frame_id": result.nav_path.header.frame_id,
            "operation_width_m": operation_width,
            "route_type": str(config["route_type"]),
            "path_type": str(config["path_type"]),
            "nav_path": nav_points,
            "swaths": swaths,
            "raw_swaths": raw_swaths,
            "swath_endpoint_compatibility_repair": swath_endpoints_repaired,
            "turns": turns,
            "brush_schedule": schedule,
        }
        path_output = Path(str(self.get_parameter("path_output_path").value))
        path_output.parent.mkdir(parents=True, exist_ok=True)
        path_output.write_text(
            json.dumps(path_report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

        report = {
            "success": bool(
                swaths
                and nav_points
                and handoff["accepted"]
                and handoff["execution_started"]
            ),
            "mission_id": str(config["mission_id"]),
            "planner": "OpenNav Coverage + Fields2Cover",
            "route_type": str(config["route_type"]),
            "path_type": str(config["path_type"]),
            "operation_width_m": operation_width,
            "swath_count": len(swaths),
            "turn_count": len(turns),
            "nav_path_pose_count": len(nav_points),
            "planning_time_sec": (
                float(result.planning_time.sec)
                + float(result.planning_time.nanosec) / 1.0e9
            ),
            "metrics": metrics,
            "brush_schedule": {
                "swath_components_on": len(swaths),
                "turn_components_off": len(turns),
            },
            "nav2_handoff": handoff,
            "execution_boundary": (
                "The full coverage path was generated, while only a bounded path prefix was "
                "handed to Nav2 for physical integration evidence; coverage metrics are "
                "planned, not empirical, because Stage 3 localization endpoint error is "
                "material."
            ),
        }
        return self._write_report(report)

    def _handoff(self, nav_path):
        if not self._wait_controller_active():
            return {"accepted": False, "error": "controller_inactive"}
        if not self._wait_sim_time(1.5):
            return {"accepted": False, "error": "simulation_time_not_settled"}
        if not self.follow_client.wait_for_server(timeout_sec=30.0):
            return {"accepted": False, "error": "follow_path_timeout"}

        goal = FollowPath.Goal()
        # RPP prunes a looping coverage plan to the globally nearest repeated
        # segment. Sending a bounded prefix avoids pruning the entire plan and
        # provides a deterministic physical integration check while Stage 3
        # localization remains outside the acceptance tolerance.
        reference = self.last_amcl_pose.pose.pose.position
        closest_index = min(
            range(len(nav_path.poses)),
            key=lambda index: math.hypot(
                nav_path.poses[index].pose.position.x - reference.x,
                nav_path.poses[index].pose.position.y - reference.y,
            ),
        )
        execution_path = NavPath()
        execution_path.header = nav_path.header
        execution_path.poses = nav_path.poses[
            closest_index : min(closest_index + 180, len(nav_path.poses))
        ]

        # The coverage request can complete before the simulated TF buffer has
        # retained the path's original timestamp. A zero stamp requests the
        # latest transform and avoids an immediate past-extrapolation abort.
        execution_path.header.stamp.sec = 0
        execution_path.header.stamp.nanosec = 0
        for pose in execution_path.poses:
            pose.header.stamp.sec = 0
            pose.header.stamp.nanosec = 0
        goal.path = execution_path
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "goal_checker"
        goal.progress_checker_id = "progress_checker"
        send_future = self.follow_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result() if send_future.done() else None
        if goal_handle is None or not goal_handle.accepted:
            return {"accepted": False, "error": "follow_path_rejected"}

        start_xy = self._odom_xy()
        result_future = goal_handle.get_result_async()
        duration = float(self.get_parameter("handoff_duration_sec").value)
        deadline = time.monotonic() + duration
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            self.brush_publisher.publish(Bool(data=True))
            rclpy.spin_once(self, timeout_sec=0.1)
        bounded_cancel = not result_future.done()
        if bounded_cancel:
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
        self.brush_publisher.publish(Bool(data=False))
        end_xy = self._odom_xy()
        displacement = None
        if start_xy is not None and end_xy is not None:
            displacement = math.dist(start_xy, end_xy)
        terminal_status = None
        terminal_error_code = None
        if result_future.done():
            wrapped = result_future.result()
            terminal_status = int(wrapped.status)
            terminal_error_code = int(getattr(wrapped.result, "error_code", 0))
        execution_started = bool(
            bounded_cancel or (displacement is not None and displacement > 0.05)
        )
        return {
            "accepted": True,
            "execution_started": execution_started,
            "full_plan_pose_count": len(nav_path.poses),
            "handoff_path_pose_count": len(execution_path.poses),
            "handoff_start_pose_index": closest_index,
            "amcl_reference_xy": [float(reference.x), float(reference.y)],
            "bounded_duration_sec": duration,
            "bounded_cancel_requested": bounded_cancel,
            "completed_during_window": result_future.done(),
            "terminal_status": terminal_status,
            "terminal_error_code": terminal_error_code,
            "odom_displacement_m": displacement,
            "brush_enabled_during_window": True,
            "brush_disabled_on_exit": True,
        }

    def _wait_controller_active(self):
        if not self.controller_state_client.wait_for_service(timeout_sec=60.0):
            return False
        deadline = time.monotonic() + 60.0
        while rclpy.ok() and time.monotonic() < deadline:
            future = self.controller_state_client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result().current_state.label == "active":
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def _wait_sim_time(self, minimum_sec):
        """Wait for odom/TF history to advance beyond AMCL's initial samples."""
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_odom is None or self.last_amcl_pose is None:
                continue
            stamp = self.last_odom.header.stamp
            stamp_sec = float(stamp.sec) + float(stamp.nanosec) / 1.0e9
            if stamp_sec >= minimum_sec:
                return True
        return False

    def _odom_xy(self):
        if self.last_odom is None:
            return None
        position = self.last_odom.pose.pose.position
        return [float(position.x), float(position.y)]

    def _write_error(self, error, extra=None):
        report = {"success": False, "error": error}
        if extra:
            report.update(extra)
        return self._write_report(report)

    def _write_report(self, report):
        output = Path(str(self.get_parameter("output_path").value))
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        self.get_logger().info(json.dumps(report, ensure_ascii=False))
        return 0 if report["success"] else 2


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoverageProbe()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
