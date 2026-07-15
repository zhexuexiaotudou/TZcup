import csv
import json
import math
import time
from pathlib import Path

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import yaml

from sanitation_coverage.metrics import (
    empirical_swept_metrics,
    path_length,
    raster_coverage_metrics,
    repair_degenerate_swaths,
)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CoverageProbe(Node):
    """Plan and execute every coverage component, then score actual GT sweep."""

    def __init__(self):
        super().__init__("sanitation_coverage_probe")
        default_config = Path(get_package_share_directory("sanitation_tasks")) / "config" / "demo_area.yaml"
        self.declare_parameter("config_path", str(default_config))
        self.declare_parameter("output_path", "coverage_metrics.json")
        self.declare_parameter("path_output_path", "coverage_path.json")
        self.declare_parameter("trajectory_output_path", "coverage_trajectory.csv")
        self.declare_parameter("component_retry_limit", 1)
        self.declare_parameter("minimum_component_timeout_sec", 45.0)
        self.coverage_client = ActionClient(self, ComputeCoveragePath, "/compute_coverage_path")
        self.follow_client = ActionClient(self, FollowPath, "/follow_path")
        self.navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.controller_state_client = self.create_client(GetState, "/controller_server/get_state")
        self.brush_publisher = self.create_publisher(Bool, "/brush_enabled", 10)
        self.brush_enabled = False
        self.truth_samples = []
        self.brush_samples = []
        self.minimum_scan_range = math.inf
        self.collision_events = 0
        self._collision_active = False
        self.create_subscription(Odometry, "/ground_truth/odom", self._on_truth, 20)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)

    def _on_truth(self, message):
        pose = message.pose.pose
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        yaw = yaw_from_quaternion(pose.orientation)
        sample = (stamp, pose.position.x, pose.position.y, yaw, self.brush_enabled)
        self.truth_samples.append(sample)
        if self.brush_enabled:
            # Cleaning footprint is 0.55 m forward of base_footprint in URDF.
            self.brush_samples.append((stamp, pose.position.x + 0.55 * math.cos(yaw), pose.position.y + 0.55 * math.sin(yaw)))

    def _on_scan(self, message):
        finite = [value for value in message.ranges if math.isfinite(value)]
        if not finite:
            return
        minimum = min(finite)
        self.minimum_scan_range = min(self.minimum_scan_range, minimum)
        collision_now = minimum < 0.12
        if collision_now and not self._collision_active:
            self.collision_events += 1
        self._collision_active = collision_now

    def _set_brush(self, enabled):
        self.brush_enabled = bool(enabled)
        self.brush_publisher.publish(Bool(data=self.brush_enabled))

    def run(self):
        config = yaml.safe_load(Path(self.get_parameter("config_path").value).read_text(encoding="utf-8"))
        polygon = [[float(x), float(y)] for x, y in config["polygon"]]
        width = float(config["operation_width_m"])
        planning = self._plan(config, polygon)
        if "error" in planning:
            return self._write_report({"success": False, **planning})
        raw_swaths, turns, nav_points, result = planning["raw_swaths"], planning["turns"], planning["nav_points"], planning["result"]
        swaths, repaired = repair_degenerate_swaths(raw_swaths, turns, nav_points)
        planned_metrics = raster_coverage_metrics(polygon, swaths, width, resolution=0.10)
        planned_metrics.update({
            "metric_basis": "planned_fields2cover_swaths_rasterized",
            "path_length_m": path_length(nav_points),
            "swath_endpoint_compatibility_repair": repaired,
        })
        components = []
        for index, swath in enumerate(swaths):
            components.append({"kind": "swath", "index": index, "brush": True, "points": self._interpolate(*swath)})
            if index < len(turns):
                components.append({"kind": "turn", "index": index, "brush": False, "points": turns[index]})

        path_report = {
            "frame_id": result.nav_path.header.frame_id,
            "operation_width_m": width,
            "route_type": str(config["route_type"]), "path_type": str(config["path_type"]),
            "nav_path": nav_points, "swaths": swaths, "raw_swaths": raw_swaths,
            "turns": turns,
            "component_count": len(components),
            "execution_strategy": "NavigateToPose to first swath, then one FollowPath action per swath/turn; no path truncation",
        }
        self._write_json(self.get_parameter("path_output_path").value, path_report)

        execution_started_wall = time.monotonic()
        transit = self._navigate_to(components[0]["points"][0]) if components else {"success": False, "error": "no_components"}
        component_results = []
        if transit["success"]:
            for component in components:
                component_results.append(self._follow_component(component))
                if not component_results[-1]["success"]:
                    break
        self._set_brush(False)
        execution_duration = time.monotonic() - execution_started_wall
        complete = bool(components and transit["success"] and len(component_results) == len(components) and all(item["success"] for item in component_results))
        empirical = empirical_swept_metrics(polygon, self.brush_samples, width, resolution=0.10)
        actual_path_points = [(sample[1], sample[2]) for sample in self.truth_samples]
        actual_path_length = path_length(actual_path_points)
        empirical.update({
            "actual_path_length_m": actual_path_length,
            "actual_duration_sec": execution_duration,
            "gross_efficiency_m2_h": empirical["covered_area_m2"] / execution_duration * 3600.0 if execution_duration > 0 else 0.0,
            "net_efficiency_m2_h": empirical["covered_area_m2"] / execution_duration * 3600.0 if execution_duration > 0 else 0.0,
        })
        empirical_pass = complete and empirical["coverage_rate"] >= 0.90
        safety_pass = self.collision_events == 0
        recovery_count = sum(item["retries"] for item in component_results) + int(transit.get("retries", 0))
        efficiency_pass = empirical["net_efficiency_m2_h"] >= 3500.0
        self._write_trajectory()
        report = {
            "schema_version": 2,
            "mission_id": str(config["mission_id"]),
            "planner": "OpenNav Coverage + Fields2Cover",
            "planning_success": True,
            "transit_to_start_success": transit["success"],
            "full_execution_success": complete,
            "empirical_coverage_success": empirical_pass,
            "safety_success": safety_pass,
            "competition_efficiency_pass": efficiency_pass,
            "success": bool(complete and empirical_pass and safety_pass and not self.brush_enabled),
            "operation_width_m": width,
            "swath_count": len(swaths), "turn_count": len(turns),
            "nav_path_pose_count": len(nav_points),
            "component_count": len(components), "component_results": component_results,
            "transit_to_start": transit,
            "planned_metrics": planned_metrics,
            "empirical_metrics": empirical,
            "recovery_count": recovery_count,
            "collision_count": self.collision_events,
            "minimum_lidar_range_m": self.minimum_scan_range if math.isfinite(self.minimum_scan_range) else None,
            "brush_disabled_on_exit": not self.brush_enabled,
            "execution_boundary": "All path components were required to terminate SUCCEEDED; no 180-pose shortcut or bounded cancellation is accepted.",
        }
        return self._write_report(report)

    def _plan(self, config, polygon):
        if not self.coverage_client.wait_for_server(timeout_sec=60.0):
            return {"error": "coverage_action_timeout"}
        goal = ComputeCoveragePath.Goal()
        goal.generate_headland = False; goal.generate_route = True; goal.generate_path = True
        goal.frame_id = str(config["frame_id"])
        closed = polygon if polygon[0] == polygon[-1] else polygon + [polygon[0]]
        coordinates = Coordinates(); coordinates.coordinates = [Coordinate(axis1=x, axis2=y) for x, y in closed]
        goal.polygons = [coordinates]
        future = self.coverage_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            return {"error": "coverage_goal_rejected"}
        result_future = handle.get_result_async(); rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        if not result_future.done():
            return {"error": "coverage_result_timeout"}
        wrapped = result_future.result(); result = wrapped.result
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or result.error_code != 0:
            return {"error": "coverage_planning_failed", "status": int(wrapped.status), "error_code": int(result.error_code)}
        raw_swaths = [((float(item.start.x), float(item.start.y)), (float(item.end.x), float(item.end.y))) for item in result.coverage_path.swaths]
        turns = [[(float(pose.pose.position.x), float(pose.pose.position.y)) for pose in turn.poses] for turn in result.coverage_path.turns]
        nav_points = [(float(pose.pose.position.x), float(pose.pose.position.y)) for pose in result.nav_path.poses]
        return {"raw_swaths": raw_swaths, "turns": turns, "nav_points": nav_points, "result": result}

    @staticmethod
    def _interpolate(start, end, spacing=0.10):
        length = math.dist(start, end)
        count = max(2, int(math.ceil(length / spacing)) + 1)
        return [(start[0] + (end[0] - start[0]) * index / (count - 1), start[1] + (end[1] - start[1]) * index / (count - 1)) for index in range(count)]

    def _path_message(self, points):
        path = NavPath(); path.header.frame_id = "map"
        if len(points) == 1: points = [points[0], points[0]]
        for index, (x, y) in enumerate(points):
            next_point = points[min(index + 1, len(points) - 1)]
            previous = points[max(0, index - 1)]
            yaw = math.atan2(next_point[1] - previous[1], next_point[0] - previous[0])
            pose = PoseStamped(); pose.header.frame_id = "map"
            pose.pose.position.x = float(x); pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(yaw / 2.0); pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        return path

    def _navigate_to(self, point):
        self._set_brush(False)
        if not self._wait_controller_active() or not self.navigate_client.wait_for_server(timeout_sec=30.0):
            return {"success": False, "error": "navigate_server_unavailable", "retries": 0}
        goal = NavigateToPose.Goal(); goal.pose = self._path_message([point]).poses[0]
        return self._run_action(self.navigate_client, goal, False, 180.0)

    def _follow_component(self, component):
        goal = FollowPath.Goal(); goal.path = self._path_message(component["points"])
        goal.controller_id = "FollowPath"; goal.goal_checker_id = "goal_checker"; goal.progress_checker_id = "progress_checker"
        timeout = max(self.get_parameter("minimum_component_timeout_sec").value, path_length(component["points"]) / 0.10 + 30.0)
        result = self._run_action(self.follow_client, goal, component["brush"], timeout)
        result.update({"kind": component["kind"], "index": component["index"], "brush_enabled": component["brush"], "path_pose_count": len(component["points"]), "planned_length_m": path_length(component["points"])})
        return result

    def _run_action(self, client, goal, brush, timeout):
        retry_limit = int(self.get_parameter("component_retry_limit").value)
        attempts = []
        for attempt in range(retry_limit + 1):
            self._set_brush(brush)
            send = client.send_goal_async(goal); rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
            handle = send.result() if send.done() else None
            if handle is None or not handle.accepted:
                attempts.append({"attempt": attempt + 1, "accepted": False})
                continue
            result_future = handle.get_result_async(); deadline = time.monotonic() + timeout
            while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
                self._set_brush(brush); rclpy.spin_once(self, timeout_sec=0.05); time.sleep(0.02)
            if not result_future.done():
                cancel = handle.cancel_goal_async(); rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
                attempts.append({"attempt": attempt + 1, "accepted": True, "timeout": True})
                continue
            wrapped = result_future.result(); error_code = int(getattr(wrapped.result, "error_code", 0))
            succeeded = wrapped.status == GoalStatus.STATUS_SUCCEEDED and error_code == 0
            attempts.append({"attempt": attempt + 1, "accepted": True, "terminal_status": int(wrapped.status), "error_code": error_code, "succeeded": succeeded})
            if succeeded:
                self._set_brush(False if brush else brush)
                return {"success": True, "retries": attempt, "attempts": attempts}
        self._set_brush(False)
        return {"success": False, "retries": max(0, len(attempts) - 1), "attempts": attempts}

    def _wait_controller_active(self):
        if not self.controller_state_client.wait_for_service(timeout_sec=60.0): return False
        deadline = time.monotonic() + 60.0
        while rclpy.ok() and time.monotonic() < deadline:
            future = self.controller_state_client.call_async(GetState.Request()); rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result().current_state.label == "active": return True
        return False

    def _write_trajectory(self):
        output = Path(self.get_parameter("trajectory_output_path").value); output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream); writer.writerow(["stamp_sec", "base_x_m", "base_y_m", "yaw_rad", "brush_enabled"]); writer.writerows(self.truth_samples)

    @staticmethod
    def _write_json(path, data):
        output = Path(path); output.parent.mkdir(parents=True, exist_ok=True); output.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def _write_report(self, report):
        self._write_json(self.get_parameter("output_path").value, report)
        self.get_logger().info(json.dumps(report, ensure_ascii=False))
        return 0 if report.get("success") else 2


def main(args=None):
    rclpy.init(args=args); node = CoverageProbe()
    try: code = node.run()
    finally:
        node._set_brush(False); node.destroy_node(); rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__": main()
