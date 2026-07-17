import csv
import json
import math
import time
from pathlib import Path

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
import yaml

from sanitation_coverage.metrics import (
    empirical_swept_metrics,
    path_length,
    point_in_cleanable_area,
    point_in_polygon,
    raster_coverage_metrics,
    repair_degenerate_swaths,
    summarize_distances,
    synchronized_xy_errors,
)
from sanitation_coverage.mission_geometry import (
    compile_mission_geometry,
    exclusion_clearance,
    target_grid_viable,
)
from sanitation_coverage.route_entry import (
    entry_points,
    route_candidates,
    segment_heading,
    transit_pose,
)


NAVIGATE_TO_POSE_ERRORS = {
    0: "NONE",
    100: "UNKNOWN",
    101: "FAILED_TO_LOAD_BEHAVIOR_TREE",
    102: "TF_ERROR",
    103: "TIMEOUT",
    104: "CANCELED",
    105: "FAILED_TO_MAKE_PROGRESS",
}

FOLLOW_PATH_ERRORS = {
    0: "NONE",
    100: "UNKNOWN",
    101: "INVALID_CONTROLLER",
    102: "TF_ERROR",
    103: "INVALID_PATH",
    104: "PATIENCE_EXCEEDED",
    105: "FAILED_TO_MAKE_PROGRESS",
    106: "NO_VALID_CONTROL",
    107: "CONTROLLER_TIMED_OUT",
}

COMPUTE_PATH_ERRORS = {
    0: "NONE",
    200: "UNKNOWN",
    201: "INVALID_PLANNER",
    202: "TF_ERROR",
    203: "START_OUTSIDE_MAP",
    204: "GOAL_OUTSIDE_MAP",
    205: "START_OCCUPIED",
    206: "GOAL_OCCUPIED",
    207: "TIMEOUT",
    208: "NO_VALID_PATH",
}


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
        self.compute_path_client = ActionClient(
            self, ComputePathToPose, "/compute_path_to_pose"
        )
        self.controller_state_client = self.create_client(GetState, "/controller_server/get_state")
        self.brush_publisher = self.create_publisher(Bool, "/brush_enabled", 10)
        self.state_publisher = self.create_publisher(String, "/coverage/state", 10)
        self.component_state_publisher = self.create_publisher(
            String, "/coverage/component_state", 10
        )
        self.current_path_publisher = self.create_publisher(
            NavPath, "/coverage/current_path", 10
        )
        self.diagnostics_publisher = self.create_publisher(
            String, "/coverage/diagnostics", 10
        )
        self.brush_enabled = False
        self.state = "PLANNING"
        self.estimated_pose = None
        self.truth_pose = None
        self.truth_samples = []
        self.estimate_samples = []
        self.brush_samples = []
        self.minimum_scan_range = math.inf
        self.global_costmap = None
        self.keepout_mask = None
        self.speed_mask = None
        self.collision_events = 0
        self._collision_active = False
        self.create_subscription(Odometry, "/ground_truth/odom", self._on_truth, 20)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/fused_pose",
            self._on_estimate,
            20,
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        costmap_qos = QoSProfile(depth=1)
        costmap_qos.reliability = ReliabilityPolicy.RELIABLE
        costmap_qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap",
            lambda message: setattr(self, "global_costmap", message), costmap_qos,
        )
        self.create_subscription(
            OccupancyGrid, "/keepout_filter_mask",
            lambda message: setattr(self, "keepout_mask", message), latched_qos,
        )
        self.create_subscription(
            OccupancyGrid, "/speed_filter_mask",
            lambda message: setattr(self, "speed_mask", message), latched_qos,
        )

    def _on_truth(self, message):
        pose = message.pose.pose
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        yaw = yaw_from_quaternion(pose.orientation)
        self.truth_pose = (pose.position.x, pose.position.y, yaw)
        sample = (
            stamp, pose.position.x, pose.position.y, yaw,
            self.brush_enabled, self.state,
        )
        self.truth_samples.append(sample)
        if self.brush_enabled:
            # Cleaning footprint is 0.55 m forward of base_footprint in URDF.
            self.brush_samples.append((stamp, pose.position.x + 0.55 * math.cos(yaw), pose.position.y + 0.55 * math.sin(yaw)))

    def _on_estimate(self, message):
        pose = message.pose.pose
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.estimated_pose = (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
        )
        self.estimate_samples.append((stamp, *self.estimated_pose))

    def _set_state(self, state, details=None):
        self.state = state
        self.state_publisher.publish(String(data=state))
        payload = {
            "state": state,
            "brush_enabled": self.brush_enabled,
            "estimated_pose": self.estimated_pose,
            "ground_truth_pose_evaluation_only": self.truth_pose,
        }
        if details:
            payload.update(details)
        encoded = json.dumps(payload, ensure_ascii=False)
        self.component_state_publisher.publish(String(data=encoded))
        self.diagnostics_publisher.publish(String(data=encoded))

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
        geometry = compile_mission_geometry(config)
        polygon = geometry["outer_polygon"]
        exclusions = geometry["exclusion_polygons"]
        cleanable_polygon = geometry["cleanable_outer_polygon"]
        cleanable_exclusions = geometry["cleanable_exclusion_polygons"]
        width = float(config["operation_width_m"])
        self._set_state("PLANNING", {"geometry": geometry})
        if not geometry["headland_clearance_valid"]:
            return self._write_report({
                "success": False,
                "error": "configured_headland_below_compiled_clearance",
                "mission_geometry": geometry,
            })
        planning = self._plan(config, polygon, exclusions)
        if "error" in planning:
            self._set_state("FAILED", planning)
            return self._write_report({"success": False, **planning})
        raw_swaths, turns, nav_points, result = planning["raw_swaths"], planning["turns"], planning["nav_points"], planning["result"]
        swaths, repaired = repair_degenerate_swaths(raw_swaths, turns, nav_points)
        planned_metrics = raster_coverage_metrics(
            cleanable_polygon, swaths, width, resolution=0.10,
            exclusion_polygons=cleanable_exclusions,
        )
        planned_metrics.update({
            "metric_basis": "planned_fields2cover_swaths_rasterized",
            "path_length_m": path_length(nav_points),
            "swath_endpoint_compatibility_repair": repaired,
            "outer_area_m2": geometry["outer_area_m2"],
            "excluded_area_m2": geometry["excluded_area_m2"],
            "cleanable_area_m2": geometry["cleanable_area_m2"],
        })
        components = []
        for index, swath in enumerate(swaths):
            components.append({"kind": "swath", "index": index, "brush": True, "points": self._interpolate(*swath)})
            if index < len(turns):
                components.append({"kind": "turn", "index": index, "brush": False, "points": turns[index]})

        intersection_count = self._swath_exclusion_intersections(
            swaths, cleanable_polygon, cleanable_exclusions
        )
        self._set_state("TRANSIT_PREFLIGHT")
        self._wait_for_estimated_pose(15.0)
        selection = self._select_route(
            components, geometry, float(config["staging_offset_m"])
        )
        path_report = {
            "frame_id": result.nav_path.header.frame_id,
            "operation_width_m": width,
            "route_type": str(config["route_type"]), "path_type": str(config["path_type"]),
            "nav_path": nav_points, "swaths": swaths, "raw_swaths": raw_swaths,
            "turns": turns,
            "component_count": len(components),
            "mission_geometry": geometry,
            "swath_exclusion_intersection_count": intersection_count,
            "route_selection": selection["report"],
            "execution_strategy": "preflight forward/reverse staging poses, NavigateToPose with explicit swath yaw, brush-off entry, then every swath/turn",
        }
        self._write_json(self.get_parameter("path_output_path").value, path_report)

        execution_started_wall = time.monotonic()
        if selection["selected"] is None or intersection_count:
            transit = {
                "success": False,
                "error": "no_reachable_clean_route",
                "swath_exclusion_intersection_count": intersection_count,
                "preflight": selection["report"],
            }
            selected_components = []
        else:
            selected = selection["selected"]
            selected_components = selected["components"]
            self._set_state("TRANSIT", {"selected_direction": selected["direction"]})
            transit = self._navigate_to(selected["staging_pose"])
            if transit["success"]:
                self._set_state("ALIGNING")
                current_point = self.estimated_pose[:2] if self.estimated_pose else (
                    selected["staging_pose"]["x"], selected["staging_pose"]["y"]
                )
                entry = {
                    "kind": "entry", "index": -1, "brush": False,
                    "points": entry_points(current_point, selected_components[0]),
                }
                transit["entry"] = self._follow_component(entry)
                transit["success"] = transit["entry"]["success"]
        component_results = []
        if transit["success"]:
            for component in selected_components:
                self._set_state(
                    "EXECUTING_SWATH" if component["kind"] == "swath" else "EXECUTING_TURN",
                    {"kind": component["kind"], "index": component["index"]},
                )
                component_results.append(self._follow_component(component))
                if not component_results[-1]["success"]:
                    self._set_state("RECOVERY", component_results[-1])
                    break
        self._set_brush(False)
        execution_duration = time.monotonic() - execution_started_wall
        complete = bool(selected_components and transit["success"] and len(component_results) == len(selected_components) and all(item["success"] for item in component_results))
        empirical = empirical_swept_metrics(
            cleanable_polygon, self.brush_samples, width, resolution=0.10,
            exclusion_polygons=cleanable_exclusions,
        )
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
        keepout_violation_samples = sum(
            any(point_in_polygon(sample[1], sample[2], polygon) for polygon in exclusions)
            for sample in self.truth_samples
        )
        brush_state_violation_samples = sum(
            bool(sample[4]) and sample[5] != "EXECUTING_SWATH"
            for sample in self.truth_samples
        )
        recovery_count = sum(item["retries"] for item in component_results) + int(transit.get("retries", 0))
        efficiency_pass = empirical["net_efficiency_m2_h"] >= 3500.0
        localization_xy_errors, localization_sync_errors, dropped_estimates = (
            synchronized_xy_errors(self.estimate_samples, self.truth_samples, 0.05)
        )
        localization = summarize_distances(localization_xy_errors)
        sync_summary = summarize_distances(localization_sync_errors)
        localization.update({
            "estimate_sample_count": len(self.estimate_samples),
            "truth_sample_count": len(self.truth_samples),
            "dropped_estimate_count": dropped_estimates,
            "sync_tolerance_sec": 0.05,
            "sync_error_sec": {
                "sample_count": sync_summary["sample_count"],
                "rmse_sec": sync_summary["rmse_m"],
                "p95_sec": sync_summary["p95_m"],
                "max_sec": sync_summary["max_m"],
            },
            "metric_basis": "nearest_neighbour_ros_timestamp_without_truth_reuse",
        })
        localization["pass_rmse_and_p95_at_most_0_05m"] = bool(
            localization["rmse_m"] is not None
            and localization["rmse_m"] <= 0.05
            and localization["p95_m"] <= 0.05
        )
        localization["pass_rmse_at_most_0_05m"] = bool(
            localization["rmse_m"] is not None
            and localization["rmse_m"] <= 0.05
        )
        localization["formal_gate_basis"] = (
            "Stage4V-compatible per-seed XY RMSE <= 0.05 m; pointwise P95 is diagnostic"
        )
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
            "component_count": len(selected_components), "component_results": component_results,
            "transit_to_start": transit,
            "mission_geometry": geometry,
            "route_selection": selection["report"],
            "swath_exclusion_intersection_count": intersection_count,
            "planned_metrics": planned_metrics,
            "empirical_metrics": empirical,
            "recovery_count": recovery_count,
            "collision_count": self.collision_events,
            "keepout_violation_sample_count": keepout_violation_samples,
            "brush_state_violation_sample_count": brush_state_violation_samples,
            "minimum_lidar_range_m": self.minimum_scan_range if math.isfinite(self.minimum_scan_range) else None,
            "brush_disabled_on_exit": not self.brush_enabled,
            "localization_regression_during_coverage": localization,
            "execution_boundary": "All path components were required to terminate SUCCEEDED; no 180-pose shortcut or bounded cancellation is accepted.",
        }
        self._set_state("COMPLETED" if report["success"] else "FAILED", report)
        return self._write_report(report)

    def _plan(self, config, polygon, exclusions):
        if not self.coverage_client.wait_for_server(timeout_sec=60.0):
            return {"error": "coverage_action_timeout"}
        goal = ComputeCoveragePath.Goal()
        goal.generate_headland = bool(config["headland"]["enabled"]); goal.generate_route = True; goal.generate_path = True
        goal.frame_id = str(config["frame_id"])
        closed = polygon if polygon[0] == polygon[-1] else polygon + [polygon[0]]
        coordinates = Coordinates(); coordinates.coordinates = [Coordinate(axis1=x, axis2=y) for x, y in closed]
        goal.polygons = [coordinates]
        for exclusion in exclusions:
            closed_exclusion = exclusion if exclusion[0] == exclusion[-1] else exclusion + [exclusion[0]]
            cutout = Coordinates(); cutout.coordinates = [
                Coordinate(axis1=x, axis2=y) for x, y in closed_exclusion
            ]
            goal.polygons.append(cutout)
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

    @staticmethod
    def _swath_exclusion_intersections(swaths, outer, exclusions):
        count = 0
        for start, end in swaths:
            samples = CoverageProbe._interpolate(start, end, spacing=0.05)
            if any(
                not point_in_cleanable_area(x, y, outer, exclusions)
                for x, y in samples[1:-1]
            ):
                count += 1
        return count

    def _select_route(self, components, geometry, offset_m):
        candidates = route_candidates(components, offset_m)
        staging_points = [
            (candidate["staging_pose"]["x"], candidate["staging_pose"]["y"])
            for candidate in candidates
        ]
        grid_diagnostics_ready = self._wait_for_grid_diagnostics(
            30.0, staging_points
        )
        reports = []
        viable = []
        for candidate in candidates:
            operation_entry_yaw = candidate["staging_pose"]["yaw"]
            if self.estimated_pose is None:
                reports.append({
                    "direction": candidate["direction"],
                    "success": False,
                    "error": "fused_pose_unavailable_for_explicit_transit_yaw",
                })
                continue
            pose = transit_pose(self.estimated_pose[:2], candidate["staging_pose"])
            candidate["staging_pose"] = pose
            clearance = exclusion_clearance(
                (pose["x"], pose["y"]),
                geometry["exclusion_polygons"],
            )
            preflight = self._compute_path(pose)
            target_grid_values = self._target_grid_values(pose)
            target_cost_clear = target_grid_viable(target_grid_values)
            report = {
                "direction": candidate["direction"],
                "staging_pose": pose,
                "transit_yaw_source": "fused_pose_to_staging_pose",
                "operation_entry_yaw": operation_entry_yaw,
                "footprint_clearance_m": clearance,
                "target_grid_values": target_grid_values,
                "target_cost_and_keepout_clear": target_cost_clear,
                "staging_inside_operation_polygon": point_in_cleanable_area(
                    pose["x"], pose["y"], geometry["outer_polygon"], []
                ),
                "collision_free_footprint_path": preflight["success"],
                **preflight,
            }
            reports.append(report)
            if preflight["success"] and (
                clearance is None or clearance >= geometry["safety_margin_m"]
            ) and target_cost_clear:
                candidate["preflight"] = report
                viable.append(candidate)
        selected = min(
            viable, key=lambda item: item["preflight"]["path_length_m"],
            default=None,
        )
        return {
            "selected": selected,
            "report": {
                "candidates": reports,
                "selected_direction": selected["direction"] if selected else None,
                "grid_diagnostics_ready": grid_diagnostics_ready,
                "selection_basis": "shortest reachable staging path with geometry clearance",
            },
        }

    def _wait_for_estimated_pose(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and self.estimated_pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.estimated_pose is not None

    def _wait_for_grid_diagnostics(self, timeout_sec, required_global_points=None):
        required_global_points = required_global_points or []
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            grids_ready = all(
                (self.global_costmap, self.keepout_mask, self.speed_mask)
            )
            points_covered = grids_ready and all(
                self._sample_grid(self.global_costmap, x, y) is not None
                for x, y in required_global_points
            )
            if grids_ready and points_covered:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _compute_path(self, pose):
        if not self.compute_path_client.wait_for_server(timeout_sec=30.0):
            return {"success": False, "error": "compute_path_server_unavailable"}
        goal = ComputePathToPose.Goal()
        goal.goal = self._pose_message(pose)
        goal.planner_id = "GridBased"
        goal.use_start = False
        future = self.compute_path_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            return {"success": False, "error": "compute_path_rejected"}
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        if not result_future.done():
            return {"success": False, "error": "compute_path_timeout"}
        wrapped = result_future.result()
        result = wrapped.result
        points = [
            (item.pose.position.x, item.pose.position.y)
            for item in result.path.poses
        ]
        error_code = int(getattr(result, "error_code", 0))
        success = wrapped.status == GoalStatus.STATUS_SUCCEEDED and error_code == 0 and bool(points)
        return {
            "success": success,
            "terminal_status": int(wrapped.status),
            "error_code": error_code,
            "error_name": COMPUTE_PATH_ERRORS.get(error_code, f"ERROR_{error_code}"),
            "error_msg": str(getattr(result, "error_msg", "")),
            "path_length_m": path_length(points),
            "path_pose_count": len(points),
            "path_start": points[0] if points else None,
            "path_goal": points[-1] if points else None,
        }

    @staticmethod
    def _pose_message(pose):
        message = PoseStamped(); message.header.frame_id = "map"
        message.pose.position.x = float(pose["x"])
        message.pose.position.y = float(pose["y"])
        message.pose.orientation.z = math.sin(float(pose["yaw"]) / 2.0)
        message.pose.orientation.w = math.cos(float(pose["yaw"]) / 2.0)
        return message

    def _path_message(self, points):
        path = NavPath(); path.header.frame_id = "map"
        if len(points) < 2:
            raise ValueError("a coverage path needs at least two points for yaw")
        for index, (x, y) in enumerate(points):
            next_point = points[min(index + 1, len(points) - 1)]
            previous = points[max(0, index - 1)]
            yaw = math.atan2(next_point[1] - previous[1], next_point[0] - previous[0])
            pose = PoseStamped(); pose.header.frame_id = "map"
            pose.pose.position.x = float(x); pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(yaw / 2.0); pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        return path

    def _navigate_to(self, pose):
        self._set_brush(False)
        if not self._wait_controller_active() or not self.navigate_client.wait_for_server(timeout_sec=30.0):
            return {"success": False, "error": "navigate_server_unavailable", "retries": 0}
        goal = NavigateToPose.Goal(); goal.pose = self._pose_message(pose)
        result = self._run_action(
            self.navigate_client, goal, False, 180.0, "navigate_to_pose"
        )
        result["goal_pose"] = pose
        result["controller"] = "NavigateToPose default controller"
        result["progress_checker"] = "nav2_controller::PoseProgressChecker"
        result["terminal_tracking_error"] = self._tracking_error(pose)
        return result

    def _follow_component(self, component):
        goal = FollowPath.Goal(); goal.path = self._path_message(component["points"])
        self.current_path_publisher.publish(goal.path)
        goal.controller_id = "FollowPath"; goal.goal_checker_id = "goal_checker"; goal.progress_checker_id = "progress_checker"
        timeout = max(self.get_parameter("minimum_component_timeout_sec").value, path_length(component["points"]) / 0.10 + 30.0)
        result = self._run_action(
            self.follow_client, goal, component["brush"], timeout, "follow_path"
        )
        goal_pose = {"x": component["points"][-1][0], "y": component["points"][-1][1], "yaw": segment_heading(component["points"][-2], component["points"][-1])}
        result.update({"kind": component["kind"], "index": component["index"], "brush_enabled": component["brush"], "path_pose_count": len(component["points"]), "planned_length_m": path_length(component["points"]), "goal_pose": goal_pose, "terminal_tracking_error": self._tracking_error(goal_pose)})
        return result

    def _run_action(self, client, goal, brush, timeout, action_kind):
        retry_limit = int(self.get_parameter("component_retry_limit").value)
        attempts = []
        for attempt in range(retry_limit + 1):
            feedback_samples = []
            last_feedback_sample_time = 0.0

            def on_feedback(message):
                nonlocal last_feedback_sample_time
                now = time.monotonic()
                if now - last_feedback_sample_time < 1.0 or len(feedback_samples) >= 300:
                    return
                last_feedback_sample_time = now
                feedback = message.feedback
                sample = {}
                for name in (
                    "distance_remaining", "distance_to_goal", "speed",
                    "number_of_recoveries",
                ):
                    value = getattr(feedback, name, None)
                    if value is not None:
                        sample[name] = float(value)
                feedback_samples.append(sample)

            self._set_brush(brush)
            send = client.send_goal_async(goal, feedback_callback=on_feedback); rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
            handle = send.result() if send.done() else None
            if handle is None or not handle.accepted:
                attempts.append({"attempt": attempt + 1, "accepted": False, "feedback": feedback_samples})
                continue
            result_future = handle.get_result_async(); deadline = time.monotonic() + timeout
            while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
                self._set_brush(brush); rclpy.spin_once(self, timeout_sec=0.05); time.sleep(0.02)
            if not result_future.done():
                cancel = handle.cancel_goal_async(); rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
                cancel_response = cancel.result() if cancel.done() else None
                attempts.append({
                    "attempt": attempt + 1, "accepted": True, "timeout": True,
                    "cancel_requested": True,
                    "cancel_response_received": cancel_response is not None,
                    "cancel_return_code": int(cancel_response.return_code) if cancel_response else None,
                    "canceling_goal_count": len(cancel_response.goals_canceling) if cancel_response else 0,
                    "feedback": feedback_samples,
                    "terminal_estimated_pose": self.estimated_pose,
                    "terminal_ground_truth_pose_evaluation_only": self.truth_pose,
                })
                continue
            wrapped = result_future.result(); error_code = int(getattr(wrapped.result, "error_code", 0))
            succeeded = wrapped.status == GoalStatus.STATUS_SUCCEEDED and error_code == 0
            error_names = (
                FOLLOW_PATH_ERRORS
                if action_kind == "follow_path"
                else NAVIGATE_TO_POSE_ERRORS
            )
            attempts.append({
                "attempt": attempt + 1,
                "accepted": True,
                "terminal_status": int(wrapped.status),
                "error_code": error_code,
                "error_name": error_names.get(error_code, f"ERROR_{error_code}"),
                "error_msg": str(getattr(wrapped.result, "error_msg", "")),
                "succeeded": succeeded,
                "feedback": feedback_samples,
                "terminal_estimated_pose": self.estimated_pose,
                "terminal_ground_truth_pose_evaluation_only": self.truth_pose,
                "action_kind": action_kind,
                "controller_id": "FollowPath" if action_kind == "follow_path" else None,
                "progress_checker_id": "progress_checker",
                "goal_checker_id": "goal_checker",
            })
            if succeeded:
                self._set_brush(False if brush else brush)
                return {"success": True, "retries": attempt, "attempts": attempts}
        self._set_brush(False)
        return {"success": False, "retries": max(0, len(attempts) - 1), "attempts": attempts}

    @staticmethod
    def _sample_grid(grid, x, y):
        if grid is None or grid.info.resolution <= 0.0:
            return None
        origin = grid.info.origin
        yaw = yaw_from_quaternion(origin.orientation)
        dx = float(x) - origin.position.x
        dy = float(y) - origin.position.y
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        column = int(math.floor(local_x / grid.info.resolution))
        row = int(math.floor(local_y / grid.info.resolution))
        if not (0 <= column < grid.info.width and 0 <= row < grid.info.height):
            return None
        index = row * grid.info.width + column
        return int(grid.data[index]) if 0 <= index < len(grid.data) else None

    def _target_grid_values(self, pose):
        return {
            "global_costmap_received": self.global_costmap is not None,
            "global_costmap_cost": self._sample_grid(
                self.global_costmap, pose["x"], pose["y"]
            ),
            "keepout_mask_received": self.keepout_mask is not None,
            "keepout_mask_value": self._sample_grid(
                self.keepout_mask, pose["x"], pose["y"]
            ),
            "speed_mask_received": self.speed_mask is not None,
            "speed_mask_value": self._sample_grid(
                self.speed_mask, pose["x"], pose["y"]
            ),
            "global_costmap_bounds": self._grid_bounds(self.global_costmap),
        }

    @staticmethod
    def _grid_bounds(grid):
        if grid is None:
            return None
        origin = grid.info.origin.position
        return {
            "origin_xy": [origin.x, origin.y],
            "resolution_m": grid.info.resolution,
            "width": grid.info.width,
            "height": grid.info.height,
            "axis_aligned_max_xy": [
                origin.x + grid.info.width * grid.info.resolution,
                origin.y + grid.info.height * grid.info.resolution,
            ],
        }

    def _tracking_error(self, goal_pose):
        def error(pose):
            if pose is None:
                return None
            return {
                "position_m": math.hypot(
                    pose[0] - goal_pose["x"], pose[1] - goal_pose["y"]
                ),
                "heading_rad": abs(math.atan2(
                    math.sin(pose[2] - goal_pose["yaw"]),
                    math.cos(pose[2] - goal_pose["yaw"]),
                )),
            }
        return {
            "estimated_pose": error(self.estimated_pose),
            "ground_truth_evaluation_only": error(self.truth_pose),
        }

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
            writer = csv.writer(stream); writer.writerow(["stamp_sec", "base_x_m", "base_y_m", "yaw_rad", "brush_enabled", "coverage_state"]); writer.writerows(self.truth_samples)

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
