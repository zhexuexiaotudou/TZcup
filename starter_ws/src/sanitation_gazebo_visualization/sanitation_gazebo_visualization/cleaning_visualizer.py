"""Render the real cleaning mission directly in Gazebo's MarkerManager."""

from __future__ import annotations

import json
import math
import queue
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path as FilePath

from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String
import yaml

from .telemetry_v2 import SCHEMA as TELEMETRY_V2_SCHEMA, classify_motion_state, decimate_xy


def yaw_from_quaternion(quaternion) -> float:
    siny = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny, cosy)


def quote_proto(value: str) -> str:
    return json.dumps(value, ensure_ascii=True)


def color_proto(red: float, green: float, blue: float, alpha: float) -> str:
    color = f"r:{red:.4f} g:{green:.4f} b:{blue:.4f} a:{alpha:.4f}"
    return (
        f"material {{ ambient {{{color}}} diffuse {{{color}}} "
        "lighting:false render_order:20.0 }"
    )


@dataclass(frozen=True)
class Point2:
    x: float
    y: float


def point_in_polygon(point: Point2, polygon: list[Point2]) -> bool:
    """Return whether a point is inside a simple polygon."""
    if len(polygon) < 3:
        return False
    inside = False
    previous = polygon[-1]
    for current in polygon:
        if (previous.y > point.y) != (current.y > point.y):
            crossing_x = (
                (current.x - previous.x)
                * (point.y - previous.y)
                / (current.y - previous.y)
                + previous.x
            )
            if point.x < crossing_x:
                inside = not inside
        previous = current
    return inside


class CleaningVisualizer(Node):
    """Send route, swept-area, and status markers to the Gazebo GUI only."""

    def __init__(self) -> None:
        super().__init__("sanitation_gazebo_cleaning_visualizer")
        self.marker_service = str(
            self.declare_parameter("marker_service", "/marker").value
        )
        self.operation_width_m = float(
            self.declare_parameter("operation_width_m", 0.65).value
        )
        self.brush_forward_offset_m = float(
            self.declare_parameter("brush_forward_offset_m", 0.55).value
        )
        self.trail_spacing_m = float(
            self.declare_parameter("trail_spacing_m", 0.24).value
        )
        self.trail_height_m = float(
            self.declare_parameter("trail_height_m", 0.018).value
        )
        self.expected_components = int(
            self.declare_parameter("expected_components", 17).value
        )
        self.profile_label = str(
            self.declare_parameter("profile_label", "STANDARD DEMO").value
        )
        self.map_area_m2 = float(
            self.declare_parameter("map_area_m2", 4000.0).value
        )
        self.mission_scope = str(
            self.declare_parameter("mission_scope", "LIVE DEMO AREA").value
        )
        self.mission_config = str(
            self.declare_parameter("mission_config", "").value
        )
        telemetry_output_path = str(
            self.declare_parameter("telemetry_output_path", "").value
        )
        self.telemetry_output_path = (
            FilePath(telemetry_output_path) if telemetry_output_path else None
        )
        self.world_to_map_x = float(
            self.declare_parameter("world_to_map_x", 8.0).value
        )
        self.world_to_map_y = float(
            self.declare_parameter("world_to_map_y", 0.0).value
        )
        self.world_to_map_yaw = float(
            self.declare_parameter("world_to_map_yaw", 0.0).value
        )
        self.service_timeout_ms = int(
            self.declare_parameter("service_timeout_ms", 3000).value
        )
        self.cleaning_cell_m = float(
            self.declare_parameter("cleaning_cell_m", 0.20).value
        )
        self.simulation_speed_label = str(
            self.declare_parameter("simulation_speed_label", "2X TARGET").value
        )
        self.world_name = str(
            self.declare_parameter("world_name", "sanitation_competition_demo").value
        )

        self.gz_binary = shutil.which("gz")
        if not self.gz_binary:
            raise RuntimeError("Gazebo command-line client 'gz' is unavailable")

        self.map_x = 0.0
        self.map_y = 0.0
        self.map_yaw = 0.0
        self.have_pose = False
        self.current_speed_mps = 0.0
        self.total_distance_m = 0.0
        self.last_pose_map: Point2 | None = None
        self.last_pose_stamp_sec: float | None = None
        self.mission_start_stamp_sec: float | None = None
        self.trajectory_map: list[Point2] = []
        self.planned_path_map: list[Point2] = []
        self.planned_swaths_map: list[list[Point2]] = []
        self.planned_connectors_map: list[list[Point2]] = []
        self.planned_repairs_map: list[list[Point2]] = []
        self.actual_cleaning_map: list[list[Point2]] = []
        self.actual_transit_map: list[list[Point2]] = []
        self.actual_repair_map: list[list[Point2]] = []
        self.last_motion_layer = ""
        self.brush_enabled = False
        self.coverage_state = "WAITING"
        self.completed_components = 0
        self.last_component_signature = ""
        self.last_brush_point: Point2 | None = None
        self.trail_points: list[Point2] = []
        self.trail_session_id = 0
        self.trail_dirty = False
        self.last_trail_update = 0.0
        self.markers_cleared = False
        self.clear_queued = False
        self.dropped_requests = 0
        self.pending_kinds: set[str] = set()
        self.outer_polygon_map: list[Point2] = []
        self.cleanable_polygon_map: list[Point2] = []
        self.exclusion_polygons_map: list[list[Point2]] = []
        self.cleanable_cells: dict[tuple[int, int], Point2] = {}
        self.cleaned_cells: set[tuple[int, int]] = set()
        self.targets: list[dict] = []
        self.show_zone_fill = False
        self.home_world: Point2 | None = None
        self.work_start_world: Point2 | None = None
        self.static_markers_queued = False
        self.static_markers_sent = False
        self._load_mission_geometry()

        self.request_queue: queue.Queue[tuple[str, list[str]]] = queue.Queue(
            maxsize=48
        )
        self.worker_stop = threading.Event()
        self.worker = threading.Thread(
            target=self._service_worker,
            name="gazebo-marker-service",
            daemon=True,
        )
        self.worker.start()

        self.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self._on_truth,
            qos_profile_sensor_data,
        )
        self.create_subscription(Bool, "/brush_enabled", self._on_brush, 10)
        self.create_subscription(String, "/coverage/state", self._on_state, 10)
        self.create_subscription(
            String, "/coverage/component_state", self._on_component, 10
        )
        self.create_subscription(Path, "/coverage/current_path", self._on_path, 10)
        plan_qos = QoSProfile(depth=1)
        plan_qos.reliability = ReliabilityPolicy.RELIABLE
        plan_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, "/coverage/full_plan", self._on_full_plan, plan_qos)
        self.create_subscription(String, "/coverage/planned_repairs", self._on_repairs, plan_qos)
        self.telemetry_publisher = self.create_publisher(
            String, "/coverage/gazebo_telemetry", 10
        )
        self.actual_cleaning_publisher = self.create_publisher(String, "/coverage/actual_cleaning_trajectory", 10)
        self.actual_transit_publisher = self.create_publisher(String, "/coverage/actual_transit_trajectory", 10)
        self.actual_repair_publisher = self.create_publisher(String, "/coverage/actual_repair_trajectory", 10)
        self.create_timer(0.1, self._update_markers)
        self.create_timer(0.5, self._publish_telemetry)
        self.get_logger().info(
            "Gazebo cleaning visualization ready: "
            f"service={self.marker_service} width={self.operation_width_m:.2f} m "
            f"components={self.expected_components}"
        )

    def destroy_node(self) -> bool:
        self.worker_stop.set()
        self.worker.join(timeout=2.0)
        return super().destroy_node()

    def map_to_world(self, map_x: float, map_y: float) -> Point2:
        dx = map_x - self.world_to_map_x
        dy = map_y - self.world_to_map_y
        cosine = math.cos(self.world_to_map_yaw)
        sine = math.sin(self.world_to_map_yaw)
        return Point2(cosine * dx + sine * dy, -sine * dx + cosine * dy)

    def _load_mission_geometry(self) -> None:
        if not self.mission_config:
            return
        config_path = FilePath(self.mission_config)
        try:
            config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as error:
            self.get_logger().warning(f"Mission boundary unavailable: {error}")
            return
        polygon = config.get("outer_polygon", []) if isinstance(config, dict) else []
        try:
            self.outer_polygon_map = [
                Point2(float(point[0]), float(point[1])) for point in polygon
            ]
        except (TypeError, ValueError, IndexError):
            self.outer_polygon_map = []
            self.get_logger().warning("Mission outer_polygon is malformed")
            return
        keepouts = config.get("keepout_polygons", [])
        exclusions = config.get("exclusion_polygons", [])
        for polygon in [*keepouts, *exclusions]:
            try:
                self.exclusion_polygons_map.append(
                    [Point2(float(point[0]), float(point[1])) for point in polygon]
                )
            except (TypeError, ValueError, IndexError):
                self.exclusion_polygons_map = []
                break
        for target in config.get("cleaning_targets", []):
            try:
                position = target["position"]
                self.targets.append({
                    "id": str(target["id"]),
                    "class": str(target.get("class", "other")),
                    "x": float(position[0]),
                    "y": float(position[1]),
                    "radius_m": float(target.get("collection_radius_m", 0.55)),
                    "model_name": str(target.get("model_name", "")),
                    "cleaned": False,
                    "removed_from_scene": False,
                })
            except (KeyError, TypeError, ValueError, IndexError):
                self.get_logger().warning("Ignoring malformed cleaning target")
        self.show_zone_fill = (
            len(self.outer_polygon_map) == 4 and not keepouts and not exclusions
        )
        self.cleanable_polygon_map = list(self.outer_polygon_map)
        if len(self.outer_polygon_map) == 4:
            inset = float(config.get("headland", {}).get("width_m", 0.0))
            min_x = min(point.x for point in self.outer_polygon_map) + inset
            max_x = max(point.x for point in self.outer_polygon_map) - inset
            min_y = min(point.y for point in self.outer_polygon_map) + inset
            max_y = max(point.y for point in self.outer_polygon_map) - inset
            if min_x < max_x and min_y < max_y:
                self.cleanable_polygon_map = [
                    Point2(min_x, min_y), Point2(max_x, min_y),
                    Point2(max_x, max_y), Point2(min_x, max_y),
                ]
        self._build_cleanable_grid()

    def _build_cleanable_grid(self) -> None:
        polygon = self.cleanable_polygon_map
        if not polygon or self.cleaning_cell_m <= 0.0:
            return
        min_x = min(point.x for point in polygon)
        max_x = max(point.x for point in polygon)
        min_y = min(point.y for point in polygon)
        max_y = max(point.y for point in polygon)
        rows = int(math.ceil((max_y - min_y) / self.cleaning_cell_m))
        columns = int(math.ceil((max_x - min_x) / self.cleaning_cell_m))
        for row in range(rows):
            for column in range(columns):
                point = Point2(
                    min_x + (column + 0.5) * self.cleaning_cell_m,
                    min_y + (row + 0.5) * self.cleaning_cell_m,
                )
                if point_in_polygon(point, polygon) and not any(
                    point_in_polygon(point, polygon)
                    for polygon in self.exclusion_polygons_map
                ):
                    self.cleanable_cells[(column, row)] = point

    def _on_truth(self, message: Odometry) -> None:
        pose = message.pose.pose
        stamp_sec = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) / 1_000_000_000.0
        )
        self.map_x = float(pose.position.x)
        self.map_y = float(pose.position.y)
        self.map_yaw = yaw_from_quaternion(pose.orientation)
        self.current_speed_mps = math.hypot(
            float(message.twist.twist.linear.x),
            float(message.twist.twist.linear.y),
        )
        current = Point2(self.map_x, self.map_y)
        if self.last_pose_map is not None:
            step = math.hypot(
                current.x - self.last_pose_map.x,
                current.y - self.last_pose_map.y,
            )
            if step < 1.0:
                self.total_distance_m += step
        if not self.trajectory_map or math.hypot(
            current.x - self.trajectory_map[-1].x,
            current.y - self.trajectory_map[-1].y,
        ) >= 0.12:
            self.trajectory_map.append(current)
            if len(self.trajectory_map) > 1200:
                self.trajectory_map = self.trajectory_map[-1200:]
            layer_name = classify_motion_state(self.coverage_state, self.brush_enabled)
            layer_segments = {
                "cleaning": self.actual_cleaning_map,
                "transit": self.actual_transit_map,
                "repair": self.actual_repair_map,
            }[layer_name]
            if layer_name != self.last_motion_layer or not layer_segments:
                layer_segments.append([current])
                self.last_motion_layer = layer_name
            else:
                segment = layer_segments[-1]
                if not segment or math.hypot(current.x - segment[-1].x, current.y - segment[-1].y) >= 0.12:
                    segment.append(current)
                    if len(segment) > 1200:
                        del segment[:-1200]
        self.last_pose_map = current
        self.last_pose_stamp_sec = stamp_sec
        if self.home_world is None:
            self.home_world = self.map_to_world(self.map_x, self.map_y)
        self.have_pose = True
        if self.brush_enabled:
            self._record_cleaning_footprint()

    def _on_brush(self, message: Bool) -> None:
        next_enabled = bool(message.data)
        if next_enabled and not self.brush_enabled:
            self.trail_session_id += 1
            self.trail_points = []
            self.last_brush_point = None
        self.brush_enabled = next_enabled
        if not next_enabled:
            self.last_brush_point = None

    def _on_state(self, message: String) -> None:
        self.coverage_state = message.data
        if self.mission_start_stamp_sec is None and self.coverage_state not in {
            "WAITING", "READY", "WAITING_FOR_START", "STOPPED", "COMPLETED", "FAILED"
        }:
            self._begin_mission_metrics()
        if self.coverage_state == "COMPLETED":
            self.completed_components = self.expected_components

    def _on_component(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        self.coverage_state = str(payload.get("state", self.coverage_state))
        if self.mission_start_stamp_sec is None and self.coverage_state not in {
            "WAITING", "READY", "WAITING_FOR_START", "STOPPED", "COMPLETED", "FAILED"
        }:
            self._begin_mission_metrics()
        if self.coverage_state.startswith("EXECUTING_") or self.coverage_state.startswith("REPAIR_"):
            signature = json.dumps(
                {
                    "state": self.coverage_state,
                    "kind": payload.get("kind"),
                    "index": payload.get("index"),
                },
                sort_keys=True,
            )
            if signature != self.last_component_signature:
                self.last_component_signature = signature
                self.completed_components = min(
                    self.completed_components + 1, self.expected_components
                )

    def _begin_mission_metrics(self) -> None:
        self.mission_start_stamp_sec = self.last_pose_stamp_sec
        self.total_distance_m = 0.0
        self.trajectory_map = []
        self.actual_cleaning_map = []
        self.actual_transit_map = []
        self.actual_repair_map = []
        self.last_motion_layer = ""
        self.cleaned_cells.clear()
        self.completed_components = 0
        for target in self.targets:
            target["cleaned"] = False
            target["removed_from_scene"] = False

    def _on_path(self, message: Path) -> None:
        if len(message.poses) < 2 or not self.markers_cleared:
            return
        if (
            self.work_start_world is None
            and self.coverage_state == "EXECUTING_SWATH"
        ):
            first_pose = message.poses[0].pose.position
            self.work_start_world = self.map_to_world(first_pose.x, first_pose.y)
            self._enqueue(
                "work_start", self._build_point_markers(
                    "tzcup_cleaning_start",
                    self.work_start_world,
                    "CLEANING START",
                    (0.16, 1.0, 0.38, 1.0),
                )
            )
        points = []
        self.planned_path_map = []
        for stamped_pose in message.poses:
            self.planned_path_map.append(Point2(
                float(stamped_pose.pose.position.x),
                float(stamped_pose.pose.position.y),
            ))
            point = self.map_to_world(
                stamped_pose.pose.position.x, stamped_pose.pose.position.y
            )
            points.append(f"point {{x:{point.x:.5f} y:{point.y:.5f} z:0.055}}")
        marker = (
            "ns:\"tzcup_current_cleaning_path\" id:1 "
            "action:ADD_MODIFY type:LINE_STRIP visibility:GUI layer:3 "
            "scale {x:0.065 y:0.065 z:0.065} "
            f"{color_proto(1.0, 0.72, 0.08, 0.95)} "
            + " ".join(points)
        )
        self._enqueue("path", [marker])

    @staticmethod
    def _component_points(component: dict) -> list[Point2]:
        return [Point2(float(point[0]), float(point[1])) for point in component.get("points", [])]

    def _on_full_plan(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            components = payload["components"]
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning("Ignoring malformed /coverage/full_plan")
            return
        self.expected_components = int(payload.get("component_count", len(components)))
        self.planned_swaths_map = [
            self._component_points(item) for item in components if item.get("kind") == "SWATH"
        ]
        self.planned_connectors_map = [
            self._component_points(item) for item in components
            if item.get("kind") in {"TRANSIT", "ROTATE", "SHIFT", "BACKUP", "OBSTACLE_BYPASS", "RETURN_HOME"}
            and len(item.get("points", [])) >= 2
        ]

    def _on_repairs(self, message: String) -> None:
        try:
            components = json.loads(message.data)
            self.planned_repairs_map = [self._component_points(item) for item in components]
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning("Ignoring malformed /coverage/planned_repairs")

    def _record_cleaning_footprint(self) -> None:
        brush = Point2(
            self.map_x + self.brush_forward_offset_m * math.cos(self.map_yaw),
            self.map_y + self.brush_forward_offset_m * math.sin(self.map_yaw),
        )
        radius = self.operation_width_m / 2.0
        for cell_index, point in self.cleanable_cells.items():
            if cell_index in self.cleaned_cells:
                continue
            if math.hypot(point.x - brush.x, point.y - brush.y) <= radius:
                self.cleaned_cells.add(cell_index)
        for target in self.targets:
            if not target["cleaned"] and math.hypot(
                target["x"] - brush.x, target["y"] - brush.y
            ) <= target["radius_m"]:
                target["cleaned"] = True
                if target["model_name"]:
                    self._enqueue(
                        f"remove_target:{target['id']}", [target["model_name"]]
                    )

    def _publish_telemetry(self) -> None:
        if not self.have_pose:
            return
        total_cells = len(self.cleanable_cells)
        cleaned_count = len(self.cleaned_cells)
        cleaned_area = cleaned_count * self.cleaning_cell_m**2
        total_area = total_cells * self.cleaning_cell_m**2
        elapsed_sec = 0.0
        if self.mission_start_stamp_sec is not None and self.last_pose_stamp_sec is not None:
            elapsed_sec = max(0.0, self.last_pose_stamp_sec - self.mission_start_stamp_sec)
        rate = cleaned_area / elapsed_sec * 60.0 if elapsed_sec > 0.0 else 0.0
        trajectory = self.trajectory_map[::max(1, len(self.trajectory_map) // 240)]
        cleaned_points = [self.cleanable_cells[index] for index in self.cleaned_cells]
        semantic_paths = {
            "planned_swaths": [decimate_xy(points, 120) for points in self.planned_swaths_map],
            "planned_connectors": [decimate_xy(points, 120) for points in self.planned_connectors_map],
            "planned_repairs": [decimate_xy(points, 120) for points in self.planned_repairs_map],
            "current_component": [[point.x, point.y] for point in self.planned_path_map],
            "actual_cleaning": [decimate_xy(points) for points in self.actual_cleaning_map],
            "actual_transit": [decimate_xy(points) for points in self.actual_transit_map],
            "actual_repair": [decimate_xy(points) for points in self.actual_repair_map],
        }
        payload = {
            "schema": TELEMETRY_V2_SCHEMA,
            "compatible_schema": "tzcup.gazebo_cleaning_telemetry.v1",
            "metric_basis": "gazebo_ground_truth_brush_footprint_evaluation_only",
            "state": self.coverage_state,
            "brush_enabled": self.brush_enabled,
            "progress_percent": 100.0 * cleaned_count / total_cells if total_cells else 0.0,
            "cleaned_area_m2": cleaned_area,
            "total_area_m2": total_area,
            "cleaning_rate_m2_min": rate,
            "targets_cleaned": sum(target["cleaned"] for target in self.targets),
            "targets_total": len(self.targets),
            "completed_components": self.completed_components,
            "expected_components": self.expected_components,
            "elapsed_sim_sec": elapsed_sec,
            "distance_m": self.total_distance_m,
            "speed_mps": self.current_speed_mps,
            "simulation_speed": self.simulation_speed_label,
            "robot": {"x": self.map_x, "y": self.map_y, "yaw": self.map_yaw},
            "field_boundary": [[point.x, point.y] for point in self.outer_polygon_map],
            "boundary": [[point.x, point.y] for point in self.cleanable_polygon_map],
            "planned_path": [[point.x, point.y] for point in self.planned_path_map],
            "trajectory": [[point.x, point.y] for point in trajectory],
            "paths": semantic_paths,
            "cleaned_cells": [[point.x, point.y] for point in cleaned_points],
            "cell_size_m": self.cleaning_cell_m,
            "targets": self.targets,
        }
        encoded = json.dumps(payload, separators=(",", ":"))
        self.telemetry_publisher.publish(String(data=encoded))
        if self.telemetry_output_path is not None:
            self.telemetry_output_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.telemetry_output_path.with_suffix(".json.tmp")
            temporary.write_text(encoded, encoding="utf-8")
            temporary.replace(self.telemetry_output_path)
        self.actual_cleaning_publisher.publish(String(data=json.dumps(semantic_paths["actual_cleaning"], separators=(",", ":"))))
        self.actual_transit_publisher.publish(String(data=json.dumps(semantic_paths["actual_transit"], separators=(",", ":"))))
        self.actual_repair_publisher.publish(String(data=json.dumps(semantic_paths["actual_repair"], separators=(",", ":"))))

    def _update_markers(self) -> None:
        if not self.have_pose:
            return
        if not self.markers_cleared:
            if not self.clear_queued:
                self.clear_queued = True
                self._enqueue(
                    "clear",
                    [
                        f"ns:{quote_proto(namespace)} action:DELETE_ALL"
                        for namespace in (
                            "tzcup_cleaned_swath",
                            "tzcup_current_cleaning_path",
                            "tzcup_cleaning_status",
                            "tzcup_cleaning_zone",
                            "tzcup_cleanable_zone",
                            "tzcup_cleaning_home",
                            "tzcup_cleaning_start",
                        )
                    ],
                )
            return
        if not self.static_markers_sent:
            if not self.static_markers_queued:
                self.static_markers_queued = True
                self._enqueue("static", self._build_static_markers())
            return

        now = time.monotonic()
        trail_marker = self._build_trail_marker()
        if trail_marker:
            self._enqueue("update", [trail_marker])

    def _build_trail_marker(self) -> str | None:
        if not self.brush_enabled:
            self.last_brush_point = None
            if self.trail_dirty and len(self.trail_points) >= 2:
                return self._render_trail_marker()
            return None
        brush_map_x = self.map_x + self.brush_forward_offset_m * math.cos(
            self.map_yaw
        )
        brush_map_y = self.map_y + self.brush_forward_offset_m * math.sin(
            self.map_yaw
        )
        brush_world = self.map_to_world(brush_map_x, brush_map_y)
        if self.last_brush_point is None:
            self.last_brush_point = brush_world
            self.trail_points = [brush_world]
            return None
        dx = brush_world.x - self.last_brush_point.x
        dy = brush_world.y - self.last_brush_point.y
        distance = math.hypot(dx, dy)
        if distance < self.trail_spacing_m:
            return None
        if distance > 1.0:
            self.last_brush_point = brush_world
            self.trail_points = [brush_world]
            return None
        self.trail_points.append(brush_world)
        self.trail_dirty = True
        self.last_brush_point = brush_world
        now = time.monotonic()
        if now - self.last_trail_update < 2.0:
            return None
        return self._render_trail_marker()

    def _render_trail_marker(self) -> str:
        points = " ".join(
            f"point {{x:{point.x:.5f} y:{point.y:.5f} z:{self.trail_height_m:.5f}}}"
            for point in self.trail_points
        )
        marker = (
            "ns:\"tzcup_cleaned_swath\" "
            f"id:{self.trail_session_id} action:ADD_MODIFY type:LINE_STRIP "
            "visibility:GUI layer:2 "
            f"scale {{x:{self.operation_width_m:.5f} y:{self.operation_width_m:.5f} "
            f"z:{self.operation_width_m:.5f}}} "
            f"{color_proto(0.05, 0.82, 0.58, 0.58)} {points}"
        )
        self.trail_dirty = False
        self.last_trail_update = time.monotonic()
        return marker

    def _build_static_markers(self) -> list[str]:
        markers: list[str] = []
        if self.outer_polygon_map:
            polygon_world = [
                self.map_to_world(point.x, point.y)
                for point in self.outer_polygon_map
            ]
            closed = polygon_world + [polygon_world[0]]
            points = " ".join(
                f"point {{x:{point.x:.5f} y:{point.y:.5f} z:0.06000}}"
                for point in closed
            )
            markers.append(
                "ns:\"tzcup_cleaning_zone\" id:1 action:ADD_MODIFY "
                "type:LINE_STRIP visibility:GUI layer:4 "
                "scale {x:0.09000 y:0.09000 z:0.09000} "
                f"{color_proto(0.96, 0.55, 0.08, 1.0)} {points}"
            )
        if self.cleanable_polygon_map:
            cleanable_world = [
                self.map_to_world(point.x, point.y)
                for point in self.cleanable_polygon_map
            ]
            closed = cleanable_world + [cleanable_world[0]]
            points = " ".join(
                f"point {{x:{point.x:.5f} y:{point.y:.5f} z:0.07000}}"
                for point in closed
            )
            markers.append(
                "ns:\"tzcup_cleanable_zone\" id:1 action:ADD_MODIFY "
                "type:LINE_STRIP visibility:GUI layer:5 "
                "scale {x:0.07500 y:0.07500 z:0.07500} "
                f"{color_proto(0.08, 0.82, 1.0, 1.0)} {points}"
            )
            if self.show_zone_fill:
                xs = [point.x for point in cleanable_world]
                ys = [point.y for point in cleanable_world]
                center_x = 0.5 * (min(xs) + max(xs))
                center_y = 0.5 * (min(ys) + max(ys))
                markers.append(
                    "ns:\"tzcup_cleanable_zone\" id:2 action:ADD_MODIFY "
                    "type:BOX visibility:GUI layer:1 "
                    f"pose {{position {{x:{center_x:.5f} y:{center_y:.5f} z:0.01200}} "
                    "orientation {w:1.0}} "
                    f"scale {{x:{max(xs) - min(xs):.5f} "
                    f"y:{max(ys) - min(ys):.5f} z:0.00800}} "
                    f"{color_proto(0.08, 0.64, 0.82, 0.10)}"
                )
        if self.home_world is not None:
            markers.extend(
                self._build_point_markers(
                    "tzcup_cleaning_home",
                    self.home_world,
                    "HOME / MISSION START",
                    (0.28, 0.64, 1.0, 1.0),
                )
            )
        return markers

    def _build_point_markers(
        self,
        namespace: str,
        point: Point2,
        label: str,
        color: tuple[float, float, float, float],
    ) -> list[str]:
        material = color_proto(*color)
        return [
            f"ns:{quote_proto(namespace)} id:1 action:ADD_MODIFY "
            "type:BOX visibility:GUI layer:5 "
            f"pose {{position {{x:{point.x:.5f} y:{point.y:.5f} z:0.012}} "
            "orientation {w:1.0}} scale {x:0.14 y:0.14 z:0.018} "
            f"{material}",
        ]

    def _enqueue(self, kind: str, markers: list[str]) -> None:
        if kind in self.pending_kinds:
            return
        try:
            self.request_queue.put_nowait((kind, markers))
            self.pending_kinds.add(kind)
        except queue.Full:
            self.dropped_requests += 1
            self.get_logger().warning(
                "Gazebo marker queue is full; "
                f"dropped_requests={self.dropped_requests}"
            )

    def _service_worker(self) -> None:
        while not self.worker_stop.is_set():
            try:
                kind, markers = self.request_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if kind.startswith("remove_target:"):
                success = bool(markers) and self._call_remove_service(markers[0])
                target_id = kind.split(":", 1)[1]
                for target in self.targets:
                    if target["id"] == target_id:
                        target["removed_from_scene"] = success
                        break
            else:
                success = all(self._call_marker_service(marker) for marker in markers)
            if kind == "clear":
                self.markers_cleared = success
                self.clear_queued = False
            elif kind == "static":
                self.static_markers_sent = success
                self.static_markers_queued = False
            self.pending_kinds.discard(kind)
            self.request_queue.task_done()

    def _call_marker_service(self, request: str) -> bool:
        try:
            completed = subprocess.run(
                [
                    self.gz_binary,
                    "service",
                    "--timeout",
                    str(max(self.service_timeout_ms, 1)),
                    "-s",
                    self.marker_service,
                    "--reqtype",
                    "gz.msgs.Marker",
                    "--reptype",
                    "gz.msgs.Empty",
                    "--req",
                    request,
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=max(2.0, self.service_timeout_ms / 1000.0 + 2.0),
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            self.get_logger().warning(f"Gazebo marker request failed: {error}")
            return False
        success = completed.returncode == 0 and not completed.stderr.strip()
        if not success:
            detail = (completed.stderr or completed.stdout).strip().replace("\n", " ")
            self.get_logger().warning(
                f"Gazebo MarkerManager is not ready: {detail[:240]}"
            )
        return success

    def _call_remove_service(self, model_name: str) -> bool:
        service = f"/world/{self.world_name}/remove"
        request = f"name:{quote_proto(model_name)} type:MODEL"
        try:
            completed = subprocess.run(
                [
                    self.gz_binary, "service", "--timeout",
                    str(max(self.service_timeout_ms, 1)), "-s", service,
                    "--reqtype", "gz.msgs.Entity",
                    "--reptype", "gz.msgs.Boolean", "--req", request,
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=max(2.0, self.service_timeout_ms / 1000.0 + 2.0),
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            self.get_logger().warning(f"Target scene removal failed: {error}")
            return False
        success = completed.returncode == 0 and "true" in completed.stdout.lower()
        if not success:
            detail = (completed.stderr or completed.stdout).strip().replace("\n", " ")
            self.get_logger().warning(
                f"Target scene removal rejected for {model_name}: {detail[:240]}"
            )
        return success


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CleaningVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
