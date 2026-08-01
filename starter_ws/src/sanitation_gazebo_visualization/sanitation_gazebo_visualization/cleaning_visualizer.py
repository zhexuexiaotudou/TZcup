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

from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String


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

        self.gz_binary = shutil.which("gz")
        if not self.gz_binary:
            raise RuntimeError("Gazebo command-line client 'gz' is unavailable")

        self.map_x = 0.0
        self.map_y = 0.0
        self.map_yaw = 0.0
        self.have_pose = False
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
        self.last_status_update = 0.0
        self.dropped_requests = 0
        self.pending_kinds: set[str] = set()

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
        self.create_timer(0.1, self._update_markers)
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

    def _on_truth(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.map_x = float(pose.position.x)
        self.map_y = float(pose.position.y)
        self.map_yaw = yaw_from_quaternion(pose.orientation)
        self.have_pose = True

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
        if self.coverage_state == "COMPLETED":
            self.completed_components = self.expected_components

    def _on_component(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        self.coverage_state = str(payload.get("state", self.coverage_state))
        if self.coverage_state in {"EXECUTING_SWATH", "EXECUTING_TURN"}:
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

    def _on_path(self, message: Path) -> None:
        if len(message.poses) < 2 or not self.markers_cleared:
            return
        points = []
        for stamped_pose in message.poses:
            point = self.map_to_world(
                stamped_pose.pose.position.x, stamped_pose.pose.position.y
            )
            points.append(f"point {{x:{point.x:.5f} y:{point.y:.5f} z:0.055}}")
        marker = (
            "ns:\"tzcup_current_cleaning_path\" id:0 "
            "action:ADD_MODIFY type:LINE_STRIP visibility:GUI layer:3 "
            "scale {x:0.065 y:0.065 z:0.065} "
            f"{color_proto(1.0, 0.72, 0.08, 0.95)} "
            + " ".join(points)
        )
        self._enqueue("path", [marker])

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
                        )
                    ],
                )
            return

        now = time.monotonic()
        status_due = now - self.last_status_update >= 3.0
        trail_marker = self._build_trail_marker()
        if trail_marker or status_due:
            markers = []
            if trail_marker:
                markers.append(trail_marker)
            if status_due:
                markers.append(self._build_status_marker())
                self.last_status_update = now
            self._enqueue("update", markers)

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

    def _build_status_marker(self) -> str:
        world_pose = self.map_to_world(self.map_x, self.map_y)
        text = (
            f"{self.coverage_state} | {self.completed_components}/"
            f"{self.expected_components} | BRUSH "
            f"{'ON' if self.brush_enabled else 'OFF'}"
        )
        if self.coverage_state == "COMPLETED":
            color = color_proto(0.16, 1.0, 0.38, 1.0)
        elif self.coverage_state == "FAILED":
            color = color_proto(1.0, 0.18, 0.12, 1.0)
        else:
            color = color_proto(1.0, 0.90, 0.28, 1.0)
        return (
            "ns:\"tzcup_cleaning_status\" id:0 action:ADD_MODIFY "
            "type:TEXT visibility:GUI layer:5 "
            f"pose {{position {{x:{world_pose.x:.5f} y:{world_pose.y:.5f} z:2.2}} "
            "orientation {w:1.0}} scale {x:0.32 y:0.32 z:0.32} "
            f"text:{quote_proto(text)} {color}"
        )

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
            success = all(self._call_marker_service(marker) for marker in markers)
            if kind == "clear":
                self.markers_cleared = success
                self.clear_queued = False
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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CleaningVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
