"""Optional ROS 2 adapter for the local human visualization console."""

from __future__ import annotations

import json
import math
import struct
import threading
import time
import zlib
from typing import Any

from .state import VisualizationState


def _yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    return (
        struct.pack(">I", len(payload))
        + kind
        + payload
        + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
    )


def encode_image_png(message) -> bytes:
    """Encode common sensor_msgs/Image layouts using only the standard library."""

    width = int(message.width)
    height = int(message.height)
    encoding = str(message.encoding).lower()
    formats = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}
    if width <= 0 or height <= 0 or encoding not in formats:
        raise ValueError(f"unsupported image: {width}x{height} {encoding}")
    channels = formats[encoding]
    raw = bytes(message.data)
    step = int(message.step) or width * channels
    if len(raw) < step * height:
        raise ValueError("truncated sensor image")
    scanlines = []
    for y in range(height):
        row = raw[y * step : y * step + width * channels]
        if encoding == "rgb8":
            rgb = row
        elif encoding == "bgr8":
            converted = bytearray(width * 3)
            converted[0::3] = row[2::3]
            converted[1::3] = row[1::3]
            converted[2::3] = row[0::3]
            rgb = bytes(converted)
        elif encoding in {"rgba8", "bgra8"}:
            converted = bytearray(width * 3)
            if encoding == "rgba8":
                converted[0::3] = row[0::4]
                converted[1::3] = row[1::4]
                converted[2::3] = row[2::4]
            else:
                converted[0::3] = row[2::4]
                converted[1::3] = row[1::4]
                converted[2::3] = row[0::4]
            rgb = bytes(converted)
        else:
            converted = bytearray(width * 3)
            converted[0::3] = row
            converted[1::3] = row
            converted[2::3] = row
            rgb = bytes(converted)
        scanlines.append(b"\x00" + rgb)
    compressed = zlib.compress(b"".join(scanlines), level=1)
    signature = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return signature + _png_chunk(b"IHDR", ihdr) + _png_chunk(b"IDAT", compressed) + _png_chunk(b"IEND", b"")


class RosAdapter:
    """Run rclpy in a background thread and expose a safe dispatch boundary."""

    def __init__(self, state: VisualizationState, *, camera_topic: str = "/camera/color/image_raw"):
        self.state = state
        self.camera_topic = camera_topic
        self.node = None
        self.thread: threading.Thread | None = None
        self._rclpy = None
        self._bool_type = None
        self._estop_publisher = None
        self._external_shutdown_exception = RuntimeError
        self._last_image_at: dict[str, float] = {}

    def start(self) -> None:
        import rclpy
        from nav_msgs.msg import OccupancyGrid, Odometry, Path
        from rclpy.executors import ExternalShutdownException
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image, LaserScan
        from std_msgs.msg import Bool, Float32, String

        self._rclpy = rclpy
        self._external_shutdown_exception = ExternalShutdownException
        self._bool_type = Bool
        rclpy.init(args=[])
        node = rclpy.create_node("sanitation_hmi_adapter")
        self.node = node
        self._estop_publisher = node.create_publisher(
            Bool, "/safety/operator_estop_command", 10
        )
        node.create_timer(1.0, self._check_safety_interface)
        node.create_subscription(Clock, "/clock", lambda _msg: self.state.touch("clock"), 10)
        node.create_subscription(Odometry, "/odom", self._on_odom, 20)
        node.create_subscription(OccupancyGrid, "/map", self._on_map, 2)
        node.create_subscription(Path, "/coverage/current_path", lambda msg: self._on_path("planned_path", msg), 10)
        node.create_subscription(Path, "/plan", lambda msg: self._on_path("planned_path", msg), 10)
        node.create_subscription(Path, "/local_plan", lambda msg: self._on_path("local_path", msg), 10)
        node.create_subscription(Image, self.camera_topic, lambda msg: self._on_image("camera", msg), 2)
        node.create_subscription(Image, "/world_overview/image", lambda msg: self._on_image("gazebo_overview", msg), 2)
        node.create_subscription(LaserScan, "/scan", lambda _msg: self.state.touch("lidar"), 10)
        node.create_subscription(Bool, "/brush_enabled", self._on_brush, 10)
        node.create_subscription(Bool, "/emergency_stop", self._on_estop, 10)
        node.create_subscription(String, "/coverage/state", self._on_coverage_state, 10)
        node.create_subscription(String, "/spot_clean/state", self._on_spot_state, 10)
        node.create_subscription(String, "/product/health", self._on_product_health, 10)
        node.create_subscription(Float32, "/metrics/coverage_ratio", self._on_planned_ratio, 10)
        node.create_subscription(Float32, "/metrics/empirical_coverage_ratio", self._on_actual_ratio, 10)
        self._install_perception_subscriptions(node)
        self.state.add_event("connection", "ROS 数据适配器已启动", "等待真实话题进入界面", source="hmi")
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self) -> None:
        try:
            self._rclpy.spin(self.node)
        except self._external_shutdown_exception:
            pass

    def _check_safety_interface(self) -> None:
        if self._estop_publisher is None:
            self.state.touch("safety", error="safety_publisher_unavailable")
            return
        if self._estop_publisher.get_subscription_count() >= 1:
            self.state.touch("safety")
        else:
            self.state.touch("safety", error="no_external_safety_subscriber")

    def _install_perception_subscriptions(self, node) -> None:
        try:
            from sanitation_perception_interfaces.msg import CleaningEvent, GarbageTargetArray

            node.create_subscription(
                GarbageTargetArray,
                "/perception/garbage/targets",
                lambda msg: self._on_targets("perception", msg),
                10,
            )
            node.create_subscription(CleaningEvent, "/garbage/cleaning_events", self._on_cleaning_event, 10)
        except (ImportError, ModuleNotFoundError) as exc:
            self.state.touch("perception", error=f"message_package_unavailable:{exc.__class__.__name__}")

    def stop(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
        if self._rclpy is not None and self._rclpy.ok():
            self._rclpy.shutdown()
        if self.thread is not None:
            self.thread.join(timeout=2.0)

    def dispatch(self, dsl: dict[str, Any]) -> dict[str, Any]:
        intent = dsl.get("intent")
        if intent in {"emergency_stop", "clear_emergency_stop"}:
            if self._estop_publisher is None or self._bool_type is None:
                return {"accepted": False, "dispatched": False, "reason": "ros_safety_interface_unavailable"}
            message = self._bool_type()
            message.data = intent == "emergency_stop"
            self._estop_publisher.publish(message)
            self.state.add_event(
                "safety",
                "触发急停" if message.data else "请求解除急停",
                "命令已提交给 /safety/operator_estop_command 安全权威",
                severity="critical" if message.data else "warning",
                source="operator",
            )
            return {"accepted": True, "dispatched": True, "reason": None}
        if intent == "status":
            return {"accepted": True, "dispatched": False, "reason": "read_only_status_request"}
        return {
            "accepted": False,
            "dispatched": False,
            "reason": "safe_task_orchestrator_unavailable",
        }

    def _on_odom(self, message) -> None:
        pose = message.pose.pose
        twist = message.twist.twist.linear
        speed = math.hypot(float(twist.x), float(twist.y))
        self.state.update_vehicle(pose.position.x, pose.position.y, _yaw(pose.orientation), speed)

    def _on_map(self, message) -> None:
        info = message.info
        self.state.update_map(
            width=info.width,
            height=info.height,
            resolution=info.resolution,
            origin_x=info.origin.position.x,
            origin_y=info.origin.position.y,
            data=list(message.data),
        )

    def _on_path(self, name: str, message) -> None:
        self.state.set_path(name, [[pose.pose.position.x, pose.pose.position.y] for pose in message.poses])

    def _on_image(self, name: str, message) -> None:
        now = time.monotonic()
        if now - self._last_image_at.get(name, 0.0) < 0.5:
            return
        self._last_image_at[name] = now
        try:
            self.state.set_image(name, encode_image_png(message))
        except (ValueError, zlib.error) as exc:
            self.state.touch(name, error=str(exc))

    def _on_brush(self, message) -> None:
        self.state.brush_enabled = bool(message.data)
        self.state.touch("brush")

    def _on_estop(self, message) -> None:
        value = bool(message.data)
        changed = self.state.emergency_stop is not None and self.state.emergency_stop != value
        self.state.emergency_stop = value
        self.state.touch("safety")
        if changed:
            self.state.add_event(
                "safety",
                "急停已生效" if value else "急停已解除",
                "安全速度门状态发生变化",
                severity="critical" if value else "info",
                source="safety_gate",
            )

    def _on_coverage_state(self, message) -> None:
        value = message.data or "UNKNOWN"
        if value != self.state.coverage_state:
            self.state.add_event("mission", "清扫任务状态变化", value, source="coverage")
        self.state.coverage_state = value
        self.state.touch("coverage_state")

    def _on_spot_state(self, message) -> None:
        try:
            payload = json.loads(message.data)
            value = f"{payload.get('mode', 'UNKNOWN')}，队列 {payload.get('queued_target_count', '?')}"
        except (json.JSONDecodeError, TypeError):
            value = message.data or "UNKNOWN"
        self.state.spot_state = value
        self.state.touch("spot_state")

    def _on_product_health(self, message) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise TypeError("product health must be an object")
            self.state.set_product_health(payload)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.state.touch("product_health", error=f"invalid_health:{exc}")

    def _on_planned_ratio(self, message) -> None:
        self.state.coverage_metrics["planned_ratio"] = float(message.data)
        self.state.coverage_metrics["basis"] = "planned_or_unspecified_topic"
        self.state.touch("coverage_metrics")

    def _on_actual_ratio(self, message) -> None:
        self.state.coverage_metrics["actual_ratio"] = float(message.data)
        self.state.coverage_metrics["basis"] = "empirical_cleaning_footprint"
        self.state.touch("coverage_metrics")

    @staticmethod
    def _target_dict(target) -> dict[str, Any]:
        pose = target.map_pose.pose
        return {
            "uuid": str(target.uuid),
            "class_id": str(target.class_id),
            "confidence": float(target.confidence),
            "position": [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
            "yaw": _yaw(pose.orientation),
            "size": [float(target.size.x), float(target.size.y), float(target.size.z)],
        }

    def _on_targets(self, source: str, message) -> None:
        self.state.set_targets(source, [self._target_dict(target) for target in message.targets])

    def _on_cleaning_event(self, message) -> None:
        if message.result == "cleaned":
            self.state.cleaned_targets.add(str(message.target_uuid))
        self.state.add_event(
            "perception",
            "清扫目标状态更新",
            f"{message.target_uuid}: {message.result}",
            source="cleaning_event",
        )
