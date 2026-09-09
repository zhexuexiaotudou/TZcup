"""Exit only after the canonical scan has delivered a usable physical frame."""

from __future__ import annotations

import math

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from .scan_startup_gate_core import has_usable_scan, scan_rejection_reason


class FormalSlamScanStartupGate(Node):
    """One-shot launch prerequisite; it neither publishes data nor controls motion."""

    def __init__(self) -> None:
        super().__init__("formal_slam_scan_startup_gate")
        self.declare_parameter("scan_topic", "/scan/navigation")
        self._scan_topic = str(self.get_parameter("scan_topic").value)
        self._completed = False
        self._received_count = 0
        self._last_frame_id = ""
        self._last_ray_count = 0
        self._last_non_nan_count = 0
        self._last_rejection_reason = "no_callbacks"
        self._completion_timer = None
        self._scan_subscription = self.create_subscription(
            LaserScan,
            self._scan_topic,
            self._on_scan,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._diagnostic_timer = self.create_timer(
            5.0,
            self._report_health,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def _on_scan(self, message: LaserScan) -> None:
        self._received_count += 1
        self._last_frame_id = message.header.frame_id
        self._last_ray_count = len(message.ranges)
        self._last_non_nan_count = sum(
            not math.isnan(float(value)) for value in message.ranges
        )
        self._last_rejection_reason = scan_rejection_reason(
            frame_id=message.header.frame_id, ranges=message.ranges
        )
        if self._completed or not has_usable_scan(
            frame_id=message.header.frame_id, ranges=message.ranges
        ):
            return
        self._completed = True
        self.get_logger().info(
            "canonical scan ready; starting standard autostart SLAM lifecycle"
        )
        self._completion_timer = self.create_timer(0.001, self._complete)

    def _report_health(self) -> None:
        if self._completed:
            return
        self.get_logger().info(
            "canonical scan startup gate waiting: "
            f"received={self._received_count} "
            f"publishers={self.count_publishers(self._scan_topic)} "
            f"reason={self._last_rejection_reason} "
            f"frame_id={self._last_frame_id!r} "
            f"rays={self._last_ray_count} "
            f"non_nan_rays={self._last_non_nan_count}"
        )

    @staticmethod
    def _complete() -> None:
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = FormalSlamScanStartupGate()
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
