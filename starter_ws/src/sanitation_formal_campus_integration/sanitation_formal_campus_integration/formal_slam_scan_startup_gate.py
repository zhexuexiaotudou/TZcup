"""Exit only after the canonical scan has delivered a usable physical frame."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .scan_startup_gate_core import has_usable_scan


class FormalSlamScanStartupGate(Node):
    """One-shot launch prerequisite; it neither publishes data nor controls motion."""

    def __init__(self) -> None:
        super().__init__("formal_slam_scan_startup_gate")
        self.declare_parameter("scan_topic", "/scan/navigation")
        self._completed = False
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )

    def _on_scan(self, message: LaserScan) -> None:
        if self._completed or not has_usable_scan(
            frame_id=message.header.frame_id, ranges=message.ranges
        ):
            return
        self._completed = True
        self.get_logger().info(
            "canonical scan ready; starting standard autostart SLAM lifecycle"
        )
        self.create_timer(0.001, self._complete)

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
