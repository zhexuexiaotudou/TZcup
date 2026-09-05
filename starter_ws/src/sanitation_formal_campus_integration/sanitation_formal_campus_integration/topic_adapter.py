"""Republish formal sensor topics under the autonomy-stack compatibility names."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, NavSatFix, PointCloud2


class FormalLegacyTopicAdapter(Node):
    """Type-preserving adapter that leaves message contents and frames unchanged."""

    def __init__(self) -> None:
        super().__init__("formal_legacy_topic_adapter")
        self._relay_sensor(
            LaserScan, "/sensors/lidar_2d/scan", "/scan"
        )
        self._relay_sensor(Imu, "/sensors/imu/data", "/imu/data")
        self._relay_sensor(NavSatFix, "/sensors/gnss/fix", "/gnss/fix")
        self._relay_sensor(
            Image,
            "/sensors/front_rgbd/depth/image_rect_raw/image",
            "/camera/color/image_raw",
        )
        self._relay_sensor(
            Image,
            "/sensors/front_rgbd/depth/image_rect_raw/depth_image",
            "/camera/depth/image_rect_raw",
        )
        self._relay_sensor(
            PointCloud2,
            "/sensors/front_rgbd/depth/image_rect_raw/points",
            "/camera/depth/color/points",
        )
        self._relay_sensor(
            CameraInfo,
            "/sensors/front_rgbd/depth/image_rect_raw/camera_info",
            "/camera/color/camera_info",
        )

    def _relay_sensor(  # type: ignore[no-untyped-def]
        self, message_type, source: str, destination: str
    ) -> None:
        publisher = self.create_publisher(
            message_type, destination, qos_profile_sensor_data
        )
        self.create_subscription(
            message_type,
            source,
            lambda message, output=publisher: output.publish(message),
            qos_profile_sensor_data,
        )

def main() -> None:
    rclpy.init()
    node = FormalLegacyTopicAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
