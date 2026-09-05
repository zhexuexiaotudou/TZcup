"""Publish the conservative Nav2 footprint for the current mechanism state."""

from __future__ import annotations

import json
from pathlib import Path

from geometry_msgs.msg import Point32, PolygonStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from .dynamic_footprint_core import load_footprints, select_profile


class DynamicFootprintManager(Node):
    def __init__(self) -> None:
        super().__init__("formal_dynamic_footprint_manager")
        self.declare_parameter("motion_profile_file", "")
        self.declare_parameter("publish_period_sec", 0.2)
        path = Path(str(self.get_parameter("motion_profile_file").value))
        if not path.is_file():
            raise ValueError("motion_profile_file must identify the formal profile")
        self._footprints = load_footprints(path)
        self._joints: dict[str, float] = {}
        self._base_motion_inhibited = False
        self._profile = "arm_deployed"
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._local = self.create_publisher(
            PolygonStamped, "/local_costmap/footprint", qos
        )
        self._global = self.create_publisher(
            PolygonStamped, "/global_costmap/footprint", qos
        )
        self._status = self.create_publisher(
            String, "/formal_vehicle/navigation/footprint_status", qos
        )
        self.create_subscription(JointState, "/joint_states", self._on_joints, 20)
        self.create_subscription(
            Bool,
            "/manipulation/base_motion_inhibited",
            self._on_base_inhibit,
            20,
        )
        self.create_timer(float(self.get_parameter("publish_period_sec").value), self._publish)

    def _on_joints(self, message: JointState) -> None:
        self._joints.update(
            (name, float(position))
            for name, position in zip(message.name, message.position, strict=False)
        )

    def _on_base_inhibit(self, message: Bool) -> None:
        self._base_motion_inhibited = bool(message.data)

    def _publish(self) -> None:
        self._profile = select_profile(self._joints, self._base_motion_inhibited)
        polygon = PolygonStamped()
        polygon.header.stamp = self.get_clock().now().to_msg()
        polygon.header.frame_id = "base_link"
        polygon.polygon.points = [
            Point32(x=x, y=y, z=0.0) for x, y in self._footprints[self._profile]
        ]
        self._local.publish(polygon)
        self._global.publish(polygon)
        status = String()
        status.data = json.dumps(
            {
                "schema_version": 1,
                "profile": self._profile,
                "navigation_allowed": self._profile != "arm_deployed",
                "base_motion_inhibited": self._base_motion_inhibited,
            },
            sort_keys=True,
        )
        self._status.publish(status)


def main() -> None:
    rclpy.init()
    node = DynamicFootprintManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
