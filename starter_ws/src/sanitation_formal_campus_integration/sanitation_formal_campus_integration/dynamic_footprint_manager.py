"""Publish the conservative Nav2 footprint for the current mechanism state."""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point32, Polygon
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from .dynamic_footprint_core import load_footprints, profile_decision

RUNTIME_TEST_OVERRIDE_TOPIC = "/formal_vehicle/navigation/footprint_runtime_test_override"


class DynamicFootprintManager(Node):
    def __init__(self) -> None:
        super().__init__("formal_dynamic_footprint_manager")
        self.declare_parameter("motion_profile_file", "")
        self.declare_parameter("publish_period_sec", 0.2)
        # This endpoint is deliberately absent from a production graph.  It is
        # only for the ROS-only runtime gate and can select a footprint while
        # the physical base remains inhibited.  It never drives a joint or
        # actuator topic.
        self.declare_parameter("enable_runtime_test_override", False)
        path = Path(str(self.get_parameter("motion_profile_file").value))
        if not path.is_file():
            raise ValueError("motion_profile_file must identify the formal profile")
        self._footprints = load_footprints(path)
        self._joints: dict[str, float] = {}
        self._base_motion_inhibited = False
        self._profile = "arm_deployed"
        self._publish_sequence = 0
        self._runtime_test_override: tuple[str, str] | None = None
        self._runtime_test_override_enabled = bool(
            self.get_parameter("enable_runtime_test_override").value
        )
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._local = self.create_publisher(
            Polygon, "/local_costmap/footprint", qos
        )
        self._global = self.create_publisher(
            Polygon, "/global_costmap/footprint", qos
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
        if self._runtime_test_override_enabled:
            self.create_subscription(
                String,
                RUNTIME_TEST_OVERRIDE_TOPIC,
                self._on_runtime_test_override,
                10,
            )
        self.create_timer(float(self.get_parameter("publish_period_sec").value), self._publish)

    def _on_joints(self, message: JointState) -> None:
        self._joints.update(
            (name, float(position))
            for name, position in zip(message.name, message.position, strict=False)
        )

    def _on_base_inhibit(self, message: Bool) -> None:
        self._base_motion_inhibited = bool(message.data)
        if not self._base_motion_inhibited:
            # An override may never survive an authorization change.  The
            # regular fail-closed mechanism-state decision resumes at once.
            self._runtime_test_override = None

    def _on_runtime_test_override(self, message: String) -> None:
        """Accept a narrow, inhibited-only state request from the runtime gate."""
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning("ignored malformed runtime test override")
            return
        if not isinstance(payload, dict) or payload.get("schema_version") != 1:
            self.get_logger().warning("ignored invalid runtime test override schema")
            return
        operation = payload.get("operation")
        nonce = payload.get("nonce")
        if not isinstance(nonce, str) or not nonce:
            self.get_logger().warning("ignored runtime test override without nonce")
            return
        if operation == "clear":
            self._runtime_test_override = None
            return
        profile = payload.get("requested_profile")
        if operation != "set" or profile not in self._footprints:
            self.get_logger().warning("ignored invalid runtime test override request")
            return
        if not self._base_motion_inhibited:
            self.get_logger().error(
                "rejected runtime test override because base motion is not inhibited"
            )
            self._runtime_test_override = None
            return
        self._runtime_test_override = (str(profile), nonce)

    def _publish(self) -> None:
        production_profile, production_reason = profile_decision(
            self._joints, self._base_motion_inhibited
        )
        runtime_test_nonce: str | None = None
        if self._runtime_test_override is not None and self._base_motion_inhibited:
            self._profile, runtime_test_nonce = self._runtime_test_override
            reason = "runtime_test_override"
        else:
            self._profile, reason = production_profile, production_reason
        # This is intentionally not a navigation authorization.  A test
        # override changes only the requested footprint and always remains
        # under the separately consumed base-motion inhibit.
        motion_authorized = (
            self._profile != "arm_deployed"
            and not self._base_motion_inhibited
            and runtime_test_nonce is None
        )
        polygon = Polygon()
        polygon.points = [
            Point32(x=x, y=y, z=0.0) for x, y in self._footprints[self._profile]
        ]
        self._local.publish(polygon)
        self._global.publish(polygon)
        self._publish_sequence += 1
        status = String()
        status.data = json.dumps(
            {
                "schema_version": 1,
                "profile": self._profile,
                "requested_profile": self._profile,
                "navigation_allowed": motion_authorized,
                "motion_authorized": motion_authorized,
                "base_motion_inhibited": self._base_motion_inhibited,
                "reason": reason,
                "production_profile": production_profile,
                "production_reason": production_reason,
                "runtime_test_override_active": runtime_test_nonce is not None,
                "runtime_test_nonce": runtime_test_nonce,
                "publish_sequence": self._publish_sequence,
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
