#!/usr/bin/env python3
"""Publish one transient-local formal description, with bounded discovery retry."""

from __future__ import annotations

import os

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class FormalRobotDescriptionPublisher(Node):
    """Provide one reliable transient-local writer without unbounded URDF spam."""

    def __init__(self) -> None:
        super().__init__("formal_robot_description_publisher")
        self.declare_parameter("robot_description", "")
        description = str(self.get_parameter("robot_description").value)
        if not description.startswith("<?xml") or "tzcup_formal_sanitation_vehicle" not in description:
            raise ValueError("robot_description is missing or is not the formal vehicle")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._description_message = String(data=description)
        self._description_publisher = self.create_publisher(
            String, "/robot_description", qos
        )
        self._matched = False
        self._publish_count = 0
        self._publish()
        self._timer = self.create_timer(0.25, self._publish_until_matched)
        self.get_logger().info(
            "formal description writer started: domain=%s rmw=%s bytes=%d"
            % (
                os.environ.get("ROS_DOMAIN_ID", "0"),
                os.environ.get("RMW_IMPLEMENTATION", "default"),
                len(description.encode("utf-8")),
            )
        )

    def _publish(self) -> None:
        self._description_publisher.publish(self._description_message)
        self._publish_count += 1

    def _publish_until_matched(self) -> None:
        """Retry only until one consumer is discovered.

        The cached TRANSIENT_LOCAL sample serves every later subscriber.  An
        unbounded 234 kB publish once per second forced the embedded controller
        manager to deserialize and reject an already-loaded URDF forever,
        starving physics and rendering on the full vehicle.
        """
        subscription_count = self._description_publisher.get_subscription_count()
        if subscription_count > 0:
            if not self._matched:
                self._matched = True
                self.get_logger().info(
                    "robot_description matched %d subscription(s) after %d publish(es)"
                    % (subscription_count, self._publish_count)
                )
            self._timer.cancel()
            return
        self._publish()
        if self._publish_count % 20 == 0:
            self.get_logger().info(
                "robot_description still waiting for a subscription after %d publish(es)"
                % self._publish_count
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FormalRobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok(context=node.context):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
