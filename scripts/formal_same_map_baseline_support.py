#!/usr/bin/env python3
"""Evaluator-only pose adapter plus sustained public operator safety heartbeat."""

from __future__ import annotations

import argparse
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


class Support(Node):
    def __init__(self, entity: str, start_x: float, start_y: float, start_yaw: float) -> None:
        super().__init__("formal_same_map_baseline_support")
        self.entity = entity
        self.start_x, self.start_y, self.start_yaw = start_x, start_y, start_yaw
        self.truth = self.create_publisher(Odometry, "/ground_truth/odom", 20)
        self.estop = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/emergency_stop", 10
        )
        self.reset = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/emergency_stop_reset", 10
        )
        self.power = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/main_power", 10
        )
        self.create_subscription(
            TFMessage, "/evaluation/formal_same_map/dynamic_pose", self._pose, 20
        )
        self.create_timer(0.5, self._heartbeat)

    def _heartbeat(self) -> None:
        self.estop.publish(Bool(data=False))
        self.reset.publish(Bool(data=True))
        self.power.publish(Bool(data=True))

    def _pose(self, message: TFMessage) -> None:
        matches = [transform for transform in message.transforms if (
            transform.child_frame_id == self.entity
            or transform.child_frame_id.endswith("/" + self.entity)
        )]
        if len(matches) != 1:
            return
        transform = matches[0]
        source = transform.transform.translation
        orientation = transform.transform.rotation
        world_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        dx, dy = source.x - self.start_x, source.y - self.start_y
        c, s = math.cos(self.start_yaw), math.sin(self.start_yaw)
        local_x, local_y = c * dx + s * dy, -s * dx + c * dy
        local_yaw = world_yaw - self.start_yaw
        output = Odometry()
        output.header.stamp = transform.header.stamp
        output.header.frame_id = "map_gt"
        output.child_frame_id = "ground_truth/base_footprint"
        output.pose.pose.position.x = local_x
        output.pose.pose.position.y = local_y
        output.pose.pose.position.z = source.z
        output.pose.pose.orientation.z = math.sin(local_yaw / 2.0)
        output.pose.pose.orientation.w = math.cos(local_yaw / 2.0)
        self.truth.publish(output)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--entity", default="tzcup_formal_sanitation_vehicle")
    parser.add_argument("--start-x", type=float, required=True)
    parser.add_argument("--start-y", type=float, required=True)
    parser.add_argument("--start-yaw", type=float, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Support(args.entity, args.start_x, args.start_y, args.start_yaw)
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
