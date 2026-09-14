#!/usr/bin/env python3
"""Publish a causally stabilized map->odom transform.

The node consumes only the global EKF's remapped raw TF output. It has no
ground-truth subscription and no future-looking TF lookup.
"""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from .map_odom_stabilizer_core import (
    CausalMapOdomStabilizer,
    Pose2D,
    StabilizerInputError,
)


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class MapOdomStabilizerNode(Node):
    def __init__(self) -> None:
        super().__init__("map_odom_stabilizer")
        self.declare_parameter(
            "input_tf_topic", "/localization/raw_map_odom"
        )
        self.declare_parameter("status_topic", "/localization/map_odom_stabilizer/status")
        self.declare_parameter("output_parent_frame", "map")
        self.declare_parameter("output_child_frame", "odom")
        self.declare_parameter("tau_sec", 1.5)
        self.declare_parameter("max_filter_dt_sec", 0.1)
        self.declare_parameter("max_gap_sec", 0.5)

        self.input_tf_topic = str(self.get_parameter("input_tf_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.output_parent_frame = str(
            self.get_parameter("output_parent_frame").value
        )
        self.output_child_frame = str(
            self.get_parameter("output_child_frame").value
        )
        self.filter = CausalMapOdomStabilizer(
            tau_sec=float(self.get_parameter("tau_sec").value),
            max_dt_sec=float(self.get_parameter("max_filter_dt_sec").value),
            max_gap_sec=float(self.get_parameter("max_gap_sec").value),
        )
        self.ignored_transform_count = 0
        self.published_message_count = 0
        self.last_callback_stamp_sec: float | None = None
        self.tf_publisher = self.create_publisher(TFMessage, "/tf", 100)
        self.status_publisher = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(
            TFMessage,
            self.input_tf_topic,
            self._on_raw_tf,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"causal map->odom stabilizer active: input={self.input_tf_topic} "
            f"tau={self.filter.tau_sec}s max_dt={self.filter.max_dt_sec}s"
        )

    def _on_raw_tf(self, message: TFMessage) -> None:
        matches = []
        for transform in message.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if (
                parent == self.output_parent_frame
                and child == self.output_child_frame
            ):
                matches.append(transform)
            else:
                self.ignored_transform_count += 1
        if not matches:
            return
        if len(matches) != 1:
            self._block_without_update("multiple_map_odom_transforms_in_message")
            return
        transform = matches[0]
        stamp = transform.header.stamp
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        sample = Pose2D(
            stamp_sec=float(stamp_sec),
            x_m=float(translation.x),
            y_m=float(translation.y),
            yaw_rad=_yaw_from_quaternion(
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            ),
        )
        try:
            output = self.filter.update(sample)
        except StabilizerInputError as error:
            self._publish_status(reason=str(error))
            return
        output_message = TFMessage()
        output_transform = TransformStamped()
        output_transform.header.stamp = stamp
        output_transform.header.frame_id = self.output_parent_frame
        output_transform.child_frame_id = self.output_child_frame
        output_transform.transform.translation.x = output.x_m
        output_transform.transform.translation.y = output.y_m
        output_transform.transform.translation.z = 0.0
        qx, qy, qz, qw = _quaternion_from_yaw(output.yaw_rad)
        output_transform.transform.rotation.x = qx
        output_transform.transform.rotation.y = qy
        output_transform.transform.rotation.z = qz
        output_transform.transform.rotation.w = qw
        output_message.transforms.append(output_transform)
        self.tf_publisher.publish(output_message)
        self.published_message_count += 1
        self.last_callback_stamp_sec = output.stamp_sec

    def _block_without_update(self, reason: str) -> None:
        try:
            self.filter.block(reason)
        except StabilizerInputError:
            pass
        self._publish_status(reason=reason)

    def _publish_status(self, reason: str | None = None) -> None:
        status = "BLOCKED" if self.filter.blocked_reason else (
            "READY" if self.filter.accepted_updates else "WAITING"
        )
        payload = {
            "schema_version": 1,
            "status": status,
            "blocked_reason": self.filter.blocked_reason or reason,
            "accepted_updates": self.filter.accepted_updates,
            "duplicate_updates": self.filter.duplicate_updates,
            "rejected_updates": self.filter.rejected_updates,
            "published_tf_messages": self.published_message_count,
            "ignored_transform_count": self.ignored_transform_count,
            "max_observed_gap_sec": self.filter.max_observed_gap_sec,
            "last_source_stamp_sec": (
                None
                if self.filter.last_input is None
                else self.filter.last_input.stamp_sec
            ),
            "last_output_stamp_sec": self.last_callback_stamp_sec,
            "tau_sec": self.filter.tau_sec,
            "max_filter_dt_sec": self.filter.max_dt_sec,
            "max_gap_sec": self.filter.max_gap_sec,
            "input_tf_topic": self.input_tf_topic,
            "uses_ground_truth": False,
            "uses_future": False,
        }
        message = String()
        message.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self.status_publisher.publish(message)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = MapOdomStabilizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
