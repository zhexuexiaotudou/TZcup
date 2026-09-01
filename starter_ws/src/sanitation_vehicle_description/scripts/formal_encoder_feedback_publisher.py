#!/usr/bin/env python3
"""Publish hardware-shaped, count-quantized encoder feedback from joint state."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray, MultiArrayDimension

from formal_encoder_quantization import EncoderGroupQuantizer


A300_JOINTS = (
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
)
POLOLU_JOINTS = (
    "left_side_brush_joint",
    "right_side_brush_joint",
    "central_roller_joint",
)

# Clearpath does not publish the A300 encoder resolution in the selected public
# description/manual.  4096 is therefore a simulation quantizer, not a vendor
# specification.  Pololu publishes 64 counts per motor revolution; with the
# selected nominal 70:1 gearbox this is 4480 counts per output revolution.
A300_SIM_COUNTS_PER_WHEEL_REVOLUTION = 4096
POLOLU_COUNTS_PER_OUTPUT_REVOLUTION = 64 * 70


class FormalEncoderFeedbackPublisher(Node):
    def __init__(self) -> None:
        super().__init__("formal_encoder_feedback_publisher")
        self._a300_quantizer = EncoderGroupQuantizer(
            A300_JOINTS, A300_SIM_COUNTS_PER_WHEEL_REVOLUTION
        )
        self._pololu_quantizer = EncoderGroupQuantizer(
            POLOLU_JOINTS, POLOLU_COUNTS_PER_OUTPUT_REVOLUTION
        )
        self._a300_counts = self.create_publisher(
            Int64MultiArray, "/formal_vehicle/encoders/a300/counts", 20
        )
        self._a300_states = self.create_publisher(
            JointState, "/formal_vehicle/encoders/a300/joint_states", 20
        )
        self._pololu_counts = self.create_publisher(
            Int64MultiArray, "/formal_vehicle/encoders/cleaning/counts", 20
        )
        self._pololu_states = self.create_publisher(
            JointState, "/formal_vehicle/encoders/cleaning/joint_states", 20
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self._reported_missing: set[tuple[str, ...]] = set()

    @staticmethod
    def _stamp_ns(message: JointState) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )

    @staticmethod
    def _counts_message(names: tuple[str, ...], counts: tuple[int, ...]) -> Int64MultiArray:
        message = Int64MultiArray()
        message.layout.dim = [
            MultiArrayDimension(
                label="joint_order:" + ",".join(names),
                size=len(names),
                stride=len(names),
            )
        ]
        message.data = list(counts)
        return message

    @staticmethod
    def _state_message(
        source: JointState,
        names: tuple[str, ...],
        positions: tuple[float, ...],
        velocities: tuple[float, ...],
    ) -> JointState:
        message = JointState()
        message.header = source.header
        message.name = list(names)
        message.position = list(positions)
        message.velocity = list(velocities)
        # Effort is deliberately absent: this node measures encoder counts and
        # must not relabel simulated joint effort as encoder instrumentation.
        return message

    def _publish_group(
        self,
        source: JointState,
        positions: dict[str, float],
        quantizer: EncoderGroupQuantizer,
        counts_publisher,
        states_publisher,
    ) -> None:
        try:
            sample = quantizer.sample(self._stamp_ns(source), positions)
        except KeyError:
            missing = tuple(name for name in quantizer.joint_names if name not in positions)
            if missing not in self._reported_missing:
                self._reported_missing.add(missing)
                self.get_logger().warning(
                    "encoder feedback waiting for joint state: " + ", ".join(missing)
                )
            return
        counts_publisher.publish(self._counts_message(quantizer.joint_names, sample.counts))
        states_publisher.publish(
            self._state_message(
                source,
                quantizer.joint_names,
                sample.position_rad,
                sample.velocity_rad_s,
            )
        )

    def _on_joint_state(self, message: JointState) -> None:
        positions = {
            name: float(position)
            for name, position in zip(message.name, message.position, strict=False)
        }
        self._publish_group(
            message,
            positions,
            self._a300_quantizer,
            self._a300_counts,
            self._a300_states,
        )
        self._publish_group(
            message,
            positions,
            self._pololu_quantizer,
            self._pololu_counts,
            self._pololu_states,
        )


def main() -> None:
    rclpy.init()
    node = FormalEncoderFeedbackPublisher()
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
