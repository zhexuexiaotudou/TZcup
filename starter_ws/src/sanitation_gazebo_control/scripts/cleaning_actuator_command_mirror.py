#!/usr/bin/env python3
"""Mirror post-safety cleaning references into the read-only Gazebo motor model."""

from __future__ import annotations

import math
import time

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray


class CleaningActuatorCommandMirror(Node):
    """Publish only observed, post-gate controller references; never drive joints."""

    def __init__(self) -> None:
        super().__init__("cleaning_actuator_command_mirror")
        self.declare_parameter("input_timeout_s", 0.20)
        self.declare_parameter("publish_period_s", 0.02)
        timeout = float(self.get_parameter("input_timeout_s").value)
        period = float(self.get_parameter("publish_period_s").value)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("input_timeout_s must be finite and positive")
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError("publish_period_s must be finite and positive")
        self._timeout = timeout
        self._brush = [0.0, 0.0, 0.0]
        self._pump = [0.0]
        self._lift_reference = 0.0
        self._lift_position = 0.0
        self._enabled = False
        self._brush_stamp = None
        self._pump_stamp = None
        self._lift_stamp = None
        self._enable_stamp = None

        root = "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/command"
        self._brush_pub = self.create_publisher(
            Float64MultiArray, root + "/brush", 10
        )
        self._pump_pub = self.create_publisher(
            Float64MultiArray, root + "/pump", 10
        )
        self._lift_pub = self.create_publisher(Float64, root + "/lift_position", 10)
        self._enable_pub = self.create_publisher(Bool, root + "/enable", 10)
        self.create_subscription(
            Float64MultiArray,
            "/brush_controller/commands",
            self._on_brush,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/recovery_controller/commands",
            self._on_pump,
            10,
        )
        self.create_subscription(
            JointTrajectoryControllerState,
            "/cleaning_controller/controller_state",
            self._on_lift_state,
            10,
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self.create_subscription(
            Bool, "/safety/actuators_enabled", self._on_enable, 10
        )
        self.create_timer(period, self._publish)

    def _on_brush(self, message: Float64MultiArray) -> None:
        values = [float(value) for value in message.data]
        if len(values) == 3 and all(math.isfinite(value) for value in values):
            self._brush = values
            self._brush_stamp = time.monotonic()

    def _on_pump(self, message: Float64MultiArray) -> None:
        values = [float(value) for value in message.data]
        if len(values) == 1 and math.isfinite(values[0]):
            self._pump = values
            self._pump_stamp = time.monotonic()

    def _on_lift_state(self, message: JointTrajectoryControllerState) -> None:
        if message.reference.positions:
            value = float(message.reference.positions[0])
            if math.isfinite(value):
                self._lift_reference = value
                self._lift_stamp = time.monotonic()

    def _on_joint_state(self, message: JointState) -> None:
        for name, position in zip(message.name, message.position):
            if name == "cleaning_lift_joint" and math.isfinite(position):
                self._lift_position = float(position)
                break

    def _on_enable(self, message: Bool) -> None:
        self._enabled = bool(message.data)
        self._enable_stamp = time.monotonic()

    def _fresh(self, stamp: float | None, now: float) -> bool:
        return stamp is not None and 0.0 <= now - stamp <= self._timeout

    def _publish(self) -> None:
        now = time.monotonic()
        enabled = self._enabled and self._fresh(self._enable_stamp, now)
        brush = self._brush if enabled and self._fresh(self._brush_stamp, now) else [0.0] * 3
        pump = self._pump if enabled and self._fresh(self._pump_stamp, now) else [0.0]
        lift = (
            self._lift_reference
            if enabled and self._fresh(self._lift_stamp, now)
            else self._lift_position
        )
        self._brush_pub.publish(Float64MultiArray(data=brush))
        self._pump_pub.publish(Float64MultiArray(data=pump))
        self._lift_pub.publish(Float64(data=lift))
        self._enable_pub.publish(Bool(data=enabled))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CleaningActuatorCommandMirror()
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
