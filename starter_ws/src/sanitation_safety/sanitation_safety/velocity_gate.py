import time
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


@dataclass
class VelocityGateState:
    emergency_stopped: bool = False
    command_timeout_sec: float = 0.5
    last_command_monotonic: float | None = None
    max_linear_velocity: float = 0.45
    max_angular_velocity: float = 0.35

    def output(self, linear_x: float, angular_z: float, now: float):
        timed_out = (
            self.last_command_monotonic is None
            or now - self.last_command_monotonic > self.command_timeout_sec
        )
        if self.emergency_stopped or timed_out:
            return 0.0, 0.0
        return (
            max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_x)),
            max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_z)),
        )


class VelocityGate(Node):
    def __init__(self) -> None:
        super().__init__("velocity_gate")
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("input_topic", "/cmd_vel_gate")
        self.declare_parameter("profile_name", "localization_coverage")
        self.declare_parameter("max_linear_velocity", 0.45)
        self.declare_parameter("max_angular_velocity", 0.35)
        timeout = float(self.get_parameter("command_timeout_sec").value)
        self.state = VelocityGateState(
            command_timeout_sec=timeout,
            max_linear_velocity=float(
                self.get_parameter("max_linear_velocity").value
            ),
            max_angular_velocity=float(
                self.get_parameter("max_angular_velocity").value
            ),
        )
        self.last_command = Twist()
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            Twist, str(self.get_parameter("input_topic").value), self._on_command, 10
        )
        self.create_subscription(Bool, "/emergency_stop", self._on_estop, 10)
        self.timer = self.create_timer(0.05, self._publish)

    def _on_command(self, message: Twist) -> None:
        self.last_command = message
        self.state.last_command_monotonic = time.monotonic()

    def _on_estop(self, message: Bool) -> None:
        self.state.emergency_stopped = bool(message.data)
        self._publish()

    def _publish(self) -> None:
        linear_x, angular_z = self.state.output(
            self.last_command.linear.x,
            self.last_command.angular.z,
            time.monotonic(),
        )
        output = Twist()
        output.linear.x = linear_x
        output.angular.z = angular_z
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocityGate()
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
