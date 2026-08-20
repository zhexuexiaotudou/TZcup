import math
import time
from dataclasses import dataclass

@dataclass
class VelocityGateState:
    emergency_stopped: bool = True
    command_timeout_sec: float = 0.5
    last_command_monotonic: float | None = None
    estop_heartbeat_timeout_sec: float = 0.5
    last_estop_monotonic: float | None = None
    max_linear_velocity: float = 0.45
    max_angular_velocity: float = 0.35

    def __post_init__(self) -> None:
        for name, value in (
            ("command_timeout_sec", self.command_timeout_sec),
            ("estop_heartbeat_timeout_sec", self.estop_heartbeat_timeout_sec),
            ("max_linear_velocity", self.max_linear_velocity),
            ("max_angular_velocity", self.max_angular_velocity),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")

    def observe_estop(self, stopped: bool, now: float) -> None:
        """Update authority state and require a fresh command after every stop."""
        was_stopped = self.emergency_stopped
        self.emergency_stopped = bool(stopped)
        self.last_estop_monotonic = float(now)
        if self.emergency_stopped or was_stopped:
            # Commands received before or during a stop are not valid motion
            # authority after the stop is cleared.
            self.last_command_monotonic = None

    def output(self, linear_x: float, angular_z: float, now: float):
        command_timed_out = (
            self.last_command_monotonic is None
            or now - self.last_command_monotonic > self.command_timeout_sec
        )
        safety_timed_out = (
            self.last_estop_monotonic is None
            or now - self.last_estop_monotonic
            > self.estop_heartbeat_timeout_sec
        )
        command_is_finite = math.isfinite(linear_x) and math.isfinite(angular_z)
        if (
            self.emergency_stopped
            or command_timed_out
            or safety_timed_out
            or not command_is_finite
        ):
            return 0.0, 0.0
        return (
            max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_x)),
            max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_z)),
        )


def main(args=None) -> None:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import Bool

    class VelocityGate(Node):
        def __init__(self) -> None:
            super().__init__("velocity_gate")
            self.declare_parameter("command_timeout_sec", 0.5)
            self.declare_parameter("estop_heartbeat_timeout_sec", 0.5)
            self.declare_parameter("publish_period_sec", 0.05)
            self.declare_parameter("input_topic", "/cmd_vel_gate")
            self.declare_parameter("output_topic", "/cmd_vel")
            self.declare_parameter("profile_name", "localization_coverage")
            self.declare_parameter("max_linear_velocity", 0.45)
            self.declare_parameter("max_angular_velocity", 0.35)
            timeout = float(self.get_parameter("command_timeout_sec").value)
            self.state = VelocityGateState(
                command_timeout_sec=timeout,
                estop_heartbeat_timeout_sec=float(
                    self.get_parameter("estop_heartbeat_timeout_sec").value
                ),
                max_linear_velocity=float(
                    self.get_parameter("max_linear_velocity").value
                ),
                max_angular_velocity=float(
                    self.get_parameter("max_angular_velocity").value
                ),
            )
            self.last_command = Twist()
            self.publisher = self.create_publisher(
                Twist, str(self.get_parameter("output_topic").value), 10
            )
            self.create_subscription(
                Twist,
                str(self.get_parameter("input_topic").value),
                self._on_command,
                10,
            )
            self.create_subscription(Bool, "/emergency_stop", self._on_estop, 10)
            publish_period_sec = float(
                self.get_parameter("publish_period_sec").value
            )
            if not math.isfinite(publish_period_sec) or publish_period_sec <= 0.0:
                raise ValueError("publish_period_sec must be finite and positive")
            self.timer = self.create_timer(publish_period_sec, self._publish)

        def _on_command(self, message: Twist) -> None:
            self.last_command = message
            self.state.last_command_monotonic = time.monotonic()

        def _on_estop(self, message: Bool) -> None:
            self.state.observe_estop(bool(message.data), time.monotonic())
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

    rclpy.init(args=args)
    node = VelocityGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
