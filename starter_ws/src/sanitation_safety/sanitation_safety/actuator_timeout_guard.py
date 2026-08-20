"""Independent final-command timeout guard that can only publish a zero Twist."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time


@dataclass
class ActuatorTimeoutState:
    timeout_sec: float = 0.080
    zero_epsilon: float = 1.0e-9
    last_nonzero_monotonic: float | None = None
    zero_confirmed: bool = True

    def __post_init__(self) -> None:
        if not math.isfinite(self.timeout_sec) or self.timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be finite and positive")
        if not math.isfinite(self.zero_epsilon) or self.zero_epsilon < 0.0:
            raise ValueError("zero_epsilon must be finite and nonnegative")

    def observe(self, components, now: float) -> bool:
        """Record a final actuator command; return true for malformed input."""
        values = tuple(float(value) for value in components)
        if not values or not all(math.isfinite(value) for value in values):
            self.zero_confirmed = False
            self.last_nonzero_monotonic = None
            return True
        if any(abs(value) > self.zero_epsilon for value in values):
            self.last_nonzero_monotonic = float(now)
            self.zero_confirmed = False
        else:
            self.zero_confirmed = True
            self.last_nonzero_monotonic = None
        return False

    def zero_required(self, now: float) -> bool:
        if self.zero_confirmed:
            return False
        if self.last_nonzero_monotonic is None:
            return True
        return float(now) - self.last_nonzero_monotonic >= self.timeout_sec

    def mark_zero_published(self) -> None:
        self.zero_confirmed = True
        self.last_nonzero_monotonic = None


def main(args=None) -> None:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node

    class ActuatorTimeoutGuard(Node):
        def __init__(self) -> None:
            super().__init__("actuator_timeout_guard")
            self.declare_parameter("input_topic", "/cmd_vel")
            self.declare_parameter("output_topic", "/cmd_vel")
            self.declare_parameter("forward_commands", False)
            self.declare_parameter("timeout_sec", 0.080)
            self.declare_parameter("check_period_sec", 0.010)
            self.state = ActuatorTimeoutState(
                timeout_sec=float(self.get_parameter("timeout_sec").value)
            )
            check_period_sec = float(
                self.get_parameter("check_period_sec").value
            )
            if not math.isfinite(check_period_sec) or check_period_sec <= 0.0:
                raise ValueError("check_period_sec must be finite and positive")
            input_topic = str(self.get_parameter("input_topic").value)
            output_topic = str(self.get_parameter("output_topic").value)
            self.forward_commands = bool(
                self.get_parameter("forward_commands").value
            )
            if self.forward_commands and input_topic == output_topic:
                raise ValueError(
                    "forwarding guard requires distinct input and output topics"
                )
            self.publisher = self.create_publisher(Twist, output_topic, 10)
            self.create_subscription(Twist, input_topic, self._on_command, 50)
            self.create_timer(check_period_sec, self._on_timer)
            self._publish_zero()

        @staticmethod
        def _components(message: Twist) -> tuple[float, ...]:
            return (
                message.linear.x,
                message.linear.y,
                message.linear.z,
                message.angular.x,
                message.angular.y,
                message.angular.z,
            )

        def _on_command(self, message: Twist) -> None:
            malformed = self.state.observe(
                self._components(message), time.monotonic()
            )
            if malformed:
                self._publish_zero()
            elif self.forward_commands:
                self.publisher.publish(message)

        def _on_timer(self) -> None:
            if self.state.zero_required(time.monotonic()):
                self._publish_zero()

        def _publish_zero(self) -> None:
            self.publisher.publish(Twist())
            self.state.mark_zero_published()

    rclpy.init(args=args)
    node = ActuatorTimeoutGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            try:
                node.publisher.publish(Twist())
            except RuntimeError:
                # The context can become invalid between the readiness check
                # and the final best-effort zero publish during graph teardown.
                pass
        try:
            node.destroy_node()
        except RuntimeError:
            if rclpy.ok():
                raise
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
