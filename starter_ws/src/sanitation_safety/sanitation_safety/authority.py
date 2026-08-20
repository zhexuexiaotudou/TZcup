"""Authoritative, fail-closed emergency-stop state publisher."""

from __future__ import annotations

from dataclasses import dataclass
import json
import time


@dataclass
class SafetyAuthorityState:
    """Latched safety state. Startup is stopped until an explicit clear request."""

    emergency_stopped: bool = True
    command_sequence: int = 0
    last_operator_command_monotonic: float | None = None

    def apply_operator_command(self, stopped: bool, now: float) -> None:
        self.emergency_stopped = bool(stopped)
        self.command_sequence += 1
        self.last_operator_command_monotonic = float(now)

    def snapshot(self, now: float) -> dict:
        return {
            "schema_version": 1,
            "emergency_stopped": self.emergency_stopped,
            "command_sequence": self.command_sequence,
            "operator_command_age_sec": (
                None
                if self.last_operator_command_monotonic is None
                else max(0.0, float(now) - self.last_operator_command_monotonic)
            ),
            "startup_fail_closed": True,
            "heartbeat_active": True,
        }


def main(args=None) -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String

    class SafetyAuthority(Node):
        def __init__(self) -> None:
            super().__init__("safety_authority")
            self.declare_parameter("heartbeat_period_sec", 0.1)
            self.declare_parameter("startup_emergency_stopped", True)
            self.state = SafetyAuthorityState(
                emergency_stopped=bool(
                    self.get_parameter("startup_emergency_stopped").value
                )
            )
            qos = QoSProfile(depth=1)
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.estop_publisher = self.create_publisher(
                Bool, "/emergency_stop", qos
            )
            self.state_publisher = self.create_publisher(
                String, "/safety/state", qos
            )
            self.create_subscription(
                Bool,
                "/safety/operator_estop_command",
                self._on_operator_command,
                20,
            )
            self.create_timer(
                float(self.get_parameter("heartbeat_period_sec").value),
                self._publish,
            )
            self._publish()

        def _on_operator_command(self, message: Bool) -> None:
            self.state.apply_operator_command(message.data, time.monotonic())
            self._publish()

        def _publish(self) -> None:
            now = time.monotonic()
            self.estop_publisher.publish(
                Bool(data=self.state.emergency_stopped)
            )
            self.state_publisher.publish(
                String(data=json.dumps(self.state.snapshot(now), sort_keys=True))
            )

    rclpy.init(args=args)
    node = SafetyAuthority()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.estop_publisher.publish(Bool(data=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
