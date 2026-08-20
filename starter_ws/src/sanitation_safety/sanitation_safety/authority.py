"""Authoritative, fail-closed emergency-stop state publisher."""

from __future__ import annotations

from dataclasses import dataclass
import json
import time


@dataclass
class SafetyAuthorityState:
    """Latched safety state. Startup is stopped until an explicit clear request."""

    emergency_stopped: bool = True
    require_supervisor_heartbeat: bool = False
    supervisor_heartbeat_timeout_sec: float = 0.5
    command_sequence: int = 0
    last_operator_command_monotonic: float | None = None
    last_operator_command_accepted: bool | None = None
    last_supervisor_monotonic: float | None = None
    supervisor_motion_healthy: bool = False
    supervisor_motion_faults: tuple[str, ...] = ()
    supervisor_trip_latched: bool = False

    def supervisor_fresh(self, now: float) -> bool:
        return (
            self.last_supervisor_monotonic is not None
            and float(now) - self.last_supervisor_monotonic
            <= self.supervisor_heartbeat_timeout_sec
        )

    def evaluate(self, now: float) -> bool:
        if self.require_supervisor_heartbeat and (
            not self.supervisor_fresh(now) or not self.supervisor_motion_healthy
        ):
            self.supervisor_trip_latched = True
            self.emergency_stopped = True
        return self.emergency_stopped

    def apply_operator_command(self, stopped: bool, now: float) -> bool:
        self.command_sequence += 1
        self.last_operator_command_monotonic = float(now)
        if stopped:
            self.emergency_stopped = True
            self.last_operator_command_accepted = True
            return True
        supervisor_clear_allowed = (
            not self.require_supervisor_heartbeat
            or (
                self.supervisor_fresh(now)
                and self.supervisor_motion_healthy
            )
        )
        self.last_operator_command_accepted = supervisor_clear_allowed
        if supervisor_clear_allowed:
            self.supervisor_trip_latched = False
            self.emergency_stopped = False
        else:
            self.emergency_stopped = True
        return supervisor_clear_allowed

    def apply_supervisor_report(
        self,
        *,
        motion_healthy: bool,
        motion_faults: list[str] | tuple[str, ...],
        now: float,
    ) -> None:
        self.last_supervisor_monotonic = float(now)
        self.supervisor_motion_healthy = bool(motion_healthy)
        self.supervisor_motion_faults = tuple(str(item) for item in motion_faults)
        if not self.supervisor_motion_healthy:
            self.supervisor_trip_latched = True
            self.emergency_stopped = True

    def snapshot(self, now: float) -> dict:
        self.evaluate(now)
        return {
            "schema_version": 1,
            "emergency_stopped": self.emergency_stopped,
            "command_sequence": self.command_sequence,
            "last_operator_command_accepted": self.last_operator_command_accepted,
            "operator_command_age_sec": (
                None
                if self.last_operator_command_monotonic is None
                else max(0.0, float(now) - self.last_operator_command_monotonic)
            ),
            "startup_fail_closed": True,
            "heartbeat_active": True,
            "require_supervisor_heartbeat": self.require_supervisor_heartbeat,
            "supervisor_heartbeat_fresh": self.supervisor_fresh(now),
            "supervisor_motion_healthy": self.supervisor_motion_healthy,
            "supervisor_motion_faults": list(self.supervisor_motion_faults),
            "supervisor_trip_latched": self.supervisor_trip_latched,
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
            self.declare_parameter("require_supervisor_heartbeat", False)
            self.declare_parameter("supervisor_heartbeat_timeout_sec", 0.5)
            self.state = SafetyAuthorityState(
                emergency_stopped=bool(
                    self.get_parameter("startup_emergency_stopped").value
                ),
                require_supervisor_heartbeat=bool(
                    self.get_parameter("require_supervisor_heartbeat").value
                ),
                supervisor_heartbeat_timeout_sec=float(
                    self.get_parameter("supervisor_heartbeat_timeout_sec").value
                ),
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
            self.create_subscription(
                String,
                "/safety/supervisor_health",
                self._on_supervisor_health,
                20,
            )
            self.create_timer(
                float(self.get_parameter("heartbeat_period_sec").value),
                self._publish,
            )
            self._publish()

        def _on_operator_command(self, message: Bool) -> None:
            accepted = self.state.apply_operator_command(
                message.data, time.monotonic()
            )
            if not accepted:
                self.get_logger().warning(
                    "E-stop clear rejected: product motion health is not ready"
                )
            self._publish()

        def _on_supervisor_health(self, message: String) -> None:
            try:
                payload = json.loads(message.data)
                motion_healthy = payload["motion_healthy"] is True
                faults = payload.get("motion_faults", [])
                if not isinstance(faults, list):
                    raise TypeError("motion_faults must be a list")
            except (KeyError, TypeError, ValueError):
                motion_healthy = False
                faults = ["supervisor_report_invalid"]
            self.state.apply_supervisor_report(
                motion_healthy=motion_healthy,
                motion_faults=faults,
                now=time.monotonic(),
            )
            self._publish()

        def _publish(self) -> None:
            now = time.monotonic()
            self.state.evaluate(now)
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
