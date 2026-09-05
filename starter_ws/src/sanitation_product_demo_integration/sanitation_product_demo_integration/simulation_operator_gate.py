"""Explicit operator arming gate with continuous fail-safe simulation commands."""

from __future__ import annotations


def main() -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String

    class SimulationOperatorGate(Node):
        def __init__(self) -> None:
            super().__init__("formal_product_demo_operator_gate")
            self.declare_parameter("operator_start_topic", "/product_demo/operator_start")
            self.declare_parameter(
                "mission_complete_topic", "/active_cleaning/mission_complete"
            )
            self.declare_parameter("publish_period_sec", 0.10)
            latched = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._main_power = self.create_publisher(
                Bool, "/formal_vehicle/simulation/command/main_power", 10
            )
            self._estop = self.create_publisher(
                Bool, "/formal_vehicle/simulation/command/emergency_stop", 10
            )
            self._estop_reset = self.create_publisher(
                Bool, "/formal_vehicle/simulation/command/emergency_stop_reset", 10
            )
            self._status = self.create_publisher(
                String, "/product_demo/operator_gate_status", latched
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("operator_start_topic").value),
                self._on_operator_start,
                latched,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("mission_complete_topic").value),
                self._on_mission_complete,
                latched,
            )
            self._armed = False
            self._reset_cycles_remaining = 0
            self._reason = "awaiting_explicit_operator_start"
            self.create_timer(
                float(self.get_parameter("publish_period_sec").value), self._publish
            )
            self._publish()

        def _on_operator_start(self, message: Bool) -> None:
            requested = bool(message.data)
            if requested and not self._armed:
                # Hold reset for several watchdog periods so the false
                # external request and safety-power permit are observed first.
                self._reset_cycles_remaining = 10
            self._armed = requested
            self._reason = (
                "operator_armed" if self._armed else "operator_requested_safe_stop"
            )
            self._publish()

        def _on_mission_complete(self, message: Bool) -> None:
            if message.data:
                self._armed = False
                self._reason = "mission_complete_safe_stop"
                self._publish()

        def _publish(self) -> None:
            self._main_power.publish(Bool(data=self._armed))
            self._estop.publish(Bool(data=not self._armed))
            reset_active = self._armed and self._reset_cycles_remaining > 0
            self._estop_reset.publish(Bool(data=reset_active))
            if reset_active:
                self._reset_cycles_remaining -= 1
            self._status.publish(
                String(
                    data=(
                        '{"armed":'
                        + str(self._armed).lower()
                        + ',"reason":"'
                        + self._reason
                        + '"}'
                    )
                )
            )

    rclpy.init()
    node = SimulationOperatorGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._armed = False
            node._reason = "operator_gate_shutdown"
            node._publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
