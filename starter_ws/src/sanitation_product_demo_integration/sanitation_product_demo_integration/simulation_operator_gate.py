"""Explicit operator arming with independent wall-clock control-owner leases."""
from __future__ import annotations
import json
import time
import uuid
from .operator_control_lease import OperatorControlLease


def node_class():
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String

    class SimulationOperatorGate(Node):
        def __init__(self):
            super().__init__("formal_product_demo_operator_gate")
            self.declare_parameter("operator_start_topic", "/product_demo/operator_start")
            self.declare_parameter("mission_complete_topic", "/active_cleaning/mission_complete")
            self.declare_parameter("publish_period_sec", 0.10)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            live = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
            self._main_power = self.create_publisher(
                Bool, "/formal_vehicle/simulation/command/main_power", 10)
            self._estop = self.create_publisher(
                Bool, "/formal_vehicle/simulation/command/emergency_stop", 10)
            self._estop_reset = self.create_publisher(
                Bool, "/formal_vehicle/simulation/command/emergency_stop_reset", 10)
            self._armed_publisher = self.create_publisher(
                Bool, "/product_demo/operator_armed", latched)
            self._status = self.create_publisher(String, "/product_demo/operator_gate_status", latched)
            # A restarted gate must not replay a retained operator-start command.
            self.create_subscription(Bool, str(self.get_parameter("operator_start_topic").value),
                                     self._on_operator_start, live)
            self.create_subscription(Bool, str(self.get_parameter("mission_complete_topic").value),
                                     self._on_mission_complete, latched)
            self.create_subscription(String, "/active_cleaning/control_health", self._on_planner_health, live)
            self.create_subscription(DiagnosticArray, "/active_cleaning/executor_status",
                                     self._on_executor_status, live)
            self.create_subscription(Bool, "/safety/actuators_enabled", self._on_safety_permit, latched)
            self._lease = OperatorControlLease()
            self._instance_id = uuid.uuid4().hex
            self._status_sequence = 0
            self._previous_power_requested = False
            self._reset_until = 0.0
            self._safety_permit = False
            self._safety_time = None
            self._permit_confirmed_for_current_lease = False
            self.create_timer(float(self.get_parameter("publish_period_sec").value),
                              self._publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self._publish()

        def _on_operator_start(self, message):
            self._lease.request(bool(message.data), time.monotonic())
            self._publish()

        def _on_mission_complete(self, message):
            if message.data:
                self._lease.complete()  # mission_complete_safe_stop
                self._publish()

        def _on_planner_health(self, message):
            try:
                payload = json.loads(message.data)
                instance = payload["instance_id"]
                healthy = payload["healthy"]
                published_at_monotonic_ns = payload["published_at_monotonic_ns"]
                sequence = payload["sequence"]
                if (not isinstance(instance, str) or not instance or type(healthy) is not bool
                        or isinstance(published_at_monotonic_ns, bool)
                        or not isinstance(published_at_monotonic_ns, int)
                        or isinstance(sequence, bool) or not isinstance(sequence, int)):
                    raise ValueError("invalid planner health")
            except (ValueError, TypeError, KeyError):
                self._lease.stop("invalid_planner_health_requires_new_operator_request")
                self._publish()
                return
            self._lease.observe(
                "planner", healthy, time.monotonic(), instance,
                published_at_monotonic_ns, sequence, time.monotonic_ns(),
            )
            self._publish()

        def _on_executor_status(self, message):
            rows = [row for row in message.status
                    if row.name == "formal_active_cleaning_trajectory_executor"]
            if not rows:
                return
            fields = {item.key: item.value for item in rows[0].values}
            healthy = (
                len(rows) == 1 and len(fields) == len(rows[0].values)
                and fields.get("restart_required") == "false"
                and rows[0].message in {
                    "IDLE", "BLOCKED", "SUBMITTING", "EXECUTING", "CANCELING",
                    "CANCEL_PENDING_ACCEPTANCE", "SUCCEEDED", "FAILED", "REJECTED", "CANCELED",
                }
            )
            instance = fields.get("instance_id", "")
            try:
                published_at_monotonic_ns = int(fields["published_at_monotonic_ns"])
                sequence = int(fields["sequence"])
            except (KeyError, TypeError, ValueError):
                self._lease.stop("invalid_executor_control_metadata_requires_new_operator_request")
            else:
                if not instance:
                    self._lease.stop("missing_executor_instance_requires_new_operator_request")
                else:
                    self._lease.observe(
                        "executor", healthy, time.monotonic(), instance,
                        published_at_monotonic_ns, sequence, time.monotonic_ns(),
                    )
            self._publish()

        def _on_safety_permit(self, message):
            self._safety_time = time.monotonic()
            self._safety_permit = bool(message.data)
            self._publish()

        def _publish(self):
            now = time.monotonic()
            self._lease.tick(now)
            requested = self._lease.armed
            permit_fresh_and_enabled = (
                self._safety_permit and self._safety_time is not None
                and 0.0 <= now - self._safety_time < 0.5
            )
            # Before the first permit confirmation, the reset/power sequence may
            # legitimately be waiting for the independent safety manager.  Once
            # it did confirm, a false or stale permit is a loss of the active
            # lease and must reassert the physical safe commands immediately.
            if (requested and self._permit_confirmed_for_current_lease
                    and not permit_fresh_and_enabled):
                self._lease.stop("physical_safety_permit_lost_requires_new_operator_request")
                requested = False
            if not requested:
                self._permit_confirmed_for_current_lease = False
            if requested and not self._previous_power_requested:
                self._reset_until = now + 1.0
            if not requested:
                self._reset_until = 0.0
            self._previous_power_requested = requested
            self._main_power.publish(Bool(data=requested))
            self._estop.publish(Bool(data=not requested))
            reset_active = requested and now < self._reset_until
            self._estop_reset.publish(Bool(data=reset_active))
            # Power request is not evidence that the physical interlock opened.
            confirmed = requested and not reset_active and permit_fresh_and_enabled
            if confirmed:
                self._permit_confirmed_for_current_lease = True
            self._armed_publisher.publish(Bool(data=confirmed))
            self._status_sequence += 1
            self._status.publish(String(data=json.dumps({
                "armed": confirmed, "power_requested": requested, "reason": self._lease.reason,
                "physical_safety_permit_fresh": permit_fresh_and_enabled,
                "control_owners_ready": self._lease.owners_ready(now),
                "instance_id": self._instance_id,
                "sequence": self._status_sequence,
                "published_at_unix_ns": time.time_ns(),
            }, sort_keys=True)))

    return SimulationOperatorGate


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init()
    node = None
    try:
        node = node_class()()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None and rclpy.ok():
            node._lease.stop("operator_gate_shutdown")
            node._publish()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
