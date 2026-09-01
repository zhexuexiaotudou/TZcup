"""Fail-closed cleaning-mode coordinator behind the whole-vehicle safety gate."""

from __future__ import annotations

import math
import time

from .formal_cleaning_core import FormalCleaningCore


CONTROL_INPUT_TOPICS = (
    "/active_cleaning/cleaning_requested",
    "/safety/actuators_enabled",
)


def main() -> None:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Float64MultiArray
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    class FormalCleaningCoordinator(Node):
        def __init__(self) -> None:
            super().__init__("formal_active_cleaning_cleaning_coordinator")
            self.declare_parameter("request_topic", CONTROL_INPUT_TOPICS[0])
            self.declare_parameter("safety_permit_topic", CONTROL_INPUT_TOPICS[1])
            self.declare_parameter("brush_command_topic", "/safety/command/brush")
            self.declare_parameter("pump_command_topic", "/safety/command/pump")
            self.declare_parameter(
                "cleaning_trajectory_topic",
                "/cleaning_controller/joint_trajectory",
            )
            self.declare_parameter(
                "water_enable_topic",
                "/model/tzcup_formal_sanitation_vehicle/water_recovery/command/enable",
            )
            self.declare_parameter("status_topic", "/active_cleaning/cleaning_status")
            self.declare_parameter("request_timeout_sec", 1.0)
            self.declare_parameter("safety_timeout_sec", 0.5)
            self.declare_parameter("joint_state_timeout_sec", 0.5)
            self.declare_parameter("joint_states_topic", "/joint_states")
            self.declare_parameter("lift_joint_name", "cleaning_lift_joint")
            self.declare_parameter("work_lift_m", 0.100)
            self.declare_parameter("transport_lift_m", 0.000)
            self.declare_parameter("lift_tolerance_m", 0.001)
            self.declare_parameter("left_brush_rad_s", 8.0)
            self.declare_parameter("right_brush_rad_s", -8.0)
            self.declare_parameter("roller_rad_s", 12.0)
            self.declare_parameter("pump_rad_s", 20.0)
            self.declare_parameter("publish_period_sec", 0.10)
            self.declare_parameter("trajectory_republish_sec", 1.0)

            self._core = FormalCleaningCore(
                request_timeout_sec=float(
                    self.get_parameter("request_timeout_sec").value
                ),
                safety_timeout_sec=float(
                    self.get_parameter("safety_timeout_sec").value
                ),
                joint_state_timeout_sec=float(
                    self.get_parameter("joint_state_timeout_sec").value
                ),
                work_lift_m=float(self.get_parameter("work_lift_m").value),
                transport_lift_m=float(
                    self.get_parameter("transport_lift_m").value
                ),
                lift_tolerance_m=float(
                    self.get_parameter("lift_tolerance_m").value
                ),
            )

            latched = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._brush = self.create_publisher(
                Float64MultiArray,
                str(self.get_parameter("brush_command_topic").value),
                10,
            )
            self._pump = self.create_publisher(
                Float64MultiArray,
                str(self.get_parameter("pump_command_topic").value),
                10,
            )
            self._cleaning = self.create_publisher(
                JointTrajectory,
                str(self.get_parameter("cleaning_trajectory_topic").value),
                10,
            )
            self._water_enable = self.create_publisher(
                Bool,
                str(self.get_parameter("water_enable_topic").value),
                latched,
            )
            self._status = self.create_publisher(
                DiagnosticArray,
                str(self.get_parameter("status_topic").value),
                latched,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("request_topic").value),
                self._on_request,
                latched,
            )
            self.create_subscription(
                JointState,
                str(self.get_parameter("joint_states_topic").value),
                self._on_joint_state,
                20,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("safety_permit_topic").value),
                self._on_safety,
                latched,
            )
            self._requested = False
            self._request_time: float | None = None
            self._permitted = False
            self._safety_time: float | None = None
            self._lift_position: float | None = None
            self._joint_state_time: float | None = None
            self._last_lift: float | None = None
            self._last_lift_command_time: float | None = None
            self.create_timer(
                float(self.get_parameter("publish_period_sec").value), self._publish
            )
            self._publish()

        def _on_request(self, message: Bool) -> None:
            self._requested = bool(message.data)
            self._request_time = time.monotonic()

        def _on_safety(self, message: Bool) -> None:
            self._permitted = bool(message.data)
            self._safety_time = time.monotonic()

        def _on_joint_state(self, message: JointState) -> None:
            lift_joint = str(self.get_parameter("lift_joint_name").value)
            try:
                index = message.name.index(lift_joint)
                position = float(message.position[index])
            except (ValueError, IndexError, TypeError):
                return
            if math.isfinite(position):
                self._lift_position = position
                self._joint_state_time = time.monotonic()

        def _lift_command(self, lift_m: float, now: float) -> None:
            republish = float(self.get_parameter("trajectory_republish_sec").value)
            if (
                self._last_lift is not None
                and abs(self._last_lift - lift_m) < 1.0e-9
                and self._last_lift_command_time is not None
                and now - self._last_lift_command_time < republish
            ):
                return
            point = JointTrajectoryPoint()
            point.positions = [lift_m]
            point.time_from_start.sec = 3
            message = JointTrajectory()
            message.joint_names = ["cleaning_lift_joint"]
            message.points = [point]
            self._cleaning.publish(message)
            self._last_lift = lift_m
            self._last_lift_command_time = now

        def _publish(self) -> None:
            now = time.monotonic()
            decision = self._core.evaluate(
                now=now,
                requested=self._requested,
                request_stamp=self._request_time,
                permitted=self._permitted,
                safety_stamp=self._safety_time,
                lift_position_m=self._lift_position,
                joint_state_stamp=self._joint_state_time,
            )
            active = decision.active
            brush = (
                [
                    float(self.get_parameter("left_brush_rad_s").value),
                    float(self.get_parameter("right_brush_rad_s").value),
                    float(self.get_parameter("roller_rad_s").value),
                ]
                if active
                else [0.0, 0.0, 0.0]
            )
            pump = [float(self.get_parameter("pump_rad_s").value)] if active else [0.0]
            if not all(math.isfinite(value) for value in (*brush, *pump)):
                active = False
                brush = [0.0, 0.0, 0.0]
                pump = [0.0]
            self._brush.publish(Float64MultiArray(data=brush))
            self._pump.publish(Float64MultiArray(data=pump))
            self._water_enable.publish(Bool(data=active))
            if decision.target_lift_m is not None:
                self._lift_command(decision.target_lift_m, now)
            else:
                self._last_lift = None
                self._last_lift_command_time = None

            status = DiagnosticStatus()
            status.name = "formal_active_cleaning_cleaning_coordinator"
            status.hardware_id = "safety_gated_brush_pump_lift"
            status.level = DiagnosticStatus.OK if active else DiagnosticStatus.WARN
            status.message = decision.phase
            status.values = [
                KeyValue(key="requested", value=str(self._requested).lower()),
                KeyValue(key="request_fresh", value=str(decision.request_fresh).lower()),
                KeyValue(key="safety_permitted", value=str(self._permitted).lower()),
                KeyValue(key="safety_fresh", value=str(decision.safety_fresh).lower()),
                KeyValue(
                    key="joint_state_fresh",
                    value=str(decision.joint_state_fresh).lower(),
                ),
                KeyValue(
                    key="work_pose_reached",
                    value=str(decision.work_pose_reached).lower(),
                ),
                KeyValue(key="reason", value=decision.reason),
                KeyValue(key="water_recovery_enabled", value=str(active).lower()),
            ]
            message = DiagnosticArray()
            message.header.stamp = self.get_clock().now().to_msg()
            message.status = [status]
            self._status.publish(message)

    rclpy.init()
    node = None
    try:
        node = FormalCleaningCoordinator()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
