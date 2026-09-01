"""ROS manager for the spring-return wastewater service drain."""

from __future__ import annotations

import json
import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .service_drain_core import ServiceDrainCore


WHEEL_JOINTS = (
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
)
CLEANING_JOINTS = (
    "left_side_brush_joint",
    "right_side_brush_joint",
    "central_roller_joint",
)
PUMP_JOINT = "recovery_pump_joint"
DRAIN_JOINT = "wastewater_drain_valve_joint"
CAP_JOINT = "wastewater_drain_service_cap_joint"


class ServiceDrainManager(Node):
    def __init__(self) -> None:
        super().__init__("service_drain_safety_manager")
        self._declare_parameters()
        self._core = ServiceDrainCore(
            input_timeout_s=self.get_parameter("input_timeout_s").value,
            open_position_rad=self.get_parameter("open_position_rad").value,
        )
        self._trajectory_pub = self.create_publisher(
            JointTrajectory,
            self.get_parameter("joint_command_topic").value,
            10,
        )
        self._recovery_pub = self.create_publisher(
            Bool,
            self.get_parameter("water_recovery_drain_command_topic").value,
            10,
        )
        self._status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10
        )
        self.create_subscription(
            Bool, self.get_parameter("request_topic").value,
            lambda message: self._update("request_open", message.data), 10,
        )
        self.create_subscription(
            Contacts,
            self.get_parameter("hose_contact_topic").value,
            self._on_hose_contact,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool, self.get_parameter("safety_permit_topic").value,
            lambda message: self._update("safety_permit", message.data), 10,
        )
        self.create_subscription(
            Bool, self.get_parameter("power_available_topic").value,
            lambda message: self._update("power_available", message.data), 10,
        )
        self.create_subscription(
            Float64, self.get_parameter("tank_level_topic").value,
            self._on_tank_level, 10,
        )
        self.create_subscription(
            JointState, self.get_parameter("joint_states_topic").value,
            self._on_joint_states, 20,
        )
        self._timer = self.create_timer(
            self.get_parameter("publish_period_s").value, self._publish
        )
        self._publish()

    def _declare_parameters(self) -> None:
        self.declare_parameter("request_topic", "/safety/command/service_drain_open")
        self.declare_parameter(
            "hose_contact_topic", "/formal_vehicle/service/raw/drain_hose_contact"
        )
        self.declare_parameter("safety_permit_topic", "/safety/actuators_enabled")
        self.declare_parameter("power_available_topic", "/formal_vehicle/power/branches/high_power/enabled")
        self.declare_parameter(
            "tank_level_topic",
            "/model/tzcup_formal_sanitation_vehicle/water_recovery/sensed_tank_level_fraction",
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("joint_command_topic", "/service_controller/joint_trajectory")
        self.declare_parameter(
            "water_recovery_drain_command_topic",
            "/model/tzcup_formal_sanitation_vehicle/water_recovery/command/service_drain_open",
        )
        self.declare_parameter("status_topic", "/formal_vehicle/service/drain_status_json")
        self.declare_parameter("input_timeout_s", 0.25)
        self.declare_parameter("publish_period_s", 0.02)
        self.declare_parameter("open_position_rad", math.pi / 2)
        self.declare_parameter("stationary_velocity_rad_s", 0.10)
        self.declare_parameter("cleaning_stopped_velocity_rad_s", 0.10)
        self.declare_parameter("pump_stopped_velocity_rad_s", 0.10)
        self.declare_parameter("cap_open_position_rad", 0.35)

    def _update(self, name: str, value: bool) -> None:
        self._core.update(name, bool(value), time.monotonic())
        self._publish()

    def _on_tank_level(self, message: Float64) -> None:
        value = float(message.data)
        self._update("tank_valid", math.isfinite(value) and 0.0 <= value <= 1.0)

    def _on_hose_contact(self, message: Contacts) -> None:
        # This topic is sourced only by the dedicated drain-coupling contact
        # sensor.  No environment entity identity enters vehicle control.
        self._update("hose_connected", bool(message.contacts))

    def _on_joint_states(self, message: JointState) -> None:
        position = {
            name: float(value)
            for name, value in zip(message.name, message.position)
            if math.isfinite(value)
        }
        velocity = {
            name: float(value)
            for name, value in zip(message.name, message.velocity)
            if math.isfinite(value)
        }
        wheel_limit = float(self.get_parameter("stationary_velocity_rad_s").value)
        cleaning_limit = float(
            self.get_parameter("cleaning_stopped_velocity_rad_s").value
        )
        pump_limit = float(self.get_parameter("pump_stopped_velocity_rad_s").value)
        stationary = all(
            joint in velocity and abs(velocity[joint]) <= wheel_limit
            for joint in WHEEL_JOINTS
        )
        cleaning_stopped = all(
            joint in velocity and abs(velocity[joint]) <= cleaning_limit
            for joint in CLEANING_JOINTS
        )
        pump_stopped = (
            PUMP_JOINT in velocity and abs(velocity[PUMP_JOINT]) <= pump_limit
        )
        now = time.monotonic()
        if CAP_JOINT in position:
            self._core.update(
                "cap_open",
                position[CAP_JOINT]
                >= float(self.get_parameter("cap_open_position_rad").value),
                now,
            )
        self._core.update("stationary", stationary, now)
        self._core.update("cleaning_stopped", cleaning_stopped, now)
        self._core.update("pump_stopped", pump_stopped, now)
        self._publish()

    def _publish(self) -> None:
        decision = self._core.evaluate(time.monotonic())
        point = JointTrajectoryPoint()
        point.positions = [decision.target_position_rad]
        point.time_from_start = Duration(nanosec=100_000_000)
        trajectory = JointTrajectory()
        trajectory.joint_names = [DRAIN_JOINT]
        trajectory.points = [point]
        self._trajectory_pub.publish(trajectory)
        self._recovery_pub.publish(Bool(data=decision.water_recovery_drain_open))
        self._status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "permitted": decision.permitted,
                        "target_position_rad": decision.target_position_rad,
                        "water_recovery_drain_open": decision.water_recovery_drain_open,
                        "reasons": decision.reasons,
                    },
                    separators=(",", ":"),
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServiceDrainManager()
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
