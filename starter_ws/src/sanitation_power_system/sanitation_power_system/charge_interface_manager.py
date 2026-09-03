"""ROS adapter binding physical charge hardware to the A300 BMS."""

from __future__ import annotations

import json
import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Bool, Float64, String

from .charge_interface_core import ChargeInterfaceInputs
from .charge_interface_core import evaluate_charge_interface


class ChargeInterfaceManager(Node):
    def __init__(self) -> None:
        super().__init__("charge_interface_manager")
        self.declare_parameter("publish_period_sec", 0.05)
        self.declare_parameter("input_timeout_sec", 0.25)
        self.declare_parameter("rated_charge_power_w", 650.0)
        self.declare_parameter("door_open_threshold_rad", 1.70)
        self.declare_parameter("lock_engaged_threshold_m", 0.005)
        self.declare_parameter("stationary_linear_threshold_m_s", 0.02)
        self.declare_parameter("stationary_angular_threshold_rad_s", 0.03)
        positive_parameters = (
            "publish_period_sec",
            "input_timeout_sec",
            "rated_charge_power_w",
            "door_open_threshold_rad",
            "lock_engaged_threshold_m",
            "stationary_linear_threshold_m_s",
            "stationary_angular_threshold_rad_s",
        )
        for name in positive_parameters:
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        self._values = {
            "requested": False,
            "plug_present": False,
            "traction_permitted": False,
            "emergency_stop_active": True,
            "main_power_requested": False,
            "bms_fault": True,
            "bms": False,
            "joint": False,
            "odom": False,
        }
        self._times = {name: float("-inf") for name in self._values}
        self._door_position = 0.0
        self._lock_position = 0.0
        self._linear_speed = math.inf
        self._angular_speed = math.inf
        for topic, key in (
            ("/formal_vehicle/power/charge_requested", "requested"),
            ("/formal_vehicle/power/traction_permitted", "traction_permitted"),
            ("/emergency_stop", "emergency_stop_active"),
            ("/formal_vehicle/power/main_power_requested", "main_power_requested"),
            ("/formal_vehicle/power/bms_fault", "bms_fault"),
        ):
            self.create_subscription(Bool, topic, self._bool_callback(key), 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(
            BatteryState, "/formal_vehicle/power/battery_state", self._on_bms, 10
        )
        self.create_subscription(
            Contacts,
            "/formal_vehicle/service/raw/charge_plug_contact",
            self._on_charge_plug_contact,
            qos_profile_sensor_data,
        )
        self._enable_pub = self.create_publisher(
            Bool, "/formal_vehicle/power/charge_enable", 10
        )
        self._connected_pub = self.create_publisher(
            Bool, "/formal_vehicle/power/charge_connected", 10
        )
        self._request_pub = self.create_publisher(
            Float64, "/formal_vehicle/power/charge_request_w", 10
        )
        self._status_pub = self.create_publisher(
            String, "/formal_vehicle/power/charge_status_json", 10
        )
        self.create_timer(float(self.get_parameter("publish_period_sec").value), self._tick)

    def _bool_callback(self, key: str):
        def callback(message: Bool) -> None:
            self._values[key] = bool(message.data)
            self._times[key] = time.monotonic()

        return callback

    def _on_bms(self, _message: BatteryState) -> None:
        self._values["bms"] = True
        self._times["bms"] = time.monotonic()

    def _on_charge_plug_contact(self, message: Contacts) -> None:
        self._values["plug_present"] = bool(message.contacts)
        self._times["plug_present"] = time.monotonic()

    def _on_joint_state(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        door = positions.get("charge_port_door_hinge_joint")
        lock = positions.get("charge_connector_lock_joint")
        if door is None or lock is None or not math.isfinite(door) or not math.isfinite(lock):
            return
        self._door_position = float(door)
        self._lock_position = float(lock)
        self._values["joint"] = True
        self._times["joint"] = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        values = (linear.x, linear.y, linear.z, angular.x, angular.y, angular.z)
        if not all(math.isfinite(value) for value in values):
            return
        self._linear_speed = math.sqrt(linear.x**2 + linear.y**2 + linear.z**2)
        self._angular_speed = math.sqrt(angular.x**2 + angular.y**2 + angular.z**2)
        self._values["odom"] = True
        self._times["odom"] = time.monotonic()

    def _tick(self) -> None:
        now = time.monotonic()
        timeout = float(self.get_parameter("input_timeout_sec").value)
        required = tuple(self._times)
        telemetry_fresh = all(0.0 <= now - self._times[key] <= timeout for key in required)
        plug_contact_fresh = (
            0.0 <= now - self._times["plug_present"] <= timeout
        )
        plug_present = bool(self._values["plug_present"] and plug_contact_fresh)
        stationary = bool(
            self._linear_speed
            <= float(self.get_parameter("stationary_linear_threshold_m_s").value)
            and self._angular_speed
            <= float(self.get_parameter("stationary_angular_threshold_rad_s").value)
        )
        decision = evaluate_charge_interface(
            ChargeInterfaceInputs(
                requested=bool(self._values["requested"]),
                plug_present=plug_present,
                lock_engaged=self._lock_position
                >= float(self.get_parameter("lock_engaged_threshold_m").value),
                door_open=self._door_position
                >= float(self.get_parameter("door_open_threshold_rad").value),
                stationary=stationary,
                traction_permitted=bool(self._values["traction_permitted"]),
                emergency_stop_active=bool(self._values["emergency_stop_active"]),
                main_power_requested=bool(self._values["main_power_requested"]),
                bms_fault=bool(self._values["bms_fault"]),
                telemetry_fresh=telemetry_fresh,
            ),
            rated_charge_power_w=float(self.get_parameter("rated_charge_power_w").value),
        )
        self._enable_pub.publish(Bool(data=decision.enabled))
        self._connected_pub.publish(Bool(data=decision.enabled))
        self._request_pub.publish(Float64(data=decision.charge_power_request_w))
        self._status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "schema": "tzcup.charge_interface.v1",
                        "enabled": decision.enabled,
                        "reasons": list(decision.reasons),
                        "door_position_rad": self._door_position,
                        "lock_position_m": self._lock_position,
                        "plug_present": plug_present,
                        "plug_contact_fresh": plug_contact_fresh,
                        "stationary": stationary,
                        "telemetry_fresh": telemetry_fresh,
                    },
                    sort_keys=True,
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChargeInterfaceManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok(context=node.context):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
