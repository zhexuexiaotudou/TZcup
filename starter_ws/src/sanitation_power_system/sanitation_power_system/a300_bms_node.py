"""ROS 2 product adapter for the deterministic A300 battery core."""

from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float64, String

from .a300_bms_core import A300BatteryCore, A300BatteryParameters


class A300BmsNode(Node):
    def __init__(self) -> None:
        super().__init__("a300_bms_simulator")
        defaults = A300BatteryParameters()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)
        self.declare_parameter("initial_soc", 0.8)
        self.declare_parameter("initial_temperature_c", 25.0)
        self.declare_parameter("publish_period_sec", 0.05)
        self.declare_parameter("charge_request_timeout_sec", 0.25)
        for name in ("publish_period_sec", "charge_request_timeout_sec"):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        parameters = A300BatteryParameters(
            **{
                name: float(self.get_parameter(name).value)
                for name in vars(defaults)
            }
        )
        self._core = A300BatteryCore(
            parameters=parameters,
            initial_soc=float(self.get_parameter("initial_soc").value),
            initial_temperature_c=float(
                self.get_parameter("initial_temperature_c").value
            ),
        )
        self._load_w = 0.0
        self._charge_request_w = 0.0
        self._charge_request_monotonic = float("-inf")
        self._estop = True
        self._main_power = False
        self._last_clock_ns = self.get_clock().now().nanoseconds
        # Safety telemetry cadence is a real controller watchdog boundary and
        # must not slow down with Gazebo RTF or stop with a paused /clock.
        # Energy integration below deliberately continues to use the node's
        # ROS/simulation clock so a paused simulation cannot consume energy.
        self._scheduler_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._battery = self.create_publisher(
            BatteryState, "/formal_vehicle/power/battery_state", 10
        )
        self._status = self.create_publisher(
            String, "/formal_vehicle/power/bms_status_json", 10
        )
        self._soc = self.create_publisher(
            Float64, "/formal_vehicle/power/battery_soc", 10
        )
        self._fault = self.create_publisher(
            Bool, "/formal_vehicle/power/bms_fault", 10
        )
        self._traction_permitted = self.create_publisher(
            Bool, "/formal_vehicle/power/traction_permitted", 10
        )
        self.create_subscription(
            Float64,
            "/formal_vehicle/power/load_request_w",
            lambda message: setattr(self, "_load_w", max(0.0, float(message.data))),
            10,
        )
        self.create_subscription(
            Float64,
            "/formal_vehicle/power/charge_request_w",
            self._on_charge_request,
            10,
        )
        self.create_subscription(
            Bool,
            "/emergency_stop",
            lambda message: setattr(self, "_estop", bool(message.data)),
            10,
        )
        self.create_subscription(
            Bool,
            "/formal_vehicle/power/main_power_requested",
            lambda message: setattr(self, "_main_power", bool(message.data)),
            10,
        )
        self.create_subscription(
            Bool,
            "/formal_vehicle/power/breaker_reset_request",
            self._reset_breaker,
            10,
        )
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._tick,
            clock=self._scheduler_clock,
        )

    def _reset_breaker(self, message: Bool) -> None:
        if message.data:
            self._core.reset_breaker(
                emergency_stop=self._estop,
                main_power_requested=self._main_power,
            )

    def _on_charge_request(self, message: Float64) -> None:
        value = float(message.data)
        if not math.isfinite(value) or value < 0.0:
            self._charge_request_w = 0.0
            self._charge_request_monotonic = float("-inf")
            return
        self._charge_request_w = value
        self._charge_request_monotonic = time.monotonic()

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        elapsed_sec = max(0.0, (now_ns - self._last_clock_ns) * 1e-9)
        self._last_clock_ns = now_ns
        charge_request_fresh = (
            0.0
            <= time.monotonic() - self._charge_request_monotonic
            <= float(self.get_parameter("charge_request_timeout_sec").value)
        )
        state = self._core.step(
            elapsed_sec=elapsed_sec,
            requested_load_power_w=self._load_w,
            requested_charge_power_w=(
                self._charge_request_w if charge_request_fresh else 0.0
            ),
            emergency_stop=self._estop,
            main_power_requested=self._main_power,
        )
        message = BatteryState()
        message.header.stamp = self.get_clock().now().to_msg()
        # The message represents the aggregate two-pack 40 Ah installation.
        message.header.frame_id = "base_link"
        message.voltage = float(state.voltage_v)
        message.temperature = float(state.temperature_c)
        # BatteryState specifies negative current while discharging.
        message.current = float(state.current_a)
        message.charge = float(state.charge_ah)
        message.capacity = float(self._core.parameters.design_capacity_ah)
        message.design_capacity = float(self._core.parameters.design_capacity_ah)
        message.percentage = float(state.soc)
        message.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING
            if state.charging
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            if state.load_power_delivered_w > 0.0
            else BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        )
        message.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
            if state.breaker_latched
            else BatteryState.POWER_SUPPLY_HEALTH_GOOD
        )
        message.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        message.present = True
        message.cell_voltage = [float("nan")] * 8
        message.cell_temperature = [float("nan")] * 8
        message.location = "a300_internal_two_pack_40ah_configuration"
        message.serial_number = "SIMULATION_UNSERIALIZED"
        self._battery.publish(message)
        fault = bool(state.breaker_latched or state.soc <= 0.02)
        traction_permitted = bool(
            self._main_power and not self._estop and not fault and not state.charging
        )
        self._soc.publish(Float64(data=float(state.soc)))
        self._fault.publish(Bool(data=fault))
        self._traction_permitted.publish(Bool(data=traction_permitted))
        self._status.publish(
            String(
                data=json.dumps(
                    {
                        "schema": "tzcup.a300_bms_simulation.v1",
                        "evidence_authority": "SIMULATION_ENGINEERING_ONLY",
                        "official_boundaries": {
                            "chemistry": "LiFePO4",
                            "nominal_voltage_v": 25.6,
                            "capacity_ah": 40.0,
                            "energy_wh": 1024.0,
                            "continuous_current_a": 60.0,
                            "breaker_current_a": 100.0,
                        },
                        "state": vars(state),
                        "bms_fault": fault,
                        "traction_permitted": traction_permitted,
                        "charge_request_fresh": charge_request_fresh,
                        "engineering_voltage_model_requires_hardware_calibration": True,
                    },
                    sort_keys=True,
                    default=list,
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = A300BmsNode()
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
