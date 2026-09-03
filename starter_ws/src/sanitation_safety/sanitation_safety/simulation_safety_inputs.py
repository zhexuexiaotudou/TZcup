"""Healthy-process simulation inputs and formal auxiliary product states."""

from __future__ import annotations

import json
import math
import threading
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Empty, Float64, String

from .formal_auxiliary_system_core import FormalAuxiliarySystemCore
from .formal_auxiliary_system_core import PRODUCT_BINDINGS


class SimulationSafetyInputs(Node):
    """Publish fail-closed safety inputs and engineering-only auxiliary state."""

    def __init__(self) -> None:
        super().__init__("simulation_safety_inputs")
        self._state_lock = threading.RLock()
        self._safety_output_lock = threading.RLock()
        self._declare_parameters()
        self._estop_active = bool(self.get_parameter("initial_estop_active").value)
        self._main_power_requested = False
        self._main_isolator_closed = False
        self._main_contactor_closed = False
        self._estop_state_time: float | None = None
        self._main_power_command_time: float | None = None
        self._main_isolator_state_time: float | None = None
        self._main_contactor_state_time: float | None = None
        self._charge_requested = False
        self._charge_connected = False
        self._charge_connected_monotonic = float("-inf")
        self._work_lights_requested = False
        self._tail_lights_requested = False
        self._warning_lights_requested = False
        self._battery_soc = 0.0
        self._battery_voltage_v: float | None = None
        self._battery_state_monotonic = float("-inf")
        self._core = FormalAuxiliarySystemCore(
            battery_soc=float(self.get_parameter("initial_battery_soc").value),
            simulation_battery_capacity_kwh=float(
                self.get_parameter("simulation_battery_capacity_kwh").value
            ),
            simulation_charge_power_kw=float(
                self.get_parameter("simulation_charge_power_kw").value
            ),
        )
        self._last_clock_ns = self.get_clock().now().nanoseconds
        self._front_raw_contact = Contacts()
        self._rear_raw_contact = Contacts()
        self._front_contact_until = 0.0
        self._rear_contact_until = 0.0
        self._front_raw_bridge_available = False
        self._rear_raw_bridge_available = False
        self._front_raw_sample_monotonic = float("-inf")
        self._rear_raw_sample_monotonic = float("-inf")
        self._state_timer_group = MutuallyExclusiveCallbackGroup()
        self._safety_publish_period_sec = float(
            self.get_parameter("publish_period_sec").value
        )
        self._operator_command_timeout_sec = float(
            self.get_parameter("operator_command_timeout_sec").value
        )
        self._bumper_contact_latch_sec = float(
            self.get_parameter("bumper_contact_latch_sec").value
        )
        self._raw_contact_sample_timeout_sec = float(
            self.get_parameter("raw_contact_sample_timeout_sec").value
        )
        self._battery_state_timeout_sec = float(
            self.get_parameter("battery_state_timeout_sec").value
        )
        self._charge_connected_timeout_sec = float(
            self.get_parameter("charge_connected_timeout_sec").value
        )
        self._physical_power_feedback_timeout_sec = float(
            self.get_parameter("physical_power_feedback_timeout_sec").value
        )
        self._safety_publish_stop_event = threading.Event()
        self._safety_publish_thread_error: BaseException | None = None
        self._safety_publish_count = 0
        self._maximum_safety_publish_gap_sec = 0.0
        self._last_safety_publish_monotonic: float | None = None

        # /emergency_stop has exactly one formal writer: the one-way
        # ROS-Gazebo bridge carrying the physical plugin's latched output.
        # This node consumes that state to derive relay and power branches.
        self._estop_subscription = self.create_subscription(
            Bool, "/emergency_stop", self._on_estop_state, 20
        )
        self._relay_pub = self.create_publisher(Bool, "/safety/relay_enabled", 10)
        self._heartbeat_pub = self.create_publisher(
            Empty, "/safety/control_heartbeat", 10
        )
        self._front_bumper_pub = self.create_publisher(
            Contacts, "/safety/front_bumper/contact", 10
        )
        self._rear_bumper_pub = self.create_publisher(
            Contacts, "/safety/rear_bumper/contact", 10
        )
        self._critical_status_pub = self.create_publisher(
            String,
            "/formal_vehicle/auxiliary/critical_safety_status_json",
            1,
        )
        self._product_publishers = {
            "charge_requested": self.create_publisher(
                Bool, "/formal_vehicle/power/charge_requested", 10
            ),
            "main_power_requested": self.create_publisher(
                Bool, "/formal_vehicle/power/main_power_requested", 10
            ),
            "main_contactor_command": self.create_publisher(
                Bool, "/formal_vehicle/power/main_contactor_command", 10
            ),
            "safety_branch": self.create_publisher(
                Bool, "/formal_vehicle/power/branches/safety/enabled", 10
            ),
            "low_voltage_branch": self.create_publisher(
                Bool, "/formal_vehicle/power/branches/low_voltage/enabled", 10
            ),
            "high_power_branch": self.create_publisher(
                Bool, "/formal_vehicle/power/branches/high_power/enabled", 10
            ),
            "work_lights": self.create_publisher(
                Bool, "/formal_vehicle/lighting/work_lights_on", 10
            ),
            "tail_lights": self.create_publisher(
                Bool, "/formal_vehicle/lighting/tail_lights_on", 10
            ),
            "warning_lights": self.create_publisher(
                Bool, "/formal_vehicle/lighting/warning_lights_on", 10
            ),
            "status": self.create_publisher(
                String, "/formal_vehicle/auxiliary/status_json", 10
            ),
            "load_request": self.create_publisher(
                Float64, "/formal_vehicle/power/load_request_w", 10
            ),
        }
        command_topics = {
            "main_power": self._on_main_power,
            "charge_connected": self._on_charge,
            "work_lights": self._on_work_lights,
            "tail_lights": self._on_tail_lights,
            "warning_lights": self._on_warning_lights,
        }
        self._command_subscriptions = [
            self.create_subscription(
                Bool,
                f"/formal_vehicle/simulation/command/{name}",
                callback,
                10,
            )
            for name, callback in command_topics.items()
        ]
        self._front_raw_subscription = self.create_subscription(
            Contacts,
            "/formal_vehicle/simulation/raw/front_bumper/contact",
            self._on_front_raw_contact,
            20,
            event_callbacks=SubscriptionEventCallbacks(
                matched=self._on_front_raw_match
            ),
        )
        self._rear_raw_subscription = self.create_subscription(
            Contacts,
            "/formal_vehicle/simulation/raw/rear_bumper/contact",
            self._on_rear_raw_contact,
            20,
            event_callbacks=SubscriptionEventCallbacks(
                matched=self._on_rear_raw_match
            ),
        )
        self._raw_contact_subscriptions = [
            self._front_raw_subscription,
            self._rear_raw_subscription,
        ]
        self._battery_subscription = self.create_subscription(
            BatteryState,
            "/formal_vehicle/power/battery_state",
            self._on_battery_state,
            10,
        )
        self._charge_connected_subscription = self.create_subscription(
            Bool,
            "/formal_vehicle/power/charge_connected",
            self._on_charge_connected,
            10,
        )
        self._main_isolator_subscription = self.create_subscription(
            Bool,
            "/formal_vehicle/power/main_isolator_closed",
            self._on_main_isolator_closed,
            20,
        )
        self._main_contactor_subscription = self.create_subscription(
            Bool,
            "/formal_vehicle/power/main_contactor_closed",
            self._on_main_contactor_closed,
            20,
        )
        self._timer = self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._publish,
            callback_group=self._state_timer_group,
        )
        # Do not invoke _publish synchronously from the constructor.  It asks
        # the ROS graph for raw-contact bridge endpoints; with CycloneDDS that
        # graph query can block until the node has entered an executor.  The
        # 50 ms wall-time timer provides the first fail-closed sample once the
        # executor is spinning.
        self._status_publication_started = False
        self._safety_publish_thread = threading.Thread(
            target=self._run_safety_publish_loop,
            name="simulation_safety_inputs_critical_publish",
            daemon=True,
        )
        self._safety_publish_thread.start()
        self.get_logger().info("formal auxiliary publishers and timer configured")

    def _declare_parameters(self) -> None:
        self.declare_parameter("initial_estop_active", True)
        self.declare_parameter("initial_battery_soc", 0.8)
        self.declare_parameter("simulation_battery_capacity_kwh", 1.024)
        self.declare_parameter("simulation_charge_power_kw", 0.650)
        self.declare_parameter("publish_period_sec", 0.05)
        self.declare_parameter("operator_command_timeout_sec", 0.5)
        self.declare_parameter("bumper_contact_latch_sec", 0.25)
        self.declare_parameter("raw_contact_sample_timeout_sec", 0.25)
        self.declare_parameter("battery_state_timeout_sec", 0.25)
        self.declare_parameter("charge_connected_timeout_sec", 0.25)
        self.declare_parameter("physical_power_feedback_timeout_sec", 0.25)
        for name in (
            "publish_period_sec",
            "operator_command_timeout_sec",
            "bumper_contact_latch_sec",
            "raw_contact_sample_timeout_sec",
            "battery_state_timeout_sec",
            "charge_connected_timeout_sec",
            "physical_power_feedback_timeout_sec",
        ):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")

    def _on_battery_state(self, message: BatteryState) -> None:
        percentage = float(message.percentage)
        voltage = float(message.voltage)
        if not 0.0 <= percentage <= 1.0:
            return
        with self._state_lock:
            self._battery_soc = percentage
            self._battery_voltage_v = voltage if voltage > 0.0 else None
            self._battery_state_monotonic = time.monotonic()

    def _on_front_raw_contact(self, message: Contacts) -> None:
        with self._state_lock:
            self._front_raw_contact = message
            self._front_raw_sample_monotonic = time.monotonic()
            self._front_contact_until = self._contact_deadline(message)

    def _on_rear_raw_contact(self, message: Contacts) -> None:
        with self._state_lock:
            self._rear_raw_contact = message
            self._rear_raw_sample_monotonic = time.monotonic()
            self._rear_contact_until = self._contact_deadline(message)

    def _on_front_raw_match(self, event) -> None:
        with self._state_lock:
            self._front_raw_bridge_available = event.current_count > 0

    def _on_rear_raw_match(self, event) -> None:
        with self._state_lock:
            self._rear_raw_bridge_available = event.current_count > 0

    def _contact_deadline(self, message: Contacts) -> float:
        if not message.contacts:
            return 0.0
        return time.monotonic() + self._bumper_contact_latch_sec

    def _on_estop_state(self, message: Bool) -> None:
        with self._state_lock:
            self._estop_active = bool(message.data)
            self._estop_state_time = time.monotonic()

    def _on_main_power(self, message: Bool) -> None:
        with self._state_lock:
            self._main_power_requested = bool(message.data)
            self._main_power_command_time = time.monotonic()

    def _on_charge(self, message: Bool) -> None:
        with self._state_lock:
            self._charge_requested = bool(message.data)

    def _on_charge_connected(self, message: Bool) -> None:
        with self._state_lock:
            self._charge_connected = bool(message.data)
            self._charge_connected_monotonic = time.monotonic()

    def _on_main_isolator_closed(self, message: Bool) -> None:
        with self._state_lock:
            self._main_isolator_closed = bool(message.data)
            self._main_isolator_state_time = time.monotonic()

    def _on_main_contactor_closed(self, message: Bool) -> None:
        with self._state_lock:
            self._main_contactor_closed = bool(message.data)
            self._main_contactor_state_time = time.monotonic()

    def _on_work_lights(self, message: Bool) -> None:
        self._work_lights_requested = bool(message.data)

    def _on_tail_lights(self, message: Bool) -> None:
        self._tail_lights_requested = bool(message.data)

    def _on_warning_lights(self, message: Bool) -> None:
        self._warning_lights_requested = bool(message.data)

    @staticmethod
    def _fresh(sample_time: float | None, now: float, timeout: float) -> bool:
        return bool(
            sample_time is not None
            and 0.0 <= now - sample_time <= timeout
        )

    def _critical_safety_snapshot_locked(
        self, monotonic_now: float
    ) -> tuple[bool, Contacts | None, Contacts | None]:
        command_fresh = self._fresh(
            self._estop_state_time,
            monotonic_now,
            self._operator_command_timeout_sec,
        ) and self._fresh(
            self._main_power_command_time,
            monotonic_now,
            self._operator_command_timeout_sec,
        )
        battery_fresh = self._fresh(
            self._battery_state_monotonic,
            monotonic_now,
            self._battery_state_timeout_sec,
        )
        isolator_fresh = self._fresh(
            self._main_isolator_state_time,
            monotonic_now,
            self._physical_power_feedback_timeout_sec,
        )
        contactor_fresh = self._fresh(
            self._main_contactor_state_time,
            monotonic_now,
            self._physical_power_feedback_timeout_sec,
        )
        charge_fresh = self._fresh(
            self._charge_connected_monotonic,
            monotonic_now,
            self._charge_connected_timeout_sec,
        )
        safety_power = battery_fresh and (
            self._battery_soc > self._core.safety_soc_floor
        )
        main_power = command_fresh and self._main_power_requested
        effective_main_power = (
            main_power and isolator_fresh and self._main_isolator_closed
        )
        charge_connected = bool(
            charge_fresh
            and self._charge_requested
            and self._charge_connected
            and self._estop_active
            and not main_power
            and safety_power
            and self._battery_soc < 1.0
        )
        relay_enabled = bool(
            safety_power
            and effective_main_power
            and not self._estop_active
            and not charge_connected
            and contactor_fresh
            and self._main_contactor_closed
        )

        # Gazebo's contact bridge is event-driven and may publish no empty
        # sample while clear. Endpoint matching is therefore the availability
        # contract; sample freshness remains separately visible in product
        # status and must not turn a clear, matched bumper into a false outage.
        front_available = self._front_raw_bridge_available
        rear_available = self._rear_raw_bridge_available
        front = None
        if front_available:
            front = (
                self._front_raw_contact
                if monotonic_now <= self._front_contact_until
                else Contacts()
            )
        rear = None
        if rear_available:
            rear = (
                self._rear_raw_contact
                if monotonic_now <= self._rear_contact_until
                else Contacts()
            )
        return relay_enabled, front, rear

    def _publish_critical_safety(self) -> None:
        monotonic_now = time.monotonic()
        with self._state_lock:
            relay_enabled, front, rear = self._critical_safety_snapshot_locked(
                monotonic_now
            )
            if self._last_safety_publish_monotonic is not None:
                self._maximum_safety_publish_gap_sec = max(
                    self._maximum_safety_publish_gap_sec,
                    monotonic_now - self._last_safety_publish_monotonic,
                )
            self._last_safety_publish_monotonic = monotonic_now
            self._safety_publish_count += 1
            metrics = {
                "schema_version": 1,
                "publish_count": self._safety_publish_count,
                "relay_enabled": relay_enabled,
                "front_bumper_available": front is not None,
                "rear_bumper_available": rear is not None,
                "maximum_gap_sec": round(
                    self._maximum_safety_publish_gap_sec, 6
                ),
                "thread_error": (
                    None
                    if self._safety_publish_thread_error is None
                    else type(self._safety_publish_thread_error).__name__
                ),
            }
        # No state lock crosses a ROS publisher call.
        with self._safety_output_lock:
            self._relay_pub.publish(Bool(data=relay_enabled))
            self._heartbeat_pub.publish(Empty())
            if front is not None:
                self._front_bumper_pub.publish(front)
            if rear is not None:
                self._rear_bumper_pub.publish(rear)
            self._critical_status_pub.publish(
                String(data=json.dumps(metrics, sort_keys=True))
            )

    def _run_safety_publish_loop(self) -> None:
        try:
            period = self._safety_publish_period_sec
            deadline = time.monotonic() + period
            while not self._safety_publish_stop_event.is_set():
                remaining = deadline - time.monotonic()
                if self._safety_publish_stop_event.wait(max(0.0, remaining)):
                    return
                self._publish_critical_safety()
                deadline += period
                now = time.monotonic()
                if deadline <= now:
                    deadline = now + period
        except BaseException as error:
            with self._state_lock:
                self._safety_publish_thread_error = error
            try:
                with self._safety_output_lock:
                    self._relay_pub.publish(Bool(data=False))
            except BaseException:
                pass

    def _stop_safety_publish_loop(self, timeout_sec: float = 2.0) -> None:
        self._safety_publish_stop_event.set()
        if self._safety_publish_thread.is_alive():
            self._safety_publish_thread.join(timeout=timeout_sec)
        if self._safety_publish_thread.is_alive():
            raise RuntimeError(
                "simulation_safety_inputs_publish_thread_join_timeout"
            )

    def _raise_if_safety_publish_failed(self) -> None:
        with self._state_lock:
            error = self._safety_publish_thread_error
        if error is not None:
            raise RuntimeError(
                "simulation_safety_inputs_publish_thread_failed"
            ) from error

    def destroy_node(self):
        self._stop_safety_publish_loop()
        return super().destroy_node()

    def _publish(self) -> None:
        first_cycle = not self._status_publication_started
        if first_cycle:
            self._status_publication_started = True
            self.get_logger().info("formal auxiliary periodic state publication started")
        monotonic_now = time.monotonic()
        command_timeout = float(
            self.get_parameter("operator_command_timeout_sec").value
        )
        command_fresh = (
            self._estop_state_time is not None
            and self._main_power_command_time is not None
            and 0.0 <= monotonic_now - self._estop_state_time <= command_timeout
            and 0.0
            <= monotonic_now - self._main_power_command_time
            <= command_timeout
        )
        if not command_fresh:
            self._estop_active = True
            self._main_power_requested = False
        battery_fresh = (
            0.0
            <= monotonic_now - self._battery_state_monotonic
            <= float(self.get_parameter("battery_state_timeout_sec").value)
        )
        charge_connected_fresh = (
            0.0
            <= monotonic_now - self._charge_connected_monotonic
            <= float(self.get_parameter("charge_connected_timeout_sec").value)
        )
        if not charge_connected_fresh:
            self._charge_connected = False
        physical_feedback_timeout = float(
            self.get_parameter("physical_power_feedback_timeout_sec").value
        )
        isolator_feedback_fresh = bool(
            self._main_isolator_state_time is not None
            and 0.0
            <= monotonic_now - self._main_isolator_state_time
            <= physical_feedback_timeout
        )
        contactor_feedback_fresh = bool(
            self._main_contactor_state_time is not None
            and 0.0
            <= monotonic_now - self._main_contactor_state_time
            <= physical_feedback_timeout
        )
        # The A300 BMS is the unique SOC integrator. Stale telemetry opens all
        # powered branches instead of continuing from an invented battery.
        self._core.battery_soc = self._battery_soc if battery_fresh else 0.0
        now_ns = self.get_clock().now().nanoseconds
        elapsed_sec = max(0.0, (now_ns - self._last_clock_ns) * 1.0e-9)
        self._last_clock_ns = now_ns
        state = self._core.step(
            # Power branching only; A300BmsNode owns energy integration.
            elapsed_sec=0.0,
            emergency_stop_active=self._estop_active,
            main_power_requested=self._main_power_requested,
            charge_connected_requested=self._charge_connected,
            work_lights_requested=self._work_lights_requested,
            tail_lights_requested=self._tail_lights_requested,
            warning_lights_requested=self._warning_lights_requested,
            main_isolator_closed=(
                self._main_isolator_closed if isolator_feedback_fresh else False
            ),
            main_contactor_closed=(
                self._main_contactor_closed if contactor_feedback_fresh else False
            ),
        )
        if first_cycle:
            self.get_logger().info("formal auxiliary state core evaluated")
        # DDS matched events update these cached booleans on connect and
        # disconnect without a synchronous graph query in the safety timer.
        front_bridge_available = self._front_raw_bridge_available
        rear_bridge_available = self._rear_raw_bridge_available
        raw_sample_timeout = float(
            self.get_parameter("raw_contact_sample_timeout_sec").value
        )
        front_sample_age = monotonic_now - self._front_raw_sample_monotonic
        rear_sample_age = monotonic_now - self._rear_raw_sample_monotonic
        front_sample_fresh = 0.0 <= front_sample_age <= raw_sample_timeout
        rear_sample_fresh = 0.0 <= rear_sample_age <= raw_sample_timeout
        if first_cycle:
            self.get_logger().info("formal auxiliary raw bridge graph state evaluated")
        self._product_publishers["charge_requested"].publish(
            Bool(data=self._charge_requested)
        )
        self._product_publishers["main_power_requested"].publish(
            Bool(data=self._main_power_requested)
        )
        self._product_publishers["main_contactor_command"].publish(
            Bool(data=state.relay_command_enabled)
        )
        self._product_publishers["safety_branch"].publish(
            Bool(data=state.safety_branch_enabled)
        )
        self._product_publishers["low_voltage_branch"].publish(
            Bool(data=state.low_voltage_branch_enabled)
        )
        self._product_publishers["high_power_branch"].publish(
            Bool(data=state.high_power_branch_enabled)
        )
        self._product_publishers["work_lights"].publish(
            Bool(data=state.work_lights_on)
        )
        self._product_publishers["tail_lights"].publish(
            Bool(data=state.tail_lights_on)
        )
        self._product_publishers["warning_lights"].publish(
            Bool(data=state.warning_lights_on)
        )
        self._product_publishers["load_request"].publish(
            Float64(data=max(0.0, state.net_battery_power_kw * 1000.0))
        )
        status = {
            "schema": "tzcup.formal_auxiliary_product_state.v1",
            "evidence_authority": "SIMULATION_ENGINEERING_ONLY",
            "battery_voltage_v": self._battery_voltage_v,
            "battery_soc": state.battery_soc,
            "battery_state_fresh": battery_fresh,
            "net_battery_power_kw": state.net_battery_power_kw,
            "emergency_stop_active": state.emergency_stop_active,
            "main_isolator_closed": state.main_isolator_closed,
            "main_isolator_feedback_fresh": isolator_feedback_fresh,
            "main_contactor_commanded": state.relay_command_enabled,
            "main_contactor_closed": state.main_contactor_closed,
            "main_contactor_feedback_fresh": contactor_feedback_fresh,
            "relay_enabled": state.relay_enabled,
            "charge_connected": state.charge_connected,
            "charge_connected_fresh": charge_connected_fresh,
            "branches": {
                "safety": state.safety_branch_enabled,
                "low_voltage": state.low_voltage_branch_enabled,
                "high_power": state.high_power_branch_enabled,
            },
            "lighting": {
                "work": state.work_lights_on,
                "tail": state.tail_lights_on,
                "four_corner_warning": state.warning_lights_on,
            },
            "active_reasons": list(state.active_reasons),
            "operator_command_fresh": command_fresh,
            "bumper_inputs": {
                "front_raw_bridge_available": front_bridge_available,
                "rear_raw_bridge_available": rear_bridge_available,
                "front_raw_bridge_matched": front_bridge_available,
                "rear_raw_bridge_matched": rear_bridge_available,
                "front_raw_sample_fresh": front_sample_fresh,
                "rear_raw_sample_fresh": rear_sample_fresh,
                "front_raw_sample_age_sec": (
                    round(front_sample_age, 6)
                    if math.isfinite(front_sample_age)
                    else None
                ),
                "rear_raw_sample_age_sec": (
                    round(rear_sample_age, 6)
                    if math.isfinite(rear_sample_age)
                    else None
                ),
                "front_contact": monotonic_now <= self._front_contact_until,
                "rear_contact": monotonic_now <= self._rear_contact_until,
            },
            "bindings": PRODUCT_BINDINGS,
            "interface_class": "product_simulation",
            "critical_safety_publisher": {
                "publish_count": self._safety_publish_count,
                "maximum_gap_sec": round(
                    self._maximum_safety_publish_gap_sec, 6
                ),
                "thread_error": (
                    None
                    if self._safety_publish_thread_error is None
                    else type(self._safety_publish_thread_error).__name__
                ),
            },
        }
        self._product_publishers["status"].publish(
            String(data=json.dumps(status, sort_keys=True))
        )
        if first_cycle:
            self.get_logger().info("formal auxiliary first status sample published")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimulationSafetyInputs()
    # Graph-cache queries used to prove that both raw contact bridges exist
    # must not occupy the only executor worker.  A second worker lets DDS graph
    # events complete while the 20 Hz fail-closed state timer is running.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    fatal_error: BaseException | None = None
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            node._raise_if_safety_publish_failed()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError as error:
        if rclpy.ok(context=node.context):
            fatal_error = error
    except BaseException as error:
        fatal_error = error
    finally:
        joined = False
        try:
            node._stop_safety_publish_loop()
            joined = True
        except BaseException as error:
            fatal_error = fatal_error or error
        if not joined:
            raise RuntimeError(
                "simulation_safety_inputs_publish_thread_join_fatal"
            ) from fatal_error
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if fatal_error is not None:
        raise RuntimeError("simulation_safety_inputs_fatal") from fatal_error


if __name__ == "__main__":
    main()
