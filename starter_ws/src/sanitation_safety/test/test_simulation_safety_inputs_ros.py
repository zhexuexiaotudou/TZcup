"""ROS graph test for simulation safety inputs and product state topics."""

from __future__ import annotations

import json
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Empty, String

from sanitation_safety.simulation_safety_inputs import SimulationSafetyInputs


class Observer(Node):
    """Drive simulation commands and collect product states."""

    def __init__(self) -> None:
        super().__init__("simulation_safety_inputs_test_observer")
        self.relay = []
        self.estop = []
        self.heartbeat_count = 0
        self.high_power = []
        self.work_lights = []
        self.status = []
        self.critical_status = []
        self.front_bumper = []
        self.rear_bumper = []
        # Stand in for the one-way physical Gazebo latch bridge in this node
        # unit test. Production simulation_safety_inputs never publishes this.
        self.estop_state = self.create_publisher(Bool, "/emergency_stop", 10)
        self.main_power_command = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/main_power", 10
        )
        # Stand in for the physical switchgear feedback produced by the
        # Gazebo electrical mechanism plugin. Software commands alone must not
        # energize the high-power branch.
        self.main_isolator_state = self.create_publisher(
            Bool, "/formal_vehicle/power/main_isolator_closed", 10
        )
        self.main_contactor_state = self.create_publisher(
            Bool, "/formal_vehicle/power/main_contactor_closed", 10
        )
        self.work_light_command = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/work_lights", 10
        )
        self.battery_state = self.create_publisher(
            BatteryState, "/formal_vehicle/power/battery_state", 10
        )
        self.front_raw = self.create_publisher(
            Contacts,
            "/formal_vehicle/simulation/raw/front_bumper/contact",
            10,
        )
        self.rear_raw = self.create_publisher(
            Contacts,
            "/formal_vehicle/simulation/raw/rear_bumper/contact",
            10,
        )
        self.create_subscription(
            Bool, "/safety/relay_enabled", lambda msg: self.relay.append(msg.data), 20
        )
        self.create_subscription(
            Bool, "/emergency_stop", lambda msg: self.estop.append(msg.data), 20
        )
        self.create_subscription(
            Empty,
            "/safety/control_heartbeat",
            lambda _msg: setattr(
                self, "heartbeat_count", self.heartbeat_count + 1
            ),
            20,
        )
        self.create_subscription(
            Bool,
            "/formal_vehicle/power/branches/high_power/enabled",
            lambda msg: self.high_power.append(msg.data),
            20,
        )
        self.create_subscription(
            Bool,
            "/formal_vehicle/lighting/work_lights_on",
            lambda msg: self.work_lights.append(msg.data),
            20,
        )
        self.create_subscription(
            String,
            "/formal_vehicle/auxiliary/status_json",
            lambda msg: self.status.append(json.loads(msg.data)),
            20,
        )
        self.create_subscription(
            String,
            "/formal_vehicle/auxiliary/critical_safety_status_json",
            lambda msg: self.critical_status.append(json.loads(msg.data)),
            20,
        )
        self.create_subscription(
            Contacts,
            "/safety/front_bumper/contact",
            lambda msg: self.front_bumper.append(bool(msg.contacts)),
            20,
        )
        self.create_subscription(
            Contacts,
            "/safety/rear_bumper/contact",
            lambda msg: self.rear_bumper.append(bool(msg.contacts)),
            20,
        )


def _wait_until(predicate, timeout=3.0, tick=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if tick is not None:
            tick()
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("condition did not become true before timeout")


def test_node_health_inputs_commands_and_product_interface_are_observable():
    rclpy.init()
    source = SimulationSafetyInputs()
    observer = Observer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(source)
    executor.add_node(observer)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert source._critical_status_pub.qos_profile.depth == 1
        def publish_initial_safe_inputs():
            observer.estop_state.publish(Bool(data=True))
            observer.front_raw.publish(Contacts())
            observer.rear_raw.publish(Contacts())

        _wait_until(
            lambda: observer.relay
            and observer.relay[-1] is False
            and observer.estop
            and observer.estop[-1] is True
            and observer.heartbeat_count >= 2
            and observer.status
            and observer.critical_status
            and observer.front_bumper
            and observer.rear_bumper,
            tick=publish_initial_safe_inputs,
        )
        snapshot = observer.status[-1]
        critical_snapshot = observer.critical_status[-1]
        assert critical_snapshot["schema_version"] == 1
        assert critical_snapshot["thread_error"] is None
        assert critical_snapshot["front_bumper_available"] is True
        assert critical_snapshot["rear_bumper_available"] is True
        critical_count = len(observer.critical_status)
        time.sleep(0.30)
        assert 4 <= len(observer.critical_status) - critical_count <= 8
        assert snapshot["battery_voltage_v"] is None
        assert snapshot["interface_class"] == "product_simulation"
        assert snapshot["bindings"]["charge_interface"]["datum"] == (
            "charge_interface_datum_link"
        )
        assert snapshot["bumper_inputs"]["front_raw_bridge_available"] is True
        assert snapshot["bumper_inputs"]["rear_raw_bridge_available"] is True
        assert observer.front_bumper[-1] is False
        assert observer.rear_bumper[-1] is False

        # Contact samples are event-driven and may be stale while the bumper is
        # clear. A matched endpoint keeps the clear stream alive; losing the
        # endpoint stops it fail-closed, and rematching restores it.
        time.sleep(0.50)
        with source._state_lock:
            source._front_raw_sample_monotonic = time.monotonic() - 1.0
            source._rear_raw_sample_monotonic = time.monotonic() - 1.0
        time.sleep(0.10)
        matched_front_count = len(observer.front_bumper)
        matched_rear_count = len(observer.rear_bumper)
        time.sleep(0.15)
        assert len(observer.front_bumper) > matched_front_count
        assert len(observer.rear_bumper) > matched_rear_count
        with source._state_lock:
            source._front_raw_bridge_available = False
            source._rear_raw_bridge_available = False
        time.sleep(0.10)
        unmatched_front_count = len(observer.front_bumper)
        unmatched_rear_count = len(observer.rear_bumper)
        time.sleep(0.15)
        assert len(observer.front_bumper) == unmatched_front_count
        assert len(observer.rear_bumper) == unmatched_rear_count
        with source._state_lock:
            source._front_raw_bridge_available = True
            source._rear_raw_bridge_available = True
        _wait_until(
            lambda: len(observer.front_bumper) > unmatched_front_count
            and len(observer.rear_bumper) > unmatched_rear_count
        )

        def publish_armed_commands():
            observer.estop_state.publish(Bool(data=False))
            observer.main_power_command.publish(Bool(data=True))
            observer.main_isolator_state.publish(Bool(data=True))
            observer.main_contactor_state.publish(Bool(data=True))
            observer.work_light_command.publish(Bool(data=True))
            observer.battery_state.publish(
                BatteryState(percentage=0.8, voltage=51.2)
            )

        _wait_until(
            lambda: observer.estop[-1] is False
            and observer.high_power
            and observer.high_power[-1] is True
            and observer.work_lights
            and observer.work_lights[-1] is True,
            tick=publish_armed_commands,
        )

        _wait_until(
            lambda: observer.high_power[-1] is False
            and observer.status[-1]["emergency_stop_active"] is True,
            timeout=2.0,
        )

        executor.remove_node(source)
        before_product_stall = observer.heartbeat_count
        source_count_before_product_stall = source._safety_publish_count
        _wait_until(
            lambda: source._safety_publish_count
            >= source_count_before_product_stall + 2
            and observer.heartbeat_count >= before_product_stall + 1,
            timeout=2.0,
        )
        assert source._safety_publish_count >= observer.heartbeat_count
        assert source._safety_publish_thread_error is None
        source._stop_safety_publish_loop()
        time.sleep(0.10)
        before_stop = observer.heartbeat_count
        time.sleep(0.15)
        assert observer.heartbeat_count == before_stop
    finally:
        source._stop_safety_publish_loop()
        executor.shutdown()
        thread.join(timeout=2.0)
        source.destroy_node()
        observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_critical_publish_thread_failure_is_supervised_and_fail_closed():
    rclpy.init()
    source = SimulationSafetyInputs()
    source._stop_safety_publish_loop()
    original_relay_publisher = source._relay_pub
    publish_attempts = []

    class FailingPublisher:
        @staticmethod
        def publish(message):
            assert not source._state_lock._is_owned()
            publish_attempts.append(bool(message.data))
            raise RuntimeError("injected_critical_publish_failure")

    try:
        source._relay_pub = FailingPublisher()
        source._safety_publish_stop_event.clear()
        source._safety_publish_thread_error = None
        source._safety_publish_thread = threading.Thread(
            target=source._run_safety_publish_loop,
            daemon=True,
        )
        source._safety_publish_thread.start()
        _wait_until(lambda: source._safety_publish_thread_error is not None)
        source._safety_publish_thread.join(timeout=1.0)
        assert not source._safety_publish_thread.is_alive()
        assert publish_attempts[-1] is False
        with pytest.raises(
            RuntimeError,
            match="simulation_safety_inputs_publish_thread_failed",
        ):
            source._raise_if_safety_publish_failed()
    finally:
        source._relay_pub = original_relay_publisher
        source.destroy_node()
        rclpy.shutdown()


def test_critical_status_failure_is_supervised_and_relay_fails_closed():
    rclpy.init()
    source = SimulationSafetyInputs()
    source._stop_safety_publish_loop()
    original_status_publisher = source._critical_status_pub
    original_relay_publisher = source._relay_pub
    relay_attempts = []

    class FailingStatusPublisher:
        @staticmethod
        def publish(_message):
            assert not source._state_lock._is_owned()
            raise RuntimeError("injected_critical_status_publish_failure")

    class RecordingRelayPublisher:
        @staticmethod
        def publish(message):
            relay_attempts.append(bool(message.data))

    try:
        source._critical_status_pub = FailingStatusPublisher()
        source._relay_pub = RecordingRelayPublisher()
        source._safety_publish_stop_event.clear()
        source._safety_publish_thread_error = None
        source._safety_publish_thread = threading.Thread(
            target=source._run_safety_publish_loop,
            daemon=True,
        )
        source._safety_publish_thread.start()
        _wait_until(lambda: source._safety_publish_thread_error is not None)
        source._safety_publish_thread.join(timeout=1.0)
        assert not source._safety_publish_thread.is_alive()
        assert relay_attempts[-1] is False
        with pytest.raises(
            RuntimeError,
            match="simulation_safety_inputs_publish_thread_failed",
        ):
            source._raise_if_safety_publish_failed()
    finally:
        source._critical_status_pub = original_status_publisher
        source._relay_pub = original_relay_publisher
        source.destroy_node()
        rclpy.shutdown()


def test_critical_publish_thread_join_timeout_is_fatal():
    rclpy.init()
    source = SimulationSafetyInputs()
    source._stop_safety_publish_loop()
    stopped_thread = source._safety_publish_thread

    class StuckThread:
        def __init__(self):
            self.join_timeouts = []

        @staticmethod
        def is_alive():
            return True

        def join(self, timeout=None):
            self.join_timeouts.append(timeout)

    stuck_thread = StuckThread()
    try:
        source._safety_publish_thread = stuck_thread
        with pytest.raises(
            RuntimeError,
            match="simulation_safety_inputs_publish_thread_join_timeout",
        ):
            source._stop_safety_publish_loop(timeout_sec=0.0)
        assert stuck_thread.join_timeouts == [0.0]
    finally:
        source._safety_publish_thread = stopped_thread
        source.destroy_node()
        rclpy.shutdown()
