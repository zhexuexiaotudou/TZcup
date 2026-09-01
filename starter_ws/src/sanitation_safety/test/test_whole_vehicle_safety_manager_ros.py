"""ROS-level unit test for the whole-vehicle actuator command gateway."""

from __future__ import annotations

import json
import threading
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("controller_manager_msgs")
pytest.importorskip("ros_gz_interfaces")

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory

from sanitation_safety.whole_vehicle_safety_manager import WholeVehicleSafetyManager
from sanitation_safety.whole_vehicle_safety_core import (
    SAFETY_FIXED_SAFE_CONTROLLER_POSITIONS,
    SAFETY_HELD_CONTROLLER_JOINTS,
    SafetyReason,
    SafetyState,
)


class Harness(Node):
    def __init__(self) -> None:
        super().__init__("whole_vehicle_safety_manager_test_harness")
        self.estop = True
        self.switch_requests = []
        self.controller_states = {
            "brush_controller": "inactive",
            "recovery_controller": "inactive",
        }
        self.cancel_request_count = 0
        self.brush_outputs = []
        self.pump_outputs = []
        self.permits = []
        self.safety_status_outputs = []
        self.safety_status_json_outputs = []
        self.hold_outputs = {
            "cleaning_controller": [],
            "arm_controller": [],
            "gripper_controller": [],
            "storage_controller": [],
            "service_controller": [],
        }
        self.joint_position_values = [
            0.02, 0.1, -0.2, 0.3, 0.0, 0.0, 0.0, 0.4, 0.0, 1.2
        ]
        self.create_service(
            SwitchController,
            "/controller_manager/switch_controller",
            self._on_switch,
        )
        self.create_service(
            ListControllers,
            "/controller_manager/list_controllers",
            self._on_list_controllers,
        )
        self.cancel_services = [
            self.create_service(
                CancelGoal,
                action + "/_action/cancel_goal",
                self._on_cancel,
            )
            for action in (
                "/cleaning_controller/follow_joint_trajectory",
                "/arm_controller/follow_joint_trajectory",
                "/gripper_controller/follow_joint_trajectory",
                "/storage_controller/follow_joint_trajectory",
            )
        ]
        self.estop_pub = self.create_publisher(Bool, "/emergency_stop", 10)
        self.relay_pub = self.create_publisher(Bool, "/safety/relay_enabled", 10)
        self.bms_fault_pub = self.create_publisher(
            Bool, "/formal_vehicle/power/bms_fault", 10
        )
        self.cleaning_motor_fault_pub = self.create_publisher(
            Bool,
            "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/fault_active",
            10,
        )
        self.traction_permitted_pub = self.create_publisher(
            Bool, "/formal_vehicle/power/traction_permitted", 10
        )
        self.heartbeat_pub = self.create_publisher(
            Empty, "/safety/control_heartbeat", 10
        )
        self.front_pub = self.create_publisher(
            Contacts, "/safety/front_bumper/contact", 10
        )
        self.rear_pub = self.create_publisher(
            Contacts, "/safety/rear_bumper/contact", 10
        )
        self.brush_input = self.create_publisher(
            Float64MultiArray, "/safety/command/brush", 10
        )
        self.pump_input = self.create_publisher(
            Float64MultiArray, "/safety/command/pump", 10
        )
        self.joint_states = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(
            Float64MultiArray,
            "/brush_controller/commands",
            lambda message: self.brush_outputs.append(tuple(message.data)),
            20,
        )
        for controller in self.hold_outputs:
            self.create_subscription(
                JointTrajectory,
                f"/{controller}/joint_trajectory",
                lambda message, name=controller: self.hold_outputs[name].append(
                    (
                        tuple(message.joint_names),
                        tuple(message.points[0].positions),
                    )
                ),
                20,
            )
        self.create_subscription(
            Float64MultiArray,
            "/recovery_controller/commands",
            lambda message: self.pump_outputs.append(tuple(message.data)),
            20,
        )
        self.create_subscription(
            Bool,
            "/safety/actuators_enabled",
            lambda message: self.permits.append(bool(message.data)),
            20,
        )
        self.create_subscription(
            DiagnosticArray,
            "/safety/status",
            lambda message: self.safety_status_outputs.append(
                (time.monotonic(), message)
            ),
            1,
        )
        self.create_subscription(
            String,
            "/safety/status_json",
            lambda message: self.safety_status_json_outputs.append(
                (time.monotonic(), json.loads(message.data))
            ),
            1,
        )
        self.input_timer = self.create_timer(0.05, self._publish_inputs)

    def _on_switch(self, request, response):
        self.switch_requests.append(
            (tuple(request.activate_controllers), tuple(request.deactivate_controllers))
        )
        for name in request.activate_controllers:
            self.controller_states[name] = "active"
        for name in request.deactivate_controllers:
            self.controller_states[name] = "inactive"
        response.ok = True
        return response

    def _on_list_controllers(self, _request, response):
        response.controller = [
            ControllerState(name=name, state=state)
            for name, state in self.controller_states.items()
        ]
        return response

    def _on_cancel(self, _request, response):
        self.cancel_request_count += 1
        response.return_code = CancelGoal.Response.ERROR_NONE
        return response

    def _publish_inputs(self) -> None:
        self.estop_pub.publish(Bool(data=self.estop))
        self.relay_pub.publish(Bool(data=True))
        self.bms_fault_pub.publish(Bool(data=False))
        self.cleaning_motor_fault_pub.publish(Bool(data=False))
        self.traction_permitted_pub.publish(Bool(data=True))
        self.heartbeat_pub.publish(Empty())
        self.front_pub.publish(Contacts())
        self.rear_pub.publish(Contacts())
        self.brush_input.publish(Float64MultiArray(data=[8.0, -8.0, 12.0]))
        self.pump_input.publish(Float64MultiArray(data=[20.0]))
        joint_state = JointState()
        joint_state.name = [
            "cleaning_lift_joint",
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
            "robotiq_85_left_knuckle_joint",
            "dry_deposit_gate_joint",
            "wastewater_drain_valve_joint",
        ]
        joint_state.position = list(self.joint_position_values)
        self.joint_states.publish(joint_state)


def _wait_until(predicate, timeout=3.0, details=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    suffix = "" if details is None else f": {details()}"
    raise AssertionError(f"condition did not become true before timeout{suffix}")


def _prime_healthy_inputs(manager: WholeVehicleSafetyManager) -> None:
    manager._on_manual_estop(Bool(data=False))
    manager._on_front_bumper(Contacts())
    manager._on_rear_bumper(Contacts())
    manager._on_safety_relay(Bool(data=True))
    manager._on_bms_fault(Bool(data=False))
    manager._on_cleaning_motor_fault(Bool(data=False))
    manager._on_traction_permitted(Bool(data=True))
    manager._on_heartbeat(Empty())
    manager._on_command(Twist())
    joint_state = JointState()
    joint_state.name = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    joint_state.position = [-1.0, -1.0, 1.8, -1.5, -1.55, 0.25]
    manager._on_joint_states(joint_state)


def _new_stopped_manager() -> WholeVehicleSafetyManager:
    manager = WholeVehicleSafetyManager()
    manager._stop_publish_loop()
    return manager


def test_periodic_status_is_volatile_while_actuator_permit_is_latched():
    rclpy.init()
    manager = _new_stopped_manager()
    try:
        for publisher in (
            manager._status_publisher,
            manager._status_json_publisher,
        ):
            assert publisher.qos_profile.depth == 1
            assert publisher.qos_profile.reliability is ReliabilityPolicy.RELIABLE
            assert publisher.qos_profile.durability is DurabilityPolicy.VOLATILE
        assert manager._actuator_enable_publisher.qos_profile.depth == 1
        assert (
            manager._actuator_enable_publisher.qos_profile.durability
            is DurabilityPolicy.TRANSIENT_LOCAL
        )
    finally:
        manager.destroy_node()
        rclpy.shutdown()


def test_unsafe_edge_starts_position_cancels_before_periodic_reconciliation():
    rclpy.init()
    harness = Harness()
    harness.input_timer.cancel()
    manager = _new_stopped_manager()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(harness)
    executor.add_node(manager)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        _wait_until(
            lambda: all(
                client.service_is_ready()
                for client in manager._trajectory_cancel_clients.values()
            )
        )
        trigger_positions = {
            joint: float(index) / 100.0
            for index, joint in enumerate(
                joint
                for controller, joints in SAFETY_HELD_CONTROLLER_JOINTS.items()
                for joint in joints
                if joint
                not in SAFETY_FIXED_SAFE_CONTROLLER_POSITIONS.get(controller, {})
            )
        }
        with manager._state_lock:
            manager._joint_positions = dict(trigger_positions)
            manager._unsafe_input_state["manual_estop"] = False
        with manager._controller_lock:
            manager._inhibit_cancel_started = False
            manager._inhibit_cancel_barrier_complete = False
            manager._cancel_futures = {}

        started_at = time.monotonic()
        manager._on_manual_estop(Bool(data=True))
        assert time.monotonic() - started_at <= 0.05
        assert manager._last_evaluation_monotonic == float("-inf")
        assert manager._inhibit_cancel_started is True
        assert set(manager._cancel_futures) == set(
            manager._trajectory_cancel_clients
        )
        assert manager._hold_positions == trigger_positions
        assert manager._hold_inhibited is True
        cancel_futures = dict(manager._cancel_futures)
        assert (
            manager._publish_position_holds(
                inhibited=False, joint_positions=trigger_positions
            )
            is False
        )
        assert manager._cancel_futures == cancel_futures
        assert manager._hold_positions == trigger_positions
        _wait_until(lambda: harness.cancel_request_count == 4)
        _wait_until(
            lambda: all(
                harness.hold_outputs[controller]
                for controller in (
                    "cleaning_controller",
                    "arm_controller",
                    "gripper_controller",
                    "storage_controller",
                )
            )
        )
        assert harness.hold_outputs["cleaning_controller"][-1] == (
            ("cleaning_lift_joint",),
            (trigger_positions["cleaning_lift_joint"],),
        )
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        assert not thread.is_alive()
        manager.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()


def test_ros_gateway_zeros_velocity_actuators_and_switches_trajectory_controllers():
    rclpy.init()
    harness = Harness()
    manager = WholeVehicleSafetyManager()
    manager.set_parameters(
        [Parameter("controller_reassert_period_sec", value=0.05)]
    )
    # Two workers are reserved by the manager in production (input + safety
    # timer); this in-process test harness needs one additional publisher /
    # service worker.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(harness)
    executor.add_node(manager)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        _wait_until(
            lambda: manager._velocity_controller_state_known
            and not manager._velocity_controllers_active
        )
        _wait_until(lambda: harness.cancel_request_count >= 4)
        _wait_until(lambda: all(harness.hold_outputs.values()))
        initial_hold_count = len(harness.hold_outputs["cleaning_controller"])
        harness.joint_position_values[0] = 0.08
        _wait_until(
            lambda: len(harness.hold_outputs["cleaning_controller"])
            > initial_hold_count
        )
        assert harness.hold_outputs["cleaning_controller"][-1][1][0] == 0.02
        assert harness.hold_outputs["service_controller"][-1] == (
            ("wastewater_drain_valve_joint",),
            (0.0,),
        )
        # The launch loader already left these controllers inactive.  The
        # manager confirms that state through ListControllers and therefore
        # does not emit controller_manager warnings by deactivating them twice.
        assert not any(deactivate for _, deactivate in harness.switch_requests)
        _wait_until(lambda: harness.brush_outputs and harness.pump_outputs)
        assert harness.brush_outputs[-1] == (0.0, 0.0, 0.0)
        assert harness.pump_outputs[-1] == (0.0,)
        assert harness.permits[-1] is False
        initial_deactivations = sum(
            bool(deactivate) for _, deactivate in harness.switch_requests
        )
        initial_cancellations = harness.cancel_request_count
        time.sleep(0.3)
        assert (
            sum(bool(deactivate) for _, deactivate in harness.switch_requests)
            == initial_deactivations
        )
        assert harness.cancel_request_count == initial_cancellations

        harness.estop = False
        _wait_until(lambda: any(activate for activate, _ in harness.switch_requests))
        assert {
            name for activate, _ in harness.switch_requests for name in activate
        } == {"brush_controller", "recovery_controller"}
        _wait_until(
            lambda: harness.brush_outputs[-1] == (8.0, -8.0, 12.0)
            and harness.pump_outputs[-1] == (20.0,)
            and harness.permits[-1] is True
        )

        # The harness interleaves ten product-rate 20 Hz inputs. Safety output
        # must remain
        # bounded by the manager's one 20 Hz evaluation timer instead of
        # publishing once for every input callback.
        status_publish_count_before = manager._status_publish_count
        status_json_count_before = len(harness.safety_status_json_outputs)
        time.sleep(0.31)
        assert 1 <= manager._status_publish_count - status_publish_count_before <= 9
        assert 1 <= len(harness.safety_status_json_outputs) - status_json_count_before <= 9
        status_json = harness.safety_status_json_outputs[-1][1]
        assert set(status_json) == {
            "schema_version",
            "state",
            "safety_inputs_permit_actuators",
            "actuators_enabled",
            "managed_controllers_active",
            "active_reasons",
            "unsafe_generation",
            "consumed_unsafe_generation",
            "status_publish_count",
            "maximum_timer_gap_sec",
            "publish_thread_error",
        }
        assert len(
            json.dumps(status_json, sort_keys=True, separators=(",", ":")).encode()
        ) < 1024
        assert status_json["schema_version"] == 1
        assert status_json["state"] == manager._last_decision.state.value
        assert isinstance(status_json["safety_inputs_permit_actuators"], bool)
        assert isinstance(status_json["actuators_enabled"], bool)
        assert isinstance(status_json["managed_controllers_active"], bool)
        assert int(status_json["unsafe_generation"]) == manager._unsafe_generation
        assert int(status_json["consumed_unsafe_generation"]) == (
            manager._consumed_unsafe_generation
        )
        _wait_until(
            lambda: manager._last_decision is not None
            and manager._last_decision.actuators_enabled
            and manager._last_requested_permit is True
            and manager._velocity_controller_state_known
            and manager._velocity_controllers_active
        )

        deactivations_before = sum(
            bool(deactivate) for _, deactivate in harness.switch_requests
        )
        cancellations_before = harness.cancel_request_count
        harness.estop = True
        estop_received_at = time.monotonic()
        immediate_stops_before = manager._immediate_stop_count
        manager._on_manual_estop(Bool(data=True))
        assert time.monotonic() - estop_received_at <= 0.05
        assert manager._immediate_stop_count == immediate_stops_before + 1
        _wait_until(
            lambda: manager._last_evaluation_monotonic >= estop_received_at
            and manager._last_decision is not None
            and manager._last_decision.inputs.manual_estop_active
        )
        unsafe_generation = manager._unsafe_generation
        _wait_until(
            lambda: manager._consumed_unsafe_generation == unsafe_generation
            and manager._last_decision is not None
            and manager._last_decision.state is SafetyState.INHIBITED
            and manager._last_requested_permit is False
        )
        # Full controller reconciliation remains owned by the unique timer.
        # The request may have completed before this test thread observes it,
        # so assert the safety outcome rather than forcing a duplicate switch.
        _wait_until(
            lambda: manager._switch_future is not None
            and manager._switch_future.done()
            and manager._velocity_controller_state_known
            and not manager._velocity_controllers_active,
            details=lambda: {
                "unsafe_generation": manager._unsafe_generation,
                "consumed_unsafe_generation": manager._consumed_unsafe_generation,
                "decision": manager._last_decision.state.value,
                "decision_permit": manager._last_decision.actuators_enabled,
                "last_requested_permit": manager._last_requested_permit,
                "controller_state_known": manager._velocity_controller_state_known,
                "controllers_active": manager._velocity_controllers_active,
                "switch_future_present": manager._switch_future is not None,
                "switch_future_done": (
                    manager._switch_future is not None
                    and manager._switch_future.done()
                ),
                "switch_requests": list(harness.switch_requests),
            },
        )
        assert (
            sum(bool(deactivate) for _, deactivate in harness.switch_requests)
            >= deactivations_before
        )
        _wait_until(
            lambda: harness.brush_outputs[-1] == (0.0, 0.0, 0.0)
            and harness.pump_outputs[-1] == (0.0,)
            and harness.permits[-1] is False
        )
        _wait_until(lambda: harness.cancel_request_count >= cancellations_before + 4)
        _wait_until(
            lambda: manager._cancel_futures
            and all(future.done() for future in manager._cancel_futures.values())
            and manager._position_hold_ready
        )
        for future in manager._cancel_futures.values():
            assert future.result().return_code == CancelGoal.Response.ERROR_NONE
        completed_cancel_count = harness.cancel_request_count
        time.sleep(0.2)
        assert harness.cancel_request_count == completed_cancel_count
    finally:
        harness.input_timer.cancel()
        manager._stop_publish_loop()
        # With both producers stopped, let the executor consume the final
        # depth-one samples before destroying their ROS entities.
        time.sleep(0.1)
        executor.shutdown()
        thread.join(timeout=2.0)
        assert not thread.is_alive()
        manager.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()


def test_input_callbacks_only_mutate_latest_state_until_the_next_publish_cycle():
    rclpy.init()
    manager = WholeVehicleSafetyManager()
    manager._stop_publish_loop()
    publish_calls = []
    immediate_stop_calls = []
    manager._publish = lambda: publish_calls.append(time.monotonic())
    manager._publish_immediate_stop = (
        lambda *_args: immediate_stop_calls.append(time.monotonic())
    )
    try:
        for _ in range(200):
            manager._on_manual_estop(Bool(data=True))
            manager._on_manual_estop(Bool(data=False))
            manager._on_front_bumper(Contacts())
            manager._on_rear_bumper(Contacts())
            manager._on_safety_relay(Bool(data=True))
            manager._on_bms_fault(Bool(data=True))
            manager._on_bms_fault(Bool(data=False))
            manager._on_cleaning_motor_fault(Bool(data=True))
            manager._on_cleaning_motor_fault(Bool(data=False))
            manager._on_traction_permitted(Bool(data=True))
            manager._on_heartbeat(Empty())

        assert publish_calls == []
        assert len(immediate_stop_calls) == 600
        decision = manager._core.evaluate(time.monotonic())
        assert decision.inputs.manual_estop_active is False
        assert decision.inputs.front_bumper_available is True
        assert decision.inputs.rear_bumper_available is True
        assert decision.inputs.safety_relay_enabled is True
        assert decision.inputs.bms_fault_active is False
        assert decision.inputs.cleaning_motor_fault_active is False
        assert decision.inputs.traction_permitted is True
        assert decision.inputs.heartbeat_fresh is True

        # An unsafe latest value is visible to the very next evaluation; no
        # queued publish/reconcile work is needed in the callback itself.
        manager._on_manual_estop(Bool(data=True))
        manager._on_manual_estop(Bool(data=True))
        assert len(immediate_stop_calls) == 601
        assert manager._core.evaluate(time.monotonic()).actuators_enabled is False
    finally:
        manager.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    "publisher_attribute", ["_status_publisher", "_status_json_publisher"]
)
def test_publish_thread_failure_latches_estop_and_is_reported(
    publisher_attribute,
):
    rclpy.init()
    manager = _new_stopped_manager()
    original_status_publisher = getattr(manager, publisher_attribute)

    class FailingPublisher:
        def publish(self, _message):
            assert not manager._state_lock._is_owned()
            raise RuntimeError("injected_status_publish_failure")

    try:
        _prime_healthy_inputs(manager)
        setattr(manager, publisher_attribute, FailingPublisher())
        manager._publish_stop_event.clear()
        manager._publish_thread_error = None
        manager._publish_thread = threading.Thread(
            target=manager._run_publish_loop,
            daemon=True,
        )
        manager._publish_thread.start()

        _wait_until(lambda: manager._publish_thread_error is not None)
        manager._publish_thread.join(timeout=1.0)
        assert not manager._publish_thread.is_alive()
        assert manager._core.manual_estop is True
        assert manager._unsafe_generation > manager._consumed_unsafe_generation
        with pytest.raises(
            RuntimeError,
            match="whole_vehicle_safety_publish_thread_failed",
        ):
            manager._raise_if_publish_failed()
    finally:
        setattr(manager, publisher_attribute, original_status_publisher)
        manager.destroy_node()
        rclpy.shutdown()


def test_immediate_global_stop_revokes_permit_before_zero_commands():
    rclpy.init()
    manager = _new_stopped_manager()
    original_publishers = (
        manager._actuator_enable_publisher,
        manager._base_publisher,
        manager._brush_publisher,
        manager._pump_publisher,
    )
    events = []

    class RecordingPublisher:
        def __init__(self, name):
            self.name = name

        def publish(self, message):
            assert not manager._state_lock._is_owned()
            events.append((self.name, message))

    try:
        manager._actuator_enable_publisher = RecordingPublisher("permit")
        manager._base_publisher = RecordingPublisher("base")
        manager._brush_publisher = RecordingPublisher("brush")
        manager._pump_publisher = RecordingPublisher("pump")

        started = time.monotonic()
        manager._on_manual_estop(Bool(data=True))
        assert time.monotonic() - started <= 0.05
        assert [name for name, _ in events] == [
            "permit",
            "base",
            "brush",
            "pump",
        ]
        assert events[0][1].data is False
        assert tuple(events[2][1].data) == (0.0, 0.0, 0.0)
        assert tuple(events[3][1].data) == (0.0,)
    finally:
        (
            manager._actuator_enable_publisher,
            manager._base_publisher,
            manager._brush_publisher,
            manager._pump_publisher,
        ) = original_publishers
        manager.destroy_node()
        rclpy.shutdown()


def test_manipulator_inhibit_edge_immediately_zeros_only_the_base():
    rclpy.init()
    manager = _new_stopped_manager()
    original_base_publisher = manager._base_publisher
    base_outputs = []

    class RecordingBasePublisher:
        @staticmethod
        def publish(message):
            assert not manager._state_lock._is_owned()
            base_outputs.append(message)

    try:
        manager._base_publisher = RecordingBasePublisher()
        started = time.monotonic()
        manager._on_manipulator_base_inhibit(Bool(data=True))
        assert time.monotonic() - started <= 0.05
        assert len(base_outputs) == 1
        assert manager._immediate_base_stop_count == 1

        # A repeated high sample is not another edge; clearing and raising the
        # inhibit again produces exactly one additional base-only stop.
        manager._on_manipulator_base_inhibit(Bool(data=True))
        assert len(base_outputs) == 1
        manager._on_manipulator_base_inhibit(Bool(data=False))
        manager._on_manipulator_base_inhibit(Bool(data=True))
        assert len(base_outputs) == 2
        assert manager._immediate_base_stop_count == 2
    finally:
        manager._base_publisher = original_base_publisher
        manager.destroy_node()
        rclpy.shutdown()


def test_publish_thread_join_timeout_is_fatal():
    rclpy.init()
    manager = _new_stopped_manager()
    stopped_thread = manager._publish_thread

    class StuckThread:
        def __init__(self):
            self.join_timeouts = []

        def is_alive(self):
            return True

        def join(self, timeout=None):
            self.join_timeouts.append(timeout)

    stuck_thread = StuckThread()
    try:
        manager._publish_thread = stuck_thread
        with pytest.raises(
            RuntimeError,
            match="whole_vehicle_safety_publish_thread_join_timeout",
        ):
            manager._stop_publish_loop(timeout_sec=0.0)
        assert stuck_thread.join_timeouts == [0.0]
    finally:
        manager._publish_thread = stopped_thread
        manager.destroy_node()
        rclpy.shutdown()


def test_short_unsafe_pulse_is_consumed_by_one_periodic_decision():
    rclpy.init()
    manager = _new_stopped_manager()
    manager._publish_immediate_stop = lambda *_args: None
    try:
        _prime_healthy_inputs(manager)
        with manager._state_lock:
            healthy, _ = manager._evaluate_locked(time.monotonic())
        assert healthy.state is SafetyState.ENABLED

        manager._on_bms_fault(Bool(data=True))
        unsafe_generation = manager._unsafe_generation
        manager._on_bms_fault(Bool(data=False))
        assert manager._core.bms_fault_active is False

        with manager._state_lock:
            inhibited, _ = manager._evaluate_locked(time.monotonic())
            consumed_generation = manager._consumed_unsafe_generation
            recovered, _ = manager._evaluate_locked(time.monotonic())

        assert inhibited.state is SafetyState.INHIBITED
        assert inhibited.actuators_enabled is False
        assert SafetyReason.BMS_FAULT_ACTIVE in inhibited.active_reasons
        assert consumed_generation == unsafe_generation
        assert recovered.state is SafetyState.ENABLED
        assert recovered.actuators_enabled is True
    finally:
        manager.destroy_node()
        rclpy.shutdown()


def test_concurrent_unsafe_then_safe_update_preserves_generation_latch():
    rclpy.init()
    manager = _new_stopped_manager()
    manager._publish_immediate_stop = lambda *_args: None
    unsafe_written = threading.Event()
    errors = []

    def write_unsafe():
        try:
            manager._on_cleaning_motor_fault(Bool(data=True))
            unsafe_written.set()
        except BaseException as error:
            errors.append(error)

    def write_safe():
        try:
            assert unsafe_written.wait(timeout=1.0)
            manager._on_cleaning_motor_fault(Bool(data=False))
        except BaseException as error:
            errors.append(error)

    try:
        _prime_healthy_inputs(manager)
        unsafe_thread = threading.Thread(target=write_unsafe)
        safe_thread = threading.Thread(target=write_safe)
        unsafe_thread.start()
        safe_thread.start()
        unsafe_thread.join(timeout=1.0)
        safe_thread.join(timeout=1.0)
        assert not unsafe_thread.is_alive()
        assert not safe_thread.is_alive()
        assert errors == []
        assert manager._core.cleaning_motor_fault_active is False
        unsafe_generation = manager._unsafe_generation

        with manager._state_lock:
            inhibited, _ = manager._evaluate_locked(time.monotonic())
            consumed_generation = manager._consumed_unsafe_generation
            recovered, _ = manager._evaluate_locked(time.monotonic())

        assert inhibited.state is SafetyState.INHIBITED
        assert SafetyReason.CLEANING_MOTOR_FAULT_ACTIVE in inhibited.active_reasons
        assert consumed_generation == unsafe_generation
        assert recovered.state is SafetyState.ENABLED
    finally:
        manager.destroy_node()
        rclpy.shutdown()


def test_switch_completion_and_reconcile_are_serialized_by_controller_lock():
    rclpy.init()
    manager = _new_stopped_manager()
    completion_finished = threading.Event()
    reconcile_finished = threading.Event()

    class CompletedFuture:
        @staticmethod
        def result():
            return SimpleNamespace(ok=True)

    class UnavailableSwitchClient:
        @staticmethod
        def service_is_ready():
            return False

    try:
        with manager._controller_lock:
            completion_thread = threading.Thread(
                target=lambda: (
                    manager._on_switch_complete(CompletedFuture(), True),
                    completion_finished.set(),
                )
            )
            completion_thread.start()
            time.sleep(0.02)
            assert not completion_finished.is_set()
        completion_thread.join(timeout=1.0)
        assert completion_finished.is_set()
        assert manager._velocity_controller_state_known is True
        assert manager._velocity_controllers_active is True

        manager._switch_controller_client = UnavailableSwitchClient()
        manager._last_requested_permit = True
        with manager._controller_lock:
            reconcile_thread = threading.Thread(
                target=lambda: (
                    manager._reconcile_controllers(False, time.monotonic()),
                    reconcile_finished.set(),
                )
            )
            reconcile_thread.start()
            time.sleep(0.02)
            assert not reconcile_finished.is_set()
        reconcile_thread.join(timeout=1.0)
        assert reconcile_finished.is_set()
        assert manager._last_requested_permit is False
        assert manager._velocity_controller_state_known is False
        assert manager._velocity_controllers_active is False
    finally:
        manager.destroy_node()
        rclpy.shutdown()
