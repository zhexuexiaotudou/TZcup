"""ROS 2 gateway for the fail-closed whole-vehicle safety state machine."""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import replace

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .whole_vehicle_safety_core import ACTUATOR_CHANNELS
from .whole_vehicle_safety_core import SAFETY_FIXED_SAFE_CONTROLLER_POSITIONS
from .whole_vehicle_safety_core import SAFETY_HELD_CONTROLLER_JOINTS
from .whole_vehicle_safety_core import SAFETY_NATIVE_CANCEL_HOLD_CONTROLLERS
from .whole_vehicle_safety_core import SAFETY_SWITCHED_CONTROLLERS
from .whole_vehicle_safety_core import (
    SafeCommand,
    SafetyDecision,
    SafetyReason,
    SafetyState,
)
from .whole_vehicle_safety_core import VelocityActuatorGate
from .whole_vehicle_safety_core import WholeVehicleSafetyCore
from .dry_speed_qualification_core import DrySpeedQualificationState


ARM_STOWED_POSITIONS = {
    "shoulder_pan_joint": -1.0,
    "shoulder_lift_joint": -1.0,
    "elbow_joint": 1.8,
    "wrist_1_joint": -1.5,
    "wrist_2_joint": -1.55,
    "wrist_3_joint": 0.25,
}


class WholeVehicleSafetyManager(Node):
    """Own the formal vehicle's safe base command and global actuator permit."""

    def __init__(self) -> None:
        super().__init__("whole_vehicle_safety_manager")
        self._state_lock = threading.RLock()
        self._output_lock = threading.RLock()
        self._controller_lock = threading.RLock()
        self._timer_callback_group = MutuallyExclusiveCallbackGroup()
        # Dangerous input edges must not queue behind the normal command,
        # joint-state, and heartbeat streams in the node's default mutually
        # exclusive callback group.  A dedicated worker can therefore start
        # cancellation and position holds while the normal input worker is
        # busy draining high-rate telemetry.
        self._unsafe_input_callback_group = MutuallyExclusiveCallbackGroup()
        self._declare_parameters()
        self._publish_period_sec = self._float_parameter("publish_period_sec")
        self._arm_stowed_tolerance_rad = self._float_parameter(
            "arm_stowed_tolerance_rad"
        )
        self._core = WholeVehicleSafetyCore(
            command_timeout_sec=self._float_parameter("command_timeout_sec"),
            heartbeat_timeout_sec=self._float_parameter("heartbeat_timeout_sec"),
            bumper_timeout_sec=self._float_parameter("bumper_timeout_sec"),
            safety_relay_timeout_sec=self._float_parameter(
                "safety_relay_timeout_sec"
            ),
            bms_fault_timeout_sec=self._float_parameter("bms_fault_timeout_sec"),
            cleaning_motor_fault_timeout_sec=self._float_parameter(
                "cleaning_motor_fault_timeout_sec"
            ),
            traction_permit_timeout_sec=self._float_parameter(
                "traction_permit_timeout_sec"
            ),
            max_linear_velocity=self._float_parameter("max_linear_velocity"),
            max_angular_velocity=self._float_parameter("max_angular_velocity"),
        )
        self._base_frame_id = self._string_parameter("base_frame_id")
        actuator_timeout = self._float_parameter("actuator_command_timeout_sec")
        self._brush_gate = VelocityActuatorGate(3, actuator_timeout)
        self._pump_gate = VelocityActuatorGate(1, actuator_timeout)
        self._dry_speed_qualification = DrySpeedQualificationState(
            configured_max_linear_velocity_mps=self._float_parameter("max_linear_velocity"),
            mission_mode=self._string_parameter("mission_mode"),
            operation_speed_profile=self._string_parameter("operation_speed_profile"),
            qualification_state=self._string_parameter("speed_qualification_state"),
            heartbeat_timeout_sec=self._float_parameter("speed_qualification_heartbeat_timeout_sec"),
        )
        self._velocity_controllers_active = False
        self._velocity_controller_state_known = False
        self._controller_state_future = None
        self._last_controller_state_query_monotonic = float("-inf")
        self._switch_future = None
        self._last_controller_request_monotonic = float("-inf")
        self._last_requested_permit = None
        self._cancel_futures = {}
        self._inhibit_cancel_started = False
        self._inhibit_cancel_barrier_complete = False
        self._joint_positions = {}
        self._hold_positions = {}
        self._hold_inhibited = False
        self._position_hold_ready = False
        self._external_base_motion_inhibited = False
        self._last_evaluation_monotonic = float("-inf")
        self._last_decision: SafetyDecision | None = None
        self._input_arrival_monotonic: dict[str, float] = {}
        self._unsafe_input_state: dict[str, bool] = {}
        self._unsafe_generation = 0
        self._consumed_unsafe_generation = 0
        self._latched_unsafe_reasons: set[SafetyReason] = set()
        self._callback_count = 0
        self._immediate_stop_count = 0
        self._immediate_base_stop_count = 0
        self._status_publish_count = 0
        self._callback_rate_hz = 0.0
        self._status_rate_hz = 0.0
        self._rate_window_started_monotonic = time.monotonic()
        self._rate_window_callback_count = 0
        self._rate_window_status_count = 0
        self._last_timer_started_monotonic: float | None = None
        self._timer_overrun_count = 0
        self._maximum_timer_gap_sec = 0.0
        self._publish_thread_error: BaseException | None = None

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Periodic safety inputs are latest-state streams.  Keeping only one
        # sample prevents a briefly starved executor from replaying an obsolete
        # queue after it resumes.  Sensor bridges may publish BEST_EFFORT,
        # while the product power and heartbeat publishers are RELIABLE.
        latest_reliable_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latest_sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._base_publisher = self.create_publisher(
            TwistStamped,
            self._string_parameter("base_command_output_topic"),
            10,
        )
        self._actuator_enable_publisher = self.create_publisher(
            Bool,
            self._string_parameter("actuator_enable_topic"),
            latched_qos,
        )
        self._brush_publisher = self.create_publisher(
            Float64MultiArray,
            self._string_parameter("brush_command_output_topic"),
            10,
        )
        self._pump_publisher = self.create_publisher(
            Float64MultiArray,
            self._string_parameter("pump_command_output_topic"),
            10,
        )
        self._hold_publishers = {
            controller: self.create_publisher(
                JointTrajectory,
                f"/{controller}/joint_trajectory",
                10,
            )
            for controller in SAFETY_HELD_CONTROLLER_JOINTS
        }
        self._status_publisher = self.create_publisher(
            DiagnosticArray,
            self._string_parameter("status_topic"),
            latest_reliable_qos,
        )
        self._status_json_publisher = self.create_publisher(
            String,
            self._string_parameter("status_json_topic"),
            latest_reliable_qos,
        )

        self.create_subscription(
            Twist,
            self._string_parameter("command_input_topic"),
            self._on_command,
            latest_reliable_qos,
        )
        self.create_subscription(
            Float64MultiArray,
            self._string_parameter("brush_command_input_topic"),
            self._on_brush_command,
            latest_reliable_qos,
        )
        self.create_subscription(
            Bool,
            self._string_parameter("speed_qualification_active_topic"),
            self._on_speed_qualification_active,
            latest_reliable_qos,
        )
        self.create_subscription(
            Bool,
            self._string_parameter("dry_brush_active_topic"),
            self._on_dry_brush_active,
            latest_reliable_qos,
        )
        self.create_subscription(
            Float64MultiArray,
            self._string_parameter("pump_command_input_topic"),
            self._on_pump_command,
            latest_reliable_qos,
        )
        self.create_subscription(
            JointState,
            self._string_parameter("joint_states_topic"),
            self._on_joint_states,
            latest_reliable_qos,
        )
        self.create_subscription(
            Bool,
            self._string_parameter("manual_estop_topic"),
            self._on_manual_estop,
            latest_reliable_qos,
            callback_group=self._unsafe_input_callback_group,
        )
        self.create_subscription(
            Bool,
            self._string_parameter("manipulator_base_inhibit_topic"),
            self._on_manipulator_base_inhibit,
            latest_reliable_qos,
            callback_group=self._unsafe_input_callback_group,
        )
        self._switch_controller_client = self.create_client(
            SwitchController,
            self._string_parameter("switch_controller_service"),
            callback_group=self._timer_callback_group,
        )
        self._list_controllers_client = self.create_client(
            ListControllers,
            self._string_parameter("list_controllers_service"),
            callback_group=self._timer_callback_group,
        )
        trajectory_actions = {
            self._string_parameter("cleaning_trajectory_action"): "cleaning_controller",
            self._string_parameter("arm_trajectory_action"): "arm_controller",
            self._string_parameter("gripper_trajectory_action"): "gripper_controller",
            self._string_parameter("storage_trajectory_action"): "storage_controller",
        }
        self._trajectory_hold_controller_by_action = dict(trajectory_actions)
        self._trajectory_cancel_clients = {
            action_name: self.create_client(
                CancelGoal,
                action_name + "/_action/cancel_goal",
                callback_group=self._timer_callback_group,
            )
            for action_name in trajectory_actions
        }
        self.create_subscription(
            Contacts,
            self._string_parameter("front_bumper_topic"),
            self._on_front_bumper,
            latest_sensor_qos,
            callback_group=self._unsafe_input_callback_group,
        )
        self.create_subscription(
            Contacts,
            self._string_parameter("rear_bumper_topic"),
            self._on_rear_bumper,
            latest_sensor_qos,
            callback_group=self._unsafe_input_callback_group,
        )
        self.create_subscription(
            Bool,
            self._string_parameter("safety_relay_topic"),
            self._on_safety_relay,
            latest_reliable_qos,
            callback_group=self._unsafe_input_callback_group,
        )
        self.create_subscription(
            Bool,
            self._string_parameter("bms_fault_topic"),
            self._on_bms_fault,
            latest_reliable_qos,
            callback_group=self._unsafe_input_callback_group,
        )
        self.create_subscription(
            Bool,
            self._string_parameter("cleaning_motor_fault_topic"),
            self._on_cleaning_motor_fault,
            latest_sensor_qos,
            callback_group=self._unsafe_input_callback_group,
        )
        self.create_subscription(
            Bool,
            self._string_parameter("traction_permitted_topic"),
            self._on_traction_permitted,
            latest_reliable_qos,
            callback_group=self._unsafe_input_callback_group,
        )
        self.create_subscription(
            Empty,
            self._string_parameter("heartbeat_topic"),
            self._on_heartbeat,
            latest_reliable_qos,
        )
        self._publish_stop_event = threading.Event()
        self._publish_thread = threading.Thread(
            target=self._run_publish_loop,
            name="whole_vehicle_safety_publish",
            daemon=True,
        )
        self._publish_thread.start()

    def _declare_parameters(self) -> None:
        self.declare_parameter("command_input_topic", "/cmd_vel_gate")
        self.declare_parameter(
            "base_command_output_topic", "/base_controller/cmd_vel"
        )
        self.declare_parameter("manual_estop_topic", "/emergency_stop")
        self.declare_parameter(
            "manipulator_base_inhibit_topic",
            "/manipulation/base_motion_inhibited",
        )
        self.declare_parameter("arm_stowed_tolerance_rad", 0.08)
        self.declare_parameter(
            "front_bumper_topic", "/safety/front_bumper/contact"
        )
        self.declare_parameter(
            "rear_bumper_topic", "/safety/rear_bumper/contact"
        )
        self.declare_parameter("safety_relay_topic", "/safety/relay_enabled")
        self.declare_parameter("bms_fault_topic", "/formal_vehicle/power/bms_fault")
        self.declare_parameter(
            "cleaning_motor_fault_topic",
            "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/fault_active",
        )
        self.declare_parameter(
            "traction_permitted_topic", "/formal_vehicle/power/traction_permitted"
        )
        self.declare_parameter("heartbeat_topic", "/safety/control_heartbeat")
        self.declare_parameter(
            "actuator_enable_topic", "/safety/actuators_enabled"
        )
        self.declare_parameter(
            "brush_command_input_topic", "/safety/command/brush"
        )
        self.declare_parameter(
            "brush_command_output_topic", "/brush_controller/commands"
        )
        self.declare_parameter(
            "pump_command_input_topic", "/safety/command/pump"
        )
        self.declare_parameter(
            "pump_command_output_topic", "/recovery_controller/commands"
        )
        self.declare_parameter(
            "switch_controller_service", "/controller_manager/switch_controller"
        )
        self.declare_parameter(
            "list_controllers_service", "/controller_manager/list_controllers"
        )
        self.declare_parameter(
            "cleaning_trajectory_action",
            "/cleaning_controller/follow_joint_trajectory",
        )
        self.declare_parameter(
            "arm_trajectory_action", "/arm_controller/follow_joint_trajectory"
        )
        self.declare_parameter(
            "gripper_trajectory_action",
            "/gripper_controller/follow_joint_trajectory",
        )
        self.declare_parameter(
            "storage_trajectory_action",
            "/storage_controller/follow_joint_trajectory",
        )
        self.declare_parameter("status_topic", "/safety/status")
        self.declare_parameter("status_json_topic", "/safety/status_json")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("heartbeat_timeout_sec", 0.5)
        self.declare_parameter("bumper_timeout_sec", 0.5)
        self.declare_parameter("safety_relay_timeout_sec", 0.5)
        self.declare_parameter("bms_fault_timeout_sec", 0.5)
        self.declare_parameter("cleaning_motor_fault_timeout_sec", 0.25)
        self.declare_parameter("traction_permit_timeout_sec", 0.5)
        self.declare_parameter("publish_period_sec", 0.05)
        self.declare_parameter("actuator_command_timeout_sec", 0.5)
        self.declare_parameter("controller_reassert_period_sec", 0.1)
        self.declare_parameter("max_linear_velocity", 0.45)
        self.declare_parameter("max_angular_velocity", 0.35)
        self.declare_parameter("mission_mode", "")
        self.declare_parameter("operation_speed_profile", "")
        self.declare_parameter("speed_qualification_state", "none")
        self.declare_parameter("speed_qualification_active_topic", "/safety/dry_cleaning_qualification_active")
        self.declare_parameter("dry_brush_active_topic", "/brush_enabled")
        self.declare_parameter("speed_qualification_heartbeat_timeout_sec", 0.25)

    def _on_command(self, message: Twist) -> None:
        with self._state_lock:
            now = self._record_input_arrival("command")
            self._refresh_effective_max_linear_velocity_locked(now)
            self._core.set_command(
                linear_x=float(message.linear.x),
                angular_z=float(message.angular.z),
                now=now,
            )

    def _on_brush_command(self, message: Float64MultiArray) -> None:
        with self._state_lock:
            now = self._record_input_arrival("brush_command")
            self._brush_gate.set_command(message.data, now)
            self._refresh_effective_max_linear_velocity_locked(now)

    def _on_pump_command(self, message: Float64MultiArray) -> None:
        with self._state_lock:
            now = self._record_input_arrival("pump_command")
            self._pump_gate.set_command(message.data, now)
            self._refresh_effective_max_linear_velocity_locked(now)

    def _on_speed_qualification_active(self, message: Bool) -> None:
        with self._state_lock:
            now = self._record_input_arrival("speed_qualification_active")
            self._dry_speed_qualification.set_qualification_active(message.data, now)
            self._refresh_effective_max_linear_velocity_locked(now)

    def _on_dry_brush_active(self, message: Bool) -> None:
        with self._state_lock:
            now = self._record_input_arrival("dry_brush_active")
            self._dry_speed_qualification.set_dry_brush_active(message.data, now)
            self._refresh_effective_max_linear_velocity_locked(now)

    def _refresh_effective_max_linear_velocity_locked(self, now: float) -> float:
        cap = self._dry_speed_qualification.effective_max_linear_velocity_mps(
            now=now,
            pump_output=self._pump_gate.evaluate(permitted=True, now=now),
        )
        self._core.set_effective_max_linear_velocity(cap)
        return cap

    def _on_joint_states(self, message: JointState) -> None:
        with self._state_lock:
            self._record_input_arrival("joint_states")
            self._joint_positions.update(
                {
                    name: float(position)
                    for name, position in zip(message.name, message.position)
                    if math.isfinite(position)
                }
            )

    def _on_manipulator_base_inhibit(self, message: Bool) -> None:
        with self._state_lock:
            self._record_input_arrival("manipulator_base_inhibit")
            inhibited = bool(message.data)
            new_inhibit_edge = (
                inhibited and not self._external_base_motion_inhibited
            )
            self._external_base_motion_inhibited = inhibited
        if new_inhibit_edge:
            self._publish_immediate_base_stop()

    def _arm_is_stowed(self) -> bool:
        return all(
            name in self._joint_positions
            and abs(self._joint_positions[name] - expected)
            <= self._arm_stowed_tolerance_rad
            for name, expected in ARM_STOWED_POSITIONS.items()
        )

    def _on_manual_estop(self, message: Bool) -> None:
        with self._state_lock:
            self._record_input_arrival("manual_estop")
            self._core.set_manual_estop(message.data)
            unsafe_edge = self._is_new_unsafe_edge(
                "manual_estop", bool(message.data), SafetyReason.MANUAL_ESTOP
            )
            trigger_joint_positions = dict(self._joint_positions)
        if unsafe_edge:
            self._stop_on_unsafe_edge(trigger_joint_positions)

    def _on_front_bumper(self, message: Contacts) -> None:
        with self._state_lock:
            now = self._record_input_arrival("front_bumper")
            contact = bool(message.contacts)
            self._core.set_front_bumper(contact, now)
            unsafe_edge = self._is_new_unsafe_edge(
                "front_bumper", contact, SafetyReason.FRONT_BUMPER_CONTACT
            )
            trigger_joint_positions = dict(self._joint_positions)
        if unsafe_edge:
            self._stop_on_unsafe_edge(trigger_joint_positions)

    def _on_rear_bumper(self, message: Contacts) -> None:
        with self._state_lock:
            now = self._record_input_arrival("rear_bumper")
            contact = bool(message.contacts)
            self._core.set_rear_bumper(contact, now)
            unsafe_edge = self._is_new_unsafe_edge(
                "rear_bumper", contact, SafetyReason.REAR_BUMPER_CONTACT
            )
            trigger_joint_positions = dict(self._joint_positions)
        if unsafe_edge:
            self._stop_on_unsafe_edge(trigger_joint_positions)

    def _on_safety_relay(self, message: Bool) -> None:
        with self._state_lock:
            now = self._record_input_arrival("safety_relay")
            self._core.set_safety_relay(message.data, now)
            unsafe_edge = self._is_new_unsafe_edge(
                "safety_relay",
                not bool(message.data),
                SafetyReason.SAFETY_RELAY_DISABLED,
            )
            trigger_joint_positions = dict(self._joint_positions)
        if unsafe_edge:
            self._stop_on_unsafe_edge(trigger_joint_positions)

    def _on_bms_fault(self, message: Bool) -> None:
        with self._state_lock:
            now = self._record_input_arrival("bms_fault")
            self._core.set_bms_fault(message.data, now)
            unsafe_edge = self._is_new_unsafe_edge(
                "bms_fault", bool(message.data), SafetyReason.BMS_FAULT_ACTIVE
            )
            trigger_joint_positions = dict(self._joint_positions)
        if unsafe_edge:
            self._stop_on_unsafe_edge(trigger_joint_positions)

    def _on_cleaning_motor_fault(self, message: Bool) -> None:
        with self._state_lock:
            now = self._record_input_arrival("cleaning_motor_fault")
            self._core.set_cleaning_motor_fault(message.data, now)
            unsafe_edge = self._is_new_unsafe_edge(
                "cleaning_motor_fault",
                bool(message.data),
                SafetyReason.CLEANING_MOTOR_FAULT_ACTIVE,
            )
            trigger_joint_positions = dict(self._joint_positions)
        if unsafe_edge:
            self._stop_on_unsafe_edge(trigger_joint_positions)

    def _on_traction_permitted(self, message: Bool) -> None:
        with self._state_lock:
            now = self._record_input_arrival("traction_permitted")
            self._core.set_traction_permitted(message.data, now)
            unsafe_edge = self._is_new_unsafe_edge(
                "traction_permitted",
                not bool(message.data),
                SafetyReason.TRACTION_NOT_PERMITTED,
            )
            trigger_joint_positions = dict(self._joint_positions)
        if unsafe_edge:
            self._stop_on_unsafe_edge(trigger_joint_positions)

    def _on_heartbeat(self, _message: Empty) -> None:
        with self._state_lock:
            now = self._record_input_arrival("heartbeat")
            self._core.heartbeat(now)

    def _record_input_arrival(self, name: str) -> float:
        now = time.monotonic()
        self._input_arrival_monotonic[name] = now
        self._callback_count += 1
        self._rate_window_callback_count += 1
        return now

    def _is_new_unsafe_edge(
        self, name: str, unsafe: bool, reason: SafetyReason
    ) -> bool:
        was_unsafe = self._unsafe_input_state.get(name, False)
        self._unsafe_input_state[name] = unsafe
        new_edge = unsafe and not was_unsafe
        if new_edge:
            self._unsafe_generation += 1
            self._latched_unsafe_reasons.add(reason)
        return new_edge

    def _stop_on_unsafe_edge(
        self, trigger_joint_positions: dict[str, float]
    ) -> None:
        """Start trajectory cancellation before any congested output can delay it."""

        # Do not wait for the periodic 20 Hz reconciliation loop.  In a loaded
        # graph its output lock can be occupied by reliable status delivery;
        # initiating the non-blocking cancel requests first bounds continued
        # position motion at the actual dangerous input edge.
        with self._controller_lock:
            new_cancel_futures = self._cancel_trajectory_goals()
        # A CancelGoal response only acknowledges the request; it does not
        # guarantee that the trajectory controller has already stopped
        # consuming its active trajectory.  Publish an edge-position command
        # immediately after dispatching cancellation so the controller topic
        # preempts the remaining motion instead of waiting for every action
        # server's cancellation response.  The completion callbacks and the
        # periodic inhibited loop deliberately repeat these holds.
        self._publish_trigger_position_holds(trigger_joint_positions)
        for action_name, future in new_cancel_futures.items():
            controller = self._trajectory_hold_controller_by_action[action_name]
            future.add_done_callback(
                lambda completed,
                controller=controller,
                positions=dict(trigger_joint_positions): (
                    self._publish_edge_controller_hold(
                        completed,
                        controller=controller,
                        trigger_joint_positions=positions,
                    )
                )
            )
        self._publish_immediate_stop(trigger_joint_positions)

    def _publish_trigger_position_holds(
        self, trigger_joint_positions: dict[str, float]
    ) -> None:
        """Preempt position motion at the dangerous input edge."""

        for controller, joints in SAFETY_HELD_CONTROLLER_JOINTS.items():
            if controller in SAFETY_NATIVE_CANCEL_HOLD_CONTROLLERS:
                continue
            fixed_positions = SAFETY_FIXED_SAFE_CONTROLLER_POSITIONS.get(
                controller, {}
            )
            if any(
                joint not in fixed_positions
                and joint not in trigger_joint_positions
                for joint in joints
            ):
                continue
            self._publish_controller_hold(
                controller,
                joints,
                {
                    joint: fixed_positions.get(
                        joint, trigger_joint_positions.get(joint, 0.0)
                    )
                    for joint in joints
                },
            )

    def _publish_edge_controller_hold(
        self,
        future,
        *,
        controller: str,
        trigger_joint_positions: dict[str, float],
    ) -> None:
        """Hold one controller as soon as its own cancellation is acknowledged."""

        if controller in SAFETY_NATIVE_CANCEL_HOLD_CONTROLLERS:
            return

        try:
            if future.result().return_code != CancelGoal.Response.ERROR_NONE:
                return
        except BaseException:
            return
        joints = SAFETY_HELD_CONTROLLER_JOINTS[controller]
        fixed_positions = SAFETY_FIXED_SAFE_CONTROLLER_POSITIONS.get(controller, {})
        if any(
            joint not in fixed_positions and joint not in trigger_joint_positions
            for joint in joints
        ):
            return
        self._publish_controller_hold(
            controller,
            joints,
            {
                joint: fixed_positions.get(
                    joint, trigger_joint_positions.get(joint, 0.0)
                )
                for joint in joints
            },
        )

    def _publish_immediate_stop(
        self, trigger_joint_positions: dict[str, float] | None = None
    ) -> None:
        """Zero motion immediately on a dangerous edge without heavy work."""

        with self._output_lock:
            if trigger_joint_positions is not None:
                # The output lock serializes this capture after any already
                # running permitted publish cycle.  Preserve the dangerous
                # edge position instead of a later post-cancel position so the
                # hold command cannot ratchet a moving mechanism onward.
                self._hold_positions = {
                    joint: trigger_joint_positions[joint]
                    for controller, joints in SAFETY_HELD_CONTROLLER_JOINTS.items()
                    for joint in joints
                    if joint
                    not in SAFETY_FIXED_SAFE_CONTROLLER_POSITIONS.get(
                        controller, {}
                    )
                    and joint in trigger_joint_positions
                }
                required = {
                    joint
                    for controller, joints in SAFETY_HELD_CONTROLLER_JOINTS.items()
                    for joint in joints
                    if joint
                    not in SAFETY_FIXED_SAFE_CONTROLLER_POSITIONS.get(
                        controller, {}
                    )
                }
                self._hold_inhibited = required <= set(self._hold_positions)
            with self._state_lock:
                self._immediate_stop_count += 1
            # Revoke the global permit before emitting individual zero
            # commands, so downstream actuators fail closed even if a later
            # publisher blocks or fails.
            self._actuator_enable_publisher.publish(Bool(data=False))
            stamp = self.get_clock().now().to_msg()
            command = TwistStamped()
            command.header.stamp = stamp
            command.header.frame_id = self._base_frame_id
            self._base_publisher.publish(command)
            self._brush_publisher.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
            self._pump_publisher.publish(Float64MultiArray(data=[0.0]))

    def _publish_immediate_base_stop(self) -> None:
        """Zero only the base on a manipulator-motion inhibit edge."""

        with self._output_lock:
            with self._state_lock:
                self._immediate_base_stop_count += 1
            stamp = self.get_clock().now().to_msg()
            command = TwistStamped()
            command.header.stamp = stamp
            command.header.frame_id = self._base_frame_id
            self._base_publisher.publish(command)

    def _publish(self) -> None:
        with self._output_lock:
            now = time.monotonic()
            with self._state_lock:
                effective_max_linear_velocity = self._refresh_effective_max_linear_velocity_locked(now)
                decision, joint_positions = self._evaluate_locked(now)

            # No state lock crosses a ROS service, clock, or publisher call.
            self._reconcile_controllers(decision.actuators_enabled, now)
            with self._controller_lock:
                controllers_active = self._velocity_controllers_active
            self._position_hold_ready = self._publish_position_holds(
                inhibited=not decision.actuators_enabled,
                joint_positions=joint_positions,
            )
            effective_permit = decision.actuators_enabled and controllers_active
            with self._state_lock:
                brush_output = list(
                    self._brush_gate.evaluate(permitted=effective_permit, now=now)
                )
                pump_output = list(
                    self._pump_gate.evaluate(permitted=effective_permit, now=now)
                )
                runtime_metrics = self._runtime_metrics(now)

            stamp = self.get_clock().now().to_msg()
            command = TwistStamped()
            command.header.stamp = stamp
            command.header.frame_id = self._base_frame_id
            command.twist.linear.x = decision.command.linear_x
            command.twist.angular.z = decision.command.angular_z
            self._base_publisher.publish(command)
            self._brush_publisher.publish(Float64MultiArray(data=brush_output))
            self._pump_publisher.publish(Float64MultiArray(data=pump_output))
            self._actuator_enable_publisher.publish(Bool(data=effective_permit))
            status_json_payload = {
                "schema_version": 1,
                "state": decision.state.value,
                "safety_inputs_permit_actuators": decision.actuators_enabled,
                "actuators_enabled": effective_permit,
                "managed_controllers_active": controllers_active,
                "active_reasons": ",".join(
                    reason.value for reason in decision.active_reasons
                ),
                "unsafe_generation": int(runtime_metrics["unsafe_generation"]),
                "consumed_unsafe_generation": int(
                    runtime_metrics["consumed_unsafe_generation"]
                ),
                "status_publish_count": int(
                    runtime_metrics["status_publish_count"]
                )
                + 1,
                "maximum_timer_gap_sec": float(
                    runtime_metrics["maximum_timer_gap_sec"]
                ),
                "publish_thread_error": str(
                    runtime_metrics["publish_thread_error"]
                ),
                "effective_max_linear_velocity_mps": effective_max_linear_velocity,
                "operation_speed_profile": self._string_parameter("operation_speed_profile"),
                "speed_qualification_state": self._string_parameter("speed_qualification_state"),
            }
            # Keep the machine acceptance stream below one ordinary Ethernet
            # MTU and emit it before the larger operator DiagnosticArray.
            # Copying every Diagnostic KeyValue here made both status samples
            # require fragmentation in the full graph and left supervision
            # blind even though the 20 Hz loop itself remained healthy.
            self._status_json_publisher.publish(
                String(
                    data=json.dumps(
                        status_json_payload,
                        sort_keys=True,
                        separators=(",", ":"),
                    )
                )
            )
            diagnostic = self._diagnostic(
                decision,
                stamp,
                controllers_active,
                self._position_hold_ready,
                runtime_metrics=runtime_metrics,
            )
            self._status_publisher.publish(diagnostic)
            with self._state_lock:
                self._status_publish_count += 1
                self._rate_window_status_count += 1

    def _run_publish_loop(self) -> None:
        try:
            period = self._publish_period_sec
            deadline = time.monotonic() + period
            while not self._publish_stop_event.is_set():
                remaining = deadline - time.monotonic()
                if self._publish_stop_event.wait(max(0.0, remaining)):
                    return
                self._publish()
                deadline += period
                now = time.monotonic()
                if deadline <= now:
                    deadline = now + period
        except BaseException as error:  # supervised by the main executor loop
            with self._state_lock:
                self._publish_thread_error = error
                self._core.set_manual_estop(True)
                self._unsafe_generation += 1
                self._latched_unsafe_reasons.add(SafetyReason.MANUAL_ESTOP)
            try:
                self._publish_immediate_stop()
            except BaseException:
                # A publisher failure may also prevent the final zero sample;
                # the main loop still exits non-zero instead of claiming health.
                pass

    def _stop_publish_loop(self, timeout_sec: float = 2.0) -> None:
        self._publish_stop_event.set()
        if self._publish_thread.is_alive():
            self._publish_thread.join(timeout=timeout_sec)
        if self._publish_thread.is_alive():
            raise RuntimeError("whole_vehicle_safety_publish_thread_join_timeout")

    def _raise_if_publish_failed(self) -> None:
        with self._state_lock:
            error = self._publish_thread_error
        if error is not None:
            raise RuntimeError("whole_vehicle_safety_publish_thread_failed") from error

    def destroy_node(self):
        self._stop_publish_loop()
        return super().destroy_node()

    def _evaluate_locked(
        self, now: float
    ) -> tuple[SafetyDecision, dict[str, float]]:
        publish_period = self._publish_period_sec
        if self._last_timer_started_monotonic is not None:
            timer_gap = now - self._last_timer_started_monotonic
            self._maximum_timer_gap_sec = max(self._maximum_timer_gap_sec, timer_gap)
            if timer_gap > publish_period * 1.5:
                self._timer_overrun_count += 1
        self._last_timer_started_monotonic = now
        rate_window_elapsed = now - self._rate_window_started_monotonic
        if rate_window_elapsed >= 1.0:
            self._callback_rate_hz = (
                self._rate_window_callback_count / rate_window_elapsed
            )
            self._status_rate_hz = self._rate_window_status_count / rate_window_elapsed
            self._rate_window_started_monotonic = now
            self._rate_window_callback_count = 0
            self._rate_window_status_count = 0
        self._core.set_base_motion_inhibited(
            self._external_base_motion_inhibited or not self._arm_is_stowed()
        )
        decision = self._core.evaluate(now)
        if self._consumed_unsafe_generation < self._unsafe_generation:
            reasons = tuple(
                dict.fromkeys((*decision.active_reasons, *self._latched_unsafe_reasons))
            )
            decision = replace(
                decision,
                state=SafetyState.INHIBITED,
                command=SafeCommand(),
                actuators_enabled=False,
                base_command_enabled=False,
                active_reasons=reasons,
            )
            self._consumed_unsafe_generation = self._unsafe_generation
            self._latched_unsafe_reasons.clear()
        self._last_evaluation_monotonic = now
        self._last_decision = decision
        return decision, dict(self._joint_positions)

    def _runtime_metrics(self, now: float) -> dict[str, float | int | str]:
        metrics: dict[str, float | int | str] = {
            "callback_count": self._callback_count,
            "callback_rate_hz": self._callback_rate_hz,
            "status_publish_count": self._status_publish_count,
            "status_rate_hz": self._status_rate_hz,
            "immediate_stop_count": self._immediate_stop_count,
            "immediate_base_stop_count": self._immediate_base_stop_count,
            "timer_overrun_count": self._timer_overrun_count,
            "maximum_timer_gap_sec": self._maximum_timer_gap_sec,
            "publish_thread_error": (
                "none"
                if self._publish_thread_error is None
                else type(self._publish_thread_error).__name__
            ),
            "unsafe_generation": self._unsafe_generation,
            "consumed_unsafe_generation": self._consumed_unsafe_generation,
        }
        for name, arrival in self._input_arrival_monotonic.items():
            metrics[f"{name}_arrival_age_sec"] = max(0.0, now - arrival)
        return metrics

    def _reconcile_controllers(self, permit: bool, now: float) -> None:
        """Converge controller state, retrying only while it is unconfirmed."""

        with self._controller_lock:
            self._reconcile_controllers_locked(permit, now)

    def _reconcile_controllers_locked(self, permit: bool, now: float) -> None:

        reassert_due = (
            now - self._last_controller_request_monotonic
            >= self._float_parameter("controller_reassert_period_sec")
        )
        permit_changed = permit != self._last_requested_permit
        self._last_requested_permit = permit
        if not permit:
            # A cancel service may not be discovered on the exact inhibit
            # edge. Retry only missing or failed requests until every action
            # server has acknowledged cancellation; completed successes are
            # never sent twice during the same inhibit episode.
            self._cancel_trajectory_goals()
        if self._switch_future is not None and not self._switch_future.done():
            return
        query_requires_switch = False
        if (
            not self._velocity_controller_state_known
            and self._controller_state_future is not None
            and self._controller_state_future.done()
        ):
            try:
                response = self._controller_state_future.result()
                states = {
                    controller.name: controller.state
                    for controller in response.controller
                    if controller.name in SAFETY_SWITCHED_CONTROLLERS
                }
                if set(states) == set(SAFETY_SWITCHED_CONTROLLERS):
                    if all(state == "active" for state in states.values()):
                        self._velocity_controller_state_known = True
                        self._velocity_controllers_active = True
                    elif all(state == "inactive" for state in states.values()):
                        self._velocity_controller_state_known = True
                        self._velocity_controllers_active = False
                    else:
                        query_requires_switch = True
                else:
                    query_requires_switch = True
            except BaseException:
                query_requires_switch = True
            finally:
                self._controller_state_future = None
        if (
            self._velocity_controller_state_known
            and permit == self._velocity_controllers_active
        ):
            return
        if not permit_changed and not reassert_due:
            return

        # Query the real controller state before issuing a switch.  This avoids
        # asking controller_manager to deactivate controllers which the
        # launch-time loader has already left inactive, while still failing
        # closed when either controller is unexpectedly active or absent.
        if not self._velocity_controller_state_known and not query_requires_switch:
            if self._controller_state_future is not None:
                return
            if self._list_controllers_client.service_is_ready():
                self._last_controller_state_query_monotonic = now
                self._last_controller_request_monotonic = now
                self._controller_state_future = (
                    self._list_controllers_client.call_async(
                        ListControllers.Request()
                    )
                )
                return

        if not self._switch_controller_client.service_is_ready():
            self._velocity_controller_state_known = False
            self._velocity_controllers_active = False
            return

        request = SwitchController.Request()
        if permit:
            request.activate_controllers = list(SAFETY_SWITCHED_CONTROLLERS)
        else:
            request.deactivate_controllers = list(SAFETY_SWITCHED_CONTROLLERS)
        request.strictness = (
            SwitchController.Request.STRICT
            if permit
            else SwitchController.Request.BEST_EFFORT
        )
        request.activate_asap = True
        request.timeout = Duration(sec=1)
        self._last_controller_request_monotonic = now
        self._switch_future = self._switch_controller_client.call_async(request)
        self._switch_future.add_done_callback(
            lambda future, requested_permit=permit: self._on_switch_complete(
                future, requested_permit
            )
        )

    def _cancel_trajectory_goals(self) -> dict[str, object]:
        request = CancelGoal.Request()
        self._inhibit_cancel_started = True
        created = {}
        for action_name, client in self._trajectory_cancel_clients.items():
            previous = self._cancel_futures.get(action_name)
            if previous is not None:
                if not previous.done():
                    continue
                try:
                    if (
                        previous.result().return_code
                        == CancelGoal.Response.ERROR_NONE
                    ):
                        continue
                except BaseException:
                    pass
            if client.service_is_ready():
                future = client.call_async(request)
                self._cancel_futures[action_name] = future
                created[action_name] = future
        return created

    def _all_trajectory_cancels_succeeded(self) -> bool:
        if len(self._cancel_futures) != len(self._trajectory_cancel_clients):
            return False
        for future in self._cancel_futures.values():
            if not future.done():
                return False
            try:
                if future.result().return_code != CancelGoal.Response.ERROR_NONE:
                    return False
            except BaseException:
                return False
        return True

    def _publish_position_holds(
        self, *, inhibited: bool, joint_positions: dict[str, float]
    ) -> bool:
        if not inhibited:
            with self._state_lock:
                unsafe_edge_pending = (
                    self._consumed_unsafe_generation < self._unsafe_generation
                )
            if unsafe_edge_pending:
                # This permitted decision was evaluated immediately before an
                # input callback recorded a dangerous edge.  That callback may
                # already have sent cancel requests; do not erase their futures
                # from this stale in-flight publish cycle.
                return False
            with self._controller_lock:
                self._inhibit_cancel_started = False
                self._inhibit_cancel_barrier_complete = False
                self._cancel_futures = {}
            self._hold_inhibited = False
            self._hold_positions = {}
            return False
        if not self._inhibit_cancel_barrier_complete:
            with self._controller_lock:
                self._inhibit_cancel_barrier_complete = (
                    self._inhibit_cancel_started
                    and self._all_trajectory_cancels_succeeded()
                )
            if not self._inhibit_cancel_barrier_complete:
                return False
        missing = [
            joint
            for controller, joints in SAFETY_HELD_CONTROLLER_JOINTS.items()
            for joint in joints
            if joint
            not in SAFETY_FIXED_SAFE_CONTROLLER_POSITIONS.get(controller, {})
            if joint not in joint_positions
        ]
        if missing:
            return False
        if not self._hold_inhibited:
            self._hold_positions = {
                joint: self._hold_positions.get(joint, joint_positions[joint])
                for controller, joints in SAFETY_HELD_CONTROLLER_JOINTS.items()
                for joint in joints
                if joint
                not in SAFETY_FIXED_SAFE_CONTROLLER_POSITIONS.get(controller, {})
            }
            self._hold_inhibited = True
        for controller, joints in SAFETY_HELD_CONTROLLER_JOINTS.items():
            if controller in SAFETY_NATIVE_CANCEL_HOLD_CONTROLLERS:
                continue
            fixed_positions = SAFETY_FIXED_SAFE_CONTROLLER_POSITIONS.get(
                controller, {}
            )
            self._publish_controller_hold(
                controller,
                joints,
                {
                    joint: fixed_positions.get(
                        joint, self._hold_positions.get(joint, 0.0)
                    )
                    for joint in joints
                },
            )
        return True

    def _publish_controller_hold(
        self,
        controller: str,
        joints: tuple[str, ...],
        positions: dict[str, float],
    ) -> None:
        point = JointTrajectoryPoint()
        point.positions = [positions[joint] for joint in joints]
        point.time_from_start = Duration(nanosec=100_000_000)
        command = JointTrajectory()
        command.joint_names = list(joints)
        command.points = [point]
        self._hold_publishers[controller].publish(command)

    def _on_switch_complete(self, future, requested_permit: bool) -> None:
        with self._controller_lock:
            self._on_switch_complete_locked(future, requested_permit)

    def _on_switch_complete_locked(self, future, requested_permit: bool) -> None:
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - exercised by ROS runtime
            self.get_logger().error(f"controller switch failed: {error}")
            self._velocity_controller_state_known = False
            self._velocity_controllers_active = False
            return
        if response is None or not response.ok:
            self.get_logger().error("controller switch was rejected")
            self._velocity_controller_state_known = False
            self._velocity_controllers_active = False
            return
        self._velocity_controller_state_known = True
        self._velocity_controllers_active = requested_permit

    @staticmethod
    def _diagnostic(
        decision: SafetyDecision,
        stamp,
        controllers_active: bool = False,
        position_hold_ready: bool = False,
        runtime_metrics: dict[str, float | int | str] | None = None,
    ) -> DiagnosticArray:
        if decision.state is SafetyState.ENABLED and controllers_active:
            level = DiagnosticStatus.OK
        elif decision.state is SafetyState.BASE_COMMAND_STOPPED:
            level = DiagnosticStatus.WARN
        else:
            level = DiagnosticStatus.ERROR

        reasons = ",".join(reason.value for reason in decision.active_reasons)
        values = [
            KeyValue(key="state", value=decision.state.value),
            KeyValue(
                key="actuators_enabled",
                value=str(
                    decision.actuators_enabled and controllers_active
                ).lower(),
            ),
            KeyValue(
                key="safety_inputs_permit_actuators",
                value=str(decision.actuators_enabled).lower(),
            ),
            KeyValue(
                key="managed_controllers_active",
                value=str(controllers_active).lower(),
            ),
            KeyValue(
                key="position_hold_ready",
                value=str(position_hold_ready).lower(),
            ),
            KeyValue(
                key="base_command_enabled",
                value=str(decision.base_command_enabled).lower(),
            ),
            KeyValue(key="active_reasons", value=reasons),
            KeyValue(
                key="actuator_channels", value=",".join(ACTUATOR_CHANNELS)
            ),
            KeyValue(
                key="manual_estop_active",
                value=str(decision.inputs.manual_estop_active).lower(),
            ),
            KeyValue(
                key="front_bumper_contact",
                value=str(decision.inputs.front_bumper_contact).lower(),
            ),
            KeyValue(
                key="front_bumper_available",
                value=str(decision.inputs.front_bumper_available).lower(),
            ),
            KeyValue(
                key="rear_bumper_contact",
                value=str(decision.inputs.rear_bumper_contact).lower(),
            ),
            KeyValue(
                key="rear_bumper_available",
                value=str(decision.inputs.rear_bumper_available).lower(),
            ),
            KeyValue(
                key="safety_relay_enabled",
                value=str(decision.inputs.safety_relay_enabled).lower(),
            ),
            KeyValue(
                key="safety_relay_available",
                value=str(decision.inputs.safety_relay_available).lower(),
            ),
            KeyValue(
                key="bms_fault_active",
                value=str(decision.inputs.bms_fault_active).lower(),
            ),
            KeyValue(
                key="bms_fault_available",
                value=str(decision.inputs.bms_fault_available).lower(),
            ),
            KeyValue(
                key="cleaning_motor_fault_active",
                value=str(decision.inputs.cleaning_motor_fault_active).lower(),
            ),
            KeyValue(
                key="cleaning_motor_fault_available",
                value=str(decision.inputs.cleaning_motor_fault_available).lower(),
            ),
            KeyValue(
                key="traction_permitted",
                value=str(decision.inputs.traction_permitted).lower(),
            ),
            KeyValue(
                key="traction_permit_available",
                value=str(decision.inputs.traction_permit_available).lower(),
            ),
            KeyValue(
                key="heartbeat_fresh",
                value=str(decision.inputs.heartbeat_fresh).lower(),
            ),
            KeyValue(
                key="command_fresh",
                value=str(decision.inputs.command_fresh).lower(),
            ),
            KeyValue(
                key="command_valid",
                value=str(decision.inputs.command_valid).lower(),
            ),
        ]
        for key, value in sorted((runtime_metrics or {}).items()):
            if isinstance(value, float):
                encoded = f"{value:.6f}"
            else:
                encoded = str(value)
            values.append(KeyValue(key=key, value=encoded))
        status = DiagnosticStatus(
            level=level,
            name="whole_vehicle_safety",
            hardware_id="formal_competition_vehicle",
            message=decision.state.value,
            values=values,
        )
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = stamp
        diagnostic.status = [status]
        return diagnostic

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WholeVehicleSafetyManager()
    # One worker each for ordinary inputs, dangerous input edges, and
    # controller service responses.  The publish loop has its own thread.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    fatal_error: BaseException | None = None
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            node._raise_if_publish_failed()
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
            node._stop_publish_loop()
            joined = True
        except BaseException as error:
            fatal_error = fatal_error or error
        if not joined:
            # Never destroy ROS entities while the publisher thread may still
            # be using them. A join timeout is process-fatal and deliberately
            # skips executor / node teardown; the thread is daemonized so the
            # non-zero process exit remains deterministic.
            raise RuntimeError(
                "whole_vehicle_safety_manager_publish_thread_join_fatal"
            ) from fatal_error
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if fatal_error is not None:
        raise RuntimeError("whole_vehicle_safety_manager_fatal") from fatal_error


if __name__ == "__main__":
    main()
