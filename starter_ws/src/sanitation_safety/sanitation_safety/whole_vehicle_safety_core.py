"""Gazebo-independent fail-closed whole-vehicle safety state machine."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Sequence


ACTUATOR_CHANNELS = (
    "base",
    "brush",
    "pump",
    "lift",
    "arm",
    "gripper",
    "service_drain",
)
SAFETY_SWITCHED_CONTROLLERS = (
    "brush_controller",
    "recovery_controller",
)
SAFETY_HELD_CONTROLLER_JOINTS = {
    "cleaning_controller": ("cleaning_lift_joint",),
    "arm_controller": (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ),
    "gripper_controller": ("robotiq_85_left_knuckle_joint",),
    "storage_controller": ("dry_deposit_gate_joint",),
    "service_controller": ("wastewater_drain_valve_joint",),
}
SAFETY_FIXED_SAFE_CONTROLLER_POSITIONS = {
    "service_controller": {"wastewater_drain_valve_joint": 0.0},
}
SAFETY_MANAGED_CONTROLLERS = SAFETY_SWITCHED_CONTROLLERS + tuple(
    SAFETY_HELD_CONTROLLER_JOINTS
)


class SafetyReason(str, Enum):
    """Stable reason tokens emitted by the safety state machine."""

    MANUAL_ESTOP = "manual_estop"
    FRONT_BUMPER_CONTACT = "front_bumper_contact"
    REAR_BUMPER_CONTACT = "rear_bumper_contact"
    FRONT_BUMPER_UNAVAILABLE = "front_bumper_unavailable"
    REAR_BUMPER_UNAVAILABLE = "rear_bumper_unavailable"
    SAFETY_RELAY_UNAVAILABLE = "safety_relay_unavailable"
    SAFETY_RELAY_DISABLED = "safety_relay_disabled"
    BMS_FAULT_UNAVAILABLE = "bms_fault_unavailable"
    BMS_FAULT_ACTIVE = "bms_fault_active"
    CLEANING_MOTOR_FAULT_UNAVAILABLE = "cleaning_motor_fault_unavailable"
    CLEANING_MOTOR_FAULT_ACTIVE = "cleaning_motor_fault_active"
    TRACTION_PERMIT_UNAVAILABLE = "traction_permit_unavailable"
    TRACTION_NOT_PERMITTED = "traction_not_permitted"
    HEARTBEAT_TIMEOUT = "heartbeat_timeout"
    COMMAND_TIMEOUT = "command_timeout"
    INVALID_COMMAND = "invalid_command"
    MANIPULATOR_BASE_INHIBIT = "manipulator_base_inhibit"


class SafetyState(str, Enum):
    """Externally visible whole-vehicle safety states."""

    INHIBITED = "INHIBITED"
    BASE_COMMAND_STOPPED = "BASE_COMMAND_STOPPED"
    ENABLED = "ENABLED"


@dataclass(frozen=True)
class SafeCommand:
    """The two supported differential-drive command components."""

    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class SafetyInputStatus:
    """Auditable state and freshness of every safety input."""

    manual_estop_active: bool
    front_bumper_contact: bool | None
    front_bumper_available: bool
    rear_bumper_contact: bool | None
    rear_bumper_available: bool
    safety_relay_enabled: bool
    safety_relay_available: bool
    bms_fault_active: bool
    bms_fault_available: bool
    cleaning_motor_fault_active: bool
    cleaning_motor_fault_available: bool
    traction_permitted: bool
    traction_permit_available: bool
    heartbeat_fresh: bool
    command_fresh: bool
    command_valid: bool
    base_motion_inhibited: bool


@dataclass(frozen=True)
class SafetyDecision:
    """One atomic safety decision shared by every vehicle actuator."""

    state: SafetyState
    command: SafeCommand
    actuators_enabled: bool
    base_command_enabled: bool
    active_reasons: tuple[SafetyReason, ...]
    inputs: SafetyInputStatus

    def actuator_enabled(self, channel: str) -> bool:
        """Return the single global enable for a named actuator channel."""

        if channel not in ACTUATOR_CHANNELS:
            raise KeyError(f"unknown actuator channel: {channel}")
        return self.actuators_enabled


@dataclass
class VelocityActuatorGate:
    """Fail-closed gate for a fixed-width joint velocity command vector."""

    width: int
    timeout_sec: float = 0.5
    last_command_monotonic: float | None = None
    requested_command: tuple[float, ...] = ()
    command_valid: bool = False

    def __post_init__(self) -> None:
        if self.width <= 0:
            raise ValueError("width must be greater than zero")
        if not math.isfinite(self.timeout_sec) or self.timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be finite and greater than zero")
        if not self.requested_command:
            self.requested_command = self.zero_command

    @property
    def zero_command(self) -> tuple[float, ...]:
        return (0.0,) * self.width

    def set_command(self, values: Sequence[float], now: float) -> None:
        WholeVehicleSafetyCore._require_finite_time(now)
        command = tuple(float(value) for value in values)
        self.last_command_monotonic = now
        self.command_valid = len(command) == self.width and all(
            math.isfinite(value) for value in command
        )
        self.requested_command = command if self.command_valid else self.zero_command

    def evaluate(self, *, permitted: bool, now: float) -> tuple[float, ...]:
        WholeVehicleSafetyCore._require_finite_time(now)
        fresh = WholeVehicleSafetyCore._fresh(
            self.last_command_monotonic, now, self.timeout_sec
        )
        if not permitted or not fresh or not self.command_valid:
            return self.zero_command
        return self.requested_command


@dataclass
class WholeVehicleSafetyCore:
    """Pure state machine; arrival times use an injected monotonic clock."""

    command_timeout_sec: float = 0.5
    heartbeat_timeout_sec: float = 0.5
    bumper_timeout_sec: float = 0.5
    safety_relay_timeout_sec: float = 0.5
    bms_fault_timeout_sec: float = 0.5
    cleaning_motor_fault_timeout_sec: float = 0.25
    traction_permit_timeout_sec: float = 0.5
    max_linear_velocity: float = 0.45
    max_angular_velocity: float = 0.35

    # Power-up is deliberately inhibited until every safety input is observed.
    manual_estop: bool = True
    safety_relay_enabled: bool = False
    bms_fault_active: bool = True
    cleaning_motor_fault_active: bool = True
    traction_permitted: bool = False
    front_bumper_contact: bool | None = None
    rear_bumper_contact: bool | None = None
    last_heartbeat_monotonic: float | None = None
    last_front_bumper_monotonic: float | None = None
    last_rear_bumper_monotonic: float | None = None
    last_safety_relay_monotonic: float | None = None
    last_bms_fault_monotonic: float | None = None
    last_cleaning_motor_fault_monotonic: float | None = None
    last_traction_permit_monotonic: float | None = None
    last_command_monotonic: float | None = None
    requested_command: SafeCommand = SafeCommand()
    command_valid: bool = False
    base_motion_inhibited: bool = False

    def __post_init__(self) -> None:
        positive = {
            "command_timeout_sec": self.command_timeout_sec,
            "heartbeat_timeout_sec": self.heartbeat_timeout_sec,
            "bumper_timeout_sec": self.bumper_timeout_sec,
            "safety_relay_timeout_sec": self.safety_relay_timeout_sec,
            "bms_fault_timeout_sec": self.bms_fault_timeout_sec,
            "cleaning_motor_fault_timeout_sec": self.cleaning_motor_fault_timeout_sec,
            "traction_permit_timeout_sec": self.traction_permit_timeout_sec,
            "max_linear_velocity": self.max_linear_velocity,
            "max_angular_velocity": self.max_angular_velocity,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero")

    def set_manual_estop(self, active: bool) -> None:
        self.manual_estop = bool(active)

    def set_safety_relay(self, enabled: bool, now: float) -> None:
        self._require_finite_time(now)
        self.safety_relay_enabled = bool(enabled)
        self.last_safety_relay_monotonic = now

    def set_bms_fault(self, active: bool, now: float) -> None:
        self._require_finite_time(now)
        self.bms_fault_active = bool(active)
        self.last_bms_fault_monotonic = now

    def set_cleaning_motor_fault(self, active: bool, now: float) -> None:
        self._require_finite_time(now)
        self.cleaning_motor_fault_active = bool(active)
        self.last_cleaning_motor_fault_monotonic = now

    def set_traction_permitted(self, permitted: bool, now: float) -> None:
        self._require_finite_time(now)
        self.traction_permitted = bool(permitted)
        self.last_traction_permit_monotonic = now

    def set_front_bumper(self, contact: bool, now: float) -> None:
        self._require_finite_time(now)
        self.front_bumper_contact = bool(contact)
        self.last_front_bumper_monotonic = now

    def set_rear_bumper(self, contact: bool, now: float) -> None:
        self._require_finite_time(now)
        self.rear_bumper_contact = bool(contact)
        self.last_rear_bumper_monotonic = now

    def heartbeat(self, now: float) -> None:
        self._require_finite_time(now)
        self.last_heartbeat_monotonic = now

    def set_base_motion_inhibited(self, inhibited: bool) -> None:
        self.base_motion_inhibited = bool(inhibited)

    def set_command(self, linear_x: float, angular_z: float, now: float) -> None:
        self._require_finite_time(now)
        self.last_command_monotonic = now
        self.command_valid = math.isfinite(linear_x) and math.isfinite(angular_z)
        if not self.command_valid:
            self.requested_command = SafeCommand()
            return
        self.requested_command = SafeCommand(
            linear_x=self._clamp(linear_x, self.max_linear_velocity),
            angular_z=self._clamp(angular_z, self.max_angular_velocity),
        )

    def evaluate(self, now: float) -> SafetyDecision:
        """Compute the current command and the atomic actuator enable state."""

        self._require_finite_time(now)
        reasons: list[SafetyReason] = []

        if self.manual_estop:
            reasons.append(SafetyReason.MANUAL_ESTOP)
        relay_fresh = self._fresh(
            self.last_safety_relay_monotonic,
            now,
            self.safety_relay_timeout_sec,
        )
        if not relay_fresh:
            reasons.append(SafetyReason.SAFETY_RELAY_UNAVAILABLE)
        elif not self.safety_relay_enabled:
            reasons.append(SafetyReason.SAFETY_RELAY_DISABLED)
        bms_fault_fresh = self._fresh(
            self.last_bms_fault_monotonic,
            now,
            self.bms_fault_timeout_sec,
        )
        if not bms_fault_fresh:
            reasons.append(SafetyReason.BMS_FAULT_UNAVAILABLE)
        elif self.bms_fault_active:
            reasons.append(SafetyReason.BMS_FAULT_ACTIVE)
        cleaning_motor_fault_fresh = self._fresh(
            self.last_cleaning_motor_fault_monotonic,
            now,
            self.cleaning_motor_fault_timeout_sec,
        )
        if not cleaning_motor_fault_fresh:
            reasons.append(SafetyReason.CLEANING_MOTOR_FAULT_UNAVAILABLE)
        elif self.cleaning_motor_fault_active:
            reasons.append(SafetyReason.CLEANING_MOTOR_FAULT_ACTIVE)
        traction_permit_fresh = self._fresh(
            self.last_traction_permit_monotonic,
            now,
            self.traction_permit_timeout_sec,
        )
        if not traction_permit_fresh:
            reasons.append(SafetyReason.TRACTION_PERMIT_UNAVAILABLE)
        elif not self.traction_permitted:
            reasons.append(SafetyReason.TRACTION_NOT_PERMITTED)
        heartbeat_fresh = self._fresh(
            self.last_heartbeat_monotonic, now, self.heartbeat_timeout_sec
        )
        if not heartbeat_fresh:
            reasons.append(SafetyReason.HEARTBEAT_TIMEOUT)

        front_fresh = self._fresh(
            self.last_front_bumper_monotonic, now, self.bumper_timeout_sec
        )
        rear_fresh = self._fresh(
            self.last_rear_bumper_monotonic, now, self.bumper_timeout_sec
        )
        if not front_fresh:
            reasons.append(SafetyReason.FRONT_BUMPER_UNAVAILABLE)
        elif self.front_bumper_contact:
            reasons.append(SafetyReason.FRONT_BUMPER_CONTACT)
        if not rear_fresh:
            reasons.append(SafetyReason.REAR_BUMPER_UNAVAILABLE)
        elif self.rear_bumper_contact:
            reasons.append(SafetyReason.REAR_BUMPER_CONTACT)

        global_reasons = tuple(reasons)
        actuators_enabled = not global_reasons

        command_fresh = self._fresh(
            self.last_command_monotonic, now, self.command_timeout_sec
        )
        if not command_fresh:
            reasons.append(SafetyReason.COMMAND_TIMEOUT)
        elif not self.command_valid:
            reasons.append(SafetyReason.INVALID_COMMAND)
        if self.base_motion_inhibited:
            reasons.append(SafetyReason.MANIPULATOR_BASE_INHIBIT)

        base_command_enabled = (
            actuators_enabled
            and command_fresh
            and self.command_valid
            and not self.base_motion_inhibited
        )
        if not actuators_enabled:
            state = SafetyState.INHIBITED
        elif not base_command_enabled:
            state = SafetyState.BASE_COMMAND_STOPPED
        else:
            state = SafetyState.ENABLED

        return SafetyDecision(
            state=state,
            command=self.requested_command if base_command_enabled else SafeCommand(),
            actuators_enabled=actuators_enabled,
            base_command_enabled=base_command_enabled,
            active_reasons=tuple(reasons),
            inputs=SafetyInputStatus(
                manual_estop_active=self.manual_estop,
                front_bumper_contact=self.front_bumper_contact,
                front_bumper_available=front_fresh,
                rear_bumper_contact=self.rear_bumper_contact,
                rear_bumper_available=rear_fresh,
                safety_relay_enabled=self.safety_relay_enabled,
                safety_relay_available=relay_fresh,
                bms_fault_active=self.bms_fault_active,
                bms_fault_available=bms_fault_fresh,
                cleaning_motor_fault_active=self.cleaning_motor_fault_active,
                cleaning_motor_fault_available=cleaning_motor_fault_fresh,
                traction_permitted=self.traction_permitted,
                traction_permit_available=traction_permit_fresh,
                heartbeat_fresh=heartbeat_fresh,
                command_fresh=command_fresh,
                command_valid=self.command_valid,
                base_motion_inhibited=self.base_motion_inhibited,
            ),
        )

    @staticmethod
    def _fresh(stamp: float | None, now: float, timeout: float) -> bool:
        if stamp is None:
            return False
        age = now - stamp
        return 0.0 <= age <= timeout

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    @staticmethod
    def _require_finite_time(now: float) -> None:
        if not math.isfinite(now):
            raise ValueError("monotonic time must be finite")
