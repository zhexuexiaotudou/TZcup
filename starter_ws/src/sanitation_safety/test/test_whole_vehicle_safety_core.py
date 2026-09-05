import math

import pytest

from sanitation_safety.whole_vehicle_safety_core import ACTUATOR_CHANNELS
from sanitation_safety.whole_vehicle_safety_core import SAFETY_MANAGED_CONTROLLERS
from sanitation_safety.whole_vehicle_safety_core import SAFETY_HELD_CONTROLLER_JOINTS
from sanitation_safety.whole_vehicle_safety_core import SAFETY_FIXED_SAFE_CONTROLLER_POSITIONS
from sanitation_safety.whole_vehicle_safety_core import SAFETY_SWITCHED_CONTROLLERS
from sanitation_safety.whole_vehicle_safety_core import SafetyReason, SafetyState
from sanitation_safety.whole_vehicle_safety_core import VelocityActuatorGate
from sanitation_safety.whole_vehicle_safety_core import WholeVehicleSafetyCore


def ready_core(now=10.0):
    core = WholeVehicleSafetyCore()
    core.set_manual_estop(False)
    core.set_safety_relay(True, now)
    core.set_bms_fault(False, now)
    core.set_cleaning_motor_fault(False, now)
    core.set_traction_permitted(True, now)
    core.set_front_bumper(False, now)
    core.set_rear_bumper(False, now)
    core.heartbeat(now)
    core.set_command(0.25, -0.2, now)
    return core


def test_power_up_is_fail_closed_for_every_actuator():
    decision = WholeVehicleSafetyCore().evaluate(1.0)

    assert decision.state is SafetyState.INHIBITED
    assert decision.command.linear_x == 0.0
    assert decision.command.angular_z == 0.0
    assert not decision.actuators_enabled
    assert all(not decision.actuator_enabled(name) for name in ACTUATOR_CHANNELS)
    assert SafetyReason.MANUAL_ESTOP in decision.active_reasons
    assert SafetyReason.SAFETY_RELAY_UNAVAILABLE in decision.active_reasons
    assert SafetyReason.BMS_FAULT_UNAVAILABLE in decision.active_reasons
    assert SafetyReason.CLEANING_MOTOR_FAULT_UNAVAILABLE in decision.active_reasons
    assert SafetyReason.TRACTION_PERMIT_UNAVAILABLE in decision.active_reasons
    assert SafetyReason.HEARTBEAT_TIMEOUT in decision.active_reasons
    assert SafetyReason.FRONT_BUMPER_UNAVAILABLE in decision.active_reasons
    assert SafetyReason.REAR_BUMPER_UNAVAILABLE in decision.active_reasons
    assert decision.inputs.manual_estop_active
    assert not decision.inputs.safety_relay_available
    assert not decision.inputs.bms_fault_available
    assert not decision.inputs.cleaning_motor_fault_available
    assert not decision.inputs.traction_permit_available
    assert not decision.inputs.heartbeat_fresh


def test_ready_state_uses_one_enable_and_clamps_base_command():
    core = ready_core()
    core.set_command(4.0, -3.0, 10.0)

    decision = core.evaluate(10.1)

    assert decision.state is SafetyState.ENABLED
    assert decision.command.linear_x == 0.45
    assert decision.command.angular_z == -0.35
    assert decision.actuators_enabled
    assert decision.base_command_enabled
    assert all(decision.actuator_enabled(name) for name in ACTUATOR_CHANNELS)
    assert not decision.active_reasons
    assert decision.inputs.front_bumper_available
    assert decision.inputs.rear_bumper_available
    assert decision.inputs.safety_relay_available
    assert decision.inputs.bms_fault_available
    assert not decision.inputs.bms_fault_active
    assert decision.inputs.traction_permit_available
    assert decision.inputs.traction_permitted
    assert decision.inputs.heartbeat_fresh
    assert decision.inputs.command_fresh
    assert decision.inputs.command_valid


def test_deployed_manipulator_stops_only_base_and_keeps_arm_powered():
    core = ready_core()
    core.set_base_motion_inhibited(True)

    decision = core.evaluate(10.1)

    assert decision.state is SafetyState.BASE_COMMAND_STOPPED
    assert decision.actuators_enabled is True
    assert decision.actuator_enabled("arm") is True
    assert decision.base_command_enabled is False
    assert decision.command.linear_x == 0.0
    assert decision.inputs.base_motion_inhibited is True
    assert SafetyReason.MANIPULATOR_BASE_INHIBIT in decision.active_reasons


@pytest.mark.parametrize(
    "trigger,expected_reason",
    [
        (lambda core: core.set_manual_estop(True), SafetyReason.MANUAL_ESTOP),
        (
            lambda core: core.set_front_bumper(True, 10.1),
            SafetyReason.FRONT_BUMPER_CONTACT,
        ),
        (
            lambda core: core.set_rear_bumper(True, 10.1),
            SafetyReason.REAR_BUMPER_CONTACT,
        ),
        (
            lambda core: core.set_safety_relay(False, 10.1),
            SafetyReason.SAFETY_RELAY_DISABLED,
        ),
        (
            lambda core: core.set_bms_fault(True, 10.1),
            SafetyReason.BMS_FAULT_ACTIVE,
        ),
        (
            lambda core: core.set_cleaning_motor_fault(True, 10.1),
            SafetyReason.CLEANING_MOTOR_FAULT_ACTIVE,
        ),
        (
            lambda core: core.set_traction_permitted(False, 10.1),
            SafetyReason.TRACTION_NOT_PERMITTED,
        ),
    ],
)
def test_hard_interlock_zeros_base_and_disables_every_actuator(
    trigger, expected_reason
):
    core = ready_core()
    trigger(core)

    decision = core.evaluate(10.1)

    assert decision.state is SafetyState.INHIBITED
    assert decision.command.linear_x == 0.0
    assert not decision.actuators_enabled
    assert not decision.base_command_enabled
    assert all(not decision.actuator_enabled(name) for name in ACTUATOR_CHANNELS)
    assert expected_reason in decision.active_reasons


def test_stale_heartbeat_disables_the_whole_vehicle():
    core = ready_core(now=10.0)
    core.set_safety_relay(True, 10.4)
    core.set_bms_fault(False, 10.4)
    core.set_cleaning_motor_fault(False, 10.4)
    core.set_traction_permitted(True, 10.4)
    core.set_front_bumper(False, 10.4)
    core.set_rear_bumper(False, 10.4)
    core.set_command(0.2, 0.0, 10.4)

    decision = core.evaluate(10.501)

    assert decision.state is SafetyState.INHIBITED
    assert not decision.actuators_enabled
    assert SafetyReason.HEARTBEAT_TIMEOUT in decision.active_reasons


@pytest.mark.parametrize(
    "side,expected_reason",
    [
        ("front", SafetyReason.FRONT_BUMPER_UNAVAILABLE),
        ("rear", SafetyReason.REAR_BUMPER_UNAVAILABLE),
    ],
)
def test_stale_bumper_feed_is_fail_closed(side, expected_reason):
    core = ready_core(now=10.0)
    core.heartbeat(10.4)
    core.set_safety_relay(True, 10.4)
    core.set_cleaning_motor_fault(False, 10.4)
    core.set_command(0.2, 0.0, 10.4)
    if side == "front":
        core.set_rear_bumper(False, 10.4)
    else:
        core.set_front_bumper(False, 10.4)

    decision = core.evaluate(10.501)

    assert decision.state is SafetyState.INHIBITED
    assert not decision.actuators_enabled
    assert expected_reason in decision.active_reasons


def test_stale_safety_relay_feed_is_fail_closed():
    core = ready_core(now=10.0)
    core.heartbeat(10.4)
    core.set_front_bumper(False, 10.4)
    core.set_rear_bumper(False, 10.4)
    core.set_cleaning_motor_fault(False, 10.4)
    core.set_command(0.2, 0.0, 10.4)

    decision = core.evaluate(10.501)

    assert decision.state is SafetyState.INHIBITED
    assert not decision.actuators_enabled
    assert SafetyReason.SAFETY_RELAY_UNAVAILABLE in decision.active_reasons


@pytest.mark.parametrize(
    "stale_feed,expected_reason",
    [
        ("bms_fault", SafetyReason.BMS_FAULT_UNAVAILABLE),
        ("cleaning_motor_fault", SafetyReason.CLEANING_MOTOR_FAULT_UNAVAILABLE),
        ("traction_permit", SafetyReason.TRACTION_PERMIT_UNAVAILABLE),
    ],
)
def test_stale_bms_feeds_are_independently_fail_closed(stale_feed, expected_reason):
    core = ready_core(now=10.0)
    core.heartbeat(10.4)
    core.set_safety_relay(True, 10.4)
    core.set_front_bumper(False, 10.4)
    core.set_rear_bumper(False, 10.4)
    core.set_command(0.2, 0.0, 10.4)
    if stale_feed != "bms_fault":
        core.set_bms_fault(False, 10.4)
    if stale_feed != "cleaning_motor_fault":
        core.set_cleaning_motor_fault(False, 10.4)
    if stale_feed != "traction_permit":
        core.set_traction_permitted(True, 10.4)

    decision = core.evaluate(10.501)

    assert decision.state is SafetyState.INHIBITED
    assert not decision.actuators_enabled
    assert expected_reason in decision.active_reasons


def test_command_timeout_stops_base_without_dropping_actuator_power():
    core = ready_core(now=10.0)
    core.heartbeat(10.4)
    core.set_safety_relay(True, 10.4)
    core.set_bms_fault(False, 10.4)
    core.set_cleaning_motor_fault(False, 10.4)
    core.set_traction_permitted(True, 10.4)
    core.set_front_bumper(False, 10.4)
    core.set_rear_bumper(False, 10.4)

    decision = core.evaluate(10.501)

    assert decision.state is SafetyState.BASE_COMMAND_STOPPED
    assert decision.actuators_enabled
    assert not decision.base_command_enabled
    assert decision.command.linear_x == 0.0
    assert SafetyReason.COMMAND_TIMEOUT in decision.active_reasons


def test_non_finite_command_is_rejected_and_never_reaches_output():
    core = ready_core()
    core.set_command(math.nan, math.inf, 10.1)

    decision = core.evaluate(10.2)

    assert decision.state is SafetyState.BASE_COMMAND_STOPPED
    assert decision.actuators_enabled
    assert not decision.base_command_enabled
    assert decision.command.linear_x == 0.0
    assert decision.command.angular_z == 0.0
    assert SafetyReason.INVALID_COMMAND in decision.active_reasons


def test_time_rollback_is_treated_as_stale():
    decision = ready_core(now=10.0).evaluate(9.9)

    assert decision.state is SafetyState.INHIBITED
    assert SafetyReason.HEARTBEAT_TIMEOUT in decision.active_reasons
    assert SafetyReason.FRONT_BUMPER_UNAVAILABLE in decision.active_reasons
    assert SafetyReason.REAR_BUMPER_UNAVAILABLE in decision.active_reasons


def test_unknown_actuator_channel_is_rejected():
    decision = ready_core().evaluate(10.1)

    with pytest.raises(KeyError, match="unknown actuator channel"):
        decision.actuator_enabled("conveyor")


@pytest.mark.parametrize(
    "field,value",
    [
        ("command_timeout_sec", 0.0),
        ("heartbeat_timeout_sec", -1.0),
        ("bumper_timeout_sec", math.inf),
        ("safety_relay_timeout_sec", 0.0),
        ("bms_fault_timeout_sec", 0.0),
        ("cleaning_motor_fault_timeout_sec", 0.0),
        ("traction_permit_timeout_sec", math.nan),
        ("max_linear_velocity", math.nan),
        ("max_angular_velocity", 0.0),
    ],
)
def test_invalid_configuration_is_rejected(field, value):
    with pytest.raises(ValueError, match=field):
        WholeVehicleSafetyCore(**{field: value})


def test_velocity_actuator_gate_requires_permit_fresh_exact_finite_command():
    gate = VelocityActuatorGate(width=3, timeout_sec=0.5)

    assert gate.evaluate(permitted=True, now=1.0) == (0.0, 0.0, 0.0)
    gate.set_command([8.0, -8.0, 12.0], now=1.0)
    assert gate.evaluate(permitted=False, now=1.1) == (0.0, 0.0, 0.0)
    assert gate.evaluate(permitted=True, now=1.1) == (8.0, -8.0, 12.0)
    assert gate.evaluate(permitted=True, now=1.501) == (0.0, 0.0, 0.0)

    gate.set_command([1.0, 2.0], now=2.0)
    assert gate.evaluate(permitted=True, now=2.1) == (0.0, 0.0, 0.0)
    gate.set_command([1.0, math.nan, 3.0], now=2.2)
    assert gate.evaluate(permitted=True, now=2.3) == (0.0, 0.0, 0.0)


def test_managed_controller_set_covers_all_non_base_formal_actuators():
    assert SAFETY_MANAGED_CONTROLLERS == (
        "brush_controller",
        "recovery_controller",
        "cleaning_controller",
        "arm_controller",
        "gripper_controller",
        "storage_controller",
        "service_controller",
    )
    assert SAFETY_SWITCHED_CONTROLLERS == (
        "brush_controller",
        "recovery_controller",
    )
    assert set(SAFETY_HELD_CONTROLLER_JOINTS) == {
        "cleaning_controller",
        "arm_controller",
        "gripper_controller",
        "storage_controller",
        "service_controller",
    }
    assert SAFETY_FIXED_SAFE_CONTROLLER_POSITIONS == {
        "service_controller": {"wastewater_drain_valve_joint": 0.0}
    }


@pytest.mark.parametrize(
    "kwargs,match",
    [
        ({"width": 0}, "width"),
        ({"width": 1, "timeout_sec": 0.0}, "timeout_sec"),
    ],
)
def test_velocity_actuator_gate_rejects_invalid_configuration(kwargs, match):
    with pytest.raises(ValueError, match=match):
        VelocityActuatorGate(**kwargs)
