import math

import pytest

from sanitation_safety.velocity_gate import VelocityGateState


def test_emergency_stop_has_priority():
    state = VelocityGateState(
        emergency_stopped=True,
        command_timeout_sec=0.5,
        last_command_monotonic=1.0,
        last_estop_monotonic=1.0,
    )
    assert state.output(0.4, 0.2, 1.1) == (0.0, 0.0)


def test_fresh_command_passes():
    state = VelocityGateState(
        emergency_stopped=False,
        command_timeout_sec=0.5,
        last_command_monotonic=1.0,
        last_estop_monotonic=1.0,
    )
    assert state.output(0.4, -0.2, 1.1) == (0.4, -0.2)


def test_stale_command_fails_safe():
    state = VelocityGateState(
        emergency_stopped=False,
        command_timeout_sec=0.5,
        last_command_monotonic=1.0,
        last_estop_monotonic=1.0,
    )
    assert state.output(0.4, 0.2, 1.6) == (0.0, 0.0)


def test_operational_envelope_clamps_both_directions():
    state = VelocityGateState(
        command_timeout_sec=0.5,
        last_command_monotonic=1.0,
        emergency_stopped=False,
        last_estop_monotonic=1.0,
        max_linear_velocity=0.30,
        max_angular_velocity=0.25,
    )
    assert state.output(0.7, -0.8, 1.1) == (0.30, -0.25)


def test_startup_without_safety_heartbeat_fails_closed():
    state = VelocityGateState(last_command_monotonic=1.0)
    assert state.output(0.4, 0.2, 1.1) == (0.0, 0.0)


def test_stale_safety_heartbeat_fails_closed_after_clear():
    state = VelocityGateState(
        emergency_stopped=False,
        last_command_monotonic=1.0,
        last_estop_monotonic=1.0,
        command_timeout_sec=1.0,
        estop_heartbeat_timeout_sec=0.5,
    )
    assert state.output(0.4, 0.2, 1.6) == (0.0, 0.0)


@pytest.mark.parametrize(
    ("linear_x", "angular_z"),
    [
        (math.nan, 0.0),
        (math.inf, 0.0),
        (-math.inf, 0.0),
        (0.0, math.nan),
        (0.0, math.inf),
        (0.0, -math.inf),
    ],
)
def test_nonfinite_command_fails_closed(linear_x, angular_z):
    state = VelocityGateState(
        emergency_stopped=False,
        last_command_monotonic=1.0,
        last_estop_monotonic=1.0,
    )
    assert state.output(linear_x, angular_z, 1.1) == (0.0, 0.0)


def test_estop_clear_requires_a_fresh_post_clear_command():
    state = VelocityGateState(
        emergency_stopped=False,
        last_command_monotonic=1.0,
        last_estop_monotonic=1.0,
    )
    assert state.output(0.4, 0.2, 1.1) == (0.4, 0.2)

    state.observe_estop(True, 1.2)
    state.last_command_monotonic = 1.25  # command received while stopped
    state.observe_estop(False, 1.3)
    assert state.output(0.4, 0.2, 1.31) == (0.0, 0.0)

    state.last_command_monotonic = 1.32
    assert state.output(0.4, 0.2, 1.33) == (0.4, 0.2)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"command_timeout_sec": 0.0},
        {"estop_heartbeat_timeout_sec": math.nan},
        {"max_linear_velocity": -1.0},
        {"max_angular_velocity": math.inf},
    ],
)
def test_invalid_gate_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        VelocityGateState(**kwargs)
