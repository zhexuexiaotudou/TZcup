import math

import pytest

from sanitation_safety.actuator_timeout_guard import ActuatorTimeoutState


def test_nonzero_command_requires_zero_after_80ms() -> None:
    state = ActuatorTimeoutState(timeout_sec=0.080)
    assert not state.observe((0.4, 0.0, 0.0, 0.0, 0.0, 0.1), 1.0)
    assert not state.zero_required(1.079)
    assert state.zero_required(1.080)


def test_zero_command_confirms_safe_state() -> None:
    state = ActuatorTimeoutState(timeout_sec=0.080)
    state.observe((0.4, 0.0), 1.0)
    assert not state.observe((0.0, 0.0), 1.05)
    assert not state.zero_required(2.0)


@pytest.mark.parametrize("invalid", [math.nan, math.inf, -math.inf])
def test_nonfinite_final_command_requires_immediate_zero(invalid) -> None:
    state = ActuatorTimeoutState()
    assert state.observe((invalid, 0.0), 1.0)
    assert state.zero_required(1.0)


def test_guard_never_generates_a_motion_value() -> None:
    state = ActuatorTimeoutState()
    state.observe((1.0, -0.5), 1.0)
    assert state.zero_required(1.2)
    state.mark_zero_published()
    assert state.zero_confirmed
    assert state.last_nonzero_monotonic is None


@pytest.mark.parametrize(
    "kwargs",
    [
        {"timeout_sec": 0.0},
        {"timeout_sec": math.inf},
        {"zero_epsilon": -1.0},
        {"zero_epsilon": math.nan},
    ],
)
def test_invalid_guard_configuration_is_rejected(kwargs) -> None:
    with pytest.raises(ValueError):
        ActuatorTimeoutState(**kwargs)
