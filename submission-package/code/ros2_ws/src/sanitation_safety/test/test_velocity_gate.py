from sanitation_safety.velocity_gate import VelocityGateState


def test_emergency_stop_has_priority():
    state = VelocityGateState(
        emergency_stopped=True,
        command_timeout_sec=0.5,
        last_command_monotonic=1.0,
    )
    assert state.output(0.4, 0.2, 1.1) == (0.0, 0.0)


def test_fresh_command_passes():
    state = VelocityGateState(
        emergency_stopped=False,
        command_timeout_sec=0.5,
        last_command_monotonic=1.0,
    )
    assert state.output(0.4, -0.2, 1.1) == (0.4, -0.2)


def test_stale_command_fails_safe():
    state = VelocityGateState(
        emergency_stopped=False,
        command_timeout_sec=0.5,
        last_command_monotonic=1.0,
    )
    assert state.output(0.4, 0.2, 1.6) == (0.0, 0.0)
