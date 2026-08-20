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
