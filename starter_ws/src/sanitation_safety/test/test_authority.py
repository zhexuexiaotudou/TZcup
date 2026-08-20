from sanitation_safety.authority import SafetyAuthorityState


def test_authority_starts_stopped_and_requires_explicit_clear() -> None:
    state = SafetyAuthorityState()
    assert state.emergency_stopped
    assert state.snapshot(10.0)["operator_command_age_sec"] is None

    state.apply_operator_command(False, 10.0)
    assert not state.emergency_stopped
    assert state.command_sequence == 1
    assert state.snapshot(10.25)["operator_command_age_sec"] == 0.25


def test_stop_request_relatches_authority() -> None:
    state = SafetyAuthorityState(emergency_stopped=False)
    state.apply_operator_command(True, 5.0)
    assert state.emergency_stopped
    assert state.command_sequence == 1


def test_required_supervisor_blocks_clear_and_latches_fault() -> None:
    state = SafetyAuthorityState(
        require_supervisor_heartbeat=True,
        supervisor_heartbeat_timeout_sec=0.5,
    )
    assert state.apply_operator_command(False, 1.0) is False
    assert state.emergency_stopped

    state.apply_supervisor_report(
        motion_healthy=True, motion_faults=[], now=1.1
    )
    assert state.apply_operator_command(False, 1.2) is True
    assert not state.emergency_stopped

    state.apply_supervisor_report(
        motion_healthy=False,
        motion_faults=["scan:stale"],
        now=1.3,
    )
    assert state.emergency_stopped
    assert state.supervisor_trip_latched

    state.apply_supervisor_report(
        motion_healthy=True, motion_faults=[], now=1.4
    )
    assert state.evaluate(1.5), "recovery must not auto-resume motion"
    assert state.apply_operator_command(False, 1.5) is True
    assert not state.emergency_stopped


def test_supervisor_heartbeat_timeout_trips_motion() -> None:
    state = SafetyAuthorityState(
        emergency_stopped=False,
        require_supervisor_heartbeat=True,
        supervisor_heartbeat_timeout_sec=0.5,
    )
    state.apply_supervisor_report(
        motion_healthy=True, motion_faults=[], now=2.0
    )
    assert not state.evaluate(2.49)
    assert state.evaluate(2.51)
    assert state.snapshot(2.51)["supervisor_trip_latched"] is True
