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
