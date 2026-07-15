from sanitation_tasks.operational_envelope_audit import command_violation


def test_envelope_detects_linear_and_angular_violations():
    assert not command_violation(0.30, -0.25, 0.30, 0.25)
    assert command_violation(0.31, 0.0, 0.30, 0.25)
    assert command_violation(0.0, -0.26, 0.30, 0.25)
