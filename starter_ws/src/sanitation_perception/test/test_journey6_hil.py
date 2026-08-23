import pytest

from sanitation_perception.journey6_hil import AckermannHilCommand, HilCommandAuthority


def command(sequence=1, stamp=1.0, valid_until=1.2):
    return AckermannHilCommand(
        stamp_s=stamp,
        sequence=sequence,
        speed_mps=0.5,
        steering_angle_rad=0.1,
        acceleration_limit_mps2=0.5,
        source_id="j6-algorithm",
        valid_until_s=valid_until,
    )


def test_hil_rejects_replay_and_expires_to_safe_stop():
    gate = HilCommandAuthority(source_id="j6-algorithm")
    gate.accept(command(), now_s=1.0)
    assert gate.output(now_s=1.1).speed_mps == 0.5
    assert gate.output(now_s=1.2) is None
    with pytest.raises(ValueError, match="stale or replayed"):
        gate.accept(command(), now_s=1.0)


def test_hil_network_recovery_requires_operator_resume_and_new_command():
    gate = HilCommandAuthority(source_id="j6-algorithm")
    gate.accept(command(), now_s=1.0)
    gate.network_lost()
    assert gate.output(now_s=1.05) is None
    gate.network_restored()
    with pytest.raises(RuntimeError, match="operator resume"):
        gate.accept(command(sequence=2, stamp=1.1, valid_until=1.3), now_s=1.1)
    gate.operator_resume()
    assert gate.output(now_s=1.1) is None
    gate.accept(command(sequence=2, stamp=1.1, valid_until=1.3), now_s=1.1)
    assert gate.output(now_s=1.2) is not None
