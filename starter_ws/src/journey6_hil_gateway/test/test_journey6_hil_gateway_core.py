import pytest

from journey6_hil_gateway.core import CommandSafetyGate, HealthFrame
from sanitation_perception.journey6_hil import AckermannHilCommand


def healthy(sequence=1, stamp=1.0):
    return HealthFrame(
        source_id="j6-algorithm",
        sequence=sequence,
        stamp_s=stamp,
        healthy=True,
    )


def command(sequence=1, stamp=1.0, valid_until=1.2, speed=0.5):
    return AckermannHilCommand(
        stamp_s=stamp,
        sequence=sequence,
        speed_mps=speed,
        steering_angle_rad=0.1,
        acceleration_limit_mps2=0.5,
        source_id="j6-algorithm",
        valid_until_s=valid_until,
    )


def arm(gate, *, monotonic=10.0):
    gate.update_health(healthy(), received_monotonic_s=monotonic)
    gate.set_placement_gate(True)
    gate.set_estop(False)
    gate.operator_resume(now_monotonic_s=monotonic)


def test_startup_and_command_timeout_are_zero_only():
    gate = CommandSafetyGate()
    startup = gate.output(now_sim_s=0.0, now_monotonic_s=10.0)
    assert startup.speed_mps == 0.0
    assert startup.source_id == "pc-safety-gate-zero-only"

    arm(gate)
    gate.accept(command(), now_sim_s=1.0, now_monotonic_s=10.0)
    assert gate.output(now_sim_s=1.05, now_monotonic_s=10.05).speed_mps == 0.5
    timed_out = gate.output(now_sim_s=1.2, now_monotonic_s=10.10)
    assert timed_out.speed_mps == 0.0
    assert timed_out.source_id == "pc-safety-gate-zero-only"


def test_network_health_loss_requires_resume_and_new_sequence():
    gate = CommandSafetyGate(health_timeout_s=0.2)
    arm(gate)
    gate.accept(command(), now_sim_s=1.0, now_monotonic_s=10.0)
    assert gate.output(now_sim_s=1.05, now_monotonic_s=10.21).speed_mps == 0.0

    gate.update_health(healthy(sequence=2, stamp=1.2), received_monotonic_s=10.22)
    with pytest.raises(RuntimeError, match="operator resume"):
        gate.accept(
            command(sequence=2, stamp=1.2, valid_until=1.4),
            now_sim_s=1.2,
            now_monotonic_s=10.22,
        )
    gate.operator_resume(now_monotonic_s=10.22)
    with pytest.raises(ValueError, match="stale or replayed"):
        gate.accept(command(), now_sim_s=1.2, now_monotonic_s=10.22)
    gate.accept(
        command(sequence=2, stamp=1.2, valid_until=1.4),
        now_sim_s=1.2,
        now_monotonic_s=10.22,
    )
    assert gate.output(now_sim_s=1.3, now_monotonic_s=10.25).speed_mps == 0.5


def test_non_j6_and_out_of_envelope_commands_fail_closed():
    gate = CommandSafetyGate(maximum_speed_mps=1.0)
    arm(gate)
    wrong_source = command()
    wrong_source = AckermannHilCommand(
        **{**wrong_source.__dict__, "source_id": "pc-planner"}
    )
    with pytest.raises(ValueError, match="non-Journey6"):
        gate.accept(wrong_source, now_sim_s=1.0, now_monotonic_s=10.0)
    assert gate.output(now_sim_s=1.0, now_monotonic_s=10.0).speed_mps == 0.0

    gate.operator_resume(now_monotonic_s=10.0)
    with pytest.raises(ValueError, match="speed exceeds"):
        gate.accept(
            command(sequence=2, stamp=1.1, valid_until=1.3, speed=1.1),
            now_sim_s=1.1,
            now_monotonic_s=10.0,
        )


def test_clock_rollback_and_replayed_health_trip_the_gate():
    gate = CommandSafetyGate()
    arm(gate)
    gate.accept(command(), now_sim_s=1.0, now_monotonic_s=10.0)
    with pytest.raises(ValueError, match="clock rollback"):
        gate.accept(
            command(sequence=2, stamp=0.9, valid_until=1.1),
            now_sim_s=0.9,
            now_monotonic_s=10.0,
        )
    with pytest.raises(ValueError, match="stale or replayed J6 health"):
        gate.update_health(healthy(), received_monotonic_s=10.01)
