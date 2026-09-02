from __future__ import annotations

from formal_squeegee_compliance_core import (
    SQUEEGEE_SIGNALS,
    evaluate_squeegee_compliance,
)


def _signals(float_position: float, float_force: float) -> dict[str, list[float]]:
    values = {
        "float_position_m": float_position,
        "float_velocity_m_s": 0.0,
        "float_force_n": float_force,
        "pitch_position_rad": 0.01,
        "pitch_velocity_rad_s": 0.0,
        "pitch_torque_nm": -0.32,
    }
    assert set(values) == set(SQUEEGEE_SIGNALS)
    return {name: [value] * 12 for name, value in values.items()}


def _passing_evidence():
    phase_signals = {
        "raised_free": _signals(-0.0135, 13.5),
        "grounded_preload": _signals(0.0002, -11.16),
        "raised_recovery": _signals(-0.0133, 13.14),
    }
    phase_joints = {
        phase: {
            "squeegee_float_joint": list(signals["float_position_m"]),
            "squeegee_pitch_joint": list(signals["pitch_position_rad"]),
        }
        for phase, signals in phase_signals.items()
    }
    phase_contacts = {
        "raised_free": {"nonempty_messages": 0, "collision_pairs": set()},
        "grounded_preload": {
            "nonempty_messages": 12,
            "collision_pairs": {
                "ground_plane::link::collision <-> "
                "tzcup_formal_sanitation_vehicle::squeegee_link::squeegee_blade_collision"
            },
        },
        "raised_recovery": {"nonempty_messages": 0, "collision_pairs": set()},
    }
    return phase_signals, phase_joints, phase_contacts


def test_accepts_live_three_phase_preload_contact_and_recovery_chain() -> None:
    report, failures = evaluate_squeegee_compliance(*_passing_evidence())
    assert failures == []
    assert report["passed"] is True
    assert report["evidence_level"] == (
        "LIVE_GAZEBO_JOINT_FORCE_AND_RECOVERY_SEQUENCE"
    )
    assert all(report["checks"].values())


def test_reports_missing_contact_transport_without_fabricating_a_collision_pair() -> None:
    phase_signals, phase_joints, phase_contacts = _passing_evidence()
    phase_contacts["grounded_preload"]["collision_pairs"] = {
        "front_bumper::collision <-> cone::collision"
    }
    report, failures = evaluate_squeegee_compliance(
        phase_signals, phase_joints, phase_contacts
    )
    assert report["passed"] is True
    assert report["checks"]["grounded_blade_contact_transport_observed"] is False
    assert report["contact_transport"]["status"] == "UNAVAILABLE_EMPTY_STREAM"
    assert failures == []


def test_accepts_independent_physical_contact_evidence_when_transport_is_empty() -> None:
    phase_signals, phase_joints, phase_contacts = _passing_evidence()
    phase_contacts["grounded_preload"] = {
        "nonempty_messages": 0,
        "collision_pairs": set(),
    }
    report, failures = evaluate_squeegee_compliance(
        phase_signals, phase_joints, phase_contacts
    )
    assert failures == []
    assert report["passed"] is True
    assert report["checks"]["grounded_blade_has_physical_contact"] is True
    assert report["contact_transport"]["status"] == "UNAVAILABLE_EMPTY_STREAM"


def test_rejects_static_joint_and_force_values_that_never_compress_or_recover() -> None:
    phase_signals, phase_joints, phase_contacts = _passing_evidence()
    phase_signals["grounded_preload"] = _signals(-0.0135, 0.0)
    phase_joints["raised_recovery"].pop("squeegee_float_joint")
    report, failures = evaluate_squeegee_compliance(
        phase_signals, phase_joints, phase_contacts
    )
    assert report["passed"] is False
    assert "ground_contact_compresses_float_suspension" in failures
    assert "grounded_preload_is_downward" in failures
    assert "squeegee_raised_recovery_joint_state_missing" in failures


def test_rejects_joint_limit_and_effort_overrun() -> None:
    phase_signals, phase_joints, phase_contacts = _passing_evidence()
    phase_signals["raised_free"]["float_position_m"][0] = -0.016
    phase_signals["grounded_preload"]["pitch_torque_nm"][0] = 25.0
    report, failures = evaluate_squeegee_compliance(
        phase_signals, phase_joints, phase_contacts
    )
    assert report["passed"] is False
    assert "float_travel_stays_within_joint_limits" in failures
    assert "bounded_compliance_effort" in failures
