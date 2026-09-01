#!/usr/bin/env python3
"""ROS-independent evaluator for live squeegee compliance evidence."""

from __future__ import annotations

import math
import statistics


SQUEEGEE_FLOAT_REFERENCE_M = -0.006
SQUEEGEE_FLOAT_LIMIT_M = 0.015
SQUEEGEE_PITCH_LIMIT_RAD = 0.174533
SQUEEGEE_SIGNALS = (
    "float_position_m",
    "float_velocity_m_s",
    "float_force_n",
    "pitch_position_rad",
    "pitch_velocity_rad_s",
    "pitch_torque_nm",
)


def _median(values: list[float]) -> float | None:
    finite = [value for value in values if math.isfinite(value)]
    return statistics.median(finite) if finite else None


def evaluate_squeegee_compliance(
    phase_signals: dict[str, dict[str, list[float]]],
    phase_joint_positions: dict[str, dict[str, list[float]]],
    phase_contacts: dict[str, dict[str, object]],
) -> tuple[dict[str, object], list[str]]:
    """Evaluate measured free, preloaded contact and post-contact recovery states."""
    failures: list[str] = []
    phases: dict[str, object] = {}
    for phase in ("raised_free", "grounded_preload", "raised_recovery"):
        signals = phase_signals.get(phase, {})
        joint_positions = phase_joint_positions.get(phase, {})
        summary = {
            "telemetry_samples": {
                name: len(signals.get(name, [])) for name in SQUEEGEE_SIGNALS
            },
            "float_position_median_m": _median(signals.get("float_position_m", [])),
            "float_force_median_n": _median(signals.get("float_force_n", [])),
            "pitch_position_abs_max_rad": max(
                (abs(value) for value in signals.get("pitch_position_rad", [])),
                default=None,
            ),
            "pitch_torque_abs_max_nm": max(
                (abs(value) for value in signals.get("pitch_torque_nm", [])),
                default=None,
            ),
            "joint_state_float_median_m": _median(
                joint_positions.get("squeegee_float_joint", [])
            ),
            "joint_state_pitch_median_rad": _median(
                joint_positions.get("squeegee_pitch_joint", [])
            ),
            "nonempty_contact_messages": int(
                phase_contacts.get(phase, {}).get("nonempty_messages", 0)
            ),
            "contact_pairs": sorted(
                phase_contacts.get(phase, {}).get("collision_pairs", set())
            ),
        }
        phases[phase] = summary
        if any(count < 10 for count in summary["telemetry_samples"].values()):
            failures.append(f"squeegee_{phase}_telemetry_missing")
        if summary["joint_state_float_median_m"] is None or summary["joint_state_pitch_median_rad"] is None:
            failures.append(f"squeegee_{phase}_joint_state_missing")

    free = phases["raised_free"]
    work = phases["grounded_preload"]
    recovery = phases["raised_recovery"]
    free_float = free["float_position_median_m"]
    work_float = work["float_position_median_m"]
    recovered_float = recovery["float_position_median_m"]
    work_force = work["float_force_median_n"]

    checks = {
        "raised_free_relaxes_near_reference": free_float is not None
        and abs(free_float - SQUEEGEE_FLOAT_REFERENCE_M) <= 0.0035,
        "grounded_blade_has_real_contact": work["nonempty_contact_messages"] >= 5
        and any(
            "squeegee" in pair.lower() and "ground" in pair.lower()
            for pair in work["contact_pairs"]
        ),
        "ground_contact_compresses_float_suspension": free_float is not None
        and work_float is not None
        and work_float - free_float >= 0.002,
        "grounded_preload_is_downward": work_force is not None and work_force <= -3.0,
        "raised_recovery_returns_to_free_state": free_float is not None
        and recovered_float is not None
        and abs(recovered_float - free_float) <= 0.0035,
        "raised_recovery_has_no_blade_contact": recovery["nonempty_contact_messages"] == 0,
        "float_travel_stays_within_joint_limits": all(
            abs(value) <= SQUEEGEE_FLOAT_LIMIT_M + 1.0e-4
            for signals in phase_signals.values()
            for value in signals.get("float_position_m", [])
        ),
        "pitch_travel_stays_within_joint_limits": all(
            abs(value) <= SQUEEGEE_PITCH_LIMIT_RAD + 1.0e-4
            for signals in phase_signals.values()
            for value in signals.get("pitch_position_rad", [])
        ),
        "bounded_compliance_effort": all(
            abs(value) <= 120.0 + 1.0e-6
            for signals in phase_signals.values()
            for value in signals.get("float_force_n", [])
        ) and all(
            abs(value) <= 24.0 + 1.0e-6
            for signals in phase_signals.values()
            for value in signals.get("pitch_torque_nm", [])
        ),
    }
    for name, passed in checks.items():
        if not passed:
            failures.append(name)
    return {
        "evidence_level": "LIVE_GAZEBO_JOINT_FORCE_CONTACT_AND_RECOVERY_SEQUENCE",
        "passive_joint_command_interfaces": [],
        "phases": phases,
        "checks": checks,
        "passed": all(checks.values()) and not any(
            failure.startswith("squeegee_") for failure in failures
        ),
    }, failures
