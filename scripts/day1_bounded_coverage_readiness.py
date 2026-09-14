"""Pure pre-coverage readiness contract for the bounded cleaning bridge."""

from __future__ import annotations

import math
from typing import Any, Mapping, Sequence


PRECOVERAGE_LIFT_POSITION_M = 0.095
MIN_CONTACT_CLEARANCE_M = -0.004
MAX_CONTACT_CLEARANCE_M = 0.015


def _number(value: object) -> float | None:
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(float(value))
    ):
        return None
    return float(value)


def precoverage_actuator_readiness(
    samples: Sequence[Mapping[str, Any]],
    permitted: bool,
) -> dict[str, Any]:
    """Evaluate actuator readiness without requiring a mission brush command.

    The ground-dirt plugin's ``*_ready`` flags intentionally require the tools
    to be lowered, in contact, and rotating.  Rotation is commanded only after
    the coverage probe publishes ``/brush_enabled``, so those flags are
    post-start cleaning evidence, not a valid pre-probe arming gate.

    Before the probe starts, require the live status/ledger, safety permit,
    work-pose lift, and physical contact clearances.  The full rotating-tool
    flags remain independently required in the bounded mission summary.
    """

    latest = dict(samples[-1]) if samples else {}
    lift_position = _number(latest.get("lift_position_m"))
    left_clearance = _number(latest.get("left_clearance_m"))
    right_clearance = _number(latest.get("right_clearance_m"))
    roller_clearance = _number(latest.get("roller_clearance_m"))
    cell_count = latest.get("cell_count")
    checks = {
        "status_sample_observed": bool(samples),
        "safety_permit_observed": bool(permitted),
        "dirt_system_enabled": latest.get("enabled") is True,
        "dirt_cell_layout_ready": latest.get("cell_layout_ready") is True,
        "dirt_cells_present": (
            isinstance(cell_count, int)
            and not isinstance(cell_count, bool)
            and cell_count > 0
        ),
        "lift_at_work_pose": (
            lift_position is not None
            and lift_position >= PRECOVERAGE_LIFT_POSITION_M
        ),
        "left_contact_clearance_valid": (
            left_clearance is not None
            and MIN_CONTACT_CLEARANCE_M
            <= left_clearance
            <= MAX_CONTACT_CLEARANCE_M
        ),
        "right_contact_clearance_valid": (
            right_clearance is not None
            and MIN_CONTACT_CLEARANCE_M
            <= right_clearance
            <= MAX_CONTACT_CLEARANCE_M
        ),
        "roller_contact_clearance_valid": (
            roller_clearance is not None
            and MIN_CONTACT_CLEARANCE_M
            <= roller_clearance
            <= MAX_CONTACT_CLEARANCE_M
        ),
    }
    return {
        "ready": all(checks.values()),
        "checks": checks,
        "sample": latest or None,
        "mission_tool_ready_flags_required": True,
    }
