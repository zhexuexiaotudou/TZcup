"""Pure fail-closed contract for the safety manager status JSON."""

from __future__ import annotations

import math
from collections.abc import Mapping


STATUS_JSON_KEYS = frozenset({
    "schema_version", "state", "safety_inputs_permit_actuators",
    "actuators_enabled", "managed_controllers_active", "active_reasons",
    "unsafe_generation", "consumed_unsafe_generation", "status_publish_count",
    "maximum_timer_gap_sec", "publish_thread_error",
    "effective_max_linear_velocity_mps", "operation_speed_profile",
    "speed_qualification_state",
})
OPERATION_SPEED_PROFILES = frozenset({
    "mapping_safe", "dry_cleaning_competition_candidate", "wet_puddle_recovery",
})
SPEED_QUALIFICATION_STATES = frozenset({"none", "isolated_same_map_dry_coverage"})


def validate_speed_qualification_status(payload: Mapping[str, object]) -> None:
    """Reject malformed or unqualified speed-cap telemetry."""

    cap = payload["effective_max_linear_velocity_mps"]
    if type(cap) is not float or not math.isfinite(cap) or not 0.0 < cap <= 1.0:
        raise ValueError("effective_max_linear_velocity_mps must be a finite float in (0, 1]")
    profile = payload["operation_speed_profile"]
    if type(profile) is not str or (
        profile and profile not in OPERATION_SPEED_PROFILES
    ):
        raise ValueError("operation_speed_profile is not a supported profile")
    state = payload["speed_qualification_state"]
    if type(state) is not str or state not in SPEED_QUALIFICATION_STATES:
        raise ValueError("speed_qualification_state is not a supported state")
    if cap > 0.45 and (
        cap != 1.0
        or profile != "dry_cleaning_competition_candidate"
        or state != "isolated_same_map_dry_coverage"
    ):
        raise ValueError("speed above 0.45 m/s requires the exact dry qualification")
