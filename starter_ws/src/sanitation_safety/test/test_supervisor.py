import json
from types import SimpleNamespace

import pytest

from sanitation_safety.supervisor import (
    ProductSupervisorState,
    SourceRule,
    coverage_health,
    has_localization_transform,
    localization_health,
    perception_health,
)


RULES = (
    SourceRule("localization", "motion", 0.5),
    SourceRule("perception", "cleaning", 1.0),
)


def test_missing_or_stale_motion_source_fails_closed() -> None:
    state = ProductSupervisorState(rules=RULES)
    missing = state.snapshot(0.0)
    assert missing["state"] == "ERROR"
    assert missing["motion_faults"] == ["localization:missing"]

    state.observe("localization", 1.0)
    state.observe("perception", 1.0)
    assert state.snapshot(1.2)["state"] == "ACTIVE"
    stale = state.snapshot(1.6)
    assert stale["state"] == "ERROR"
    assert stale["motion_faults"] == ["localization:stale"]


def test_cleaning_fault_degrades_without_disabling_motion() -> None:
    state = ProductSupervisorState(rules=RULES)
    state.observe("localization", 4.0)
    state.observe(
        "perception", 4.0, healthy=False, reason="state_degraded"
    )
    snapshot = state.snapshot(4.1)
    assert snapshot["state"] == "DEGRADED"
    assert snapshot["motion_healthy"] is True
    assert snapshot["cleaning_healthy"] is False
    assert snapshot["cleaning_faults"] == ["perception:state_degraded"]


def test_parsers_reject_malformed_and_unhealthy_sources() -> None:
    assert perception_health("not-json") == (False, "invalid_json")
    assert perception_health(json.dumps({
        "state": "ACTIVE", "perception_spot_clean_allowed": True
    })) == (True, "state_active")
    assert perception_health(json.dumps({
        "state": "DEGRADED", "perception_spot_clean_allowed": False
    })) == (False, "state_degraded")
    assert coverage_health("FAILED") == (False, "state_failed")
    assert coverage_health("READY") == (True, "state_ready")
    assert localization_health([0.1] + [0.0] * 6 + [0.1]) == (True, "ok")
    assert localization_health([0.2] + [0.0] * 6 + [0.1]) == (
        False, "covariance_excessive"
    )


def test_rules_reject_unknown_planes_and_names() -> None:
    with pytest.raises(ValueError):
        SourceRule("x", "unknown", 1.0)
    state = ProductSupervisorState(rules=RULES)
    with pytest.raises(KeyError):
        state.observe("not-configured", 0.0)


def test_only_map_to_odom_counts_as_localization_heartbeat() -> None:
    def transform(parent: str, child: str):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id=parent),
            child_frame_id=child,
        )

    assert has_localization_transform([transform("map", "odom")]) is True
    assert has_localization_transform([transform("/map", "/odom")]) is True
    assert has_localization_transform([transform("odom", "base_footprint")]) is False
