import pytest

from sanitation_gazebo_visualization.telemetry_v2 import (
    SCHEMA, classify_motion_state, validate_telemetry_v2,
)


def test_motion_states_are_kept_in_separate_layers():
    assert classify_motion_state("EXECUTING_SWATH", True) == "cleaning"
    assert classify_motion_state("EXECUTING_SHIFT", False) == "transit"
    assert classify_motion_state("REPAIR_SWATH", True) == "repair"


def test_v2_contract_requires_all_semantic_path_layers():
    paths = {name: [] for name in (
        "planned_swaths", "planned_connectors", "planned_repairs",
        "current_component", "actual_cleaning", "actual_transit", "actual_repair",
        "blocked_intervals",
    )}
    payload = {
        "schema": SCHEMA,
        "paths": paths,
        "blocked_intervals": [],
        "deferred_swaths": [],
    }
    assert validate_telemetry_v2(payload)
    paths.pop("actual_repair")
    with pytest.raises(ValueError, match="incomplete"):
        validate_telemetry_v2(payload)


def test_v2_contract_requires_blocked_and_deferred_state_layers():
    paths = {name: [] for name in (
        "planned_swaths", "planned_connectors", "planned_repairs",
        "current_component", "actual_cleaning", "actual_transit", "actual_repair",
        "blocked_intervals",
    )}
    with pytest.raises(ValueError, match="blocked_intervals"):
        validate_telemetry_v2({
            "schema": SCHEMA, "paths": paths, "deferred_swaths": [],
        })
