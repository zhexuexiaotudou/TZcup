import pytest

from sanitation_gazebo_visualization.telemetry_v2 import (
    SCHEMA, classify_motion_state, measured_motion, validate_telemetry_v2,
    resolve_cleanable_polygon, virtual_steering_angle,
)


def test_motion_states_are_kept_in_separate_layers():
    assert classify_motion_state("EXECUTING_SWATH", True) == "cleaning"
    assert classify_motion_state("EXECUTING_SHIFT", False) == "transit"
    assert classify_motion_state("REPAIR_SWATH", True) == "repair"


def test_v2_contract_requires_all_semantic_path_layers():
    paths = {name: [] for name in (
        "planned_swaths", "planned_connectors", "planned_repairs",
        "current_component", "actual_cleaning", "actual_transit", "actual_repair",
        "blocked_intervals", "planned_ackermann_forward",
        "planned_ackermann_reverse", "actual_forward", "actual_reverse",
    )}
    payload = {
        "schema": SCHEMA,
        "paths": paths,
        "blocked_intervals": [],
        "deferred_swaths": [],
        "steering": {
            "front_left_rad": 0.0, "front_right_rad": 0.0,
            "virtual_rad": 0.0, "configured_min_radius_m": 1.429352,
        },
        "motion": {"gear": "STOP"},
    }
    assert validate_telemetry_v2(payload)
    paths.pop("actual_repair")
    with pytest.raises(ValueError, match="incomplete"):
        validate_telemetry_v2(payload)


def test_v2_contract_requires_blocked_and_deferred_state_layers():
    paths = {name: [] for name in (
        "planned_swaths", "planned_connectors", "planned_repairs",
        "current_component", "actual_cleaning", "actual_transit", "actual_repair",
        "blocked_intervals", "planned_ackermann_forward",
        "planned_ackermann_reverse", "actual_forward", "actual_reverse",
    )}
    with pytest.raises(ValueError, match="blocked_intervals"):
        validate_telemetry_v2({
            "schema": SCHEMA, "paths": paths, "deferred_swaths": [],
        })


def test_ackermann_joint_angles_recover_virtual_steering_angle():
    # 0.76 m wheelbase, 0.80 m track, 28 degree virtual command.
    left = 0.635989405
    right = 0.393751965
    assert virtual_steering_angle(left, right) == pytest.approx(0.488692191, abs=1e-5)
    assert virtual_steering_angle(-right, -left) == pytest.approx(-0.488692191, abs=1e-5)


def test_measured_motion_exposes_gear_curvature_and_radius():
    forward = measured_motion(0.50, 0.25)
    assert forward == {
        "gear": "FORWARD", "curvature_1pm": 0.5, "turning_radius_m": 2.0,
    }
    reverse = measured_motion(-0.25, 0.125)
    assert reverse["gear"] == "REVERSE"
    assert reverse["curvature_1pm"] == pytest.approx(-0.5)
    assert measured_motion(0.01, 2.0)["gear"] == "STOP"


def test_explicit_cleanable_polygon_drives_demo_progress_denominator():
    config = {
        "outer_polygon": [[-6.3, -6.2], [6.3, -6.2], [6.3, 3.0], [-6.3, 3.0]],
        "cleanable_outer_polygon": [[-2.0, -3.0], [2.0, -3.0], [2.0, 0.0], [-2.0, 0.0]],
        "headland": {"width_m": 2.0},
    }
    assert resolve_cleanable_polygon(config) == [
        (-2.0, -3.0), (2.0, -3.0), (2.0, 0.0), (-2.0, 0.0)
    ]
