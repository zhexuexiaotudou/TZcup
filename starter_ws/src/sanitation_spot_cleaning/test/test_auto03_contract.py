import pytest

from sanitation_spot_cleaning.auto03_contract import (
    ORACLE_FIELDS,
    projection_measurement,
    summarize_auto03,
    validate_oracle_candidate,
)
from sanitation_spot_cleaning.auto03_matrix_probe import _angular_distance


def candidate():
    return {
        "candidate_id": "candidate",
        "x_m": 1.0,
        "y_m": -1.0,
        "target_size_m": 0.12,
        "class_id": "metal_can",
        "covariance_trace": 0.002,
        "timestamp_s": 10.0,
    }


def test_oracle_boundary_rejects_truth_pose_and_path_shortcuts():
    assert set(validate_oracle_candidate(candidate())) == ORACLE_FIELDS
    with pytest.raises(ValueError, match="unexpected"):
        validate_oracle_candidate({**candidate(), "observation_pose": [0.0, 0.0, 0.0]})
    with pytest.raises(ValueError, match="unexpected"):
        validate_oracle_candidate({**candidate(), "path_available": True})
    with pytest.raises(ValueError):
        validate_oracle_candidate({**candidate(), "covariance_trace": -1.0})


def test_projection_measurement_uses_actual_bbox():
    row = projection_measurement([90.0, 90.0, 130.0, 130.0], [100, 100, 20, 20])
    assert row["actual_target_center_inside_predicted_roi"]
    assert row["center_pixel_error"] == pytest.approx(0.0)
    assert row["short_side_relative_error"] == pytest.approx(1.0)


def test_projection_measurement_separates_search_roi_from_target_size():
    row = projection_measurement(
        [80.0, 80.0, 140.0, 140.0],
        [105, 105, 20, 20],
        predicted_target_short_side_px=18.0,
    )
    assert row["actual_target_center_inside_predicted_roi"]
    assert row["search_roi_short_side_px"] == 60.0
    assert row["predicted_short_side_px"] == 18.0
    assert row["short_side_relative_error"] == pytest.approx(0.1)


def test_angular_distance_wraps_for_yaw_handoff():
    assert _angular_distance(-1.1, 4.0) == pytest.approx(1.1831853071795866)
    assert _angular_distance(0.1, 0.2) == pytest.approx(0.1)


def test_acceptance_fails_closed_when_runtime_evidence_is_missing():
    truth = [{
        "candidate_id": "candidate",
        "world_id": "world_a",
        "scene_id": "scene_0",
        "class_id": "metal_can",
        "case_type": "reachable",
    }]
    report = summarize_auto03(truth, [])
    assert report["auto03_gate_pass"] is False
    assert report["checks"]["navigate_success_at_least_0_95"] is False
    assert report["checks"]["gt_control_violation_zero"] is True
    assert report["checks"]["median_extra_distance_at_most_8_m"] is False
    assert report["checks"]["median_extra_time_at_most_45_s"] is False
    assert report["checks"]["throughput_penalty_at_most_0_25"] is False
