import math

import pytest

from sanitation_tasks.localization_metrics import (
    TRANSFORM_CONVENTION,
    apply_se2,
    canonical_payload_sha256,
    compose_se2,
    invert_se2,
    load_map_calibration,
    map_gate_result,
    particle_statistics,
    particle_topic_type_pass,
)


def test_se2_inverse_and_composition_round_trip():
    transform = {"x_m": 1.25, "y_m": -0.75, "yaw_rad": 0.42}
    inverse = invert_se2(transform)
    identity = compose_se2(transform, inverse)
    assert identity["x_m"] == pytest.approx(0.0, abs=1.0e-12)
    assert identity["y_m"] == pytest.approx(0.0, abs=1.0e-12)
    assert identity["yaw_rad"] == pytest.approx(0.0, abs=1.0e-12)
    pose = (3.0, -2.0, -0.3)
    assert apply_se2(inverse, apply_se2(transform, pose)) == pytest.approx(pose)


def test_transform_direction_is_observable():
    transform = {"x_m": 2.0, "y_m": 0.0, "yaw_rad": math.pi / 2.0}
    assert apply_se2(transform, (1.0, 0.0, 0.0)) == pytest.approx(
        (2.0, 1.0, math.pi / 2.0)
    )
    assert apply_se2(invert_se2(transform), (1.0, 0.0, 0.0)) != pytest.approx(
        (2.0, 1.0, math.pi / 2.0)
    )


def test_calibration_rejects_units_and_inverse_errors(tmp_path):
    document = {
        "schema_version": 1,
        "transform_convention": TRANSFORM_CONVENTION,
        "T_map_gt_map": {"x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.1},
        "T_map_map_gt": invert_se2({"x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.1}),
    }
    document["calibration_payload_sha256"] = canonical_payload_sha256(document)
    path = tmp_path / "calibration.yaml"
    import yaml

    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    assert load_map_calibration(path)["T_map_gt_map"]["x_m"] == 1.0

    document["T_map_map_gt"]["x_m"] += 100.0  # metre/radian or direction mistake
    document["calibration_payload_sha256"] = canonical_payload_sha256(
        {key: value for key, value in document.items() if key != "calibration_payload_sha256"}
    )
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match="not inverses"):
        load_map_calibration(path)


def test_particle_statistics_uses_weights_and_reports_ess():
    report = particle_statistics(
        [(0.0, 0.0, 0.0, 0.9), (10.0, 0.0, 0.0, 0.1)]
    )
    assert report["valid"]
    assert report["weighted_mean"]["x_m"] == pytest.approx(1.0)
    assert report["effective_sample_size"] == pytest.approx(1.0 / 0.82)
    assert report["max_normalized_weight"] == pytest.approx(0.9)


def test_particle_statistics_fails_closed_without_weights():
    assert particle_statistics([])["reason"] == "empty_particle_cloud"
    assert particle_statistics([(0.0, 0.0, 0.0, 0.0)])["reason"] == "zero_weight_sum"


def test_particle_cloud_topic_type_accepts_nav2_and_rejects_pose_array():
    assert particle_topic_type_pass(["nav2_msgs/msg/ParticleCloud"])
    assert not particle_topic_type_pass(["geometry_msgs/msg/PoseArray"])


def test_stage4t_selected_map_is_not_localization_grade():
    gate = map_gate_result(
        {"success": True, "route_completed": True},
        {"slam_quality_pass": True},
        {
            "map_resolution_m": 0.05,
            "boundary_chamfer_distance_m": 0.09403,
            "boundary_p95_m": 0.26926,
            "straight_line_angle_error_deg": {"rmse": 29.89},
            "loop_ghosting_ratio": 0.01966,
            "occupancy_iou": 0.16515,
        },
    )
    assert gate["map_generation_pass"]
    assert gate["map_basic_quality_pass"]
    assert not gate["map_localization_geometry_pass"]
