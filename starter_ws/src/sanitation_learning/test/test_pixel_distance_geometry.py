from pathlib import Path

import pytest

from sanitation_learning.oprv3_geometry import derive_product_geometry


ROOT = Path(__file__).resolve().parents[4]


def test_geometry_audit_reads_frozen_product_camera_and_navigation_sources():
    report = derive_product_geometry(ROOT)
    camera = report["camera"]
    vehicle = report["vehicle_and_action"]
    assert camera["profile_id"] == "auto05r_v5_retracted_primary_perception_v1"
    assert camera["resolution"] == [640, 480]
    assert camera["horizontal_fov_rad"] == pytest.approx(1.50098)
    assert camera["base_link_xyz_m"] == [0.36, 0.0, 0.66]
    assert camera["pitch_deg"] == -50.0
    assert vehicle["normal_product_speed_m_s"] == pytest.approx(0.65)
    assert vehicle["maximum_deceleration_m_s2"] == pytest.approx(0.90)


def test_pixel_distance_levels_are_monotonic_and_use_true_g4_sizes():
    report = derive_product_geometry(ROOT)
    for class_id, window in report["class_actionable_windows"].items():
        distances = window["distance_at_bbox_short_side_px_m"]
        assert distances["8"] > distances["12"] > distances["18"] > distances["24"] > distances["32"]
        assert report["target_geometry"][class_id]["source"].endswith("GEOMETRY_PARAMS")
        assert report["target_geometry"][class_id]["variant_count"] >= 6
