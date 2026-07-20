from pathlib import Path

from sanitation_learning.observability import (
    build_report,
    camera_ground_geometry,
    load_policy,
    recognition_ready,
    summarize_rows,
)


ROOT = Path(__file__).resolve().parents[1]
POLICY = ROOT / "config" / "perception_evaluability_policy.yaml"
CONFIG = ROOT / "config" / "stage5br4_active_perception.yaml"


def test_ready_policy_separates_discrete_and_area_without_hiding_non_ready():
    policy = load_policy(POLICY)
    rows = [
        {"class_id": "plastic_bottle", "visibility": "visible", "bbox_shortest_side_px": 12, "mask_area_px": 80, "distance_m": 2.5},
        {"class_id": "plastic_bottle", "visibility": "visible", "bbox_shortest_side_px": 11, "mask_area_px": 100, "distance_m": 1.0},
        {"class_id": "leaf_pile", "visibility": "visible", "bbox_shortest_side_px": 5, "mask_area_px": 256, "distance_m": 4.0},
    ]
    assert recognition_ready(rows[0], policy)
    assert not recognition_ready(rows[1], policy)
    assert recognition_ready(rows[2], policy)
    summary = summarize_rows(rows, policy)
    assert summary["all_visible"]["count"] == 3
    assert summary["recognition_ready"]["count"] == 2
    assert summary["non_ready"]["count"] == 1


def test_downward_camera_has_finite_ground_polygon_and_dual_bandwidth():
    geometry = camera_ground_geometry(
        {"xyz_m": [0.53, 0.0, 0.70], "pitch_deg": -30.0},
        {"resolution": [640, 480], "horizontal_fov_rad": 1.50098, "near_clip_m": 0.3, "far_clip_m": 100.0},
    )
    assert geometry["near_field_blind_zone_m_from_front_bumper"] >= 0.0
    assert len(geometry["ground_coverage_polygon_base_xy_m"]) == 4
    report = build_report([], POLICY, CONFIG, "C3")
    assert report["estimated_uncompressed_rgbd_bandwidth_mbps"] == 294.912
    assert report["self_pixel_fraction"] is None
