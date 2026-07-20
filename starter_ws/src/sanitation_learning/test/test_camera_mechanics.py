from pathlib import Path

import pytest
import yaml

from sanitation_learning.camera_mechanics import evaluate_all, pitched_camera_aabb


ROOT = Path(__file__).resolve().parents[1]


def test_pitched_camera_aabb_rotates_length_into_height():
    flat = pitched_camera_aabb((0, 0, 0), (0.10, 0.20, 0.04), 0)
    pitched = pitched_camera_aabb((0, 0, 0), (0.10, 0.20, 0.04), 90)
    assert flat.half == (0.05, 0.10, 0.02)
    assert abs(pitched.half[0] - 0.02) < 1e-9
    assert abs(pitched.half[2] - 0.05) < 1e-9


def test_stage5br5_camera_grid_uses_exact_collision_geometry_and_trial_footprint():
    document = yaml.safe_load((ROOT / "config" / "stage5br5_active_observation.yaml").read_text(encoding="utf-8"))
    report = evaluate_all(document)
    assert not report["all_candidates_mechanical_gate_pass"]
    assert report["mechanically_viable_candidates"] == ["V1", "V2", "V4"]
    assert report["mechanically_pruned_candidates"] == ["V3"]
    assert report["mechanical_grid_has_viable_candidate"]
    assert report["production_nav2_footprint_unchanged"]
    for camera_id, result in report["camera_results"].items():
        assert result["base_link_xyz_m"][0] - 0.575 == pytest.approx(result["front_bumper_relative_xyz_m"][0])
        assert result["collision_free"]
        assert not result["inside_current_production_nav2_footprint"], camera_id
        assert result["production_footprint_change_applied"] is False
    assert not report["camera_results"]["V3"]["inside_stage5br5_trial_footprint"]
