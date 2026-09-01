from __future__ import annotations

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_both_nav2_profiles_use_mid360_for_3d_obstacle_marking() -> None:
    for relative in (
        "starter_ws/src/sanitation_navigation/config/nav2.yaml",
        "starter_ws/src/sanitation_navigation/config/nav2_auto12.yaml",
    ):
        config = yaml.safe_load((ROOT / relative).read_text(encoding="utf-8"))
        for costmap_name in ("local_costmap", "global_costmap"):
            params = config[costmap_name][costmap_name]["ros__parameters"]
            obstacle = params["obstacle_layer"]
            assert set(obstacle["observation_sources"].split()) == {
                "scan",
                "mid360",
            }
            mid360 = obstacle["mid360"]
            assert mid360["topic"] == "/sensors/lidar_3d/points"
            assert mid360["data_type"] == "PointCloud2"
            assert mid360["clearing"] is True
            assert mid360["marking"] is True
            assert 0.0 < mid360["min_obstacle_height"] < 0.2
            assert mid360["max_obstacle_height"] >= 2.0
            assert mid360["raytrace_max_range"] > mid360["obstacle_max_range"]

        collision = config["collision_monitor"]["ros__parameters"]
        assert collision["observation_sources"] == ["scan", "mid360"]
        collision_mid360 = collision["mid360"]
        assert collision_mid360["type"] == "pointcloud"
        assert collision_mid360["topic"] == "/sensors/lidar_3d/points"
        assert collision_mid360["enabled"] is True
        assert 0.0 < collision_mid360["min_height"] < 0.2
        assert collision_mid360["max_height"] >= 2.0


def test_mid360_is_not_mislabeled_as_the_mapping_source() -> None:
    register = yaml.safe_load(
        (ROOT / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml")
        .read_text(encoding="utf-8")
    )
    semantics = register["navigation_sensor_semantics"]
    assert semantics["mapping"]["primary_sensor_installation"] == "hokuyo_utm30lx"
    assert semantics["mapping"]["dimensionality"] == "2d_occupancy_grid"
    assert semantics["obstacle_perception"]["primary_sensor_installation"] == "livox_mid360"
    assert semantics["obstacle_perception"]["mapping_claim"] == "prohibited"
