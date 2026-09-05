import importlib.util
import json
import os
from pathlib import Path

import yaml


TEST_PATH = Path(__file__).resolve()
ROOT = next(
    (
        parent
        for parent in TEST_PATH.parents
        if (parent / "scripts" / "stage5br6w_profile.py").is_file()
    ),
    Path(os.environ.get("TZCUP_ROOT", "/auto01")),
)
SPEC = importlib.util.spec_from_file_location(
    "stage5br6w_profile", ROOT / "scripts" / "stage5br6w_profile.py"
)
PROFILE_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROFILE_MODULE)


def test_g2_probe_uses_simulation_clock_qos():
    source = (ROOT / "scripts/auto01_g2_obstacle_probe.py").read_text(encoding="utf-8")
    assert "from rclpy.qos import qos_profile_sensor_data" in source
    assert 'Clock,\n            "/clock",' in source
    assert "qos_profile_sensor_data," in source


def _inputs():
    profile = yaml.safe_load(
        (
            ROOT
            / "starter_ws/src/sanitation_navigation/config/auto01_g1_height_banded.yaml"
        ).read_text(encoding="utf-8")
    )
    nav2 = yaml.safe_load(
        (ROOT / "starter_ws/src/sanitation_navigation/config/nav2.yaml").read_text(
            encoding="utf-8"
        )
    )
    mission = yaml.safe_load(
        (ROOT / "starter_ws/src/sanitation_tasks/config/demo_area.yaml").read_text(
            encoding="utf-8"
        )
    )
    return profile, nav2, mission


def _g2_inputs():
    profile = yaml.safe_load(
        (
            ROOT
            / "starter_ws/src/sanitation_navigation/config/auto01_g2_v5_retracted.yaml"
        ).read_text(encoding="utf-8")
    )
    nav2 = yaml.safe_load(
        (ROOT / "starter_ws/src/sanitation_navigation/config/nav2.yaml").read_text(
            encoding="utf-8"
        )
    )
    mission = yaml.safe_load(
        (ROOT / "starter_ws/src/sanitation_tasks/config/demo_area.yaml").read_text(
            encoding="utf-8"
        )
    )
    return profile, nav2, mission


def test_g1_splits_ground_navigation_and_high_overhang_envelopes():
    profile, nav2, mission = _inputs()
    materialized_nav2 = PROFILE_MODULE.materialize_nav2(nav2, profile)
    materialized_mission = PROFILE_MODULE.materialize_mission(mission, profile)
    navigation = profile["navigation_footprint_xy_m"]
    assert json.loads(
        materialized_nav2["local_costmap"]["local_costmap"]["ros__parameters"][
            "footprint"
        ]
    ) == navigation
    assert materialized_mission["robot_footprint"] == navigation
    monitor = materialized_nav2["collision_monitor"]["ros__parameters"]
    ground_monitor = materialized_nav2["ground_collision_monitor"]["ros__parameters"]
    assert ground_monitor["polygons"] == ["FootprintApproach"]
    assert ground_monitor["cmd_vel_out_topic"] == "/cmd_vel_ground_safe"
    assert ground_monitor["source_timeout"] == 5.0
    assert ground_monitor["scan"]["topic"] == "/scan/navigation"
    for costmap_name in ("local_costmap", "global_costmap"):
        assert materialized_nav2[costmap_name][costmap_name]["ros__parameters"][
            "obstacle_layer"
        ]["scan"]["topic"] == "/scan/navigation"
    scan_filter = materialized_nav2["scan_self_filter"]["ros__parameters"]
    assert scan_filter["input_topic"] == "/scan"
    assert scan_filter["output_topic"] == "/scan/navigation"
    assert ground_monitor["observation_sources"] == ["scan", "ground_cloud"]
    assert ground_monitor["ground_cloud"]["topic"] == "/camera/depth/color/points"
    assert [
        ground_monitor["ground_cloud"]["min_height"],
        ground_monitor["ground_cloud"]["max_height"],
    ] == profile["ground_obstacles"]["z_band_base_link_m"]
    assert monitor["cmd_vel_in_topic"] == "/cmd_vel_ground_safe"
    assert monitor["source_timeout"] == 5.0
    assert monitor["polygons"] == ["HighOverhangStop"]
    assert json.loads(monitor["HighOverhangStop"]["points"]) == profile[
        "high_overhang"
    ]["collision_polygon_xy_m"]
    assert monitor["HighOverhangStop"]["action_type"] == "stop"
    assert monitor["HighOverhangStop"]["min_points"] == 1
    cloud = monitor["high_overhang_cloud"]
    assert cloud["type"] == "pointcloud"
    assert cloud["topic"] == "/camera/depth/color/points"
    assert [cloud["min_height"], cloud["max_height"]] == profile["high_overhang"][
        "z_band_base_link_m"
    ]


def test_g1_pointcloud_height_filter_matches_v4_camera_height_band():
    profile, _, _ = _inputs()
    high = profile["high_overhang"]
    lower, upper = high["z_band_base_link_m"]
    assert lower <= high["lidar_plane_base_link_m"] <= upper


def test_g1_high_overhang_polygon_is_camera_local_not_union_bounding_box():
    profile, _, _ = _inputs()
    navigation = profile["navigation_footprint_xy_m"]
    high = profile["high_overhang"]["collision_polygon_xy_m"]
    assert min(point[0] for point in high) > max(point[0] for point in navigation)
    assert min(point[1] for point in high) > 0.0


def test_g2_v5_camera_is_inside_navigation_envelope_and_above_lidar():
    profile, nav2, mission = _g2_inputs()
    camera = profile["camera_mechanical_reconstruction"]
    minimum = camera["rotated_aabb_min_m"]
    maximum = camera["rotated_aabb_max_m"]
    footprint = profile["navigation_footprint_xy_m"]
    assert min(point[0] for point in footprint) <= minimum[0]
    assert maximum[0] <= max(point[0] for point in footprint)
    assert min(point[1] for point in footprint) <= minimum[1]
    assert maximum[1] <= max(point[1] for point in footprint)
    assert minimum[2] > camera["lidar_plane_base_link_m"]
    assert maximum[2] <= camera["maximum_mount_height_m"]
    assert camera["collision_free_from_body_bumper_arm_and_brush"]
    materialized = PROFILE_MODULE.materialize_nav2(nav2, profile)
    monitor = materialized["collision_monitor"]["ros__parameters"]
    assert monitor["polygons"] == ["FootprintApproach"]
    assert monitor["observation_sources"] == ["scan", "ground_cloud"]
    assert (
        monitor["ground_cloud"]["topic"]
        == "/verification_camera/depth/color/points/navigation"
    )
    self_filter = materialized["pointcloud_self_filter"]["ros__parameters"]
    assert (
        self_filter["input_topic"]
        == "/verification_camera/depth/color/points"
    )
    assert (
        self_filter["output_topic"]
        == "/verification_camera/depth/color/points/navigation"
    )
    assert self_filter["output_frame"] == "base_footprint"
    assert self_filter["mask_min_xyz_m"] == [-0.60, -0.43, -0.20]
    assert self_filter["mask_max_xyz_m"] == [0.72, 0.43, 0.75]
    assert self_filter["sampling_stride"] == 4
    assert monitor["source_timeout"] == 5.0
    assert "ground_collision_monitor" not in materialized
    assert "scan_self_filter" not in materialized
    materialized_mission = PROFILE_MODULE.materialize_mission(mission, profile)
    assert materialized_mission["robot_footprint"] == footprint


def test_g2_is_opt_in_and_production_launch_default_remains_production():
    profile, _, _ = _g2_inputs()
    assert profile["opt_in_only"]
    assert profile["production_default_unchanged"]
    launch_text = (
        ROOT / "starter_ws/src/sanitation_bringup/launch/sim.launch.py"
    ).read_text(encoding="utf-8")
    assert 'default_value="production"' in launch_text
    assert "V5_retracted" in launch_text
