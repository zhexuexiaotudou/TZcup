import ast
from copy import deepcopy
import inspect
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).parents[1]
ROOT = Path(__file__).parents[4]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_campus_scenario"))

from sanitation_formal_campus_integration.campus_materializer import (  # noqa: E402
    materialize_campus_artifacts,
)
from sanitation_formal_campus_integration.contract import (  # noqa: E402
    IntegrationContractError,
)
from sanitation_formal_campus_integration.map_lifecycle_core import (  # noqa: E402
    prepare_public_lifecycle_artifacts,
)
from sanitation_formal_campus_integration.saved_map_coverage_core import (  # noqa: E402
    ProductCoverageTelemetry,
)
from sanitation_campus_scenario.generator import (  # noqa: E402
    generate_episode,
    load_config,
)


MOTION_PROFILE = ROOT / "config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml"
LAUNCH = PACKAGE / "launch/formal_campus.launch.py"
SCENARIO_CONFIG = (
    ROOT / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml"
)


def _manifest() -> dict:
    return {
        "schema_version": 1,
        "episode_id": "formal-map-001-mission-002",
        "profile": "formal",
        "field": {
            "width_m": 10.0,
            "height_m": 6.0,
            "geofence_frame": "map",
            "geofence_polygon_m": [
                [-5.0, -3.0],
                [5.0, -3.0],
                [5.0, 3.0],
                [-5.0, 3.0],
            ],
        },
        "counts": {
            "static_assets": 3,
            "dirt_patches": 1,
            "discrete_cubes": 1,
            "pedestrians": 1,
        },
        "vehicle": {"included": False, "profile": None, "urdf_claim": False},
        "vehicle_start_pose_map": {"x_m": -3.0, "y_m": 0.0, "yaw_rad": 0.2},
    }


def _public_world() -> str:
    return """<?xml version="1.0"?>
<sdf version="1.10">
  <world name="campus_formal">
    <model name="ground_plane">
      <static>true</static>
      <link name="link"><collision name="ground"><geometry><plane>
        <normal>0 0 1</normal><size>12 8</size>
      </plane></geometry></collision></link>
    </model>
    <model name="opaque_building">
      <static>true</static><pose>1 0 1 0 0 0.3</pose><link name="link">
        <collision name="collision"><geometry><box><size>2 1 2</size></box></geometry></collision>
      </link>
    </model>
    <model name="opaque_pole">
      <static>true</static><pose>-0.5 1.2 3 0 0 0</pose><link name="link">
        <collision name="collision"><geometry><cylinder>
          <radius>0.12</radius><length>6</length>
        </cylinder></geometry></collision>
      </link>
    </model>
    <model name="opaque_bin">
      <static>true</static><pose>-0.5 -1.2 0.5 0 0 -0.2</pose><link name="link">
        <collision name="collision"><geometry><box>
          <size>0.7 0.6 1</size>
        </box></geometry></collision>
      </link>
    </model>
    <model name="surface_decal">
      <static>true</static><pose>2 -2 0.002 0 0 0</pose><link name="link">
        <visual name="dirt"><geometry><box><size>1 1 0.004</size></box></geometry></visual>
      </link>
    </model>
    <model name="dynamic_cube">
      <static>false</static><pose>2 2 0.015 0 0 0</pose><link name="link">
        <collision name="collision"><geometry><box>
          <size>0.03 0.03 0.03</size>
        </box></geometry></collision>
      </link>
    </model>
    <model name="walker_synthetic">
      <static>true</static><pose>0 2 0 0 0 0</pose><link name="link">
        <collision name="collision"><pose>0 0 0.9 0 0 0</pose><geometry><cylinder>
          <radius>0.25</radius><length>1.8</length>
        </cylinder></geometry></collision>
      </link>
    </model>
  </world>
</sdf>
"""


def _write_inputs(tmp_path: Path, manifest: dict | None = None) -> tuple[Path, Path]:
    manifest_path = tmp_path / "public" / "episode_manifest.json"
    world_path = tmp_path / "public" / "world.sdf"
    manifest_path.parent.mkdir()
    manifest_path.write_text(
        json.dumps(_manifest() if manifest is None else manifest), encoding="utf-8"
    )
    world_path.write_text(_public_world(), encoding="utf-8")
    return manifest_path, world_path


def _read_pgm(path: Path) -> tuple[int, int, bytes]:
    magic, dimensions, maximum, pixels = path.read_bytes().split(b"\n", 3)
    width, height = dimensions.split()
    assert magic == b"P5"
    assert maximum == b"255"
    return int(width), int(height), pixels


def _map_value(yaml_path: Path, x: float, y: float) -> int:
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    width, height, pixels = _read_pgm(yaml_path.parent / metadata["image"])
    origin_x, origin_y, _ = metadata["origin"]
    resolution = metadata["resolution"]
    column = math.floor((x - origin_x) / resolution)
    map_row = math.floor((y - origin_y) / resolution)
    image_row = height - 1 - map_row
    pixel = pixels[image_row * width + column]
    return round((255 - pixel) * 100 / 255)


def test_materializes_consistent_public_only_maps_and_formal_mission(tmp_path):
    manifest_path, world_path = _write_inputs(tmp_path)
    artifacts = materialize_campus_artifacts(
        manifest_path,
        world_path,
        MOTION_PROFILE,
        tmp_path / "runtime",
        resolution=0.25,
        safety_margin_m=0.15,
        slow_zone_width_m=1.0,
        slow_zone_percent=50,
    )

    map_metadata = [
        yaml.safe_load(path.read_text(encoding="utf-8"))
        for path in (
            artifacts.occupancy_map,
            artifacts.keepout_map,
            artifacts.speed_map,
        )
    ]
    assert {tuple(item["origin"]) for item in map_metadata} == {(-5.25, -3.25, 0.0)}
    assert {item["resolution"] for item in map_metadata} == {0.25}
    assert {_read_pgm(path.parent / item["image"])[:2] for path, item in zip(
        (artifacts.occupancy_map, artifacts.keepout_map, artifacts.speed_map),
        map_metadata,
    )} == {(42, 26)}
    assert artifacts.static_collision_count == 3
    assert artifacts.world_name == "campus_formal"

    for center in ((1.0, 0.0), (-0.5, 1.2), (-0.5, -1.2)):
        assert _map_value(artifacts.occupancy_map, *center) == 100
        assert _map_value(artifacts.keepout_map, *center) == 100
        assert _map_value(artifacts.speed_map, *center) == 0
    assert _map_value(artifacts.keepout_map, 3.5, 0.0) == 0
    assert _map_value(artifacts.speed_map, 3.5, 0.0) == 50
    assert _map_value(artifacts.occupancy_map, -5.0, 0.0) == 100
    assert _map_value(artifacts.keepout_map, -4.5, 0.0) == 100
    assert _map_value(artifacts.occupancy_map, -3.0, 0.0) == 0
    assert _map_value(artifacts.keepout_map, -3.0, 0.0) == 0

    mission = yaml.safe_load(artifacts.mission_geometry.read_text(encoding="utf-8"))
    profile = yaml.safe_load(MOTION_PROFILE.read_text(encoding="utf-8"))
    assert mission["outer_polygon"] == _manifest()["field"]["geofence_polygon_m"]
    assert mission["vehicle_start_pose_map"] == _manifest()["vehicle_start_pose_map"]
    assert mission["robot_footprint"] == (
        profile["motion_footprints"]["cleaning_deployed"]["footprint_xy_m"]
    )
    assert mission["operation_width_m"] == pytest.approx(1.32)
    assert mission["kinematic_model"] == "four_wheel_skid_steer"
    assert mission["planning_kinematic_constraint"] == (
        "curvature_limited_reference_path_for_skid_steer"
    )
    assert mission["physical_steering_claim"] is False
    assert len(mission["keepout_polygons"]) == 3
    assert len(mission["materialized_static_obstacles"]) == 3
    assert mission["truth_boundary"]["evaluator_truth_used"] is False
    assert mission["truth_boundary"]["dirt_truth_used"] is False
    assert "cleaning_targets" not in mission
    assert "dirt_patches" not in mission
    cleaning_radius = max(
        math.hypot(*point)
        for point in profile["motion_footprints"]["cleaning_deployed"][
            "footprint_xy_m"
        ]
    )
    assert mission["headland"]["width_m"] == pytest.approx(
        cleaning_radius + mission["safety_margin_m"] + mission["operation_width_m"] / 2.0
    )

    report = yaml.safe_load(artifacts.contract_report.read_text(encoding="utf-8"))
    assert report["formal_cleaning_footprint_radius_m"] == pytest.approx(
        cleaning_radius
    )
    assert report["resolution_contract"] == {
        "static_materializer": {"resolution_m": 0.25, "value_source": "materialize_campus_artifacts(resolution)", "purpose": "public_world_static_collision_raster"},
        "lifecycle_support_mask": {"value_source": "prepare_public_lifecycle_artifacts(resolution)", "purpose": "public_geofence_support_mask"},
        "slam_occupancy": {"value_source": "saved_map_metadata", "maximum_accepted_resolution_value_source": "map_lifecycle_core.MAXIMUM_SAVED_MAP_RESOLUTION_M", "purpose": "runtime_lidar_slam_occupancy"},
        "coverage_planning": {"value_source": "ProductCoverageTelemetry(raster_resolution_m)", "purpose": "saved_map_coverage_raster_planning"},
    }
    # Dirt visuals, a dynamic cube and a driver-moved static walker do not
    # become static occupancy.
    assert _map_value(artifacts.occupancy_map, 2.0, -2.0) == 0
    assert _map_value(artifacts.occupancy_map, 2.0, 2.0) == 0
    assert _map_value(artifacts.occupancy_map, 0.0, 2.0) == 0


@pytest.mark.parametrize("map_index", [0, 1])
def test_real_formal_generator_materializes_without_dynamic_truth_leakage(
    tmp_path, map_index
):
    config = load_config(SCENARIO_CONFIG)
    files = generate_episode(
        config,
        "formal",
        "train",
        map_index,
        0,
        include_proxy=False,
    )
    public = tmp_path / f"episode-{map_index}" / "public"
    public.mkdir(parents=True)
    manifest_path = public / "episode_manifest.json"
    world_path = public / "world.sdf"
    manifest_path.write_text(
        files["public/episode_manifest.json"], encoding="utf-8"
    )
    world_path.write_text(files["public/world.sdf"], encoding="utf-8")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["field"]["width_m"] * manifest["field"]["height_m"] == pytest.approx(
        20_000.0
    )
    if map_index == 0:
        assert manifest["field"]["width_m"] == pytest.approx(200.0)
        assert manifest["field"]["height_m"] == pytest.approx(100.0)
    else:
        assert manifest["field"]["aspect_ratio"] != pytest.approx(2.0)
    assert manifest["vehicle"] == {
        "included": False,
        "profile": None,
        "urdf_claim": False,
    }

    artifacts = materialize_campus_artifacts(
        manifest_path,
        world_path,
        MOTION_PROFILE,
        tmp_path / f"runtime-{map_index}",
        resolution=0.5,
    )
    assert artifacts.static_collision_count == manifest["counts"]["static_assets"]
    assert artifacts.static_collision_count == 120
    mission = yaml.safe_load(artifacts.mission_geometry.read_text(encoding="utf-8"))
    assert mission["truth_boundary"]["evaluator_truth_used"] is False
    assert mission["truth_boundary"]["dirt_truth_used"] is False
    assert "dirt_patches" not in mission
    assert "discrete_cubes" not in mission
    assert "pedestrians" not in mission

    world = ET.parse(world_path).getroot().find("world")
    assert world is not None
    assert world.find("model[@name='proxy_chassis_not_urdf']") is None
    public_dynamic_models = [
        model
        for model in world.findall("model")
        if (model.get("name") or "").startswith(("surface_", "object_", "walker_"))
    ]
    assert len(public_dynamic_models) == sum(
        manifest["counts"][key]
        for key in ("dirt_patches", "discrete_cubes", "pedestrians")
    )
    for model in public_dynamic_models:
        pose = _pose_values_for_test(model.findtext("pose"))
        assert _map_value(artifacts.occupancy_map, pose[0], pose[1]) == 0


def _pose_values_for_test(text: str | None) -> tuple[float, ...]:
    values = tuple(float(value) for value in (text or "").split())
    assert len(values) == 6
    return values


def test_materializer_rejects_evaluator_manifest_and_world_count_mismatch(tmp_path):
    evaluator_like = deepcopy(_manifest())
    evaluator_like["seeds"] = {"layout": 123}
    manifest_path, world_path = _write_inputs(tmp_path, evaluator_like)
    with pytest.raises(IntegrationContractError, match="evaluator-only"):
        materialize_campus_artifacts(
            manifest_path, world_path, MOTION_PROFILE, tmp_path / "rejected"
        )

    public = _manifest()
    public["counts"]["static_assets"] = 4
    manifest_path.write_text(json.dumps(public), encoding="utf-8")
    with pytest.raises(IntegrationContractError, match="count disagrees"):
        materialize_campus_artifacts(
            manifest_path, world_path, MOTION_PROFILE, tmp_path / "rejected"
        )


def test_materializer_rejects_embedded_proxy_vehicle(tmp_path):
    manifest = _manifest()
    manifest["vehicle"] = {
        "included": True,
        "profile": "proxy_chassis_not_urdf",
        "urdf_claim": False,
    }
    manifest_path, world_path = _write_inputs(tmp_path, manifest)
    with pytest.raises(IntegrationContractError, match="proxy vehicle"):
        materialize_campus_artifacts(
            manifest_path, world_path, MOTION_PROFILE, tmp_path / "rejected"
        )


def test_materializer_rejects_nonformal_profile(tmp_path):
    manifest = _manifest()
    manifest["profile"] = "research"
    manifest_path, world_path = _write_inputs(tmp_path, manifest)
    with pytest.raises(IntegrationContractError, match="profile=formal"):
        materialize_campus_artifacts(
            manifest_path, world_path, MOTION_PROFILE, tmp_path / "rejected"
        )


def test_materializer_rejects_ambiguous_or_repeated_geofence_transform(tmp_path):
    manifest = _manifest()
    source = manifest["field"]["geofence_polygon_m"]
    start = manifest["vehicle_start_pose_map"]
    cosine, sine = math.cos(start["yaw_rad"]), math.sin(start["yaw_rad"])
    localized = [
        [
            cosine * (x - start["x_m"]) + sine * (y - start["y_m"]),
            -sine * (x - start["x_m"]) + cosine * (y - start["y_m"]),
        ]
        for x, y in source
    ]
    manifest["field"].update(
        source_world_geofence={"frame_id": "source_world", "polygon_m": source},
        localization_map_geofence={"frame_id": "map", "polygon_m": localized},
        legacy_geofence={"field": "geofence_polygon_m", "frame_id": "source_world", "deprecation": "use explicit fields"},
        geofence_frame="source_world",
    )
    manifest["vehicle_start_pose_source_world"] = start
    manifest_path, world_path = _write_inputs(tmp_path, manifest)
    materialize_campus_artifacts(
        manifest_path, world_path, MOTION_PROFILE, tmp_path / "accepted"
    )

    manifest["field"]["localization_map_geofence"]["polygon_m"] = source
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(IntegrationContractError, match="exactly once"):
        materialize_campus_artifacts(
            manifest_path, world_path, MOTION_PROFILE, tmp_path / "rejected-repeat"
        )

    manifest["field"].pop("source_world_geofence")
    manifest["field"].pop("localization_map_geofence")
    manifest["field"].pop("legacy_geofence")
    manifest["field"]["geofence_frame"] = "odom"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(IntegrationContractError, match="legacy geofence frame"):
        materialize_campus_artifacts(
            manifest_path, world_path, MOTION_PROFILE, tmp_path / "rejected-frame"
        )

def test_materializer_fails_closed_when_vehicle_start_is_not_usable(tmp_path):
    manifest = _manifest()
    manifest["vehicle_start_pose_map"] = {"x_m": 1.0, "y_m": 0.0, "yaw_rad": 0.0}
    manifest_path, world_path = _write_inputs(tmp_path, manifest)
    with pytest.raises(IntegrationContractError, match="free occupancy"):
        materialize_campus_artifacts(
            manifest_path, world_path, MOTION_PROFILE, tmp_path / "rejected"
        )

    manifest["vehicle_start_pose_map"] = {"x_m": -4.8, "y_m": 0.0, "yaw_rad": 0.0}
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(IntegrationContractError, match="keepout"):
        materialize_campus_artifacts(
            manifest_path, world_path, MOTION_PROFILE, tmp_path / "rejected"
        )


def test_launch_materializes_maps_and_keeps_power_on_estop_latched():
    source = LAUNCH.read_text(encoding="utf-8")
    ast.parse(source)
    assert "materialize_campus_artifacts" in source
    assert 'DeclareLaunchArgument("start_navigation", default_value="true")' in source
    assert 'DeclareLaunchArgument("start_coverage", default_value="true")' in source
    assert 'DeclareLaunchArgument("localization_backend", default_value="amcl")' in source
    assert '"map_file": str(artifacts.occupancy_map)' in source
    assert '"keepout_map": str(artifacts.keepout_map)' in source
    assert '"speed_map": str(artifacts.speed_map)' in source
    assert '"initial_pose_x": str(artifacts.start_pose[0])' in source
    assert '"start_simulation_safety_inputs": LaunchConfiguration(' in source
    assert "publish_selected_odom" not in source
    assert '"simulation_initial_estop_active": LaunchConfiguration(' in source
    assert 'DeclareLaunchArgument("start_pedestrians", default_value="true")' in source
    assert '"operation_width": cleaning_width' in source
    assert '"start_velocity_gate": "false"' in source
    assert 'executable="formal-dynamic-footprint-manager"' in source
    assert '"motion_profile_file": LaunchConfiguration("motion_profile_file")' in source
    assert '"command_input_topic": "/cmd_vel_gate"' in source
    assert "configured_world_name != artifacts.world_name" in source
    assert source.count('default_value="true"') >= 4
    assert "main_power=true" in source
    assert "emergency_stop=false" in source


def test_resolution_contract_values_are_bound_to_executable_defaults():
    launch_source = LAUNCH.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("map_resolution", default_value="0.10")' in launch_source
    assert inspect.signature(materialize_campus_artifacts).parameters["resolution"].default == 0.10
    assert inspect.signature(prepare_public_lifecycle_artifacts).parameters["resolution"].default == 0.25
    assert ProductCoverageTelemetry.__dataclass_fields__["raster_resolution_m"].default == 0.25
