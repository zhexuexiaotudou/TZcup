import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from sanitation_campus_scenario.cli import main as cli_main
from sanitation_campus_scenario.generator import (
    EVALUATOR_NAMESPACE,
    GenerationError,
    generate_episode,
    load_config,
    split_index,
)
from sanitation_campus_scenario.io import write_episode
from sanitation_campus_scenario.motion import interpolate_loop, load_schedule


CONFIG = Path(__file__).parents[1] / "config" / "default_scenario.yaml"


def test_default_config_and_frozen_split_are_deterministic():
    config = load_config(CONFIG)
    first = split_index(config)
    second = split_index(config)
    assert first == second
    assert first["counts"] == {
        "train": {"map_count": 32, "missions_per_map": 200, "mission_count": 6400},
        "val": {"map_count": 8, "missions_per_map": 100, "mission_count": 800},
        "hidden": {"map_count": 12, "missions_per_map": 100, "mission_count": 1200},
    }
    assert first["total_map_count"] == 52
    assert first["total_mission_count"] == 8400
    assert len(first["maps"]) == 52
    assert len({row["layout_seed"] for row in first["maps"]}) == 52
    missions = [mission for row in first["maps"] for mission in row["missions"]]
    assert len(missions) == 8400
    seed_tuples = {tuple(row["seeds"].values()) for row in missions}
    assert len(seed_tuples) == 8400
    assert all(len(set(row["seeds"].values())) == 4 for row in missions)


@pytest.mark.parametrize("profile,dimensions", [("research", (106.0, 53.0)), ("formal", (200.0, 100.0))])
def test_episode_contract_and_truth_separation(profile, dimensions):
    config = load_config(CONFIG)
    files = generate_episode(config, profile, "train", 0, 0)
    assert set(files) == {
        "public/world.sdf",
        "public/episode_manifest.json",
        "evaluator/episode_manifest.json",
        "environment/pedestrian_schedule.json",
        "evaluator/ground_truth.json",
    }
    root = ET.fromstring(files["public/world.sdf"])
    assert root.tag == "sdf"
    assert not root.findall(".//model[@name='boundary_wall']")
    manifest = json.loads(files["public/episode_manifest.json"])
    assert (manifest["field"]["width_m"], manifest["field"]["height_m"]) == dimensions
    assert manifest["field"]["physical_boundary_walls"] is False
    assert manifest["counts"]["discrete_cubes"] == 20
    assert manifest["cube_contract"]["edge_m"] == 0.03
    assert "discrete_cubes" not in manifest
    assert manifest["map_id"] == "train-map-000"
    assert manifest["episode_id"] == "train-map-000-mission-000"
    truth = json.loads(files["evaluator/ground_truth.json"])
    assert truth["control_use_prohibited"] is True
    assert truth["namespace"] == EVALUATOR_NAMESPACE
    assert len(truth["discrete_cubes"]) == 20
    assert all(cube["edge_m"] == 0.03 for cube in truth["discrete_cubes"])
    evaluator = json.loads(files["evaluator/episode_manifest.json"])
    assert evaluator["truth_boundary"]["evaluator_namespace"] == EVALUATOR_NAMESPACE


def test_cube_clearance_from_assets_and_other_cubes():
    config = load_config(CONFIG)
    files = generate_episode(config, "research", "val", 3, 7)
    truth = json.loads(files["evaluator/ground_truth.json"])
    clearance = config["episode"]["grasp_clearance_m"]
    cubes = truth["discrete_cubes"]
    for cube in cubes:
        x, y = cube["pose"]["x_m"], cube["pose"]["y_m"]
        for asset in truth["static_assets"]:
            ax, ay = asset["pose"]["x_m"], asset["pose"]["y_m"]
            radius = (asset["size_m"][0] ** 2 + asset["size_m"][1] ** 2) ** 0.5 / 2.0
            assert ((x - ax) ** 2 + (y - ay) ** 2) ** 0.5 >= clearance + radius + 0.015 - 1e-6
    points = [(c["pose"]["x_m"], c["pose"]["y_m"]) for c in cubes]
    assert len(points) == len(set(points))


def test_same_map_reuses_layout_while_mission_randomization_changes():
    config = load_config(CONFIG)
    first = generate_episode(config, "research", "train", 0, 0)
    again = generate_episode(config, "research", "train", 0, 0)
    other_mission = generate_episode(config, "research", "train", 0, 1)
    other_map = generate_episode(config, "research", "train", 1, 0)
    assert first == again
    first_truth = json.loads(first["evaluator/ground_truth.json"])
    mission_truth = json.loads(other_mission["evaluator/ground_truth.json"])
    map_truth = json.loads(other_map["evaluator/ground_truth.json"])
    assert first_truth["static_assets"] == mission_truth["static_assets"]
    assert first_truth["dirt_patches"] != mission_truth["dirt_patches"]
    assert [tuple(item["size_m"]) for item in first_truth["dirt_patches"]] != [
        tuple(item["size_m"]) for item in mission_truth["dirt_patches"]
    ]
    assert first_truth["discrete_cubes"] != mission_truth["discrete_cubes"]
    assert first_truth["pedestrians"] != mission_truth["pedestrians"]
    assert first_truth["static_assets"] != map_truth["static_assets"]
    first_public = json.loads(first["public/episode_manifest.json"])
    mission_public = json.loads(other_mission["public/episode_manifest.json"])
    map_public = json.loads(other_map["public/episode_manifest.json"])
    assert first_public["field"] == mission_public["field"]
    assert first_public["vehicle_start_pose_map"] == mission_public["vehicle_start_pose_map"]
    assert first_public["field"] != map_public["field"]


def test_fail_closed_validation_rejects_unsafe_configuration():
    config = load_config(CONFIG)
    too_many = copy.deepcopy(config)
    too_many["episode"]["cube_count"] = 21
    with pytest.raises(GenerationError, match="cannot exceed 20"):
        generate_episode(too_many, "research", "train", 0, 0)
    wrong_truth = copy.deepcopy(config)
    wrong_truth["truth"]["evaluator_namespace"] = "/control/truth"
    with pytest.raises(GenerationError, match="evaluator_namespace"):
        generate_episode(wrong_truth, "research", "train", 0, 0)
    with pytest.raises(GenerationError, match="map_index out of range"):
        generate_episode(config, "research", "hidden", 12, 0)
    with pytest.raises(GenerationError, match="mission_index out of range"):
        generate_episode(config, "research", "hidden", 0, 100)
    unknown_key = copy.deepcopy(config)
    unknown_key["episode"]["unreviewed_randomization"] = True
    with pytest.raises(GenerationError, match="unexpected"):
        generate_episode(unknown_key, "research", "train", 0, 0)


def test_atomic_write_refuses_existing_output(tmp_path):
    files = generate_episode(load_config(CONFIG), "research", "train", 2, 5)
    output = write_episode(tmp_path / "episode", files)
    assert (output / "public" / "world.sdf").is_file()
    with pytest.raises(GenerationError, match="already exists"):
        write_episode(output, files)


def test_proxy_is_explicitly_not_urdf():
    files = generate_episode(load_config(CONFIG), "research", "train", 0, 0, include_proxy=True)
    assert "proxy_chassis_not_urdf" in files["public/world.sdf"]
    manifest = json.loads(files["public/episode_manifest.json"])
    assert manifest["vehicle"] == {"included": True, "profile": "proxy_chassis_not_urdf", "urdf_claim": False}


def test_pedestrian_interpolation_loops_and_rejects_bad_schedules():
    waypoints = ((0.0, 0.0, 0.0), (2.0, 2.0, 0.0), (4.0, 0.0, 0.0))
    assert interpolate_loop(waypoints, 1.0) == pytest.approx((1.0, 0.0, 0.0))
    assert interpolate_loop(waypoints, 5.0) == pytest.approx((1.0, 0.0, 0.0))
    with pytest.raises(GenerationError):
        interpolate_loop(((1.0, 0.0, 0.0), (2.0, 1.0, 0.0)), 0.0)


def test_generated_pedestrian_schedule_is_environment_only(tmp_path):
    files = generate_episode(load_config(CONFIG), "research", "hidden", 0, 0)
    path = tmp_path / "schedule.json"
    path.write_text(files["environment/pedestrian_schedule.json"], encoding="utf-8")
    schedule = load_schedule(path)
    assert schedule["access"] == "environment_driver_only_not_robot_control"
    assert len(schedule["pedestrians"]) == 8


def test_every_frozen_seed_generates_for_both_profiles():
    config = load_config(CONFIG)
    aspects = {"research": set(), "formal": set()}
    expected_areas = {"research": 5618.0, "formal": 20000.0}
    for split, map_count, missions_per_map in (
        ("train", 32, 200),
        ("val", 8, 100),
        ("hidden", 12, 100),
    ):
        for map_index in range(map_count):
            for profile in ("research", "formal"):
                files = generate_episode(config, profile, split, map_index, 0)
                assert files["public/world.sdf"].startswith("<?xml")
                manifest = json.loads(files["public/episode_manifest.json"])
                field = manifest["field"]
                assert field["area_m2"] == expected_areas[profile]
                assert field["width_m"] * field["height_m"] == pytest.approx(
                    expected_areas[profile], rel=1e-12
                )
                aspects[profile].add(round(field["aspect_ratio"], 9))
                if map_index == 0:
                    expected_dimensions = (
                        (106.0, 53.0) if profile == "research" else (200.0, 100.0)
                    )
                    assert (field["width_m"], field["height_m"]) == expected_dimensions
        # Exercise the upper mission boundary without materializing all 8,400 worlds.
        generate_episode(config, "research", split, 0, missions_per_map - 1)
    assert all(len(values) >= 4 for values in aspects.values())


def test_cli_generates_selected_map_mission_and_index_only(tmp_path):
    episode_output = tmp_path / "episode"
    assert cli_main(
        [
            "generate",
            "--config",
            str(CONFIG),
            "--profile",
            "research",
            "--split",
            "train",
            "--map-index",
            "4",
            "--mission-index",
            "19",
            "--output",
            str(episode_output),
        ]
    ) == 0
    manifest = json.loads(
        (episode_output / "public" / "episode_manifest.json").read_text(
            encoding="utf-8"
        )
    )
    assert (manifest["map_index"], manifest["mission_index"]) == (4, 19)

    index_output = tmp_path / "split-index.json"
    assert cli_main(
        ["split-index", "--config", str(CONFIG), "--output", str(index_output)]
    ) == 0
    assert json.loads(index_output.read_text(encoding="utf-8"))["total_mission_count"] == 8400


def test_hidden_public_bundle_has_no_seed_truth_or_schedule_disclosure():
    files = generate_episode(load_config(CONFIG), "research", "hidden", 4, 17)
    assert set(path.split("/", 1)[0] for path in files) == {
        "public",
        "evaluator",
        "environment",
    }
    public_manifest = json.loads(files["public/episode_manifest.json"])
    serialized = json.dumps(public_manifest, sort_keys=True).lower()
    for prohibited in ("seed", "truth", "ground_truth", "schedule", "evaluator"):
        assert prohibited not in serialized
    assert "public/world.sdf" in files
    assert all(
        not path.startswith("public/") or path in {
            "public/world.sdf",
            "public/episode_manifest.json",
        }
        for path in files
    )


def test_dirt_area_is_fixed_and_rotated_rectangles_are_mutually_exclusive():
    files = generate_episode(load_config(CONFIG), "formal", "train", 2, 37)
    truth = json.loads(files["evaluator/ground_truth.json"])
    dirt = truth["dirt_patches"]
    assert {item["area_m2"] for item in dirt} == {1.0}
    assert all(item["size_m"][0] * item["size_m"][1] == 1.0 for item in dirt)
    assert truth["dirt_union_area_m2"] == len(dirt) * 1.0
    spacing = load_config(CONFIG)["episode"]["dirt_spacing_m"]
    for index, left in enumerate(dirt):
        lx, ly = left["pose"]["x_m"], left["pose"]["y_m"]
        lr = (left["size_m"][0] ** 2 + left["size_m"][1] ** 2) ** 0.5 / 2.0
        for right in dirt[index + 1 :]:
            rx, ry = right["pose"]["x_m"], right["pose"]["y_m"]
            rr = (right["size_m"][0] ** 2 + right["size_m"][1] ** 2) ** 0.5 / 2.0
            assert ((lx - rx) ** 2 + (ly - ry) ** 2) ** 0.5 >= lr + rr + spacing - 1e-6


def test_nonbaseline_world_uses_derived_dimensions_for_ground_geofence_and_start():
    files = generate_episode(load_config(CONFIG), "formal", "train", 1, 9, include_proxy=True)
    manifest = json.loads(files["public/episode_manifest.json"])
    field = manifest["field"]
    assert (field["width_m"], field["height_m"]) != (200.0, 100.0)
    assert field["width_m"] * field["height_m"] == pytest.approx(20000.0)
    assert field["geofence_polygon_m"] == [
        [-field["width_m"] / 2.0, -field["height_m"] / 2.0],
        [field["width_m"] / 2.0, -field["height_m"] / 2.0],
        [field["width_m"] / 2.0, field["height_m"] / 2.0],
        [-field["width_m"] / 2.0, field["height_m"] / 2.0],
    ]
    start = manifest["vehicle_start_pose_map"]
    assert start == {"x_m": -field["width_m"] / 2.0 + 2.0, "y_m": 0.0, "yaw_rad": 0.0}
    root = ET.fromstring(files["public/world.sdf"])
    plane_size = root.findtext(".//model[@name='ground_plane']/link/visual/geometry/plane/size")
    assert tuple(float(value) for value in plane_size.split()) == pytest.approx(
        (field["width_m"] + 10.0, field["height_m"] + 10.0), abs=1e-5
    )
    proxy_pose = root.findtext(".//model[@name='proxy_chassis_not_urdf']/pose")
    assert float(proxy_pose.split()[0]) == pytest.approx(start["x_m"], abs=1e-5)
