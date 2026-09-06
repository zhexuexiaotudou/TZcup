from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from prepare_formal_dynamic_obstacle_schedule import materialize_schedule
from generate_formal_dynamic_runtime_build_manifest import (
    REQUIRED_PLUGIN_LIBRARIES,
    SOURCE_INSTALL_BINDINGS,
    SOURCE_ONLY_RUNTIME_FILES,
    generate_manifest,
)


def test_dynamic_runtime_manifest_binds_gripper_physics_description() -> None:
    assert (
        "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/"
        "manipulator_stack.xacro"
    ) in SOURCE_INSTALL_BINDINGS


def _write_inputs(root: Path) -> tuple[Path, Path, Path]:
    manifest = root / "episode_manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "episode_id": "episode",
                "map_id": "map",
                "vehicle_start_pose_map": {
                    "x_m": -98.0,
                    "y_m": 0.0,
                    "yaw_rad": 0.0,
                },
                "field": {
                    # Historical bundles called these source-world values
                    # `map`; keep the fixture's legacy semantics explicit.
                    "geofence_frame": "map",
                    "geofence_polygon_m": [
                        [-100.0, -50.0],
                        [100.0, -50.0],
                        [100.0, 50.0],
                        [-100.0, 50.0],
                    ]
                },
                "counts": {"pedestrians": 8},
            }
        ),
        encoding="utf-8",
    )
    world = root / "world.sdf"
    world.write_text(
        """<?xml version='1.0'?>
<sdf version='1.10'><world name='campus_formal'>
  <model name='asset_far'><pose>-80 20 1 0 0 0</pose><static>true</static>
    <link name='link'><collision name='collision'><geometry><box>
      <size>2 2 2</size></box></geometry></collision></link>
  </model>
  %s
</world></sdf>
""" % "\n  ".join(
            f"<model name='walker_{index}'><static>true</static></model>"
            for index in range(8)
        ),
        encoding="utf-8",
    )
    schedule = root / "schedule.json"
    schedule.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "access": "environment_driver_only_not_robot_control",
                "world_name": "campus_formal",
                "loop": True,
                "pedestrians": [
                    {
                        "object_id": f"walker_{index}",
                        "radius_m": 0.25,
                        "height_m": 1.7,
                        "speed_mps": 0.7,
                        "waypoints": [
                            [0.0, 20.0 + index, 20.0],
                            [10.0, 20.0 + index, 30.0],
                            [20.0, 20.0 + index, 20.0],
                        ],
                    }
                    for index in range(8)
                ],
            }
        ),
        encoding="utf-8",
    )
    return manifest, world, schedule


def test_schedule_is_seeded_deterministic_and_has_three_real_crossings(
    tmp_path: Path,
) -> None:
    manifest, world, base = _write_inputs(tmp_path)
    first = materialize_schedule(
        episode_manifest=manifest,
        public_world=world,
        base_schedule=base,
        seed=81422,
        nominal_leg_m=30.0,
    )
    second = materialize_schedule(
        episode_manifest=manifest,
        public_world=world,
        base_schedule=base,
        seed=81422,
        nominal_leg_m=30.0,
    )
    assert first == second
    contract = first["acceptance_environment"]
    assert contract["seed"] == 81422
    assert contract["mission_corridor_crossing_count"] == 3
    assert contract["pedestrian_model_ids"] == [
        f"walker_{index}" for index in range(8)
    ]
    for pedestrian in first["pedestrians"][:3]:
        first_point, second_point, _ = pedestrian["waypoints"]
        # The fixed public mission leg lies on y=0, so each route genuinely
        # crosses it rather than merely approaching it.
        assert first_point[2] * second_point[2] < 0.0
        assert -92.0 <= first_point[1] <= -74.0
        speed = math.dist(first_point[1:], second_point[1:]) / second_point[0]
        assert 0.45 <= speed <= 0.80


def test_different_seed_changes_environment_routes(tmp_path: Path) -> None:
    manifest, world, base = _write_inputs(tmp_path)
    first = materialize_schedule(
        episode_manifest=manifest,
        public_world=world,
        base_schedule=base,
        seed=1,
        nominal_leg_m=30.0,
    )
    second = materialize_schedule(
        episode_manifest=manifest,
        public_world=world,
        base_schedule=base,
        seed=2,
        nominal_leg_m=30.0,
    )
    assert first["pedestrians"][:3] != second["pedestrians"][:3]


def test_schedule_must_name_the_exact_existing_world_walkers(tmp_path: Path) -> None:
    manifest, world, base = _write_inputs(tmp_path)
    value = json.loads(base.read_text(encoding="utf-8"))
    value["pedestrians"][0]["object_id"] = "walker_missing"
    base.write_text(json.dumps(value), encoding="utf-8")
    with pytest.raises(ValueError, match="exactly match"):
        materialize_schedule(
            episode_manifest=manifest,
            public_world=world,
            base_schedule=base,
            seed=1,
            nominal_leg_m=30.0,
        )


def test_runtime_build_manifest_binds_install_to_source_bytes(tmp_path: Path) -> None:
    repository = tmp_path / "repository"
    install = tmp_path / "install"
    for index, (source_relative, installed_pattern) in enumerate(
        SOURCE_INSTALL_BINDINGS.items()
    ):
        content = f"binding-{index}\n"
        source = repository / source_relative
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text(content, encoding="utf-8")
        installed_relative = installed_pattern.replace("python*", "python3.12")
        installed = install / installed_relative
        installed.parent.mkdir(parents=True, exist_ok=True)
        installed.write_text(content, encoding="utf-8")
    for library in REQUIRED_PLUGIN_LIBRARIES:
        path = install / "lib" / library
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"plugin")
    for relative in SOURCE_ONLY_RUNTIME_FILES:
        path = repository / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("runtime-source\n", encoding="utf-8")
    report = generate_manifest(repository, install)
    assert report["current_source_build_completed"] is True
    assert len(report["source_install_bindings"]) == 25
    assert len(report["source_only_runtime_files"]) == len(SOURCE_ONLY_RUNTIME_FILES)
    assert {
        item["source"] for item in report["source_only_runtime_files"]
    } >= {
        "starter_ws/src/sanitation_gazebo_control/src/A300DrivetrainPlantCore.cc",
        "starter_ws/src/sanitation_gazebo_control/src/A300DrivetrainPlantSystem.cc",
        "starter_ws/src/sanitation_gazebo_control/src/A300DrivetrainCommandAdapter.cc",
        "starter_ws/src/sanitation_gazebo_control/src/A300DrivetrainNativeBridge.cc",
        "starter_ws/src/sanitation_gazebo_control/src/WaterRecoverySystem.cc",
        "starter_ws/src/sanitation_safety/setup.py",
        "starter_ws/src/sanitation_power_system/setup.py",
    }

    first_binding = report["source_install_bindings"][0]
    (install / first_binding["installed_relative"]).write_text(
        "stale install\n", encoding="utf-8"
    )
    stale = generate_manifest(repository, install)
    assert stale["current_source_build_completed"] is False
