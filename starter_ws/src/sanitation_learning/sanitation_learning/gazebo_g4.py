"""G4 world generation: 12 textured worlds aligned with the production camera.

Unlike G3, every world includes real model directories produced by
`g4_assets.write_g4_assets` (procedural texture PNGs referenced through PBR
albedo maps) and a per-world procedural ground texture.  The worlds are split
8 train / 2 val / 2 test with distinct material, layout, geometry, and
lighting families.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from .g2_contract import read_production_camera_contract
from .g4_assets import (
    GEOMETRY_PARAMS,
    _class_variants,
    _texture_png,
    load_g4_asset_registry,
    write_g4_assets,
)
from .rendered import CLASS_INDEX, CLASS_ORDER


# (world_id, material_id, layout_family, geometry_family, lighting_family,
#  ground_texture_family, light_diffuse, light_direction, roughness, split)
WORLD_PROFILES = (
    (
        "world_g4_01_asphalt_campus",
        "asphalt_dark",
        "campus_open_lane",
        "ground_flat_open",
        "directional_high_noon",
        "asphalt_coarse_dark",
        "0.95 0.93 0.88 1",
        "-0.45 0.2 -0.87",
        0.92,
        "train",
    ),
    (
        "world_g4_02_concrete_sidewalk",
        "concrete_light",
        "sidewalk_corridor",
        "ground_corridor_walls",
        "directional_soft_morning",
        "concrete_fine_light",
        "0.88 0.90 0.92 1",
        "-0.30 -0.30 -0.90",
        0.84,
        "train",
    ),
    (
        "world_g4_03_wet_courtyard",
        "wet_dark_asphalt",
        "wet_courtyard",
        "ground_wet_open",
        "overcast_diffuse",
        "asphalt_wet_dark",
        "0.62 0.66 0.72 1",
        "-0.15 0.1 -0.97",
        0.28,
        "train",
    ),
    (
        "world_g4_04_cobblestone_arcade",
        "cobblestone_gray",
        "arcade_columns",
        "ground_columned",
        "backlight_evening",
        "cobblestone_coarse",
        "0.78 0.72 0.64 1",
        "0.55 0.15 -0.82",
        0.96,
        "train",
    ),
    (
        "world_g4_05_red_brick_promenade",
        "red_brick_paving",
        "promenade_benches",
        "ground_bench_flanked",
        "directional_late_afternoon",
        "brick_dense",
        "0.88 0.76 0.62 1",
        "-0.20 -0.35 -0.91",
        0.90,
        "train",
    ),
    (
        "world_g4_06_tiled_plaza",
        "tiled_plaza",
        "plaza_islands",
        "ground_island_grid",
        "soft_morning_side",
        "tile_grid",
        "0.92 0.92 0.88 1",
        "0.30 -0.25 -0.92",
        0.78,
        "train",
    ),
    (
        "world_g4_07_service_road",
        "service_road_gray",
        "service_lane",
        "ground_lane_props",
        "hard_noon_shadow",
        "service_road_marked",
        "0.90 0.88 0.84 1",
        "0.10 0.45 -0.88",
        0.88,
        "train",
    ),
    (
        "world_g4_08_mixed_curb_vegetation",
        "mixed_curb_soil",
        "curb_chicane",
        "ground_curb_slalom",
        "dappled_tree_shadow",
        "soil_mixed_vegetation",
        "0.72 0.76 0.66 1",
        "-0.35 0.35 -0.86",
        0.82,
        "train",
    ),
    (
        "world_g4_09_light_paver_pedestrian",
        "light_paver",
        "pedestrian_crosswalk",
        "ground_crosswalk",
        "bright_overcast",
        "paver_light",
        "0.94 0.95 0.92 1",
        "-0.10 -0.10 -0.98",
        0.86,
        "val",
    ),
    (
        "world_g4_10_dark_gravel_parking",
        "dark_gravel",
        "parking_lot_stripes",
        "ground_parking_striped",
        "low_sun_glare",
        "gravel_dark",
        "0.86 0.78 0.68 1",
        "0.65 0.05 -0.75",
        0.94,
        "val",
    ),
    (
        "world_g4_11_brick_market_street",
        "brick_market_worn",
        "market_stalls",
        "ground_stall_alleys",
        "mixed_fluorescent_shade",
        "brick_market_worn",
        "0.82 0.80 0.78 1",
        "-0.25 0.45 -0.85",
        0.89,
        "test",
    ),
    (
        "world_g4_12_smooth_industrial_floor",
        "smooth_industrial",
        "industrial_bays",
        "ground_bay_open",
        "high_bay_diffuse",
        "industrial_smooth",
        "0.80 0.82 0.84 1",
        "0.20 -0.30 -0.93",
        0.74,
        "test",
    ),
)

GROUND_TEXTURE_SPECS = {
    "asphalt_coarse_dark": ("speckle", ["asphalt", "dark_gray", "gray"]),
    "concrete_fine_light": ("clean_stone", ["concrete", "white", "gray"]),
    "asphalt_wet_dark": ("mottled", ["asphalt", "water_dark", "dark_gray"]),
    "cobblestone_coarse": ("paver_grid", ["paver", "stone", "dark_gray"]),
    "brick_dense": ("patchwork", ["red", "rust", "brown"]),
    "tile_grid": ("grid_lines", ["concrete", "paver", "white"]),
    "service_road_marked": ("dashed_line", ["gray", "white", "asphalt"]),
    "soil_mixed_vegetation": ("leaf_mosaic", ["leaf_brown", "leaf_green", "brown"]),
    "paver_light": ("paver_grid", ["white", "paver", "concrete"]),
    "gravel_dark": ("mottled", ["dark_gray", "gray", "asphalt"]),
    "brick_market_worn": ("patchwork", ["brown", "rust", "gray"]),
    "industrial_smooth": ("clean_stone", ["gray", "dark_gray", "concrete"]),
}

PRODUCTION_SENSOR_TOPICS = [
    "/camera/color/image_raw",
    "/camera/depth/image_rect_raw",
    "/camera/color/camera_info",
    "/ground_truth/semantic/image",
    "/ground_truth/instance/image",
]

GAZEBO_SENSOR_TOPICS = [
    "/camera/image",
    "/camera/depth_image",
    "/g2/semantic_gt/labels_map",
    "/g2/instance_gt/labels_map",
]


def _layout_models(layout_family: str) -> str:
    layouts = {
        "campus_open_lane": [("planter", "box", "1.8 1.8 0.45", "5 3 0.225")],
        "sidewalk_corridor": [
            ("wall_left", "box", "14 0.25 0.7", "2 3 0.35"),
            ("wall_right", "box", "14 0.25 0.7", "2 -3 0.35"),
        ],
        "wet_courtyard": [
            ("shelter", "box", "2.5 1.2 0.35", "4 2 0.175"),
            ("drain", "box", "0.35 7 0.08", "1 -1 0.04"),
        ],
        "arcade_columns": [
            ("column_a", "cylinder", "0.30 2.2", "1.5 2.4 1.1"),
            ("column_b", "cylinder", "0.30 2.2", "5.5 -2.4 1.1"),
        ],
        "promenade_benches": [
            ("bench_a", "box", "1.8 0.55 0.55", "2.5 2.5 0.275"),
            ("bench_b", "box", "1.8 0.55 0.55", "6 -2.5 0.275"),
        ],
        "plaza_islands": [
            ("island_a", "cylinder", "1.1 0.35", "3 2 0.175"),
            ("island_b", "cylinder", "0.8 0.35", "6 -2 0.175"),
        ],
        "service_lane": [
            ("loading_bay", "box", "3.0 1.4 0.25", "5 2.4 0.125"),
            ("bollard", "cylinder", "0.18 1.0", "2 -2 0.5"),
        ],
        "curb_chicane": [
            ("curb_a", "box", "5 0.22 0.28", "1 2 0.14"),
            ("curb_b", "box", "5 0.22 0.28", "5 -2 0.14"),
        ],
        "pedestrian_crosswalk": [
            ("crosswalk_a", "box", "0.6 7 0.02", "3 1 0.01"),
            ("crosswalk_b", "box", "0.6 7 0.02", "3 -1 0.01"),
        ],
        "parking_lot_stripes": [
            ("stripe_a", "box", "4 0.18 0.015", "1 2.5 0.008"),
            ("stripe_b", "box", "4 0.18 0.015", "5 -2.5 0.008"),
        ],
        "market_stalls": [
            ("stall_a", "box", "2.2 1.0 0.30", "2 2.6 0.15"),
            ("stall_b", "box", "2.2 1.0 0.30", "6 -2.6 0.15"),
        ],
        "industrial_bays": [
            ("bay_rack_a", "box", "2.8 0.9 0.45", "1.5 2.2 0.225"),
            ("bay_rack_b", "box", "2.8 0.9 0.45", "5 -2.2 0.225"),
        ],
    }
    chunks = []
    for name, kind, size, pose in layouts[layout_family]:
        if kind == "box":
            geometry = f"<box><size>{size}</size></box>"
        else:
            radius, length = size.split()
            geometry = (
                f"<cylinder><radius>{radius}</radius><length>{length}</length></cylinder>"
            )
        chunks.append(
            f'<model name="{name}"><static>true</static><pose>{pose} 0 0 0</pose>'
            f'<link name="body"><collision name="collision"><geometry>{geometry}</geometry></collision>'
            f'<visual name="visual"><geometry>{geometry}</geometry>'
            '<material><pbr><metal>'
            '<albedo_map>ground_textures/prop_gray.png</albedo_map>'
            "<roughness>0.85</roughness><metalness>0</metalness>"
            "</metal></pbr></material></visual>"
            "</link></model>"
        )
    return "".join(chunks)


def _ground_texture_png(
    texture_family: str,
    palette_names: list[str],
    palette_rgb: dict,
    seed: int,
    size: tuple[int, int] = (512, 512),
) -> np.ndarray:
    return _texture_png(
        texture_family, palette_rgb, palette_names, seed, size=size
    )


GROUND_COLORS = {
    "asphalt_dark": "0.20 0.21 0.22 1",
    "concrete_light": "0.72 0.72 0.70 1",
    "wet_dark_asphalt": "0.12 0.14 0.16 1",
    "cobblestone_gray": "0.48 0.47 0.45 1",
    "red_brick_paving": "0.62 0.34 0.25 1",
    "tiled_plaza": "0.70 0.66 0.58 1",
    "service_road_gray": "0.38 0.39 0.40 1",
    "mixed_curb_soil": "0.42 0.38 0.28 1",
    "light_paver": "0.76 0.74 0.70 1",
    "dark_gravel": "0.26 0.24 0.22 1",
    "brick_market_worn": "0.52 0.32 0.24 1",
    "smooth_industrial": "0.60 0.62 0.64 1",
}


def write_g4_worlds(
    registry_path: str | Path,
    assets_dir: str | Path,
    xacro_path: str | Path,
    output_dir: str | Path,
) -> dict:
    """Generate G4 worlds and the g4_world_manifest.json contract."""
    registry_path, assets_dir, output_dir = (
        Path(registry_path),
        Path(assets_dir),
        Path(output_dir),
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    registry = load_g4_asset_registry(registry_path)
    contract = read_production_camera_contract(xacro_path)
    registry_sha = hashlib.sha256(registry_path.read_bytes()).hexdigest()
    asset_manifest_path = assets_dir / "g4_generated_asset_manifest.json"
    if (
        not asset_manifest_path.is_file()
        or json.loads(asset_manifest_path.read_text(encoding="utf-8")).get(
            "registry_sha256"
        )
        != registry_sha
    ):
        write_g4_assets(registry_path, assets_dir)
    (output_dir / "ground_textures").mkdir(parents=True, exist_ok=True)
    models: list[dict] = []
    for class_id in CLASS_ORDER[1:]:
        for variant_index, variant in enumerate(_class_variants(registry, class_id)):
            kind, values = GEOMETRY_PARAMS[variant["geometry_family"]]
            models.append(
                {
                    "model_name": variant["id"],
                    "class_id": class_id,
                    "semantic_label": CLASS_INDEX[class_id],
                    "variant_index": variant_index,
                    "geometry_family": variant["geometry_family"],
                    "material_family": variant["material_family"],
                    "texture_family": variant["texture_family"],
                    "physical_geometry_values_m": list(values),
                    "geometry_kind": kind,
                    "split_eligibility": variant["split_eligibility"],
                    "scale_factor": 1.0,
                    "license": variant["license"],
                    "source": variant["source"],
                    "registry_sha256": variant["sha256"],
                }
            )
    negative_models = []
    for negative in registry["hard_negatives"]:
        kind, values = GEOMETRY_PARAMS[negative["geometry_family"]]
        negative_models.append(
            {
                "model_name": negative["id"],
                "negative_id": negative["id"],
                "taxonomy": negative["taxonomy"],
                "semantic_label": 0,
                "geometry_family": negative["geometry_family"],
                "material_family": negative["material_family"],
                "texture_family": negative["texture_family"],
                "physical_geometry_values_m": list(values),
                "geometry_kind": kind,
                "split_eligibility": negative["split_eligibility"],
                "license": negative["license"],
                "source": negative["source"],
                "registry_sha256": negative["sha256"],
            }
        )
    worlds = []
    for index, profile in enumerate(WORLD_PROFILES):
        (
            world_id,
            material_id,
            layout_family,
            geometry_family,
            lighting_family,
            ground_texture_family,
            light_diffuse,
            light_direction,
            roughness,
            split,
        ) = profile
        texture_family, palette_names = GROUND_TEXTURE_SPECS[ground_texture_family]
        ground_png = _ground_texture_png(
            texture_family,
            palette_names,
            registry["palette_rgb"],
            int(hashlib.sha256(world_id.encode("utf-8")).hexdigest()[:12], 16),
        )
        ground_path = output_dir / "ground_textures" / f"{world_id}.png"
        cv2.imwrite(str(ground_path), cv2.cvtColor(ground_png, cv2.COLOR_RGB2BGR))
        ground_rgba = GROUND_COLORS[material_id]
        prop_tex_path = output_dir / "ground_textures" / "prop_gray.png"
        if not prop_tex_path.is_file():
            prop_texture = _ground_texture_png(
                "clean_stone",
                ["gray", "dark_gray", "stone"],
                registry["palette_rgb"],
                991,
            )
            cv2.imwrite(
                str(prop_tex_path), cv2.cvtColor(prop_texture, cv2.COLOR_RGB2BGR)
            )
        included_models = models + negative_models
        includes = "".join(
            f'<include><uri>model://{item["model_name"]}</uri>'
            f'<pose>{-200 - index * 0.3:.3f} 200 -5 0 0 0</pose></include>'
            for index, item in enumerate(included_models)
        )
        world_text = f"""<?xml version="1.0"?>
<sdf version="1.9"><world name="{world_id}">
  <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <light type="directional" name="sun"><pose>0 0 10 0 0 0</pose><cast_shadows>true</cast_shadows><diffuse>{light_diffuse}</diffuse><direction>{light_direction}</direction></light>
  <model name="ground"><static>true</static><link name="ground">
    <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry></collision>
    <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>
      <material><ambient>{ground_rgba}</ambient><diffuse>{ground_rgba}</diffuse><pbr><metal><albedo_map>ground_textures/{world_id}.png</albedo_map>
      <roughness>{roughness}</roughness><metalness>0</metalness></metal></pbr></material></visual>
  </link></model>
  {_layout_models(layout_family)}
  {includes}
</world></sdf>
"""
        world_text = "\n".join(line.rstrip() for line in world_text.splitlines()) + "\n"
        path = output_dir / f"{world_id}.sdf"
        path.write_text(world_text, encoding="utf-8")
        worlds.append(
            {
                "world_id": world_id,
                "material_id": material_id,
                "layout_family": layout_family,
                "geometry_family": geometry_family,
                "background_family": material_id,
                "lighting_family": lighting_family,
                "ground_texture_family": ground_texture_family,
                "ground_texture_path": f"ground_textures/{world_id}.png",
                "path": path.name,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "split_eligibility": [split],
                "allowed_trajectory_family": f"{layout_family}_vehicle_motion",
                "vehicle_spawned_by": "AUTO-05R G4 capture runner",
                "sensor_topics": list(PRODUCTION_SENSOR_TOPICS),
                "gazebo_topics": list(GAZEBO_SENSOR_TOPICS),
                "textured_asset_models": True,
            }
        )
    manifest = {
        "schema_version": 2,
        "dataset_domain": "G4_deployment_aligned_vehicle_camera_gazebo",
        "camera_contract": contract,
        "native_capture_resolution": contract["native_resolution"],
        "training_only_ground_truth": True,
        "production_launch_modified": False,
        "actual_vehicle_model_required": (
            "sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro"
        ),
        "static_independent_camera_rig_forbidden": True,
        "world_split_counts": {"train": 8, "val": 2, "test": 2},
        "scenes_per_world": 25,
        "frames_per_scene": 10,
        "vehicle_motion_required": True,
        "distance_envelope_m": [0.5, 8.0],
        "worlds": worlds,
        "assets": models,
        "negative_assets": negative_models,
        "sensor_topics": list(PRODUCTION_SENSOR_TOPICS),
        "gazebo_topics": list(GAZEBO_SENSOR_TOPICS),
        "asset_policy": (
            "self-authored procedural G4 models with deterministic texture PNGs, "
            "true physical geometry, split-isolated variants"
        ),
        "registry_sha256": registry_sha,
        "test_used_for_model_selection": False,
        "G4_dataset_gate_pass": False,
    }
    manifest_path = output_dir / "g4_world_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", required=True)
    parser.add_argument("--assets-dir", required=True)
    parser.add_argument("--xacro", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    print(
        json.dumps(
            write_g4_worlds(
                args.registry, args.assets_dir, args.xacro, args.output_dir
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
