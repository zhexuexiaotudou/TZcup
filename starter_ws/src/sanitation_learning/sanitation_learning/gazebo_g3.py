from __future__ import annotations

import hashlib
import json
from pathlib import Path

from .assets import _geometry_for, _geometry_xml, load_asset_registry
from .g2_contract import read_production_camera_contract
from .gazebo_g2 import _asset_model, _material
from .rendered import CLASS_INDEX, CLASS_ORDER


WORLD_PROFILES = (
    ("world_a_asphalt_campus", "asphalt_coarse", "campus_open", "0.24 0.25 0.27 1", 0.92, "train", "0.82 0.80 0.76 1", "-0.45 0.2 -0.87", "hard"),
    ("world_b_concrete_sidewalk", "concrete_light", "sidewalk_corridor", "0.58 0.57 0.54 1", 0.84, "train", "0.90 0.88 0.82 1", "-0.25 -0.35 -0.90", "soft"),
    ("world_c_wet_dark_ground", "wet_dark", "wet_courtyard", "0.10 0.12 0.13 1", 0.28, "train", "0.42 0.48 0.56 1", "0.35 0.15 -0.92", "low_key"),
    ("world_g_cobblestone_arcade", "cobblestone_gray", "arcade_columns", "0.36 0.35 0.34 1", 0.96, "train", "0.68 0.66 0.62 1", "0.72 0.05 -0.69", "backlight"),
    ("world_d_mixed_curb_vegetation", "mixed_curb_vegetation", "curb_chicane", "0.31 0.35 0.27 1", 0.76, "val", "0.78 0.82 0.72 1", "-0.55 0.25 -0.80", "mixed"),
    ("world_h_red_brick_promenade", "red_brick", "promenade_benches", "0.46 0.22 0.16 1", 0.88, "val", "0.84 0.62 0.48 1", "0.65 -0.15 -0.74", "backlight"),
    ("world_e_tiled_plaza", "tiled_plaza", "plaza_islands", "0.64 0.61 0.55 1", 0.74, "test", "0.86 0.84 0.78 1", "-0.20 0.55 -0.81", "soft"),
    ("world_f_service_road", "service_road", "service_lane", "0.29 0.30 0.31 1", 0.88, "test", "0.72 0.74 0.76 1", "0.48 -0.48 -0.73", "hard"),
)


def _layout_models(layout_family: str) -> str:
    layouts = {
        "campus_open": [("planter", "box", "1.8 1.8 0.45", "5 3 0.225")],
        "sidewalk_corridor": [("wall_left", "box", "14 0.25 0.7", "2 3 0.35"), ("wall_right", "box", "14 0.25 0.7", "2 -3 0.35")],
        "wet_courtyard": [("shelter", "box", "2.5 1.2 0.35", "4 2 0.175"), ("drain", "box", "0.35 7 0.08", "1 -1 0.04")],
        "arcade_columns": [("column_a", "cylinder", "0.30 2.2", "1.5 2.4 1.1"), ("column_b", "cylinder", "0.30 2.2", "5.5 -2.4 1.1")],
        "curb_chicane": [("curb_a", "box", "5 0.22 0.28", "1 2 0.14"), ("curb_b", "box", "5 0.22 0.28", "5 -2 0.14")],
        "promenade_benches": [("bench_a", "box", "1.8 0.55 0.55", "2.5 2.5 0.275"), ("bench_b", "box", "1.8 0.55 0.55", "6 -2.5 0.275")],
        "plaza_islands": [("island_a", "cylinder", "1.1 0.35", "3 2 0.175"), ("island_b", "cylinder", "0.8 0.35", "6 -2 0.175")],
        "service_lane": [("loading_bay", "box", "3.0 1.4 0.25", "5 2.4 0.125"), ("bollard", "cylinder", "0.18 1.0", "2 -2 0.5")],
    }
    chunks = []
    for name, kind, size, pose in layouts[layout_family]:
        if kind == "box":
            geometry = f"<box><size>{size}</size></box>"
        else:
            radius, length = size.split()
            geometry = f"<cylinder><radius>{radius}</radius><length>{length}</length></cylinder>"
        chunks.append(
            f'<model name="{name}"><static>true</static><pose>{pose} 0 0 0</pose>'
            f'<link name="body"><collision name="collision"><geometry>{geometry}</geometry></collision>'
            f'<visual name="visual"><geometry>{geometry}</geometry>{_material("0.38 0.39 0.37 1", 0.85)}</visual>'
            "</link></model>"
        )
    return "".join(chunks)


def write_g3_worlds(
    registry_path: str | Path,
    xacro_path: str | Path,
    output_dir: str | Path,
) -> dict:
    registry_path, output_dir = Path(registry_path), Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    registry = load_asset_registry(registry_path)
    contract = read_production_camera_contract(xacro_path)
    models, records = [], []
    offset = 0
    for class_id in CLASS_ORDER[1:]:
        for variant_index, variant in enumerate(registry["classes"][class_id]["variants"]):
            kind, values = _geometry_for(class_id, variant_index)
            geometry = _geometry_xml(kind, values)
            palette_name = variant["palette"][variant_index % len(variant["palette"])]
            color = registry["palette_rgb"][palette_name]
            rgba = " ".join(f"{channel / 255:.5f}" for channel in color) + " 1"
            models.append(
                _asset_model(
                    variant["id"],
                    geometry,
                    rgba,
                    CLASS_INDEX[class_id],
                    f"{-200 - offset * 0.3:.3f} 200 -5 0 0 0",
                )
            )
            records.append(
                {
                    "model_name": variant["id"],
                    "class_id": class_id,
                    "semantic_label": CLASS_INDEX[class_id],
                    "variant_index": variant_index,
                    "texture_id": variant["texture"],
                    "geometry_kind": kind,
                    "physical_geometry_values_m": list(values),
                    "scale_factor": 1.0,
                    "license": "Apache-2.0",
                    "source": "project-authored procedural primitive",
                }
            )
            offset += 1
    negative_records = []
    for index, negative_id in enumerate(registry["negative_assets"]):
        name = f"negative_{negative_id}"
        geometry = "<box><size>0.16 0.12 0.08</size></box>"
        models.append(
            _asset_model(
                name,
                geometry,
                "0.22 0.42 0.25 1",
                0,
                f"{-220 - index * .25} 200 -5 0 0 0",
            )
        )
        negative_records.append(
            {"model_name": name, "negative_id": negative_id, "semantic_label": 0}
        )
    worlds = []
    for (
        world_name,
        material_id,
        layout_family,
        rgba,
        roughness,
        split,
        light_diffuse,
        light_direction,
        shadow_profile,
    ) in WORLD_PROFILES:
        world_text = f"""<?xml version="1.0"?>
<sdf version="1.9"><world name="{world_name}">
  <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <light type="directional" name="sun"><pose>0 0 10 0 0 0</pose><cast_shadows>true</cast_shadows><diffuse>{light_diffuse}</diffuse><direction>{light_direction}</direction></light>
  <model name="ground"><static>true</static><link name="ground"><collision name="collision"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry></collision><visual name="visual"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>{_material(rgba, roughness)}</visual></link></model>
  {_layout_models(layout_family)}
  {''.join(models)}
</world></sdf>
"""
        world_text = "\n".join(line.rstrip() for line in world_text.splitlines()) + "\n"
        path = output_dir / f"{world_name}.sdf"
        path.write_text(world_text, encoding="utf-8")
        worlds.append(
            {
                "world_id": world_name,
                "material_id": material_id,
                "layout_family": layout_family,
                "geometry_family": layout_family,
                "background_family": material_id,
                "lighting_family": shadow_profile,
                "path": path.name,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "split_eligibility": [split],
                "allowed_trajectory_family": f"{layout_family}_vehicle_motion",
                "vehicle_spawned_by": "AUTO-05 G3 capture runner",
                "sensor_topics": [
                    "/camera/image",
                    "/camera/depth_image",
                    "/g3/semantic_gt/labels_map",
                    "/g3/instance_gt/labels_map",
                ],
            }
        )
    manifest = {
        "schema_version": 1,
        "dataset_domain": "G3_deployment_aligned_vehicle_camera_gazebo",
        "camera_contract": contract,
        "native_capture_resolution": contract["native_resolution"],
        "training_only_ground_truth": True,
        "production_launch_modified": False,
        "actual_vehicle_model_required": "sanitation_vehicle_description/urdf/sanitation_vehicle.urdf.xacro",
        "static_independent_camera_rig_forbidden": True,
        "world_split_counts": {"train": 4, "val": 2, "test": 2},
        "scenes_per_world": 15,
        "frames_per_scene": 10,
        "vehicle_motion_required": True,
        "distance_envelope_m": [0.5, 8.0],
        "worlds": worlds,
        "assets": records,
        "negative_assets": negative_records,
        "asset_policy": "self-authored procedural, true physical geometry, split-isolated variants",
        "registry_sha256": hashlib.sha256(registry_path.read_bytes()).hexdigest(),
    }
    manifest_path = output_dir / "g3_world_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", required=True)
    parser.add_argument("--xacro", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    print(
        json.dumps(
            write_g3_worlds(args.registry, args.xacro, args.output_dir), indent=2
        )
    )


if __name__ == "__main__":
    main()
