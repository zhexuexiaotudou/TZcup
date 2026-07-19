from __future__ import annotations

import hashlib
import json
from pathlib import Path

from .assets import _geometry_for, _geometry_xml, load_asset_registry
from .g2_contract import read_production_camera_contract
from .rendered import CLASS_INDEX, CLASS_ORDER


WORLD_PROFILES = (
    ("world_a_asphalt_campus", "asphalt_coarse", "0.24 0.25 0.27 1", 0.92),
    ("world_b_concrete_sidewalk", "concrete_light", "0.58 0.57 0.54 1", 0.84),
    ("world_c_wet_dark_ground", "wet_dark", "0.10 0.12 0.13 1", 0.28),
    ("world_d_mixed_curb_vegetation", "mixed_curb_vegetation", "0.31 0.35 0.27 1", 0.76),
)


def _material(rgba: str, roughness: float) -> str:
    return (
        f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>"
        f"<specular>0.10 0.10 0.10 1</specular><pbr><metal>"
        f"<roughness>{roughness}</roughness><metalness>0</metalness>"
        "</metal></pbr></material>"
    )


def _asset_model(name: str, geometry: str, rgba: str, label: int, pose: str) -> str:
    return f"""
    <model name="{name}"><static>true</static><pose>{pose}</pose><link name="body">
      <visual name="visual"><geometry>{geometry}</geometry>{_material(rgba, 0.68)}</visual>
      <collision name="collision"><geometry>{geometry}</geometry></collision>
    </link><plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>{label}</label></plugin></model>"""


def _camera_rig(contract: dict, topic_prefix: str, width: int, height: int) -> str:
    xyz = " ".join(str(value) for value in contract["extrinsics"]["xyz_m"])
    rpy = " ".join(str(value) for value in contract["extrinsics"]["rpy_rad"])
    camera = f"""
          <horizontal_fov>{contract['horizontal_fov_rad']}</horizontal_fov>
          <image><width>{width}</width><height>{height}</height><format>R8G8B8</format></image>
          <clip><near>{contract['near_clip_m']}</near><far>{contract['far_clip_m']}</far></clip>"""
    rate = contract["update_rate_hz"]
    return f"""
    <model name="g2_vehicle_training_rig"><static>true</static><pose>0 0 0.35 0 0 0</pose>
      <link name="base_link">
        <visual name="vehicle_body"><pose>0 0 0 0 0 0</pose><geometry><box><size>0.90 0.60 0.38</size></box></geometry>{_material('0.10 0.17 0.26 1', 0.55)}</visual>
        <collision name="vehicle_collision"><geometry><box><size>0.90 0.60 0.38</size></box></geometry></collision>
      </link>
      <link name="camera_link"><pose relative_to="base_link">{xyz} {rpy}</pose>
        <sensor name="rgbd" type="rgbd_camera"><topic>{topic_prefix}/rgbd</topic><always_on>true</always_on><update_rate>{rate}</update_rate><camera>{camera}<optical_frame_id>{contract['optical_frame']}</optical_frame_id></camera></sensor>
        <sensor name="semantic_gt" type="segmentation"><topic>{topic_prefix}/semantic_gt</topic><always_on>true</always_on><update_rate>{rate}</update_rate><camera><segmentation_type>semantic</segmentation_type>{camera}</camera></sensor>
        <sensor name="instance_gt" type="segmentation"><topic>{topic_prefix}/instance_gt</topic><always_on>true</always_on><update_rate>{rate}</update_rate><camera><segmentation_type>instance</segmentation_type>{camera}</camera></sensor>
      </link>
      <joint name="camera_joint" type="fixed"><parent>base_link</parent><child>camera_link</child></joint>
    </model>"""


def write_g2_worlds(
    registry_path: str | Path,
    xacro_path: str | Path,
    output_dir: str | Path,
    width: int = 384,
    height: int = 288,
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
            geometry = _geometry_xml(kind, values)  # true physical size; no G1 scale factor
            palette_name = variant["palette"][variant_index % len(variant["palette"])]
            color = registry["palette_rgb"][palette_name]
            rgba = " ".join(f"{channel / 255:.5f}" for channel in color) + " 1"
            z = max(float(values[-1]) / 2.0, 0.008)
            pose = f"{25 + offset * 0.3:.3f} 25 {z:.4f} 0 0 0"
            models.append(_asset_model(variant["id"], geometry, rgba, CLASS_INDEX[class_id], pose))
            records.append({
                "model_name": variant["id"], "class_id": class_id,
                "semantic_label": CLASS_INDEX[class_id], "variant_index": variant_index,
                "texture_id": variant["texture"], "geometry_kind": kind,
                "physical_geometry_values_m": list(values), "scale_factor": 1.0,
                "license": "Apache-2.0", "source": "project-authored procedural primitive",
            })
            offset += 1
    negative_records = []
    for index, negative_id in enumerate(registry["negative_assets"]):
        name = f"negative_{negative_id}"
        geometry = "<box><size>0.16 0.12 0.08</size></box>"
        models.append(_asset_model(name, geometry, "0.22 0.42 0.25 1", 0, f"{28 + index * .25} 27 0.04 0 0 0"))
        negative_records.append({"model_name": name, "negative_id": negative_id, "semantic_label": 0})
    worlds = []
    for world_name, material_id, rgba, roughness in WORLD_PROFILES:
        prefix = f"g2/{world_name}"
        world = f"""<?xml version="1.0"?>
<sdf version="1.9"><world name="{world_name}">
  <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <light type="directional" name="sun"><pose>0 0 10 0 0 0</pose><cast_shadows>true</cast_shadows><diffuse>0.82 0.80 0.76 1</diffuse><direction>-0.45 0.2 -0.87</direction></light>
  <model name="ground"><static>true</static><link name="ground"><collision name="collision"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry></collision><visual name="visual"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>{_material(rgba, roughness)}</visual></link></model>
  {_camera_rig(contract, prefix, width, height)}
  {''.join(models)}
</world></sdf>
"""
        world = "\n".join(line.rstrip() for line in world.splitlines()) + "\n"
        path = output_dir / f"{world_name}.sdf"
        path.write_text(world, encoding="utf-8")
        worlds.append({
            "world_id": world_name, "material_id": material_id, "path": path.name,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "split_eligibility": (
                ["train"] if world_name in {"world_a_asphalt_campus", "world_b_concrete_sidewalk"}
                else ["val"] if world_name == "world_c_wet_dark_ground" else ["test"]
            ),
            "topics": [f"/{prefix}/rgbd/image", f"/{prefix}/rgbd/depth_image", f"/{prefix}/semantic_gt/labels_map", f"/{prefix}/instance_gt/labels_map"],
        })
    manifest = {
        "schema_version": 1,
        "dataset_domain": "G2_deployment_aligned_vehicle_camera_gazebo",
        "camera_contract": contract,
        "selected_resolution": {"width": width, "height": height, "status": "candidate_not_yet_selected_by_scan"},
        "training_only_ground_truth": True,
        "production_launch_modified": False,
        "vehicle_motion_required": True,
        "distance_envelope_m": [0.5, 4.0],
        "worlds": worlds,
        "assets": records,
        "negative_assets": negative_records,
        "asset_policy": "self-authored procedural, true physical geometry, no G1 enlargement",
        "registry_sha256": hashlib.sha256(registry_path.read_bytes()).hexdigest(),
    }
    manifest_path = output_dir / "g2_world_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", required=True)
    parser.add_argument("--xacro", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--width", type=int, default=384)
    parser.add_argument("--height", type=int, default=288)
    args = parser.parse_args()
    print(json.dumps(write_g2_worlds(args.registry, args.xacro, args.output_dir, args.width, args.height), indent=2))


if __name__ == "__main__":
    main()
