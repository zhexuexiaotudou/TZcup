from __future__ import annotations

import hashlib
import json
from pathlib import Path

import yaml

from .assets import _geometry_for, _geometry_xml, load_asset_registry
from .rendered import CLASS_INDEX, CLASS_ORDER


def _material(rgba: str, roughness: float = 0.65) -> str:
    return (
        f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>"
        f"<specular>0.08 0.08 0.08 1</specular>"
        f"<pbr><metal><roughness>{roughness}</roughness><metalness>0.0</metalness></metal></pbr>"
        "</material>"
    )


def _model(name: str, geometry: str, rgba: str, label: int, pose: str) -> str:
    return f"""
    <model name="{name}">
      <static>true</static><pose>{pose}</pose>
      <link name="body">
        <visual name="visual"><geometry>{geometry}</geometry>{_material(rgba)}</visual>
        <collision name="collision"><geometry>{geometry}</geometry></collision>
      </link>
      <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>{label}</label></plugin>
    </model>"""


def _camera_rig() -> str:
    camera = """
          <horizontal_fov>1.0472</horizontal_fov>
          <image><width>128</width><height>96</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>12.0</far></clip>"""
    return f"""
    <model name="g1_camera_rig">
      <static>true</static><pose>0 0 2.6 0 1.57079632679 0</pose>
      <link name="camera_link">
        <sensor name="rgbd" type="rgbd_camera">
          <topic>g1/rgbd</topic><always_on>true</always_on><update_rate>10</update_rate>
          <camera>{camera}<optical_frame_id>g1_camera_optical</optical_frame_id></camera>
        </sensor>
        <sensor name="semantic" type="segmentation">
          <topic>g1/semantic</topic><always_on>true</always_on><update_rate>10</update_rate>
          <camera><segmentation_type>semantic</segmentation_type>{camera}</camera>
        </sensor>
        <sensor name="instance" type="segmentation">
          <topic>g1/instance</topic><always_on>true</always_on><update_rate>10</update_rate>
          <camera><segmentation_type>instance</segmentation_type>{camera}</camera>
        </sensor>
      </link>
    </model>"""


def write_g1_world(registry_path: str | Path, output_path: str | Path) -> dict:
    registry_path = Path(registry_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    registry = load_asset_registry(registry_path)
    models = []
    records = []
    visible_slot = 0
    for class_id in CLASS_ORDER[1:]:
        for variant_index, variant in enumerate(registry["classes"][class_id]["variants"]):
            color = registry["palette_rgb"][variant["palette"][variant_index % len(variant["palette"])]]
            rgba = " ".join(f"{channel / 255.0:.5f}" for channel in color) + " 1"
            kind, values = _geometry_for(class_id, variant_index)
            # The 128x96 deployment input otherwise leaves discrete litter at
            # only 1-3 pixels across.  G1 uses conservative visual collision
            # proxies while preserving the class's physical aspect ratio.
            if class_id == "plastic_bottle":
                values = (values[0] * 1.50, values[1])
            elif class_id == "metal_can":
                values = (values[0] * 1.35, values[1])
            geometry = _geometry_xml(kind, values)
            if variant_index == 0:
                x = -1.20 + 0.60 * visible_slot
                y = -0.20 + 0.20 * (visible_slot % 2)
                z = max(values[-1] / 2.0, 0.01)
                pose = f"{x:.3f} {y:.3f} {z:.3f} 0 0 {0.25 * visible_slot:.3f}"
                visible_slot += 1
            else:
                pose = f"{20 + variant_index:.3f} {20 + CLASS_INDEX[class_id]:.3f} 0.1 0 0 0"
            models.append(_model(variant["id"], geometry, rgba, CLASS_INDEX[class_id], pose))
            records.append({
                "model_name": variant["id"], "class_id": class_id,
                "semantic_label": CLASS_INDEX[class_id], "variant_index": variant_index,
                "texture_id": variant["texture"], "palette": variant["palette"],
                "geometry_kind": kind, "geometry_values_m": list(values),
                "initial_pose": pose,
            })
    negative_colors = [(0.68, 0.16, 0.18), (0.18, 0.56, 0.24), (0.18, 0.30, 0.72)]
    for index, negative_id in enumerate(registry["negative_assets"]):
        rgba = " ".join(f"{value:.4f}" for value in negative_colors[index % 3]) + " 1"
        geometry = "<box><size>0.16 0.12 0.08</size></box>"
        pose = f"{22 + index:.3f} 24 0.04 0 0 0"
        models.append(_model(f"negative_{negative_id}", geometry, rgba, 0, pose))
        records.append({
            "model_name": f"negative_{negative_id}", "class_id": "background",
            "semantic_label": 0, "negative_id": negative_id, "initial_pose": pose,
        })
    world = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="stage5br_g1">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <light type="directional" name="sun"><pose>0 0 10 0 0 0</pose><cast_shadows>true</cast_shadows>
      <diffuse>0.82 0.80 0.76 1</diffuse><specular>0.15 0.15 0.15 1</specular><direction>-0.45 0.2 -0.87</direction>
    </light>
    <model name="g1_ground"><static>true</static><link name="ground">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry></collision>
      <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>
        {_material('0.36 0.38 0.40 1', 0.9)}</visual>
    </link></model>
    {_camera_rig()}
    {''.join(models)}
  </world>
</sdf>
"""
    output_path.write_text(world, encoding="utf-8")
    registry_hash = hashlib.sha256(registry_path.read_bytes()).hexdigest()
    world_hash = hashlib.sha256(output_path.read_bytes()).hexdigest()
    manifest = {
        "schema_version": 1, "dataset_domain": "G1_actual_gazebo_camera_rendered_synthetic",
        "world": output_path.name, "world_sha256": world_hash,
        "registry_sha256": registry_hash, "class_order": list(CLASS_ORDER),
        "camera_contract": {
            "co_visible_pose": "g1_camera_rig/camera_link", "camera_height_m": 2.6,
            "width": 128, "height": 96,
            "horizontal_fov_rad": 1.0472, "update_rate_hz": 10,
            "topics": ["/g1/rgbd/image", "/g1/rgbd/depth_image",
                       "/g1/semantic/labels_map", "/g1/instance/labels_map"],
        },
        "models": records, "online_fuel_dependency": False,
        "annotation_source": "Gazebo Harmonic SegmentationCamera + Label system",
    }
    manifest_path = output_path.with_suffix(".manifest.json")
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    print(json.dumps(write_g1_world(args.registry, args.output), indent=2))


if __name__ == "__main__":
    main()
