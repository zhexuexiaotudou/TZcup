from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re


MODELS = {
    "same_color_non_garbage": ("stage5br6_same_color", "<box><size>0.18 0.12 0.08</size></box>", "0.05 0.55 0.18 1"),
    "bottle_or_can_shaped_obstacle": ("stage5br6_bottle_shape", "<cylinder><radius>0.045</radius><length>0.18</length></cylinder>", "0.25 0.33 0.36 1"),
    "wet_ground_non_puddle": ("stage5br6_wet_patch", "<cylinder><radius>0.24</radius><length>0.006</length></cylinder>", "0.05 0.08 0.10 1"),
    "shadow": ("stage5br6_shadow_patch", "<box><size>0.38 0.20 0.004</size></box>", "0.035 0.035 0.04 1"),
    "leaf_background_non_target": ("stage5br6_leaf_background", "<box><size>0.28 0.18 0.018</size></box>", "0.22 0.30 0.08 1"),
    "vehicle_self_structure": ("stage5br6_vehicle_bracket", "<box><size>0.22 0.08 0.10</size></box>", "0.12 0.16 0.19 1"),
}


def build(source: Path, output: Path) -> dict:
    text = source.read_text(encoding="utf-8")
    chunks = []
    for index, (category, (name, geometry, rgba)) in enumerate(MODELS.items()):
        chunks.append(f"""
  <model name="{name}"><static>true</static><pose>{-240-index} 220 -5 0 0 0</pose><link name="body">
    <collision name="collision"><geometry>{geometry}</geometry></collision>
    <visual name="visual"><geometry>{geometry}</geometry><material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><specular>0.7 0.7 0.7 1</specular><pbr><metal><roughness>0.18</roughness><metalness>0.05</metalness></metal></pbr></material></visual>
  </link><plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>0</label></plugin></model>""")
    if "</world>" not in text:
        raise RuntimeError("source SDF has no world close tag")
    rendered = re.sub(r'<world name="[^"]+">', f'<world name="{output.stem}">', text, count=1)
    rendered = rendered.replace("</world>", "\n".join(chunks) + "\n</world>")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(rendered, encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "stage": "Stage5BR6-A",
        "source_world": source.name,
        "source_world_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "output_world": output.name,
        "runtime_world_id": output.stem,
        "output_world_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
        "training_only_label_zero_models": [
            {"negative_category": category, "model_name": values[0], "semantic_label": 0, "project_authored_primitive": True}
            for category, values in MODELS.items()
        ],
        "production_world_modified": False,
    }
    manifest_path = output.with_suffix(".manifest.json")
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    print(json.dumps(build(Path(args.source), Path(args.output)), indent=2))


if __name__ == "__main__":
    main()
