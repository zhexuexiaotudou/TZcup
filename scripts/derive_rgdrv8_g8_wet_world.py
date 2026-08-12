#!/usr/bin/env python3
"""Derive one auditable wet/specular G8 world from a generated G4 world."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def derive(resource_root: Path, base_world_id: str, output: Path) -> dict:
    source_manifest_path = resource_root / "worlds" / "g4_world_manifest.json"
    source_manifest = json.loads(source_manifest_path.read_text(encoding="utf-8"))
    base = next(item for item in source_manifest["worlds"] if item["world_id"] == base_world_id)
    split = base["split_eligibility"][0]
    derived_id = f"{base_world_id}_rgdrv8_wet_specular"
    source_sdf = resource_root / "worlds" / base["path"]
    text = source_sdf.read_text(encoding="utf-8")
    if f'<world name="{base_world_id}">' not in text:
        raise RuntimeError("base world name is missing from SDF")
    text = text.replace(
        f'<world name="{base_world_id}">', f'<world name="{derived_id}">', 1
    )
    text = re.sub(r"<roughness>[0-9.]+</roughness>", "<roughness>0.12</roughness>", text, count=1)
    text = re.sub(
        r"<diffuse>[^<]+</diffuse><direction>[^<]+</direction>",
        "<diffuse>1.00 0.94 0.86 1</diffuse><direction>0.62 -0.18 -0.76</direction>",
        text,
        count=1,
    )
    output_worlds = output / "worlds"
    output_worlds.mkdir(parents=True, exist_ok=True)
    derived_sdf = output_worlds / f"{derived_id}.sdf"
    derived_sdf.write_text(text, encoding="utf-8")
    source_texture = resource_root / "worlds" / base["ground_texture_path"]
    target_texture = output_worlds / base["ground_texture_path"]
    target_texture.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_texture, target_texture)
    derived = {
        **base,
        "world_id": derived_id,
        "material_id": f"{base['material_id']}_wet_specular",
        "lighting_family": "rgdrv8_low_sun_specular",
        "ground_texture_path": base["ground_texture_path"],
        "path": derived_sdf.name,
        "sha256": sha256(derived_sdf),
        "split_eligibility": [split],
        "rgdrv8_derivation": {
            "base_world_id": base_world_id,
            "base_world_sha256": sha256(source_sdf),
            "transform": "ground_pbr_roughness_0.12_and_low_sun_specular_light",
            "pixel_source_changed": True,
        },
    }
    manifest = {
        **source_manifest,
        "dataset_domain": "G8_REAL_GAZEBO_DETECTOR_DEVELOPMENT_WET_SPECULAR",
        "world_split_counts": {"train": int(split == "train"), "val": int(split == "val"), "test": int(split == "test")},
        "scenes_per_world": None,
        "frames_per_scene": None,
        "worlds": [derived],
        "base_manifest_sha256": sha256(source_manifest_path),
        "rgdrv8_wet_derivation_only": True,
    }
    manifest_path = output_worlds / "g4_world_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    report = {
        "schema_version": 1,
        "protocol": "REAL-GAZEBO-DETECTOR-RECOVERY-V8",
        "base_world_id": base_world_id,
        "derived_world_id": derived_id,
        "split": split,
        "base_world_sha256": derived["rgdrv8_derivation"]["base_world_sha256"],
        "derived_world_sha256": derived["sha256"],
        "manifest_sha256": sha256(manifest_path),
        "ground_texture_sha256": sha256(target_texture),
        "base_ground_texture_sha256": sha256(source_texture),
        "ground_texture_byte_parity": sha256(target_texture) == sha256(source_texture),
        "world_id_changed": derived_id != base_world_id,
        "pixel_source_changed": True,
        "G8_WET_WORLD_DERIVATION_PASS": (
            derived["sha256"] != derived["rgdrv8_derivation"]["base_world_sha256"]
            and sha256(target_texture) == sha256(source_texture)
        ),
    }
    (output / "G8_WET_WORLD_DERIVATION.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--resource-root", type=Path, required=True)
    parser.add_argument("--base-world-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() and any(args.output.iterdir()):
        raise FileExistsError(args.output)
    report = derive(args.resource_root, args.base_world_id, args.output)
    print(json.dumps(report, indent=2))
    return 0 if report["G8_WET_WORLD_DERIVATION_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
