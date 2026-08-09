#!/usr/bin/env python3
"""Prepare one isolated native-Gazebo capture root for AUTO-05R D1-D5."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil


ROLE_SPECS = {
    "D1": {
        "source_world": "world_g4_01_asphalt_campus",
        "asset_source_split": "val",
        "negative_source_split": "train",
        "world_seen": True,
        "asset_seen": False,
    },
    "D2": {
        "source_world": "world_g4_09_light_paver_pedestrian",
        "asset_source_split": "train",
        "negative_source_split": "train",
        "world_seen": False,
        "asset_seen": True,
    },
    "D3": {
        "source_world": "world_g4_01_asphalt_campus",
        "asset_source_split": "train",
        "negative_source_split": "train",
        "world_seen": True,
        "geometry_seen": True,
        "material_seen": False,
    },
    "D4": {
        "source_world": "world_g4_01_asphalt_campus",
        "asset_source_split": "train",
        "negative_source_split": "train",
        "world_seen": True,
        "asset_seen": True,
        "lighting_seen": False,
    },
    "D5": {
        "source_world": "world_g4_01_asphalt_campus",
        "asset_source_split": "train",
        "negative_source_split": "val",
        "force_negative_only": True,
        "world_seen": True,
        "negative_only": True,
        "negative_asset_unseen": True,
    },
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare_capture_root(base: Path, role: str, output: Path) -> dict:
    if role not in ROLE_SPECS:
        raise ValueError(f"unsupported factorized role: {role}")
    if output.exists():
        raise FileExistsError(f"capture output must not exist: {output}")
    manifest_path = base / "worlds" / "g4_world_manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"base world manifest is missing: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    spec = dict(ROLE_SPECS[role])
    source = next(
        item for item in manifest["worlds"]
        if item["world_id"] == spec["source_world"]
    )

    shutil.copytree(base / "models", output / "models")
    shutil.copytree(base / "worlds", output / "worlds")
    source_path = output / "worlds" / source["path"]
    world = dict(source)
    world["split_eligibility"] = [role]
    world["factorized_diagnostic_role"] = role
    world["source_world_id"] = source["world_id"]

    if role in {"D3", "D4"}:
        diagnostic_id = f"world_auto05r_{role.lower()}_factorized"
        text = source_path.read_text(encoding="utf-8")
        text = text.replace(
            f'<world name="{source["world_id"]}">',
            f'<world name="{diagnostic_id}">',
            1,
        )
        if role == "D3":
            text = text.replace(
                f"ground_textures/{source['world_id']}.png",
                "ground_textures/world_g4_09_light_paver_pedestrian.png",
            )
            text = re.sub(
                r"<roughness>[0-9.]+</roughness>",
                "<roughness>0.37</roughness>",
                text,
                count=1,
            )
            world["material_id"] = "diagnostic_unseen_light_paver_wet_mix"
            world["ground_texture_family"] = "diagnostic_unseen_material"
            world["ground_texture_path"] = (
                "ground_textures/world_g4_09_light_paver_pedestrian.png"
            )
        else:
            text = re.sub(
                r"<diffuse>[^<]+</diffuse>",
                "<diffuse>0.41 0.48 0.63 1</diffuse>",
                text,
                count=1,
            )
            text = re.sub(
                r"<direction>[^<]+</direction>",
                "<direction>0.72 -0.58 -0.39</direction>",
                text,
                count=1,
            )
            world["lighting_family"] = "diagnostic_unseen_low_angle_blue_hour"
        diagnostic_path = output / "worlds" / f"{diagnostic_id}.sdf"
        diagnostic_path.write_text(
            "\n".join(line.rstrip() for line in text.splitlines()) + "\n",
            encoding="utf-8",
        )
        world["world_id"] = diagnostic_id
        world["path"] = diagnostic_path.name
        world["sha256"] = _sha256(diagnostic_path)

    manifest["dataset_domain"] = "G4_factorized_native_gazebo_diagnostic"
    manifest["worlds"] = [world]
    manifest["world_split_counts"] = {role: 1}
    manifest["scenes_per_world"] = 10
    manifest["factorized_diagnostic"] = {
        "schema_version": 1,
        "role": role,
        "single_factor_native_capture": True,
        "source_formal_manifest_sha256": _sha256(manifest_path),
        "asset_source_split": spec["asset_source_split"],
        "negative_source_split": spec["negative_source_split"],
        "force_negative_only": bool(spec.get("force_negative_only", False)),
        "axis_expectations": {
            key: value
            for key, value in spec.items()
            if key.endswith("_seen")
            or key in {"negative_only", "negative_asset_unseen"}
        },
    }
    output_manifest = output / "worlds" / "g4_world_manifest.json"
    output_manifest.write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    runner_environment = {
        "AUTO05R_DATA_ROOT": "/data/g4_screening_native",
        "AUTO05R_SKIP_WORLD_GENERATION": "1",
        "AUTO05R_WORLD_MANIFEST": (
            "/data/g4_screening_native/worlds/g4_world_manifest.json"
        ),
        "AUTO05R_SCENES_PER_WORLD": "10",
        "AUTO05R_DIAGNOSTIC_ROLE": role,
        "AUTO05R_ASSET_SOURCE_SPLIT": spec["asset_source_split"],
        "AUTO05R_NEGATIVE_SOURCE_SPLIT": spec["negative_source_split"],
        "AUTO05R_FORCE_NEGATIVE_ONLY": (
            "1" if spec.get("force_negative_only") else "0"
        ),
    }
    result = {
        "schema_version": 1,
        "role": role,
        "output": str(output),
        "world_id": world["world_id"],
        "world_sha256": world["sha256"],
        "manifest_sha256": _sha256(output_manifest),
        "runner_environment": runner_environment,
        "factor_contract": manifest["factorized_diagnostic"],
    }
    (output / "factorized_capture_plan.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-data-root", required=True, type=Path)
    parser.add_argument("--role", required=True, choices=tuple(ROLE_SPECS))
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = prepare_capture_root(args.base_data_root, args.role, args.output)
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
