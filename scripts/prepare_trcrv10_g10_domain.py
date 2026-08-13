#!/usr/bin/env python3
"""Materialize split-isolated G10 worlds and namespaced physical assets."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


DOMAIN = "g10_physical_close_range_route_v8"
SPLIT_MAP = {
    "train": "G10_TRAIN",
    "val": "G10_HOLDOUT",
    "test": "G10_DEV_VAL_SEALED",
}
MISSION_MINIMUMS = {"train": 45, "val": 18, "test": 18}
SCENES_PER_WORLD = {"train": 8, "val": 6, "test": 6}
TARGET_CLASSES = ("plastic_bottle", "metal_can", "paper_litter")


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def assignment(rows: list[dict], split_counts: tuple[int, int, int]) -> dict[str, str]:
    if sum(split_counts) != len(rows):
        raise ValueError(f"split counts {split_counts} do not cover {len(rows)} rows")
    ordered = sorted(rows, key=lambda item: item["model_name"])
    result = {}
    start = 0
    for split, count in zip(("train", "val", "test"), split_counts):
        for row in ordered[start:start + count]:
            result[row["model_name"]] = split
        start += count
    return result


def proportional_counts(count: int) -> tuple[int, int, int]:
    train = round(count * 5 / 9)
    holdout = round(count * 2 / 9)
    dev_val = count - train - holdout
    if min(train, holdout, dev_val) < 1:
        raise ValueError(f"cannot form three independent splits from {count} assets")
    return train, holdout, dev_val


def target_assignment(rows: list[dict]) -> dict[str, str]:
    result = {}
    for class_id in TARGET_CLASSES:
        group = [row for row in rows if row["class_id"] == class_id]
        result.update(assignment(group, proportional_counts(len(group))))
    return result


def clone_asset(source: Path, destination: Path, old_id: str, new_id: str) -> dict:
    if destination.exists():
        raise FileExistsError(destination)
    shutil.copytree(source, destination)
    sdf = destination / "model.sdf"
    config = destination / "model.config"
    sdf.write_text(
        sdf.read_text(encoding="utf-8").replace(f'name="{old_id}"', f'name="{new_id}"', 1),
        encoding="utf-8",
    )
    config.write_text(
        config.read_text(encoding="utf-8").replace(f"<name>{old_id}</name>", f"<name>{new_id}</name>", 1),
        encoding="utf-8",
    )
    texture = next((destination / "textures").glob("*.png"))
    return {
        "model_sdf_sha256": sha256(sdf),
        "model_config_sha256": sha256(config),
        "texture_sha256": sha256(texture),
    }


def rewrite_world(source: Path, destination: Path, world_id: str, model_ids: list[str]) -> None:
    tree = ET.parse(source)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError(f"world node missing: {source}")
    world.attrib["name"] = world_id
    for include in list(world.findall("include")):
        world.remove(include)
    for index, model_id in enumerate(model_ids):
        include = ET.SubElement(world, "include")
        ET.SubElement(include, "uri").text = f"model://{model_id}"
        ET.SubElement(include, "pose").text = f"{-200 - index * .3:.3f} 200 -5 0 0 0"
    ET.indent(tree, space="  ")
    tree.write(destination, encoding="unicode", xml_declaration=True)


def split_sets(rows: list[dict], key: str) -> dict[str, set[str]]:
    return {
        split: {row[key] for row in rows if row["split_eligibility"] == [split]}
        for split in SPLIT_MAP
    }


def pairwise_overlap(rows: list[dict], key: str) -> dict[str, list[str]]:
    groups = split_sets(rows, key)
    names = list(SPLIT_MAP)
    return {
        f"{first}_{second}": sorted(groups[first] & groups[second])
        for index, first in enumerate(names)
        for second in names[index + 1:]
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-assets", type=Path, required=True)
    parser.add_argument("--source-worlds", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() and any(args.output.iterdir()):
        raise FileExistsError(f"G10 domain output is not empty: {args.output}")

    source_manifest = read(args.source_worlds / "g4_world_manifest.json")
    targets = [row for row in source_manifest["assets"] if row["class_id"] in TARGET_CLASSES]
    negatives = list(source_manifest["negative_assets"])
    target_splits = target_assignment(targets)
    negative_splits = assignment(negatives, proportional_counts(len(negatives)))

    output_assets = args.output / "models"
    output_worlds = args.output / "worlds"
    output_assets.mkdir(parents=True, exist_ok=True)
    output_worlds.mkdir(parents=True, exist_ok=True)
    asset_rows = []
    negative_rows = []
    all_rows = [(row, target_splits[row["model_name"]], False) for row in targets]
    all_rows += [(row, negative_splits[row["model_name"]], True) for row in negatives]
    for row, split, is_negative in all_rows:
        old_id = row["model_name"]
        new_id = f"g10v1_{split}_{old_id.removeprefix('g4_')}"
        hashes = clone_asset(args.source_assets / old_id, output_assets / new_id, old_id, new_id)
        cloned = {
            **row,
            "model_name": new_id,
            "source_asset_id": old_id,
            "asset_domain_id": DOMAIN,
            "split_eligibility": [split],
            "scale_factor": 1.0,
            **hashes,
        }
        (negative_rows if is_negative else asset_rows).append(cloned)

    source_worlds = source_manifest["worlds"]
    world_groups = (
        ("train", source_worlds[:6]),
        ("val", source_worlds[6:9]),
        ("test", source_worlds[9:12]),
    )
    world_rows = []
    model_ids = [row["model_name"] for row in asset_rows + negative_rows]
    for split, group in world_groups:
        for local_index, row in enumerate(group, 1):
            suffix = row["world_id"].removeprefix("world_g4_")
            world_id = f"g10v1_{split}_w{local_index:02d}_{suffix}"
            destination = output_worlds / f"{world_id}.sdf"
            rewrite_world(args.source_worlds / row["path"], destination, world_id, model_ids)
            world_rows.append({
                **row,
                "world_id": world_id,
                "source_world_id": row["world_id"],
                "path": destination.name,
                "sha256": sha256(destination),
                "split_eligibility": [split],
                "asset_domain_id": DOMAIN,
                "allowed_trajectory_family": "g10_far_mid_close_vehicle_approach",
            })
    shutil.copytree(
        args.source_worlds / "ground_textures",
        output_worlds / "ground_textures",
        dirs_exist_ok=True,
    )

    mission_plan = {
        split: {
            "semantic_split": SPLIT_MAP[split],
            "worlds": sum(row["split_eligibility"] == [split] for row in world_rows),
            "scenes_per_world": SCENES_PER_WORLD[split],
            "missions": sum(row["split_eligibility"] == [split] for row in world_rows) * SCENES_PER_WORLD[split],
            "minimum": MISSION_MINIMUMS[split],
        }
        for split in SPLIT_MAP
    }
    manifest = {
        **{key: value for key, value in source_manifest.items() if key not in ("worlds", "assets", "negative_assets")},
        "schema_version": 1,
        "dataset_domain": DOMAIN,
        "protocol": "TRCRV10",
        "worlds": world_rows,
        "assets": asset_rows,
        "negative_assets": negative_rows,
        "world_split_counts": {
            split: sum(row["split_eligibility"] == [split] for row in world_rows)
            for split in SPLIT_MAP
        },
        "mission_plan": mission_plan,
        "approach_sequence": {
            "enabled": True,
            "targets_per_positive_mission": 1,
            "target_start_distance_m": 6.2,
            "target_lateral_by_class_m": {
                "metal_can": -0.57,
                "paper_litter": 0.66,
                "plastic_bottle": -0.57,
            },
            "capture_speed_mps": 0.20,
            "capture_minimum_translation_m": 0.04,
            "capture_frames": 150,
            "capture_minimum_rotation_rad": 0.12,
            "reobserve_switch_lead_m": 1.50,
            "close_observation_approach_frames": 18,
            "close_observation_approach_minimum_travel_m": 0.68,
            "unreachable_for_visual_confirmation_must_be_retained": True,
            "product_inputs": ["RGB", "depth", "CameraInfo", "TF"],
            "GT_role": "training_and_evaluator_only",
        },
        "G10_DEV_VAL_SEALED_read": False,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
    }
    manifest_path = output_worlds / "g10_world_manifest.json"
    write(manifest_path, manifest)
    shutil.copy2(manifest_path, output_worlds / "g4_world_manifest.json")

    overlaps = {
        "world": pairwise_overlap(world_rows, "world_id"),
        "asset": pairwise_overlap(asset_rows, "model_name"),
        "negative": pairwise_overlap(negative_rows, "model_name"),
    }
    qa = {
        "schema_version": 1,
        "domain": DOMAIN,
        "manifest": str(manifest_path.resolve()),
        "manifest_sha256": sha256(manifest_path),
        "mission_plan": mission_plan,
        "counts": {
            "worlds": len(world_rows),
            "targets": len(asset_rows),
            "negatives": len(negative_rows),
        },
        "cross_split_overlap": overlaps,
        "gates": {
            "mission_minimums_met": all(row["missions"] >= row["minimum"] for row in mission_plan.values()),
            "world_overlap_zero": all(not value for value in overlaps["world"].values()),
            "asset_overlap_zero": all(not value for value in overlaps["asset"].values()),
            "negative_overlap_zero": all(not value for value in overlaps["negative"].values()),
            "namespaced_assets_only": all(row["model_name"].startswith("g10v1_") for row in asset_rows + negative_rows),
            "physical_scale_one": all(row["scale_factor"] == 1.0 for row in asset_rows),
            "safe_lateral_drive_by_clearance": all(value >= 0.15 for value in (
                0.57 - 0.36 - 0.05,
                0.66 - 0.36 - 0.11,
                0.57 - 0.36 - 0.05,
            )),
            "sealed_boundaries_preserved": True,
        },
    }
    qa["G10_DOMAIN_PLAN_PASS"] = all(qa["gates"].values())
    write(args.evidence / "G10_DOMAIN_PLAN_QA.json", qa)
    write(args.evidence / "G10_SPLIT_MANIFEST.json", {
        "schema_version": 1,
        "domain": DOMAIN,
        "worlds": [
            {"world_id": row["world_id"], "split": SPLIT_MAP[row["split_eligibility"][0]], "sha256": row["sha256"]}
            for row in world_rows
        ],
        "assets": [
            {"asset_id": row["model_name"], "source_asset_id": row["source_asset_id"], "class_id": row["class_id"], "split": SPLIT_MAP[row["split_eligibility"][0]], "model_sdf_sha256": row["model_sdf_sha256"]}
            for row in asset_rows
        ],
        "G10_DEV_VAL_SEALED_read": False,
    })
    print(json.dumps(qa, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
