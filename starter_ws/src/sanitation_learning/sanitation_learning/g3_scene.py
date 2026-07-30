from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import random

from .g2_scene import set_poses


SPLIT_VARIANTS = {"train": {0, 1, 2}, "val": {3}, "test": {4, 5}}


def randomize(
    manifest_path: Path,
    world_id: str,
    scene_seed: int,
    scene_index: int,
    output: Path,
) -> dict:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    world = next(item for item in manifest["worlds"] if item["world_id"] == world_id)
    split = world["split_eligibility"][0]
    rng = random.Random(20260730 + scene_seed * 7919)
    assets = manifest["assets"]
    negatives = manifest["negative_assets"]
    negative_partitions = {
        "train": negatives[:8],
        "val": negatives[8:10],
        "test": negatives[10:12],
    }
    force_negative = scene_index == 0 if split == "train" else scene_index < 5
    selected = []
    if not force_negative:
        for class_id in sorted({item["class_id"] for item in assets}):
            pool = [
                item
                for item in assets
                if item["class_id"] == class_id
                and item["variant_index"] in SPLIT_VARIANTS[split]
            ]
            count = (scene_index + sum(map(ord, class_id))) % 4
            if scene_index == 5:
                count = max(1, count)
            selected.extend(rng.sample(pool, min(count, len(pool))))
    negative_pool = negative_partitions[split]
    negative_count = max(1, (scene_index * 3 + 1) % (len(negative_pool) + 1))
    selected_negatives = rng.sample(
        negative_pool, min(negative_count, len(negative_pool))
    )
    if force_negative and not selected_negatives:
        selected_negatives = [negative_pool[0]]

    updates = [
        {"name": "sanitation_vehicle", "xyz": [-8.0, 0.0, 0.18], "yaw": 0.0}
    ]
    for index, item in enumerate(assets + negatives):
        updates.append(
            {
                "name": item["model_name"],
                "xyz": [-200.0 - index * 0.25, 200.0, -5.0],
                "yaw": 0.0,
            }
        )
    observation_phase = "before" if scene_index in (10, 12) else "after" if scene_index in (11, 13) else None
    objects = []
    selected_all = selected + selected_negatives
    for index, item in enumerate(selected_all):
        if observation_phase == "before":
            distance = rng.uniform(5.0, 8.0)
        elif observation_phase == "after":
            distance = rng.uniform(0.8, 3.0)
        else:
            distance = rng.uniform(0.5, 8.0)
        lateral = rng.uniform(-2.1, 2.1)
        # The 10-frame physical capture spans about 2.25 m and the chassis
        # extends 0.58 m forward. Keep every object inside the first 4.5 m out
        # of the 0.65 m half-width drive corridor so a visible static asset
        # cannot truncate the required trajectory.
        if distance < 4.5 and abs(lateral) < 0.65:
            lateral = 0.75 if lateral >= 0 else -0.75
        if scene_index % 5 == 2 and index < 2:
            distance = 3.0 + 0.08 * index
            lateral = 0.9 + 0.06 * index
        geometry = item.get("physical_geometry_values_m", [0.16, 0.12, 0.08])
        z = max(float(geometry[-1]) / 2.0, 0.008)
        pose = {
            "name": item["model_name"],
            "xyz": [-8.0 + distance, lateral, z],
            "yaw": rng.uniform(-math.pi, math.pi),
        }
        updates.append(pose)
        objects.append(
            {
                "model_name": item["model_name"],
                "class_id": item.get("class_id", "background"),
                "semantic_label": item["semantic_label"],
                "xyz_m": pose["xyz"],
                "yaw_rad": pose["yaw"],
                "distance_bucket_m": [0.5, 2.0]
                if distance < 2
                else [2.0, 4.0]
                if distance < 4
                else [4.0, 8.0],
                "horizontal_region": "left"
                if lateral > 0.7
                else "right"
                if lateral < -0.7
                else "middle",
                "keepout_or_unreachable": abs(lateral) > 1.6,
                "physical_geometry_values_m": geometry,
            }
        )
    dynamic_plan = None
    if scene_index % 4 == 0 and selected_negatives:
        moving = next(
            item
            for item in objects
            if item["model_name"] == selected_negatives[0]["model_name"]
        )
        # Dynamic hard negatives must remain outside the chassis corridor for
        # the full 10-frame capture. Place them at a safe lateral offset and
        # move longitudinally; moving farther sideways can push an object into
        # a world's curb or wall and stall the Gazebo physics solver.
        if abs(moving["xyz_m"][1]) < 0.75:
            moving["xyz_m"][1] = 0.75 if moving["xyz_m"][1] >= 0 else -0.75
        dynamic_plan = {
            "model_name": moving["model_name"],
            "start_xyz_m": moving["xyz_m"],
            "delta_per_frame_m": [0.08, 0.0, 0.0],
            "executed_by_capture": True,
        }
    set_poses(world_id, updates)
    classes = sorted({item["class_id"] for item in assets})
    scene = {
        "schema_version": 1,
        "scene_seed": scene_seed,
        "scene_index_in_world": scene_index,
        "world_id": world_id,
        "split": split,
        "world_sha256": world["sha256"],
        "trajectory_id": f"{world_id}_scene_{scene_seed:04d}",
        "objects": objects,
        "target_count_by_class": {
            class_id: sum(item.get("class_id") == class_id for item in selected)
            for class_id in classes
        },
        "hard_negative_count": len(selected_negatives),
        "negative_only": not selected and bool(selected_negatives),
        "same_color_negative_present": bool(selected_negatives),
        "missing_target_classes": sorted(
            set(classes) - {item["class_id"] for item in selected}
        ),
        "overlap_executed": len(selected) >= 2 and scene_index % 5 == 2,
        "dynamic_motion_plan": dynamic_plan,
        "lighting_executed_by_world": world["lighting_family"],
        "ground_material_executed_by_world": world["material_id"],
        "active_observation_pair_id": f"{world_id}_{scene_index // 2}"
        if observation_phase
        else None,
        "active_observation_phase": observation_phase,
        "vehicle_start_xyz_m": [-8.0, 0.0, 0.18],
        "vehicle_motion_command": {"linear_x_mps": 0.35, "duration_s": 8.0},
    }
    scene["manifest_sha256"] = hashlib.sha256(
        json.dumps(scene, sort_keys=True).encode()
    ).hexdigest()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(scene, indent=2) + "\n", encoding="utf-8")
    return scene


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--world-id", required=True)
    parser.add_argument("--scene-seed", type=int, required=True)
    parser.add_argument("--scene-index", type=int, required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    print(
        json.dumps(
            randomize(
                Path(args.manifest),
                args.world_id,
                args.scene_seed,
                args.scene_index,
                Path(args.output),
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
