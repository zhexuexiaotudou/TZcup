from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import subprocess


def set_poses(world_id: str, poses: list[dict]) -> None:
    request = " ".join(
        "pose { "
        f'name: "{item["name"]}" position {{ x: {item["xyz"][0]:.6f} y: {item["xyz"][1]:.6f} z: {item["xyz"][2]:.6f} }} '
        f'orientation {{ z: {math.sin(item["yaw"] / 2):.8f} w: {math.cos(item["yaw"] / 2):.8f} }} }}'
        for item in poses
    )
    result = subprocess.run([
        "gz", "service", "-s", f"/world/{world_id}/set_pose_vector",
        "--reqtype", "gz.msgs.Pose_V", "--reptype", "gz.msgs.Boolean",
        "--timeout", "5000", "--req", request,
    ], capture_output=True, text=True)
    if result.returncode or "data: true" not in result.stdout:
        raise RuntimeError(f"set_pose_vector failed: {result.stdout} {result.stderr}")


def randomize(manifest_path: Path, world_id: str, scene_seed: int, output: Path) -> dict:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    world = next(item for item in manifest["worlds"] if item["world_id"] == world_id)
    split = world["split_eligibility"][0]
    allowed_variants = {"train": {0, 1, 2}, "val": {3}, "test": {4, 5}}[split]
    rng = random.Random(20260720 + scene_seed * 7919)
    all_assets = manifest["assets"]
    all_negatives = manifest["negative_assets"]
    negative_partitions = {
        "train": all_negatives[:8], "val": all_negatives[8:10], "test": all_negatives[10:12],
    }
    selected = []
    for class_id in sorted({item["class_id"] for item in all_assets}):
        candidates = [item for item in all_assets if item["class_id"] == class_id and item["variant_index"] in allowed_variants]
        count = rng.randrange(4)
        selected.extend(rng.sample(candidates, min(count, len(candidates))))
    negative_count = rng.randrange(9)
    candidates_negative = negative_partitions[split]
    selected_negatives = rng.sample(candidates_negative, min(negative_count, len(candidates_negative)))
    if scene_seed % 19 == 0:
        selected = []
        if not selected_negatives:
            selected_negatives = [rng.choice(candidates_negative)]
    updates = [{"name": "sanitation_vehicle", "xyz": [-8.0, 0.0, 0.18], "yaw": 0.0}]
    for index, item in enumerate(all_assets + all_negatives):
        updates.append({"name": item["model_name"], "xyz": [-200.0 - index * 0.25, 200.0, -5.0], "yaw": 0.0})
    objects = []
    for index, item in enumerate(selected + selected_negatives):
        distance = rng.uniform(0.5, 8.0)
        lateral = rng.uniform(-2.1, 2.1)
        # Preserve the 0.5 m near bucket without placing collision geometry in
        # the vehicle's straight capture corridor (half-width 0.36 m).
        if distance < 3.5 and abs(lateral) < 0.65:
            lateral = 0.75 if lateral >= 0 else -0.75
        geometry = item.get("physical_geometry_values_m", [0.16, 0.12, 0.08])
        z = max(float(geometry[-1]) / 2.0, 0.008)
        pose = {"name": item["model_name"], "xyz": [-8.0 + distance, lateral, z], "yaw": rng.uniform(-math.pi, math.pi)}
        updates.append(pose)
        objects.append({
            "model_name": item["model_name"], "class_id": item.get("class_id", "background"),
            "semantic_label": item["semantic_label"], "xyz_m": pose["xyz"], "yaw_rad": pose["yaw"],
            "distance_bucket_m": [0.5, 2.0] if distance < 2 else [2.0, 4.0] if distance < 4 else [4.0, 8.0],
            "horizontal_region": "left" if lateral > 0.7 else "right" if lateral < -0.7 else "middle",
            "keepout_or_unreachable": abs(lateral) > 1.6,
            "physical_geometry_values_m": geometry,
        })
    set_poses(world_id, updates)
    scene = {
        "schema_version": 1, "scene_seed": scene_seed, "world_id": world_id, "split": split,
        "world_sha256": world["sha256"], "trajectory_id": f"{world_id}_scene_{scene_seed:04d}",
        "objects": objects, "target_count_by_class": {class_id: sum(item.get("class_id") == class_id for item in selected) for class_id in sorted({item["class_id"] for item in all_assets})},
        "hard_negative_count": len(selected_negatives), "negative_only": not selected and bool(selected_negatives),
        "missing_target_classes": sorted({item["class_id"] for item in all_assets} - {item["class_id"] for item in selected}),
        "overlap_requested": len(selected) >= 2 and scene_seed % 3 == 0,
        "dynamic_obstacle_requested": scene_seed % 4 == 0,
        "appearance_randomization": {"exposure": rng.uniform(0.7, 1.3), "white_balance_k": rng.randint(3500, 7500), "noise_sigma": rng.uniform(0, 0.025), "motion_blur_px": rng.choice([0, 3, 5]), "shadow_profile": rng.choice(["soft", "hard", "mixed"])},
        "vehicle_start_xyz_m": [-8.0, 0.0, 0.18], "vehicle_motion_command": {"linear_x_mps": 0.35, "duration_s": 8.0},
    }
    scene["manifest_sha256"] = hashlib.sha256(json.dumps(scene, sort_keys=True).encode()).hexdigest()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(scene, indent=2) + "\n", encoding="utf-8")
    return scene


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--world-id", required=True)
    parser.add_argument("--scene-seed", type=int, required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    print(json.dumps(randomize(Path(args.manifest), args.world_id, args.scene_seed, Path(args.output)), indent=2))


if __name__ == "__main__":
    main()
