from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import random
import subprocess


def _set_poses(poses: list[tuple[str, float, float, float, float]]) -> None:
    request = " ".join(
        "pose { "
        f'name: "{model_name}" position {{ x: {x:.6f} y: {y:.6f} z: {z:.6f} }} '
        f'orientation {{ z: {math.sin(yaw / 2):.8f} w: {math.cos(yaw / 2):.8f} }} }}'
        for model_name, x, y, z, yaw in poses
    )
    command = [
        "gz", "service", "-s", "/world/stage5br_g1/set_pose_vector",
        "--reqtype", "gz.msgs.Pose_V", "--reptype", "gz.msgs.Boolean",
        "--timeout", "3000", "--req", request,
    ]
    result = subprocess.run(command, check=False, capture_output=True, text=True)
    if result.returncode != 0 or "data: true" not in result.stdout:
        raise RuntimeError(f"set_pose_vector failed: {result.stdout} {result.stderr}")


def _set_light(rng: random.Random) -> dict:
    diffuse = [rng.uniform(0.55, 1.0) for _ in range(3)]
    direction = [rng.uniform(-0.75, -0.2), rng.uniform(-0.35, 0.35), rng.uniform(-0.95, -0.65)]
    request = (
        'name: "sun" type: DIRECTIONAL '
        f'diffuse {{ r: {diffuse[0]:.6f} g: {diffuse[1]:.6f} b: {diffuse[2]:.6f} a: 1 }} '
        'specular { r: 0.15 g: 0.15 b: 0.15 a: 1 } '
        f'direction {{ x: {direction[0]:.6f} y: {direction[1]:.6f} z: {direction[2]:.6f} }} '
        'cast_shadows: true'
    )
    command = [
        "gz", "service", "-s", "/world/stage5br_g1/light_config",
        "--reqtype", "gz.msgs.Light", "--reptype", "gz.msgs.Boolean",
        "--timeout", "3000", "--req", request,
    ]
    result = subprocess.run(command, check=False, capture_output=True, text=True)
    if result.returncode != 0 or "data: true" not in result.stdout:
        raise RuntimeError(f"light_config failed: {result.stdout} {result.stderr}")
    return {"profile": "g1_random_directional_v1", "diffuse_rgb": diffuse, "direction_xyz": direction, "randomized": True}


def _split(seed: int) -> tuple[str, list[int]]:
    slot = seed % 10
    if slot <= 6:
        return "train", [0, 1, 2, 3]
    if slot == 7:
        return "val", [4]
    return "test", [5]


def randomize_scene(
    world_manifest_path: str | Path,
    scene_seed: int,
    output_path: str | Path,
    state_path: str | Path,
) -> dict:
    world_manifest_path = Path(world_manifest_path)
    output_path = Path(output_path)
    state_path = Path(state_path)
    manifest = json.loads(world_manifest_path.read_text(encoding="utf-8"))
    rng = random.Random(20260719 + int(scene_seed) * 1009)
    split, variant_indices = _split(scene_seed)
    targets = [item for item in manifest["models"] if item["class_id"] != "background"]
    negatives = [item for item in manifest["models"] if item["class_id"] == "background"]
    selected = []
    for class_id in manifest["class_order"][1:]:
        candidates = [item for item in targets if item["class_id"] == class_id and item["variant_index"] in variant_indices]
        selected.append(rng.choice(candidates))
    selected_negatives = rng.sample(negatives, 4)
    selected_names = {item["model_name"] for item in selected + selected_negatives}
    previous = set()
    if state_path.is_file():
        previous = set(json.loads(state_path.read_text(encoding="utf-8")).get("visible_models", []))
    pose_updates = []
    for index, name in enumerate(sorted(previous - selected_names)):
        pose_updates.append((name, 30.0 + index * 0.25, 28.0, 0.10, 0.0))
    positions = []
    slots = [(-1.05, -0.48), (-0.55, 0.20), (0.0, -0.30), (0.55, 0.38), (1.05, -0.12),
             (-1.08, 0.52), (-0.32, 0.57), (0.40, -0.58), (1.08, 0.55)]
    rng.shuffle(slots)
    for index, item in enumerate(selected + selected_negatives):
        base_x, base_y = slots[index]
        x = base_x + rng.uniform(-0.12, 0.12)
        y = base_y + rng.uniform(-0.10, 0.10)
        initial = [float(value) for value in item["initial_pose"].split()]
        z = max(initial[2], 0.008)
        yaw = rng.uniform(-math.pi, math.pi)
        pose_updates.append((item["model_name"], x, y, z, yaw))
        positions.append({
            "model_name": item["model_name"], "class_id": item["class_id"],
            "semantic_label": item["semantic_label"], "xyz_m": [x, y, z], "yaw_rad": yaw,
            "variant_index": item.get("variant_index"), "texture_id": item.get("texture_id"),
            "negative_id": item.get("negative_id"),
        })
    _set_poses(pose_updates)
    lighting = _set_light(rng)
    state_path.parent.mkdir(parents=True, exist_ok=True)
    state_path.write_text(json.dumps({"visible_models": sorted(selected_names)}, indent=2) + "\n", encoding="utf-8")
    scene = {
        "schema_version": 1, "scene_seed": int(scene_seed), "split": split,
        "world_sha256": manifest["world_sha256"], "registry_sha256": manifest["registry_sha256"],
        "asset_ids": sorted(item["model_name"] for item in selected),
        "texture_ids": sorted(item["texture_id"] for item in selected),
        "negative_ids": sorted(item["negative_id"] for item in selected_negatives),
        "objects": positions, "camera_pose_id": "g1_topdown_v1",
        "camera_contract": manifest["camera_contract"],
        "lighting": lighting,
        "ground_material": "g1_asphalt_fixed_v1", "annotation_source": manifest["annotation_source"],
        "scene_manifest_sha256": None,
    }
    scene["scene_manifest_sha256"] = hashlib.sha256(
        json.dumps({**scene, "scene_manifest_sha256": None}, sort_keys=True).encode("utf-8")
    ).hexdigest()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(scene, indent=2) + "\n", encoding="utf-8")
    return scene


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--world-manifest", required=True)
    parser.add_argument("--scene-seed", required=True, type=int)
    parser.add_argument("--output", required=True)
    parser.add_argument("--state", required=True)
    args = parser.parse_args()
    report = randomize_scene(args.world_manifest, args.scene_seed, args.output, args.state)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
