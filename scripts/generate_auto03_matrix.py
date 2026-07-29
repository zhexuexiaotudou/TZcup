from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import random


CLASS_ORDER = (
    "plastic_bottle",
    "metal_can",
    "paper_litter",
    "leaf_pile",
    "puddle",
)
REACHABLE_TARGETS = (
    (2.40, -1.20),
    (2.55, -1.55),
    (2.70, -1.20),
    (2.85, -1.60),
    (3.00, -1.20),
    (3.15, -1.60),
    (3.30, -1.20),
    (3.45, -1.60),
)
KEEPOUT_TARGETS = (
    (2.35, 1.40),
    (2.75, 1.80),
    (3.20, 2.10),
    (3.60, 2.45),
)


def target_short_size(asset: dict) -> float:
    values = [float(value) for value in asset["physical_geometry_values_m"]]
    kind = asset["geometry_kind"]
    if kind == "cylinder":
        return 2.0 * values[0]
    if kind in {"box", "sphere"}:
        return min(values[:2]) * (2.0 if kind == "sphere" else 1.0)
    raise ValueError(f"unsupported geometry kind: {kind}")


def scene_identity(index: int, worlds: list[dict]) -> tuple[dict, str]:
    slot = index % 60
    world = worlds[slot % len(worlds)]
    local_scene = slot // len(worlds)
    return world, f"{world['world_id']}_scene_{local_scene:02d}"


def choose_asset(manifest: dict, world: dict, class_id: str, index: int) -> dict:
    split = world["split_eligibility"][0]
    allowed = {"train": {0, 1, 2}, "val": {3}, "test": {4, 5}}[split]
    assets = [
        item for item in manifest["assets"]
        if item["class_id"] == class_id and item["variant_index"] in allowed
    ]
    return assets[index % len(assets)]


def generate(manifest: dict) -> dict:
    worlds = manifest["worlds"]
    if len(worlds) < 6:
        raise ValueError("AUTO-03 requires at least six G2 worlds")
    rng = random.Random(20260729)
    trials = []

    for index in range(200):
        world, scene_id = scene_identity(index, worlds)
        class_id = CLASS_ORDER[index % len(CLASS_ORDER)]
        asset = choose_asset(manifest, world, class_id, index)
        unreachable = index < 30
        target_xy = (
            KEEPOUT_TARGETS[index % len(KEEPOUT_TARGETS)]
            if unreachable
            else REACHABLE_TARGETS[(index - 30) % len(REACHABLE_TARGETS)]
        )
        noise_x = rng.gauss(0.0, 0.008)
        noise_y = rng.gauss(0.0, 0.008)
        trials.append({
            "candidate_id": f"auto03_target_{index:03d}",
            "world_id": world["world_id"],
            "scene_id": scene_id,
            "case_type": "unreachable_keepout" if unreachable else "reachable",
            "class_id": class_id,
            "semantic_label": int(asset["semantic_label"]),
            "active_model_name": asset["model_name"],
            "active_model_world_xyz_m": [
                target_xy[0] - 8.0,
                target_xy[1],
                max(float(asset["physical_geometry_values_m"][-1]) / 2.0, 0.008),
            ],
            "active_model_yaw_rad": rng.uniform(-math.pi, math.pi),
            "oracle_candidate": {
                "candidate_id": f"auto03_target_{index:03d}",
                "x_m": target_xy[0] + noise_x,
                "y_m": target_xy[1] + noise_y,
                "target_size_m": target_short_size(asset),
                "class_id": class_id,
                "covariance_trace": 2.0 * 0.008 * 0.008,
                "timestamp_s": 0.0,
            },
        })

    negatives = manifest["negative_assets"]
    for offset in range(30):
        index = 200 + offset
        world, scene_id = scene_identity(index, worlds)
        class_id = CLASS_ORDER[offset % len(CLASS_ORDER)]
        negative = negatives[offset % len(negatives)]
        target_xy = REACHABLE_TARGETS[(offset + 3) % len(REACHABLE_TARGETS)]
        trials.append({
            "candidate_id": f"auto03_false_{offset:03d}",
            "world_id": world["world_id"],
            "scene_id": scene_id,
            "case_type": "false_candidate",
            "class_id": class_id,
            "semantic_label": CLASS_ORDER.index(class_id) + 1,
            "active_model_name": negative["model_name"],
            "active_model_world_xyz_m": [target_xy[0] - 8.0, target_xy[1], 0.04],
            "active_model_yaw_rad": rng.uniform(-math.pi, math.pi),
            "oracle_candidate": {
                "candidate_id": f"auto03_false_{offset:03d}",
                "x_m": target_xy[0] + rng.gauss(0.0, 0.008),
                "y_m": target_xy[1] + rng.gauss(0.0, 0.008),
                "target_size_m": 0.12,
                "class_id": class_id,
                "covariance_trace": 2.0 * 0.008 * 0.008,
                "timestamp_s": 0.0,
            },
        })

    for offset in range(20):
        index = 230 + offset
        world, scene_id = scene_identity(index, worlds)
        class_id = CLASS_ORDER[offset % len(CLASS_ORDER)]
        target_xy = REACHABLE_TARGETS[offset % len(REACHABLE_TARGETS)]
        trials.append({
            "candidate_id": f"auto03_stale_{offset:03d}",
            "world_id": world["world_id"],
            "scene_id": scene_id,
            "case_type": "stale_dropout",
            "class_id": class_id,
            "semantic_label": CLASS_ORDER.index(class_id) + 1,
            "active_model_name": None,
            "active_model_world_xyz_m": None,
            "active_model_yaw_rad": 0.0,
            "oracle_candidate": {
                "candidate_id": f"auto03_stale_{offset:03d}",
                "x_m": target_xy[0],
                "y_m": target_xy[1],
                "target_size_m": 0.12,
                "class_id": class_id,
                "covariance_trace": 2.0 * 0.008 * 0.008,
                "timestamp_s": -5.0,
            },
        })

    payload = {
        "schema_version": 1,
        "stage": "AUTO-03",
        "attempt_id": "AUTO-03-ORACLE-V1",
        "world_to_map_translation_m": [8.0, 0.0],
        "oracle_policy": {
            "published_fields": [
                "candidate_id", "x_m", "y_m", "target_size_m",
                "class_id", "covariance_trace", "timestamp_s",
            ],
            "observation_pose_forbidden": True,
            "set_robot_pose_forbidden": True,
            "path_available_shortcut_forbidden": True,
            "gt_planner_nav_control_subscription_forbidden": True,
        },
        "all_model_names": [
            item["model_name"]
            for item in manifest["assets"] + manifest["negative_assets"]
        ],
        "trials": trials,
    }
    payload["matrix_sha256"] = hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    return payload


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    manifest = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
    payload = generate(manifest)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "output": str(output),
        "trial_count": len(payload["trials"]),
        "world_count": len({item["world_id"] for item in payload["trials"]}),
        "scene_count": len({(item["world_id"], item["scene_id"]) for item in payload["trials"]}),
        "matrix_sha256": payload["matrix_sha256"],
    }, indent=2))


if __name__ == "__main__":
    main()
