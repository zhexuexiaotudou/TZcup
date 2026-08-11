"""G4 scene randomization with a frozen 25%-35% negative-only prior.

The scene plan is deterministic for (world, scene_seed, scene_index) and is
executed by the native Gazebo capture runner.  Offline sensor augmentation is
explicitly separated: this task only emits native plans with
``offline_sensor_augmentation.requested_only: false``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import random

from .g2_scene import set_poses
from .g4_assets import REQUIRED_PAPER_TAXONOMIES


SCENES_PER_WORLD = 25
FRAMES_PER_SCENE = 10
FORMAL_WORLD_SPLITS = {"train": 8, "val": 2, "test": 2}

# Negative-only scene count per world, chosen so every split stays inside
# [25%, 35%] and the cross-split delta stays <= 10pp:
#   train: 7/25 = 28%  val: 8/25 = 32%  test: 7/25 = 28%
NEGATIVE_ONLY_HITS = {"train": 7, "val": 8, "test": 7}
SEALED_FINAL_SPLIT = "G5_SEALED_FINAL"

PAPER_LIKE_TAXONOMIES = frozenset(REQUIRED_PAPER_TAXONOMIES)

DISTANCE_BUCKETS = ((0.5, 2.0), (2.0, 4.0), (4.0, 8.0))
SIZE_BUCKETS = ("small", "medium", "large", "small_area", "medium_area", "large_area")
OCCLUSION_BUCKETS = ("none", "partial", "heavy")
VISIBLE_FRACTION_BUCKETS = ("full", "partial", "low")
PARKING_ORIGIN_X_M = -200.0
PARKING_ORIGIN_Y_M = 200.0
PARKING_Z_M = -5.0
PARKING_SPACING_M = 0.3
TARGET_DISTANCE_LANES_M = (1.80, 2.10, 2.40, 2.80, 3.40)
TARGET_LATERAL_LANES_M = (-0.80, 0.80, -0.60, 0.60, -1.05)


def negative_only_rule(split: str, scene_index: int) -> bool:
    """Deterministic per-world negative-only prior (25%-35% for every split)."""
    hits = 7 if split == SEALED_FINAL_SPLIT else NEGATIVE_ONLY_HITS[split]
    return (scene_index * 7) % SCENES_PER_WORLD < hits


def paper_like_train_rule(scene_index: int) -> bool:
    """Guarantees >= 9 paper-like hard-negative scenes per train world."""
    return (scene_index * 11) % SCENES_PER_WORLD < 9


def _footprint_area_m2(geometry_kind: str, values: list[float]) -> float:
    if geometry_kind == "cylinder":
        return (2.0 * values[0]) * values[1]
    if geometry_kind == "box":
        return values[0] * values[1]
    return (2.0 * values[0]) * (2.0 * values[1])


def _size_bucket(class_id: str, geometry_kind: str, values: list[float]) -> str:
    area = _footprint_area_m2(geometry_kind, values)
    if class_id in {"leaf_pile", "puddle"}:
        if area < 0.15:
            return "small_area"
        if area < 0.30:
            return "medium_area"
        return "large_area"
    if area < 0.012:
        return "small"
    if area < 0.030:
        return "medium"
    return "large"


def _occlusion_bucket(ratio: float) -> str:
    if ratio <= 0.05:
        return "none"
    if ratio <= 0.35:
        return "partial"
    return "heavy"


def _visible_fraction_bucket(fraction: float) -> str:
    if fraction >= 0.90:
        return "full"
    if fraction >= 0.55:
        return "partial"
    return "low"


def _parked_asset_updates(manifest: dict, selected_names: set[str]) -> list[dict]:
    """Return one off-camera reset pose for every asset not used this scene.

    Gazebo worlds are reused for 25 consecutive scenes.  Without this reset,
    objects selected by an earlier scene remain visible and silently corrupt
    later positive and negative-only captures.
    """
    updates = []
    all_items = manifest["assets"] + manifest["negative_assets"]
    for asset_index, item in enumerate(all_items):
        if item["model_name"] in selected_names:
            continue
        updates.append(
            {
                "name": item["model_name"],
                "xyz": [
                    PARKING_ORIGIN_X_M - asset_index * PARKING_SPACING_M,
                    PARKING_ORIGIN_Y_M,
                    PARKING_Z_M,
                ],
                "yaw": 0.0,
            }
        )
    return updates


def randomize(
    manifest_path: Path,
    world_id: str,
    scene_seed: int,
    scene_index: int,
    output: Path,
    *,
    diagnostic_role: str | None = None,
    asset_source_split: str | None = None,
    negative_source_split: str | None = None,
    force_negative_only: bool = False,
    oprv3_coverage_profile: str | None = None,
) -> dict:
    """Build and persist one G4 scene plan (no capture is performed here)."""
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    world = next(item for item in manifest["worlds"] if item["world_id"] == world_id)
    world_split = world["split_eligibility"][0]
    split = diagnostic_role or world_split
    asset_split = asset_source_split or world_split
    negative_split = negative_source_split or asset_split
    rng = random.Random(20260805 + scene_seed * 7919)
    assets = manifest["assets"]
    negatives = manifest["negative_assets"]
    split_assets = [
        item for item in assets if item["split_eligibility"] == [asset_split]
    ]
    split_negatives = [
        item for item in negatives if item["split_eligibility"] == [negative_split]
    ]
    paper_like_negatives = [
        item
        for item in split_negatives
        if item.get("taxonomy") in PAPER_LIKE_TAXONOMIES
    ]
    schedule_split = (
        asset_split if asset_split in NEGATIVE_ONLY_HITS else "train"
    )
    force_negative = force_negative_only or negative_only_rule(
        schedule_split, scene_index
    )
    force_paper_like = negative_split == "train" and paper_like_train_rule(
        scene_index
    )
    selected = []
    if not force_negative:
        for class_id in sorted({item["class_id"] for item in assets}):
            pool = [item for item in split_assets if item["class_id"] == class_id]
            # One variant per class gives each declaration a unique instance.
            # Variant diversity is obtained across the 25 deterministic scenes.
            selected.extend(rng.sample(pool, min(1, len(pool))))
    negative_count = rng.randint(2, 3)
    selected_negatives = rng.sample(
        split_negatives, min(negative_count, len(split_negatives))
    )
    if force_negative and not selected_negatives:
        selected_negatives = [split_negatives[0]]
    if force_paper_like and paper_like_negatives:
        paper_pick = rng.choice(paper_like_negatives)
        if paper_pick not in selected_negatives:
            selected_negatives.append(paper_pick)
    if force_negative:
        selected = []

    selected_all = selected + selected_negatives
    selected_names = {item["model_name"] for item in selected_all}
    updates = [
        {"name": "sanitation_vehicle", "xyz": [-8.0, 0.0, 0.18], "yaw": 0.0},
        *_parked_asset_updates(manifest, selected_names),
    ]
    objects = []
    for index, item in enumerate(selected_all):
        if index < len(selected):
            # AUTO-05R uses the V5-derived downward primary perception pose. The
            # vehicle advances about 2.25 m over ten frames, so targets are
            # staggered along the path and become visible at different times.
            # The nearest lane starts at 1.8 m so even low leaf geometry crosses
            # the downward camera footprint in at least two sampled frames.
            # Lateral separation avoids target-on-target masking and keeps all
            # physical target collision shapes outside the vehicle sweep.
            lane = index % len(TARGET_DISTANCE_LANES_M)
            distance = TARGET_DISTANCE_LANES_M[lane] + rng.uniform(-0.04, 0.04)
            lateral = TARGET_LATERAL_LANES_M[lane]
        else:
            # Hard negatives remain visible around the target group. A frozen
            # subset spans the close bucket so the metric-scale audit retains
            # 0.5-2/2-4/4-8 m coverage without putting positive labels behind
            # the moving camera.
            negative_index = index - len(selected)
            if (scene_index + negative_index) % 7 == 0:
                distance = rng.uniform(0.7, 1.8)
            elif (scene_index + negative_index) % 5 == 0:
                distance = rng.uniform(4.2, 6.5)
            else:
                distance = rng.uniform(2.8, 4.8)
            sign = -1.0 if negative_index % 2 else 1.0
            lateral = sign * 1.50
        geometry = item.get("physical_geometry_values_m", [0.16, 0.12, 0.08])
        geometry_kind = item.get("geometry_kind", "box")
        distance_bucket = next(
            (bucket for bucket in DISTANCE_BUCKETS if bucket[0] <= distance < bucket[1]),
            (4.0, 8.0),
        )
        z = max(float(geometry[-1]) / 2.0, 0.008)
        pose = {
            "name": item["model_name"],
            "xyz": [-8.0 + distance, lateral, z],
            "yaw": rng.uniform(-math.pi, math.pi),
        }
        updates.append(pose)
        overlap = False
        occlusion_ratio = 0.0
        truncation = lateral > 1.9 or abs(lateral) < 0.28
        estimated_visible_fraction = max(
            0.35,
            1.0 - occlusion_ratio - (0.22 if truncation else 0.0),
        )
        if item.get("keepout_or_unreachable", False):
            estimated_visible_fraction = rng.uniform(0.55, 0.85)
        class_id = item.get("class_id", "background")
        size_bucket = _size_bucket(class_id, geometry_kind, geometry)
        objects.append(
            {
                "model_name": item["model_name"],
                "asset_id": item["model_name"],
                "class_id": class_id,
                "taxonomy": item.get("taxonomy"),
                "semantic_label": item["semantic_label"],
                "split_eligibility": item.get("split_eligibility", [split]),
                "xyz_m": pose["xyz"],
                "yaw_rad": pose["yaw"],
                "distance_m": distance,
                "distance_bucket_m": list(distance_bucket),
                "size_bucket": size_bucket,
                "occlusion_ratio": round(occlusion_ratio, 4),
                "occlusion_bucket": _occlusion_bucket(occlusion_ratio),
                "estimated_visible_fraction": round(
                    estimated_visible_fraction, 4
                ),
                "visible_fraction_bucket": _visible_fraction_bucket(
                    estimated_visible_fraction
                ),
                "truncation": truncation,
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
        if abs(moving["xyz_m"][1]) < 0.75:
            moving["xyz_m"][1] = 0.75 if moving["xyz_m"][1] >= 0 else -0.75
        dynamic_plan = {
            "model_name": moving["model_name"],
            "start_xyz_m": moving["xyz_m"],
            "delta_per_frame_m": [0.08, 0.0, 0.0],
            "executed_by_capture": True,
        }
    vehicle_start_yaw_rad = 0.0
    motion_profile = None
    coverage_requirements = {
        "behind_vehicle_fov_entry": False,
        "turning": False,
        "occlusion": False,
        "reflection": False,
        "dynamic_removal": False,
    }
    dynamic_removal_plan = None
    overlap_executed = False
    if oprv3_coverage_profile == "turn_entry":
        if not selected:
            raise ValueError("turn_entry coverage requires positive targets")
        vehicle_start_yaw_rad = math.pi / 2.0
        updates[0]["yaw"] = vehicle_start_yaw_rad
        motion_profile = {
            "name": "turn_then_approach",
            "frame_counted": True,
            "phases": [
                {
                    "name": "turn_into_target_fov",
                    # The real 0.50 rad/s DiffDrive turn advances about
                    # 0.10 rad per admitted 0.03-rad sample.  Sixteen samples
                    # therefore produce the intended quarter turn instead of
                    # rotating past the targets before the approach phase.
                    "frame_count": 16,
                    "linear_x_mps": 0.0,
                    "angular_z_rad_s": -0.50,
                },
                {
                    "name": "straight_approach",
                    "frame_count": 4096,
                    "linear_x_mps": 0.65,
                    "angular_z_rad_s": 0.0,
                },
            ],
        }
        coverage_requirements["behind_vehicle_fov_entry"] = True
        coverage_requirements["turning"] = True
    elif oprv3_coverage_profile == "occlusion":
        positives = [item for item in objects if item["class_id"] != "background"]
        if len(positives) < 3:
            raise ValueError("occlusion coverage requires at least three targets")
        front = next(
            (item for item in positives if item["class_id"] == "metal_can"),
            positives[0],
        )
        behind = next(
            (item for item in positives if item["class_id"] == "paper_litter"),
            positives[1],
        )
        for item, distance in ((front, 2.10), (behind, 2.42)):
            item["xyz_m"][0] = -8.0 + distance
            item["xyz_m"][1] = 0.62
            item["distance_m"] = distance
            item["distance_bucket_m"] = [2.0, 4.0]
            pose = next(update for update in updates if update["name"] == item["model_name"])
            pose["xyz"] = list(item["xyz_m"])
        behind["occlusion_ratio"] = 0.40
        behind["occlusion_bucket"] = _occlusion_bucket(0.40)
        behind["estimated_visible_fraction"] = 0.60
        behind["visible_fraction_bucket"] = _visible_fraction_bucket(0.60)
        behind["occluded_by_model_name"] = front["model_name"]
        overlap_executed = True
        coverage_requirements["occlusion"] = True
    elif oprv3_coverage_profile == "reflection":
        if "wet" not in world["world_id"].lower() and "wet" not in world["material_id"].lower():
            raise ValueError("reflection coverage requires a wet world/material")
        # The wet courtyard's straight lane ends after roughly eight metres.
        # Preserve the full target approach, turn only long enough to leave
        # the lane, then continue on the new heading.  A sustained circle can
        # contact courtyard geometry near y=0.8; the short frame-counted turn
        # avoids both that collision and the straight lane's x=0 boundary.
        motion_profile = {
            "name": "reflection_approach_turn_continue",
            "frame_counted": True,
            "phases": [
                {
                    "name": "reflection_straight_approach",
                    "frame_count": 45,
                    "linear_x_mps": 0.35,
                    "angular_z_rad_s": 0.0,
                },
                {
                    "name": "reflection_lane_exit_turn",
                    "frame_count": 10,
                    "linear_x_mps": 0.20,
                    "angular_z_rad_s": 0.25,
                },
                {
                    "name": "reflection_diagonal_continue",
                    "frame_count": 4096,
                    "linear_x_mps": 0.35,
                    "angular_z_rad_s": 0.0,
                },
            ],
        }
        coverage_requirements["reflection"] = True
    elif oprv3_coverage_profile == "dynamic_removal":
        removable = next(
            (
                item
                for item in objects
                if item["class_id"]
                in {"plastic_bottle", "metal_can", "paper_litter"}
            ),
            None,
        )
        if removable is None:
            raise ValueError(
                "dynamic_removal coverage requires a discrete positive target"
            )
        removable["xyz_m"] = [-5.60, 0.60, removable["xyz_m"][2]]
        removable["distance_m"] = 2.40
        removable["distance_bucket_m"] = [2.0, 4.0]
        pose = next(
            update
            for update in updates
            if update["name"] == removable["model_name"]
        )
        pose["xyz"] = list(removable["xyz_m"])
        dynamic_plan = None
        dynamic_removal_plan = {
            "model_name": removable["model_name"],
            "class_id": removable["class_id"],
            "initial_xyz_m": list(removable["xyz_m"]),
            "parked_xyz_m": [PARKING_ORIGIN_X_M, PARKING_ORIGIN_Y_M, PARKING_Z_M],
            "trigger_fraction": 0.55,
            "executed_by_capture": True,
        }
        coverage_requirements["dynamic_removal"] = True
    elif oprv3_coverage_profile is not None:
        raise ValueError(f"unsupported OPRV3 coverage profile: {oprv3_coverage_profile}")
    if os.environ.get("G4_SCENE_PLAN_ONLY") != "1":
        set_poses(world_id, updates)
    classes = sorted({item["class_id"] for item in assets})
    distance_bucket_counts = {
        f"{low}_{high}": sum(
            item["distance_bucket_m"] == [low, high] for item in objects
        )
        for low, high in DISTANCE_BUCKETS
    }
    size_bucket_counts = {
        bucket: sum(item["size_bucket"] == bucket for item in objects)
        for bucket in SIZE_BUCKETS
    }
    occlusion_bucket_counts = {
        bucket: sum(item["occlusion_bucket"] == bucket for item in objects)
        for bucket in OCCLUSION_BUCKETS
    }
    visible_fraction_bucket_counts = {
        bucket: sum(item["visible_fraction_bucket"] == bucket for item in objects)
        for bucket in VISIBLE_FRACTION_BUCKETS
    }
    scene = {
        "schema_version": 2,
        "scene_seed": scene_seed,
        "scene_index_in_world": scene_index,
        "world_id": world_id,
        "split": split,
        "source_world_split": world_split,
        "world_sha256": world["sha256"],
        "trajectory_id": f"{world_id}_trajectory_{scene_seed:04d}",
        "objects": objects,
        "target_count_by_class": {
            class_id: sum(item.get("class_id") == class_id for item in selected)
            for class_id in classes
        },
        "hard_negative_count": len(selected_negatives),
        "paper_like_hard_negative_count": sum(
            item.get("taxonomy") in PAPER_LIKE_TAXONOMIES
            for item in selected_negatives
        ),
        "negative_only": not selected and bool(selected_negatives),
        "missing_target_classes": sorted(
            set(classes) - {item["class_id"] for item in selected}
        ),
        "overlap_executed": overlap_executed,
        "dynamic_motion_plan": dynamic_plan,
        "dynamic_removal_plan": dynamic_removal_plan,
        "native_gazebo_applied": True,
        "offline_sensor_augmentation": {
            "requested_only": False,
            "applied": False,
            "plan": None,
        },
        "factorized_diagnostic": {
            "enabled": diagnostic_role is not None,
            "role": diagnostic_role,
            "asset_source_split": asset_split,
            "negative_source_split": negative_split,
            "force_negative_only": bool(force_negative_only),
            "single_factor_capture": diagnostic_role is not None,
        },
        "lighting_executed_by_world": world["lighting_family"],
        "ground_material_executed_by_world": world["material_id"],
        "distance_bucket_counts": distance_bucket_counts,
        "size_bucket_counts": size_bucket_counts,
        "occlusion_bucket_counts": occlusion_bucket_counts,
        "visible_fraction_bucket_counts": visible_fraction_bucket_counts,
        "vehicle_start_xyz_m": [-8.0, 0.0, 0.18],
        "vehicle_start_yaw_rad": vehicle_start_yaw_rad,
        "vehicle_motion_command": {"linear_x_mps": 0.35, "duration_s": 8.0},
        "oprv3_coverage_profile": oprv3_coverage_profile,
        "oprv3_coverage_requirements": coverage_requirements,
        "oprv3_motion_profile": motion_profile,
        "pose_reset_contract": {
            "all_world_assets_accounted_for": len(updates)
            == 1 + len(manifest["assets"]) + len(manifest["negative_assets"]),
            "selected_asset_count": len(selected_names),
            "parked_asset_count": len(updates) - 1 - len(selected_names),
            "asset_pose_count": len(updates) - 1,
            "duplicate_asset_pose_names": len(updates) - 1
            - len({item["name"] for item in updates[1:]}),
        },
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
    parser.add_argument("--diagnostic-role", choices=("D1", "D2", "D3", "D4", "D5"))
    parser.add_argument("--asset-source-split", choices=("train", "val", "test"))
    parser.add_argument("--negative-source-split", choices=("train", "val", "test"))
    parser.add_argument("--force-negative-only", action="store_true")
    parser.add_argument(
        "--oprv3-coverage-profile",
        choices=("turn_entry", "occlusion", "reflection", "dynamic_removal"),
    )
    args = parser.parse_args()
    print(
        json.dumps(
            randomize(
                Path(args.manifest),
                args.world_id,
                args.scene_seed,
                args.scene_index,
                Path(args.output),
                diagnostic_role=args.diagnostic_role,
                asset_source_split=args.asset_source_split,
                negative_source_split=args.negative_source_split,
                force_negative_only=args.force_negative_only,
                oprv3_coverage_profile=args.oprv3_coverage_profile,
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
