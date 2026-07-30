#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np


CLASS_NAMES = {
    1: "plastic_bottle",
    2: "metal_can",
    3: "paper_litter",
    4: "leaf_pile",
    5: "puddle",
}


def phash(path: Path) -> str:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    small = cv2.resize(image, (32, 32), interpolation=cv2.INTER_AREA).astype(
        np.float32
    )
    coefficients = cv2.dct(small)[:8, :8]
    bits = coefficients > np.median(coefficients[1:])
    return f"{int(''.join('1' if bit else '0' for bit in bits.ravel()), 2):016x}"


def finite_pose(path: Path) -> bool:
    payload = json.loads(path.read_text(encoding="utf-8"))
    values = payload.get("world_to_base_xy", []) + payload.get(
        "base_to_camera_xyz_m", []
    )
    return len(values) == 5 and all(math.isfinite(float(value)) for value in values)


def intersections(parts: dict[str, set[str]]) -> list[str]:
    return sorted(
        (parts["train"] & parts["val"])
        | (parts["train"] & parts["test"])
        | (parts["val"] & parts["test"])
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    root, output = Path(args.data_root), Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    world_manifest = json.loads(
        (root / "worlds" / "g3_world_manifest.json").read_text(encoding="utf-8")
    )
    expected_worlds = {
        item["world_id"]: item["split_eligibility"][0]
        for item in world_manifest["worlds"]
    }
    scenes = sorted((root / "scenes").glob("scene_*"))
    errors: list[dict] = []
    frames: list[dict] = []
    instances: list[dict] = []
    split_assets = defaultdict(set)
    split_negatives = defaultdict(set)
    split_trajectories = defaultdict(set)
    split_worlds = defaultdict(set)
    exact_seen: dict[str, dict] = {}
    phash_seen: dict[str, dict] = {}
    exact_cross, phash_cross = [], []
    semantic_error_pixels = 0
    instance_pixels = 0
    negative_frames_by_world = Counter()
    scenario_flags = Counter()
    class_presence = Counter()
    scene_counts_by_world = Counter()
    split_scene_counts = Counter()
    dynamic_scene_count = 0
    dynamic_executed_count = 0
    tf_valid_count = 0
    exact_sync_count = 0

    for scene_dir in scenes:
        manifest_path = scene_dir / "scene_manifest.json"
        capture_path = scene_dir / "capture_report.json"
        if not manifest_path.is_file() or not capture_path.is_file():
            errors.append(
                {"scene": scene_dir.name, "reason": "manifest_or_capture_missing"}
            )
            continue
        scene = json.loads(manifest_path.read_text(encoding="utf-8"))
        capture = json.loads(capture_path.read_text(encoding="utf-8"))
        world_id, split = scene["world_id"], scene["split"]
        scene_counts_by_world[world_id] += 1
        split_scene_counts[split] += 1
        split_worlds[split].add(world_id)
        split_trajectories[split].add(scene["trajectory_id"])
        for item in scene["objects"]:
            if item["semantic_label"]:
                split_assets[split].add(item["model_name"])
            else:
                split_negatives[split].add(item["model_name"])
        if expected_worlds.get(world_id) != split:
            errors.append(
                {"scene": scene_dir.name, "reason": "world_split_mismatch"}
            )
        if not capture.get("capture_pass") or capture.get("captured_frames") != 10:
            errors.append(
                {"scene": scene_dir.name, "reason": "capture_gate_failed"}
            )
        if scene["negative_only"]:
            negative_frames_by_world[world_id] += len(capture.get("records", []))
            scenario_flags["negative_only"] += 1
        if scene["missing_target_classes"]:
            scenario_flags["missing_class"] += 1
        if sum(scene["target_count_by_class"].values()) >= 2:
            scenario_flags["multi_instance"] += 1
        if scene.get("overlap_executed"):
            scenario_flags["overlap"] += 1
        if scene.get("same_color_negative_present"):
            scenario_flags["same_color_negative"] += 1
        if scene.get("active_observation_phase") == "before":
            scenario_flags["active_before"] += 1
        if scene.get("active_observation_phase") == "after":
            scenario_flags["active_after"] += 1
        if scene.get("dynamic_motion_plan"):
            dynamic_scene_count += 1
            dynamic_executed_count += int(capture.get("dynamic_motion_executed", False))
        counts = scene["target_count_by_class"].values()
        if any(count < 0 or count > 3 for count in counts):
            errors.append(
                {"scene": scene_dir.name, "reason": "class_instance_count_out_of_range"}
            )
        positions = [
            tuple(record["vehicle_xy_m"]) for record in capture.get("records", [])
        ]
        adjacent = [
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(positions, positions[1:])
        ]
        if len(adjacent) != 9 or any(distance < 0.25 for distance in adjacent):
            errors.append(
                {
                    "scene": scene_dir.name,
                    "reason": "adjacent_motion_below_0.25m",
                    "values": adjacent,
                }
            )
        for record in capture.get("records", []):
            rgb_path = scene_dir / record["paths"]["rgb"]
            semantic_path = scene_dir / record["paths"]["semantic"]
            instance_path = scene_dir / record["paths"]["instance"]
            tf_path = scene_dir / record["paths"]["tf"]
            semantic = np.load(semantic_path, allow_pickle=False)
            instance = np.load(instance_path, allow_pickle=False)
            if semantic.shape != (480, 640) or instance.shape != semantic.shape:
                errors.append(
                    {
                        "scene": scene_dir.name,
                        "frame": record["frame_index"],
                        "reason": "native_shape_mismatch",
                    }
                )
            labels = {int(value) for value in np.unique(semantic)}
            if not labels.issubset(set(range(6))):
                errors.append(
                    {
                        "scene": scene_dir.name,
                        "frame": record["frame_index"],
                        "reason": "unknown_semantic_id",
                    }
                )
            exact_sync_count += int(record.get("exact_four_sensor_timestamp", False))
            tf_valid = tf_path.is_file() and finite_pose(tf_path)
            tf_valid_count += int(tf_valid)
            for label in labels - {0}:
                class_presence[CLASS_NAMES[label]] += 1
            for instance_id in (
                int(value) for value in np.unique(instance) if int(value) != 0
            ):
                mask = instance == instance_id
                values = semantic[mask].astype(np.int64)
                majority = int(np.bincount(values, minlength=6).argmax())
                semantic_error_pixels += int((values != majority).sum())
                instance_pixels += int(mask.sum())
                ys, xs = np.nonzero(mask)
                instances.append(
                    {
                        "scene_seed": scene["scene_seed"],
                        "frame_index": record["frame_index"],
                        "split": split,
                        "world_id": world_id,
                        "instance_id": instance_id,
                        "semantic_id": majority,
                        "semantic_class": CLASS_NAMES.get(majority, "background"),
                        "bbox_xyxy_px": [
                            int(xs.min()),
                            int(ys.min()),
                            int(xs.max() + 1),
                            int(ys.max() + 1),
                        ],
                        "bbox_shortest_side_px": int(
                            min(xs.max() - xs.min() + 1, ys.max() - ys.min() + 1)
                        ),
                        "mask_area_px": int(mask.sum()),
                    }
                )
            rgb_hash = hashlib.sha256(rgb_path.read_bytes()).hexdigest()
            perceptual_hash = phash(rgb_path)
            identity = {
                "scene_seed": scene["scene_seed"],
                "frame_index": record["frame_index"],
                "split": split,
                "world_id": world_id,
            }
            for value, seen, found in (
                (rgb_hash, exact_seen, exact_cross),
                (perceptual_hash, phash_seen, phash_cross),
            ):
                previous = seen.get(value)
                if previous and previous["split"] != split:
                    found.append([previous, identity])
                else:
                    seen[value] = identity
            frames.append(
                {
                    **identity,
                    "timestamp_ns": record["timestamp_ns"],
                    "rgb_path": str(rgb_path.relative_to(root)).replace("\\", "/"),
                    "depth_path": str(
                        (scene_dir / record["paths"]["depth"]).relative_to(root)
                    ).replace("\\", "/"),
                    "semantic_path": str(semantic_path.relative_to(root)).replace(
                        "\\", "/"
                    ),
                    "instance_path": str(instance_path.relative_to(root)).replace(
                        "\\", "/"
                    ),
                    "rgb_sha256": rgb_hash,
                    "phash": perceptual_hash,
                    "negative_only": bool(scene["negative_only"]),
                    "same_color_negative": bool(
                        scene.get("same_color_negative_present")
                    ),
                    "tf_valid": tf_valid,
                }
            )

    semantic_error_rate = semantic_error_pixels / max(instance_pixels, 1)
    leakage = {
        "target_asset_leakage": intersections(split_assets),
        "hard_negative_asset_leakage": intersections(split_negatives),
        "trajectory_leakage": intersections(split_trajectories),
        "world_leakage": intersections(split_worlds),
        "cross_split_exact_duplicate_count": len(exact_cross),
        "cross_split_phash_duplicate_count": len(phash_cross),
        "cross_split_exact_duplicates": exact_cross,
        "cross_split_phash_duplicates": phash_cross,
    }
    heldout_worlds = sorted(split_worlds["val"] | split_worlds["test"])
    gates = {
        "worlds_at_least_8": len(expected_worlds) >= 8,
        "world_split_4_2_2": {
            split: len(split_worlds[split]) for split in ("train", "val", "test")
        }
        == {"train": 4, "val": 2, "test": 2},
        "scenes_at_least_120": len(scenes) >= 120,
        "native_frames_at_least_1200": len(frames) >= 1200,
        "negative_only_each_split": all(
            any(frame["split"] == split and frame["negative_only"] for frame in frames)
            for split in ("train", "val", "test")
        ),
        "heldout_negative_frames_each_world_at_least_50": all(
            negative_frames_by_world[world] >= 50 for world in heldout_worlds
        ),
        "annotation_completeness_100_percent": len(frames) == len(scenes) * 10,
        "four_sensor_sync_100_percent": exact_sync_count == len(frames),
        "tf_valid_100_percent": tf_valid_count == len(frames),
        "sampled_pixel_object_label_error_at_most_0_01": semantic_error_rate <= 0.01,
        "asset_leakage_zero": not leakage["target_asset_leakage"]
        and not leakage["hard_negative_asset_leakage"],
        "world_leakage_zero": not leakage["world_leakage"],
        "trajectory_leakage_zero": not leakage["trajectory_leakage"],
        "exact_duplicate_zero": not exact_cross,
        "cross_split_phash_duplicate_zero": not phash_cross,
        "semantic_instance_consistency_error_zero": semantic_error_rate == 0.0,
        "class_count_0_to_3": not any(
            error["reason"] == "class_instance_count_out_of_range"
            for error in errors
        ),
        "scenario_missing_class_present": scenario_flags["missing_class"] > 0,
        "scenario_multi_instance_present": scenario_flags["multi_instance"] > 0,
        "scenario_overlap_present": scenario_flags["overlap"] > 0,
        "scenario_same_color_negative_present": scenario_flags[
            "same_color_negative"
        ]
        > 0,
        "scenario_dynamic_motion_executed": dynamic_scene_count > 0
        and dynamic_executed_count == dynamic_scene_count,
        "scenario_active_before_after_present": scenario_flags["active_before"] > 0
        and scenario_flags["active_after"] > 0,
        "all_five_classes_visible": all(
            class_presence[name] > 0 for name in CLASS_NAMES.values()
        ),
        "no_capture_errors": not errors,
    }
    report = {
        "schema_version": 1,
        "stage": "AUTO-05",
        "dataset_domain": world_manifest["dataset_domain"],
        "scene_count": len(scenes),
        "frame_count": len(frames),
        "world_count": len(expected_worlds),
        "split_scene_counts": dict(split_scene_counts),
        "split_worlds": {
            split: sorted(split_worlds[split])
            for split in ("train", "val", "test")
        },
        "scene_counts_by_world": dict(scene_counts_by_world),
        "negative_frames_by_world": dict(negative_frames_by_world),
        "annotation_completeness": len(frames) / max(len(scenes) * 10, 1),
        "four_sensor_sync_rate": exact_sync_count / max(len(frames), 1),
        "tf_valid_rate": tf_valid_count / max(len(frames), 1),
        "semantic_instance_error_rate": semantic_error_rate,
        "dynamic_scene_count": dynamic_scene_count,
        "dynamic_executed_count": dynamic_executed_count,
        "scenario_flags": dict(scenario_flags),
        "class_visible_frame_counts": dict(class_presence),
        "instance_record_count": len(instances),
        "leakage": leakage,
        "errors": errors,
        "gates": gates,
        "dataset_gate_pass": all(gates.values()),
    }
    (output / "g3_dataset_qa.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    (output / "split_manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "split_worlds": report["split_worlds"],
                "scene_counts": dict(split_scene_counts),
                "frame_counts": dict(Counter(frame["split"] for frame in frames)),
                "test_used_for_model_selection": False,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    (output / "leakage_report.json").write_text(
        json.dumps(leakage, indent=2) + "\n", encoding="utf-8"
    )
    with (output / "g3_frame_manifest.jsonl").open("w", encoding="utf-8") as stream:
        for row in frames:
            stream.write(json.dumps(row) + "\n")
    with (output / "g3_instance_records.jsonl").open(
        "w", encoding="utf-8"
    ) as stream:
        for row in instances:
            stream.write(json.dumps(row) + "\n")
    print(
        json.dumps(
            {
                "dataset_gate_pass": report["dataset_gate_pass"],
                "scene_count": len(scenes),
                "frame_count": len(frames),
                "failed_gates": [
                    name for name, passed in gates.items() if not passed
                ],
                "errors": errors[:10],
            },
            indent=2,
        )
    )
    return 0 if report["dataset_gate_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
