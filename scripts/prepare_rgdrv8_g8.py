#!/usr/bin/env python3
"""Build and fail-closed audit the RGDRV8 G8 real-Gazebo detector pack."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import shutil

import cv2
import numpy as np


CLASS_NAMES = {1: "plastic_bottle", 2: "metal_can", 3: "paper_litter"}
SPLITS = ("TRAIN_NEW", "HOLDOUT_NEW", "VAL_NEW")
MISSION_MINIMUMS = {"TRAIN_NEW": 45, "HOLDOUT_NEW": 15, "VAL_NEW": 15}
MISSION_TARGETS = {"TRAIN_NEW": 60, "HOLDOUT_NEW": 20, "VAL_NEW": 20}
NEGATIVE_MINIMUMS = {"TRAIN_NEW": 20, "HOLDOUT_NEW": 5, "VAL_NEW": 5}
ENCOUNTER_MINIMUMS = {"TRAIN_NEW": 150, "HOLDOUT_NEW": 50, "VAL_NEW": 50}
SMALL_MINIMUMS = {"TRAIN_NEW": 60, "HOLDOUT_NEW": 20, "VAL_NEW": 20}


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def phash(path: Path) -> str:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"unreadable RGB image: {path}")
    small = cv2.resize(image, (32, 32), interpolation=cv2.INTER_AREA).astype(np.float32)
    coefficients = cv2.dct(small)[:8, :8]
    bits = coefficients > np.median(coefficients[1:])
    return f"{int(''.join('1' if bit else '0' for bit in bits.ravel()), 2):016x}"


def split_arg(value: str) -> tuple[str, Path]:
    try:
        split, path = value.split("=", 1)
    except ValueError as error:
        raise argparse.ArgumentTypeError("expected SPLIT=PATH") from error
    if split not in SPLITS:
        raise argparse.ArgumentTypeError(f"split must be one of {SPLITS}")
    root = Path(path).resolve()
    if not root.is_dir():
        raise argparse.ArgumentTypeError(f"not a directory: {root}")
    return split, root


def scene_dirs(root: Path) -> list[Path]:
    candidates = sorted(root.glob("**/scenes/scene_*"))
    return [path for path in candidates if path.is_dir()]


def materialize(source: Path, target: Path) -> str:
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        if sha256(target) != sha256(source):
            raise FileExistsError(f"materialized image differs: {target}")
        return "existing"
    try:
        os.link(source, target)
        return "hardlink"
    except OSError:
        shutil.copy2(source, target)
        return "copy"


def finite_pose(path: Path) -> bool:
    payload = read_json(path)
    values = payload.get("world_to_base_xy", []) + payload.get("base_to_camera_xyz_m", [])
    return len(values) == 5 and all(math.isfinite(float(value)) for value in values)


def bbox(mask: np.ndarray) -> tuple[list[int], int, int]:
    ys, xs = np.nonzero(mask)
    width, height = int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1)
    return [int(xs.min()), int(ys.min()), width, height], min(width, height), int(mask.sum())


def intersections(parts: dict[str, set]) -> list:
    found = set()
    for index, left in enumerate(SPLITS):
        for right in SPLITS[index + 1 :]:
            found |= parts[left] & parts[right]
    return sorted(found)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--split-root", action="append", type=split_arg, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames-per-mission", type=int, default=20)
    parser.add_argument(
        "--exclude-mission",
        action="append",
        default=[],
        help="Quarantine one whole mission as SPLIT:WORLD:SEED; never excludes frames partially",
    )
    args = parser.parse_args()
    roots: dict[str, list[Path]] = defaultdict(list)
    for split, root in args.split_root:
        roots[split].append(root)
    missing = [split for split in SPLITS if not roots[split]]
    if missing:
        raise ValueError(f"missing roots for: {missing}")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)

    errors: list[dict] = []
    missions: list[dict] = []
    frames: list[dict] = []
    annotations: list[dict] = []
    encounters: dict[tuple[str, int, int], dict] = {}
    visibility: dict[tuple[str, int, int], set[int]] = defaultdict(set)
    worlds, seeds, assets = (defaultdict(set) for _ in range(3))
    rgb_hashes, phashes = defaultdict(set), defaultdict(set)
    hash_owner: dict[tuple[str, str], dict] = {}
    exact_duplicates, perceptual_duplicates = [], []
    domain_counts: dict[str, Counter] = defaultdict(Counter)
    hard_negatives: dict[str, Counter] = defaultdict(Counter)
    materialization = Counter()
    exclusions = set(args.exclude_mission)
    excluded_missions: list[dict] = []
    image_id = annotation_id = 0
    semantic_error_pixels = instance_pixels = 0

    for split in SPLITS:
        seen_scene_paths: set[Path] = set()
        for root_index, root in enumerate(roots[split]):
            for scene_dir in scene_dirs(root):
                resolved = scene_dir.resolve()
                if resolved in seen_scene_paths:
                    continue
                seen_scene_paths.add(resolved)
                manifest_path, capture_path = scene_dir / "scene_manifest.json", scene_dir / "capture_report.json"
                if not manifest_path.is_file() or not capture_path.is_file():
                    errors.append({"split": split, "scene": scene_dir.name, "reason": "manifest_or_capture_missing"})
                    continue
                scene, capture = read_json(manifest_path), read_json(capture_path)
                seed, world = int(scene["scene_seed"]), str(scene["world_id"])
                mission_key = f"{split}:{world}:{seed}"
                if mission_key in exclusions:
                    excluded_missions.append(
                        {
                            "mission_key": mission_key,
                            "reason": "whole_mission_cross_split_phash_quarantine",
                            "manifest_sha256": sha256(manifest_path),
                            "capture_report_sha256": sha256(capture_path),
                        }
                    )
                    continue
                source_split = str(scene.get("source_world_split", scene.get("split")))
                expected_source = {"TRAIN_NEW": "train", "HOLDOUT_NEW": "val", "VAL_NEW": "test"}[split]
                if source_split != expected_source:
                    errors.append({"mission": mission_key, "reason": "source_split_mismatch", "actual": source_split, "expected": expected_source})
                records = capture.get("records", [])
                mission_errors = []
                if not capture.get("capture_pass"):
                    mission_errors.append("capture_pass_false")
                if capture.get("captured_frames") != args.frames_per_mission or len(records) != args.frames_per_mission:
                    mission_errors.append("frame_count_mismatch")
                if len({int(row.get("frame_index", -1)) for row in records}) != args.frames_per_mission:
                    mission_errors.append("partial_or_duplicate_frame_index")
                if not all(row.get("exact_four_sensor_timestamp") for row in records):
                    mission_errors.append("four_sensor_sync_violation")
                if not capture.get("sensor_odom_sync", {}).get("pass"):
                    mission_errors.append("sensor_odom_sync_violation")
                requirements = scene.get("oprv3_coverage_requirements", {})
                profile = scene.get("oprv3_coverage_profile") or "normal"
                domain_counts[split]["negative_only" if scene.get("negative_only") else profile] += 1
                domain_counts[split][f"lighting:{scene.get('lighting_executed_by_world')}"] += 1
                domain_counts[split][f"material:{scene.get('ground_material_executed_by_world')}"] += 1
                if requirements.get("turning"):
                    domain_counts[split]["turning"] += 1
                if requirements.get("behind_vehicle_fov_entry"):
                    domain_counts[split]["behind_fov_entry"] += 1
                if requirements.get("occlusion"):
                    domain_counts[split]["occlusion"] += 1
                if requirements.get("reflection"):
                    domain_counts[split]["reflection"] += 1
                if scene.get("dynamic_removal_plan"):
                    domain_counts[split]["dynamic_removal_declared"] += 1
                    if not capture.get("dynamic_removal_executed"):
                        mission_errors.append("dynamic_removal_not_executed")
                if scene.get("dynamic_insertion_plan"):
                    domain_counts[split]["dynamic_insertion_declared"] += 1
                    if not capture.get("dynamic_insertion_executed"):
                        mission_errors.append("dynamic_insertion_not_executed")
                for item in scene.get("objects", []):
                    asset = str(item.get("asset_id", item.get("model_name")))
                    assets[split].add(asset)
                    if not item.get("semantic_label"):
                        hard_negatives[split][str(item.get("taxonomy", "unknown"))] += 1
                worlds[split].add(world)
                seeds[split].add(seed)
                mission = {"mission_key": mission_key, "split": split, "source_root": str(root), "world_id": world, "world_sha256": scene.get("world_sha256"), "scene_seed": seed, "negative_only": bool(scene.get("negative_only")), "profile": profile, "manifest_sha256": sha256(manifest_path), "capture_report_sha256": sha256(capture_path), "mission_errors": mission_errors}
                missions.append(mission)
                for reason in mission_errors:
                    errors.append({"mission": mission_key, "reason": reason})

                for record in records:
                    index = int(record["frame_index"])
                    paths = {name: scene_dir / record["paths"][name] for name in ("rgb", "depth", "semantic", "instance", "tf")}
                    missing_paths = [name for name, path in paths.items() if not path.is_file()]
                    if missing_paths:
                        errors.append({"mission": mission_key, "frame": index, "reason": "frame_files_missing", "paths": missing_paths})
                        continue
                    semantic = np.load(paths["semantic"], allow_pickle=False)
                    instance = np.load(paths["instance"], allow_pickle=False)
                    if semantic.shape != instance.shape or semantic.ndim != 2:
                        errors.append({"mission": mission_key, "frame": index, "reason": "mask_shape_mismatch"})
                        continue
                    if not finite_pose(paths["tf"]):
                        errors.append({"mission": mission_key, "frame": index, "reason": "invalid_tf"})
                    image_id += 1
                    target = output / "images" / split.lower() / f"w{root_index}_{world}_{seed}" / f"frame_{index:02d}.png"
                    materialization[materialize(paths["rgb"], target)] += 1
                    rgb_digest, visual_digest = sha256(paths["rgb"]), phash(paths["rgb"])
                    identity = {"split": split, "mission": mission_key, "frame_index": index}
                    for kind, digest, by_split, found in (("rgb", rgb_digest, rgb_hashes, exact_duplicates), ("phash", visual_digest, phashes, perceptual_duplicates)):
                        owner = hash_owner.get((kind, digest))
                        if owner and owner["split"] != split:
                            found.append({"digest": digest, "first": owner, "second": identity})
                        else:
                            hash_owner[(kind, digest)] = identity
                        by_split[split].add(digest)
                    frame_row = {"id": image_id, "file_name": str(target.relative_to(output)).replace("\\", "/"), "width": int(semantic.shape[1]), "height": int(semantic.shape[0]), "mission_key": mission_key, "world_id": world, "scene_seed": seed, "frame_index": index, "negative_only": bool(scene.get("negative_only")), "rgb_sha256": rgb_digest, "perceptual_hash": visual_digest}
                    frames.append({"split": split, **frame_row})
                    for instance_id in (int(value) for value in np.unique(instance) if int(value) != 0):
                        mask = instance == instance_id
                        labels = semantic[mask].astype(np.int64)
                        majority = int(np.bincount(labels, minlength=6).argmax())
                        semantic_error_pixels += int((labels != majority).sum())
                        instance_pixels += int(mask.sum())
                        if majority not in CLASS_NAMES:
                            continue
                        box, short_side, area = bbox(mask)
                        annotation_id += 1
                        annotations.append({"id": annotation_id, "image_id": image_id, "category_id": majority, "bbox": box, "area": area, "iscrowd": 0, "instance_id": instance_id, "bbox_short_side_px": short_side})
                        key = (mission_key, instance_id, majority)
                        visibility[key].add(index)
                        if key not in encounters:
                            encounters[key] = {"split": split, "mission_key": mission_key, "instance_id": instance_id, "class_id": majority, "class_name": CLASS_NAMES[majority], "first_visible_frame": index, "first_visible_short_side_px": short_side, "visible_frame_count": 0}
                        encounters[key]["visible_frame_count"] += 1

    for key, visible in visibility.items():
        ordered = sorted(visible)
        if any(right - left > 1 for left, right in zip(ordered, ordered[1:])):
            domain_counts[encounters[key]["split"]]["empirical_reappearance"] += 1

    mission_counts = Counter(row["split"] for row in missions)
    negative_counts = Counter(row["split"] for row in missions if row["negative_only"])
    frame_counts = Counter(row["split"] for row in frames)
    encounter_counts = {split: Counter(row["class_name"] for row in encounters.values() if row["split"] == split) for split in SPLITS}
    small_counts = {split: Counter(row["class_name"] for row in encounters.values() if row["split"] == split and row["first_visible_short_side_px"] < 18) for split in SPLITS}
    world_overlap, seed_overlap, asset_overlap = intersections(worlds), intersections(seeds), intersections(assets)
    semantic_error_rate = semantic_error_pixels / max(instance_pixels, 1)
    gates = {
        "mission_minimums_met": all(mission_counts[s] >= MISSION_MINIMUMS[s] for s in SPLITS),
        "negative_only_minimums_met": all(negative_counts[s] >= NEGATIVE_MINIMUMS[s] for s in SPLITS),
        "encounter_minimums_met": all(encounter_counts[s][name] >= ENCOUNTER_MINIMUMS[s] for s in SPLITS for name in CLASS_NAMES.values()),
        "first_visible_lt18_minimums_met": all(small_counts[s][name] >= SMALL_MINIMUMS[s] for s in SPLITS for name in CLASS_NAMES.values()),
        "partial_mission_zero": not errors,
        "world_overlap_zero": not world_overlap,
        "seed_overlap_zero": not seed_overlap,
        "asset_overlap_zero": not asset_overlap,
        "exact_rgb_cross_split_duplicate_zero": not exact_duplicates,
        "phash_cross_split_duplicate_zero": not perceptual_duplicates,
        "semantic_instance_error_zero": semantic_error_rate == 0.0,
        "required_domain_roles_present": all(all(domain_counts[s][role] > 0 for role in ("normal", "turning", "behind_fov_entry", "occlusion", "reflection", "dynamic_removal_declared", "dynamic_insertion_declared", "negative_only")) for s in SPLITS),
    }
    qa = {"schema_version": 1, "protocol": "REAL-GAZEBO-DETECTOR-RECOVERY-V8", "stage": "G8-REAL-GAZEBO-DATA-QA", "mission_counts": dict(mission_counts), "mission_targets": MISSION_TARGETS, "frame_counts": dict(frame_counts), "negative_only_counts": dict(negative_counts), "encounter_counts_by_class": {s: dict(encounter_counts[s]) for s in SPLITS}, "first_visible_lt18_counts_by_class": {s: dict(small_counts[s]) for s in SPLITS}, "semantic_instance_error_rate": semantic_error_rate, "errors": errors, "cross_split": {"world_overlap": world_overlap, "seed_overlap": seed_overlap, "asset_overlap": asset_overlap, "exact_rgb_duplicates": exact_duplicates, "phash_duplicates": perceptual_duplicates}, "materialization": dict(materialization), "gates": gates, "G8_REAL_GAZEBO_DATA_PASS": all(gates.values())}
    reports = {
        "G8_DATASET_QA.json": qa,
        "G8_SPLIT_MANIFEST.json": {"schema_version": 1, "missions": missions, "excluded_whole_missions": excluded_missions, "counts": {"missions": dict(mission_counts), "frames": dict(frame_counts), "negative_only": dict(negative_counts)}, "HOLDOUT_NEW_selection_only": True, "VAL_NEW_used_before_route_freeze": False},
        "G8_WORLD_REGISTRY.json": {"schema_version": 1, "worlds_by_split": {s: sorted(worlds[s]) for s in SPLITS}, "cross_split_overlap": world_overlap},
        "G8_ASSET_REGISTRY.json": {"schema_version": 1, "assets_by_split": {s: sorted(assets[s]) for s in SPLITS}, "cross_split_overlap": asset_overlap},
        "G8_DOMAIN_MATRIX.json": {"schema_version": 1, "counts_by_split": {s: dict(domain_counts[s]) for s in SPLITS}},
        "G8_SCALE_DISTRIBUTION.json": {"schema_version": 1, "encounters": list(encounters.values()), "counts_by_split_class": {s: dict(encounter_counts[s]) for s in SPLITS}, "first_visible_lt18_by_split_class": {s: dict(small_counts[s]) for s in SPLITS}},
        "G8_HARD_NEGATIVE_TAXONOMY.json": {"schema_version": 1, "object_counts_by_split": {s: dict(hard_negatives[s]) for s in SPLITS}},
    }
    for name, payload in reports.items():
        (output / name).write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    categories = [{"id": key, "name": value} for key, value in CLASS_NAMES.items()]
    for split, name in (("TRAIN_NEW", "fit.json"), ("HOLDOUT_NEW", "holdout.json"), ("VAL_NEW", "val.json")):
        split_images = [{key: value for key, value in row.items() if key != "split"} for row in frames if row["split"] == split]
        ids = {row["id"] for row in split_images}
        split_annotations = [row for row in annotations if row["image_id"] in ids]
        (output / name).write_text(json.dumps({"info": {"description": f"RGDRV8 G8 {split}"}, "images": split_images, "annotations": split_annotations, "categories": categories}, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(output), "missions": dict(mission_counts), "frames": dict(frame_counts), "pass": qa["G8_REAL_GAZEBO_DATA_PASS"], "failed_gates": [key for key, value in gates.items() if not value]}, indent=2))
    return 0 if qa["G8_REAL_GAZEBO_DATA_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
