"""G4 dataset finalization and QA gates.

`finalize_g4_dataset` validates the G4 contract against a captured dataset:
scale (12 worlds / 300 scenes / 3000 frames), negative-only prior, taxonomy
coverage, annotation completeness, sensor sync, CameraInfo, TF, semantic /
instance consistency, split leakage, duplicates, and bucket coverage.  Partial
smoke datasets are reported with ``formal_scale.expected`` / ``actual`` and
never flip ``G4_dataset_gate_pass`` to true.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml

from .g4_assets import REQUIRED_PAPER_TAXONOMIES
from .g4_scene import (
    DISTANCE_BUCKETS,
    FRAMES_PER_SCENE,
    SIZE_BUCKETS,
)


CLASS_NAMES = {
    1: "plastic_bottle",
    2: "metal_can",
    3: "paper_litter",
    4: "leaf_pile",
    5: "puddle",
}
MIN_DECLARED_TARGET_VISIBLE_FRAMES = 2

SCALE_GATES = {
    "worlds_12_and_8_2_2",
    "scenes_300_and_frames_3000",
    "scenes_per_world_25",
    "train_negative_only_frames_at_least_500",
    "train_paper_like_hard_negative_frames_at_least_300",
    "required_paper_taxonomy_covered_in_train",
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


def _load_mask(path: Path) -> np.ndarray:
    if path.suffix == ".npy":
        return np.load(path, allow_pickle=False)
    return cv2.imread(str(path), cv2.IMREAD_UNCHANGED)


def _load_contract(path: Path) -> dict:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if payload.get("schema_version") != 1:
        raise ValueError("unsupported G4 contract schema")
    data_contract = payload["data_contract"]
    if data_contract.get("test_used_for_model_selection") is not False:
        raise ValueError("G4 contract must force test_used_for_model_selection=false")
    if data_contract.get("legacy_g3_test_used_as_selection") is not False:
        raise ValueError("G4 contract must keep legacy G3 test out of selection")
    return payload


def _intersections(parts: dict[str, set[str]]) -> list[str]:
    return sorted(
        (parts["train"] & parts["val"])
        | (parts["train"] & parts["test"])
        | (parts["val"] & parts["test"])
    )


def finalize_g4_dataset(
    data_root: str | Path,
    output_dir: str | Path,
    contract_path: str | Path | None = None,
    strict: bool = False,
) -> dict:
    root, output = Path(data_root), Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    contract = _load_contract(
        Path(contract_path)
        if contract_path is not None
        else Path(__file__).resolve().parents[2]
        / "config"
        / "auto05r_g4_contract.yaml"
    )
    world_manifest_path = root / "worlds" / "g4_world_manifest.json"
    if not world_manifest_path.is_file():
        raise ValueError(f"missing {world_manifest_path}")
    world_manifest = json.loads(world_manifest_path.read_text(encoding="utf-8"))
    expected_worlds = {
        item["world_id"]: item["split_eligibility"][0]
        for item in world_manifest["worlds"]
    }
    camera_resolution = world_manifest["native_capture_resolution"]
    expected_width = int(camera_resolution["width"])
    expected_height = int(camera_resolution["height"])
    formal = contract["formal_scale"]
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
    negative_only_by_split = Counter()
    negative_only_scene_counts = Counter()
    scene_counts_by_world = Counter()
    split_scene_counts = Counter()
    taxonomy_seen_in_train: set[str] = set()
    paper_like_frames_train = 0
    negative_only_frames_train = 0
    tf_valid_count = 0
    exact_sync_count = 0
    camera_info_valid_count = 0
    scene_pose_reset_valid_count = 0
    manifest_pixel_consistent_count = 0
    declared_scene_class_total = 0
    declared_scene_class_visible = 0
    distance_seen: set[tuple[float, float]] = set()
    size_seen: set[str] = set()
    distance_bucket_frame_counts: Counter = Counter()
    size_bucket_frame_counts: Counter = Counter()

    scenes = sorted((root / "scenes").glob("scene_*"))
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
        if expected_worlds.get(world_id) != split:
            errors.append({"scene": scene_dir.name, "reason": "world_split_mismatch"})
        if scene.get("schema_version") != 2:
            errors.append({"scene": scene_dir.name, "reason": "scene_schema_version"})
        if scene.get("native_gazebo_applied") is not True:
            errors.append({"scene": scene_dir.name, "reason": "native_gazebo_applied_false"})
        pose_reset = scene.get("pose_reset_contract", {})
        pose_reset_valid = (
            pose_reset.get("all_world_assets_accounted_for") is True
            and int(pose_reset.get("duplicate_asset_pose_names", -1)) == 0
            and int(pose_reset.get("asset_pose_count", -1))
            == len(world_manifest["assets"])
            + len(world_manifest["negative_assets"])
        )
        scene_pose_reset_valid_count += int(pose_reset_valid)
        if not pose_reset_valid:
            errors.append(
                {"scene": scene_dir.name, "reason": "scene_pose_reset_contract_invalid"}
            )
        offline = scene.get("offline_sensor_augmentation", {})
        if offline.get("requested_only") is not False or offline.get("applied") is not False:
            errors.append(
                {"scene": scene_dir.name, "reason": "offline_augmentation_contract_violation"}
            )
        for item in scene["objects"]:
            distance_bucket = tuple(item["distance_bucket_m"])
            if distance_bucket not in DISTANCE_BUCKETS:
                errors.append(
                    {"scene": scene_dir.name, "reason": "invalid_distance_bucket"}
                )
            else:
                distance_seen.add(distance_bucket)
            size_seen.add(item["size_bucket"])
            distance_bucket_frame_counts[
                f"{item['distance_bucket_m'][0]}_{item['distance_bucket_m'][1]}"
            ] += 1
            size_bucket_frame_counts[item["size_bucket"]] += 1
            if item["semantic_label"]:
                split_assets[split].add(item["asset_id"])
            else:
                split_negatives[split].add(item["asset_id"])
                taxonomy = item.get("taxonomy")
                if split == "train" and taxonomy:
                    taxonomy_seen_in_train.add(taxonomy)
        if not capture.get("capture_pass"):
            errors.append(
                {"scene": scene_dir.name, "reason": "capture_gate_failed"}
            )
        records = capture.get("records", [])
        if len(records) != FRAMES_PER_SCENE:
            errors.append(
                {
                    "scene": scene_dir.name,
                    "reason": "frame_count_mismatch",
                    "actual": len(records),
                }
            )
        if scene["negative_only"]:
            negative_only_by_split[split] += len(records)
            negative_only_scene_counts[split] += 1
            if split == "train":
                negative_only_frames_train += len(records)
        if split == "train" and scene.get("paper_like_hard_negative_count", 0) > 0:
            paper_like_frames_train += len(records)
        declared_target_counts = Counter(
            int(item["semantic_label"])
            for item in scene["objects"]
            if int(item.get("semantic_label") or 0) in CLASS_NAMES
        )
        sequence_max_observed: Counter = Counter()
        sequence_full_visibility_frames: Counter = Counter()
        positions = [tuple(record["vehicle_xy_m"]) for record in records]
        adjacent = [
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(positions, positions[1:])
        ]
        if len(adjacent) != FRAMES_PER_SCENE - 1 or any(
            distance < 0.25 for distance in adjacent
        ):
            errors.append(
                {
                    "scene": scene_dir.name,
                    "reason": "adjacent_motion_below_0.25m",
                    "values": adjacent,
                }
            )
        for record in records:
            rgb_path = scene_dir / record["paths"]["rgb"]
            semantic_path = scene_dir / record["paths"]["semantic"]
            instance_path = scene_dir / record["paths"]["instance"]
            camera_path = scene_dir / record["paths"]["camera"]
            tf_path = scene_dir / record["paths"]["tf"]
            if not all(path.is_file() for path in (rgb_path, semantic_path, instance_path, camera_path, tf_path)):
                errors.append(
                    {
                        "scene": scene_dir.name,
                        "frame": record["frame_index"],
                        "reason": "frame_file_missing",
                    }
                )
                continue
            semantic = _load_mask(semantic_path)
            instance = _load_mask(instance_path)
            if (
                semantic.shape != (expected_height, expected_width)
                or instance.shape != semantic.shape
            ):
                errors.append(
                    {
                        "scene": scene_dir.name,
                        "frame": record["frame_index"],
                        "reason": "native_shape_mismatch",
                    }
                )
                continue
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
            camera_ok = False
            try:
                camera = json.loads(camera_path.read_text(encoding="utf-8"))
                camera_ok = (
                    int(camera.get("width", -1)) == expected_width
                    and int(camera.get("height", -1)) == expected_height
                    and len(camera.get("k", [])) == 9
                    and all(math.isfinite(float(value)) for value in camera.get("k", []))
                )
            except (ValueError, json.JSONDecodeError):
                camera_ok = False
            camera_info_valid_count += int(camera_ok)
            if not camera_ok:
                errors.append(
                    {
                        "scene": scene_dir.name,
                        "frame": record["frame_index"],
                        "reason": "camera_info_invalid",
                    }
                )
            observed_target_counts: Counter = Counter()
            for instance_id in (
                int(value) for value in np.unique(instance) if int(value) != 0
            ):
                mask = instance == instance_id
                values = semantic[mask].astype(np.int64)
                majority = int(np.bincount(values, minlength=6).argmax())
                if majority in CLASS_NAMES:
                    observed_target_counts[majority] += 1
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
            for semantic_id in CLASS_NAMES:
                sequence_max_observed[semantic_id] = max(
                    sequence_max_observed[semantic_id],
                    observed_target_counts[semantic_id],
                )
                if (
                    declared_target_counts[semantic_id] > 0
                    and observed_target_counts[semantic_id]
                    >= declared_target_counts[semantic_id]
                ):
                    sequence_full_visibility_frames[semantic_id] += 1
            target_count_mismatch = {
                CLASS_NAMES[semantic_id]: {
                    "declared": int(declared_target_counts[semantic_id]),
                    "observed": int(observed_target_counts[semantic_id]),
                }
                for semantic_id in CLASS_NAMES
                if observed_target_counts[semantic_id]
                > declared_target_counts[semantic_id]
            }
            manifest_pixel_consistent_count += int(not target_count_mismatch)
            if target_count_mismatch:
                errors.append(
                    {
                        "scene": scene_dir.name,
                        "frame": record["frame_index"],
                        "reason": "undeclared_pixel_target_count_exceeded",
                        "classes": target_count_mismatch,
                        "negative_only": bool(scene["negative_only"]),
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
                    "tf_valid": tf_valid,
                }
            )
        for semantic_id, declared_count in sorted(declared_target_counts.items()):
            if declared_count <= 0:
                continue
            declared_scene_class_total += 1
            visible_frames = sequence_full_visibility_frames[semantic_id]
            visible = (
                sequence_max_observed[semantic_id] >= declared_count
                and visible_frames >= MIN_DECLARED_TARGET_VISIBLE_FRAMES
            )
            declared_scene_class_visible += int(visible)
            if not visible:
                errors.append(
                    {
                        "scene": scene_dir.name,
                        "reason": "declared_target_sequence_visibility_failed",
                        "semantic_class": CLASS_NAMES[semantic_id],
                        "declared": int(declared_count),
                        "maximum_observed_in_one_frame": int(
                            sequence_max_observed[semantic_id]
                        ),
                        "full_visibility_frames": int(visible_frames),
                        "minimum_required_frames": (
                            MIN_DECLARED_TARGET_VISIBLE_FRAMES
                        ),
                    }
                )

    semantic_error_rate = semantic_error_pixels / max(instance_pixels, 1)
    split_scene_actual = {
        split: split_scene_counts[split] for split in ("train", "val", "test")
    }
    negative_only_ratio = {
        split: negative_only_scene_counts[split] / max(split_scene_actual[split], 1)
        for split in ("train", "val", "test")
    }
    delta_pp = max(negative_only_ratio.values()) - min(negative_only_ratio.values())
    leakage = {
        "target_asset_leakage": _intersections(split_assets),
        "hard_negative_asset_leakage": _intersections(split_negatives),
        "trajectory_leakage": _intersections(split_trajectories),
        "world_leakage": _intersections(split_worlds),
        "cross_split_exact_duplicate_count": len(exact_cross),
        "cross_split_phash_duplicate_count": len(phash_cross),
        "cross_split_exact_duplicates": exact_cross,
        "cross_split_phash_duplicates": phash_cross,
    }
    missing_taxonomies = sorted(set(REQUIRED_PAPER_TAXONOMIES) - taxonomy_seen_in_train)
    gates = {
        "worlds_12_and_8_2_2": len(expected_worlds) == formal["worlds"]
        and {
            split: len(split_worlds[split]) for split in ("train", "val", "test")
        }
        == formal["world_split_counts"],
        "scenes_300_and_frames_3000": len(scenes) == formal["scenes"]
        and len(frames) == formal["frames"],
        "scenes_per_world_25": (
            len(scene_counts_by_world) == formal["worlds"]
            and set(scene_counts_by_world.values()) == {formal["scenes_per_world"]}
        ),
        "negative_only_ratio_in_25_to_35_percent": all(
            low <= negative_only_ratio[split] <= high
            for split in ("train", "val", "test")
            for low, high in [tuple(contract["negative_prior"]["negative_only_ratio_range"])]
        ),
        "negative_only_cross_split_delta_at_most_10pp": delta_pp
        <= contract["negative_prior"]["cross_split_ratio_delta_max"],
        "train_negative_only_frames_at_least_500": negative_only_frames_train
        >= contract["negative_prior"]["train_negative_only_frames_min"],
        "train_paper_like_hard_negative_frames_at_least_300": paper_like_frames_train
        >= contract["negative_prior"]["train_paper_like_hard_negative_frames_min"],
        "required_paper_taxonomy_covered_in_train": not missing_taxonomies,
        "annotation_completeness_100_percent": len(frames) == len(scenes) * FRAMES_PER_SCENE,
        "four_sensor_sync_100_percent": exact_sync_count == len(frames),
        "camera_info_valid_100_percent": camera_info_valid_count == len(frames),
        "tf_valid_100_percent": tf_valid_count == len(frames),
        "semantic_instance_error_zero": semantic_error_rate == 0.0,
        "scene_pose_reset_contract_100_percent": scene_pose_reset_valid_count
        == len(scenes),
        "manifest_pixel_target_consistency_100_percent": (
            manifest_pixel_consistent_count == len(frames)
        ),
        "declared_target_sequence_visibility_100_percent": (
            declared_scene_class_visible == declared_scene_class_total
        ),
        "asset_split_leakage_zero": not leakage["target_asset_leakage"]
        and not leakage["hard_negative_asset_leakage"],
        "world_split_leakage_zero": not leakage["world_leakage"],
        "trajectory_split_leakage_zero": not leakage["trajectory_leakage"],
        "exact_duplicate_zero": not exact_cross,
        "cross_split_phash_duplicate_zero": not phash_cross,
        "distance_bucket_coverage_all": set(DISTANCE_BUCKETS).issubset(distance_seen),
        "size_bucket_coverage_all": set(SIZE_BUCKETS).issubset(size_seen),
        "test_used_for_model_selection_false": True,
    }
    report = {
        "schema_version": 2,
        "stage": "AUTO-05R",
        "task": "AUTO-05R-1",
        "dataset_domain": world_manifest["dataset_domain"],
        "contract_sha256": hashlib.sha256(
            Path(contract_path).read_bytes()
            if contract_path is not None
            else Path(__file__).resolve().parents[2]
            / "config"
            / "auto05r_g4_contract.yaml"
        ).hexdigest(),
        "formal_scale": {
            "expected": {
                "worlds": formal["worlds"],
                "world_split_counts": formal["world_split_counts"],
                "scenes": formal["scenes"],
                "scenes_per_world": formal["scenes_per_world"],
                "frames": formal["frames"],
                "frames_per_scene": formal["frames_per_scene"],
            },
            "actual": {
                "worlds": len(
                    {
                        world
                        for split in ("train", "val", "test")
                        for world in split_worlds[split]
                    }
                ),
                "world_split_counts": {
                    split: len(split_worlds[split])
                    for split in ("train", "val", "test")
                },
                "scenes": len(scenes),
                "scenes_per_world": dict(scene_counts_by_world),
                "frames": len(frames),
                "frames_per_scene": FRAMES_PER_SCENE,
            },
        },
        "scene_count": len(scenes),
        "frame_count": len(frames),
        "world_count": len(expected_worlds),
        "split_scene_counts": split_scene_actual,
        "split_worlds": {
            split: sorted(split_worlds[split])
            for split in ("train", "val", "test")
        },
        "scene_counts_by_world": dict(scene_counts_by_world),
        "negative_only_frames_by_split": dict(negative_only_by_split),
        "negative_only_ratio_by_split": negative_only_ratio,
        "negative_only_cross_split_delta_pp": round(delta_pp * 100.0, 2),
        "train_negative_only_frames": negative_only_frames_train,
        "train_paper_like_hard_negative_frames": paper_like_frames_train,
        "taxonomy_covered_in_train": sorted(taxonomy_seen_in_train),
        "missing_paper_taxonomies_in_train": missing_taxonomies,
        "annotation_completeness": len(frames) / max(len(scenes) * FRAMES_PER_SCENE, 1),
        "four_sensor_sync_rate": exact_sync_count / max(len(frames), 1),
        "camera_info_valid_rate": camera_info_valid_count / max(len(frames), 1),
        "tf_valid_rate": tf_valid_count / max(len(frames), 1),
        "semantic_instance_error_rate": semantic_error_rate,
        "scene_pose_reset_valid_rate": scene_pose_reset_valid_count
        / max(len(scenes), 1),
        "manifest_pixel_target_consistency_rate": manifest_pixel_consistent_count
        / max(len(frames), 1),
        "declared_target_sequence_visibility_rate": (
            declared_scene_class_visible / max(declared_scene_class_total, 1)
        ),
        "declared_target_scene_class_count": declared_scene_class_total,
        "instance_record_count": len(instances),
        "leakage": leakage,
        "distance_bucket_counts": dict(distance_bucket_frame_counts),
        "size_bucket_counts": dict(size_bucket_frame_counts),
        "errors": errors,
        "gates": gates,
        "test_used_for_model_selection": False,
        "full_capture_executed": (
            len(scenes) == formal["scenes"]
            and len(frames) == formal["frames"]
            and len(scene_counts_by_world) == formal["worlds"]
        ),
        "G4_dataset_gate_pass": all(gates.values()),
    }
    report["quality_gates_pass"] = all(
        passed for name, passed in gates.items() if name not in SCALE_GATES
    )
    (output / "g4_dataset_qa.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    (output / "split_manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 2,
                "split_worlds": report["split_worlds"],
                "scene_counts": split_scene_actual,
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
    with (output / "g4_frame_manifest.jsonl").open("w", encoding="utf-8") as stream:
        for row in frames:
            stream.write(json.dumps(row) + "\n")
    with (output / "g4_instance_records.jsonl").open(
        "w", encoding="utf-8"
    ) as stream:
        for row in instances:
            stream.write(json.dumps(row) + "\n")
    return report


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--contract", default=None)
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    report = finalize_g4_dataset(
        args.data_root,
        args.output_dir,
        contract_path=args.contract,
        strict=args.strict,
    )
    failed = [name for name, passed in report["gates"].items() if not passed]
    print(
        json.dumps(
            {
                "G4_dataset_gate_pass": report["G4_dataset_gate_pass"],
                "quality_gates_pass": report["quality_gates_pass"],
                "scene_count": report["scene_count"],
                "frame_count": report["frame_count"],
                "formal_scale": report["formal_scale"],
                "failed_gates": failed,
                "errors": report["errors"][:10],
            },
            indent=2,
        )
    )
    if args.strict:
        return 0 if report["G4_dataset_gate_pass"] else 2
    return 0 if report["quality_gates_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
