#!/usr/bin/env python3
"""Audit captured G10 approach missions without exposing sealed splits to training.

Semantic GT is consumed only by this offline QA/evaluation tool.  The product
runtime contract remains RGB-D/CameraInfo/TF and explicitly forbids GT inputs.
"""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


TARGETS = {1: "plastic_bottle", 2: "metal_can", 3: "paper_litter"}
BUCKETS = (("<18", 0, 18), ("18-32", 18, 32), ("32-64", 32, 64),
           ("64-96", 64, 96), (">=96", 96, 1_000_000))
EXPECTED_DOMAIN_MANIFEST_SHA256 = (
    "3bdb3006226943e4149cd84144b488e5eb112ab35ad3692c5da8cc48c88b5208"
)
EXPECTED_ROUTE_ID = "g10_route_v21_reverse_xneg1p95"
EXPECTED_ROUTE_CONFIG_SHA256 = (
    "ad6cb131739626827296cb37bcd365f58e3a0e4455f9c11f11dfe2b8111b4e36"
)
SPLIT_CONTRACT = {
    "G10_TRAIN": {
        "capture_split": "train",
        "world_ids": {
            "g10v15_train_w01_01_asphalt_campus",
            "g10v15_train_w02_02_concrete_sidewalk",
            "g10v15_train_w03_03_wet_courtyard",
            "g10v15_train_w04_04_cobblestone_arcade",
            "g10v15_train_w05_05_red_brick_promenade",
            "g10v15_train_w06_06_tiled_plaza",
        },
    },
    "G10_HOLDOUT": {
        "capture_split": "val",
        "world_ids": {
            "g10v15_val_w01_07_service_road",
            "g10v15_val_w02_08_mixed_curb_vegetation",
            "g10v15_val_w03_09_light_paver_pedestrian",
        },
    },
}


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def route_config_sha256(profile: dict) -> str:
    unsigned = dict(profile)
    unsigned.pop("route_config_sha256", None)
    return hashlib.sha256(
        json.dumps(unsigned, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def phash(path: Path) -> str:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"unreadable RGB frame: {path}")
    resized = cv2.resize(image, (32, 32), interpolation=cv2.INTER_AREA)
    spectrum = cv2.dct(np.float32(resized))[:8, :8]
    bits = spectrum > np.median(spectrum[1:])
    return f"{sum(int(value) << index for index, value in enumerate(bits.flat)):016x}"


def bbox_row(mask: np.ndarray, label: int, frame_index: int) -> dict | None:
    ys, xs = np.where(mask == label)
    if not len(xs):
        return None
    width = int(xs.max() - xs.min() + 1)
    height = int(ys.max() - ys.min() + 1)
    return {
        "frame_index": frame_index,
        "bbox_xyxy": [int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1)],
        "short_side_px": min(width, height),
        "pixels": int(len(xs)),
    }


def bucket_counts(rows: list[dict]) -> dict[str, int]:
    return {
        name: sum(low <= row["short_side_px"] < high for row in rows)
        for name, low, high in BUCKETS
    }


def ordered_size_transition(frames: list[dict]) -> bool:
    """Require far, mid, then close observations in temporal order."""
    stages = ((0, 18), (18, 32), (32, 64))
    cursor = -1
    for low, high in stages:
        match = next(
            (row["frame_index"] for row in frames
             if row["frame_index"] > cursor and low <= row["short_side_px"] < high),
            None,
        )
        if match is None:
            return False
        cursor = match
    return True


def audit_scene(scene: Path, split: str, minimum_reliable_short_side: int) -> tuple[dict, list[dict]]:
    if split not in SPLIT_CONTRACT:
        raise ValueError(f"unsupported G10 split: {split}")
    manifest = read(scene / "scene_manifest.json")
    report = read(scene / "capture_report.json")
    split_contract = SPLIT_CONTRACT[split]
    if manifest.get("split") != split_contract["capture_split"]:
        raise RuntimeError(f"{split} capture split mismatch: {scene}")
    if manifest.get("world_id") not in split_contract["world_ids"]:
        raise RuntimeError(f"{split} contains an unapproved world: {scene}")
    sequence_contract = manifest.get("trcrv10_g10_approach_sequence", {})
    profile = manifest.get("oprv3_motion_profile")
    if not isinstance(profile, dict) or report.get("oprv3_motion_profile") != profile:
        raise RuntimeError(f"scene/report route profile mismatch: {scene}")
    if (
        sequence_contract.get("route_id") != EXPECTED_ROUTE_ID
        or profile.get("route_id") != EXPECTED_ROUTE_ID
        or sequence_contract.get("route_config_sha256") != EXPECTED_ROUTE_CONFIG_SHA256
        or profile.get("route_config_sha256") != EXPECTED_ROUTE_CONFIG_SHA256
        or route_config_sha256(profile) != EXPECTED_ROUTE_CONFIG_SHA256
    ):
        raise RuntimeError(f"fixed G10 route identity mismatch: {scene}")
    if sequence_contract.get("source_domain_manifest_sha256") != EXPECTED_DOMAIN_MANIFEST_SHA256:
        raise RuntimeError(f"fixed G10 domain identity mismatch: {scene}")
    expected_trajectory = (
        f"{manifest['world_id']}_{EXPECTED_ROUTE_ID}_trajectory_"
        f"{int(manifest['scene_seed']):04d}"
    )
    if manifest.get("trajectory_id") != expected_trajectory:
        raise RuntimeError(f"G10 trajectory identity mismatch: {scene}")
    requested = int(report["requested_frames"])
    records = report.get("records", [])
    target_rows = []
    for label, class_id in TARGETS.items():
        frames = []
        for index in range(requested):
            row = bbox_row(np.load(scene / "semantic" / f"frame_{index:02d}.npy"), label, index)
            if row:
                frames.append(row)
        if frames:
            counts = bucket_counts(frames)
            maximum = max(frames, key=lambda item: item["short_side_px"])
            target_rows.append({
                "class_id": class_id,
                "visible_frames": len(frames),
                "first_visible": frames[0],
                "maximum": maximum,
                "last_visible": frames[-1],
                "bucket_counts": counts,
                "crosses_lt18_to_32_64": ordered_size_transition(frames),
                "minimum_reliable_short_side_px": minimum_reliable_short_side,
                "reaches_minimum_reliable": maximum["short_side_px"] >= minimum_reliable_short_side,
                "reaches_64": maximum["short_side_px"] >= 64,
                "reaches_96": maximum["short_side_px"] >= 96,
            })
    expected = 0 if manifest["negative_only"] else int(
        manifest.get("trcrv10_g10_approach_sequence", {}).get("targets_per_positive_mission", 3)
    )
    target_assets = [
        row["asset_id"] for row in manifest["objects"]
        if row["class_id"] in TARGETS.values()
    ]
    hard_negatives = [
        {"asset_id": row["asset_id"], "taxonomy": row.get("taxonomy")}
        for row in manifest["objects"] if row["class_id"] == "background"
    ]
    exact_hashes, perceptual_hashes = [], []
    for index in range(requested):
        rgb = scene / "rgb" / f"frame_{index:02d}.png"
        exact_hashes.append(hashlib.sha256(rgb.read_bytes()).hexdigest())
        perceptual_hashes.append(phash(rgb))
    motion = [row["vehicle_xy_m"] for row in records]
    travel = 0.0
    for first, second in zip(motion, motion[1:]):
        travel += float(np.linalg.norm(np.asarray(second) - np.asarray(first)))
    summary = {
        "scene": scene.name,
        "mission_id": scene.name,
        "world_id": manifest["world_id"],
        "scene_seed": manifest["scene_seed"],
        "trajectory_id": manifest["trajectory_id"],
        "split": split,
        "negative_only": manifest["negative_only"],
        "captured_frames": report["captured_frames"],
        "requested_frames": requested,
        "capture_pass": report["capture_pass"],
        "sensor_odom_sync_pass": report["sensor_odom_sync"]["pass"],
        "maximum_sensor_odom_skew_ns": report["sensor_odom_sync"]["maximum_skew_ns"],
        "observed_travel_m": travel,
        "target_classes_observed": sorted(row["class_id"] for row in target_rows),
        "target_count_expected": expected,
        "target_count_observed": len(target_rows),
        "target_asset_ids": target_assets,
        "hard_negatives": hard_negatives,
        "route_profile": report.get("oprv3_motion_profile"),
        "route_id": EXPECTED_ROUTE_ID,
        "route_config_sha256": EXPECTED_ROUTE_CONFIG_SHA256,
        "g10_domain_manifest_sha256": EXPECTED_DOMAIN_MANIFEST_SHA256,
        "motion_phase_counts": dict(Counter(row.get("motion_phase") for row in records)),
        "partial_mission": len(target_rows) != expected,
        "gt_product_input_violation": not bool(
            manifest.get("trcrv10_g10_approach_sequence", {}).get("gt_runtime_forbidden")
        ),
        "targets": target_rows,
    }
    frame_rows = [
        {"split": split, "scene": scene.name, "frame_index": index,
         "sha256": exact_hashes[index], "phash": perceptual_hashes[index]}
        for index in range(requested)
    ]
    return summary, frame_rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", action="append", required=True,
                        help="SPLIT=path containing g4_screening_native/scenes")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--minimum-reliable-short-side", type=int, required=True)
    parser.add_argument(
        "--train-only-authorization",
        action="store_true",
        help="return success only for a complete TRAIN route QA before HOLDOUT capture",
    )
    args = parser.parse_args()
    missions, frames = [], []
    declared_splits = set()
    for value in args.capture:
        split, raw_path = value.split("=", 1)
        if split not in SPLIT_CONTRACT:
            raise ValueError(f"unsupported G10 split declaration: {split}")
        if split in declared_splits:
            raise ValueError(f"duplicate G10 split declaration: {split}")
        declared_splits.add(split)
        scenes = Path(raw_path) / "g4_screening_native" / "scenes"
        for scene in sorted(path for path in scenes.glob("scene_*")
                            if (path / "capture_report.json").is_file()):
            mission, mission_frames = audit_scene(
                scene, split, args.minimum_reliable_short_side
            )
            missions.append(mission)
            frames.extend(mission_frames)
    split_counts = Counter(row["split"] for row in missions)
    split_names = sorted(split_counts)
    def overlap(key: str) -> list:
        values = {
            split: {
                value
                for row in missions if row["split"] == split
                for value in (row[key] if isinstance(row[key], list) else [row[key]])
            }
            for split in split_names
        }
        return sorted(set.intersection(*values.values())) if len(values) > 1 else []
    world_overlap = overlap("world_id")
    seed_overlap = overlap("scene_seed")
    trajectory_overlap = overlap("trajectory_id")
    asset_overlap = overlap("target_asset_ids")
    exact_overlap, phash_overlap = [], []
    exact_seen: dict[str, dict] = {}
    phash_seen: dict[str, dict] = {}
    for row in frames:
        prior = exact_seen.get(row["sha256"])
        if prior and prior["split"] != row["split"]:
            exact_overlap.append([prior, row])
        else:
            exact_seen.setdefault(row["sha256"], row)
        prior = phash_seen.get(row["phash"])
        if prior and prior["split"] != row["split"]:
            phash_overlap.append([prior, row])
        else:
            phash_seen.setdefault(row["phash"], row)
    def unique(key: str) -> bool:
        values = [row[key] for row in missions]
        return len(values) == len(set(values))

    gates = {
        "capture_pass": all(row["capture_pass"] for row in missions),
        "sensor_sync_pass": all(row["sensor_odom_sync_pass"] for row in missions),
        "partial_mission_zero": not any(row["partial_mission"] for row in missions),
        "positive_targets_cross_required_size_stages": all(
            target["crosses_lt18_to_32_64"]
            for row in missions if not row["negative_only"]
            for target in row["targets"]
        ),
        "positive_targets_reach_frozen_minimum": all(
            target["reaches_minimum_reliable"]
            for row in missions if not row["negative_only"]
            for target in row["targets"]
        ),
        "train_missions_at_least_45": split_counts.get("G10_TRAIN", 0) >= 45,
        "holdout_missions_at_least_18": split_counts.get("G10_HOLDOUT", 0) >= 18,
        "approved_world_sets_exact": all(
            {
                row["world_id"] for row in missions if row["split"] == split
            } == contract["world_ids"]
            for split, contract in SPLIT_CONTRACT.items()
        ),
        "mission_id_unique": unique("mission_id"),
        "scene_seed_unique": unique("scene_seed"),
        "trajectory_id_unique": unique("trajectory_id"),
        "world_overlap_zero": not world_overlap,
        "seed_overlap_zero": not seed_overlap,
        "trajectory_overlap_zero": not trajectory_overlap,
        "target_asset_overlap_zero": not asset_overlap,
        "gt_product_input_violation_zero": not any(row["gt_product_input_violation"] for row in missions),
        "exact_rgb_cross_split_duplicate_zero": not exact_overlap,
        "phash_cross_split_duplicate_zero": not phash_overlap,
    }
    train_missions = [row for row in missions if row["split"] == "G10_TRAIN"]
    train_positive_missions = [row for row in train_missions if not row["negative_only"]]
    train_negative_missions = [row for row in train_missions if row["negative_only"]]
    train_world_counts = Counter(row["world_id"] for row in train_missions)
    train_route_gates = {
        "only_train_declared": declared_splits == {"G10_TRAIN"},
        "train_missions_exactly_48": len(train_missions) == 48,
        "train_world_missions_exactly_8": all(
            train_world_counts[world_id] == 8
            for world_id in SPLIT_CONTRACT["G10_TRAIN"]["world_ids"]
        ),
        "train_positive_missions_exactly_30": len(train_positive_missions) == 30,
        "train_negative_missions_exactly_18": len(train_negative_missions) == 18,
        "train_target_classes_exact": {
            target["class_id"]
            for row in train_positive_missions for target in row["targets"]
        } == set(TARGETS.values()),
        "approved_train_world_set_exact": {
            row["world_id"] for row in train_missions
        } == SPLIT_CONTRACT["G10_TRAIN"]["world_ids"],
        "capture_pass": bool(train_missions) and all(
            row["capture_pass"] for row in train_missions
        ),
        "sensor_sync_pass": bool(train_missions) and all(
            row["sensor_odom_sync_pass"] for row in train_missions
        ),
        "partial_mission_zero": not any(
            row["partial_mission"] for row in train_missions
        ),
        "positive_targets_cross_required_size_stages": bool(train_missions) and all(
            target["crosses_lt18_to_32_64"]
            for row in train_missions if not row["negative_only"]
            for target in row["targets"]
        ),
        "positive_targets_reach_frozen_minimum": bool(train_missions) and all(
            target["reaches_minimum_reliable"]
            for row in train_missions if not row["negative_only"]
            for target in row["targets"]
        ),
        "mission_id_unique": len({row["mission_id"] for row in train_missions})
        == len(train_missions),
        "scene_seed_unique": len({row["scene_seed"] for row in train_missions})
        == len(train_missions),
        "trajectory_id_unique": len({row["trajectory_id"] for row in train_missions})
        == len(train_missions),
        "gt_product_input_violation_zero": not any(
            row["gt_product_input_violation"] for row in train_missions
        ),
    }
    train_route_qa_pass = all(train_route_gates.values())
    payload = {
        "schema_version": 1,
        "protocol": "TRCRV10",
        "route_id": EXPECTED_ROUTE_ID,
        "route_config_sha256": EXPECTED_ROUTE_CONFIG_SHA256,
        "g10_domain_manifest_sha256": EXPECTED_DOMAIN_MANIFEST_SHA256,
        "semantic_gt_role": "offline_QA_and_evaluator_only",
        "minimum_reliable_classification_short_side_px": args.minimum_reliable_short_side,
        "mission_counts": dict(split_counts),
        "missions": missions,
        "cross_split_duplicates": {"exact": exact_overlap, "phash": phash_overlap},
        "cross_split_identity_overlap": {
            "world": world_overlap,
            "seed": seed_overlap,
            "trajectory": trajectory_overlap,
            "target_asset": asset_overlap,
        },
        "gates": gates,
        "train_route_gates": train_route_gates,
        "G10_TRAIN_ROUTE_QA_PASS": train_route_qa_pass,
        "G10_CAPTURE_QA_PASS": bool(missions) and all(gates.values()),
    }
    write(args.output, payload)
    positive = [row for row in missions if not row["negative_only"]]
    write(args.output.parent / "G10_APPROACH_SEQUENCE_STATS.json", {
        "schema_version": 1,
        "protocol": "TRCRV10",
        "mission_counts": dict(split_counts),
        "positive_missions": len(positive),
        "negative_only_missions": len(missions) - len(positive),
        "route_profiles": dict(Counter(
            (row.get("route_profile") or {}).get("name", "straight_approach")
            for row in missions
        )),
        "motion_phase_counts": dict(Counter(
            phase
            for row in missions
            for phase, count in row["motion_phase_counts"].items()
            for _ in range(count)
        )),
        "partial_missions": sum(row["partial_mission"] for row in missions),
    })
    write(args.output.parent / "G10_SIZE_TRANSITION_STATS.json", {
        "schema_version": 1,
        "protocol": "TRCRV10",
        "targets": [
            {"split": row["split"], "scene": row["scene"], **target}
            for row in missions for target in row["targets"]
        ],
        "targets_crossing_lt18_to_32_64": sum(
            target["crosses_lt18_to_32_64"]
            for row in missions for target in row["targets"]
        ),
        "minimum_reliable_classification_short_side_px": args.minimum_reliable_short_side,
        "targets_reaching_minimum_reliable": sum(
            target["reaches_minimum_reliable"]
            for row in missions for target in row["targets"]
        ),
        "targets_reaching_64": sum(
            target["reaches_64"] for row in missions for target in row["targets"]
        ),
        "targets_reaching_96": sum(
            target["reaches_96"] for row in missions for target in row["targets"]
        ),
    })
    write(args.output.parent / "G10_HARD_NEGATIVE_MATRIX.json", {
        "schema_version": 1,
        "protocol": "TRCRV10",
        "by_split_taxonomy": {
            split: dict(Counter(
                item.get("taxonomy") or "unspecified"
                for row in missions if row["split"] == split
                for item in row["hard_negatives"]
            ))
            for split in split_names
        },
        "by_split_asset_count": {
            split: len({
                item["asset_id"]
                for row in missions if row["split"] == split
                for item in row["hard_negatives"]
            })
            for split in split_names
        },
    })
    write(args.output.parent / "G10_SPLIT_MANIFEST.json", {
        "schema_version": 1,
        "protocol": "TRCRV10",
        "splits": {
            split: [
                {key: row[key] for key in (
                    "scene", "mission_id", "world_id", "scene_seed", "trajectory_id",
                    "target_asset_ids", "negative_only",
                )}
                for row in missions if row["split"] == split
            ]
            for split in split_names
        },
        "cross_split_identity_overlap": payload["cross_split_identity_overlap"],
        "G10_DEV_VAL_SEALED_read": False,
    })
    print(json.dumps(payload, indent=2))
    if args.train_only_authorization:
        return 0 if train_route_qa_pass else 2
    return 0 if payload["G10_CAPTURE_QA_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
