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


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


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


def audit_scene(scene: Path, split: str) -> tuple[dict, list[dict]]:
    manifest = read(scene / "scene_manifest.json")
    report = read(scene / "capture_report.json")
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
            target_rows.append({
                "class_id": class_id,
                "visible_frames": len(frames),
                "first_visible": frames[0],
                "maximum": max(frames, key=lambda item: item["short_side_px"]),
                "last_visible": frames[-1],
                "bucket_counts": bucket_counts(frames),
                "crosses_lt18_to_32_64": (
                    bucket_counts(frames)["<18"] > 0
                    and bucket_counts(frames)["18-32"] > 0
                    and bucket_counts(frames)["32-64"] > 0
                ),
                "reaches_64": max(item["short_side_px"] for item in frames) >= 64,
                "reaches_96": max(item["short_side_px"] for item in frames) >= 96,
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
    args = parser.parse_args()
    missions, frames = [], []
    for value in args.capture:
        split, raw_path = value.split("=", 1)
        scenes = Path(raw_path) / "g4_screening_native" / "scenes"
        for scene in sorted(path for path in scenes.glob("scene_*")
                            if (path / "capture_report.json").is_file()):
            mission, mission_frames = audit_scene(scene, split)
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
    gates = {
        "capture_pass": all(row["capture_pass"] for row in missions),
        "sensor_sync_pass": all(row["sensor_odom_sync_pass"] for row in missions),
        "partial_mission_zero": not any(row["partial_mission"] for row in missions),
        "train_missions_at_least_45": split_counts.get("G10_TRAIN", 0) >= 45,
        "holdout_missions_at_least_18": split_counts.get("G10_HOLDOUT", 0) >= 18,
        "world_overlap_zero": not world_overlap,
        "seed_overlap_zero": not seed_overlap,
        "trajectory_overlap_zero": not trajectory_overlap,
        "target_asset_overlap_zero": not asset_overlap,
        "gt_product_input_violation_zero": not any(row["gt_product_input_violation"] for row in missions),
        "exact_rgb_cross_split_duplicate_zero": not exact_overlap,
        "phash_cross_split_duplicate_zero": not phash_overlap,
    }
    payload = {
        "schema_version": 1,
        "protocol": "TRCRV10",
        "semantic_gt_role": "offline_QA_and_evaluator_only",
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
                    "scene", "world_id", "scene_seed", "trajectory_id",
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
    return 0 if payload["G10_CAPTURE_QA_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
