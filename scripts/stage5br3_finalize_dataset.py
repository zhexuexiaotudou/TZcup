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


CLASS_NAMES = {1: "plastic_bottle", 2: "metal_can", 3: "paper_litter", 4: "leaf_pile", 5: "puddle"}


def phash(path: Path) -> str:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    small = cv2.resize(image, (32, 32), interpolation=cv2.INTER_AREA).astype(np.float32)
    coeff = cv2.dct(small)[:8, :8]
    bits = coeff > np.median(coeff[1:])
    return f"{int(''.join('1' if bit else '0' for bit in bits.ravel()), 2):016x}"


def percentile(values, q):
    return float(np.percentile(values, q)) if values else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    root = Path(args.data_root); errors = []; frames = []; instance_rows = []
    split_targets = defaultdict(set); split_negatives = defaultdict(set); split_trajectories = defaultdict(set)
    exact_seen = {}; phash_seen = {}; exact_cross = []; phash_cross = []
    semantic_error_pixels = 0; instance_pixels = 0; class_presence = Counter(); missing_class_scenes = Counter()
    negative_only_count = 0; hard_negative_counts = Counter(); size_short_sides = defaultdict(list)
    scene_dirs = sorted((root / "scenes").glob("scene_*"))
    for scene_dir in scene_dirs:
        scene_path = scene_dir / "scene_manifest.json"; capture_path = scene_dir / "capture_report.json"
        if not scene_path.is_file() or not capture_path.is_file():
            errors.append({"scene": scene_dir.name, "reason": "manifest_or_capture_missing"}); continue
        scene = json.loads(scene_path.read_text(encoding="utf-8")); capture = json.loads(capture_path.read_text(encoding="utf-8"))
        split = scene["split"]; split_trajectories[split].add(scene["trajectory_id"])
        split_targets[split].update(item["model_name"] for item in scene["objects"] if item["semantic_label"] != 0)
        split_negatives[split].update(item["model_name"] for item in scene["objects"] if item["semantic_label"] == 0)
        negative_only_count += int(scene["negative_only"]); hard_negative_counts[scene["hard_negative_count"]] += 1
        for name in scene["missing_target_classes"]: missing_class_scenes[name] += 1
        if not capture.get("capture_pass") or capture.get("captured_frames") != 10:
            errors.append({"scene_seed": scene["scene_seed"], "reason": "capture_gate_failed"})
        positions = [tuple(item["vehicle_xy_m"]) for item in capture["records"]]
        adjacent = [math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(positions, positions[1:])]
        if len(adjacent) != 9 or any(value < 0.25 for value in adjacent):
            errors.append({"scene_seed": scene["scene_seed"], "reason": "adjacent_motion_below_0.25m", "values": adjacent})
        for record in capture["records"]:
            rgb_path = scene_dir / record["paths"]["rgb"]
            semantic = np.load(scene_dir / record["paths"]["semantic"], allow_pickle=False)
            instance = np.load(scene_dir / record["paths"]["instance"], allow_pickle=False)
            if semantic.shape != (480, 640) or instance.shape != semantic.shape:
                errors.append({"scene_seed": scene["scene_seed"], "frame": record["frame_index"], "reason": "native_shape_mismatch"})
            labels = set(int(value) for value in np.unique(semantic))
            if not labels.issubset(set(range(6))):
                errors.append({"scene_seed": scene["scene_seed"], "frame": record["frame_index"], "reason": "unknown_semantic_id", "labels": sorted(labels)})
            for label in labels - {0}: class_presence[CLASS_NAMES[label]] += 1
            for instance_id in (int(v) for v in np.unique(instance) if int(v) != 0):
                mask = instance == instance_id; values = semantic[mask].astype(np.int64); area = int(mask.sum())
                majority = int(np.bincount(values, minlength=6).argmax())
                semantic_error_pixels += int((values != majority).sum()); instance_pixels += area
                ys, xs = np.nonzero(mask); width = int(xs.max()-xs.min()+1); height = int(ys.max()-ys.min()+1)
                short = min(width, height); size_short_sides[CLASS_NAMES.get(majority, "background")].append(short)
                instance_rows.append({"scene_seed": scene["scene_seed"], "frame_index": record["frame_index"], "split": split, "instance_id": instance_id, "semantic_id": majority, "semantic_class": CLASS_NAMES.get(majority, "background"), "bbox_xywh_px": [int(xs.min()), int(ys.min()), width, height], "bbox_shortest_side_px": short, "mask_area_px": area, "visibility": "visible"})
            rgb_hash = hashlib.sha256(rgb_path.read_bytes()).hexdigest(); perceptual = phash(rgb_path)
            for value, seen, found in ((rgb_hash, exact_seen, exact_cross), (perceptual, phash_seen, phash_cross)):
                previous = seen.get(value)
                if previous and previous["split"] != split: found.append([previous, {"scene_seed": scene["scene_seed"], "frame_index": record["frame_index"], "split": split}])
                else: seen[value] = {"scene_seed": scene["scene_seed"], "frame_index": record["frame_index"], "split": split}
            frames.append({"scene_seed": scene["scene_seed"], "frame_index": record["frame_index"], "split": split, "world_id": scene["world_id"], "timestamp_ns": record["timestamp_ns"], "rgb_sha256": rgb_hash, "phash": perceptual})

    target_leakage = sorted((split_targets["train"] & split_targets["val"]) | (split_targets["train"] & split_targets["test"]) | (split_targets["val"] & split_targets["test"]))
    negative_leakage = sorted((split_negatives["train"] & split_negatives["val"]) | (split_negatives["train"] & split_negatives["test"]) | (split_negatives["val"] & split_negatives["test"]))
    trajectory_leakage = sorted((split_trajectories["train"] & split_trajectories["val"]) | (split_trajectories["train"] & split_trajectories["test"]) | (split_trajectories["val"] & split_trajectories["test"]))
    semantic_error_rate = semantic_error_pixels / max(instance_pixels, 1)
    report = {
        "schema_version": 1, "stage": "Stage5BR3 G2 80-scene screening QA",
        "data_root": str(root), "scene_count": len(scene_dirs), "frame_count": len(frames),
        "annotation_completeness": len(frames) / 800, "native_resolution": [640, 480],
        "split_scene_counts": Counter(json.loads((p/"scene_manifest.json").read_text())["split"] for p in scene_dirs),
        "split_worlds": {split: sorted({frame["world_id"] for frame in frames if frame["split"] == split}) for split in ("train", "val", "test")},
        "target_asset_leakage": target_leakage, "hard_negative_asset_leakage": negative_leakage, "trajectory_leakage": trajectory_leakage,
        "cross_split_exact_duplicate_count": len(exact_cross), "cross_split_phash_duplicate_count": len(phash_cross),
        "semantic_instance_error_rate": semantic_error_rate, "semantic_instance_error_threshold": 0.01,
        "negative_only_scene_count": negative_only_count, "missing_class_scene_counts": dict(missing_class_scenes),
        "hard_negative_count_distribution": dict(sorted(hard_negative_counts.items())), "class_visible_frame_counts": dict(class_presence),
        "instance_bbox_shortest_side_px": {name: {"count": len(values), "p10": percentile(values, 10), "p50": percentile(values, 50), "p90": percentile(values, 90)} for name, values in sorted(size_short_sides.items())},
        "instance_record_count": len(instance_rows), "errors": errors,
    }
    report["annotation_qa_pass"] = all((len(scene_dirs)==80, len(frames)==800, report["annotation_completeness"]==1.0, not target_leakage, not negative_leakage, not trajectory_leakage, not exact_cross, not phash_cross, semantic_error_rate <= 0.01, negative_only_count > 0, all(missing_class_scenes[name] > 0 for name in CLASS_NAMES.values()), set(hard_negative_counts) >= {0, 8}, all(class_presence[name] > 0 for name in CLASS_NAMES.values()), not errors))
    output = Path(args.output); output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, default=dict) + "\n", encoding="utf-8")
    instance_output = output.with_name("g2_instance_qa_records.jsonl")
    with instance_output.open("w", encoding="utf-8") as stream:
        for row in instance_rows: stream.write(json.dumps(row) + "\n")
    print(json.dumps({key: report[key] for key in ("scene_count", "frame_count", "target_asset_leakage", "hard_negative_asset_leakage", "trajectory_leakage", "cross_split_exact_duplicate_count", "cross_split_phash_duplicate_count", "semantic_instance_error_rate", "negative_only_scene_count", "hard_negative_count_distribution", "errors", "annotation_qa_pass")}, indent=2))
    return 0 if report["annotation_qa_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
