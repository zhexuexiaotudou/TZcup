#!/usr/bin/env python3
"""Prepare and audit development-only real-Gazebo COCO data for GA1."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np


CLASSES = ("plastic_bottle", "metal_can", "paper_litter")
CLASS_LABELS = {name: index + 1 for index, name in enumerate(CLASSES)}
SPLIT_BY_WORLD_INDEX = {0: "GA1_TRAIN", 1: "GA1_TRAIN", 2: "GA1_TRAIN", 3: "GA1_HOLDOUT"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def bbox(mask: np.ndarray) -> list[int] | None:
    ys, xs = np.nonzero(mask)
    if not len(xs):
        return None
    return [int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1)]


def world_indices(manifest: dict) -> dict[str, int]:
    return {
        item["world_id"]: index for index, item in enumerate(manifest["worlds"])
    }


def prepare(
    data_root: Path,
    world_manifest_path: Path,
    output: Path,
    *,
    expected_seed_min: int = 2000,
    expected_seed_max: int = 2023,
) -> dict:
    if output.exists():
        raise FileExistsError(output)
    world_manifest = json.loads(world_manifest_path.read_text(encoding="utf-8"))
    indices = world_indices(world_manifest)
    scenes = []
    for scene_dir in sorted((data_root / "scenes").glob("scene_*")):
        manifest_path = scene_dir / "scene_manifest.json"
        capture_path = scene_dir / "capture_report.json"
        if not manifest_path.is_file() or not capture_path.is_file():
            continue
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        capture = json.loads(capture_path.read_text(encoding="utf-8"))
        seed = int(manifest["scene_seed"])
        if not expected_seed_min <= seed <= expected_seed_max:
            raise RuntimeError(f"GA1 seed outside sealed range: {seed}")
        if capture.get("capture_pass") is not True:
            raise RuntimeError(f"failed capture in GA1 pack: {scene_dir}")
        if len(capture.get("records", [])) != 90:
            raise RuntimeError(f"GA1 mission must have 90 frames: {scene_dir}")
        world_index = indices.get(manifest["world_id"])
        if world_index not in SPLIT_BY_WORLD_INDEX:
            raise RuntimeError(f"GA1 world outside bounded four-world pack: {manifest['world_id']}")
        scenes.append(
            {
                "path": scene_dir,
                "manifest": manifest,
                "capture": capture,
                "split": SPLIT_BY_WORLD_INDEX[world_index],
            }
        )
    expected_seeds = set(range(expected_seed_min, expected_seed_max + 1))
    actual_seeds = {int(item["manifest"]["scene_seed"]) for item in scenes}
    if actual_seeds != expected_seeds:
        raise RuntimeError(
            f"GA1 seed set incomplete: missing={sorted(expected_seeds-actual_seeds)} "
            f"extra={sorted(actual_seeds-expected_seeds)}"
        )

    output.mkdir(parents=True)
    categories = [
        {"id": index + 1, "name": name, "supercategory": "litter"}
        for index, name in enumerate(CLASSES)
    ]
    image_hashes: dict[str, tuple[str, int, int]] = {}
    split_payloads = {}
    split_stats = {}
    for split in ("GA1_TRAIN", "GA1_HOLDOUT"):
        images = []
        annotations = []
        class_counts = Counter()
        negative_frames = 0
        image_id = 1
        annotation_id = 1
        selected = [item for item in scenes if item["split"] == split]
        for scene in selected:
            scene_dir = scene["path"]
            manifest = scene["manifest"]
            capture = scene["capture"]
            for record in capture["records"]:
                rgb_path = scene_dir / record["paths"]["rgb"]
                semantic_path = scene_dir / record["paths"]["semantic"]
                rgb_hash = sha256(rgb_path)
                prior = image_hashes.get(rgb_hash)
                identity = (
                    split,
                    int(manifest["scene_seed"]),
                    int(record["frame_index"]),
                )
                if prior is not None:
                    raise RuntimeError(
                        f"exact RGB duplicate across GA1 frames: {prior} and {identity}"
                    )
                image_hashes[rgb_hash] = identity
                relative = rgb_path.relative_to(data_root).as_posix()
                images.append(
                    {
                        "id": image_id,
                        "file_name": relative,
                        "width": 640,
                        "height": 480,
                        "mission_id": f"ga1-{manifest['world_id']}-{manifest['scene_seed']}",
                        "scene_seed": int(manifest["scene_seed"]),
                        "frame_index": int(record["frame_index"]),
                        "negative_only": bool(manifest.get("negative_only", False)),
                        "rgb_sha256": rgb_hash,
                    }
                )
                semantic = np.load(semantic_path, allow_pickle=False)
                frame_annotations = 0
                for class_name in CLASSES:
                    box = bbox(semantic == CLASS_LABELS[class_name])
                    if box is None:
                        continue
                    x1, y1, x2, y2 = box
                    annotations.append(
                        {
                            "id": annotation_id,
                            "image_id": image_id,
                            "category_id": CLASS_LABELS[class_name],
                            "bbox": [x1, y1, x2 - x1, y2 - y1],
                            "area": (x2 - x1) * (y2 - y1),
                            "iscrowd": 0,
                            "bbox_short_side_px": min(x2 - x1, y2 - y1),
                        }
                    )
                    annotation_id += 1
                    frame_annotations += 1
                    class_counts[class_name] += 1
                negative_frames += int(frame_annotations == 0)
                image_id += 1
        payload = {
            "info": {
                "description": f"GOCV7 GA1 development-only real Gazebo {split}",
                "G5_read": False,
                "G5_V2_read": False,
                "formal_30seed_read": False,
            },
            "licenses": [],
            "categories": categories,
            "images": images,
            "annotations": annotations,
        }
        path = output / ("train.json" if split == "GA1_TRAIN" else "holdout.json")
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        split_payloads[split] = path
        split_stats[split] = {
            "mission_count": len(selected),
            "frame_count": len(images),
            "annotation_count": len(annotations),
            "negative_frame_count": negative_frames,
            "world_ids": sorted({item["manifest"]["world_id"] for item in selected}),
            "scene_seeds": sorted(int(item["manifest"]["scene_seed"]) for item in selected),
            "class_annotations": dict(class_counts),
            "annotation_sha256": sha256(path),
        }
    train_worlds = set(split_stats["GA1_TRAIN"]["world_ids"])
    holdout_worlds = set(split_stats["GA1_HOLDOUT"]["world_ids"])
    train_seeds = set(split_stats["GA1_TRAIN"]["scene_seeds"])
    holdout_seeds = set(split_stats["GA1_HOLDOUT"]["scene_seeds"])
    leakage = {
        "world_overlap": sorted(train_worlds & holdout_worlds),
        "seed_overlap": sorted(train_seeds & holdout_seeds),
        "exact_rgb_duplicate_count": 0,
    }
    report = {
        "schema_version": 1,
        "protocol": "GAZEBO-ONLINE-CLOSURE-V7",
        "stage": "GOCV7-01-GA1-PREP",
        "dataset_root": data_root.as_posix(),
        "world_manifest": {
            "path": world_manifest_path.as_posix(),
            "sha256": sha256(world_manifest_path),
        },
        "splits": split_stats,
        "leakage_audit": leakage,
        "development_only": True,
        "GA1_HOLDOUT_used_for_threshold_only": True,
        "G5_read": False,
        "G5_V2_read": False,
        "formal_30seed_read": False,
        "GA1_PREP_PASS": not any(leakage.values()),
    }
    report_path = output / "GOCV7_GA1_DATA_PREP.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--world-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = prepare(args.data_root, args.world_manifest, args.output)
    print(json.dumps(report, indent=2))
    return 0 if report["GA1_PREP_PASS"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
