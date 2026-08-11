#!/usr/bin/env python3
"""Prepare G7-only COCO inputs and the deterministic D1 exposure policy."""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import random
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.ddrv4_boundary import G7_DATASET_ID, require_ddrv4_selection_inputs  # noqa: E402
from sanitation_learning.g7_detector_dataset import CLASS_INDEX, CLASSES, load_jsonl  # noqa: E402


EXPOSURE_RATIOS = {
    "metal_can_targeted": 0.25,
    "negative_only": 0.25,
    "small_object_positive": 0.20,
    "general_positive": 0.30,
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def index_instances(rows: list[dict]) -> dict[tuple[int, int], list[dict]]:
    result: dict[tuple[int, int], list[dict]] = defaultdict(list)
    for row in rows:
        result[(int(row["scene_seed"]), int(row["frame_index"]))].append(row)
    return dict(result)


def frame_category(frame: dict, instances: list[dict]) -> str:
    if frame["negative_only"]:
        return "negative_only"
    if any(item["class_id"] == "metal_can" for item in instances):
        return "metal_can_targeted"
    if any(int(item["bbox_short_side_px"]) < 18 for item in instances):
        return "small_object_positive"
    return "general_positive"


def sampled_exposures(frames: list[dict], instances: dict[tuple[int, int], list[dict]], total: int = 2000, seed: int = 20260814) -> tuple[list[dict], dict]:
    if total <= 0 or total % 20:
        raise ValueError("D1 epoch exposures must be a positive multiple of 20")
    pools: dict[str, list[dict]] = {name: [] for name in EXPOSURE_RATIOS}
    for frame in frames:
        key = (int(frame["scene_seed"]), int(frame["frame_index"]))
        pools[frame_category(frame, instances.get(key, []))].append(frame)
    if any(not rows for rows in pools.values()):
        raise ValueError("D1 sampling pool is empty")
    rng = random.Random(seed)
    selected: list[dict] = []
    counts: dict[str, int] = {}
    for category, ratio in EXPOSURE_RATIOS.items():
        count = int(round(total * ratio))
        counts[category] = count
        pool = list(pools[category])
        rng.shuffle(pool)
        selected.extend({**pool[index % len(pool)], "exposure_category": category, "exposure_index": index} for index in range(count))
    rng.shuffle(selected)
    audit = {
        "schema_version": 1,
        "dataset_id": G7_DATASET_ID,
        "selection_inputs": [G7_DATASET_ID],
        "epoch_exposure_total": len(selected),
        "ratios": {name: counts[name] / len(selected) for name in counts},
        "counts": counts,
        "source_pool_counts": {name: len(rows) for name, rows in pools.items()},
        "mutually_exclusive_priority": ["negative_only", "metal_can_targeted", "small_object_positive", "general_positive"],
        "sampling_with_replacement": True,
        "seed": seed,
        "G6_used": False,
        "G5_used": False,
        "G5_V2_used": False,
    }
    return selected, audit


def to_coco(frames: list[dict], instances: dict[tuple[int, int], list[dict]], description: str) -> dict:
    images, annotations = [], []
    annotation_id = 1
    for image_id, frame in enumerate(frames, 1):
        images.append({
            "id": image_id, "file_name": frame["rgb_path"].replace("\\", "/"),
            "width": 640, "height": 480, "world_id": frame["world_id"],
            "scene_seed": frame["scene_seed"], "frame_index": frame["frame_index"],
            "negative_only": bool(frame["negative_only"]),
            "exposure_category": frame.get("exposure_category"),
        })
        key = (int(frame["scene_seed"]), int(frame["frame_index"]))
        for item in instances.get(key, []):
            x1, y1, x2, y2 = [float(value) for value in item["bbox_xyxy"]]
            annotations.append({
                "id": annotation_id, "image_id": image_id,
                "category_id": CLASS_INDEX[item["class_id"]],
                "bbox": [x1, y1, x2 - x1, y2 - y1],
                "area": float((x2 - x1) * (y2 - y1)), "iscrowd": 0,
                "bbox_short_side_px": int(item["bbox_short_side_px"]),
                "material": item["material"], "lighting": item["lighting"],
            })
            annotation_id += 1
    return {
        "info": {"description": description, "dataset_id": G7_DATASET_ID},
        "licenses": [],
        "categories": [{"id": CLASS_INDEX[name], "name": name, "supercategory": "litter"} for name in CLASSES],
        "images": images,
        "annotations": annotations,
    }


def prepare(g7_root: Path, output: Path, epoch_exposures: int = 2000) -> dict:
    require_ddrv4_selection_inputs([G7_DATASET_ID])
    if output.exists():
        raise FileExistsError(f"D1 prepared output exists: {output}")
    qa = json.loads((g7_root / "reports/G7_DATASET_QA.json").read_text(encoding="utf-8"))
    audit = json.loads((g7_root / "reports/G7_INDEPENDENT_AUDIT.json").read_text(encoding="utf-8"))
    if qa.get("G7_DATASET_PASS") is not True or audit.get("G7_INDEPENDENT_AUDIT_PASS") is not True:
        raise RuntimeError("G7 generator and independent audit must pass before D1")
    frames = load_jsonl(g7_root / "G7_FRAME_MANIFEST.jsonl")
    instances = index_instances(load_jsonl(g7_root / "G7_INSTANCE_RECORDS.jsonl"))
    by_split = {split: [row for row in frames if row["split"] == split] for split in ("TRAIN", "IN_DOMAIN_HOLDOUT", "CROSS_WORLD_VAL")}
    fit, policy = sampled_exposures(by_split["TRAIN"], instances, epoch_exposures)
    output.mkdir(parents=True, exist_ok=False)
    payloads = {
        "fit.json": to_coco(fit, instances, "DDRV4-D1 G7 TRAIN quota exposures"),
        "holdout.json": to_coco(by_split["IN_DOMAIN_HOLDOUT"], instances, "DDRV4-D1 selection-only G7 holdout"),
        "val.json": to_coco(by_split["CROSS_WORLD_VAL"], instances, "DDRV4-D1 one-time untouched G7 VAL"),
        "D1_SAMPLING_POLICY.json": policy,
    }
    for name, payload in payloads.items():
        (output / name).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    report = {
        "schema_version": 1, "stage": "DDRV4-03-PREP", "dataset_id": G7_DATASET_ID,
        "fit_images": len(fit), "holdout_images": len(by_split["IN_DOMAIN_HOLDOUT"]),
        "val_images": len(by_split["CROSS_WORLD_VAL"]),
        "selection_dataset": "IN_DOMAIN_HOLDOUT", "untouched_val_used_for_selection": False,
        "G6_used_for_selection": False, "G5_read": False, "G5_V2_read": False,
        "sha256": {name: sha256(output / name) for name in payloads},
    }
    (output / "D1_PREP_REPORT.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--g7-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--epoch-exposures", type=int, default=2000)
    args = parser.parse_args()
    print(json.dumps(prepare(args.g7_root, args.output, args.epoch_exposures), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
