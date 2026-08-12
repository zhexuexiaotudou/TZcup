#!/usr/bin/env python3
"""Prepare the bounded Route A source pool and frozen exposure policy."""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import os
from pathlib import Path
import random
import shutil


RATIOS = {"small_positive": 0.25, "metal_targeted": 0.20, "general_positive": 0.25, "hard_negative": 0.30}


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def link(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(source, target)
    except OSError:
        shutil.copy2(source, target)


def remap(payload: dict, source_root: Path, output: Path, prefix: str, start_image: int, start_annotation: int) -> tuple[list[dict], list[dict]]:
    images, annotations, ids = [], [], {}
    for offset, row in enumerate(payload["images"]):
        image_id = start_image + offset
        source = source_root / row["file_name"]
        suffix = source.suffix.lower() or ".png"
        target = output / "images" / "source" / prefix / f"{image_id:07d}{suffix}"
        link(source, target)
        ids[int(row["id"])] = image_id
        images.append({**row, "id": image_id, "file_name": target.relative_to(output).as_posix(), "source_pool": prefix})
    for offset, row in enumerate(payload["annotations"]):
        annotations.append({**row, "id": start_annotation + offset, "image_id": ids[int(row["image_id"])]})
    return images, annotations


def category(image: dict, annotations: list[dict]) -> str:
    if not annotations:
        return "hard_negative"
    if any(float(row.get("bbox_short_side_px", min(row["bbox"][2:]))) < 18 for row in annotations):
        return "small_positive"
    if any(int(row["category_id"]) == 2 for row in annotations):
        return "metal_targeted"
    return "general_positive"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--g8-root", type=Path, required=True)
    parser.add_argument("--ga1-data-root", type=Path, required=True)
    parser.add_argument("--ga1-prepared", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--proposal-coco", type=Path)
    parser.add_argument("--epoch-exposures", type=int, default=4000)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    if args.epoch_exposures <= 0 or args.epoch_exposures % 20:
        raise ValueError("epoch exposures must be a positive multiple of 20")
    args.output.mkdir(parents=True)
    g8_qa = load(args.g8_root / "G8_DATASET_QA.json")
    ga1_qa = load(args.ga1_prepared / "GOCV7_GA1_DATA_PREP.json")
    if g8_qa.get("G8_REAL_GAZEBO_DATA_PASS") is not True or ga1_qa.get("GA1_PREP_PASS") is not True:
        raise RuntimeError("G8 and GA1 preparation must pass")
    if g8_qa.get("protocol") != "REAL-GAZEBO-DETECTOR-RECOVERY-V8":
        raise RuntimeError("unexpected G8 protocol")

    images: list[dict] = []
    annotations: list[dict] = []
    for payload, root, prefix in (
        (load(args.g8_root / "fit.json"), args.g8_root, "g8_train"),
        (load(args.ga1_prepared / "train.json"), args.ga1_data_root, "legacy_ga1_train"),
    ):
        added_images, added_annotations = remap(payload, root, args.output, prefix, len(images) + 1, len(annotations) + 1)
        images.extend(added_images)
        annotations.extend(added_annotations)
    source = {"info": {"description": "RGDRV8 Route A TRAIN-only source before proposal mining"}, "images": images, "annotations": annotations, "categories": load(args.g8_root / "fit.json")["categories"]}
    (args.output / "source_train.json").write_text(json.dumps(source, indent=2) + "\n", encoding="utf-8")

    if args.proposal_coco is None:
        report = {"schema_version": 1, "stage": "RGDRV8-02-ROUTE-A-SOURCE", "source_image_count": len(images), "source_annotation_count": len(annotations), "source_train_sha256": sha256(args.output / "source_train.json"), "selection_data_read": False, "VAL_NEW_read": False, "G5_V2_read": False, "ROUTE_A_SOURCE_PASS": len(images) > 0 and len(annotations) > 0}
        (args.output / "ROUTE_A_SOURCE_REPORT.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
        return 0 if report["ROUTE_A_SOURCE_PASS"] else 2

    proposals = load(args.proposal_coco)
    if proposals.get("info", {}).get("TRAIN_ONLY") is not True or len(proposals["images"]) < 2000:
        raise RuntimeError("proposal mining did not provide >=2000 TRAIN-only crops")
    added_images, added_annotations = remap(proposals, args.proposal_coco.parent, args.output, "proposal_hard_negative", len(images) + 1, len(annotations) + 1)
    if added_annotations:
        raise RuntimeError("hard-negative proposal crops must have empty annotations")
    images.extend(added_images)
    by_image: dict[int, list[dict]] = defaultdict(list)
    for row in annotations:
        by_image[int(row["image_id"])].append(row)
    pools: dict[str, list[dict]] = {name: [] for name in RATIOS}
    for image in images:
        pools[category(image, by_image[int(image["id"])])].append(image)
    if any(not rows for rows in pools.values()):
        raise RuntimeError(f"Route A sampling pool empty: { {name: len(rows) for name, rows in pools.items()} }")
    rng = random.Random(20260813)
    selected: list[dict] = []
    counts = {}
    for name, ratio in RATIOS.items():
        count = round(args.epoch_exposures * ratio)
        counts[name] = count
        pool = list(pools[name])
        rng.shuffle(pool)
        selected.extend({**pool[index % len(pool)], "exposure_category": name, "source_image_id": pool[index % len(pool)]["id"]} for index in range(count))
    rng.shuffle(selected)
    fit_images, fit_annotations = [], []
    annotation_id = 1
    for image_id, image in enumerate(selected, 1):
        source_image_id = int(image["source_image_id"])
        fit_images.append({**image, "id": image_id})
        for row in by_image[source_image_id]:
            fit_annotations.append({**row, "id": annotation_id, "image_id": image_id})
            annotation_id += 1
    categories = source["categories"]
    (args.output / "fit.json").write_text(json.dumps({"info": {"description": "RGDRV8 Route A frozen TRAIN exposure"}, "images": fit_images, "annotations": fit_annotations, "categories": categories}, indent=2) + "\n", encoding="utf-8")
    holdout_images, holdout_annotations = remap(load(args.g8_root / "holdout.json"), args.g8_root, args.output, "g8_holdout_selection", 1, 1)
    (args.output / "holdout.json").write_text(json.dumps({"info": {"description": "RGDRV8 Route A HOLDOUT_NEW selection only"}, "images": holdout_images, "annotations": holdout_annotations, "categories": categories}, indent=2) + "\n", encoding="utf-8")
    policy = {"schema_version": 1, "stage": "RGDRV8-02-ROUTE-A-SAMPLING", "ratios": RATIOS, "counts": counts, "source_pool_counts": {name: len(rows) for name, rows in pools.items()}, "epoch_exposures": len(selected), "sampling_with_replacement": True, "seed": 20260813, "proposal_crop_count": len(proposals["images"]), "TRAIN_sources": ["LEGACY_GA1_TRAIN", "G8_TRAIN_NEW", "G8_HARD_NEGATIVE_PROPOSALS"], "selection_data_read_for_training": False, "VAL_NEW_read": False, "G5_V2_read": False, "ROUTE_A_SAMPLING_PASS": all(len(pools[name]) > 0 for name in RATIOS) and len(proposals["images"]) >= 2000}
    (args.output / "ROUTE_A_SAMPLING_POLICY.json").write_text(json.dumps(policy, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(policy, indent=2))
    return 0 if policy["ROUTE_A_SAMPLING_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
