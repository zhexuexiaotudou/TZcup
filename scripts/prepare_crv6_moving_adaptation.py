#!/usr/bin/env python3
"""Prepare COCO TRAIN/HOLDOUT annotations from the frozen G7-MOVING pack."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

CLASSES = ("plastic_bottle", "metal_can", "paper_litter")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def coco(root: Path, split: str) -> dict:
    rows = [row for row in jsonl(root / "G7_MOVING_FRAME_MANIFEST.jsonl") if row["split"] == split]
    images, annotations = [], []
    ann_id = 1
    for image_id, row in enumerate(rows, 1):
        images.append({"id": image_id, "file_name": row["rgb_path"], "width": 640, "height": 480, "mission_id": row["mission_id"], "frame_index": row["frame_index"]})
        gt = json.loads((root / row["evaluator_gt_path"]).read_text(encoding="utf-8"))
        for item in gt["objects"]:
            x1, y1, x2, y2 = item["bbox_xyxy"]
            annotations.append({"id": ann_id, "image_id": image_id, "category_id": CLASSES.index(item["class_name"]) + 1, "bbox": [x1, y1, x2-x1, y2-y1], "area": (x2-x1)*(y2-y1), "iscrowd": 0, "bbox_short_side_px": item["bbox_short_side_px"], "target_id": item["target_id"]})
            ann_id += 1
    return {"info": {"description": f"G7_MOVING_DEVELOPMENT {split}"}, "licenses": [], "categories": [{"id": i+1, "name": name, "supercategory": "litter"} for i, name in enumerate(CLASSES)], "images": images, "annotations": annotations}


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--data-root", required=True, type=Path); parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    outputs = {}
    for split, name in (("MOVING_TRAIN", "train.json"), ("MOVING_HOLDOUT", "holdout.json")):
        path = args.output / name; path.write_text(json.dumps(coco(args.data_root, split), indent=2) + "\n", encoding="utf-8"); outputs[name] = sha256(path)
    report = {"schema_version": 1, "stage": "CRV6-05-PREP", "dataset": "G7_MOVING_DEVELOPMENT", "training_split": "MOVING_TRAIN", "selection_split": "MOVING_HOLDOUT", "MOVING_VAL_read": False, "G5_read": False, "G5_V2_read": False, "sha256": outputs}
    (args.output / "CRV6_MOVING_ADAPTATION_PREP.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__": raise SystemExit(main())
