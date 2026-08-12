#!/usr/bin/env python3
"""Re-crop fixed Route B proposals with deployable Route C context.

The proposal coordinates and labels are unchanged.  Only the crop operator is
changed from a narrow detector crop to the same deterministic square/context
operator used by the product verifier.  HOLDOUT proposals remain fixed once;
VAL_NEW is deliberately not accepted by this command.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import sys

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
from sanitation_learning.g4_data import square_crop  # noqa: E402


def image_index(coco_path: Path, image_root: Path) -> dict[int, Path]:
    payload = json.loads(coco_path.read_text())
    return {
        int(row["id"]): image_root / row["file_name"]
        for row in payload["images"]
    }


def recrop(rows: list[dict], images: dict[int, Path], output: Path, split: str) -> list[dict]:
    target = output / "images" / split.lower()
    target.mkdir(parents=True, exist_ok=True)
    result = []
    cached_id = None
    cached = None
    for position, row in enumerate(rows, 1):
        image_id = int(row["source_image_id"])
        if image_id != cached_id:
            cached = cv2.imread(str(images[image_id]), cv2.IMREAD_COLOR)
            if cached is None:
                raise RuntimeError(f"unable to read source image {images[image_id]}")
            cached_id = image_id
        assert cached is not None
        height, width = cached.shape[:2]
        box = square_crop(
            width,
            height,
            tuple(float(value) for value in row["proposal_bbox_xyxy"]),
            scale=6.0,
            minimum_side=64,
        )
        x0, y0, x1, y1 = box
        crop = cached[y0:y1, x0:x1]
        if not crop.size:
            raise RuntimeError(f"empty contextual crop for proposal {row['id']}")
        crop_path = target / f"crop_{position:07d}.png"
        if not cv2.imwrite(str(crop_path), crop):
            raise RuntimeError(f"unable to write {crop_path}")
        result.append(
            {
                **row,
                "id": position,
                "split": split,
                "crop_path": crop_path.relative_to(output).as_posix(),
                "context_bbox_xyxy": list(box),
                "context_scale": 6.0,
                "minimum_side": 64,
                "proposal_source": "fixed_route_b_proposals",
            }
        )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--route-b-crops", type=Path, required=True)
    parser.add_argument("--prepared", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    source_report = json.loads((args.route_b_crops / "ROUTE_B_CROP_REPORT.json").read_text())
    if not source_report["HOLDOUT_proposals_fixed_once"] or source_report["VAL_NEW_read"]:
        raise RuntimeError("Route C requires fixed HOLDOUT and unread VAL_NEW")
    args.output.mkdir(parents=True)
    train = recrop(
        json.loads((args.route_b_crops / "train_crops.json").read_text()),
        image_index(args.prepared / "fit.json", args.prepared),
        args.output,
        "TRAIN",
    )
    holdout = recrop(
        json.loads((args.route_b_crops / "holdout_new_crops.json").read_text()),
        image_index(args.prepared / "holdout.json", args.prepared),
        args.output,
        "HOLDOUT_NEW",
    )
    (args.output / "train_crops.json").write_text(json.dumps(train, indent=2) + "\n")
    (args.output / "holdout_new_crops.json").write_text(json.dumps(holdout, indent=2) + "\n")
    report = {
        "schema_version": 1,
        "stage": "RGDRV8-04-ROUTE-C-CONTEXT-CROPS",
        "operator": "square_crop_scale_6_minimum_64",
        "proposal_coordinates_changed": False,
        "proposal_labels_changed": False,
        "source_counts": {
            "TRAIN": dict(Counter(row["class_name"] for row in train)),
            "HOLDOUT_NEW": dict(Counter(row["class_name"] for row in holdout)),
        },
        "HOLDOUT_proposals_fixed_once": True,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
        "ROUTE_C_CONTEXT_CROPS_PASS": len(train) >= 6000 and len(holdout) >= 1000,
    }
    (args.output / "ROUTE_C_CONTEXT_CROP_REPORT.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    print(json.dumps(report, indent=2))
    return 0 if report["ROUTE_C_CONTEXT_CROPS_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
