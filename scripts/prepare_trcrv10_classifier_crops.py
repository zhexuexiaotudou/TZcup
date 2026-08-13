#!/usr/bin/env python3
"""Build the five-view G10 TRAIN crop set for the close-range classifier."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2

from evaluate_trcrv10_proposals import iou


CLASSES = {1: "plastic_bottle", 2: "metal_can", 3: "paper_litter"}
POSITIVE_VIEWS = ("gt", "detector_jitter", "proposal", "partial", "context")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clip(box: list[float], width: int, height: int) -> list[int]:
    x1, y1, x2, y2 = box
    return [
        max(0, min(width - 1, int(round(x1)))),
        max(0, min(height - 1, int(round(y1)))),
        max(1, min(width, int(round(x2)))),
        max(1, min(height, int(round(y2)))),
    ]


def expand(box: list[float], scale: float) -> list[float]:
    x1, y1, x2, y2 = box
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    half_w, half_h = (x2 - x1) * scale / 2, (y2 - y1) * scale / 2
    return [cx - half_w, cy - half_h, cx + half_w, cy + half_h]


def jitter(box: list[float]) -> list[float]:
    x1, y1, x2, y2 = box
    width, height = x2 - x1, y2 - y1
    return [x1 - .06 * width, y1 + .04 * height, x2 + .03 * width, y2 - .02 * height]


def partial(box: list[float]) -> list[float]:
    x1, y1, x2, y2 = box
    return [x1, y1, x2 - .18 * (x2 - x1), y2]


def write_crop(image, box: list[float], path: Path) -> None:
    height, width = image.shape[:2]
    x1, y1, x2, y2 = clip(box, width, height)
    if x2 <= x1 or y2 <= y1:
        raise ValueError(f"empty crop: {box}")
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image[y1:y2, x1:x2]):
        raise RuntimeError(f"failed to write crop: {path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--coco", type=Path, required=True)
    parser.add_argument("--raw-inference", type=Path, required=True)
    parser.add_argument("--threshold", type=float, required=True)
    parser.add_argument("--min-reliable-short-side", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    coco = json.loads(args.coco.read_text(encoding="utf-8"))
    raw = json.loads(args.raw_inference.read_text(encoding="utf-8"))
    if not coco["images"] or {row.get("source_split") for row in coco["images"]} != {"train"}:
        raise ValueError("classifier crops accept G10 TRAIN only")
    if args.min_reliable_short_side < 18:
        raise ValueError("MIN_RELIABLE_CLASSIFICATION_SHORT_SIDE_PX must be frozen and >=18")
    images = {int(row["id"]): row for row in coco["images"]}
    annotations = {}
    for row in coco["annotations"]:
        x, y, width, height = row["bbox"]
        annotations.setdefault(int(row["image_id"]), []).append({
            "class_id": CLASSES[int(row["category_id"])],
            "bbox": [x, y, x + width, y + height],
            "short_side_px": int(row["bbox_short_side_px"]),
        })
    predictions = {int(row["image_id"]): row["detections"] for row in raw["frames"]}
    rows = []
    for image_id, meta in sorted(images.items()):
        image = cv2.imread(meta["file_name"], cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"unreadable RGB: {meta['file_name']}")
        truth = annotations.get(image_id, [])
        proposals = [row for row in predictions.get(image_id, []) if row["score"] >= args.threshold]
        used = set()
        for target_index, target in enumerate(truth):
            if target["short_side_px"] < args.min_reliable_short_side:
                continue
            matches = sorted(
                ((iou(target["bbox"], row["bbox_xyxy"]), index, row) for index, row in enumerate(proposals)),
                reverse=True, key=lambda value: value[0],
            )
            proposal_box = target["bbox"]
            if matches and matches[0][0] >= .5:
                used.add(matches[0][1])
                proposal_box = matches[0][2]["bbox_xyxy"]
            boxes = {
                "gt": target["bbox"],
                "detector_jitter": jitter(target["bbox"]),
                "proposal": proposal_box,
                "partial": partial(proposal_box),
                "context": expand(proposal_box, 1.6),
            }
            for view in POSITIVE_VIEWS:
                relative = Path(target["class_id"]) / view / f"{meta['scene']}_{meta['frame_index']:03d}_{target_index}.png"
                write_crop(image, boxes[view], args.output / relative)
                rows.append({
                    "path": relative.as_posix(), "class_id": target["class_id"], "view": view,
                    "scene": meta["scene"], "frame_index": meta["frame_index"], "world_id": meta["world_id"],
                    "scene_seed": meta["scene_seed"], "source_split": "G10_TRAIN",
                    "source_bbox_xyxy": boxes[view], "production_runtime_eligible": view in {"proposal", "partial", "context"},
                })
        for proposal_index, proposal in enumerate(proposals):
            if proposal_index in used or any(iou(proposal["bbox_xyxy"], target["bbox"]) >= .5 for target in truth):
                continue
            relative = Path("background_or_unknown") / "proposal_false_positive" / f"{meta['scene']}_{meta['frame_index']:03d}_{proposal_index}.png"
            write_crop(image, proposal["bbox_xyxy"], args.output / relative)
            rows.append({
                "path": relative.as_posix(), "class_id": "background_or_unknown", "view": "proposal_false_positive",
                "scene": meta["scene"], "frame_index": meta["frame_index"], "world_id": meta["world_id"],
                "scene_seed": meta["scene_seed"], "source_split": "G10_TRAIN",
                "source_bbox_xyxy": proposal["bbox_xyxy"], "production_runtime_eligible": True,
            })
    counts = {class_id: sum(row["class_id"] == class_id for row in rows) for class_id in (*CLASSES.values(), "background_or_unknown")}
    payload = {
        "schema_version": 1, "protocol": "TRCRV10", "stage": "TRCRV10-04-CLASSIFIER-TRAIN-CROPS",
        "coco_sha256": sha256(args.coco), "raw_inference_sha256": sha256(args.raw_inference),
        "threshold": args.threshold, "MIN_RELIABLE_CLASSIFICATION_SHORT_SIDE_PX": args.min_reliable_short_side,
        "positive_views": list(POSITIVE_VIEWS), "counts": counts, "rows": rows,
        "HOLDOUT_read": False, "G10_DEV_VAL_SEALED_read": False, "VAL_NEW_read": False, "G5_V2_read": False,
        "pass": bool(rows) and all(counts[class_id] > 0 for class_id in CLASSES.values()) and counts["background_or_unknown"] > 0,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "CLASSIFIER_TRAIN_CROP_MANIFEST.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"counts": counts, "pass": payload["pass"]}, indent=2))
    return 0 if payload["pass"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
