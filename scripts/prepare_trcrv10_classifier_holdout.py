#!/usr/bin/env python3
"""Build runtime-faithful tight/context proposal crops for G10 HOLDOUT scoring."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from evaluate_trcrv10_proposals import iou
from prepare_trcrv10_classifier_crops import CLASSES, expand, write_crop


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def size_bucket(short_side: float) -> str:
    for name, low, high in (("lt18", 0, 18), ("18_32", 18, 32), ("32_64", 32, 64), ("64_96", 64, 96), ("ge96", 96, float("inf"))):
        if low <= short_side < high:
            return name
    raise AssertionError(short_side)


def distance_bucket(distance_m: float | None) -> str:
    if distance_m is None:
        return "invalid_depth"
    for name, low, high in (("lt1", 0, 1), ("1_2", 1, 2), ("2_4", 2, 4), ("ge4", 4, float("inf"))):
        if low <= distance_m < high:
            return name
    return "invalid_depth"


def median_depth(depth: np.ndarray, box: list[float]) -> float | None:
    height, width = depth.shape[:2]
    x1, y1, x2, y2 = [int(round(value)) for value in box]
    x1, x2 = max(0, min(width - 1, x1)), max(1, min(width, x2))
    y1, y2 = max(0, min(height - 1, y1)), max(1, min(height, y2))
    values = depth[y1:y2, x1:x2]
    values = values[np.isfinite(values) & (values > 0)]
    return float(np.median(values)) if values.size else None


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
    if not coco["images"] or {row.get("source_split") for row in coco["images"]} != {"val"}:
        raise ValueError("classifier HOLDOUT crops accept G10 val/HOLDOUT only")
    if args.min_reliable_short_side < 18:
        raise ValueError("MIN_RELIABLE_CLASSIFICATION_SHORT_SIDE_PX must be frozen and >=18")
    images = {int(row["id"]): row for row in coco["images"]}
    annotations = {}
    for row in coco["annotations"]:
        x, y, width, height = row["bbox"]
        annotations.setdefault(int(row["image_id"]), []).append({
            "class_id": CLASSES[int(row["category_id"])], "bbox": [x, y, x + width, y + height],
            "short_side_px": int(row["bbox_short_side_px"]),
        })
    predictions = {int(row["image_id"]): row["detections"] for row in raw["frames"]}
    rows = []
    for image_id, meta in sorted(images.items()):
        image = cv2.imread(meta["file_name"], cv2.IMREAD_COLOR)
        depth = np.load(meta["depth_file_name"])
        if image is None:
            raise ValueError(f"unreadable RGB: {meta['file_name']}")
        truth = annotations.get(image_id, [])
        for proposal_index, proposal in enumerate(predictions.get(image_id, [])):
            if proposal["score"] < args.threshold:
                continue
            box = proposal["bbox_xyxy"]
            short_side = min(box[2] - box[0], box[3] - box[1])
            if short_side < args.min_reliable_short_side:
                continue
            matches = sorted(((iou(box, target["bbox"]), target) for target in truth), reverse=True, key=lambda value: value[0])
            matched = matches[0][1] if matches and matches[0][0] >= .5 else None
            class_id = matched["class_id"] if matched else "background_or_unknown"
            depth_m = median_depth(depth, box)
            for view, crop_box in (("tight", box), ("context", expand(box, 1.6))):
                relative = Path(class_id) / view / f"{meta['scene']}_{meta['frame_index']:03d}_{proposal_index}.png"
                write_crop(image, crop_box, args.output / relative)
                rows.append({
                    "path": relative.as_posix(), "class_id": class_id, "view": view,
                    "scene": meta["scene"], "frame_index": meta["frame_index"], "world_id": meta["world_id"],
                    "scene_seed": meta["scene_seed"], "source_split": "G10_HOLDOUT",
                    "source_bbox_xyxy": box, "proposal_score": proposal["score"], "size_bucket": size_bucket(short_side),
                    "distance_m": depth_m, "distance_bucket": distance_bucket(depth_m),
                    "occlusion_bucket": "not_available_from_product_inputs",
                    "gt_role": "offline_label_assignment_only", "production_runtime_eligible": True,
                })
    counts = {class_id: sum(row["class_id"] == class_id for row in rows) for class_id in (*CLASSES.values(), "background_or_unknown")}
    payload = {
        "schema_version": 1, "protocol": "TRCRV10", "stage": "TRCRV10-04-CLASSIFIER-HOLDOUT-CROPS",
        "coco_sha256": sha256(args.coco), "raw_inference_sha256": sha256(args.raw_inference),
        "threshold": args.threshold, "MIN_RELIABLE_CLASSIFICATION_SHORT_SIDE_PX": args.min_reliable_short_side,
        "counts": counts, "rows": rows, "gt_role": "offline_evaluator_only", "production_runtime_gt_used": False,
        "G10_DEV_VAL_SEALED_read": False, "VAL_NEW_read": False, "G5_V2_read": False,
        "pass": bool(rows) and all(counts[class_id] > 0 for class_id in (*CLASSES.values(), "background_or_unknown")),
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "CLASSIFIER_HOLDOUT_CROP_MANIFEST.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"counts": counts, "pass": payload["pass"]}, indent=2))
    return 0 if payload["pass"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
