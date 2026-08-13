#!/usr/bin/env python3
"""Index captured G10 frames for detector inference and offline GT evaluation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


CATEGORIES = (
    {"id": 1, "name": "plastic_bottle"},
    {"id": 2, "name": "metal_can"},
    {"id": 3, "name": "paper_litter"},
)


def annotations(mask: np.ndarray, image_id: int, next_id: int) -> tuple[list[dict], int]:
    rows = []
    for category in CATEGORIES:
        label = category["id"]
        ys, xs = np.where(mask == label)
        if not len(xs):
            continue
        x1, y1, x2, y2 = int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1)
        width, height = x2 - x1, y2 - y1
        rows.append({
            "id": next_id,
            "image_id": image_id,
            "category_id": label,
            "bbox": [x1, y1, width, height],
            "area": int(len(xs)),
            "iscrowd": 0,
            "bbox_short_side_px": min(width, height),
        })
        next_id += 1
    return rows, next_id


def build(scenes_root: Path) -> dict:
    images, labels = [], []
    image_id = annotation_id = 1
    worlds, seeds = set(), set()
    for scene in sorted(path for path in scenes_root.glob("scene_*") if path.is_dir()):
        manifest = json.loads((scene / "scene_manifest.json").read_text(encoding="utf-8"))
        report = json.loads((scene / "capture_report.json").read_text(encoding="utf-8"))
        if report.get("capture_pass") is not True:
            raise RuntimeError(f"capture did not pass: {scene}")
        if manifest.get("trcrv10_g10_approach_sequence", {}).get("gt_runtime_forbidden") is not True:
            raise RuntimeError(f"GT runtime contract missing: {scene}")
        worlds.add(manifest["world_id"])
        seeds.add(int(manifest["scene_seed"]))
        for index in range(int(report["captured_frames"])):
            rgb = scene / "rgb" / f"frame_{index:02d}.png"
            semantic = scene / "semantic" / f"frame_{index:02d}.npy"
            mask = np.load(semantic)
            height, width = mask.shape[:2]
            images.append({
                "id": image_id,
                "file_name": str(rgb.resolve()),
                "width": int(width),
                "height": int(height),
                "scene": scene.name,
                "mission_id": scene.name,
                "frame_index": index,
                "negative_only": bool(manifest["negative_only"]),
                "world_id": manifest["world_id"],
                "scene_seed": int(manifest["scene_seed"]),
            })
            new_rows, annotation_id = annotations(mask, image_id, annotation_id)
            labels.extend(new_rows)
            image_id += 1
    return {
        "info": {
            "protocol": "TRCRV10",
            "semantic_gt_role": "offline_evaluator_and_training_index_only",
            "production_runtime_gt_used": False,
        },
        "images": images,
        "annotations": labels,
        "categories": list(CATEGORIES),
        "qa": {
            "scenes": len({row["scene"] for row in images}),
            "frames": len(images),
            "annotations": len(labels),
            "worlds": len(worlds),
            "seeds": len(seeds),
            "duplicate_image_ids_zero": len({row["id"] for row in images}) == len(images),
            "duplicate_annotation_ids_zero": len({row["id"] for row in labels}) == len(labels),
        },
        "G10_DEV_VAL_SEALED_read": False,
        "VAL_NEW_read": False,
        "G5_V2_read": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenes", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = build(args.scenes)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload["qa"], indent=2))
    return 0 if payload["images"] and all(
        payload["qa"][key] for key in ("duplicate_image_ids_zero", "duplicate_annotation_ids_zero")
    ) else 2


if __name__ == "__main__":
    raise SystemExit(main())
