#!/usr/bin/env python3
"""Index captured G10 frames for detector inference and offline GT evaluation."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

CATEGORIES = (
    {"id": 1, "name": "plastic_bottle"},
    {"id": 2, "name": "metal_can"},
    {"id": 3, "name": "paper_litter"},
)
EXPECTED_DOMAIN_MANIFEST_SHA256 = (
    "3bdb3006226943e4149cd84144b488e5eb112ab35ad3692c5da8cc48c88b5208"
)
HOLDOUT_WORLD_IDS = {
    "g10v15_val_w01_07_service_road",
    "g10v15_val_w02_08_mixed_curb_vegetation",
    "g10v15_val_w03_09_light_paver_pedestrian",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


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
            "area": len(xs),
            "iscrowd": 0,
            "bbox_short_side_px": min(width, height),
        })
        next_id += 1
    return rows, next_id


def build(
    scenes_root: Path,
    *,
    declared_source_split: str | None = None,
    domain_manifest: Path | None = None,
) -> dict:
    if declared_source_split not in (None, "G10_HOLDOUT"):
        raise ValueError("declared_source_split must be G10_HOLDOUT when provided")
    scenes_root = scenes_root.resolve()
    if scenes_root.name != "scenes" or scenes_root.parent.name != "g4_screening_native":
        raise ValueError("scenes_root must be capture_root/g4_screening_native/scenes")
    capture_root = scenes_root.parents[1]

    def relative(path: Path) -> str:
        return path.resolve().relative_to(capture_root).as_posix()

    images, labels = [], []
    image_id = annotation_id = 1
    worlds, seeds, capture_source_splits = set(), set(), set()
    for scene in sorted(path for path in scenes_root.glob("scene_*") if path.is_dir()):
        manifest = json.loads((scene / "scene_manifest.json").read_text(encoding="utf-8"))
        report = json.loads((scene / "capture_report.json").read_text(encoding="utf-8"))
        if report.get("capture_pass") is not True:
            raise RuntimeError(f"capture did not pass: {scene}")
        if manifest.get("trcrv10_g10_approach_sequence", {}).get("gt_runtime_forbidden") is not True:
            raise RuntimeError(f"GT runtime contract missing: {scene}")
        capture_source_split = manifest.get("split")
        if not isinstance(capture_source_split, str) or not capture_source_split:
            raise RuntimeError(f"capture split missing: {scene}")
        if declared_source_split == "G10_HOLDOUT" and capture_source_split != "val":
            raise RuntimeError(
                f"G10_HOLDOUT may only be declared from an explicit val capture: {scene}"
            )
        capture_source_splits.add(capture_source_split)
        worlds.add(manifest["world_id"])
        seeds.add(int(manifest["scene_seed"]))
        for index in range(int(report["captured_frames"])):
            rgb = scene / "rgb" / f"frame_{index:02d}.png"
            semantic = scene / "semantic" / f"frame_{index:02d}.npy"
            mask = np.load(semantic)
            height, width = mask.shape[:2]
            images.append({
                "id": image_id,
                "file_name": relative(rgb),
                "depth_file_name": relative(scene / "depth" / f"frame_{index:02d}.npy"),
                "camera_file_name": relative(scene / "camera" / f"frame_{index:02d}.json"),
                "width": int(width),
                "height": int(height),
                "scene": scene.name,
                "mission_id": scene.name,
                "frame_index": index,
                "negative_only": bool(manifest["negative_only"]),
                "source_split": declared_source_split or capture_source_split,
                "capture_source_split": capture_source_split,
                "world_id": manifest["world_id"],
                "world_sha256": manifest["world_sha256"],
                "scene_seed": int(manifest["scene_seed"]),
                "trajectory_id": manifest["trajectory_id"],
                "scene_manifest": relative(scene / "scene_manifest.json"),
                "capture_report": relative(scene / "capture_report.json"),
                "semantic_file_name": relative(semantic),
                "instance_file_name": relative(
                    scene / "instance" / f"frame_{index:02d}.npy"
                ),
            })
            new_rows, annotation_id = annotations(mask, image_id, annotation_id)
            labels.extend(new_rows)
            image_id += 1
    domain_manifest_sha256 = None
    if declared_source_split == "G10_HOLDOUT":
        if domain_manifest is None or not domain_manifest.is_file():
            raise RuntimeError("G10_HOLDOUT declaration requires the fixed domain manifest")
        domain_manifest_sha256 = sha256(domain_manifest)
        if domain_manifest_sha256 != EXPECTED_DOMAIN_MANIFEST_SHA256:
            raise RuntimeError("fixed G10 domain manifest SHA-256 mismatch")
        if worlds != HOLDOUT_WORLD_IDS:
            raise RuntimeError(
                f"G10_HOLDOUT must contain the exact three approved worlds: {sorted(worlds)}"
            )
    return {
        "info": {
            "protocol": "TRCRV10",
            "semantic_gt_role": "offline_evaluator_and_training_index_only",
            "production_runtime_gt_used": False,
            "declared_source_split": declared_source_split,
            "capture_source_splits": sorted(capture_source_splits),
            "source_split_declaration_explicit": declared_source_split is not None,
            "path_contract": "capture_root_relative_posix",
            "g10_domain_manifest_sha256": domain_manifest_sha256,
            "holdout_world_ids": (
                sorted(HOLDOUT_WORLD_IDS)
                if declared_source_split == "G10_HOLDOUT"
                else None
            ),
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
    parser.add_argument("--declare-source-split", choices=("G10_HOLDOUT",))
    parser.add_argument("--domain-manifest", type=Path)
    args = parser.parse_args()
    payload = build(
        args.scenes,
        declared_source_split=args.declare_source_split,
        domain_manifest=args.domain_manifest,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload["qa"], indent=2))
    return 0 if payload["images"] and all(
        payload["qa"][key] for key in ("duplicate_image_ids_zero", "duplicate_annotation_ids_zero")
    ) else 2


if __name__ == "__main__":
    raise SystemExit(main())
