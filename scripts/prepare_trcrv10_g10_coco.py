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
EXPECTED_ROUTE_ID = "g10_route_v19_spin_xneg1p5"
EXPECTED_ROUTE_CONFIG_SHA256 = (
    "1ffc51411821b1350b63e35e081b9fd9280e5ac3372f8827e648b64f48cffe28"
)
TRAIN_WORLD_IDS = {
    "g10v15_train_w01_01_asphalt_campus",
    "g10v15_train_w02_02_concrete_sidewalk",
    "g10v15_train_w03_03_wet_courtyard",
    "g10v15_train_w04_04_cobblestone_arcade",
    "g10v15_train_w05_05_red_brick_promenade",
    "g10v15_train_w06_06_tiled_plaza",
}
HOLDOUT_WORLD_IDS = {
    "g10v15_val_w01_07_service_road",
    "g10v15_val_w02_08_mixed_curb_vegetation",
    "g10v15_val_w03_09_light_paver_pedestrian",
}
SOURCE_SPLIT_CONTRACT = {
    "G10_TRAIN": ("train", TRAIN_WORLD_IDS),
    "G10_HOLDOUT": ("val", HOLDOUT_WORLD_IDS),
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def route_config_sha256(profile: dict) -> str:
    unsigned = dict(profile)
    unsigned.pop("route_config_sha256", None)
    return hashlib.sha256(
        json.dumps(unsigned, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


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
    if declared_source_split not in (None, *SOURCE_SPLIT_CONTRACT):
        raise ValueError(
            "declared_source_split must be G10_TRAIN or G10_HOLDOUT when provided"
        )
    scenes_root = scenes_root.resolve()
    if scenes_root.name != "scenes" or scenes_root.parent.name != "g4_screening_native":
        raise ValueError("scenes_root must be capture_root/g4_screening_native/scenes")
    capture_root = scenes_root.parents[1]

    def relative(path: Path) -> str:
        return path.resolve().relative_to(capture_root).as_posix()

    images, labels = [], []
    image_id = annotation_id = 1
    worlds, seeds, trajectories, capture_source_splits = set(), set(), set(), set()
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
        expected_capture_split = (
            SOURCE_SPLIT_CONTRACT[declared_source_split][0]
            if declared_source_split is not None
            else None
        )
        if expected_capture_split and capture_source_split != expected_capture_split:
            raise RuntimeError(
                f"{declared_source_split} may only be declared from an explicit "
                f"{expected_capture_split} capture: {scene}"
            )
        if (
            declared_source_split is not None
            and manifest.get("world_id") not in SOURCE_SPLIT_CONTRACT[declared_source_split][1]
        ):
            raise RuntimeError(f"{declared_source_split} contains an unapproved world: {scene}")
        contract = manifest.get("trcrv10_g10_approach_sequence", {})
        profile = manifest.get("oprv3_motion_profile", {})
        report_profile = report.get("oprv3_motion_profile")
        if profile != report_profile:
            raise RuntimeError(f"scene/report route profile mismatch: {scene}")
        if (
            contract.get("route_id") != EXPECTED_ROUTE_ID
            or profile.get("route_id") != EXPECTED_ROUTE_ID
            or contract.get("route_config_sha256") != EXPECTED_ROUTE_CONFIG_SHA256
            or profile.get("route_config_sha256") != EXPECTED_ROUTE_CONFIG_SHA256
            or route_config_sha256(profile) != EXPECTED_ROUTE_CONFIG_SHA256
        ):
            raise RuntimeError(f"fixed G10 route identity mismatch: {scene}")
        if contract.get("source_domain_manifest_sha256") != EXPECTED_DOMAIN_MANIFEST_SHA256:
            raise RuntimeError(f"fixed G10 domain identity mismatch: {scene}")
        capture_source_splits.add(capture_source_split)
        worlds.add(manifest["world_id"])
        scene_seed = int(manifest["scene_seed"])
        trajectory_id = manifest.get("trajectory_id")
        expected_trajectory = (
            f"{manifest['world_id']}_{EXPECTED_ROUTE_ID}_trajectory_{scene_seed:04d}"
        )
        if trajectory_id != expected_trajectory:
            raise RuntimeError(f"G10 trajectory identity mismatch: {scene}")
        if scene_seed in seeds:
            raise RuntimeError(f"duplicate G10 scene seed: {scene_seed}")
        if trajectory_id in trajectories:
            raise RuntimeError(f"duplicate G10 trajectory identity: {trajectory_id}")
        seeds.add(scene_seed)
        trajectories.add(trajectory_id)
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
                "scene_seed": scene_seed,
                "trajectory_id": trajectory_id,
                "route_id": EXPECTED_ROUTE_ID,
                "route_config_sha256": EXPECTED_ROUTE_CONFIG_SHA256,
                "g10_domain_manifest_sha256": EXPECTED_DOMAIN_MANIFEST_SHA256,
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
    approved_world_ids = None
    if declared_source_split is not None:
        if domain_manifest is None or not domain_manifest.is_file():
            raise RuntimeError(
                f"{declared_source_split} declaration requires the fixed domain manifest"
            )
        domain_manifest_sha256 = sha256(domain_manifest)
        if domain_manifest_sha256 != EXPECTED_DOMAIN_MANIFEST_SHA256:
            raise RuntimeError("fixed G10 domain manifest SHA-256 mismatch")
        approved_world_ids = SOURCE_SPLIT_CONTRACT[declared_source_split][1]
        if worlds != approved_world_ids:
            raise RuntimeError(
                f"{declared_source_split} must contain the exact approved worlds: "
                f"{sorted(worlds)}"
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
            "route_id": EXPECTED_ROUTE_ID if declared_source_split else None,
            "route_config_sha256": (
                EXPECTED_ROUTE_CONFIG_SHA256 if declared_source_split else None
            ),
            "approved_world_ids": sorted(approved_world_ids) if approved_world_ids else None,
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
    parser.add_argument("--declare-source-split", choices=tuple(SOURCE_SPLIT_CONTRACT))
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
