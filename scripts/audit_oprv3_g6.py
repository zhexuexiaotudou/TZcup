#!/usr/bin/env python3
"""Independently re-read and audit a generated OPRV3 G6 corpus."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

cv2.setNumThreads(1)


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g6_dataset import (  # noqa: E402
    HEIGHT,
    WIDTH,
    _bucket_for_short_side,
    _phash,
    load_jsonl,
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def audit_frame(package: tuple[Path, dict, list[dict]]) -> dict:
    root, row, current = package
    key = (int(row["scene_seed"]), int(row["frame_index"]))
    mismatches: list[dict] = []
    paths = {
        name: root / row[name]
        for name in (
            "rgb_path",
            "depth_path",
            "semantic_path",
            "instance_path",
            "frame_path",
        )
    }
    missing = [name for name, path in paths.items() if not path.is_file()]
    if missing:
        return {
            "key": key,
            "split": row["split"],
            "mismatches": [{"key": key, "missing": missing}],
            "checked_files": 0,
            "checked_instances": 0,
            "image_hash": None,
            "phash": None,
        }
    rgb_bgr = cv2.imread(str(paths["rgb_path"]), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(paths["depth_path"]), cv2.IMREAD_UNCHANGED)
    semantic = cv2.imread(str(paths["semantic_path"]), cv2.IMREAD_UNCHANGED)
    instance = cv2.imread(str(paths["instance_path"]), cv2.IMREAD_UNCHANGED)
    if any(item is None for item in (rgb_bgr, depth, semantic, instance)):
        mismatches.append({"key": key, "decode_failure": True})
        return {
            "key": key,
            "split": row["split"],
            "mismatches": mismatches,
            "checked_files": 5,
            "checked_instances": 0,
            "image_hash": None,
            "phash": None,
        }
    if any(
        item.shape[:2] != (HEIGHT, WIDTH)
        for item in (rgb_bgr, depth, semantic, instance)
    ):
        mismatches.append({"key": key, "shape_failure": True})
        return {
            "key": key,
            "split": row["split"],
            "mismatches": mismatches,
            "checked_files": 5,
            "checked_instances": 0,
            "image_hash": None,
            "phash": None,
        }
    rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)
    actual_hash = sha256(paths["rgb_path"])
    actual_phash = _phash(rgb)
    if actual_hash != row["image_sha256"] or actual_phash != row["perceptual_hash"]:
        mismatches.append({"key": key, "image_hash_failure": True})
    payload = json.loads(paths["frame_path"].read_text(encoding="utf-8"))
    if len(current) != int(row["target_instance_count"]):
        mismatches.append({"key": key, "target_count_failure": True})
    if not payload.get("scene_pose_reset"):
        mismatches.append({"key": key, "pose_reset_failure": True})
    if not payload.get("camera", {}).get("timestamp_ns") or not payload.get("tf"):
        mismatches.append({"key": key, "sensor_contract_failure": True})
    for record in current:
        selected = instance == int(record["instance_id"])
        rows_px, cols_px = np.nonzero(selected)
        if not rows_px.size:
            mismatches.append(
                {"key": key, "instance_id": record["instance_id"], "invisible": True}
            )
            continue
        bbox = [
            int(cols_px.min()),
            int(rows_px.min()),
            int(cols_px.max() + 1),
            int(rows_px.max() + 1),
        ]
        short_side = min(bbox[2] - bbox[0], bbox[3] - bbox[1])
        valid = (
            bbox == record["bbox_xyxy"]
            and int(selected.sum()) == int(record["mask_area_px"])
            and np.all(semantic[selected] == int(record["class_index"]))
            and short_side == int(record["bbox_short_side_px"])
            and _bucket_for_short_side(short_side) == record["short_side_bucket"]
            and not record["truncated"]
            and record["visible"]
        )
        if not valid:
            mismatches.append(
                {
                    "key": key,
                    "instance_id": record["instance_id"],
                    "pixel_contract_failure": True,
                }
            )
    if row["negative_only"] and np.any(semantic):
        mismatches.append({"key": key, "negative_only_stale_positive": True})
    if np.any((semantic > 0) & (instance == 0)):
        mismatches.append({"key": key, "unowned_semantic_positive": True})
    return {
        "key": key,
        "split": row["split"],
        "mismatches": mismatches,
        "checked_files": 5,
        "checked_instances": len(current),
        "image_hash": actual_hash,
        "phash": actual_phash,
    }


def audit(root: Path) -> dict:
    rows = load_jsonl(root / "G6_FRAME_MANIFEST.jsonl")
    records = load_jsonl(root / "G6_INSTANCE_RECORDS.jsonl")
    by_key: dict[tuple[int, int], list[dict]] = {}
    for record in records:
        by_key.setdefault(
            (int(record["scene_seed"]), int(record["frame_index"])), []
        ).append(record)

    exact_hashes: set[str] = set()
    phashes_by_split: dict[str, set[str]] = {}
    mismatches: list[dict] = []
    checked_instances = 0
    checked_files = 0
    packages = [
        (
            root,
            row,
            by_key.get((int(row["scene_seed"]), int(row["frame_index"])), []),
        )
        for row in rows
    ]
    with ThreadPoolExecutor(max_workers=8) as executor:
        results = executor.map(audit_frame, packages, chunksize=16)
        for position, result in enumerate(results, 1):
            mismatches.extend(result["mismatches"])
            checked_files += result["checked_files"]
            checked_instances += result["checked_instances"]
            actual_hash = result["image_hash"]
            actual_phash = result["phash"]
            split = result["split"]
            key = result["key"]
            if actual_hash is None or actual_phash is None:
                continue
            if actual_hash in exact_hashes:
                mismatches.append({"key": key, "exact_duplicate": True})
            exact_hashes.add(actual_hash)
            other_phashes = set().union(
                *(
                    values
                    for other, values in phashes_by_split.items()
                    if other != split
                )
            )
            if actual_phash in other_phashes:
                mismatches.append({"key": key, "cross_split_phash_duplicate": True})
            phashes_by_split.setdefault(split, set()).add(actual_phash)
            if position % 1000 == 0:
                print(f"[G6 audit] {position}/{len(rows)} frames", flush=True)

    split_report = json.loads(
        (root / "reports/G6_SPLIT_MANIFEST.json").read_text(encoding="utf-8")
    )
    gates = {
        "frame_manifest_exactly_8000": len(rows) == 8000,
        "all_40000_payload_files_read": checked_files == len(rows) * 5,
        "all_instance_records_pixel_verified": checked_instances == len(records),
        "pixel_contract_mismatch_zero": not mismatches,
        "exact_duplicate_zero": len(exact_hashes) == len(rows),
        "cross_split_phash_duplicate_zero": all(
            not (phashes_by_split[left] & phashes_by_split[right])
            for index, left in enumerate(phashes_by_split)
            for right in list(phashes_by_split)[index + 1 :]
        ),
        "world_overlap_zero": split_report["cross_world_overlap"] == 0,
        "asset_overlap_zero": split_report["cross_asset_overlap"] == 0,
        "sealed_final_not_read": True,
    }
    result = {
        "schema_version": 1,
        "stage": "OPRV3-04-G6-INDEPENDENT-AUDIT",
        "dataset_root": root.as_posix(),
        "frame_count": len(rows),
        "instance_count": len(records),
        "checked_file_count": checked_files,
        "mismatch_count": len(mismatches),
        "mismatch_examples": mismatches[:20],
        "gates": gates,
        "G6_INDEPENDENT_AUDIT_PASS": all(gates.values()),
        "sealed_final_read": False,
    }
    output = root / "G6_INDEPENDENT_AUDIT.json"
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.dataset_root)
    print(args.dataset_root / "G6_INDEPENDENT_AUDIT.json")
    return 0 if result["G6_INDEPENDENT_AUDIT_PASS"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
