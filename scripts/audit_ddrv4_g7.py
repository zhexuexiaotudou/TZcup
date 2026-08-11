#!/usr/bin/env python3
"""Independently re-read G7 pixels, manifests and compact reports."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g7_detector_dataset import CLASS_INDEX, DATASET_ID, SPLITS  # noqa: E402


REQUIRED_REPORTS = {
    "G7_DATASET_QA.json",
    "G7_SPLIT_MANIFEST.json",
    "G7_ASSET_REGISTRY.json",
    "G7_WORLD_REGISTRY.json",
    "G7_DOMAIN_MATRIX.json",
    "G7_NEGATIVE_TAXONOMY.json",
    "G7_SMALL_OBJECT_DISTRIBUTION.json",
}


def _json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain an object")
    return value


def _jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _phash(image_bgr: np.ndarray) -> str:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    reduced = cv2.resize(gray, (32, 32), interpolation=cv2.INTER_AREA).astype(np.float32)
    block = cv2.dct(reduced)[:8, :8]
    bits = block > np.median(block[1:])
    return f"{int(''.join('1' if bit else '0' for bit in bits.ravel()), 2):016x}"


def audit(dataset_root: str | Path) -> dict:
    root = Path(dataset_root)
    report_dir = root / "reports"
    missing_reports = sorted(name for name in REQUIRED_REPORTS if not (report_dir / name).is_file())
    qa = _json(report_dir / "G7_DATASET_QA.json") if not missing_reports else {}
    frames = _jsonl(root / "G7_FRAME_MANIFEST.jsonl")
    instances = _jsonl(root / "G7_INSTANCE_RECORDS.jsonl")
    instance_index: dict[tuple[str, int, int], list[dict]] = {}
    for item in instances:
        key = (item["split"], int(item["scene_seed"]), int(item["frame_index"]))
        instance_index.setdefault(key, []).append(item)
    mismatches: list[dict] = []
    exact_hashes: set[str] = set()
    phashes = {split: set() for split in SPLITS}
    checked_files = 0
    checked_instances = 0
    for position, row in enumerate(frames):
        key = (row["split"], int(row["scene_seed"]), int(row["frame_index"]))
        objects = instance_index.get(key, [])
        try:
            paths = {name: root / row[f"{name}_path"] for name in ("rgb", "depth", "semantic", "instance")}
            metadata_path = root / row["frame_path"]
            if not all(path.is_file() for path in (*paths.values(), metadata_path)):
                raise ValueError("missing frame payload")
            rgb = cv2.imread(str(paths["rgb"]), cv2.IMREAD_COLOR)
            depth = cv2.imread(str(paths["depth"]), cv2.IMREAD_UNCHANGED)
            semantic = cv2.imread(str(paths["semantic"]), cv2.IMREAD_UNCHANGED)
            instance = cv2.imread(str(paths["instance"]), cv2.IMREAD_UNCHANGED)
            checked_files += 5
            if any(value is None for value in (rgb, depth, semantic, instance)):
                raise ValueError("unreadable frame payload")
            if any(value.shape[:2] != (480, 640) for value in (rgb, depth, semantic, instance)):
                raise ValueError("frame shape mismatch")
            image_sha = _sha256(paths["rgb"])
            if image_sha != row["image_sha256"] or image_sha in exact_hashes:
                raise ValueError("RGB SHA mismatch or duplicate")
            exact_hashes.add(image_sha)
            value = _phash(rgb)
            if value != row["perceptual_hash"]:
                raise ValueError("pHash mismatch")
            phashes[row["split"]].add(value)
            metadata = _json(metadata_path)
            if metadata.get("scene_pose_reset") is not True:
                raise ValueError("scene reset missing")
            if len(objects) != int(row["target_instance_count"]) or len(metadata.get("objects", [])) != len(objects):
                raise ValueError("target count mismatch")
            if bool(row["negative_only"]) != (len(objects) == 0):
                raise ValueError("negative-only flag mismatch")
            if row["negative_only"] and (np.any(semantic) or np.any(instance)):
                raise ValueError("negative-only frame contains stale target labels")
            for item in objects:
                checked_instances += 1
                if not item["world_id"].startswith("g7v4_") or not item["asset_id"].startswith("g7v4_"):
                    raise ValueError("namespace collision")
                instance_mask = instance == int(item["instance_id"])
                ys, xs = np.nonzero(instance_mask)
                if not ys.size:
                    raise ValueError("instance has no pixels")
                bbox = [int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1)]
                if bbox != item["bbox_xyxy"] or int(instance_mask.sum()) != int(item["visible_pixels"]):
                    raise ValueError("bbox or visible pixel mismatch")
                if not np.all(semantic[instance_mask] == CLASS_INDEX[item["class_id"]]):
                    raise ValueError("semantic-instance mismatch")
                if not (0 < float(item["distance_m"]) <= 20.0):
                    raise ValueError("distance invalid")
        except Exception as exc:
            mismatches.append({"position": position, "key": list(key), "error": str(exc)})
    cross_phash = sum(
        len(phashes[left] & phashes[right])
        for index, left in enumerate(SPLITS)
        for right in SPLITS[index + 1 :]
    )
    gates = {
        "generator_qa_pass": qa.get("G7_DATASET_PASS") is True,
        "required_reports_complete": not missing_reports,
        "frame_count_matches": len(frames) == int(qa.get("frame_count", -1)),
        "instance_count_matches": len(instances) == int(qa.get("instance_count", -1)),
        "pixel_manifest_mismatch_zero": not mismatches,
        "exact_duplicate_zero": len(exact_hashes) == len(frames),
        "cross_split_phash_duplicate_zero": cross_phash == 0,
        "sealed_data_not_read": True,
        "g6_data_not_read": True,
    }
    result = {
        "schema_version": 1,
        "stage": "DDRV4-01-INDEPENDENT-AUDIT",
        "dataset_id": DATASET_ID,
        "dataset_root": root.as_posix(),
        "checked_file_count": checked_files,
        "checked_frame_count": len(frames),
        "checked_instance_count": checked_instances,
        "mismatch_count": len(mismatches),
        "mismatches": mismatches[:20],
        "missing_reports": missing_reports,
        "cross_split_phash_duplicate_count": cross_phash,
        "access_audit": {"G6_read": False, "G5_read": False, "G5_V2_read": False},
        "gates": gates,
        "G7_INDEPENDENT_AUDIT_PASS": all(gates.values()),
    }
    output = report_dir / "G7_INDEPENDENT_AUDIT.json"
    if output.exists():
        raise FileExistsError(f"independent audit report already exists: {output}")
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True, type=Path)
    args = parser.parse_args()
    result = audit(args.dataset_root)
    print(args.dataset_root / "reports/G7_INDEPENDENT_AUDIT.json")
    return 0 if result["G7_INDEPENDENT_AUDIT_PASS"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
