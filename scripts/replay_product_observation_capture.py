"""Offline replay of hashed product captures; never an accuracy/runtime gate."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception"))
from sanitation_perception.pc_open_vocab_adapter import (  # noqa: E402
    _ground_dirt_prompt_indices, _projection_masks,
)
from sanitation_perception.product_projection import (  # noqa: E402
    CameraIntrinsics, PublicGrid, project_rgbd_observation,
)


def _hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _bundle(path: Path) -> tuple[dict, dict]:
    manifest = json.loads((path / "manifest.json").read_text(encoding="utf-8"))
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1 or set(manifest.get("files", {})) != {"arrays.npz", "metadata.json"}:
        raise ValueError("unsupported capture manifest")
    for name, digest in manifest["files"].items():
        if _hash(path / name) != digest:
            raise ValueError(f"capture hash mismatch: {name}")
    metadata = json.loads((path / "metadata.json").read_text(encoding="utf-8"))
    if not isinstance(metadata, dict) or metadata.get("schema_version", 1) != 1:
        raise ValueError("unsupported capture schema")
    with np.load(path / "arrays.npz", allow_pickle=False) as arrays:
        payload = {key: arrays[key] for key in arrays.files}
    return metadata, payload


def _replay_frame(root: Path, frame: Path, max_skew_s: float) -> dict:
    meta, arrays = _bundle(frame)
    map_hash = meta["map_content_sha256"]
    if not isinstance(map_hash, str) or re.fullmatch(r"[0-9a-f]{64}", map_hash) is None:
        raise ValueError("invalid map content hash")
    map_path = root / "maps" / map_hash
    if _hash(map_path / "manifest.json") != meta["map_manifest_sha256"]:
        raise ValueError("map manifest hash mismatch")
    grid_meta, map_arrays = _bundle(map_path)
    hash_metadata = {k: v for k, v in grid_meta.items() if k != "map_content_sha256"}
    encoded = (json.dumps(hash_metadata, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
    if hashlib.sha256(encoded + map_arrays["occupancy"].tobytes(order="C")).hexdigest() != map_hash:
        raise ValueError("map content hash mismatch")
    if grid_meta["frame_id"] != "map" or meta["sensor"] != "front":
        raise ValueError("unsupported capture frame or sensor")
    stamps = [float(meta[key]) for key in ("rgb_stamp_s", "depth_stamp_s")]
    if not all(math.isfinite(v) and v >= 0 for v in stamps) or abs(stamps[0] - stamps[1]) > max_skew_s:
        raise ValueError("RGB-D timestamp skew invalid")
    rgb, depth = arrays["rgb"], arrays["depth"]
    info = meta["camera_info"]
    if rgb.ndim != 3 or rgb.shape[2] != 3 or depth.shape != rgb.shape[:2] or depth.shape != (info["height"], info["width"]):
        raise ValueError("RGB-D/calibration dimensions mismatch")
    camera_k = arrays["camera_k"].reshape(9)
    if not np.array_equal(camera_k, np.asarray(info["k"])):
        raise ValueError("capture calibration mismatch")
    detections = meta["detections"]
    if [d["detection_index"] for d in detections] != list(range(len(detections))):
        raise ValueError("detection indices must be contiguous")
    boxes = np.asarray([d["xyxy"] for d in detections], dtype=np.float32).reshape(-1, 4)
    classes = [d["class_id"] for d in detections]
    indices = arrays["prompt_detection_indices"]
    if not np.array_equal(indices, _ground_dirt_prompt_indices(classes, boxes, depth.shape)):
        raise ValueError("captured prompt selection differs from current product policy")
    masks, qualities = _projection_masks(depth.shape, boxes, classes,
        [arrays[f"prompt_mask_{i:03d}"] for i in range(len(indices))], arrays["prompt_qualities"])
    scores = [float(d["confidence"]) for d in detections]
    if not all(math.isfinite(v) and 0 <= v <= 1 for v in scores):
        raise ValueError("invalid detector confidence")
    grid = PublicGrid(**{k: grid_meta[k] for k in ("width", "height", "resolution", "origin_x", "origin_y")}, occupancy=map_arrays["occupancy"])
    params = meta["projection"]
    diagnostics = {}
    raster, targets = project_rgbd_observation(depth,
        CameraIntrinsics(camera_k[0], camera_k[4], camera_k[2], camera_k[5]),
        arrays["map_from_camera"], grid, boxes_xyxy=boxes, class_ids=classes,
        masks=masks, confidences=[s*q for s, q in zip(scores, qualities)],
        sample_stride=int(params["sample_stride"]), ground_z=float(params["ground_z_m"]),
        ground_tolerance=float(params["ground_tolerance_m"]), diagnostics_out=diagnostics)
    if arrays["final_union_raster"].shape != raster.shape:
        raise ValueError("captured raster dimensions mismatch")
    observed = int(np.count_nonzero(raster))
    usable = observed > 0 or len(targets) > 0
    return {"frame": frame.name, "status": "replayed" if usable else "not_ready",
        "reason": "product_geometry_replayed" if usable else "no_usable_projected_observation",
        "observed_cells": observed, "dirty_cells": int(np.count_nonzero(raster > 1)),
        "projected_targets": [{"detection_index": t.detection_index, "xyz": list(t.xyz)} for t in targets],
        "changed_raster_cells": int(np.count_nonzero(raster != arrays["final_union_raster"])),
        "historical_raster_is_acceptance_oracle": False}


def replay_capture(root: str | Path, *, max_skew_s: float = 0.5) -> dict:
    """Replay recorded model outputs, without running or validating models."""
    root = Path(root)
    report = {"schema_version": 1, "status": "not_ready", "runtime_verified": False,
        "recognition_accuracy_verified": False, "model_inference_executed": False,
        "capture_authenticity_verified": False, "frames": []}
    if not math.isfinite(max_skew_s) or max_skew_s < 0:
        report["reason"] = "invalid_max_skew_s"
        return report
    frames = sorted((root / "frames").glob("frame-[0-9][0-9][0-9][0-9]"))
    if not frames:
        report["reason"] = "no_product_capture_frames"
        return report
    for frame in frames:
        try:
            result = _replay_frame(root, frame, max_skew_s)
        except (OSError, ValueError, KeyError, TypeError, IndexError, OverflowError, AttributeError, zipfile.BadZipFile) as exc:
            result = {"frame": frame.name, "status": "not_ready", "reason": str(exc)}
        report["frames"].append(result)
    if all(row["status"] == "replayed" for row in report["frames"]):
        report["status"] = "replayed"
        report["reason"] = "offline_projection_only"
    else:
        report["reason"] = "capture_validation_or_projection_failed"
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_root", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--max-skew-s", type=float, default=0.5)
    args = parser.parse_args()
    report = replay_capture(args.capture_root, max_skew_s=args.max_skew_s)
    encoded = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if report["status"] == "replayed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
