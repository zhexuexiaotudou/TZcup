#!/usr/bin/env python3
"""Re-score hashed product captures with separately held evaluator truth.

This is an offline evaluator-only tool.  It never writes to a product capture,
runs no model, and refuses a capture without an independently supplied session
and closure binding.  Its output is diagnostic evidence, never formal product
acceptance.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception_evaluator"))
sys.path.insert(0, str(ROOT / "scripts"))

from replay_product_observation_capture import _bundle, _replay_frame  # noqa: E402
from sanitation_perception.formal_random_scene_evaluator_core import (  # noqa: E402
    BoxObservation, match_boxes, rasterize_dirt_truth, segmentation_metrics,
)
from sanitation_perception_evaluator.formal_random_scene_evaluator import _project_cube  # noqa: E402


class CaptureRescoreError(ValueError):
    """A missing or mismatched evidence boundary."""


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _inventory_digest(root: Path) -> str:
    rows = []
    for path in sorted(root.rglob("*")):
        if path.is_file():
            rows.append((path.relative_to(root).as_posix(), _sha256(path)))
    if not rows:
        raise CaptureRescoreError("no_product_capture_frames")
    return hashlib.sha256(json.dumps(rows, separators=(",", ":")).encode()).hexdigest()


def _load_json(path: Path, label: str) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CaptureRescoreError(f"cannot read {label}: {path}") from exc
    if not isinstance(value, dict):
        raise CaptureRescoreError(f"{label} must be a JSON object")
    return value


def _require_binding(binding: dict, *, capture_root: Path, public: Path, truth: Path) -> None:
    required = ("episode_id", "map_id", "source_commit", "capture_manifest_sha256",
                "public_manifest_sha256", "evaluator_truth_sha256",
                "acceptance_session_binding", "runtime_closure_binding")
    if binding.get("schema_version") != 1 or any(name not in binding for name in required):
        raise CaptureRescoreError("capture binding is incomplete")
    if binding.get("product_truth_input_used") is not False:
        raise CaptureRescoreError("capture binding does not prove product truth isolation")
    if not isinstance(binding["source_commit"], str) or len(binding["source_commit"]) != 40:
        raise CaptureRescoreError("capture binding source commit is invalid")
    if not isinstance(binding["acceptance_session_binding"], dict) or not isinstance(binding["runtime_closure_binding"], dict):
        raise CaptureRescoreError("capture binding lacks session or closure object")
    expected = {
        "capture_manifest_sha256": _inventory_digest(capture_root),
        "public_manifest_sha256": _sha256(public),
        "evaluator_truth_sha256": _sha256(truth),
    }
    for name, digest in expected.items():
        if binding.get(name) != digest:
            raise CaptureRescoreError(f"capture binding hash mismatch: {name}")


def _truth_for_frame(truth: dict, arrays: dict, metadata: dict):
    camera_k = np.asarray(arrays["camera_k"], dtype=np.float64).reshape(9)
    info = type("CameraInfo", (), {"k": camera_k, "width": int(metadata["camera_info"]["width"]), "height": int(metadata["camera_info"]["height"])})()
    camera_from_map = np.linalg.inv(np.asarray(arrays["map_from_camera"], dtype=np.float64))
    boxes = []
    for cube in truth["discrete_cubes"]:
        projected = _project_cube(cube, camera_from_map, info)
        if projected is not None:
            boxes.append(projected[0])
    return boxes


def _class_dirt_metrics(truth: dict, arrays: dict, map_metadata: dict) -> tuple[dict, dict]:
    output, failures = {}, {}
    for kind, class_id in (("leaf", "fallen_leaves"), ("dust", "dust_or_soil"), ("puddle", "puddle")):
        truth_raster = rasterize_dirt_truth(
            [row for row in truth["dirt_patches"] if row.get("kind") == kind],
            width=int(map_metadata["width"]), height=int(map_metadata["height"]),
            resolution=float(map_metadata["resolution"]), origin_x=float(map_metadata["origin_x"]), origin_y=float(map_metadata["origin_y"]),
        )
        key = f"class_raster_{class_id}"
        if key not in arrays:
            failures[class_id] = "product_class_raster_missing"
            continue
        metrics = segmentation_metrics(arrays[key] > 0, truth_raster)
        metrics["missed_area_m2"] = (metrics["truth_cell_count"] - metrics["intersection_cell_count"]) * float(map_metadata["resolution"]) ** 2
        output[class_id] = metrics
    return output, failures


def rescore(capture_root: Path, public_path: Path, truth_path: Path, binding_path: Path) -> dict:
    report = {"schema_version": 1, "status": "BLOCKED", "claim_boundary": {
        "evaluator_only_offline_diagnostic": True, "eligible_as_formal_product_acceptance": False,
        "truth_used_to_modify_product_output": False, "model_inference_executed": False}}
    try:
        public, truth, binding = (_load_json(public_path, "public manifest"), _load_json(truth_path, "evaluator truth"), _load_json(binding_path, "capture binding"))
        if truth.get("control_use_prohibited") is not True or not str(truth.get("namespace", "")).startswith("/evaluation/"):
            raise CaptureRescoreError("evaluator truth isolation contract is invalid")
        if public.get("episode_id") != truth.get("episode_id") or public.get("map_id") != truth.get("map_id"):
            raise CaptureRescoreError("public manifest and evaluator truth identity mismatch")
        if binding["episode_id"] != public["episode_id"] or binding["map_id"] != public["map_id"]:
            raise CaptureRescoreError("capture binding episode or map identity mismatch")
        frames = sorted((capture_root / "frames").glob("frame-[0-9][0-9][0-9][0-9]"))
        if not frames:
            raise CaptureRescoreError("no_product_capture_frames")
        _require_binding(binding, capture_root=capture_root, public=public_path, truth=truth_path)
        cube_tp = cube_fp = cube_fn = 0
        dirt_rows, failures, frame_rows = {}, {}, []
        for frame in frames:
            replay = _replay_frame(capture_root, frame, 0.5)
            if replay["status"] != "replayed":
                raise CaptureRescoreError(f"capture frame rejected: {frame.name}")
            meta, arrays = _bundle(frame)
            map_meta, _ = _bundle(capture_root / "maps" / meta["map_content_sha256"])
            truth_boxes = _truth_for_frame(truth, arrays, meta)
            predictions = [BoxObservation("litter_cube", float(row["confidence"]), tuple(map(float, row["xyxy"]))) for row in meta["detections"] if row.get("class_id") == "litter_cube"]
            matched = match_boxes(predictions, truth_boxes, iou_threshold=0.5)
            cube_tp += matched["true_positive_count"]; cube_fp += matched["false_positive_count"]; cube_fn += matched["false_negative_count"]
            class_metrics, class_failures = _class_dirt_metrics(truth, arrays, map_meta)
            for class_id, metrics in class_metrics.items():
                dirt_rows.setdefault(class_id, []).append(metrics)
            failures.update(class_failures)
            frame_rows.append({"frame": frame.name, "cube_unmatched_truth_object_ids": matched["unmatched_truth_object_ids"], "cube_false_positive_indices": matched["false_positive_indices"], "replay": replay})
        precision = cube_tp / (cube_tp + cube_fp) if cube_tp + cube_fp else 0.0
        recall = cube_tp / (cube_tp + cube_fn) if cube_tp + cube_fn else 0.0
        report.update({"status": "RESCORED_OFFLINE", "episode_id": public["episode_id"], "map_id": public["map_id"],
            "bindings": {"source_commit": binding["source_commit"], "capture_manifest_sha256": binding["capture_manifest_sha256"], "public_manifest_sha256": binding["public_manifest_sha256"], "evaluator_truth_sha256": binding["evaluator_truth_sha256"], "acceptance_session_binding": binding["acceptance_session_binding"], "runtime_closure_binding": binding["runtime_closure_binding"]},
            "litter_cube": {"true_positive_count": cube_tp, "false_positive_count": cube_fp, "false_negative_count": cube_fn, "precision": precision, "recall": recall, "f1": 2 * precision * recall / (precision + recall) if precision + recall else 0.0},
            "dirt_by_class": {key: {"frame_count": len(rows), "iou_mean": float(np.mean([row["iou"] for row in rows])), "recall_mean": float(np.mean([row["recall"] for row in rows])), "missed_area_m2": float(sum(row["missed_area_m2"] for row in rows))} for key, rows in dirt_rows.items()},
            "map_projection": {"rmse_m": None, "p95_m": None, "reason": "capture_format_has_no_product_target_identity_or_position"}, "failure_samples": frame_rows, "unavailable_metrics": failures})
    except (OSError, KeyError, TypeError, ValueError, np.linalg.LinAlgError) as exc:
        report["reason"] = str(exc)
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-root", type=Path, required=True)
    parser.add_argument("--public-manifest", type=Path, required=True)
    parser.add_argument("--evaluator-truth", type=Path, required=True)
    parser.add_argument("--capture-binding", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = rescore(args.capture_root, args.public_manifest, args.evaluator_truth, args.capture_binding)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": report["status"], "output": str(args.output)}))
    return 0 if report["status"] == "RESCORED_OFFLINE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
