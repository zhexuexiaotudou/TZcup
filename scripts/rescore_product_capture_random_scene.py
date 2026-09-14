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
import uuid

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception_evaluator"))
sys.path.insert(0, str(ROOT / "scripts"))

from replay_product_observation_capture import _bundle, _replay_frame  # noqa: E402
from sanitation_perception.formal_random_scene_evaluator_core import (  # noqa: E402
    BoxObservation, match_boxes, projection_error_metrics, rasterize_dirt_truth, segmentation_metrics,
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


def _finite_xyz(value: object, label: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise CaptureRescoreError(f"{label} must be a three-dimensional map point")
    point = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in point):
        raise CaptureRescoreError(f"{label} must be finite")
    return point


def _projection_metrics(evidence_path: Path | None, binding: dict, truth: dict,
                        frame_rows: list[dict], frame_metadata: dict[str, dict],
                        capture_root: Path) -> tuple[dict, str | None]:
    """Evaluate product target positions only from a separately held evaluation sidecar.

    The sidecar is deliberately not a ProductIntermediateCapture artifact.  It can
    associate a product UUID with evaluator truth only after the product run has
    ended; its namespace and control prohibition make that direction explicit.
    """
    if evidence_path is None:
        return {"rmse_m": None, "p95_m": None,
                "reason": "projection_evidence_not_supplied_for_legacy_capture"}, None
    evidence = _load_json(evidence_path, "projection evidence")
    required = ("capture_manifest_sha256", "source_commit", "acceptance_session_binding",
                "runtime_closure_binding", "camera_frame_id", "map_frame_id", "samples")
    if (evidence.get("schema_version") != 1
            or evidence.get("control_use_prohibited") is not True
            or not str(evidence.get("namespace", "")).startswith("/evaluation/")
            or any(name not in evidence for name in required)):
        raise CaptureRescoreError("projection evidence contract is incomplete or not evaluator-only")
    if evidence["capture_manifest_sha256"] != binding["capture_manifest_sha256"]:
        raise CaptureRescoreError("projection evidence capture hash mismatch")
    if evidence["capture_manifest_sha256"] != _inventory_digest(capture_root):
        raise CaptureRescoreError("projection evidence capture inventory changed")
    if evidence["source_commit"] != binding["source_commit"]:
        raise CaptureRescoreError("projection evidence source commit mismatch")
    if (evidence["acceptance_session_binding"] != binding["acceptance_session_binding"]
            or evidence["runtime_closure_binding"] != binding["runtime_closure_binding"]):
        raise CaptureRescoreError("projection evidence session or closure mismatch")
    if evidence["map_frame_id"] != "map" or not isinstance(evidence["camera_frame_id"], str):
        raise CaptureRescoreError("projection evidence camera or map frame mismatch")
    if not isinstance(evidence["samples"], list) or not evidence["samples"]:
        raise CaptureRescoreError("projection evidence has no samples")
    truth_by_id = {row.get("object_id"): row for row in truth.get("discrete_cubes", [])}
    if len(truth_by_id) != len(truth.get("discrete_cubes", [])):
        raise CaptureRescoreError("evaluator truth has duplicate cube object IDs")
    replay_by_frame = {row["frame"]: row["replay"] for row in frame_rows}
    seen_uuid, seen_track, seen_detection, seen_truth = set(), {}, set(), set()
    previous_stamp = -math.inf
    errors = []
    for index, row in enumerate(evidence["samples"]):
        if not isinstance(row, dict):
            raise CaptureRescoreError(f"projection sample {index} is not an object")
        required_sample = ("target_uuid", "track_identity", "frame", "detection_index",
                           "observation_stamp_s", "map_point_xyz", "evaluator_object_id")
        if any(name not in row for name in required_sample):
            raise CaptureRescoreError(f"projection sample {index} is incomplete")
        try:
            target_uuid = str(uuid.UUID(str(row["target_uuid"])))
        except (AttributeError, ValueError) as exc:
            raise CaptureRescoreError(f"projection sample {index} target UUID is invalid") from exc
        track = row["track_identity"]
        if not isinstance(track, str) or not track.strip():
            raise CaptureRescoreError(f"projection sample {index} track identity is invalid")
        if target_uuid in seen_uuid:
            raise CaptureRescoreError("projection evidence has duplicate target UUID")
        if track in seen_track and seen_track[track] != target_uuid:
            raise CaptureRescoreError("projection evidence reuses a track identity across targets")
        seen_uuid.add(target_uuid); seen_track[track] = target_uuid
        frame = row["frame"]
        if frame not in replay_by_frame or frame not in frame_metadata:
            raise CaptureRescoreError(f"projection sample {index} references an unknown capture frame")
        meta = frame_metadata[frame]
        if meta["camera_info"].get("frame_id") != evidence["camera_frame_id"]:
            raise CaptureRescoreError("projection evidence camera frame mismatch")
        stamp = float(row["observation_stamp_s"])
        if (not math.isfinite(stamp) or stamp <= previous_stamp
                or abs(stamp - float(meta["rgb_stamp_s"])) > 1e-6
                or abs(stamp - float(meta["depth_stamp_s"])) > 1e-6):
            raise CaptureRescoreError("projection evidence timestamp is non-monotonic or does not match RGB-D")
        previous_stamp = stamp
        detection_key = (frame, int(row["detection_index"]))
        if detection_key in seen_detection:
            raise CaptureRescoreError("projection evidence duplicates a product detection")
        seen_detection.add(detection_key)
        product_point = _finite_xyz(row["map_point_xyz"], f"projection sample {index} product map point")
        projected = {int(item["detection_index"]): item["xyz"] for item in replay_by_frame[frame]["projected_targets"]}
        if detection_key[1] not in projected:
            raise CaptureRescoreError("projection sample has no reprojectable product target")
        replay_point = _finite_xyz(projected[detection_key[1]], "replayed product map point")
        if not np.allclose(product_point, replay_point, rtol=0.0, atol=1e-6):
            raise CaptureRescoreError("projection sample map point differs from captured product reprojection")
        truth_row = truth_by_id.get(row["evaluator_object_id"])
        if truth_row is None:
            raise CaptureRescoreError("projection sample evaluator object identity is absent from evaluator truth")
        if row["evaluator_object_id"] in seen_truth:
            raise CaptureRescoreError("projection evidence reuses an evaluator object identity")
        seen_truth.add(row["evaluator_object_id"])
        truth_point = _finite_xyz([truth_row["pose"]["x_m"], truth_row["pose"]["y_m"], truth_row["pose"]["z_m"]], "evaluator truth map point")
        errors.append(float(np.linalg.norm(np.asarray(product_point) - np.asarray(truth_point))))
    metrics = projection_error_metrics(errors)
    metrics["source"] = "evaluator_only_projection_evidence"
    metrics["projection_evidence_sha256"] = _sha256(evidence_path)
    return metrics, None


def rescore(capture_root: Path, public_path: Path, truth_path: Path, binding_path: Path,
            projection_evidence_path: Path | None = None) -> dict:
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
        dirt_rows, failures, frame_rows, frame_metadata = {}, {}, [], {}
        for frame in frames:
            replay = _replay_frame(capture_root, frame, 0.5)
            if replay["status"] != "replayed":
                raise CaptureRescoreError(f"capture frame rejected: {frame.name}")
            meta, arrays = _bundle(frame)
            frame_metadata[frame.name] = meta
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
        projection, _ = _projection_metrics(projection_evidence_path, binding, truth, frame_rows, frame_metadata, capture_root)
        precision = cube_tp / (cube_tp + cube_fp) if cube_tp + cube_fp else 0.0
        recall = cube_tp / (cube_tp + cube_fn) if cube_tp + cube_fn else 0.0
        report.update({"status": "RESCORED_OFFLINE", "episode_id": public["episode_id"], "map_id": public["map_id"],
            "bindings": {"source_commit": binding["source_commit"], "capture_manifest_sha256": binding["capture_manifest_sha256"], "public_manifest_sha256": binding["public_manifest_sha256"], "evaluator_truth_sha256": binding["evaluator_truth_sha256"], "acceptance_session_binding": binding["acceptance_session_binding"], "runtime_closure_binding": binding["runtime_closure_binding"]},
            "litter_cube": {"true_positive_count": cube_tp, "false_positive_count": cube_fp, "false_negative_count": cube_fn, "precision": precision, "recall": recall, "f1": 2 * precision * recall / (precision + recall) if precision + recall else 0.0},
            "dirt_by_class": {key: {"frame_count": len(rows), "iou_mean": float(np.mean([row["iou"] for row in rows])), "recall_mean": float(np.mean([row["recall"] for row in rows])), "missed_area_m2": float(sum(row["missed_area_m2"] for row in rows))} for key, rows in dirt_rows.items()},
            "map_projection": projection, "failure_samples": frame_rows, "unavailable_metrics": failures})
    except (OSError, KeyError, TypeError, ValueError, np.linalg.LinAlgError) as exc:
        report["reason"] = str(exc)
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-root", type=Path, required=True)
    parser.add_argument("--public-manifest", type=Path, required=True)
    parser.add_argument("--evaluator-truth", type=Path, required=True)
    parser.add_argument("--capture-binding", type=Path, required=True)
    parser.add_argument("--projection-evidence", type=Path,
                        help="optional evaluator-only UUID/track/map-point sidecar for projection error")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = rescore(args.capture_root, args.public_manifest, args.evaluator_truth, args.capture_binding,
                     args.projection_evidence)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": report["status"], "output": str(args.output)}))
    return 0 if report["status"] == "RESCORED_OFFLINE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
