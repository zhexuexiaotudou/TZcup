#!/usr/bin/env python3
"""One-shot OPRV3-09 G5 evaluator for the frozen x86 product pipeline."""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_spot_cleaning"))
sys.path.insert(0, str(ROOT / "scripts"))

from sanitation_learning.g4_manifest import config_hash  # noqa: E402
from sanitation_learning.g5_dataset import _dataset_tree_digest  # noqa: E402


ACCESS_RECORD = "sealed_final_access.json"
RESULT_RECORD = "sealed_final_result.json"
CLASS_TO_LABEL = {
    "plastic_bottle": 1, "metal_can": 2, "paper_litter": 3,
    "leaf_pile": 4, "puddle": 5,
}
DISCRETE_CLASSES = ("plastic_bottle", "metal_can", "paper_litter")
IOU_THRESHOLD = 0.50


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def bbox_iou(first, second) -> float:
    left = max(float(first[0]), float(second[0]))
    top = max(float(first[1]), float(second[1]))
    right = min(float(first[2]), float(second[2]))
    bottom = min(float(first[3]), float(second[3]))
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = max(0.0, float(first[2]) - float(first[0])) * max(0.0, float(first[3]) - float(first[1]))
    second_area = max(0.0, float(second[2]) - float(second[0])) * max(0.0, float(second[3]) - float(second[1]))
    return intersection / max(first_area + second_area - intersection, 1e-12)


def scaled_bbox(native_bbox, input_size):
    return [
        native_bbox[0] * input_size[0] / 640.0,
        native_bbox[1] * input_size[1] / 480.0,
        native_bbox[2] * input_size[0] / 640.0,
        native_bbox[3] * input_size[1] / 480.0,
    ]


def atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode()
    try:
        descriptor = os.open(str(path), os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
    except FileExistsError as exc:
        raise RuntimeError(f"sealed-final record already exists: {path}") from exc
    try:
        os.write(descriptor, encoded)
    finally:
        os.close(descriptor)


def nested(payload: dict, dotted: str):
    value = payload
    for key in dotted.split("."):
        value = value[key]
    return value


def evaluate_policy(policy: dict, metrics: dict) -> dict:
    results = {}
    for gate_id, gate in policy["gates"].items():
        value = nested(metrics, gate["metric"])
        threshold = gate["threshold"]
        operator = gate["operator"]
        passed = {
            "ge": value >= threshold,
            "le": value <= threshold,
            "eq": value == threshold,
        }[operator]
        results[gate_id] = {**gate, "value": value, "pass": bool(passed)}
    return {"pass": all(item["pass"] for item in results.values()), "gates": results}


def validate_pre_access(args, freeze: dict, sealed: dict, development: dict) -> dict:
    from perception_oprv3_moving_benchmark import create_cuda_ort_session

    require(freeze.get("protocol") == "OPRV3-08", "freeze protocol is not OPRV3-08")
    require(freeze.get("status") == "FROZEN_X86", "x86 pipeline is not frozen")
    require(freeze.get("release_boundary", {}).get("x86_development_pass") is True, "x86 development did not pass")
    require(freeze.get("release_boundary", {}).get("sealed_final_pass") is False, "freeze already claims sealed final")
    sealed_contract = freeze.get("sealed_final", {})
    require(sealed_contract.get("maximum_accesses") == 1, "freeze does not enforce one-shot access")
    require(sha256(args.evaluator) == sealed_contract.get("evaluator", {}).get("sha256"), "evaluator SHA-256 mismatch")
    require(sha256(args.policy) == sealed_contract.get("policy", {}).get("sha256"), "sealed policy SHA-256 mismatch")
    require(sha256(args.geometry) == sealed_contract.get("geometry_audit", {}).get("sha256"), "geometry audit SHA-256 mismatch")
    require(sha256(args.development_manifest) == sealed_contract.get("development_world_manifest", {}).get("sha256"), "development world manifest SHA-256 mismatch")
    require(sha256(args.area_gate) == freeze.get("evidence", {}).get("area", {}).get("sha256"), "Area gate SHA-256 mismatch")
    require(sha256(args.pipeline_config) == freeze.get("implementation", {}).get("pipeline_config", {}).get("sha256"), "pipeline config SHA-256 mismatch")
    require(sealed.get("schema_version") == 1, "sealed manifest schema mismatch")
    require(sealed.get("dataset_id") == "G5_SEALED_FINAL", "sealed dataset id mismatch")
    require(sealed.get("dataset_gate_pass") is True, "G5 dataset QA did not pass")
    require(len(sealed.get("worlds", [])) >= 4, "G5 requires at least four worlds")
    require(int(sealed.get("scenes", 0)) >= 100, "G5 requires at least 100 scenes")
    require(int(sealed.get("frames", 0)) >= 1000, "G5 requires at least 1000 frames")
    hash_payload = dict(sealed)
    declared_hash = hash_payload.pop("manifest_sha256", None)
    require(declared_hash == config_hash(hash_payload), "sealed manifest hash mismatch")
    development_worlds = {item["world_id"] for item in development["worlds"]}
    development_targets = {item["model_name"] for item in development["assets"]}
    development_negatives = {item["model_name"] for item in development["negative_assets"]}
    require(not set(sealed["worlds"]) & development_worlds, "sealed worlds overlap development")
    require(not set(sealed["target_assets"]) & development_targets, "sealed targets overlap development")
    require(not set(sealed["hard_negative_assets"]) & development_negatives, "sealed negatives overlap development")
    for role, path in (
        ("detector_checkpoint", args.detector_checkpoint),
        ("detector_onnx", args.detector_onnx),
        ("leaf_checkpoint", args.leaf_checkpoint),
        ("leaf_onnx", args.leaf_onnx),
        ("puddle_checkpoint", args.puddle_checkpoint),
        ("puddle_onnx", args.puddle_onnx),
    ):
        expected = freeze["models"][role.split("_")[0]][role.split("_", 1)[1]]["sha256"]
        require(sha256(path) == expected, f"{role} SHA-256 mismatch")
    sessions = {
        "detector": create_cuda_ort_session(args.detector_onnx),
        "leaf": create_cuda_ort_session(args.leaf_onnx),
        "puddle": create_cuda_ort_session(args.puddle_onnx),
    }
    providers = {name: session.get_providers() for name, session in sessions.items()}
    del sessions
    return {"passed": True, "providers": providers, "manifest_sha256": declared_hash}


def average_precision(frames: list[dict], class_name: str, iou_threshold: float) -> float:
    truth_total = sum(sum(item["class_name"] == class_name for item in frame["truth"]) for frame in frames)
    ranked = []
    for frame_index, frame in enumerate(frames):
        for prediction in frame["predictions"]:
            if prediction["class_name"] == class_name:
                ranked.append((float(prediction["score"]), frame_index, prediction))
    ranked.sort(key=lambda item: item[0], reverse=True)
    used: dict[int, set[int]] = defaultdict(set)
    tp, fp = [], []
    for _, frame_index, prediction in ranked:
        best_index, best_iou = -1, 0.0
        for truth_index, truth in enumerate(frames[frame_index]["truth"]):
            if truth["class_name"] != class_name or truth_index in used[frame_index]:
                continue
            overlap = bbox_iou(prediction["bbox_xyxy"], truth["bbox_xyxy"])
            if overlap > best_iou:
                best_index, best_iou = truth_index, overlap
        matched = best_index >= 0 and best_iou >= iou_threshold
        if matched:
            used[frame_index].add(best_index)
        tp.append(int(matched)); fp.append(int(not matched))
    if not truth_total:
        return 0.0
    tp_cumulative = np.cumsum(tp); fp_cumulative = np.cumsum(fp)
    recall = tp_cumulative / truth_total
    precision = tp_cumulative / np.maximum(tp_cumulative + fp_cumulative, 1)
    return float(np.mean([
        float(precision[recall >= level].max()) if np.any(recall >= level) else 0.0
        for level in np.linspace(0.0, 1.0, 101)
    ]))


def discrete_metrics(rows, context, detector_frames, detector_metadata) -> dict:
    frames = []
    threshold = float(detector_metadata["action_threshold"])
    for row in rows:
        key = (int(row["scene_seed"]), int(row["frame_index"]))
        truth = []
        for class_name in DISCRETE_CLASSES:
            bbox = context["frame_truth"][key][class_name]["bbox"]
            if bbox is not None:
                truth.append({
                    "class_name": class_name,
                    "bbox_xyxy": scaled_bbox(bbox, detector_metadata["input_size"]),
                    "small": min(bbox[2] - bbox[0], bbox[3] - bbox[1]) < 18,
                })
        predictions = [item for item in detector_frames[key]["detections"] if float(item["score"]) >= threshold]
        frames.append({"truth": truth, "predictions": predictions})
    per_class = {}
    small_total = small_matched = 0
    for class_name in DISCRETE_CLASSES:
        tp = fp = fn = 0
        for frame in frames:
            truths = [item for item in frame["truth"] if item["class_name"] == class_name]
            predictions = [item for item in frame["predictions"] if item["class_name"] == class_name]
            used = set()
            for prediction in sorted(predictions, key=lambda item: float(item["score"]), reverse=True):
                overlaps = [(i, bbox_iou(prediction["bbox_xyxy"], truth["bbox_xyxy"])) for i, truth in enumerate(truths) if i not in used]
                best = max(overlaps, key=lambda item: item[1], default=(-1, 0.0))
                if best[0] >= 0 and best[1] >= IOU_THRESHOLD:
                    used.add(best[0]); tp += 1
                    if truths[best[0]]["small"]:
                        small_matched += 1
                else:
                    fp += 1
            fn += len(truths) - len(used)
            small_total += sum(item["small"] for item in truths)
        precision = tp / max(tp + fp, 1); recall = tp / max(tp + fn, 1)
        per_class[class_name] = {
            "tp": tp, "fp": fp, "fn": fn, "precision": precision, "recall": recall,
            "f1": 2 * precision * recall / max(precision + recall, 1e-12),
        }
    ap50_by_class = {name: average_precision(frames, name, 0.50) for name in DISCRETE_CLASSES}
    ap5095_by_class = {
        name: float(np.mean([average_precision(frames, name, value) for value in np.arange(0.50, 0.96, 0.05)]))
        for name in DISCRETE_CLASSES
    }
    return {
        "per_class": per_class,
        "macro_precision": float(np.mean([item["precision"] for item in per_class.values()])),
        "macro_recall": float(np.mean([item["recall"] for item in per_class.values()])),
        "macro_f1": float(np.mean([item["f1"] for item in per_class.values()])),
        "minimum_per_class_recall": min(item["recall"] for item in per_class.values()),
        "small_object_recall": small_matched / max(small_total, 1),
        "small_object_truth_instances": small_total,
        "AP50_by_class": ap50_by_class,
        "AP50": float(np.mean(list(ap50_by_class.values()))),
        "AP50_95_by_class": ap5095_by_class,
        "AP50_95": float(np.mean(list(ap5095_by_class.values()))),
    }


def area_metrics(rows, areas) -> dict:
    counts = {
        name: {key: 0 for key in ("intersection", "union", "boundary_intersection", "boundary_union")}
        for name in ("leaf_pile", "puddle")
    }
    negative_frames = negative_fp_frames = 0
    for row in rows:
        key = (int(row["scene_seed"]), int(row["frame_index"]))
        if row.get("negative_only"):
            negative_frames += 1
            negative_fp_frames += int(any(areas[key][name]["has_area_candidate"] for name in counts))
        for name, accumulator in counts.items():
            item = areas[key][name]
            accumulator["intersection"] += item["intersection_pixels"]
            accumulator["union"] += item["union_pixels"]
            accumulator["boundary_intersection"] += item["boundary_intersection_pixels"]
            accumulator["boundary_union"] += item["boundary_union_pixels"]
    iou = {name: value["intersection"] / max(value["union"], 1) for name, value in counts.items()}
    boundary = {
        name: 2 * value["boundary_intersection"] / max(value["boundary_intersection"] + value["boundary_union"], 1)
        for name, value in counts.items()
    }
    return {
        "iou_by_class": iou,
        "macro_miou": float(np.mean(list(iou.values()))),
        "boundary_f1_by_class": boundary,
        "boundary_f1": float(np.mean(list(boundary.values()))),
        "negative_only_frames": negative_frames,
        "negative_area_fp_frames": negative_fp_frames,
        "negative_area_fp_per_frame": negative_fp_frames / max(negative_frames, 1),
        "pixel_totals": counts,
    }


def build_encounters(rows, context, geometry, detector_frames, detector_metadata, areas):
    from sanitation_learning.oprv3_moving import summarize_encounter
    from perception_oprv3_moving_benchmark import target_frame_facts

    row_by_key = {(int(row["scene_seed"]), int(row["frame_index"])): row for row in rows}
    encounters = []
    for seed, scene in sorted(context["scenes"].items()):
        report = context["capture_reports"][seed]
        object_by_name = {item["model_name"]: item for item in scene.get("objects", [])}
        record_by_index = {int(item["frame_index"]): item for item in report["records"]}
        for target in scene.get("objects", []):
            class_name = target.get("class_id")
            if class_name not in CLASS_TO_LABEL:
                continue
            facts = []
            for index in sorted(record_by_index):
                key = (seed, index)
                facts.append(target_frame_facts(
                    row=row_by_key[key], capture_record=record_by_index[index], target=target,
                    truth_fact=context["frame_truth"][key][class_name],
                    geometry_window=geometry["class_actionable_windows"][class_name],
                    detector_frame=detector_frames[key], area_frame=areas[key],
                    detector_metadata=detector_metadata,
                    occluder_truth_fact=(context["frame_truth"][key][object_by_name[target["occluded_by_model_name"]]["class_id"]] if target.get("occluded_by_model_name") else None),
                ))
            encounter = summarize_encounter(
                target, facts,
                int(geometry["vehicle_and_action"]["spot_clean_confirmation_observations"]),
                int(geometry["class_actionable_windows"][class_name]["minimum_visible_frames"]),
            )
            encounter["scene_seed"] = seed; encounter["world_id"] = scene["world_id"]
            encounters.append(encounter)
    return encounters


def dataset_coverage(rows, context) -> dict:
    moving = 0; static_target_scenes = 0; dynamic_scenes = 0
    for seed, report in context["capture_reports"].items():
        records = report["records"]
        displacement = math.hypot(
            records[-1]["vehicle_xy_m"][0] - records[0]["vehicle_xy_m"][0],
            records[-1]["vehicle_xy_m"][1] - records[0]["vehicle_xy_m"][1],
        )
        moving += int(report.get("adjacent_motion_gate_pass") is True and displacement >= 0.20)
        scene = context["scenes"][seed]
        static_target_scenes += int(any(item.get("class_id") in CLASS_TO_LABEL for item in scene.get("objects", [])))
        dynamic_scenes += int(report.get("dynamic_motion_requested") is True)
    return {
        "worlds": len({row["world_id"] for row in rows}), "scenes": len(context["scenes"]), "frames": len(rows),
        "independent_moving_sequences": moving, "static_target_scenes": static_target_scenes,
        "dynamic_object_scenes": dynamic_scenes,
        "pass": len({row["world_id"] for row in rows}) >= 4 and len(context["scenes"]) >= 100 and len(rows) >= 1000 and moving >= 20 and static_target_scenes > 0 and dynamic_scenes > 0,
    }


def evaluate_after_access(args, freeze, sealed) -> dict:
    from sanitation_learning.oprv3_moving import summarize_route
    from perception_oprv3_moving_benchmark import (
        combined_frame_maps_onnx,
        create_cuda_ort_session,
        detector_metadata_only,
        false_discrete_actions,
        load_area_gate,
        load_development_rows,
        product_map_evaluation,
    )

    require(_dataset_tree_digest(args.dataset_root).get("sha256") == sealed["dataset_content"]["sha256"], "sealed dataset tree digest mismatch")
    rows, instances, context = load_development_rows(args.dataset_root)
    coverage = dataset_coverage(rows, context)
    require(coverage["pass"], "sealed static/moving coverage contract failed")
    detector_metadata = detector_metadata_only(args.detector_checkpoint)
    geometry = load_json(args.geometry)
    area_configs, _ = load_area_gate(
        args.area_gate, leaf_checkpoint=args.leaf_checkpoint, puddle_checkpoint=args.puddle_checkpoint
    )
    detector = create_cuda_ort_session(args.detector_onnx)
    leaf = create_cuda_ort_session(args.leaf_onnx)
    puddle = create_cuda_ort_session(args.puddle_onnx)
    pipeline = yaml.safe_load(args.pipeline_config.read_text(encoding="utf-8"))
    camera_pitch = math.radians(-float(geometry["camera"]["pitch_deg"]))
    areas, area_detections, detector_frames = combined_frame_maps_onnx(
        detector, detector_metadata, leaf, puddle, rows,
        area_configs=area_configs, camera_pitch_down_rad=camera_pitch,
        minimum_physical_area_m2=float(pipeline["runtime"]["minimum_area_region_m2"]),
        minimum_physical_area_m2_by_class=pipeline["runtime"]["minimum_area_region_m2_by_class"],
    )
    encounters = build_encounters(rows, context, geometry, detector_frames, detector_metadata, areas)
    route = summarize_route(encounters, false_discrete_actions(detector_frames, instances, detector_metadata))
    product_map = product_map_evaluation(
        rows=rows, context=context, detector_frames=detector_frames,
        detector_metadata=detector_metadata, area_detections=area_detections,
        encounters=encounters, camera_pitch_down_rad=camera_pitch, manifest=pipeline,
    )
    discrete = discrete_metrics(rows, context, detector_frames, detector_metadata)
    area = area_metrics(rows, areas)
    product = product_map["metrics"]
    object_precision = product["product_target_precision"]
    object_recall = product["map_localization_coverage"]
    small = [
        item for item in encounters if item["entered_actionable_window"]
        and any(frame["actionable_window"] and frame["visible_bbox_short_side_px"] < 18 for frame in item["frames"])
    ]
    metrics = {
        "schema_version": 1, "protocol": "OPRV3-09", "dataset_id": "G5_SEALED_FINAL",
        "freeze_id": freeze["freeze_id"], "dataset_coverage": coverage,
        "object": {
            "precision": object_precision, "recall": object_recall,
            "f1": 2 * object_precision * object_recall / max(object_precision + object_recall, 1e-12),
        },
        "discrete": discrete,
        "online": {
            "eventual_detection_recall": route["eventual_detection_recall"],
            "eventual_correct_class_recall": route["eventual_correct_class_recall"],
            "eventual_track_confirmation_recall": route["eventual_track_confirmation_recall"],
            "small_object_eligible_targets": len(small),
            "small_object_eventual_recall": sum(item["eventual_correct_class"] for item in small) / max(len(small), 1),
            "false_actionable_target_rate": max(
                route["wrong_actionable_target_rate"],
                product["false_confirmed_target_rate"],
            ),
            "pre_fov_target_creation": product["pre_fov_target_creation"],
            "product_map": product,
        },
        "area": area,
        "runtime": {"provider": "CUDAExecutionProvider", "model_inference_frames": len(rows)},
    }
    policy = yaml.safe_load(args.policy.read_text(encoding="utf-8"))
    policy_result = evaluate_policy(policy, metrics)
    metrics["policy"] = policy_result
    metrics["OPRV3_SEALED_FINAL_PASS"] = policy_result["pass"]
    return metrics


def main() -> int:
    parser = argparse.ArgumentParser()
    for name in ("freeze", "sealed-manifest", "development-manifest", "dataset-root", "evidence-dir", "evaluator", "policy", "geometry", "area-gate", "pipeline-config", "detector-checkpoint", "detector-onnx", "leaf-checkpoint", "leaf-onnx", "puddle-checkpoint", "puddle-onnx"):
        parser.add_argument(f"--{name}", required=True, type=Path)
    args = parser.parse_args()
    access_path = args.evidence_dir / ACCESS_RECORD
    result_path = args.evidence_dir / RESULT_RECORD
    try:
        require(not access_path.exists() and not result_path.exists(), "sealed final was already accessed")
        freeze = load_json(args.freeze); sealed = load_json(args.sealed_manifest)
        development = load_json(args.development_manifest)
        preflight = validate_pre_access(args, freeze, sealed, development)
        atomic_json(access_path, {
            "schema_version": 1, "event": "sealed_final_first_access", "dataset_id": "G5_SEALED_FINAL",
            "freeze_id": freeze["freeze_id"], "manifest_sha256": sealed["manifest_sha256"],
            "access_timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "evaluation_count": 0, "preflight": preflight,
        })
        metrics = evaluate_after_access(args, freeze, sealed)
        atomic_json(result_path, {
            "schema_version": 1, "event": "sealed_final_evaluation", "dataset_id": "G5_SEALED_FINAL",
            "freeze_id": freeze["freeze_id"], "evaluation_timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "one_shot": True, "rerun_allowed": False, "metrics": metrics,
        })
        print(json.dumps({"OPRV3_SEALED_FINAL_PASS": metrics["OPRV3_SEALED_FINAL_PASS"], "result": result_path.as_posix(), "failed_gates": [name for name, gate in metrics["policy"]["gates"].items() if not gate["pass"]]}, indent=2))
        return 0 if metrics["OPRV3_SEALED_FINAL_PASS"] else 2
    except Exception as exc:
        print(json.dumps({"sealed_final_blocked": True, "reason": str(exc), "access_may_be_consumed": access_path.exists()}, indent=2), file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
