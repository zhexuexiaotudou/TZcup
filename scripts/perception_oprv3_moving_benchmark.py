#!/usr/bin/env python3
"""Benchmark frozen detectors on OPRV3 moving-camera encounters.

Ground truth is consumed only by this evaluator.  The production candidates
are produced from RGB (and RGB-D for the two area heads) without target IDs or
target coordinates.  Actionable-window eligibility is frozen in the OPRV3-01
geometry audit and never depends on a model score.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import platform
import subprocess
import sys
import time

import cv2
import numpy as np
import torch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))
sys.path.insert(0, str(ROOT / "scripts"))

from sanitation_learning.g4_data import (  # noqa: E402
    G4AreaDataset,
    index_instance_records,
)
from sanitation_learning.g4_direct_fcos import (  # noqa: E402
    MRV2_C_P2_ARCHITECTURE,
    build_direct_fcos,
    build_p2_direct_fcos,
    direct_predictions,
)
from sanitation_learning.g4_models import build_g4_model  # noqa: E402
from sanitation_learning.oprv3_moving import (  # noqa: E402
    actionable_window_eligible,
    bbox_from_mask,
    bbox_iou,
    empirical_special_coverage,
    percentile,
    summarize_encounter,
    summarize_route,
)
from perception_prod_x1_full_pipeline import (  # noqa: E402
    AREA_THRESHOLDS,
)


CLASS_TO_LABEL = {
    "plastic_bottle": 1,
    "metal_can": 2,
    "paper_litter": 3,
    "leaf_pile": 4,
    "puddle": 5,
}
DISCRETE_CLASSES = ("plastic_bottle", "metal_can", "paper_litter")
AREA_CLASSES = ("leaf_pile", "puddle")
OBSERVATION_THRESHOLD = 0.05
IOU_THRESHOLD = 0.50
AREA_TARGET_COVERAGE_THRESHOLD = 0.50


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def repository_commit() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unavailable_in_runtime"


def load_geometry(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not payload.get("frozen_before_moving_model_measurement"):
        raise RuntimeError("actionable geometry was not frozen before model measurement")
    return payload


def load_development_rows(data_root: Path) -> tuple[list[dict], list[dict], dict]:
    rows: list[dict] = []
    instances: list[dict] = []
    scenes: dict[int, dict] = {}
    capture_reports: dict[int, dict] = {}
    frame_truth: dict[tuple[int, int], dict[str, dict]] = {}
    for scene_dir in sorted((data_root / "scenes").glob("scene_*")):
        report_path = scene_dir / "capture_report.json"
        manifest_path = scene_dir / "scene_manifest.json"
        if not report_path.is_file() or not manifest_path.is_file():
            continue
        report = json.loads(report_path.read_text(encoding="utf-8"))
        if not report.get("capture_pass"):
            continue
        if report.get("captured_frames") != report.get("requested_frames"):
            raise RuntimeError(f"partial capture cannot enter benchmark: {scene_dir.name}")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        seed = int(manifest["scene_seed"])
        if seed in scenes:
            raise RuntimeError(f"duplicate scene seed {seed}")
        scenes[seed] = manifest
        capture_reports[seed] = report
        positive_objects = [
            item for item in manifest.get("objects", [])
            if item.get("class_id") in CLASS_TO_LABEL
        ]
        counts = defaultdict(int)
        for item in positive_objects:
            counts[item["class_id"]] += 1
        if any(value > 1 for value in counts.values()):
            raise RuntimeError(
                f"moving evaluator requires at most one target per class in scene {seed}"
            )
        for record in report["records"]:
            index = int(record["frame_index"])
            row = {
                "scene_seed": seed,
                "frame_index": index,
                "split": manifest["split"],
                "world_id": manifest["world_id"],
                "negative_only": bool(manifest.get("negative_only", False)),
                "rgb_path": scene_dir / record["paths"]["rgb"],
                "depth_path": scene_dir / record["paths"]["depth"],
                "semantic_path": scene_dir / record["paths"]["semantic"],
                "instance_path": scene_dir / record["paths"]["instance"],
            }
            rows.append(row)
            semantic = np.load(row["semantic_path"], allow_pickle=False)
            depth = np.load(row["depth_path"], allow_pickle=False)
            key = (seed, index)
            frame_truth[key] = {}
            for class_name, label in CLASS_TO_LABEL.items():
                mask = semantic == label
                bbox = bbox_from_mask(mask)
                frame_truth[key][class_name] = {
                    "bbox": bbox,
                    "mask_area_px": int(mask.sum()),
                    "depth_valid_ratio": (
                        float(np.isfinite(depth[mask]).mean()) if mask.any() else 0.0
                    ),
                }
            if row["negative_only"]:
                continue
            for class_name in DISCRETE_CLASSES:
                fact = frame_truth[key][class_name]
                bbox = fact["bbox"]
                if bbox is None:
                    continue
                instances.append(
                    {
                        "scene_seed": seed,
                        "frame_index": index,
                        "semantic_class": class_name,
                        "bbox_xyxy_px": bbox,
                        "bbox_shortest_side_px": min(
                            bbox[2] - bbox[0], bbox[3] - bbox[1]
                        ),
                        "mask_area_px": fact["mask_area_px"],
                    }
                )
    if not rows:
        raise RuntimeError("no passing moving-camera captures found")
    return rows, instances, {
        "scenes": scenes,
        "capture_reports": capture_reports,
        "frame_truth": frame_truth,
    }


def load_detector(path: Path, device: torch.device) -> tuple[torch.nn.Module, dict]:
    payload = torch.load(path, map_location=device, weights_only=False)
    if payload.get("checkpoint_status") not in (
        "training_complete",
        "training_complete_candidate_not_frozen",
    ):
        raise RuntimeError(f"detector checkpoint is not complete: {path}")
    if payload.get("G5_SEALED_FINAL_read") is not False:
        raise RuntimeError(f"detector violates G5 boundary: {path}")
    input_size = tuple(payload.get("input_size", (640, 480)))
    model = (
        build_p2_direct_fcos(input_size=input_size)
        if payload.get("architecture") == MRV2_C_P2_ARCHITECTURE
        else build_direct_fcos(
            input_size=input_size, checkpoint_load_control=True
        )
    ).to(device)
    model.load_state_dict(payload["state_dict"], strict=True)
    model.eval()
    return model, {
        "route": payload.get("route", path.stem),
        "path": path.as_posix(),
        "sha256": sha256(path),
        "architecture": payload.get("architecture"),
        "input_size": list(input_size),
        "action_threshold": float(payload["frozen_threshold_from_train_world_holdout"]),
        "checkpoint_status": payload.get("checkpoint_status"),
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": bool(payload.get("legacy_G4_D6_read", False)),
    }


def load_area_checkpoint(task: str, path: Path, device: torch.device):
    checkpoint = torch.load(path, map_location=device, weights_only=False)
    if checkpoint.get("checkpoint_status") != "training_complete":
        raise RuntimeError(f"{task} checkpoint is not training_complete")
    contract = checkpoint.get("model_contract") or {}
    area_architecture = (
        "deeplab_resnet50"
        if "deeplab" in str(contract.get("model_id", ""))
        else "dual_resnet18"
    )
    # The strict checkpoint load overwrites every tensor.  Starting with the
    # same architecture but no pretrained download makes formal evaluation
    # network-independent without changing the resulting model state.
    model = build_g4_model(
        task,
        area_architecture=area_architecture,
        from_scratch_control=True,
    ).to(device)
    model.load_state_dict(checkpoint["state_dict"], strict=True)
    model.eval()
    return model, {
        "path": path.as_posix(),
        "sha256": sha256(path),
        "checkpoint_status": checkpoint.get("checkpoint_status"),
        "model_contract": contract,
        "load_initialization": "same_architecture_no_download_then_strict_checkpoint",
    }


def detector_frame_map(model, metadata, rows, instances, device) -> dict:
    frames = direct_predictions(
        model,
        rows,
        index_instance_records(instances),
        device=device,
        score_threshold=OBSERVATION_THRESHOLD,
        batch_size=4,
        input_size=tuple(metadata["input_size"]),
        top_k=100,
    )
    return {
        (int(frame["scene_seed"]), int(frame["frame_index"])): frame
        for frame in frames
    }


def area_frame_map(leaf, puddle, rows, device) -> dict:
    from torch.utils.data import DataLoader

    output = {
        (int(row["scene_seed"]), int(row["frame_index"])): {} for row in rows
    }
    for class_name, model, channel, threshold in (
        ("leaf_pile", leaf, 0, AREA_THRESHOLDS[0]),
        ("puddle", puddle, 1, AREA_THRESHOLDS[1]),
    ):
        dataset = G4AreaDataset(rows, channel=channel)
        # Batch one is intentional on the shared development workstation: the
        # two DeepLab checkpoints coexist with other user GPU workloads.  A
        # larger batch caused paging/thrashing and is not representative of
        # the later isolated product-performance gate.
        loader = DataLoader(
            dataset, batch_size=1, shuffle=False, num_workers=0
        )
        offset = 0
        model.eval()
        with torch.no_grad():
            for inputs, targets, _boundaries in loader:
                probabilities = torch.sigmoid(
                    model(inputs.to(device))["logits"]
                ).cpu().numpy()
                truth_batch = targets.numpy() > 0.5
                for local_index in range(len(inputs)):
                    row = rows[offset + local_index]
                    truth = truth_batch[local_index, 0]
                    probability = probabilities[local_index, 0]
                    prediction = probability >= threshold
                    intersection = int(np.logical_and(prediction, truth).sum())
                    union = int(np.logical_or(prediction, truth).sum())
                    target_pixels = int(truth.sum())
                    output[(int(row["scene_seed"]), int(row["frame_index"]))][class_name] = {
                        "score": (
                            float(probability[truth].max()) if target_pixels else 0.0
                        ),
                        "target_coverage": intersection / max(target_pixels, 1),
                        "iou": intersection / max(union, 1),
                        "target_pixels": target_pixels,
                        "threshold": float(threshold),
                    }
                offset += len(inputs)
    return output


def scaled_bbox(native_bbox: list[float], input_size: list[int]) -> list[float]:
    return [
        native_bbox[0] * input_size[0] / 640.0,
        native_bbox[1] * input_size[1] / 480.0,
        native_bbox[2] * input_size[0] / 640.0,
        native_bbox[3] * input_size[1] / 480.0,
    ]


def target_frame_facts(
    *, row, capture_record, target, truth_fact, geometry_window, detector_frame,
    area_frame, detector_metadata, occluder_truth_fact=None,
) -> dict:
    class_name = target["class_id"]
    bbox = truth_fact["bbox"]
    valid_depth_ratio = truth_fact["depth_valid_ratio"]
    vehicle_xy = capture_record["vehicle_xy_m"]
    distance = math.hypot(
        float(target["xyz_m"][0]) - float(vehicle_xy[0]),
        float(target["xyz_m"][1]) - float(vehicle_xy[1]),
    )
    scene_visibility = float(target.get("estimated_visible_fraction", 1.0))
    actionable = actionable_window_eligible(
        visible_bbox=bbox,
        distance_m=distance,
        scene_visibility_ratio=scene_visibility,
        depth_valid_ratio=valid_depth_ratio,
        frozen_window=geometry_window,
    )
    base = {
        "frame_index": int(row["frame_index"]),
        "frame_stamp_ns": int(capture_record["timestamp_ns"]),
        "distance_m": distance,
        "vehicle_xy_m": list(vehicle_xy),
        "vehicle_yaw_rad": float(capture_record.get("vehicle_yaw_rad", 0.0)),
        "visible": bbox is not None,
        "visible_mask_area_px": truth_fact["mask_area_px"],
        "visible_bbox_xyxy_px": bbox,
        "visible_bbox_short_side_px": (
            min(bbox[2] - bbox[0], bbox[3] - bbox[1]) if bbox else 0.0
        ),
        "depth_valid_ratio": valid_depth_ratio,
        "scene_estimated_visibility_ratio": scene_visibility,
        "declared_occluder_visible_bbox_xyxy_px": (
            occluder_truth_fact["bbox"] if occluder_truth_fact else None
        ),
        "declared_occluder_bbox_iou": (
            bbox_iou(bbox, occluder_truth_fact["bbox"])
            if bbox and occluder_truth_fact and occluder_truth_fact["bbox"]
            else 0.0
        ),
        "actionable_window": actionable,
    }
    if class_name in DISCRETE_CLASSES:
        truth_box = scaled_bbox(bbox, detector_metadata["input_size"]) if bbox else None
        overlaps = []
        if truth_box:
            for prediction in detector_frame["detections"]:
                overlap = bbox_iou(truth_box, prediction["bbox_xyxy"])
                if overlap >= IOU_THRESHOLD:
                    overlaps.append((prediction, overlap))
        candidate_score = max((float(item[0]["score"]) for item in overlaps), default=0.0)
        correct_score = max(
            (
                float(item[0]["score"])
                for item in overlaps
                if item[0]["class_name"] == class_name
            ),
            default=0.0,
        )
        predicted_class = (
            max(overlaps, key=lambda item: float(item[0]["score"]))[0]["class_name"]
            if overlaps
            else None
        )
        threshold = detector_metadata["action_threshold"]
        base.update(
            {
                "model_score": candidate_score,
                "correct_class_score": correct_score,
                "predicted_class": predicted_class,
                "observation_created": candidate_score >= OBSERVATION_THRESHOLD,
                "action_detection": candidate_score >= threshold,
                "correct_action_detection": correct_score >= threshold,
            }
        )
    else:
        item = area_frame[class_name]
        target_pixels = item["target_pixels"]
        coverage = item["target_coverage"]
        overlap_iou = item["iou"]
        score = item["score"]
        action = (
            target_pixels > 0
            and coverage >= AREA_TARGET_COVERAGE_THRESHOLD
            and overlap_iou >= IOU_THRESHOLD
        )
        base.update(
            {
                "model_score": score,
                "correct_class_score": score,
                "predicted_class": class_name if action else None,
                "area_target_coverage": coverage,
                "area_iou": overlap_iou,
                "observation_created": score >= OBSERVATION_THRESHOLD,
                "action_detection": action,
                "correct_action_detection": action,
            }
        )
    return base


def false_discrete_actions(frame_map, instances, metadata) -> dict:
    truth_by_key = index_instance_records(instances)
    false_count = 0
    total_actions = 0
    negative_frame_actions = 0
    for key, frame in frame_map.items():
        truth_boxes = [
            scaled_bbox(item["bbox_xyxy_px"], metadata["input_size"])
            for item in truth_by_key.get(key, [])
        ]
        for prediction in frame["detections"]:
            if float(prediction["score"]) < metadata["action_threshold"]:
                continue
            total_actions += 1
            matched = any(
                bbox_iou(box, prediction["bbox_xyxy"]) >= IOU_THRESHOLD
                for box in truth_boxes
            )
            false_count += int(not matched)
            if frame["negative_only"] and not matched:
                negative_frame_actions += 1
    return {
        "actionable_predictions": total_actions,
        "wrong_actionable_predictions": false_count,
        "wrong_actionable_target_rate": false_count / max(total_actions, 1),
        "negative_frame_actionable_predictions": negative_frame_actions,
    }


def capture_audit(context: dict) -> dict:
    reports = list(context["capture_reports"].values())
    rates = [item["capture_timing"]["effective_captured_fps"] for item in reports]
    worlds = sorted({item["world_id"] for item in reports})
    return {
        "mission_count": len(reports),
        "frame_count": sum(int(item["captured_frames"]) for item in reports),
        "world_count": len(worlds),
        "world_ids": worlds,
        "all_capture_pass": all(item["capture_pass"] for item in reports),
        "all_exact_four_sensor_timestamp": all(
            record["exact_four_sensor_timestamp"]
            for item in reports for record in item["records"]
        ),
        "maximum_sensor_odom_skew_ns": max(
            item["sensor_odom_sync"]["maximum_skew_ns"] for item in reports
        ),
        "effective_captured_fps": {
            "minimum": min(rates),
            "median": percentile(rates, 50),
            "maximum": max(rates),
        },
        "negative_only_missions": sum(
            bool(context["scenes"][int(item["scene_seed"])].get("negative_only"))
            for item in reports
        ),
        "oprv3_coverage_profiles": sorted(
            {
                context["scenes"][int(item["scene_seed"])].get(
                    "oprv3_coverage_profile"
                )
                for item in reports
                if context["scenes"][int(item["scene_seed"])].get(
                    "oprv3_coverage_profile"
                )
            }
        ),
        "maximum_observed_absolute_yaw_change_rad": max(
            (float(item.get("observed_absolute_yaw_change_rad", 0.0)) for item in reports),
            default=0.0,
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument(
        "--detector", action="append", required=True,
        help="NAME=/absolute/path/checkpoint.pt",
    )
    parser.add_argument("--leaf-checkpoint", type=Path, required=True)
    parser.add_argument("--puddle-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--pixel-distance-output", type=Path, required=True)
    args = parser.parse_args()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("formal OPRV3 moving benchmark requires CUDA")
    geometry = load_geometry(args.geometry)
    rows, instances, context = load_development_rows(args.data_root)
    leaf, leaf_metadata = load_area_checkpoint("leaf", args.leaf_checkpoint, device)
    puddle, puddle_metadata = load_area_checkpoint(
        "puddle", args.puddle_checkpoint, device
    )
    started = time.perf_counter()
    areas = area_frame_map(leaf, puddle, rows, device)
    del leaf, puddle
    torch.cuda.empty_cache()
    routes = {}
    pixel_distance = None
    row_by_key = {
        (int(row["scene_seed"]), int(row["frame_index"])): row for row in rows
    }
    for spec in args.detector:
        name, raw_path = spec.split("=", 1)
        model, metadata = load_detector(Path(raw_path), device)
        detector_frames = detector_frame_map(model, metadata, rows, instances, device)
        del model
        torch.cuda.empty_cache()
        encounters = []
        for seed, scene in sorted(context["scenes"].items()):
            report = context["capture_reports"][seed]
            object_by_name = {
                item["model_name"]: item for item in scene.get("objects", [])
            }
            record_by_index = {
                int(item["frame_index"]): item for item in report["records"]
            }
            for target in scene.get("objects", []):
                class_name = target.get("class_id")
                if class_name not in CLASS_TO_LABEL:
                    continue
                facts = []
                for index in sorted(record_by_index):
                    key = (seed, index)
                    facts.append(
                        target_frame_facts(
                            row=row_by_key[key],
                            capture_record=record_by_index[index],
                            target=target,
                            truth_fact=context["frame_truth"][key][class_name],
                            geometry_window=geometry["class_actionable_windows"][class_name],
                            detector_frame=detector_frames[key],
                            area_frame=areas[key],
                            detector_metadata=metadata,
                            occluder_truth_fact=(
                                context["frame_truth"][key][
                                    object_by_name[target["occluded_by_model_name"]][
                                        "class_id"
                                    ]
                                ]
                                if target.get("occluded_by_model_name")
                                else None
                            ),
                        )
                    )
                encounter = summarize_encounter(
                    target,
                    facts,
                    int(geometry["vehicle_and_action"]["spot_clean_confirmation_observations"]),
                    int(geometry["class_actionable_windows"][class_name]["minimum_visible_frames"]),
                )
                encounter["scene_seed"] = seed
                encounter["world_id"] = scene["world_id"]
                encounters.append(encounter)
        false_actions = false_discrete_actions(detector_frames, instances, metadata)
        routes[name] = {
            "detector": metadata,
            "area_models": {"leaf": leaf_metadata, "puddle": puddle_metadata},
            "metrics": summarize_route(encounters, false_actions),
            "encounters": encounters,
        }
        if pixel_distance is None or name == "MRV2-C":
            pixel_distance = {
                "selected_route": name,
                "detector": metadata,
                "area_models": {"leaf": leaf_metadata, "puddle": puddle_metadata},
                "encounters": encounters,
            }
    audit = capture_audit(context)
    special_coverage = empirical_special_coverage(context, routes)
    coverage = {
        "far_first_appearance": True,
        "vehicle_gradually_approaches": True,
        "small_paper_and_can": True,
        "multiple_world_material_light": audit["world_count"] >= 3,
        "negative_regions": audit["negative_only_missions"] > 0,
        **special_coverage,
    }
    payload = {
        "schema_version": 1,
        "protocol": "OPRV3-02",
        "source_commit": repository_commit(),
        "environment": {
            "platform": platform.platform(),
            "torch": torch.__version__,
            "cuda": torch.version.cuda,
            "gpu": torch.cuda.get_device_name(0),
        },
        "gt_boundary": {
            "ground_truth_used_by": "offline_evaluator_only",
            "production_inputs": ["RGB", "depth_for_area_heads"],
            "production_target_ids_or_coordinates_provided": False,
            "actionable_eligibility_uses_model_output": False,
        },
        "thresholds": {
            "observation_threshold": OBSERVATION_THRESHOLD,
            "low_confidence_observation_is_actionable": False,
            "detector_action_threshold": "checkpoint_frozen_train_holdout",
            "bbox_iou": IOU_THRESHOLD,
            "area": list(AREA_THRESHOLDS),
            "area_target_coverage": AREA_TARGET_COVERAGE_THRESHOLD,
        },
        "capture_audit": audit,
        "required_coverage": coverage,
        "coverage_complete": all(coverage.values()),
        "routes": routes,
        "duration_s": time.perf_counter() - started,
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    pixel_payload = {
        "schema_version": 1,
        "protocol": "OPRV3-01",
        "source_commit": payload["source_commit"],
        "geometry_audit_sha256": sha256(args.geometry),
        "frozen_before_moving_model_measurement": True,
        "empirical_moving_camera_probe_executed": True,
        "capture_audit": audit,
        "gt_boundary": payload["gt_boundary"],
        **pixel_distance,
    }
    args.pixel_distance_output.parent.mkdir(parents=True, exist_ok=True)
    args.pixel_distance_output.write_text(
        json.dumps(pixel_payload, indent=2) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "output": args.output.as_posix(),
                "pixel_distance_output": args.pixel_distance_output.as_posix(),
                "capture_audit": audit,
                "coverage_complete": payload["coverage_complete"],
                "metrics": {
                    name: value["metrics"] for name, value in routes.items()
                },
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
