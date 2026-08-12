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
import os
from pathlib import Path
import platform
import re
import subprocess
import sys
import time

import cv2
import numpy as np
import torch
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_perception"))
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_spot_cleaning"))
sys.path.insert(0, str(ROOT / "scripts"))

from sanitation_learning.g4_data import (  # noqa: E402
    AREA_MODEL_SIZE,
    DISCRETE_NAMES,
    G4AreaDataset,
    build_area_input,
    index_instance_records,
    load_camera_info,
    mask_boundary,
    read_rgb,
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
from sanitation_learning.opr_c_rtmdet import patch_mmdet_cuda_nms  # noqa: E402
from sanitation_learning.g6_area_recovery import preprocess_g6_area  # noqa: E402
from sanitation_perception.camera_frustum_model import (  # noqa: E402
    CameraFrustumModel,
)
from sanitation_perception.dynamic_trash_map import (  # noqa: E402
    DynamicTrashMap,
    DynamicTrashMapConfig,
)
from sanitation_perception.map_projection_v2 import (  # noqa: E402
    mask_regions_to_map,
)
from sanitation_perception.product_pipeline_node import (  # noqa: E402
    track_to_online_observation,
)
from sanitation_perception.projection import (  # noqa: E402
    ProjectionError,
    project_pixel_to_map,
    robust_depth,
)
from sanitation_perception.tracker_v2 import (  # noqa: E402
    ProductTrackerV2,
    TrackerV2Config,
)
from sanitation_perception.trash_map_messages import TargetState  # noqa: E402
from sanitation_spot_cleaning.cleaning_task_scheduler import (  # noqa: E402
    CleaningTaskScheduler,
    CoverageContext,
    SafetyContext,
    SchedulerAction,
    TargetSchedulingInput,
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
    injected = os.environ.get("TZCUP_SOURCE_COMMIT", "").strip()
    if injected:
        if not re.fullmatch(r"[0-9a-fA-F]{40}", injected):
            raise RuntimeError(
                "TZCUP_SOURCE_COMMIT must be a full 40-character git SHA"
            )
        return injected.lower()
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
                "camera_path": scene_dir / record["paths"]["camera"],
                "tf_path": scene_dir / record["paths"]["tf"],
                "capture_record": record,
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
    model_id = str(contract.get("model_id", ""))
    if "deeplab_boundary_refine" in model_id:
        area_architecture = "deeplab_resnet50_boundary_refine"
    elif "deeplab" in model_id:
        area_architecture = "deeplab_resnet50"
    else:
        area_architecture = "dual_resnet18"
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


def load_area_gate(
    path: Path,
    *,
    leaf_checkpoint: Path,
    puddle_checkpoint: Path,
    leaf_onnx: Path | None = None,
    puddle_onnx: Path | None = None,
) -> tuple[dict, dict]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("OPRV3_06_AREA_PASS") is not True:
        raise RuntimeError("moving benchmark requires a passing OPRV3-06 Area gate")
    if payload.get("G5_SEALED_FINAL_read") is not False:
        raise RuntimeError("Area gate violates the G5 sealed-final boundary")
    if payload.get("legacy_G4_D6_read") is not False:
        raise RuntimeError("Area gate violates the legacy D6 boundary")
    checkpoint_paths = {"leaf": leaf_checkpoint, "puddle": puddle_checkpoint}
    for task, checkpoint_path in checkpoint_paths.items():
        model_record = payload.get("models", {}).get(task, {})
        expected = model_record.get("shared_training_checkpoint_sha256") or model_record.get("sha256")
        actual = sha256(checkpoint_path)
        if expected != actual:
            raise RuntimeError(
                f"Area gate {task} hash mismatch: expected {expected}, got {actual}"
            )
    onnx_paths = {"leaf": leaf_onnx, "puddle": puddle_onnx}
    if any(onnx_paths.values()) and not all(onnx_paths.values()):
        raise RuntimeError("Area ONNX paths are atomic")
    for task, onnx_path in onnx_paths.items():
        if onnx_path is None:
            continue
        expected = payload.get("models", {}).get(task, {}).get("sha256")
        actual = sha256(onnx_path)
        if expected != actual:
            raise RuntimeError(
                f"Area gate {task} ONNX hash mismatch: expected {expected}, got {actual}"
            )
    shared_hashes = {
        payload.get("models", {}).get(task, {}).get("shared_training_checkpoint_sha256")
        for task in ("leaf", "puddle")
    }
    shared_g6 = len(shared_hashes) == 1 and None not in shared_hashes
    selected_config = payload.get("selected_config", {})
    selected = selected_config.get("by_class", selected_config)
    configs = {}
    for class_name in ("leaf_pile", "puddle"):
        gate_key = "leaf" if class_name == "leaf_pile" else class_name
        record = selected.get(class_name) or selected.get(gate_key)
        if not isinstance(record, dict):
            raise RuntimeError(f"Area gate lacks {class_name} selected config")
        configs[class_name] = {
            "threshold": float(record["threshold"]),
            "morphology": str(record["morphology"]),
        }
    return configs, {
        "path": path.as_posix(),
        "sha256": sha256(path),
        "protocol": payload.get("protocol"),
        "OPRV3_06_AREA_PASS": True,
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
        "runtime_input_contract": (
            "g6_shared_rgb_hsv_depth_geometry_texture_v1"
            if shared_g6
            else "g4_task_specific_area_v1"
        ),
    }


def area_runtime_input(
    rgb: np.ndarray,
    depth: np.ndarray,
    *,
    task: str,
    camera_info: dict,
    input_contract: str,
) -> np.ndarray:
    if input_contract == "g6_shared_rgb_hsv_depth_geometry_texture_v1":
        # Persisted moving-capture depth is float32 metres, while the frozen
        # G6 preprocessing contract was trained from uint16 millimetres.
        depth_mm = np.asarray(depth, dtype=np.float32) * 1000.0
        return preprocess_g6_area(rgb, depth_mm)
    if input_contract == "g4_task_specific_area_v1":
        return np.ascontiguousarray(
            build_area_input(
                rgb,
                depth,
                AREA_MODEL_SIZE,
                task=task,
                camera_info=camera_info,
            ).transpose(2, 0, 1),
            dtype=np.float32,
        )
    raise RuntimeError(f"unsupported Area runtime input contract: {input_contract}")


def create_cuda_ort_session(path: Path):
    import onnxruntime as ort

    session = ort.InferenceSession(
        path.as_posix(), providers=["CUDAExecutionProvider"]
    )
    if not session.get_providers() or session.get_providers()[0] != "CUDAExecutionProvider":
        raise RuntimeError(
            f"ONNX Runtime silently failed CUDA provider activation for {path}"
        )
    return session


def detector_metadata_only(path: Path) -> dict:
    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    if checkpoint.get("checkpoint_status") != "training_complete_candidate_not_frozen":
        raise RuntimeError("MRV2-A checkpoint status is not a development candidate")
    return {
        "route": checkpoint["route"],
        "path": path.as_posix(),
        "sha256": sha256(path),
        "architecture": checkpoint["architecture"],
        "input_size": list(checkpoint["input_size"]),
        "action_threshold": float(
            checkpoint["frozen_threshold_from_train_world_holdout"]
        ),
        "checkpoint_status": checkpoint["checkpoint_status"],
        "execution_backend": "onnxruntime_cuda",
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
    }


def area_metadata_only(task: str, path: Path) -> dict:
    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    if checkpoint.get("checkpoint_status") not in {
        "training_complete", "training_complete_candidate_not_frozen"
    }:
        raise RuntimeError(f"{task} checkpoint is not a complete development candidate")
    return {
        "path": path.as_posix(),
        "sha256": sha256(path),
        "checkpoint_status": checkpoint["checkpoint_status"],
        "model_contract": checkpoint.get("model_contract") or {},
        "load_initialization": "metadata_only_for_hash_bound_onnx_runtime",
    }


def g6_area_metadata_only(task: str, path: Path) -> dict:
    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    if checkpoint.get("checkpoint_status") != "training_complete_candidate_not_frozen":
        raise RuntimeError(f"{task} G6 shared checkpoint is not a complete candidate")
    if checkpoint.get("model") != "G6BoundaryAwareAreaNet":
        raise RuntimeError(f"{task} checkpoint is not the G6 shared Area model")
    return {
        "path": path.as_posix(),
        "sha256": sha256(path),
        "checkpoint_status": checkpoint["checkpoint_status"],
        "model_contract": {
            "model_id": "g6_shared_boundary_aware_area_v1",
            "input_contract": "g6_shared_rgb_hsv_depth_geometry_texture_v1",
            "input_shape": [1, 10, 384, 512],
        },
        "load_initialization": "metadata_only_for_hash_bound_onnx_runtime",
    }


def load_mmdet_detector(config: Path, checkpoint: Path, selection_path: Path):
    """Load a hash-bound DDRV4 detector selected before G7 VAL was opened."""
    patch_mmdet_cuda_nms()
    from mmdet.apis import init_detector

    # PowerShell 5 commonly emits a UTF-8 BOM.  Accept it without weakening
    # the checkpoint SHA-256 or holdout-only selection checks below.
    selection = json.loads(selection_path.read_text(encoding="utf-8-sig"))
    if selection.get("selection_data") == "G7_MOVING_HOLDOUT_ONLY":
        route = "MA1"
        expected = selection.get("checkpoint_sha256")
        val_read_before_freeze = selection.get("MOVING_VAL_read_before_selection_freeze")
    else:
        route = str(selection.get("selected_route"))
        route_record = selection.get("route_results", {}).get(route, {})
        expected = route_record.get("checkpoint", {}).get("sha256")
        val_read_before_freeze = selection.get("G7_VAL_read_before_selection_freeze")
    if selection.get("selection_data") not in {"G7_IN_DOMAIN_HOLDOUT_ONLY", "G7_MOVING_HOLDOUT_ONLY"}:
        raise RuntimeError("MMDetection detector selection was not holdout-only")
    if val_read_before_freeze is not False:
        raise RuntimeError("MMDetection detector selection violated the VAL boundary")
    if expected != sha256(checkpoint):
        raise RuntimeError("DDRV4 selected checkpoint SHA-256 mismatch")
    metadata = {
        "route": route,
        "path": checkpoint.as_posix(),
        "sha256": expected,
        "architecture": "official_mmdetection_v3.3.0_rtmdet_s",
        "input_size": [640, 480],
        "action_threshold": float(selection["selected_threshold"]),
        "checkpoint_status": "training_complete_candidate_not_frozen",
        "execution_backend": "mmdetection_pytorch_cuda",
        "selection_path": selection_path.as_posix(),
        "selection_sha256": sha256(selection_path),
        "G5_SEALED_FINAL_read": False,
        "G5_V2_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
    }
    return init_detector(str(config), str(checkpoint), device="cuda:0"), metadata


def detector_frame_map_mmdet(model, metadata: dict, rows: list[dict], batch_size: int = 8) -> dict:
    """Run MMDetection from RGB only and preserve native-image coordinates."""
    from mmdet.apis import inference_detector

    frames: dict[tuple[int, int], dict] = {}
    for offset in range(0, len(rows), batch_size):
        batch = rows[offset : offset + batch_size]
        # MMDetection ndarray input follows the OpenCV BGR convention.
        images = [np.ascontiguousarray(read_rgb(row)[..., ::-1]) for row in batch]
        outputs = inference_detector(model, images)
        if not isinstance(outputs, list):
            outputs = [outputs]
        for row, output in zip(batch, outputs):
            predictions = output.pred_instances.to("cpu")
            detections = []
            for box, score, label in zip(
                predictions.bboxes.tolist(),
                predictions.scores.tolist(),
                predictions.labels.tolist(),
            ):
                if float(score) < OBSERVATION_THRESHOLD:
                    continue
                class_index = int(label)
                if not 0 <= class_index < len(DISCRETE_NAMES):
                    raise RuntimeError(f"detector produced invalid class index {class_index}")
                detections.append(
                    {
                        "class_name": DISCRETE_NAMES[class_index],
                        "score": float(score),
                        "bbox_xyxy": [float(value) for value in box],
                    }
                )
            detections.sort(key=lambda item: item["score"], reverse=True)
            key = (int(row["scene_seed"]), int(row["frame_index"]))
            frames[key] = {
                "scene_seed": key[0],
                "frame_index": key[1],
                "split": row["split"],
                "world_id": row["world_id"],
                "negative_only": bool(row.get("negative_only", False)),
                "detections": detections[:100],
            }
    return frames


def detector_frame_map_onnx(session, metadata, rows) -> dict:
    input_width, input_height = metadata["input_size"]
    frames = {}
    input_name = session.get_inputs()[0].name
    for row in rows:
        resized = cv2.resize(
            read_rgb(row),
            (input_width, input_height),
            interpolation=cv2.INTER_CUBIC,
        )
        images = np.ascontiguousarray(
            resized.transpose(2, 0, 1)[None], dtype=np.float32
        ) / 255.0
        boxes, scores, labels = session.run(None, {input_name: images})
        detections = []
        for box, score, label in zip(boxes, scores, labels):
            if float(score) < OBSERVATION_THRESHOLD:
                continue
            class_index = int(label)
            if not 0 <= class_index < len(DISCRETE_NAMES):
                raise RuntimeError(
                    f"detector produced invalid class index {class_index}"
                )
            detections.append(
                {
                    "class_name": DISCRETE_NAMES[class_index],
                    "score": float(score),
                    "bbox_xyxy": [float(value) for value in box],
                }
            )
        detections.sort(key=lambda item: item["score"], reverse=True)
        key = (int(row["scene_seed"]), int(row["frame_index"]))
        frames[key] = {
            "scene_seed": key[0],
            "frame_index": key[1],
            "split": row["split"],
            "world_id": row["world_id"],
            "negative_only": bool(row.get("negative_only", False)),
            "detections": detections[:100],
        }
    return frames


def area_frame_map_onnx(
    leaf,
    puddle,
    rows,
    *,
    area_configs: dict,
    camera_pitch_down_rad: float,
    minimum_physical_area_m2: float,
    minimum_physical_area_m2_by_class: dict[str, float] | None = None,
    input_contract: str = "g4_task_specific_area_v1",
) -> tuple[dict, dict]:
    output = {
        (int(row["scene_seed"]), int(row["frame_index"])): {} for row in rows
    }
    product_detections = {key: [] for key in output}
    for class_name, session, channel in (
        ("leaf_pile", leaf, 0),
        ("puddle", puddle, 1),
    ):
        threshold = float(area_configs[class_name]["threshold"])
        morphology = str(area_configs[class_name]["morphology"])
        dataset = G4AreaDataset(rows, channel=channel)
        input_name = session.get_inputs()[0].name
        for index in range(len(dataset)):
            row = rows[index]
            inputs, target, _boundary = dataset[index]
            if input_contract == "g6_shared_rgb_hsv_depth_geometry_texture_v1":
                rgb = read_rgb(row)
                depth = np.load(row["depth_path"], allow_pickle=False).astype(np.float32)
                inputs = torch.from_numpy(
                    area_runtime_input(
                        rgb,
                        depth,
                        task="leaf" if class_name == "leaf_pile" else "puddle",
                        camera_info=load_camera_info(row),
                        input_contract=input_contract,
                    )
                )
            logits = session.run(
                None,
                {input_name: inputs.numpy()[None].astype(np.float32)},
            )[0]
            stable = np.clip(
                np.asarray(logits[0, 0], dtype=np.float32), -80.0, 80.0
            )
            probability = 1.0 / (1.0 + np.exp(-stable))
            truth = target.numpy()[0] > 0.5
            prediction = apply_area_morphology(
                probability >= threshold, morphology
            )
            intersection = int(np.logical_and(prediction, truth).sum())
            union = int(np.logical_or(prediction, truth).sum())
            target_pixels = int(truth.sum())
            key = (int(row["scene_seed"]), int(row["frame_index"]))
            output[key][class_name] = {
                "score": (
                    float(probability[truth].max()) if target_pixels else 0.0
                ),
                "target_coverage": intersection / max(target_pixels, 1),
                "iou": intersection / max(union, 1),
                "target_pixels": target_pixels,
                "threshold": threshold,
            }
            product_detections[key].extend(
                project_area_frame(
                    probability,
                    row,
                    class_name,
                    threshold,
                    morphology=morphology,
                    camera_pitch_down_rad=camera_pitch_down_rad,
                    minimum_physical_area_m2=(
                        minimum_physical_area_m2_by_class.get(
                            class_name, minimum_physical_area_m2
                        )
                        if minimum_physical_area_m2_by_class
                        else minimum_physical_area_m2
                    ),
                    source_backend="onnxruntime_cuda",
                )
            )
    return output, product_detections


def combined_frame_maps_onnx(
    detector,
    detector_metadata,
    leaf,
    puddle,
    rows,
    *,
    area_configs: dict,
    camera_pitch_down_rad: float,
    minimum_physical_area_m2: float,
    minimum_physical_area_m2_by_class: dict[str, float] | None = None,
    area_input_contract: str = "g4_task_specific_area_v1",
) -> tuple[dict, dict, dict]:
    """Run all three ONNX heads in one pass over persisted replay frames."""
    area_output = {}
    area_product_detections = {}
    detector_frames = {}
    detector_name = detector.get_inputs()[0].name
    area_sessions = {"leaf_pile": leaf, "puddle": puddle}
    input_width, input_height = detector_metadata["input_size"]
    for row in rows:
        key = (int(row["scene_seed"]), int(row["frame_index"]))
        rgb = read_rgb(row)
        depth = np.load(row["depth_path"], allow_pickle=False).astype(np.float32)
        camera_info = load_camera_info(row)
        projection_inputs = camera_projection_inputs(
            row, camera_pitch_down_rad=camera_pitch_down_rad
        )
        resized = cv2.resize(
            rgb, (input_width, input_height), interpolation=cv2.INTER_CUBIC
        )
        images = np.ascontiguousarray(
            resized.transpose(2, 0, 1)[None], dtype=np.float32
        ) / 255.0
        boxes, scores, labels = detector.run(None, {detector_name: images})
        detections = []
        for box, score, label in zip(boxes, scores, labels):
            if float(score) < OBSERVATION_THRESHOLD:
                continue
            class_index = int(label)
            if not 0 <= class_index < len(DISCRETE_NAMES):
                raise RuntimeError(
                    f"detector produced invalid class index {class_index}"
                )
            detections.append(
                {
                    "class_name": DISCRETE_NAMES[class_index],
                    "score": float(score),
                    "bbox_xyxy": [float(value) for value in box],
                }
            )
        detections.sort(key=lambda item: item["score"], reverse=True)
        detector_frames[key] = {
            "scene_seed": key[0],
            "frame_index": key[1],
            "split": row["split"],
            "world_id": row["world_id"],
            "negative_only": bool(row.get("negative_only", False)),
            "detections": detections[:100],
        }
        semantic = np.load(row["semantic_path"], allow_pickle=False)
        semantic_model = cv2.resize(
            semantic, AREA_MODEL_SIZE, interpolation=cv2.INTER_NEAREST
        )
        area_output[key] = {}
        area_product_detections[key] = []
        for class_name, task, semantic_id in (
            ("leaf_pile", "leaf", 4),
            ("puddle", "puddle", 5),
        ):
            session = area_sessions[class_name]
            inputs = area_runtime_input(
                rgb,
                depth,
                task=task,
                camera_info=camera_info,
                input_contract=area_input_contract,
            )
            logits = session.run(
                None,
                {
                    session.get_inputs()[0].name: np.ascontiguousarray(
                        inputs[None], dtype=np.float32
                    )
                },
            )[0]
            stable = np.clip(
                np.asarray(logits[0, 0], dtype=np.float32), -80.0, 80.0
            )
            probability = 1.0 / (1.0 + np.exp(-stable))
            threshold = float(area_configs[class_name]["threshold"])
            morphology = str(area_configs[class_name]["morphology"])
            prediction = apply_area_morphology(
                probability >= threshold, morphology
            )
            truth = semantic_model == semantic_id
            if row.get("negative_only"):
                truth = np.zeros_like(truth)
            intersection = int(np.logical_and(prediction, truth).sum())
            union = int(np.logical_or(prediction, truth).sum())
            prediction_boundary = mask_boundary(prediction) > 0
            truth_boundary = mask_boundary(truth) > 0
            boundary_intersection = int(
                np.logical_and(prediction_boundary, truth_boundary).sum()
            )
            boundary_union = int(
                np.logical_or(prediction_boundary, truth_boundary).sum()
            )
            region_count, _, region_stats, _ = cv2.connectedComponentsWithStats(
                prediction.astype(np.uint8), 8
            )
            has_area_candidate = bool(
                region_count > 1
                and int(region_stats[1:, cv2.CC_STAT_AREA].max()) >= 20
            )
            target_pixels = int(truth.sum())
            area_output[key][class_name] = {
                "score": (
                    float(probability[truth].max()) if target_pixels else 0.0
                ),
                "target_coverage": intersection / max(target_pixels, 1),
                "iou": intersection / max(union, 1),
                "intersection_pixels": intersection,
                "union_pixels": union,
                "boundary_intersection_pixels": boundary_intersection,
                "boundary_union_pixels": boundary_union,
                "predicted_pixels": int(prediction.sum()),
                "has_area_candidate": has_area_candidate,
                "target_pixels": target_pixels,
                "threshold": threshold,
            }
            area_product_detections[key].extend(
                project_area_frame(
                    probability,
                    row,
                    class_name,
                    threshold,
                    morphology=morphology,
                    camera_pitch_down_rad=camera_pitch_down_rad,
                    minimum_physical_area_m2=(
                        minimum_physical_area_m2_by_class.get(
                            class_name, minimum_physical_area_m2
                        )
                        if minimum_physical_area_m2_by_class
                        else minimum_physical_area_m2
                    ),
                    depth=depth,
                    projection_inputs=projection_inputs,
                    source_backend="onnxruntime_cuda",
                )
            )
    return area_output, area_product_detections, detector_frames


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


def camera_projection_inputs(
    row: dict, *, camera_pitch_down_rad: float
) -> tuple[dict, np.ndarray]:
    """Load calibration and reconstruct map<-optical TF from captured odometry.

    The capture persists base pose plus the fixed base-to-camera translation.
    Camera optical +Z is robot-forward, +X is robot-right, and +Y is down.
    """
    calibration = json.loads(row["camera_path"].read_text(encoding="utf-8"))
    tf = json.loads(row["tf_path"].read_text(encoding="utf-8"))
    k = calibration["k"]
    camera = {
        "fx": float(k[0]),
        "fy": float(k[4]),
        "cx": float(k[2]),
        "cy": float(k[5]),
        "pixel_sigma": 0.5,
        "depth_sigma_m": 0.02,
    }
    capture = row["capture_record"]
    base_x, base_y = (float(value) for value in capture["vehicle_xy_m"])
    yaw = float(capture.get("vehicle_yaw_rad", 0.0))
    pitch_cos = math.cos(camera_pitch_down_rad)
    pitch_sin = math.sin(camera_pitch_down_rad)
    planar_forward = np.asarray([math.cos(yaw), math.sin(yaw), 0.0])
    forward = np.asarray(
        [
            pitch_cos * math.cos(yaw),
            pitch_cos * math.sin(yaw),
            -pitch_sin,
        ]
    )
    right = np.asarray([math.sin(yaw), -math.cos(yaw), 0.0])
    down = np.asarray(
        [
            -pitch_sin * math.cos(yaw),
            -pitch_sin * math.sin(yaw),
            -pitch_cos,
        ]
    )
    offset_x, offset_y, offset_z = (
        float(value) for value in tf["base_to_camera_xyz_m"]
    )
    base_left = np.asarray([-math.sin(yaw), math.cos(yaw), 0.0])
    translation = (
        np.asarray([base_x, base_y, 0.0])
        + offset_x * planar_forward
        + offset_y * base_left
        + np.asarray([0.0, 0.0, offset_z])
    )
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.column_stack((right, down, forward))
    transform[:3, 3] = translation
    return camera, transform


def project_discrete_frame(
    frame: dict,
    row: dict,
    detector_metadata: dict,
    *,
    camera_pitch_down_rad: float,
    depth: np.ndarray | None = None,
    projection_inputs: tuple[dict, np.ndarray] | None = None,
) -> list[dict]:
    if depth is None:
        depth = np.load(row["depth_path"], allow_pickle=False).astype(np.float32)
    camera, transform = (
        projection_inputs
        if projection_inputs is not None
        else camera_projection_inputs(
            row, camera_pitch_down_rad=camera_pitch_down_rad
        )
    )
    native_height, native_width = depth.shape
    input_width, input_height = detector_metadata["input_size"]
    projected = []
    for prediction in frame["detections"]:
        confidence = float(prediction["score"])
        if confidence < float(detector_metadata["action_threshold"]):
            continue
        x1, y1, x2, y2 = (float(value) for value in prediction["bbox_xyxy"])
        native = (
            max(0, int(math.floor(x1 * native_width / input_width))),
            max(0, int(math.floor(y1 * native_height / input_height))),
            min(native_width, int(math.ceil(x2 * native_width / input_width))),
            min(native_height, int(math.ceil(y2 * native_height / input_height))),
        )
        if native[2] <= native[0] or native[3] <= native[1]:
            continue
        inset_x = max(1, int((native[2] - native[0]) * 0.2))
        inset_y = max(1, int((native[3] - native[1]) * 0.2))
        left, right_edge = native[0] + inset_x, native[2] - inset_x
        top, bottom = native[1] + inset_y, native[3] - inset_y
        if right_edge <= left or bottom <= top:
            left, top, right_edge, bottom = native
        try:
            depth_m = robust_depth(depth[top:bottom, left:right_edge].reshape(-1))
            xyz, covariance = project_pixel_to_map(
                (native[0] + native[2]) * 0.5,
                (native[1] + native[3]) * 0.5,
                depth_m,
                camera,
                transform,
            )
        except ProjectionError:
            continue
        class_name = str(prediction["class_name"])
        projected.append(
            {
                "class_id": class_name,
                "class_probabilities": {
                    class_name: confidence,
                    "background": 1.0 - confidence,
                },
                "confidence": confidence,
                "bbox_xyxy": native,
                "x_m": float(xyz[0]),
                "y_m": float(xyz[1]),
                "z_m": float(xyz[2]),
                "covariance_trace": float(np.trace(covariance[:2, :2])),
                "source_backend": detector_metadata.get(
                    "execution_backend", "pytorch_cuda_development"
                ),
                "target_type": "DISCRETE",
            }
        )
    return projected


def project_area_frame(
    probability: np.ndarray,
    row: dict,
    class_name: str,
    threshold: float,
    *,
    morphology: str,
    camera_pitch_down_rad: float,
    minimum_physical_area_m2: float,
    source_backend: str = "pytorch_cuda_development",
    depth: np.ndarray | None = None,
    projection_inputs: tuple[dict, np.ndarray] | None = None,
) -> list[dict]:
    if depth is None:
        depth = np.load(row["depth_path"], allow_pickle=False).astype(np.float32)
    native_probability = cv2.resize(
        probability.astype(np.float32),
        (depth.shape[1], depth.shape[0]),
        interpolation=cv2.INTER_LINEAR,
    )
    binary = apply_area_morphology(
        native_probability >= float(threshold), morphology
    )
    camera, transform = (
        projection_inputs
        if projection_inputs is not None
        else camera_projection_inputs(
            row, camera_pitch_down_rad=camera_pitch_down_rad
        )
    )
    regions = mask_regions_to_map(
        binary,
        native_probability,
        depth,
        camera,
        transform,
        minimum_pixels=20,
        minimum_physical_area_m2=minimum_physical_area_m2,
    )
    projected = []
    for region in regions:
        center = np.mean(np.asarray(region.polygon_xy_m), axis=0)
        projected.append(
            {
                "class_id": class_name,
                "class_probabilities": {
                    class_name: region.confidence,
                    "background": 1.0 - region.confidence,
                },
                "confidence": region.confidence,
                "bbox_xyxy": None,
                "x_m": float(center[0]),
                "y_m": float(center[1]),
                "z_m": 0.0,
                "covariance_trace": float(
                    region.covariance_xy[0][0]
                    + region.covariance_xy[1][1]
                ),
                "polygon_xy_m": region.polygon_xy_m,
                "physical_area_m2": region.physical_area_m2,
                "pixel_area": region.pixel_area,
                "source_backend": source_backend,
                "target_type": "AREA",
            }
        )
    return projected


def apply_area_morphology(mask: np.ndarray, name: str) -> np.ndarray:
    source = np.asarray(mask, dtype=np.uint8)
    kernel = np.ones((3, 3), dtype=np.uint8)
    if name == "none":
        result = source
    elif name == "open3":
        result = cv2.morphologyEx(source, cv2.MORPH_OPEN, kernel)
    elif name == "close3":
        result = cv2.morphologyEx(source, cv2.MORPH_CLOSE, kernel)
    elif name == "open_close3":
        result = cv2.morphologyEx(
            cv2.morphologyEx(source, cv2.MORPH_OPEN, kernel),
            cv2.MORPH_CLOSE,
            kernel,
        )
    elif name == "dilate3":
        result = cv2.dilate(source, kernel)
    elif name == "dilate5":
        result = cv2.dilate(source, np.ones((5, 5), dtype=np.uint8))
    elif name == "erode3":
        result = cv2.erode(source, kernel)
    else:
        raise ValueError(f"unsupported frozen Area morphology: {name}")
    return result.astype(bool)


def area_frame_map(
    leaf,
    puddle,
    rows,
    device,
    *,
    area_configs: dict,
    camera_pitch_down_rad: float,
    minimum_physical_area_m2: float,
    minimum_physical_area_m2_by_class: dict[str, float] | None = None,
) -> tuple[dict, dict]:
    from torch.utils.data import DataLoader

    output = {
        (int(row["scene_seed"]), int(row["frame_index"])): {} for row in rows
    }
    product_detections = {key: [] for key in output}
    for class_name, model, channel in (
        ("leaf_pile", leaf, 0),
        ("puddle", puddle, 1),
    ):
        threshold = float(area_configs[class_name]["threshold"])
        morphology = str(area_configs[class_name]["morphology"])
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
                    prediction = apply_area_morphology(
                        probability >= threshold, morphology
                    )
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
                    product_detections[
                        (int(row["scene_seed"]), int(row["frame_index"]))
                    ].extend(
                        project_area_frame(
                            probability,
                            row,
                            class_name,
                            threshold,
                            morphology=morphology,
                            camera_pitch_down_rad=camera_pitch_down_rad,
                            minimum_physical_area_m2=(
                                minimum_physical_area_m2_by_class.get(
                                    class_name, minimum_physical_area_m2
                                )
                                if minimum_physical_area_m2_by_class
                                else minimum_physical_area_m2
                            ),
                        )
                    )
                offset += len(inputs)
    return output, product_detections


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


def _nearest_gt(
    x_m: float, y_m: float, targets: list[dict], maximum_distance_m: float
) -> tuple[dict | None, float | None]:
    candidates = [
        (
            math.hypot(
                x_m - float(target["world_xyz_m"][0]),
                y_m - float(target["world_xyz_m"][1]),
            ),
            target,
        )
        for target in targets
    ]
    if not candidates:
        return None, None
    distance, target = min(candidates, key=lambda item: item[0])
    return (target, distance) if distance <= maximum_distance_m else (None, distance)


def _partition_product_map_encounters(encounters: list[dict]):
    """Separate detector-window GT from product-map scoring GT.

    Detection metrics are intentionally limited to targets that enter the
    frozen actionable window. Product-map precision instead recognizes every
    real target that entered the camera frustum. Otherwise a correct online
    observation outside the detector window is mislabeled as a false confirmed
    product target during the offline-only GT audit.
    """
    actionable_groups = defaultdict(list)
    map_scorable_groups = defaultdict(list)
    for encounter in encounters:
        seed = int(encounter["scene_seed"])
        if encounter.get("entered_actionable_window"):
            actionable_groups[seed].append(encounter)
        if encounter.get("ever_in_camera_frustum"):
            map_scorable_groups[seed].append(encounter)
    return actionable_groups, map_scorable_groups


def schedule_current_target(
    scheduler: CleaningTaskScheduler,
    dynamic_map: DynamicTrashMap,
    target,
    capture: dict,
) -> dict | None:
    """Exercise the shipped scheduler from current online state only.

    Safe Coverage opportunities are frozen by frame cadence, independent of
    GT and model results.  A target can therefore become actionable only when
    it is currently confirmed/deferred and the product map has a fresh online
    observation for the current frame.
    """
    if target.track_state not in {TargetState.CONFIRMED, TargetState.DEFERRED}:
        return None
    frame_index = int(capture["frame_index"])
    vehicle_x, vehicle_y = (float(value) for value in capture["vehicle_xy_m"])
    route_distance = math.hypot(
        target.map_x_m - vehicle_x, target.map_y_m - vehicle_y
    )
    safe_boundary = frame_index % 10 == 0
    decision = scheduler.decide(
        TargetSchedulingInput(
            target_uuid=target.uuid,
            track_state=target.track_state.value,
            confidence=target.confidence,
            observation_count=target.observation_count,
            distance_from_current_route_m=route_distance,
            detour_length_m=route_distance,
            target_priority=1.0,
            cleaning_cost=0.20,
            return_to_coverage_cost=0.20,
            source_models=tuple(target.source_models),
        ),
        CoverageContext(
            coverage_state="RUNNING",
            at_component_boundary=safe_boundary,
            current_swath_state=(
                "SWATH_COMPLETE" if safe_boundary else "CLEANING"
            ),
        ),
        SafetyContext(
            nav2_path_available=True,
            keepout_clear=True,
            dynamic_obstacle_clear=True,
            localization_healthy=True,
            perception_healthy=True,
            footprint_clearance_m=0.30,
            covariance_trace=target.covariance_trace,
        ),
    )
    if decision.action == SchedulerAction.CLEAN_NOW:
        dynamic_map.transition(
            target.uuid,
            TargetState.SCHEDULED,
            int(capture["timestamp_ns"]),
            "oprv3_development_scheduler_clean_now",
        )
    elif (
        decision.action == SchedulerAction.DEFER
        and target.track_state == TargetState.CONFIRMED
    ):
        dynamic_map.transition(
            target.uuid,
            TargetState.DEFERRED,
            int(capture["timestamp_ns"]),
            "oprv3_development_scheduler_defer",
        )
    return {
        **decision.to_record(),
        "stamp_ns": int(capture["timestamp_ns"]),
        "frame_index": frame_index,
        "class_name": target.current_class,
        "x_m": target.map_x_m,
        "y_m": target.map_y_m,
        "fresh_online_observation": True,
        "coverage_safe_boundary": safe_boundary,
    }


def product_map_evaluation(
    *, rows, context, detector_frames, detector_metadata, area_detections,
    encounters, camera_pitch_down_rad, manifest,
) -> dict:
    """Run prediction-derived observations through the shipped tracker/map.

    GT positions are used only after the product map has completed each
    mission, to score localization and identity consistency.
    """
    runtime = manifest["runtime"]
    tracker_config = TrackerV2Config.from_pipeline_manifest(manifest)
    map_config = DynamicTrashMapConfig(**runtime["dynamic_trash_map"])
    frustum = CameraFrustumModel(**runtime["camera_frustum"])
    row_groups = defaultdict(list)
    for row in rows:
        row_groups[int(row["scene_seed"])].append(row)
    (
        actionable_encounter_groups,
        map_scorable_encounter_groups,
    ) = _partition_product_map_encounters(encounters)

    mission_reports = []
    all_final_errors = []
    all_identity_observations = defaultdict(list)
    total_eligible = 0
    total_matched = 0
    total_confirmed = 0
    discrete_eligible = 0
    discrete_matched = 0
    discrete_confirmed = 0
    area_eligible = 0
    area_matched = 0
    area_confirmed = 0
    pre_fov_creations = 0
    wrong_class_confirmed_actions = 0
    projection_failures = 0
    projection_eligible_correct_detections = 0
    projection_successful_correct_detections = 0
    direct_projection_errors = []
    removed_target_stale_actions = 0
    removal_capture_count = 0
    evaluator_truth_by_frame = defaultdict(list)
    for encounter in encounters:
        if encounter.get("class_name") not in DISCRETE_NAMES:
            continue
        for frame in encounter.get("frames", []):
            if not frame.get("correct_action_detection"):
                continue
            bbox = frame.get("visible_bbox_xyxy_px")
            if bbox is None:
                continue
            evaluator_truth_by_frame[
                (int(encounter["scene_seed"]), int(frame["frame_index"]))
            ].append(
                {
                    "class_name": encounter["class_name"],
                    "bbox_xyxy": bbox,
                    "world_xyz_m": encounter["world_xyz_m"],
                }
            )
    for seed, mission_rows in sorted(row_groups.items()):
        mission_id = f"oprv3-dev-{seed}"
        tracker = ProductTrackerV2(tracker_config)
        cleaning_scheduler = CleaningTaskScheduler()
        dynamic_map = DynamicTrashMap.start_new(
            mission_id, config=map_config
        )
        accepted_records = []
        scheduler_records = []
        mission_rows.sort(key=lambda item: int(item["frame_index"]))
        for row in mission_rows:
            key = (seed, int(row["frame_index"]))
            capture = row["capture_record"]
            stamp_ns = int(capture["timestamp_ns"])
            image_frame_id = f"camera_depth_link:{stamp_ns}"
            camera, transform = camera_projection_inputs(
                row, camera_pitch_down_rad=camera_pitch_down_rad
            )
            sweep = frustum.make_sweep(
                sweep_id=f"sweep:{stamp_ns}",
                mission_id=mission_id,
                stamp_ns=stamp_ns,
                camera_frame_id="camera_depth_link",
                image_frame_id=image_frame_id,
                camera_x_m=float(transform[0, 3]),
                camera_y_m=float(transform[1, 3]),
                camera_yaw_rad=float(capture.get("vehicle_yaw_rad", 0.0)),
            )
            dynamic_map.observed_regions.record(sweep)
            try:
                detections = project_discrete_frame(
                    detector_frames[key],
                    row,
                    detector_metadata,
                    camera_pitch_down_rad=camera_pitch_down_rad,
                )
            except (OSError, ValueError, ProjectionError):
                projection_failures += 1
                detections = []
            frame_truth = evaluator_truth_by_frame[key]
            projection_eligible_correct_detections += len(frame_truth)
            used_detection_indices = set()
            for truth in frame_truth:
                candidates = [
                    (
                        bbox_iou(
                            detection["bbox_xyxy"], truth["bbox_xyxy"]
                        ),
                        index,
                        detection,
                    )
                    for index, detection in enumerate(detections)
                    if index not in used_detection_indices
                    and detection["class_id"] == truth["class_name"]
                ]
                if not candidates:
                    continue
                overlap, index, detection = max(candidates, key=lambda item: item[0])
                if overlap < 0.50:
                    continue
                used_detection_indices.add(index)
                projection_successful_correct_detections += 1
                direct_projection_errors.append(
                    math.hypot(
                        float(detection["x_m"])
                        - float(truth["world_xyz_m"][0]),
                        float(detection["y_m"])
                        - float(truth["world_xyz_m"][1]),
                    )
                )
            detections.extend(area_detections[key])
            stamp_s = stamp_ns / 1_000_000_000.0
            tracks = tracker.update(detections, stamp_s)
            for track in tracks:
                if abs(track.last_seen_s - stamp_s) > 1e-6:
                    continue
                observation = track_to_online_observation(
                    track,
                    mission_id=mission_id,
                    stamp_ns=stamp_ns,
                    camera_frame_id="camera_depth_link",
                    image_frame_id=image_frame_id,
                    source_model="MRV2-A-oprv3-development",
                )
                target = dynamic_map.ingest(observation)
                if target is not None:
                    accepted_records.append(
                        {
                            "stamp_ns": stamp_ns,
                            "track_uuid": track.uuid,
                            "map_uuid": target.uuid,
                            "class_name": target.current_class,
                            "x_m": target.map_x_m,
                            "y_m": target.map_y_m,
                        }
                    )
                    decision = schedule_current_target(
                        cleaning_scheduler, dynamic_map, target, capture
                    )
                    if decision is not None:
                        scheduler_records.append(decision)
            dynamic_map.expire(stamp_ns)

        actionable = actionable_encounter_groups[seed]
        map_scorable = map_scorable_encounter_groups[seed]
        total_eligible += len(map_scorable)
        mission_discrete_gt = [
            item for item in map_scorable if item["class_name"] in DISCRETE_NAMES
        ]
        mission_area_gt = [
            item for item in map_scorable if item["class_name"] not in DISCRETE_NAMES
        ]
        discrete_eligible += len(mission_discrete_gt)
        area_eligible += len(mission_area_gt)
        per_gt_ids = defaultdict(list)
        for record in accepted_records:
            target, _distance = _nearest_gt(
                record["x_m"],
                record["y_m"],
                [
                    item
                    for item in map_scorable
                    if item["class_name"] == record["class_name"]
                ],
                0.50,
            )
            if target is not None:
                per_gt_ids[target["target_id"]].append(record["map_uuid"])
                all_identity_observations[
                    f"{seed}:{target['target_id']}"
                ].append(
                    record["map_uuid"]
                )

        confirmed = [
            target
            for target in dynamic_map.targets.values()
            if any(
                transition.get("to") == "CONFIRMED"
                for transition in target.transitions
            )
        ]
        matched_gt_ids = set()
        matched_discrete_gt_ids = set()
        matched_area_gt_ids = set()
        mission_errors = []
        mission_wrong_class = 0
        mission_pre_fov = 0
        earliest_mission_visibility = min(
            (
                int(frame["frame_stamp_ns"])
                for gt in map_scorable
                for frame in gt["frames"]
                if frame["visible"]
            ),
            default=None,
        )
        for target in confirmed:
            gt, distance = _nearest_gt(
                target.map_x_m,
                target.map_y_m,
                [
                    item
                    for item in map_scorable
                    if item["class_name"] == target.current_class
                ],
                0.50,
            )
            if gt is None:
                if (
                    earliest_mission_visibility is not None
                    and target.first_seen_stamp_ns < earliest_mission_visibility
                ):
                    mission_pre_fov += 1
                continue
            matched_gt_ids.add(gt["target_id"])
            if gt["class_name"] in DISCRETE_NAMES:
                matched_discrete_gt_ids.add(gt["target_id"])
            else:
                matched_area_gt_ids.add(gt["target_id"])
            mission_errors.append(float(distance))
            first_visible_stamps = [
                int(frame["frame_stamp_ns"])
                for frame in gt["frames"]
                if frame["visible"]
            ]
            if first_visible_stamps and target.first_seen_stamp_ns < min(
                first_visible_stamps
            ):
                mission_pre_fov += 1
        total_confirmed += len(confirmed)
        mission_discrete_confirmed = sum(
            target.target_type == "DISCRETE" for target in confirmed
        )
        mission_area_confirmed = sum(
            target.target_type == "AREA" for target in confirmed
        )
        discrete_confirmed += mission_discrete_confirmed
        area_confirmed += mission_area_confirmed
        clean_actions = [
            record
            for record in scheduler_records
            if record["action"] == SchedulerAction.CLEAN_NOW.value
        ]
        for action in clean_actions:
            gt, _distance = _nearest_gt(
                action["x_m"], action["y_m"], map_scorable, 0.50
            )
            if gt is not None and action["class_name"] != gt["class_name"]:
                mission_wrong_class += 1

        capture_report = context["capture_reports"][seed]
        scene = context["scenes"][seed]
        removal_plan = scene.get("dynamic_removal_plan")
        removal_events = capture_report.get("dynamic_removal_events", [])
        mission_stale_actions = None
        if removal_plan is not None:
            if (
                capture_report.get("dynamic_removal_requested") is not True
                or capture_report.get("dynamic_removal_executed") is not True
                or len(removal_events) != 1
            ):
                raise RuntimeError(
                    f"scene {seed} dynamic removal capture is incomplete"
                )
            removal_capture_count += 1
            event = removal_events[0]
            first_post_index = int(event["first_post_removal_frame"])
            first_post_stamp = next(
                int(row["capture_record"]["timestamp_ns"])
                for row in mission_rows
                if int(row["frame_index"]) == first_post_index
            )
            removed_xyz = removal_plan["initial_xyz_m"]
            mission_stale_actions = sum(
                action["stamp_ns"] >= first_post_stamp
                and math.hypot(
                    action["x_m"] - float(removed_xyz[0]),
                    action["y_m"] - float(removed_xyz[1]),
                )
                <= 0.50
                for action in clean_actions
            )
            removed_target_stale_actions += mission_stale_actions
        total_matched += len(matched_gt_ids)
        discrete_matched += len(matched_discrete_gt_ids)
        area_matched += len(matched_area_gt_ids)
        all_final_errors.extend(mission_errors)
        pre_fov_creations += mission_pre_fov
        wrong_class_confirmed_actions += mission_wrong_class
        mission_reports.append(
            {
                "scene_seed": seed,
                "world_id": context["scenes"][seed]["world_id"],
                "frames": len(mission_rows),
                "actionable_detection_targets": len(actionable),
                "eligible_targets": len(map_scorable),
                "confirmed_product_targets": len(confirmed),
                "matched_eligible_targets": len(matched_gt_ids),
                "discrete_eligible_targets": len(mission_discrete_gt),
                "discrete_matched_eligible_targets": len(matched_discrete_gt_ids),
                "discrete_confirmed_product_targets": mission_discrete_confirmed,
                "area_eligible_targets": len(mission_area_gt),
                "area_matched_eligible_targets": len(matched_area_gt_ids),
                "area_confirmed_product_targets": mission_area_confirmed,
                "map_rmse_m": (
                    math.sqrt(
                        sum(value * value for value in mission_errors)
                        / len(mission_errors)
                    )
                    if mission_errors
                    else None
                ),
                "pre_fov_target_creation": mission_pre_fov,
                "wrong_class_confirmed_action": mission_wrong_class,
                "clean_now_actions": len(clean_actions),
                "scheduler_decisions": len(scheduler_records),
                "removed_target_stale_action": mission_stale_actions,
                "accepted_online_observations": len(accepted_records),
                "product_targets": [
                    {
                        "uuid": target.uuid,
                        "target_type": target.target_type,
                        "class_name": target.current_class,
                        "track_state": target.track_state.value,
                        "task_state": target.task_state.value,
                        "observation_count": target.observation_count,
                        "confidence": target.confidence,
                        "covariance_trace": target.covariance_trace,
                        "physical_area_m2": (
                            target.estimated_size_m[0]
                            if target.estimated_size_m
                            else 0.0
                        ),
                        "polygon_xy_m": target.polygon_xy_m,
                        "x_m": target.map_x_m,
                        "y_m": target.map_y_m,
                        "ever_confirmed": any(
                            transition.get("to") == "CONFIRMED"
                            for transition in target.transitions
                        ),
                        "transitions": list(target.transitions),
                    }
                    for target in sorted(
                        dynamic_map.targets.values(), key=lambda item: item.uuid
                    )
                ],
            }
        )

    identity_numerators = []
    duplicate_count = 0
    fragmented_targets = 0
    for identifiers in all_identity_observations.values():
        counts = defaultdict(int)
        for identifier in identifiers:
            counts[identifier] += 1
        identity_numerators.append(max(counts.values()) / len(identifiers))
        duplicate_count += max(0, len(counts) - 1)
        fragmented_targets += int(len(counts) > 1)
    map_rmse = (
        math.sqrt(
            sum(value * value for value in all_final_errors)
            / len(all_final_errors)
        )
        if all_final_errors
        else None
    )
    return {
        "schema_version": 1,
        "evaluator": "product_tracker_v2_plus_dynamic_trash_map",
        "production_inputs": ["RGB", "depth", "camera_intrinsics", "odometry_TF"],
        "camera_pitch_down_rad": camera_pitch_down_rad,
        "GT_used_by_product_pipeline": False,
        "GT_used_only_for_post_run_scoring": True,
        "metrics": {
            "eligible_targets": total_eligible,
            "matched_eligible_targets": total_matched,
            "confirmed_product_targets": total_confirmed,
            "product_target_precision": total_matched / max(total_confirmed, 1),
            "false_confirmed_target_rate": (
                max(0, total_confirmed - total_matched)
                / max(total_confirmed, 1)
            ),
            "map_localization_coverage": total_matched / max(total_eligible, 1),
            "discrete_product_target_precision": (
                discrete_matched / max(discrete_confirmed, 1)
            ),
            "discrete_map_coverage": discrete_matched / max(discrete_eligible, 1),
            "area_product_target_precision": area_matched / max(area_confirmed, 1),
            "area_map_coverage": area_matched / max(area_eligible, 1),
            "combined_product_target_precision": (
                total_matched / max(total_confirmed, 1)
            ),
            "combined_map_coverage": total_matched / max(total_eligible, 1),
            "map_rmse_m": map_rmse,
            "map_localization_median_error_m": (
                percentile(all_final_errors, 50.0)
                if all_final_errors
                else None
            ),
            "map_localization_p95_error_m": (
                percentile(all_final_errors, 95.0)
                if all_final_errors
                else None
            ),
            "valid_depth_correct_detection_projection_success": (
                projection_successful_correct_detections
                / max(projection_eligible_correct_detections, 1)
            ),
            "direct_projection_median_error_m": (
                percentile(direct_projection_errors, 50.0)
                if direct_projection_errors
                else None
            ),
            "direct_projection_p95_error_m": (
                percentile(direct_projection_errors, 95.0)
                if direct_projection_errors
                else None
            ),
            "id_consistency": (
                sum(identity_numerators) / len(identity_numerators)
                if identity_numerators
                else None
            ),
            "duplicate_target_rate": duplicate_count / max(total_eligible, 1),
            "track_fragmentation": fragmented_targets / max(total_eligible, 1),
            "wrong_class_leading_to_wrong_clean_action": (
                wrong_class_confirmed_actions
            ),
            "pre_fov_target_creation": pre_fov_creations,
            "removed_target_stale_action": (
                removed_target_stale_actions if removal_capture_count else None
            ),
            "projection_frame_failures": projection_failures,
        },
        "aggregation_counts": {
            "localization_squared_error_sum": sum(
                value * value for value in all_final_errors
            ),
            "localization_error_count": len(all_final_errors),
            "identity_consistency_sum": sum(identity_numerators),
            "identity_target_count": len(identity_numerators),
            "duplicate_target_count": duplicate_count,
            "fragmented_target_count": fragmented_targets,
            "eligible_target_count": total_eligible,
            "matched_target_count": total_matched,
            "confirmed_product_target_count": total_confirmed,
            "discrete_eligible_target_count": discrete_eligible,
            "discrete_matched_target_count": discrete_matched,
            "discrete_confirmed_product_target_count": discrete_confirmed,
            "area_eligible_target_count": area_eligible,
            "area_matched_target_count": area_matched,
            "area_confirmed_product_target_count": area_confirmed,
            "wrong_class_confirmed_action_count": (
                wrong_class_confirmed_actions
            ),
            "pre_fov_creation_count": pre_fov_creations,
            "removed_target_stale_action_count": removed_target_stale_actions,
            "removal_capture_count": removal_capture_count,
            "projection_frame_failure_count": projection_failures,
            "projection_eligible_correct_detection_count": (
                projection_eligible_correct_detections
            ),
            "projection_successful_correct_detection_count": (
                projection_successful_correct_detections
            ),
        },
        "scheduler_evaluation": {
            "implementation": "CleaningTaskScheduler",
            "coverage_safe_boundary_rule": "frame_index_mod_10_equals_0",
            "rule_frozen_independent_of_GT_and_model_output": True,
            "only_fresh_online_observations_scheduled": True,
        },
        "removed_target_stale_action_reason": (
            None
            if removal_capture_count
            else "development captures contain no independent post-removal sequence"
        ),
        "missions": mission_reports,
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
    parser.add_argument("--detector-onnx", type=Path)
    parser.add_argument("--mmdet-config", type=Path)
    parser.add_argument("--mmdet-checkpoint", type=Path)
    parser.add_argument("--mmdet-selection", type=Path)
    parser.add_argument("--leaf-onnx", type=Path)
    parser.add_argument("--puddle-onnx", type=Path)
    parser.add_argument("--area-gate", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--pixel-distance-output", type=Path, required=True)
    args = parser.parse_args()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("formal OPRV3 moving benchmark requires CUDA")
    mmdet_paths = (args.mmdet_config, args.mmdet_checkpoint, args.mmdet_selection)
    if any(mmdet_paths) and not all(mmdet_paths):
        raise RuntimeError("MMDetection config, checkpoint and selection are atomic")
    use_mmdet = all(mmdet_paths)
    if use_mmdet and args.detector_onnx:
        raise RuntimeError("MMDetection and detector ONNX modes are mutually exclusive")
    if bool(args.leaf_onnx) != bool(args.puddle_onnx):
        raise RuntimeError("leaf and puddle ONNX paths are atomic")
    use_area_onnx = bool(args.leaf_onnx and args.puddle_onnx)
    use_onnx = bool(args.detector_onnx and use_area_onnx)
    if use_onnx and len(args.detector) != 1:
        raise RuntimeError("ONNX product benchmark accepts exactly one detector")
    if use_mmdet and len(args.detector) != 1:
        raise RuntimeError("MMDetection product benchmark accepts exactly one detector")
    geometry = load_geometry(args.geometry)
    rows, instances, context = load_development_rows(args.data_root)
    if use_area_onnx:
        leaf = create_cuda_ort_session(args.leaf_onnx)
        puddle = create_cuda_ort_session(args.puddle_onnx)
    else:
        leaf, leaf_metadata = load_area_checkpoint(
            "leaf", args.leaf_checkpoint, device
        )
        puddle, puddle_metadata = load_area_checkpoint(
            "puddle", args.puddle_checkpoint, device
        )
    area_configs, area_gate_provenance = load_area_gate(
        args.area_gate,
        leaf_checkpoint=args.leaf_checkpoint,
        puddle_checkpoint=args.puddle_checkpoint,
        leaf_onnx=args.leaf_onnx,
        puddle_onnx=args.puddle_onnx,
    )
    if use_area_onnx:
        metadata_loader = (
            g6_area_metadata_only
            if area_gate_provenance["runtime_input_contract"]
            == "g6_shared_rgb_hsv_depth_geometry_texture_v1"
            else area_metadata_only
        )
        leaf_metadata = metadata_loader("leaf", args.leaf_checkpoint)
        puddle_metadata = metadata_loader("puddle", args.puddle_checkpoint)
    manifest_path = (
        ROOT / "starter_ws" / "src" / "sanitation_perception" / "config"
        / "perception_pipeline_manifest.yaml"
    )
    product_manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    started = time.perf_counter()
    camera_pitch_down_rad = math.radians(-float(geometry["camera"]["pitch_deg"]))
    if not 0.0 <= camera_pitch_down_rad < math.pi * 0.5:
        raise RuntimeError("frozen camera pitch is outside the supported down-view range")
    onnx_detector_frames = None
    onnx_detector_metadata = None
    if use_onnx:
        _name, raw_path = args.detector[0].split("=", 1)
        onnx_detector_metadata = detector_metadata_only(Path(raw_path))
        detector_session = create_cuda_ort_session(args.detector_onnx)
        runtime_models = {
            "execution_backend": "onnxruntime",
            "execution_provider": "CUDAExecutionProvider",
            "detector": {
                "path": args.detector_onnx.as_posix(),
                "sha256": sha256(args.detector_onnx),
                "providers": detector_session.get_providers(),
            },
            "leaf": {
                "path": args.leaf_onnx.as_posix(),
                "sha256": sha256(args.leaf_onnx),
                "providers": leaf.get_providers(),
            },
            "puddle": {
                "path": args.puddle_onnx.as_posix(),
                "sha256": sha256(args.puddle_onnx),
                "providers": puddle.get_providers(),
            },
        }
        (
            areas,
            area_product_detections,
            onnx_detector_frames,
        ) = combined_frame_maps_onnx(
            detector_session,
            onnx_detector_metadata,
            leaf,
            puddle,
            rows,
            area_configs=area_configs,
            camera_pitch_down_rad=camera_pitch_down_rad,
            minimum_physical_area_m2=float(
                product_manifest["runtime"]["minimum_area_region_m2"]
            ),
            minimum_physical_area_m2_by_class={
                name: float(value)
                for name, value in product_manifest["runtime"].get(
                    "minimum_area_region_m2_by_class", {}
                ).items()
            },
            area_input_contract=area_gate_provenance["runtime_input_contract"],
        )
        del detector_session
    elif use_area_onnx:
        runtime_models = {
            "execution_backend": "mixed_mmdetection_pytorch_and_onnxruntime",
            "execution_provider": "CUDA",
            "leaf": {"path": args.leaf_onnx.as_posix(), "sha256": sha256(args.leaf_onnx), "providers": leaf.get_providers()},
            "puddle": {"path": args.puddle_onnx.as_posix(), "sha256": sha256(args.puddle_onnx), "providers": puddle.get_providers()},
        }
        areas, area_product_detections = area_frame_map_onnx(
            leaf,
            puddle,
            rows,
            area_configs=area_configs,
            camera_pitch_down_rad=camera_pitch_down_rad,
            minimum_physical_area_m2=float(product_manifest["runtime"]["minimum_area_region_m2"]),
            minimum_physical_area_m2_by_class={
                name: float(value)
                for name, value in product_manifest["runtime"].get("minimum_area_region_m2_by_class", {}).items()
            },
            input_contract=area_gate_provenance["runtime_input_contract"],
        )
    else:
        runtime_models = {
            "execution_backend": "pytorch",
            "execution_provider": "torch.cuda",
        }
        areas, area_product_detections = area_frame_map(
            leaf,
            puddle,
            rows,
            device,
            area_configs=area_configs,
            camera_pitch_down_rad=camera_pitch_down_rad,
            minimum_physical_area_m2=float(
                product_manifest["runtime"]["minimum_area_region_m2"]
            ),
            minimum_physical_area_m2_by_class={
                name: float(value)
                for name, value in product_manifest["runtime"].get(
                    "minimum_area_region_m2_by_class", {}
                ).items()
            },
        )
    del leaf, puddle
    torch.cuda.empty_cache()
    routes = {}
    pixel_distance = None
    row_by_key = {
        (int(row["scene_seed"]), int(row["frame_index"])): row for row in rows
    }
    for spec in args.detector:
        name, raw_path = spec.split("=", 1)
        if use_onnx:
            metadata = onnx_detector_metadata
            detector_frames = onnx_detector_frames
            model = None
        elif use_mmdet:
            model, metadata = load_mmdet_detector(
                args.mmdet_config, args.mmdet_checkpoint, args.mmdet_selection
            )
            if name != metadata["route"] or Path(raw_path) != args.mmdet_checkpoint:
                raise RuntimeError("--detector must name the hash-bound DDRV4 selected checkpoint")
            detector_frames = detector_frame_map_mmdet(model, metadata, rows)
            runtime_models["detector"] = {
                "path": args.mmdet_checkpoint.as_posix(),
                "sha256": metadata["sha256"],
                "selection_sha256": metadata["selection_sha256"],
                "backend": metadata["execution_backend"],
            }
        else:
            model, metadata = load_detector(Path(raw_path), device)
            detector_frames = detector_frame_map(
                model, metadata, rows, instances, device
            )
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
            "product_map": product_map_evaluation(
                rows=rows,
                context=context,
                detector_frames=detector_frames,
                detector_metadata=metadata,
                area_detections=area_product_detections,
                encounters=encounters,
                camera_pitch_down_rad=camera_pitch_down_rad,
                manifest=product_manifest,
            ),
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
        "runtime_models": runtime_models,
        "gt_boundary": {
            "ground_truth_used_by": "offline_evaluator_only",
            "production_inputs": [
                "RGB",
                "depth",
                "camera_intrinsics",
                "odometry_TF",
            ],
            "production_target_ids_or_coordinates_provided": False,
            "actionable_eligibility_uses_model_output": False,
        },
        "thresholds": {
            "observation_threshold": OBSERVATION_THRESHOLD,
            "low_confidence_observation_is_actionable": False,
            "detector_action_threshold": "checkpoint_frozen_train_holdout",
            "bbox_iou": IOU_THRESHOLD,
            "area": area_configs,
            "area_target_coverage": AREA_TARGET_COVERAGE_THRESHOLD,
        },
        "capture_audit": audit,
        "required_coverage": coverage,
        "coverage_complete": all(coverage.values()),
        "routes": routes,
        "area_gate": area_gate_provenance,
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
