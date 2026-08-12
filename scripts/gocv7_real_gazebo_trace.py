#!/usr/bin/env python3
"""Trace native MA1 through the real-Gazebo product chain for GOCV7.

The production path receives RGB, depth, CameraInfo and captured vehicle pose
only. Semantic masks and object poses are opened exclusively by the evaluator
branch after each product stage has emitted its result.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
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
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_spot_cleaning"))
sys.path.insert(0, str(ROOT / "scripts"))

from sanitation_learning.g4_data import DISCRETE_NAMES  # noqa: E402
from sanitation_learning.opr_c_rtmdet import patch_mmdet_cuda_nms  # noqa: E402
from sanitation_learning.oprv3_moving import (  # noqa: E402
    actionable_window_eligible,
    bbox_from_mask,
    bbox_iou,
)
from sanitation_perception.camera_frustum_model import CameraFrustumModel  # noqa: E402
from sanitation_perception.dynamic_trash_map import (  # noqa: E402
    DynamicTrashMap,
    DynamicTrashMapConfig,
)
from sanitation_perception.product_pipeline_node import (  # noqa: E402
    track_to_online_observation,
)
from sanitation_perception.rtmdet_product_runtime import (  # noqa: E402
    RTMDetProductRuntime,
    decode_rtmdet_result,
    file_sha256,
)
from sanitation_perception.tracker_v2 import (  # noqa: E402
    ProductTrackerV2,
    TrackerV2Config,
)
CLASS_LABELS = {"plastic_bottle": 1, "metal_can": 2, "paper_litter": 3}
REQUIRED_ROLES = {
    "normal",
    "turn",
    "occlusion",
    "reflection_wet",
    "small_targets",
    "negative_only",
}
IOU_THRESHOLD = 0.50
OBSERVATION_THRESHOLD = 0.05


def sha256(path: Path) -> str:
    return file_sha256(path)


def repository_commit() -> str:
    injected = os.environ.get("TZCUP_SOURCE_COMMIT", "").strip()
    if injected:
        if not re.fullmatch(r"[0-9a-fA-F]{40}", injected):
            raise RuntimeError("TZCUP_SOURCE_COMMIT must be a full git SHA")
        return injected.lower()
    return subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
    ).strip()


def parse_mission_specs(values: list[str]) -> list[tuple[str, Path]]:
    parsed = []
    for value in values:
        if "=" not in value:
            raise ValueError("--mission must use ROLE=/absolute/scene_dir")
        role, raw_path = value.split("=", 1)
        if role in {item[0] for item in parsed}:
            raise ValueError(f"duplicate mission role: {role}")
        parsed.append((role, Path(raw_path)))
    roles = {item[0] for item in parsed}
    if roles != REQUIRED_ROLES:
        raise ValueError(
            f"mission roles must be exactly {sorted(REQUIRED_ROLES)}; got {sorted(roles)}"
        )
    return parsed


def load_mission(role: str, scene_dir: Path) -> dict:
    manifest_path = scene_dir / "scene_manifest.json"
    capture_path = scene_dir / "capture_report.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    capture = json.loads(capture_path.read_text(encoding="utf-8"))
    if capture.get("capture_pass") is not True:
        raise RuntimeError(f"capture did not pass: {scene_dir}")
    if len(capture.get("records", [])) < 50:
        raise RuntimeError(f"mission has fewer than 50 frames: {scene_dir}")
    if role == "negative_only" and manifest.get("negative_only") is not True:
        raise RuntimeError("negative_only role must bind a negative-only mission")
    if role != "negative_only" and manifest.get("negative_only") is True:
        raise RuntimeError(f"positive role {role} bound a negative-only mission")
    return {
        "role": role,
        "scene_dir": scene_dir,
        "manifest_path": manifest_path,
        "capture_path": capture_path,
        "manifest": manifest,
        "capture": capture,
        "mission_id": f"gocv7-{role}-{manifest['world_id']}-{manifest['scene_seed']}",
    }


def row_for(mission: dict, record: dict) -> dict:
    scene_dir = mission["scene_dir"]
    paths = record["paths"]
    return {
        "scene_seed": int(mission["manifest"]["scene_seed"]),
        "frame_index": int(record["frame_index"]),
        "world_id": mission["manifest"]["world_id"],
        "negative_only": bool(mission["manifest"].get("negative_only", False)),
        "rgb_path": scene_dir / paths["rgb"],
        "depth_path": scene_dir / paths["depth"],
        "semantic_path": scene_dir / paths["semantic"],
        "instance_path": scene_dir / paths["instance"],
        "camera_path": scene_dir / paths["camera"],
        "tf_path": scene_dir / paths["tf"],
        "capture_record": record,
    }


def detection_signature(rows: list[dict]) -> list[tuple]:
    return [
        (
            item["class_name"],
            round(float(item["score"]), 8),
            tuple(round(float(value), 5) for value in item["bbox_xyxy"]),
            bool(item["actionable"]),
        )
        for item in rows
    ]


def matching_proposals(
    detections: list[dict], truth_bbox: list[float] | None
) -> list[dict]:
    if truth_bbox is None:
        return []
    output = []
    for detection in detections:
        overlap = bbox_iou(truth_bbox, detection["bbox_xyxy"])
        if overlap >= IOU_THRESHOLD:
            output.append({**detection, "iou": overlap})
    return sorted(output, key=lambda item: float(item["score"]), reverse=True)


def target_fact(
    *, target: dict, semantic: np.ndarray, depth: np.ndarray, record: dict,
    frozen_window: dict, pipelines: dict[str, list[dict]], threshold: float,
) -> dict:
    class_name = target["class_id"]
    mask = semantic == CLASS_LABELS[class_name]
    bbox = bbox_from_mask(mask)
    valid_depth_ratio = (
        float(np.isfinite(depth[mask]).mean()) if mask.any() else 0.0
    )
    vehicle_xy = record["vehicle_xy_m"]
    distance_m = math.hypot(
        float(target["xyz_m"][0]) - float(vehicle_xy[0]),
        float(target["xyz_m"][1]) - float(vehicle_xy[1]),
    )
    visible_fraction = float(target.get("estimated_visible_fraction", 1.0))
    actionable = actionable_window_eligible(
        visible_bbox=bbox,
        distance_m=distance_m,
        scene_visibility_ratio=visible_fraction,
        depth_valid_ratio=valid_depth_ratio,
        frozen_window=frozen_window,
    )
    result = {
        "target_id": target["model_name"],
        "class_name": class_name,
        "visible": bbox is not None,
        "actionable": actionable,
        "distance_m": distance_m,
        "bbox_xyxy": bbox,
        "bbox_short_side_px": (
            min(bbox[2] - bbox[0], bbox[3] - bbox[1]) if bbox else 0.0
        ),
        "depth_valid_ratio": valid_depth_ratio,
        "pipelines": {},
    }
    for name, detections in pipelines.items():
        proposals = matching_proposals(detections, bbox)
        correct = [item for item in proposals if item["class_name"] == class_name]
        result["pipelines"][name] = {
            "overlap_proposals": proposals,
            "observation_proposal": bool(proposals),
            "action_proposal": any(float(item["score"]) >= threshold for item in proposals),
            "correct_action_proposal": any(
                float(item["score"]) >= threshold for item in correct
            ),
            "best_correct_score": max(
                (float(item["score"]) for item in correct), default=0.0
            ),
            "best_predicted_class": proposals[0]["class_name"] if proposals else None,
        }
    return result


def current_track_records(tracker: ProductTrackerV2, stamp_s: float) -> list[dict]:
    return [
        {
            "track_id": track.uuid,
            "x_m": track.x_m,
            "y_m": track.y_m,
            "association_distance_m": None,
            "image_iou": None,
            "age_s": stamp_s - track.first_seen_s,
            "hit_count": track.observation_count,
            "miss_count": 0,
            "class_name": track.class_id,
            "class_posterior": track.class_posterior,
            "score_ema": track.score_ema,
            "state": track.state,
            "last_seen_current_frame": abs(track.last_seen_s - stamp_s) <= 1e-6,
        }
        for track in sorted(tracker.tracks.values(), key=lambda item: item.uuid)
    ]


def encounter_metrics(targets: dict[str, dict], pipeline: str) -> dict:
    eligible = [item for item in targets.values() if item["entered_actionable_window"]]
    per_class = {}
    for class_name in DISCRETE_NAMES:
        selected = [item for item in eligible if item["class_name"] == class_name]
        per_class[class_name] = {
            "eligible": len(selected),
            "eventual_detection_recall": sum(
                item["pipelines"][pipeline]["eventual_detection"] for item in selected
            ) / max(len(selected), 1),
            "eventual_correct_class_recall": sum(
                item["pipelines"][pipeline]["eventual_correct_class"] for item in selected
            ) / max(len(selected), 1),
        }
    small = [item for item in eligible if item["small_target"]]
    return {
        "eligible_targets": len(eligible),
        "eventual_detection_recall": sum(
            item["pipelines"][pipeline]["eventual_detection"] for item in eligible
        ) / max(len(eligible), 1),
        "eventual_correct_class_recall": sum(
            item["pipelines"][pipeline]["eventual_correct_class"] for item in eligible
        ) / max(len(eligible), 1),
        "small_target_count": len(small),
        "small_target_correct_recall": sum(
            item["pipelines"][pipeline]["eventual_correct_class"] for item in small
        ) / max(len(small), 1),
        "per_class": per_class,
    }


def false_metrics(frame_predictions: list[dict]) -> dict:
    actionable = sum(item["actionable_predictions"] for item in frame_predictions)
    wrong = sum(item["wrong_predictions"] for item in frame_predictions)
    negative = sum(item["negative_wrong_predictions"] for item in frame_predictions)
    return {
        "actionable_predictions": actionable,
        "wrong_actionable_predictions": wrong,
        "actionable_precision": (actionable - wrong) / max(actionable, 1),
        "wrong_actionable_rate": wrong / max(actionable, 1),
        "negative_only_wrong_actionable_predictions": negative,
    }


def root_cause_for(target: dict) -> str | None:
    if not target["entered_actionable_window"]:
        return None
    p0 = target["pipelines"]["P0_NATIVE"]
    p1 = target["pipelines"]["P1_ADAPTER"]
    p2 = target["pipelines"]["P2_PRODUCT"]
    if p0["eventual_correct_class"]:
        if not p1["eventual_correct_class"]:
            return "CLASS_INDEX_MISMATCH"
        if not p2["eventual_correct_class"]:
            return "OTHER_AUDITED"
        return None
    if p0["ever_correct_observation"] and not p0["eventual_correct_class"]:
        return "SCORE_CALIBRATION_MISMATCH"
    return "IMAGE_DOMAIN_SHIFT"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mission", action="append", required=True)
    parser.add_argument("--geometry", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--threshold", required=True, type=float)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--container-digest", required=True)
    parser.add_argument("--existing-full-report", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    if sha256(args.checkpoint) != args.expected_sha256:
        raise RuntimeError("checkpoint SHA-256 mismatch")
    mission_specs = parse_mission_specs(args.mission)
    missions = [load_mission(role, path) for role, path in mission_specs]
    geometry = json.loads(args.geometry.read_text(encoding="utf-8"))
    if geometry.get("frozen_before_moving_model_measurement") is not True:
        raise RuntimeError("geometry was not frozen before measurement")
    source_commit = repository_commit()
    manifest_path = (
        ROOT / "starter_ws/src/sanitation_perception/config/perception_pipeline_manifest.yaml"
    )
    product_manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    tracker_config = TrackerV2Config.from_pipeline_manifest(product_manifest)
    map_config = DynamicTrashMapConfig(**product_manifest["runtime"]["dynamic_trash_map"])
    frustum_config = product_manifest["runtime"]["camera_frustum"]
    camera_pitch_down_rad = math.radians(
        -float(geometry["camera"]["pitch_deg"])
    )

    patch_mmdet_cuda_nms()
    import torch
    from mmdet.apis import inference_detector, init_detector
    from perception_oprv3_moving_benchmark import (
        camera_projection_inputs,
        project_discrete_frame,
        schedule_current_target,
    )

    native_model = init_detector(str(args.config), str(args.checkpoint), device="cuda:0")
    product_runtime = RTMDetProductRuntime(
        args.config,
        args.checkpoint,
        expected_sha256=args.expected_sha256,
        observation_threshold=OBSERVATION_THRESHOLD,
        action_threshold=args.threshold,
    )
    started = time.perf_counter()
    frame_traces = []
    target_frames: dict[str, list[dict]] = defaultdict(list)
    false_rows: dict[str, list[dict]] = defaultdict(list)
    parity = Counter()
    mission_summaries = []

    for mission in missions:
        tracker = ProductTrackerV2(tracker_config)
        dynamic_map = DynamicTrashMap.start_new(
            mission["mission_id"], config=map_config
        )
        frustum = CameraFrustumModel(**frustum_config)
        scheduler = __import__(
            "sanitation_spot_cleaning.cleaning_task_scheduler",
            fromlist=["CleaningTaskScheduler"],
        ).CleaningTaskScheduler()
        targets = [
            item for item in mission["manifest"].get("objects", [])
            if item.get("class_id") in CLASS_LABELS
        ]
        map_ids_seen = set()
        for record in mission["capture"]["records"]:
            row = row_for(mission, record)
            bgr = cv2.imread(str(row["rgb_path"]), cv2.IMREAD_COLOR)
            if bgr is None:
                raise RuntimeError(f"failed to read RGB: {row['rgb_path']}")
            native_result = inference_detector(native_model, bgr)
            p0 = decode_rtmdet_result(
                native_result,
                observation_threshold=OBSERVATION_THRESHOLD,
                action_threshold=args.threshold,
            )
            p1 = decode_rtmdet_result(
                native_result,
                observation_threshold=OBSERVATION_THRESHOLD,
                action_threshold=args.threshold,
            )
            p2 = product_runtime.infer_bgr(bgr)
            parity["P0_P1_equal"] += detection_signature(p0) == detection_signature(p1)
            parity["P1_P2_equal"] += detection_signature(p1) == detection_signature(p2)
            parity["frames"] += 1

            semantic = np.load(row["semantic_path"], allow_pickle=False)
            depth = np.load(row["depth_path"], allow_pickle=False).astype(np.float32)
            pipelines = {"P0_NATIVE": p0, "P1_ADAPTER": p1, "P2_PRODUCT": p2}
            facts = []
            for target in targets:
                fact = target_fact(
                    target=target,
                    semantic=semantic,
                    depth=depth,
                    record=record,
                    frozen_window=geometry["class_actionable_windows"][target["class_id"]],
                    pipelines=pipelines,
                    threshold=args.threshold,
                )
                fact["mission_id"] = mission["mission_id"]
                fact["frame_index"] = int(record["frame_index"])
                facts.append(fact)
                target_frames[f"{mission['mission_id']}:{target['model_name']}"] .append(fact)

            truth_boxes = [fact["bbox_xyxy"] for fact in facts if fact["bbox_xyxy"]]
            for pipeline_name, detections in pipelines.items():
                selected = [item for item in detections if item["actionable"]]
                wrong = [
                    item for item in selected
                    if not any(bbox_iou(item["bbox_xyxy"], box) >= IOU_THRESHOLD for box in truth_boxes)
                ]
                false_rows[pipeline_name].append(
                    {
                        "mission_id": mission["mission_id"],
                        "frame_index": int(record["frame_index"]),
                        "negative_only": bool(row["negative_only"]),
                        "actionable_predictions": len(selected),
                        "wrong_predictions": len(wrong),
                        "negative_wrong_predictions": len(wrong) if row["negative_only"] else 0,
                        "detections": wrong,
                    }
                )

            detector_frame = {"detections": p2}
            projected = project_discrete_frame(
                detector_frame,
                row,
                {"input_size": [640, 480], "action_threshold": args.threshold},
                camera_pitch_down_rad=camera_pitch_down_rad,
                depth=depth,
            )
            stamp_ns = int(record["timestamp_ns"])
            stamp_s = stamp_ns / 1e9
            camera, transform = camera_projection_inputs(
                row, camera_pitch_down_rad=camera_pitch_down_rad
            )
            image_frame_id = f"camera_depth_link:{stamp_ns}"
            dynamic_map.observed_regions.record(
                frustum.make_sweep(
                    sweep_id=f"sweep:{stamp_ns}",
                    mission_id=mission["mission_id"],
                    stamp_ns=stamp_ns,
                    camera_frame_id="camera_depth_link",
                    image_frame_id=image_frame_id,
                    camera_x_m=float(transform[0, 3]),
                    camera_y_m=float(transform[1, 3]),
                    camera_yaw_rad=float(record.get("vehicle_yaw_rad", 0.0)),
                )
            )
            tracks = tracker.update(projected, stamp_s)
            map_records = []
            scheduler_records = []
            for track in tracks:
                if abs(track.last_seen_s - stamp_s) > 1e-6:
                    continue
                observation = track_to_online_observation(
                    track,
                    mission_id=mission["mission_id"],
                    stamp_ns=stamp_ns,
                    camera_frame_id="camera_depth_link",
                    image_frame_id=image_frame_id,
                    source_model=f"MA1:{args.expected_sha256}",
                )
                target = dynamic_map.ingest(observation)
                if target is None:
                    map_records.append(
                        {"track_id": track.uuid, "accepted": False, "rejected": True}
                    )
                    continue
                map_ids_seen.add(target.uuid)
                map_records.append(
                    {
                        "track_id": track.uuid,
                        "accepted": True,
                        "map_id": target.uuid,
                        "class_name": target.current_class,
                        "class_posterior": target.class_posterior,
                        "state": target.track_state.value,
                        "observation_count": target.observation_count,
                    }
                )
                decision = schedule_current_target(
                    scheduler, dynamic_map, target, record
                )
                if decision is not None:
                    scheduler_records.append(decision)
            dynamic_map.expire(stamp_ns)
            frame_traces.append(
                {
                    "mission_id": mission["mission_id"],
                    "role": mission["role"],
                    "frame_index": int(record["frame_index"]),
                    "timestamp_ns": stamp_ns,
                    "gt": facts,
                    "P0_NATIVE": p0,
                    "P1_ADAPTER": p1,
                    "P2_PRODUCT": p2,
                    "depth_valid_for_actionable_gt": all(
                        fact["depth_valid_ratio"] >= 0.8
                        for fact in facts if fact["actionable"]
                    ),
                    "projection": projected,
                    "tracker": current_track_records(tracker, stamp_s),
                    "dynamic_map": map_records,
                    "scheduler": scheduler_records,
                }
            )
        mission_summaries.append(
            {
                "mission_id": mission["mission_id"],
                "role": mission["role"],
                "world_id": mission["manifest"]["world_id"],
                "scene_seed": mission["manifest"]["scene_seed"],
                "coverage_profile": mission["manifest"].get("oprv3_coverage_profile"),
                "negative_only": mission["manifest"].get("negative_only", False),
                "frame_count": len(mission["capture"]["records"]),
                "map_target_count": len(map_ids_seen),
                "capture_report_sha256": sha256(mission["capture_path"]),
                "scene_manifest_sha256": sha256(mission["manifest_path"]),
            }
        )

    target_summaries = {}
    for key, frames in target_frames.items():
        actionable = [item for item in frames if item["actionable"]]
        entered = len(actionable) >= 3
        first_visible = next((item for item in frames if item["visible"]), None)
        target_summaries[key] = {
            "target_key": key,
            "mission_id": frames[0]["mission_id"],
            "target_id": frames[0]["target_id"],
            "class_name": frames[0]["class_name"],
            "entered_actionable_window": entered,
            "actionable_frame_count": len(actionable),
            "small_target": bool(
                first_visible and first_visible["bbox_short_side_px"] < 18.0
            ),
            "pipelines": {
                name: {
                    "eventual_detection": any(
                        item["pipelines"][name]["action_proposal"] for item in actionable
                    ) if entered else False,
                    "eventual_correct_class": any(
                        item["pipelines"][name]["correct_action_proposal"] for item in actionable
                    ) if entered else False,
                    "ever_correct_observation": any(
                        item["pipelines"][name]["best_correct_score"] >= OBSERVATION_THRESHOLD
                        for item in actionable
                    ) if entered else False,
                    "maximum_correct_score": max(
                        (item["pipelines"][name]["best_correct_score"] for item in actionable),
                        default=0.0,
                    ),
                }
                for name in ("P0_NATIVE", "P1_ADAPTER", "P2_PRODUCT")
            },
        }
    for item in target_summaries.values():
        item["root_cause"] = root_cause_for(item)

    metrics = {
        name: {
            **encounter_metrics(target_summaries, name),
            **false_metrics(false_rows[name]),
        }
        for name in ("P0_NATIVE", "P1_ADAPTER", "P2_PRODUCT")
    }
    p0 = metrics["P0_NATIVE"]
    parity_pass = (
        parity["P0_P1_equal"] == parity["frames"]
        and parity["P1_P2_equal"] == parity["frames"]
    )
    runtime_bug = not parity_pass and p0["eventual_correct_class_recall"] >= 0.95
    ga1_required = p0["eventual_correct_class_recall"] < 0.95
    root_counts = Counter(
        item["root_cause"] for item in target_summaries.values()
        if item["root_cause"]
    )
    common = {
        "schema_version": 1,
        "protocol": "GAZEBO-ONLINE-CLOSURE-V7",
        "stage": "GOCV7-00-01",
        "source_commit": source_commit,
        "checkpoint_sha256": args.expected_sha256,
        "config_sha256": sha256(args.config),
        "geometry_sha256": sha256(args.geometry),
        "pipeline_manifest_sha256": sha256(manifest_path),
        "container_digest": args.container_digest,
        "threshold": args.threshold,
        "environment": {
            "platform": platform.platform(),
            "python": platform.python_version(),
            "torch": torch.__version__,
            "cuda": torch.version.cuda,
            "gpu": torch.cuda.get_device_name(0),
        },
        "gt_boundary": {
            "production_inputs": ["RGB", "depth", "CameraInfo", "vehicle_pose_TF"],
            "semantic_and_object_GT_used_by": "offline_evaluator_only",
            "GT_target_coordinates_provided_to_product": False,
        },
        "mission_count": len(missions),
        "frame_count": parity["frames"],
        "duration_s": time.perf_counter() - started,
        "G5_read": False,
        "G5_V2_read": False,
        "formal_30seed_read": False,
    }
    existing = None
    if args.existing_full_report:
        existing_payload = json.loads(args.existing_full_report.read_text(encoding="utf-8"))
        existing = {
            "path": args.existing_full_report.as_posix(),
            "sha256": sha256(args.existing_full_report),
            "source_commit": existing_payload.get("source_commit"),
            "moving_discrete": existing_payload.get("sections", {}).get(
                "moving_discrete", {}
            ).get("metrics"),
        }

    args.output.mkdir(parents=True)
    attrition = {
        **common,
        "missions": mission_summaries,
        "pipeline_metrics": metrics,
        "parity": dict(parity),
        "target_summaries": list(target_summaries.values()),
        "frame_trace": frame_traces,
    }
    by_class = {
        **common,
        "pipeline_metrics": {
            name: value["per_class"] for name, value in metrics.items()
        },
        "small_target_metrics": {
            name: {
                "count": value["small_target_count"],
                "correct_recall": value["small_target_correct_recall"],
            }
            for name, value in metrics.items()
        },
    }
    false_targets = {
        **common,
        "pipeline_metrics": {
            name: {
                key: value[key]
                for key in (
                    "actionable_predictions",
                    "wrong_actionable_predictions",
                    "actionable_precision",
                    "wrong_actionable_rate",
                    "negative_only_wrong_actionable_predictions",
                )
            }
            for name, value in metrics.items()
        },
        "frames_with_false_targets": {
            name: [item for item in rows if item["wrong_predictions"]]
            for name, rows in false_rows.items()
        },
    }
    decision = {
        **common,
        "P0_native_metrics": p0,
        "existing_24_mission_native_path_evidence": existing,
        "P0_P1_P2_exact_parity": parity_pass,
        "RUNTIME_CONTRACT_BUG": runtime_bug,
        "GA1_REQUIRED": ga1_required,
        "root_cause_counts": dict(root_counts),
        "primary_root_cause": (
            root_counts.most_common(1)[0][0] if root_counts else "OTHER_AUDITED"
        ),
        "decision": (
            "GA1_REAL_GAZEBO_DEVELOPMENT_ONLY_FINE_TUNE"
            if ga1_required
            else "FIX_RUNTIME_CONTRACT_ONLY" if runtime_bug else "DETECTOR_GATE_READY"
        ),
        "GOCV7_DETECTOR_GAZEBO_PASS": all(
            (
                p0["eventual_detection_recall"] >= 0.95,
                p0["eventual_correct_class_recall"] >= 0.95,
                p0["per_class"]["metal_can"]["eventual_correct_class_recall"] >= 0.95,
                p0["per_class"]["paper_litter"]["eventual_correct_class_recall"] >= 0.95,
                p0["small_target_correct_recall"] >= 0.90,
                p0["actionable_precision"] >= 0.95,
                p0["wrong_actionable_rate"] <= 0.01,
            )
        ),
    }
    outputs = {
        "GOCV7_REAL_GAZEBO_ATTRITION.json": attrition,
        "GOCV7_REAL_GAZEBO_BY_CLASS.json": by_class,
        "GOCV7_REAL_GAZEBO_FALSE_TARGETS.json": false_targets,
        "GOCV7_ROOT_CAUSE_DECISION.json": decision,
    }
    for name, payload in outputs.items():
        (args.output / name).write_text(
            json.dumps(payload, indent=2) + "\n", encoding="utf-8"
        )
    index = {
        name: {"sha256": sha256(args.output / name), "bytes": (args.output / name).stat().st_size}
        for name in outputs
    }
    (args.output / "GOCV7_TRACE_ARTIFACT_INDEX.json").write_text(
        json.dumps({**common, "artifacts": index}, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(decision, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
