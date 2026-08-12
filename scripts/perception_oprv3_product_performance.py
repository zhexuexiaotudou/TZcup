#!/usr/bin/env python3
"""Replay the prediction-derived OPRV3 candidate through the product queue."""

from __future__ import annotations

import argparse
import json
import math
import platform
from pathlib import Path
import sys
from threading import Event, Thread
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
    build_area_input,
    load_camera_info,
    read_rgb,
)
from sanitation_perception.camera_frustum_model import (  # noqa: E402
    CameraFrustumModel,
)
from sanitation_perception.dynamic_trash_map import (  # noqa: E402
    DynamicTrashMap,
    DynamicTrashMapConfig,
)
from sanitation_perception.frame_synchronizer import (  # noqa: E402
    LatestFrameScheduler,
    StampedPayload,
    SynchronizedFrame,
)
from sanitation_perception.product_pipeline_node import (  # noqa: E402
    track_to_online_observation,
)
from sanitation_perception.tracker_v2 import (  # noqa: E402
    ProductTrackerV2,
    TrackerV2Config,
)
from sanitation_spot_cleaning.cleaning_task_scheduler import (  # noqa: E402
    CleaningTaskScheduler,
)
from perception_oprv3_moving_benchmark import (  # noqa: E402
    OBSERVATION_THRESHOLD,
    apply_area_morphology,
    camera_projection_inputs,
    load_area_checkpoint,
    load_area_gate,
    load_detector,
    load_mmdet_detector,
    project_area_frame,
    project_discrete_frame,
    repository_commit,
    schedule_current_target,
    sha256,
)


THRESHOLDS = {
    "effective_hz": 10.0,
    "end_to_end_p95_ms": 200.0,
    "drop_rate": 0.01,
}


def detector_metadata_only(path: Path) -> dict:
    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    if checkpoint.get("checkpoint_status") != "training_complete_candidate_not_frozen":
        raise RuntimeError("MRV2-A checkpoint status is not a development candidate")
    if checkpoint.get("G5_SEALED_FINAL_read") is not False:
        raise RuntimeError("MRV2-A metadata violates the sealed-final boundary")
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
    if checkpoint.get("checkpoint_status") not in (
        "training_complete",
        "training_complete_candidate_not_frozen",
    ):
        raise RuntimeError(f"{task} checkpoint is not a completed candidate")
    return {
        "path": path.as_posix(),
        "sha256": sha256(path),
        "checkpoint_status": checkpoint["checkpoint_status"],
        "model_contract": checkpoint.get("model_contract") or {},
        "load_initialization": "metadata_only_for_hash_bound_onnx_runtime",
    }


def load_product_rows(data_root: Path) -> list[dict]:
    """Load captured product inputs without reading semantic/instance GT."""
    rows = []
    for scene_dir in sorted((data_root / "scenes").glob("scene_*")):
        report_path = scene_dir / "capture_report.json"
        if not report_path.is_file():
            continue
        report = json.loads(report_path.read_text(encoding="utf-8"))
        if report.get("capture_pass") is not True:
            continue
        for record in report["records"]:
            paths = record["paths"]
            rows.append(
                {
                    "scene_seed": int(report["scene_seed"]),
                    "frame_index": int(record["frame_index"]),
                    "split": report["split"],
                    "world_id": report["world_id"],
                    "rgb_path": scene_dir / paths["rgb"],
                    "depth_path": scene_dir / paths["depth"],
                    "camera_path": scene_dir / paths["camera"],
                    "tf_path": scene_dir / paths["tf"],
                    "capture_record": record,
                }
            )
    if not rows:
        raise RuntimeError("no passing product-input captures found")
    return sorted(rows, key=lambda row: (row["scene_seed"], row["frame_index"]))


def detector_inference(model, metadata: dict, rgb: np.ndarray, device) -> dict:
    input_width, input_height = metadata["input_size"]
    resized = cv2.resize(
        rgb,
        (input_width, input_height),
        interpolation=cv2.INTER_CUBIC,
    )
    image = torch.from_numpy(
        np.ascontiguousarray(resized.transpose(2, 0, 1), dtype=np.float32)
        / 255.0
    ).to(device)
    with torch.no_grad():
        output = model([image])[0]
    detections = []
    for box, score, label in zip(
        output["boxes"].detach().cpu().tolist(),
        output["scores"].detach().cpu().tolist(),
        output["labels"].detach().cpu().tolist(),
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
    return {"detections": detections[:100]}


def detector_mmdet_inference(model, metadata: dict, rgb: np.ndarray) -> dict:
    """Run the hash-bound DDRV4 MMDetection candidate from RGB only."""
    from mmdet.apis import inference_detector

    output = inference_detector(model, np.ascontiguousarray(rgb[..., ::-1]))
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
    return {"detections": detections[:100]}


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


def detector_onnx_inference(session, metadata: dict, rgb: np.ndarray) -> dict:
    input_width, input_height = metadata["input_size"]
    resized = cv2.resize(
        rgb, (input_width, input_height), interpolation=cv2.INTER_CUBIC
    )
    images = np.ascontiguousarray(
        resized.transpose(2, 0, 1)[None], dtype=np.float32
    ) / 255.0
    boxes, scores, labels = session.run(
        None, {session.get_inputs()[0].name: images}
    )
    detections = []
    for box, score, label in zip(boxes, scores, labels):
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
    return {"detections": detections[:100]}


def area_inference(
    model,
    task: str,
    row: dict,
    rgb: np.ndarray,
    depth: np.ndarray,
    config: dict,
    device,
    *,
    camera_pitch_down_rad: float,
    minimum_physical_area_m2: float,
    camera_info: dict | None = None,
    projection_inputs: tuple[dict, np.ndarray] | None = None,
) -> list[dict]:
    model_input = build_area_input(
        rgb,
        depth,
        AREA_MODEL_SIZE,
        task=task,
        camera_info=camera_info or load_camera_info(row),
    )
    tensor = torch.from_numpy(model_input.transpose(2, 0, 1)).unsqueeze(0)
    with torch.no_grad():
        probability = (
            torch.sigmoid(model(tensor.to(device))["logits"])[0, 0]
            .detach()
            .cpu()
            .numpy()
        )
    class_name = "leaf_pile" if task == "leaf" else "puddle"
    return project_area_frame(
        probability,
        row,
        class_name,
        float(config["threshold"]),
        morphology=str(config["morphology"]),
        camera_pitch_down_rad=camera_pitch_down_rad,
        minimum_physical_area_m2=minimum_physical_area_m2,
        depth=depth,
        projection_inputs=projection_inputs,
    )


def area_onnx_inference(
    session,
    task: str,
    row: dict,
    rgb: np.ndarray,
    depth: np.ndarray,
    config: dict,
    *,
    camera_pitch_down_rad: float,
    minimum_physical_area_m2: float,
    camera_info: dict | None = None,
    projection_inputs: tuple[dict, np.ndarray] | None = None,
) -> list[dict]:
    model_input = build_area_input(
        rgb,
        depth,
        AREA_MODEL_SIZE,
        task=task,
        camera_info=camera_info or load_camera_info(row),
    )
    tensor = np.ascontiguousarray(
        model_input.transpose(2, 0, 1)[None], dtype=np.float32
    )
    logits = session.run(None, {session.get_inputs()[0].name: tensor})[0]
    stable_logits = np.clip(
        np.asarray(logits[0, 0], dtype=np.float32), -80.0, 80.0
    )
    probability = 1.0 / (1.0 + np.exp(-stable_logits))
    class_name = "leaf_pile" if task == "leaf" else "puddle"
    return project_area_frame(
        probability,
        row,
        class_name,
        float(config["threshold"]),
        morphology=str(config["morphology"]),
        camera_pitch_down_rad=camera_pitch_down_rad,
        minimum_physical_area_m2=minimum_physical_area_m2,
        depth=depth,
        projection_inputs=projection_inputs,
        source_backend="onnxruntime_cuda",
    )


def summarize_performance(
    *, submitted: int, consumed: int, dropped: int,
    completion_times: list[float], latencies_ms: list[float],
) -> dict:
    effective_hz = (
        (consumed - 1) / (completion_times[-1] - completion_times[0])
        if consumed > 1 and completion_times[-1] > completion_times[0]
        else 0.0
    )
    p95 = float(np.percentile(latencies_ms, 95)) if latencies_ms else None
    drop_rate = dropped / max(submitted, 1)
    metrics = {
        "input_frames": submitted,
        "processed_frames": consumed,
        "dropped_frames": dropped,
        "effective_hz": effective_hz,
        "end_to_end_p95_ms": p95,
        "drop_rate": drop_rate,
        "formal_product_pipeline_executed": bool(consumed > 0),
    }
    gates = {
        "effective_hz": effective_hz >= THRESHOLDS["effective_hz"],
        "end_to_end_p95_ms": p95 is not None
        and p95 <= THRESHOLDS["end_to_end_p95_ms"],
        "drop_rate": drop_rate <= THRESHOLDS["drop_rate"],
        "formal_product_pipeline_executed": bool(consumed > 0),
    }
    return {"metrics": metrics, "gates": gates, "pass": all(gates.values())}


def formal_pipeline_complete(
    *, consumed: int, model_counts: dict[str, int],
    latency_samples: int, mission_count: int,
) -> bool:
    return bool(
        consumed > 0
        and model_counts.get("detector") == consumed
        and model_counts.get("leaf", 0) + model_counts.get("puddle", 0)
        == consumed
        and model_counts.get("leaf", 0) > 0
        and model_counts.get("puddle", 0) > 0
        and latency_samples == consumed
        and mission_count > 0
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument("--detector", type=Path, required=True)
    parser.add_argument("--mmdet-config", type=Path)
    parser.add_argument("--mmdet-selection", type=Path)
    parser.add_argument("--leaf-checkpoint", type=Path, required=True)
    parser.add_argument("--puddle-checkpoint", type=Path, required=True)
    parser.add_argument("--detector-onnx", type=Path)
    parser.add_argument("--leaf-onnx", type=Path)
    parser.add_argument("--puddle-onnx", type=Path)
    parser.add_argument("--area-gate", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--input-hz", type=float, default=10.0)
    parser.add_argument("--maximum-frames", type=int, default=90)
    parser.add_argument("--warmup-iterations", type=int, default=3)
    args = parser.parse_args()
    if args.input_hz < 10.0:
        raise ValueError("formal OPRV3 performance replay requires input-hz >= 10")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device.type != "cuda":
        raise RuntimeError("formal OPRV3 product performance requires CUDA")
    mmdet_paths = (args.mmdet_config, args.mmdet_selection)
    if any(mmdet_paths) and not all(mmdet_paths):
        raise RuntimeError("MMDetection config and selection paths are atomic")
    use_mmdet = all(mmdet_paths)
    if use_mmdet and args.detector_onnx:
        raise RuntimeError("MMDetection and detector ONNX paths are mutually exclusive")
    if bool(args.leaf_onnx) != bool(args.puddle_onnx):
        raise RuntimeError("leaf and puddle ONNX paths are atomic")
    use_detector_onnx = bool(args.detector_onnx)
    use_area_onnx = bool(args.leaf_onnx and args.puddle_onnx)
    if use_detector_onnx and not use_area_onnx:
        raise RuntimeError("the legacy detector ONNX route requires both area ONNX paths")

    rows = load_product_rows(args.data_root)[: args.maximum_frames]
    if use_mmdet:
        detector, detector_metadata = load_mmdet_detector(
            args.mmdet_config, args.detector, args.mmdet_selection
        )
    elif use_detector_onnx:
        detector_metadata = detector_metadata_only(args.detector)
        detector = None
    else:
        detector, detector_metadata = load_detector(args.detector, device)
    if use_area_onnx:
        leaf_metadata = area_metadata_only("leaf", args.leaf_checkpoint)
        puddle_metadata = area_metadata_only("puddle", args.puddle_checkpoint)
        leaf = puddle = None
    else:
        leaf, leaf_metadata = load_area_checkpoint(
            "leaf", args.leaf_checkpoint, device
        )
        puddle, puddle_metadata = load_area_checkpoint(
            "puddle", args.puddle_checkpoint, device
        )
    area_configs, area_gate = load_area_gate(
        args.area_gate,
        leaf_checkpoint=args.leaf_checkpoint,
        puddle_checkpoint=args.puddle_checkpoint,
    )
    geometry = json.loads(args.geometry.read_text(encoding="utf-8"))
    camera_pitch_down_rad = math.radians(-float(geometry["camera"]["pitch_deg"]))
    manifest_path = (
        ROOT
        / "starter_ws"
        / "src"
        / "sanitation_perception"
        / "config"
        / "perception_pipeline_manifest.yaml"
    )
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    tracker_config = TrackerV2Config.from_pipeline_manifest(manifest)
    map_config = DynamicTrashMapConfig(**manifest["runtime"]["dynamic_trash_map"])
    frustum = CameraFrustumModel(**manifest["runtime"]["camera_frustum"])
    if use_detector_onnx:
        detector = create_cuda_ort_session(args.detector_onnx)
    if use_area_onnx:
        leaf = create_cuda_ort_session(args.leaf_onnx)
        puddle = create_cuda_ort_session(args.puddle_onnx)

    def run_detector(rgb: np.ndarray) -> dict:
        if use_mmdet:
            return detector_mmdet_inference(detector, detector_metadata, rgb)
        if use_detector_onnx:
            return detector_onnx_inference(detector, detector_metadata, rgb)
        return detector_inference(detector, detector_metadata, rgb, device)

    # A replay file is only the transport for recorded camera messages.  Load
    # it before measurement so Windows bind-mount I/O is not misreported as
    # product pipeline latency.
    replay_inputs = []
    for row in rows:
        replay_inputs.append(
            {
                "row": row,
                "rgb": read_rgb(row),
                "depth": np.load(
                    row["depth_path"], allow_pickle=False
                ).astype(np.float32),
                "camera_info": load_camera_info(row),
                "projection_inputs": camera_projection_inputs(
                    row, camera_pitch_down_rad=camera_pitch_down_rad
                ),
            }
        )

    # Warm the exact model graphs outside the measured replay.
    warm = replay_inputs[0]
    warm_row = warm["row"]
    for _ in range(args.warmup_iterations):
        rgb = warm["rgb"]
        depth = warm["depth"]
        run_detector(rgb)
        area_call = area_onnx_inference if use_area_onnx else area_inference
        area_call(
            leaf, "leaf", warm_row, rgb, depth, area_configs["leaf_pile"],
            *(() if use_area_onnx else (device,)),
            camera_pitch_down_rad=camera_pitch_down_rad,
            minimum_physical_area_m2=float(
                manifest["runtime"].get(
                    "minimum_area_region_m2_by_class", {}
                ).get(
                    "leaf_pile", manifest["runtime"]["minimum_area_region_m2"]
                )
            ),
            camera_info=warm["camera_info"],
            projection_inputs=warm["projection_inputs"],
        )
        area_call(
            puddle, "puddle", warm_row, rgb, depth, area_configs["puddle"],
            *(() if use_area_onnx else (device,)),
            camera_pitch_down_rad=camera_pitch_down_rad,
            minimum_physical_area_m2=float(
                manifest["runtime"].get(
                    "minimum_area_region_m2_by_class", {}
                ).get("puddle", manifest["runtime"]["minimum_area_region_m2"])
            ),
            camera_info=warm["camera_info"],
            projection_inputs=warm["projection_inputs"],
        )
    torch.cuda.synchronize()

    frame_scheduler = LatestFrameScheduler(queue_depth=2)
    producer_done = Event()
    producer_failure: list[BaseException] = []
    submitted_at: dict[int, float] = {}

    def produce() -> None:
        try:
            started = time.perf_counter()
            for ordinal, replay in enumerate(replay_inputs):
                deadline = started + ordinal / args.input_hz
                remaining = deadline - time.perf_counter()
                if remaining > 0.0:
                    time.sleep(remaining)
                arrival = time.perf_counter()
                row = replay["row"]
                stamp_ns = int(row["capture_record"]["timestamp_ns"])
                payload = {"ordinal": ordinal, "replay": replay}
                stamped = StampedPayload(stamp_ns, payload)
                submitted_at[stamp_ns] = arrival
                frame_scheduler.submit(
                    SynchronizedFrame(stamped, stamped, stamped)
                )
        except BaseException as exc:  # pragma: no cover - fail-closed handoff
            producer_failure.append(exc)
        finally:
            producer_done.set()

    producer = Thread(target=produce, name="oprv3-input-replay", daemon=True)
    producer.start()
    completion_times = []
    latencies_ms = []
    model_counts = {"detector": 0, "leaf": 0, "puddle": 0}
    action_count = 0
    mission_state = {}
    while not producer_done.is_set() or frame_scheduler.depth:
        frame = frame_scheduler.pop_latest()
        if frame is None:
            time.sleep(0.001)
            continue
        payload = frame.rgb.payload
        ordinal = int(payload["ordinal"])
        replay = payload["replay"]
        row = replay["row"]
        seed = int(row["scene_seed"])
        if seed not in mission_state:
            mission_id = f"oprv3-performance-{seed}"
            mission_state[seed] = {
                "mission_id": mission_id,
                "tracker": ProductTrackerV2(tracker_config),
                "map": DynamicTrashMap.start_new(mission_id, config=map_config),
                "scheduler": CleaningTaskScheduler(),
            }
        state = mission_state[seed]
        rgb = replay["rgb"]
        depth = replay["depth"]
        detector_frame = run_detector(rgb)
        model_counts["detector"] += 1
        detections = project_discrete_frame(
            detector_frame,
            row,
            detector_metadata,
            camera_pitch_down_rad=camera_pitch_down_rad,
            depth=depth,
            projection_inputs=replay["projection_inputs"],
        )
        area_task = "leaf" if ordinal % 2 == 0 else "puddle"
        class_name = "leaf_pile" if area_task == "leaf" else "puddle"
        area_model = leaf if area_task == "leaf" else puddle
        detections.extend(
            (area_onnx_inference if use_area_onnx else area_inference)(
                area_model,
                area_task,
                row,
                rgb,
                depth,
                area_configs[class_name],
                *(() if use_area_onnx else (device,)),
                camera_pitch_down_rad=camera_pitch_down_rad,
                minimum_physical_area_m2=float(
                    manifest["runtime"].get(
                        "minimum_area_region_m2_by_class", {}
                    ).get(
                        class_name,
                        manifest["runtime"]["minimum_area_region_m2"],
                    )
                ),
                camera_info=replay["camera_info"],
                projection_inputs=replay["projection_inputs"],
            )
        )
        model_counts[area_task] += 1

        capture = row["capture_record"]
        stamp_ns = int(capture["timestamp_ns"])
        camera, transform = replay["projection_inputs"]
        del camera
        state["map"].observed_regions.record(
            frustum.make_sweep(
                sweep_id=f"sweep:{stamp_ns}",
                mission_id=state["mission_id"],
                stamp_ns=stamp_ns,
                camera_frame_id="camera_depth_link",
                image_frame_id=f"camera_depth_link:{stamp_ns}",
                camera_x_m=float(transform[0, 3]),
                camera_y_m=float(transform[1, 3]),
                camera_yaw_rad=float(capture.get("vehicle_yaw_rad", 0.0)),
            )
        )
        stamp_s = stamp_ns / 1_000_000_000.0
        tracks = state["tracker"].update(detections, stamp_s)
        for track in tracks:
            if abs(track.last_seen_s - stamp_s) > 1e-6:
                continue
            observation = track_to_online_observation(
                track,
                mission_id=state["mission_id"],
                stamp_ns=stamp_ns,
                camera_frame_id="camera_depth_link",
                image_frame_id=f"camera_depth_link:{stamp_ns}",
                source_model=f"{detector_metadata['route']}-product-performance",
            )
            target = state["map"].ingest(observation)
            if target is None:
                continue
            decision = schedule_current_target(
                state["scheduler"], state["map"], target, capture
            )
            if decision is not None and decision["action"] == "CLEAN_NOW":
                action_count += 1
        state["map"].expire(stamp_ns)
        if use_mmdet or not (use_detector_onnx and use_area_onnx):
            torch.cuda.synchronize()
        completed = time.perf_counter()
        completion_times.append(completed)
        latencies_ms.append((completed - submitted_at[frame.rgb_stamp_ns]) * 1000.0)

    producer.join()
    if producer_failure:
        raise RuntimeError("performance replay producer failed") from producer_failure[0]
    summary = summarize_performance(
        submitted=frame_scheduler.submitted,
        consumed=frame_scheduler.consumed,
        dropped=frame_scheduler.dropped,
        completion_times=completion_times,
        latencies_ms=latencies_ms,
    )
    formal_pipeline_executed = formal_pipeline_complete(
        consumed=frame_scheduler.consumed,
        model_counts=model_counts,
        latency_samples=len(latencies_ms),
        mission_count=len(mission_state),
    )
    summary["metrics"]["formal_product_pipeline_executed"] = (
        formal_pipeline_executed
    )
    summary["gates"]["formal_product_pipeline_executed"] = (
        formal_pipeline_executed
    )
    summary["pass"] = all(summary["gates"].values())
    report = {
        "schema_version": 1,
        "protocol": "OPRV3-07-product-performance",
        "source_commit": repository_commit(),
        "runtime": (
            "mixed_mmdetection_pytorch_and_onnxruntime_cuda_product_pipeline"
            if use_mmdet and use_area_onnx
            else "onnxruntime_cuda_prediction_derived_product_pipeline"
            if use_detector_onnx and use_area_onnx
            else "pytorch_cuda_prediction_derived_product_pipeline"
        ),
        "environment": {
            "platform": platform.platform(),
            "torch": torch.__version__,
            "cuda": torch.version.cuda,
            "gpu": torch.cuda.get_device_name(0),
        },
        "input_contract": {
            "source_hz": args.input_hz,
            "queue": "LatestFrameScheduler(queue_depth=2)",
            "detector_hz": args.input_hz,
            "leaf_hz": args.input_hz / 2.0,
            "puddle_hz": args.input_hz / 2.0,
            "GT_inputs_loaded": False,
            "product_inputs": ["RGB", "depth", "camera_intrinsics", "odometry_TF"],
        },
        "models": {
            "detector": detector_metadata,
            "leaf": leaf_metadata,
            "puddle": puddle_metadata,
            "area_gate": area_gate,
            "onnx": (
                {
                    "provider": "CUDAExecutionProvider",
                    "detector": (
                        {"path": args.detector_onnx.as_posix(), "sha256": sha256(args.detector_onnx)}
                        if use_detector_onnx else None
                    ),
                    "leaf": {"path": args.leaf_onnx.as_posix(), "sha256": sha256(args.leaf_onnx)},
                    "puddle": {"path": args.puddle_onnx.as_posix(), "sha256": sha256(args.puddle_onnx)},
                }
                if use_area_onnx
                else None
            ),
        },
        "execution": {
            "model_inference_counts": model_counts,
            "clean_now_action_count": action_count,
            "mission_count": len(mission_state),
            "latency_samples": len(latencies_ms),
        },
        **summary,
        "thresholds": THRESHOLDS,
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
    }
    report["inputs"] = {
        "geometry": {"path": args.geometry.as_posix(), "sha256": sha256(args.geometry)},
        "detector": {"path": args.detector.as_posix(), "sha256": sha256(args.detector)},
        "leaf": {"path": args.leaf_checkpoint.as_posix(), "sha256": sha256(args.leaf_checkpoint)},
        "puddle": {"path": args.puddle_checkpoint.as_posix(), "sha256": sha256(args.puddle_checkpoint)},
        "area_gate": {"path": args.area_gate.as_posix(), "sha256": sha256(args.area_gate)},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
