#!/usr/bin/env python3
"""Offline policy-stage replay from the retained RGB-D MCAP and frozen frames."""
from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception"))
from sanitation_perception.competition_policy import in_ground_roi  # noqa: E402
from sanitation_perception.preprocessing import (  # noqa: E402
    CLASS_ORDER,
    preprocess_rgb,
    resize_labels,
)
from sanitation_perception.projection import (  # noqa: E402
    ProjectionError,
    project_pixel_to_map,
    robust_depth,
)
from competition_perception_score import score  # noqa: E402


IMAGE_TOPIC = "/sensors/front_rgbd/depth/image_rect_raw/image"
DEPTH_TOPIC = "/sensors/front_rgbd/depth/image_rect_raw/depth_image"
INFO_TOPIC = "/sensors/front_rgbd/depth/image_rect_raw/camera_info"
TF_STATIC_TOPIC = "/tf_static"
FROZEN_THRESHOLDS = {class_id: 0.8 for class_id in CLASS_ORDER[1:]}
FROZEN_ROI = [0.4, 3.0, -0.8, 0.8, -0.08, 0.30]
FROZEN_MINIMUM_PIXELS = 24
FROZEN_IOU = 0.5


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def message_stamp(message) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quaternion_matrix(rotation) -> np.ndarray:
    x, y, z, w = (float(value) for value in (rotation.x, rotation.y, rotation.z, rotation.w))
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def transform_matrix(transform) -> np.ndarray:
    translation = transform.translation
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_matrix(transform.rotation)
    matrix[:3, 3] = (
        float(translation.x),
        float(translation.y),
        float(translation.z),
    )
    return matrix


def rigid_inverse(matrix: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = matrix[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ matrix[:3, 3]
    return result


def lookup_static_transform(edges, target: str, source: str) -> np.ndarray:
    """Return T_target_source from parent-child static transforms."""
    if target == source:
        return np.eye(4, dtype=np.float64)
    adjacency = {}
    for parent, child, matrix in edges:
        adjacency.setdefault(parent, []).append((child, matrix, False))
        adjacency.setdefault(child, []).append((parent, matrix, True))
    queue = deque([(source, np.eye(4, dtype=np.float64))])
    seen = {source}
    while queue:
        node, transform_node_source = queue.popleft()
        for neighbor, matrix_parent_child, node_is_child in adjacency.get(node, []):
            if neighbor in seen:
                continue
            if node_is_child:
                transform_neighbor_source = matrix_parent_child @ transform_node_source
            else:
                transform_neighbor_source = rigid_inverse(matrix_parent_child) @ transform_node_source
            if neighbor == target:
                return transform_neighbor_source
            seen.add(neighbor)
            queue.append((neighbor, transform_neighbor_source))
    raise KeyError(f"no static transform path from {source!r} to {target!r}")


def decode_depth(message) -> np.ndarray:
    if message.encoding == "32FC1":
        values = np.frombuffer(message.data, dtype="<f4")
    elif message.encoding == "16UC1":
        values = np.frombuffer(message.data, dtype="<u2").astype(np.float32) * 0.001
    else:
        raise ValueError(f"unsupported depth encoding: {message.encoding}")
    expected = int(message.height) * int(message.width)
    if values.size != expected:
        raise ValueError(f"depth byte count mismatch: {values.size} != {expected}")
    return values.reshape((int(message.height), int(message.width)))


def camera_from_info(message) -> dict:
    if len(message.k) != 9 or min(float(message.k[0]), float(message.k[4])) <= 0:
        raise ValueError("invalid CameraInfo intrinsics")
    return {
        "fx": float(message.k[0]),
        "fy": float(message.k[4]),
        "cx": float(message.k[2]),
        "cy": float(message.k[5]),
        "pixel_sigma": 0.5,
        "depth_sigma_m": 0.02,
    }


def nearest_message(messages, timestamp: float, maximum_skew_s: float):
    if not messages:
        return None
    candidate = min(messages, key=lambda item: abs(item[0] - timestamp))
    return candidate if abs(candidate[0] - timestamp) <= maximum_skew_s else None


def load_mcap_context(bag_path: Path, selected_frames: list[dict], maximum_skew_s: float):
    try:
        from mcap.reader import make_reader
        from rosbags.typesys import Stores, get_types_from_msg, get_typestore
    except ImportError as exc:
        raise RuntimeError("mcap and rosbags are required for MCAP policy replay") from exc

    if bag_path.is_dir():
        mcap_paths = sorted(bag_path.glob("*.mcap"))
        if len(mcap_paths) != 1:
            raise ValueError(f"expected exactly one MCAP file in {bag_path}")
        mcap_path = mcap_paths[0]
    else:
        mcap_path = bag_path
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with mcap_path.open("rb") as stream:
        reader = make_reader(stream)
        summary = reader.get_summary()
        if summary is None:
            raise ValueError("MCAP summary is required for schema-based policy replay")
        for schema in summary.schemas.values():
            if schema.encoding != "ros2msg":
                raise ValueError(f"unsupported MCAP schema encoding: {schema.encoding}")
            typestore.register(
                get_types_from_msg(schema.data.decode("utf-8"), schema.name)
            )

        def decoded_messages(topic: str):
            for schema, _, message in reader.iter_messages(topics=[topic]):
                yield typestore.deserialize_cdr(message.data, schema.name)

        image_times = []
        for message in decoded_messages(IMAGE_TOPIC):
            image_times.append(message_stamp(message))
        if len(image_times) != len(set(image_times)):
            raise ValueError("image timestamps are missing or duplicated")
        targets = []
        for frame in selected_frames:
            target = float(frame["actual_sim_s"])
            nearest = min(image_times, key=lambda value: abs(value - target))
            if abs(nearest - target) > maximum_skew_s:
                raise ValueError(f"{frame['frame_id']} has no image within the frozen skew")
            targets.append((frame["frame_id"], target, nearest))

        depth_messages = []
        info_messages = []
        static_edges = []
        for message in decoded_messages(DEPTH_TOPIC):
            timestamp = message_stamp(message)
            if any(abs(timestamp - nearest) <= maximum_skew_s for _, _, nearest in targets):
                depth_messages.append((timestamp, message))
        for message in decoded_messages(INFO_TOPIC):
            info_messages.append((message_stamp(message), message))
        for message in decoded_messages(TF_STATIC_TOPIC):
            for transform in message.transforms:
                static_edges.append(
                    (
                        transform.header.frame_id,
                        transform.child_frame_id,
                        transform_matrix(transform.transform),
                    )
                )

    context = {}
    missing = []
    zero_stamp_infos = [item for item in info_messages if item[0] == 0.0]
    for frame_id, target, nearest in targets:
        depth_match = nearest_message(depth_messages, nearest, maximum_skew_s)
        info_match = (
            zero_stamp_infos[-1]
            if zero_stamp_infos
            else nearest_message(info_messages, nearest, maximum_skew_s)
        )
        if depth_match is None:
            missing.append({"frame_id": frame_id, "reason": "depth_missing_or_skew"})
            context[frame_id] = None
            continue
        if info_match is None:
            missing.append({"frame_id": frame_id, "reason": "camera_info_missing_or_skew"})
            context[frame_id] = None
            continue
        try:
            depth = decode_depth(depth_match[1])
            camera_info = info_match[1]
            camera_frame = camera_info.header.frame_id
            if depth_match[1].header.frame_id != camera_frame:
                raise ValueError("depth and CameraInfo frames differ")
            map_camera = lookup_static_transform(static_edges, "map", camera_frame)
            base_camera = lookup_static_transform(static_edges, "base_footprint", camera_frame)
        except (KeyError, ValueError) as exc:
            missing.append({"frame_id": frame_id, "reason": str(exc)})
            context[frame_id] = None
            continue
        context[frame_id] = {
            "target_s": target,
            "image_s": nearest,
            "depth_s": depth_match[0],
            "camera_s": info_match[0],
            "depth": depth,
            "camera": camera_from_info(camera_info),
            "T_map_camera": map_camera,
            "T_base_camera": base_camera,
        }
    return context, {
        "image_message_count": len(image_times),
        "depth_message_count": len(depth_messages),
        "camera_info_message_count": len(info_messages),
        "static_transform_count": len(static_edges),
        "missing": missing,
    }


def predict_logits(session, image_rgb: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    logits = session.run(["logits"], {"images": preprocess_rgb(image_rgb)})[0][0]
    shifted = logits - logits.max(axis=0, keepdims=True)
    probability = np.exp(shifted) / np.exp(shifted).sum(axis=0, keepdims=True)
    return logits, probability


def policy_predictions(
    image_rgb: np.ndarray,
    logits: np.ndarray,
    probability: np.ndarray,
    context: dict,
) -> list[dict]:
    labels = np.argmax(logits, axis=0).astype(np.uint8)
    full_size = resize_labels(labels, image_rgb.shape[1], image_rgb.shape[0])
    depth = context["depth"]
    if depth.shape != full_size.shape:
        raise ValueError("registered depth shape mismatch")
    camera = context["camera"]
    predictions = []
    for class_index, class_id in enumerate(CLASS_ORDER[1:], 1):
        count, components, stats, centroids = cv2.connectedComponentsWithStats(
            (full_size == class_index).astype(np.uint8),
            8,
        )
        for component in range(1, count):
            if int(stats[component, cv2.CC_STAT_AREA]) < FROZEN_MINIMUM_PIXELS:
                continue
            component_mask = components == component
            model_mask = cv2.resize(
                component_mask.astype(np.uint8),
                (logits.shape[2], logits.shape[1]),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)
            confidence = float(probability[class_index][model_mask].mean())
            if confidence < FROZEN_THRESHOLDS[class_id]:
                continue
            depth_m = robust_depth(depth[component_mask].reshape(-1))
            u, v = (float(value) for value in centroids[component])
            xyz_base, _ = project_pixel_to_map(u, v, depth_m, camera, context["T_base_camera"])
            if not in_ground_roi(xyz_base, FROZEN_ROI):
                continue
            project_pixel_to_map(u, v, depth_m, camera, context["T_map_camera"])
            x = int(stats[component, cv2.CC_STAT_LEFT])
            y = int(stats[component, cv2.CC_STAT_TOP])
            width = int(stats[component, cv2.CC_STAT_WIDTH])
            height = int(stats[component, cv2.CC_STAT_HEIGHT])
            predictions.append(
                {
                    "class_id": class_id,
                    "confidence": confidence,
                    "xyxy": [x, y, x + width, y + height],
                }
            )
    return predictions


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--frames", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--expected-model-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--provider", action="append", default=[])
    parser.add_argument("--maximum-skew-s", type=float, default=0.03)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"fresh output required: {args.output}")
    model_sha256 = sha256_file(args.model)
    if model_sha256 != args.expected_model_sha256:
        raise SystemExit("model hash mismatch")

    frames = json.loads(args.frames.read_text(encoding="utf-8"))
    summary = json.loads(args.summary.read_text(encoding="utf-8"))
    selected = summary.get("selected_frames")
    if not isinstance(selected, list) or len(selected) != len(frames):
        raise SystemExit("summary and frame lists do not match")
    context_by_frame, mcap_binding = load_mcap_context(
        args.bag,
        selected,
        args.maximum_skew_s,
    )
    if mcap_binding["missing"]:
        result = {
            "status": "POLICY_INPUTS_INCOMPLETE",
            "policy_replay_status": "NOT_MEASURED",
            "policy_metrics": None,
            "model_sha256": model_sha256,
            "mcap_binding": mcap_binding,
            "reason": "required RGB-D or transform inputs were not recoverable",
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2))
        return 2

    import onnxruntime as ort

    providers = args.provider or ["CPUExecutionProvider"]
    session = ort.InferenceSession(str(args.model), providers=providers)
    replay = []
    rejection_counts = {}
    for frame, selected_frame in zip(frames, selected):
        frame_id = frame["frame_id"]
        image_path = Path(frame["image_path"])
        if not image_path.is_absolute():
            image_path = args.frames.parent / image_path
        image = cv2.cvtColor(cv2.imread(str(image_path)), cv2.COLOR_BGR2RGB)
        predictions = []
        if selected_frame.get("raw_output_present") is True:
            context = context_by_frame[frame_id]
            try:
                logits, probability = predict_logits(session, image)
                predictions = policy_predictions(
                    image,
                    logits,
                    probability,
                    context,
                )
            except ProjectionError as exc:
                rejection_counts.setdefault(str(exc), 0)
                rejection_counts[str(exc)] += 1
        replay.append(
            {
                "frame_id": frame_id,
                "image_path": frame["image_path"],
                "predictions": predictions,
                "truth": frame["truth"],
            }
        )
    policy_metrics = score(replay)
    result = {
        "status": "POLICY_REPLAY_MEASURED_NOT_OFFICIAL_ACCEPTANCE",
        "policy_replay_status": "MEASURED_OFFLINE_MCAP",
        "policy_metrics": policy_metrics,
        "model_sha256": model_sha256,
        "onnx_providers": session.get_providers(),
        "iou_threshold": FROZEN_IOU,
        "class_thresholds": FROZEN_THRESHOLDS,
        "minimum_component_pixels": FROZEN_MINIMUM_PIXELS,
        "ground_roi_bounds": FROZEN_ROI,
        "raw_output_presence_used": True,
        "mcap_binding": mcap_binding,
        "rejection_counts": rejection_counts,
        "scope": "controlled fixture policy replay only; not official R01 or a 95% claim",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "status": result["status"],
                "policy_replay_status": result["policy_replay_status"],
                "model_sha256": model_sha256,
                "class_metrics": policy_metrics["class_metrics"],
                "totals": {
                    "tp": sum(row["tp"] for row in policy_metrics["class_metrics"].values()),
                    "fp": sum(row["fp"] for row in policy_metrics["class_metrics"].values()),
                    "fn": sum(row["fn"] for row in policy_metrics["class_metrics"].values()),
                },
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
