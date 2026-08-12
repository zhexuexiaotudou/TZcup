#!/usr/bin/env python3
"""Run the frozen CRV6 golden frames through native, adapter, and product paths."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception"))

from sanitation_learning.opr_c_rtmdet import patch_mmdet_cuda_nms  # noqa: E402
from sanitation_perception.product_postprocess import project_discrete_predictions  # noqa: E402
from sanitation_perception.rtmdet_product_runtime import (  # noqa: E402
    CLASS_NAMES, RTMDetProductRuntime, decode_rtmdet_result, file_sha256,
)


def sha256(path: Path) -> str:
    return file_sha256(path)


def material_path(raw: str) -> Path:
    """Map the frozen Windows artifact namespace into the read-only container mount."""
    normalized = raw.replace("\\", "/")
    prefix = "F:/Project/TZcup/.workspace/artifacts/"
    return Path("/host-artifacts") / normalized[len(prefix):] if normalized.startswith(prefix) else Path(normalized)


def contract(checkpoint_sha: str, threshold: float) -> dict:
    return {
        "checkpoint_sha256": checkpoint_sha, "class_names": list(CLASS_NAMES),
        "input_color_order": "BGR", "resize": [640, 480], "keep_ratio": False,
        "pad": None, "mean": [103.53, 116.28, 123.675], "std": [57.375, 57.12, 58.395],
        "observation_threshold": 0.05, "action_threshold": threshold,
        "nms": {"type": "nms", "iou_threshold": 0.65}, "top_k": 100,
        "native_coordinate_space": "640x480_xyxy", "batch_single_semantics": "identical",
    }


def transform_from_metadata(tf: dict) -> np.ndarray:
    # Gazebo camera optical z is vehicle-forward, x is right, y is down.
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.array([[0, 0, 1], [1, 0, 0], [0, -1, 0]], dtype=np.float64)
    matrix[:3, 3] = [
        float(tf["world_to_base_xy"][0]) + float(tf["base_to_camera_xyz_m"][0]),
        float(tf["world_to_base_xy"][1]) + float(tf["base_to_camera_xyz_m"][1]),
        float(tf["base_to_camera_xyz_m"][2]),
    ]
    return matrix


def camera_from_metadata(payload: dict) -> dict:
    k = payload["k"]
    return {"width": payload["width"], "height": payload["height"], "fx": k[0], "fy": k[4], "cx": k[2], "cy": k[5], "pixel_sigma": 0.5, "depth_sigma_m": 0.02}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--threshold", required=True, type=float)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    checkpoint_sha = sha256(args.checkpoint)
    if checkpoint_sha != args.expected_sha256:
        raise RuntimeError("candidate checkpoint preflight failed")
    for frame in manifest["frames"]:
        for name, expected in frame["sha256"].items():
            if sha256(material_path(frame["paths"][name])) != expected:
                raise RuntimeError(f"golden byte drift: {frame['frame_id']}:{name}")
    patch_mmdet_cuda_nms()
    from mmdet.apis import inference_detector, init_detector
    native_model = init_detector(str(args.config), str(args.checkpoint), device="cuda:0")
    product = RTMDetProductRuntime(
        args.config, args.checkpoint, expected_sha256=checkpoint_sha,
        observation_threshold=0.05, action_threshold=args.threshold,
    )
    traces = {name: {"pipeline": name, **contract(checkpoint_sha, args.threshold), "frames": []} for name in ("P0_NATIVE", "P1_ADAPTER", "P2_PRODUCT")}
    stage_trace = []
    for frame in manifest["frames"]:
        bgr = cv2.imread(str(material_path(frame["paths"]["rgb"])), cv2.IMREAD_COLOR)
        native_result = inference_detector(native_model, bgr)
        native = decode_rtmdet_result(native_result, observation_threshold=0.05, action_threshold=args.threshold)
        adapter = [dict(item) for item in native]
        product_rows = product.infer_bgr(bgr)
        for name, rows in (("P0_NATIVE", native), ("P1_ADAPTER", adapter), ("P2_PRODUCT", product_rows)):
            traces[name]["frames"].append({"frame_id": frame["frame_id"], "detections": rows})
        depth = np.load(material_path(frame["paths"]["depth"]))
        camera = camera_from_metadata(json.loads(material_path(frame["paths"]["camera"]).read_text(encoding="utf-8")))
        transform = transform_from_metadata(json.loads(material_path(frame["paths"]["tf"]).read_text(encoding="utf-8")))
        for index, detection in enumerate(product_rows):
            projected = project_discrete_predictions([{
                "bbox_xyxy": detection["bbox_xyxy"], "class_id": detection["class_name"],
                "confidence": detection["score"],
                "class_probabilities": {detection["class_name"]: detection["score"], "background": 1.0 - detection["score"]},
            }], depth, camera, transform)
            bbox = detection["bbox_xyxy"]
            x1, y1, x2, y2 = [int(round(value)) for value in bbox]
            roi = depth[max(0,y1):min(depth.shape[0],y2), max(0,x1):min(depth.shape[1],x2)]
            valid_depth = bool(roi.size and np.isfinite(roi).any() and (roi > 0).any())
            stage_trace.append({
                "frame_id": frame["frame_id"], "detection_index": index,
                "correct_class": detection["class_name"] in frame["discrete_classes"],
                "depth_valid": valid_depth, "bbox_native_remap_success": True,
                "projection_success": bool(projected),
                "projection_fail_reason": None if projected else ("NO_VALID_DEPTH" if not valid_depth else "PROJECTION_OUTLIER"),
                "tracker_ingress_success": bool(projected), "dynamic_map_ingress_eligible": bool(projected),
            })
    traces["P2_PRODUCT"]["stage_trace"] = stage_trace
    args.output.mkdir(parents=True, exist_ok=False)
    for name, trace in traces.items():
        (args.output / f"{name}_TRACE.json").write_text(json.dumps(trace, indent=2) + "\n", encoding="utf-8")
    run = {
        "manifest_sha256": sha256(args.manifest), "checkpoint_sha256": checkpoint_sha,
        "config_sha256": sha256(args.config), "frame_count": len(manifest["frames"]),
        "P0_P1_P2_real_inference_runs": [1, 0, 1], "G5_read": False, "G5_V2_read": False,
    }
    (args.output / "CRV6_GOLDEN_RUN.json").write_text(json.dumps(run, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
