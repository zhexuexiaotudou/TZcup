#!/usr/bin/env python3
"""Evaluator-only raw DOSOD diagnostics for one saved real Gazebo frame."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from sanitation_perception.dosod_ros_adapter import CLASS_IDS, DosodOnnxDetector, preprocess_rgb


def _percentiles(values: np.ndarray) -> dict[str, float]:
    return {
        "min": float(np.min(values)),
        "p50": float(np.percentile(values, 50)),
        "p90": float(np.percentile(values, 90)),
        "p99": float(np.percentile(values, 99)),
        "p99_9": float(np.percentile(values, 99.9)),
        "max": float(np.max(values)),
    }


def diagnose(image_path: Path, model_path: Path, metadata_path: Path | None) -> dict:
    if not image_path.is_file() or not model_path.is_file():
        raise FileNotFoundError("diagnostic image and DOSOD ONNX model are required")
    bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError(f"failed to decode image: {image_path}")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    tensor, scale, padding = preprocess_rgb(rgb)

    import onnxruntime as ort

    session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    scores_raw, boxes_raw = session.run(["scores", "boxes"], {input_name: tensor})
    scores = np.asarray(scores_raw, dtype=np.float32)[0]
    boxes = np.asarray(boxes_raw, dtype=np.float32)[0]
    if scores.ndim != 2 or scores.shape[1] != len(CLASS_IDS) or boxes.shape != (scores.shape[0], 4):
        raise RuntimeError(f"unexpected DOSOD outputs scores={scores.shape} boxes={boxes.shape}")

    threshold_results = {}
    for threshold in (0.002, 0.0025, 0.003, 0.005, 0.01, 0.02, 0.05, 0.25):
        filtered_at_threshold = DosodOnnxDetector(session=session, score_threshold=threshold).infer(rgb)
        threshold_results[f"{threshold:.3f}"] = {
            "count": len(filtered_at_threshold),
            "detections": [
                {"class_id": item.class_id, "confidence": item.confidence, "xyxy": list(item.xyxy)}
                for item in filtered_at_threshold[:50]
            ],
        }
    widths = boxes[:, 2] - boxes[:, 0]
    heights = boxes[:, 3] - boxes[:, 1]
    metadata = {}
    if metadata_path is not None and metadata_path.is_file():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    truth_pixel_sizes = []
    for item in metadata.get("truth_boxes_xyxy", []):
        x1, y1, x2, y2 = [float(value) for value in item["xyxy"]]
        truth_pixel_sizes.append(
            {
                "object_id": str(item["object_id"]),
                "width_px": x2 - x1,
                "height_px": y2 - y1,
                "area_px2": max(0.0, x2 - x1) * max(0.0, y2 - y1),
            }
        )
    height, width = rgb.shape[:2]
    pad_y, pad_x = padding
    top_candidates = {}
    for class_index, class_id in enumerate(CLASS_IDS):
        indices = np.argsort(scores[:, class_index])[::-1][:10]
        rows = []
        for index in indices:
            x1, y1, x2, y2 = boxes[index].astype(float)
            mapped = [
                float(np.clip(x1 / scale - pad_x, 0.0, width)),
                float(np.clip(y1 / scale - pad_y, 0.0, height)),
                float(np.clip(x2 / scale - pad_x, 0.0, width)),
                float(np.clip(y2 / scale - pad_y, 0.0, height)),
            ]
            rows.append(
                {
                    "anchor_index": int(index),
                    "score": float(scores[index, class_index]),
                    "box_model_640_xyxy": [x1, y1, x2, y2],
                    "box_original_image_xyxy": mapped,
                }
            )
        top_candidates[class_id] = rows
    return {
        "report_id": "tzcup_formal_dosod_real_gazebo_frame_diagnostic_v1",
        "claim_boundary": {
            "evaluator_only_offline_diagnostic": True,
            "eligible_as_product_acceptance": False,
            "truth_used_by_product_perception_or_control": False,
        },
        "input": {
            "image_path": str(image_path),
            "image_sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
            "image_shape_hwc": list(rgb.shape),
            "real_gazebo_camera_frame": bool(metadata.get("real_gazebo_camera_frame", False)),
            "source_topic": metadata.get("source_topic"),
            "truth_overlay_metadata_path": str(metadata_path) if metadata_path is not None else None,
            "preprocess_tensor_shape": list(tensor.shape),
            "preprocess_scale": float(scale),
            "preprocess_padding_yx": list(padding),
        },
        "raw_scores": {
            class_id: {
                **_percentiles(scores[:, index]),
                "count_ge_0_25": int(np.count_nonzero(scores[:, index] >= 0.25)),
            }
            for index, class_id in enumerate(CLASS_IDS)
        },
        "raw_boxes": {
            "shape": list(boxes.shape),
            "x1": _percentiles(boxes[:, 0]),
            "y1": _percentiles(boxes[:, 1]),
            "x2": _percentiles(boxes[:, 2]),
            "y2": _percentiles(boxes[:, 3]),
            "width": _percentiles(widths),
            "height": _percentiles(heights),
            "finite": bool(np.isfinite(boxes).all()),
        },
        "postprocess_at_0_25": {
            **threshold_results["0.250"],
        },
        "postprocess_threshold_sweep": threshold_results,
        "top_raw_candidates_by_class": top_candidates,
        "evaluator_truth_pixel_sizes": truth_pixel_sizes,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    report = diagnose(args.image, args.model, args.metadata)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "postprocess_count": report["postprocess_at_0_25"]["count"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
