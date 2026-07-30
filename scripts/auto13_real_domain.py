#!/usr/bin/env python3
"""AUTO-13 real-domain capture, privacy, ingestion, and evaluation tools."""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np


DISCRETE_CLASSES = ("plastic_bottle", "metal_can", "paper_litter")
AREA_CLASSES = ("leaf_pile", "puddle")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def box_iou(left, right) -> float:
    x1, y1 = max(left[0], right[0]), max(left[1], right[1])
    x2, y2 = min(left[2], right[2]), min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = (
        max(0.0, left[2] - left[0]) * max(0.0, left[3] - left[1])
        + max(0.0, right[2] - right[0]) * max(0.0, right[3] - right[1])
        - intersection
    )
    return intersection / max(union, 1e-12)


def apply_privacy_regions(
    image: np.ndarray, regions: list[list[int]]
) -> np.ndarray:
    output = image.copy()
    height, width = output.shape[:2]
    for x1, y1, x2, y2 in regions:
        x1, x2 = max(0, x1), min(width, x2)
        y1, y2 = max(0, y1), min(height, y2)
        if x2 <= x1 or y2 <= y1:
            continue
        patch = output[y1:y2, x1:x2]
        kernel = max(7, (min(patch.shape[:2]) // 4) * 2 + 1)
        output[y1:y2, x1:x2] = cv2.GaussianBlur(patch, (kernel, kernel), 0)
    return output


def capture(args) -> int:
    if not args.consent:
        raise RuntimeError("capture requires explicit --consent")
    output = Path(args.output)
    (output / "frames").mkdir(parents=True, exist_ok=True)
    privacy = {}
    if args.privacy_regions:
        privacy = json.loads(Path(args.privacy_regions).read_text(encoding="utf-8"))
    source = int(args.source) if str(args.source).isdigit() else args.source
    stream = cv2.VideoCapture(source)
    if not stream.isOpened():
        raise RuntimeError(f"cannot open capture source: {args.source}")
    rows = []
    try:
        for index in range(args.frames):
            ok, image = stream.read()
            if not ok:
                raise RuntimeError(f"capture stopped at frame {index}")
            regions = privacy.get(str(index), privacy.get("default", []))
            sanitized = apply_privacy_regions(image, regions)
            path = output / "frames" / f"frame_{index:06d}.png"
            cv2.imwrite(str(path), sanitized)
            rows.append(
                {
                    "frame_id": f"frame_{index:06d}",
                    "timestamp_ns": time.time_ns(),
                    "path": path.relative_to(output).as_posix(),
                    "sha256": sha256(path),
                    "privacy_regions_applied": len(regions),
                    "source_kind": "camera" if isinstance(source, int) else "video",
                }
            )
    finally:
        stream.release()
    manifest = {
        "schema_version": 1,
        "domain": "real",
        "capture_consent_recorded": True,
        "frame_count": len(rows),
        "frames": rows,
        "exif_preserved": False,
        "ground_truth_status": "UNANNOTATED",
    }
    (output / "capture_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    return 0


def calibrate(args) -> int:
    image_paths = sorted(Path(args.images).glob("*.png"))
    pattern = (args.columns, args.rows)
    object_points = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    object_points[:, :2] = (
        np.mgrid[0 : pattern[0], 0 : pattern[1]].T.reshape(-1, 2)
        * args.square_size_m
    )
    objects, images, image_size = [], [], None
    used = []
    for path in image_paths:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            continue
        image_size = (image.shape[1], image.shape[0])
        found, corners = cv2.findChessboardCorners(image, pattern)
        if found:
            objects.append(object_points.copy())
            images.append(corners)
            used.append(path.name)
    if len(used) < args.minimum_images:
        raise RuntimeError(
            f"only {len(used)} valid calibration images; need {args.minimum_images}"
        )
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(
        objects, images, image_size, None, None
    )
    payload = {
        "schema_version": 1,
        "model": "pinhole",
        "image_size": list(image_size),
        "camera_matrix": matrix.tolist(),
        "distortion_coefficients": distortion.ravel().tolist(),
        "rms_reprojection_error_px": float(rms),
        "square_size_m": args.square_size_m,
        "images_used": used,
    }
    Path(args.output).write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )
    return 0


def ingest(args) -> int:
    capture_manifest = json.loads(
        Path(args.capture_manifest).read_text(encoding="utf-8")
    )
    annotations = json.loads(Path(args.annotations).read_text(encoding="utf-8"))
    calibration = json.loads(Path(args.calibration).read_text(encoding="utf-8"))
    frames = {item["frame_id"]: item for item in capture_manifest["frames"]}
    annotated = {item["frame_id"] for item in annotations["frames"]}
    missing = sorted(set(frames) - annotated)
    extra = sorted(annotated - set(frames))
    scenes = {item["scene_id"] for item in annotations["frames"]}
    class_counts = defaultdict(int)
    hard_negative_count = 0
    for item in annotations["frames"]:
        hard_negative_count += int(item.get("hard_negative", False))
        for instance in item.get("instances", []):
            class_counts[instance["class_id"]] += 1
    payload = {
        "schema_version": 1,
        "domain": "real",
        "capture_manifest_sha256": sha256(Path(args.capture_manifest)),
        "annotations_sha256": sha256(Path(args.annotations)),
        "calibration_sha256": sha256(Path(args.calibration)),
        "frame_count": len(frames),
        "scene_count": len(scenes),
        "class_instance_counts": dict(class_counts),
        "hard_negative_frame_count": hard_negative_count,
        "missing_annotations": missing,
        "unknown_annotation_frames": extra,
        "privacy_filter_applied": all(
            "privacy_regions_applied" in item for item in frames.values()
        ),
        "calibration_model": calibration["model"],
        "real_domain_minimum_resource_gate": len(frames) >= 1000
        and len(scenes) >= 20
        and all(class_counts[name] > 0 for name in DISCRETE_CLASSES + AREA_CLASSES)
        and hard_negative_count > 0
        and not missing
        and not extra,
    }
    Path(args.output).write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )
    return 0 if not missing and not extra else 2


def evaluate(args) -> int:
    ground_truth = json.loads(Path(args.ground_truth).read_text(encoding="utf-8"))
    predictions = json.loads(Path(args.predictions).read_text(encoding="utf-8"))
    truth_by_frame = {item["frame_id"]: item for item in ground_truth["frames"]}
    predictions_by_frame = {
        item["frame_id"]: item for item in predictions["frames"]
    }
    tp = defaultdict(int)
    fp = defaultdict(int)
    fn = defaultdict(int)
    negative_frames = 0
    negative_correct = 0
    area_intersection = defaultdict(int)
    area_union = defaultdict(int)
    for frame_id, truth_frame in truth_by_frame.items():
        predicted_frame = predictions_by_frame.get(frame_id, {"instances": []})
        true_discrete = [
            item
            for item in truth_frame.get("instances", [])
            if item["class_id"] in DISCRETE_CLASSES
        ]
        predicted_discrete = [
            item
            for item in predicted_frame.get("instances", [])
            if item["class_id"] in DISCRETE_CLASSES
        ]
        if truth_frame.get("hard_negative", False):
            negative_frames += 1
            negative_correct += int(not predicted_discrete)
        for class_name in DISCRETE_CLASSES:
            truths = [item for item in true_discrete if item["class_id"] == class_name]
            guesses = sorted(
                (
                    item
                    for item in predicted_discrete
                    if item["class_id"] == class_name
                ),
                key=lambda item: item.get("confidence", 1.0),
                reverse=True,
            )
            used = set()
            for guess in guesses:
                overlaps = [
                    box_iou(guess["bbox_xyxy"], item["bbox_xyxy"])
                    if index not in used
                    else -1.0
                    for index, item in enumerate(truths)
                ]
                best = int(np.argmax(overlaps)) if overlaps else -1
                if best >= 0 and overlaps[best] >= 0.5:
                    tp[class_name] += 1
                    used.add(best)
                else:
                    fp[class_name] += 1
            fn[class_name] += len(truths) - len(used)
        for class_name in AREA_CLASSES:
            truth_path = truth_frame.get("area_masks", {}).get(class_name)
            prediction_path = predicted_frame.get("area_masks", {}).get(class_name)
            if truth_path is None:
                continue
            truth_mask = np.load(
                Path(args.ground_truth).parent / truth_path, allow_pickle=False
            ).astype(bool)
            predicted_mask = (
                np.load(
                    Path(args.predictions).parent / prediction_path,
                    allow_pickle=False,
                ).astype(bool)
                if prediction_path
                else np.zeros_like(truth_mask)
            )
            area_intersection[class_name] += int((truth_mask & predicted_mask).sum())
            area_union[class_name] += int((truth_mask | predicted_mask).sum())
    precision = {
        name: tp[name] / max(tp[name] + fp[name], 1) for name in DISCRETE_CLASSES
    }
    recall = {
        name: tp[name] / max(tp[name] + fn[name], 1) for name in DISCRETE_CLASSES
    }
    f1 = {
        name: 2
        * precision[name]
        * recall[name]
        / max(precision[name] + recall[name], 1e-12)
        for name in DISCRETE_CLASSES
    }
    area_iou = {
        name: area_intersection[name] / max(area_union[name], 1)
        for name in AREA_CLASSES
    }
    metrics = {
        "schema_version": 1,
        "domain": "real",
        "frame_count": len(truth_by_frame),
        "discrete_precision_by_class": precision,
        "discrete_recall_by_class": recall,
        "discrete_f1_by_class": f1,
        "discrete_macro_f1": float(np.mean(list(f1.values()))),
        "area_iou_by_class": area_iou,
        "area_macro_miou": float(np.mean(list(area_iou.values()))),
        "negative_specificity": negative_correct / max(negative_frames, 1),
        "negative_frame_count": negative_frames,
        "map_localization_rmse_m": ground_truth.get(
            "map_localization_rmse_m"
        ),
        "synthetic_to_real_f1_drop": predictions.get(
            "synthetic_reference_macro_f1", 0.0
        )
        - float(np.mean(list(f1.values()))),
    }
    metrics["real_domain_gate_pass"] = (
        metrics["frame_count"] >= 1000
        and metrics["discrete_macro_f1"] >= 0.90
        and min(recall.values()) >= 0.85
        and metrics["area_macro_miou"] >= 0.75
        and metrics["negative_specificity"] >= 0.95
        and metrics["map_localization_rmse_m"] is not None
        and metrics["map_localization_rmse_m"] <= 0.15
        and metrics["synthetic_to_real_f1_drop"] <= 0.10
    )
    Path(args.output).write_text(
        json.dumps(metrics, indent=2) + "\n", encoding="utf-8"
    )
    return 0 if metrics["real_domain_gate_pass"] else 2


def build_parser():
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest="command", required=True)
    capture_parser = commands.add_parser("capture")
    capture_parser.add_argument("--source", required=True)
    capture_parser.add_argument("--frames", type=int, required=True)
    capture_parser.add_argument("--output", required=True)
    capture_parser.add_argument("--privacy-regions")
    capture_parser.add_argument("--consent", action="store_true")
    capture_parser.set_defaults(handler=capture)
    calibration = commands.add_parser("calibrate")
    calibration.add_argument("--images", required=True)
    calibration.add_argument("--columns", type=int, default=9)
    calibration.add_argument("--rows", type=int, default=6)
    calibration.add_argument("--square-size-m", type=float, required=True)
    calibration.add_argument("--minimum-images", type=int, default=12)
    calibration.add_argument("--output", required=True)
    calibration.set_defaults(handler=calibrate)
    ingestion = commands.add_parser("ingest")
    ingestion.add_argument("--capture-manifest", required=True)
    ingestion.add_argument("--annotations", required=True)
    ingestion.add_argument("--calibration", required=True)
    ingestion.add_argument("--output", required=True)
    ingestion.set_defaults(handler=ingest)
    evaluation = commands.add_parser("evaluate")
    evaluation.add_argument("--ground-truth", required=True)
    evaluation.add_argument("--predictions", required=True)
    evaluation.add_argument("--output", required=True)
    evaluation.set_defaults(handler=evaluate)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return args.handler(args)


if __name__ == "__main__":
    raise SystemExit(main())
