"""G4 screening evaluation helpers for the AUTO-05R learned pipeline."""

from __future__ import annotations

from collections import defaultdict
import math
from typing import Callable

import cv2
import numpy as np

from .auto04_contract import box_iou, decode_centernet_outputs
from .g4_data import (
    AREA_MODEL_SIZE,
    CLASSIFIER_MODEL_SIZE,
    DISCOVERY_MODEL_SIZE,
    DISCOVERY_STRIDE,
    DISCRETE_NAMES,
    _positive_area_crop,
    _resize_area_image_crop,
    _resize_area_mask_crop,
    build_area_input,
    discrete_boxes_for_frame,
    load_camera_info,
    mask_boundary,
    normalize_depth,
    read_frame,
    read_rgb,
    square_crop,
)


def discovery_predictions(
    model,
    rows: list[dict],
    instances_by_key: dict[tuple[int, int], list[dict]],
    *,
    device,
    threshold: float = 0.5,
    max_detections: int = 100,
    nms_iou_threshold: float = 0.5,
    local_maximum_radius: int = 1,
    stride: int = DISCOVERY_STRIDE,
) -> list[dict]:
    model.eval()
    frames: list[dict] = []
    with device:
        for row in rows:
            rgb = read_rgb(row)
            resized = cv2.resize(
                rgb, DISCOVERY_MODEL_SIZE, interpolation=cv2.INTER_AREA
            ).astype(np.float32) / 255.0
            tensor = torch_from_numpy(
                np.ascontiguousarray(resized.transpose(2, 0, 1)[None], dtype=np.float32)
            ).to(device)
            with torch_no_grad():
                outputs = model(tensor)
            objectness = torch_sigmoid(outputs["objectness_logits"])[0].cpu().numpy()
            offset = outputs["offset"][0].cpu().numpy()
            size = outputs["bbox_size"][0].cpu().numpy()
            detections = decode_centernet_outputs(
                objectness,
                offset,
                size,
                stride=stride,
                score_threshold=threshold,
                nms_iou_threshold=nms_iou_threshold,
                max_detections=max_detections,
                local_maximum_radius=local_maximum_radius,
            )
            truth = discrete_boxes_for_frame(row, instances_by_key)
            frames.append(
                {
                    "row": row,
                    "scene_seed": int(row["scene_seed"]),
                    "frame_index": int(row["frame_index"]),
                    "split": row["split"],
                    "world_id": row["world_id"],
                    "negative_only": bool(row.get("negative_only", False)),
                    "detections": [
                        {
                            "class_index": 0,
                            "score": float(detection.score),
                            "bbox_xyxy": list(detection.bbox_xyxy),
                        }
                        for detection in detections
                    ],
                    "truth": truth,
                }
            )
    return frames


def grid_proposal_predictions(
    classifier,
    rows: list[dict],
    instances_by_key: dict[tuple[int, int], list[dict]],
    *,
    device,
    class_threshold: float = 0.5,
    scales: tuple[int, ...] = (64, 128, 224, 320),
    stride_ratio: float = 0.75,
    max_detections: int = 40,
    nms_iou_threshold: float = 0.5,
) -> list[dict]:
    """Class-agnostic grid proposals from the trained crop classifier."""
    classifier.eval()
    frames: list[dict] = []
    with device:
        with torch_no_grad():
            for row in rows:
                rgb = read_rgb(row)
                height, width = rgb.shape[:2]
                raw: list[dict] = []
                for scale in scales:
                    if scale > min(width, height):
                        continue
                    step = max(1, int(scale * stride_ratio))
                    for y in range(0, height - scale + 1, step):
                        for x in range(0, width - scale + 1, step):
                            crop = cv2.resize(
                                rgb[y : y + scale, x : x + scale],
                                CLASSIFIER_MODEL_SIZE,
                                interpolation=cv2.INTER_AREA,
                            )
                            tensor = torch_from_numpy(
                                np.ascontiguousarray(
                                    crop.transpose(2, 0, 1)[None],
                                    dtype=np.float32,
                                )
                                / 255.0
                            ).to(device)
                            logits = classifier(tensor)[0].cpu().numpy()
                            probabilities = np.exp(
                                logits - logits.max(keepdims=True)
                            )
                            probabilities /= probabilities.sum(keepdims=True)
                            class_index = int(np.argmax(probabilities[1:])) + 1
                            class_score = float(probabilities[class_index])
                            if (
                                class_score >= class_threshold
                                and class_score > float(probabilities[0])
                            ):
                                raw.append(
                                    {
                                        "class_index": 0,
                                        "score": class_score,
                                        "bbox_xyxy": [
                                            x,
                                            y,
                                            x + scale,
                                            y + scale,
                                        ],
                                    }
                                )
                raw.sort(key=lambda item: item["score"], reverse=True)
                kept: list[dict] = []
                for item in raw:
                    if any(
                        box_iou(
                            tuple(float(value) for value in item["bbox_xyxy"]),
                            tuple(float(value) for value in other["bbox_xyxy"]),
                        )
                        >= nms_iou_threshold
                        for other in kept
                    ):
                        continue
                    kept.append(item)
                    if len(kept) >= max_detections:
                        break
                truth = discrete_boxes_for_frame(row, instances_by_key)
                frames.append(
                    {
                        "row": row,
                        "scene_seed": int(row["scene_seed"]),
                        "frame_index": int(row["frame_index"]),
                        "split": row["split"],
                        "world_id": row["world_id"],
                        "negative_only": bool(row.get("negative_only", False)),
                        "detections": kept,
                        "truth": truth,
                    }
                )
    return frames


def discovery_crop_predictions(
    model,
    samples: list[dict],
    *,
    device,
    threshold: float = 0.5,
    max_detections: int = 60,
    nms_iou_threshold: float = 0.5,
    local_maximum_radius: int = 1,
) -> list[dict]:
    model.eval()
    frames: list[dict] = []
    with device:
        with torch_no_grad():
            for sample in samples:
                rgb = read_rgb(sample)
                left, top, right, bottom = sample["crop"]
                crop = rgb[top:bottom, left:right]
                resized = cv2.resize(
                    crop,
                    DISCOVERY_MODEL_SIZE,
                    interpolation=cv2.INTER_AREA,
                ).astype(np.float32) / 255.0
                tensor = torch_from_numpy(
                    np.ascontiguousarray(
                        resized.transpose(2, 0, 1)[None], dtype=np.float32
                    )
                ).to(device)
                outputs = model(tensor)
                objectness = torch_sigmoid(outputs["objectness_logits"])[0].cpu().numpy()
                offset = outputs["offset"][0].cpu().numpy()
                size = outputs["bbox_size"][0].cpu().numpy()
                detections = decode_centernet_outputs(
                    objectness,
                    offset,
                    size,
                    stride=DISCOVERY_STRIDE,
                    score_threshold=threshold,
                    nms_iou_threshold=nms_iou_threshold,
                    local_maximum_radius=local_maximum_radius,
                    max_detections=max_detections,
                )
                frames.append(
                    {
                        "row": sample,
                        "scene_seed": int(sample.get("scene_seed", 0)),
                        "frame_index": int(sample.get("frame_index", 0)),
                        "split": sample.get("split", "train"),
                        "world_id": sample.get("world_id", ""),
                        "negative_only": bool(sample.get("negative_only", False)),
                        "detections": [
                            {
                                "class_index": 0,
                                "score": float(item.score),
                                "bbox_xyxy": list(item.bbox_xyxy),
                            }
                            for item in detections
                        ],
                        "truth": sample["boxes"],
                    }
                )
    return frames


def sliding_discovery_predictions(
    model,
    rows: list[dict],
    instances_by_key: dict[tuple[int, int], list[dict]],
    *,
    device,
    threshold: float = 0.35,
    scales: tuple[int, ...] = (48, 96, 192, 320),
    stride_ratio: float = 0.5,
    max_detections: int = 60,
    nms_iou_threshold: float = 0.5,
) -> list[dict]:
    """Sliding-window candidates from the crop discovery model."""
    model.eval()
    frames: list[dict] = []
    with device:
        for row in rows:
            rgb = read_rgb(row)
            height, width = rgb.shape[:2]
            samples: list[dict] = []
            for scale in scales:
                if scale > min(width, height):
                    continue
                step = max(1, int(scale * stride_ratio))
                for y in range(0, height - scale + 1, step):
                    for x in range(0, width - scale + 1, step):
                        samples.append(
                            {
                                "rgb_path": row["rgb_path"],
                                "crop": (x, y, x + scale, y + scale),
                                "boxes": [],
                                "negative_only": bool(
                                    row.get("negative_only", False)
                                ),
                                "scene_seed": int(row["scene_seed"]),
                                "frame_index": int(row["frame_index"]),
                                "split": row["split"],
                            }
                        )
            crop_frames = discovery_crop_predictions(
                model,
                samples,
                device=device,
                threshold=threshold,
                max_detections=20,
                nms_iou_threshold=0.5,
                local_maximum_radius=0,
            )
            raw: list[dict] = []
            for crop_frame, sample in zip(crop_frames, samples):
                left, top, right, bottom = sample["crop"]
                crop_width = max(right - left, 1)
                crop_height = max(bottom - top, 1)
                scale_x = crop_width / DISCOVERY_MODEL_SIZE[0]
                scale_y = crop_height / DISCOVERY_MODEL_SIZE[1]
                for detection in crop_frame["detections"]:
                    x1, y1, x2, y2 = (
                        float(value) for value in detection["bbox_xyxy"]
                    )
                    raw.append(
                        {
                            "class_index": 0,
                            "score": float(detection["score"]),
                            "bbox_xyxy": [
                                left + x1 * scale_x,
                                top + y1 * scale_y,
                                left + x2 * scale_x,
                                top + y2 * scale_y,
                            ],
                        }
                    )
            raw.sort(key=lambda item: item["score"], reverse=True)
            kept: list[dict] = []
            for item in raw:
                if any(
                    box_iou(
                        tuple(float(value) for value in item["bbox_xyxy"]),
                        tuple(float(value) for value in other["bbox_xyxy"]),
                    )
                    >= nms_iou_threshold
                    for other in kept
                ):
                    continue
                kept.append(item)
                if len(kept) >= max_detections:
                    break
            truth = discrete_boxes_for_frame(row, instances_by_key)
            frames.append(
                {
                    "row": row,
                    "scene_seed": int(row["scene_seed"]),
                    "frame_index": int(row["frame_index"]),
                    "split": row["split"],
                    "world_id": row["world_id"],
                    "negative_only": bool(row.get("negative_only", False)),
                    "detections": kept,
                    "truth": truth,
                }
            )
    return frames


def _torch():
    try:
        import torch
    except ImportError as exc:
        raise RuntimeError("PyTorch is required for G4 evaluation") from exc
    return torch


def torch_from_numpy(value: np.ndarray):
    return _torch().from_numpy(value)


def torch_no_grad():
    return _torch().no_grad()


def torch_sigmoid(value):
    return _torch().sigmoid(value)


def detection_to_native_bbox(
    bbox_xyxy: list[float], model_size=DISCOVERY_MODEL_SIZE
) -> tuple[float, float, float, float]:
    x1, y1, x2, y2 = bbox_xyxy
    return (
        x1 * 640.0 / model_size[0],
        y1 * 480.0 / model_size[1],
        x2 * 640.0 / model_size[0],
        y2 * 480.0 / model_size[1],
    )


def classifier_crop_for_detection(
    rgb: np.ndarray,
    detection_bbox_model: list[float],
    *,
    size=CLASSIFIER_MODEL_SIZE,
    scale: float = 3.0,
) -> np.ndarray:
    height, width = rgb.shape[:2]
    native = detection_to_native_bbox(detection_bbox_model)
    crop = square_crop(width, height, native, scale=scale, minimum_side=48)
    left, top, right, bottom = crop
    return cv2.resize(
        rgb[top:bottom, left:right], size, interpolation=cv2.INTER_AREA
    )


def classify_detections(
    classifier,
    frames: list[dict],
    *,
    device,
    class_threshold: float = 0.5,
    background_priority: float = 0.0,
) -> list[dict]:
    classifier.eval()
    output_frames: list[dict] = []
    with device:
        with torch_no_grad():
            for frame in frames:
                rgb = read_rgb(frame["row"])
                refined: list[dict] = []
                for detection in frame["detections"]:
                    crop = classifier_crop_for_detection(
                        rgb, detection["bbox_xyxy"]
                    )
                    tensor = torch_from_numpy(
                        np.ascontiguousarray(
                            crop.transpose(2, 0, 1)[None], dtype=np.float32
                        )
                        / 255.0
                    ).to(device)
                    logits = classifier(tensor)[0].cpu().numpy()
                    probabilities = np.exp(
                        logits - logits.max(keepdims=True)
                    )
                    probabilities /= probabilities.sum(keepdims=True)
                    background_score = float(probabilities[0])
                    class_index = int(np.argmax(probabilities[1:])) + 1
                    class_score = float(probabilities[class_index])
                    accepted = (
                        class_score >= class_threshold
                        and class_score > background_score + background_priority
                    )
                    refined.append(
                        {
                            "class_index": class_index if accepted else 0,
                            "class_name": (
                                DISCRETE_NAMES[class_index - 1] if accepted else "background"
                            ),
                            "score": class_score if accepted else background_score,
                            "bbox_xyxy": detection["bbox_xyxy"],
                        }
                    )
                output_frames.append({**frame, "predictions": refined})
    return output_frames


def match_discrete_predictions(
    frames: list[dict],
) -> list[dict]:
    matched_frames: list[dict] = []
    for frame in frames:
        truth = frame["truth"]
        predictions = [
            item for item in frame.get("predictions", []) if item["class_index"] > 0
        ]
        used_truth: set[int] = set()
        used_pred: set[int] = set()
        matches: list[tuple[int, int]] = []
        for class_name in DISCRETE_NAMES:
            class_index = DISCRETE_NAMES.index(class_name) + 1
            truth_indices = [
                index
                for index, item in enumerate(truth)
                if item["semantic_class"] == class_name
            ]
            pred_indices = [
                index
                for index, item in enumerate(predictions)
                if item["class_index"] == class_index
            ]
            for truth_index in truth_indices:
                best_pred = -1
                best_iou = 0.0
                for pred_index in pred_indices:
                    if pred_index in used_pred:
                        continue
                    iou = box_iou(
                        tuple(float(value) for value in truth[truth_index]["bbox_xyxy"]),
                        tuple(float(value) for value in predictions[pred_index]["bbox_xyxy"]),
                    )
                    if iou > best_iou:
                        best_iou = iou
                        best_pred = pred_index
                if best_pred >= 0 and best_iou >= 0.5:
                    used_truth.add(truth_index)
                    used_pred.add(best_pred)
                    matches.append((truth_index, best_pred))
        matched_frames.append(
            {
                **frame,
                "matched_truth": used_truth,
                "matched_predictions": used_pred,
                "unmatched_predictions": [
                    index
                    for index in range(len(predictions))
                    if index not in used_pred
                ],
            }
        )
    return matched_frames


def discrete_metrics(frames: list[dict]) -> dict:
    confusion = {name: {"tp": 0, "fp": 0, "fn": 0} for name in DISCRETE_NAMES}
    small_truth_total = 0
    small_truth_matched = 0
    negative_fp_frames = 0
    negative_frames = 0
    total_false_positives = 0
    all_truth = 0
    all_matched = 0
    for frame in frames:
        predictions = [
            item
            for item in frame.get("predictions", [])
            if item["class_index"] > 0
        ]
        truth = frame["truth"]
        for index, item in enumerate(truth):
            all_truth += 1
            matched = index in frame.get("matched_truth", set())
            all_matched += int(matched)
            if float(item.get("native_short_side_px", 0.0)) < 18.0:
                small_truth_total += 1
                small_truth_matched += int(matched)
        for index, item in enumerate(predictions):
            name = item["class_name"]
            if index in frame.get("matched_predictions", set()):
                confusion[name]["tp"] += 1
            else:
                confusion[name]["fp"] += 1
                total_false_positives += 1
        for name in DISCRETE_NAMES:
            confusion[name]["fn"] += sum(
                1
                for index, item in enumerate(truth)
                if item["semantic_class"] == name
                and index not in frame.get("matched_truth", set())
            )
        if frame["negative_only"]:
            negative_frames += 1
            negative_fp_frames += int(len(frame.get("unmatched_predictions", [])) > 0)
    per_class = {}
    for name in DISCRETE_NAMES:
        tp = confusion[name]["tp"]
        fp = confusion[name]["fp"]
        fn = confusion[name]["fn"]
        precision = tp / max(tp + fp, 1)
        recall = tp / max(tp + fn, 1)
        f1 = 2 * precision * recall / max(precision + recall, 1e-12)
        per_class[name] = {
            "tp": tp,
            "fp": fp,
            "fn": fn,
            "precision": precision,
            "recall": recall,
            "f1": f1,
        }
    macro_precision = float(
        np.mean([per_class[name]["precision"] for name in DISCRETE_NAMES])
    )
    macro_recall = float(
        np.mean([per_class[name]["recall"] for name in DISCRETE_NAMES])
    )
    macro_f1 = float(np.mean([per_class[name]["f1"] for name in DISCRETE_NAMES]))
    duration_minutes = max(len(frames) / 10.0 / 60.0, 1e-9)
    return {
        "per_class": per_class,
        "macro_precision": macro_precision,
        "macro_recall": macro_recall,
        "macro_f1": macro_f1,
        "paper_precision": per_class["paper_litter"]["precision"],
        "paper_recall": per_class["paper_litter"]["recall"],
        "small_object_recall": small_truth_matched / max(small_truth_total, 1),
        "small_object_total": small_truth_total,
        "small_object_matched": small_truth_matched,
        "all_gt_candidate_recall": all_matched / max(all_truth, 1),
        "all_gt_candidate_total": all_truth,
        "all_gt_candidate_matched": all_matched,
        "negative_only_frames": negative_frames,
        "negative_only_fp_frames": negative_fp_frames,
        "negative_only_fp_per_frame": negative_fp_frames / max(negative_frames, 1),
        "false_candidates_per_min": total_false_positives / duration_minutes,
        "total_false_positives": total_false_positives,
        "frame_count": len(frames),
    }


def discovery_metrics(frames: list[dict]) -> dict:
    """Candidate-level metrics before classifier refinement."""
    total_truth = 0
    matched_truth = 0
    total_false_positives = 0
    negative_fp_frames = 0
    negative_frames = 0
    for frame in frames:
        truth = frame["truth"]
        detections = frame["detections"]
        used_truth: set[int] = set()
        frame_fp = 0
        for detection in detections:
            best_iou = 0.0
            best_index = -1
            for index, item in enumerate(truth):
                if index in used_truth:
                    continue
                iou = box_iou(
                    tuple(float(value) for value in detection["bbox_xyxy"]),
                    tuple(float(value) for value in item["bbox_xyxy"]),
                )
                if iou > best_iou:
                    best_iou = iou
                    best_index = index
            if best_index >= 0 and best_iou >= 0.5:
                used_truth.add(best_index)
                matched_truth += 1
            else:
                frame_fp += 1
                total_false_positives += 1
        total_truth += len(truth)
        if frame["negative_only"]:
            negative_frames += 1
            negative_fp_frames += int(frame_fp > 0)
    duration_minutes = max(len(frames) / 10.0 / 60.0, 1e-9)
    return {
        "all_gt_candidate_recall": matched_truth / max(total_truth, 1),
        "matched_truth": matched_truth,
        "total_truth": total_truth,
        "total_false_positives": total_false_positives,
        "negative_only_frames": negative_frames,
        "negative_only_fp_frames": negative_fp_frames,
        "negative_only_fp_per_frame": negative_fp_frames
        / max(negative_frames, 1),
        "false_candidates_per_min": total_false_positives / duration_minutes,
    }


def area_predictions(
    model,
    rows: list[dict],
    *,
    device,
    thresholds: tuple[float, float] = (0.5, 0.5),
    task: str | None = None,
) -> list[dict]:
    model.eval()
    outputs: list[dict] = []
    with device:
        with torch_no_grad():
            for row in rows:
                rgb, depth, semantic, _ = read_frame(row)
                inputs = build_area_input(
                    rgb,
                    depth,
                    AREA_MODEL_SIZE,
                    task=task if task in ("leaf", "puddle") else "leaf",
                    camera_info=load_camera_info(row),
                )
                tensor = torch_from_numpy(
                    np.ascontiguousarray(inputs.transpose(2, 0, 1)[None], dtype=np.float32)
                ).to(device)
                logits = model(tensor)["logits"][0].cpu().numpy()
                boundary = model(tensor)["boundary_logits"][0].cpu().numpy()
                probabilities = 1.0 / (1.0 + np.exp(-logits))
                boundary_probabilities = 1.0 / (1.0 + np.exp(-boundary))
                if probabilities.ndim == 3 and probabilities.shape[0] == 1:
                    zeros = np.zeros_like(probabilities)
                    if task == "leaf":
                        probabilities = np.concatenate(
                            (probabilities, zeros), axis=0
                        )
                        boundary_probabilities = np.concatenate(
                            (boundary_probabilities, zeros), axis=0
                        )
                    elif task == "puddle":
                        probabilities = np.concatenate(
                            (zeros, probabilities), axis=0
                        )
                        boundary_probabilities = np.concatenate(
                            (zeros, boundary_probabilities), axis=0
                        )
                    else:
                        probabilities = np.repeat(probabilities, 2, axis=0)
                        boundary_probabilities = np.repeat(
                            boundary_probabilities, 2, axis=0
                        )
                semantic_model = cv2.resize(
                    semantic, AREA_MODEL_SIZE, interpolation=cv2.INTER_NEAREST
                )
                truth = np.stack(
                    (semantic_model == 4, semantic_model == 5), axis=0
                ).astype(np.float32)
                if row.get("negative_only"):
                    truth = np.zeros_like(truth)
                outputs.append(
                    {
                        "row": row,
                        "negative_only": bool(row.get("negative_only", False)),
                        "probabilities": probabilities,
                        "boundary_probabilities": boundary_probabilities,
                        "truth": truth,
                        "thresholds": thresholds,
                    }
                )
    return outputs


def area_crop_predictions(
    model,
    rows: list[dict],
    *,
    device,
    thresholds: tuple[float, float] = (0.5, 0.5),
    task: str = "leaf",
) -> list[dict]:
    """Evaluate the area model on positive target crops plus full negatives."""
    model.eval()
    channel = 0 if task == "leaf" else 1
    outputs: list[dict] = []
    with device:
        with torch_no_grad():
            for row in rows:
                rgb, depth, semantic, _ = read_frame(row)
                inputs = build_area_input(
                    rgb,
                    depth,
                    AREA_MODEL_SIZE,
                    task=task,
                    camera_info=load_camera_info(row),
                )
                semantic_model = cv2.resize(
                    semantic, AREA_MODEL_SIZE, interpolation=cv2.INTER_NEAREST
                )
                truth = np.stack(
                    (semantic_model == 4, semantic_model == 5), axis=0
                ).astype(np.float32)
                if row.get("negative_only"):
                    truth = np.zeros_like(truth)
                else:
                    crop = _positive_area_crop(truth, channel)
                    if crop is not None:
                        inputs = _resize_area_image_crop(inputs, crop)
                        truth = _resize_area_mask_crop(truth, crop)
                tensor = torch_from_numpy(
                    np.ascontiguousarray(
                        inputs.transpose(2, 0, 1)[None], dtype=np.float32
                    )
                ).to(device)
                logits = model(tensor)["logits"][0].cpu().numpy()
                boundary = model(tensor)["boundary_logits"][0].cpu().numpy()
                probabilities = 1.0 / (1.0 + np.exp(-logits))
                boundary_probabilities = 1.0 / (1.0 + np.exp(-boundary))
                zeros = np.zeros_like(probabilities)
                if task == "leaf":
                    probabilities = np.concatenate(
                        (probabilities, zeros), axis=0
                    )
                    boundary_probabilities = np.concatenate(
                        (boundary_probabilities, zeros), axis=0
                    )
                else:
                    probabilities = np.concatenate(
                        (zeros, probabilities), axis=0
                    )
                    boundary_probabilities = np.concatenate(
                        (zeros, boundary_probabilities), axis=0
                    )
                outputs.append(
                    {
                        "row": row,
                        "negative_only": bool(row.get("negative_only", False)),
                        "probabilities": probabilities,
                        "boundary_probabilities": boundary_probabilities,
                        "truth": truth,
                        "thresholds": thresholds,
                    }
                )
    return outputs


def area_metrics(predictions: list[dict]) -> dict:
    intersections = np.zeros(2, dtype=np.int64)
    unions = np.zeros(2, dtype=np.int64)
    boundary_intersections = np.zeros(2, dtype=np.int64)
    boundary_unions = np.zeros(2, dtype=np.int64)
    negative_fp_frames = 0
    negative_frames = 0
    for frame in predictions:
        predicted = frame["probabilities"] >= np.asarray(frame["thresholds"])[:, None, None]
        truth = frame["truth"].astype(bool)
        intersections += (predicted & truth).sum((1, 2))
        unions += (predicted | truth).sum((1, 2))
        for channel in range(2):
            predicted_edge = mask_boundary(predicted[channel]) > 0
            truth_edge = mask_boundary(truth[channel]) > 0
            boundary_intersections[channel] += int((predicted_edge & truth_edge).sum())
            boundary_unions[channel] += int((predicted_edge | truth_edge).sum())
        if frame["negative_only"]:
            negative_frames += 1
            false_candidate = False
            for channel in range(2):
                count, _, stats, _ = cv2.connectedComponentsWithStats(
                    predicted[channel].astype(np.uint8), 8
                )
                if count > 1 and int(stats[1:, cv2.CC_STAT_AREA].max()) >= 20:
                    false_candidate = True
            negative_fp_frames += int(false_candidate)
    iou = [
        float(intersection / max(union, 1))
        for intersection, union in zip(intersections, unions)
    ]
    boundary_f1 = [
        float(
            2 * boundary_intersections[channel]
            / max(
                boundary_intersections[channel]
                + boundary_unions[channel],
                1,
            )
        )
        for channel in range(2)
    ]
    return {
        "iou_by_class": {
            "leaf_pile": iou[0],
            "puddle": iou[1],
        },
        "macro_miou": float(np.mean(iou)),
        "boundary_f1_by_class": {
            "leaf_pile": boundary_f1[0],
            "puddle": boundary_f1[1],
        },
        "boundary_f1": float(np.mean(boundary_f1)),
        "negative_only_frames": negative_frames,
        "negative_only_fp_frames": negative_fp_frames,
        "negative_area_fp_per_frame": negative_fp_frames / max(negative_frames, 1),
        "intersection_pixels": intersections.tolist(),
        "union_pixels": unions.tolist(),
    }


def stress_transform(rgb: np.ndarray, name: str) -> np.ndarray:
    if name == "grayscale":
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        return np.repeat(gray[:, :, None], 3, axis=2)
    if name == "hue_shift":
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        hsv[:, :, 0] = (hsv[:, :, 0].astype(np.int16) + 67) % 180
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
    if name == "exposure":
        return np.clip(rgb.astype(np.float32) * 0.55 + 15, 0, 255).astype(np.uint8)
    if name == "background_color_swap":
        result = rgb.copy()
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        low_texture = gray < 150
        result[low_texture] = result[low_texture][:, [1, 2, 0]]
        return result
    return rgb


def evaluate_pipeline(
    discovery,
    classifier,
    leaf,
    puddle,
    rows: list[dict],
    instances_by_key: dict[tuple[int, int], list[dict]],
    *,
    device,
    discovery_threshold: float,
    class_threshold: float,
    area_thresholds: tuple[float, float],
    stress_names: tuple[str, ...] = ("grayscale", "hue_shift", "exposure"),
) -> dict:
    frames = discovery_predictions(
        discovery,
        rows,
        instances_by_key,
        device=device,
        threshold=discovery_threshold,
    )
    classified = classify_detections(
        classifier, frames, device=device, class_threshold=class_threshold
    )
    matched = match_discrete_predictions(classified)
    discrete = discrete_metrics(matched)
    leaf_preds = area_predictions(
        leaf,
        rows,
        device=device,
        thresholds=area_thresholds,
        task="leaf",
    )
    puddle_preds = area_predictions(
        puddle,
        rows,
        device=device,
        thresholds=area_thresholds,
        task="puddle",
    )
    area_preds = []
    for leaf_frame, puddle_frame in zip(leaf_preds, puddle_preds):
        combined = dict(leaf_frame)
        combined["probabilities"] = np.stack(
            (
                leaf_frame["probabilities"][0],
                puddle_frame["probabilities"][1],
            ),
            axis=0,
        )
        combined["boundary_probabilities"] = np.stack(
            (
                leaf_frame["boundary_probabilities"][0],
                puddle_frame["boundary_probabilities"][1],
            ),
            axis=0,
        )
        area_preds.append(combined)
    area = area_metrics(area_preds)
    stress_reports = {}
    if stress_names:
        stress_rows = rows[:: max(1, len(rows) // 20)][:20]
        for name in stress_names:
            stress_frames = []
            for row in stress_rows:
                rgb = read_rgb(row)
                transformed = stress_transform(rgb, name)
                resized = cv2.resize(
                    transformed,
                    DISCOVERY_MODEL_SIZE,
                    interpolation=cv2.INTER_AREA,
                ).astype(np.float32) / 255.0
                tensor = torch_from_numpy(
                    np.ascontiguousarray(
                        resized.transpose(2, 0, 1)[None], dtype=np.float32
                    )
                ).to(device)
                with torch_no_grad():
                    outputs = discovery(tensor)
                objectness = torch_sigmoid(outputs["objectness_logits"])[0].cpu().numpy()
                detections = decode_centernet_outputs(
                    objectness,
                    outputs["offset"][0].cpu().numpy(),
                    outputs["bbox_size"][0].cpu().numpy(),
                    stride=DISCOVERY_STRIDE,
                    score_threshold=discovery_threshold,
                )
                truth = discrete_boxes_for_frame(row, instances_by_key)
                stress_frames.append(
                    {
                        "row": row,
                        "scene_seed": row["scene_seed"],
                        "frame_index": row["frame_index"],
                        "split": row["split"],
                        "world_id": row["world_id"],
                        "negative_only": bool(row.get("negative_only", False)),
                        "detections": [
                            {
                                "class_index": 0,
                                "score": float(item.score),
                                "bbox_xyxy": list(item.bbox_xyxy),
                            }
                            for item in detections
                        ],
                        "truth": truth,
                    }
                )
            stress_classified = classify_detections(
                classifier,
                stress_frames,
                device=device,
                class_threshold=class_threshold,
            )
            stress_matched = match_discrete_predictions(stress_classified)
            stress_reports[name] = discrete_metrics(stress_matched)
    stress_macro_f1 = (
        float(
            np.mean(
                [report["macro_f1"] for report in stress_reports.values()]
            )
        )
        if stress_reports
        else None
    )
    return {
        "discrete": discrete,
        "area": area,
        "stress": {
            "reports": stress_reports,
            "macro_f1": stress_macro_f1,
        },
        "thresholds": {
            "discovery": discovery_threshold,
            "classifier": class_threshold,
            "area": list(area_thresholds),
        },
    }


__all__ = [
    "area_crop_predictions",
    "area_metrics",
    "area_predictions",
    "classifier_crop_for_detection",
    "classify_detections",
    "discovery_metrics",
    "discovery_crop_predictions",
    "grid_proposal_predictions",
    "detection_to_native_bbox",
    "sliding_discovery_predictions",
    "discrete_metrics",
    "discovery_predictions",
    "evaluate_pipeline",
    "match_discrete_predictions",
    "stress_transform",
]
