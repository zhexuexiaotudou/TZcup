#!/usr/bin/env python3
"""Evaluate canonical D1 ONNX on the explicitly TRAIN-only development index."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
BLOCKED_PATH_TOKENS = ("DEV_VAL", "G5_V2", "SEALED_FINAL")
D1_ONNX_SHA256 = "01c72cdbcd08b6fd91c9a56a065f19837bffd67cca175a75b39e295c3afc01f5"
CATEGORY_NAMES = {1: "plastic_bottle", 2: "metal_can", 3: "paper_litter"}
SOURCE_CLASS_NAMES = {
    0: "cigarette_butt",
    1: "plastic_bottle",
    2: "drinks_can",
    3: "fast_food_packaging",
    4: "plastic_bag",
    5: "coffee_cup",
    6: "glass_bottle",
    7: "paper_waste",
    8: "food_wrapper",
    9: "general_litter",
}
THRESHOLDS = (0.001, 0.0025, 0.005, 0.01, 0.025, 0.05, 0.1, 0.2, 0.25, 0.3, 0.4, 0.5)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def remap_path(value: str, old_prefix: Path, new_prefix: Path) -> Path:
    normalized = value.replace("/", "\\")
    old = str(old_prefix).replace("/", "\\").rstrip("\\")
    if not normalized.lower().startswith((old + "\\").lower()):
        raise RuntimeError(f"dataset path is outside declared stale prefix: {value}")
    suffix = normalized[len(old) + 1 :]
    return new_prefix.joinpath(*suffix.split("\\"))


def prepare_development_selection(
    coco_path: Path,
    old_prefix: Path,
    development_root: Path,
) -> dict[str, Any]:
    coco = json.loads(coco_path.read_text(encoding="utf-8"))
    forbidden_reads = {
        key: coco.get(key)
        for key in ("G10_DEV_VAL_SEALED_read", "VAL_NEW_read", "G5_V2_read")
    }
    if any(value is not False for value in forbidden_reads.values()):
        raise RuntimeError(f"sealed/validation read flags are not all false: {forbidden_reads}")
    categories = {
        int(item["id"]): str(item["name"]) for item in coco.get("categories", [])
    }
    if categories != CATEGORY_NAMES:
        raise RuntimeError(f"unexpected development category contract: {categories}")
    annotations_by_image: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for annotation in coco.get("annotations", []):
        x, y, width, height = (float(value) for value in annotation["bbox"])
        category_id = int(annotation["category_id"])
        if category_id not in CATEGORY_NAMES:
            raise RuntimeError(f"unexpected annotation category: {category_id}")
        annotations_by_image[int(annotation["image_id"])].append(
            {
                "annotation_id": int(annotation["id"]),
                "category_id": category_id,
                "bbox_xyxy": [x, y, x + width, y + height],
                "area": float(annotation.get("area", width * height)),
                "bbox_short_side_px": float(
                    annotation.get("bbox_short_side_px", min(width, height))
                ),
            }
        )
    root = development_root.resolve()
    images: list[dict[str, Any]] = []
    for image in sorted(coco.get("images", []), key=lambda item: int(item["id"])):
        if image.get("source_split") != "train":
            raise RuntimeError(
                f"development index contains non-TRAIN image {image.get('id')}: "
                f"{image.get('source_split')}"
            )
        source_value = str(image.get("file_name", ""))
        if any(token.lower() in source_value.lower() for token in BLOCKED_PATH_TOKENS):
            raise RuntimeError(f"blocked split token in TRAIN image path: {source_value}")
        resolved = remap_path(source_value, old_prefix, root).resolve()
        if not resolved.is_relative_to(root) or not resolved.is_file():
            raise RuntimeError(f"mapped TRAIN image is missing or escapes root: {resolved}")
        image_id = int(image["id"])
        images.append(
            {
                "image_id": image_id,
                "relative_path": resolved.relative_to(root).as_posix(),
                "sha256": sha256(resolved),
                "source_split": "train",
                "width": int(image["width"]),
                "height": int(image["height"]),
                "mission_id": str(image.get("mission_id", image.get("scene", image_id))),
                "scene": str(image.get("scene", "")),
                "frame_index": int(image.get("frame_index", 0)),
                "annotations": annotations_by_image.pop(image_id, []),
            }
        )
    if annotations_by_image:
        raise RuntimeError(
            f"annotations refer to missing TRAIN images: {sorted(annotations_by_image)}"
        )
    annotation_count = sum(len(item["annotations"]) for item in images)
    return {
        "schema_version": 1,
        "development_only": True,
        "competition_claim_allowed": False,
        "release_allowed": False,
        "selection_rule": (
            "every COCO image must have source_split=train; reject paths containing "
            "DEV_VAL, G5_V2, or SEALED_FINAL; remap only the declared stale prefix; "
            "verify every image SHA256"
        ),
        "source_coco": str(coco_path.resolve()),
        "source_coco_sha256": sha256(coco_path),
        "old_prefix": str(old_prefix),
        "mapped_root": str(root),
        "forbidden_read_flags": forbidden_reads,
        "category_names": CATEGORY_NAMES,
        "image_count": len(images),
        "annotation_count": annotation_count,
        "images": images,
    }


def box_iou(left: list[float], right: list[float]) -> float:
    x1 = max(left[0], right[0])
    y1 = max(left[1], right[1])
    x2 = min(left[2], right[2])
    y2 = min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = max(0.0, left[2] - left[0]) * max(0.0, left[3] - left[1])
    right_area = max(0.0, right[2] - right[0]) * max(0.0, right[3] - right[1])
    union = left_area + right_area - intersection
    return intersection / union if union > 0 else 0.0


def greedy_matches(
    ground_truth: list[dict[str, Any]],
    predictions: list[dict[str, Any]],
    *,
    class_aware: bool,
    iou_threshold: float = 0.5,
) -> tuple[list[tuple[int, int, float]], set[int], set[int]]:
    matches: list[tuple[int, int, float]] = []
    unmatched_ground_truth = set(range(len(ground_truth)))
    prediction_order = sorted(
        range(len(predictions)),
        key=lambda index: float(predictions[index]["confidence"]),
        reverse=True,
    )
    for prediction_index in prediction_order:
        prediction = predictions[prediction_index]
        candidates = []
        for ground_truth_index in unmatched_ground_truth:
            target = ground_truth[ground_truth_index]
            if class_aware and int(prediction["target_category_id"]) != int(
                target["category_id"]
            ):
                continue
            overlap = box_iou(prediction["bbox_xyxy"], target["bbox_xyxy"])
            if overlap >= iou_threshold:
                candidates.append((overlap, ground_truth_index))
        if candidates:
            overlap, ground_truth_index = max(candidates)
            unmatched_ground_truth.remove(ground_truth_index)
            matches.append((prediction_index, ground_truth_index, overlap))
    matched_predictions = {item[0] for item in matches}
    return (
        matches,
        set(range(len(predictions))) - matched_predictions,
        unmatched_ground_truth,
    )


def prf(tp: int, fp: int, fn: int) -> dict[str, Any]:
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
    return {
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": precision,
        "recall": recall,
        "f1": f1,
    }


def evaluate_threshold(
    selection: dict[str, Any],
    inference: dict[str, Any],
    confidence_threshold: float,
) -> dict[str, Any]:
    inference_by_image = {
        int(item["image_id"]): item for item in inference.get("images", [])
    }
    class_counts = {
        category_id: {"tp": 0, "fp": 0, "fn": 0}
        for category_id in CATEGORY_NAMES
    }
    frame_tp = frame_fp = frame_fn = 0
    object_tp = object_fp = object_fn = 0
    proposal_tp = proposal_fp = 0
    small_total = small_tp = 0
    negative_frames = negative_frames_with_fp = negative_prediction_count = 0
    eventual_total: set[tuple[str, int]] = set()
    eventual_hit: set[tuple[str, int]] = set()
    confusion: dict[str, dict[str, int]] = {
        CATEGORY_NAMES[category_id]: {
            **{name: 0 for name in CATEGORY_NAMES.values()},
            "missed": 0,
        }
        for category_id in CATEGORY_NAMES
    }
    proposal_source_confusion: dict[str, dict[str, int]] = {
        CATEGORY_NAMES[category_id]: {
            **{name: 0 for name in SOURCE_CLASS_NAMES.values()},
            "missed": 0,
        }
        for category_id in CATEGORY_NAMES
    }
    for image in selection["images"]:
        image_id = int(image["image_id"])
        if image_id not in inference_by_image:
            raise RuntimeError(f"inference missing image id {image_id}")
        ground_truth = image["annotations"]
        all_predictions = [
            item
            for item in inference_by_image[image_id]["predictions"]
            if float(item["confidence"]) >= confidence_threshold
        ]
        predictions = [
            item for item in all_predictions if item.get("target_category_id") is not None
        ]
        if not ground_truth:
            negative_frames += 1
            if all_predictions:
                negative_frames_with_fp += 1
                negative_prediction_count += len(all_predictions)
        matches, unmatched_predictions, unmatched_ground_truth = greedy_matches(
            ground_truth, predictions, class_aware=True
        )
        object_tp += len(matches)
        object_fp += len(unmatched_predictions)
        object_fn += len(unmatched_ground_truth)
        for prediction_index, ground_truth_index, _overlap in matches:
            category_id = int(ground_truth[ground_truth_index]["category_id"])
            class_counts[category_id]["tp"] += 1
            eventual_hit.add((str(image["mission_id"]), category_id))
        for prediction_index in unmatched_predictions:
            category_id = int(predictions[prediction_index]["target_category_id"])
            class_counts[category_id]["fp"] += 1
        for ground_truth_index in unmatched_ground_truth:
            category_id = int(ground_truth[ground_truth_index]["category_id"])
            class_counts[category_id]["fn"] += 1
        proposal_matches, _proposal_fp, _proposal_fn = greedy_matches(
            ground_truth, all_predictions, class_aware=False
        )
        proposal_tp += len(proposal_matches)
        proposal_fp += len(all_predictions) - len(proposal_matches)
        if ground_truth and proposal_matches:
            frame_tp += 1
        elif ground_truth:
            frame_fn += 1
        elif all_predictions:
            frame_fp += 1
        matched_ground_truth = {item[1] for item in matches}
        for ground_truth_index, item in enumerate(ground_truth):
            category_id = int(item["category_id"])
            eventual_total.add((str(image["mission_id"]), category_id))
            if float(item["bbox_short_side_px"]) <= 16.0:
                small_total += 1
                if ground_truth_index in matched_ground_truth:
                    small_tp += 1
            overlaps = [
                (box_iou(item["bbox_xyxy"], prediction["bbox_xyxy"]), prediction)
                for prediction in predictions
            ]
            best = max(overlaps, default=(0.0, None), key=lambda value: value[0])
            actual_name = CATEGORY_NAMES[category_id]
            if best[0] >= 0.5 and best[1] is not None:
                predicted_name = CATEGORY_NAMES[int(best[1]["target_category_id"])]
                confusion[actual_name][predicted_name] += 1
            else:
                confusion[actual_name]["missed"] += 1
            proposal_overlaps = [
                (box_iou(item["bbox_xyxy"], prediction["bbox_xyxy"]), prediction)
                for prediction in all_predictions
            ]
            proposal_best = max(
                proposal_overlaps, default=(0.0, None), key=lambda value: value[0]
            )
            if proposal_best[0] >= 0.5 and proposal_best[1] is not None:
                source_name = SOURCE_CLASS_NAMES[
                    int(proposal_best[1]["source_class_index"])
                ]
                proposal_source_confusion[actual_name][source_name] += 1
            else:
                proposal_source_confusion[actual_name]["missed"] += 1
    image_count = len(selection["images"])
    annotation_count = int(selection["annotation_count"])
    per_class = {
        CATEGORY_NAMES[category_id]: prf(**counts)
        for category_id, counts in class_counts.items()
    }
    frame = prf(frame_tp, frame_fp, frame_fn)
    objects = prf(object_tp, object_fp, object_fn)
    return {
        "confidence_threshold": confidence_threshold,
        "iou_threshold": 0.5,
        "frame": frame,
        "frame_definition": (
            "positive only when at least one class-agnostic proposal overlaps a "
            "ground-truth box at IoU >= 0.5; a negative frame is positive when it "
            "contains any proposal"
        ),
        "objects_micro": objects,
        "per_class": per_class,
        "proposal_recall_class_agnostic": (
            proposal_tp / annotation_count if annotation_count else 0.0
        ),
        "eventual_recall_mission_category": (
            len(eventual_hit) / len(eventual_total) if eventual_total else 0.0
        ),
        "eventual_hit_count": len(eventual_hit),
        "eventual_total_count": len(eventual_total),
        "small_definition": "ground-truth bbox_short_side_px <= 16",
        "small_recall": small_tp / small_total if small_total else 0.0,
        "small_tp": small_tp,
        "small_total": small_total,
        "false_positives_per_frame": proposal_fp / image_count if image_count else 0.0,
        "negative_frames": negative_frames,
        "negative_frames_with_false_positive": negative_frames_with_fp,
        "negative_frame_false_positive_rate": (
            negative_frames_with_fp / negative_frames if negative_frames else 0.0
        ),
        "negative_false_positive_count": negative_prediction_count,
        "negative_false_positives_per_negative_frame": (
            negative_prediction_count / negative_frames if negative_frames else 0.0
        ),
        "confusion_gt_rows": confusion,
        "proposal_source_confusion_gt_rows": proposal_source_confusion,
    }


def build_report(
    selection: dict[str, Any],
    inference: dict[str, Any],
) -> dict[str, Any]:
    scan = [evaluate_threshold(selection, inference, value) for value in THRESHOLDS]
    selected = max(
        scan,
        key=lambda item: (
            item["objects_micro"]["f1"],
            -item["false_positives_per_frame"],
            item["confidence_threshold"],
        ),
    )
    failed_classes = [
        name for name, metrics in selected["per_class"].items() if metrics["tp"] == 0
    ]
    failure_reasons = [f"zero_true_positive:{name}" for name in failed_classes]
    if selected["false_positives_per_frame"] > 1.0:
        failure_reasons.append("false_positives_per_frame_gt_1")
    development_usable = not failure_reasons
    return {
        "schema_version": 1,
        "model_id": "d1_littercam_yolov9c_development_export",
        "development_only": True,
        "training_tuned_evaluation": True,
        "competition_claim_allowed": False,
        "release_allowed": False,
        "development_usable": development_usable,
        "failure_reasons": failure_reasons,
        "selection_policy": (
            "maximize TRAIN-only object micro F1; tie-break by lower false positives "
            "per frame and then higher confidence threshold"
        ),
        "usability_policy": (
            "all three target classes require at least one true positive and "
            "false positives per frame must be <= 1"
        ),
        "dataset": {
            key: selection[key]
            for key in (
                "source_coco",
                "source_coco_sha256",
                "selection_rule",
                "forbidden_read_flags",
                "image_count",
                "annotation_count",
                "category_names",
            )
        },
        "model": {
            "onnx_sha256": inference["onnx_sha256"],
            "providers": inference["providers"],
            "preprocessing": inference["preprocessing"],
            "output_contract": inference["output_contract"],
            "nms": inference["nms"],
            "source_to_target_category": inference["source_to_target_category"],
            "raw_score_diagnostics": inference.get("raw_score_diagnostics"),
        },
        "threshold_scan": scan,
        "selected_threshold": selected["confidence_threshold"],
        "selected_metrics": selected,
    }


def mount(path: Path, target: str, mode: str = "rw") -> str:
    return f"{path.resolve()}:{target}:{mode}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--coco", type=Path, required=True)
    parser.add_argument("--old-prefix", type=Path, required=True)
    parser.add_argument("--development-root", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--yolov9-source", type=Path, required=True)
    parser.add_argument("--site-packages", type=Path, required=True)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--image", default="tzcup/perception-product:v12-functional")
    parser.add_argument("--expected-images", type=int, default=410)
    parser.add_argument("--expected-annotations", type=int, default=81)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if sha256(args.model.resolve()) != D1_ONNX_SHA256:
        raise RuntimeError("refusing development evaluation of an unpinned ONNX")
    evidence = args.evidence_root.resolve()
    evidence.mkdir(parents=True, exist_ok=True)
    selection = prepare_development_selection(
        args.coco.resolve(), args.old_prefix, args.development_root.resolve()
    )
    if selection["image_count"] != args.expected_images:
        raise RuntimeError(
            f"expected {args.expected_images} TRAIN images, got {selection['image_count']}"
        )
    if selection["annotation_count"] != args.expected_annotations:
        raise RuntimeError(
            f"expected {args.expected_annotations} annotations, "
            f"got {selection['annotation_count']}"
        )
    selection_path = evidence / "D1_DEVELOPMENT_IMAGE_SELECTION.json"
    selection_path.write_text(
        json.dumps(selection, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    inference_path = evidence / "D1_DEVELOPMENT_RAW_INFERENCE.json"
    command = [
        "docker",
        "run",
        "--rm",
        "--gpus",
        "all",
        "--entrypoint",
        "/bin/bash",
        "-e",
        "PYTHONPATH=/opt/d1site",
        "-v",
        mount(args.model.resolve().parent, "/onnx", "ro"),
        "-v",
        mount(args.yolov9_source, "/source", "ro"),
        "-v",
        mount(args.site_packages, "/opt/d1site", "ro"),
        "-v",
        mount(args.development_root, "/devroot", "ro"),
        "-v",
        mount(evidence, "/evidence"),
        "-v",
        mount(ROOT / "scripts", "/tools", "ro"),
        args.image,
        "-lc",
        "python3 /tools/d1_export_worker.py infer-development "
        "--source /source "
        f"--model /onnx/{args.model.name} "
        "--selection /evidence/D1_DEVELOPMENT_IMAGE_SELECTION.json "
        "--development-root /devroot "
        "--output /evidence/D1_DEVELOPMENT_RAW_INFERENCE.json",
    ]
    result = subprocess.run(
        command,
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        check=False,
    )
    (evidence / "D1_DEVELOPMENT_INFERENCE.stdout.log").write_text(
        result.stdout, encoding="utf-8"
    )
    (evidence / "D1_DEVELOPMENT_INFERENCE.stderr.log").write_text(
        result.stderr, encoding="utf-8"
    )
    if result.returncode != 0 or not inference_path.is_file():
        blocked = {
            "schema_version": 1,
            "development_only": True,
            "development_usable": False,
            "status": "blocked_inference",
            "exit_code": result.returncode,
            "raw_error_tail": result.stderr[-8000:],
        }
        (evidence / "D1_DEVELOPMENT_OFFLINE_REPORT.json").write_text(
            json.dumps(blocked, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(result.stderr, file=sys.stderr)
        return 2
    inference = json.loads(inference_path.read_text(encoding="utf-8"))
    if inference.get("image_count") != selection["image_count"]:
        raise RuntimeError("raw inference image count does not match selection")
    report = build_report(selection, inference)
    report_path = evidence / "D1_DEVELOPMENT_OFFLINE_REPORT.json"
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["development_usable"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
