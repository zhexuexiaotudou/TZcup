#!/usr/bin/env python3
"""Run the frozen EMFJ6V3 YOLOX COCO proposal-only development screen."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
PERCEPTION = ROOT / "starter_ws" / "src" / "sanitation_perception"
sys.path.insert(0, str(PERCEPTION))

from sanitation_perception.journey6_provider import TensorContract  # noqa: E402
from sanitation_perception.onnx_provider import StrictOnnxProvider  # noqa: E402
from sanitation_perception.pretrained_contracts import decode_yolo_detect  # noqa: E402


MODEL_ID = "d6_yolox_tiny_coco_onnx"
TARGET_CLASSES = ("plastic_bottle", "metal_can", "paper_litter")
THRESHOLDS = (0.001, 0.0025, 0.005, 0.01, 0.025, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5)
FORBIDDEN_MARKERS = ("G5_V2", "SEALED_FINAL", "VAL_NEW", "DEV_VAL")


class ScreeningError(ValueError):
    """Raised when data, model, or semantic screening contracts are unsafe."""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def reject_forbidden_path(path: str | Path) -> None:
    text = str(path).replace("/", "\\").upper()
    if any(marker in text for marker in FORBIDDEN_MARKERS):
        raise ScreeningError("forbidden validation or sealed path")
    if re.search(r"(?:^|[\\_.-])G5(?:$|[\\_.-])", text):
        raise ScreeningError("generic G5 path is forbidden")


def load_candidate(registry_path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(registry_path.read_text(encoding="utf-8"))
    matches = [item for item in payload["candidates"] if item["model_id"] == MODEL_ID]
    if len(matches) != 1:
        raise ScreeningError("frozen YOLOX candidate is missing or duplicated")
    candidate = matches[0]
    if candidate["class_semantics"] != {"bottle": "generic_bottle_proposal_not_plastic_bottle"}:
        raise ScreeningError("YOLOX semantics must remain proposal-only")
    return candidate


def rebase_source_path(file_name: str, source_prefix: str, data_root: Path) -> tuple[Path, str]:
    normalized = file_name.replace("\\", "/")
    prefix = source_prefix.replace("\\", "/").rstrip("/")
    if not normalized.lower().startswith(prefix.lower() + "/"):
        raise ScreeningError("COCO image path is outside the frozen source prefix")
    relative = normalized[len(prefix) + 1 :]
    actual = (data_root / Path(*relative.split("/"))).resolve()
    root = data_root.resolve()
    if actual != root and root not in actual.parents:
        raise ScreeningError("rebased image escapes the development root")
    return actual, relative


def build_dataset(
    coco_path: Path,
    *,
    expected_coco_sha256: str,
    source_prefix: str,
    data_root: Path,
) -> dict[str, Any]:
    reject_forbidden_path(coco_path)
    reject_forbidden_path(data_root)
    actual_coco_sha = sha256(coco_path)
    if actual_coco_sha != expected_coco_sha256.lower():
        raise ScreeningError("COCO SHA-256 mismatch")
    payload = json.loads(coco_path.read_text(encoding="utf-8"))
    for flag in ("G10_DEV_VAL_SEALED_read", "VAL_NEW_read", "G5_V2_read"):
        if payload.get(flag) is not False:
            raise ScreeningError(f"forbidden read flag is not false: {flag}")
    categories = {int(item["id"]): item["name"] for item in payload["categories"]}
    if tuple(categories[index] for index in sorted(categories)) != TARGET_CLASSES:
        raise ScreeningError("development categories do not match the fixed three classes")
    annotations: dict[int, list[dict[str, Any]]] = {}
    for item in payload["annotations"]:
        x, y, width, height = (float(value) for value in item["bbox"])
        annotations.setdefault(int(item["image_id"]), []).append({
            "category_id": int(item["category_id"]),
            "category_name": categories[int(item["category_id"])],
            "bbox_xyxy": [x, y, x + width, y + height],
            "bbox_short_side_px": float(min(width, height)),
        })
    images = []
    for item in sorted(payload["images"], key=lambda row: int(row["id"])):
        if item.get("source_split") != "train":
            raise ScreeningError("only the fixed TRAIN development split is allowed")
        actual, relative = rebase_source_path(item["file_name"], source_prefix, data_root)
        if not actual.is_file():
            raise ScreeningError(f"rebased development image is missing: {relative}")
        image_annotations = annotations.get(int(item["id"]), [])
        images.append({
            "image_id": int(item["id"]),
            "relative_file": relative,
            "runtime_file": str(actual),
            "sha256": sha256(actual),
            "width": int(item["width"]),
            "height": int(item["height"]),
            "mission_id": item["mission_id"],
            "world_id": item["world_id"],
            "scene_seed": int(item["scene_seed"]),
            "frame_index": int(item["frame_index"]),
            "source_negative_only": bool(item["negative_only"]),
            "annotations": image_annotations,
        })
    annotation_count = sum(len(item["annotations"]) for item in images)
    return {
        "schema_version": 1,
        "protocol_id": "EMFJ6V3",
        "split": "TRAIN_DEVELOPMENT_FIXED",
        "source_coco_sha256": actual_coco_sha,
        "image_count": len(images),
        "annotation_count": annotation_count,
        "unannotated_frame_count": sum(not item["annotations"] for item in images),
        "negative_only_frame_count": sum(item["source_negative_only"] for item in images),
        "target_counts": {
            name: sum(
                annotation["category_name"] == name
                for item in images
                for annotation in item["annotations"]
            )
            for name in TARGET_CLASSES
        },
        "formal_A4_negative_domain_ready": False,
        "images": images,
        "truth_boundary": (
            "Fixed nonsealed TRAIN proposal screen only. Unannotated frames are not "
            "an independent negative-only domain, and this manifest is not HOLDOUT, VAL, or sealed evidence."
        ),
    }


def yolox_preprocess(image: np.ndarray) -> tuple[np.ndarray, float]:
    if image.ndim != 3 or image.shape[2] != 3:
        raise ScreeningError("YOLOX input must be BGR HWC")
    height, width = image.shape[:2]
    ratio = min(416.0 / height, 416.0 / width)
    resized = cv2.resize(
        image,
        (int(width * ratio), int(height * ratio)),
        interpolation=cv2.INTER_LINEAR,
    )
    padded = np.full((416, 416, 3), 114, dtype=np.uint8)
    padded[: resized.shape[0], : resized.shape[1]] = resized
    tensor = padded.transpose(2, 0, 1)[None].astype(np.float32)
    return np.ascontiguousarray(tensor), ratio


def box_iou(first: list[float], second: list[float]) -> float:
    x1 = max(first[0], second[0])
    y1 = max(first[1], second[1])
    x2 = min(first[2], second[2])
    y2 = min(first[3], second[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    second_area = max(0.0, second[2] - second[0]) * max(0.0, second[3] - second[1])
    union = first_area + second_area - intersection
    return intersection / union if union > 0 else 0.0


def greedy_matches(predictions: list[dict[str, Any]], targets: list[dict[str, Any]]) -> list[tuple[int, int]]:
    pairs = []
    for prediction_index, prediction in enumerate(predictions):
        for target_index, target in enumerate(targets):
            pairs.append((box_iou(prediction["bbox_xyxy"], target["bbox_xyxy"]), prediction_index, target_index))
    used_predictions: set[int] = set()
    used_targets: set[int] = set()
    matches = []
    for overlap, prediction_index, target_index in sorted(pairs, reverse=True):
        if overlap < 0.5:
            break
        if prediction_index in used_predictions or target_index in used_targets:
            continue
        used_predictions.add(prediction_index)
        used_targets.add(target_index)
        matches.append((prediction_index, target_index))
    return matches


def evaluate_threshold(dataset: dict[str, Any], inference: dict[int, list[dict[str, Any]]], threshold: float) -> dict[str, Any]:
    matched_total = false_positives = 0
    matched_by_class = {name: 0 for name in TARGET_CLASSES}
    unannotated_predictions = 0
    for image in dataset["images"]:
        predictions = [row for row in inference[image["image_id"]] if row["confidence"] >= threshold]
        matches = greedy_matches(predictions, image["annotations"])
        matched_total += len(matches)
        false_positives += len(predictions) - len(matches)
        if not image["annotations"]:
            unannotated_predictions += len(predictions)
        for _, target_index in matches:
            matched_by_class[image["annotations"][target_index]["category_name"]] += 1
    annotation_count = dataset["annotation_count"]
    image_count = dataset["image_count"]
    unannotated_count = dataset["unannotated_frame_count"]
    return {
        "threshold": threshold,
        "proposal_recall_class_agnostic": matched_total / annotation_count if annotation_count else 0.0,
        "proposal_false_positives_per_frame": false_positives / image_count if image_count else 0.0,
        "unannotated_predictions_per_unannotated_frame": (
            unannotated_predictions / unannotated_count if unannotated_count else 0.0
        ),
        "per_target_proposal_recall": {
            name: (
                matched_by_class[name] / dataset["target_counts"][name]
                if dataset["target_counts"][name]
                else None
            )
            for name in TARGET_CLASSES
        },
        "semantic_precision_recall_f1": "not_applicable_semantic_mapping_absent",
        "product_candidate_pass": False,
    }


def run_inference(dataset: dict[str, Any], model: Path, model_sha: str, class_order: list[str], provider_id: str) -> tuple[dict[int, list[dict[str, Any]]], dict[str, Any]]:
    provider = StrictOnnxProvider(
        artifact=model,
        artifact_sha256=model_sha,
        inputs=(TensorContract("images", (1, 3, 416, 416), "float32"),),
        outputs=(TensorContract("output", (1, 3549, 85), "float32"),),
        provider=provider_id,
    )
    provider.load()
    provider.warmup(2)
    inference: dict[int, list[dict[str, Any]]] = {}
    for image in dataset["images"]:
        frame = cv2.imread(image["runtime_file"], cv2.IMREAD_COLOR)
        if frame is None:
            raise ScreeningError(f"failed to read development image: {image['relative_file']}")
        tensor, ratio = yolox_preprocess(frame)
        output = provider.infer({"images": tensor})["output"]
        detections = decode_yolo_detect(
            output,
            class_order=class_order,
            class_mapping={},
            score_threshold=min(THRESHOLDS),
            nms_iou_threshold=0.45,
            input_size=(416, 416),
            has_objectness=True,
            maximum_detections=300,
        )
        rows = []
        for detection in detections:
            x1, y1, x2, y2 = detection.bbox_xyxy
            rows.append({
                "bbox_xyxy": [
                    max(0.0, min(image["width"], x1 / ratio)),
                    max(0.0, min(image["height"], y1 / ratio)),
                    max(0.0, min(image["width"], x2 / ratio)),
                    max(0.0, min(image["height"], y2 / ratio)),
                ],
                "confidence": detection.score,
                "source_class_name": detection.source_class,
                "target_category_id": None,
            })
        inference[image["image_id"]] = rows
    return inference, provider.health()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", type=Path, default=ROOT / "config" / "existing_model_candidates_v3.yaml")
    parser.add_argument("--coco", type=Path, required=True)
    parser.add_argument("--coco-sha256", required=True)
    parser.add_argument("--source-prefix", required=True)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--model-sha256", required=True)
    parser.add_argument("--provider", choices=("onnx_cpu", "onnx_cuda"), default="onnx_cpu")
    parser.add_argument("--raw-output", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    for path in (arguments.registry, arguments.model, arguments.raw_output, arguments.output):
        reject_forbidden_path(path)
    candidate = load_candidate(arguments.registry)
    registered_sha = next(item["sha256"] for item in candidate["files"] if item["name"] == "yolox_tiny.onnx")
    if arguments.model_sha256.lower() != registered_sha:
        raise ScreeningError("CLI model SHA does not match frozen registry")
    dataset = build_dataset(
        arguments.coco,
        expected_coco_sha256=arguments.coco_sha256,
        source_prefix=arguments.source_prefix,
        data_root=arguments.data_root,
    )
    inference, health = run_inference(
        dataset,
        arguments.model,
        arguments.model_sha256.lower(),
        candidate["class_order"],
        arguments.provider,
    )
    raw_payload = {
        "schema_version": 1,
        "protocol_id": "EMFJ6V3",
        "model_id": MODEL_ID,
        "model_sha256": arguments.model_sha256.lower(),
        "source_coco_sha256": dataset["source_coco_sha256"],
        "source_class_order": candidate["class_order"],
        "class_mapping": {},
        "semantic_mapping_available": False,
        "predictions": [
            {
                "image_id": image["image_id"],
                "relative_file": image["relative_file"],
                "rows": inference[image["image_id"]],
            }
            for image in dataset["images"]
        ],
    }
    arguments.raw_output.parent.mkdir(parents=True, exist_ok=True)
    arguments.raw_output.write_text(
        json.dumps(raw_payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    dataset_public = {**dataset, "images": [{key: value for key, value in row.items() if key != "runtime_file"} for row in dataset["images"]]}
    report = {
        "schema_version": 1,
        "protocol_id": "EMFJ6V3",
        "model_id": MODEL_ID,
        "model_sha256": arguments.model_sha256.lower(),
        "provider": health,
        "raw_inference_file": arguments.raw_output.name,
        "raw_inference_sha256": sha256(arguments.raw_output),
        "dataset": dataset_public,
        "threshold_scan": [evaluate_threshold(dataset, inference, value) for value in THRESHOLDS],
        "EMF_EXISTING_MODEL_SCREENING_COMPLETE": False,
        "EMF_EXISTING_MODEL_FUNCTIONAL_CANDIDATE": False,
        "EMF_EXISTING_MODEL_PRODUCT_CANDIDATE": False,
        "training_authorized": False,
        "truth_boundary": (
            "YOLOX COCO proposal-only TRAIN diagnostic. COCO bottle is not plastic_bottle; "
            "metal_can and paper_litter semantics are absent. No product semantic, HOLDOUT, "
            "independent negative-domain, Journey 6, or training claim is permitted."
        ),
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({
        "model_id": MODEL_ID,
        "images": dataset["image_count"],
        "annotations": dataset["annotation_count"],
        "report_sha256": sha256(arguments.output),
        "screening_complete": False,
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
