#!/usr/bin/env python3
"""Run a fail-closed C1 ONNX smoke on fixed development-only GT crops."""

from __future__ import annotations

import argparse
import json
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
from screen_emf_yolox_reference import (  # noqa: E402
    ScreeningError,
    rebase_source_path,
    reject_forbidden_path,
    sha256,
)


MODEL_ID = "c1_wastewise_yolov8n_cls"
CLASS_ORDER = ("battery", "biological", "cardboard", "glass", "metal", "paper", "plastic", "trash")
PRODUCT_CLASSES = ("background", "plastic_bottle", "metal_can", "paper_litter")
CLASS_MAPPING = {"metal": "metal_can", "paper": "paper_litter", "plastic": "plastic_bottle"}


def load_candidate(registry_path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(registry_path.read_text(encoding="utf-8"))
    matches = [item for item in payload["candidates"] if item["model_id"] == MODEL_ID]
    if len(matches) != 1:
        raise ScreeningError("frozen C1 candidate is missing or duplicated")
    candidate = matches[0]
    if tuple(candidate["class_order"]) != CLASS_ORDER:
        raise ScreeningError("C1 class order changed")
    return candidate


def classifier_preprocess(image_bgr: np.ndarray, bbox_xyxy: list[float]) -> np.ndarray:
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ScreeningError("classifier source image must be BGR HWC")
    height, width = image_bgr.shape[:2]
    x1, y1, x2, y2 = bbox_xyxy
    left = max(0, min(width - 1, int(np.floor(x1))))
    top = max(0, min(height - 1, int(np.floor(y1))))
    right = max(left + 1, min(width, int(np.ceil(x2))))
    bottom = max(top + 1, min(height, int(np.ceil(y2))))
    crop = image_bgr[top:bottom, left:right]
    rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (224, 224), interpolation=cv2.INTER_NEAREST)
    tensor = resized.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return np.ascontiguousarray(tensor)


def classification_metrics(rows: list[dict[str, Any]]) -> dict[str, Any]:
    confusion = {
        actual: {predicted: 0 for predicted in PRODUCT_CLASSES}
        for actual in PRODUCT_CLASSES
    }
    for row in rows:
        confusion[row["actual_product_class"]][row["predicted_product_class"]] += 1
    per_class = {}
    f1_values = []
    for class_name in PRODUCT_CLASSES:
        true_positive = confusion[class_name][class_name]
        false_positive = sum(confusion[actual][class_name] for actual in PRODUCT_CLASSES if actual != class_name)
        false_negative = sum(confusion[class_name][predicted] for predicted in PRODUCT_CLASSES if predicted != class_name)
        precision = true_positive / (true_positive + false_positive) if true_positive + false_positive else 0.0
        recall = true_positive / (true_positive + false_negative) if true_positive + false_negative else 0.0
        f1 = 2 * precision * recall / (precision + recall) if precision + recall else 0.0
        per_class[class_name] = {"precision": precision, "recall": recall, "f1": f1, "support": sum(confusion[class_name].values())}
        f1_values.append(f1)
    background_support = sum(confusion["background"].values())
    return {
        "confusion": confusion,
        "per_class": per_class,
        "macro_f1": sum(f1_values) / len(f1_values),
        "background_specificity": (
            confusion["background"]["background"] / background_support
            if background_support else None
        ),
    }


def load_records(
    manifest_path: Path,
    *,
    expected_manifest_sha256: str,
    source_prefix: str,
    data_root: Path,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    reject_forbidden_path(manifest_path)
    reject_forbidden_path(data_root)
    if sha256(manifest_path) != expected_manifest_sha256.lower():
        raise ScreeningError("GT smoke manifest SHA-256 mismatch")
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    if payload.get("source_split") != "G10_TRAIN":
        raise ScreeningError("C1 smoke only allows the fixed G10_TRAIN manifest")
    if payload.get("formal_eligible") is not False or payload.get("development_only") is not True:
        raise ScreeningError("GT crop manifest must remain development-only and informal")
    if payload.get("production_runtime_gt_used") is not False:
        raise ScreeningError("production runtime GT use must remain false")
    records = []
    for item in payload["records"]:
        if item.get("crop_source") != "offline_gt_box_development_only":
            raise ScreeningError("C1 smoke requires explicitly marked offline GT crops")
        actual, relative = rebase_source_path(item["rgb_path"], source_prefix, data_root)
        if not actual.is_file():
            raise ScreeningError(f"GT smoke source image missing: {relative}")
        actual_class = item["class_id"]
        if actual_class not in PRODUCT_CLASSES:
            raise ScreeningError("GT smoke class is outside the fixed product classes")
        records.append({
            "record_id": item["record_id"],
            "relative_file": relative,
            "runtime_file": str(actual),
            "source_image_sha256": sha256(actual),
            "bbox_xyxy": [float(value) for value in item["proposal_bbox_native_xyxy"]],
            "actual_product_class": actual_class,
            "world_id": item["world_id"],
            "mission_id": item["scene"],
            "frame_index": int(item["frame_index"]),
            "crop_source": item["crop_source"],
        })
    return payload, records


def run(
    records: list[dict[str, Any]],
    *,
    model: Path,
    model_sha256: str,
    provider_id: str,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    provider = StrictOnnxProvider(
        artifact=model,
        artifact_sha256=model_sha256,
        inputs=(TensorContract("images", (1, 3, 224, 224), "float32"),),
        outputs=(TensorContract("output0", (1, 8), "float32"),),
        provider=provider_id,
    )
    provider.load()
    provider.warmup(2)
    rows = []
    for record in records:
        image = cv2.imread(record["runtime_file"], cv2.IMREAD_COLOR)
        if image is None:
            raise ScreeningError(f"failed to read GT smoke source: {record['relative_file']}")
        probabilities = provider.infer({"images": classifier_preprocess(image, record["bbox_xyxy"])})["output0"][0]
        if not np.isfinite(probabilities).all():
            raise ScreeningError("C1 output contains non-finite values")
        index = int(np.argmax(probabilities))
        source_class = CLASS_ORDER[index]
        predicted_product = CLASS_MAPPING.get(source_class, "background")
        rows.append({
            **{key: value for key, value in record.items() if key != "runtime_file"},
            "source_class": source_class,
            "source_confidence": float(probabilities[index]),
            "predicted_product_class": predicted_product,
            "probabilities": {name: float(probabilities[position]) for position, name in enumerate(CLASS_ORDER)},
        })
    return rows, provider.health()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", type=Path, default=ROOT / "config" / "existing_model_candidates_v3.yaml")
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--manifest-sha256", required=True)
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
    registered_sha = next(item["sha256"] for item in candidate["files"] if item["name"] == "wastewise-yolo.onnx")
    if arguments.model_sha256.lower() != registered_sha:
        raise ScreeningError("CLI C1 model SHA does not match frozen registry")
    manifest, records = load_records(
        arguments.manifest,
        expected_manifest_sha256=arguments.manifest_sha256,
        source_prefix=arguments.source_prefix,
        data_root=arguments.data_root,
    )
    rows, health = run(
        records,
        model=arguments.model,
        model_sha256=arguments.model_sha256.lower(),
        provider_id=arguments.provider,
    )
    raw_payload = {
        "schema_version": 1,
        "protocol_id": "EMFJ6V3",
        "model_id": MODEL_ID,
        "model_sha256": arguments.model_sha256.lower(),
        "source_manifest_sha256": arguments.manifest_sha256.lower(),
        "class_order": list(CLASS_ORDER),
        "class_mapping": CLASS_MAPPING,
        "rows": rows,
    }
    arguments.raw_output.parent.mkdir(parents=True, exist_ok=True)
    arguments.raw_output.write_text(json.dumps(raw_payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    metrics = classification_metrics(rows)
    report = {
        "schema_version": 1,
        "protocol_id": "EMFJ6V3",
        "model_id": MODEL_ID,
        "model_sha256": arguments.model_sha256.lower(),
        "provider": health,
        "dataset": {
            "split": manifest["source_split"],
            "records": len(records),
            "class_counts": manifest["qa"]["class_counts"],
            "crop_source": "offline_gt_box_development_only",
            "formal_eligible": False,
            "independent_negative_only_domain": False,
        },
        "metrics": metrics,
        "raw_inference_file": arguments.raw_output.name,
        "raw_inference_sha256": sha256(arguments.raw_output),
        "EMF_EXISTING_MODEL_SCREENING_COMPLETE": False,
        "EMF_EXISTING_MODEL_FUNCTIONAL_CANDIDATE": False,
        "EMF_EXISTING_MODEL_PRODUCT_CANDIDATE": False,
        "training_authorized": False,
        "truth_boundary": (
            "Development-only native ONNX smoke on offline GT-derived crops. It is not a "
            "proposal-crop A4 screen, threshold selection, independent negative-domain "
            "specificity, HOLDOUT, product, release, Journey 6, or training evidence."
        ),
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({
        "model_id": MODEL_ID,
        "records": len(records),
        "macro_f1": metrics["macro_f1"],
        "background_specificity": metrics["background_specificity"],
        "report_sha256": sha256(arguments.output),
        "screening_complete": False,
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
