#!/usr/bin/env python3
"""Offline, safetensors-only C4 native worker for development GT-crop smoke."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import time
from pathlib import Path
from typing import Any


MODEL_ID = "c4_prithiv_trash_net_siglip2"
EXPECTED_TRANSFORMERS = "4.50.2"
EXPECTED_SAFETENSORS = "0.5.3"
CLASS_ORDER = ("cardboard", "glass", "metal", "paper", "plastic", "trash")
PRODUCT_CLASSES = ("background", "plastic_bottle", "metal_can", "paper_litter")
CLASS_MAPPING = {"metal": "metal_can", "paper": "paper_litter", "plastic": "plastic_bottle"}
FORBIDDEN_MARKERS = ("G5_V2", "SEALED_FINAL", "VAL_NEW", "DEV_VAL")


class WorkerError(ValueError):
    """Raised when the offline native worker contract is unsafe."""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def reject_forbidden_path(path: str | Path) -> None:
    text = str(path).replace("/", "\\").upper()
    if any(marker in text for marker in FORBIDDEN_MARKERS):
        raise WorkerError("forbidden validation or sealed path")
    if re.search(r"(?:^|[\\_.-])G5(?:$|[\\_.-])", text):
        raise WorkerError("generic G5 path is forbidden")


def rebase(file_name: str, source_prefix: str, data_root: Path) -> tuple[Path, str]:
    normalized = file_name.replace("\\", "/")
    prefix = source_prefix.replace("\\", "/").rstrip("/")
    if not normalized.lower().startswith(prefix.lower() + "/"):
        raise WorkerError("record path is outside the frozen source prefix")
    relative = normalized[len(prefix) + 1 :]
    actual = (data_root / Path(*relative.split("/"))).resolve()
    root = data_root.resolve()
    if actual != root and root not in actual.parents:
        raise WorkerError("rebased record escapes the development root")
    return actual, relative


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
        "background_specificity": confusion["background"]["background"] / background_support if background_support else None,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--model-sha256", required=True)
    parser.add_argument("--config-sha256", required=True)
    parser.add_argument("--processor-sha256", required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--manifest-sha256", required=True)
    parser.add_argument("--source-prefix", required=True)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    for path in (arguments.model_dir, arguments.manifest, arguments.data_root, arguments.output):
        reject_forbidden_path(path)
    artifacts = {
        "model.safetensors": arguments.model_sha256.lower(),
        "config.json": arguments.config_sha256.lower(),
        "preprocessor_config.json": arguments.processor_sha256.lower(),
    }
    for name, expected in artifacts.items():
        path = arguments.model_dir / name
        if not path.is_file() or sha256(path) != expected:
            raise WorkerError(f"C4 artifact SHA mismatch: {name}")
    if sha256(arguments.manifest) != arguments.manifest_sha256.lower():
        raise WorkerError("C4 GT manifest SHA mismatch")
    manifest = json.loads(arguments.manifest.read_text(encoding="utf-8"))
    if manifest.get("source_split") != "G10_TRAIN":
        raise WorkerError("C4 worker only accepts fixed G10_TRAIN")
    if manifest.get("formal_eligible") is not False or manifest.get("development_only") is not True:
        raise WorkerError("C4 GT crop input must remain informal development-only")
    if manifest.get("production_runtime_gt_used") is not False:
        raise WorkerError("production runtime GT use must remain false")

    import safetensors
    import numpy
    import torch
    import transformers
    from PIL import Image
    from transformers import AutoImageProcessor, AutoModelForImageClassification

    if transformers.__version__ != EXPECTED_TRANSFORMERS:
        raise WorkerError("unexpected Transformers version")
    if safetensors.__version__ != EXPECTED_SAFETENSORS:
        raise WorkerError("unexpected safetensors version")
    processor = AutoImageProcessor.from_pretrained(
        arguments.model_dir,
        local_files_only=True,
        trust_remote_code=False,
        use_fast=False,
    )
    processor_payload = processor.to_dict()
    model = AutoModelForImageClassification.from_pretrained(
        arguments.model_dir,
        local_files_only=True,
        trust_remote_code=False,
        use_safetensors=True,
    )
    id2label = tuple(model.config.id2label[index] for index in range(model.config.num_labels))
    if id2label != CLASS_ORDER:
        raise WorkerError(f"C4 native class order mismatch: {id2label}")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.eval().to(device)
    rows = []
    started = time.perf_counter()
    with torch.inference_mode():
        for item in manifest["records"]:
            if item.get("crop_source") != "offline_gt_box_development_only":
                raise WorkerError("C4 worker requires explicitly marked offline GT crops")
            actual_class = item["class_id"]
            if actual_class not in PRODUCT_CLASSES:
                raise WorkerError("C4 GT class is outside the product classes")
            actual, relative = rebase(item["rgb_path"], arguments.source_prefix, arguments.data_root)
            if not actual.is_file():
                raise WorkerError(f"C4 source image missing: {relative}")
            x1, y1, x2, y2 = (float(value) for value in item["proposal_bbox_native_xyxy"])
            with Image.open(actual) as image:
                rgb = image.convert("RGB")
                width, height = rgb.size
                crop = rgb.crop((
                    max(0, min(width - 1, math.floor(x1))),
                    max(0, min(height - 1, math.floor(y1))),
                    max(1, min(width, math.ceil(x2))),
                    max(1, min(height, math.ceil(y2))),
                ))
                inputs = processor(
                    images=crop,
                    return_tensors="pt",
                    input_data_format="channels_last",
                )
            logits = model(pixel_values=inputs["pixel_values"].to(device)).logits[0]
            probabilities = torch.softmax(logits, dim=0).cpu().tolist()
            index = max(range(len(probabilities)), key=probabilities.__getitem__)
            source_class = CLASS_ORDER[index]
            rows.append({
                "record_id": item["record_id"],
                "relative_file": relative,
                "source_image_sha256": sha256(actual),
                "bbox_xyxy": [x1, y1, x2, y2],
                "actual_product_class": actual_class,
                "source_class": source_class,
                "source_confidence": probabilities[index],
                "predicted_product_class": CLASS_MAPPING.get(source_class, "background"),
                "probabilities": {name: probabilities[position] for position, name in enumerate(CLASS_ORDER)},
            })
    elapsed = time.perf_counter() - started
    metrics = classification_metrics(rows)
    report = {
        "schema_version": 1,
        "protocol_id": "EMFJ6V3",
        "model_id": MODEL_ID,
        "runtime": {
            "torch": torch.__version__,
            "transformers": transformers.__version__,
            "safetensors": safetensors.__version__,
            "numpy": numpy.__version__,
            "device": str(device),
            "cuda_available": torch.cuda.is_available(),
            "network_required": False,
            "trust_remote_code": False,
            "weight_format": "safetensors",
            "records": len(rows),
            "elapsed_seconds": elapsed,
        },
        "processor_contract": {
            "processor_class": processor.__class__.__name__,
            "use_fast": False,
            "forced_rgb": True,
            "input_data_format": "channels_last",
            "size": processor_payload.get("size"),
            "resample": processor_payload.get("resample"),
            "do_rescale": processor_payload.get("do_rescale"),
            "rescale_factor": processor_payload.get("rescale_factor"),
            "do_normalize": processor_payload.get("do_normalize"),
            "image_mean": processor_payload.get("image_mean"),
            "image_std": processor_payload.get("image_std"),
        },
        "artifacts": artifacts,
        "source_manifest_sha256": arguments.manifest_sha256.lower(),
        "class_order": list(CLASS_ORDER),
        "class_mapping": CLASS_MAPPING,
        "metrics": metrics,
        "rows": rows,
        "EMF_EXISTING_MODEL_SCREENING_COMPLETE": False,
        "EMF_EXISTING_MODEL_FUNCTIONAL_CANDIDATE": False,
        "EMF_EXISTING_MODEL_PRODUCT_CANDIDATE": False,
        "training_authorized": False,
        "truth_boundary": (
            "Development-only native safetensors smoke on offline GT-derived crops. "
            "It is not proposal-crop A4, independent negative-domain, HOLDOUT, ONNX parity, "
            "product, release, Journey 6, or training evidence."
        ),
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({
        "model_id": MODEL_ID,
        "records": len(rows),
        "macro_f1": metrics["macro_f1"],
        "background_specificity": metrics["background_specificity"],
        "output_sha256": sha256(arguments.output),
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
