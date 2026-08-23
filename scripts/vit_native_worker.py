#!/usr/bin/env python3
"""Offline native TRAIN smoke worker for the frozen EMF C5/C6 ViT models."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import time
from collections import Counter
from pathlib import Path, PurePosixPath
from typing import Any

EXPECTED_RUNTIME_IMAGE = "tzcup/tgarv9-grounding-dino:v3.3.0"
EXPECTED_RUNTIME_IMAGE_DIGEST = (
    "sha256:bf61e2b6bca3b1fc6a66100986b59b2eec8aef91a24079b2560bd365622ecf86"
)
EXPECTED_RUNTIME_VERSIONS = {
    "torch": "2.5.1+cu124",
    "transformers": "4.44.2",
    "safetensors": "0.8.0",
    "numpy": "1.26.4",
    "pillow": "10.2.0",
}

EXPECTED_MANIFEST_SHA256 = (
    "2f226ed4925779348e218c49c16d0b33ba719958289e493948fd5427e94e3166"
)
EXPECTED_IDENTITY_LOCK_SHA256 = (
    "998f5666410ef5893cbbddbe22e8831fd70eb548559cda9739ba84086c8d37f8"
)
EXPECTED_RECORD_COUNT = 183
EXPECTED_CLASS_COUNTS = {
    "background": 102,
    "plastic_bottle": 14,
    "metal_can": 47,
    "paper_litter": 20,
}
PRODUCT_CLASSES = tuple(EXPECTED_CLASS_COUNTS)

MODEL_CONTRACTS: dict[str, dict[str, Any]] = {
    "c5_dima806_garbage_types_vit": {
        "source_uri": "https://huggingface.co/dima806/garbage_types_image_detection",
        "revision": "766892e572f51ff457be945d0e80654e1c7c874d",
        "artifacts": {
            "model.safetensors": "b9d9dec6fe465c468dd9f4cd986e633c5ad770d2a5603c22a74cfeaf6c8327b4",
            "config.json": "ce5640ed05a3c574ba08574f263e2381588af7ba31533ab9a0179941952a6323",
            "preprocessor_config.json": "a34861a24e781942424ad790b82bc99348404a2e64ea882de88c40905851698d",
            "README.md": "c662e53e8cbf24c4a7c5ba70978f53c8f88567252496917ee1996f75d446e655",
            "transformers_v4.44.0_image_processing_vit.py": "3ef6438c91099182c145e0286cc238d3b78d1a0de51a3203a6a343480d545646",
        },
        "class_order": (
            "battery",
            "biological",
            "brown-glass",
            "cardboard",
            "clothes",
            "green-glass",
            "metal",
            "paper",
            "plastic",
            "shoes",
            "trash",
            "white-glass",
        ),
        "target_sources": {
            "metal_can": ("metal",),
            "paper_litter": ("paper",),
            "plastic_bottle": ("plastic",),
        },
        "processor_class": "ViTImageProcessor",
        "source_transformers_version": "4.44.0",
        "license": "apache-2.0",
        "training_data_boundary": (
            "linked Kaggle notebook; exact dataset identity and license unresolved"
        ),
    },
    "c6_giecom_recycling_vit": {
        "source_uri": (
            "https://huggingface.co/Giecom/giecom-vit-model-clasification-waste"
        ),
        "revision": "49101a014c16be969b2c9210011681745449c63b",
        "artifacts": {
            "model.safetensors": "ccba6310d281d99efeac188c4c76bcafa48488f02d4c554a86f1d594b480e4ef",
            "config.json": "bd200ceae157a434a8a25fdd839160f2a56db65b9e9c6ecec37a388dc4532f52",
            "preprocessor_config.json": "b09d2030f83f2a59d12c717d41a9135a3f0c1ba0a2a5df694dbc40f77735daed",
            "README.md": "fa35a8f89fb36ecde46a7449bde7f70c7ca1459caffba07c1872b36c42095c54",
            "transformers_v4.35.0_image_processing_vit.py": "b6caaf89e0ef0c28317f30053b26c389f567b6147a6c49bad847010f72326632",
        },
        "class_order": (
            "aluminium",
            "batteries",
            "cardboard",
            "disposable plates",
            "glass",
            "hard plastic",
            "paper",
            "paper towel",
            "polystyrene",
            "soft plastics",
            "takeaway cups",
        ),
        "target_sources": {
            "metal_can": ("aluminium",),
            "paper_litter": ("paper",),
            "plastic_bottle": ("hard plastic", "soft plastics"),
        },
        "processor_class": "ViTFeatureExtractor",
        "source_transformers_version": "4.35.0",
        "license": "apache-2.0",
        "training_data_boundary": (
            "viola77data/recycling-dataset named; dataset card and license not frozen"
        ),
    },
}

FORBIDDEN_PATTERN = re.compile(
    r"(?:^|[^A-Z0-9])(?:G5(?:_V2)?|VAL_NEW|DEV_VAL|SEALED)(?:$|[^A-Z0-9])",
    re.IGNORECASE,
)


class WorkerError(ValueError):
    """Raised when a fixed model, data, or isolation contract is violated."""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def reject_forbidden(value: object, *, field: str) -> None:
    text = str(value)
    normalized_words = re.sub(r"[^A-Z0-9]+", "_", text.upper()).split("_")
    normalized = "_".join(word for word in normalized_words if word)
    if FORBIDDEN_PATTERN.search(text) or any(
        word == "G5" or word.startswith("G5V2") for word in normalized_words
    ) or "VAL_NEW" in normalized or "DEV_VAL" in normalized:
        raise WorkerError(f"forbidden validation or sealed marker in {field}")


def _iter_strings(value: object):
    if isinstance(value, str):
        yield value
    elif isinstance(value, dict):
        for key, item in value.items():
            yield str(key)
            yield from _iter_strings(item)
    elif isinstance(value, list):
        for item in value:
            yield from _iter_strings(item)


def load_json(path: Path, *, expected_sha256: str, field: str) -> dict[str, Any]:
    reject_forbidden(path.resolve(), field=f"{field} path")
    if not path.is_file() or sha256(path) != expected_sha256:
        raise WorkerError(f"fixed {field} SHA-256 mismatch")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise WorkerError(f"cannot read fixed {field}: {exc}") from exc
    if not isinstance(payload, dict):
        raise WorkerError(f"fixed {field} root must be an object")
    for value in _iter_strings(payload):
        reject_forbidden(value, field=f"{field} payload")
    return payload


def verify_artifacts(model_dir: Path, contract: dict[str, Any]) -> None:
    reject_forbidden(model_dir.resolve(), field="model directory")
    for name, expected in contract["artifacts"].items():
        artifact = model_dir / name
        if not artifact.is_file() or sha256(artifact) != expected:
            raise WorkerError(f"fixed model artifact SHA-256 mismatch: {name}")
    unsafe_weights = [
        path.name
        for path in model_dir.iterdir()
        if path.is_file() and path.suffix.lower() in {".bin", ".pt", ".pth", ".pkl"}
    ]
    if unsafe_weights:
        raise WorkerError(f"non-safetensors weight files are forbidden: {unsafe_weights}")
    config = json.loads((model_dir / "config.json").read_text(encoding="utf-8"))
    if config.get("architectures") != ["ViTForImageClassification"]:
        raise WorkerError("model architecture is outside the fixed ViT allowlist")
    if config.get("auto_map") is not None:
        raise WorkerError("remote/custom model code is forbidden")
    class_order = tuple(
        config.get("id2label", {}).get(str(index))
        for index in range(len(contract["class_order"]))
    )
    if class_order != contract["class_order"]:
        raise WorkerError("config class order does not match the fixed contract")


def rebase_source_path(file_name: object, source_prefix: str, data_root: Path) -> tuple[Path, str]:
    if not isinstance(file_name, str) or not file_name:
        raise WorkerError("record rgb_path must be a non-empty string")
    reject_forbidden(file_name, field="record rgb_path")
    normalized = file_name.replace("\\", "/")
    prefix = source_prefix.replace("\\", "/").rstrip("/")
    if not normalized.casefold().startswith(prefix.casefold() + "/"):
        raise WorkerError("record path is outside the fixed source prefix")
    relative = normalized[len(prefix) + 1 :]
    root = data_root.resolve()
    actual = (root / Path(*relative.split("/"))).resolve()
    if actual != root and root not in actual.parents:
        raise WorkerError("rebased record escapes the fixed development root")
    reject_forbidden(actual, field="rebased RGB path")
    return actual, relative


def validate_dataset_payloads(
    manifest: dict[str, Any],
    identity_lock: dict[str, Any],
    *,
    expected_count: int = EXPECTED_RECORD_COUNT,
    expected_class_counts: dict[str, int] = EXPECTED_CLASS_COUNTS,
) -> dict[str, dict[str, Any]]:
    expected_manifest_fields = {
        "schema_version": 1,
        "protocol": "CLOSE-RANGE-RGBD-RECOVERY-V12-DEV-ONLY",
        "stage": "CRRGBDV12-DEV-ONLY-GT-CROPS",
        "source_split": "G10_TRAIN",
        "development_only": True,
        "formal_eligible": False,
        "production_runtime_gt_used": False,
        "pass": True,
    }
    for key, expected in expected_manifest_fields.items():
        if manifest.get(key) != expected:
            raise WorkerError(f"fixed TRAIN manifest field mismatch: {key}")
    if identity_lock.get("protocol_id") != "EMFJ6V3":
        raise WorkerError("identity lock protocol mismatch")
    if identity_lock.get("model_id") != "c4_prithiv_trash_net_siglip2":
        raise WorkerError("identity lock must be the frozen C4 native raw evidence")
    if identity_lock.get("source_manifest_sha256") != EXPECTED_MANIFEST_SHA256:
        raise WorkerError("identity lock source manifest mismatch")

    records = manifest.get("records")
    lock_rows = identity_lock.get("rows")
    if not isinstance(records, list) or len(records) != expected_count:
        raise WorkerError("fixed TRAIN manifest record count mismatch")
    if not isinstance(lock_rows, list) or len(lock_rows) != expected_count:
        raise WorkerError("fixed identity lock row count mismatch")
    by_id: dict[str, dict[str, Any]] = {}
    for row in lock_rows:
        if not isinstance(row, dict):
            raise WorkerError("identity lock row must be an object")
        record_id = row.get("record_id")
        if not isinstance(record_id, str) or record_id in by_id:
            raise WorkerError("identity lock record IDs must be unique strings")
        by_id[record_id] = row

    manifest_ids: set[str] = set()
    class_counts: Counter[str] = Counter()
    for record in records:
        if not isinstance(record, dict):
            raise WorkerError("manifest record must be an object")
        record_id = record.get("record_id")
        if not isinstance(record_id, str) or record_id in manifest_ids:
            raise WorkerError("manifest record IDs must be unique strings")
        manifest_ids.add(record_id)
        if not record_id.startswith("G10_TRAIN:"):
            raise WorkerError("only G10_TRAIN records are allowed")
        if record.get("source_split") != "G10_TRAIN":
            raise WorkerError("record source split is not G10_TRAIN")
        if record.get("crop_source") != "offline_gt_box_development_only":
            raise WorkerError("only development-only offline GT crops are allowed")
        if record.get("gt_role") != "offline_training_label_only":
            raise WorkerError("record GT role is outside the fixed development contract")
        if record.get("production_runtime_gt_used") is not False:
            raise WorkerError("production runtime GT use must remain false")
        actual = record.get("class_id")
        if actual not in PRODUCT_CLASSES:
            raise WorkerError("record class is outside the fixed product classes")
        class_counts[actual] += 1
        locked = by_id.get(record_id)
        if locked is None:
            raise WorkerError("manifest record is absent from the identity lock")
        if locked.get("actual_product_class") != actual:
            raise WorkerError("manifest and identity-lock classes differ")
        manifest_bbox = tuple(float(value) for value in record.get("proposal_bbox_native_xyxy", ()))
        lock_bbox = tuple(float(value) for value in locked.get("bbox_xyxy", ()))
        if len(manifest_bbox) != 4 or manifest_bbox != lock_bbox:
            raise WorkerError("manifest and identity-lock bboxes differ")
        locked_sha = locked.get("source_image_sha256")
        if not isinstance(locked_sha, str) or not re.fullmatch(r"[0-9a-f]{64}", locked_sha):
            raise WorkerError("identity lock contains an invalid image SHA-256")
    if manifest_ids != set(by_id):
        raise WorkerError("manifest and identity-lock record ID sets differ")
    if dict(class_counts) != expected_class_counts:
        raise WorkerError("fixed TRAIN manifest class counts differ")
    return by_id


def parse_mounts(text: str) -> list[tuple[PurePosixPath, set[str]]]:
    mounts = []
    for line in text.splitlines():
        fields = line.split()
        if len(fields) >= 4:
            mounts.append(
                (
                    PurePosixPath(fields[1].replace("\\040", " ")),
                    set(fields[3].split(",")),
                )
            )
    return mounts


def mount_is_read_only(
    path: Path | PurePosixPath,
    mounts: list[tuple[PurePosixPath, set[str]]],
) -> bool:
    resolved = PurePosixPath(path.as_posix())
    candidates = []
    for mount_point, options in mounts:
        try:
            resolved.relative_to(mount_point)
        except ValueError:
            continue
        candidates.append((len(mount_point.parts), options))
    return bool(candidates) and "ro" in max(candidates, key=lambda item: item[0])[1]


def validate_runtime_isolation(read_only_paths: list[Path]) -> dict[str, Any]:
    if os.name != "posix" or not hasattr(os, "geteuid") or os.geteuid() == 0:
        raise WorkerError("native worker must run as a non-root POSIX user")
    if os.environ.get("TRANSFORMERS_OFFLINE") != "1" or os.environ.get("HF_HUB_OFFLINE") != "1":
        raise WorkerError("Transformers and Hugging Face offline modes must both be enabled")
    interfaces = sorted(path.name for path in Path("/sys/class/net").iterdir())
    if interfaces != ["lo"]:
        raise WorkerError("container network namespace is not isolated to loopback")
    mounts = parse_mounts(Path("/proc/mounts").read_text(encoding="utf-8"))
    if not mount_is_read_only(Path("/"), mounts):
        raise WorkerError("container root filesystem must be read-only")
    for path in read_only_paths:
        if not mount_is_read_only(path, mounts):
            raise WorkerError(f"required read-only mount is writable: {path}")
    return {
        "uid": os.geteuid(),
        "non_root": True,
        "network_interfaces": interfaces,
        "network_mode": "none",
        "root_filesystem_read_only": True,
        "input_mounts_read_only": True,
        "transformers_offline": True,
        "hf_hub_offline": True,
    }


def mapped_target_probabilities(
    probabilities: dict[str, float], contract: dict[str, Any]
) -> dict[str, float]:
    return {
        target: sum(probabilities[source] for source in sources)
        for target, sources in contract["target_sources"].items()
    }


def classification_metrics(rows: list[dict[str, Any]]) -> dict[str, Any]:
    confusion = {
        actual: {predicted: 0 for predicted in PRODUCT_CLASSES}
        for actual in PRODUCT_CLASSES
    }
    for row in rows:
        confusion[row["actual_product_class"]][row["predicted_product_class"]] += 1
    per_class = {}
    for class_name in PRODUCT_CLASSES:
        tp = confusion[class_name][class_name]
        fp = sum(confusion[actual][class_name] for actual in PRODUCT_CLASSES if actual != class_name)
        fn = sum(confusion[class_name][choice] for choice in PRODUCT_CLASSES if choice != class_name)
        precision = tp / max(tp + fp, 1)
        recall = tp / max(tp + fn, 1)
        f1 = 2 * precision * recall / max(precision + recall, 1e-12)
        per_class[class_name] = {
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "support": sum(confusion[class_name].values()),
        }
    return {
        "confusion": confusion,
        "per_class": per_class,
        "macro_f1": sum(per_class[name]["f1"] for name in PRODUCT_CLASSES)
        / len(PRODUCT_CLASSES),
        "target_macro_f1": sum(per_class[name]["f1"] for name in PRODUCT_CLASSES[1:])
        / len(PRODUCT_CLASSES[1:]),
        "background_specificity": per_class["background"]["recall"],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", required=True, choices=sorted(MODEL_CONTRACTS))
    parser.add_argument("--model-dir", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--identity-lock", required=True, type=Path)
    parser.add_argument("--source-prefix", required=True)
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--runtime-image-digest", required=True)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    contract = MODEL_CONTRACTS[args.candidate]
    for value, field in (
        (args.model_dir, "model directory"),
        (args.manifest, "manifest path"),
        (args.identity_lock, "identity lock path"),
        (args.data_root, "data root"),
        (args.output, "output path"),
        (args.source_prefix, "source prefix"),
    ):
        reject_forbidden(value, field=field)
    if args.runtime_image_digest != EXPECTED_RUNTIME_IMAGE_DIGEST:
        raise WorkerError("runtime image digest is outside the fixed allowlist")

    model_dir = args.model_dir.resolve(strict=True)
    manifest_path = args.manifest.resolve(strict=True)
    identity_path = args.identity_lock.resolve(strict=True)
    data_root = args.data_root.resolve(strict=True)
    isolation = validate_runtime_isolation(
        [model_dir, manifest_path, identity_path, data_root]
    )
    verify_artifacts(model_dir, contract)
    manifest = load_json(
        manifest_path,
        expected_sha256=EXPECTED_MANIFEST_SHA256,
        field="TRAIN manifest",
    )
    identity_lock = load_json(
        identity_path,
        expected_sha256=EXPECTED_IDENTITY_LOCK_SHA256,
        field="identity lock",
    )
    identities = validate_dataset_payloads(manifest, identity_lock)

    import numpy
    import safetensors
    import torch
    import transformers
    from PIL import Image
    from PIL import __version__ as pillow_version
    from transformers import AutoImageProcessor, AutoModelForImageClassification

    versions = {
        "torch": torch.__version__,
        "transformers": transformers.__version__,
        "safetensors": safetensors.__version__,
        "numpy": numpy.__version__,
        "pillow": pillow_version,
    }
    if versions != EXPECTED_RUNTIME_VERSIONS:
        raise WorkerError(f"runtime package versions differ from the fixed image: {versions}")
    if not torch.cuda.is_available():
        raise WorkerError("fixed native smoke requires the CUDA runtime")

    processor = AutoImageProcessor.from_pretrained(
        model_dir,
        local_files_only=True,
        trust_remote_code=False,
        use_fast=False,
    )
    model = AutoModelForImageClassification.from_pretrained(
        model_dir,
        local_files_only=True,
        trust_remote_code=False,
        use_safetensors=True,
    )
    processor_payload = processor.to_dict()
    if processor.__class__.__name__ != contract["processor_class"]:
        raise WorkerError("native processor class differs from the fixed contract")
    expected_processor = {
        "size": {"height": 224, "width": 224},
        "resample": 2,
        "do_resize": True,
        "do_rescale": True,
        "rescale_factor": 1.0 / 255.0,
        "do_normalize": True,
        "image_mean": [0.5, 0.5, 0.5],
        "image_std": [0.5, 0.5, 0.5],
    }
    for key, expected in expected_processor.items():
        if processor_payload.get(key) != expected:
            raise WorkerError(f"native processor field differs: {key}")
    class_order = tuple(model.config.id2label[index] for index in range(model.config.num_labels))
    if class_order != contract["class_order"]:
        raise WorkerError("loaded model class order differs from the fixed contract")

    device = torch.device("cuda:0")
    model.eval().to(device)
    rows = []
    started = time.perf_counter()
    with torch.inference_mode():
        for record in manifest["records"]:
            locked = identities[record["record_id"]]
            actual, relative = rebase_source_path(
                record["rgb_path"], args.source_prefix, data_root
            )
            if not actual.is_file() or sha256(actual) != locked["source_image_sha256"]:
                raise WorkerError(f"fixed source image SHA-256 mismatch: {record['record_id']}")
            if relative.replace("\\", "/") != locked["relative_file"].replace("\\", "/"):
                raise WorkerError("rebased source path differs from the identity lock")
            x1, y1, x2, y2 = (
                float(value) for value in record["proposal_bbox_native_xyxy"]
            )
            with Image.open(actual) as source_image:
                rgb = source_image.convert("RGB")
                width, height = rgb.size
                crop = rgb.crop(
                    (
                        max(0, min(width - 1, math.floor(x1))),
                        max(0, min(height - 1, math.floor(y1))),
                        max(1, min(width, math.ceil(x2))),
                        max(1, min(height, math.ceil(y2))),
                    )
                )
                inputs = processor(
                    images=crop,
                    return_tensors="pt",
                    input_data_format="channels_last",
                )
            logits = model(pixel_values=inputs["pixel_values"].to(device)).logits[0]
            values = torch.softmax(logits, dim=0).cpu().tolist()
            probabilities = {
                source_class: float(values[index])
                for index, source_class in enumerate(class_order)
            }
            target_probabilities = mapped_target_probabilities(probabilities, contract)
            predicted_product = max(
                PRODUCT_CLASSES,
                key=lambda name: (
                    1.0 - sum(target_probabilities.values())
                    if name == "background"
                    else target_probabilities[name],
                    name,
                ),
            )
            source_index = max(range(len(values)), key=values.__getitem__)
            rows.append(
                {
                    "record_id": record["record_id"],
                    "relative_file": relative.replace("\\", "/"),
                    "source_image_sha256": locked["source_image_sha256"],
                    "bbox_xyxy": [x1, y1, x2, y2],
                    "actual_product_class": record["class_id"],
                    "source_class": class_order[source_index],
                    "source_confidence": float(values[source_index]),
                    "predicted_product_class": predicted_product,
                    "target_probabilities": target_probabilities,
                    "unknown_probability": 1.0 - sum(target_probabilities.values()),
                    "probabilities": probabilities,
                }
            )
    torch.cuda.synchronize(device)
    elapsed = time.perf_counter() - started
    metrics = classification_metrics(rows)
    source_to_target = {
        source: target
        for target, sources in contract["target_sources"].items()
        for source in sources
    }
    report = {
        "schema_version": 1,
        "protocol_id": "EMFJ6V3",
        "stage": "A4_CLASSIFIER_GT_TRAIN_NATIVE_SMOKE",
        "model_id": args.candidate,
        "source_uri": contract["source_uri"],
        "revision": contract["revision"],
        "license": contract["license"],
        "training_data_boundary": contract["training_data_boundary"],
        "model_contract": {
            "architecture": "ViTForImageClassification",
            "source_transformers_version": contract["source_transformers_version"],
            "native_logits_shape": [1, len(class_order)],
        },
        "runtime": {
            "image": EXPECTED_RUNTIME_IMAGE,
            "image_digest": EXPECTED_RUNTIME_IMAGE_DIGEST,
            **versions,
            "device": str(device),
            "cuda_available": True,
            "records": len(rows),
            "elapsed_seconds": elapsed,
            **isolation,
            "trust_remote_code": False,
            "local_files_only": True,
            "weight_format": "safetensors",
        },
        "artifacts": contract["artifacts"],
        "source_manifest_sha256": EXPECTED_MANIFEST_SHA256,
        "identity_lock_sha256": EXPECTED_IDENTITY_LOCK_SHA256,
        "dataset": {
            "source_split": "G10_TRAIN",
            "development_only": True,
            "formal_eligible": False,
            "production_runtime_gt_used": False,
            "crop_source": "offline_gt_box_development_only",
            "record_count": len(rows),
            "class_counts": EXPECTED_CLASS_COUNTS,
            "all_record_image_bbox_identities_verified": True,
            "independent_negative_only_domain": False,
        },
        "processor_contract": {
            "processor_class": processor.__class__.__name__,
            "use_fast": False,
            "forced_rgb": True,
            "input_data_format": "channels_last",
            **expected_processor,
        },
        "class_order": list(class_order),
        "class_mapping": source_to_target,
        "target_sources": {
            target: list(sources) for target, sources in contract["target_sources"].items()
        },
        "prediction_contract": {
            "target_probability": "sum of every explicitly mapped native source class",
            "unknown_probability": "one minus the sum of mapped target probabilities",
            "predicted_product_class": (
                "argmax across the three mapped target probabilities and aggregate unknown "
                "as background"
            ),
            "threshold_selected": False,
        },
        "metrics": metrics,
        "rows": rows,
        "EMF_EXISTING_MODEL_SCREENING_COMPLETE": False,
        "EMF_NONTRAINING_ADJUSTMENT_COMPLETE": False,
        "EMF_EXISTING_MODEL_FUNCTIONAL_CANDIDATE": False,
        "EMF_EXISTING_MODEL_PRODUCT_CANDIDATE": False,
        "training_authorized": False,
        "truth_boundary": (
            "Development-only native safetensors smoke on fixed offline GT-derived TRAIN crops. "
            "It is not proposal-crop A4, independent negative-domain, HOLDOUT threshold "
            "selection, ONNX parity, product, release, Journey 6, or training evidence."
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "model_id": args.candidate,
                "records": len(rows),
                "macro_f1": metrics["macro_f1"],
                "target_macro_f1": metrics["target_macro_f1"],
                "background_specificity": metrics["background_specificity"],
                "elapsed_seconds": elapsed,
                "output_sha256": sha256(args.output),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
