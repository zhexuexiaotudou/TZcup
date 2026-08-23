#!/usr/bin/env python3
"""Isolated C1 ONNX inference on the fixed offline-GT HOLDOUT crop bank.

This worker reports native probabilities only.  It never trains, selects or
freezes a threshold, and its output is not functional, product, release, or
Journey 6 evidence.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
import re
import sys
import tempfile
import time
from collections import Counter
from pathlib import Path, PurePosixPath
from typing import Any, Sequence

MODEL_ID = "c1_wastewise_yolov8n_cls"
MODEL_SHA256 = "2b46d491091dbc0ed98a0f1eaee7fe5739c8fd3eb5bd5935396c3b2712e1f7a6"
RUNTIME_IMAGE_DIGEST = "sha256:f0ab59405bde11477999779a000d766a660bc5dfb4214d23436713d08271cb2d"
RUNTIME_VERSIONS = {
    "python": "3.12.3",
    "onnxruntime": "1.20.2",
    "opencv": "4.6.0",
    "numpy": "1.26.4",
}
DOMAIN_MANIFEST_SHA256 = "3bdb3006226943e4149cd84144b488e5eb112ab35ad3692c5da8cc48c88b5208"
HOLDOUT_WORLDS = {
    "g10v15_val_w01_07_service_road",
    "g10v15_val_w02_08_mixed_curb_vegetation",
    "g10v15_val_w03_09_light_paver_pedestrian",
}
CLASS_ORDER = (
    "battery",
    "biological",
    "cardboard",
    "glass",
    "metal",
    "paper",
    "plastic",
    "trash",
)
CLASS_MAPPING = {
    "metal": "metal_can",
    "paper": "paper_litter",
    "plastic": "plastic_bottle",
}
HOLDOUT_CLASSES = (
    "background_or_unknown",
    "plastic_bottle",
    "metal_can",
    "paper_litter",
)
PRODUCT_CLASSES = ("background", "plastic_bottle", "metal_can", "paper_litter")
SOURCE_HASH_KEYS = {
    "rgb",
    "depth",
    "camera",
    "semantic",
    "instance",
    "scene_manifest",
    "capture_report",
}
SHA256_RE = re.compile(r"[0-9a-f]{64}")
SCENE_RE = re.compile(r"scene_[A-Za-z0-9][A-Za-z0-9._-]*")
FORBIDDEN_MARKERS = ("G5", "G5_V2", "G5V2", "VAL_NEW", "DEV_VAL", "SEALED")


class WorkerError(ValueError):
    """Raised when an artifact, input, isolation, or inference contract fails."""


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: object) -> str:
    encoded = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return sha256_bytes(encoded)


def exact_int(value: object, field: str, minimum: int | None = None) -> int:
    if type(value) is not int:
        raise WorkerError(f"{field} must be an integer")
    if minimum is not None and value < minimum:
        raise WorkerError(f"{field} must be >= {minimum}")
    return value


def exact_bool(value: object, field: str) -> bool:
    if type(value) is not bool:
        raise WorkerError(f"{field} must be a boolean")
    return value


def exact_string(value: object, field: str) -> str:
    if not isinstance(value, str) or not value:
        raise WorkerError(f"{field} must be a non-empty string")
    return value


def exact_sha256(value: object, field: str) -> str:
    rendered = exact_string(value, field)
    if SHA256_RE.fullmatch(rendered) is None:
        raise WorkerError(f"{field} must be a lowercase SHA-256")
    return rendered


def reject_forbidden(value: object, field: str = "input") -> None:
    """Reject forbidden markers in paths and serialized string values, not keys."""

    if isinstance(value, str) or isinstance(value, Path):
        rendered = str(value)
        normalized = re.sub(r"[^A-Z0-9]+", "_", rendered.upper()).split("_")
        if any(word == "G5" or word.startswith("G5V2") for word in normalized):
            raise WorkerError(f"forbidden marker in {field}")
        upper = rendered.upper()
        for marker in FORBIDDEN_MARKERS[3:]:
            if re.search(
                rf"(?:^|[^A-Z0-9]){re.escape(marker)}(?:$|[^A-Z0-9])", upper
            ):
                raise WorkerError(f"forbidden marker in {field}")
    elif isinstance(value, dict):
        for key, child in value.items():
            reject_forbidden(child, f"{field}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            reject_forbidden(child, f"{field}[{index}]")


def load_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=lambda value: (_ for _ in ()).throw(
                ValueError(f"non-finite JSON constant {value}")
            ),
        )
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        raise WorkerError(f"cannot load JSON {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise WorkerError("manifest root must be an object")
    return payload


def canonical_relative_path(value: object, field: str) -> PurePosixPath:
    rendered = exact_string(value, field)
    if "\\" in rendered or re.match(r"^[A-Za-z]:", rendered) or rendered.startswith("//"):
        raise WorkerError(f"{field} must be a root-relative POSIX path")
    relative = PurePosixPath(rendered)
    if relative.is_absolute() or any(part in ("", ".", "..") for part in relative.parts):
        raise WorkerError(f"{field} must be a root-relative POSIX path")
    if relative.as_posix() != rendered:
        raise WorkerError(f"{field} must be a canonical POSIX path")
    reject_forbidden(rendered, field)
    return relative


def require_contained_file(root: Path, relative: PurePosixPath, field: str) -> Path:
    resolved_root = root.resolve()
    actual = resolved_root.joinpath(*relative.parts).resolve()
    try:
        actual.relative_to(resolved_root)
    except ValueError as exc:
        raise WorkerError(f"{field} escapes crop root") from exc
    if not actual.is_file():
        raise WorkerError(f"missing {field}: {relative.as_posix()}")
    return actual


def validate_manifest(payload: dict[str, Any]) -> list[dict[str, Any]]:
    reject_forbidden(payload, "manifest")
    if payload.get("schema_version") != "emfj6v3.classifier_holdout_gt.v1":
        raise WorkerError("unexpected HOLDOUT manifest schema")
    if payload.get("protocol_id") != "EMFJ6V3":
        raise WorkerError("unexpected protocol_id")
    if payload.get("stage") != "CLASSIFIER_HOLDOUT_OFFLINE_GT_DEVELOPMENT":
        raise WorkerError("unexpected HOLDOUT stage")
    if payload.get("source_split") != "G10_HOLDOUT":
        raise WorkerError("C1 worker only accepts G10_HOLDOUT")
    if payload.get("g10_domain_manifest_sha256") != DOMAIN_MANIFEST_SHA256:
        raise WorkerError("HOLDOUT domain manifest SHA-256 changed")
    if payload.get("holdout_world_ids") != sorted(HOLDOUT_WORLDS):
        raise WorkerError("HOLDOUT world allowlist changed")
    for field, expected in (
        ("offline_gt_development_only", True),
        ("production_runtime_gt_forbidden", True),
        ("training_performed", False),
        ("threshold_selected", False),
        ("threshold_frozen", False),
        ("formal_product_evidence", False),
        ("pass", True),
    ):
        if exact_bool(payload.get(field), field) is not expected:
            raise WorkerError(f"unsafe manifest flag: {field}")
    exact_sha256(payload.get("input_coco_sha256"), "input_coco_sha256")
    exact_sha256(
        payload.get("all_validated_source_frames_sha256"),
        "all_validated_source_frames_sha256",
    )
    manifest_hash = exact_sha256(
        payload.get("canonical_manifest_sha256"), "canonical_manifest_sha256"
    )
    unsigned = dict(payload)
    unsigned.pop("canonical_manifest_sha256")
    if canonical_sha256(unsigned) != manifest_hash:
        raise WorkerError("canonical manifest SHA-256 mismatch")
    selection = payload.get("selection_contract")
    if selection != {
        "seed": 20260824,
        "positive_per_class": 60,
        "background_per_negative_frame": 1,
        "positive_crop_scale": 4.0,
        "minimum_crop_side": 64,
        "background_crop_side": 96,
        "background_max_gt_iou_exclusive": 0.1,
    }:
        raise WorkerError("HOLDOUT selection contract changed")
    atomic = payload.get("atomic_output_contract")
    if atomic != {
        "visibility": "same_filesystem_directory_rename",
        "concurrent_writer_policy": "exclusive_sibling_lock_required",
        "power_loss_durability_guaranteed": False,
    }:
        raise WorkerError("HOLDOUT atomic output contract changed")
    counts = payload.get("counts")
    if not isinstance(counts, dict) or set(counts) != set(HOLDOUT_CLASSES):
        raise WorkerError("HOLDOUT counts must contain exactly four classes")
    normalized_counts = {
        name: exact_int(counts[name], f"counts.{name}", minimum=1)
        for name in HOLDOUT_CLASSES
    }
    for name in HOLDOUT_CLASSES[1:]:
        if normalized_counts[name] != 60:
            raise WorkerError("every positive HOLDOUT class must contain exactly 60 crops")
    negative_frames = exact_int(
        payload.get("negative_only_frame_count"), "negative_only_frame_count", minimum=1
    )
    negative_scenes = exact_int(
        payload.get("negative_only_scene_count"), "negative_only_scene_count", minimum=1
    )
    if negative_scenes > negative_frames:
        raise WorkerError("negative scene/frame counts are inconsistent")
    if normalized_counts["background_or_unknown"] != negative_frames:
        raise WorkerError("background count does not equal fixed one-per-negative-frame contract")
    records = payload.get("records")
    if not isinstance(records, list) or len(records) != sum(normalized_counts.values()):
        raise WorkerError("record count does not match class counts")
    if canonical_sha256(records) != exact_sha256(
        payload.get("identity_lock_sha256"), "identity_lock_sha256"
    ):
        raise WorkerError("record identity lock SHA-256 mismatch")
    observed_counts: Counter[str] = Counter()
    observed_worlds: set[str] = set()
    seen_ids: set[str] = set()
    validated: list[dict[str, Any]] = []
    for index, record in enumerate(records):
        if not isinstance(record, dict):
            raise WorkerError("every HOLDOUT record must be an object")
        record_id = exact_string(record.get("record_id"), f"records[{index}].record_id")
        if not re.fullmatch(r"emf-holdout-[0-9a-f]{24}", record_id):
            raise WorkerError("invalid HOLDOUT record_id")
        if record_id in seen_ids:
            raise WorkerError("duplicate HOLDOUT record_id")
        seen_ids.add(record_id)
        class_name = record.get("class_name")
        if class_name not in HOLDOUT_CLASSES:
            raise WorkerError("record class is outside the fixed four classes")
        observed_counts[class_name] += 1
        if record.get("offline_gt_development_only") is not True:
            raise WorkerError("record must remain offline-GT development-only")
        if record.get("production_runtime_eligible") is not False:
            raise WorkerError("record must remain production-runtime ineligible")
        crop_relative = canonical_relative_path(record.get("crop_path"), "crop_path")
        expected_crop = PurePosixPath("crops", class_name, f"{record_id}.png")
        if crop_relative != expected_crop:
            raise WorkerError("crop path does not match record identity/class")
        crop_hash = exact_sha256(record.get("crop_sha256"), "crop_sha256")
        bbox = record.get("crop_bbox_xyxy")
        if not isinstance(bbox, list) or len(bbox) != 4:
            raise WorkerError("crop bbox must contain four integers")
        bbox_tuple = tuple(exact_int(value, "crop bbox") for value in bbox)
        if not (bbox_tuple[2] > bbox_tuple[0] and bbox_tuple[3] > bbox_tuple[1]):
            raise WorkerError("crop bbox is empty")
        identity = record.get("source_identity")
        if not isinstance(identity, dict) or set(identity) != {
            "source_split",
            "image_id",
            "annotation_id",
            "world_id",
            "scene",
            "scene_seed",
            "frame_index",
            "negative_only",
        }:
            raise WorkerError("source identity keys changed")
        if identity.get("source_split") != "G10_HOLDOUT":
            raise WorkerError("record source split is not G10_HOLDOUT")
        exact_int(identity.get("image_id"), "source image_id", minimum=1)
        world_id = exact_string(identity.get("world_id"), "source world_id")
        if world_id not in HOLDOUT_WORLDS:
            raise WorkerError("record world is outside the frozen HOLDOUT allowlist")
        observed_worlds.add(world_id)
        scene = exact_string(identity.get("scene"), "source scene")
        if SCENE_RE.fullmatch(scene) is None:
            raise WorkerError("record source scene is invalid")
        exact_int(identity.get("scene_seed"), "source scene_seed", minimum=0)
        exact_int(identity.get("frame_index"), "source frame_index", minimum=0)
        negative_only = exact_bool(identity.get("negative_only"), "source negative_only")
        annotation_id = identity.get("annotation_id")
        if class_name == "background_or_unknown":
            if annotation_id is not None or not negative_only:
                raise WorkerError("background record is not bound to a negative-only frame")
        else:
            exact_int(annotation_id, "source annotation_id", minimum=1)
            if negative_only:
                raise WorkerError("positive record cannot come from a negative-only frame")
        source_hashes = record.get("source_sha256")
        if not isinstance(source_hashes, dict) or set(source_hashes) != SOURCE_HASH_KEYS:
            raise WorkerError("source SHA-256 key set changed")
        for name, value in source_hashes.items():
            exact_sha256(value, f"source_sha256.{name}")
        source_paths = record.get("source_paths")
        if not isinstance(source_paths, dict) or set(source_paths) != SOURCE_HASH_KEYS:
            raise WorkerError("source path key set changed")
        for name, value in source_paths.items():
            canonical_relative_path(value, f"source_paths.{name}")
        identity_hash = exact_sha256(
            record.get("source_identity_sha256"), "source_identity_sha256"
        )
        if canonical_sha256({"identity": identity, "sha256": source_hashes}) != identity_hash:
            raise WorkerError("source identity SHA-256 mismatch")
        expected_record_id = "emf-holdout-" + canonical_sha256(
            {
                "source_identity_sha256": identity_hash,
                "class_name": class_name,
                "crop_bbox_xyxy": bbox,
            }
        )[:24]
        if expected_record_id != record_id:
            raise WorkerError("record_id does not match its locked content")
        validated.append(
            {
                "record": record,
                "record_id": record_id,
                "class_name": class_name,
                "crop_relative": crop_relative,
                "crop_sha256": crop_hash,
                "source_identity": identity,
                "source_identity_sha256": identity_hash,
            }
        )
    if [row["record_id"] for row in validated] != sorted(seen_ids):
        raise WorkerError("HOLDOUT records must remain sorted by record_id")
    if dict(observed_counts) != normalized_counts:
        raise WorkerError("observed record classes do not match counts")
    if observed_worlds != HOLDOUT_WORLDS:
        raise WorkerError("records do not cover the exact frozen HOLDOUT world set")
    return validated


def load_dataset(
    manifest_path: Path, manifest_sha256: str, crop_root: Path
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    expected_manifest_hash = exact_sha256(manifest_sha256.lower(), "manifest CLI SHA-256")
    try:
        manifest_bytes = manifest_path.read_bytes()
        payload = json.loads(manifest_bytes.decode("utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise WorkerError("cannot read HOLDOUT manifest") from exc
    if sha256_bytes(manifest_bytes) != expected_manifest_hash:
        raise WorkerError("HOLDOUT manifest file SHA-256 mismatch")
    if not isinstance(payload, dict):
        raise WorkerError("HOLDOUT manifest root must be an object")
    validated = validate_manifest(payload)
    rows = []
    for item in validated:
        crop_path = require_contained_file(
            crop_root, item["crop_relative"], f"crop {item['record_id']}"
        )
        crop_bytes = crop_path.read_bytes()
        if sha256_bytes(crop_bytes) != item["crop_sha256"]:
            raise WorkerError(f"crop SHA-256 mismatch: {item['record_id']}")
        rows.append({**item, "crop_bytes": crop_bytes})
    return payload, rows


def parse_mounts(text: str) -> list[tuple[PurePosixPath, set[str]]]:
    mounts = []
    for line in text.splitlines():
        fields = line.split()
        if len(fields) < 4:
            continue
        mount_point = fields[1].replace("\\040", " ")
        mounts.append((PurePosixPath(mount_point), set(fields[3].split(","))))
    return mounts


def mount_options_for_resolved(
    resolved: PurePosixPath, mounts: list[tuple[PurePosixPath, set[str]]]
) -> set[str]:
    candidates = []
    for mount_point, options in mounts:
        try:
            resolved.relative_to(mount_point)
        except ValueError:
            continue
        candidates.append((len(mount_point.parts), options))
    if not candidates:
        raise WorkerError(f"cannot resolve mount options for {resolved}")
    return max(candidates, key=lambda item: item[0])[1]


def mount_options(path: Path, mounts: list[tuple[PurePosixPath, set[str]]]) -> set[str]:
    return mount_options_for_resolved(PurePosixPath(path.resolve().as_posix()), mounts)


def validate_runtime_isolation(read_only_paths: Sequence[Path], output_parent: Path) -> dict[str, Any]:
    if os.name != "posix" or not hasattr(os, "geteuid") or os.geteuid() == 0:
        raise WorkerError("C1 worker must run as a non-root POSIX user")
    interfaces = sorted(path.name for path in Path("/sys/class/net").iterdir())
    if interfaces != ["lo"]:
        raise WorkerError("container network namespace is not isolated to loopback")
    mounts = parse_mounts(Path("/proc/mounts").read_text(encoding="utf-8"))
    if "ro" not in mount_options(Path("/"), mounts):
        raise WorkerError("container root filesystem must be read-only")
    for path in read_only_paths:
        if "ro" not in mount_options(path, mounts):
            raise WorkerError(f"required input mount is writable: {path}")
    if "rw" not in mount_options(output_parent, mounts):
        raise WorkerError("output directory must be a dedicated writable mount")
    return {
        "uid": os.geteuid(),
        "non_root": True,
        "network_interfaces": interfaces,
        "network_mode": "none",
        "root_filesystem_read_only": True,
        "input_mounts_read_only": True,
        "dedicated_output_mount_writable": True,
    }


def validate_model_session(session: Any) -> dict[str, Any]:
    inputs = session.get_inputs()
    outputs = session.get_outputs()
    if len(inputs) != 1 or (inputs[0].name, inputs[0].shape, inputs[0].type) != (
        "images",
        [1, 3, 224, 224],
        "tensor(float)",
    ):
        raise WorkerError("C1 ONNX input contract changed")
    if len(outputs) != 1 or (outputs[0].name, outputs[0].shape, outputs[0].type) != (
        "output0",
        [1, 8],
        "tensor(float)",
    ):
        raise WorkerError("C1 ONNX output contract changed")
    metadata = session.get_modelmeta().custom_metadata_map
    try:
        embedded_names = ast.literal_eval(metadata["names"])
    except (KeyError, ValueError, SyntaxError) as exc:
        raise WorkerError("C1 ONNX embedded class metadata is invalid") from exc
    if not isinstance(embedded_names, dict) or tuple(
        embedded_names.get(index) for index in range(len(CLASS_ORDER))
    ) != CLASS_ORDER:
        raise WorkerError("C1 ONNX embedded class order changed")
    if metadata.get("task") != "classify" or metadata.get("imgsz") != "[224, 224]":
        raise WorkerError("C1 ONNX task/image-size metadata changed")
    return {
        "input": {"name": "images", "shape": [1, 3, 224, 224], "dtype": "float32"},
        "output": {"name": "output0", "shape": [1, 8], "dtype": "float32"},
        "embedded_class_order_verified": True,
    }


def preprocess_crop(crop_bytes: bytes, cv2_module: Any, numpy_module: Any) -> Any:
    image = cv2_module.imdecode(
        numpy_module.frombuffer(crop_bytes, dtype=numpy_module.uint8),
        cv2_module.IMREAD_COLOR,
    )
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise WorkerError("cannot decode HOLDOUT crop as BGR HWC")
    rgb = cv2_module.cvtColor(image, cv2_module.COLOR_BGR2RGB)
    resized = cv2_module.resize(
        rgb, (224, 224), interpolation=cv2_module.INTER_NEAREST
    )
    tensor = resized.transpose(2, 0, 1)[None].astype(numpy_module.float32) / 255.0
    if tensor.shape != (1, 3, 224, 224) or not numpy_module.isfinite(tensor).all():
        raise WorkerError("C1 preprocessing tensor contract failed")
    return numpy_module.ascontiguousarray(tensor)


def classification_metrics(rows: list[dict[str, Any]]) -> dict[str, Any]:
    confusion = {
        actual: {predicted: 0 for predicted in PRODUCT_CLASSES}
        for actual in PRODUCT_CLASSES
    }
    for row in rows:
        confusion[row["actual_product_class"]][row["predicted_product_class"]] += 1
    per_class = {}
    for class_name in PRODUCT_CLASSES:
        true_positive = confusion[class_name][class_name]
        false_positive = sum(
            confusion[actual][class_name]
            for actual in PRODUCT_CLASSES
            if actual != class_name
        )
        false_negative = sum(
            confusion[class_name][predicted]
            for predicted in PRODUCT_CLASSES
            if predicted != class_name
        )
        precision = true_positive / (true_positive + false_positive) if true_positive + false_positive else 0.0
        recall = true_positive / (true_positive + false_negative) if true_positive + false_negative else 0.0
        f1 = 2.0 * precision * recall / (precision + recall) if precision + recall else 0.0
        per_class[class_name] = {
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "support": sum(confusion[class_name].values()),
        }
    return {
        "confusion": confusion,
        "per_class": per_class,
        "macro_f1": sum(row["f1"] for row in per_class.values()) / len(PRODUCT_CLASSES),
        "target_macro_f1": sum(per_class[name]["f1"] for name in PRODUCT_CLASSES[1:]) / 3.0,
        "background_specificity": per_class["background"]["recall"],
    }


def run_inference(
    dataset_rows: list[dict[str, Any]], session: Any, cv2_module: Any, numpy_module: Any
) -> list[dict[str, Any]]:
    rows = []
    for item in dataset_rows:
        output = session.run(
            ["output0"],
            {"images": preprocess_crop(item["crop_bytes"], cv2_module, numpy_module)},
        )
        if not isinstance(output, list) or len(output) != 1:
            raise WorkerError("C1 ONNX returned an unexpected output list")
        probabilities = numpy_module.asarray(output[0])
        if probabilities.shape != (1, 8) or not numpy_module.isfinite(probabilities).all():
            raise WorkerError("C1 ONNX output shape or finiteness mismatch")
        probabilities = probabilities[0].astype(numpy_module.float64, copy=False)
        if (probabilities < 0.0).any() or abs(float(probabilities.sum()) - 1.0) > 1e-5:
            raise WorkerError("C1 ONNX output is not a probability distribution")
        class_index = int(numpy_module.argmax(probabilities))
        source_class = CLASS_ORDER[class_index]
        actual_class = (
            "background"
            if item["class_name"] == "background_or_unknown"
            else item["class_name"]
        )
        rows.append(
            {
                "record_id": item["record_id"],
                "actual_holdout_class": item["class_name"],
                "actual_product_class": actual_class,
                "crop_path": item["crop_relative"].as_posix(),
                "crop_sha256": item["crop_sha256"],
                "source_identity": item["source_identity"],
                "source_identity_sha256": item["source_identity_sha256"],
                "source_class": source_class,
                "source_confidence": float(probabilities[class_index]),
                "predicted_product_class": CLASS_MAPPING.get(source_class, "background"),
                "probabilities": {
                    name: float(probabilities[index])
                    for index, name in enumerate(CLASS_ORDER)
                },
            }
        )
    return rows


def write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    if path.exists():
        raise WorkerError("output path already exists")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, ensure_ascii=False, sort_keys=True, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists():
            raise WorkerError("output path appeared during atomic write")
        os.replace(temporary, path)
    except Exception:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        raise


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--manifest-sha256", required=True)
    parser.add_argument("--crop-root", type=Path, required=True)
    parser.add_argument("--runtime-image-digest", required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    for value, field in (
        (args.model, "model path"),
        (args.manifest, "manifest path"),
        (args.crop_root, "crop root"),
        (args.output, "output path"),
    ):
        reject_forbidden(value, field)
    if args.runtime_image_digest != RUNTIME_IMAGE_DIGEST:
        raise WorkerError("runtime image digest is outside the fixed allowlist")
    model = args.model.resolve()
    manifest = args.manifest.resolve()
    crop_root = args.crop_root.resolve()
    output = args.output.resolve()
    if not model.is_file() or sha256(model) != MODEL_SHA256:
        raise WorkerError("C1 ONNX model SHA-256 mismatch")
    if not crop_root.is_dir() or not manifest.is_file():
        raise WorkerError("HOLDOUT manifest/crop root is missing")
    if output.exists() or not output.parent.is_dir():
        raise WorkerError("output must be a new file in an existing writable mount")
    isolation = validate_runtime_isolation(
        [Path(__file__).resolve(), model, manifest, crop_root], output.parent
    )

    import cv2
    import numpy
    import onnxruntime

    versions = {
        "python": sys.version.split()[0],
        "onnxruntime": onnxruntime.__version__,
        "opencv": cv2.__version__,
        "numpy": numpy.__version__,
    }
    if versions != RUNTIME_VERSIONS:
        raise WorkerError(f"runtime versions differ from the frozen image: {versions}")
    session_options = onnxruntime.SessionOptions()
    session_options.enable_mem_pattern = False
    session_options.enable_cpu_mem_arena = False
    session = onnxruntime.InferenceSession(
        str(model),
        sess_options=session_options,
        providers=["CPUExecutionProvider"],
    )
    if sha256(model) != MODEL_SHA256:
        raise WorkerError("C1 ONNX model changed while loading")
    model_contract = validate_model_session(session)
    manifest_payload, dataset_rows = load_dataset(
        manifest, args.manifest_sha256, crop_root
    )
    started = time.perf_counter()
    rows = run_inference(dataset_rows, session, cv2, numpy)
    elapsed = time.perf_counter() - started
    metrics = classification_metrics(rows)
    payload = {
        "schema_version": "emfj6v3.classifier_holdout_raw_inference.v1",
        "protocol_id": "EMFJ6V3",
        "stage": "CLASSIFIER_HOLDOUT_RAW_INFERENCE",
        "source_split": "G10_HOLDOUT",
        "model_id": MODEL_ID,
        "model_sha256": MODEL_SHA256,
        "runtime": {
            "backend": "PC_ONNX_CPU",
            "image_digest": RUNTIME_IMAGE_DIGEST,
            "versions": versions,
            "provider": session.get_providers(),
            "fallback_used": False,
            "isolation": isolation,
            "records": len(rows),
            "elapsed_seconds": elapsed,
        },
        "model_contract": model_contract,
        "class_order": list(CLASS_ORDER),
        "class_order_source": "embedded_onnx_metadata",
        "class_mapping": CLASS_MAPPING,
        "source_manifest": {
            "file_sha256": args.manifest_sha256.lower(),
            "canonical_manifest_sha256": manifest_payload["canonical_manifest_sha256"],
            "identity_lock_sha256": manifest_payload["identity_lock_sha256"],
            "g10_domain_manifest_sha256": DOMAIN_MANIFEST_SHA256,
            "holdout_world_ids": sorted(HOLDOUT_WORLDS),
            "counts": manifest_payload["counts"],
        },
        "dataset": {
            "builder_declared_all_validated_source_frames_sha256": manifest_payload[
                "all_validated_source_frames_sha256"
            ],
            "offline_gt_development_only": True,
            "production_runtime_gt_forbidden": True,
            "formal_product_evidence": False,
        },
        "metrics": metrics,
        "rows": rows,
        "training_performed": False,
        "raw_probabilities_only": True,
        "threshold_applied": False,
        "offline_gt_development_only": True,
        "production_runtime_eligible": False,
        "threshold_selected": False,
        "threshold_frozen": False,
        "route_selected": False,
        "route_frozen": False,
        "EMF_EXISTING_MODEL_SCREENING_COMPLETE": False,
        "EMF_EXISTING_MODEL_FUNCTIONAL_CANDIDATE": False,
        "EMF_EXISTING_MODEL_PRODUCT_CANDIDATE": False,
        "truth_boundary": (
            "Offline-GT development HOLDOUT native C1 ONNX probabilities only. "
            "No threshold or route was selected/frozen; no functional, product, release, "
            "Journey 6, board, or training claim is made."
        ),
    }
    write_json_atomic(output, payload)
    print(
        json.dumps(
            {
                "model_id": MODEL_ID,
                "records": len(rows),
                "output_sha256": sha256(output),
                "selected": False,
                "frozen": False,
                "training": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
