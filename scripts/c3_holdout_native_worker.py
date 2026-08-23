#!/usr/bin/env python3
"""Isolated native C3 inference over the fixed EMFJ6V3 GT HOLDOUT bank.

Python 3.6 compatibility is intentional: the locked runtime is TensorFlow
1.15.5.  The upstream ``prediction.py`` helper is never imported or executed.
"""

import argparse
import hashlib
import io
import json
import math
import os
import re
import sys
import tempfile
import time
from collections import Counter
from pathlib import Path

import c3_native_worker as base

MODEL_ID = base.MODEL_ID
MODEL_SHA256 = base.MODEL_SHA256
MODEL_BYTES = base.MODEL_BYTES
SOURCE_REVISION = base.SOURCE_REVISION
CLASS_ORDER_SOURCE_SHA256 = base.CLASS_ORDER_SOURCE_SHA256
RUNTIME_IMAGE = base.RUNTIME_IMAGE
RUNTIME_IMAGE_DIGEST = base.RUNTIME_IMAGE_DIGEST
RUNTIME_VERSIONS = base.RUNTIME_VERSIONS
CLASS_ORDER = base.CLASS_ORDER
WorkerError = base.WorkerError
BASE_MODULE_SHA256 = "ebc454d96a29e17a1a89c40061c484f8b8213647ede3760e619f32aba99512bd"
DOMAIN_MANIFEST_SHA256 = "3bdb3006226943e4149cd84144b488e5eb112ab35ad3692c5da8cc48c88b5208"
HOLDOUT_WORLDS = {
    "g10v15_val_w01_07_service_road",
    "g10v15_val_w02_08_mixed_curb_vegetation",
    "g10v15_val_w03_09_light_paver_pedestrian",
}

SCHEMA_VERSION = "emfj6v3.classifier_holdout_gt.v1"
SOURCE_SPLIT = "G10_HOLDOUT"
PRODUCT_CLASSES = (
    "background_or_unknown",
    "plastic_bottle",
    "metal_can",
    "paper_litter",
)
CLASS_MAPPING = {
    "cardboard": "background_or_unknown",
    "glass": "background_or_unknown",
    "metal": "metal_can",
    "paper": "paper_litter",
    "plastic": "plastic_bottle",
    "trash": "background_or_unknown",
}
POSITIVE_PER_CLASS = 60
BACKGROUND_PER_NEGATIVE_FRAME = 1
REQUIRED_SOURCE_SHA_FIELDS = (
    "rgb",
    "depth",
    "camera",
    "semantic",
    "instance",
    "scene_manifest",
    "capture_report",
)


def canonical_sha256(value):
    encoded = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _valid_sha256(value):
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value) is not None


def _require_bool(payload, field, expected):
    if payload.get(field) is not expected:
        raise WorkerError("HOLDOUT manifest field mismatch: {}".format(field))


def _inside(root, relative_value, field):
    if not isinstance(relative_value, str) or not relative_value:
        raise WorkerError("{} must be a non-empty relative path".format(field))
    base.reject_forbidden(relative_value, field)
    relative = Path(relative_value.replace("\\", "/"))
    if relative.is_absolute() or ".." in relative.parts:
        raise WorkerError("{} escapes the read-only data root".format(field))
    actual = (root / relative).resolve()
    try:
        actual.relative_to(root)
    except ValueError:
        raise WorkerError("{} escapes the read-only data root".format(field))
    base.reject_forbidden(actual, field)
    return actual


def _validate_bbox(value):
    if not isinstance(value, list) or len(value) != 4:
        raise WorkerError("record crop bbox must contain four values")
    try:
        bbox = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        raise WorkerError("record crop bbox is not numeric")
    if not all(math.isfinite(item) for item in bbox):
        raise WorkerError("record crop bbox is non-finite")
    if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
        raise WorkerError("record crop bbox is empty")
    return bbox


def load_manifest(path, expected_file_sha256):
    base.reject_forbidden(path.resolve(), "HOLDOUT manifest path")
    if not _valid_sha256(expected_file_sha256):
        raise WorkerError("expected HOLDOUT manifest SHA-256 is invalid")
    try:
        raw = path.read_bytes()
        payload = json.loads(raw.decode("utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        raise WorkerError("cannot read fixed HOLDOUT manifest: {}".format(exc))
    if hashlib.sha256(raw).hexdigest() != expected_file_sha256:
        raise WorkerError("fixed HOLDOUT manifest file SHA-256 mismatch")
    if not isinstance(payload, dict):
        raise WorkerError("fixed HOLDOUT manifest root must be an object")
    for value in base.iter_strings(payload):
        base.reject_forbidden(value, "HOLDOUT manifest payload")
    declared = payload.get("canonical_manifest_sha256")
    unsigned = dict(payload)
    unsigned.pop("canonical_manifest_sha256", None)
    if not _valid_sha256(declared) or canonical_sha256(unsigned) != declared:
        raise WorkerError("HOLDOUT manifest canonical SHA-256 mismatch")
    return payload


def validate_manifest(manifest, data_root):
    expected_fields = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": "EMFJ6V3",
        "stage": "CLASSIFIER_HOLDOUT_OFFLINE_GT_DEVELOPMENT",
        "source_split": SOURCE_SPLIT,
    }
    for field, expected in expected_fields.items():
        if manifest.get(field) != expected:
            raise WorkerError("HOLDOUT manifest field mismatch: {}".format(field))
    if manifest.get("g10_domain_manifest_sha256") != DOMAIN_MANIFEST_SHA256:
        raise WorkerError("HOLDOUT domain manifest SHA-256 changed")
    if manifest.get("holdout_world_ids") != sorted(HOLDOUT_WORLDS):
        raise WorkerError("HOLDOUT world allowlist changed")
    if not _valid_sha256(manifest.get("all_validated_source_frames_sha256")):
        raise WorkerError("HOLDOUT all-source-frame lock is invalid")
    if manifest.get("atomic_output_contract") != {
        "visibility": "same_filesystem_directory_rename",
        "concurrent_writer_policy": "exclusive_sibling_lock_required",
        "power_loss_durability_guaranteed": False,
    }:
        raise WorkerError("HOLDOUT atomic output contract changed")
    for field, expected in (
        ("offline_gt_development_only", True),
        ("production_runtime_gt_forbidden", True),
        ("training_performed", False),
        ("threshold_selected", False),
        ("threshold_frozen", False),
        ("formal_product_evidence", False),
        ("pass", True),
    ):
        _require_bool(manifest, field, expected)
    if not _valid_sha256(manifest.get("input_coco_sha256")):
        raise WorkerError("HOLDOUT input COCO SHA-256 is invalid")

    selection = manifest.get("selection_contract")
    if not isinstance(selection, dict):
        raise WorkerError("HOLDOUT selection contract is missing")
    expected_selection = {
        "seed": 20260824,
        "positive_per_class": POSITIVE_PER_CLASS,
        "background_per_negative_frame": BACKGROUND_PER_NEGATIVE_FRAME,
        "positive_crop_scale": 4.0,
        "minimum_crop_side": 64,
        "background_crop_side": 96,
        "background_max_gt_iou_exclusive": 0.1,
    }
    if selection != expected_selection:
        raise WorkerError("HOLDOUT fixed crop selection contract differs")

    records = manifest.get("records")
    counts = manifest.get("counts")
    if not isinstance(records, list) or not records:
        raise WorkerError("HOLDOUT manifest records are missing")
    if not isinstance(counts, dict) or set(counts) != set(PRODUCT_CLASSES):
        raise WorkerError("HOLDOUT manifest must contain exactly four class counts")
    if any(isinstance(value, bool) or not isinstance(value, int) or value <= 0 for value in counts.values()):
        raise WorkerError("HOLDOUT manifest class counts must be positive integers")
    for class_name in PRODUCT_CLASSES[1:]:
        if counts.get(class_name) != POSITIVE_PER_CLASS:
            raise WorkerError("HOLDOUT target class count differs: {}".format(class_name))
    negative_frames = manifest.get("negative_only_frame_count")
    negative_scenes = manifest.get("negative_only_scene_count")
    if (
        isinstance(negative_frames, bool)
        or not isinstance(negative_frames, int)
        or negative_frames <= 0
        or isinstance(negative_scenes, bool)
        or not isinstance(negative_scenes, int)
        or negative_scenes <= 0
    ):
        raise WorkerError("HOLDOUT negative-only domain is missing")
    if counts["background_or_unknown"] != negative_frames * BACKGROUND_PER_NEGATIVE_FRAME:
        raise WorkerError("HOLDOUT background count does not bind every negative-only frame")
    if sum(counts.values()) != len(records):
        raise WorkerError("HOLDOUT record count and class counts differ")
    if manifest.get("identity_lock_sha256") != canonical_sha256(records):
        raise WorkerError("HOLDOUT records identity-lock SHA-256 mismatch")

    observed_counts = Counter()
    record_ids = set()
    identity_hashes = set()
    background_scenes = set()
    positive_scenes = set()
    class_worlds = {class_name: set() for class_name in PRODUCT_CLASSES}
    observed_worlds = set()
    validated = []
    for record in records:
        if not isinstance(record, dict):
            raise WorkerError("HOLDOUT record must be an object")
        class_name = record.get("class_name")
        if class_name not in PRODUCT_CLASSES:
            raise WorkerError("HOLDOUT record class is outside the four-class contract")
        if record.get("offline_gt_development_only") is not True:
            raise WorkerError("HOLDOUT record is not offline-GT development-only")
        if record.get("production_runtime_eligible") is not False:
            raise WorkerError("HOLDOUT GT record cannot be runtime eligible")
        source_identity = record.get("source_identity")
        source_hashes = record.get("source_sha256")
        source_paths = record.get("source_paths")
        if not isinstance(source_identity, dict):
            raise WorkerError("HOLDOUT record source identity is missing")
        if not isinstance(source_hashes, dict) or set(source_hashes) != set(REQUIRED_SOURCE_SHA_FIELDS):
            raise WorkerError("HOLDOUT record source SHA fields differ")
        if not isinstance(source_paths, dict) or set(source_paths) != set(REQUIRED_SOURCE_SHA_FIELDS):
            raise WorkerError("HOLDOUT record source path fields differ")
        if any(not _valid_sha256(source_hashes[field]) for field in REQUIRED_SOURCE_SHA_FIELDS):
            raise WorkerError("HOLDOUT record contains an invalid source SHA-256")
        for field in REQUIRED_SOURCE_SHA_FIELDS:
            if not isinstance(source_paths[field], str) or not source_paths[field]:
                raise WorkerError("HOLDOUT record source path is invalid")
            base.reject_forbidden(source_paths[field], "record source path")
        required_identity = {
            "source_split",
            "image_id",
            "annotation_id",
            "world_id",
            "scene",
            "scene_seed",
            "frame_index",
            "negative_only",
        }
        if set(source_identity) != required_identity:
            raise WorkerError("HOLDOUT record source identity fields differ")
        if source_identity.get("source_split") != SOURCE_SPLIT:
            raise WorkerError("HOLDOUT record source split differs")
        world_id = source_identity.get("world_id")
        scene = source_identity.get("scene")
        if not isinstance(world_id, str) or not world_id or not isinstance(scene, str) or not scene:
            raise WorkerError("HOLDOUT record world/scene identity is invalid")
        if world_id not in HOLDOUT_WORLDS:
            raise WorkerError("HOLDOUT record world is outside the frozen allowlist")
        observed_worlds.add(world_id)
        base.reject_forbidden(world_id, "record world_id")
        base.reject_forbidden(scene, "record scene")
        for field in ("image_id", "scene_seed", "frame_index"):
            value = source_identity.get(field)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise WorkerError("HOLDOUT record {} is invalid".format(field))
        negative_only = source_identity.get("negative_only")
        annotation_id = source_identity.get("annotation_id")
        if not isinstance(negative_only, bool):
            raise WorkerError("HOLDOUT record negative_only must be boolean")
        if class_name == "background_or_unknown":
            if negative_only is not True or annotation_id is not None:
                raise WorkerError("background record is not from a negative-only frame")
            background_scenes.add((world_id, scene, source_identity["scene_seed"]))
        else:
            if negative_only is not False or isinstance(annotation_id, bool) or not isinstance(annotation_id, int):
                raise WorkerError("target record does not bind a positive GT annotation")
            positive_scenes.add((world_id, scene, source_identity["scene_seed"]))
        class_worlds[class_name].add(world_id)

        source_identity_sha = record.get("source_identity_sha256")
        calculated_identity_sha = canonical_sha256(
            {"identity": source_identity, "sha256": source_hashes}
        )
        if source_identity_sha != calculated_identity_sha:
            raise WorkerError("HOLDOUT record source identity SHA-256 mismatch")
        bbox = _validate_bbox(record.get("crop_bbox_xyxy"))
        calculated_record_id = "emf-holdout-" + canonical_sha256(
            {
                "source_identity_sha256": source_identity_sha,
                "class_name": class_name,
                "crop_bbox_xyxy": list(record["crop_bbox_xyxy"]),
            }
        )[:24]
        record_id = record.get("record_id")
        if record_id != calculated_record_id or record_id in record_ids:
            raise WorkerError("HOLDOUT record_id is invalid, changed, or duplicated")
        if source_identity_sha in identity_hashes:
            raise WorkerError("HOLDOUT source identity is duplicated")
        crop_path = _inside(data_root, record.get("crop_path"), "record crop path")
        expected_parent = Path("crops") / class_name
        relative_crop = crop_path.relative_to(data_root)
        if relative_crop.parent != expected_parent:
            raise WorkerError("HOLDOUT crop path does not match its class")
        if relative_crop.name != record_id + ".png":
            raise WorkerError("HOLDOUT crop filename does not match record_id")
        if not crop_path.is_file() or not _valid_sha256(record.get("crop_sha256")):
            raise WorkerError("HOLDOUT crop file or SHA-256 is missing")
        crop_bytes = crop_path.read_bytes()
        if hashlib.sha256(crop_bytes).hexdigest() != record["crop_sha256"]:
            raise WorkerError("HOLDOUT crop SHA-256 mismatch")
        observed_counts[class_name] += 1
        record_ids.add(record_id)
        identity_hashes.add(source_identity_sha)
        validated.append(
            {
                "record": record,
                "crop_path": crop_path,
                "crop_bytes": crop_bytes,
                "bbox": bbox,
            }
        )
    if dict(observed_counts) != counts:
        raise WorkerError("HOLDOUT observed record counts differ from manifest")
    if len(background_scenes) != negative_scenes:
        raise WorkerError("HOLDOUT negative-only scene count differs from records")
    if background_scenes.intersection(positive_scenes):
        raise WorkerError("HOLDOUT negative-only and positive scene identities overlap")
    if any(not worlds for worlds in class_worlds.values()):
        raise WorkerError("HOLDOUT class lacks an explicit source world")
    if observed_worlds != HOLDOUT_WORLDS:
        raise WorkerError("records do not cover the exact frozen HOLDOUT world set")
    return sorted(validated, key=lambda item: item["record"]["record_id"])


def preprocess_crop(crop_value, numpy_module, image_module):
    try:
        source_value = io.BytesIO(crop_value) if isinstance(crop_value, bytes) else str(crop_value)
        with image_module.open(source_value) as source:
            rgb = source.convert("RGB")
            resized = rgb.resize((300, 300), resample=image_module.NEAREST)
            uint8_nhwc = numpy_module.asarray(resized, dtype=numpy_module.uint8)
    except (OSError, ValueError) as exc:
        raise WorkerError("cannot decode fixed HOLDOUT crop: {}".format(exc))
    if uint8_nhwc.shape != (300, 300, 3):
        raise WorkerError("C3 HOLDOUT preprocessing did not produce RGB NHWC 300x300")
    batch = numpy_module.expand_dims(uint8_nhwc, axis=0) / 255.0
    if batch.shape != (1, 300, 300, 3):
        raise WorkerError("C3 HOLDOUT preprocessing batch shape mismatch")
    return batch


def probabilities_and_prediction(output, numpy_module):
    array = numpy_module.asarray(output)
    if array.shape != (1, 6) or not numpy_module.isfinite(array).all():
        raise WorkerError("C3 HOLDOUT output shape or finiteness mismatch")
    vector = array[0].astype(numpy_module.float64, copy=False)
    if (vector < 0.0).any() or abs(float(vector.sum()) - 1.0) > 1e-5:
        raise WorkerError("C3 HOLDOUT output is not a normalized probability vector")
    source_index = int(numpy_module.argmax(vector))
    probabilities = {
        name: float(vector[index]) for index, name in enumerate(CLASS_ORDER)
    }
    source_class = CLASS_ORDER[source_index]
    return probabilities, source_class, CLASS_MAPPING[source_class]


def write_json_atomic(path, payload):
    if path.exists():
        raise WorkerError("output already exists; refusing to overwrite evidence")
    if not path.parent.is_dir():
        raise WorkerError("output parent must be a pre-existing writable mount")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix="." + path.name + ".", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists():
            raise WorkerError("output path appeared during atomic write")
        os.replace(str(temporary), str(path))
    except Exception:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def classification_metrics(rows):
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
        precision = (
            true_positive / float(true_positive + false_positive)
            if true_positive + false_positive
            else 0.0
        )
        recall = (
            true_positive / float(true_positive + false_negative)
            if true_positive + false_negative
            else 0.0
        )
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
        "macro_f1": sum(per_class[name]["f1"] for name in PRODUCT_CLASSES)
        / float(len(PRODUCT_CLASSES)),
        "target_macro_f1": sum(per_class[name]["f1"] for name in PRODUCT_CLASSES[1:])
        / float(len(PRODUCT_CLASSES[1:])),
        "background_specificity": per_class["background_or_unknown"]["recall"],
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--expected-manifest-sha256", required=True)
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--runtime-image-digest", required=True)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    for value, field in (
        (args.model, "model path"),
        (args.manifest, "manifest path"),
        (args.data_root, "data root"),
        (args.output, "output path"),
    ):
        base.reject_forbidden(value, field)
    if args.runtime_image_digest != RUNTIME_IMAGE_DIGEST:
        raise WorkerError("runtime image digest is outside the fixed allowlist")
    if args.output.exists():
        raise WorkerError("output already exists; refusing to overwrite evidence")

    model = args.model.resolve()
    manifest_path = args.manifest.resolve()
    data_root = args.data_root.resolve()
    base_path = Path(base.__file__).resolve()
    if base_path.parent != Path(__file__).resolve().parent or base.sha256(base_path) != BASE_MODULE_SHA256:
        raise WorkerError("c3_native_worker dependency path or SHA-256 changed")
    if not data_root.is_dir():
        raise WorkerError("explicit HOLDOUT crop root is missing")
    isolation = base.validate_runtime_isolation(
        [Path(__file__).resolve(), base_path, model, manifest_path, data_root]
    )
    mounts = base.parse_mounts(Path("/proc/mounts").read_text(encoding="utf-8"))
    if base.mount_is_read_only(args.output.parent.resolve(), mounts):
        raise WorkerError("output parent must be a dedicated writable mount")
    isolation["output_mount_writable"] = True
    base.verify_model(model)
    manifest = load_manifest(manifest_path, args.expected_manifest_sha256)
    validated = validate_manifest(manifest, data_root)

    import h5py
    import numpy
    import PIL
    import tensorflow as tf
    from PIL import Image

    versions = {
        "python": sys.version.split()[0],
        "tensorflow": tf.__version__,
        "numpy": numpy.__version__,
        "pillow": PIL.__version__,
        "h5py": h5py.__version__,
    }
    if versions != RUNTIME_VERSIONS:
        raise WorkerError(
            "runtime package versions differ from the fixed image: {}".format(versions)
        )
    model_object = tf.keras.models.load_model(str(model), compile=False)
    if base.sha256(model) != MODEL_SHA256:
        raise WorkerError("C3 H5 changed while loading")
    if tuple(model_object.input_shape) != (None, 300, 300, 3):
        raise WorkerError("loaded C3 model input shape differs")
    if tuple(model_object.output_shape) != (None, 6):
        raise WorkerError("loaded C3 model output shape differs")
    if model_object.layers[-1].__class__.__name__ != "Dense":
        raise WorkerError("loaded C3 final layer is not Dense")
    if getattr(model_object.layers[-1].activation, "__name__", None) != "softmax":
        raise WorkerError("loaded C3 final activation is not softmax")

    rows = []
    started = time.perf_counter()
    for item in validated:
        record = item["record"]
        batch = preprocess_crop(item["crop_bytes"], numpy, Image)
        output = model_object.predict(batch, batch_size=1, verbose=0)
        probabilities, source_class, predicted = probabilities_and_prediction(
            output, numpy
        )
        source_confidence = probabilities[source_class]
        rows.append(
            {
                "record_id": record["record_id"],
                "crop_path": record["crop_path"],
                "crop_sha256": record["crop_sha256"],
                "crop_bbox_xyxy": record["crop_bbox_xyxy"],
                "actual_product_class": record["class_name"],
                "source_class": source_class,
                "source_confidence": source_confidence,
                "predicted_product_class": predicted,
                "probabilities": probabilities,
                "source_identity": record["source_identity"],
                "source_identity_sha256": record["source_identity_sha256"],
            }
        )
    elapsed = time.perf_counter() - started
    metrics = classification_metrics(rows)
    report = {
        "schema_version": "emfj6v3.classifier_holdout_raw_inference.v1",
        "protocol_id": "EMFJ6V3",
        "stage": "CLASSIFIER_HOLDOUT_RAW_INFERENCE",
        "source_split": SOURCE_SPLIT,
        "model_id": MODEL_ID,
        "model_sha256": None,
        "source_uri": "https://github.com/vasantvohra/TrashNet",
        "revision": SOURCE_REVISION,
        "license": "development_only",
        "training_data_boundary": "TrashNet dataset license is not frozen",
        "artifact": {
            "name": "trained_model.h5",
            "bytes": MODEL_BYTES,
            "sha256": MODEL_SHA256,
            "weight_format": "keras_hdf5",
        },
        "runtime": dict(
            {
                "image": RUNTIME_IMAGE,
                "image_digest": RUNTIME_IMAGE_DIGEST,
                "records": len(rows),
                "elapsed_seconds": elapsed,
                "device": "cpu",
            },
            **versions
        ),
        "isolation": isolation,
        "source_manifest": {
            "file_sha256": args.expected_manifest_sha256,
            "canonical_manifest_sha256": manifest["canonical_manifest_sha256"],
            "identity_lock_sha256": manifest["identity_lock_sha256"],
            "g10_domain_manifest_sha256": DOMAIN_MANIFEST_SHA256,
            "holdout_world_ids": sorted(HOLDOUT_WORLDS),
            "counts": manifest["counts"],
        },
        "dataset": {
            "input_coco_sha256": manifest["input_coco_sha256"],
            "builder_declared_all_validated_source_frames_sha256": manifest["all_validated_source_frames_sha256"],
            "records": len(rows),
            "negative_only_scene_count": manifest["negative_only_scene_count"],
            "negative_only_frame_count": manifest["negative_only_frame_count"],
            "all_record_crop_source_identities_verified": True,
            "offline_gt_development_only": True,
            "production_runtime_gt_forbidden": True,
            "formal_product_evidence": False,
        },
        "preprocess": {
            "input": "builder_emitted_crop_png",
            "color": "PIL_RGB",
            "resize": [300, 300],
            "resample": "PIL_NEAREST",
            "layout": "NHWC",
            "source_dtype": "uint8",
            "rescale": "divide_by_255.0",
            "batch_dtype": "float64",
        },
        "class_order": list(CLASS_ORDER),
        "class_order_source": {
            "revision": SOURCE_REVISION,
            "file": "prediction.py",
            "sha256": CLASS_ORDER_SOURCE_SHA256,
            "executed": False,
        },
        "class_mapping": {
            "metal": "metal_can",
            "paper": "paper_litter",
            "plastic": "plastic_bottle",
        },
        "metrics": metrics,
        "rows": rows,
        "selected": False,
        "frozen": False,
        "threshold_selected": False,
        "threshold_frozen": False,
        "training_performed": False,
        "raw_probabilities_only": True,
        "threshold_applied": False,
        "offline_gt_development_only": True,
        "production_runtime_eligible": False,
        "training_authorized": False,
        "EMF_EXISTING_MODEL_FUNCTIONAL_CANDIDATE": False,
        "EMF_EXISTING_MODEL_PRODUCT_CANDIDATE": False,
        "decision": "development_gt_holdout_native_diagnostic_only",
        "truth_boundary": (
            "Native C3 inference on an offline-GT development HOLDOUT bank. "
            "No threshold or model is selected/frozen; this is not proposal-crop A4, "
            "license clearance, product, release, Journey 6, or training evidence."
        ),
    }
    write_json_atomic(args.output, report)
    print(
        json.dumps(
            {
                "output": str(args.output),
                "bytes": args.output.stat().st_size,
                "sha256": base.sha256(args.output),
                "records": len(rows),
                "macro_f1": metrics["macro_f1"],
                "target_macro_f1": metrics["target_macro_f1"],
                "background_specificity": metrics["background_specificity"],
                "selected": False,
                "frozen": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except WorkerError as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        sys.exit(2)
