#!/usr/bin/env python3
"""Offline native TRAIN smoke worker for frozen C3 TrashNet H5.

This file intentionally remains Python 3.6 compatible for the immutable
TensorFlow 1.15.5 runtime.  It never imports or calls the upstream
``prediction.py`` helper (which deletes its input image).
"""

import argparse
import hashlib
import json
import math
import os
import re
import sys
import time
from collections import Counter
from pathlib import Path, PurePosixPath

MODEL_ID = "c3_vasantvohra_trashnet"
MODEL_SHA256 = "013afdc86a673cb2354f4559c165301d5abda1c5878bb523a5995e483d4cc90a"
MODEL_BYTES = 34180888
SOURCE_REVISION = "db53f56e140e38e42b948d377b814fec2954da51"
CLASS_ORDER_SOURCE_SHA256 = (
    "1922931ce576f39ff47cf5fbef5c48efcddcde49ef2631550364113d6ac6b0b8"
)
RUNTIME_IMAGE = "tensorflow/tensorflow:1.15.5-py3-jupyter"
RUNTIME_IMAGE_DIGEST = (
    "sha256:47aa058918aa7b09343c05ccbd23ccef976006a07b579143e9adde34a937b419"
)
RUNTIME_VERSIONS = {
    "python": "3.6.9",
    "tensorflow": "1.15.5",
    "numpy": "1.18.5",
    "pillow": "8.1.0",
    "h5py": "2.10.0",
}
MANIFEST_SHA256 = "2f226ed4925779348e218c49c16d0b33ba719958289e493948fd5427e94e3166"
IDENTITY_LOCK_SHA256 = (
    "998f5666410ef5893cbbddbe22e8831fd70eb548559cda9739ba84086c8d37f8"
)
RECORD_COUNT = 183
CLASS_COUNTS = {
    "background": 102,
    "plastic_bottle": 14,
    "metal_can": 47,
    "paper_litter": 20,
}
PRODUCT_CLASSES = ("background", "plastic_bottle", "metal_can", "paper_litter")
CLASS_ORDER = ("cardboard", "glass", "metal", "paper", "plastic", "trash")
CLASS_MAPPING = {
    "metal": "metal_can",
    "paper": "paper_litter",
    "plastic": "plastic_bottle",
}
FORBIDDEN_PATTERN = re.compile(
    r"(?:^|[^A-Z0-9])(?:G5(?:_V2)?|VAL_NEW|DEV_VAL|SEALED)(?:$|[^A-Z0-9])",
    re.IGNORECASE,
)


class WorkerError(ValueError):
    """Raised when the frozen artifact, data, or runtime contract is violated."""


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def reject_forbidden(value, field):
    text = str(value)
    normalized_words = re.sub(r"[^A-Z0-9]+", "_", text.upper()).split("_")
    normalized = "_".join(word for word in normalized_words if word)
    if (
        FORBIDDEN_PATTERN.search(text)
        or any(word == "G5" or word.startswith("G5V2") for word in normalized_words)
        or "VAL_NEW" in normalized
        or "DEV_VAL" in normalized
    ):
        raise WorkerError(f"forbidden validation or sealed marker in {field}")


def iter_strings(value):
    if isinstance(value, str):
        yield value
    elif isinstance(value, dict):
        for key, item in value.items():
            yield str(key)
            for nested in iter_strings(item):
                yield nested
    elif isinstance(value, list):
        for item in value:
            for nested in iter_strings(item):
                yield nested


def load_json(path, expected_sha256, field):
    reject_forbidden(path.resolve(), f"{field} path")
    if not path.is_file() or sha256(path) != expected_sha256:
        raise WorkerError(f"fixed {field} SHA-256 mismatch")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        raise WorkerError(f"cannot read fixed {field}: {exc}")
    if not isinstance(payload, dict):
        raise WorkerError(f"fixed {field} root must be an object")
    for value in iter_strings(payload):
        reject_forbidden(value, f"{field} payload")
    return payload


def verify_model(model):
    reject_forbidden(model.resolve(), "model path")
    if not model.is_file():
        raise WorkerError("fixed C3 H5 is missing")
    if model.stat().st_size != MODEL_BYTES or sha256(model) != MODEL_SHA256:
        raise WorkerError("fixed C3 H5 bytes or SHA-256 mismatch")


def rebase_source_path(file_name, source_prefix, data_root):
    if not isinstance(file_name, str) or not file_name:
        raise WorkerError("record rgb_path must be a non-empty string")
    reject_forbidden(file_name, "record rgb_path")
    normalized = file_name.replace("\\", "/")
    prefix = source_prefix.replace("\\", "/").rstrip("/")
    if not prefix or not normalized.casefold().startswith(prefix.casefold() + "/"):
        raise WorkerError("record path is outside the explicit source prefix")
    relative = normalized[len(prefix) + 1 :]
    root = data_root.resolve()
    actual = (root / Path(*relative.split("/"))).resolve()
    if actual != root and root not in actual.parents:
        raise WorkerError("rebased record escapes the explicit development root")
    reject_forbidden(actual, "rebased RGB path")
    return actual, relative


def validate_dataset_payloads(
    manifest,
    identity_lock,
    expected_count=RECORD_COUNT,
    expected_class_counts=CLASS_COUNTS,
):
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
        raise WorkerError("identity lock is not the fixed C4 raw evidence")
    if identity_lock.get("source_manifest_sha256") != MANIFEST_SHA256:
        raise WorkerError("identity lock source manifest mismatch")

    records = manifest.get("records")
    lock_rows = identity_lock.get("rows")
    if not isinstance(records, list) or len(records) != expected_count:
        raise WorkerError("fixed TRAIN manifest record count mismatch")
    if not isinstance(lock_rows, list) or len(lock_rows) != expected_count:
        raise WorkerError("fixed identity lock row count mismatch")
    by_id = {}
    for row in lock_rows:
        if not isinstance(row, dict):
            raise WorkerError("identity lock row must be an object")
        record_id = row.get("record_id")
        if not isinstance(record_id, str) or record_id in by_id:
            raise WorkerError("identity lock record IDs must be unique strings")
        by_id[record_id] = row

    manifest_ids = set()
    class_counts = Counter()
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
            raise WorkerError("record GT role is outside the development contract")
        if record.get("production_runtime_gt_used") is not False:
            raise WorkerError("production runtime GT use must remain false")
        actual = record.get("class_id")
        if actual not in PRODUCT_CLASSES:
            raise WorkerError("record class is outside the fixed product classes")
        class_counts[actual] += 1
        locked = by_id.get(record_id)
        if locked is None or locked.get("actual_product_class") != actual:
            raise WorkerError("manifest and identity-lock identities differ")
        try:
            manifest_bbox = tuple(float(value) for value in record["proposal_bbox_native_xyxy"])
            lock_bbox = tuple(float(value) for value in locked["bbox_xyxy"])
        except (KeyError, TypeError, ValueError):
            raise WorkerError("manifest or identity-lock bbox is invalid")
        if len(manifest_bbox) != 4 or manifest_bbox != lock_bbox:
            raise WorkerError("manifest and identity-lock bboxes differ")
        x1, y1, x2, y2 = manifest_bbox
        if not all(math.isfinite(value) for value in manifest_bbox) or x2 <= x1 or y2 <= y1:
            raise WorkerError("manifest bbox is empty or non-finite")
        locked_sha = locked.get("source_image_sha256")
        if not isinstance(locked_sha, str) or not re.fullmatch(r"[0-9a-f]{64}", locked_sha):
            raise WorkerError("identity lock contains an invalid image SHA-256")
        relative_file = locked.get("relative_file")
        if not isinstance(relative_file, str) or not relative_file:
            raise WorkerError("identity lock relative file is invalid")
    if manifest_ids != set(by_id):
        raise WorkerError("manifest and identity-lock record ID sets differ")
    if dict(class_counts) != expected_class_counts:
        raise WorkerError("fixed TRAIN manifest class counts differ")
    return by_id


def parse_mounts(text):
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


def mount_is_read_only(path, mounts):
    resolved = PurePosixPath(path.as_posix())
    candidates = []
    for mount_point, options in mounts:
        try:
            resolved.relative_to(mount_point)
        except ValueError:
            continue
        candidates.append((len(mount_point.parts), options))
    return bool(candidates) and "ro" in max(candidates, key=lambda item: item[0])[1]


def validate_runtime_isolation(read_only_paths):
    if os.name != "posix" or not hasattr(os, "geteuid") or os.geteuid() == 0:
        raise WorkerError("native worker must run as a non-root POSIX user")
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
    }


def crop_box(image, bbox):
    width, height = image.size
    x1, y1, x2, y2 = bbox
    left = max(0, min(width - 1, math.floor(x1)))
    top = max(0, min(height - 1, math.floor(y1)))
    right = max(left + 1, min(width, math.ceil(x2)))
    bottom = max(top + 1, min(height, math.ceil(y2)))
    return image.crop((left, top, right, bottom))


def preprocess_crop(source_image, bbox, numpy_module, image_module):
    rgb = source_image.convert("RGB")
    crop = crop_box(rgb, bbox)
    resized = crop.resize((300, 300), resample=image_module.NEAREST)
    uint8_nhwc = numpy_module.asarray(resized, dtype=numpy_module.uint8)
    if uint8_nhwc.shape != (300, 300, 3):
        raise WorkerError("C3 preprocessing did not produce RGB NHWC 300x300")
    batch = numpy_module.expand_dims(uint8_nhwc, axis=0) / 255.0
    if batch.shape != (1, 300, 300, 3):
        raise WorkerError("C3 preprocessing batch shape mismatch")
    return batch


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
        "background_specificity": per_class["background"]["recall"],
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--identity-lock", required=True, type=Path)
    parser.add_argument("--source-prefix", required=True)
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--runtime-image-digest", required=True)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    for value, field in (
        (args.model, "model path"),
        (args.manifest, "manifest path"),
        (args.identity_lock, "identity lock path"),
        (args.source_prefix, "source prefix"),
        (args.data_root, "data root"),
        (args.output, "output path"),
    ):
        reject_forbidden(value, field)
    if args.runtime_image_digest != RUNTIME_IMAGE_DIGEST:
        raise WorkerError("runtime image digest is outside the fixed allowlist")

    model = args.model.resolve()
    manifest_path = args.manifest.resolve()
    identity_path = args.identity_lock.resolve()
    data_root = args.data_root.resolve()
    if not data_root.is_dir():
        raise WorkerError("explicit development data root is missing")
    isolation = validate_runtime_isolation([model, manifest_path, identity_path, data_root])
    verify_model(model)
    manifest = load_json(manifest_path, MANIFEST_SHA256, "TRAIN manifest")
    identity_lock = load_json(identity_path, IDENTITY_LOCK_SHA256, "identity lock")
    identities = validate_dataset_payloads(manifest, identity_lock)

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
        raise WorkerError(f"runtime package versions differ from the fixed image: {versions}")
    model_object = tf.keras.models.load_model(str(model), compile=False)
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
    for record in manifest["records"]:
        locked = identities[record["record_id"]]
        actual, relative = rebase_source_path(record["rgb_path"], args.source_prefix, data_root)
        if not actual.is_file() or sha256(actual) != locked["source_image_sha256"]:
            raise WorkerError("fixed source image SHA-256 mismatch: {}".format(record["record_id"]))
        if relative.replace("\\", "/") != locked["relative_file"].replace("\\", "/"):
            raise WorkerError("rebased source path differs from the identity lock")
        bbox = tuple(float(value) for value in record["proposal_bbox_native_xyxy"])
        with Image.open(str(actual)) as source_image:
            batch = preprocess_crop(source_image, bbox, numpy, Image)
        output = model_object.predict(batch, batch_size=1, verbose=0)
        output = numpy.asarray(output)
        if output.shape != (1, 6) or not numpy.isfinite(output).all():
            raise WorkerError("C3 inference output shape or finiteness mismatch")
        probabilities_array = output[0].astype(numpy.float64, copy=False)
        if (probabilities_array < 0.0).any() or abs(float(probabilities_array.sum()) - 1.0) > 1e-5:
            raise WorkerError("C3 output is not a normalized probability vector")
        source_index = int(numpy.argmax(probabilities_array))
        source_class = CLASS_ORDER[source_index]
        probabilities = {
            name: float(probabilities_array[index])
            for index, name in enumerate(CLASS_ORDER)
        }
        rows.append(
            {
                "record_id": record["record_id"],
                "relative_file": relative.replace("\\", "/"),
                "source_image_sha256": locked["source_image_sha256"],
                "bbox_xyxy": list(bbox),
                "actual_product_class": record["class_id"],
                "source_class": source_class,
                "source_confidence": float(probabilities_array[source_index]),
                "predicted_product_class": CLASS_MAPPING.get(source_class, "background"),
                "probabilities": probabilities,
            }
        )
    elapsed = time.perf_counter() - started
    metrics = classification_metrics(rows)
    report = {
        "schema_version": 1,
        "protocol_id": "EMFJ6V3",
        "stage": "A4_CLASSIFIER_GT_TRAIN_NATIVE_SMOKE",
        "model_id": MODEL_ID,
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
        "source_manifest_sha256": MANIFEST_SHA256,
        "identity_lock_sha256": IDENTITY_LOCK_SHA256,
        "dataset": {
            "source_split": "G10_TRAIN",
            "development_only": True,
            "formal_eligible": False,
            "production_runtime_gt_used": False,
            "crop_source": "offline_gt_box_development_only",
            "record_count": len(rows),
            "class_counts": CLASS_COUNTS,
            "all_record_image_bbox_identities_verified": True,
            "independent_negative_only_domain": False,
        },
        "preprocess_contract": {
            "crop_bounds": "floor_xy1_ceil_xy2_clamped",
            "color": "PIL_RGB",
            "resize": [300, 300],
            "resample": "PIL_NEAREST",
            "layout": "NHWC",
            "source_dtype": "uint8",
            "rescale": "divide_by_255.0",
            "batch_dtype": "float64",
        },
        "model_contract": {
            "architecture": "custom_tensorflow_keras_cnn",
            "input_shape": [None, 300, 300, 3],
            "input_dtype": "float32",
            "output_shape": [None, 6],
            "output_activation": "softmax",
        },
        "class_order": list(CLASS_ORDER),
        "class_order_source": {
            "revision": SOURCE_REVISION,
            "file": "prediction.py",
            "sha256": CLASS_ORDER_SOURCE_SHA256,
            "executed": False,
        },
        "class_mapping": CLASS_MAPPING,
        "metrics": metrics,
        "rows": rows,
        "EMF_EXISTING_MODEL_SCREENING_COMPLETE": False,
        "EMF_EXISTING_MODEL_FUNCTIONAL_CANDIDATE": False,
        "EMF_EXISTING_MODEL_PRODUCT_CANDIDATE": False,
        "training_authorized": False,
        "decision": "development_gt_train_native_smoke_only",
        "truth_boundary": (
            "Development-only native H5 smoke on offline GT-derived TRAIN crops. "
            "It is not proposal-crop A4, independent negative-domain specificity, "
            "HOLDOUT, license clearance, product, release, Journey 6, or training evidence."
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "output": str(args.output),
                "bytes": args.output.stat().st_size,
                "sha256": sha256(args.output),
                "records": len(rows),
                "macro_f1": metrics["macro_f1"],
                "target_macro_f1": metrics["target_macro_f1"],
                "background_specificity": metrics["background_specificity"],
                "screening_complete": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except WorkerError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(2)
