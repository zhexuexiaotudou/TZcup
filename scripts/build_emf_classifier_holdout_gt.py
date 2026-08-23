#!/usr/bin/env python3
"""Build a deterministic, HOLDOUT-only offline-GT classifier crop bank.

This builder is deliberately separate from the product proposal-crop path.  It
uses Gazebo GT only to construct a development HOLDOUT coverage/unknown bank;
the resulting crops are never runtime-eligible and cannot select a threshold.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import os
import random
import re
import shutil
import tempfile
from collections import Counter, defaultdict
from collections.abc import Sequence
from pathlib import Path, PurePosixPath
from typing import Any

import cv2
import numpy as np
from prepare_trcrv10_classifier_crops import write_crop

ROOT = Path(__file__).resolve().parents[1]
import sys

sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
from sanitation_learning.g4_data import (
    _random_background_crop,
    box_iou,
    square_crop,
)

SCHEMA_VERSION = "emfj6v3.classifier_holdout_gt.v1"
SOURCE_SPLIT = "G10_HOLDOUT"
TARGET_CLASSES = {
    1: "plastic_bottle",
    2: "metal_can",
    3: "paper_litter",
}
POSITIVE_PER_CLASS = 60
BACKGROUND_PER_NEGATIVE_FRAME = 1
SELECTION_SEED = 20260824
POSITIVE_CROP_SCALE = 4.0
MINIMUM_CROP_SIDE = 64
BACKGROUND_CROP_SIDE = 96
FORBIDDEN_MARKERS = ("G5", "G5_V2", "VAL_NEW", "DEV_VAL", "SEALED")
G10_DOMAIN_MANIFEST_SHA256 = (
    "3bdb3006226943e4149cd84144b488e5eb112ab35ad3692c5da8cc48c88b5208"
)
HOLDOUT_WORLD_SHA256 = {
    "g10v15_val_w01_07_service_road": (
        "b538734d3bf10e1614cc3e21f141dfd48c34b54262b3cc944abc515d3ac074b1"
    ),
    "g10v15_val_w02_08_mixed_curb_vegetation": (
        "4172b49e3e7b37e64bee36f5b48f3905ea119852329810d55352ec70e1bc6c99"
    ),
    "g10v15_val_w03_09_light_paver_pedestrian": (
        "f92e9130460c0fb77b8d325953d6c79cdf411dcf467ce87815ed11b1daecb2ed"
    ),
}
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
SCENE_NAME_PATTERN = re.compile(r"scene_[A-Za-z0-9][A-Za-z0-9._-]*")


class HoldoutContractError(ValueError):
    """Raised when an input violates the fail-closed HOLDOUT contract."""


def _exact_int(value: object, *, field: str, minimum: int | None = None) -> int:
    if type(value) is not int:
        raise HoldoutContractError(f"{field} must be an integer")
    if minimum is not None and value < minimum:
        raise HoldoutContractError(f"{field} must be >= {minimum}")
    return value


def _exact_bool(value: object, *, field: str) -> bool:
    if type(value) is not bool:
        raise HoldoutContractError(f"{field} must be a boolean")
    return value


def _exact_string(value: object, *, field: str) -> str:
    if not isinstance(value, str) or not value:
        raise HoldoutContractError(f"{field} must be a non-empty string")
    return value


def _exact_sha256(value: object, *, field: str) -> str:
    rendered = _exact_string(value, field=field)
    if SHA256_PATTERN.fullmatch(rendered) is None:
        raise HoldoutContractError(f"{field} must be a lowercase SHA256")
    return rendered


def _marker_pattern(marker: str) -> re.Pattern[str]:
    return re.compile(
        rf"(?:^|[^A-Z0-9]){re.escape(marker.upper())}(?:$|[^A-Z0-9])",
        re.IGNORECASE,
    )


FORBIDDEN_PATTERNS = tuple(
    (marker, _marker_pattern(marker)) for marker in FORBIDDEN_MARKERS
)


def _reject_forbidden(value: object, *, field: str) -> None:
    rendered = str(value)
    normalized_words = re.sub(r"[^A-Z0-9]+", "_", rendered.upper()).split("_")
    if any(word == "G5" or word.startswith("G5V2") for word in normalized_words):
        raise HoldoutContractError(f"forbidden marker 'G5' in {field}: {rendered}")
    for marker, pattern in FORBIDDEN_PATTERNS:
        if pattern.search(rendered):
            raise HoldoutContractError(
                f"forbidden marker {marker!r} in {field}: {rendered}"
            )


def _reject_string_values(value: object, *, field: str = "COCO") -> None:
    """Reject forbidden data references without rejecting false audit-field keys."""

    if isinstance(value, str):
        _reject_forbidden(value, field=field)
    elif isinstance(value, dict):
        for key, child in value.items():
            _reject_string_values(child, field=f"{field}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _reject_string_values(child, field=f"{field}[{index}]")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_sha256(value: object) -> str:
    encoded = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load_json(path: Path, *, field: str) -> dict[str, Any]:
    try:
        payload = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=lambda value: (_ for _ in ()).throw(
                ValueError(f"non-finite JSON constant {value}")
            ),
        )
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        raise HoldoutContractError(f"cannot read {field} JSON {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise HoldoutContractError(f"{field} JSON root must be an object: {path}")
    return payload


def _json_from_bytes(data: bytes, *, field: str, path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(
            data.decode("utf-8"),
            parse_constant=lambda value: (_ for _ in ()).throw(
                ValueError(f"non-finite JSON constant {value}")
            ),
        )
    except (UnicodeError, json.JSONDecodeError, ValueError) as exc:
        raise HoldoutContractError(f"cannot read {field} JSON {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise HoldoutContractError(f"{field} JSON root must be an object: {path}")
    return payload


def _require_inside(path: Path, root: Path, *, field: str) -> Path:
    candidate = path.resolve()
    _reject_forbidden(candidate, field=field)
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise HoldoutContractError(f"{field} escapes capture root: {candidate}") from exc
    if not candidate.is_file():
        raise HoldoutContractError(f"missing {field}: {candidate}")
    return candidate


def _scenes_root(capture_root: Path) -> Path:
    candidates = (
        capture_root / "g4_screening_native" / "scenes",
        capture_root / "scenes",
        capture_root,
    )
    valid = [path.resolve() for path in candidates if path.is_dir() and path.name == "scenes"]
    if len(set(valid)) != 1:
        raise HoldoutContractError(
            "capture root must identify exactly one g4_screening_native/scenes tree"
        )
    return valid[0]


def _canonical_frame_paths(
    *, root: Path, scenes: Path, scene_name: str, frame_index: int
) -> dict[str, Path]:
    _reject_forbidden(scene_name, field="scene")
    if SCENE_NAME_PATTERN.fullmatch(scene_name) is None:
        raise HoldoutContractError(f"invalid scene name: {scene_name!r}")
    scene = scenes / scene_name
    suffix = f"frame_{frame_index:02d}"
    raw = {
        "rgb": scene / "rgb" / f"{suffix}.png",
        "depth": scene / "depth" / f"{suffix}.npy",
        "camera": scene / "camera" / f"{suffix}.json",
        "semantic": scene / "semantic" / f"{suffix}.npy",
        "instance": scene / "instance" / f"{suffix}.npy",
        "scene_manifest": scene / "scene_manifest.json",
        "capture_report": scene / "capture_report.json",
    }
    return {
        name: _require_inside(path, root, field=f"{name} path")
        for name, path in raw.items()
    }


def _relative(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def _bbox_xyxy(annotation: dict[str, Any], *, image_id: int) -> tuple[float, ...]:
    bbox = annotation.get("bbox")
    if (
        not isinstance(bbox, list)
        or len(bbox) != 4
        or any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in bbox)
    ):
        raise HoldoutContractError(f"invalid bbox for image {image_id}: {bbox!r}")
    x, y, width, height = (float(value) for value in bbox)
    if not all(np.isfinite(value) for value in (x, y, width, height)) or width <= 0 or height <= 0:
        raise HoldoutContractError(f"non-positive/non-finite bbox for image {image_id}")
    return (x, y, x + width, y + height)


def _validate_coco(coco: dict[str, Any]) -> tuple[dict[int, dict], dict[int, list[dict]]]:
    _reject_string_values(coco)
    info = coco.get("info")
    if not isinstance(info, dict):
        raise HoldoutContractError("COCO info must be an object")
    if info.get("g10_domain_manifest_sha256") != G10_DOMAIN_MANIFEST_SHA256:
        raise HoldoutContractError("COCO is not bound to the frozen G10 domain manifest")
    if info.get("holdout_world_ids") != sorted(HOLDOUT_WORLD_SHA256):
        raise HoldoutContractError("COCO holdout_world_ids do not match the frozen allowlist")
    categories = coco.get("categories")
    if not isinstance(categories, list) or len(categories) != len(TARGET_CLASSES):
        raise HoldoutContractError("COCO categories must contain exactly three rows")
    observed_categories: dict[int, str] = {}
    for row in categories:
        if not isinstance(row, dict):
            raise HoldoutContractError("every COCO category must be an object")
        category_id = _exact_int(row.get("id"), field="category id", minimum=1)
        category_name = _exact_string(row.get("name"), field="category name")
        if category_id in observed_categories:
            raise HoldoutContractError(f"duplicate COCO category id: {category_id}")
        observed_categories[category_id] = category_name
    if observed_categories != TARGET_CLASSES:
        raise HoldoutContractError(
            f"COCO categories must exactly equal {TARGET_CLASSES}, got {observed_categories}"
        )
    raw_images = coco.get("images")
    raw_annotations = coco.get("annotations")
    if not isinstance(raw_images, list) or not raw_images:
        raise HoldoutContractError("COCO images must be a non-empty list")
    if not isinstance(raw_annotations, list) or not raw_annotations:
        raise HoldoutContractError("COCO annotations must be a non-empty list")
    images: dict[int, dict] = {}
    for row in raw_images:
        if not isinstance(row, dict):
            raise HoldoutContractError("every COCO image must be an object")
        image_id = _exact_int(row.get("id"), field="COCO image id", minimum=1)
        if image_id in images:
            raise HoldoutContractError(f"duplicate COCO image id: {image_id}")
        if row.get("source_split") != SOURCE_SPLIT:
            raise HoldoutContractError(
                f"image {image_id} source_split must be {SOURCE_SPLIT!r}"
            )
        scene = _exact_string(row.get("scene"), field=f"image {image_id} scene")
        if SCENE_NAME_PATTERN.fullmatch(scene) is None:
            raise HoldoutContractError(f"invalid scene name: {scene!r}")
        world_id = _exact_string(
            row.get("world_id"), field=f"image {image_id} world_id"
        )
        if world_id not in HOLDOUT_WORLD_SHA256:
            raise HoldoutContractError(f"image {image_id} uses a non-HOLDOUT world")
        _exact_int(row.get("scene_seed"), field=f"image {image_id} scene_seed", minimum=0)
        _exact_int(row.get("frame_index"), field=f"image {image_id} frame_index", minimum=0)
        _exact_bool(row.get("negative_only"), field=f"image {image_id} negative_only")
        _exact_int(row.get("width"), field=f"image {image_id} width", minimum=1)
        _exact_int(row.get("height"), field=f"image {image_id} height", minimum=1)
        mission_id = _exact_string(
            row.get("mission_id"), field=f"image {image_id} mission_id"
        )
        if mission_id != scene:
            raise HoldoutContractError("COCO mission_id must equal the capture scene")
        images[image_id] = row
    annotations: dict[int, list[dict]] = defaultdict(list)
    annotation_ids: set[int] = set()
    for row in raw_annotations:
        if not isinstance(row, dict):
            raise HoldoutContractError("every COCO annotation must be an object")
        annotation_id = row.get("id")
        image_id = row.get("image_id")
        category_id = row.get("category_id")
        annotation_id = _exact_int(annotation_id, field="annotation id", minimum=1)
        image_id = _exact_int(image_id, field="annotation image_id", minimum=1)
        category_id = _exact_int(category_id, field="annotation category_id", minimum=1)
        if annotation_id in annotation_ids:
            raise HoldoutContractError(f"duplicate COCO annotation id: {annotation_id}")
        if image_id not in images:
            raise HoldoutContractError(f"annotation references unknown image: {image_id}")
        if category_id not in TARGET_CLASSES:
            raise HoldoutContractError(f"unknown target category id: {category_id}")
        _bbox_xyxy(row, image_id=image_id)
        area = row.get("area")
        if (
            isinstance(area, bool)
            or not isinstance(area, (int, float))
            or not np.isfinite(area)
            or area <= 0
        ):
            raise HoldoutContractError("annotation area must be positive and finite")
        if row.get("iscrowd") != 0 or type(row.get("iscrowd")) is not int:
            raise HoldoutContractError("annotation iscrowd must be integer zero")
        _exact_int(
            row.get("bbox_short_side_px"),
            field="annotation bbox_short_side_px",
            minimum=1,
        )
        annotation_ids.add(annotation_id)
        annotations[image_id].append(row)
    for rows in annotations.values():
        rows.sort(key=lambda row: int(row["id"]))
    return images, annotations


def _validate_scene_contract(
    paths: dict[str, Path],
    image: dict[str, Any],
    manifest: dict[str, Any],
    report: dict[str, Any],
    source_hashes: dict[str, str],
) -> dict[str, Any]:
    if _exact_int(manifest.get("schema_version"), field="scene schema_version") != 2:
        raise HoldoutContractError("scene manifest schema_version must be 2")
    if manifest.get("split") != "val" or manifest.get("source_world_split") != "val":
        raise HoldoutContractError("scene manifest is not a G10 development HOLDOUT")
    if manifest.get("world_id") != image.get("world_id"):
        raise HoldoutContractError("COCO/scene world_id mismatch")
    world_id = _exact_string(manifest.get("world_id"), field="scene world_id")
    if world_id not in HOLDOUT_WORLD_SHA256:
        raise HoldoutContractError("scene world is outside the frozen HOLDOUT allowlist")
    if _exact_sha256(manifest.get("world_sha256"), field="scene world_sha256") != HOLDOUT_WORLD_SHA256[world_id]:
        raise HoldoutContractError("scene world_sha256 does not match the frozen domain")
    scene_seed = _exact_int(manifest.get("scene_seed"), field="scene scene_seed", minimum=0)
    if scene_seed != image.get("scene_seed"):
        raise HoldoutContractError("COCO/scene seed mismatch")
    negative_only = _exact_bool(manifest.get("negative_only"), field="scene negative_only")
    if negative_only != image.get("negative_only"):
        raise HoldoutContractError("COCO/scene negative_only mismatch")
    expected_trajectory = f"{world_id}_trajectory_{scene_seed}"
    if manifest.get("trajectory_id") != expected_trajectory:
        raise HoldoutContractError("scene trajectory_id does not match world/seed identity")
    contract = manifest.get("trcrv10_g10_approach_sequence")
    if not isinstance(contract, dict):
        raise HoldoutContractError("scene G10 approach contract is missing")
    if contract.get("enabled") is not True or contract.get("gt_runtime_forbidden") is not True:
        raise HoldoutContractError("scene G10 approach/runtime contract is not enabled")
    if contract.get("target_classes") != sorted(TARGET_CLASSES.values()):
        raise HoldoutContractError("scene G10 target_classes do not match the frozen classes")
    if _exact_int(
        contract.get("targets_per_positive_mission"),
        field="targets_per_positive_mission",
        minimum=1,
    ) != 1:
        raise HoldoutContractError("G10 approach must contain one target per positive mission")
    selected_target = contract.get("selected_target_class")
    if negative_only:
        if selected_target is not None:
            raise HoldoutContractError("negative mission selected_target_class must be null")
    elif selected_target not in TARGET_CLASSES.values():
        raise HoldoutContractError("positive mission selected_target_class is invalid")
    target_counts = manifest.get("target_count_by_class")
    if not isinstance(target_counts, dict) or set(target_counts) != set(TARGET_CLASSES.values()):
        raise HoldoutContractError("scene target_count_by_class keys are invalid")
    normalized_counts = {
        name: _exact_int(target_counts[name], field=f"target_count_by_class.{name}", minimum=0)
        for name in TARGET_CLASSES.values()
    }
    expected_counts = {
        name: 0 if negative_only or name != selected_target else 1
        for name in TARGET_CLASSES.values()
    }
    if normalized_counts != expected_counts:
        raise HoldoutContractError("scene target_count_by_class violates the G10 mission")
    objects = manifest.get("objects")
    if not isinstance(objects, list) or any(not isinstance(row, dict) for row in objects):
        raise HoldoutContractError("scene objects must be a list of objects")
    target_objects = [row for row in objects if row.get("class_id") in TARGET_CLASSES.values()]
    if negative_only:
        if target_objects:
            raise HoldoutContractError("negative mission contains target objects")
    else:
        if len(target_objects) != 1 or target_objects[0].get("class_id") != selected_target:
            raise HoldoutContractError("positive mission object does not match selected target")
        expected_label = next(
            category_id for category_id, name in TARGET_CLASSES.items() if name == selected_target
        )
        semantic_label = _exact_int(
            target_objects[0].get("semantic_label"), field="target object semantic_label"
        )
        if semantic_label != expected_label:
            raise HoldoutContractError("positive mission object semantic_label is invalid")

    if _exact_int(report.get("schema_version"), field="capture schema_version") != 4:
        raise HoldoutContractError("capture report schema_version must be 4")
    if report.get("capture_pass") is not True:
        raise HoldoutContractError("scene capture did not pass")
    captured = _exact_int(report.get("captured_frames"), field="captured_frames", minimum=1)
    requested = _exact_int(report.get("requested_frames"), field="requested_frames", minimum=1)
    if captured != requested:
        raise HoldoutContractError("scene capture is partial or has invalid frame counts")
    sync = report.get("sensor_odom_sync")
    if not isinstance(sync, dict) or sync.get("pass") is not True:
        raise HoldoutContractError("capture sensor_odom_sync did not pass")
    maximum_skew = _exact_int(
        sync.get("maximum_skew_ns"), field="maximum sensor/odom skew", minimum=0
    )
    gate_skew = _exact_int(
        sync.get("gate_maximum_skew_ns"), field="sensor/odom skew gate", minimum=0
    )
    if maximum_skew > gate_skew:
        raise HoldoutContractError("capture sensor/odom skew exceeds its gate")
    records = report.get("records")
    if not isinstance(records, list) or len(records) != captured:
        raise HoldoutContractError("capture records must exactly cover captured_frames")
    record_indices = []
    for record in records:
        if not isinstance(record, dict):
            raise HoldoutContractError("every capture record must be an object")
        record_indices.append(
            _exact_int(record.get("frame_index"), field="capture record frame_index", minimum=0)
        )
    if record_indices != list(range(captured)):
        raise HoldoutContractError("capture records are not unique contiguous frame identities")
    frame_index = image["frame_index"]
    if frame_index < 0 or frame_index >= captured:
        raise HoldoutContractError("COCO frame index is outside the complete capture")
    record = records[frame_index]
    if record.get("exact_four_sensor_timestamp") is not True:
        raise HoldoutContractError("capture record lacks exact four-sensor timestamp")
    for field in ("timestamp_ns", "odom_timestamp_ns", "sensor_odom_skew_ns"):
        _exact_int(record.get(field), field=f"capture record {field}", minimum=0)
    record_paths = record.get("paths")
    expected_record_paths = {
        "rgb": f"rgb/frame_{frame_index:02d}.png",
        "depth": f"depth/frame_{frame_index:02d}.npy",
        "semantic": f"semantic/frame_{frame_index:02d}.npy",
        "instance": f"instance/frame_{frame_index:02d}.npy",
        "camera": f"camera/frame_{frame_index:02d}.json",
        "tf": f"tf/frame_{frame_index:02d}.json",
        "capture": f"capture/frame_{frame_index:02d}.json",
    }
    if record_paths != expected_record_paths:
        raise HoldoutContractError("capture record paths do not match the canonical frame")
    scene_dir = paths["rgb"].parents[1]
    for name in ("tf", "capture"):
        _require_inside(
            scene_dir.joinpath(*PurePosixPath(record_paths[name]).parts),
            scene_dir,
            field=f"capture record {name} path",
        )
    if _exact_sha256(record.get("rgb_sha256"), field="capture record rgb_sha256") != source_hashes["rgb"]:
        raise HoldoutContractError("capture record rgb_sha256 does not match RGB bytes")
    return {"captured_frames": captured, "selected_target_class": selected_target}


def _validate_declared_paths(
    image: dict[str, Any], paths: dict[str, Path], root: Path
) -> None:
    declared = {
        "file_name": "rgb",
        "depth_file_name": "depth",
        "camera_file_name": "camera",
        "semantic_file_name": "semantic",
        "instance_file_name": "instance",
        "scene_manifest": "scene_manifest",
        "capture_report": "capture_report",
    }
    for field, name in declared.items():
        value = image.get(field)
        if not isinstance(value, str) or not value:
            raise HoldoutContractError(f"missing COCO {field}")
        if "\\" in value or re.match(r"^[A-Za-z]:", value) or value.startswith("//"):
            raise HoldoutContractError(
                f"COCO {field} must be a capture-root-relative POSIX path"
            )
        relative = PurePosixPath(value)
        if relative.is_absolute() or any(part in ("", ".", "..") for part in relative.parts):
            raise HoldoutContractError(
                f"COCO {field} must be a capture-root-relative POSIX path"
            )
        if relative.as_posix() != value:
            raise HoldoutContractError(f"COCO {field} is not a canonical POSIX path")
        candidate = root.joinpath(*relative.parts)
        if _require_inside(candidate, root, field=field) != paths[name]:
            raise HoldoutContractError(f"COCO {field} does not match canonical capture path")


def _selection_rank(class_name: str, image: dict, annotation: dict) -> str:
    identity = "|".join(
        (
            str(SELECTION_SEED),
            class_name,
            str(image["world_id"]),
            str(image["scene"]),
            str(image["scene_seed"]),
            str(image["frame_index"]),
            str(annotation["id"]),
        )
    )
    return hashlib.sha256(identity.encode("utf-8")).hexdigest()


SOURCE_NAMES = (
    "rgb",
    "depth",
    "camera",
    "semantic",
    "instance",
    "scene_manifest",
    "capture_report",
)


def _load_modalities(
    paths: dict[str, Path], image: dict[str, Any]
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, str], dict[str, Any], dict[str, Any]]:
    source_bytes: dict[str, bytes] = {}
    source_hashes: dict[str, str] = {}
    for name in SOURCE_NAMES:
        try:
            data = paths[name].read_bytes()
        except OSError as exc:
            raise HoldoutContractError(f"cannot read {name}: {paths[name]}: {exc}") from exc
        source_bytes[name] = data
        source_hashes[name] = hashlib.sha256(data).hexdigest()
    rgb = cv2.imdecode(np.frombuffer(source_bytes["rgb"], dtype=np.uint8), cv2.IMREAD_COLOR)
    if rgb is None or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise HoldoutContractError(f"cannot decode RGB: {paths['rgb']}")
    try:
        depth = np.load(io.BytesIO(source_bytes["depth"]), allow_pickle=False)
        semantic = np.load(io.BytesIO(source_bytes["semantic"]), allow_pickle=False)
        instance = np.load(io.BytesIO(source_bytes["instance"]), allow_pickle=False)
    except (OSError, ValueError) as exc:
        raise HoldoutContractError(f"cannot safely load frame tensors: {exc}") from exc
    shape = rgb.shape[:2]
    if any(array.ndim != 2 or array.shape != shape for array in (depth, semantic, instance)):
        raise HoldoutContractError("RGB/depth/semantic/instance shapes do not match")
    if not np.issubdtype(semantic.dtype, np.integer) or not np.issubdtype(instance.dtype, np.integer):
        raise HoldoutContractError("semantic and instance tensors must have integer dtype")
    if depth.dtype == object or not np.issubdtype(depth.dtype, np.number):
        raise HoldoutContractError("depth tensor must have numeric dtype")
    if (image["height"], image["width"]) != shape:
        raise HoldoutContractError("COCO image dimensions do not match RGB")
    camera = _json_from_bytes(source_bytes["camera"], field="camera", path=paths["camera"])
    if not camera:
        raise HoldoutContractError("camera JSON must not be empty")
    manifest = _json_from_bytes(
        source_bytes["scene_manifest"], field="scene manifest", path=paths["scene_manifest"]
    )
    report = _json_from_bytes(
        source_bytes["capture_report"], field="capture report", path=paths["capture_report"]
    )
    return rgb, semantic, instance, source_hashes, manifest, report


def _verify_annotation_mask(
    semantic: np.ndarray,
    instance: np.ndarray,
    annotation: dict[str, Any],
    bbox: tuple[float, ...],
) -> None:
    category_id = annotation["category_id"]
    ys, xs = np.where(semantic == category_id)
    if not len(xs):
        raise HoldoutContractError("GT annotation has no matching semantic pixels")
    expected = (int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1))
    observed = tuple(bbox)
    if observed != expected:
        raise HoldoutContractError(
            f"COCO bbox does not equal semantic GT extent: {observed} != {expected}"
        )
    if annotation["area"] != len(xs):
        raise HoldoutContractError("COCO area does not equal semantic GT pixel count")
    if annotation["bbox_short_side_px"] != min(expected[2] - expected[0], expected[3] - expected[1]):
        raise HoldoutContractError("COCO bbox_short_side_px does not match semantic GT")
    instance_ids = np.unique(instance[semantic == category_id])
    nonzero_ids = [int(value) for value in instance_ids if int(value) != 0]
    if len(nonzero_ids) != 1 or len(instance_ids) != 1:
        raise HoldoutContractError(
            "target semantic pixels must correspond to exactly one nonzero instance"
        )


def _verify_frame_gt(
    semantic: np.ndarray,
    instance: np.ndarray,
    image: dict[str, Any],
    image_annotations: list[dict],
    selected_target_class: str | None,
) -> None:
    semantic_categories = {
        category_id for category_id in TARGET_CLASSES if np.any(semantic == category_id)
    }
    annotation_categories = {row["category_id"] for row in image_annotations}
    if len(annotation_categories) != len(image_annotations):
        raise HoldoutContractError("a frame may contain at most one annotation per target class")
    if image["negative_only"] and semantic_categories:
        raise HoldoutContractError("negative-only frame contains target semantic pixels")
    if semantic_categories != annotation_categories:
        raise HoldoutContractError("COCO annotations do not exactly match semantic target IDs")
    if image["negative_only"]:
        return
    expected_category = next(
        category_id for category_id, name in TARGET_CLASSES.items() if name == selected_target_class
    )
    if any(category_id != expected_category for category_id in semantic_categories):
        raise HoldoutContractError("positive frame contains a non-mission target class")
    for annotation in image_annotations:
        _verify_annotation_mask(
            semantic,
            instance,
            annotation,
            _bbox_xyxy(annotation, image_id=image["id"]),
        )


def build_dataset(
    coco_path: Path,
    capture_root: Path,
    output_dir: Path,
) -> dict[str, Any]:
    coco_path = coco_path.resolve()
    capture_root = capture_root.resolve()
    output_dir = output_dir.resolve()
    for value, field in (
        (coco_path, "COCO path"),
        (capture_root, "capture root"),
        (output_dir, "output path"),
    ):
        _reject_forbidden(value, field=field)
    if not coco_path.is_file():
        raise HoldoutContractError(f"missing COCO: {coco_path}")
    if not capture_root.is_dir():
        raise HoldoutContractError(f"missing capture root: {capture_root}")
    if output_dir.exists():
        raise HoldoutContractError(f"output directory already exists: {output_dir}")
    scenes = _scenes_root(capture_root)
    coco = _load_json(coco_path, field="COCO")
    images, annotations = _validate_coco(coco)
    if {row["world_id"] for row in images.values()} != set(HOLDOUT_WORLD_SHA256):
        raise HoldoutContractError(
            "COCO images do not cover the exact frozen HOLDOUT world set"
        )

    prepared: dict[int, dict[str, Any]] = {}
    candidates: dict[str, list[tuple[str, int, dict]]] = defaultdict(list)
    negative_image_ids: list[int] = []
    negative_scenes: set[str] = set()
    seen_source_identities: set[tuple] = set()
    frame_indices_by_scene: dict[str, set[int]] = defaultdict(set)
    expected_frames_by_scene: dict[str, int] = {}
    for image_id, image in sorted(images.items()):
        identity = (
            image["world_id"],
            image["scene"],
            image["scene_seed"],
            image["frame_index"],
        )
        if identity in seen_source_identities:
            raise HoldoutContractError(f"duplicate source frame identity: {identity}")
        seen_source_identities.add(identity)
        paths = _canonical_frame_paths(
            root=capture_root,
            scenes=scenes,
            scene_name=image["scene"],
            frame_index=image["frame_index"],
        )
        _validate_declared_paths(image, paths, capture_root)
        rgb, semantic, instance, source_hashes, manifest, report = _load_modalities(paths, image)
        scene_contract = _validate_scene_contract(
            paths, image, manifest, report, source_hashes
        )
        captured_frames = scene_contract["captured_frames"]
        scene_name = image["scene"]
        prior_expected = expected_frames_by_scene.setdefault(scene_name, captured_frames)
        if prior_expected != captured_frames:
            raise HoldoutContractError("inconsistent capture frame count within scene")
        frame_indices_by_scene[scene_name].add(image["frame_index"])
        image_annotations = annotations.get(image_id, [])
        _verify_frame_gt(
            semantic,
            instance,
            image,
            image_annotations,
            scene_contract["selected_target_class"],
        )
        prepared[image_id] = {
            "image": image,
            "paths": paths,
            "source_hashes": source_hashes,
        }
        if image["negative_only"]:
            negative_image_ids.append(image_id)
            negative_scenes.add(image["scene"])
        else:
            for annotation in image_annotations:
                class_name = TARGET_CLASSES[annotation["category_id"]]
                candidates[class_name].append(
                    (_selection_rank(class_name, image, annotation), image_id, annotation)
                )

    capture_scene_names = {
        path.name for path in scenes.glob("scene_*") if path.is_dir()
    }
    coco_scene_names = set(frame_indices_by_scene)
    if capture_scene_names != coco_scene_names:
        raise HoldoutContractError(
            "COCO scene set does not exactly match the explicit capture root"
        )
    for scene_name, expected_frames in sorted(expected_frames_by_scene.items()):
        if frame_indices_by_scene[scene_name] != set(range(expected_frames)):
            raise HoldoutContractError(
                f"COCO does not contain every captured frame for scene {scene_name}"
            )

    if not negative_scenes or not negative_image_ids:
        raise HoldoutContractError("at least one negative-only scene/frame is required")
    selected: list[tuple[str, int, dict | None, int]] = []
    for class_name in TARGET_CLASSES.values():
        ranked = sorted(candidates[class_name], key=lambda item: (item[0], item[1], int(item[2]["id"])))
        if len(ranked) < POSITIVE_PER_CLASS:
            raise HoldoutContractError(
                f"{class_name} has {len(ranked)} candidates; {POSITIVE_PER_CLASS} required"
            )
        selected.extend((class_name, image_id, annotation, 0) for _, image_id, annotation in ranked[:POSITIVE_PER_CLASS])
    for image_id in sorted(negative_image_ids):
        for crop_index in range(BACKGROUND_PER_NEGATIVE_FRAME):
            selected.append(("background_or_unknown", image_id, None, crop_index))

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    lock_path = output_dir.parent / f".{output_dir.name}.lock"
    try:
        lock_fd = os.open(lock_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    except FileExistsError as exc:
        raise HoldoutContractError(f"concurrent output writer lock exists: {lock_path}") from exc
    staging: Path | None = None
    try:
        os.write(lock_fd, f"pid={os.getpid()}\n".encode("ascii"))
        if output_dir.exists():
            raise HoldoutContractError(f"output directory already exists: {output_dir}")
        staging = Path(
            tempfile.mkdtemp(
                prefix=f".{output_dir.name}.staging-", dir=output_dir.parent
            )
        )
        records = []
        for class_name, image_id, annotation, crop_index in sorted(
            selected,
            key=lambda row: (
                row[0],
                prepared[row[1]]["image"]["world_id"],
                prepared[row[1]]["image"]["scene"],
                prepared[row[1]]["image"]["frame_index"],
                row[2]["id"] if row[2] else row[3],
            ),
        ):
            image = prepared[image_id]["image"]
            paths = prepared[image_id]["paths"]
            rgb, semantic, instance, source_hashes, _manifest, _report = _load_modalities(
                paths, image
            )
            if source_hashes != prepared[image_id]["source_hashes"]:
                raise HoldoutContractError("source changed after full-frame validation")
            height, width = rgb.shape[:2]
            if annotation is not None:
                native_bbox = _bbox_xyxy(annotation, image_id=image_id)
                _verify_annotation_mask(semantic, instance, annotation, native_bbox)
                crop_bbox = square_crop(
                    width,
                    height,
                    native_bbox,
                    scale=POSITIVE_CROP_SCALE,
                    minimum_side=MINIMUM_CROP_SIDE,
                )
                annotation_id: int | None = annotation["id"]
            else:
                rng_material = (
                    f"{SELECTION_SEED}|{image['world_id']}|{image['scene']}|"
                    f"{image['scene_seed']}|{image['frame_index']}|{crop_index}"
                )
                rng = random.Random(int(hashlib.sha256(rng_material.encode()).hexdigest(), 16))
                crop_bbox = _random_background_crop(
                    width, height, [], rng, side=BACKGROUND_CROP_SIDE
                )
                annotation_id = None
            gt_boxes = [
                _bbox_xyxy(row, image_id=image_id)
                for row in annotations.get(image_id, [])
            ]
            if class_name == "background_or_unknown" and any(
                box_iou(box, crop_bbox) >= 0.1 for box in gt_boxes
            ):
                raise HoldoutContractError("background crop overlaps GT at IoU >= 0.1")
            source_identity = {
                "source_split": SOURCE_SPLIT,
                "image_id": image_id,
                "annotation_id": annotation_id,
                "world_id": image["world_id"],
                "scene": image["scene"],
                "scene_seed": image["scene_seed"],
                "frame_index": image["frame_index"],
                "negative_only": image["negative_only"],
            }
            source_identity_sha256 = _canonical_sha256(
                {"identity": source_identity, "sha256": source_hashes}
            )
            record_id = "emf-holdout-" + _canonical_sha256(
                {
                    "source_identity_sha256": source_identity_sha256,
                    "class_name": class_name,
                    "crop_bbox_xyxy": list(crop_bbox),
                }
            )[:24]
            relative_crop = Path("crops") / class_name / f"{record_id}.png"
            crop_path = staging / relative_crop
            write_crop(rgb, list(crop_bbox), crop_path)
            records.append(
                {
                    "record_id": record_id,
                    "class_name": class_name,
                    "crop_path": relative_crop.as_posix(),
                    "crop_sha256": _sha256_file(crop_path),
                    "crop_bbox_xyxy": list(crop_bbox),
                    "source_identity": source_identity,
                    "source_identity_sha256": source_identity_sha256,
                    "source_paths": {
                        name: _relative(paths[name], capture_root)
                        for name in (
                            "rgb",
                            "depth",
                            "camera",
                            "semantic",
                            "instance",
                            "scene_manifest",
                            "capture_report",
                        )
                    },
                    "source_sha256": source_hashes,
                    "offline_gt_development_only": True,
                    "production_runtime_eligible": False,
                }
            )
        records.sort(key=lambda row: row["record_id"])
        record_ids = [row["record_id"] for row in records]
        if len(set(record_ids)) != len(record_ids):
            raise HoldoutContractError("record_id collision")
        counts = Counter(row["class_name"] for row in records)
        required_counts = {
            class_name: POSITIVE_PER_CLASS for class_name in TARGET_CLASSES.values()
        }
        required_counts["background_or_unknown"] = (
            len(negative_image_ids) * BACKGROUND_PER_NEGATIVE_FRAME
        )
        if dict(counts) != required_counts or any(value <= 0 for value in counts.values()):
            raise HoldoutContractError(
                f"output class counts do not match fixed contract: {dict(counts)}"
            )
        payload: dict[str, Any] = {
            "schema_version": SCHEMA_VERSION,
            "protocol_id": "EMFJ6V3",
            "stage": "CLASSIFIER_HOLDOUT_OFFLINE_GT_DEVELOPMENT",
            "source_split": SOURCE_SPLIT,
            "g10_domain_manifest_sha256": G10_DOMAIN_MANIFEST_SHA256,
            "holdout_world_ids": sorted(HOLDOUT_WORLD_SHA256),
            "input_coco_sha256": _sha256_file(coco_path),
            "all_validated_source_frames_sha256": _canonical_sha256(
                [
                    {
                        "image_id": image_id,
                        "identity": [
                            prepared[image_id]["image"][field]
                            for field in (
                                "world_id",
                                "scene",
                                "scene_seed",
                                "frame_index",
                                "negative_only",
                            )
                        ],
                        "sha256": prepared[image_id]["source_hashes"],
                    }
                    for image_id in sorted(prepared)
                ]
            ),
            "selection_contract": {
                "seed": SELECTION_SEED,
                "positive_per_class": POSITIVE_PER_CLASS,
                "background_per_negative_frame": BACKGROUND_PER_NEGATIVE_FRAME,
                "positive_crop_scale": POSITIVE_CROP_SCALE,
                "minimum_crop_side": MINIMUM_CROP_SIDE,
                "background_crop_side": BACKGROUND_CROP_SIDE,
                "background_max_gt_iou_exclusive": 0.1,
            },
            "counts": dict(sorted(counts.items())),
            "negative_only_scene_count": len(negative_scenes),
            "negative_only_frame_count": len(negative_image_ids),
            "records": records,
            "identity_lock_sha256": _canonical_sha256(records),
            "offline_gt_development_only": True,
            "production_runtime_gt_forbidden": True,
            "training_performed": False,
            "threshold_selected": False,
            "threshold_frozen": False,
            "formal_product_evidence": False,
            "atomic_output_contract": {
                "visibility": "same_filesystem_directory_rename",
                "concurrent_writer_policy": "exclusive_sibling_lock_required",
                "power_loss_durability_guaranteed": False,
            },
            "pass": True,
        }
        payload["canonical_manifest_sha256"] = _canonical_sha256(payload)
        manifest_path = staging / "CLASSIFIER_HOLDOUT_GT_MANIFEST.json"
        manifest_path.write_text(
            json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(staging, output_dir)
        return payload
    except Exception:
        if staging is not None:
            shutil.rmtree(staging, ignore_errors=True)
        raise
    finally:
        os.close(lock_fd)
        try:
            lock_path.unlink()
        except FileNotFoundError:
            pass


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--coco", type=Path, required=True)
    parser.add_argument("--capture-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    payload = build_dataset(args.coco, args.capture_root, args.output_dir)
    print(
        json.dumps(
            {
                "counts": payload["counts"],
                "identity_lock_sha256": payload["identity_lock_sha256"],
                "pass": payload["pass"],
            },
            sort_keys=True,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
