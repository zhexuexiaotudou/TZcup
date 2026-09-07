"""Evaluator-only exact-stamp labels for public-Gazebo DOSOD evidence.

This is intentionally a data sidecar.  It receives no detector result and
exposes no ROS/control API.  SegmentationCamera emits its semantic label in
the final byte and its panoptic instance count in the first two bytes; the
caller must bind the observed bridge byte order before it reaches this module.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Any

import numpy as np


CLASS_BY_LABEL = {1: "litter_cube", 2: "fallen_leaves", 3: "dust_or_soil", 4: "puddle"}


class SidecarRejected(ValueError):
    pass


def _rgb(value: np.ndarray, name: str) -> np.ndarray:
    image = np.asarray(value)
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise SidecarRejected(f"{name}_must_be_uint8_rgb")
    return image


def decode_semantic_rgb(value: np.ndarray) -> np.ndarray:
    """Read Gazebo SegmentationCamera's semantic label from byte channel 2."""
    image = _rgb(value, "semantic")
    labels = image[:, :, 2]
    if not set(np.unique(labels).tolist()).issubset({0, *CLASS_BY_LABEL}):
        raise SidecarRejected("semantic_label_unknown")
    return labels


def decode_instance_rgb(value: np.ndarray) -> np.ndarray:
    """Return ``(semantic_label, uint16_instance_count)`` per native byte order."""
    image = _rgb(value, "instance")
    values = image.astype(np.uint16)
    return image[:, :, 2], values[:, :, 1] * 256 + values[:, :, 0]


def build_sidecar(*, identity: dict[str, Any], semantic_rgb: np.ndarray, instance_rgb: np.ndarray) -> dict[str, Any]:
    required = {"generation_nonce", "episode_manifest_sha256", "rgb_source_sha256", "rgb_stamp_ns", "semantic_stamp_ns", "instance_stamp_ns"}
    if set(identity) != required:
        raise SidecarRejected("sidecar_identity_keyset_invalid")
    if not isinstance(identity["generation_nonce"], str) or len(identity["generation_nonce"]) != 32:
        raise SidecarRejected("sidecar_nonce_invalid")
    for name in ("episode_manifest_sha256", "rgb_source_sha256"):
        if not isinstance(identity[name], str) or len(identity[name]) != 64:
            raise SidecarRejected("sidecar_hash_invalid")
    stamps = [identity[name] for name in ("rgb_stamp_ns", "semantic_stamp_ns", "instance_stamp_ns")]
    if not all(isinstance(value, int) and not isinstance(value, bool) and value > 0 for value in stamps) or len(set(stamps)) != 1:
        raise SidecarRejected("sidecar_stamp_not_exact")
    semantic = decode_semantic_rgb(semantic_rgb)
    instance_labels, instance_counts = decode_instance_rgb(instance_rgb)
    if semantic.shape != instance_counts.shape:
        raise SidecarRejected("sidecar_label_dimensions_mismatch")
    if not np.array_equal(semantic, instance_labels):
        raise SidecarRejected("sidecar_instance_semantic_mismatch")
    boxes: list[dict[str, Any]] = []
    keys = sorted((int(label), int(count)) for label, count in np.unique(
        np.stack((semantic, instance_counts), axis=-1).reshape(-1, 2), axis=0
    ) if label and count)
    for label, instance_count in keys:
        pixels = (semantic == label) & (instance_counts == instance_count)
        if label == 0:
            continue
        if label not in CLASS_BY_LABEL:
            raise SidecarRejected("semantic_label_unknown")
        y, x = np.where(pixels)
        boxes.append({
            "instance_id": instance_count,
            "class_id": CLASS_BY_LABEL[label],
            "label": label,
            "xyxy": [int(x.min()), int(y.min()), int(x.max()) + 1, int(y.max()) + 1],
        })
    return {
        "schema_version": 1,
        "classification": "EVALUATOR_ONLY_PUBLIC_GAZEBO_GT",
        **identity,
        "image_width": int(semantic.shape[1]),
        "image_height": int(semantic.shape[0]),
        "boxes": boxes,
    }


def validate_sidecar_identity(value: dict[str, Any], identity: dict[str, Any], width: int, height: int) -> None:
    if not isinstance(value, dict) or any(value.get(key) != expected for key, expected in identity.items()):
        raise SidecarRejected("sidecar_identity_binding_invalid")
    if value.get("schema_version") != 1 or value.get("classification") != "EVALUATOR_ONLY_PUBLIC_GAZEBO_GT":
        raise SidecarRejected("sidecar_schema_invalid")
    if value.get("image_width") != width or value.get("image_height") != height or not isinstance(value.get("boxes"), list):
        raise SidecarRejected("sidecar_dimensions_or_boxes_invalid")
    for box in value["boxes"]:
        if not isinstance(box, dict) or box.get("class_id") not in CLASS_BY_LABEL.values() or box.get("label") not in CLASS_BY_LABEL or CLASS_BY_LABEL[box["label"]] != box["class_id"]:
            raise SidecarRejected("sidecar_box_class_invalid")


def write_sidecar(path: Path, value: dict[str, Any]) -> dict[str, Any]:
    if path.is_symlink() or path.exists():
        raise SidecarRejected("sidecar_destination_not_fresh_regular")
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.parent.is_symlink():
        raise SidecarRejected("sidecar_destination_symlink")
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8") + b"\n"
    pending = path.with_name(f".{path.name}.pending.{os.getpid()}")
    pending.write_bytes(encoded)
    os.replace(pending, path)
    return {"relative_path": str(path.name), "sha256": hashlib.sha256(encoded).hexdigest(), "byte_size": len(encoded)}
