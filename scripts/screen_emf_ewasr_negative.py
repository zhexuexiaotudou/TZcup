#!/usr/bin/env python3
"""Run a semantics-preserving eWaSR negative-only development diagnostic."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
from collections.abc import Callable
from pathlib import Path

import cv2
import numpy as np

EXPECTED_MODEL_SHA256 = (
    "15e520fc5f7e910c9367556db9a9c1107bdc9c74e7982b3bf89c7050509411d7"
)
MODEL_INPUT_NAME = "image"
MODEL_INPUT_SHAPE = (1, 3, 384, 512)
MODEL_OUTPUT_NAME = "prediction"
MODEL_OUTPUT_SHAPE = (1, 3, 96, 128)
MODEL_CLASS_ORDER = ("obstacles_or_environment", "water", "sky")
AREA_DATASET_SCHEMA = "emfj6v3.area_dataset_manifest.v1"
AREA_MANIFEST_SHA256_ALLOWLIST = frozenset(
    {
        (
            "8c9f4c06bcf2a59a3ce15bc53c716c6411945b92870eaee18789bf4ddc291720",
            "056a3b599e8b2b3aa5de141a0e6234ec6b1dbe1ed561c999b7e43123a827828e",
        ),
        (
            "767edad22fcbe1d52666188c7d1a803e34e93ac97225649b55f70488fd22f2f5",
            "5d52c3af6c55ed73dcdec0bd4b587c7ea7b5169b289142d3537d690d25f5e72f",
        ),
    }
)
STRICT_NEGATIVE_IDENTITY_SHA_PAIRS = frozenset(
    {
        (
            "767edad22fcbe1d52666188c7d1a803e34e93ac97225649b55f70488fd22f2f5",
            "5d52c3af6c55ed73dcdec0bd4b587c7ea7b5169b289142d3537d690d25f5e72f",
        )
    }
)
ALLOWED_SEMANTIC_IDS = frozenset(range(6))
IMAGENET_MEAN = np.asarray((0.485, 0.456, 0.406), dtype=np.float32)
IMAGENET_STD = np.asarray((0.229, 0.224, 0.225), dtype=np.float32)
TARGET_LABEL_CONTRACT = {"leaf_pile": 4, "puddle": 5}
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _normalized_token(value: str) -> str:
    return re.sub(r"[^A-Z0-9]+", "_", value.upper()).strip("_")


def validate_nonsealed_value(value: str, *, field: str = "value") -> None:
    """Reject every forbidden dataset family, including separator variants."""
    normalized = _normalized_token(value)
    padded = f"_{normalized}_"
    words = normalized.split("_") if normalized else []
    matched = []
    if any(word == "G5" or word.startswith("G5V2") for word in words):
        matched.append("G5")
    if "_VAL_NEW_" in padded or normalized == "VAL_NEW":
        matched.append("VAL_NEW")
    if "_DEV_VAL_" in padded or normalized == "DEV_VAL":
        matched.append("DEV_VAL")
    if any(word == "SEALED" or word.startswith("SEALED") for word in words):
        matched.append("SEALED")
    if matched:
        raise ValueError(f"forbidden source in {field}: {sorted(set(matched))}")


def _canonical_manifest_sha256(payload: dict) -> str:
    unsigned = dict(payload)
    unsigned.pop("manifest_sha256", None)
    encoded = json.dumps(
        unsigned,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _reject_symlink_chain(path: Path, *, field: str) -> None:
    """Reject a symlink at any existing component without resolving through it."""
    absolute = Path(os.path.abspath(path))
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current /= part
        if current.is_symlink():
            raise ValueError(f"symlinked input is not allowed: {field}")


def _resolve_input_path(manifest: Path, raw_path: object, *, field: str) -> Path:
    if not isinstance(raw_path, str) or not raw_path:
        raise ValueError(f"{field} must be a non-empty path string")
    validate_nonsealed_value(raw_path, field=field)
    candidate = Path(raw_path)
    if not candidate.is_absolute():
        candidate = manifest.parent / candidate
    _reject_symlink_chain(candidate, field=field)
    resolved = candidate.resolve(strict=True)
    validate_nonsealed_value(str(resolved), field=field)
    if not resolved.is_file():
        raise ValueError(f"{field} must identify a regular file")
    return resolved


def _resolve_root_child(root: Path, raw_path: object, *, field: str) -> Path:
    if not isinstance(raw_path, str) or not raw_path:
        raise ValueError(f"{field} must be a non-empty relative path string")
    validate_nonsealed_value(raw_path, field=field)
    relative = Path(raw_path)
    if relative.is_absolute():
        raise ValueError(f"{field} must be relative to its selected source root")
    candidate = root / relative
    _reject_symlink_chain(candidate, field=field)
    resolved = candidate.resolve(strict=True)
    validate_nonsealed_value(str(resolved), field=field)
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"{field} escapes its selected source root") from exc
    if not resolved.is_file():
        raise ValueError(f"{field} must identify a regular file")
    return resolved


def _locked_sha(record: dict, key: str) -> str:
    value = record.get(key)
    if not isinstance(value, str) or not SHA256_PATTERN.fullmatch(value.lower()):
        raise ValueError(f"record {key} must be a lowercase SHA-256")
    return value.lower()


def _validate_negative_record(
    *,
    record_id: str,
    rgb: Path,
    semantic: Path,
    rgb_sha: str,
    semantic_sha: str,
    seen_rgb_hashes: set[str],
) -> dict:
    if sha256(rgb) != rgb_sha:
        raise ValueError(f"RGB SHA-256 mismatch for record {record_id}")
    if sha256(semantic) != semantic_sha:
        raise ValueError(f"semantic SHA-256 mismatch for record {record_id}")
    if rgb_sha in seen_rgb_hashes:
        raise ValueError("duplicate RGB SHA-256 would inflate negative screening")
    seen_rgb_hashes.add(rgb_sha)

    bgr = cv2.imread(str(rgb), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError(f"failed to decode RGB for record {record_id}")
    labels = np.load(semantic, allow_pickle=False)
    if labels.ndim != 2 or not np.issubdtype(labels.dtype, np.integer):
        raise ValueError(f"semantic labels are not a 2-D integer array: {record_id}")
    if labels.shape != bgr.shape[:2]:
        raise ValueError(f"RGB/semantic dimensions differ for record {record_id}")
    observed_labels = {int(label) for label in np.unique(labels).tolist()}
    unknown_labels = sorted(observed_labels.difference(ALLOWED_SEMANTIC_IDS))
    if unknown_labels:
        raise ValueError(
            f"negative-only record contains unknown semantic IDs: {record_id} "
            f"{unknown_labels}"
        )
    forbidden_labels = sorted(
        int(label)
        for label in TARGET_LABEL_CONTRACT.values()
        if np.any(labels == label)
    )
    if forbidden_labels:
        raise ValueError(
            f"negative-only record contains target semantic labels: {record_id} "
            f"{forbidden_labels}"
        )
    return {
        "record_id": record_id,
        "rgb": rgb,
        "rgb_sha256": rgb_sha,
        "semantic": semantic,
        "semantic_sha256": semantic_sha,
        "rgb_bgr": bgr,
    }


def _load_area_dataset_manifest(
    manifest: Path,
    payload: dict,
    source_root_id: str | None,
    source_root_path: str | Path | None,
) -> list[dict]:
    if payload.get("protocol_id") != "EMFJ6V3":
        raise ValueError("area dataset manifest protocol_id must be EMFJ6V3")
    if payload.get("development_only") is not True:
        raise ValueError("area dataset manifest must be development_only")
    if payload.get("sealed_access_allowed") is not False:
        raise ValueError("area dataset manifest must forbid sealed access")
    if not isinstance(source_root_id, str) or not source_root_id:
        raise ValueError("source_root_id is required for the area dataset manifest")
    validate_nonsealed_value(source_root_id, field="source_root_id")

    declared_sha = payload.get("manifest_sha256")
    canonical_sha = _canonical_manifest_sha256(payload)
    if (
        not isinstance(declared_sha, str)
        or not SHA256_PATTERN.fullmatch(declared_sha.lower())
        or declared_sha.lower() != canonical_sha
    ):
        raise ValueError("area dataset manifest declared SHA-256 mismatch")
    file_sha = sha256(manifest)
    if (file_sha, canonical_sha) not in AREA_MANIFEST_SHA256_ALLOWLIST:
        raise ValueError("area dataset manifest SHA-256 pair is not allowlisted")

    raw_roots = payload.get("source_roots")
    if not isinstance(raw_roots, list) or not raw_roots:
        raise ValueError("area dataset manifest has no source_roots")
    root_ids = [
        item.get("root_id") if isinstance(item, dict) else None for item in raw_roots
    ]
    if any(not isinstance(root_id, str) or not root_id for root_id in root_ids):
        raise ValueError("area dataset manifest has an invalid source root id")
    if len(set(root_ids)) != len(root_ids):
        raise ValueError("area dataset manifest source root ids must be unique")
    selected = [
        item
        for item in raw_roots
        if isinstance(item, dict) and item.get("root_id") == source_root_id
    ]
    if len(selected) != 1:
        raise ValueError("source_root_id must select exactly one source root")
    root_record = selected[0]
    root_split = root_record.get("split")
    if root_split not in {"TRAIN", "HOLDOUT"}:
        raise ValueError("selected eWaSR negative source root must use TRAIN/HOLDOUT")
    validate_nonsealed_value(root_record["root_id"], field="source root id")
    raw_root_path = (
        str(source_root_path)
        if source_root_path is not None
        else root_record.get("path")
    )
    if not isinstance(raw_root_path, str) or not raw_root_path:
        raise ValueError("selected source root path is missing")
    validate_nonsealed_value(raw_root_path, field="source root path")
    unresolved_root = Path(raw_root_path)
    _reject_symlink_chain(unresolved_root, field="source root path")
    root = unresolved_root.resolve(strict=True)
    validate_nonsealed_value(str(root), field="source root path")
    if not root.is_dir():
        raise ValueError("selected source root is not a directory")

    raw_frames = payload.get("frames")
    if not isinstance(raw_frames, list):
        raise TypeError("area dataset manifest frames must be a list")
    strict_negative_identity = (file_sha, canonical_sha) in (
        STRICT_NEGATIVE_IDENTITY_SHA_PAIRS
    )
    if strict_negative_identity:
        contract = payload.get("screening_dataset_contract")
        if not isinstance(contract, dict):
            raise ValueError("strict negative-only manifest contract is missing")
        scene_counts = contract.get("negative_only_scene_counts_by_split")
        frame_counts = contract.get("negative_only_frame_counts_by_split")
        if scene_counts != {"HOLDOUT": 1, "TRAIN": 1}:
            raise ValueError("strict negative-only scene counts changed")
        if frame_counts != {"HOLDOUT": 10, "TRAIN": 10}:
            raise ValueError("strict negative-only frame counts changed")
    selected_frames = [
        item
        for item in raw_frames
        if isinstance(item, dict) and item.get("root_id") == source_root_id
        and (not strict_negative_identity or item.get("negative_only") is True)
    ]
    if not selected_frames:
        raise ValueError("selected source root has no frames")

    records = []
    seen_ids: set[str] = set()
    seen_rgb_hashes: set[str] = set()
    for raw in selected_frames:
        if raw.get("split") != root_split:
            raise ValueError("selected source-root frame split must match its root")
        for field in ("world_id", "scene_id"):
            value = raw.get(field)
            if not isinstance(value, str) or not value:
                raise ValueError(f"area dataset frame has invalid {field}")
            validate_nonsealed_value(value, field=field)
        frame_index = raw.get("frame_index")
        if isinstance(frame_index, bool) or not isinstance(frame_index, int):
            raise TypeError("area dataset frame_index must be an integer")
        record_id = f"{source_root_id}:{raw['world_id']}:{raw['scene_id']}:{frame_index}"
        if record_id in seen_ids:
            raise ValueError("area dataset frame identity must be unique")
        seen_ids.add(record_id)

        paths = raw.get("paths")
        hashes = raw.get("sha256")
        if not isinstance(paths, dict) or not isinstance(hashes, dict):
            raise TypeError("area dataset frame paths/SHA-256 contract is missing")
        counts = raw.get("semantic_pixel_counts")
        if not isinstance(counts, dict):
            raise TypeError("area dataset frame semantic_pixel_counts is missing")
        unknown_count_ids = sorted(set(counts).difference({str(i) for i in range(6)}))
        if unknown_count_ids:
            raise ValueError(
                f"selected negative frame declares unknown semantic IDs: "
                f"{unknown_count_ids}"
            )
        if any(
            isinstance(value, bool) or not isinstance(value, int) or value < 0
            for value in counts.values()
        ):
            raise ValueError("semantic_pixel_counts values must be nonnegative integers")
        if counts.get("4") != 0 or counts.get("5") != 0:
            raise ValueError("selected negative frame declares semantic ID 4 or 5 pixels")
        rgb = _resolve_root_child(root, paths.get("rgb"), field="frame rgb path")
        semantic = _resolve_root_child(
            root, paths.get("semantic"), field="frame semantic path"
        )
        locked = {name: hashes.get(name) for name in ("rgb", "semantic")}
        for name, value in locked.items():
            if not isinstance(value, str) or not SHA256_PATTERN.fullmatch(value.lower()):
                raise ValueError(f"frame {name} SHA-256 is invalid")
            locked[name] = value.lower()
        records.append(
            _validate_negative_record(
                record_id=record_id,
                rgb=rgb,
                semantic=semantic,
                rgb_sha=locked["rgb"],
                semantic_sha=locked["semantic"],
                seen_rgb_hashes=seen_rgb_hashes,
            )
        )
    if strict_negative_identity and len(records) != 10:
        raise ValueError("strict selected source root must expose exactly 10 negative frames")
    return records


def load_negative_manifest(
    path: str | Path,
    *,
    source_root_id: str | None = None,
    source_root_path: str | Path | None = None,
) -> tuple[dict, list[dict]]:
    manifest = Path(path)
    validate_nonsealed_value(str(manifest), field="manifest path")
    _reject_symlink_chain(manifest, field="manifest path")
    manifest = manifest.resolve(strict=True)
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise TypeError("negative manifest root must be an object")
    if payload.get("schema_version") == AREA_DATASET_SCHEMA:
        return payload, _load_area_dataset_manifest(
            manifest, payload, source_root_id, source_root_path
        )
    if payload.get("schema_version") != 1:
        raise ValueError("negative manifest schema_version is unsupported")
    for field in ("dataset_id", "split"):
        value = payload.get(field)
        if isinstance(value, str):
            validate_nonsealed_value(value, field=field)
    if payload.get("negative_only") is not True:
        raise ValueError("eWaSR screening manifest must be explicitly negative_only")
    if payload.get("split") not in {"train", "development", "holdout"}:
        raise ValueError("negative manifest split must be train/development/holdout")
    if payload.get("semantic_label_contract") != TARGET_LABEL_CONTRACT:
        raise ValueError("negative manifest semantic label contract mismatch")
    raw_records = payload.get("records")
    if not isinstance(raw_records, list) or not raw_records:
        raise ValueError("negative manifest must contain records")

    records = []
    seen_ids: set[str] = set()
    seen_rgb_hashes: set[str] = set()
    for raw in raw_records:
        if not isinstance(raw, dict):
            raise TypeError("negative manifest record must be an object")
        record_id = raw.get("record_id")
        if not isinstance(record_id, str) or not record_id or record_id in seen_ids:
            raise ValueError("negative manifest record_id must be unique and non-empty")
        validate_nonsealed_value(record_id, field="record_id")
        seen_ids.add(record_id)
        rgb = _resolve_input_path(manifest, raw.get("rgb_path"), field="rgb_path")
        semantic = _resolve_input_path(
            manifest, raw.get("semantic_path"), field="semantic_path"
        )
        rgb_sha = _locked_sha(raw, "rgb_sha256")
        semantic_sha = _locked_sha(raw, "semantic_sha256")
        records.append(
            _validate_negative_record(
                record_id=record_id,
                rgb=rgb,
                semantic=semantic,
                rgb_sha=rgb_sha,
                semantic_sha=semantic_sha,
                seen_rgb_hashes=seen_rgb_hashes,
            )
        )
    return payload, records


def preprocess_rgb(rgb: np.ndarray) -> np.ndarray:
    image = np.asarray(rgb)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError("eWaSR RGB must be HxWx3 uint8")
    resized = cv2.resize(image, (512, 384), interpolation=cv2.INTER_LINEAR)
    normalized = resized.astype(np.float32) / 255.0
    normalized = (normalized - IMAGENET_MEAN) / IMAGENET_STD
    return np.ascontiguousarray(normalized.transpose(2, 0, 1)[None])


def _shape(value: object) -> tuple[int, ...]:
    try:
        return tuple(int(item) for item in value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as exc:
        raise ValueError("eWaSR exposes a non-static tensor shape") from exc


def _validate_session(session: object) -> None:
    providers = session.get_providers()
    if not providers or providers[0] != "CPUExecutionProvider":
        raise RuntimeError("eWaSR negative screening requires CPUExecutionProvider")
    inputs = session.get_inputs()
    if len(inputs) != 1:
        raise ValueError("eWaSR must expose exactly one input")
    if (
        inputs[0].name != MODEL_INPUT_NAME
        or _shape(inputs[0].shape) != MODEL_INPUT_SHAPE
    ):
        raise ValueError("eWaSR input tensor contract mismatch")
    outputs = {item.name: _shape(item.shape) for item in session.get_outputs()}
    if outputs.get(MODEL_OUTPUT_NAME) != MODEL_OUTPUT_SHAPE:
        raise ValueError("eWaSR prediction output tensor contract mismatch")
    if outputs.get("intermediate") != (1, 256, 24, 32):
        raise ValueError("eWaSR intermediate output tensor contract mismatch")


def evaluate(
    model_path: str | Path,
    manifest_path: str | Path,
    *,
    source_root_id: str | None = None,
    source_root_path: str | Path | None = None,
    session_factory: Callable[..., object] | None = None,
) -> dict:
    model = Path(model_path)
    validate_nonsealed_value(str(model), field="model path")
    _reject_symlink_chain(model, field="model path")
    model = model.resolve(strict=True)
    if not model.is_file() or sha256(model) != EXPECTED_MODEL_SHA256:
        raise ValueError("fixed eWaSR model SHA-256 mismatch")
    manifest = Path(manifest_path)
    payload, records = load_negative_manifest(
        manifest,
        source_root_id=source_root_id,
        source_root_path=source_root_path,
    )
    manifest = manifest.resolve(strict=True)

    if session_factory is None:
        import onnxruntime as ort

        session_factory = ort.InferenceSession
    session = session_factory(str(model), providers=["CPUExecutionProvider"])
    _validate_session(session)

    per_frame = []
    activated_frames = 0
    water_pixels = 0
    component_count = 0
    pixels_per_frame = MODEL_OUTPUT_SHAPE[2] * MODEL_OUTPUT_SHAPE[3]
    for record in records:
        rgb = cv2.cvtColor(record["rgb_bgr"], cv2.COLOR_BGR2RGB)
        outputs = session.run(
            [MODEL_OUTPUT_NAME], {MODEL_INPUT_NAME: preprocess_rgb(rgb)}
        )
        if not isinstance(outputs, list) or len(outputs) != 1:
            raise RuntimeError("eWaSR runtime returned an unexpected output list")
        logits = np.asarray(outputs[0], dtype=np.float32)
        if logits.shape != MODEL_OUTPUT_SHAPE or not np.isfinite(logits).all():
            raise ValueError("eWaSR prediction tensor is invalid")
        native_classes = np.argmax(logits, axis=1)[0]
        water_mask = np.ascontiguousarray(native_classes == 1, dtype=np.uint8)
        frame_water_pixels = int(water_mask.sum())
        frame_components = (
            int(cv2.connectedComponents(water_mask, connectivity=8)[0] - 1)
            if frame_water_pixels
            else 0
        )
        activated_frames += int(frame_water_pixels > 0)
        water_pixels += frame_water_pixels
        component_count += frame_components
        per_frame.append(
            {
                "record_id": record["record_id"],
                "rgb_sha256": record["rgb_sha256"],
                "semantic_sha256": record["semantic_sha256"],
                "water_pixel_count": frame_water_pixels,
                "water_pixel_fraction": frame_water_pixels / pixels_per_frame,
                "water_component_count": frame_components,
                "water_activated": frame_water_pixels > 0,
            }
        )

    frame_count = len(records)
    area_manifest_selected = payload.get("schema_version") == AREA_DATASET_SCHEMA
    source_area_manifest_sha = sha256(manifest) if area_manifest_selected else None
    source_area_manifest_canonical_sha = (
        _canonical_manifest_sha256(payload) if area_manifest_selected else None
    )
    return {
        "schema_version": 1,
        "stage": "EMF_EWASR_NEGATIVE_PROXY_SCREENING",
        "development_only": True,
        "negative_only": True,
        "competition_claim_allowed": False,
        "release_allowed": False,
        "not_journey6_runtime": True,
        "runtime_backend": "PC_ONNX_CPU",
        "source_root_id": source_root_id if area_manifest_selected else None,
        "source_area_manifest_sha256": source_area_manifest_sha,
        "source_area_manifest_canonical_sha256": source_area_manifest_canonical_sha,
        "model": {
            "model_id": "area_ewasr_resnet18",
            "sha256": EXPECTED_MODEL_SHA256,
            "input_name": MODEL_INPUT_NAME,
            "input_shape": list(MODEL_INPUT_SHAPE),
            "output_name": MODEL_OUTPUT_NAME,
            "output_shape": list(MODEL_OUTPUT_SHAPE),
            "class_order": list(MODEL_CLASS_ORDER),
            "decode": "raw_logits_argmax",
        },
        "preprocess": {
            "color_order": "RGB",
            "resize_wh": [512, 384],
            "resize_interpolation": "bilinear",
            "scale": 1.0 / 255.0,
            "mean": IMAGENET_MEAN.tolist(),
            "std": IMAGENET_STD.tolist(),
        },
        "dataset": {
            "dataset_id": payload.get("dataset_id"),
            "split": (
                next(
                    root["split"]
                    for root in payload["source_roots"]
                    if root["root_id"] == source_root_id
                )
                if area_manifest_selected
                else payload["split"]
            ),
            "manifest_sha256": sha256(manifest),
            "source_root_id": (
                source_root_id if area_manifest_selected else None
            ),
            "source_area_manifest_sha256": source_area_manifest_sha,
            "source_area_manifest_canonical_sha256": (
                source_area_manifest_canonical_sha
            ),
            "frame_count": frame_count,
            "target_semantic_labels_verified_absent": sorted(
                TARGET_LABEL_CONTRACT.values()
            ),
        },
        "water_activation": {
            "measurement_grid_hw": [MODEL_OUTPUT_SHAPE[2], MODEL_OUTPUT_SHAPE[3]],
            "activated_frame_count": activated_frames,
            "activated_frame_rate": activated_frames / frame_count,
            "water_pixel_count": water_pixels,
            "total_pixel_count": frame_count * pixels_per_frame,
            "water_pixel_fraction": water_pixels / (frame_count * pixels_per_frame),
            "water_component_count": component_count,
        },
        "target_class_mapping": None,
        "target_area_metrics_computed": False,
        "a4_area_pass_computed": False,
        "truth_boundary": (
            "This diagnostic preserves eWaSR source semantics. Water is not mapped "
            "to puddle or leaf_pile, and the result is not an Area A4 pass."
        ),
        "frames": per_frame,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--source-root-id", required=True)
    parser.add_argument("--source-root-path", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    validate_nonsealed_value(str(args.output), field="output path")
    report = evaluate(
        args.model,
        args.manifest,
        source_root_id=args.source_root_id,
        source_root_path=args.source_root_path,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {key: value for key, value in report.items() if key != "frames"}, indent=2
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
