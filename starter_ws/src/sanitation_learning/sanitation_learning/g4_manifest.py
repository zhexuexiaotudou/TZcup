"""Immutable frozen-model configuration and artifact manifest contracts.

Artifact manifests are generated from an immutable frozen model
configuration (``MODEL_FREEZE.json``) rather than hand-maintained.  Every
field and hash is validated fail-closed: missing fields, mismatched config
hashes, missing artifacts or wrong SHA-256 reject the manifest.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any, Iterable


FREEZE_REQUIRED_FIELDS = (
    "schema_version",
    "freeze_id",
    "freeze_timestamp",
    "config_hash",
    "model_config",
    "architecture_hashes",
    "preprocess_hashes",
    "postprocess_hashes",
    "thresholds",
    "nms",
    "calibration",
    "training_data_hashes",
    "validation_metrics",
    "model_artifact_hashes",
    "pretrained_provenance",
    "onnx_contracts",
    "p4_screening",
    "final_evaluator_sha256",
)

REQUIRED_PRODUCT_MODELS = ("discovery", "classifier", "leaf", "puddle")

MANIFEST_REQUIRED_FIELDS = (
    "schema_version",
    "manifest_version",
    "model_id",
    "model_type",
    "freeze_id",
    "freeze_timestamp",
    "config_hash",
    "preprocessing",
    "postprocessing",
    "thresholds",
    "class_map",
    "input",
    "output",
    "operator_inventory",
    "artifact",
    "artifact_sha256",
    "provenance",
    "acceptance",
)


def canonical_json(value: Any) -> str:
    """Deterministic, compact JSON serialization for hashing."""
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
    )


def config_hash(config: Any) -> str:
    return hashlib.sha256(
        canonical_json(config).encode("utf-8")
    ).hexdigest()


def file_sha256(path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_freeze_payload(payload: dict) -> dict:
    """Validate ``MODEL_FREEZE.json`` and return the payload unchanged."""
    if not isinstance(payload, dict):
        raise ValueError("MODEL_FREEZE.json must be a JSON object")
    missing = [
        field for field in FREEZE_REQUIRED_FIELDS if field not in payload
    ]
    if missing:
        raise ValueError(
            "MODEL_FREEZE.json missing required fields: "
            + ", ".join(missing)
        )
    if payload.get("schema_version") != 1:
        raise ValueError("MODEL_FREEZE.json schema_version must be 1")
    model_config = payload.get("model_config")
    if not isinstance(model_config, dict):
        raise ValueError("MODEL_FREEZE.json model_config must be a mapping")
    if payload.get("config_hash") != config_hash(model_config):
        raise ValueError("MODEL_FREEZE.json config_hash mismatch")
    missing_models = sorted(set(REQUIRED_PRODUCT_MODELS) - set(model_config))
    if missing_models:
        raise ValueError(
            "MODEL_FREEZE.json missing product models: "
            + ", ".join(missing_models)
        )
    for group in (
        "architecture_hashes",
        "preprocess_hashes",
        "postprocess_hashes",
        "thresholds",
        "nms",
        "calibration",
        "training_data_hashes",
        "validation_metrics",
        "model_artifact_hashes",
        "pretrained_provenance",
        "onnx_contracts",
        "p4_screening",
    ):
        if not isinstance(payload.get(group), dict):
            raise ValueError(
                f"MODEL_FREEZE.json {group} must be a mapping"
            )
    p4 = payload["p4_screening"]
    if (
        p4.get("policy_id") != "perception_p4_screening_policy"
        or p4.get("pass") is not True
        or not _is_sha256(p4.get("evidence_sha256"))
    ):
        raise ValueError(
            "MODEL_FREEZE.json requires verified P4 screening pass evidence"
        )
    if not _is_sha256(payload.get("final_evaluator_sha256")):
        raise ValueError(
            "MODEL_FREEZE.json final_evaluator_sha256 must be SHA-256"
        )
    for model_type in REQUIRED_PRODUCT_MODELS:
        artifact_hash = payload["model_artifact_hashes"].get(model_type)
        if not _is_sha256(artifact_hash):
            raise ValueError(
                f"MODEL_FREEZE.json missing valid artifact hash for {model_type}"
            )
        provenance = payload["pretrained_provenance"].get(model_type)
        if not isinstance(provenance, dict) or (
            provenance.get("pretrained") is not True
            or provenance.get("from_scratch_control") is not False
            or not _is_sha256(provenance.get("sha256"))
            or not provenance.get("weight_enum")
            or not provenance.get("source_url")
            or not provenance.get("license")
        ):
            raise ValueError(
                "MODEL_FREEZE.json requires verified official pretrained "
                f"provenance for {model_type}"
            )
        onnx_contract = payload["onnx_contracts"].get(model_type)
        if not isinstance(onnx_contract, dict) or not all(
            (
                onnx_contract.get("passed") is True,
                onnx_contract.get("fixed_input") is True,
                onnx_contract.get("opset") == 17,
                onnx_contract.get("custom_ops") == 0,
                isinstance(onnx_contract.get("operator_inventory"), dict),
            )
        ):
            raise ValueError(
                f"MODEL_FREEZE.json invalid ONNX contract for {model_type}"
            )
    return payload


def _is_sha256(value: Any) -> bool:
    if not isinstance(value, str) or len(value) != 64:
        return False
    try:
        int(value, 16)
    except ValueError:
        return False
    return True


def load_freeze(path) -> dict:
    freeze_path = Path(path)
    if not freeze_path.is_file():
        raise FileNotFoundError(f"MODEL_FREEZE.json not found: {freeze_path}")
    payload = json.loads(freeze_path.read_text(encoding="utf-8"))
    return validate_freeze_payload(payload)


def _as_list(value: Any) -> list:
    if isinstance(value, list):
        return list(value)
    if isinstance(value, tuple):
        return list(value)
    return [int(value) for value in value]


def build_artifact_manifest(
    freeze: dict,
    *,
    model_type: str,
    artifact_path,
    input_shape: Iterable[int],
    output_shapes: dict[str, list[int]],
    operator_inventory: dict[str, int],
    class_map: dict,
    provenance: dict,
    acceptance: dict,
) -> dict:
    """Generate a manifest for one frozen model artifact."""
    artifact = Path(artifact_path)
    if not artifact.is_file():
        raise FileNotFoundError(f"model artifact not found: {artifact}")
    model_config = freeze["model_config"]
    model_entry = model_config.get(model_type)
    if not isinstance(model_entry, dict):
        raise ValueError(
            f"frozen model_config has no entry for model_type {model_type!r}"
        )
    model_id = model_entry.get("model_id")
    if not isinstance(model_id, str):
        raise ValueError("frozen model_config model_id must be a string")
    manifest = {
        "schema_version": 1,
        "manifest_version": 1,
        "model_id": model_id,
        "model_type": model_type,
        "freeze_id": freeze["freeze_id"],
        "freeze_timestamp": freeze["freeze_timestamp"],
        "config_hash": freeze["config_hash"],
        "preprocessing": freeze.get("preprocess_hashes"),
        "postprocessing": freeze.get("postprocess_hashes"),
        "thresholds": freeze.get("thresholds", {}).get(model_type, {}),
        "class_map": class_map,
        "input": {
            "name": model_entry.get("input_name", model_type),
            "shape": _as_list(input_shape),
            "dtype": "float32",
        },
        "output": output_shapes,
        "operator_inventory": dict(operator_inventory),
        "artifact": artifact.name,
        "artifact_sha256": file_sha256(artifact),
        "provenance": dict(provenance),
        "acceptance": dict(acceptance),
    }
    return manifest


def validate_artifact_manifest(
    manifest: dict,
    *,
    artifact_root,
    expected_config_hash: str | None = None,
) -> dict:
    """Validate a generated manifest fail-closed; returns it unchanged."""
    if not isinstance(manifest, dict):
        raise ValueError("artifact manifest must be a JSON object")
    missing = [
        field
        for field in MANIFEST_REQUIRED_FIELDS
        if field not in manifest
    ]
    if missing:
        raise ValueError(
            "artifact manifest missing required fields: "
            + ", ".join(missing)
        )
    if manifest.get("schema_version") != 1 or manifest.get(
        "manifest_version"
    ) != 1:
        raise ValueError("unsupported artifact manifest version")
    if expected_config_hash is not None and (
        manifest.get("config_hash") != expected_config_hash
    ):
        raise ValueError("artifact manifest config_hash mismatch")
    artifact = Path(artifact_root) / str(manifest["artifact"])
    if not artifact.is_file():
        raise FileNotFoundError(
            f"manifest artifact missing: {artifact}"
        )
    if file_sha256(artifact) != str(manifest["artifact_sha256"]):
        raise ValueError(
            f"artifact SHA-256 mismatch for {manifest['artifact']}"
        )
    for field in ("input", "output", "class_map", "acceptance", "provenance"):
        if not isinstance(manifest.get(field), dict):
            raise ValueError(f"artifact manifest {field} must be a mapping")
    if not isinstance(manifest.get("operator_inventory"), dict):
        raise ValueError(
            "artifact manifest operator_inventory must be a mapping"
        )
    return manifest


__all__ = [
    "FREEZE_REQUIRED_FIELDS",
    "MANIFEST_REQUIRED_FIELDS",
    "REQUIRED_PRODUCT_MODELS",
    "build_artifact_manifest",
    "canonical_json",
    "config_hash",
    "file_sha256",
    "load_freeze",
    "validate_artifact_manifest",
    "validate_freeze_payload",
]
