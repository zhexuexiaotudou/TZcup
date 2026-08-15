"""Manifest v2 loading, validation, and backend-eligibility contract.

The repository has no formal perception model yet.  Every v2 manifest is a
placeholder with ``artifact: null`` and all gate statuses ``false``; the
validator reports that state as ``not_available`` and never lets a claim pass.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Mapping

import yaml


V2_REQUIRED_FIELDS = (
    "schema_version",
    "model_id",
    "version",
    "artifact",
    "artifact_sha256",
    "framework",
    "opset",
    "license",
    "weight_source",
    "pretraining_source",
    "input",
    "normalization",
    "output",
    "class_order",
    "thresholds",
    "NMS",
    "provider_compatibility",
    "screening_pass",
    "formal_pass",
    "live_pass",
    "synthetic_only",
    "competition_claim_allowed",
)

V2_BOOL_FIELDS = (
    "screening_pass",
    "formal_pass",
    "live_pass",
    "synthetic_only",
    "competition_claim_allowed",
)

PIPELINE_MODEL_ROLES = ("detector", "classifier", "leaf_segmenter", "puddle_segmenter")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_model_manifest(path: str | Path) -> dict:
    manifest_path = Path(path)
    if not manifest_path.is_file():
        raise FileNotFoundError(f"model manifest not found: {manifest_path}")
    try:
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid model manifest YAML: {exc}") from exc
    if not isinstance(manifest, dict):
        raise ValueError("model manifest must be a YAML mapping")
    return manifest


def load_pipeline_manifest(path: str | Path) -> dict:
    pipeline_path = Path(path)
    if not pipeline_path.is_file():
        raise FileNotFoundError(f"pipeline manifest not found: {pipeline_path}")
    try:
        pipeline = yaml.safe_load(pipeline_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid pipeline manifest YAML: {exc}") from exc
    if not isinstance(pipeline, dict):
        raise ValueError("pipeline manifest must be a YAML mapping")
    if pipeline.get("schema_version") != 2:
        raise ValueError("pipeline manifest schema_version must be 2")
    if not isinstance(pipeline.get("pipeline_id"), str):
        raise ValueError("pipeline manifest pipeline_id must be a string")
    model_manifests = pipeline.get("model_manifests")
    if not isinstance(model_manifests, dict):
        raise ValueError("pipeline manifest model_manifests must be a mapping")
    missing = [role for role in PIPELINE_MODEL_ROLES if role not in model_manifests]
    if missing:
        raise ValueError(
            "pipeline manifest missing model roles: " + ", ".join(missing)
        )
    from sanitation_perception.tracker_v2 import TrackerV2Config
    from sanitation_perception.lifecycle_health import WatchdogConfig
    from sanitation_perception.performance_monitor import PerformanceConfig

    try:
        TrackerV2Config.from_pipeline_manifest(pipeline)
        WatchdogConfig.from_pipeline_manifest(pipeline)
        PerformanceConfig.from_pipeline_manifest(pipeline)
        runtime = pipeline["runtime"]
        if not 0.0 < float(runtime["sync_tolerance_ms"]) <= 20.0:
            raise ValueError("sync_tolerance_ms must be in (0, 20]")
        if int(runtime["frame_queue_depth"]) not in (1, 2):
            raise ValueError("frame_queue_depth must be 1 or 2")
    except ValueError as exc:
        raise ValueError(f"pipeline manifest runtime invalid: {exc}") from exc
    except (KeyError, TypeError) as exc:
        raise ValueError("pipeline manifest runtime is incomplete") from exc
    return pipeline


def _finite_number(value: object) -> bool:
    if value is None:
        return True
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    import math

    return math.isfinite(float(value))


def _validate_v2(manifest: dict, artifact_root: Path | None) -> list[str]:
    errors: list[str] = []
    missing = [name for name in V2_REQUIRED_FIELDS if name not in manifest]
    if missing:
        errors.append("missing required fields: " + ", ".join(missing))
    for field in V2_BOOL_FIELDS:
        if field in manifest and not isinstance(manifest[field], bool):
            errors.append(f"{field} must be a boolean")
    for field in ("input", "output", "normalization", "thresholds", "NMS"):
        if field in manifest and not isinstance(manifest[field], dict):
            errors.append(f"{field} must be a mapping")
    if "version" in manifest and (
        not isinstance(manifest["version"], str) or not manifest["version"].strip()
    ):
        errors.append("version must be a non-empty string")
    for field in ("class_order", "provider_compatibility"):
        if field in manifest and not isinstance(manifest[field], list):
            errors.append(f"{field} must be a list")
    thresholds = manifest.get("thresholds")
    if isinstance(thresholds, dict):
        for key, value in thresholds.items():
            if not _finite_number(value):
                errors.append(f"thresholds.{key} must be finite or null")
    nms = manifest.get("NMS")
    if isinstance(nms, dict):
        for key, value in nms.items():
            if key in {"iou_threshold", "score_threshold"} and not _finite_number(value):
                errors.append(f"NMS.{key} must be finite or null")
        if "classwise" in nms and not isinstance(nms["classwise"], bool):
            errors.append("NMS.classwise must be a boolean")

    artifact = manifest.get("artifact")
    artifact_sha256 = manifest.get("artifact_sha256")
    if (artifact is None) != (artifact_sha256 is None):
        errors.append("artifact and artifact_sha256 must both be null or both be set")
    if artifact is not None:
        if not isinstance(artifact, str) or not artifact:
            errors.append("artifact must be a non-empty string when set")
        if (
            not isinstance(artifact_sha256, str)
            or len(artifact_sha256) != 64
            or any(char not in "0123456789abcdefABCDEF" for char in artifact_sha256)
        ):
            errors.append("artifact_sha256 must be a 64-character hex string when set")
        if artifact_root is None:
            errors.append("artifact_root is required to validate a non-null artifact")
        else:
            artifact_path = Path(artifact_root) / artifact
            if not artifact_path.is_file():
                errors.append(f"artifact file missing: {artifact_path}")
            else:
                actual = _sha256(artifact_path)
                if actual.lower() != str(artifact_sha256).lower():
                    errors.append(
                        f"artifact SHA-256 mismatch for {artifact_path}: "
                        f"manifest={artifact_sha256}, actual={actual}"
                    )
    return errors


def _validate_v1_legacy(manifest: dict, artifact_root: Path | None) -> list[str]:
    errors: list[str] = []
    for field in ("model_id", "artifact", "framework", "license", "weight_source"):
        if field not in manifest:
            errors.append(f"legacy manifest missing {field}")
    for field in ("synthetic_only", "competition_claim_allowed"):
        if field in manifest and not isinstance(manifest[field], bool):
            errors.append(f"{field} must be a boolean")
    artifact = manifest.get("artifact")
    if artifact is not None and artifact_root is not None:
        artifact_path = Path(artifact_root) / artifact
        if not artifact_path.is_file():
            errors.append(f"artifact file missing: {artifact_path}")
    return errors


def validate_model_manifest(
    manifest: Mapping[str, object],
    artifact_root: str | Path | None = None,
) -> list[str]:
    """Validate a model manifest and return a list of error strings.

    An empty list means the manifest is valid.  When ``artifact`` is non-null
    the artifact file must exist under ``artifact_root`` and its SHA-256 must
    match ``artifact_sha256``.
    """
    if not isinstance(manifest, dict):
        return ["model manifest must be a mapping"]
    version = manifest.get("schema_version")
    root = Path(artifact_root) if artifact_root is not None else None
    if version == 1:
        return _validate_v1_legacy(manifest, root)
    if version == 2:
        return _validate_v2(manifest, root)
    return [f"unsupported manifest schema_version: {version!r}"]


def backend_eligibility(manifest: Mapping[str, object]) -> dict[str, bool]:
    """Return screening/formal/live/competition eligibility booleans."""
    artifact_available = bool(manifest.get("artifact"))
    formal_pass = bool(manifest.get("formal_pass"))
    return {
        "screening": bool(manifest.get("screening_pass")) and artifact_available,
        "formal": formal_pass and artifact_available,
        "live": bool(manifest.get("live_pass")) and artifact_available,
        "competition": (
            bool(manifest.get("competition_claim_allowed"))
            and formal_pass
            and artifact_available
        ),
    }


def model_status(manifest: Mapping[str, object]) -> str:
    """Machine-readable availability status derived from the manifest."""
    if not isinstance(manifest, dict):
        return "invalid"
    if manifest.get("schema_version") != 2:
        return "legacy"
    if manifest.get("artifact") is None:
        return "not_available"
    return "available"
