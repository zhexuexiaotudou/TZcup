from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from sanitation_perception.pipeline_manifest import (
    _sha256,
    backend_eligibility,
    load_model_manifest,
    validate_model_manifest,
)


class BackendUnavailable(RuntimeError):
    pass


CLAIM_TO_ELIGIBILITY_KEY = {
    "screening": "screening",
    "formal": "formal",
    "live": "live",
    "competition": "competition",
}


@dataclass(frozen=True)
class BackendSelection:
    requested: str
    active: str
    synthetic_only: bool
    ground_truth_control_allowed: bool
    detail: str
    screening_pass: bool = False
    formal_pass: bool = False
    live_pass: bool = False
    competition_claim_allowed: bool = False


def _select_onnxruntime(
    model_path: str | Path | None,
    manifest_path: str | Path | None,
    required_claim: str | None,
    artifact_root: str | Path | None,
) -> BackendSelection:
    if manifest_path is None:
        raise BackendUnavailable("model manifest is missing; fail-closed")

    manifest_file = Path(manifest_path)
    if not manifest_file.is_file():
        raise BackendUnavailable("model manifest is missing; fail-closed")
    try:
        manifest = load_model_manifest(manifest_file)
    except (ValueError, FileNotFoundError) as exc:
        raise BackendUnavailable(f"model manifest invalid: {exc}") from exc
    resolved_root = (
        Path(artifact_root) if artifact_root is not None else manifest_file.parent
    )
    validation_errors = validate_model_manifest(manifest, artifact_root=resolved_root)
    if validation_errors:
        raise BackendUnavailable(
            "model manifest invalid: " + "; ".join(validation_errors)
        )
    if manifest.get("artifact") is None:
        raise BackendUnavailable("model manifest has no artifact; model not available")
    if required_claim is not None:
        if required_claim not in CLAIM_TO_ELIGIBILITY_KEY:
            raise BackendUnavailable(f"unknown required claim: {required_claim}")
        eligibility = backend_eligibility(manifest)
        if not eligibility[CLAIM_TO_ELIGIBILITY_KEY[required_claim]]:
            raise BackendUnavailable(
                f"model manifest does not satisfy required claim "
                f"{required_claim!r}; fail-closed"
            )

    artifact_path = resolved_root / manifest["artifact"]
    effective_model_path = Path(model_path) if model_path is not None else artifact_path
    if not effective_model_path.is_file():
        raise BackendUnavailable("ONNX model artifact is missing")
    if effective_model_path.resolve() != artifact_path.resolve():
        actual = _sha256(effective_model_path)
        if actual.lower() != str(manifest["artifact_sha256"]).lower():
            raise BackendUnavailable(
                "ONNX model artifact SHA-256 does not match manifest"
            )
    return BackendSelection(
        requested="onnxruntime",
        active="onnxruntime",
        synthetic_only=bool(manifest.get("synthetic_only", True)),
        ground_truth_control_allowed=False,
        detail="x86_onnxruntime_manifest_validated",
        screening_pass=bool(manifest.get("screening_pass", False)),
        formal_pass=bool(manifest.get("formal_pass", False)),
        live_pass=bool(manifest.get("live_pass", False)),
        competition_claim_allowed=bool(
            manifest.get("competition_claim_allowed", False)
        ),
    )


def select_backend(
    requested: str,
    *,
    model_path: str | Path | None = None,
    test_mode: bool = False,
    allow_ground_truth: bool = False,
    j6_toolchain_available: bool = False,
    j6_runtime_available: bool = False,
    manifest_path: str | Path | None = None,
    required_claim: str = "screening",
    artifact_root: str | Path | None = None,
) -> BackendSelection:
    if requested == "ground_truth":
        if not allow_ground_truth:
            raise BackendUnavailable("ground_truth backend is evaluation-only")
        return BackendSelection(
            requested, requested, True, False, "evaluation_annotations_only"
        )
    if requested == "mock":
        if not test_mode:
            raise BackendUnavailable("mock backend is test-only")
        return BackendSelection(requested, requested, True, False, "unit_test_only")
    if requested == "onnxruntime":
        return _select_onnxruntime(
            model_path, manifest_path, required_claim, artifact_root
        )
    if requested == "horizon_j6":
        if not j6_toolchain_available:
            raise BackendUnavailable("Horizon J6 toolchain unavailable; fail-closed")
        if not j6_runtime_available:
            raise BackendUnavailable("Horizon J6 runtime unavailable; fail-closed")
        return BackendSelection(
            requested, requested, False, False, "horizon_j6_runtime"
        )
    raise BackendUnavailable(f"unknown perception backend: {requested}")
