from __future__ import annotations

import hashlib
from pathlib import Path

import pytest
import yaml

from sanitation_perception.pipeline_manifest import (
    backend_eligibility,
    load_model_manifest,
    load_pipeline_manifest,
    model_status,
    validate_model_manifest,
)


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


def _valid_manifest(artifact: str | None = None) -> dict:
    return {
        "schema_version": 2,
        "model_id": "test_model",
        "version": "1.0.0",
        "artifact": artifact,
        "artifact_sha256": None if artifact is None else hashlib.sha256(b"x").hexdigest(),
        "framework": "onnxruntime",
        "opset": 17,
        "license": "Apache-2.0",
        "weight_source": "test_fixture",
        "pretraining_source": None,
        "input": {
            "names": ["images"],
            "shapes": [[1, 3, 96, 128]],
            "dtypes": ["float32"],
        },
        "normalization": {
            "scale": 0.00392156862745098,
            "mean": [0.0, 0.0, 0.0],
            "std": [1.0, 1.0, 1.0],
        },
        "output": {
            "names": ["logits"],
            "shapes": [[1, 6, 96, 128]],
            "dtypes": ["float32"],
        },
        "class_order": ["background", "plastic_bottle", "metal_can"],
        "thresholds": {"score": 0.5, "nms_iou": 0.5},
        "NMS": {
            "classwise": True,
            "iou_threshold": 0.5,
            "score_threshold": 0.5,
        },
        "provider_compatibility": ["onnxruntime", "horizon_j6"],
        "screening_pass": False,
        "formal_pass": False,
        "live_pass": False,
        "synthetic_only": True,
        "competition_claim_allowed": False,
    }


def test_v2_manifests_are_not_available_and_never_claim_gates() -> None:
    for name in (
        "detector_manifest.yaml",
        "classifier_manifest.yaml",
        "leaf_segmenter_manifest.yaml",
        "puddle_segmenter_manifest.yaml",
    ):
        manifest = load_model_manifest(CONFIG_DIR / name)
        assert manifest["artifact"] is None
        assert manifest["artifact_sha256"] is None
        assert model_status(manifest) == "not_available"
        assert validate_model_manifest(manifest) == []
        assert backend_eligibility(manifest) == {
            "screening": False,
            "formal": False,
            "live": False,
            "competition": False,
        }
        assert manifest["competition_claim_allowed"] is False


def test_pipeline_manifest_loads_all_four_roles() -> None:
    pipeline = load_pipeline_manifest(CONFIG_DIR / "perception_pipeline_manifest.yaml")
    assert pipeline["schema_version"] == 2
    assert set(pipeline["model_manifests"]) == {
        "detector",
        "classifier",
        "leaf_segmenter",
        "puddle_segmenter",
    }
    assert pipeline["status"]["formal_pipeline_pass"] is False
    assert pipeline["status"]["live_pipeline_pass"] is False
    assert pipeline["status"]["competition_pipeline_claim_allowed"] is False
    assert pipeline["runtime"]["tracker_v2"]["confirmation_observations"] == 3
    assert pipeline["runtime"]["sync_tolerance_ms"] == 20.0
    assert pipeline["runtime"]["frame_queue_depth"] == 2
    assert pipeline["runtime"]["watchdog"]["camera_stale_ms"] == 500.0


def test_legacy_synthetic_manifest_is_preserved() -> None:
    legacy = load_model_manifest(CONFIG_DIR / "model_manifest.yaml")
    copy = load_model_manifest(CONFIG_DIR / "model_manifest_legacy_synthetic.yaml")
    assert legacy == copy
    assert model_status(copy) == "legacy"
    assert validate_model_manifest(copy) == []


def test_validate_reports_missing_required_fields() -> None:
    manifest = _valid_manifest()
    manifest.pop("NMS")
    errors = validate_model_manifest(manifest)
    assert any("missing required fields" in error and "NMS" in error for error in errors)


def test_validate_reports_wrong_status_type() -> None:
    manifest = _valid_manifest()
    manifest["screening_pass"] = "yes"
    errors = validate_model_manifest(manifest)
    assert any("screening_pass must be a boolean" in error for error in errors)


def test_validate_reports_non_finite_threshold() -> None:
    manifest = _valid_manifest()
    manifest["thresholds"]["score"] = float("nan")
    errors = validate_model_manifest(manifest)
    assert any("thresholds.score must be finite" in error for error in errors)


def test_validate_reports_null_artifact_sha_mismatch() -> None:
    manifest = _valid_manifest()
    manifest["artifact"] = "model.onnx"
    manifest["artifact_sha256"] = None
    errors = validate_model_manifest(manifest, artifact_root=Path("."))
    assert any(
        "artifact and artifact_sha256 must both be null or both be set" in error
        for error in errors
    )


def test_validate_checks_artifact_file_and_sha(tmp_path: Path) -> None:
    artifact = tmp_path / "model.onnx"
    artifact.write_bytes(b"payload")
    correct = hashlib.sha256(b"payload").hexdigest()

    manifest = _valid_manifest(artifact="model.onnx")
    manifest["artifact_sha256"] = correct
    assert validate_model_manifest(manifest, artifact_root=tmp_path) == []

    manifest["artifact_sha256"] = "0" * 64
    errors = validate_model_manifest(manifest, artifact_root=tmp_path)
    assert any("SHA-256 mismatch" in error for error in errors)

    manifest["artifact_sha256"] = correct
    manifest["artifact"] = "missing.onnx"
    errors = validate_model_manifest(manifest, artifact_root=tmp_path)
    assert any("artifact file missing" in error for error in errors)


def test_validate_requires_artifact_root_for_non_null_artifact() -> None:
    manifest = _valid_manifest(artifact="model.onnx")
    manifest["artifact_sha256"] = hashlib.sha256(b"x").hexdigest()
    errors = validate_model_manifest(manifest)
    assert any("artifact_root is required" in error for error in errors)


def test_backend_eligibility_requires_artifact_and_formal_for_competition() -> None:
    manifest = _valid_manifest()
    manifest["screening_pass"] = True
    assert backend_eligibility(manifest)["screening"] is False  # no artifact
    manifest["artifact"] = "model.onnx"
    manifest["artifact_sha256"] = hashlib.sha256(b"x").hexdigest()
    assert backend_eligibility(manifest)["screening"] is True
    manifest["competition_claim_allowed"] = True
    assert backend_eligibility(manifest)["competition"] is False  # formal not set
    manifest["formal_pass"] = True
    assert backend_eligibility(manifest)["competition"] is True


def test_load_model_manifest_fails_closed_on_missing_file() -> None:
    with pytest.raises(FileNotFoundError, match="not found"):
        load_model_manifest(CONFIG_DIR / "missing.yaml")


def test_load_model_manifest_fails_closed_on_invalid_yaml(tmp_path: Path) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text("not: [valid", encoding="utf-8")
    with pytest.raises(ValueError, match="invalid model manifest YAML"):
        load_model_manifest(path)


def test_load_pipeline_manifest_rejects_missing_role(tmp_path: Path) -> None:
    pipeline = {
        "schema_version": 2,
        "pipeline_id": "bad_pipeline",
        "model_manifests": {"detector": "detector_manifest.yaml"},
        "runtime": {"backend": "onnxruntime"},
        "status": {},
    }
    path = tmp_path / "pipeline.yaml"
    path.write_text(yaml.safe_dump(pipeline), encoding="utf-8")
    with pytest.raises(ValueError, match="missing model roles"):
        load_pipeline_manifest(path)
