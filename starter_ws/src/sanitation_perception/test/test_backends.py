import hashlib
from pathlib import Path

import pytest
import yaml

from sanitation_perception.backends import BackendUnavailable, select_backend


def _write_manifest(
    directory: Path,
    artifact_name: str,
    *,
    screening_pass: bool,
    formal_pass: bool = False,
    live_pass: bool = False,
    sha256: str | None = None,
) -> Path:
    artifact = directory / artifact_name
    artifact.write_bytes(b"fake-onnx-artifact")
    if sha256 is None:
        sha256 = hashlib.sha256(artifact.read_bytes()).hexdigest()
    manifest = {
        "schema_version": 2,
        "model_id": "test_model",
        "version": "1.0.0",
        "artifact": artifact_name,
        "artifact_sha256": sha256,
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
        "provider_compatibility": ["onnxruntime"],
        "screening_pass": screening_pass,
        "formal_pass": formal_pass,
        "live_pass": live_pass,
        "synthetic_only": True,
        "competition_claim_allowed": False,
    }
    manifest_path = directory / "model_manifest.yaml"
    manifest_path.write_text(yaml.safe_dump(manifest), encoding="utf-8")
    return manifest_path


def test_j6_and_ground_truth_fail_closed():
    with pytest.raises(BackendUnavailable, match="toolchain unavailable"):
        select_backend("horizon_j6")
    with pytest.raises(BackendUnavailable, match="evaluation-only"):
        select_backend("ground_truth")
    with pytest.raises(BackendUnavailable, match="test-only"):
        select_backend("mock")


def test_onnx_requires_manifest_and_real_artifact(tmp_path: Path):
    with pytest.raises(BackendUnavailable, match="model manifest is missing"):
        select_backend("onnxruntime", model_path=tmp_path / "missing.onnx")
    model = tmp_path / "model.onnx"; model.write_bytes(b"not-validated-here")
    with pytest.raises(BackendUnavailable, match="model manifest is missing"):
        select_backend("onnxruntime", model_path=model)


def test_onnx_fails_closed_when_manifest_missing(tmp_path: Path):
    with pytest.raises(BackendUnavailable, match="manifest is missing"):
        select_backend("onnxruntime", manifest_path=tmp_path / "missing.yaml")


def test_onnx_fails_closed_on_artifact_sha_mismatch(tmp_path: Path):
    manifest_path = _write_manifest(
        tmp_path,
        "model.onnx",
        screening_pass=True,
        sha256="0" * 64,
    )
    with pytest.raises(BackendUnavailable, match="SHA-256 mismatch"):
        select_backend("onnxruntime", manifest_path=manifest_path)


def test_onnx_fails_closed_when_status_insufficient(tmp_path: Path):
    manifest_path = _write_manifest(tmp_path, "model.onnx", screening_pass=False)
    with pytest.raises(BackendUnavailable, match="required claim 'screening'"):
        select_backend(
            "onnxruntime",
            manifest_path=manifest_path,
            required_claim="screening",
        )
    manifest_path = _write_manifest(tmp_path, "model.onnx", screening_pass=True)
    with pytest.raises(BackendUnavailable, match="required claim 'formal'"):
        select_backend(
            "onnxruntime",
            manifest_path=manifest_path,
            required_claim="formal",
        )


def test_onnx_succeeds_with_sufficient_status(tmp_path: Path):
    manifest_path = _write_manifest(tmp_path, "model.onnx", screening_pass=True)
    selection = select_backend(
        "onnxruntime",
        manifest_path=manifest_path,
        required_claim="screening",
    )
    assert selection.active == "onnxruntime"
    assert selection.synthetic_only is True
    assert selection.screening_pass is True
    assert selection.formal_pass is False
    assert selection.live_pass is False
    assert selection.competition_claim_allowed is False


def test_onnx_fails_closed_when_manifest_has_no_artifact(tmp_path: Path):
    manifest = {
        "schema_version": 2,
        "model_id": "placeholder",
        "version": "0.0.0-placeholder",
        "artifact": None,
        "artifact_sha256": None,
        "framework": "onnxruntime",
        "opset": None,
        "license": "Apache-2.0",
        "weight_source": "not_available",
        "pretraining_source": None,
        "input": {"names": ["images"], "shapes": [[1, 3, 96, 128]], "dtypes": ["float32"]},
        "normalization": {"scale": 0.00392156862745098, "mean": [0.0, 0.0, 0.0], "std": [1.0, 1.0, 1.0]},
        "output": {"names": ["logits"], "shapes": [[1, 6, 96, 128]], "dtypes": ["float32"]},
        "class_order": ["background", "plastic_bottle"],
        "thresholds": {"score": None, "nms_iou": None},
        "NMS": {"classwise": False, "iou_threshold": None, "score_threshold": None},
        "provider_compatibility": ["onnxruntime"],
        "screening_pass": False,
        "formal_pass": False,
        "live_pass": False,
        "synthetic_only": True,
        "competition_claim_allowed": False,
    }
    manifest_path = tmp_path / "placeholder.yaml"
    manifest_path.write_text(yaml.safe_dump(manifest), encoding="utf-8")
    with pytest.raises(BackendUnavailable, match="no artifact"):
        select_backend(
            "onnxruntime",
            manifest_path=manifest_path,
            required_claim="screening",
        )
