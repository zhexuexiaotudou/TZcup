from pathlib import Path

import pytest

from sanitation_perception.backends import BackendUnavailable, select_backend


def test_j6_and_ground_truth_fail_closed():
    with pytest.raises(BackendUnavailable, match="toolchain unavailable"):
        select_backend("horizon_j6")
    with pytest.raises(BackendUnavailable, match="evaluation-only"):
        select_backend("ground_truth")
    with pytest.raises(BackendUnavailable, match="test-only"):
        select_backend("mock")


def test_onnx_requires_real_artifact(tmp_path: Path):
    with pytest.raises(BackendUnavailable, match="missing"):
        select_backend("onnxruntime", model_path=tmp_path / "missing.onnx")
    model = tmp_path / "model.onnx"; model.write_bytes(b"not-validated-here")
    assert select_backend("onnxruntime", model_path=model).active == "onnxruntime"
