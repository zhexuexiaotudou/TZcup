import hashlib

import numpy as np
import pytest

from sanitation_perception.journey6_provider import TensorContract
from sanitation_perception.onnx_provider import StrictOnnxProvider


class FakeSession:
    def __init__(self, _path, providers):
        self.providers = providers
        self.fallback_disabled = False

    def disable_fallback(self):
        self.fallback_disabled = True

    def get_providers(self):
        return self.providers

    def run(self, names, _feed):
        assert names == ["scores"]
        return [np.ones((1, 3), dtype=np.float32)]


class FakeOrt:
    @staticmethod
    def get_available_providers():
        return ["CUDAExecutionProvider", "CPUExecutionProvider"]

    InferenceSession = FakeSession


def test_strict_onnx_cuda_provider_disables_fallback_and_checks_io(tmp_path):
    artifact = tmp_path / "model.onnx"
    artifact.write_bytes(b"fixture")
    provider = StrictOnnxProvider(
        artifact=artifact,
        artifact_sha256=hashlib.sha256(artifact.read_bytes()).hexdigest(),
        inputs=(TensorContract("images", (1, 3, 4, 4), "float32"),),
        outputs=(TensorContract("scores", (1, 3), "float32"),),
        provider="onnx_cuda",
        ort_module=FakeOrt,
    )
    provider.load()
    assert provider.session.fallback_disabled is True
    result = provider.infer({"images": np.zeros((1, 3, 4, 4), dtype=np.float32)})
    assert result["scores"].shape == (1, 3)
    assert provider.health()["fallback_used"] is False


def test_strict_onnx_provider_rejects_bad_hash(tmp_path):
    artifact = tmp_path / "model.onnx"
    artifact.write_bytes(b"fixture")
    provider = StrictOnnxProvider(
        artifact=artifact,
        artifact_sha256="0" * 64,
        inputs=(TensorContract("images", (1, 3, 4, 4), "float32"),),
        outputs=(TensorContract("scores", (1, 3), "float32"),),
        provider="onnx_cpu",
        ort_module=FakeOrt,
    )
    with pytest.raises(ValueError, match="SHA-256"):
        provider.load()
