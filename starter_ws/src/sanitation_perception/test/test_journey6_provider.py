import hashlib

import numpy as np
import pytest

from sanitation_perception.journey6_provider import (
    Journey6HbmContract,
    Journey6HbmProvider,
    TensorContract,
)


def contract_fixture(tmp_path):
    artifact = tmp_path / "model.hbm"
    artifact.write_bytes(b"official-j6-fixture")
    return Journey6HbmContract(
        model_id="fixture",
        artifact=artifact,
        artifact_sha256=hashlib.sha256(artifact.read_bytes()).hexdigest(),
        march="nash-e",
        runtime_version="1.2.3",
        input_format="nv12",
        inputs=(TensorContract("images", (1, 6, 8), "uint8", stride_bytes=8),),
        outputs=(TensorContract("scores", (1, 3), "float32"),),
    )


def test_journey6_provider_forbids_missing_official_runtime(tmp_path):
    provider = Journey6HbmProvider(
        contract_fixture(tmp_path),
        installed_runtime_version="1.2.3",
        installed_march="nash-e",
        loader=None,
        runner=None,
    )
    with pytest.raises(RuntimeError, match="official Journey 6"):
        provider.load()


def test_journey6_provider_checks_runtime_and_tensor_contract(tmp_path):
    provider = Journey6HbmProvider(
        contract_fixture(tmp_path),
        installed_runtime_version="1.2.3",
        installed_march="nash-e",
        loader=lambda _contract: object(),
        runner=lambda _handle, _inputs: {"scores": np.ones((1, 3), dtype=np.float32)},
    )
    provider.load()
    result = provider.infer({"images": np.zeros((1, 6, 8), dtype=np.uint8)})
    assert result["scores"].shape == (1, 3)
    assert provider.health()["fallback_used"] is False
    with pytest.raises(ValueError, match="dtype"):
        provider.infer({"images": np.zeros((1, 6, 8), dtype=np.float32)})
