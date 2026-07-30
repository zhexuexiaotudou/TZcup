import hashlib
import json

import numpy as np
import pytest

from sanitation_perception.j6_runtime import J6Artifact, J6RuntimeAdapter


def artifact_fixture(tmp_path):
    hbm = tmp_path / "model.hbm"
    hbm.write_bytes(b"hbm-fixture")
    manifest = tmp_path / "manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "model_name": "fixture",
                "hbm_path": "model.hbm",
                "sha256": hashlib.sha256(hbm.read_bytes()).hexdigest(),
                "input": {"name": "images", "shape": [1, 3, 16, 16]},
                "output_names": ["scores"],
                "march": "nash-e",
            }
        ),
        encoding="utf-8",
    )
    return J6Artifact.from_manifest(manifest)


def test_runtime_forbids_silent_cpu_fallback(tmp_path):
    adapter = J6RuntimeAdapter(artifact_fixture(tmp_path), board_available=False)
    with pytest.raises(RuntimeError, match="silent CPU/ONNX fallback is forbidden"):
        adapter.infer(np.zeros((1, 3, 16, 16), dtype=np.float32))


def test_runtime_checks_shape_and_output_contract(tmp_path):
    artifact = artifact_fixture(tmp_path)
    adapter = J6RuntimeAdapter(
        artifact,
        board_available=True,
        runner=lambda _artifact, _tensor: {
            "scores": np.ones((1, 3), dtype=np.float32)
        },
    )
    assert adapter.infer(np.zeros((1, 3, 16, 16)))["scores"].shape == (1, 3)
    with pytest.raises(ValueError, match="input shape"):
        adapter.infer(np.zeros((1, 3, 8, 8)))


def test_artifact_sha_mismatch_fails_closed(tmp_path):
    artifact = artifact_fixture(tmp_path)
    artifact.hbm_path.write_bytes(b"tampered")
    with pytest.raises(ValueError, match="SHA-256"):
        artifact.validate()
