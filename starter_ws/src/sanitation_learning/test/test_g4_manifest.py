from __future__ import annotations

import json
from pathlib import Path
import sys

import pytest


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


from sanitation_learning.g4_manifest import (  # noqa: E402
    build_artifact_manifest,
    canonical_json,
    config_hash,
    load_freeze,
    validate_artifact_manifest,
    validate_freeze_payload,
)


def _freeze_payload() -> dict:
    model_config = {
        model_type: {
            "model_id": f"g4_{model_type}_v1",
            "input_name": model_type,
            "input_shape": [1, 3, 480, 640],
        }
        for model_type in ("discovery", "classifier", "leaf", "puddle")
    }
    sha = "a" * 64
    provenance = {
        model_type: {
            "pretrained": True,
            "from_scratch_control": False,
            "sha256": sha,
            "weight_enum": "Official_Weights.V1",
            "source_url": "https://download.pytorch.org/model.pth",
            "license": "BSD-3-Clause",
        }
        for model_type in model_config
    }
    onnx_contracts = {
        model_type: {
            "passed": True,
            "fixed_input": True,
            "opset": 17,
            "custom_ops": 0,
            "operator_inventory": {"Conv": 1},
        }
        for model_type in model_config
    }
    return {
        "schema_version": 1,
        "freeze_id": "freeze-20260808-001",
        "freeze_timestamp": "2026-08-08T00:00:00Z",
        "config_hash": config_hash(model_config),
        "model_config": model_config,
        "architecture_hashes": {"discovery": "arch-hash"},
        "preprocess_hashes": {"discovery": "pre-hash"},
        "postprocess_hashes": {"discovery": "post-hash"},
        "thresholds": {"discovery": {"score": 0.5}},
        "nms": {"discovery": {"iou": 0.5}},
        "calibration": {"discovery": {"temperature": 1.0}},
        "training_data_hashes": {"g4_train": "data-hash"},
        "validation_metrics": {"discovery": {"macro_f1": None}},
        "model_artifact_hashes": {
            model_type: sha for model_type in model_config
        },
        "pretrained_provenance": provenance,
        "onnx_contracts": onnx_contracts,
        "p4_screening": {
            "policy_id": "perception_p4_screening_policy",
            "pass": True,
            "evidence_sha256": sha,
        },
        "final_evaluator_sha256": sha,
    }


def test_config_hash_is_deterministic() -> None:
    payload = {"a": [1, 2], "b": {"x": "y"}}
    assert config_hash(payload) == config_hash(
        {"b": {"x": "y"}, "a": [1, 2]}
    )
    assert canonical_json(payload) == canonical_json(
        {"b": {"x": "y"}, "a": [1, 2]}
    )


def test_validate_freeze_fails_closed_on_missing_and_mismatch(tmp_path) -> None:
    payload = _freeze_payload()
    assert validate_freeze_payload(payload) is payload
    missing = dict(payload)
    del missing["freeze_id"]
    with pytest.raises(ValueError, match="missing required fields"):
        validate_freeze_payload(missing)
    mismatched = dict(payload)
    mismatched["config_hash"] = "wrong"
    with pytest.raises(ValueError, match="config_hash mismatch"):
        validate_freeze_payload(mismatched)
    no_p4 = _freeze_payload()
    no_p4["p4_screening"]["pass"] = False
    with pytest.raises(ValueError, match="P4 screening pass"):
        validate_freeze_payload(no_p4)
    from_scratch = _freeze_payload()
    from_scratch["pretrained_provenance"]["classifier"] = {
        "pretrained": False,
        "from_scratch_control": True,
    }
    with pytest.raises(ValueError, match="official pretrained"):
        validate_freeze_payload(from_scratch)
    freeze_path = tmp_path / "MODEL_FREEZE.json"
    freeze_path.write_text(json.dumps(payload), encoding="utf-8")
    assert load_freeze(freeze_path)["freeze_id"] == payload["freeze_id"]
    with pytest.raises(FileNotFoundError):
        load_freeze(tmp_path / "missing.json")


def test_build_and_validate_artifact_manifest(tmp_path) -> None:
    freeze = _freeze_payload()
    artifact = tmp_path / "discovery.onnx"
    artifact.write_bytes(b"\x00onnx-artifact")
    manifest = build_artifact_manifest(
        freeze,
        model_type="discovery",
        artifact_path=artifact,
        input_shape=(1, 3, 480, 640),
        output_shapes={
            "objectness_logits": [1, 3, 120, 160],
            "offset": [1, 6, 120, 160],
            "bbox_size": [1, 6, 120, 160],
        },
        operator_inventory={"Conv": 20},
        class_map={"class_agnostic": 0},
        provenance={"pretrained": True, "weight_enum": "ResNet18_Weights.IMAGENET1K_V1"},
        acceptance={"screening_pass": False, "status": "not_claimed"},
    )
    assert manifest["config_hash"] == freeze["config_hash"]
    assert manifest["artifact_sha256"]
    validated = validate_artifact_manifest(
        manifest,
        artifact_root=tmp_path,
        expected_config_hash=freeze["config_hash"],
    )
    assert validated is manifest
    corrupted = dict(manifest)
    corrupted["artifact_sha256"] = "0" * 64
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        validate_artifact_manifest(
            corrupted,
            artifact_root=tmp_path,
            expected_config_hash=freeze["config_hash"],
        )
    with pytest.raises(ValueError, match="config_hash mismatch"):
        validate_artifact_manifest(
            manifest,
            artifact_root=tmp_path,
            expected_config_hash="wrong",
        )
    missing_field = {key: value for key, value in manifest.items() if key != "acceptance"}
    with pytest.raises(ValueError, match="missing required fields"):
        validate_artifact_manifest(missing_field, artifact_root=tmp_path)


def test_build_manifest_missing_artifact_fails_closed(tmp_path) -> None:
    with pytest.raises(FileNotFoundError, match="model artifact not found"):
        build_artifact_manifest(
            _freeze_payload(),
            model_type="discovery",
            artifact_path=tmp_path / "missing.onnx",
            input_shape=(1, 3, 480, 640),
            output_shapes={},
            operator_inventory={},
            class_map={},
            provenance={},
            acceptance={},
        )
