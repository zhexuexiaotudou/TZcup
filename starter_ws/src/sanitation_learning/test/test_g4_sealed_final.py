from __future__ import annotations

import json
from pathlib import Path
import sys

import pytest


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


from sanitation_learning.g4_manifest import config_hash  # noqa: E402
from sanitation_learning.g4_sealed_final import (  # noqa: E402
    SEALED_MIN_FRAMES,
    SEALED_MIN_SCENES,
    SEALED_MIN_WORLDS,
    SealedFinalGate,
    SealedFinalReuseError,
    validate_sealed_manifest,
)


def _freeze(tmp_path: Path) -> dict:
    model_config = {
        model_type: {
            "model_id": f"g4_{model_type}_v1",
            "input_shape": [1, 3, 480, 640],
        }
        for model_type in ("discovery", "classifier", "leaf", "puddle")
    }
    sha = "a" * 64
    payload = {
        "schema_version": 1,
        "freeze_id": "freeze-g5-001",
        "freeze_timestamp": "2026-08-08T00:00:00Z",
        "config_hash": config_hash(model_config),
        "model_config": model_config,
        "architecture_hashes": {"discovery": "a"},
        "preprocess_hashes": {"discovery": "b"},
        "postprocess_hashes": {"discovery": "c"},
        "thresholds": {"discovery": {"score": 0.5}},
        "nms": {"discovery": {"iou": 0.5}},
        "calibration": {"discovery": {}},
        "training_data_hashes": {"g4_train": "d"},
        "validation_metrics": {"discovery": {}},
        "model_artifact_hashes": {
            model_type: sha for model_type in model_config
        },
        "pretrained_provenance": {
            model_type: {
                "pretrained": True,
                "from_scratch_control": False,
                "sha256": sha,
                "weight_enum": "Official_Weights.V1",
                "source_url": "https://download.pytorch.org/model.pth",
                "license": "BSD-3-Clause",
            }
            for model_type in model_config
        },
        "onnx_contracts": {
            model_type: {
                "passed": True,
                "fixed_input": True,
                "opset": 17,
                "custom_ops": 0,
                "operator_inventory": {"Conv": 1},
            }
            for model_type in model_config
        },
        "p4_screening": {
            "policy_id": "perception_p4_screening_policy",
            "pass": True,
            "evidence_sha256": sha,
        },
        "final_evaluator_sha256": sha,
    }
    path = tmp_path / "MODEL_FREEZE.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return payload


def _sealed_manifest() -> dict:
    manifest = {
        "schema_version": 1,
        "dataset_id": "G5_SEALED_FINAL",
        "worlds": [f"g5_world_{index:02d}" for index in range(6)],
        "scenes": 120,
        "frames": 1200,
        "target_assets": [f"g5_target_{index:03d}" for index in range(60)],
        "hard_negative_assets": [
            f"g5_neg_{index:03d}" for index in range(60)
        ],
        "sealed_by": "test-fixture",
    }
    manifest["manifest_sha256"] = config_hash(manifest)
    return manifest


def test_sealed_contract_minimums() -> None:
    assert SEALED_MIN_WORLDS == 4
    assert SEALED_MIN_SCENES == 100
    assert SEALED_MIN_FRAMES == 1000


def test_validate_sealed_manifest_accepts_synthetic_metadata(tmp_path) -> None:
    freeze = _freeze(tmp_path)
    manifest = _sealed_manifest()
    validated = validate_sealed_manifest(
        manifest,
        freeze,
        development_world_ids=["dev_world_1", "dev_world_2"],
        development_target_assets=["dev_target_1"],
        development_hard_negative_assets=["dev_neg_1"],
    )
    assert validated is manifest


def test_validate_sealed_manifest_rejects_contracted_sets(tmp_path) -> None:
    freeze = _freeze(tmp_path)
    too_few_worlds = _sealed_manifest()
    too_few_worlds["worlds"] = ["w1", "w2", "w3"]
    with pytest.raises(ValueError, match="at least 4 unseen worlds"):
        validate_sealed_manifest(
            too_few_worlds,
            freeze,
            development_world_ids=[],
            development_target_assets=[],
            development_hard_negative_assets=[],
        )
    seen_world = _sealed_manifest()
    seen_world["worlds"][0] = "dev_world_1"
    with pytest.raises(ValueError, match="unseen in development"):
        validate_sealed_manifest(
            seen_world,
            freeze,
            development_world_ids=["dev_world_1"],
            development_target_assets=[],
            development_hard_negative_assets=[],
        )
    seen_target = _sealed_manifest()
    seen_target["target_assets"][0] = "dev_target_1"
    with pytest.raises(ValueError, match="target assets must be unseen"):
        validate_sealed_manifest(
            seen_target,
            freeze,
            development_world_ids=[],
            development_target_assets=["dev_target_1"],
            development_hard_negative_assets=[],
        )
    seen_hard = _sealed_manifest()
    seen_hard["hard_negative_assets"][0] = "dev_neg_1"
    with pytest.raises(ValueError, match="hard-negative assets must be unseen"):
        validate_sealed_manifest(
            seen_hard,
            freeze,
            development_world_ids=[],
            development_target_assets=[],
            development_hard_negative_assets=["dev_neg_1"],
        )

    bad_hash = _sealed_manifest()
    bad_hash["frames"] += 1
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        validate_sealed_manifest(
            bad_hash,
            freeze,
            development_world_ids=[],
            development_target_assets=[],
            development_hard_negative_assets=[],
        )


def test_sealed_gate_is_one_shot_and_refuses_rerun(tmp_path) -> None:
    freeze = _freeze(tmp_path)
    manifest = _sealed_manifest()
    gate = SealedFinalGate(tmp_path / "evidence")
    access = gate.open_once(
        freeze_path=tmp_path / "MODEL_FREEZE.json",
        sealed_manifest=manifest,
        development_world_ids=[],
        development_target_assets=[],
        development_hard_negative_assets=[],
    )
    assert access["event"] == "sealed_final_first_access"
    assert (tmp_path / "evidence" / "sealed_final_access.json").is_file()
    with pytest.raises(SealedFinalReuseError):
        gate.open_once(
            freeze_path=tmp_path / "MODEL_FREEZE.json",
            sealed_manifest=manifest,
            development_world_ids=[],
            development_target_assets=[],
            development_hard_negative_assets=[],
        )
    result = gate.evaluate_once(
        metrics={"discrete_macro_f1": 0.96},
        freeze_id=freeze["freeze_id"],
    )
    assert result["one_shot"] is True
    with pytest.raises(SealedFinalReuseError):
        gate.evaluate_once(
            metrics={"discrete_macro_f1": 0.99},
            freeze_id=freeze["freeze_id"],
        )


def test_evaluation_without_access_refused(tmp_path) -> None:
    gate = SealedFinalGate(tmp_path / "evidence2")
    with pytest.raises(SealedFinalReuseError, match="before evaluation"):
        gate.evaluate_once(metrics={}, freeze_id="x")


def test_partial_probe_consumes_one_shot(tmp_path) -> None:
    freeze = _freeze(tmp_path)
    manifest = _sealed_manifest()
    gate = SealedFinalGate(tmp_path / "evidence3")
    gate.open_once(
        freeze_path=tmp_path / "MODEL_FREEZE.json",
        sealed_manifest=manifest,
        development_world_ids=[],
        development_target_assets=[],
        development_hard_negative_assets=[],
    )
    # Any second open (even without evaluation) is a forbidden partial probe.
    with pytest.raises(SealedFinalReuseError):
        gate.open_once(
            freeze_path=tmp_path / "MODEL_FREEZE.json",
            sealed_manifest=manifest,
            development_world_ids=[],
            development_target_assets=[],
            development_hard_negative_assets=[],
        )
