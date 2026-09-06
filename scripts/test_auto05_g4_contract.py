#!/usr/bin/env python3
"""Static, dependency-free guardrails for the unexecuted G4 recovery path."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "starter_ws" / "src" / "sanitation_learning" / "config" / "auto05_g4_screening.yaml"
SCREENING = ROOT / "scripts" / "auto05_screening.py"
CAPTURE = ROOT / "scripts" / "run_auto05_g4_capture_runtime.sh"
FINALIZER = ROOT / "scripts" / "finalize_auto05_g4.py"
RUNTIME_GATE = ROOT / "scripts" / "bind_auto05_g4_runtime.py"
PIPELINE = ROOT / "scripts" / "run_auto05_g4_pipeline.sh"


def test_contract_is_single_config_and_validation_only() -> None:
    text = CONFIG.read_text(encoding="utf-8")
    for required in (
        "direct_anchor_free_center_offset_bbox",
        "test_used_for_model_selection: false",
        "maximum_configs_per_architecture: 1",
        "frozen_test_runs: 1",
        "segmentation_connected_components_used_as_detector: false",
    ):
        assert required in text


def test_screening_rejects_blind_g4_and_test_selection() -> None:
    text = SCREENING.read_text(encoding="utf-8")
    assert "G4 requires --g4-contract; blind attempt 4 is forbidden" in text
    assert 'require_split(rows, "val", "detector threshold selection")' in text
    assert 'require_split(rows, "val", "area threshold selection")' in text
    assert 'require_split(rows, "train", "detector training")' in text
    assert "G4DirectDetector" in text and "g4_giou_loss" in text
    assert "G4IndependentAreaHeads" in text


def test_capture_wrapper_refuses_outside_work_and_stage1_fallback() -> None:
    text = CAPTURE.read_text(encoding="utf-8")
    assert '"$repo/.work"/*' in text
    assert "AUTO05_G4_RUNTIME_BOUND=1" in text
    assert "AUTO05_COMBINED_RUNTIME_SETUP" in text
    assert "formal_runtime_configure" in text
    assert "FORMAL_RUNTIME_LOCK_FILE" in text


def test_finalizer_is_review_gated_and_cannot_overwrite_history() -> None:
    text = FINALIZER.read_text(encoding="utf-8")
    assert 'repo / ".work" / "auto05-g4"' in text
    assert '"historical_evidence_overwritten": False' in text
    assert '"auto06_authorized": False' in text


def test_pipeline_reuses_formal_runtime_gate_and_one_test_lock() -> None:
    assert "build_binding" in RUNTIME_GATE.read_text(encoding="utf-8")
    text = PIPELINE.read_text(encoding="utf-8")
    assert "run_auto05_g4_capture_runtime.sh" in text
    assert "auto05_finalize_dataset.py" in text
    assert "g4_attempt_ledger.json" in text
    assert "g4_test_consumed_lock.json" in text
    assert "screening_image.json" in text


def test_g4_image_build_executes_real_runtime_parity_test() -> None:
    text = (ROOT / "scripts" / "build_auto05_g4_screening_image.sh").read_text(encoding="utf-8")
    assert "test_auto05_g4_torch_runtime.py" in text
    assert '"runtime_parity_test_passed": True' in text


if __name__ == "__main__":
    import pytest

    raise SystemExit(pytest.main([__file__]))
