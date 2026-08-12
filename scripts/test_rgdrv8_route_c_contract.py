#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_route_c_context_preserves_fixed_proposals_and_val_boundary() -> None:
    source = (ROOT / "scripts/build_rgdrv8_route_c_crops.py").read_text()
    assert '"proposal_coordinates_changed": False' in source
    assert '"proposal_labels_changed": False' in source
    assert '"HOLDOUT_proposals_fixed_once": True' in source
    assert '"VAL_NEW_read": False' in source
    assert "square_crop" in source and "scale=6.0" in source
    assert '"source_train.json"' in source


def test_route_c_verifier_is_single_model_hard_negative_extension() -> None:
    source = (ROOT / "scripts/train_rgdrv8_route_c_verifier.py").read_text()
    assert "CandidateCropClassifier" in source
    assert "fixed_proposals_square_context_scale_6" in source
    assert "background_specificity" in source
    assert '"VAL_NEW_read": False' in source
    assert "ROUTE_C_VERIFIER_HOLDOUT_PASS" in source
