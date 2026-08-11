#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g6_opr_b import ANCHOR_SIZES, ASPECT_RATIOS  # noqa: E402


def test_opr_b_anchor_contract_starts_at_native_small_scale() -> None:
    assert ANCHOR_SIZES[0] == (8, 12, 16)
    assert len(ANCHOR_SIZES) == 5
    assert ASPECT_RATIOS == ((0.5, 1.0, 2.0),) * 5


def test_opr_b_builds_four_class_two_stage_head_without_download() -> None:
    if importlib.util.find_spec("torch") is None:
        pytest.skip("host fast-test environment does not include torch")
    from sanitation_learning.g6_opr_b import build_opr_b

    model = build_opr_b(weights_required=False)
    assert model.roi_heads.box_predictor.cls_score.out_features == 4
    assert model.rpn.anchor_generator.sizes[0] == (8, 12, 16)
    assert model.model_id.startswith("opr_b_fasterrcnn")
