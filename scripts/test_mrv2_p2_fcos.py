from __future__ import annotations

import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))


def test_p2_builder_exposes_stride4_and_six_scale_anchors():
    torch = pytest.importorskip("torch")
    pytest.importorskip("torchvision")
    from sanitation_learning.g4_direct_fcos import build_p2_direct_fcos

    model = build_p2_direct_fcos(input_size=(960, 720))
    with torch.no_grad():
        features = model.backbone(torch.zeros(1, 3, 720, 960))
    assert features["0"].shape[-2:] == (180, 240)
    assert model.anchor_generator.sizes == ((4,), (8,), (16,), (32,), (64,), (128,))


def test_p2_transplant_preserves_trained_head_and_shifts_fpn_levels():
    torch = pytest.importorskip("torch")
    pytest.importorskip("torchvision")
    from sanitation_learning.g4_direct_fcos import (
        build_direct_fcos,
        build_p2_direct_fcos,
        load_direct_state_into_p2,
    )

    source = build_direct_fcos(input_size=(640, 480)).state_dict()
    target = build_p2_direct_fcos(input_size=(960, 720))
    report = load_direct_state_into_p2(target, source)
    state = target.state_dict()
    assert torch.equal(
        state["head.classification_head.cls_logits.weight"],
        source["head.classification_head.cls_logits.weight"],
    )
    assert torch.equal(
        state["backbone.fpn.inner_blocks.1.0.weight"],
        source["backbone.fpn.inner_blocks.0.0.weight"],
    )
    assert report["new_p2_tensor_names"]
