from __future__ import annotations

import sys
from pathlib import Path

import pytest


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


def test_objectness_loss_variants_are_finite_and_auditable() -> None:
    torch = pytest.importorskip("torch")
    from sanitation_learning.g4_losses import (
        OBJECTNESS_LOSS_VARIANTS,
        objectness_loss_audit,
    )

    logits = torch.zeros(2, 1, 8, 8, requires_grad=True)
    target = torch.zeros_like(logits)
    target[0, 0, 2, 3] = 1.0
    target[1, 0, 5, 4] = 1.0
    for variant in OBJECTNESS_LOSS_VARIANTS:
        audit = objectness_loss_audit(logits, target, variant=variant)
        assert set(audit) == {
            "total",
            "positive",
            "negative",
            "hard_negative",
            "positive_count",
        }
        assert torch.isfinite(audit["total"])
        assert audit["positive"] > 0
        assert audit["negative"] > 0
        assert audit["hard_negative"] > 0


def test_l2_negative_term_is_independent_of_positive_count() -> None:
    torch = pytest.importorskip("torch")
    from sanitation_learning.g4_losses import objectness_loss_audit

    logits = torch.zeros(1, 1, 8, 8)
    one = torch.zeros_like(logits)
    one[0, 0, 1, 1] = 1.0
    many = one.clone()
    many[0, 0, 2, 2] = 1.0
    one_audit = objectness_loss_audit(
        logits, one, variant="L2_independent_ohem"
    )
    many_audit = objectness_loss_audit(
        logits, many, variant="L2_independent_ohem"
    )
    assert float(one_audit["hard_negative"]) == pytest.approx(
        float(many_audit["hard_negative"]), rel=1e-6
    )


def test_unknown_objectness_loss_variant_fails_closed() -> None:
    torch = pytest.importorskip("torch")
    from sanitation_learning.g4_losses import objectness_loss_audit

    with pytest.raises(ValueError):
        objectness_loss_audit(
            torch.zeros(1, 1, 2, 2),
            torch.zeros(1, 1, 2, 2),
            variant="not-a-loss",
        )


def test_quality_focal_is_autocast_safe() -> None:
    torch = pytest.importorskip("torch")
    if not torch.cuda.is_available():
        pytest.skip("CUDA autocast regression requires CUDA")
    from sanitation_learning.g4_losses import objectness_loss_audit

    logits = torch.zeros(1, 1, 8, 8, device="cuda", requires_grad=True)
    target = torch.zeros_like(logits)
    target[0, 0, 3, 4] = 1.0
    with torch.autocast(device_type="cuda", dtype=torch.float16):
        loss = objectness_loss_audit(
            logits, target, variant="L3_quality_focal"
        )["total"]
    loss.backward()
    assert torch.isfinite(loss)
    assert torch.isfinite(logits.grad).all()


def test_a2_objectness_and_quality_heads_receive_independent_gradients() -> None:
    torch = pytest.importorskip("torch")
    from sanitation_learning.g4_losses import discovery_loss

    raw = torch.zeros(1, 1, 8, 8, requires_grad=True)
    quality = torch.zeros(1, 1, 8, 8, requires_grad=True)
    combined = raw + quality
    offset = torch.zeros(1, 2, 8, 8, requires_grad=True)
    size = torch.ones(1, 2, 8, 8, requires_grad=True)
    heatmap = torch.zeros(1, 1, 8, 8)
    heatmap[0, 0, 3, 4] = 1.0
    mask = torch.zeros(1, 1, 8, 8)
    mask[0, 0, 3, 4] = 1.0
    teacher_quality = torch.zeros(1, 1, 8, 8)
    teacher_quality[0, 0, 3, 4] = 0.73
    report = discovery_loss(
        {
            "objectness_logits": combined,
            "objectness_raw_logits": raw,
            "quality_logits": quality,
            "offset": offset,
            "bbox_size": size,
        },
        {
            "heatmap": heatmap,
            "offset": torch.zeros(1, 2, 8, 8),
            "size": torch.ones(1, 2, 8, 8),
            "regression_mask": mask,
            "teacher_quality": teacher_quality,
        },
    )
    report["total"].backward()
    assert report["quality"] > 0
    assert raw.grad is not None and torch.isfinite(raw.grad).all()
    assert quality.grad is not None and torch.isfinite(quality.grad).all()
    assert float(raw.grad.abs().sum()) > 0.0
    assert float(quality.grad.abs().sum()) > 0.0


def test_discovery_box_size_has_direct_regression_gradient() -> None:
    torch = pytest.importorskip("torch")
    from sanitation_learning.g4_losses import discovery_loss

    size = torch.full((1, 2, 8, 8), 0.25, requires_grad=True)
    heatmap = torch.zeros(1, 1, 8, 8)
    heatmap[0, 0, 3, 4] = 1.0
    mask = torch.zeros(1, 1, 8, 8)
    mask[0, 0, 3, 4] = 1.0
    target_size = torch.zeros(1, 2, 8, 8)
    target_size[0, :, 3, 4] = torch.tensor([8.0, 10.0])
    report = discovery_loss(
        {
            "objectness_logits": torch.zeros(1, 1, 8, 8),
            "offset": torch.zeros(1, 2, 8, 8),
            "bbox_size": size,
        },
        {
            "heatmap": heatmap,
            "offset": torch.zeros(1, 2, 8, 8),
            "size": target_size,
            "regression_mask": mask,
        },
    )
    report["total"].backward()
    assert float(report["size"]) > 0.0
    assert size.grad is not None
    assert float(size.grad[0, :, 3, 4].abs().sum()) > 0.0
    assert float(size.grad[..., :3, :].abs().sum()) == 0.0
