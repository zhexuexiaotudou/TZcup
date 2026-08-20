from __future__ import annotations

import sys
from pathlib import Path

import pytest


torch = pytest.importorskip("torch")
PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from sanitation_learning.g4_losses import area_loss, discovery_loss


def test_area_boundary_loss_keeps_sparse_positive_gradient() -> None:
    logits = torch.full((1, 1, 32, 32), -3.0, requires_grad=True)
    boundary_logits = torch.full((1, 1, 32, 32), -3.0, requires_grad=True)
    target = torch.zeros((1, 1, 32, 32))
    boundary = torch.zeros((1, 1, 32, 32))
    target[:, :, 10:20, 10:20] = 1.0
    boundary[:, :, 10, 10:20] = 1.0
    report = area_loss(
        {"logits": logits, "boundary_logits": boundary_logits},
        target,
        boundary,
    )
    report["total"].backward()
    assert report["boundary_binary"] > 0
    assert report["boundary_dice"] > 0
    assert report["semantic_boundary_dice"] > 0
    assert boundary_logits.grad[0, 0, 10, 15] < 0


def test_semantic_boundary_loss_backpropagates_to_mask_logits() -> None:
    logits = torch.full((1, 1, 24, 24), -2.0, requires_grad=True)
    boundary_logits = torch.zeros((1, 1, 24, 24), requires_grad=True)
    target = torch.zeros((1, 1, 24, 24))
    target[:, :, 6:18, 6:18] = 1.0
    boundary = torch.zeros_like(target)
    boundary[:, :, 6, 6:18] = 1.0
    report = area_loss(
        {"logits": logits, "boundary_logits": boundary_logits},
        target,
        boundary,
        semantic_boundary_weight=1.0,
    )
    report["total"].backward()
    assert report["semantic_boundary_dice"] > 0
    assert logits.grad is not None
    assert torch.isfinite(logits.grad).all()


def test_pyramid_discovery_loss_backpropagates_every_level() -> None:
    outputs = {
        "objectness_logits": torch.zeros(1, 3, 32, 32, requires_grad=True),
        "offset": torch.zeros(1, 6, 32, 32, requires_grad=True),
        "bbox_size": torch.ones(1, 6, 32, 32, requires_grad=True),
    }
    targets = {}
    for stride, height, width in ((4, 32, 32), (8, 16, 16), (16, 8, 8)):
        targets[f"heatmap_s{stride}"] = torch.zeros(1, 1, height, width)
        targets[f"offset_s{stride}"] = torch.zeros(1, 2, height, width)
        targets[f"size_s{stride}"] = torch.ones(1, 2, height, width) * 4
        targets[f"regression_mask_s{stride}"] = torch.zeros(1, 1, height, width)
        targets[f"heatmap_s{stride}"][0, 0, height // 2, width // 2] = 1
        targets[f"regression_mask_s{stride}"][0, 0, height // 2, width // 2] = 1
    report = discovery_loss(outputs, targets)
    report["total"].backward()
    gradients = outputs["objectness_logits"].grad.abs().sum(dim=(0, 2, 3))
    assert torch.all(gradients > 0)
