"""AUTO-05R-2/3 task-specific losses.

The losses are kept separate from the model definitions so micro-overfit,
screening and future architecture attempts can be ablated without touching the
network code.
"""

from __future__ import annotations

import torch
import torch.nn.functional as functional


OBJECTNESS_LOSS_VARIANTS = (
    "L1_centernet",
    "L2_independent_ohem",
    "L3_quality_focal",
)


def objectness_loss_audit(
    logits: torch.Tensor,
    target: torch.Tensor,
    *,
    variant: str = "L2_independent_ohem",
    negative_weight: float = 1.0,
) -> dict[str, torch.Tensor]:
    """Return auditable positive, negative and hard-negative contributions.

    L1 is the standard CenterNet normalization. L2 keeps positive
    normalization independent from an OHEM negative mean. L3 is a
    quality-focal-style continuous heatmap objective with the same independent
    hard-negative term. The explicit components prevent coefficient changes
    from hiding a collapsed negative contribution.
    """
    if variant not in OBJECTNESS_LOSS_VARIANTS:
        raise ValueError(f"unknown objectness loss variant {variant!r}")
    prediction = torch.sigmoid(logits).clamp(1e-5, 1.0 - 1e-5)
    positive = target.eq(1.0).float()
    negative = target.lt(1.0).float()
    gaussian_negative_weight = (1.0 - target).pow(4)
    if variant == "L3_quality_focal":
        modulation = torch.abs(target - prediction).pow(2)
        elementwise = torch.nn.functional.binary_cross_entropy_with_logits(
            logits, target, reduction="none"
        ) * modulation
        positive_loss = elementwise * positive
        negative_loss = elementwise * gaussian_negative_weight * negative
    else:
        positive_loss = -(prediction.log()) * (1.0 - prediction).pow(2) * positive
        negative_loss = (
            -((1.0 - prediction).log())
            * prediction.pow(2)
            * gaussian_negative_weight
            * negative
        )
    batch_size = max(int(logits.shape[0]), 1)
    negative_flat = negative_loss.reshape(batch_size, -1)
    topk_count = max(1, int(negative_flat.shape[1] * 0.02))
    topk, _ = torch.topk(negative_flat, topk_count, dim=1)
    positive_count = positive.sum().clamp(min=1.0)
    positive_contribution = positive_loss.sum() / positive_count
    negative_contribution = negative_loss.sum() / positive_count
    hard_negative_contribution = topk.mean()
    if variant == "L1_centernet":
        total = positive_contribution + negative_contribution
    else:
        total = (
            positive_contribution
            + float(negative_weight) * hard_negative_contribution
        )
    return {
        "total": total,
        "positive": positive_contribution,
        "negative": negative_contribution,
        "hard_negative": hard_negative_contribution,
        "positive_count": positive_count,
    }


def focal_objectness_loss(
    logits: torch.Tensor,
    target: torch.Tensor,
    *,
    variant: str = "L2_independent_ohem",
    negative_weight: float = 1.0,
) -> torch.Tensor:
    return objectness_loss_audit(
        logits,
        target,
        variant=variant,
        negative_weight=negative_weight,
    )["total"]


def _generalized_iou(predicted: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
    pred_x1, pred_y1, pred_x2, pred_y2 = predicted.unbind(dim=-1)
    target_x1, target_y1, target_x2, target_y2 = target.unbind(dim=-1)
    intersection_x1 = torch.maximum(pred_x1, target_x1)
    intersection_y1 = torch.maximum(pred_y1, target_y1)
    intersection_x2 = torch.minimum(pred_x2, target_x2)
    intersection_y2 = torch.minimum(pred_y2, target_y2)
    intersection = (
        torch.clamp(intersection_x2 - intersection_x1, min=0.0)
        * torch.clamp(intersection_y2 - intersection_y1, min=0.0)
    )
    pred_area = torch.clamp(pred_x2 - pred_x1, min=0.0) * torch.clamp(
        pred_y2 - pred_y1, min=0.0
    )
    target_area = torch.clamp(target_x2 - target_x1, min=0.0) * torch.clamp(
        target_y2 - target_y1, min=0.0
    )
    union = pred_area + target_area - intersection
    iou = intersection / union.clamp(min=1e-6)
    enclosure_x1 = torch.minimum(pred_x1, target_x1)
    enclosure_y1 = torch.minimum(pred_y1, target_y1)
    enclosure_x2 = torch.maximum(pred_x2, target_x2)
    enclosure_y2 = torch.maximum(pred_y2, target_y2)
    enclosure = (
        torch.clamp(enclosure_x2 - enclosure_x1, min=0.0)
        * torch.clamp(enclosure_y2 - enclosure_y1, min=0.0)
    )
    return iou - (enclosure - union) / enclosure.clamp(min=1e-6)


def discovery_loss(
    outputs: dict[str, torch.Tensor],
    targets: dict[str, torch.Tensor],
    *,
    stride: int = 4,
    objectness_variant: str = "L2_independent_ohem",
    objectness_negative_weight: float = 1.0,
) -> dict[str, torch.Tensor]:
    objectness_audit = objectness_loss_audit(
        outputs["objectness_logits"],
        targets["heatmap"],
        variant=objectness_variant,
        negative_weight=objectness_negative_weight,
    )
    objectness = objectness_audit["total"]
    mask = targets["regression_mask"]
    denominator = mask.sum().clamp(min=1.0)
    offset_loss = (
        (torch.abs(outputs["offset"] - targets["offset"]) * mask).sum()
        / denominator
    )
    _, _, height, width = outputs["objectness_logits"].shape
    yy, xx = torch.meshgrid(
        torch.arange(height, device=mask.device),
        torch.arange(width, device=mask.device),
        indexing="ij",
    )
    xx = xx[None].float()
    yy = yy[None].float()
    predicted_size = torch.clamp(outputs["bbox_size"], min=1e-4)
    target_size = torch.clamp(targets["size"], min=1e-4)
    predicted_boxes = torch.stack(
        (
            (xx + outputs["offset"][:, 0]) * stride
            - predicted_size[:, 0] * stride * 0.5,
            (yy + outputs["offset"][:, 1]) * stride
            - predicted_size[:, 1] * stride * 0.5,
            (xx + outputs["offset"][:, 0]) * stride
            + predicted_size[:, 0] * stride * 0.5,
            (yy + outputs["offset"][:, 1]) * stride
            + predicted_size[:, 1] * stride * 0.5,
        ),
        dim=-1,
    )
    target_boxes = torch.stack(
        (
            (xx + targets["offset"][:, 0]) * stride
            - target_size[:, 0] * stride * 0.5,
            (yy + targets["offset"][:, 1]) * stride
            - target_size[:, 1] * stride * 0.5,
            (xx + targets["offset"][:, 0]) * stride
            + target_size[:, 0] * stride * 0.5,
            (yy + targets["offset"][:, 1]) * stride
            + target_size[:, 1] * stride * 0.5,
        ),
        dim=-1,
    )
    giou = _generalized_iou(predicted_boxes, target_boxes)
    giou_loss = (1.0 - giou) * mask.squeeze(1)
    giou_loss = giou_loss.sum() / denominator
    probability = torch.sigmoid(outputs["objectness_logits"])
    negative_penalty = probability[targets["heatmap"] < 1.0].pow(2).mean()
    total = (
        objectness
        + offset_loss
        + 0.25 * giou_loss
        + 0.1 * negative_penalty
    )
    return {
        "total": total,
        "objectness": objectness,
        "offset": offset_loss,
        "giou": giou_loss,
        "negative_penalty": negative_penalty,
        "objectness_positive": objectness_audit["positive"],
        "objectness_negative": objectness_audit["negative"],
        "objectness_hard_negative": objectness_audit["hard_negative"],
    }


def classifier_loss(
    logits: torch.Tensor,
    targets: torch.Tensor,
    *,
    weights: tuple[float, ...] = (0.6, 1.0, 1.0, 1.2),
) -> torch.Tensor:
    tensor = torch.tensor(weights, dtype=logits.dtype, device=logits.device)
    return functional.cross_entropy(logits, targets, weight=tensor)


def area_loss(
    outputs: dict[str, torch.Tensor],
    targets: torch.Tensor,
    boundary_targets: torch.Tensor,
    *,
    boundary_weight: float = 0.35,
    negative_weight: float = 1.0,
    negative_ratio: float = 0.01,
    tversky_alpha: float = 0.3,
    tversky_beta: float = 0.8,
    boundary_pixel_weight: float = 2.0,
) -> dict[str, torch.Tensor]:
    logits = outputs["logits"]
    positive_counts = targets.sum(dim=(0, 2, 3))
    pixel_count = targets.shape[0] * targets.shape[2] * targets.shape[3]
    pos_weight = torch.clamp(
        (pixel_count - positive_counts) / positive_counts.clamp(min=1.0),
        min=1.0,
        max=40.0,
    )
    pixel_weight = 1.0 + boundary_pixel_weight * boundary_targets
    binary = functional.binary_cross_entropy_with_logits(
        logits,
        targets,
        pos_weight=pos_weight.view(1, -1, 1, 1),
        weight=pixel_weight,
    )
    probability = torch.sigmoid(logits)
    intersection = (probability * targets).sum(dim=(0, 2, 3))
    false_positive = (probability * (1.0 - targets)).sum(dim=(0, 2, 3))
    false_negative = ((1.0 - probability) * targets).sum(dim=(0, 2, 3))
    denominator = (
        intersection
        + tversky_alpha * false_positive
        + tversky_beta * false_negative
    )
    tversky = (intersection + 1.0) / (denominator + 1.0)
    tversky_loss = 1.0 - tversky.mean()
    boundary_logits = outputs["boundary_logits"]
    boundary_loss = functional.binary_cross_entropy_with_logits(
        boundary_logits, boundary_targets
    )
    negative_logits = logits[targets == 0].flatten()
    if negative_logits.numel() > 0:
        top_count = max(1, int(negative_logits.numel() * negative_ratio))
        top_count = min(top_count, negative_logits.numel())
        top_negative, _ = torch.topk(negative_logits, top_count)
        negative_penalty = torch.sigmoid(top_negative).pow(2).mean()
    else:
        negative_penalty = logits.new_tensor(0.0)
    total = (
        binary
        + tversky_loss
        + boundary_weight * boundary_loss
        + negative_weight * negative_penalty
    )
    return {
        "total": total,
        "binary": binary,
        "tversky": tversky_loss,
        "boundary": boundary_loss,
        "negative_penalty": negative_penalty,
    }


__all__ = [
    "OBJECTNESS_LOSS_VARIANTS",
    "area_loss",
    "classifier_loss",
    "discovery_loss",
    "focal_objectness_loss",
    "objectness_loss_audit",
]
