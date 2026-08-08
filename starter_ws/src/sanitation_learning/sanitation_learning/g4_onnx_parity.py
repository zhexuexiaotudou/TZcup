"""Task-specific ONNX parity evaluators.

Generic tensor maximum error is not sufficient for product screening:

- discovery: decoded candidate count, box and score agreement;
- classifier: top-1 agreement and maximum probability error;
- segmenter: binary-mask IoU/pixel agreement plus boundary-mask agreement.

All evaluators are pure NumPy so they can be tested without a torch backend.
The torch-facing wrapper in ``g4_models`` converts tensors and delegates here.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .auto04_contract import box_iou


def _as_float_array(value: Any) -> np.ndarray:
    return np.asarray(value, dtype=np.float32)


def _softmax(logits: np.ndarray) -> np.ndarray:
    shifted = logits - logits.max(axis=-1, keepdims=True)
    exp = np.exp(shifted)
    return exp / exp.sum(axis=-1, keepdims=True)


def _decode_discovery_candidates(
    flat: np.ndarray,
    *,
    stride: int = 4,
    score_threshold: float = 0.75,
):
    from .auto04_contract import decode_centernet_outputs

    flat = _as_float_array(flat)
    if flat.ndim != 4 or flat.shape[1] != 5:
        raise ValueError("discovery flat output must be Nx5xHxW")
    objectness = 1.0 / (1.0 + np.exp(-flat[:, 0:1]))
    return decode_centernet_outputs(
        objectness[0],
        flat[:, 1:3][0],
        flat[:, 3:5][0],
        stride=stride,
        score_threshold=score_threshold,
    )


def discovery_parity(
    torch_flat,
    onnx_flat,
    *,
    stride: int = 4,
    score_threshold: float = 0.75,
    max_score_error: float = 1e-4,
    max_bbox_coordinate_error_px: float = 0.5,
) -> dict:
    """Compare decoded discovery candidates between torch and ONNX."""
    torch_detections = _decode_discovery_candidates(
        torch_flat, stride=stride, score_threshold=score_threshold
    )
    onnx_detections = _decode_discovery_candidates(
        onnx_flat, stride=stride, score_threshold=score_threshold
    )
    count_agreement = len(torch_detections) == len(onnx_detections)
    matched = 0
    used: set[int] = set()
    max_box_error = 0.0
    max_det_score_error = 0.0
    for det in torch_detections:
        best_index = -1
        best_iou = 0.0
        for index, other in enumerate(onnx_detections):
            if index in used:
                continue
            if det.class_index != other.class_index:
                continue
            iou = box_iou(det.bbox_xyxy, other.bbox_xyxy)
            if iou > best_iou:
                best_iou = iou
                best_index = index
        if best_index >= 0 and best_iou >= 0.9:
            used.add(best_index)
            matched += 1
            other = onnx_detections[best_index]
            max_box_error = max(
                max_box_error,
                float(
                    max(
                        abs(a - b)
                        for a, b in zip(det.bbox_xyxy, other.bbox_xyxy)
                    )
                ),
            )
            max_det_score_error = max(
                max_det_score_error, abs(det.score - other.score)
            )
    candidate_agreement = (
        1.0
        if not torch_detections and not onnx_detections
        else matched / max(len(torch_detections), len(onnx_detections), 1)
    )
    passed = bool(
        count_agreement
        and candidate_agreement >= 0.9999
        and max_box_error <= max_bbox_coordinate_error_px
        and max_det_score_error <= max_score_error
    )
    return {
        "decoded_agreement": passed,
        "decoded_candidate_count_agreement": count_agreement,
        "decoded_candidate_agreement": float(candidate_agreement),
        "torch_candidate_count": len(torch_detections),
        "onnx_candidate_count": len(onnx_detections),
        "max_bbox_coordinate_error_px": float(max_box_error),
        "max_bbox_coordinate_error_threshold_px": float(
            max_bbox_coordinate_error_px
        ),
        "max_detection_score_error": float(max_det_score_error),
        "score_threshold": score_threshold,
    }


def classifier_parity(
    torch_flat,
    onnx_flat,
    *,
    max_probability_error: float = 1e-4,
) -> dict:
    """Compare classifier top-1 class and softmax probabilities."""
    torch_np = _as_float_array(torch_flat)
    onnx_np = _as_float_array(onnx_flat)
    if torch_np.shape != onnx_np.shape:
        raise ValueError(
            "classifier parity requires identical shapes: "
            f"{torch_np.shape} vs {onnx_np.shape}"
        )
    if torch_np.ndim == 4 and torch_np.shape[1] > 1:
        torch_np = torch_np[:, 0] if torch_np.shape[1] == 1 else torch_np
        torch_prob = _softmax(torch_np.transpose(0, 2, 3, 1).reshape(-1, torch_np.shape[1]))
        onnx_prob = _softmax(
            onnx_np.transpose(0, 2, 3, 1).reshape(-1, onnx_np.shape[1])
        )
        torch_top1 = torch_prob.argmax(axis=-1)
        onnx_top1 = onnx_prob.argmax(axis=-1)
    else:
        logits = np.atleast_2d(torch_np)
        onnx_logits = np.atleast_2d(onnx_np)
        if logits.shape[1] <= 1:
            raise ValueError("classifier output must have at least 2 classes")
        torch_prob = _softmax(logits)
        onnx_prob = _softmax(onnx_logits)
        torch_top1 = torch_prob.argmax(axis=-1)
        onnx_top1 = onnx_prob.argmax(axis=-1)
    top1_agreement = float((torch_top1 == onnx_top1).mean())
    measured_probability_error = float(
        np.abs(torch_prob - onnx_prob).max()
    )
    return {
        "argmax_agreement": top1_agreement,
        "top1_agreement": top1_agreement,
        "max_probability_error": measured_probability_error,
        "max_probability_error_threshold": float(max_probability_error),
        "passed": bool(
            top1_agreement == 1.0
            and measured_probability_error <= max_probability_error
        ),
    }


def segmenter_parity(
    torch_flat,
    onnx_flat,
    *,
    mask_threshold: float = 0.5,
    min_binary_agreement: float = 0.9999,
    min_boundary_agreement: float = 0.999,
) -> dict:
    """Compare segmenter binary masks and boundary masks."""
    torch_np = _as_float_array(torch_flat)
    onnx_np = _as_float_array(onnx_flat)
    if torch_np.shape != onnx_np.shape:
        raise ValueError(
            "segmenter parity requires identical shapes: "
            f"{torch_np.shape} vs {onnx_np.shape}"
        )
    if torch_np.ndim != 4 or torch_np.shape[1] != 2:
        raise ValueError(
            "segmenter flat output must be Nx2xHxW "
            "(logits + boundary logits)"
        )
    torch_mask = 1.0 / (1.0 + np.exp(-torch_np[:, 0:1])) > mask_threshold
    onnx_mask = 1.0 / (1.0 + np.exp(-onnx_np[:, 0:1])) > mask_threshold
    intersection = int((torch_mask & onnx_mask).sum())
    union = int((torch_mask | onnx_mask).sum())
    binary_iou = 1.0 if union == 0 else intersection / union
    pixel_agreement = float((torch_mask == onnx_mask).mean())
    # Channel 1 is the exported boundary-logit head. Comparing a boundary
    # derived from channel 0 would let a broken/miswired boundary head pass.
    torch_boundary = (
        1.0 / (1.0 + np.exp(-torch_np[:, 1:2])) > mask_threshold
    )
    onnx_boundary = (
        1.0 / (1.0 + np.exp(-onnx_np[:, 1:2])) > mask_threshold
    )
    boundary_intersection = int((torch_boundary & onnx_boundary).sum())
    boundary_union = int((torch_boundary | onnx_boundary).sum())
    boundary_agreement = float((torch_boundary == onnx_boundary).mean())
    boundary_iou = (
        1.0
        if boundary_union == 0
        else boundary_intersection / boundary_union
    )
    return {
        "binary_mask_iou": float(binary_iou),
        "binary_mask_pixel_agreement": float(pixel_agreement),
        "boundary_mask_agreement": float(boundary_agreement),
        "boundary_mask_iou": float(boundary_iou),
        "passed": bool(
            binary_iou >= min_binary_agreement
            and pixel_agreement >= min_binary_agreement
            and boundary_agreement >= min_boundary_agreement
        ),
    }


def task_specific_parity(
    task: str,
    torch_flat,
    onnx_flat,
) -> dict:
    """Dispatch to the task-specific parity evaluator."""
    if task == "discovery":
        return discovery_parity(torch_flat, onnx_flat)
    if task == "classifier":
        return classifier_parity(torch_flat, onnx_flat)
    if task in ("leaf", "puddle"):
        return segmenter_parity(torch_flat, onnx_flat)
    raise ValueError(f"unsupported task-specific parity task {task!r}")


def assert_onnx_contract(
    onnx_path,
    *,
    expected_input_shape=None,
    expected_opset: int = 17,
) -> dict:
    """Fail-closed ONNX contract check: opset, fixed shapes, zero custom ops."""
    import onnx

    onnx_model = onnx.load(str(onnx_path))
    onnx.checker.check_model(onnx_model)
    opset = max(
        (int(entry.version) for entry in onnx_model.opset_import),
        default=-1,
    )
    if opset != expected_opset:
        raise ValueError(
            f"ONNX opset must be {expected_opset}, got {opset}"
        )
    if expected_input_shape is not None:
        graph_input = onnx_model.graph.input[0]
        dims = []
        for dim in graph_input.type.tensor_type.shape.dim:
            if dim.dim_param:
                raise ValueError(
                    "exported ONNX must use fixed shapes (dynamic_axes=None)"
                )
            dims.append(int(dim.dim_value))
        if tuple(dims) != tuple(int(value) for value in expected_input_shape):
            raise ValueError(
                f"ONNX input shape {tuple(dims)} does not match "
                f"expected {tuple(expected_input_shape)}"
            )
    inventory: dict[str, int] = {}
    for node in onnx_model.graph.node:
        inventory[node.op_type] = inventory.get(node.op_type, 0) + 1
        if node.op_type.startswith("custom") or node.domain not in ("", "ai.onnx"):
            raise ValueError(
                f"custom/unsupported ONNX operator: {node.op_type}"
            )
    return {
        "opset": opset,
        "fixed_input": True,
        "operator_inventory": inventory,
        "custom_ops": 0,
        "passed": True,
    }


__all__ = [
    "assert_onnx_contract",
    "classifier_parity",
    "discovery_parity",
    "segmenter_parity",
    "task_specific_parity",
]
