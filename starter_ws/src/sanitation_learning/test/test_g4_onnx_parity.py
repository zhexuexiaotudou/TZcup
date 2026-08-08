from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


from sanitation_learning.g4_onnx_parity import (  # noqa: E402
    assert_onnx_contract,
    classifier_parity,
    discovery_parity,
    segmenter_parity,
    task_specific_parity,
)


def test_classifier_parity_top1_and_probability_error() -> None:
    rng = np.random.default_rng(7)
    logits = rng.normal(size=(4, 4))
    tiny_error = np.array(
        [[1e-6, 0.0, 0.0, 0.0]] * 4, dtype=np.float32
    )
    result = classifier_parity(logits, logits + tiny_error)
    assert result["top1_agreement"] == 1.0
    assert result["max_probability_error"] <= 1e-4
    assert result["passed"] is True
    assert result["argmax_agreement"] == 1.0


def test_classifier_parity_detects_top1_drift() -> None:
    logits = np.array(
        [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=np.float32
    )
    drifted = np.array(
        [[0.0, 1.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=np.float32
    )
    result = classifier_parity(logits, drifted)
    assert result["top1_agreement"] == 0.5
    assert result["passed"] is False


def test_classifier_parity_rejects_large_probability_error_with_same_top1() -> None:
    torch_logits = np.array([[10.0, 0.0]], dtype=np.float32)
    onnx_logits = np.array([[1.0, 0.0]], dtype=np.float32)
    result = classifier_parity(
        torch_logits,
        onnx_logits,
        max_probability_error=1e-6,
    )
    assert result["top1_agreement"] == 1.0
    assert result["max_probability_error"] > 0.2
    assert result["passed"] is False


def test_segmenter_parity_masks_and_boundaries() -> None:
    rng = np.random.default_rng(11)
    logits = rng.normal(size=(1, 2, 32, 32)).astype(np.float32)
    identical = logits.copy()
    result = segmenter_parity(logits, identical)
    assert result["binary_mask_iou"] == 1.0
    assert result["binary_mask_pixel_agreement"] == 1.0
    assert result["boundary_mask_agreement"] == 1.0
    assert result["passed"] is True


def test_segmenter_parity_detects_mask_drift() -> None:
    logits = np.zeros((1, 2, 16, 16), dtype=np.float32)
    drifted = logits.copy()
    drifted[0, 0, :, :] = 10.0
    result = segmenter_parity(logits, drifted)
    assert result["binary_mask_pixel_agreement"] < 1.0
    assert result["passed"] is False


def test_segmenter_parity_compares_exported_boundary_channel() -> None:
    torch_flat = np.zeros((1, 2, 8, 8), dtype=np.float32)
    onnx_flat = torch_flat.copy()
    torch_flat[:, 1] = 10.0
    onnx_flat[:, 1] = -10.0
    result = segmenter_parity(torch_flat, onnx_flat)
    assert result["binary_mask_pixel_agreement"] == 1.0
    assert result["boundary_mask_agreement"] == 0.0
    assert result["passed"] is False


def test_empty_discovery_and_segmenter_outputs_agree() -> None:
    discovery = np.full((1, 5, 4, 4), -20.0, dtype=np.float32)
    discovery[:, 1:] = 0.0
    discovery_result = discovery_parity(discovery, discovery.copy())
    assert discovery_result["decoded_candidate_agreement"] == 1.0
    assert discovery_result["decoded_agreement"] is True

    segmenter = np.full((1, 2, 4, 4), -20.0, dtype=np.float32)
    segmenter_result = segmenter_parity(segmenter, segmenter.copy())
    assert segmenter_result["binary_mask_iou"] == 1.0
    assert segmenter_result["boundary_mask_iou"] == 1.0
    assert segmenter_result["passed"] is True


def test_segmenter_parity_rejects_shape_mismatch() -> None:
    with pytest.raises(ValueError, match="identical shapes"):
        segmenter_parity(
            np.zeros((1, 2, 16, 16), dtype=np.float32),
            np.zeros((1, 2, 8, 8), dtype=np.float32),
        )
    with pytest.raises(ValueError, match="Nx2xHxW"):
        segmenter_parity(
            np.zeros((1, 1, 16, 16), dtype=np.float32),
            np.zeros((1, 1, 16, 16), dtype=np.float32),
        )


def test_discovery_parity_agreement_on_identical_outputs() -> None:
    flat = np.zeros((1, 5, 32, 32), dtype=np.float32)
    # A single strong positive peak decodes into one candidate.
    flat[0, 0, 16, 16] = 3.0
    flat[0, 3, 16, 16] = 4.0
    flat[0, 4, 16, 16] = 4.0
    result = discovery_parity(flat, flat.copy())
    assert result["torch_candidate_count"] == 1
    assert result["decoded_candidate_count_agreement"] is True
    assert result["decoded_candidate_agreement"] == 1.0
    assert result["decoded_agreement"] is True


def test_discovery_parity_detects_candidate_drift() -> None:
    flat = np.zeros((1, 5, 32, 32), dtype=np.float32)
    flat[0, 0, 16, 16] = 3.0
    flat[0, 3, 16, 16] = 4.0
    flat[0, 4, 16, 16] = 4.0
    drifted = flat.copy()
    drifted[0, 0, 16, 16] = -3.0
    result = discovery_parity(flat, drifted)
    assert result["decoded_candidate_count_agreement"] is False
    assert result["decoded_agreement"] is False


def test_task_specific_dispatch() -> None:
    assert task_specific_parity(
        "classifier",
        np.zeros((2, 4), dtype=np.float32),
        np.zeros((2, 4), dtype=np.float32),
    )["passed"] is True
    with pytest.raises(ValueError, match="unsupported task-specific parity"):
        task_specific_parity(
            "tracker",
            np.zeros((1, 2), dtype=np.float32),
            np.zeros((1, 2), dtype=np.float32),
        )


def test_onnx_contract_fails_on_wrong_opset(tmp_path) -> None:
    onnx = pytest.importorskip("onnx")
    node = onnx.helper.make_node("Relu", ["x"], ["y"])
    graph = onnx.helper.make_graph(
        [node], "g", [onnx.helper.make_tensor_value_info("x", onnx.TensorProto.FLOAT, [1, 3])],
        [onnx.helper.make_tensor_value_info("y", onnx.TensorProto.FLOAT, [1, 3])],
    )
    model = onnx.helper.make_model(
        graph,
        opset_imports=[onnx.helper.make_opsetid("", 13)],
    )
    path = tmp_path / "wrong.onnx"
    onnx.save(model, str(path))
    with pytest.raises(ValueError, match="opset must be 17"):
        assert_onnx_contract(path, expected_opset=17)


def test_onnx_contract_rejects_dynamic_shape(tmp_path) -> None:
    onnx = pytest.importorskip("onnx")
    dynamic = onnx.helper.make_tensor_value_info(
        "x", onnx.TensorProto.FLOAT, ["batch", 3]
    )
    node = onnx.helper.make_node("Relu", ["x"], ["y"])
    graph = onnx.helper.make_graph(
        [node],
        "g",
        [dynamic],
        [
            onnx.helper.make_tensor_value_info(
                "y", onnx.TensorProto.FLOAT, ["batch", 3]
            )
        ],
    )
    model = onnx.helper.make_model(
        graph,
        opset_imports=[onnx.helper.make_opsetid("", 17)],
    )
    path = tmp_path / "dynamic.onnx"
    onnx.save(model, str(path))
    with pytest.raises(ValueError, match="fixed shapes"):
        assert_onnx_contract(
            path, expected_input_shape=(1, 3), expected_opset=17
        )
