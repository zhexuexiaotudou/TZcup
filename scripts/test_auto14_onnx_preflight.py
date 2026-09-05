from __future__ import annotations

import numpy as np

from auto14_onnx_preflight import calibration_inventory, validate_onnx_contract


def test_calibration_inventory_rejects_text_embedding_shape(tmp_path) -> None:
    np.save(tmp_path / "frame.npy", np.zeros((1, 3, 4, 6), dtype=np.float32))
    np.save(tmp_path / "text_embedding.npy", np.zeros((4, 512), dtype=np.float32))
    report = calibration_inventory(tmp_path, (1, 3, 4, 6))
    assert report["candidate_count"] == 2
    assert report["valid_count"] == 1
    assert report["records"][0]["path"] == "frame.npy"
    assert report["invalid"] == [
        {
            "path": "text_embedding.npy",
            "reason": "shape_mismatch",
            "actual_shape": [4, 512],
            "expected_shape": [1, 3, 4, 6],
        }
    ]


def test_calibration_inventory_requires_float32(tmp_path) -> None:
    np.save(tmp_path / "frame.npy", np.zeros((1, 3, 4, 6), dtype=np.uint8))
    report = calibration_inventory(tmp_path, (1, 3, 4, 6))
    assert report["valid_count"] == 0
    assert report["invalid"][0]["reason"] == "dtype_mismatch"


def _model_contract() -> dict:
    return {
        "ir_version": 6,
        "opset": 11,
        "node_count": 327,
        "inputs": [
            {"name": "images", "dtype": "FLOAT", "shape": [1, 3, 640, 640]}
        ],
        "outputs": [
            {"name": "scores", "dtype": "FLOAT", "shape": [1, 8400, 4]},
            {"name": "boxes", "dtype": "FLOAT", "shape": [1, 8400, 4]},
        ],
        "forbidden_operator_types": ["NonMaxSuppression"],
        "custom_operator_domains_allowed": [],
    }


def test_exact_four_class_onnx_contract_passes() -> None:
    contract = _model_contract()
    blockers = validate_onnx_contract(
        contract["inputs"],
        contract["outputs"],
        contract,
        ir_version=6,
        opsets={"ai.onnx": 11},
        node_count=327,
        operator_types={"Conv", "Sigmoid"},
        custom_domains=set(),
    )
    assert blockers == []


def test_coco80_or_nms_graph_cannot_pass_four_class_contract() -> None:
    contract = _model_contract()
    outputs = [dict(row) for row in contract["outputs"]]
    outputs[0] = {"name": "scores", "dtype": "FLOAT", "shape": [1, 8400, 80]}
    blockers = validate_onnx_contract(
        contract["inputs"],
        outputs,
        contract,
        ir_version=6,
        opsets={"ai.onnx": 11},
        node_count=328,
        operator_types={"Conv", "NonMaxSuppression"},
        custom_domains={"vendor.example"},
    )
    assert blockers == [
        "onnx_output_signature_mismatch",
        "onnx_node_count_mismatch",
        "onnx_forbidden_operator_present",
        "onnx_custom_operator_domain_forbidden",
    ]
