from auto14_onnx_preflight import contract_gate
from j6_operator_audit import audit_inventory


def test_j6_contract_requires_supported_onnx_and_1000_calibration_frames():
    passed = contract_gate(
        opset=17, ir_version=9, fixed_batch_one=True,
        custom_operator_count=0, calibration_file_count=1000,
    )
    assert all(passed.values())
    assert not contract_gate(
        opset=20, ir_version=10, fixed_batch_one=True,
        custom_operator_count=0, calibration_file_count=999,
    )["calibration_at_least_1000"]


def test_j6_operator_audit_keeps_nms_and_topk_outside_graph():
    clean = audit_inventory(
        {"ai.onnx::Conv": 5, "ai.onnx::Relu": 4, "ai.onnx::Resize": 2}
    )
    assert clean["static_operator_profile_pass"] is True
    blocked = audit_inventory(
        {"ai.onnx::Conv": 5, "ai.onnx::TopK": 1, "custom::Plugin": 1}
    )
    assert blocked["static_operator_profile_pass"] is False
    assert blocked["graph_external_postprocess_violations"] == ["TopK"]
    assert blocked["custom_operators"] == ["custom::Plugin"]
