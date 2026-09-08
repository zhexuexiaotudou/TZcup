from __future__ import annotations

import copy

import pytest

import dosod_hbm_abi_contract as abi


def test_compile_filter_keeps_compiler_terminal_transposes() -> None:
    assert abi.REMOVE_NODE_TYPE == "Dequantize;Quantize;Cast;Reshape"
    with pytest.raises(ValueError, match="remove_node_type"):
        abi.validate_remove_node_type("Dequantize;Quantize;Transpose;Cast;Reshape")


def test_pre_onnx_outputs_are_ordered_prediction_major_float() -> None:
    abi.validate_pre_onnx_outputs(copy.deepcopy(abi.PRE_ONNX_OUTPUTS))
    wrong = copy.deepcopy(abi.PRE_ONNX_OUTPUTS)
    wrong.reverse()
    with pytest.raises(ValueError, match="onnx_output_abi_mismatch"):
        abi.validate_pre_onnx_outputs(wrong)


def test_post_hbm_info_is_ordered_int16_prediction_major() -> None:
    inputs = [
        {"index": index, "name": name, "shape": shape, "dtype": dtype, "aligned_byte_size": -1}
        for index, name, shape, dtype in abi.POST_HBM_INPUTS
    ]
    abi.validate_post_hbm_inputs(inputs)
    abi.validate_post_hbm_outputs(copy.deepcopy(abi.POST_HBM_OUTPUTS))
    wrong = copy.deepcopy(abi.POST_HBM_OUTPUTS)
    wrong["boxes"]["shape"] = [1, 4, 8400]
    with pytest.raises(ValueError, match="hbm_output_abi_mismatch"):
        abi.validate_post_hbm_outputs(wrong)


def _disas_fixture() -> dict:
    variables = []
    for identifier, (_, name, dims, _) in zip((11, 12), abi.POST_HBM_INPUTS):
        variables.append({"id": identifier, "name": name, "type": {"tensorType": {"dims": dims, "elemType": {"typeTag": "TYPE_TAG_UI8"}}}})
    for identifier, (name, binding) in zip((13, 14), abi.POST_HBM_OUTPUTS.items()):
        variables.append({"id": identifier, "name": name, "type": {"tensorType": {"dims": binding["shape"], "elemType": {"typeTag": "TYPE_TAG_SI16"}, "strides": abi.POST_HBM_OUTPUT_STRIDES_BYTES}}})
    return {"graphs": [{"inputVarIds": [11, 12], "inputVarNames": ["images_y", "images_uv"], "outputVarIds": [13, 14], "outputVarNames": ["scores", "boxes"], "variables": variables}]}


def test_disas_rejects_shape_compatible_but_stride_incompatible_boxes() -> None:
    abi.validate_hbrt4_disas(_disas_fixture())
    wrong = _disas_fixture()
    wrong["graphs"][0]["variables"][-1]["type"]["tensorType"]["strides"] = [67584, 16896, 2]
    with pytest.raises(ValueError, match="output_abi_mismatch:boxes"):
        abi.validate_hbrt4_disas(wrong)
