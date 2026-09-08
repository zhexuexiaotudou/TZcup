"""Exact DOSOD tensor ABI shared by compile and HBM-info validation paths.

This contract deliberately distinguishes the ONNX graph ABI from the compiled
HBM ABI.  In particular, the compiler-generated terminal transpose restoring
``boxes`` to prediction-major order must remain in the HBM graph.
"""

from __future__ import annotations

from typing import Any


REMOVE_NODE_TYPE = "Dequantize;Quantize;Cast;Reshape"
PRE_ONNX_OUTPUTS = [
    {"name": "scores", "dtype": "FLOAT", "shape": [1, 8400, 4]},
    {"name": "boxes", "dtype": "FLOAT", "shape": [1, 8400, 4]},
]
POST_HBM_INPUTS = [
    (0, "images_y", [1, 640, 640, 1], "HB_DNN_TENSOR_TYPE_U8"),
    (1, "images_uv", [1, 320, 320, 2], "HB_DNN_TENSOR_TYPE_U8"),
]
POST_HBM_OUTPUTS = {
    "scores": {"index": 0, "name": "scores", "shape": [1, 8400, 4], "dtype": "HB_DNN_TENSOR_TYPE_S16"},
    "boxes": {"index": 1, "name": "boxes", "shape": [1, 8400, 4], "dtype": "HB_DNN_TENSOR_TYPE_S16"},
}
POST_HBM_OUTPUT_STRIDES_BYTES = [67200, 8, 2]


def validate_remove_node_type(value: Any) -> None:
    if value != REMOVE_NODE_TYPE:
        raise ValueError("compile_config_remove_node_type_mismatch")


def validate_pre_onnx_outputs(value: Any) -> None:
    if value != PRE_ONNX_OUTPUTS:
        raise ValueError("onnx_output_abi_mismatch")


def validate_post_hbm_inputs(value: Any) -> None:
    observed = []
    if isinstance(value, list):
        for item in value:
            if isinstance(item, dict):
                observed.append((item.get("index"), item.get("name"), item.get("shape"), item.get("dtype")))
    if observed != POST_HBM_INPUTS:
        raise ValueError("hbm_input_abi_mismatch")


def validate_post_hbm_outputs(value: Any) -> None:
    if not isinstance(value, dict) or list(value) != list(POST_HBM_OUTPUTS) or value != POST_HBM_OUTPUTS:
        raise ValueError("hbm_output_abi_mismatch")


def validate_hbrt4_disas(value: Any) -> None:
    """Reject a compiled graph unless its physical tensors preserve the ABI.

    ``hbrt4-disas --json`` records byte strides, so shape-only validation cannot
    mistake a channel-major ``[1,4,8400]`` box tensor for prediction-major
    ``[1,8400,4]``.  The parser intentionally accepts exactly one graph and
    pairs the explicit graph output IDs and names before reading variables.
    """
    if not isinstance(value, dict) or not isinstance(value.get("graphs"), list) or len(value["graphs"]) != 1:
        raise ValueError("hbrt4_disas_graphs_invalid")
    graph = value["graphs"][0]
    if not isinstance(graph, dict):
        raise ValueError("hbrt4_disas_graph_invalid")
    input_ids, input_names = graph.get("inputVarIds"), graph.get("inputVarNames")
    output_ids, output_names = graph.get("outputVarIds"), graph.get("outputVarNames")
    if input_names != [item[1] for item in POST_HBM_INPUTS] or output_names != list(POST_HBM_OUTPUTS):
        raise ValueError("hbrt4_disas_tensor_order_mismatch")
    if not all(isinstance(items, list) for items in (input_ids, output_ids)) or len(input_ids) != 2 or len(output_ids) != 2 or any(not isinstance(item, int) or isinstance(item, bool) for item in input_ids + output_ids):
        raise ValueError("hbrt4_disas_tensor_ids_invalid")
    variables = graph.get("variables")
    if not isinstance(variables, list):
        raise ValueError("hbrt4_disas_variables_invalid")
    by_id = {item.get("id"): item for item in variables if isinstance(item, dict) and isinstance(item.get("id"), int)}
    if len(by_id) != len(variables) or any(identifier not in by_id for identifier in input_ids + output_ids):
        raise ValueError("hbrt4_disas_variable_id_mismatch")

    def tensor(identifier: int, name: str) -> dict[str, Any]:
        variable = by_id[identifier]
        tensor_type = variable.get("type", {}).get("tensorType") if isinstance(variable.get("type"), dict) else None
        if variable.get("name") != name or not isinstance(tensor_type, dict):
            raise ValueError("hbrt4_disas_variable_tensor_mismatch")
        return tensor_type

    for identifier, (_, name, dims, type_tag) in zip(input_ids, POST_HBM_INPUTS):
        item = tensor(identifier, name)
        element = item.get("elemType")
        if item.get("dims") != dims or not isinstance(element, dict) or element.get("typeTag") != "TYPE_TAG_UI8":
            raise ValueError(f"hbrt4_disas_input_abi_mismatch:{name}")
    for identifier, (name, expected) in zip(output_ids, POST_HBM_OUTPUTS.items()):
        item = tensor(identifier, name)
        element = item.get("elemType")
        if (item.get("dims") != expected["shape"]
                or not isinstance(element, dict) or element.get("typeTag") != "TYPE_TAG_SI16"
                or item.get("strides") != POST_HBM_OUTPUT_STRIDES_BYTES):
            raise ValueError(f"hbrt4_disas_output_abi_mismatch:{name}")
