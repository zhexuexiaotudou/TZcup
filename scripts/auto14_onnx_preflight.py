#!/usr/bin/env python3
"""Create a J6/Nash compile preflight and official hb_compile configuration."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import yaml


MIN_CALIBRATION_FRAMES = 1000
SUPPORTED_ONNX_OPSET_RANGE = (10, 19)
MAX_ONNX_IR_VERSION = 9


def contract_gate(
    *, opset: int, ir_version: int, fixed_batch_one: bool,
    custom_operator_count: int, calibration_file_count: int,
) -> dict:
    return {
        "opset_supported": SUPPORTED_ONNX_OPSET_RANGE[0]
        <= int(opset)
        <= SUPPORTED_ONNX_OPSET_RANGE[1],
        "ir_version_supported": int(ir_version) <= MAX_ONNX_IR_VERSION,
        "fixed_batch_one": bool(fixed_batch_one),
        "custom_operator_count_zero": int(custom_operator_count) == 0,
        "calibration_at_least_1000": int(calibration_file_count)
        >= MIN_CALIBRATION_FRAMES,
    }


def tensor_shape(value_info) -> list[int | str]:
    dimensions = value_info.type.tensor_type.shape.dim
    return [
        int(dim.dim_value)
        if dim.dim_value > 0
        else dim.dim_param
        if dim.dim_param
        else "dynamic"
        for dim in dimensions
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--calibration-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--model-name", required=True)
    parser.add_argument("--march", default="nash-e")
    args = parser.parse_args()
    import onnx

    model_path = Path(args.model).resolve()
    calibration = Path(args.calibration_dir).resolve()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    model = onnx.load(str(model_path))
    onnx.checker.check_model(model)
    inferred = onnx.shape_inference.infer_shapes(model)
    initializers = {item.name for item in inferred.graph.initializer}
    inputs = [
        {"name": item.name, "shape": tensor_shape(item)}
        for item in inferred.graph.input
        if item.name not in initializers
    ]
    outputs = [
        {"name": item.name, "shape": tensor_shape(item)}
        for item in inferred.graph.output
    ]
    operators = sorted(
        {
            f"{node.domain or 'ai.onnx'}::{node.op_type}"
            for node in inferred.graph.node
        }
    )
    custom_ops = [
        item
        for item in operators
        if not item.startswith("ai.onnx::")
    ]
    calibration_files = sorted(
        path
        for path in calibration.rglob("*")
        if path.is_file()
        and path.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".npy"}
    )
    static_batch_one = (
        len(inputs) == 1
        and inputs[0]["shape"]
        and inputs[0]["shape"][0] == 1
        and all(isinstance(value, int) and value > 0 for value in inputs[0]["shape"])
    )
    opset = max(item.version for item in inferred.opset_import)
    gates = contract_gate(
        opset=opset,
        ir_version=inferred.ir_version,
        fixed_batch_one=static_batch_one,
        custom_operator_count=len(custom_ops),
        calibration_file_count=len(calibration_files),
    )
    config = {
        "model_parameters": {
            "onnx_model": str(model_path),
            "march": args.march,
            "working_dir": str((output / "compiled").resolve()),
            "output_model_file_prefix": args.model_name,
        },
        "input_parameters": {
            "input_name": inputs[0]["name"] if len(inputs) == 1 else "",
            "input_type_rt": "featuremap",
            "input_type_train": "featuremap",
            "input_layout_train": "NCHW",
            "input_shape": "x".join(map(str, inputs[0]["shape"]))
            if len(inputs) == 1
            else "",
            "input_batch": 1,
            "norm_type": "no_preprocess",
        },
        "calibration_parameters": {
            "cal_data_dir": str(calibration),
            "calibration_type": "default",
        },
        "compiler_parameters": {"optimize_level": "O2"},
    }
    config_path = output / f"{args.model_name}_config.yaml"
    config_path.write_text(
        yaml.safe_dump(config, sort_keys=False), encoding="utf-8"
    )
    report = {
        "schema_version": 1,
        "model_name": args.model_name,
        "model_path": str(model_path),
        "model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
        "onnx_checker_pass": True,
        "shape_inference_pass": True,
        "inputs": inputs,
        "outputs": outputs,
        "operator_inventory": operators,
        "custom_operators": custom_ops,
        "custom_operator_count": len(custom_ops),
        "fixed_batch_one": static_batch_one,
        "opset": opset,
        "ir_version": inferred.ir_version,
        "calibration_file_count": len(calibration_files),
        "calibration_at_least_1000": len(calibration_files)
        >= MIN_CALIBRATION_FRAMES,
        "gates": gates,
        "compile_config": config_path.name,
        "preflight_pass": (
            all(gates.values())
        ),
    }
    (output / f"{args.model_name}_preflight.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    return 0 if report["preflight_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
