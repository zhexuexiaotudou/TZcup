#!/usr/bin/env python3
"""Conservative J6E/M BPU graph audit for deployable perception ONNX models."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


# Intersection of the operations used by the planned perception graphs and
# documented as BPU-supported for J6E/M. This is intentionally narrower than
# Horizon's full list; hb_compile remains the authoritative constraint check.
J6EM_PRODUCT_BPU_PROFILE = frozenset(
    {
        "Add", "AveragePool", "BatchNormalization", "Cast", "Clip", "Concat",
        "Constant", "ConstantOfShape", "Conv", "ConvTranspose", "Div", "Elu",
        "Equal", "Exp", "Expand", "Flatten", "Gather", "Gemm",
        "GlobalAveragePool", "GlobalMaxPool", "Greater", "GreaterOrEqual",
        "HardSigmoid", "HardSwish", "Identity", "InstanceNormalization",
        "LayerNormalization", "LeakyRelu", "Less", "LessOrEqual", "Log",
        "MatMul", "Max", "MaxPool", "Min", "Mish", "Mul", "Neg", "Pad",
        "PRelu", "Pow", "Reciprocal", "ReduceL1", "ReduceL2", "ReduceMax",
        "ReduceMean", "ReduceMin", "ReduceSum", "Relu", "Reshape", "Resize",
        "Round", "Shape", "Sigmoid", "Sign", "Softmax", "Softplus",
        "SpaceToDepth", "Split", "Sqrt", "Squeeze", "Sub", "Sum", "Tanh",
        "Tile", "Transpose", "Unsqueeze", "Where",
    }
)
EXTERNAL_POSTPROCESS_REQUIRED = frozenset(
    {"NonMaxSuppression", "NonZero", "RoiAlign", "TopK"}
)


def audit_inventory(inventory: dict[str, int]) -> dict:
    names = {
        key.split("::", 1)[-1]: int(count)
        for key, count in inventory.items()
        if int(count) > 0
    }
    custom = sorted(key for key in inventory if not key.startswith("ai.onnx::"))
    postprocess = sorted(set(names) & EXTERNAL_POSTPROCESS_REQUIRED)
    unsupported = sorted(
        name for name in names if name not in J6EM_PRODUCT_BPU_PROFILE
    )
    return {
        "schema_version": 1,
        "target": "J6E/M",
        "operator_inventory": dict(sorted(names.items())),
        "custom_operators": custom,
        "graph_external_postprocess_violations": postprocess,
        "outside_conservative_bpu_profile": unsupported,
        "static_operator_profile_pass": not custom and not postprocess and not unsupported,
        "authoritative_compile_required": True,
        "truth_boundary": (
            "Static profile pass does not replace official hb_compile checking, PTQ, "
            "quantized accuracy parity, or physical-board evidence."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--inventory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = json.loads(args.inventory.read_text(encoding="utf-8"))
    inventory = payload.get("operator_inventory", payload)
    report = audit_inventory(inventory)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["static_operator_profile_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
