#!/usr/bin/env python3
"""Audit a pinned ONNX model without treating static checks as J6 proof."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = ROOT / "config" / "pretrained_model_sources.yaml"
DEFAULT_ARTIFACT_ROOT = ROOT / ".workspace" / "models"
STANDARD_DOMAINS = {"", "ai.onnx", "ai.onnx.ml"}


class AuditBlocked(RuntimeError):
    """An integrity, format, or dependency boundary blocked the audit."""


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_onnx():
    try:
        import onnx
    except ImportError as error:
        raise AuditBlocked(
            "ONNX_AUDIT_BLOCKED: Python package 'onnx' is not installed; "
            "install the project-approved onnx package before claiming checker, "
            "shape, opset, IR, or operator results"
        ) from error
    return onnx


def _dimension(dim: Any) -> int | str:
    if dim.HasField("dim_value") and int(dim.dim_value) > 0:
        return int(dim.dim_value)
    if dim.HasField("dim_param") and dim.dim_param:
        return str(dim.dim_param)
    return "dynamic"


def _value_info(value: Any, onnx: Any) -> dict[str, Any]:
    tensor = value.type.tensor_type
    return {
        "name": value.name,
        "element_type": onnx.TensorProto.DataType.Name(tensor.elem_type),
        "shape": [_dimension(dim) for dim in tensor.shape.dim],
    }


def audit_model(model_path: Path, expected_sha256: str | None = None) -> tuple[dict, dict]:
    model_path = model_path.resolve()
    if not model_path.is_file():
        raise AuditBlocked(f"model does not exist: {model_path}")
    actual_sha = file_sha256(model_path)
    if expected_sha256 and actual_sha != expected_sha256.lower():
        raise AuditBlocked(
            f"model SHA-256 {actual_sha} does not match expected {expected_sha256.lower()}"
        )
    if model_path.suffix.lower() != ".onnx":
        raise AuditBlocked("source artifact is not an ONNX file")
    onnx = load_onnx()
    try:
        model = onnx.load(str(model_path), load_external_data=False)
        onnx.checker.check_model(model)
        inferred = onnx.shape_inference.infer_shapes(model)
    except Exception as error:
        raise AuditBlocked(f"ONNX checker or shape inference failed: {error}") from error

    initializer_names = {item.name for item in inferred.graph.initializer}
    inputs = [
        _value_info(item, onnx)
        for item in inferred.graph.input
        if item.name not in initializer_names
    ]
    outputs = [_value_info(item, onnx) for item in inferred.graph.output]
    dynamic = any(
        not isinstance(dimension, int)
        for tensor in inputs + outputs
        for dimension in tensor["shape"]
    )
    operators = Counter(
        f"{node.domain or 'ai.onnx'}::{node.op_type}"
        for node in inferred.graph.node
    )
    custom = sorted(
        key
        for key in operators
        if key.split("::", 1)[0] not in STANDARD_DOMAINS
    )
    embedded_nms = sorted(
        key for key in operators if key.endswith("::NonMaxSuppression")
    )
    opsets = {
        item.domain or "ai.onnx": int(item.version)
        for item in inferred.opset_import
    }
    metadata = {item.key: item.value for item in inferred.metadata_props}
    inventory = {
        "schema_version": 1,
        "model": str(model_path),
        "sha256": actual_sha,
        "size_bytes": model_path.stat().st_size,
        "onnx_checker_pass": True,
        "shape_inference_pass": True,
        "inputs": inputs,
        "outputs": outputs,
        "opsets": opsets,
        "ir_version": int(inferred.ir_version),
        "dynamic_shape": dynamic,
        "custom_operators": custom,
        "custom_operator_count": len(custom),
        "embedded_nms": bool(embedded_nms),
        "embedded_nms_operators": embedded_nms,
        "metadata": metadata,
        "j6_compile_claim_allowed": False,
        "truth_boundary": (
            "ONNX checker and static graph inspection do not replace official "
            "OpenExplorer hb_compile, PTQ parity, x86 runtime, or board evidence."
        ),
    }
    operator_audit = {
        "schema_version": 1,
        "model": str(model_path),
        "sha256": actual_sha,
        "operator_inventory": dict(sorted(operators.items())),
        "custom_operators": custom,
        "custom_operator_count": len(custom),
        "embedded_nms": bool(embedded_nms),
        "authoritative_j6_compile_required": True,
    }
    return inventory, operator_audit


def resolve_registered_model(
    registry_path: Path, model_id: str, artifact_root: Path
) -> tuple[Path, str]:
    registry = yaml.safe_load(registry_path.read_text(encoding="utf-8"))
    try:
        source = registry["models"][model_id]["source"]
    except (KeyError, TypeError) as error:
        raise AuditBlocked(f"unknown or malformed registered model: {model_id}") from error
    if source.get("file_present_at_revision") is not True:
        raise AuditBlocked(
            f"{model_id}: pinned revision has no source ONNX; audit cannot proceed"
        )
    expected = source.get("expected_sha256")
    if not expected:
        raise AuditBlocked(f"{model_id}: registry has no expected SHA-256")
    return artifact_root / model_id / str(source["filename"]), str(expected)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--model", type=Path)
    target.add_argument("--model-id")
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY)
    parser.add_argument("--artifact-root", type=Path, default=DEFAULT_ARTIFACT_ROOT)
    parser.add_argument("--expected-sha256")
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if args.model_id:
            model_path, expected = resolve_registered_model(
                args.registry.resolve(), args.model_id, args.artifact_root.resolve()
            )
            if args.expected_sha256 and args.expected_sha256.lower() != expected:
                raise AuditBlocked(
                    "--expected-sha256 differs from the pinned registry"
                )
        else:
            model_path = args.model.resolve()
            expected = args.expected_sha256
        inventory, operators = audit_model(model_path, expected)
    except AuditBlocked as error:
        print(f"BLOCKED: {error}", file=sys.stderr)
        return 3 if "ONNX_AUDIT_BLOCKED" in str(error) else 2
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    (output / "PRETRAINED_MODEL_INVENTORY.json").write_text(
        json.dumps(inventory, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (output / "PRETRAINED_MODEL_OPERATOR_AUDIT.json").write_text(
        json.dumps(operators, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(inventory, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
