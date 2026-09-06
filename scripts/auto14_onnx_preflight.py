#!/usr/bin/env python3
"""Create a J6/Nash compile preflight and official hb_compile configuration."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import yaml

from validate_dosod_s100p_hbm_compile_contract import (
    audit_compile_inputs,
    load_json,
)


UPSTREAM_DOSOD_REVISION = "c50129b5badf6ed7bb85e692ab493d8bdb58da6a"
FORMAL_MARCH = "nash-m"
FORMAL_INPUT_TYPE_TRAIN = "rgb"
FORMAL_INPUT_TYPE_RUNTIME = "nv12"
FORMAL_SCALE_VALUE = 0.003921568627451
MINIMUM_CALIBRATION_SAMPLES = 500
FORMAL_MODEL_NAME = "dosod_mlp3x_s_tzcup_rep"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def calibration_inventory(calibration: Path, expected_shape: tuple[int, ...]) -> dict:
    """Accept only upstream-compatible float32 NCHW tensors.

    The official S100 DOSOD recipe writes one ``.npy`` tensor per source image.
    Exact shape/dtype checks prevent vocabulary embeddings or unrelated arrays
    from being miscounted as calibration samples.
    """

    import numpy as np

    valid: list[dict] = []
    invalid: list[dict] = []
    candidates = sorted(path for path in calibration.rglob("*.npy") if path.is_file())
    for path in candidates:
        relative = path.relative_to(calibration).as_posix()
        if path.is_symlink():
            invalid.append({"path": relative, "reason": "symbolic_link"})
            continue
        try:
            array = np.load(path, mmap_mode="r", allow_pickle=False)
        except Exception as exc:
            invalid.append({"path": relative, "reason": f"load_error:{type(exc).__name__}"})
            continue
        if tuple(array.shape) != expected_shape:
            invalid.append(
                {
                    "path": relative,
                    "reason": "shape_mismatch",
                    "actual_shape": list(array.shape),
                    "expected_shape": list(expected_shape),
                }
            )
            continue
        if array.dtype != np.dtype("float32"):
            invalid.append(
                {
                    "path": relative,
                    "reason": "dtype_mismatch",
                    "actual_dtype": str(array.dtype),
                }
            )
            continue
        valid.append(
            {
                "path": relative,
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
                "shape": list(expected_shape),
                "dtype": "float32",
            }
        )
    canonical = json.dumps(valid, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return {
        "candidate_count": len(candidates),
        "valid_count": len(valid),
        "invalid": invalid,
        "records": valid,
        "inventory_sha256": hashlib.sha256(canonical).hexdigest(),
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


def validate_onnx_contract(
    inputs: list[dict],
    outputs: list[dict],
    model_contract: dict,
    *,
    ir_version: int,
    opsets: dict[str, int],
    node_count: int,
    operator_types: set[str],
    custom_domains: set[str],
) -> list[str]:
    """Return exact graph-contract blockers without weakening to shape compatibility."""

    blockers: list[str] = []
    if inputs != model_contract.get("inputs"):
        blockers.append("onnx_input_signature_mismatch")
    if outputs != model_contract.get("outputs"):
        blockers.append("onnx_output_signature_mismatch")
    if ir_version != model_contract.get("ir_version"):
        blockers.append("onnx_ir_version_mismatch")
    if opsets != {"ai.onnx": model_contract.get("opset")}:
        blockers.append("onnx_opset_mismatch")
    if node_count != model_contract.get("node_count"):
        blockers.append("onnx_node_count_mismatch")
    forbidden = set(model_contract.get("forbidden_operator_types", []))
    if operator_types & forbidden:
        blockers.append("onnx_forbidden_operator_present")
    allowed_custom = set(model_contract.get("custom_operator_domains_allowed", []))
    if custom_domains - allowed_custom:
        blockers.append("onnx_custom_operator_domain_forbidden")
    return blockers


def _write_report(output: Path, model_name: str, report: dict) -> None:
    (output / f"{model_name}_preflight.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--calibration-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--model-name", required=True)
    parser.add_argument(
        "--contract",
        default="config/dosod_s100p_hbm_compile_contract.json",
    )
    parser.add_argument("--repository-root", default=".")
    parser.add_argument("--artifact-root", required=True)
    parser.add_argument("--upstream-root", required=True)
    parser.add_argument("--compiler-identity", required=True)
    parser.add_argument("--march", default=FORMAL_MARCH)
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--allow-blocked-exit-zero", action="store_true")
    args = parser.parse_args()

    model_path = Path(args.model).resolve()
    calibration = Path(args.calibration_dir).resolve()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    contract_path = Path(args.contract)
    contract = load_json(contract_path)
    contract_audit = audit_compile_inputs(
        contract_path,
        Path(args.repository_root),
        Path(args.artifact_root),
        Path(args.upstream_root),
        calibration,
        Path(args.compiler_identity),
    )
    expected_model_path = (Path(args.artifact_root).resolve() / contract["model"]["relative_path"]).resolve()
    entry_blockers: list[str] = []
    if model_path != expected_model_path:
        entry_blockers.append("model_path_not_contract_artifact")
    if args.model_name != FORMAL_MODEL_NAME:
        entry_blockers.append("model_name_not_frozen")
    jobs_range = contract["compile_recipe"].get("jobs_allowed_range", [])
    if (
        len(jobs_range) != 2
        or not all(isinstance(value, int) for value in jobs_range)
        or not jobs_range[0] <= args.jobs <= jobs_range[1]
    ):
        entry_blockers.append("compiler_jobs_outside_contract_range")
    if args.march != contract["compile_recipe"].get("march"):
        entry_blockers.append("compiler_march_not_frozen")
    if contract_audit["status"] != "READY_FOR_ONNX_TOOLCHAIN_PREFLIGHT" or entry_blockers:
        config_path = output / f"{args.model_name}_config.yaml"
        if config_path.exists():
            config_path.unlink()
        blockers = list(contract_audit.get("blockers", [])) + entry_blockers
        report = {
            "schema_version": 3,
            "report_id": "tzcup_dosod_s100p_compile_preflight_v3",
            "model_name": args.model_name,
            "model_path": str(model_path),
            "contract_audit_status": contract_audit.get("status"),
            "contract_sha256": contract_audit.get("contract_sha256"),
            "compile_plan_sha256": contract_audit.get("compile_plan_sha256"),
            "blockers": blockers,
            "compile_config": None,
            "compile_config_emitted": False,
            "compile_executed": False,
            "hbm_status": "HBM_NOT_PRODUCED",
            "hbm_runtime_validated": False,
            "preflight_pass": False,
        }
        _write_report(output, args.model_name, report)
        return 0 if args.allow_blocked_exit_zero else 2

    try:
        import onnx
    except Exception as exc:
        report = {
            "schema_version": 3,
            "report_id": "tzcup_dosod_s100p_compile_preflight_v3",
            "model_name": args.model_name,
            "model_path": str(model_path),
            "contract_audit_status": contract_audit.get("status"),
            "contract_sha256": contract_audit.get("contract_sha256"),
            "compile_plan_sha256": contract_audit.get("compile_plan_sha256"),
            "blockers": [f"onnx_module_unavailable:{type(exc).__name__}"],
            "compile_config": None,
            "compile_config_emitted": False,
            "compile_executed": False,
            "hbm_status": "HBM_NOT_PRODUCED",
            "hbm_runtime_validated": False,
            "preflight_pass": False,
        }
        _write_report(output, args.model_name, report)
        return 0 if args.allow_blocked_exit_zero else 2

    model = onnx.load(str(model_path))
    onnx.checker.check_model(model)
    inferred = onnx.shape_inference.infer_shapes(model)
    initializers = {item.name for item in inferred.graph.initializer}
    inputs = [
        {
            "name": item.name,
            "dtype": onnx.TensorProto.DataType.Name(item.type.tensor_type.elem_type),
            "shape": tensor_shape(item),
        }
        for item in inferred.graph.input
        if item.name not in initializers
    ]
    outputs = [
        {
            "name": item.name,
            "dtype": onnx.TensorProto.DataType.Name(item.type.tensor_type.elem_type),
            "shape": tensor_shape(item),
        }
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
    operator_types = {node.op_type for node in inferred.graph.node}
    custom_domains = {node.domain for node in inferred.graph.node if node.domain}
    opsets = {
        item.domain or "ai.onnx": int(item.version)
        for item in inferred.opset_import
    }
    graph_blockers = validate_onnx_contract(
        inputs,
        outputs,
        contract["model"],
        ir_version=int(inferred.ir_version),
        opsets=opsets,
        node_count=len(inferred.graph.node),
        operator_types=operator_types,
        custom_domains=custom_domains,
    )
    static_batch_one = (
        len(inputs) == 1
        and inputs[0]["shape"]
        and inputs[0]["shape"][0] == 1
        and all(isinstance(value, int) and value > 0 for value in inputs[0]["shape"])
    )
    input_shape = tuple(inputs[0]["shape"]) if static_batch_one else ()
    calibration_result = calibration_inventory(calibration, input_shape) if input_shape else {
        "candidate_count": 0,
        "valid_count": 0,
        "invalid": [{"reason": "onnx_input_not_static_batch_one"}],
        "records": [],
        "inventory_sha256": None,
    }
    formal_profile = args.march == FORMAL_MARCH and jobs_range[0] <= args.jobs <= jobs_range[1]
    recipe = contract["compile_recipe"]
    config = {
        "model_parameters": {
            "onnx_model": str(model_path),
            "march": args.march,
            "working_dir": str((output / "compiled").resolve()),
            "output_model_file_prefix": recipe["output_model_file_prefix"],
            "remove_node_type": "Dequantize;Quantize;Transpose;Cast;Reshape",
            "layer_out_dump": False,
        },
        "input_parameters": {
            "input_name": recipe["input_name"],
            "input_type_train": recipe["input_type_train"],
            "input_layout_train": recipe["input_layout_train"],
            "input_shape": recipe["input_shape"],
            "input_batch": recipe["input_batch"],
            "norm_type": recipe["norm_type"],
            "mean_value": "",
            "scale_value": recipe["scale_value"],
            "input_layout_rt": recipe["input_layout_rt"],
            "input_type_rt": recipe["input_type_rt"],
        },
        "calibration_parameters": {
            "cal_data_dir": str(calibration),
            "cal_data_type": recipe["cal_data_type"],
            "preprocess_on": recipe["preprocess_on"],
            "calibration_type": recipe["calibration_type"],
            "max_percentile": recipe["max_percentile"],
            "optimization": recipe["optimization"],
        },
        "compiler_parameters": {
            "compile_mode": recipe["compile_mode"],
            "debug": False,
            "optimize_level": recipe["optimize_level"],
            "jobs": args.jobs,
        },
    }
    config_path = output / f"{args.model_name}_config.yaml"
    preflight_pass = (
        static_batch_one
        and not custom_ops
        and not graph_blockers
        and formal_profile
        and contract_audit["status"] == "READY_FOR_ONNX_TOOLCHAIN_PREFLIGHT"
        and calibration_result["valid_count"] >= MINIMUM_CALIBRATION_SAMPLES
        and not calibration_result["invalid"]
    )
    if preflight_pass:
        config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    elif config_path.exists():
        config_path.unlink()
    report = {
        "schema_version": 3,
        "model_name": args.model_name,
        "model_path": str(model_path),
        "report_id": "tzcup_dosod_s100p_compile_preflight_v3",
        "model_sha256": sha256_file(model_path),
        "upstream_dosod_revision": UPSTREAM_DOSOD_REVISION,
        "onnx_checker_pass": True,
        "shape_inference_pass": True,
        "inputs": inputs,
        "outputs": outputs,
        "operator_inventory": operators,
        "custom_operators": custom_ops,
        "custom_operator_count": len(custom_ops),
        "onnx_ir_version": int(inferred.ir_version),
        "onnx_opsets": opsets,
        "onnx_node_count": len(inferred.graph.node),
        "graph_contract_blockers": graph_blockers,
        "graph_contract_pass": not graph_blockers,
        "fixed_batch_one": static_batch_one,
        "contract_audit_status": contract_audit["status"],
        "contract_sha256": contract_audit["contract_sha256"],
        "compile_plan_sha256": contract_audit["compile_plan_sha256"],
        "formal_mapper_profile": {
            "march": args.march,
            "expected_march": FORMAL_MARCH,
            "input_type_train": FORMAL_INPUT_TYPE_TRAIN,
            "input_type_rt": FORMAL_INPUT_TYPE_RUNTIME,
            "preprocess_on": False,
            "jobs": args.jobs,
            "matches_frozen_upstream": formal_profile,
        },
        "calibration": calibration_result,
        "calibration_at_least_500": calibration_result["valid_count"] >= MINIMUM_CALIBRATION_SAMPLES,
        "compile_config": config_path.name if preflight_pass else None,
        "compile_config_sha256": sha256_file(config_path) if preflight_pass else None,
        "calibration_manifest_sha256": contract_audit.get("calibration", {}).get("manifest_sha256"),
        "compile_config_emitted": preflight_pass,
        "compile_executed": False,
        "hbm_status": "HBM_NOT_PRODUCED",
        "hbm_runtime_validated": False,
        "blockers": graph_blockers,
        "preflight_pass": preflight_pass,
    }
    _write_report(output, args.model_name, report)
    if report["preflight_pass"]:
        return 0
    return 0 if args.allow_blocked_exit_zero else 2


if __name__ == "__main__":
    raise SystemExit(main())
