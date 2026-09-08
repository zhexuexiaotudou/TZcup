#!/usr/bin/env python3
"""Run actual x86 Nash HBM outputs against ONNX on frozen non-calibration data.

The official runner interface used here is the existing project adapter's
``hbrt4-run-model --model HBM --input BINARY --output-path DIRECTORY``.  Its
output decoder is intentionally strict: each declared output must be an NPY
file produced under that directory.  If an installed official runner emits a
different wire format, this tool blocks rather than guessing its layout.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import subprocess
import sys
import time
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any, Callable

import numpy as np

from hbm_evidence_common import atomic_json, fresh_directory, load_object, normal_file, path_under, run_owned_process, sha256_file
from dosod_hbm_abi_contract import validate_post_hbm_inputs, validate_post_hbm_outputs


REPORT_ID = "tzcup_dosod_hbm_x86_nash_parity_v1"
OUTPUTS = {"scores", "boxes"}
RUNNER_IDENTITY_ID = "tzcup_dosod_hbm_runner_identity_v1"
MODEL_INFO_TEMPLATE = ["{runner}", "model_info", "--model_file={hbm}"]
COMMAND_TEMPLATE = ["{runner}", "infer", "--model_file={hbm}", "--input_file={inputs}", "--frame_count=1", "--enable_dump=true", "--dump_format=npy", "--dump_path={dump_path}", "--dequantize_process=true"]
CANONICAL_CONTRACT = Path(__file__).resolve().parents[1] / "config" / "dosod_s100p_hbm_compile_contract.json"
PARITY_DEADLINE_SECONDS = 1800


def _digest(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value.lower())


def raw_parity_metrics(expected: np.ndarray, actual: np.ndarray) -> dict[str, float]:
    """Separate raw tensor math: no allclose and no cross-output concatenation."""
    if expected.shape != actual.shape or not np.isfinite(expected).all() or not np.isfinite(actual).all():
        raise ValueError("raw_parity_nonfinite_or_shape_mismatch")
    left, right = expected.reshape(-1).astype(np.float64), actual.reshape(-1).astype(np.float64)
    left_norm, right_norm = float(np.linalg.norm(left)), float(np.linalg.norm(right))
    cosine = 1.0 if left_norm == 0.0 and right_norm == 0.0 else (0.0 if left_norm == 0.0 or right_norm == 0.0 else float(np.dot(left, right) / (left_norm * right_norm)))
    diff = right - left
    rmse = float(np.sqrt(np.mean(diff * diff)))
    rms = float(np.sqrt(np.mean(left * left)))
    return {"cosine": cosine, "normalized_rmse": rmse / max(rms, 1e-12),
            "pixel_mae": float(np.mean(np.abs(diff))), "pixel_p95": float(np.percentile(np.abs(diff), 95))}


def _block(report: dict[str, Any], value: str) -> None:
    if value not in report["blockers"]:
        report["blockers"].append(value)


def _relative(root: Path, value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label}_missing")
    return path_under(root, value, label)


def _load_calibration_sources(path: Path) -> set[str]:
    manifest = load_object(path)
    if manifest.get("status") != "FROZEN" or not isinstance(manifest.get("records"), list):
        raise ValueError("calibration_manifest_not_frozen")
    values = {row.get("source_sha256") for row in manifest["records"] if isinstance(row, dict)}
    if not values or not all(_digest(item) for item in values):
        raise ValueError("calibration_manifest_source_sha_invalid")
    return {str(item).lower() for item in values}


def _compile_receipt_relocated(receipt: dict[str, Any], path: Path) -> bool:
    """Allow a copied evidence directory, never a rewritten receipt location."""
    def origin_path(value: str):
        for cls in (PureWindowsPath, PurePosixPath):
            candidate = cls(value)
            if candidate.is_absolute():
                return candidate
        raise ValueError("compile_receipt_origin_location_invalid")

    root, receipt_path = receipt.get("evidence_root"), receipt.get("receipt_path")
    if not isinstance(root, str) or not isinstance(receipt_path, str):
        raise ValueError("compile_receipt_origin_location_invalid")
    origin_receipt, origin_root = origin_path(receipt_path), origin_path(root)
    if origin_receipt.name != path.name or origin_receipt.parent != origin_root:
        raise ValueError("compile_receipt_origin_location_invalid")
    relocated = origin_receipt != path.resolve()
    if not relocated and (Path(root).is_symlink() or Path(root).resolve() != path.parent.resolve()):
        raise ValueError("compile_receipt_evidence_root_invalid")
    return relocated


def is_absolute_origin(value: Any) -> bool:
    return isinstance(value, str) and (PureWindowsPath(value).is_absolute() or PurePosixPath(value).is_absolute())


def _validate_compile_receipt(path: Path, hbm: Path) -> dict[str, Any]:
    receipt = load_object(path)
    if receipt.get("receipt_id") != "tzcup_s100p_dosod_hbm_compile_receipt_v1":
        raise ValueError("compile_receipt_id_invalid")
    if receipt.get("status") != "COMPILED_NOT_BOARD_ACCEPTED" or receipt.get("returncode") != 0:
        raise ValueError("compile_receipt_not_successful")
    if receipt.get("output_created_by_this_compile") is not True:
        raise ValueError("compile_receipt_does_not_prove_fresh_output")
    relocated = _compile_receipt_relocated(receipt, path)
    producer = Path(__file__).resolve().with_name("execute_dosod_hbm_compile.py")
    source_producer = receipt.get("producer_script_path")
    if not is_absolute_origin(source_producer) or receipt.get("producer_script_sha256") != sha256_file(producer):
        raise ValueError("compile_receipt_producer_identity_mismatch")
    if not relocated and source_producer != str(producer):
        raise ValueError("compile_receipt_producer_identity_mismatch")
    inputs = receipt.get("inputs")
    if not isinstance(inputs, dict) or inputs.get("contract_sha256") != sha256_file(CANONICAL_CONTRACT):
        raise ValueError("compile_receipt_canonical_contract_mismatch")
    for stream in ("stdout", "stderr"):
        name, digest = receipt.get(f"raw_{stream}_path"), receipt.get(f"raw_{stream}_sha256")
        if not isinstance(name, str) or not _digest(digest):
            raise ValueError("compile_receipt_raw_output_identity_missing")
        candidate = (path.parent / name).resolve()
        if not candidate.is_relative_to(path.parent.resolve()):
            raise ValueError("compile_receipt_raw_output_path_escape")
        normal_file(candidate, f"compile_receipt_{stream}")
        if sha256_file(candidate) != digest:
            raise ValueError("compile_receipt_raw_output_identity_mismatch")
    if receipt.get("output_sha256") != sha256_file(hbm) or receipt.get("output_byte_size") != hbm.stat().st_size:
        raise ValueError("compile_receipt_hbm_identity_mismatch")
    return receipt


def _holdout_records(path: Path, calibration_sources: set[str]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    manifest = load_object(path)
    if manifest.get("schema_version") != 1 or manifest.get("status") != "FROZEN":
        raise ValueError("holdout_manifest_not_frozen")
    adapter = manifest.get("hbm_input_adapter")
    if not isinstance(adapter, dict) or adapter.get("status") != "VERIFIED" or not isinstance(adapter.get("command"), list):
        raise ValueError("holdout_hbm_input_adapter_unverified")
    records = manifest.get("records")
    if not isinstance(records, list) or not records:
        raise ValueError("holdout_records_missing")
    seen: set[str] = set()
    for index, row in enumerate(records):
        if not isinstance(row, dict):
            raise ValueError(f"holdout_record_not_object:{index}")
        source = row.get("source_sha256")
        if not _digest(source):
            raise ValueError(f"holdout_source_sha_invalid:{index}")
        source = str(source).lower()
        if source in seen:
            raise ValueError("holdout_duplicate_source_sha")
        if source in calibration_sources:
            raise ValueError("holdout_calibration_source_overlap")
        seen.add(source)
    return manifest, records


def _onnx_session(model: Path):
    try:
        import onnxruntime as ort
    except ImportError as exc:
        raise ValueError("onnxruntime_unavailable") from exc
    return ort.InferenceSession(str(model), providers=["CPUExecutionProvider"])


def _output_binding(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict) or set(value) != {"index", "name", "shape", "dtype"}:
        raise ValueError(f"runner_identity_output_binding_invalid:{label}")
    index, name, shape, dtype = value["index"], value["name"], value["shape"], value["dtype"]
    if not isinstance(index, int) or isinstance(index, bool) or index < 0:
        raise ValueError(f"runner_identity_output_index_invalid:{label}")
    if not isinstance(name, str) or not name or any(token in name for token in ("\\", "\n", "\r")):
        raise ValueError(f"runner_identity_output_name_invalid:{label}")
    if not isinstance(shape, list) or not shape or any(not isinstance(item, int) or isinstance(item, bool) or item <= 0 for item in shape):
        raise ValueError(f"runner_identity_output_shape_invalid:{label}")
    if not isinstance(dtype, str) or not re.fullmatch(r"HB_DNN_TENSOR_TYPE_[A-Z0-9]+", dtype):
        raise ValueError(f"runner_identity_output_dtype_invalid:{label}")
    return {"index": index, "name": name, "shape": shape, "dtype": dtype}


def _input_binding(value: Any) -> dict[str, Any]:
    binding = _output_binding({key: value[key] for key in ("index", "name", "shape", "dtype")} if isinstance(value, dict) and set(value) == {"index", "name", "shape", "dtype", "aligned_byte_size"} else value, "input")
    aligned = value.get("aligned_byte_size") if isinstance(value, dict) else None
    if not isinstance(aligned, int) or isinstance(aligned, bool) or aligned < -1:
        raise ValueError("runner_identity_input_aligned_size_invalid")
    binding["aligned_byte_size"] = aligned
    return binding


def _input_bindings(value: Any) -> list[dict[str, Any]]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError("runner_identity_inputs_invalid")
    bindings = [_input_binding(item) for item in value]
    try:
        validate_post_hbm_inputs(bindings)
    except ValueError:
        raise ValueError("runner_identity_inputs_not_project_nv12_planes")
    return bindings


def _dtype_size(dtype: str) -> int:
    sizes = {"HB_DNN_TENSOR_TYPE_BOOL8": 1, "HB_DNN_TENSOR_TYPE_S8": 1, "HB_DNN_TENSOR_TYPE_U8": 1,
             "HB_DNN_TENSOR_TYPE_F16": 2, "HB_DNN_TENSOR_TYPE_S16": 2, "HB_DNN_TENSOR_TYPE_U16": 2,
             "HB_DNN_TENSOR_TYPE_F32": 4, "HB_DNN_TENSOR_TYPE_S32": 4, "HB_DNN_TENSOR_TYPE_U32": 4,
             "HB_DNN_TENSOR_TYPE_F64": 8, "HB_DNN_TENSOR_TYPE_S64": 8, "HB_DNN_TENSOR_TYPE_U64": 8}
    if dtype not in sizes:
        raise ValueError("runner_model_info_input_dtype_unsupported")
    return sizes[dtype]


def _validate_runner_identity(path: Path, holdout: dict[str, Any]) -> tuple[Path, str, list[dict[str, Any]], dict[str, dict[str, Any]], str, str]:
    """Bind the official x86 tool and its declared HBM output contract."""

    identity = load_object(path)
    runner = identity.get("runner")
    if identity.get("schema_version") != 1 or identity.get("report_id") != RUNNER_IDENTITY_ID or identity.get("status") != "VERIFIED" or not isinstance(runner, dict):
        raise ValueError("runner_identity_not_verified")
    executable = runner.get("absolute_path")
    if not isinstance(executable, str) or not Path(executable).is_absolute():
        raise ValueError("runner_identity_path_not_absolute")
    executable_path = Path(executable)
    normal_file(executable_path, "runner_executable")
    if runner.get("sha256") != sha256_file(executable_path):
        raise ValueError("runner_identity_executable_sha_mismatch")
    if not isinstance(runner.get("version"), str) or not runner["version"].strip():
        raise ValueError("runner_identity_version_missing")
    if identity.get("command_template") != COMMAND_TEMPLATE:
        raise ValueError("runner_identity_command_template_mismatch")
    model_name = identity.get("model_name")
    if not isinstance(model_name, str) or not model_name.strip() or any(token in model_name for token in ("\n", "\r")):
        raise ValueError("runner_identity_model_name_invalid")
    input_binding = _input_bindings(identity.get("inputs"))
    output_map = identity.get("output_map")
    if not isinstance(output_map, dict) or set(output_map) != OUTPUTS:
        raise ValueError("runner_identity_output_map_invalid")
    bindings = {name: _output_binding(value, name) for name, value in output_map.items()}
    try:
        validate_post_hbm_outputs(bindings)
    except ValueError:
        raise ValueError("runner_identity_outputs_not_project_int16_pred_major") from None
    if len({binding["index"] for binding in bindings.values()}) != len(bindings) or len({binding["name"] for binding in bindings.values()}) != len(bindings):
        raise ValueError("runner_identity_output_map_not_unique")
    if identity.get("hbm_input_adapter") != holdout.get("hbm_input_adapter"):
        raise ValueError("runner_identity_adapter_mismatch")
    return executable_path, model_name, input_binding, bindings, sha256_file(path), runner["version"]


def _run(command: list[str], invoke: Callable[..., subprocess.CompletedProcess[str]], deadline: float) -> tuple[int, str, str, dict[str, Any]]:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise ValueError("parity_whole_deadline_exceeded")
    if invoke is subprocess.run:
        return run_owned_process(command, timeout_seconds=remaining)
    completed = invoke(command, capture_output=True, text=True, check=False)
    return completed.returncode, completed.stdout or "", completed.stderr or "", {"deadline_seconds": remaining, "zero_survivor": True, "fixture_injected": True}


def _parse_model_info(stdout: str, model_name: str) -> dict[str, dict[int, dict[str, Any]]]:
    if model_name not in re.findall(r"^\[model name\]:\s*(.+?)\s*$", stdout, flags=re.MULTILINE):
        raise ValueError("runner_model_info_model_name_missing")
    parsed: dict[str, dict[int, dict[str, Any]]] = {"input": {}, "output": {}}
    starts = list(re.finditer(r"^(input|output)\[(\d+)\]:\s*$", stdout, flags=re.MULTILINE))
    if not starts:
        raise ValueError("runner_model_info_tensors_missing")
    for position, match in enumerate(starts):
        section = stdout[match.end(): starts[position + 1].start() if position + 1 < len(starts) else len(stdout)]
        name = re.search(r"^name:\s*(.+?)\s*$", section, flags=re.MULTILINE)
        shape = re.search(r"^valid shape:\s*\(([^)]*)\)\s*$", section, flags=re.MULTILINE)
        dtype = re.search(r"^tensor type:\s*(HB_DNN_TENSOR_TYPE_[A-Z0-9]+)\s*$", section, flags=re.MULTILINE)
        if name is None or shape is None or dtype is None:
            raise ValueError(f"runner_model_info_tensor_invalid:{match.group(1)}")
        dimensions = [int(item.strip()) for item in shape.group(1).split(",") if item.strip()]
        if not dimensions or any(item <= 0 for item in dimensions):
            raise ValueError(f"runner_model_info_shape_invalid:{match.group(2)}")
        kind, index = match.group(1), int(match.group(2))
        if index in parsed[kind]:
            raise ValueError(f"runner_model_info_{kind}_index_duplicate")
        binding = {"index": index, "name": name.group(1), "shape": dimensions, "dtype": dtype.group(1)}
        aligned = re.search(r"^aligned byte size:\s*(-?\d+)\s*$", section, flags=re.MULTILINE)
        if kind == "input":
            if aligned is None or int(aligned.group(1)) < -1:
                raise ValueError("runner_model_info_input_aligned_size_invalid")
            binding["aligned_byte_size"] = int(aligned.group(1))
        parsed[kind][index] = binding
    return parsed


def _validate_model_info(stdout: str, model_name: str, input_binding: list[dict[str, Any]], output_map: dict[str, dict[str, Any]]) -> dict[str, dict[int, dict[str, Any]]]:
    try:
        validate_post_hbm_inputs(input_binding)
        validate_post_hbm_outputs(output_map)
    except ValueError:
        raise ValueError("runner_model_info_abi_contract_mismatch") from None
    observed = _parse_model_info(stdout, model_name)
    if observed["input"] != {item["index"]: item for item in input_binding}:
        raise ValueError("runner_model_info_input_binding_mismatch")
    if len(observed["output"]) != len(output_map):
        raise ValueError("runner_model_info_output_count_mismatch")
    for logical_name, expected in output_map.items():
        actual = observed["output"].get(expected["index"])
        if actual != expected:
            raise ValueError(f"runner_model_info_output_binding_mismatch:{logical_name}")
    return observed


def _dump_filename(binding: dict[str, Any]) -> str:
    return f"model_infer_output_{binding['index']}_{binding['name'].replace('/', '_')}.npy"


def run_parity(
    *, hbm: Path, onnx_model: Path, compile_receipt_path: Path,
    calibration_manifest: Path, holdout_manifest: Path, holdout_root: Path,
    runner_identity_path: Path, output: Path, deadline_seconds: int = PARITY_DEADLINE_SECONDS,
    invoke: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> dict[str, Any]:
    for path, label in ((hbm, "hbm"), (onnx_model, "onnx_model"), (compile_receipt_path, "compile_receipt"),
                        (calibration_manifest, "calibration_manifest"), (holdout_manifest, "holdout_manifest"),
                        (runner_identity_path, "runner_identity")):
        normal_file(path, label)
    if not isinstance(deadline_seconds, int) or isinstance(deadline_seconds, bool) or deadline_seconds <= 0:
        raise ValueError("parity_deadline_invalid")
    fresh_directory(output, "evidence_output")
    report: dict[str, Any] = {
        "schema_version": 1, "report_id": REPORT_ID, "status": "BLOCKED",
        "claim_boundary": "x86 HBM/ONNX numeric parity only; no S100P board execution or metric acceptance is claimed",
        "hbm": {"path": str(hbm.resolve()), "sha256": sha256_file(hbm), "byte_size": hbm.stat().st_size},
        "onnx": {"path": str(onnx_model.resolve()), "sha256": sha256_file(onnx_model)},
        "raw_output_gate": {"cosine_min": 0.99, "normalized_rmse_max": 0.02}, "deadline_seconds": deadline_seconds, "runner": None,
        "records": [], "blockers": [], "started_epoch_ns": time.time_ns(), "ended_epoch_ns": None,
    }
    try:
        compile_receipt = _validate_compile_receipt(compile_receipt_path, hbm)
        compiled_model_sha = compile_receipt.get("inputs", {}).get("model_sha256") if isinstance(compile_receipt.get("inputs"), dict) else None
        if not _digest(compiled_model_sha) or compiled_model_sha != sha256_file(onnx_model):
            raise ValueError("compile_receipt_onnx_identity_mismatch")
        calibration_sources = _load_calibration_sources(calibration_manifest)
        manifest, records = _holdout_records(holdout_manifest, calibration_sources)
        runner_path, model_name, input_binding, output_map, runner_identity_sha256, runner_version = _validate_runner_identity(runner_identity_path, manifest)
        report["compile_receipt_sha256"] = sha256_file(compile_receipt_path)
        report["holdout_manifest_sha256"] = sha256_file(holdout_manifest)
        report["calibration_manifest_sha256"] = sha256_file(calibration_manifest)
        report["runner_identity_sha256"] = runner_identity_sha256
        report["runner"] = {"absolute_path": str(runner_path), "sha256": sha256_file(runner_path), "expected_version": runner_version}
        report["holdout_input_adapter"] = manifest["hbm_input_adapter"]
        deadline = time.monotonic() + deadline_seconds
        version_command = [str(runner_path), "--version"]
        version_rc, version_stdout, version_stderr, version_execution = _run(version_command, invoke, deadline)
        (output / "runner.version.stdout.txt").write_text(version_stdout, encoding="utf-8")
        (output / "runner.version.stderr.txt").write_text(version_stderr, encoding="utf-8")
        if version_rc != 0 or version_execution.get("zero_survivor") is not True:
            raise ValueError("runner_version_nonzero_returncode")
        if version_stdout.strip() != runner_version.strip():
            raise ValueError("runner_version_mismatch")
        model_info_command = [part.format(runner=str(runner_path), hbm=str(hbm.resolve()), model_name=model_name) for part in MODEL_INFO_TEMPLATE]
        info_rc, info_stdout, info_stderr, info_execution = _run(model_info_command, invoke, deadline)
        (output / "runner.model_info.stdout.txt").write_text(info_stdout, encoding="utf-8")
        (output / "runner.model_info.stderr.txt").write_text(info_stderr, encoding="utf-8")
        if info_rc != 0 or info_execution.get("zero_survivor") is not True:
            raise ValueError("runner_model_info_nonzero_returncode")
        observed_tensors = _validate_model_info(info_stdout, model_name, input_binding, output_map)
        report["runner"].update({
            "version": version_stdout.strip(), "version_command": version_command,
            "version_stdout_sha256": sha256_file(output / "runner.version.stdout.txt"),
            "model_info_command": model_info_command,
            "model_info_stdout_sha256": sha256_file(output / "runner.model_info.stdout.txt"),
            "model_name": model_name, "inputs": input_binding,
            "outputs": [observed_tensors["output"][index] for index in sorted(observed_tensors["output"])],
        })
        session = _onnx_session(onnx_model)
        input_meta = session.get_inputs()
        output_names = [item.name for item in session.get_outputs()]
        if len(input_meta) != 1 or input_meta[0].name != "images" or set(output_names) != OUTPUTS:
            raise ValueError("onnx_io_contract_invalid")
        for index, row in enumerate(records):
            sample_id = row.get("sample_id")
            if not isinstance(sample_id, str) or not sample_id or any(token in sample_id for token in ("/", "\\", "..")):
                raise ValueError(f"holdout_sample_id_invalid:{index}")
            tensor_path = _relative(holdout_root, row.get("onnx_input_npy"), f"holdout_onnx_input:{sample_id}")
            files = row.get("hbm_input_files")
            by_role = {item.get("role"): item for item in files if isinstance(item, dict)} if isinstance(files, list) else {}
            if len(by_role) != len(input_binding):
                raise ValueError(f"holdout_hbm_inputs_contract_invalid:{sample_id}")
            binary_paths: list[Path] = []
            for binding in input_binding:
                item = by_role.get(binding["name"])
                if not isinstance(item, dict) or set(item) != {"role", "relative_path", "sha256", "byte_size"}:
                    raise ValueError(f"holdout_hbm_inputs_contract_invalid:{sample_id}")
                binary_path = _relative(holdout_root, item["relative_path"], f"holdout_hbm_input:{sample_id}:{binding['name']}")
                normal_file(binary_path, f"holdout_hbm_input:{sample_id}:{binding['name']}")
                valid_byte_size = math.prod(binding["shape"]) * _dtype_size(binding["dtype"])
                if binary_path.suffix != ".bin" or binary_path.stat().st_size != valid_byte_size or item["byte_size"] != valid_byte_size or item["sha256"] != sha256_file(binary_path):
                    raise ValueError(f"holdout_hbm_inputs_contract_invalid:{sample_id}")
                binary_paths.append(binary_path)
            tensor = np.load(tensor_path, allow_pickle=False)
            if tensor.dtype != np.dtype("float32") or tuple(tensor.shape) != (1, 3, 640, 640) or not np.isfinite(tensor).all():
                raise ValueError(f"holdout_onnx_tensor_contract_invalid:{sample_id}")
            if float(tensor.min()) < 0.0 or float(tensor.max()) > 1.0:
                raise ValueError(f"holdout_onnx_tensor_range_invalid:{sample_id}")
            reference = dict(zip(output_names, session.run(output_names, {"images": tensor})))
            if any(tuple(reference[name].shape) != (1, 8400, 4) for name in OUTPUTS):
                raise ValueError(f"onnx_output_shape_contract_invalid:{sample_id}")
            sample_dir = output / "samples" / sample_id
            sample_dir.mkdir(parents=True, exist_ok=False)
            dump_path = sample_dir.as_posix().rstrip("/") + "/"
            command = [part.format(runner=str(runner_path), hbm=str(hbm.resolve()), model_name=model_name,
                                   inputs=",".join(str(path) for path in binary_paths), dump_path=dump_path) for part in COMMAND_TEMPLATE]
            returncode, stdout, stderr, execution = _run(command, invoke, deadline)
            (sample_dir / "runner.stdout.txt").write_text(stdout, encoding="utf-8")
            (sample_dir / "runner.stderr.txt").write_text(stderr, encoding="utf-8")
            result: dict[str, Any] = {"sample_id": sample_id, "source_sha256": row["source_sha256"], "command": command,
                                      "returncode": returncode, "execution": execution, "outputs": {}, "pass": False}
            if returncode != 0 or execution.get("zero_survivor") is not True:
                result["blocker"] = "hbrt_runner_nonzero_returncode"
                report["records"].append(result)
                continue
            all_pass = True
            for name in sorted(OUTPUTS):
                binding = output_map[name]
                produced = path_under(sample_dir, _dump_filename(binding), f"hrt_output:{sample_id}:{name}")
                candidate = np.load(produced, allow_pickle=False)
                expected = reference[name]
                comparable = candidate.dtype == np.dtype("float32") and tuple(binding["shape"]) == tuple(expected.shape) and candidate.shape == expected.shape and np.isfinite(candidate).all()
                metrics = raw_parity_metrics(expected, candidate) if comparable else None
                passed = bool(metrics and metrics["cosine"] >= 0.99 and metrics["normalized_rmse"] <= 0.02)
                result["outputs"][name] = {"hbm_path": produced.relative_to(output).as_posix(), "hbm_sha256": sha256_file(produced),
                                           "onnx_shape": list(expected.shape), "hbm_shape": list(candidate.shape),
                                           "hbm_dtype": str(candidate.dtype), "model_info": binding,
                                           "raw_metrics": metrics, "pass": passed}
                np.save(sample_dir / f"onnx_{name}.npy", expected, allow_pickle=False)
                all_pass = all_pass and passed
            result["pass"] = all_pass
            report["records"].append(result)
        if report["records"] and all(row["pass"] for row in report["records"]):
            report["status"] = "PARITY_PASSED"
        elif not report["records"]:
            _block(report, "no_holdout_records_executed")
        else:
            _block(report, "x86_hbm_onnx_parity_failed")
    except Exception as exc:
        _block(report, f"parity_precondition_or_execution_failed:{type(exc).__name__}")
    finally:
        report["ended_epoch_ns"] = time.time_ns()
        atomic_json(output / "dosod_hbm_x86_parity.json", report)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hbm", required=True, type=Path)
    parser.add_argument("--onnx-model", required=True, type=Path)
    parser.add_argument("--compile-receipt", required=True, type=Path)
    parser.add_argument("--calibration-manifest", required=True, type=Path)
    parser.add_argument("--holdout-manifest", required=True, type=Path)
    parser.add_argument("--holdout-root", required=True, type=Path)
    parser.add_argument("--runner-identity", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        report = run_parity(hbm=args.hbm, onnx_model=args.onnx_model, compile_receipt_path=args.compile_receipt,
                            calibration_manifest=args.calibration_manifest, holdout_manifest=args.holdout_manifest,
                            holdout_root=args.holdout_root, runner_identity_path=args.runner_identity, output=args.output,
                            deadline_seconds=PARITY_DEADLINE_SECONDS)
    except Exception as exc:
        print(f"x86_parity_blocked:{type(exc).__name__}:{exc}", file=sys.stderr)
        return 2
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["status"] == "PARITY_PASSED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
