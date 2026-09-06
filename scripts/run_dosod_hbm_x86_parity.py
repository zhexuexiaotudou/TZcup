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
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Callable

import numpy as np

from hbm_evidence_common import atomic_json, fresh_directory, load_object, normal_file, path_under, sha256_file


REPORT_ID = "tzcup_dosod_hbm_x86_nash_parity_v1"
OUTPUTS = {"scores", "boxes"}
RUNNER_IDENTITY_ID = "tzcup_dosod_hbm_runner_identity_v1"
COMMAND_TEMPLATE = ["{runner}", "--model", "{hbm}", "--input", "{input}", "--output-path", "{output}"]


def _digest(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value.lower())


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


def _validate_compile_receipt(path: Path, hbm: Path) -> dict[str, Any]:
    receipt = load_object(path)
    if receipt.get("receipt_id") != "tzcup_s100p_dosod_hbm_compile_receipt_v1":
        raise ValueError("compile_receipt_id_invalid")
    if receipt.get("status") != "COMPILED_NOT_BOARD_ACCEPTED" or receipt.get("returncode") != 0:
        raise ValueError("compile_receipt_not_successful")
    if receipt.get("output_created_by_this_compile") is not True:
        raise ValueError("compile_receipt_does_not_prove_fresh_output")
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


def _validate_runner_identity(path: Path, holdout: dict[str, Any]) -> tuple[Path, dict[str, str], str, str]:
    """Bind the actual executable, adapter and exact command form before use."""

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
    output_map = identity.get("output_map")
    if not isinstance(output_map, dict) or set(output_map) != OUTPUTS or not all(isinstance(value, str) and value.endswith(".npy") for value in output_map.values()):
        raise ValueError("runner_identity_output_map_invalid")
    if identity.get("hbm_input_adapter") != holdout.get("hbm_input_adapter"):
        raise ValueError("runner_identity_adapter_mismatch")
    # The runner cannot self-attest.  Until the existing S100 live collector
    # records an official hbrt4 inventory receipt, this lane stays blocked.
    official = identity.get("official_runtime_receipt")
    if not isinstance(official, dict) or not isinstance(official.get("path"), str) or not Path(official["path"]).is_absolute():
        raise ValueError("runner_identity_official_receipt_missing")
    receipt_path = Path(official["path"])
    normal_file(receipt_path, "runner_official_runtime_receipt")
    if official.get("sha256") != sha256_file(receipt_path):
        raise ValueError("runner_identity_official_receipt_sha_mismatch")
    official_receipt = load_object(receipt_path)
    inventory = official_receipt.get("system", {}).get("runtime_inventory") if isinstance(official_receipt.get("system"), dict) else None
    hbrt4 = inventory.get("hbrt4") if isinstance(inventory, dict) else None
    if official_receipt.get("report_id") != "tzcup_formal_rdk_s100_live_runtime_raw_v1" or not isinstance(hbrt4, dict) or hbrt4.get("returncode") != 0:
        raise ValueError("runner_identity_official_receipt_unverified")
    return executable_path, {str(key): str(value) for key, value in output_map.items()}, sha256_file(path), runner["version"]


def run_parity(
    *, hbm: Path, onnx_model: Path, compile_receipt_path: Path,
    calibration_manifest: Path, holdout_manifest: Path, holdout_root: Path,
    runner_identity_path: Path, output: Path, atol: float, rtol: float,
    invoke: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> dict[str, Any]:
    for path, label in ((hbm, "hbm"), (onnx_model, "onnx_model"), (compile_receipt_path, "compile_receipt"),
                        (calibration_manifest, "calibration_manifest"), (holdout_manifest, "holdout_manifest"),
                        (runner_identity_path, "runner_identity")):
        normal_file(path, label)
    if atol < 0 or rtol < 0 or not math.isfinite(atol) or not math.isfinite(rtol):
        raise ValueError("parity_tolerance_invalid")
    fresh_directory(output, "evidence_output")
    report: dict[str, Any] = {
        "schema_version": 1, "report_id": REPORT_ID, "status": "BLOCKED",
        "claim_boundary": "x86 HBM/ONNX numeric parity only; no S100P board execution or metric acceptance is claimed",
        "hbm": {"path": str(hbm.resolve()), "sha256": sha256_file(hbm), "byte_size": hbm.stat().st_size},
        "onnx": {"path": str(onnx_model.resolve()), "sha256": sha256_file(onnx_model)},
        "tolerances": {"atol": atol, "rtol": rtol}, "runner": None,
        "records": [], "blockers": [], "started_epoch_ns": time.time_ns(), "ended_epoch_ns": None,
    }
    try:
        compile_receipt = _validate_compile_receipt(compile_receipt_path, hbm)
        compiled_model_sha = compile_receipt.get("inputs", {}).get("model_sha256") if isinstance(compile_receipt.get("inputs"), dict) else None
        if not _digest(compiled_model_sha) or compiled_model_sha != sha256_file(onnx_model):
            raise ValueError("compile_receipt_onnx_identity_mismatch")
        calibration_sources = _load_calibration_sources(calibration_manifest)
        manifest, records = _holdout_records(holdout_manifest, calibration_sources)
        runner_path, output_map, runner_identity_sha256, runner_version = _validate_runner_identity(runner_identity_path, manifest)
        report["compile_receipt_sha256"] = sha256_file(compile_receipt_path)
        report["holdout_manifest_sha256"] = sha256_file(holdout_manifest)
        report["calibration_manifest_sha256"] = sha256_file(calibration_manifest)
        report["runner_identity_sha256"] = runner_identity_sha256
        report["runner"] = {"absolute_path": str(runner_path), "sha256": sha256_file(runner_path), "version": runner_version}
        report["holdout_input_adapter"] = manifest["hbm_input_adapter"]
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
            binary_path = _relative(holdout_root, row.get("hbm_input_binary"), f"holdout_hbm_input:{sample_id}")
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
            command = [str(runner_path), "--model", str(hbm.resolve()), "--input", str(binary_path), "--output-path", str(sample_dir)]
            completed = invoke(command, capture_output=True, text=True, check=False)
            (sample_dir / "runner.stdout.txt").write_text(completed.stdout or "", encoding="utf-8")
            (sample_dir / "runner.stderr.txt").write_text(completed.stderr or "", encoding="utf-8")
            result: dict[str, Any] = {"sample_id": sample_id, "source_sha256": row["source_sha256"], "command": command,
                                      "returncode": completed.returncode, "outputs": {}, "pass": False}
            if completed.returncode != 0:
                result["blocker"] = "hbrt_runner_nonzero_returncode"
                report["records"].append(result)
                continue
            all_pass = True
            for name in sorted(OUTPUTS):
                produced = path_under(sample_dir, output_map[name], f"hbrt_output:{sample_id}:{name}")
                candidate = np.load(produced, allow_pickle=False)
                expected = reference[name]
                comparable = candidate.shape == expected.shape and np.isfinite(candidate).all()
                maximum = float(np.max(np.abs(candidate - expected))) if comparable else None
                passed = bool(comparable and np.allclose(candidate, expected, rtol=rtol, atol=atol))
                result["outputs"][name] = {"hbm_path": produced.relative_to(output).as_posix(), "hbm_sha256": sha256_file(produced),
                                           "onnx_shape": list(expected.shape), "hbm_shape": list(candidate.shape),
                                           "max_abs_error": maximum, "pass": passed}
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
    parser.add_argument("--atol", required=True, type=float)
    parser.add_argument("--rtol", default=0.0, type=float)
    args = parser.parse_args()
    try:
        report = run_parity(hbm=args.hbm, onnx_model=args.onnx_model, compile_receipt_path=args.compile_receipt,
                            calibration_manifest=args.calibration_manifest, holdout_manifest=args.holdout_manifest,
                            holdout_root=args.holdout_root, runner_identity_path=args.runner_identity, output=args.output,
                            atol=args.atol, rtol=args.rtol)
    except Exception as exc:
        print(f"x86_parity_blocked:{type(exc).__name__}:{exc}", file=sys.stderr)
        return 2
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["status"] == "PARITY_PASSED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
