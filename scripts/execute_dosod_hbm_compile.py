#!/usr/bin/env python3
"""Execute one contract-bound ``hb_compile`` and retain an honest receipt.

This program does not manufacture an HBM.  It invokes the caller-supplied
compiler only after validating a successful contract-bound preflight, the frozen
contract, and a matching live compiler identity.  The emitted receipt is a
*compile* receipt, not a board-acceptance receipt; its status remains
``COMPILED_NOT_BOARD_ACCEPTED`` even after a successful compile.
"""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Callable

try:
    import yaml
except ImportError:  # surfaced as a blocker in the receipt
    yaml = None

from hbm_evidence_common import atomic_json, fresh_directory, load_object, normal_file, sha256_file
from validate_dosod_s100p_hbm_compile_contract import audit_calibration


RECEIPT_ID = "tzcup_s100p_dosod_hbm_compile_receipt_v1"
EXPECTED_PREFIX = "dosod_mlp3x_s_tzcup_rep-int16"


def _package_versions() -> dict[str, str | None]:
    values: dict[str, str | None] = {}
    for role, package in {"hbdk4_compiler": "hbdk4-compiler", "hmct": "hmct", "horizon_tc_ui": "horizon-tc-ui"}.items():
        try:
            values[role] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            values[role] = None
    return values


def _write_text(path: Path, value: str) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value, encoding="utf-8")
    return sha256_file(path)


def _block(receipt: dict[str, Any], reason: str) -> None:
    if reason not in receipt["blockers"]:
        receipt["blockers"].append(reason)


def _validate(
    *, contract_path: Path, preflight_path: Path, config_path: Path,
    identity_path: Path, calibration_manifest_path: Path, compiler: str,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], Path | None, list[str]]:
    blockers: list[str] = []
    contract = load_object(contract_path)
    preflight = load_object(preflight_path)
    identity = load_object(identity_path)
    calibration_manifest = load_object(calibration_manifest_path)
    if yaml is None:
        blockers.append("pyyaml_unavailable")
        return contract, preflight, identity, None, blockers
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        blockers.append("compile_config_not_object")
        return contract, preflight, identity, None, blockers
    if preflight.get("preflight_pass") is not True or preflight.get("compile_config_emitted") is not True:
        blockers.append("preflight_not_passed")
    if preflight.get("compile_executed") is not False or preflight.get("hbm_status") != "HBM_NOT_PRODUCED":
        blockers.append("preflight_claim_boundary_invalid")
    if preflight.get("model_sha256") != contract.get("model", {}).get("sha256"):
        blockers.append("preflight_model_identity_mismatch")
    if preflight.get("compile_config_sha256") != sha256_file(config_path):
        blockers.append("preflight_compile_config_identity_mismatch")
    if preflight.get("calibration_manifest_sha256") != sha256_file(calibration_manifest_path):
        blockers.append("preflight_calibration_manifest_identity_mismatch")
    records = calibration_manifest.get("records")
    if calibration_manifest.get("schema_version") != 1 or calibration_manifest.get("status") != "FROZEN" or not isinstance(records, list):
        blockers.append("calibration_manifest_not_frozen")
    elif len(records) < contract.get("calibration", {}).get("minimum_sample_count", 0):
        blockers.append("calibration_manifest_sample_count_insufficient")
    # Re-run the canonical calibration audit immediately before compile.  A
    # manifest digest alone cannot detect changed tensors, extra arrays or
    # source/holdout overlap after preflight.
    calibration_blockers: list[str] = []
    calibration_audit = audit_calibration(calibration_manifest_path.parent, contract, calibration_blockers)
    if calibration_audit.get("manifest_sha256") != sha256_file(calibration_manifest_path):
        blockers.append("calibration_audit_manifest_identity_mismatch")
    blockers.extend(f"calibration_reaudit:{value}" for value in calibration_blockers)
    if identity.get("identity_verified") is not True:
        blockers.append("compiler_identity_not_verified")
    toolchain = contract.get("toolchain", {})
    if identity.get("oe_version") != toolchain.get("oe_version"):
        blockers.append("compiler_identity_oe_version_mismatch")
    if identity.get("required_versions") != toolchain.get("required_versions"):
        blockers.append("compiler_identity_package_versions_mismatch")
    executable = shutil.which(compiler)
    if not executable:
        blockers.append("hb_compile_executable_missing")
    elif identity.get("hb_compile_executable_sha256") != sha256_file(Path(executable)):
        blockers.append("hb_compile_executable_identity_mismatch")
    model_parameters = config.get("model_parameters")
    if not isinstance(model_parameters, dict):
        blockers.append("compile_config_model_parameters_missing")
        return contract, preflight, identity, None, blockers
    if model_parameters.get("march") != contract.get("compile_recipe", {}).get("march"):
        blockers.append("compile_config_march_mismatch")
    if model_parameters.get("onnx_model") != preflight.get("model_path"):
        blockers.append("compile_config_model_path_mismatch")
    if model_parameters.get("output_model_file_prefix") != EXPECTED_PREFIX:
        blockers.append("compile_config_output_prefix_mismatch")
    working_dir = model_parameters.get("working_dir")
    if not isinstance(working_dir, str) or not working_dir:
        blockers.append("compile_config_working_dir_missing")
        return contract, preflight, identity, None, blockers
    expected_output = Path(working_dir) / f"{EXPECTED_PREFIX}.hbm"
    if expected_output.parent.is_symlink() or expected_output.parent.exists():
        blockers.append("compile_working_directory_preexisted")
    calibration_parameters = config.get("calibration_parameters")
    if not isinstance(calibration_parameters, dict) or calibration_parameters.get("cal_data_dir") != str(calibration_manifest_path.parent.resolve()):
        blockers.append("compile_config_calibration_path_mismatch")
    return contract, preflight, identity, expected_output, blockers


def execute_compile(
    *, contract_path: Path, preflight_path: Path, config_path: Path,
    identity_path: Path, calibration_manifest_path: Path, output: Path, compiler: str = "hb_compile",
    runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> dict[str, Any]:
    """Run the real compiler once; dependency injection is for unit tests only."""

    for path, label in ((contract_path, "contract"), (preflight_path, "preflight"),
                        (config_path, "compile_config"), (identity_path, "compiler_identity"),
                        (calibration_manifest_path, "calibration_manifest")):
        normal_file(path, label)
    fresh_directory(output, "evidence_output")
    started_ns = time.time_ns()
    receipt: dict[str, Any] = {
        "schema_version": 1,
        "receipt_id": RECEIPT_ID,
        "status": "BLOCKED",
        "claim_boundary": "compile evidence only; no board copy, board interaction, parity, metric regression, or board acceptance is claimed",
        "started_epoch_ns": started_ns,
        "ended_epoch_ns": None,
        "command": None,
        "returncode": None,
        "compiler": {"requested": compiler, "resolved": shutil.which(compiler), "package_versions": _package_versions()},
        "inputs": {"contract": str(contract_path.resolve()), "preflight": str(preflight_path.resolve()), "compile_config": str(config_path.resolve()), "compiler_identity": str(identity_path.resolve()), "calibration_manifest": str(calibration_manifest_path.resolve())},
        "evidence_root": str(output.resolve()),
        "output_relative_path": None,
        "output_path": None,
        "output_sha256": None,
        "output_byte_size": None,
        "output_created_by_this_compile": False,
        "compiler_identity_verified": False,
        "raw_stdout_path": "hb_compile.stdout.txt",
        "raw_stderr_path": "hb_compile.stderr.txt",
        "blockers": [],
    }
    try:
        contract, preflight, identity, expected_hbm, blockers = _validate(
            contract_path=contract_path, preflight_path=preflight_path, config_path=config_path,
            identity_path=identity_path, calibration_manifest_path=calibration_manifest_path, compiler=compiler,
        )
        receipt["inputs"].update({
            "contract_sha256": sha256_file(contract_path), "preflight_sha256": sha256_file(preflight_path),
            "compile_config_sha256": sha256_file(config_path), "compiler_identity_sha256": sha256_file(identity_path),
            "calibration_manifest_sha256": sha256_file(calibration_manifest_path),
            "model_sha256": contract.get("model", {}).get("sha256"),
        })
        receipt["calibration_reaudit"] = calibration_audit if "calibration_audit" in locals() else None
        receipt["compiler_identity_verified"] = identity.get("identity_verified") is True
        for blocker in blockers:
            _block(receipt, blocker)
        if expected_hbm is not None:
            receipt["output_path"] = str(expected_hbm)
            receipt["output_relative_path"] = contract.get("output", {}).get("relative_path")
            if expected_hbm.exists() or expected_hbm.is_symlink():
                _block(receipt, "expected_hbm_preexisted_before_compile")
        if not receipt["blockers"]:
            command = [compiler, "-c", str(config_path.resolve())]
            receipt["command"] = command
            try:
                completed = runner(command, capture_output=True, text=True, check=False)
                stdout = completed.stdout or ""
                stderr = completed.stderr or ""
                receipt["returncode"] = completed.returncode
            except (OSError, subprocess.TimeoutExpired) as exc:
                stdout, stderr = "", f"{type(exc).__name__}:{exc}"
                _block(receipt, "hb_compile_invocation_failed")
            receipt["raw_stdout_sha256"] = _write_text(output / receipt["raw_stdout_path"], stdout)
            receipt["raw_stderr_sha256"] = _write_text(output / receipt["raw_stderr_path"], stderr)
            if receipt["returncode"] != 0:
                _block(receipt, "hb_compile_nonzero_returncode")
            if expected_hbm is None or expected_hbm.is_symlink() or not expected_hbm.is_file():
                _block(receipt, "expected_hbm_missing")
            elif expected_hbm.stat().st_size <= 0:
                _block(receipt, "expected_hbm_empty")
            else:
                receipt["output_sha256"] = sha256_file(expected_hbm)
                receipt["output_byte_size"] = expected_hbm.stat().st_size
                receipt["output_created_by_this_compile"] = True
        if not receipt["blockers"]:
            receipt["status"] = "COMPILED_NOT_BOARD_ACCEPTED"
    except Exception as exc:
        _block(receipt, f"precompile_validation_failed:{type(exc).__name__}")
    finally:
        receipt["ended_epoch_ns"] = time.time_ns()
        receipt_path = output / "dosod_hbm_compile_receipt.json"
        receipt["receipt_path"] = str(receipt_path.resolve())
        receipt["producer_script_path"] = str(Path(__file__).resolve())
        receipt["producer_script_sha256"] = sha256_file(Path(__file__).resolve())
        atomic_json(receipt_path, receipt)
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", required=True, type=Path)
    parser.add_argument("--preflight-report", required=True, type=Path)
    parser.add_argument("--compile-config", required=True, type=Path)
    parser.add_argument("--compiler-identity", required=True, type=Path)
    parser.add_argument("--calibration-manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--compiler", default="hb_compile")
    args = parser.parse_args()
    try:
        receipt = execute_compile(contract_path=args.contract, preflight_path=args.preflight_report,
                                  config_path=args.compile_config, identity_path=args.compiler_identity,
                                  calibration_manifest_path=args.calibration_manifest, output=args.output,
                                  compiler=args.compiler)
    except Exception as exc:
        print(f"compile_receipt_blocked:{type(exc).__name__}:{exc}", file=sys.stderr)
        return 2
    print(json.dumps(receipt, ensure_ascii=False, indent=2))
    return 0 if receipt["status"] == "COMPILED_NOT_BOARD_ACCEPTED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
