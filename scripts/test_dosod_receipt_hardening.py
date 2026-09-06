from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

import pytest

import run_dosod_hbm_x86_parity as parity
import validate_dosod_quantized_metric_regression as metric


ROOT = Path(__file__).resolve().parents[1]


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _compile_receipt(root: Path, hbm: Path, *, canonical: bool = True) -> Path:
    stdout, stderr = root / "hb_compile.stdout.txt", root / "hb_compile.stderr.txt"
    stdout.write_text("ok\n", encoding="utf-8")
    stderr.write_text("", encoding="utf-8")
    path = root / "dosod_hbm_compile_receipt.json"
    producer = ROOT / "scripts" / "execute_dosod_hbm_compile.py"
    contract = ROOT / "config" / "dosod_s100p_hbm_compile_contract.json"
    value = {
        "receipt_id": "tzcup_s100p_dosod_hbm_compile_receipt_v1",
        "status": "COMPILED_NOT_BOARD_ACCEPTED", "returncode": 0,
        "output_created_by_this_compile": True, "output_sha256": _sha(hbm),
        "output_byte_size": hbm.stat().st_size, "evidence_root": str(root.resolve()),
        "receipt_path": str(path.resolve()), "producer_script_path": str(producer),
        "producer_script_sha256": _sha(producer), "raw_stdout_path": stdout.name,
        "raw_stdout_sha256": _sha(stdout), "raw_stderr_path": stderr.name,
        "raw_stderr_sha256": _sha(stderr),
        "inputs": {"contract_sha256": _sha(contract) if canonical else "0" * 64},
    }
    path.write_text(json.dumps(value), encoding="utf-8")
    return path


def test_parity_rejects_compile_receipt_without_canonical_contract(tmp_path: Path) -> None:
    hbm = tmp_path / "model.hbm"; hbm.write_bytes(b"hbm")
    receipt = _compile_receipt(tmp_path, hbm, canonical=False)
    with pytest.raises(ValueError, match="canonical_contract"):
        parity._validate_compile_receipt(receipt, hbm)


def test_production_compile_cli_rejects_noncanonical_contract_before_execution(tmp_path: Path) -> None:
    other = tmp_path / "lookalike.json"; other.write_text("{}", encoding="utf-8")
    command = [
        sys.executable, str(ROOT / "scripts" / "execute_dosod_hbm_compile.py"),
        "--contract", str(other), "--preflight-report", str(other),
        "--compile-config", str(other), "--compiler-identity", str(other),
        "--calibration-manifest", str(other), "--output", str(tmp_path / "out"),
    ]
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    assert result.returncode == 2
    assert "canonical DOSOD S100P contract" in result.stderr


def test_metric_compile_receipt_requires_canonical_contract_path_and_digest(tmp_path: Path) -> None:
    hbm = tmp_path / "model.hbm"; hbm.write_bytes(b"hbm")
    calibration = tmp_path / "calibration_manifest.json"; calibration.write_text("{}", encoding="utf-8")
    preflight = tmp_path / "preflight.json"; preflight.write_text("{}", encoding="utf-8")
    config = tmp_path / "compile.yaml"; config.write_text("x: 1\n", encoding="utf-8")
    identity = tmp_path / "identity.json"; identity.write_text("{}", encoding="utf-8")
    receipt = _compile_receipt(tmp_path, hbm)
    value = json.loads(receipt.read_text(encoding="utf-8"))
    value["inputs"].update({
        "contract": str(ROOT / "config" / "dosod_s100p_hbm_compile_contract.json"),
        "calibration_manifest_sha256": _sha(calibration),
        "preflight": str(preflight), "preflight_sha256": _sha(preflight),
        "compile_config": str(config), "compile_config_sha256": _sha(config),
        "compiler_identity": str(identity), "compiler_identity_sha256": _sha(identity),
    })
    receipt.write_text(json.dumps(value), encoding="utf-8")
    with pytest.raises(ValueError, match="preflight_contract"):
        metric._validate_compile_receipt(receipt, hbm=hbm, calibration_manifest=calibration)
