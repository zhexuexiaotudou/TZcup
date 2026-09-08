from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

import capture_dosod_hbm_disas_abi as capture
import validate_dosod_hbm_disas_abi as subject
from test_dosod_hbm_abi_contract import _disas_fixture


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _binding(root, path):
    return {"relative_path": path.relative_to(root).as_posix(), "sha256": _sha(path), "byte_size": path.stat().st_size}


def _receipt(tmp_path):
    tmp_path.mkdir(parents=True, exist_ok=True)
    hbm, tool = tmp_path / "model.hbm", tmp_path / "hbrt4-disas"
    hbm.write_bytes(b"current hbm"); tool.write_bytes(b"tool")
    version, stderr, stdout = tmp_path / "version.txt", tmp_path / "stderr.txt", tmp_path / "disas.json"
    version.write_text("hbrt4-disas fixture", encoding="utf-8"); stderr.write_text("", encoding="utf-8")
    stdout.write_text(json.dumps(_disas_fixture()), encoding="utf-8")
    execution = {"timed_out": False, "zero_survivor": True}
    receipt_path = tmp_path / "receipt.json"
    receipt = {"schema_version": 1, "receipt_id": capture.RECEIPT_ID, "status": capture.STATUS,
               "hbm": {"path": str(hbm.resolve()), "sha256": _sha(hbm), "byte_size": hbm.stat().st_size},
               "tool": {"path": str(tool.resolve()), "sha256": _sha(tool)},
               "version": {"command": [str(tool.resolve()), "--version"], "returncode": 0, "execution": execution, "stdout": _binding(tmp_path, version), "stderr": _binding(tmp_path, stderr)},
               "disas": {"command": [str(tool.resolve()), "--json", str(hbm.resolve())], "returncode": 0, "execution": execution, "stdout": _binding(tmp_path, stdout), "stderr": _binding(tmp_path, stderr)},
               "blockers": [], "receipt_path": str(receipt_path.resolve()),
               "producer_script_path": str(Path(capture.__file__).resolve()), "producer_script_sha256": _sha(Path(capture.__file__))}
    receipt_path.write_text(json.dumps(receipt), encoding="utf-8")
    return hbm, receipt_path, stdout


def test_bound_receipt_verifies_current_hbm_and_physical_layout(tmp_path) -> None:
    hbm, receipt, _ = _receipt(tmp_path)
    assert subject.validate(hbm, receipt)["status"] == capture.STATUS


def test_receipt_rejects_replaced_hbm_or_stale_disas_json(tmp_path) -> None:
    hbm, receipt, stdout = _receipt(tmp_path)
    hbm.write_bytes(b"replacement hbm")
    with pytest.raises(ValueError, match="hbm_binding_mismatch"):
        subject.validate(hbm, receipt)
    hbm, receipt, stdout = _receipt(tmp_path / "stale")
    stdout.write_text(json.dumps({"graphs": []}), encoding="utf-8")
    with pytest.raises(ValueError, match="stdout_binding_drift"):
        subject.validate(hbm, receipt)


def test_receipt_rejects_nonzero_disas_execution(tmp_path) -> None:
    hbm, receipt_path, _ = _receipt(tmp_path)
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    receipt["disas"]["returncode"] = 1
    receipt_path.write_text(json.dumps(receipt), encoding="utf-8")
    with pytest.raises(ValueError, match="hbrt4_disas_execution_invalid"):
        subject.validate(hbm, receipt_path)
