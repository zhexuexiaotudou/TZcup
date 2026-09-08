#!/usr/bin/env python3
"""Verify a current HBM against a fresh, bound ``hbrt4-disas`` receipt."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from capture_dosod_hbm_disas_abi import RECEIPT_ID, STATUS
from dosod_hbm_abi_contract import validate_hbrt4_disas
from hbm_evidence_common import normal_file, path_under, sha256_file


def _binding(root: Path, value: Any, label: str) -> Path:
    if not isinstance(value, dict) or set(value) != {"relative_path", "sha256", "byte_size"}:
        raise ValueError(f"{label}_binding_invalid")
    path = path_under(root, value["relative_path"], label)
    if path.stat().st_size != value["byte_size"] or sha256_file(path) != value["sha256"]:
        raise ValueError(f"{label}_binding_drift")
    return path


def _execution(value: Any, command: list[str], label: str) -> None:
    if (not isinstance(value, dict) or value.get("command") != command or value.get("returncode") != 0
            or not isinstance(value.get("execution"), dict)
            or value["execution"].get("timed_out") is not False
            or value["execution"].get("zero_survivor") is not True):
        raise ValueError(f"{label}_execution_invalid")


def validate(hbm: Path, receipt_path: Path) -> dict[str, object]:
    normal_file(hbm, "hbm"); normal_file(receipt_path, "hbrt4_disas_receipt")
    root = receipt_path.parent.resolve()
    try:
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("hbrt4_disas_receipt_invalid") from exc
    producer = Path(__file__).resolve().with_name("capture_dosod_hbm_disas_abi.py")
    if (receipt.get("receipt_id") != RECEIPT_ID or receipt.get("status") != STATUS
            or receipt.get("receipt_path") != str(receipt_path.resolve())
            or receipt.get("producer_script_path") != str(producer.resolve())
            or receipt.get("producer_script_sha256") != sha256_file(producer)
            or receipt.get("blockers") != []):
        raise ValueError("hbrt4_disas_receipt_header_invalid")
    hbm_binding = receipt.get("hbm")
    if (not isinstance(hbm_binding, dict) or set(hbm_binding) != {"path", "sha256", "byte_size"}
            or hbm_binding["path"] != str(hbm.resolve())
            or hbm_binding["sha256"] != sha256_file(hbm)
            or hbm_binding["byte_size"] != hbm.stat().st_size):
        raise ValueError("hbrt4_disas_hbm_binding_mismatch")
    tool = receipt.get("tool")
    if not isinstance(tool, dict) or set(tool) != {"path", "sha256"} or not isinstance(tool["path"], str):
        raise ValueError("hbrt4_disas_tool_binding_invalid")
    tool_path = Path(tool["path"]); normal_file(tool_path, "hbrt4_disas_tool")
    if tool["sha256"] != sha256_file(tool_path):
        raise ValueError("hbrt4_disas_tool_binding_drift")
    _execution(receipt.get("version"), [str(tool_path), "--version"], "hbrt4_disas_version")
    _execution(receipt.get("disas"), [str(tool_path), "--json", str(hbm.resolve())], "hbrt4_disas")
    _binding(root, receipt["version"].get("stdout"), "hbrt4_disas_version_stdout")
    _binding(root, receipt["version"].get("stderr"), "hbrt4_disas_version_stderr")
    stdout = _binding(root, receipt["disas"].get("stdout"), "hbrt4_disas_stdout")
    _binding(root, receipt["disas"].get("stderr"), "hbrt4_disas_stderr")
    try:
        disas = json.loads(stdout.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("hbrt4_disas_stdout_not_json") from exc
    validate_hbrt4_disas(disas)
    return {"status": STATUS, "hbm_sha256": hbm_binding["sha256"], "disas_stdout_sha256": receipt["disas"]["stdout"]["sha256"]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hbm", required=True, type=Path)
    parser.add_argument("--execution-receipt", required=True, type=Path)
    args = parser.parse_args()
    try:
        print(json.dumps(validate(args.hbm, args.execution_receipt), indent=2))
    except ValueError as exc:
        print(f"hbm_abi_blocked:{exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
