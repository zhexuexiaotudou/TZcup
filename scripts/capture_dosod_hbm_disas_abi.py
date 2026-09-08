#!/usr/bin/env python3
"""Capture a fresh, process-supervised ``hbrt4-disas --json`` ABI receipt."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from dosod_hbm_abi_contract import validate_hbrt4_disas
from hbm_evidence_common import atomic_json, fresh_directory, normal_file, run_owned_process, sha256_file


RECEIPT_ID = "tzcup_dosod_hbrt4_disas_abi_receipt_v1"
STATUS = "HBM_ABI_VERIFIED"
DEADLINE_SECONDS = 120


def _file(root: Path, path: Path) -> dict[str, object]:
    return {"relative_path": path.relative_to(root).as_posix(), "sha256": sha256_file(path), "byte_size": path.stat().st_size}


def _run(command: list[str]) -> tuple[int | None, str, str, dict]:
    return run_owned_process(command, timeout_seconds=DEADLINE_SECONDS)


def capture(hbm: Path, tool: Path, output: Path) -> dict:
    normal_file(hbm, "hbm"); normal_file(tool, "hbrt4_disas_tool")
    fresh_directory(output, "hbrt4_disas_evidence_output")
    receipt_path = output / "dosod_hbrt4_disas_abi_receipt.json"
    receipt = {"schema_version": 1, "receipt_id": RECEIPT_ID, "status": "BLOCKED", "hbm": {"path": str(hbm.resolve()), "sha256": sha256_file(hbm), "byte_size": hbm.stat().st_size}, "tool": {"path": str(tool.resolve()), "sha256": sha256_file(tool)}, "version": None, "disas": None, "blockers": []}
    try:
        version_command = [str(tool.resolve()), "--version"]
        rc, stdout, stderr, execution = _run(version_command)
        version_stdout, version_stderr = output / "hbrt4-disas.version.stdout.txt", output / "hbrt4-disas.version.stderr.txt"
        version_stdout.write_text(stdout, encoding="utf-8"); version_stderr.write_text(stderr, encoding="utf-8")
        receipt["version"] = {"command": version_command, "returncode": rc, "execution": execution, "stdout": _file(output, version_stdout), "stderr": _file(output, version_stderr)}
        if rc != 0 or execution.get("timed_out") or execution.get("zero_survivor") is not True:
            raise ValueError("hbrt4_disas_version_failed")
        command = [str(tool.resolve()), "--json", str(hbm.resolve())]
        rc, stdout, stderr, execution = _run(command)
        stdout_path, stderr_path = output / "hbrt4-disas.stdout.json", output / "hbrt4-disas.stderr.txt"
        stdout_path.write_text(stdout, encoding="utf-8"); stderr_path.write_text(stderr, encoding="utf-8")
        receipt["disas"] = {"command": command, "returncode": rc, "execution": execution, "stdout": _file(output, stdout_path), "stderr": _file(output, stderr_path)}
        if rc != 0 or execution.get("timed_out") or execution.get("zero_survivor") is not True:
            raise ValueError("hbrt4_disas_failed")
        validate_hbrt4_disas(json.loads(stdout))
        receipt["status"] = STATUS
    except Exception as exc:
        receipt["blockers"].append(f"hbrt4_disas_capture_failed:{type(exc).__name__}")
    receipt.update({"receipt_path": str(receipt_path.resolve()), "producer_script_path": str(Path(__file__).resolve()), "producer_script_sha256": sha256_file(Path(__file__))})
    atomic_json(receipt_path, receipt)
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hbm", required=True, type=Path)
    parser.add_argument("--hbrt4-disas", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = capture(args.hbm, args.hbrt4_disas, args.output)
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == STATUS else 2


if __name__ == "__main__":
    raise SystemExit(main())
