#!/usr/bin/env python3
"""Supervise the non-formal preprocessing oracle as one bounded PGID."""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

from hbm_evidence_common import atomic_json, fresh_directory, load_object, memory_watchdog_evidence, normal_file, require_completed_memory_watchdog, run_owned_process, sha256_file
from validate_dosod_single_frame_preprocessing_oracle import RECEIPT_ID as CHILD_RECEIPT_ID, STATUS, _validate_raw

ROOT = Path(__file__).resolve().parents[1]
COLLECTOR = ROOT / "scripts" / "collect_dosod_single_frame_preprocessing_oracle.py"
WATCHDOG = ROOT / "scripts" / "formal_memory_watchdog.sh"
RECEIPT_ID = "tzcup_dosod_single_frame_preprocessing_oracle_supervision_receipt_v1"
WALL_DEADLINE_SECONDS = 180


def _binding(root: Path, path: Path, label: str) -> dict[str, Any]:
    normal_file(path, label)
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f"{label}_path_escape")
    return {"relative_path": path.relative_to(root).as_posix(), "sha256": sha256_file(path), "byte_size": path.stat().st_size}


def supervise(*, candidate_receipt: Path, official_capture_receipt: Path, onnx_model: Path, hrt: Path, output: Path) -> dict[str, Any]:
    for path, label in ((candidate_receipt, "candidate_receipt"), (official_capture_receipt, "official_capture_receipt"),
                        (onnx_model, "onnx_model"), (hrt, "hrt_model_exec"), (COLLECTOR, "oracle_collector"),
                        (WATCHDOG, "memory_watchdog")):
        normal_file(path, label)
    fresh_directory(output, "oracle_supervision_output")
    child_root = output / "collector"
    receipt: dict[str, Any] = {"schema_version": 1, "receipt_id": RECEIPT_ID, "status": "BLOCKED",
        "formal_compile": False, "board_acceptance": False, "wall_deadline_seconds": WALL_DEADLINE_SECONDS,
        "child_oracle": None, "collector_execution": None, "memory_watchdog": None, "collector_stdout": None,
        "collector_stderr": None, "blockers": [], "started_epoch_ns": time.time_ns(), "ended_epoch_ns": None}
    try:
        command = [sys.executable, str(COLLECTOR), "--candidate-receipt", str(candidate_receipt),
                   "--official-capture-receipt", str(official_capture_receipt), "--onnx-model", str(onnx_model),
                   "--hrt-model-exec", str(hrt), "--output", str(child_root)]
        code, stdout, stderr, execution = run_owned_process(
            command, timeout_seconds=WALL_DEADLINE_SECONDS,
            memory_watchdog={"script": WATCHDOG, "json": output / "memory_watchdog.json", "log": output / "memory_watchdog.log"},
            environment={**os.environ, "TZCUP_ORACLE_OUTER_SUPERVISED": "1", "FORMAL_NATIVE_LINUX_RUNTIME": "1",
                         "FORMAL_MEMORY_MIN_AVAILABLE_KIB": "3145728", "FORMAL_MEMORY_MAX_SWAP_USED_KIB": "1048576",
                         "FORMAL_MEMORY_MAX_GROUP_RSS_KIB": "9437184"})
        (output / "collector.stdout.txt").write_text(stdout, encoding="utf-8")
        (output / "collector.stderr.txt").write_text(stderr, encoding="utf-8")
        receipt.update({"collector_execution": {key: execution.get(key) for key in ("pgid", "deadline_seconds", "term_grace_seconds", "timed_out", "term_sent", "kill_sent", "zero_survivor", "elapsed_seconds")},
                        "memory_watchdog": memory_watchdog_evidence(output, execution),
                        "collector_stdout": _binding(output, output / "collector.stdout.txt", "collector_stdout"),
                        "collector_stderr": _binding(output, output / "collector.stderr.txt", "collector_stderr")})
        child = child_root / "dosod_single_frame_preprocessing_oracle_receipt.json"
        require_completed_memory_watchdog(output, receipt["memory_watchdog"], "oracle_outer_watchdog")
        if code != 0 or execution.get("timed_out") or execution.get("zero_survivor") is not True:
            raise ValueError("oracle_outer_execution_failed")
        normal_file(child, "oracle_inner_receipt")
        child_value = load_object(child)
        if child_value.get("receipt_id") != CHILD_RECEIPT_ID or child_value.get("status") != STATUS:
            raise ValueError("oracle_inner_receipt_not_verified")
        _validate_raw(child, outer_supervised=True)
        receipt["child_oracle"] = _binding(output, child, "oracle_inner_receipt")
        receipt["status"] = STATUS
    except Exception as exc:
        receipt["blockers"].append(f"oracle_supervision_failed:{type(exc).__name__}:{exc}")
    finally:
        receipt["ended_epoch_ns"] = time.time_ns()
        target = output / "dosod_single_frame_preprocessing_oracle_supervision_receipt.json"
        receipt.update({"receipt_path": str(target.resolve()), "producer_script_path": str(Path(__file__).resolve()),
                        "producer_script_sha256": sha256_file(Path(__file__).resolve())})
        atomic_json(target, receipt)
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate-receipt", required=True, type=Path); parser.add_argument("--official-capture-receipt", required=True, type=Path)
    parser.add_argument("--onnx-model", required=True, type=Path); parser.add_argument("--hrt-model-exec", required=True, type=Path); parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = supervise(candidate_receipt=args.candidate_receipt, official_capture_receipt=args.official_capture_receipt,
                       onnx_model=args.onnx_model, hrt=args.hrt_model_exec, output=args.output)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["status"] == STATUS else 2


if __name__ == "__main__":
    raise SystemExit(main())
