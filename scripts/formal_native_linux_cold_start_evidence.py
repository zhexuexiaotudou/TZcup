#!/usr/bin/env python3
"""Bind the audited native-Linux build-start sample for final runtime closure."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import time
from pathlib import Path
from typing import Any


SOURCE_NAME = "formal_final_build_linux_memory_preflight.json"
BOUND_NAME = "formal_windows_cold_start_evidence.json"  # legacy closure path
SOURCE_REPORT_ID = "tzcup_formal_final_build_linux_memory_start_gate_v1"
BOUND_REPORT_ID = "tzcup_formal_native_linux_cold_start_gate_v1"
MIN_MEMORY_KIB = 4 * 1024 * 1024
MAX_SWAP_KIB = 1024 * 1024


class NativeEvidenceError(RuntimeError):
    pass


def _object(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise NativeEvidenceError(f"{label} must be an object")
    return value


def _uint(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise NativeEvidenceError(f"{label} must be an unsigned integer")
    return value


def _read(path: Path) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise NativeEvidenceError(f"evidence must be a regular file: {path}")
    try:
        return _object(json.loads(path.read_text(encoding="utf-8")), "evidence")
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise NativeEvidenceError(f"cannot read evidence: {exc}") from exc


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_build_start(path: Path) -> dict[str, Any]:
    payload = _read(path)
    if payload.get("report_id") != SOURCE_REPORT_ID:
        raise NativeEvidenceError("unexpected Linux build-start report_id")
    if payload.get("status") != "FORMAL_FINAL_BUILD_LINUX_MEMORY_START_PASSED":
        raise NativeEvidenceError("Linux build-start gate did not pass")
    if payload.get("passed") is not True:
        raise NativeEvidenceError("Linux build-start passed must be true")
    thresholds = _object(payload.get("thresholds_kib"), "thresholds_kib")
    observed = _object(payload.get("observed_kib"), "observed_kib")
    checks = _object(payload.get("checks"), "checks")
    signals = _object(payload.get("signals"), "signals")
    minimum = _uint(thresholds.get("min_mem_available"), "min_mem_available")
    maximum_swap = _uint(thresholds.get("max_swap_used"), "max_swap_used")
    available = _uint(observed.get("mem_available"), "mem_available")
    swap_total = _uint(observed.get("swap_total"), "swap_total")
    swap_free = _uint(observed.get("swap_free"), "swap_free")
    swap_used = _uint(observed.get("swap_used"), "swap_used")
    if minimum < MIN_MEMORY_KIB or maximum_swap > MAX_SWAP_KIB:
        raise NativeEvidenceError("Linux build-start thresholds are weaker than formal limits")
    if swap_free > swap_total or swap_used != swap_total - swap_free:
        raise NativeEvidenceError("Linux build-start swap sample is inconsistent")
    if available < minimum or swap_used > maximum_swap:
        raise NativeEvidenceError("Linux build-start sample violates its thresholds")
    if checks != {
        "mem_available_at_least_configured_minimum": True,
        "swap_used_at_most_configured_maximum": True,
    }:
        raise NativeEvidenceError("Linux build-start checks are incomplete")
    if signals != {"exact_pgid_only": True, "docker_signalled_or_stopped": False}:
        raise NativeEvidenceError("Linux build-start signal boundary is invalid")
    _uint(payload.get("sample_epoch_ns"), "sample_epoch_ns")
    return payload


def validate_bound(path: Path, runtime_ws: Path) -> dict[str, Any]:
    payload = _read(path)
    if payload.get("report_id") != BOUND_REPORT_ID:
        raise NativeEvidenceError("unexpected native-Linux bound report_id")
    if payload.get("status") != "FORMAL_NATIVE_LINUX_COLD_START_GATE_PASSED":
        raise NativeEvidenceError("native-Linux cold-start gate did not pass")
    if payload.get("passed") is not True or payload.get("runtime_mode") != "native_linux_not_wsl":
        raise NativeEvidenceError("native-Linux runtime identity is invalid")
    checks = _object(payload.get("checks"), "checks")
    if checks != {
        "explicit_native_linux_opt_in": True,
        "linux_build_start_gate_passed": True,
        "native_linux_not_windows": True,
        "native_linux_not_wsl": True,
    }:
        raise NativeEvidenceError("native-Linux binding checks are incomplete")
    source = runtime_ws / SOURCE_NAME
    source_payload = validate_build_start(source)
    bound_source = _object(payload.get("source"), "source")
    if bound_source != {
        "path": SOURCE_NAME,
        "sha256": _sha256(source),
        "report_id": SOURCE_REPORT_ID,
        "sample_epoch_ns": source_payload["sample_epoch_ns"],
    }:
        raise NativeEvidenceError("native-Linux binding source identity drifted")
    recorded = _uint(payload.get("recorded_epoch_ns"), "recorded_epoch_ns")
    if recorded < source_payload["sample_epoch_ns"]:
        raise NativeEvidenceError("native-Linux binding predates its build-start sample")
    kernel = payload.get("kernel_osrelease")
    if not isinstance(kernel, str) or not kernel or "microsoft" in kernel.lower():
        raise NativeEvidenceError("native-Linux kernel identity is invalid")
    return payload


def bind(runtime_ws: Path) -> Path:
    if os.name != "posix" or os.environ.get("FORMAL_NATIVE_LINUX_RUNTIME") != "1":
        raise NativeEvidenceError("binding requires explicit native-Linux opt-in")
    kernel = platform.release()
    if "microsoft" in kernel.lower():
        raise NativeEvidenceError("WSL cannot use the native-Linux cold-start binding")
    runtime_ws = runtime_ws.resolve()
    source = runtime_ws / SOURCE_NAME
    source_payload = validate_build_start(source)
    target = runtime_ws / BOUND_NAME
    if target.exists() or target.is_symlink():
        raise NativeEvidenceError(f"refusing existing bound evidence: {target}")
    payload = {
        "report_id": BOUND_REPORT_ID,
        "status": "FORMAL_NATIVE_LINUX_COLD_START_GATE_PASSED",
        "passed": True,
        "runtime_mode": "native_linux_not_wsl",
        "recorded_epoch_ns": time.time_ns(),
        "kernel_osrelease": kernel,
        "source": {
            "path": SOURCE_NAME,
            "sha256": _sha256(source),
            "report_id": SOURCE_REPORT_ID,
            "sample_epoch_ns": source_payload["sample_epoch_ns"],
        },
        "checks": {
            "explicit_native_linux_opt_in": True,
            "linux_build_start_gate_passed": True,
            "native_linux_not_windows": True,
            "native_linux_not_wsl": True,
        },
    }
    pending = target.with_name(f"{target.name}.pending.{os.getpid()}")
    pending.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    pending.chmod(0o444)
    pending.replace(target)
    validate_bound(target, runtime_ws)
    return target


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-ws", type=Path, required=True)
    args = parser.parse_args()
    try:
        path = bind(args.runtime_ws)
    except NativeEvidenceError as exc:
        print(f"FORMAL_NATIVE_LINUX_COLD_START_EVIDENCE_REFUSED: {exc}")
        return 86
    print(f"FORMAL_NATIVE_LINUX_COLD_START_EVIDENCE_BOUND: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
