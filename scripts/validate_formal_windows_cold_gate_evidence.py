#!/usr/bin/env python3
"""Validate fresh Windows cold-start evidence before a WSL final build."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any


MIN_COLD_COMMIT_AVAILABLE_BYTES = 25 * 1024**3 // 2
MAX_COLD_DOCKER_PRIVATE_BYTES = 4 * 1024**3


class EvidenceError(RuntimeError):
    pass


def _object(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise EvidenceError(f"{label} must be an object")
    return value


def _uint(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise EvidenceError(f"{label} must be an unsigned integer")
    return value


def validate_evidence(
    path: Path,
    *,
    now_ns: int | None = None,
    max_age_s: int = 300,
    enforce_freshness: bool = True,
) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise EvidenceError(f"cold-start evidence must be a regular file: {path}")
    if max_age_s <= 0:
        raise EvidenceError("max_age_s must be positive")
    try:
        payload = _object(json.loads(path.read_text(encoding="utf-8")), "evidence")
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise EvidenceError(f"cannot read cold-start evidence: {exc}") from exc

    if payload.get("report_id") != "tzcup_formal_windows_memory_start_gate_v1":
        raise EvidenceError("unexpected cold-start evidence report_id")
    if payload.get("status") != "FORMAL_WINDOWS_MEMORY_START_GATE_PASSED":
        raise EvidenceError("cold-start evidence is not a passed gate")
    if payload.get("passed") is not True:
        raise EvidenceError("cold-start evidence passed must be true")
    if payload.get("require_wsl_stopped") is not True:
        raise EvidenceError("cold-start evidence did not require WSL to be stopped")
    if payload.get("require_wsl_running") is not False:
        raise EvidenceError("cold-start evidence incorrectly required WSL to be running")

    thresholds = _object(payload.get("thresholds_bytes"), "thresholds_bytes")
    min_commit = _uint(thresholds.get("min_commit_available"), "min_commit_available")
    max_docker = _uint(thresholds.get("max_docker_private"), "max_docker_private")
    if min_commit < MIN_COLD_COMMIT_AVAILABLE_BYTES:
        raise EvidenceError("cold-start commit threshold is below 12.5 GiB")
    if max_docker > MAX_COLD_DOCKER_PRIVATE_BYTES:
        raise EvidenceError("cold-start Docker ceiling exceeds 4 GiB")

    checks = _object(payload.get("checks"), "checks")
    expected_checks = {
        "windows_commit_available_at_least_configured_minimum",
        "docker_private_at_most_configured_maximum",
        "wsl_vm_stopped_when_required",
        "wsl_vm_running_when_required",
    }
    if set(checks) != expected_checks or any(checks[key] is not True for key in checks):
        raise EvidenceError("cold-start checks are incomplete or not all true")

    sample = _object(payload.get("sample"), "sample")
    commit_available = _uint(sample.get("commit_available_bytes"), "commit_available_bytes")
    docker_private = _uint(sample.get("docker_private_bytes"), "docker_private_bytes")
    vmmem_private = _uint(sample.get("vmmem_wsl_private_bytes"), "vmmem_wsl_private_bytes")
    sample_epoch_ns = _uint(sample.get("epoch_ns"), "sample epoch_ns")
    recorded_epoch_ns = _uint(payload.get("recorded_epoch_ns"), "recorded_epoch_ns")
    if commit_available < min_commit:
        raise EvidenceError("sample commit availability is below its recorded threshold")
    if docker_private > max_docker:
        raise EvidenceError("sample Docker private bytes exceed its recorded ceiling")
    if vmmem_private != 0:
        raise EvidenceError("sample was not taken with WSL stopped")
    if sample_epoch_ns > recorded_epoch_ns or recorded_epoch_ns - sample_epoch_ns > 10_000_000_000:
        raise EvidenceError("sample and recording timestamps are inconsistent")

    if enforce_freshness:
        current_ns = time.time_ns() if now_ns is None else now_ns
        if recorded_epoch_ns > current_ns + 5_000_000_000:
            raise EvidenceError("cold-start evidence timestamp is in the future")
        if current_ns - recorded_epoch_ns > max_age_s * 1_000_000_000:
            raise EvidenceError("cold-start evidence is stale")
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", required=True, type=Path)
    parser.add_argument("--max-age-s", type=int, default=300)
    args = parser.parse_args()
    try:
        validate_evidence(args.evidence, max_age_s=args.max_age_s)
    except EvidenceError as exc:
        print(f"FORMAL_WINDOWS_COLD_GATE_EVIDENCE_REFUSED: {exc}")
        return 86
    print("FORMAL_WINDOWS_COLD_GATE_EVIDENCE_PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
