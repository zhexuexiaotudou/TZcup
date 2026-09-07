#!/usr/bin/env python3
"""Fail closed A19 product-soak contract; no current-main producer may PASS."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import stat
import subprocess
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config/high_fidelity_vehicle/formal_a19_reliability_fault_contract.json"
BLOCKED = "A19_TWO_HOUR_RELIABILITY_FAULT_BLOCKED"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


def _hash(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _identity(path: Path) -> tuple[int, int, int, int, int]:
    row = path.lstat()
    return row.st_dev, row.st_ino, row.st_size, row.st_mtime_ns, row.st_mode


def _safe_file(path: Path, root: Path) -> None:
    root, path = root.absolute(), path.absolute()
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"path escapes root: {path}") from exc
    current = path
    while True:
        mode = current.lstat().st_mode
        if stat.S_ISLNK(mode):
            raise ValueError(f"symlink is forbidden: {current}")
        if current == root:
            break
        current = current.parent
    if not stat.S_ISREG(path.lstat().st_mode):
        raise ValueError(f"not a regular file: {path}")


def _stable_json(path: Path, root: Path) -> tuple[dict[str, Any], str]:
    """Reject symlink paths and a file replaced while being read."""
    _safe_file(path, root)
    before = _identity(path)
    data = path.read_bytes()
    after = _identity(path)
    if before != after:
        raise ValueError(f"file changed while read: {path}")
    try:
        payload = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid JSON: {path}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root is not an object: {path}")
    return payload, _hash(data)


def _stable_bytes(path: Path, root: Path) -> bytes:
    _safe_file(path, root)
    before = _identity(path); data = path.read_bytes(); after = _identity(path)
    if before != after:
        raise ValueError(f"file changed while read: {path}")
    return data


def _require_exact(actual: Any, expected: list[str], label: str, blockers: list[str]) -> None:
    if not isinstance(actual, list) or actual != expected or len(set(actual)) != len(actual):
        blockers.append(f"{label} must be the exact ordered, unique contract list")


def _event_state(records: Any, state: str) -> bool:
    return isinstance(records, list) and any(
        isinstance(item, dict)
        and item.get("state") == state
        and type(item.get("wall_clock_epoch_ns")) is int
        for item in records
    )


def _candidate_checks(report: dict[str, Any], contract: dict[str, Any], now_ns: int, session_started_ns: Any, evidence_root: Path) -> list[str]:
    blockers: list[str] = []
    _require_exact(report.get("profiles"), contract["required_profiles"], "profiles", blockers)
    faults = report.get("faults")
    if not isinstance(faults, list):
        blockers.append("fault evidence is missing")
        return blockers
    ids = [row.get("fault_id") for row in faults if isinstance(row, dict)]
    _require_exact(ids, contract["required_faults"], "fault ids", blockers)
    metrics = report.get("stability_metrics")
    gates = contract["stability_gates"]
    if not isinstance(metrics, dict):
        blockers.append("stability metrics are missing")
    else:
        exact_zero = ("crash_count", "deadlock_count", "queue_growth_count", "unexpected_model_reload_count", "persistent_tf_failure_count", "unrecoverable_watchdog_event_count", "unsafe_cleaning_action_count")
        for key in exact_zero:
            if type(metrics.get(key)) is not int or metrics[key] != gates[key]:
                blockers.append(f"stability gate failed: {key}")
        for key, ceiling in (("memory_growth_ratio", gates["maximum_memory_growth_ratio"]), ("localization_xy_rmse_m", gates["maximum_localization_xy_rmse_m"]), ("localization_xy_p95_m", gates["maximum_localization_xy_p95_m"]), ("estop_brake_latency_s", gates["maximum_estop_brake_latency_s"])):
            value = metrics.get(key)
            if type(value) not in (int, float) or not math.isfinite(value) or value > ceiling:
                blockers.append(f"stability gate failed: {key}")
        if metrics.get("pipeline_components") != contract["required_pipeline_components"]:
            blockers.append("full product pipeline components are not evidenced")
    collection = report.get("wall_clock_collection")
    if not isinstance(collection, dict):
        blockers.append("wall-clock collection is missing")
    else:
        start, end = collection.get("started_epoch_ns"), collection.get("ended_epoch_ns")
        if type(start) is not int or type(end) is not int or start > end or end - start < gates["minimum_duration_s"] * 1_000_000_000:
            blockers.append("wall-clock collection is not a continuous two-hour window")
        if type(start) is int and (start > now_ns or end > now_ns):
            blockers.append("wall-clock collection is in the future")
        if type(start) is not int or type(session_started_ns) is not int or start < session_started_ns:
            blockers.append("wall-clock collection predates the current session")
    evidence = report.get("raw_evidence")
    if not isinstance(evidence, dict) or set(evidence) != set(contract["required_raw_evidence"]):
        blockers.append("raw evidence does not contain the required producer receipts")
    elif any(
        not isinstance(row, dict)
        or not isinstance(row.get("path"), str)
        or not isinstance(row.get("byte_size"), int)
        or not isinstance(row.get("sha256"), str)
        or not SHA256_RE.fullmatch(row["sha256"])
        for row in evidence.values()
    ):
        blockers.append("raw evidence lacks regular-file SHA-256 and byte-size bindings")
    else:
        for name, row in evidence.items():
            try:
                data = _stable_bytes(evidence_root / row["path"], evidence_root)
                if len(data) != row["byte_size"] or _hash(data) != row["sha256"]:
                    blockers.append(f"raw evidence {name} hash or byte-size mismatch")
            except (OSError, ValueError) as exc:
                blockers.append(f"raw evidence {name}: {exc}")
    health = report.get("runtime_health")
    if not isinstance(health, dict) or any(health.get(name) is not True for name in contract["required_runtime_health"]):
        blockers.append("full Safety/Nav2/Watchdog health evidence is missing")
    for row in faults:
        if not isinstance(row, dict):
            continue
        if not isinstance(row.get("injection_parameters"), dict) or not row["injection_parameters"]:
            blockers.append(f"fault {row.get('fault_id')} lacks injection parameters")
        if not _event_state(row.get("safety_state_events"), contract["required_fault_evidence"]["required_safety_state"]):
            blockers.append(f"fault {row.get('fault_id')} lacks timestamped STOPPED state")
        if not _event_state(row.get("recovery_state_events"), contract["required_fault_evidence"]["required_recovery_state"]):
            blockers.append(f"fault {row.get('fault_id')} lacks timestamped RECOVERED state")
        if not _event_state(row.get("perception_health_events"), "DEGRADED") and not _event_state(row.get("perception_health_events"), "ERROR"):
            blockers.append(f"fault {row.get('fault_id')} lacks a degraded/error health event")
        if row.get("safety_nav_operational") is not True:
            blockers.append(f"fault {row.get('fault_id')} lacks Safety/Nav2 operational evidence")
        if row.get("unsafe_pending_clean_outcome") not in contract["required_fault_evidence"]["unsafe_pending_clean_outcomes"]:
            blockers.append(f"fault {row.get('fault_id')} lacks cancel/defer evidence")
    return blockers


def validate(report_path: Path, snapshot_path: Path, session_path: Path, closure_path: Path, evidence_root: Path, contract_path: Path = DEFAULT_CONTRACT, repository_root: Path = ROOT) -> dict[str, Any]:
    """Return only BLOCKED; an absent canonical producer is a hard A19 boundary."""
    blockers: list[str] = []
    loaded: dict[str, tuple[dict[str, Any], str]] = {}
    for name, path, root in (
        ("contract", contract_path, repository_root), ("report", report_path, evidence_root),
        ("snapshot", snapshot_path, evidence_root), ("session", session_path, evidence_root),
        ("closure", closure_path, evidence_root),
    ):
        try:
            loaded[name] = _stable_json(path, root)
        except (OSError, ValueError) as exc:
            blockers.append(f"{name}: {exc}")
    contract = loaded.get("contract", ({}, ""))[0]
    if contract.get("schema_version") != 2 or contract.get("current_state") != "BLOCKED_MISSING_CANONICAL_A19_RUNTIME_PRODUCER":
        blockers.append("A19 contract identity/state is invalid")
    producer = contract.get("canonical_producer") if isinstance(contract, dict) else None
    if not isinstance(producer, dict) or producer.get("available_on_current_main") is not False:
        blockers.append("canonical producer availability must be explicitly false until implemented")
    else:
        blockers.append(str(producer.get("blocker", "canonical A19 producer is missing")))
    if "report" in loaded and isinstance(contract.get("required_faults"), list):
        blockers.extend(_candidate_checks(loaded["report"][0], contract, time.time_ns(), loaded.get("session", ({}, ""))[0].get("started_epoch_ns"), evidence_root))
    if {"report", "snapshot", "session", "closure"} <= set(loaded):
        try:
            commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=repository_root, text=True, capture_output=True, timeout=5, check=True).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            blockers.append("current repository commit cannot be resolved")
        else:
            expected = {"repository_commit": commit, "snapshot_sha256": loaded["snapshot"][1], "session_sha256": loaded["session"][1], "runtime_closure_sha256": loaded["closure"][1]}
            if loaded["report"][0].get("current_bindings") != expected:
                blockers.append("candidate lacks same-commit snapshot/session/runtime-closure bindings")
    # Re-read every accepted input before reporting: a replacement between validation
    # and output is a failed validation, never a pass.
    for name, path, root in (("contract", contract_path, repository_root), ("report", report_path, evidence_root), ("snapshot", snapshot_path, evidence_root), ("session", session_path, evidence_root), ("closure", closure_path, evidence_root)):
        if name not in loaded:
            continue
        try:
            _, digest = _stable_json(path, root)
            if digest != loaded[name][1]:
                blockers.append(f"{name}: changed before final reread")
        except (OSError, ValueError) as exc:
            blockers.append(f"{name}: final reread failed: {exc}")
    return {"schema_version": 2, "report_id": "tzcup_formal_a19_reliability_fault_validation_v2", "status": BLOCKED, "passed": False, "contract_id": contract.get("contract_id"), "blockers": sorted(set(blockers)), "input_sha256": {name: digest for name, (_, digest) in loaded.items()}}


def _safe_output(path: Path, root: Path) -> None:
    if path.exists() or path.is_symlink():
        raise ValueError(f"refusing to overwrite retained output: {path}")
    # The output parent itself must be inside a non-symlink evidence root.
    root = root.absolute()
    path.absolute().parent.relative_to(root)
    current = path.absolute().parent
    while True:
        if stat.S_ISLNK(current.lstat().st_mode):
            raise ValueError(f"symlink is forbidden: {current}")
        if current == root:
            break
        current = current.parent


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True); parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--acceptance-session", type=Path, required=True); parser.add_argument("--runtime-closure", type=Path, required=True)
    parser.add_argument("--evidence-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT); parser.add_argument("--repository-root", type=Path, default=ROOT)
    args = parser.parse_args()
    try:
        _safe_output(args.output, args.evidence_root)
        result = validate(args.report, args.snapshot, args.acceptance_session, args.runtime_closure, args.evidence_root, args.contract, args.repository_root)
        temporary = args.output.with_suffix(args.output.suffix + f".pending.{os.getpid()}")
        temporary.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        temporary.replace(args.output)
        persisted, _ = _stable_json(args.output, args.evidence_root)
        if persisted != result:
            raise ValueError("output changed before final reread")
    except (OSError, ValueError) as exc:
        print(json.dumps({"status": "INVALID", "error": str(exc)})); return 2
    print(json.dumps(result, indent=2, sort_keys=True)); return 4


if __name__ == "__main__":
    raise SystemExit(main())
