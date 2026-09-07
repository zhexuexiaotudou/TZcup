#!/usr/bin/env python3
"""Fail closed A19 product-soak contract; no current-main producer may PASS."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import tempfile
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config/high_fidelity_vehicle/formal_a19_reliability_fault_contract.json"
BLOCKED = "A19_TWO_HOUR_RELIABILITY_FAULT_BLOCKED"
AUTHORITATIVE_EVIDENCE_PARENT = ROOT / "artifacts"


def _hash(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _identity(path: Path) -> tuple[int, int, int, int, int]:
    row = path.lstat()
    return row.st_dev, row.st_ino, row.st_size, row.st_mtime_ns, row.st_mode


def _normalized(path: Path) -> Path:
    if ".." in Path(str(path)).parts:
        raise ValueError(f"parent traversal is forbidden: {path}")
    return Path(os.path.normpath(os.path.abspath(str(path))))


def _no_symlink_ancestor(path: Path) -> None:
    current = path
    while True:
        if stat.S_ISLNK(current.lstat().st_mode):
            raise ValueError(f"symlink is forbidden: {current}")
        if current.parent == current:
            return
        current = current.parent


def _authorized_evidence_root(evidence_root: Path, evidence_parent: Path) -> Path:
    parent, root = _normalized(evidence_parent), _normalized(evidence_root)
    if not parent.is_dir() or not root.is_dir():
        raise ValueError("evidence parent/root must be existing directories")
    try:
        root.relative_to(parent)
    except ValueError as exc:
        raise ValueError(f"evidence root escapes authorized parent: {root}") from exc
    _no_symlink_ancestor(root)
    try:
        root.resolve(strict=True).relative_to(parent.resolve(strict=True))
    except ValueError as exc:
        raise ValueError(f"resolved evidence root escapes authorized parent: {root}") from exc
    return root


def _safe_file(path: Path, root: Path) -> None:
    root, path = _normalized(root), _normalized(path)
    if not root.is_dir():
        raise ValueError(f"root is not a directory: {root}")
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"path escapes root: {path}") from exc
    _no_symlink_ancestor(path)
    resolved_root, resolved_path = root.resolve(strict=True), path.resolve(strict=True)
    try:
        resolved_path.relative_to(resolved_root)
    except ValueError as exc:
        raise ValueError(f"resolved path escapes root: {path}") from exc
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


def validate(report_path: Path, snapshot_path: Path, session_path: Path, closure_path: Path, evidence_root: Path, contract_path: Path = DEFAULT_CONTRACT, repository_root: Path = ROOT, evidence_parent: Path = AUTHORITATIVE_EVIDENCE_PARENT) -> dict[str, Any]:
    """Return only BLOCKED; an absent canonical producer is a hard A19 boundary."""
    blockers: list[str] = []
    loaded: dict[str, tuple[dict[str, Any], str]] = {}
    try:
        evidence_root = _authorized_evidence_root(evidence_root, evidence_parent)
        evidence_identity = _identity(evidence_root)
    except (OSError, ValueError) as exc:
        return {"schema_version": 2, "report_id": "tzcup_formal_a19_reliability_fault_validation_v2", "status": BLOCKED, "passed": False, "contract_id": None, "blockers": [f"evidence root: {exc}"], "input_sha256": {}}
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
    blockers.append("future canonical producer must parse and cross-validate time-series, command/exit, zero-survivor, fault-state ordering, and same-commit bindings; current main has no receipt schema")
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
    if _identity(evidence_root) != evidence_identity:
        blockers.append("evidence root identity changed before final reread")
    return {"schema_version": 2, "report_id": "tzcup_formal_a19_reliability_fault_validation_v2", "status": BLOCKED, "passed": False, "contract_id": contract.get("contract_id"), "blockers": sorted(set(blockers)), "input_sha256": {name: digest for name, (_, digest) in loaded.items()}}


def _safe_output(path: Path, root: Path) -> None:
    root, path = _normalized(root), _normalized(path)
    if not root.is_dir():
        raise ValueError(f"root is not a directory: {root}")
    try:
        path.parent.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"output escapes root: {path}") from exc
    _no_symlink_ancestor(path.parent)
    try:
        path.parent.resolve(strict=True).relative_to(root.resolve(strict=True))
    except ValueError as exc:
        raise ValueError(f"resolved output escapes root: {path}") from exc
    if path.exists() or path.is_symlink():
        raise ValueError(f"refusing to overwrite retained output: {path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True); parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--acceptance-session", type=Path, required=True); parser.add_argument("--runtime-closure", type=Path, required=True)
    parser.add_argument("--evidence-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT); parser.add_argument("--repository-root", type=Path, default=ROOT)
    args = parser.parse_args()
    try:
        evidence_root = _authorized_evidence_root(args.evidence_root, AUTHORITATIVE_EVIDENCE_PARENT)
        evidence_identity = _identity(evidence_root)
        _safe_output(args.output, evidence_root)
        result = validate(args.report, args.snapshot, args.acceptance_session, args.runtime_closure, evidence_root, args.contract, args.repository_root)
        descriptor, temporary_name = tempfile.mkstemp(prefix=".a19-", suffix=".pending", dir=args.output.parent)
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(json.dumps(result, indent=2, sort_keys=True) + "\n")
            stream.flush(); os.fsync(stream.fileno())
        Path(temporary_name).replace(args.output)
        persisted, _ = _stable_json(args.output, evidence_root)
        if persisted != result:
            raise ValueError("output changed before final reread")
        if _identity(evidence_root) != evidence_identity:
            raise ValueError("evidence root identity changed while writing output")
    except (OSError, ValueError) as exc:
        print(json.dumps({"status": "INVALID", "error": str(exc)})); return 2
    print(json.dumps(result, indent=2, sort_keys=True)); return 4


if __name__ == "__main__":
    raise SystemExit(main())
