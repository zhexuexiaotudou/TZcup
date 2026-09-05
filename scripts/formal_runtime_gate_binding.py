#!/usr/bin/env python3
"""Fail-closed binding for direct formal-runtime acceptance runners."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import time
from pathlib import Path
from typing import Any

from formal_acceptance_session import _snapshot_identity
from formal_final_runtime_closure import verify_recorded_manifest
from generate_formal_vehicle_snapshot import SnapshotError, verify_snapshot


class RuntimeGateError(RuntimeError):
    """A direct runner is not bound to the frozen final runtime/session."""


def _read_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise RuntimeGateError(f"cannot read JSON object {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise RuntimeGateError(f"JSON root must be an object: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def build_binding(
    *,
    repository_root: Path,
    install_root: Path,
    closure_manifest: Path,
    session_path: Path,
    snapshot_path: Path,
) -> dict[str, Any]:
    repository_root = repository_root.resolve()
    canonical_snapshot = (
        repository_root / "reports/engineering/formal_vehicle_snapshot_manifest.json"
    ).resolve()
    if snapshot_path.resolve() != canonical_snapshot:
        raise RuntimeGateError("runtime gate requires the canonical vehicle snapshot")

    # A matching session record alone is not enough: both it and the frozen
    # closure could still point at a retained snapshot after checkout sources
    # changed.  The pure-Python snapshot verifier recomputes the authoritative
    # source/output inventories before any ROS or Gazebo process is admitted.
    try:
        verified_snapshot = verify_snapshot(repository_root, snapshot_path)
    except (SnapshotError, OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise RuntimeGateError(f"current vehicle snapshot verification failed: {exc}") from exc
    identity = _snapshot_identity(snapshot_path)
    if verified_snapshot.get("source_inventory_sha256") != identity[
        "source_inventory_sha256"
    ]:
        raise RuntimeGateError("current vehicle snapshot source hash is inconsistent")
    session = _read_object(session_path)
    if session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise RuntimeGateError("formal acceptance session is not RUNNING")
    if session.get("snapshot") != identity:
        raise RuntimeGateError("acceptance session does not match the vehicle snapshot")
    started_ns = session.get("started_epoch_ns")
    if not isinstance(started_ns, int) or started_ns <= 0 or started_ns > time.time_ns():
        raise RuntimeGateError("acceptance session has an invalid start time")

    closure = _read_object(closure_manifest)
    recorded_ns = closure.get("recorded_epoch_ns")
    if not isinstance(recorded_ns, int) or recorded_ns <= 0:
        raise RuntimeGateError("runtime closure has an invalid record time")
    if recorded_ns > started_ns:
        raise RuntimeGateError("acceptance session predates the frozen runtime closure")
    if snapshot_path.stat().st_mtime_ns > started_ns:
        raise RuntimeGateError("vehicle snapshot was modified after session start")
    if closure_manifest.stat().st_mtime_ns > started_ns:
        raise RuntimeGateError("runtime closure manifest was modified after session start")

    closure_binding = verify_recorded_manifest(
        closure_manifest, repository_root, install_root.resolve()
    )
    closure_binding["runtime_install_root"] = str(install_root.resolve())
    if session.get("runtime_closure_binding") != closure_binding:
        raise RuntimeGateError(
            "acceptance session does not match the frozen runtime closure"
        )
    return {
        "schema_version": 1,
        "status": "FORMAL_RUNTIME_GATE_BOUND",
        "verified_epoch_ns": time.time_ns(),
        "acceptance_session_binding": {
            "session_manifest": str(session_path.resolve()),
            "session_manifest_sha256": _sha256(session_path),
            "session_started_epoch_ns": started_ns,
            "session_status_at_gate": session["status"],
            "snapshot": identity,
            # Keep the historic session identity unchanged while allowing a
            # later readiness audit to bind formal reports to the complete
            # committed-output inventory too.
            "snapshot_output_inventory_sha256": verified_snapshot[
                "output_inventory_sha256"
            ],
            "snapshot_current_source_verified": True,
        },
        "runtime_closure_binding": closure_binding,
    }


def _atomic_write(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_suffix(path.suffix + f".pending.{os.getpid()}")
    pending.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    pending.replace(path)


def load_binding(path: Path) -> dict[str, Any]:
    binding = _read_object(path)
    if binding.get("status") != "FORMAL_RUNTIME_GATE_BOUND":
        raise RuntimeGateError("runtime binding is not in the BOUND state")
    session = binding.get("acceptance_session_binding")
    closure = binding.get("runtime_closure_binding")
    if not isinstance(session, dict) or not isinstance(closure, dict):
        raise RuntimeGateError("runtime binding is incomplete")
    if session.get("session_status_at_gate") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise RuntimeGateError("runtime binding has no running acceptance session")
    if closure.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED":
        raise RuntimeGateError("runtime binding has no verified final closure")
    runtime_install_root = closure.get("runtime_install_root")
    if not isinstance(runtime_install_root, str) or not runtime_install_root:
        raise RuntimeGateError("runtime binding has no frozen runtime install root")
    install_root_path = Path(runtime_install_root)
    if not install_root_path.is_absolute():
        raise RuntimeGateError("runtime binding install root must be absolute")
    # build_binding records the resolved install root.  Reject alternate
    # spellings (including relative traversal in a serialized binding) so
    # consumers compare one canonical frozen-runtime identity.
    if str(install_root_path.resolve()) != runtime_install_root:
        raise RuntimeGateError("runtime binding install root is not canonical")
    return binding


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--install-root", type=Path, required=True)
    parser.add_argument("--closure-manifest", type=Path, required=True)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"refusing to overwrite retained runtime binding: {args.output}")
    binding = build_binding(
        repository_root=args.repository_root,
        install_root=args.install_root,
        closure_manifest=args.closure_manifest,
        session_path=args.session,
        snapshot_path=args.snapshot,
    )
    _atomic_write(args.output, binding)
    print(json.dumps(binding, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
