#!/usr/bin/env python3
"""Bind AUTO-05 G4 capture to the same formal runtime gate as Gazebo gates."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path

from formal_runtime_gate_binding import RuntimeGateError, build_binding


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(repository: Path, *args: str) -> str:
    return subprocess.check_output(["git", "-C", str(repository), *args], text=True).strip()


def repository_relative(path: Path, repository: Path) -> str:
    """Serialize a portable identity, never a machine-specific data path."""
    try:
        return path.resolve().relative_to(repository).as_posix()
    except ValueError as exc:
        raise ValueError(f"G4 path escapes repository identity: {path}") from exc


def portable_evidence(value: object) -> object:
    """Keep hashes/statuses while preventing source-host paths becoming identity."""
    if isinstance(value, dict):
        return {key: portable_evidence(item) for key, item in value.items()}
    if isinstance(value, list):
        return [portable_evidence(item) for item in value]
    if isinstance(value, str) and (value.startswith("/") or re.match(r"^[A-Za-z]:[\\/]", value)):
        return "<host-local-path-redacted>"
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--install-root", type=Path, required=True)
    parser.add_argument("--closure-manifest", type=Path, required=True)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--ros-domain-id", type=int, required=True)
    parser.add_argument("--gz-partition", required=True)
    parser.add_argument("--gazebo-lock", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    repository = args.repository_root.resolve()
    work_root = repository / ".work" / "auto05-g4"
    if not all(str(path.resolve()).startswith(str(work_root.resolve()) + "/") for path in (args.data_root, args.output)):
        parser.error("G4 data root and output must be below TZcup/.work/auto05-g4")
    if args.contract.resolve() != repository / "starter_ws/src/sanitation_learning/config/auto05_g4_screening.yaml":
        parser.error("G4 requires its checked-in frozen contract")
    if args.output.exists():
        parser.error("refusing to overwrite a retained G4 runtime binding")
    if not args.gz_partition.startswith("tzcup_auto05_g4_"):
        parser.error("G4 partition must use the tzcup_auto05_g4_ prefix")
    if not args.gazebo_lock.is_absolute():
        parser.error("G4 formal Gazebo lock path must be absolute")
    if not ((0 <= args.ros_domain_id <= 101) or (215 <= args.ros_domain_id <= 231)):
        parser.error("G4 ROS domain is outside formal-safe ranges")
    try:
        binding = build_binding(
            repository_root=repository, install_root=args.install_root,
            closure_manifest=args.closure_manifest, session_path=args.session,
            snapshot_path=args.snapshot,
        )
    except RuntimeGateError as exc:
        parser.error(str(exc))
    if git(repository, "status", "--porcelain"):
        parser.error("G4 runtime gate requires a clean source checkout")
    payload = {
        "schema_version": 1,
        "status": "AUTO05_G4_RUNTIME_GATE_BOUND",
        "formal_runtime_gate": portable_evidence(binding),
        "git": {"head": git(repository, "rev-parse", "HEAD"), "tree": git(repository, "rev-parse", "HEAD^{tree}")},
        "contract": {
            "repository_relative": repository_relative(args.contract, repository),
            "sha256": digest(args.contract),
        },
        "capture": {
            "data_root_repository_relative": repository_relative(args.data_root, repository),
            "ros_domain_id": args.ros_domain_id,
            "gz_partition": args.gz_partition,
            "single_gazebo_lock": "source-local-formal-gazebo-lock",
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(payload, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
