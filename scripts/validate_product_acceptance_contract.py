#!/usr/bin/env python3
"""Fail closed until AUTO-15 has one real, session-bound receipt producer."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import stat
import subprocess
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config" / "high_fidelity_vehicle" / "product_acceptance_contract.json"
HEX64 = re.compile(r"^[0-9a-f]{64}$")


class ProductAcceptanceContractError(ValueError):
    """A receipt cannot be used as current product-acceptance evidence."""


def _in_root(path: Path, label: str) -> tuple[Path, Path]:
    root = ROOT.absolute()
    candidate = path.absolute()
    try:
        relative = candidate.relative_to(root)
    except ValueError as exc:
        raise ProductAcceptanceContractError(f"{label} must be in the repository") from exc
    for current in (root, *(root.joinpath(*relative.parts[:index]) for index in range(1, len(relative.parts) + 1))):
        try:
            if stat.S_ISLNK(os.lstat(current).st_mode):
                raise ProductAcceptanceContractError(f"{label} has a symbolic-link ancestor")
        except OSError as exc:
            raise ProductAcceptanceContractError(f"cannot inspect {label}: {exc}") from exc
    return root, candidate


def _sealed_regular_bytes(path: Path, label: str) -> bytes:
    """Read one non-link regular in-root input through one sealed descriptor."""
    _in_root(path, label)
    flags = os.O_RDONLY | getattr(os, "O_BINARY", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        fd = os.open(path, flags)
    except OSError as exc:
        raise ProductAcceptanceContractError(f"cannot open {label}: {exc}") from exc
    try:
        before = os.fstat(fd)
        if not stat.S_ISREG(before.st_mode):
            raise ProductAcceptanceContractError(f"{label} is not a regular file")
        chunks: list[bytes] = []
        while chunk := os.read(fd, 1024 * 1024):
            chunks.append(chunk)
        after = os.fstat(fd)
    finally:
        os.close(fd)
    try:
        named = os.lstat(path)
    except OSError as exc:
        raise ProductAcceptanceContractError(f"{label} disappeared while being read: {exc}") from exc
    identity = (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
    if identity != (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns) or identity != (named.st_dev, named.st_ino, named.st_size, named.st_mtime_ns):
        raise ProductAcceptanceContractError(f"{label} changed while being read")
    return b"".join(chunks)


def _json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(_sealed_regular_bytes(path, label).decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ProductAcceptanceContractError(f"cannot read JSON object {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ProductAcceptanceContractError(f"JSON root must be an object: {path}")
    return value


def _canonical_digest(data: bytes) -> str:
    text = data.decode("utf-8").replace("\r\n", "\n")
    return hashlib.sha256("\n".join(line.rstrip(" \t") for line in text.split("\n")).encode("utf-8")).hexdigest()


def _tracked(path: Path, label: str) -> None:
    root, candidate = _in_root(path, label)
    relative = candidate.relative_to(root).as_posix()
    command = ["git", "-C", str(root), "ls-files", "--error-unmatch", "--", relative]
    try:
        result = subprocess.run(command, capture_output=True, text=True, check=False)
    except OSError:
        result = None
    if result is None or result.returncode or result.stdout.strip() != relative:
        # A Windows-created worktree mounted in WSL has a Windows gitdir that
        # Linux git cannot parse.  Ask the matching Git implementation before
        # rejecting the fixed authority; an unavailable or non-matching result
        # still fails closed below.
        try:
            windows_root = subprocess.run(
                ["wslpath", "-w", str(root)],
                capture_output=True,
                text=True,
                check=True,
            ).stdout.strip()
            result = subprocess.run(
                ["git.exe", "-C", windows_root, "ls-files", "--error-unmatch", "--", relative],
                capture_output=True,
                text=True,
                check=False,
            )
        except (OSError, subprocess.CalledProcessError):
            result = None
    if result is None or result.returncode or result.stdout.strip() != relative:
        raise ProductAcceptanceContractError(f"{label} is not a tracked repository file")


def load_contract(path: Path = DEFAULT_CONTRACT) -> dict[str, Any]:
    return _json_object(path, "product acceptance contract")


def validate_static_contract(contract: dict[str, Any], authoritative_source: Path | None = None) -> dict[str, Any]:
    accounting = contract.get("auto15_execution_accounting")
    runtime_states = contract.get("product_runtime_states")
    spec = contract.get("authoritative_specification")
    runtime = contract.get("auto15_runtime_receipt_contract")
    if contract.get("contract_id") != "tzcup_product_acceptance_contract_v1":
        raise ProductAcceptanceContractError("unexpected product-acceptance contract id")
    if not all(isinstance(value, dict) for value in (accounting, runtime_states, spec, runtime)):
        raise ProductAcceptanceContractError("contract is missing required objects")
    scenarios, seeds = accounting.get("scenario_ids"), accounting.get("seeds")
    if not isinstance(scenarios, list) or len(scenarios) != 18 or len(set(scenarios)) != 18:
        raise ProductAcceptanceContractError("AUTO-15 must declare exactly 18 unique scenarios")
    if not isinstance(seeds, list) or seeds != list(range(10)) or accounting.get("required_execution_count") != 180:
        raise ProductAcceptanceContractError("AUTO-15 must declare its fixed 18x10 execution ledger")
    if accounting.get("minimum_mission_group_count") != 30 or accounting.get("required_execution_evidence") != ["video", "mcap"]:
        raise ProductAcceptanceContractError("AUTO-15 mission-group or media contract drifted")
    if spec.get("path") != "docs/a12-product-acceptance-specification.md" or spec.get("authoritative_source_path") != spec["path"]:
        raise ProductAcceptanceContractError("A12 source must be the tracked fixed specification")
    source = ROOT / spec["path"]
    _tracked(source, "A12 authoritative source")
    digest = spec.get("canonical_content_sha256")
    if not isinstance(digest, str) or not HEX64.fullmatch(digest) or _canonical_digest(_sealed_regular_bytes(source, "A12 authoritative source")) != digest:
        raise ProductAcceptanceContractError("tracked A12 authoritative source drifted")
    if authoritative_source is not None and authoritative_source.absolute() != source.absolute():
        raise ProductAcceptanceContractError("alternate or untracked A12 sources are forbidden")
    if any(value is not False for value in runtime_states.values()):
        raise ProductAcceptanceContractError("static contract must not promote product runtime states")
    if runtime.get("receipt_ingestion_status") != "BLOCKED_NO_CANONICAL_PRODUCER":
        raise ProductAcceptanceContractError("runtime receipt intake must remain fail-closed")
    return {"contract_integrity_verified": True, "runtime_receipt_ingestion_status": runtime["receipt_ingestion_status"], "scenario_count": 18, "seed_count_per_scenario": 10, "required_execution_count": 180, "required_execution_ids": [f"{scenario}:seed-{seed}" for scenario in scenarios for seed in seeds], "minimum_mission_group_count": 30, "product_runtime_states": runtime_states}


def validate_auto15_execution_evidence(contract: dict[str, Any], payload: dict[str, Any], evidence_root: Path) -> dict[str, Any]:
    validate_static_contract(contract)
    raise ProductAcceptanceContractError("AUTO-15 runtime receipt intake is BLOCKED_NO_CANONICAL_PRODUCER: wire one producer that atomically creates fresh video, MCAP, replay/recalculation, provenance, and independent mission-group receipts from the current formal session")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--execution-evidence", type=Path)
    parser.add_argument("--evidence-root", type=Path, default=ROOT)
    parser.add_argument("--authoritative-source", type=Path)
    args = parser.parse_args()
    try:
        contract = load_contract(args.contract)
        result = validate_static_contract(contract, args.authoritative_source)
        if args.execution_evidence:
            result = validate_auto15_execution_evidence(contract, {}, args.evidence_root)
    except ProductAcceptanceContractError as exc:
        print(f"product acceptance contract failed closed: {exc}")
        return 2
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
