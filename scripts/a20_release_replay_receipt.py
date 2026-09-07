#!/usr/bin/env python3
"""Fail closed until TZcup has a canonical product MCAP replay producer.

AUTO-02 and AUTO-03 replay tools are stage-specific. Neither replays the
formal product chain with coverage, localization, runtime closure and session
bindings, so A20 must not promote their reports or arbitrary JSON.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any


SCHEMA = "tzcup.a20_release_replay_receipt.v1"
BLOCKER = (
    "canonical formal product replay producer is absent; AUTO-02/AUTO-03 and "
    "coverage-only replay audits cannot satisfy A20"
)


def validate_receipt(receipt: dict[str, Any]) -> dict[str, Any]:
    """Purely reject a receipt until a complete canonical producer exists."""

    del receipt
    return {
        "schema": "tzcup.a20_release_replay_receipt_validation.v1",
        "status": "A20_RECEIPT_STATIC_BLOCKED",
        "valid": False,
        "errors": [BLOCKER],
        "release_runtime_pass": False,
        "claim_boundary": (
            "A20 has no canonical complete product replay producer. No supplied "
            "hash, embedded report, or historical AUTO-16 artifact can be promoted."
        ),
    }


def _regular_in_root(root: Path, candidate: Path, label: str) -> Path:
    if not root.is_absolute() or root.is_symlink() or not root.is_dir():
        raise ValueError("repository root must be an absolute non-symlink directory")
    if not candidate.is_absolute():
        raise ValueError(f"{label} must be absolute")
    try:
        relative = candidate.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"{label} escapes repository root") from exc
    if ".." in relative.parts:
        raise ValueError(f"{label} escapes repository root")
    current = root
    for part in relative.parts:
        current /= part
        if current.exists() and current.is_symlink():
            raise ValueError(f"{label} has a symbolic-link ancestor")
    if not candidate.is_file() or candidate.is_symlink():
        raise ValueError(f"{label} must be a regular non-symlink file")
    return candidate


def _output_in_root(root: Path, output: Path) -> Path:
    if not output.is_absolute():
        raise ValueError("output must be absolute")
    try:
        relative = output.relative_to(root)
    except ValueError as exc:
        raise ValueError("output escapes repository root") from exc
    if ".." in relative.parts:
        raise ValueError("output escapes repository root")
    parent = output.parent
    if not parent.is_dir() or parent.is_symlink():
        raise ValueError("output parent must be an existing non-symlink directory")
    probe = parent
    while probe != root:
        if probe.is_symlink():
            raise ValueError("output has a symbolic-link ancestor")
        probe = probe.parent
    if output.exists() or output.is_symlink():
        raise ValueError("output must be fresh")
    return output


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + f".pending.{os.getpid()}")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        if not args.repository_root.is_absolute() or args.repository_root.is_symlink():
            raise ValueError("repository root must be absolute and non-symlink")
        root = args.repository_root.resolve(strict=True)
        receipt_path = _regular_in_root(root, args.receipt, "receipt")
        output = _output_in_root(root, args.output)
        before = _sha256(receipt_path)
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        if not isinstance(receipt, dict):
            raise ValueError("receipt root is not an object")
        report = validate_receipt(receipt)
        after = _sha256(receipt_path)
        if before != after:
            raise ValueError("receipt changed while being read")
        report["receipt_sha256"] = after
        _atomic_json(output, report)
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        print(json.dumps({"status": "A20_RECEIPT_STATIC_BLOCKED", "error": str(exc)}, indent=2))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
