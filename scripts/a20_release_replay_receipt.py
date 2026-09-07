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
import secrets
import stat
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


def _identity(value: os.stat_result) -> tuple[int, int, int, int, int]:
    return (
        value.st_dev,
        value.st_ino,
        value.st_size,
        value.st_mtime_ns,
        value.st_ctime_ns,
    )


def _open_directory(root: Path, parts: tuple[str, ...]) -> int:
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
    expected = os.stat(root, follow_symlinks=False)
    descriptor = os.open(root, flags)
    try:
        if _identity(os.fstat(descriptor))[:2] != _identity(expected)[:2]:
            raise ValueError("repository root changed before secure open")
        for part in parts:
            child = os.open(part, flags, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
            if not stat.S_ISDIR(os.fstat(descriptor).st_mode):
                raise ValueError("path parent is not a directory")
        return descriptor
    except Exception:
        os.close(descriptor)
        raise


def _open_bound_input(root: Path, path: Path) -> tuple[int, tuple[int, int, int, int, int]]:
    relative = path.relative_to(root)
    directory = _open_directory(root, relative.parts[:-1])
    try:
        expected = os.stat(relative.name, dir_fd=directory, follow_symlinks=False)
        flags = os.O_RDONLY | getattr(os, "O_BINARY", 0) | getattr(os, "O_NOFOLLOW", 0)
        descriptor = os.open(relative.name, flags, dir_fd=directory)
    finally:
        os.close(directory)
    if not stat.S_ISREG(expected.st_mode):
        raise ValueError("receipt must be a regular file")
    try:
        actual = os.fstat(descriptor)
        if not stat.S_ISREG(actual.st_mode) or _identity(actual)[:2] != _identity(expected)[:2]:
            raise ValueError("receipt changed before secure open")
        return descriptor, _identity(actual)
    except Exception:
        os.close(descriptor)
        raise


def _read_bound_json(descriptor: int, expected: tuple[int, int, int, int, int]) -> tuple[dict[str, Any], str]:
    digest = hashlib.sha256()
    chunks: list[bytes] = []
    with os.fdopen(descriptor, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
            chunks.append(chunk)
        if _identity(os.fstat(stream.fileno())) != expected:
            raise ValueError("receipt changed while being read")
    raw = b"".join(chunks)
    try:
        payload = json.loads(raw.decode("utf-8"))
    except UnicodeDecodeError as exc:
        raise ValueError("receipt is not UTF-8 JSON") from exc
    if not isinstance(payload, dict):
        raise ValueError("receipt root is not an object")
    return payload, digest.hexdigest()


def _write_fresh_output(root: Path, path: Path, payload: dict[str, Any], *, token: str | None = None) -> None:
    """Atomically publish without following a pending link or replacing output."""

    relative = path.relative_to(root)
    directory = _open_directory(root, relative.parts[:-1])
    temporary = f".{path.name}.pending.{token or secrets.token_hex(16)}"
    try:
        try:
            os.stat(path.name, dir_fd=directory, follow_symlinks=False)
        except FileNotFoundError:
            pass
        else:
            raise ValueError("output appeared after validation")
        descriptor = os.open(
            temporary,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_BINARY", 0),
            0o600,
            dir_fd=directory,
        )
        try:
            data = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
            written = 0
            while written < len(data):
                written += os.write(descriptor, data[written:])
            os.fsync(descriptor)
            expected_file = _identity(os.fstat(descriptor))[:2]
        finally:
            os.close(descriptor)
        try:
            os.link(
                temporary,
                path.name,
                src_dir_fd=directory,
                dst_dir_fd=directory,
                follow_symlinks=False,
            )
        except FileExistsError as exc:
            raise ValueError("output appeared during commit") from exc
        committed = os.stat(path.name, dir_fd=directory, follow_symlinks=False)
        if not stat.S_ISREG(committed.st_mode) or _identity(committed)[:2] != expected_file:
            raise ValueError("output commit identity mismatch")
    finally:
        try:
            os.unlink(temporary, dir_fd=directory)
        except FileNotFoundError:
            pass
        os.close(directory)


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
        descriptor, identity = _open_bound_input(root, receipt_path)
        receipt, receipt_sha256 = _read_bound_json(descriptor, identity)
        report = validate_receipt(receipt)
        report["receipt_sha256"] = receipt_sha256
        _write_fresh_output(root, output, report)
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        print(json.dumps({"status": "A20_RECEIPT_STATIC_BLOCKED", "error": str(exc)}, indent=2))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
