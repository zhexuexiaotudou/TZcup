#!/usr/bin/env python3
"""Fail-closed audit of a checked-out repository against locked_revisions.json."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any


def _git(repo: Path, *arguments: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(repo), *arguments],
        text=True,
        stderr=subprocess.STDOUT,
    ).strip()


def audit_repository(
    repo: Path, lock_file: Path, repository: str
) -> dict[str, Any]:
    registry = json.loads(lock_file.read_text(encoding="utf-8"))
    expected = registry["repositories"][repository]
    errors: list[str] = []
    try:
        actual_commit = _git(repo, "rev-parse", "HEAD")
        actual_url = _git(repo, "remote", "get-url", "origin")
        status_text = _git(repo, "status", "--porcelain=v1", "--untracked-files=all")
    except (subprocess.CalledProcessError, OSError) as exc:
        actual_commit = "unavailable"
        actual_url = "unavailable"
        status_text = ""
        errors.append(f"git audit failed: {exc}")

    status = status_text.splitlines() if status_text else []
    if actual_commit != expected["commit"]:
        errors.append(
            f"commit mismatch: expected {expected['commit']}, got {actual_commit}"
        )
    if actual_url != expected["url"]:
        errors.append(f"origin mismatch: expected {expected['url']}, got {actual_url}")
    if status:
        errors.append(f"working tree is dirty: {status!r}")

    return {
        "schema_version": 1,
        "repository": repository,
        "expected_commit": expected["commit"],
        "actual_commit": actual_commit,
        "expected_url": expected["url"],
        "actual_url": actual_url,
        "status_porcelain": status,
        "working_tree_clean": not status,
        "verified": not errors,
        "errors": errors,
    }


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--lock-file", type=Path, required=True)
    parser.add_argument("--repository", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    payload = audit_repository(
        args.root.resolve(), args.lock_file.resolve(), args.repository
    )
    _write_json(args.output.resolve(), payload)
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True))
    if not payload["verified"]:
        print("; ".join(payload["errors"]), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
