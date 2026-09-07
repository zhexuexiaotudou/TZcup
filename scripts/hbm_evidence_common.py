"""Small shared primitives for HBM evidence producers.

The producers deliberately use only their explicit output directory.  A
``BLOCKED`` report is evidence of a failed/unfinished operation, never a pass.
"""

from __future__ import annotations

import hashlib
import json
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Any


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"json_object_required:{path}")
    return value


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_name(f".{path.name}.pending.{os.getpid()}")
    pending.write_text(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    pending.replace(path)


def fresh_directory(path: Path, label: str) -> None:
    """Create a new evidence root without accepting an old run or link."""

    for ancestor in (path, *path.parents):
        if ancestor.is_symlink():
            raise ValueError(f"{label}_symlink_forbidden")
    if path.exists():
        raise ValueError(f"{label}_must_not_preexist")
    path.mkdir(parents=True, exist_ok=False)


def normal_file(path: Path, label: str) -> None:
    for ancestor in (path, *path.parents):
        if ancestor.is_symlink():
            raise ValueError(f"{label}_symlink_forbidden")
    if not path.is_file():
        raise ValueError(f"{label}_missing")


def path_under(root: Path, candidate: str, label: str) -> Path:
    path = (root / candidate).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f"{label}_path_escape")
    normal_file(path, label)
    return path


def run_owned_process(command: list[str], *, timeout_seconds: float) -> tuple[int | None, str, str, dict[str, Any]]:
    """Run exactly one POSIX session and make timeout cleanup observable."""
    if os.name != "posix":
        raise ValueError("posix_process_group_supervision_required")
    started = time.monotonic()
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, start_new_session=True)
    pgid = os.getpgid(process.pid)
    details: dict[str, Any] = {"pgid": pgid, "deadline_seconds": timeout_seconds, "term_grace_seconds": 10,
                               "timed_out": False, "term_sent": False, "kill_sent": False, "zero_survivor": False}
    try:
        stdout, stderr = process.communicate(timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        details["timed_out"] = details["term_sent"] = True
        os.killpg(pgid, signal.SIGTERM)
        try:
            stdout, stderr = process.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            details["kill_sent"] = True
            os.killpg(pgid, signal.SIGKILL)
            stdout, stderr = process.communicate()
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        details["zero_survivor"] = True
    details["elapsed_seconds"] = time.monotonic() - started
    return process.returncode, stdout or "", stderr or "", details
