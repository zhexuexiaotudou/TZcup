"""Small shared primitives for HBM evidence producers.

The producers deliberately use only their explicit output directory.  A
``BLOCKED`` report is evidence of a failed/unfinished operation, never a pass.
"""

from __future__ import annotations

import hashlib
import json
import os
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


def normal_file(path: Path, label: str) -> None:
    if path.is_symlink():
        raise ValueError(f"{label}_symlink_forbidden")
    if not path.is_file():
        raise ValueError(f"{label}_missing")


def path_under(root: Path, candidate: str, label: str) -> Path:
    path = (root / candidate).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f"{label}_path_escape")
    normal_file(path, label)
    return path
