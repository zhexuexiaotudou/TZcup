#!/usr/bin/env python3
"""Verify evidence/index.json paths and optional SHA-256 values."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    payload = json.loads((ROOT / "evidence/index.json").read_text(encoding="utf-8"))
    errors: list[str] = []
    checked = 0
    hashed = 0
    for entry in payload:
        relative = entry.get("file")
        path = ROOT / relative
        if not path.is_file():
            errors.append(f"MISSING {relative}")
            continue
        checked += 1
        expected = entry.get("sha256")
        if expected:
            hashed += 1
            actual = sha256(path)
            if actual != expected:
                errors.append(f"HASH {relative}: {actual} != {expected}")
    print(
        json.dumps(
            {"checked": checked, "hashed": hashed, "errors": errors},
            ensure_ascii=False,
            indent=2,
        )
    )
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
