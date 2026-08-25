#!/usr/bin/env python3
"""Write or verify the deterministic SHA-256 manifest for formal vehicle assets."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MESH_ROOT = ROOT / "starter_ws" / "src" / "sanitation_vehicle_description" / "meshes"
MANIFEST = MESH_ROOT / "MANIFEST.sha256"
ASSET_SUFFIXES = {".dae", ".stl", ".png"}


def content() -> str:
    rows = []
    for path in sorted(
        (item for item in MESH_ROOT.rglob("*") if item.is_file() and item.suffix.lower() in ASSET_SUFFIXES),
        key=lambda item: item.relative_to(MESH_ROOT).as_posix(),
    ):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        rows.append(f"{digest}  {path.relative_to(MESH_ROOT).as_posix()}")
    return "\n".join(rows) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--write", action="store_true")
    group.add_argument("--check", action="store_true")
    args = parser.parse_args()
    expected = content()
    if args.write:
        MANIFEST.write_text(expected, encoding="utf-8", newline="\n")
        print(f"wrote {MANIFEST} ({len(expected.splitlines())} assets)")
        return 0
    if not MANIFEST.is_file() or MANIFEST.read_text(encoding="utf-8") != expected:
        raise SystemExit("formal vehicle mesh manifest is missing or stale")
    print(f"verified {MANIFEST} ({len(expected.splitlines())} assets)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
