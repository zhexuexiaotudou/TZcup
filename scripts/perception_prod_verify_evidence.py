#!/usr/bin/env python3
"""Verify evidence hashes against exact staged or committed Git blob bytes."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[1]
INDEX_ROW = re.compile(
    r"^\| [^|]+ \| `(?P<sha>[0-9a-f]{64})` \| (?P<bytes>\d+) \| `(?P<path>[^`]+)` \|$"
)


def parse_index(text: str) -> list[dict]:
    return [
        {
            "path": match.group("path"),
            "sha256": match.group("sha"),
            "bytes": int(match.group("bytes")),
        }
        for line in text.splitlines()
        if (match := INDEX_ROW.match(line))
    ]


def git_blob(path: Path, revision: str) -> bytes:
    relative = path.resolve().relative_to(ROOT).as_posix()
    spec = f":{relative}" if revision == "INDEX" else f"{revision}:{relative}"
    return subprocess.check_output(["git", "cat-file", "blob", spec], cwd=ROOT)


def check_record(path: Path, record: dict, revision: str) -> list[str]:
    errors = []
    blob = git_blob(path, revision)
    actual_hash = hashlib.sha256(blob).hexdigest()
    if actual_hash != record["sha256"]:
        errors.append(f"{path}: expected {record['sha256']} got {actual_hash}")
    expected_bytes = record.get("bytes")
    if expected_bytes is not None and len(blob) != int(expected_bytes):
        errors.append(f"{path}: expected {expected_bytes} bytes got {len(blob)}")
    return errors


def verify(root: Path, revision: str) -> list[str]:
    root = root.resolve()
    errors = []
    for manifest_path in sorted(root.rglob("artifact_manifest.json")):
        manifest = json.loads(git_blob(manifest_path, revision).decode("utf-8"))
        records = manifest.get("files", manifest.get("artifacts", []))
        for record in records:
            errors.extend(
                check_record(manifest_path.parent / record["path"], record, revision)
            )
    final_index = root / "PERCEPTION_PRODUCT_FINAL_EVIDENCE_INDEX.md"
    if final_index.exists():
        try:
            index_blob = git_blob(final_index, revision)
        except subprocess.CalledProcessError:
            index_blob = b""
        if index_blob:
            records = parse_index(index_blob.decode("utf-8"))
            if not records:
                errors.append(f"{final_index}: no evidence rows parsed")
            for record in records:
                errors.extend(check_record(root / record["path"], record, revision))
    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--revision", default="HEAD")
    args = parser.parse_args()
    errors = verify(args.evidence_root, args.revision)
    if errors:
        for error in errors:
            print(error)
        return 2
    print(f"evidence Git-blob hashes verified at {args.revision}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
