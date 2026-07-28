#!/usr/bin/env python3
"""Verify byte counts, SHA-256 values and coverage of a compact evidence dir."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import tarfile
import tempfile
from pathlib import Path

from autonomous_runner import ROOT, load_json, verify_manifest


def verify_git_bytes(evidence_dir: Path, git_ref: str, archive: bool) -> list[str]:
    manifest = load_json(evidence_dir / "artifact_manifest.json")
    relative_root = evidence_dir.relative_to(ROOT).as_posix()
    errors: list[str] = []
    expected = {
        entry["path"]: (entry["bytes"], entry["sha256"]) for entry in manifest["files"]
    }
    expected["artifact_manifest.json"] = (
        (evidence_dir / "artifact_manifest.json").stat().st_size,
        hashlib.sha256((evidence_dir / "artifact_manifest.json").read_bytes()).hexdigest(),
    )
    for relative, (size, digest) in expected.items():
        repo_path = f"{relative_root}/{relative}"
        completed = subprocess.run(
            ["git", "show", f"{git_ref}:{repo_path}"],
            cwd=ROOT,
            capture_output=True,
        )
        if completed.returncode != 0:
            errors.append(f"missing Git blob: {repo_path}")
            continue
        if len(completed.stdout) != size:
            errors.append(f"Git blob byte count mismatch: {repo_path}")
        if hashlib.sha256(completed.stdout).hexdigest() != digest:
            errors.append(f"Git blob sha256 mismatch: {repo_path}")
    if archive:
        with tempfile.TemporaryDirectory() as temp:
            tar_path = Path(temp) / "evidence.tar"
            with tar_path.open("wb") as handle:
                completed = subprocess.run(
                    ["git", "archive", "--format=tar", git_ref, relative_root],
                    cwd=ROOT,
                    stdout=handle,
                )
            if completed.returncode != 0:
                errors.append("git archive failed")
            else:
                with tarfile.open(tar_path) as archive_file:
                    members = {
                        member.name: member
                        for member in archive_file.getmembers()
                        if member.isfile()
                    }
                    for relative, (size, digest) in expected.items():
                        repo_path = f"{relative_root}/{relative}"
                        member = members.get(repo_path)
                        if member is None:
                            errors.append(f"missing archive file: {repo_path}")
                            continue
                        payload = archive_file.extractfile(member).read()
                        if len(payload) != size:
                            errors.append(f"archive byte count mismatch: {repo_path}")
                        if hashlib.sha256(payload).hexdigest() != digest:
                            errors.append(f"archive sha256 mismatch: {repo_path}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evidence_dir", type=Path)
    parser.add_argument("--git-ref", help="also verify exact Git blob bytes at this ref")
    parser.add_argument("--archive", action="store_true", help="also verify git archive bytes")
    args = parser.parse_args()
    errors = verify_manifest(args.evidence_dir.resolve())
    if args.git_ref:
        errors.extend(verify_git_bytes(args.evidence_dir.resolve(), args.git_ref, args.archive))
    elif args.archive:
        parser.error("--archive requires --git-ref")
    print(json.dumps({"valid": not errors, "errors": errors}, indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
