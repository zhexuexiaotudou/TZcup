#!/usr/bin/env python3
"""Compare immutable Stage5BR2 evidence bytes across all delivery surfaces."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import zipfile


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE_ROOT = "artifacts/stage5br2_20260720_review"


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--git-ref", default="HEAD")
    parser.add_argument("--final-zip", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    paths = subprocess.check_output(
        ["git", "ls-tree", "-r", "--name-only", args.git_ref, EVIDENCE_ROOT],
        cwd=ROOT, text=True,
    ).splitlines()
    with tempfile.TemporaryDirectory() as temporary:
        archive_path = Path(temporary) / "git-archive.zip"
        subprocess.run(
            ["git", "archive", "--format=zip", f"--output={archive_path}", args.git_ref, EVIDENCE_ROOT],
            cwd=ROOT, check=True,
        )
        with zipfile.ZipFile(archive_path) as archive, zipfile.ZipFile(args.final_zip) as final:
            suffix = f"/{EVIDENCE_ROOT}/"
            roots = sorted({name.split(suffix)[0] for name in final.namelist() if suffix in name})
            if len(roots) != 1:
                raise RuntimeError(f"cannot identify one final ZIP root: {roots}")
            final_prefix = roots[0] + "/"
            rows = []
            for path in paths:
                payloads = {
                    "working_tree": (ROOT / path).read_bytes(),
                    "git_blob": subprocess.check_output(["git", "show", f"{args.git_ref}:{path}"], cwd=ROOT),
                    "git_archive": archive.read(path),
                    "final_zip": final.read(final_prefix + path),
                }
                hashes = {name: sha(data) for name, data in payloads.items()}
                rows.append({
                    "path": path,
                    "bytes": {name: len(data) for name, data in payloads.items()},
                    "sha256": hashes,
                    "all_four_equal": len(set(hashes.values())) == 1,
                })
    mismatches = [row["path"] for row in rows if not row["all_four_equal"]]
    report = {
        "schema_version": 1,
        "stage": "Stage5BR3 Stage5BR2 archive integrity audit",
        "git_ref": args.git_ref,
        "evidence_root": EVIDENCE_ROOT,
        "final_zip": str(Path(args.final_zip)),
        "checked_file_count": len(rows),
        "mismatch_count": len(mismatches),
        "mismatch_paths": mismatches,
        "all_four_surfaces_byte_identical": not mismatches,
        "files": rows,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("checked_file_count", "mismatch_count", "mismatch_paths", "all_four_surfaces_byte_identical")}, indent=2))
    return 0 if not mismatches else 2


if __name__ == "__main__":
    raise SystemExit(main())
