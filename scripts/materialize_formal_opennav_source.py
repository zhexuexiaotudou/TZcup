#!/usr/bin/env python3
"""Materialize the pinned patched OpenNav checkout from an offline bundle.

This intentionally accepts no network URL.  The full official patched bundle
is retained as the source provenance, while colcon later selects only the two
packages used by sanitation_coverage.

The formal builder invokes this helper from its repository worktree so
``git bundle verify`` can validate the complete bundle in normal Git context.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import tempfile
from pathlib import Path

from formal_final_runtime_closure import (
    OPENNAV_COVERAGE_BASE_COMMIT as BASE_COMMIT,
    OPENNAV_COVERAGE_BASE_TREE as BASE_TREE,
    OPENNAV_COVERAGE_BUNDLE_SHA256 as BUNDLE_SHA256,
    OPENNAV_COVERAGE_PACKAGES as REQUIRED_PACKAGES,
    OPENNAV_COVERAGE_PATCH_SHA256 as PATCH_SHA256,
    OPENNAV_COVERAGE_PATCHED_COMMIT as PATCHED_COMMIT,
    OPENNAV_COVERAGE_PATCHED_DIFF_SHA256 as PATCHED_DIFF_SHA256,
    OPENNAV_COVERAGE_PATCHED_TREE as PATCHED_TREE,
)


class MaterializeError(RuntimeError):
    pass


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _run(arguments: list[str], label: str) -> str:
    result = subprocess.run(arguments, capture_output=True, text=True, check=False)
    if result.returncode:
        detail = (result.stderr or result.stdout).strip()
        raise MaterializeError(f"{label} failed rc={result.returncode}: {detail}")
    return result.stdout.strip()


def _verify_bundle(bundle: Path) -> None:
    """Verify a complete bundle without depending on the caller's Git metadata."""

    with tempfile.TemporaryDirectory(prefix="tzcup-opennav-bundle-verify-") as raw:
        repository = Path(raw)
        _run(["git", "init", "--bare", str(repository)], "OpenNav verification repository init")
        _run(
            ["git", "-C", str(repository), "bundle", "verify", str(bundle)],
            "OpenNav source bundle verification",
        )


def materialize(bundle: Path, destination: Path, report: Path) -> dict[str, object]:
    if bundle.is_symlink() or not bundle.is_file():
        raise MaterializeError(f"OpenNav source bundle is not a regular file: {bundle}")
    if _sha256(bundle) != BUNDLE_SHA256:
        raise MaterializeError("OpenNav source bundle SHA-256 does not match the pinned bundle")
    if destination.exists() or destination.is_symlink():
        raise MaterializeError(f"OpenNav destination is not fresh: {destination}")
    if report.exists() or report.is_symlink():
        raise MaterializeError(f"OpenNav provenance report is not fresh: {report}")
    _verify_bundle(bundle)
    _run(["git", "clone", "--no-checkout", str(bundle), str(destination)], "OpenNav bundle clone")
    _run(["git", "-C", str(destination), "checkout", "--detach", PATCHED_COMMIT], "OpenNav patched checkout")
    actual = {
        "base_commit": _run(["git", "-C", str(destination), "rev-parse", "HEAD^"], "OpenNav base commit"),
        "base_tree": _run(["git", "-C", str(destination), "rev-parse", f"{BASE_COMMIT}^{{tree}}"], "OpenNav base tree"),
        "patched_commit": _run(["git", "-C", str(destination), "rev-parse", "HEAD"], "OpenNav patched commit"),
        "patched_tree": _run(["git", "-C", str(destination), "rev-parse", "HEAD^{tree}"], "OpenNav patched tree"),
        "working_tree_clean": not _run(["git", "-C", str(destination), "status", "--porcelain"], "OpenNav working tree"),
    }
    diff = subprocess.run(
        ["git", "-C", str(destination), "diff", "--binary", BASE_COMMIT, "HEAD"],
        capture_output=True,
        check=False,
    )
    if diff.returncode:
        raise MaterializeError(f"OpenNav patched diff failed rc={diff.returncode}")
    actual["patched_diff_sha256"] = hashlib.sha256(diff.stdout).hexdigest()
    expected = {
        "base_commit": BASE_COMMIT,
        "base_tree": BASE_TREE,
        "patched_commit": PATCHED_COMMIT,
        "patched_tree": PATCHED_TREE,
        "patched_diff_sha256": PATCHED_DIFF_SHA256,
        "working_tree_clean": True,
    }
    if actual != expected:
        raise MaterializeError(f"OpenNav source identity drifted: {actual}")
    for package in REQUIRED_PACKAGES:
        package_xml = destination / package / "package.xml"
        if package_xml.is_symlink() or not package_xml.is_file():
            raise MaterializeError(f"OpenNav required package is missing: {package}")
    payload: dict[str, object] = {
        "schema_version": 1,
        "repository": "opennav_coverage",
        **expected,
        "patch_sha256": PATCH_SHA256,
        "bundle_sha256": BUNDLE_SHA256,
        "required_packages": list(REQUIRED_PACKAGES),
    }
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", required=True, type=Path)
    parser.add_argument("--destination", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args()
    try:
        print(json.dumps(materialize(args.bundle, args.destination, args.report), indent=2, sort_keys=True))
    except (MaterializeError, OSError, ValueError) as exc:
        print(json.dumps({"status": "FORMAL_OPENNAV_SOURCE_BLOCKED", "error": str(exc)}, indent=2))
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
