#!/usr/bin/env python3
"""Execute the single bounded CRV6 historical-checkpoint recovery audit."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
from typing import Iterable
import zipfile


TARGET_SHA256 = "481374d4839e72f05fff0d6d2f6135bc7d715d5c2faf84e75d7d97ca3fc6a361"
INITIALIZATION_SHA256 = "833e6148f566aed60c27378c4c1f832bb0e3f7532dae780d12ce5424579e2dfa"
MODEL_SUFFIXES = {".pth", ".pt", ".ckpt", ".onnx", ".bin"}
ARCHIVE_SUFFIXES = {".zip", ".tar", ".tgz", ".gz", ".7z"}


def tzcup_roots() -> list[Path]:
    """Return the fixed, reviewable CRV6 Windows search boundary."""
    user = Path("C:/Users/zhexu")
    return [
        Path("F:/Project/TZcup/.workspace/artifacts"),
        Path("F:/Project/TZcup/.workspace/worktrees"),
        Path("F:/Project/TZcup"),
        Path("F:/Project/TZcup-online-domain-closure-v5"),
        Path("F:/Project/TZcup-perception-recovery"),
        Path("F:/Project/TZcup-opr-c-training"),
        Path("C:/") / "$Recycle.Bin",
        user / "AppData/Local/Temp",
        user / ".cache",
        user / "Downloads",
        user / "Documents",
        user / "Desktop",
    ]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _bounded_files(roots: Iterable[Path], suffixes: set[str]) -> tuple[list[Path], list[dict]]:
    found: list[Path] = []
    scopes: list[dict] = []
    for root in roots:
        root = root.resolve()
        scope = {"root": root.as_posix(), "exists": root.exists(), "errors": []}
        if root.exists():
            try:
                for directory, names, files in os.walk(root):
                    names[:] = [
                        name for name in names
                        if name not in {".git", ".workspace", "node_modules", "__pycache__"}
                    ]
                    for name in files:
                        path = Path(directory, name)
                        lower = name.lower()
                        if path.suffix.lower() in suffixes or any(lower.endswith(item) for item in suffixes):
                            found.append(path)
            except OSError as exc:
                scope["errors"].append(str(exc))
        scopes.append(scope)
    return sorted(set(found)), scopes


def _run(command: list[str], *, timeout: int = 120) -> dict:
    try:
        process = subprocess.run(
            command, capture_output=True, text=True, encoding="utf-8", errors="replace",
            timeout=timeout, check=False,
        )
        return {
            "command": command,
            "exit_code": process.returncode,
            "stdout": (process.stdout or "")[-20000:],
            "stderr": (process.stderr or "")[-5000:],
        }
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"command": command, "exit_code": None, "stdout": "", "stderr": str(exc)}


def _zip_records(archives: Iterable[Path]) -> list[dict]:
    """Hash model-like members directly from ZIP bytes without extraction."""
    records: list[dict] = []
    for archive in archives:
        if archive.suffix.lower() != ".zip":
            continue
        try:
            with zipfile.ZipFile(archive) as bundle:
                for member in bundle.infolist():
                    if member.is_dir() or Path(member.filename).suffix.lower() not in MODEL_SUFFIXES:
                        continue
                    digest = hashlib.sha256()
                    with bundle.open(member) as stream:
                        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                            digest.update(chunk)
                    records.append({
                        "archive": archive.as_posix(),
                        "member": member.filename,
                        "size_bytes": member.file_size,
                        "sha256": digest.hexdigest(),
                    })
        except (OSError, zipfile.BadZipFile, RuntimeError) as exc:
            records.append({"archive": archive.as_posix(), "error": str(exc)})
    return records


def _remote_audits(repository: str) -> list[dict]:
    return [
        _run(["gh", "api", f"repos/{repository}/actions/artifacts", "--paginate"], timeout=90),
        _run(["gh", "release", "list", "--repo", repository, "--limit", "100"], timeout=90),
    ]


def _wsl_audit(distribution: str) -> dict:
    script = (
        # Windows audits the mounted workspace bytes directly.  Searching it
        # again through drvfs is redundant and can take unbounded time.
        "find /tmp /var/tmp -type f "
        "\\( -iname '*.pth' -o -iname '*.pt' -o -iname '*.ckpt' -o -iname '*.onnx' \\) "
        "-printf '%s|%p\\n' 2>/dev/null"
    )
    return _run(["wsl", "-d", distribution, "--", "bash", "-lc", script], timeout=180)


def _docker_audits() -> list[dict]:
    results = [
        _run(["docker", "ps", "-a", "--format", "{{.ID}}|{{.Image}}|{{.Names}}|{{.Mounts}}"]),
        _run(["docker", "images", "--format", "{{.Repository}}:{{.Tag}}|{{.ID}}|{{.Size}}"]),
        _run(["docker", "volume", "ls", "--format", "{{.Name}}"]),
    ]
    images = [
        "tzcup/opr-c-rtmdet:v3.3.0-ops",
        "tzcup/opr-c-rtmdet:v3.3.0",
        "tzcup/perception-product:oprv3-7053ff8",
        "tzcup/perception-product:oprv3-d52dfa2",
    ]
    find_script = "find / -xdev -type f \\( -name '*.pth' -o -name '*.pt' -o -name '*.ckpt' -o -name '*.onnx' \\) -printf '%s|%p\\n' 2>/dev/null"
    for image in images:
        results.append(_run([
            "docker", "run", "--rm", "--network", "none", "--read-only",
            "--entrypoint", "sh", image, "-lc", find_script,
        ], timeout=180))
    return results


def build_report(
    roots: list[Path], *, repository: str, distribution: str,
    include_external: bool = True,
) -> dict:
    candidates, scopes = _bounded_files(roots, MODEL_SUFFIXES)
    archives, archive_scopes = _bounded_files(roots, ARCHIVE_SUFFIXES)
    archive_records = _zip_records(archives)
    records = []
    exact_path = None
    initialization_path = None
    for path in candidates:
        try:
            value = sha256(path)
            record = {"path": path.as_posix(), "size_bytes": path.stat().st_size, "sha256": value}
            records.append(record)
            if value == TARGET_SHA256:
                exact_path = path.as_posix()
            if value == INITIALIZATION_SHA256:
                initialization_path = path.as_posix()
        except OSError as exc:
            records.append({"path": path.as_posix(), "error": str(exc)})
    for record in archive_records:
        if record.get("sha256") == TARGET_SHA256:
            exact_path = f"{record['archive']}::{record['member']}"
        if record.get("sha256") == INITIALIZATION_SHA256:
            initialization_path = f"{record['archive']}::{record['member']}"
    external = {
        "wsl": _wsl_audit(distribution),
        "docker": _docker_audits(),
        "github": _remote_audits(repository),
    } if include_external else {"not_run_for_unit_test": True}
    return {
        "schema_version": 1,
        "protocol": "CHECKPOINT-RECONSTITUTION-V6",
        "stage": "CRV6-00",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "single_bounded_final_pass": True,
        "target": {"historical_D1_B_sha256": TARGET_SHA256},
        "search_scopes": scopes,
        "archive_scopes": archive_scopes,
        "archive_candidates": [
            {"path": path.as_posix(), "size_bytes": path.stat().st_size}
            for path in archives if path.exists()
        ],
        "archive_model_members": archive_records,
        "model_candidates": records,
        "external_audits": external,
        "exact_checkpoint_path": exact_path,
        "exact_checkpoint_recovered": exact_path is not None,
        "HISTORICAL_D1B_CHECKPOINT_LOST": exact_path is None,
        "recovery_search_closed": True,
        "D1_B_initialization": {
            "expected_sha256": INITIALIZATION_SHA256,
            "path": initialization_path,
            "available": initialization_path is not None,
        },
        "next_route": "HISTORICAL_RECOVERED" if exact_path else (
            "R1" if initialization_path else "R2"
        ),
        "historical_DDRV4_D1_PASS_rewritten": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--repository", default="zhexuexiaotudou/TZcup")
    parser.add_argument("--distribution", default="TZcup-Ubuntu-24.04")
    parser.add_argument("--root", action="append", type=Path)
    parser.add_argument("--scope-profile", choices=["tzcup"])
    args = parser.parse_args()
    roots = list(args.root or [])
    if args.scope_profile == "tzcup":
        roots.extend(tzcup_roots())
    if not roots:
        parser.error("at least one --root or --scope-profile is required")
    if args.output.exists():
        raise FileExistsError(f"audit output already exists: {args.output}")
    report = build_report(roots, repository=args.repository, distribution=args.distribution)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "output": args.output.as_posix(), "sha256": sha256(args.output),
        "exact_checkpoint_recovered": report["exact_checkpoint_recovered"],
        "next_route": report["next_route"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
