#!/usr/bin/env python3
"""Materialize a checksum-locked Journey 6 board bundle from the Git skeleton."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

import yaml


FORBIDDEN = ("rdk", "s100", "s100p", "s600")
PROFILE_MARCHES = {"auto", "nash-e", "nash-m", "nash-p"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_profile(profile: dict) -> list[str]:
    failures: list[str] = []
    if profile.get("target_family") != "journey6":
        failures.append("profile_family_must_be_journey6")
    if profile.get("target_sku") != "auto":
        failures.append("profile_sku_must_be_auto")
    if profile.get("target_march") not in PROFILE_MARCHES:
        failures.append("profile_march_not_allowed")
    selection = profile.get("selection", {})
    if selection.get("requires_inventory_match") is not True:
        failures.append("profile_must_require_inventory_match")
    if profile.get("target_march") != "auto" and selection.get("selectable_before_inventory") is not False:
        failures.append("march_profile_cannot_be_selected_before_inventory")
    text = json.dumps(profile, sort_keys=True).lower()
    if any(marker in text for marker in FORBIDDEN):
        failures.append("profile_references_forbidden_rdk_or_s100_family")
    return failures


def checksum_rows(root: Path) -> list[str]:
    rows = []
    for path in sorted(item for item in root.rglob("*") if item.is_file() and item.name != "SHA256SUMS"):
        rows.append(f"{sha256(path)}  {path.relative_to(root).as_posix()}")
    return rows


def _git_commit(repo: Path) -> str | None:
    result = subprocess.run(["git", "rev-parse", "HEAD"], cwd=repo, check=False, capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else None


def build(source: Path, output: Path, repo: Path) -> dict:
    if output.exists():
        raise FileExistsError(f"output already exists: {output}")
    manifest = json.loads((source / "bundle_manifest.json").read_text(encoding="utf-8"))
    failures: list[str] = []
    if manifest.get("target_family") != "journey6" or manifest.get("target_sku") != "auto" or manifest.get("target_march") != "auto":
        failures.append("bundle_target_must_be_journey6_with_auto_sku_and_march")
    manifest_text = json.dumps(manifest, sort_keys=True).lower()
    if any(marker in manifest_text for marker in FORBIDDEN):
        failures.append("bundle_manifest_references_forbidden_rdk_or_s100_family")
    profiles = []
    for path in sorted((source / "profiles").glob("*.yaml")):
        profile = yaml.safe_load(path.read_text(encoding="utf-8"))
        profiles.append(profile.get("profile_id"))
        failures.extend(f"{path.name}:{failure}" for failure in validate_profile(profile))
    if sorted(profiles) != sorted(manifest.get("profiles", [])):
        failures.append("manifest_profile_inventory_mismatch")
    if not manifest.get("external_blockers"):
        for path in sorted((source / "profiles").glob("*.yaml")):
            profile = yaml.safe_load(path.read_text(encoding="utf-8"))
            if profile.get("target_march") == "auto":
                continue
            runtime = profile.get("runtime", {})
            deployment = profile.get("deployment", {})
            for field in ("abi", "minimum_version"):
                if not runtime.get(field):
                    failures.append(f"{path.name}:runtime_{field}_unresolved")
            for field in ("sanity_command", "warmup_command", "parity_command", "healthcheck_command"):
                if not deployment.get(field):
                    failures.append(f"{path.name}:deployment_{field}_unresolved")
    if failures:
        return {"status": "invalid", "bundle_ready": False, "failures": failures}

    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="tzcup-j6-bundle-", dir=output.parent) as temp_name:
        staging = Path(temp_name) / output.name
        shutil.copytree(source, staging)
        (staging / "scripts").mkdir(exist_ok=True)
        shutil.copy2(repo / "scripts" / "j6_board_inventory.py", staging / "scripts" / "j6_board_inventory.py")
        shutil.copy2(repo / "scripts" / "j6_board_inventory.sh", staging / "scripts" / "j6_board_inventory.sh")
        built_manifest = json.loads((staging / "bundle_manifest.json").read_text(encoding="utf-8"))
        built_manifest["source_commit"] = _git_commit(repo)
        built_manifest["status"] = "skeleton" if built_manifest.get("external_blockers") else "candidate"
        (staging / "bundle_manifest.json").write_text(json.dumps(built_manifest, indent=2) + "\n", encoding="utf-8")
        rows = checksum_rows(staging)
        (staging / "SHA256SUMS").write_text("\n".join(rows) + "\n", encoding="utf-8")
        staging.rename(output)
    blockers = manifest.get("external_blockers", [])
    return {
        "schema_version": 1,
        "target_family": "journey6",
        "status": "blocked_external" if blockers else "ready",
        "bundle_ready": not blockers,
        "bundle_path": str(output.resolve()),
        "bundle_manifest_sha256": sha256(output / "bundle_manifest.json"),
        "sha256sums_sha256": sha256(output / "SHA256SUMS"),
        "external_blockers": blockers,
        "truth_boundary": "A skeleton bundle is not deployable until official SDK, board inventory, runtime profile, warmup, models, and health checks are resolved.",
    }


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default=str(repo / "deploy" / "journey6" / "board_bundle"))
    parser.add_argument("--output", required=True)
    parser.add_argument("--report")
    args = parser.parse_args()
    report = build(Path(args.source).resolve(), Path(args.output).resolve(), repo)
    if args.report:
        report_path = Path(args.report)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report.get("bundle_ready") else 2


if __name__ == "__main__":
    raise SystemExit(main())
