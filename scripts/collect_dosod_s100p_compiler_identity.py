#!/usr/bin/env python3
"""Collect a live, fail-closed OpenExplorer compiler identity receipt.

Run this only inside the isolated Linux x86_64 OE 3.7.0 environment that will
execute ``hb_compile``.  It does not compile a model and does not access the
S100P board.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import platform
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


EXPECTED_OE_VERSION = "3.7.0"
EXPECTED_ARCHIVE_SHA256 = "de90da5cf58879a0883bb47856232514c3cc30e368d8864911bd05e267229c5b"
EXPECTED_VERSIONS = {
    "hbdk4_compiler": "4.7.5",
    "hmct": "2.6.5",
    "horizon_tc_ui": "3.5.3",
}
PACKAGE_NAMES = {
    "hbdk4_compiler": "hbdk4-compiler",
    "hmct": "hmct",
    "horizon_tc_ui": "horizon-tc-ui",
}


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def evaluate_identity(
    *,
    system: str,
    machine: str,
    versions: dict[str, str | None],
    executable_path: str | None,
    executable_sha256: str | None,
    probe_returncode: int | None,
    probe_output: bytes,
    discovery: dict[str, Any],
) -> dict[str, Any]:
    blockers: list[str] = []
    if system != "Linux":
        blockers.append("compiler_host_not_linux")
    if machine not in {"x86_64", "AMD64"}:
        blockers.append("compiler_host_not_x86_64")
    if versions != EXPECTED_VERSIONS:
        blockers.append("compiler_package_versions_mismatch")
    if not executable_path:
        blockers.append("hb_compile_executable_missing")
    if not isinstance(executable_sha256, str) or len(executable_sha256) != 64:
        blockers.append("hb_compile_executable_sha256_invalid")
    if probe_returncode != 0:
        blockers.append("hb_compile_probe_failed")
    decoded = probe_output.decode("utf-8", errors="replace")
    if "nash-m" not in decoded or "--config" not in decoded:
        blockers.append("hb_compile_probe_contract_missing")
    source = discovery.get("official_source", {}) if isinstance(discovery, dict) else {}
    if source.get("oe_version") != EXPECTED_OE_VERSION:
        blockers.append("toolchain_discovery_oe_version_mismatch")
    if source.get("archive_sha256") != EXPECTED_ARCHIVE_SHA256:
        blockers.append("toolchain_discovery_archive_sha256_mismatch")
    if discovery.get("required_versions") != EXPECTED_VERSIONS:
        blockers.append("toolchain_discovery_required_versions_mismatch")
    if discovery.get("official_toolchain_package_ready") is not True:
        blockers.append("toolchain_discovery_not_ready")
    return {
        "schema_version": 1,
        "report_id": "tzcup_dosod_s100p_live_compiler_identity_v1",
        "identity_verified": not blockers,
        "oe_version": EXPECTED_OE_VERSION,
        "required_versions": versions,
        "platform_system": system,
        "platform_machine": machine,
        "hb_compile_executable": executable_path,
        "hb_compile_executable_sha256": executable_sha256,
        "hb_compile_probe_argv": ["--help"],
        "hb_compile_probe_returncode": probe_returncode,
        "hb_compile_probe_output_sha256": sha256_bytes(probe_output),
        "hb_compile_probe_output_bytes": len(probe_output),
        "blockers": blockers,
        "compile_executed": False,
        "hbm_status": "HBM_NOT_PRODUCED",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--toolchain-discovery", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--allow-blocked-exit-zero", action="store_true")
    args = parser.parse_args()
    discovery_path = Path(args.toolchain_discovery)
    output_path = Path(args.output)
    try:
        discovery = json.loads(discovery_path.read_text(encoding="utf-8"))
    except Exception as exc:
        discovery = {}
        discovery_error = f"toolchain_discovery_unreadable:{type(exc).__name__}"
    else:
        discovery_error = None

    versions: dict[str, str | None] = {}
    for role, package_name in PACKAGE_NAMES.items():
        try:
            versions[role] = importlib.metadata.version(package_name)
        except importlib.metadata.PackageNotFoundError:
            versions[role] = None
    executable = shutil.which("hb_compile")
    executable_path = Path(executable).resolve() if executable else None
    executable_sha = sha256_file(executable_path) if executable_path and executable_path.is_file() else None
    if executable_path:
        try:
            probe = subprocess.run(
                [str(executable_path), "--help"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=30,
                check=False,
            )
        except Exception as exc:
            probe_returncode = None
            probe_output = f"probe_exception:{type(exc).__name__}".encode()
        else:
            probe_returncode = int(probe.returncode)
            probe_output = bytes(probe.stdout)
    else:
        probe_returncode = None
        probe_output = b""
    report = evaluate_identity(
        system=platform.system(),
        machine=platform.machine(),
        versions=versions,
        executable_path=str(executable_path) if executable_path else None,
        executable_sha256=executable_sha,
        probe_returncode=probe_returncode,
        probe_output=probe_output,
        discovery=discovery,
    )
    report["collected_utc"] = datetime.now(timezone.utc).isoformat()
    report["toolchain_discovery_path"] = str(discovery_path.resolve())
    report["toolchain_discovery_sha256"] = (
        sha256_file(discovery_path) if discovery_path.is_file() else None
    )
    if discovery_error:
        report["blockers"].append(discovery_error)
        report["identity_verified"] = False
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if report["identity_verified"]:
        return 0
    return 0 if args.allow_blocked_exit_zero else 2


if __name__ == "__main__":
    raise SystemExit(main())
