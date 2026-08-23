#!/usr/bin/env python3
"""Collect a read-only Journey 6 board inventory with explicit unknowns."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import re


def _read(path: str) -> str | None:
    try:
        value = Path(path).read_bytes().replace(b"\x00", b" ").decode(errors="replace").strip()
        return value or None
    except OSError:
        return None


def _command(argv: list[str]) -> str | None:
    executable = shutil.which(argv[0])
    if not executable:
        return None
    try:
        result = subprocess.run([executable, *argv[1:]], check=False, capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired):
        return None
    value = (result.stdout or result.stderr).strip()
    return value or None


def _memory_bytes() -> int | None:
    text = _read("/proc/meminfo")
    if not text:
        return None
    for line in text.splitlines():
        if line.startswith("MemTotal:"):
            return int(line.split()[1]) * 1024
    return None


def _release_facts() -> dict:
    sources = {
        path: _read(path)
        for path in (
            "/etc/journey6-release",
            "/etc/horizon/journey6-release",
            "/etc/horizon-release",
        )
    }
    text = "\n".join(value for value in sources.values() if value)
    lowered = text.lower()
    march_match = re.search(r"\bnash[-_ ]?([emp])\b", lowered)
    sku_match = re.search(r"\bjourney\s*6\s*([blemhp])\b", lowered)
    abi_match = re.search(r"(?:runtime[_ -]?abi|abi)\s*[:=]\s*([a-z0-9_.+-]+)", lowered)
    version_match = re.search(r"(?:runtime[_ -]?version|dnn[_ -]?version)\s*[:=]\s*([0-9][a-z0-9_.+-]*)", lowered)
    return {
        "sources": {path: value for path, value in sources.items() if value},
        "target_march": f"nash-{march_match.group(1)}" if march_match else None,
        "target_sku": f"journey6{sku_match.group(1)}" if sku_match else None,
        "runtime_abi": abi_match.group(1) if abi_match else None,
        "runtime_version": version_match.group(1) if version_match else None,
    }


def collect() -> dict:
    model = _read("/proc/device-tree/model") or _read("/sys/firmware/devicetree/base/model")
    compatible = _read("/proc/device-tree/compatible") or _read("/sys/firmware/devicetree/base/compatible")
    identity = " ".join(filter(None, (model, compatible))).lower()
    journey6_evidence = [token for token in ("journey 6", "journey6", "j6", "nash") if token in identity]
    forbidden = [token for token in ("rdk", "s100", "s100p", "s600") if token in identity]
    architecture = platform.machine().lower()
    filesystem = shutil.disk_usage("/") if os.name != "nt" else shutil.disk_usage(Path.cwd().anchor)
    release = _release_facts()
    runtime = {
        "abi": release["runtime_abi"],
        "version": release["runtime_version"],
        "hb_model_info": _command(["hb_model_info", "--version"]),
        "hb_verifier": _command(["hb_verifier", "--version"]),
        "hbdk_runtime": _command(["hbdk-runtime", "--version"]),
        "ldconfig_horizon": _command(["sh", "-c", "ldconfig -p 2>/dev/null | grep -Ei 'hbrt|dnn|horizon|hucp'"]),
    }
    thermal = []
    for path in sorted(Path("/sys/class/thermal").glob("thermal_zone*")) if Path("/sys/class/thermal").exists() else []:
        thermal.append({"zone": path.name, "type": _read(str(path / "type")), "temp_millicelsius": _read(str(path / "temp"))})
    match = bool(journey6_evidence and architecture in ("aarch64", "arm64") and not forbidden)
    blockers = []
    if not match:
        blockers.append("journey6_board_identity_not_confirmed")
    if not any(runtime.values()):
        blockers.append("journey6_runtime_capability_not_observed")
    return {
        "schema_version": 1,
        "status": "ready" if not blockers else "blocked_external",
        "target_family": "journey6",
        "target_sku": release["target_sku"] or "auto",
        "target_march": release["target_march"] or "auto",
        "board": {
            "model": model,
            "compatible": compatible,
            "soc_id": _read("/sys/devices/soc0/soc_id"),
            "revision": _read("/sys/devices/soc0/revision"),
            "journey6_identity_evidence": journey6_evidence,
            "forbidden_family_evidence": forbidden,
        },
        "os": {
            "architecture": architecture,
            "kernel": platform.release(),
            "platform": platform.platform(),
            "bsp_version": _read("/etc/bsp_version") or _read("/etc/os-release"),
            "journey6_release_facts": release["sources"],
        },
        "resources": {
            "cpu_count": os.cpu_count(),
            "memory_bytes": _memory_bytes(),
            "storage_total_bytes": filesystem.total,
            "storage_free_bytes": filesystem.free,
            "network": _command(["ip", "-j", "address"]),
            "camera_devices": sorted(str(path) for path in Path("/dev").glob("video*")) if Path("/dev").exists() else [],
            "thermal_zones": thermal,
        },
        "runtime": runtime,
        "blockers": blockers,
        "truth_boundary": "Inventory is read-only and does not infer SKU or march from a product name.",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="J6_BOARD_INVENTORY.json")
    args = parser.parse_args()
    report = collect()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "ready" else 2


if __name__ == "__main__":
    raise SystemExit(main())
