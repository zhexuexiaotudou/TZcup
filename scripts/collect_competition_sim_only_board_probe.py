#!/usr/bin/env python3
"""Collect a live S100P identity probe through a serial-qualified ADB call."""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--adb", type=Path, required=True)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    adb = str(args.adb.resolve())
    if args.output.exists():
        raise SystemExit("output must not exist")

    def shell(command: str) -> str:
        return subprocess.check_output([adb, "-s", args.serial, "shell", command], text=True, encoding="utf-8").replace("\r", "").strip()

    state = subprocess.check_output([adb, "-s", args.serial, "get-state"], text=True, encoding="utf-8").strip()
    if state != "device":
        raise SystemExit(f"ADB target is not ready: {state}")
    payload = {
        "schema_version": 1,
        "report_id": "tzcup_competition_sim_only_board_probe_v1",
        "live_probe": True,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "transport": "USB_ADB_SERIAL_QUALIFIED",
        "serial_sha256": hashlib.sha256(args.serial.encode()).hexdigest(),
        "model": shell("cat /sys/firmware/devicetree/base/model 2>/dev/null || getprop ro.product.model"),
        "architecture": shell("uname -m"),
        "kernel": shell("uname -r"),
        "bpu_nodes": shell("find /dev -maxdepth 1 -name 'bpu_core*' -type c -print | sort").splitlines(),
        "root_filesystem": shell("df -P / | tail -1"),
        "overlay_setup_sha256": shell("sha256sum /opt/tzcup/s100p/overlay/install/setup.bash 2>/dev/null | awk '{print $1}'"),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
