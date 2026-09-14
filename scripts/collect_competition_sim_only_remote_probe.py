#!/usr/bin/env python3
"""Collect a live simulation-host identity probe; run this on the host."""
from __future__ import annotations

import argparse
import json
import platform
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def command(*args: str) -> str:
    return subprocess.check_output(args, text=True, encoding="utf-8").strip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    repo = args.repo.resolve()
    if args.output.exists():
        raise SystemExit("output must not exist")
    source = {
        "head": command("git", "-C", str(repo), "rev-parse", "HEAD"),
        "tree": command("git", "-C", str(repo), "rev-parse", "HEAD^{tree}"),
        "status_porcelain": command("git", "-C", str(repo), "status", "--porcelain=v1"),
    }
    try:
        gpu = command("nvidia-smi", "--query-gpu=name,memory.total,driver_version", "--format=csv,noheader")
    except (FileNotFoundError, subprocess.CalledProcessError):
        gpu = "UNAVAILABLE"
    payload = {
        "schema_version": 1,
        "report_id": "tzcup_competition_sim_only_remote_probe_v1",
        "live_probe": True,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "hostname": platform.node(),
        "architecture": platform.machine(),
        "source": source,
        "gpu": gpu,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
