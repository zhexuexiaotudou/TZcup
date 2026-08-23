#!/usr/bin/env python3
"""Plan or apply bounded network faults inside the J6 container namespace."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import subprocess


@dataclass(frozen=True)
class FaultPlan:
    profile: str
    interface: str
    command: tuple[str, ...]


def build_fault_plan(profile: str, interface: str = "eth0") -> FaultPlan:
    if not re.fullmatch(r"[A-Za-z0-9_.:-]+", interface):
        raise ValueError("invalid network interface")
    profiles = {
        "normal": ("tc", "qdisc", "del", "dev", interface, "root"),
        "delay": (
            "tc", "qdisc", "replace", "dev", interface, "root", "netem",
            "delay", "120ms", "40ms", "distribution", "normal",
        ),
        "loss": (
            "tc", "qdisc", "replace", "dev", interface, "root", "netem",
            "loss", "15%", "5%",
        ),
        "bandwidth": (
            "tc", "qdisc", "replace", "dev", interface, "root", "tbf",
            "rate", "8mbit", "burst", "64kb", "latency", "100ms",
        ),
        "disconnect": (
            "tc", "qdisc", "replace", "dev", interface, "root", "netem",
            "loss", "100%",
        ),
    }
    if profile not in profiles:
        raise ValueError(f"unsupported fault profile: {profile}")
    return FaultPlan(profile=profile, interface=interface, command=profiles[profile])


def execute_plan(plan: FaultPlan, *, apply: bool) -> dict[str, object]:
    result: dict[str, object] = {
        "schema_version": 1,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        **asdict(plan),
        "applied": False,
        "returncode": None,
        "stdout": "",
        "stderr": "",
    }
    result["command"] = list(plan.command)
    if not apply:
        return result
    completed = subprocess.run(plan.command, text=True, capture_output=True, check=False)
    # Deleting a non-existent qdisc is an idempotent successful restore.
    idempotent_restore = plan.profile == "normal" and completed.returncode == 2
    result.update(
        applied=completed.returncode == 0 or idempotent_restore,
        returncode=completed.returncode,
        stdout=completed.stdout,
        stderr=completed.stderr,
    )
    if not result["applied"]:
        raise RuntimeError(
            f"network fault command failed with exit code {completed.returncode}: "
            f"{completed.stderr.strip()}"
        )
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "profile", choices=("normal", "delay", "loss", "bandwidth", "disconnect")
    )
    parser.add_argument("--interface", default="eth0")
    parser.add_argument(
        "--apply",
        action="store_true",
        help="execute tc; without this flag the command is dry-run only",
    )
    parser.add_argument("--evidence", type=Path)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    report = execute_plan(
        build_fault_plan(args.profile, interface=args.interface), apply=args.apply
    )
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.evidence:
        args.evidence.parent.mkdir(parents=True, exist_ok=True)
        args.evidence.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
