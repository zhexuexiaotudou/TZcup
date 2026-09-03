#!/usr/bin/env python3
"""Attest that live sensor Gazebo/bridge processes use loopback transport."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import time
from pathlib import Path
from typing import Any, Callable


REPORT_ID = "tzcup_formal_sensor_loopback_attestation_v1"
PASS = "FORMAL_SENSOR_LOOPBACK_TRANSPORT_ATTESTED"
BLOCKED = "FORMAL_SENSOR_LOOPBACK_TRANSPORT_BLOCKED"


class AttestationError(RuntimeError):
    pass


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def regular(path: Path, label: str) -> Path:
    if path.is_symlink() or not path.is_file():
        raise AttestationError(f"{label} must be a regular file: {path}")
    return path.resolve()


def _decode_environment(raw: bytes) -> dict[str, str]:
    result: dict[str, str] = {}
    for item in raw.split(b"\0"):
        if not item or b"=" not in item:
            continue
        key, value = item.split(b"=", 1)
        result[key.decode("utf-8", errors="replace")] = value.decode(
            "utf-8", errors="replace"
        )
    return result


def _role(cmdline: str) -> str | None:
    if "parameter_bridge" in cmdline:
        return "ros_gz_parameter_bridge"
    if "gzserver" in cmdline or re.search(r"(?:^|[/\s])gz\s+sim(?:\s|$)", cmdline):
        return "gazebo_sim"
    return None


def scan_partition(proc_root: Path, partition: str) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    needle = f"GZ_PARTITION={partition}".encode()
    for entry in sorted(proc_root.iterdir(), key=lambda path: path.name):
        if not entry.name.isdigit():
            continue
        try:
            raw_env = (entry / "environ").read_bytes()
            if needle not in raw_env.split(b"\0"):
                continue
            raw_cmdline = (entry / "cmdline").read_bytes()
        except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
            continue
        cmdline = " ".join(
            token.decode("utf-8", errors="replace")
            for token in raw_cmdline.split(b"\0")
            if token
        )
        role = _role(cmdline)
        if role is None:
            continue
        rows.append(
            {
                "pid": int(entry.name),
                "role": role,
                "cmdline": cmdline,
                "environment": _decode_environment(raw_env),
            }
        )
    return rows


def attest(
    *,
    partition: str,
    expected_cyclonedds_uri: str,
    session: Path,
    closure_manifest: Path,
    timeout_s: float,
    proc_root: Path = Path("/proc"),
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    if not partition or any(character.isspace() for character in partition):
        raise AttestationError("GZ partition is empty or contains whitespace")
    session = regular(session, "acceptance session")
    closure_manifest = regular(closure_manifest, "runtime closure manifest")
    try:
        session_value = json.loads(session.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise AttestationError(f"cannot read acceptance session: {exc}") from exc
    if (
        not isinstance(session_value, dict)
        or session_value.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
    ):
        raise AttestationError("acceptance session is not RUNNING")

    deadline = monotonic() + timeout_s
    rows: list[dict[str, Any]] = []
    while True:
        rows = scan_partition(proc_root, partition)
        roles = {row["role"] for row in rows}
        if {"gazebo_sim", "ros_gz_parameter_bridge"}.issubset(roles):
            break
        if monotonic() >= deadline:
            break
        sleep(min(0.25, max(0.0, deadline - monotonic())))

    blockers: list[dict[str, Any]] = []
    roles = {row["role"] for row in rows}
    for required in ("gazebo_sim", "ros_gz_parameter_bridge"):
        if required not in roles:
            blockers.append({"code": "MISSING_PROCESS_ROLE", "role": required})
    required_environment = {
        "GZ_IP": "127.0.0.1",
        "IGN_IP": "127.0.0.1",
        "GZ_PARTITION": partition,
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        "CYCLONEDDS_URI": expected_cyclonedds_uri,
    }
    forbidden_discovery_environment = ("ROS_LOCALHOST_ONLY", "ROS_STATIC_PEERS")
    process_evidence: list[dict[str, Any]] = []
    for row in rows:
        environment = row["environment"]
        mismatches = {
            name: {"expected": expected, "observed": environment.get(name)}
            for name, expected in required_environment.items()
            if environment.get(name) != expected
        }
        relays = {
            name: environment[name]
            for name in ("GZ_RELAY", "IGN_RELAY")
            if name in environment
        }
        inherited_discovery = {
            name: environment[name]
            for name in forbidden_discovery_environment
            if name in environment
        }
        if mismatches:
            blockers.append(
                {
                    "code": "LOOPBACK_ENVIRONMENT_MISMATCH",
                    "pid": row["pid"],
                    "role": row["role"],
                    "mismatches": mismatches,
                }
            )
        if relays:
            blockers.append(
                {
                    "code": "RELAY_ENVIRONMENT_PRESENT",
                    "pid": row["pid"],
                    "role": row["role"],
                    "relays": relays,
                }
            )
        if inherited_discovery:
            blockers.append(
                {
                    "code": "ROS_DISCOVERY_ENVIRONMENT_PRESENT",
                    "pid": row["pid"],
                    "role": row["role"],
                    "environment": inherited_discovery,
                }
            )
        process_evidence.append(
            {
                "pid": row["pid"],
                "role": row["role"],
                "cmdline": row["cmdline"],
                "transport_environment": {
                    name: environment.get(name)
                    for name in (
                        *required_environment,
                        *forbidden_discovery_environment,
                        "GZ_RELAY",
                        "IGN_RELAY",
                    )
                    if name in environment
                },
            }
        )
    passed = not blockers
    return {
        "schema_version": 1,
        "report_id": REPORT_ID,
        "status": PASS if passed else BLOCKED,
        "passed": passed,
        "recorded_epoch_ns": time.time_ns(),
        "gz_partition": partition,
        "expected_transport_environment": required_environment,
        "ros_discovery_variables_required_absent": list(forbidden_discovery_environment),
        "relay_variables_required_absent": ["GZ_RELAY", "IGN_RELAY"],
        "acceptance_session": {"path": str(session), "sha256": sha256(session)},
        "runtime_closure_manifest": {
            "path": str(closure_manifest),
            "sha256": sha256(closure_manifest),
        },
        "processes": process_evidence,
        "blockers": blockers,
    }


def write_exclusive(path: Path, value: dict[str, Any]) -> None:
    if path.exists() or path.is_symlink():
        raise AttestationError(f"refusing stale loopback attestation: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--partition", required=True)
    parser.add_argument("--expected-cyclonedds-uri", required=True)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--closure-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()
    try:
        result = attest(
            partition=args.partition,
            expected_cyclonedds_uri=args.expected_cyclonedds_uri,
            session=args.session,
            closure_manifest=args.closure_manifest,
            timeout_s=args.timeout,
        )
        write_exclusive(args.output, result)
    except (AttestationError, OSError, ValueError) as exc:
        blocked = {
            "schema_version": 1,
            "report_id": REPORT_ID,
            "status": BLOCKED,
            "passed": False,
            "error": str(exc),
        }
        try:
            write_exclusive(args.output, blocked)
        except (AttestationError, OSError):
            pass
        print(json.dumps(blocked, indent=2, sort_keys=True))
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
