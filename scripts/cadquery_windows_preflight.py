#!/usr/bin/env python3
"""Fail closed before a Windows-native CadQuery/OCCT bootstrap.

The CAD package set has large native wheels.  This probe deliberately performs
no installation, process termination, WSL invocation, Docker invocation, or
Gazebo invocation.  It records whether the host has enough *currently free*
physical memory and project-drive space to attempt a local virtual environment.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import os
import platform
import shutil
import struct
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


MIN_FREE_MEMORY_MIB = 4096
MIN_FREE_DISK_MIB = 8192
LOCKED_PYTHON_MINOR = (3, 13)


@dataclass(frozen=True)
class HostProbe:
    is_windows: bool
    free_memory_mib: float | None
    free_disk_mib: float


@dataclass(frozen=True)
class InterpreterProbe:
    executable: str
    implementation: str
    version: tuple[int, int, int]
    bits: int


def mib(byte_count: int) -> float:
    return round(byte_count / (1024 * 1024), 1)


def free_physical_memory_mib() -> float | None:
    """Read Windows free physical memory without starting any CAD process."""

    if os.name != "nt":
        return None

    class MemoryStatusEx(ctypes.Structure):
        _fields_ = [
            ("dwLength", ctypes.c_ulong),
            ("dwMemoryLoad", ctypes.c_ulong),
            ("ullTotalPhys", ctypes.c_ulonglong),
            ("ullAvailPhys", ctypes.c_ulonglong),
            ("ullTotalPageFile", ctypes.c_ulonglong),
            ("ullAvailPageFile", ctypes.c_ulonglong),
            ("ullTotalVirtual", ctypes.c_ulonglong),
            ("ullAvailVirtual", ctypes.c_ulonglong),
            ("ullAvailExtendedVirtual", ctypes.c_ulonglong),
        ]

    status = MemoryStatusEx()
    status.dwLength = ctypes.sizeof(MemoryStatusEx)
    if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
        raise ctypes.WinError()
    return mib(status.ullAvailPhys)


def probe_host(root: Path) -> HostProbe:
    return HostProbe(
        is_windows=os.name == "nt",
        free_memory_mib=free_physical_memory_mib(),
        free_disk_mib=mib(shutil.disk_usage(root).free),
    )


def probe_interpreter() -> InterpreterProbe:
    return InterpreterProbe(
        executable=sys.executable,
        implementation=platform.python_implementation(),
        version=sys.version_info[:3],
        bits=struct.calcsize("P") * 8,
    )


def build_report(
    root: Path,
    *,
    host: HostProbe | None = None,
    interpreter: InterpreterProbe | None = None,
) -> dict[str, Any]:
    """Build a serializable decision.  Injection keeps regression tests local."""

    host = host or probe_host(root)
    interpreter = interpreter or probe_interpreter()
    blockers: list[dict[str, Any]] = []

    if not host.is_windows:
        blockers.append({"code": "WINDOWS_NATIVE_HOST_REQUIRED"})
    if interpreter.implementation != "CPython":
        blockers.append({"code": "CPYTHON_REQUIRED", "observed": interpreter.implementation})
    if interpreter.version[:2] != LOCKED_PYTHON_MINOR:
        blockers.append(
            {
                "code": "LOCKED_CPYTHON_3_13_REQUIRED",
                "observed": ".".join(str(value) for value in interpreter.version),
            }
        )
    if interpreter.bits != 64:
        blockers.append({"code": "WINDOWS_AMD64_REQUIRED", "observed_bits": interpreter.bits})
    if host.free_memory_mib is None:
        blockers.append({"code": "FREE_PHYSICAL_MEMORY_UNAVAILABLE"})
    elif host.free_memory_mib < MIN_FREE_MEMORY_MIB:
        blockers.append(
            {
                "code": "INSUFFICIENT_FREE_PHYSICAL_MEMORY",
                "minimum_mib": MIN_FREE_MEMORY_MIB,
                "observed_mib": host.free_memory_mib,
            }
        )
    if host.free_disk_mib < MIN_FREE_DISK_MIB:
        blockers.append(
            {
                "code": "INSUFFICIENT_PROJECT_DRIVE_SPACE",
                "minimum_mib": MIN_FREE_DISK_MIB,
                "observed_mib": host.free_disk_mib,
            }
        )

    outcome = "ready_to_bootstrap" if not blockers else "blocked"
    return {
        "schema_version": 1,
        "audit_name": "cadquery_windows_bootstrap_preflight",
        "outcome": outcome,
        "bootstrap_permitted": outcome == "ready_to_bootstrap",
        "repository_root": str(root.resolve()),
        "minimums": {
            "free_physical_memory_mib": MIN_FREE_MEMORY_MIB,
            "free_project_drive_mib": MIN_FREE_DISK_MIB,
            "python_implementation": "CPython",
            "python_major_minor": list(LOCKED_PYTHON_MINOR),
            "python_bits": 64,
        },
        "observed": {"host": asdict(host), "interpreter": asdict(interpreter)},
        "blocked_reasons": blockers,
        "scope": {
            "host": "Windows-native only",
            "prohibited_execution_backends": ["WSL", "Docker", "Gazebo"],
            "does_not_change_formal_vehicle_readiness": True,
            "does_not_create_or_validate_a_vehicle_assembly": True,
        },
        "next_safe_action": (
            "Retry only after the listed host resources are free; do not terminate "
            "processes automatically."
            if blockers
            else "Create the project-local .work/cadquery-venv and run the locked bootstrap."
        ),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path, help="optional JSON report path")
    parser.add_argument("--strict", action="store_true", help="fail unless bootstrap is permitted")
    args = parser.parse_args(argv)

    report = build_report(args.root)
    rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return 0 if not args.strict or report["bootstrap_permitted"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
