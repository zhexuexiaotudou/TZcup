#!/usr/bin/env python3
"""Collect a read-only G0 inventory from an RDK S100P.

This collector intentionally has no actuator, ROS publisher, GPIO, CAN write,
or privileged operation.  It records missing tools and devices as evidence
instead of treating their absence as success.
"""

from __future__ import annotations

import argparse
import datetime as dt
import glob
import hashlib
import json
import os
import pathlib
import platform
import shlex
import shutil
import subprocess
from typing import Any


READ_ONLY_COMMANDS: dict[str, list[str]] = {
    "os_release": ["cat", "/etc/os-release"],
    "kernel": ["uname", "-a"],
    "network": ["ip", "-details", "address", "show"],
    "routes": ["ip", "route", "show"],
    "usb": ["lsusb"],
    "pci": ["lspci", "-nn"],
    "block_devices": ["lsblk", "-J", "-o", "NAME,TYPE,SIZE,MODEL,TRAN,MOUNTPOINTS"],
    "loaded_modules": ["lsmod"],
    "dpkg_all_installed": ["dpkg-query", "-W", "-f=${binary:Package}\t${Version}\n"],
}

# This is deliberately separate from generic shell probes.  Only these three
# inspected subcommands may run after sourcing a *fixed* system setup file.
ROS_READ_ONLY_ARGUMENTS: dict[str, tuple[str, ...]] = {
    "packages": ("pkg", "list"),
    "nodes": ("node", "list", "--no-daemon"),
    "topics_with_types": ("topic", "list", "-t", "--no-daemon"),
}

SYSTEM_IMAGE_PATHS = (
    "/etc/os-release",
    "/etc/version",
    "/etc/horizon-release",
    "/etc/d-robotics-release",
    "/proc/cmdline",
)
TROS_SETUP_CANDIDATES = (
    "/opt/tros/setup.bash",
    "/opt/tros/humble/setup.bash",
    "/opt/tros/jazzy/setup.bash",
    "/opt/ros/humble/setup.bash",
    "/opt/ros/jazzy/setup.bash",
)
BPU_TOOLS = (
    "hrt_model_exec",
    "hrt_model_info",
    "hb_model_info",
    "hb_perf",
    "hb_runtime",
)
DEFAULT_OVERLAY_PATHS = (
    "/home/sunrise/tzcup_ws/install/setup.bash",
    "/home/sunrise/TZcup/install/setup.bash",
    "/home/sunrise/ros2_ws/install/setup.bash",
)
DEFAULT_MODEL_DIRECTORIES = (
    "/opt/hobot/model",
    "/opt/tros/lib",
    "/home/sunrise/models",
)
MODEL_SUFFIXES = (".hbm", ".bin", ".onnx", ".yaml", ".yml")

DEVICE_PATTERNS = (
    "/dev/video*",
    "/dev/media*",
    "/dev/ttyUSB*",
    "/dev/ttyACM*",
    "/dev/can*",
    "/dev/i2c-*",
    "/dev/spidev*",
)


def _read_text(path: str) -> dict[str, Any]:
    try:
        raw = pathlib.Path(path).read_bytes()
    except OSError as exc:
        return {"status": "ABSENT", "error": str(exc), "value": None}
    value = raw.rstrip(b"\0").decode("utf-8", errors="replace")
    return {"status": "PRESENT", "value": value}


def _run(command: list[str]) -> dict[str, Any]:
    environment = os.environ.copy()
    environment["ROS2CLI_DISABLE_DAEMON"] = "1"
    try:
        proc = subprocess.run(
            command, capture_output=True, text=True, timeout=20, check=False, env=environment
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {
            "status": "UNAVAILABLE",
            "command": shlex.join(command),
            "error": str(exc),
        }
    return {
        "status": "OK" if proc.returncode == 0 else "ERROR",
        "command": shlex.join(command),
        "returncode": proc.returncode,
        "stdout": proc.stdout,
        "stderr": proc.stderr,
    }


def _path_hash(path: str) -> dict[str, Any]:
    candidate = pathlib.Path(path).expanduser()
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as exc:
        return {"path": str(candidate), "status": "ABSENT", "error": str(exc)}
    if not resolved.is_file():
        return {
            "path": str(resolved),
            "status": "NOT_A_FILE",
            "kind": "directory" if resolved.is_dir() else "other",
        }
    digest = hashlib.sha256()
    with resolved.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return {"path": str(resolved), "status": "PRESENT", "sha256": digest.hexdigest(), "size_bytes": resolved.stat().st_size}


def _tros_setups() -> list[dict[str, Any]]:
    return [_path_hash(path) for path in TROS_SETUP_CANDIDATES]


def _validated_ros_setup() -> pathlib.Path | None:
    """Select the first present fixed setup candidate; never accept CLI input."""

    for candidate in TROS_SETUP_CANDIDATES:
        path = pathlib.Path(candidate)
        try:
            path.resolve(strict=True)
        except OSError:
            continue
        if path.is_file():
            return path
    return None


def _run_sourced_ros(setup: pathlib.Path, arguments: tuple[str, ...]) -> dict[str, Any]:
    if str(setup) not in TROS_SETUP_CANDIDATES or not setup.is_file():
        raise RuntimeError("refusing non-allowlisted ROS/TROS setup")
    if arguments not in ROS_READ_ONLY_ARGUMENTS.values():
        raise RuntimeError("refusing non-read-only ros2 invocation")
    # setup and arguments are hardcoded above. Quoting remains explicit so no
    # user-provided path or shell fragment can alter the sourced command.
    command = f"source {shlex.quote(str(setup))}; exec ros2 {shlex.join(arguments)}"
    result = _run(["/bin/bash", "-lc", command])
    result["setup_path"] = str(setup)
    result["setup_sha256"] = _path_hash(str(setup)).get("sha256")
    return result


def _ros_graph_read_only() -> dict[str, Any]:
    setup = _validated_ros_setup()
    if setup is None:
        unavailable = {
            "status": "ABSENT",
            "error": "no fixed ROS/TROS setup file is present",
        }
        return {
            "ros2_cli": {"status": "ABSENT", "path": None},
            "setup_used": None,
            "packages": unavailable,
            "nodes": unavailable,
            "topics_with_types": unavailable,
            "ros2cli_daemon_disabled": True,
        }
    probes = {
        name: _run_sourced_ros(setup, arguments)
        for name, arguments in ROS_READ_ONLY_ARGUMENTS.items()
    }
    ros2_present = all(probe.get("status") == "OK" for probe in probes.values())
    return {
        "ros2_cli": {
            "status": "PRESENT" if ros2_present else "UNAVAILABLE_AFTER_ALLOWLISTED_SETUP",
            "path": shutil.which("ros2"),
        },
        "setup_used": _path_hash(str(setup)),
        **probes,
        "ros2cli_daemon_disabled": True,
    }


def _bpu_tools() -> dict[str, Any]:
    result: dict[str, Any] = {}
    for tool in BPU_TOOLS:
        executable = shutil.which(tool)
        if executable is None:
            result[tool] = {"status": "ABSENT", "path": None, "version": None}
            continue
        version = _run([executable, "--version"])
        result[tool] = {"status": "PRESENT", "path": executable, "version": version}
    return result


def _relevant_dpkg() -> dict[str, Any]:
    raw = _run(READ_ONLY_COMMANDS["dpkg_all_installed"])
    if raw.get("status") != "OK":
        return {"status": "ABSENT_OR_UNAVAILABLE", "probe": raw, "packages": []}
    keywords = ("tros", "ros-", "hobot", "horizon", "d-robotics", "bpu", "dnn")
    packages = [
        row for row in str(raw.get("stdout", "")).splitlines()
        if any(keyword in row.lower() for keyword in keywords)
    ]
    return {"status": "PRESENT", "probe": raw, "packages": packages}


def _model_files(paths: list[str]) -> list[dict[str, Any]]:
    files: set[str] = set()
    for entry in paths:
        candidate = pathlib.Path(entry).expanduser()
        if candidate.is_file():
            files.add(str(candidate))
        elif candidate.is_dir():
            for suffix in MODEL_SUFFIXES:
                files.update(str(path) for path in candidate.glob(f"*{suffix}"))
        else:
            files.add(str(candidate))
    return [_path_hash(path) for path in sorted(files)]


def collect(*, overlay_paths: list[str] | None = None, model_paths: list[str] | None = None) -> dict[str, Any]:
    overlay_paths = list(DEFAULT_OVERLAY_PATHS if overlay_paths is None else overlay_paths)
    model_paths = list(DEFAULT_MODEL_DIRECTORIES if model_paths is None else model_paths)
    devices = {pattern: sorted(glob.glob(pattern)) for pattern in DEVICE_PATTERNS}
    commands = {name: _run(command) for name, command in READ_ONLY_COMMANDS.items()}
    # The package inventory is summarised separately; retain its raw dpkg query
    # in ``commands`` so absence and command errors remain auditable.
    return {
        "schema_version": 2,
        "report_id": "tzcup_s100p_g0_read_only_software_bpu_ros_inventory_v2",
        "status": "G0_READ_ONLY_INVENTORY_COLLECTED",
        "collected_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "safety": {
            "actuator_commands_sent": False,
            "privileged_commands_used": False,
            "collector_scope": "read_only_identity_system_bpu_ros_overlay_and_model_inventory_only",
            "ros_publish_or_service_calls_sent": False,
            "can_gpio_or_actuator_access_attempted": False,
        },
        "identity": {
            "model": _read_text("/proc/device-tree/model"),
            "compatible": _read_text("/proc/device-tree/compatible"),
            "architecture": platform.machine(),
            "hostname": platform.node(),
        },
        "memory": {
            "meminfo": _read_text("/proc/meminfo"),
        },
        "devices": devices,
        "system_image": {path: _read_text(path) for path in SYSTEM_IMAGE_PATHS},
        "tros_ros_setup_files": _tros_setups(),
        "installed_relevant_dpkg": _relevant_dpkg(),
        "ros_graph_read_only": _ros_graph_read_only(),
        "bpu_runtime_and_tools": _bpu_tools(),
        "project_overlay_files": [_path_hash(path) for path in overlay_paths],
        "model_files_path_and_sha256_only": _model_files(model_paths),
        "commands": commands,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=pathlib.Path)
    parser.add_argument("--overlay-path", action="append", default=[])
    parser.add_argument("--model-path", action="append", default=[])
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"refusing to overwrite retained G0 evidence: {args.output}")
    report = collect(
        overlay_paths=args.overlay_path or None,
        model_paths=args.model_path or None,
    )
    payload = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + f".pending.{os.getpid()}")
    temporary.write_text(payload, encoding="utf-8")
    temporary.replace(args.output)
    digest = hashlib.sha256(payload.encode("utf-8")).hexdigest()
    print(f"S100P_G0_READ_ONLY_INVENTORY_COLLECTED output={args.output} sha256={digest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
