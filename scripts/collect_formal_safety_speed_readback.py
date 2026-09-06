#!/usr/bin/env python3
"""Capture a live, source-bound whole-vehicle safety speed receipt.

The collector deliberately does not accept a raw-status file: it owns the
fixed ROS commands and records their exact outputs, including failures.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import subprocess
import time
from pathlib import Path
from typing import Any


EXPECTED_PRODUCER_NODE = "whole_vehicle_safety_manager"
STATUS_TOPIC = "/safety/status_json"


class CaptureError(RuntimeError):
    pass


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise CaptureError(f"JSON root must be an object: {path}")
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _capture_text(value: str | bytes | None) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value or ""


def _capture(command: list[str], timeout_sec: float) -> dict[str, Any]:
    started = time.time_ns()
    try:
        completed = subprocess.run(
            command, text=True, capture_output=True, timeout=timeout_sec, check=False
        )
        return {
            "command": command,
            "returncode": completed.returncode,
            "stdout": completed.stdout,
            "stderr": completed.stderr,
            "started_epoch_ns": started,
            "completed_epoch_ns": time.time_ns(),
        }
    except subprocess.TimeoutExpired as exc:
        return {
            "command": command,
            "returncode": None,
            "stdout": _capture_text(exc.stdout),
            "stderr": _capture_text(exc.stderr),
            "started_epoch_ns": started,
            "completed_epoch_ns": time.time_ns(),
            "timeout": True,
        }
    except OSError as exc:
        return {
            "command": command,
            "returncode": None,
            "stdout": "",
            "stderr": str(exc),
            "started_epoch_ns": started,
            "completed_epoch_ns": time.time_ns(),
            "spawn_error": type(exc).__name__,
        }


def _producer_identity(topic_info: str) -> dict[str, str]:
    pattern = re.compile(
        r"Endpoint type:\s*PUBLISHER(?:(?!Endpoint type:).)*?"
        r"Node name:\s*(?P<name>\S+).*?Node namespace:\s*(?P<namespace>\S+)",
        re.DOTALL,
    )
    for match in pattern.finditer(topic_info):
        if match.group("name") == EXPECTED_PRODUCER_NODE:
            return {
                "node_name": match.group("name"),
                "node_namespace": match.group("namespace"),
                "topic": STATUS_TOPIC,
            }
    raise CaptureError("status topic has no expected whole_vehicle_safety_manager publisher")


def _require_runtime_binding(path: Path) -> dict[str, Any]:
    binding = _json(path)
    if (
        binding.get("status") != "FORMAL_RUNTIME_GATE_BOUND"
        or binding.get("acceptance_session_binding", {}).get("session_status_at_gate")
        != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
        or binding.get("runtime_closure_binding", {}).get("status")
        != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"
    ):
        raise CaptureError("runtime binding is not a current running formal closure")
    return binding


def collect(args: argparse.Namespace) -> tuple[dict[str, Any], bool]:
    if args.output.exists():
        raise CaptureError("refusing to overwrite safety-manager readback")
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0.0:
        raise CaptureError("timeout must be positive and finite")
    receipt: dict[str, Any] = {
        "schema_version": 2,
        "collector": "collect_formal_safety_speed_readback.py",
        "capture_status": "FAILED",
        "captured_epoch_ns": time.time_ns(),
        "expected": {
            "effective_max_linear_velocity_mps": args.expected_cap,
            "operation_speed_profile": args.expected_profile,
            "speed_qualification_state": args.expected_state,
        },
    }
    try:
        _require_runtime_binding(args.runtime_binding)
        receipt["runtime_gate_binding_sha256"] = _sha256(args.runtime_binding)
        status_capture = _capture(
            ["ros2", "topic", "echo", "--once", "--field", "data", STATUS_TOPIC],
            args.timeout_sec,
        )
        producer_capture = _capture(
            ["ros2", "topic", "info", STATUS_TOPIC, "--verbose"], args.timeout_sec
        )
        receipt["status_capture"] = status_capture
        receipt["producer_capture"] = producer_capture
        if status_capture["returncode"] != 0:
            raise CaptureError("live safety status capture failed")
        if producer_capture["returncode"] != 0:
            raise CaptureError("live safety producer identity capture failed")
        status = json.loads(str(status_capture["stdout"]).strip())
        if not isinstance(status, dict):
            raise CaptureError("live safety status sample must be a JSON object")
        receipt["producer_identity"] = _producer_identity(str(producer_capture["stdout"]))
        cap = status.get("effective_max_linear_velocity_mps")
        if isinstance(cap, bool) or not isinstance(cap, (int, float)) or not math.isclose(
            float(cap), args.expected_cap, abs_tol=1.0e-12
        ):
            raise CaptureError("safety manager effective cap differs from the required live cap")
        if status.get("operation_speed_profile") != args.expected_profile:
            raise CaptureError("safety manager profile readback differs from launch scope")
        if status.get("speed_qualification_state") != args.expected_state:
            raise CaptureError("safety manager qualification state differs from launch scope")
        receipt.update({
            "capture_status": "PASSED",
            "effective_max_linear_velocity_mps": float(cap),
            "operation_speed_profile": status["operation_speed_profile"],
            "speed_qualification_state": status["speed_qualification_state"],
        })
    except (CaptureError, OSError, UnicodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
        receipt["error"] = str(exc)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return receipt, receipt["capture_status"] == "PASSED"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument("--expected-cap", type=float, required=True)
    parser.add_argument("--expected-profile", required=True)
    parser.add_argument("--expected-state", required=True)
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    args = parser.parse_args()
    try:
        _, passed = collect(args)
    except CaptureError as exc:
        print(str(exc))
        return 2
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
