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
STATUS_MESSAGE_TYPE = "std_msgs/msg/String"


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
    topic_type = re.search(r"^Type:\s*(\S+)\s*$", topic_info, re.MULTILINE)
    publisher_count = re.search(
        r"^Publisher count:\s*(\d+)\s*$", topic_info, re.MULTILINE
    )
    if (
        topic_type is None
        or topic_type.group(1) != STATUS_MESSAGE_TYPE
        or publisher_count is None
        or int(publisher_count.group(1)) != 1
    ):
        raise CaptureError("status topic must have exactly one expected String publisher")
    patterns = (
        re.compile(
            r"Node name:\s*(?P<name>\S+)\s*Node namespace:\s*(?P<namespace>\S+)"
            r"(?:(?!Node name:).)*?Endpoint type:\s*PUBLISHER",
            re.DOTALL,
        ),
        re.compile(
            r"Endpoint type:\s*PUBLISHER(?:(?!Endpoint type:|Node name:).)*?"
            r"Node name:\s*(?P<name>\S+).*?Node namespace:\s*(?P<namespace>\S+)",
            re.DOTALL,
        ),
    )
    matches = [match for pattern in patterns for match in pattern.finditer(topic_info)]
    if len(matches) != 1 or matches[0].group("name") != EXPECTED_PRODUCER_NODE:
        raise CaptureError("status topic has no unique whole_vehicle_safety_manager publisher")
    return {
        "node_name": matches[0].group("name"),
        "node_namespace": matches[0].group("namespace"),
        "topic": STATUS_TOPIC,
        "message_type": STATUS_MESSAGE_TYPE,
        "publisher_count": "1",
    }


def _snapshot_identity(path: Path) -> dict[str, str]:
    snapshot = _json(path)
    outputs = snapshot.get("outputs")
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf") if isinstance(outputs, dict) else None
    if not isinstance(urdf, dict) or not isinstance(urdf.get("sha256"), str):
        raise CaptureError("snapshot lacks expanded URDF identity")
    source = snapshot.get("source_inventory_sha256")
    if not isinstance(source, str) or not source:
        raise CaptureError("snapshot lacks source inventory identity")
    return {
        "snapshot_manifest_sha256": _sha256(path),
        "source_inventory_sha256": source,
        "expanded_urdf_sha256": urdf["sha256"],
    }


def _require_runtime_binding(
    path: Path, snapshot_path: Path, session_path: Path,
    closure_path: Path, runtime_install: Path,
) -> dict[str, Any]:
    binding = _json(path)
    snapshot = _snapshot_identity(snapshot_path)
    session = _json(session_path)
    _json(closure_path)
    session_binding = binding.get("acceptance_session_binding")
    closure_binding = binding.get("runtime_closure_binding")
    if (
        binding.get("status") != "FORMAL_RUNTIME_GATE_BOUND"
        or not isinstance(session_binding, dict)
        or not isinstance(closure_binding, dict)
        or session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
        or session.get("snapshot") != snapshot
        or session_binding.get("session_manifest") != str(session_path.resolve())
        or session_binding.get("session_manifest_sha256") != _sha256(session_path)
        or session_binding.get("session_started_epoch_ns") != session.get("started_epoch_ns")
        or session_binding.get("session_status_at_gate") != session.get("status")
        or session_binding.get("snapshot") != snapshot
        or session_binding.get("snapshot_current_source_verified") is not True
        or closure_binding.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"
        or closure_binding.get("manifest_sha256") != _sha256(closure_path)
        or closure_binding.get("runtime_install_root") != str(runtime_install.resolve())
        or not runtime_install.is_dir()
        or session.get("runtime_closure_binding") != closure_binding
    ):
        raise CaptureError("runtime binding does not match current session/snapshot/closure/install")
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
        binding = _require_runtime_binding(
            args.runtime_binding, args.snapshot, args.session,
            args.runtime_closure, args.runtime_install,
        )
        receipt["runtime_gate_binding_sha256"] = _sha256(args.runtime_binding)
        receipt["runtime_gate_binding"] = binding
        producer_before = _capture(
            ["ros2", "topic", "info", STATUS_TOPIC, "--verbose"], args.timeout_sec
        )
        status_capture = _capture(
            ["ros2", "topic", "echo", "--once", "--field", "data", STATUS_TOPIC],
            args.timeout_sec,
        )
        producer_capture = _capture(
            ["ros2", "topic", "info", STATUS_TOPIC, "--verbose"], args.timeout_sec
        )
        receipt["producer_capture_before"] = producer_before
        receipt["status_capture"] = status_capture
        receipt["producer_capture"] = producer_capture
        if producer_before["returncode"] != 0 or status_capture["returncode"] != 0:
            raise CaptureError("live safety status capture failed")
        if producer_capture["returncode"] != 0:
            raise CaptureError("live safety producer identity capture failed")
        status = json.loads(str(status_capture["stdout"]).strip())
        if not isinstance(status, dict):
            raise CaptureError("live safety status sample must be a JSON object")
        before_identity = _producer_identity(str(producer_before["stdout"]))
        after_identity = _producer_identity(str(producer_capture["stdout"]))
        if before_identity != after_identity:
            raise CaptureError("status producer identity changed across the capture")
        receipt["producer_identity"] = after_identity
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
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--runtime-closure", type=Path, required=True)
    parser.add_argument("--runtime-install", type=Path, required=True)
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
