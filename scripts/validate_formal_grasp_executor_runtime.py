#!/usr/bin/env python3
"""Drive the product grasp request/result interface in the physical cube world.

The surrounding launch owns environment initialization.  This probe publishes
operator safety controls, a zero base request and one truth-free perception
request.  It observes (but never publishes) the product safety permit and has
no entity pose, delete, model-name or direct actuator command interface.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import time
from pathlib import Path
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from formal_runtime_gate_binding import load_binding


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SNAPSHOT = ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
DEFAULT_SESSION = ROOT / "artifacts/formal_final_acceptance_session.json"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _source_binding(snapshot_path: Path) -> dict[str, str]:
    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    outputs = snapshot.get("outputs", {})
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf", {})
    source_hash = snapshot.get("source_inventory_sha256")
    urdf_hash = urdf.get("sha256") if isinstance(urdf, dict) else None
    if not isinstance(source_hash, str) or not source_hash:
        raise ValueError("snapshot has no source_inventory_sha256")
    if not isinstance(urdf_hash, str) or not urdf_hash:
        raise ValueError("snapshot has no expanded URDF sha256")
    return {
        "snapshot_manifest_sha256": _sha256(snapshot_path),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": urdf_hash,
    }


def _active_overlay_identity(expected_install_root: Path) -> dict[str, object]:
    """Prove this probe resolves the grasp package from the frozen overlay."""

    expected = expected_install_root.resolve(strict=True)
    if expected.is_symlink():
        raise ValueError("bound frozen runtime install root is a symbolic link")
    prefixes = [
        Path(value).resolve(strict=True)
        for value in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
        if value
    ]
    if not prefixes:
        raise ValueError("AMENT_PREFIX_PATH is empty after frozen-overlay setup")
    package_marker = Path("share/ament_index/resource_index/packages/sanitation_manipulation")
    resolved_package_prefix = next(
        (prefix for prefix in prefixes if (prefix / package_marker).is_file()), None
    )
    if resolved_package_prefix != expected:
        raise ValueError(
            "sanitation_manipulation resolves outside the bound frozen overlay: "
            f"{resolved_package_prefix} != {expected}"
        )
    return {
        "active_ament_prefix_first": str(prefixes[0]),
        "resolved_sanitation_manipulation_prefix": str(resolved_package_prefix),
        "expected_runtime_install_root": str(expected),
        "active_overlay_matches_runtime_binding": True,
    }


def _bound_runtime_evidence(
    snapshot_path: Path, session_path: Path, binding_path: Path
) -> tuple[dict[str, str], dict[str, object], dict[str, object], dict[str, object]]:
    """Fail closed unless this invocation is a fresh final-runtime episode."""

    if snapshot_path.resolve() != DEFAULT_SNAPSHOT.resolve():
        raise ValueError("formal grasp runtime requires the canonical vehicle snapshot")
    source_binding = _source_binding(snapshot_path)
    session = json.loads(session_path.read_text(encoding="utf-8"))
    if not isinstance(session, dict) or session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise ValueError("formal acceptance session must be RUNNING")
    started_epoch_ns = session.get("started_epoch_ns")
    if (
        not isinstance(started_epoch_ns, int)
        or started_epoch_ns <= 0
        or started_epoch_ns > time.time_ns()
    ):
        raise ValueError("formal acceptance session start time is invalid")

    runtime_binding = load_binding(binding_path)
    bound_session = runtime_binding["acceptance_session_binding"]
    closure = runtime_binding["runtime_closure_binding"]
    if not isinstance(bound_session, dict) or not isinstance(closure, dict):
        raise ValueError("runtime binding is incomplete")
    if bound_session.get("snapshot") != source_binding:
        raise ValueError("runtime binding snapshot differs from grasp source binding")
    if (
        bound_session.get("session_manifest_sha256") != _sha256(session_path)
        or bound_session.get("session_started_epoch_ns") != started_epoch_ns
    ):
        raise ValueError("runtime binding session differs from grasp session")
    verified_epoch_ns = runtime_binding.get("verified_epoch_ns")
    if (
        not isinstance(verified_epoch_ns, int)
        or verified_epoch_ns < started_epoch_ns
        or verified_epoch_ns > time.time_ns()
        or binding_path.stat().st_mtime_ns < started_epoch_ns
    ):
        raise ValueError("runtime binding is not fresh for the active acceptance session")
    expected_install = closure.get("runtime_install_root")
    if not isinstance(expected_install, str) or not expected_install:
        raise ValueError("runtime closure has no frozen install identity")
    for key in ("manifest_sha256", "closure_sha256"):
        value = closure.get(key)
        if not isinstance(value, str) or len(value) != 64:
            raise ValueError(f"runtime closure has invalid {key}")
    if closure.get("symbolic_link_count") != 0:
        raise ValueError("runtime closure allows symbolic links")
    active_overlay = _active_overlay_identity(Path(expected_install))
    runtime_identity: dict[str, object] = {
        "runtime_gate_binding_path": str(binding_path.resolve()),
        "runtime_gate_binding_sha256": _sha256(binding_path),
        "runtime_gate_binding_verified_epoch_ns": verified_epoch_ns,
        "acceptance_session_manifest_sha256": bound_session["session_manifest_sha256"],
        "snapshot": source_binding,
        "runtime_closure": {
            key: closure[key]
            for key in (
                "manifest",
                "manifest_sha256",
                "closure_sha256",
                "runtime_install_root",
                "runtime_package_count",
                "symbolic_link_count",
                "status",
            )
        },
        "active_overlay": active_overlay,
    }
    return source_binding, bound_session, runtime_binding, runtime_identity


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("formal_grasp_executor_runtime_probe")
        self.estop_command = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/emergency_stop", 10
        )
        self.estop_reset_command = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/emergency_stop_reset", 10
        )
        self.main_power_command = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/main_power", 10
        )
        self.safe_zero_command = self.create_publisher(Twist, "/cmd_vel_gate", 10)
        self.request = self.create_publisher(String, "/active_cleaning/grasp_request", 10)
        self.wrist_recheck = self.create_publisher(
            String, "/perception/wrist/grasp_recheck", 10
        )
        self.create_subscription(
            Bool, "/safety/actuators_enabled", self._safety_permit, 10
        )
        self.create_subscription(String, "/active_cleaning/grasp_result", self._result, 10)
        self.create_subscription(
            DiagnosticArray, "/manipulation/formal_grasp_status", self._status, 10
        )
        self.create_subscription(
            DiagnosticArray, "/safety/status", self._safety_status, 10
        )
        self.result: dict | None = None
        self.safety_permit: bool | None = None
        self.status_events: list[dict] = []
        self.safety_status_events: list[dict] = []
        self.create_timer(0.05, self._heartbeat)

    def _heartbeat(self) -> None:
        # These are operator controls and a zero vehicle request, not direct
        # actuator authority. The whole-vehicle safety manager remains the
        # sole publisher of /safety/actuators_enabled and the base command.
        self.estop_command.publish(Bool(data=False))
        self.estop_reset_command.publish(Bool(data=True))
        self.main_power_command.publish(Bool(data=True))
        self.safe_zero_command.publish(Twist())

    def _result(self, message: String) -> None:
        self.result = json.loads(message.data)

    def _safety_permit(self, message: Bool) -> None:
        self.safety_permit = bool(message.data)

    def _status(self, message: DiagnosticArray) -> None:
        for row in message.status:
            if row.name == "formal_physical_grasp_executor":
                self.status_events.append(
                    {
                        "state": row.message,
                        "values": {item.key: item.value for item in row.values},
                    }
                )

    def _safety_status(self, message: DiagnosticArray) -> None:
        for row in message.status:
            if row.name == "whole_vehicle_safety":
                self.safety_status_events.append(
                    {
                        "state": row.message,
                        "values": {item.key: item.value for item in row.values},
                    }
                )
                if len(self.safety_status_events) > 200:
                    del self.safety_status_events[:-200]

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--runtime-binding", required=True, type=Path)
    parser.add_argument("--startup-wait", type=float, default=15.0)
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"refusing to overwrite retained grasp runtime report: {args.output}")
    (
        source_binding,
        acceptance_session_binding,
        runtime_gate_binding,
        runtime_identity,
    ) = _bound_runtime_evidence(args.snapshot, args.session, args.runtime_binding)
    rclpy.init()
    node = Probe()
    report = {
        "report_id": "tzcup_formal_product_grasp_executor_runtime_v1",
        "status": "FORMAL_PRODUCT_GRASP_AND_DRY_BIN_ACCEPTANCE_FAILED",
        "passed": False,
        "truth_used_for_control": False,
        "environment_initialization_owned_by_probe": False,
        "source_binding": source_binding,
        "acceptance_session_binding": acceptance_session_binding,
        "runtime_gate_binding": runtime_gate_binding,
        "runtime_identity": runtime_identity,
    }
    try:
        startup_deadline = time.monotonic() + args.startup_wait
        while rclpy.ok() and time.monotonic() < startup_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.safety_permit is True and node.request.get_subscription_count() > 0:
                break
        if node.safety_permit is not True:
            raise RuntimeError(
                "whole-vehicle safety manager did not issue an actuator permit "
                "before the truth-free grasp request; "
                f"last_status={node.safety_status_events[-1] if node.safety_status_events else None}"
            )
        if node.request.get_subscription_count() < 1:
            raise RuntimeError("formal grasp executor request subscriber is unavailable")
        request = {
            "schema_version": 2,
            "target_id": "perception-track-runtime-001",
            "frame_id": "base_footprint",
            "pose": {
                "x_m": 0.300,
                "y_m": -0.950,
                "z_m": 0.017,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
            },
            "size_m": [0.030, 0.030, 0.030],
            "material": "unknown",
            "confidence": 0.99,
            "truth_used": False,
        }
        node.request.publish(String(data=json.dumps(request, sort_keys=True)))
        deadline = time.monotonic() + args.timeout
        wrist_recheck_sent = False
        while rclpy.ok() and node.result is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if (
                not wrist_recheck_sent
                and any(
                    event["values"].get("reason")
                    == "awaiting_wrist_near_field_recheck"
                    for event in node.status_events
                )
            ):
                node.wrist_recheck.publish(
                    String(data=json.dumps(request, sort_keys=True))
                )
                wrist_recheck_sent = True
        if node.result is None:
            raise TimeoutError("formal grasp executor did not publish a terminal result")
        report.update(
            {
                "passed": node.result.get("verified_in_bin") is True,
                "result": node.result,
                "status_events": node.status_events,
                "safety_status_events": node.safety_status_events,
                "wrist_recheck_sent": wrist_recheck_sent,
            }
        )
        if report["passed"]:
            report["status"] = "FORMAL_PRODUCT_GRASP_AND_DRY_BIN_ACCEPTANCE_PASSED"
        if not report["passed"]:
            raise RuntimeError(f"physical product grasp failed: {node.result}")
    except Exception as exc:
        report["error"] = str(exc)
        report["status_events"] = node.status_events
        report["safety_status_events"] = node.safety_status_events
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
