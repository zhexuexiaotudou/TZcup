#!/usr/bin/env python3
"""Drive the Gazebo service-door evaluation interface and record JointState."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import time
from pathlib import Path
from typing import Any

from formal_runtime_gate_binding import load_binding
from validate_formal_service_door_runtime import DOORS, evaluate


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "artifacts/formal_service_door_runtime.json"
DEFAULT_SNAPSHOT = ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
DEFAULT_SESSION = ROOT / "artifacts/formal_final_acceptance_session.json"
PLUGIN_DIAGNOSTIC_PREFIX = "SERVICE_DOOR_DIAGNOSTIC "


def _parse_plugin_diagnostics(path: Path) -> dict[str, Any]:
    """Retain plugin telemetry from this fresh launch without treating it as motion proof."""

    result: dict[str, Any] = {"launch_log": str(path), "records": []}
    if not path.is_file():
        result["parse_error"] = "launch_log_missing"
        return result
    numeric_keys = {
        "sim_time_sec", "received_hinge_messages", "received_latch_messages",
        "received_hinge_target_rad", "received_latch_target_rad",
        "requested_hinge_rad", "requested_latch_rad", "effective_hinge_rad",
        "effective_latch_rad", "hinge_position_rad", "latch_position_rad",
        "hinge_force_nm", "latch_force_nm", "hinge_force_writes",
        "latch_force_writes", "postupdate_hinge_force_present",
        "postupdate_latch_force_present", "postupdate_hinge_force_nm",
        "postupdate_latch_force_nm",
    }
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        marker = line.find(PLUGIN_DIAGNOSTIC_PREFIX)
        if marker < 0:
            continue
        fields = dict(re.findall(r"([a-z_]+)=([^\s]+)", line[marker:]))
        if "door" not in fields:
            continue
        record: dict[str, Any] = {"door": fields.pop("door")}
        try:
            record.update({key: float(value) for key, value in fields.items() if key in numeric_keys})
        except ValueError as exc:
            result["parse_error"] = f"invalid_numeric_field:{exc}"
            return result
        result["records"].append(record)
    return result


def _snapshot_binding(path: Path) -> dict[str, str]:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    outputs = manifest.get("outputs", {})
    output = outputs.get("reports/engineering/formal_competition_vehicle.urdf", {})
    source_hash = manifest.get("source_inventory_sha256")
    urdf_hash = output.get("sha256") if isinstance(output, dict) else None
    if not isinstance(source_hash, str) or not source_hash:
        raise ValueError("snapshot has no source_inventory_sha256")
    if not isinstance(urdf_hash, str) or not urdf_hash:
        raise ValueError("snapshot has no expanded URDF sha256")
    return {
        "snapshot_manifest_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": urdf_hash,
    }


def _bound_runtime_evidence(
    snapshot_path: Path, session_path: Path, binding_path: Path
) -> tuple[dict[str, str], dict[str, object], dict[str, object]]:
    """Reject door evidence detached from the current formal session."""

    source_binding = _snapshot_binding(snapshot_path)
    session = json.loads(session_path.read_text(encoding="utf-8"))
    if not isinstance(session, dict) or session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise ValueError("formal acceptance session must be RUNNING")
    started_epoch_ns = session.get("started_epoch_ns")
    if not isinstance(started_epoch_ns, int) or started_epoch_ns <= 0:
        raise ValueError("formal acceptance session start time is invalid")
    binding = load_binding(binding_path)
    bound_session = binding.get("acceptance_session_binding")
    if not isinstance(bound_session, dict):
        raise ValueError("runtime binding has no acceptance-session binding")
    if bound_session.get("snapshot") != source_binding:
        raise ValueError("runtime binding snapshot differs from service-door source binding")
    if (
        bound_session.get("session_manifest_sha256")
        != hashlib.sha256(session_path.read_bytes()).hexdigest()
        or bound_session.get("session_started_epoch_ns") != started_epoch_ns
    ):
        raise ValueError("runtime binding session differs from service-door session")
    return source_binding, bound_session, binding


def run(
    output: Path,
    snapshot: Path,
    session: Path,
    runtime_binding: Path,
    plugin_diagnostic_log: Path,
    startup_timeout_s: float,
    phase_duration_s: float,
) -> int:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64

    expected_joints = {item for spec in DOORS.values() for item in spec[:2]}

    class Collector(Node):
        def __init__(self) -> None:
            super().__init__("formal_service_door_runtime_collector")
            self.latest: dict[str, float] = {}
            self.active_samples: list[dict[str, Any]] | None = None
            # `Node.publishers` is an rclpy read-only property.  Keep the
            # evaluator-owned handles under a distinct name.
            self.target_publishers = {
                (door, kind): self.create_publisher(
                    Float64,
                    f"/formal_vehicle/evaluation/bodywork_service/{door}/{kind}_target_rad",
                    10,
                )
                for door in DOORS
                for kind in ("hinge", "latch")
            }
            self.create_subscription(JointState, "/joint_states", self._on_joint_state, 50)

        def target_subscription_counts(self) -> dict[str, int]:
            return {
                f"{door}_{kind}": publisher.get_subscription_count()
                for (door, kind), publisher in sorted(self.target_publishers.items())
            }

        def _on_joint_state(self, message: JointState) -> None:
            positions = dict(zip(message.name, message.position))
            if not expected_joints <= set(positions):
                return
            self.latest = {name: float(positions[name]) for name in expected_joints}
            if self.active_samples is not None:
                self.active_samples.append(
                    {
                        "received_monotonic_ns": time.monotonic_ns(),
                        "positions_rad": dict(sorted(self.latest.items())),
                    }
                )

        def phase(self, targets: dict[str, dict[str, float]], duration_s: float) -> dict[str, Any]:
            subscription_counts = self.target_subscription_counts()
            if not all(subscription_counts.values()):
                raise RuntimeError(
                    "service-door evaluator targets lost ROS bridge subscribers: "
                    + json.dumps(subscription_counts, sort_keys=True)
                )
            self.active_samples = []
            deadline = time.monotonic() + duration_s
            while rclpy.ok() and time.monotonic() < deadline:
                for door, row in targets.items():
                    self.target_publishers[(door, "hinge")].publish(Float64(data=row["hinge"]))
                    self.target_publishers[(door, "latch")].publish(Float64(data=row["latch"]))
                rclpy.spin_once(self, timeout_sec=0.05)
            samples = self.active_samples
            self.active_samples = None
            return {
                "commanded_targets_rad": targets,
                "ros_publisher_subscription_counts": subscription_counts,
                "joint_state_samples": samples,
            }

    def targets(hinge: str, latch: float) -> dict[str, dict[str, float]]:
        return {
            door: {"hinge": (spec[4] if hinge == "open" else 0.0), "latch": latch}
            for door, spec in DOORS.items()
        }

    source_binding, acceptance_session_binding, binding = _bound_runtime_evidence(
        snapshot, session, runtime_binding
    )

    rclpy.init()
    node = Collector()
    phases: dict[str, Any] = {}
    raw: dict[str, Any] = {
        "source_binding": source_binding,
        "evidence_authority": "GAZEBO_SENSOR_MSGS_JOINT_STATE",
        "phases": phases,
    }
    try:
        deadline = time.monotonic() + startup_timeout_s
        while rclpy.ok() and set(node.latest) != expected_joints and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if set(node.latest) != expected_joints:
            raise TimeoutError("all eight physical service-door joints did not appear on /joint_states")
        target_subscription_deadline = time.monotonic() + startup_timeout_s
        target_subscription_counts = node.target_subscription_counts()
        while (
            rclpy.ok()
            and not all(target_subscription_counts.values())
            and time.monotonic() < target_subscription_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
            target_subscription_counts = node.target_subscription_counts()
        if not all(target_subscription_counts.values()):
            raise TimeoutError(
                "all service-door evaluator targets need a ROS bridge subscriber: "
                + json.dumps(target_subscription_counts, sort_keys=True)
            )
        raw["target_transport"] = {
            "ros_publisher_subscription_counts": target_subscription_counts,
        }
        phases["initial_locked"] = node.phase(targets("closed", 0.0), phase_duration_s)
        phases["locked_open_rejected"] = node.phase(targets("open", 0.0), phase_duration_s)
        phases["unlocked"] = node.phase(targets("closed", 0.6), phase_duration_s)
        phases["open"] = node.phase(targets("open", 0.6), phase_duration_s * 1.5)
        phases["closed_unlocked"] = node.phase(targets("closed", 0.6), phase_duration_s * 1.5)
        phases["transport_locked"] = node.phase(targets("closed", 0.0), phase_duration_s)
        phases["relock_open_rejected"] = node.phase(targets("open", 0.0), phase_duration_s)
    except Exception as exc:
        raw["collector_error"] = str(exc)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    raw["plugin_diagnostics"] = _parse_plugin_diagnostics(plugin_diagnostic_log)
    report = evaluate(raw)
    report["runtime_gate_binding"] = binding
    report["acceptance_session_binding"] = acceptance_session_binding
    report["runtime_closure_binding"] = binding["runtime_closure_binding"]
    if "collector_error" in raw:
        report["collector_error"] = raw["collector_error"]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, sort_keys=True))
    return 0 if report["passed"] else 2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--snapshot-manifest", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument("--plugin-diagnostic-log", type=Path, required=True)
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    parser.add_argument("--phase-duration", type=float, default=2.5)
    args = parser.parse_args()
    return run(
        args.output,
        args.snapshot_manifest,
        args.session,
        args.runtime_binding,
        args.plugin_diagnostic_log,
        args.startup_timeout,
        args.phase_duration,
    )


if __name__ == "__main__":
    raise SystemExit(main())
