#!/usr/bin/env python3
"""Drive the Gazebo service-door evaluation interface and record JointState."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from formal_runtime_gate_binding import load_binding
from validate_formal_service_door_runtime import DOORS, evaluate, report_json


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "artifacts/formal_service_door_runtime.json"
DEFAULT_SNAPSHOT = ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
DEFAULT_SESSION = ROOT / "artifacts/formal_final_acceptance_session.json"
PLUGIN_DIAGNOSTIC_PREFIX = "SERVICE_DOOR_DIAGNOSTIC "
PLUGIN_LIFECYCLE_PREFIX = "SERVICE_DOOR_LIFECYCLE "
PHYSICAL_JOINT_STATES_TOPIC = "/formal/service_door_joint_states"
PHYSICAL_JOINT_STATE_AUTHORITY = "GAZEBO_MODEL_JOINT_STATE_BRIDGE"
EXPANDED_URDF_OUTPUT = "reports/engineering/formal_competition_vehicle.urdf"
SNAPSHOT_LOGICAL_PATH = "reports/engineering/formal_vehicle_snapshot_manifest.json"
PLUGIN_ECHO_TOLERANCE_RAD = 1.0e-6


def _load_joint_velocity_limits(snapshot_path: Path) -> dict[str, float]:
    """Read the eight commanded-joint velocity limits from the bound URDF."""

    manifest = json.loads(snapshot_path.read_text(encoding="utf-8"))
    outputs = manifest.get("outputs")
    if not isinstance(outputs, dict):
        raise ValueError("snapshot has no outputs map")
    output = outputs.get(EXPANDED_URDF_OUTPUT)
    if not isinstance(output, dict):
        raise ValueError(f"snapshot has no {EXPANDED_URDF_OUTPUT} output")
    expected_hash = output.get("sha256")
    if not isinstance(expected_hash, str) or not expected_hash:
        raise ValueError("snapshot expanded URDF has no sha256")
    urdf_path = snapshot_path.parents[2] / EXPANDED_URDF_OUTPUT
    if not urdf_path.is_file():
        raise ValueError(f"expanded URDF is missing: {urdf_path}")
    actual_hash = hashlib.sha256(urdf_path.read_bytes()).hexdigest()
    if actual_hash != expected_hash:
        raise ValueError("expanded URDF hash differs from snapshot manifest")
    try:
        root = ET.fromstring(urdf_path.read_text(encoding="utf-8"))
    except ET.ParseError as exc:
        raise ValueError(f"expanded URDF is not valid XML: {exc}") from exc
    expected_joints = {item for spec in DOORS.values() for item in spec[:2]}
    velocity_limits: dict[str, float] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name not in expected_joints:
            continue
        limit = joint.find("limit")
        value = limit.get("velocity") if limit is not None else None
        try:
            velocity = float(value) if value is not None else float("nan")
        except ValueError as exc:
            raise ValueError(f"joint {name} has non-numeric velocity limit") from exc
        if not math.isfinite(velocity) or velocity <= 0.0:
            raise ValueError(f"joint {name} has invalid velocity limit")
        velocity_limits[name] = velocity
    missing = sorted(expected_joints - set(velocity_limits))
    if missing:
        raise ValueError("expanded URDF lacks velocity limits for: " + ", ".join(missing))
    return dict(sorted(velocity_limits.items()))


def _phase_duration_from_targets(
    previous_targets: dict[str, dict[str, float]],
    targets: dict[str, dict[str, float]],
    velocity_limits: dict[str, float],
    minimum_duration_s: float,
    settling_margin_s: float,
) -> float:
    """Bound simulated dwell by URDF speed and the largest commanded travel."""

    if not math.isfinite(minimum_duration_s) or minimum_duration_s <= 0.0:
        raise ValueError("minimum phase duration must be positive and finite")
    if not math.isfinite(settling_margin_s) or settling_margin_s < 0.0:
        raise ValueError("settling margin must be finite and non-negative")
    required_motion_s = 0.0
    for door, spec in DOORS.items():
        for kind, joint in (("hinge", spec[0]), ("latch", spec[1])):
            if door not in previous_targets or door not in targets:
                raise ValueError(f"missing {door} service-door target")
            previous = previous_targets[door].get(kind)
            current = targets[door].get(kind)
            if not isinstance(previous, (int, float)) or not isinstance(current, (int, float)):
                raise ValueError(f"non-numeric {door} {kind} target")
            if not math.isfinite(previous) or not math.isfinite(current):
                raise ValueError(f"non-finite {door} {kind} target")
            velocity = velocity_limits.get(joint)
            if velocity is None or not math.isfinite(velocity) or velocity <= 0.0:
                raise ValueError(f"missing usable velocity limit for {joint}")
            required_motion_s = max(required_motion_s, abs(current - previous) / velocity)
    return max(minimum_duration_s, required_motion_s + settling_margin_s)


def _plugin_target_echo_status(
    records: list[dict[str, Any]],
    targets: dict[str, dict[str, float]],
    baseline_counts: dict[str, dict[str, float]],
) -> tuple[bool, dict[str, Any]]:
    """Require each phase target to reach the plugin after the prior generation."""

    latest = {
        str(record.get("door")): record
        for record in records
        if isinstance(record, dict) and isinstance(record.get("door"), str)
    }
    observed: dict[str, Any] = {}
    ready = True
    for door, target in sorted(targets.items()):
        record = latest.get(door)
        if record is None:
            observed[door] = {"reason": "no_plugin_diagnostic"}
            ready = False
            continue
        row: dict[str, Any] = {}
        for kind in ("hinge", "latch"):
            count_key = f"received_{kind}_messages"
            target_key = f"received_{kind}_target_rad"
            count = record.get(count_key)
            echoed_target = record.get(target_key)
            baseline = baseline_counts.get(door, {}).get(kind, 0.0)
            expected = target.get(kind)
            delivered = (
                isinstance(count, (int, float))
                and math.isfinite(count)
                and count > baseline
                and isinstance(echoed_target, (int, float))
                and math.isfinite(echoed_target)
                and isinstance(expected, (int, float))
                and math.isfinite(expected)
                and math.isclose(echoed_target, expected, abs_tol=PLUGIN_ECHO_TOLERANCE_RAD)
            )
            row[kind] = {
                "received_messages": count,
                "previous_received_messages": baseline,
                "received_target_rad": echoed_target,
                "expected_target_rad": expected,
                "delivered_after_previous_generation": delivered,
            }
            ready = ready and delivered
        observed[door] = row
    return ready, observed


def _parse_plugin_diagnostics(path: Path) -> dict[str, Any]:
    """Retain plugin telemetry from this fresh launch without treating it as motion proof."""

    result: dict[str, Any] = {
        "launch_log": str(path), "lifecycle": [], "records": [],
    }
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
    lifecycle_numeric_keys = {
        "configured", "doors", "model_entity", "hinge_subscribed",
        "latch_subscribed",
    }

    def finite_float(key: str, value: str) -> float:
        parsed = float(value)
        if not math.isfinite(parsed):
            raise ValueError(f"{key} is not finite")
        return parsed

    def parse_fields(text: str) -> dict[str, str]:
        pairs = re.findall(r"([a-z_]+)=([^\s]+)", text)
        if len({key for key, _ in pairs}) != len(pairs):
            raise ValueError("duplicate_field")
        return dict(pairs)

    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        lifecycle_marker = line.find(PLUGIN_LIFECYCLE_PREFIX)
        if lifecycle_marker >= 0:
            try:
                fields = parse_fields(line[lifecycle_marker:])
            except ValueError as exc:
                result["parse_error"] = f"invalid_lifecycle_field:{exc}"
                return result
            event = fields.pop("event", None)
            if event is not None:
                lifecycle: dict[str, Any] = {"event": event}
                if "door" in fields:
                    lifecycle["door"] = fields.pop("door")
                try:
                    lifecycle.update(
                        {
                            key: finite_float(key, value)
                            for key, value in fields.items()
                            if key in lifecycle_numeric_keys
                        }
                    )
                except ValueError as exc:
                    result["parse_error"] = f"invalid_lifecycle_numeric_field:{exc}"
                    return result
                if "reason" in fields:
                    lifecycle["reason"] = fields["reason"]
                result["lifecycle"].append(lifecycle)
            continue
        marker = line.find(PLUGIN_DIAGNOSTIC_PREFIX)
        if marker < 0:
            continue
        try:
            fields = parse_fields(line[marker:])
        except ValueError as exc:
            result["parse_error"] = f"invalid_field:{exc}"
            return result
        if "door" not in fields:
            continue
        record: dict[str, Any] = {"door": fields.pop("door")}
        try:
            record.update(
                {
                    key: finite_float(key, value)
                    for key, value in fields.items()
                    if key in numeric_keys
                }
            )
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
    settling_margin_s: float,
    minimum_fresh_samples: int,
    gazebo_sidecar: Path,
) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64

    expected_joints = {item for spec in DOORS.values() for item in spec[:2]}

    class Collector(Node):
        def __init__(self) -> None:
            super().__init__(
                "formal_service_door_runtime_collector",
                parameter_overrides=[Parameter("use_sim_time", value=True)],
            )
            self.latest: dict[str, float] = {}
            self.active_samples: list[dict[str, Any]] | None = None
            self.complete_sample_count = 0
            self.complete_sample_history: list[dict[str, Any]] = []
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
            self.create_subscription(
                JointState, PHYSICAL_JOINT_STATES_TOPIC, self._on_joint_state, 50
            )

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
            self.complete_sample_count += 1
            self.complete_sample_history.append(
                {
                    "received_monotonic_ns": time.monotonic_ns(),
                    "sim_clock_ns": self.get_clock().now().nanoseconds,
                    "positions_rad": dict(sorted(self.latest.items())),
                }
            )
            if self.active_samples is not None:
                self.active_samples.append(
                    {
                        "received_monotonic_ns": time.monotonic_ns(),
                        "positions_rad": dict(sorted(self.latest.items())),
                    }
                )

        def wait_for_fresh_complete_samples(self, count: int, timeout_s: float) -> list[dict[str, Any]]:
            if count <= 0:
                raise ValueError("minimum fresh sample count must be positive")
            starting_index = len(self.complete_sample_history)
            deadline = time.monotonic() + timeout_s
            window: list[dict[str, Any]] = []
            while (
                rclpy.ok()
                and time.monotonic() < deadline
            ):
                samples = self.complete_sample_history[starting_index:]
                window = samples[-count:]
                sim_times = [sample["sim_clock_ns"] for sample in window]
                if len(window) == count and all(
                    left < right for left, right in zip(sim_times, sim_times[1:])
                ):
                    return window
                rclpy.spin_once(self, timeout_sec=0.1)
            raise TimeoutError(
                "physical service-door state stream did not produce "
                f"{count} fresh complete samples with strictly advancing simulated time; "
                f"last_window={window}"
            )

        @staticmethod
        def _latest_plugin_counts(plugin_diagnostic_log: Path) -> dict[str, dict[str, float]]:
            parsed = _parse_plugin_diagnostics(plugin_diagnostic_log)
            if "parse_error" in parsed:
                raise RuntimeError(
                    "cannot establish service-door plugin echo baseline: "
                    + str(parsed["parse_error"])
                )
            latest = {
                str(record.get("door")): record
                for record in parsed["records"]
                if isinstance(record, dict) and isinstance(record.get("door"), str)
            }
            return {
                door: {
                    kind: float(record.get(f"received_{kind}_messages", 0.0))
                    for kind in ("hinge", "latch")
                }
                for door, record in latest.items()
            }

        def phase(
            self,
            targets: dict[str, dict[str, float]],
            duration_s: float,
            wall_timeout_s: float,
            minimum_fresh_samples: int,
            plugin_diagnostic_log: Path,
        ) -> dict[str, Any]:
            subscription_counts = self.target_subscription_counts()
            if not all(subscription_counts.values()):
                raise RuntimeError(
                    "service-door evaluator targets lost ROS bridge subscribers: "
                    + json.dumps(subscription_counts, sort_keys=True)
                )
            if not math.isfinite(duration_s) or duration_s <= 0.0:
                raise ValueError("phase simulated duration must be positive and finite")
            if not math.isfinite(wall_timeout_s) or wall_timeout_s <= 0.0:
                raise ValueError("phase wall watchdog must be positive and finite")
            wall_deadline = time.monotonic() + wall_timeout_s
            baseline_counts = self._latest_plugin_counts(plugin_diagnostic_log)
            phase_echo: dict[str, Any] | None = None
            sample_history_before_echo = len(self.complete_sample_history)
            while rclpy.ok() and time.monotonic() < wall_deadline:
                for door, row in targets.items():
                    self.target_publishers[(door, "hinge")].publish(Float64(data=row["hinge"]))
                    self.target_publishers[(door, "latch")].publish(Float64(data=row["latch"]))
                rclpy.spin_once(self, timeout_sec=0.05)
                parsed = _parse_plugin_diagnostics(plugin_diagnostic_log)
                if "parse_error" in parsed:
                    raise RuntimeError(
                        "service-door plugin echo parse failed: "
                        + str(parsed["parse_error"])
                    )
                echo_ready, phase_echo = _plugin_target_echo_status(
                    parsed["records"], targets, baseline_counts
                )
                if echo_ready:
                    sample_history_before_echo = len(self.complete_sample_history)
                    break
            else:
                phase_echo = phase_echo or {}
            if phase_echo is None or not all(
                item.get("hinge", {}).get("delivered_after_previous_generation")
                and item.get("latch", {}).get("delivered_after_previous_generation")
                for item in phase_echo.values()
            ) or len(phase_echo) != len(DOORS):
                raise TimeoutError(
                    "service-door phase target was not echoed by every plugin after "
                    "the previous delivery generation: "
                    + json.dumps(phase_echo, sort_keys=True)
                )
            fresh_window: list[dict[str, Any]] = []
            while rclpy.ok() and time.monotonic() < wall_deadline:
                samples = self.complete_sample_history[sample_history_before_echo:]
                fresh_window = samples[-minimum_fresh_samples:]
                sim_times = [sample["sim_clock_ns"] for sample in fresh_window]
                if len(fresh_window) == minimum_fresh_samples and all(
                    left < right for left, right in zip(sim_times, sim_times[1:])
                ):
                    break
                for door, row in targets.items():
                    self.target_publishers[(door, "hinge")].publish(Float64(data=row["hinge"]))
                    self.target_publishers[(door, "latch")].publish(Float64(data=row["latch"]))
                rclpy.spin_once(self, timeout_sec=0.05)
            if len(fresh_window) < minimum_fresh_samples or not all(
                left < right
                for left, right in zip(
                    [sample["sim_clock_ns"] for sample in fresh_window],
                    [sample["sim_clock_ns"] for sample in fresh_window][1:],
                )
            ):
                raise TimeoutError(
                    "service-door phase did not receive enough complete physical samples "
                    "with advancing simulated time after the plugin echoed this phase target"
                )
            self.active_samples = []
            sample_count_start = self.complete_sample_count
            sim_start_ns = self.get_clock().now().nanoseconds
            sim_deadline_ns = sim_start_ns + math.ceil(duration_s * 1_000_000_000)
            sim_end_ns = sim_start_ns
            while rclpy.ok() and sim_end_ns < sim_deadline_ns:
                if time.monotonic() >= wall_deadline:
                    self.active_samples = None
                    raise TimeoutError(
                        "ROS simulated clock did not cover the required service-door phase "
                        f"duration before the {wall_timeout_s:g}s wall watchdog"
                    )
                for door, row in targets.items():
                    self.target_publishers[(door, "hinge")].publish(Float64(data=row["hinge"]))
                    self.target_publishers[(door, "latch")].publish(Float64(data=row["latch"]))
                rclpy.spin_once(self, timeout_sec=0.05)
                sim_end_ns = self.get_clock().now().nanoseconds
            samples = self.active_samples
            self.active_samples = None
            return {
                "commanded_targets_rad": targets,
                "ros_publisher_subscription_counts": subscription_counts,
                "joint_state_samples": samples,
                "simulated_duration_requested_s": duration_s,
                "sim_clock_start_ns": sim_start_ns,
                "sim_clock_end_ns": sim_end_ns,
                "plugin_target_echo": phase_echo,
                "fresh_complete_samples_after_plugin_echo": fresh_window,
                "complete_samples_observed_during_phase": (
                    self.complete_sample_count - sample_count_start
                ),
            }

    def targets(hinge: str, latch: float) -> dict[str, dict[str, float]]:
        return {
            door: {"hinge": (spec[4] if hinge == "open" else 0.0), "latch": latch}
            for door, spec in DOORS.items()
        }

    source_binding, acceptance_session_binding, binding = _bound_runtime_evidence(
        snapshot, session, runtime_binding
    )
    velocity_limits = _load_joint_velocity_limits(snapshot)

    rclpy.init()
    node = Collector()
    phases: dict[str, Any] = {}
    raw: dict[str, Any] = {
        "source_binding": source_binding,
        # The report is portable across host/container boundaries.  Its
        # logical path is paired with source_binding's manifest hash; the
        # collector passes the actual resolved path directly to evaluate().
        "snapshot_manifest": SNAPSHOT_LOGICAL_PATH,
        "evidence_authority": PHYSICAL_JOINT_STATE_AUTHORITY,
        "physical_joint_state_topic": PHYSICAL_JOINT_STATES_TOPIC,
        "urdf_velocity_limits_rad_per_s": velocity_limits,
        "phase_timing_contract": {
            "minimum_duration_s": phase_duration_s,
            "settling_margin_s": settling_margin_s,
            "minimum_fresh_samples": minimum_fresh_samples,
        },
        "gazebo_partition": os.environ.get("GZ_PARTITION"),
        "phases": phases,
    }
    try:
        deadline = time.monotonic() + startup_timeout_s
        while rclpy.ok() and set(node.latest) != expected_joints and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if set(node.latest) != expected_joints:
            raise TimeoutError(
                "all eight physical service-door joints did not appear on "
                + PHYSICAL_JOINT_STATES_TOPIC
            )
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
        raw["target_transport"]["fresh_complete_samples_after_ready"] = (
            node.wait_for_fresh_complete_samples(minimum_fresh_samples, startup_timeout_s)
        )
        previous_targets = targets("closed", 0.0)
        for name, current_targets in (
            ("initial_locked", targets("closed", 0.0)),
            ("locked_open_rejected", targets("open", 0.0)),
            ("unlocked", targets("closed", 0.6)),
            ("open", targets("open", 0.6)),
            ("closed_unlocked", targets("closed", 0.6)),
            ("transport_locked", targets("closed", 0.0)),
            ("relock_open_rejected", targets("open", 0.0)),
        ):
            duration_s = _phase_duration_from_targets(
                previous_targets,
                current_targets,
                velocity_limits,
                phase_duration_s,
                settling_margin_s,
            )
            phases[name] = node.phase(
                current_targets,
                duration_s,
                startup_timeout_s,
                minimum_fresh_samples,
                plugin_diagnostic_log,
            )
            previous_targets = current_targets
    except Exception as exc:
        raw["collector_error"] = str(exc)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    raw["plugin_diagnostics"] = _parse_plugin_diagnostics(plugin_diagnostic_log)
    try:
        raw["gazebo_joint_state_sidecar"] = json.loads(gazebo_sidecar.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raw["gazebo_joint_state_sidecar"] = {"status": "FAILED", "error": str(exc)}
    report = evaluate(raw, snapshot)
    report["runtime_gate_binding"] = binding
    report["acceptance_session_binding"] = acceptance_session_binding
    report["runtime_closure_binding"] = binding["runtime_closure_binding"]
    if "collector_error" in raw:
        report["collector_error"] = raw["collector_error"]
    output.parent.mkdir(parents=True, exist_ok=True)
    final_report, text = report_json(raw, report)
    output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0 if final_report["passed"] else 2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--snapshot-manifest", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument("--plugin-diagnostic-log", type=Path, required=True)
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    parser.add_argument("--phase-duration", type=float, default=2.5)
    parser.add_argument("--settling-margin", type=float, default=1.0)
    parser.add_argument("--minimum-fresh-samples", type=int, default=5)
    parser.add_argument("--gazebo-sidecar", type=Path, required=True)
    args = parser.parse_args()
    return run(
        args.output,
        args.snapshot_manifest,
        args.session,
        args.runtime_binding,
        args.plugin_diagnostic_log,
        args.startup_timeout,
        args.phase_duration,
        args.settling_margin,
        args.minimum_fresh_samples,
        args.gazebo_sidecar,
    )


if __name__ == "__main__":
    raise SystemExit(main())
