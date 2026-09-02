#!/usr/bin/env python3
"""Drive and measure every non-base formal cleaning/storage actuator in Gazebo."""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from collections import defaultdict
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from formal_runtime_gate_binding import load_binding
from formal_preembedded_sensor_world_binding import validate_preembedded_sensor_world
from formal_squeegee_compliance_core import (
    SQUEEGEE_SIGNALS as SQUEEGEE_SIGNAL_NAMES,
    evaluate_squeegee_compliance,
)


POSITION_TARGETS = {
    "cleaning_lift_joint": 0.100,
    "dry_deposit_gate_joint": 1.05,
    "wastewater_drain_valve_joint": 1.00,
}
VELOCITY_TARGETS = {
    "left_side_brush_joint": 8.0,
    "right_side_brush_joint": -8.0,
    "central_roller_joint": 12.0,
    "recovery_pump_joint": 20.0,
}
POSITION_REACHED_TOLERANCES = {
    "cleaning_lift_joint": 0.005,
    "dry_deposit_gate_joint": 0.025,
    "wastewater_drain_valve_joint": 0.025,
}
CONTACT_TOPICS = (
    "/cleaning/squeegee/contact",
    "/cleaning/suction_nozzle/contact",
    "/storage/dry_deposit/contact",
    "/formal_vehicle/simulation/raw/front_bumper/contact",
    "/formal_vehicle/simulation/raw/rear_bumper/contact",
)
SQUEEGEE_TOPIC_ROOT = "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance"
SQUEEGEE_SIGNALS = {
    "float_position_m": f"{SQUEEGEE_TOPIC_ROOT}/float_position_m",
    "float_velocity_m_s": f"{SQUEEGEE_TOPIC_ROOT}/float_velocity_m_s",
    "float_force_n": f"{SQUEEGEE_TOPIC_ROOT}/float_force_n",
    "pitch_position_rad": f"{SQUEEGEE_TOPIC_ROOT}/pitch_position_rad",
    "pitch_velocity_rad_s": f"{SQUEEGEE_TOPIC_ROOT}/pitch_velocity_rad_s",
    "pitch_torque_nm": f"{SQUEEGEE_TOPIC_ROOT}/pitch_torque_nm",
}
assert set(SQUEEGEE_SIGNALS) == set(SQUEEGEE_SIGNAL_NAMES)


def snapshot_binding(path: Path) -> dict[str, str]:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    output = manifest.get("outputs", {}).get(
        "reports/engineering/formal_competition_vehicle.urdf"
    )
    source_hash = manifest.get("source_inventory_sha256")
    if not isinstance(output, dict) or not isinstance(output.get("sha256"), str):
        raise RuntimeError("snapshot manifest has no expanded formal vehicle URDF hash")
    if not isinstance(source_hash, str) or not source_hash:
        raise RuntimeError("snapshot manifest has no source inventory hash")
    return {
        "snapshot_manifest_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": output["sha256"],
    }


def bound_runtime_evidence(
    snapshot_path: Path, session_path: Path, binding_path: Path
) -> tuple[dict[str, str], dict[str, object], dict[str, object]]:
    """Reject actuator evidence not bound to the active frozen runtime/session."""

    source = snapshot_binding(snapshot_path)
    session = json.loads(session_path.read_text(encoding="utf-8"))
    if not isinstance(session, dict) or session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise ValueError("formal acceptance session must be RUNNING")
    started_epoch_ns = session.get("started_epoch_ns")
    if not isinstance(started_epoch_ns, int) or started_epoch_ns <= 0:
        raise ValueError("formal acceptance session start time is invalid")
    if started_epoch_ns > time.time_ns():
        raise ValueError("formal acceptance session is future dated")
    binding = load_binding(binding_path)
    bound_session = binding.get("acceptance_session_binding")
    if not isinstance(bound_session, dict):
        raise ValueError("runtime binding has no acceptance-session binding")
    if bound_session.get("snapshot") != source:
        raise ValueError("runtime binding snapshot differs from function-position source binding")
    if (
        bound_session.get("session_manifest_sha256")
        != hashlib.sha256(session_path.read_bytes()).hexdigest()
        or bound_session.get("session_started_epoch_ns") != started_epoch_ns
        or bound_session.get("session_status_at_gate")
        != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
    ):
        raise ValueError("runtime binding session differs from function-position session")
    verified_epoch_ns = binding.get("verified_epoch_ns")
    now_ns = time.time_ns()
    if (
        not isinstance(verified_epoch_ns, int)
        or verified_epoch_ns < started_epoch_ns
        or verified_epoch_ns > now_ns
    ):
        raise ValueError("runtime binding timestamp is outside the active acceptance session")
    binding_mtime_ns = binding_path.stat().st_mtime_ns
    if binding_mtime_ns < started_epoch_ns or binding_mtime_ns > now_ns:
        raise ValueError("runtime binding file timestamp is outside the active acceptance session")
    closure = binding.get("runtime_closure_binding")
    if not isinstance(closure, dict):
        raise ValueError("runtime binding has no runtime closure identity")
    if closure.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED":
        raise ValueError("runtime binding closure is not VERIFIED")
    for key in ("manifest_sha256", "closure_sha256"):
        value = closure.get(key)
        if (
            not isinstance(value, str)
            or len(value) != 64
            or any(character not in "0123456789abcdef" for character in value)
        ):
            raise ValueError(f"runtime binding closure has invalid {key}")
    if closure.get("symbolic_link_count") != 0:
        raise ValueError("runtime binding closure contains symbolic links")
    return source, bound_session, binding


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("formal_function_positions_runtime_probe")
        self.positions: dict[str, list[float]] = defaultdict(list)
        self.velocities: dict[str, list[float]] = defaultdict(list)
        self.phase = "readiness"
        self.phase_signals: dict[str, dict[str, list[float]]] = defaultdict(
            lambda: defaultdict(list)
        )
        self.phase_joint_positions: dict[str, dict[str, list[float]]] = defaultdict(
            lambda: defaultdict(list)
        )
        self.phase_contacts: dict[str, dict[str, object]] = defaultdict(
            lambda: {"messages": 0, "nonempty_messages": 0, "collision_pairs": set()}
        )
        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self._joint_state, 20)
        self.cleaning = self.create_publisher(JointTrajectory, "/cleaning_controller/joint_trajectory", 1)
        self.storage = self.create_publisher(JointTrajectory, "/storage_controller/joint_trajectory", 1)
        self.service = self.create_publisher(JointTrajectory, "/service_controller/joint_trajectory", 1)
        self.brush = self.create_publisher(Float64MultiArray, "/brush_controller/commands", 1)
        self.recovery = self.create_publisher(Float64MultiArray, "/recovery_controller/commands", 1)
        self.squeegee_contact_sub = self.create_subscription(
            Contacts, "/cleaning/squeegee/contact", self._squeegee_contact, 50
        )
        self.squeegee_subscriptions = [
            self.create_subscription(
                Float64,
                topic,
                lambda message, signal=name: self._squeegee_signal(signal, message),
                50,
            )
            for name, topic in SQUEEGEE_SIGNALS.items()
        ]

    def _joint_state(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                self.positions[name].append(float(msg.position[index]))
                if name in ("squeegee_float_joint", "squeegee_pitch_joint"):
                    self.phase_joint_positions[self.phase][name].append(
                        float(msg.position[index])
                    )
            if index < len(msg.velocity):
                self.velocities[name].append(float(msg.velocity[index]))

    def _squeegee_signal(self, name: str, message: Float64) -> None:
        self.phase_signals[self.phase][name].append(float(message.data))

    def _squeegee_contact(self, message: Contacts) -> None:
        evidence = self.phase_contacts[self.phase]
        evidence["messages"] = int(evidence["messages"]) + 1
        if message.contacts:
            evidence["nonempty_messages"] = int(evidence["nonempty_messages"]) + 1
        pairs = evidence["collision_pairs"]
        assert isinstance(pairs, set)
        for contact in message.contacts:
            pair = " <-> ".join(
                sorted((contact.collision1.name, contact.collision2.name))
            )
            pairs.add(pair)

    def set_phase(self, phase: str) -> None:
        self.phase = phase

    @staticmethod
    def trajectory(joints: list[str], positions: list[float], seconds: int = 2) -> JointTrajectory:
        msg = JointTrajectory()
        msg.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=seconds)
        msg.points = [point]
        return msg

    def ready(self) -> bool:
        expected = set(POSITION_TARGETS) | set(VELOCITY_TARGETS) | {
            "squeegee_float_joint", "squeegee_pitch_joint"
        }
        return (
            all(
                pub.get_subscription_count() > 0
                for pub in (
                    self.cleaning,
                    self.storage,
                    self.service,
                    self.brush,
                    self.recovery,
                )
            )
            and expected <= set(self.positions)
            and all(self.get_publishers_info_by_topic(topic) for topic in CONTACT_TOPICS)
            and all(
                self.get_publishers_info_by_topic(topic)
                for topic in SQUEEGEE_SIGNALS.values()
            )
        )

    def publish_targets(self) -> None:
        self.cleaning.publish(self.trajectory(
            ["cleaning_lift_joint"],
            [POSITION_TARGETS["cleaning_lift_joint"]],
            24,
        ))
        self.storage.publish(self.trajectory(
            ["dry_deposit_gate_joint"],
            [POSITION_TARGETS["dry_deposit_gate_joint"]],
            4,
        ))
        self.service.publish(self.trajectory(
            ["wastewater_drain_valve_joint"],
            [POSITION_TARGETS["wastewater_drain_valve_joint"]],
            24,
        ))
        self.brush.publish(Float64MultiArray(data=[8.0, -8.0, 12.0]))
        self.recovery.publish(Float64MultiArray(data=[20.0]))

    def stop_rotors(self) -> None:
        self.brush.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
        self.recovery.publish(Float64MultiArray(data=[0.0]))

    def publish_recovery_pose(self) -> None:
        self.cleaning.publish(self.trajectory(["cleaning_lift_joint"], [0.000], 24))
        self.storage.publish(self.trajectory(["dry_deposit_gate_joint"], [0.0], 4))
        self.service.publish(self.trajectory(["wastewater_drain_valve_joint"], [0.0], 24))

    def positions_reached(self, targets: dict[str, float]) -> bool:
        return all(
            self.positions.get(name)
            and abs(self.positions[name][-1] - target)
            <= POSITION_REACHED_TOLERANCES[name]
            for name, target in targets.items()
        )


def spin_for(node: Probe, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.10)


def spin_until_positions(
    node: Probe, targets: dict[str, float], timeout_seconds: float
) -> tuple[bool, float]:
    started = time.monotonic()
    deadline = started + timeout_seconds
    while rclpy.ok() and not node.positions_reached(targets) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.10)
    return node.positions_reached(targets), time.monotonic() - started


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--snapshot-manifest", type=Path, required=True)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument("--preembedded-report", type=Path)
    parser.add_argument("--preembedded-world", type=Path)
    parser.add_argument("--preembedded-model-pose", default="0 0 0.005 0 0 0")
    parser.add_argument("--expanded-urdf", type=Path)
    parser.add_argument("--runtime-install-root", type=Path)
    parser.add_argument(
        "--diagnostic-skip-preembedded-binding",
        action="store_true",
        help=(
            "Diagnostic-only: mark the run as unbound when exercising a temporary "
            "candidate model. Formal acceptance must not use this option."
        ),
    )
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()
    source_binding, acceptance_session_binding, runtime_gate_binding = bound_runtime_evidence(
        args.snapshot_manifest, args.session, args.runtime_binding
    )
    binding_args = (
        args.preembedded_report,
        args.preembedded_world,
        args.expanded_urdf,
        args.runtime_install_root,
    )
    if args.diagnostic_skip_preembedded_binding:
        if any(binding_args):
            raise SystemExit(
                "diagnostic preembedded binding skip must not be combined with binding artifacts"
            )
        preembedded_world_binding = {
            "status": "DIAGNOSTIC_UNBOUND_PREEMBEDDED_WORLD",
            "formal_acceptance_eligible": False,
            "reason": "temporary candidate model differs from frozen snapshot",
        }
    else:
        if not all(binding_args):
            raise SystemExit(
                "formal acceptance requires --preembedded-report, --preembedded-world, "
                "--expanded-urdf and --runtime-install-root"
            )
        preembedded_world_binding = validate_preembedded_sensor_world(
            report_path=args.preembedded_report,
            world_path=args.preembedded_world,
            expanded_urdf_path=args.expanded_urdf,
            acceptance_session={
                "started_epoch_ns": acceptance_session_binding["session_started_epoch_ns"],
                "session_manifest_sha256": acceptance_session_binding["session_manifest_sha256"],
            },
            snapshot_identity=source_binding,
            expected_model_pose=args.preembedded_model_pose,
            expected_runtime_install_root=args.runtime_install_root,
        )
    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and not node.ready() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
    if not node.ready():
        counts = {
            "cleaning": node.cleaning.get_subscription_count(),
            "storage": node.storage.get_subscription_count(),
            "service": node.service.get_subscription_count(),
            "brush": node.brush.get_subscription_count(),
            "recovery": node.recovery.get_subscription_count(),
            "contact_topic_publishers": {
                topic: len(node.get_publishers_info_by_topic(topic)) for topic in CONTACT_TOPICS
            },
        }
        raise SystemExit(
            "formal function readiness timeout: "
            + json.dumps({"controller_subscriptions": counts, "joint_state_names": sorted(node.positions)})
        )
    node.set_phase("raised_free")
    spin_for(node, 8.0)
    node.set_phase("grounding_transition")
    node.publish_targets()
    ground_targets_reached, grounding_wall_seconds = spin_until_positions(
        node, POSITION_TARGETS, 240.0
    )
    node.set_phase("grounded_preload")
    spin_for(node, 12.0 if ground_targets_reached else 4.0)
    node.stop_rotors()
    node.set_phase("recovery_transition")
    node.publish_recovery_pose()
    recovery_targets = {name: 0.0 for name in POSITION_TARGETS}
    recovery_targets_reached, recovery_wall_seconds = spin_until_positions(
        node, recovery_targets, 240.0
    )
    node.set_phase("raised_recovery")
    spin_for(node, 12.0 if recovery_targets_reached else 4.0)

    measured: dict[str, dict[str, float | int | bool]] = {}
    failures: list[str] = []
    if not ground_targets_reached:
        failures.append("ground_position_targets_timeout")
    if not recovery_targets_reached:
        failures.append("recovery_position_targets_timeout")
    for name, target in POSITION_TARGETS.items():
        values = node.positions.get(name, [])
        terminal = values[-1] if values else None
        minimum_error = min((abs(value - target) for value in values), default=None)
        passed = minimum_error is not None and minimum_error <= 0.025
        measured[name] = {
            "target": target,
            "terminal": terminal,
            "range": max(values) - min(values) if values else 0.0,
            "minimum_target_error": minimum_error,
            "samples": len(values),
            "passed": passed,
        }
        if not passed:
            failures.append(name)
    for name, target in VELOCITY_TARGETS.items():
        values = node.velocities.get(name, [])
        peak = max(values, key=abs) if values else None
        passed = peak is not None and abs(peak - target) <= 1.5
        measured[name] = {
            "target_velocity": target,
            "peak_velocity": peak,
            "samples": len(values),
            "passed": passed,
        }
        if not passed:
            failures.append(name)

    squeegee, squeegee_failures = evaluate_squeegee_compliance(
        node.phase_signals, node.phase_joint_positions, node.phase_contacts
    )
    failures.extend(squeegee_failures)

    report = {
        "report_id": "tzcup_formal_function_positions_runtime_v3",
        "status": (
            "DIAGNOSTIC_CLEANING_STORAGE_SERVICE_AND_RECOVERY_ACTUATORS_PASSED"
            if args.diagnostic_skip_preembedded_binding and not failures
            else "FORMAL_CLEANING_STORAGE_SERVICE_AND_RECOVERY_ACTUATORS_PASSED"
            if not failures
            else "FAILED"
        ),
        "controller_count": 5,
        "actuated_joint_count": len(measured),
        "passive_measured_joint_count": 2,
        "contact_topic_publishers": {
            topic: len(node.get_publishers_info_by_topic(topic)) for topic in CONTACT_TOPICS
        },
        "measured": measured,
        "squeegee_compliance": squeegee,
        "phase_execution": {
            "ground_targets_reached": ground_targets_reached,
            "grounding_wall_seconds": grounding_wall_seconds,
            "recovery_targets_reached": recovery_targets_reached,
            "recovery_wall_seconds": recovery_wall_seconds,
            "position_reached_tolerances": POSITION_REACHED_TOLERANCES,
        },
        "failures": failures,
        "passed": not failures,
        "source_binding": source_binding,
        "acceptance_session_binding": acceptance_session_binding,
        "runtime_gate_binding": runtime_gate_binding,
        "preembedded_world_binding": preembedded_world_binding,
        "runtime_identity": runtime_gate_binding["runtime_closure_binding"],
        "claim_boundary": "This proves controller-to-joint motion for cleaning, storage, powered wastewater service valve and pump positions plus live two-axis squeegee joint state, physical ground engagement from compression and signed preload effort, and post-load spring recovery. Contact transport is reported separately and is never claimed when its stream is empty. Hydraulic recovery efficiency, brush wear and debris pickup remain separate runtime gates.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(report, ensure_ascii=False))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
