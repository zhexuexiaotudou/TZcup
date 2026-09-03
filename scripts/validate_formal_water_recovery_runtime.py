#!/usr/bin/env python3
"""Drive and machine-accept the formal vehicle's L1 water recovery proxy."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray, String

from formal_cleaning_motor_telemetry import (
    decode_cleaning_motor_telemetry,
    update_physics_revision_watchdog,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from formal_cleaning_lift_recovery_core import (
    CleaningLiftRecoverySupervisor,
    trajectory_duration_s,
)
from formal_water_motor_metrics import central_roller_duty_metrics, side_brush_duty_metrics
from formal_preembedded_sensor_world_binding import validate_preembedded_sensor_world
from formal_runtime_gate_binding import load_binding


ROOT = "/model/tzcup_formal_sanitation_vehicle/water_recovery"
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SNAPSHOT = REPOSITORY_ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
DEFAULT_SESSION = REPOSITORY_ROOT / "artifacts/formal_final_acceptance_session.json"
PUMP_LIMIT_L_MIN = 15.1 * 0.70
TANK_CAPACITY_KG = 8.30
SIM_CLOCK_STALL_WALL_S = 45.0
NORMAL_PASS_TIMEOUT_SIM_S = 90.0
NORMAL_PASS_HARD_WALL_S = 2_400.0
FULL_TANK_TIMEOUT_SIM_S = 30.0
FULL_TANK_HARD_WALL_S = 900.0
RECOVERY_CONTACT_COLLISIONS = {
    "squeegee": "squeegee_blade_collision",
    "left_side_brush": "left_side_brush_link_collision",
    "right_side_brush": "right_side_brush_link_collision",
    "central_roller": "central_roller_link_collision",
}


def _bound_runtime_evidence(
    snapshot_path: Path, session_path: Path, binding_path: Path
) -> tuple[dict[str, str], dict[str, object], dict[str, object]]:
    """Return only an active frozen-runtime binding for formal water evidence."""

    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    outputs = snapshot.get("outputs", {})
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf", {})
    source_hash = snapshot.get("source_inventory_sha256")
    urdf_hash = urdf.get("sha256") if isinstance(urdf, dict) else None
    if not isinstance(source_hash, str) or not source_hash:
        raise ValueError("snapshot has no source_inventory_sha256")
    if not isinstance(urdf_hash, str) or not urdf_hash:
        raise ValueError("snapshot has no expanded URDF sha256")
    source_binding = {
        "snapshot_manifest_sha256": hashlib.sha256(snapshot_path.read_bytes()).hexdigest(),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": urdf_hash,
    }
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
        raise ValueError("runtime binding snapshot differs from water source binding")
    if (
        bound_session.get("session_manifest_sha256")
        != hashlib.sha256(session_path.read_bytes()).hexdigest()
        or bound_session.get("session_started_epoch_ns") != started_epoch_ns
    ):
        raise ValueError("runtime binding session differs from water session")
    return source_binding, bound_session, binding


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("formal_water_recovery_acceptance")
        self.status: dict[str, object] | None = None
        self.status_samples: list[dict[str, object]] = []
        self.applied_mass: float | None = None
        self.odom: Odometry | None = None
        self.sim_time_s: float | None = None
        self.safety_permit: bool | None = None
        self.safety_json_permit: bool | None = None
        self.actuator_permit: bool | None = None
        self.safety_status_json_count = 0
        self.safety_state: str | None = None
        self.safety_active_reasons = ""
        self.latest_motor_status: dict[str, object] = {}
        self.motor_status_samples: list[dict[str, object]] = []
        self.last_motor_telemetry_sequence: int | None = None
        self.last_motor_physics_update_sequence: int | None = None
        self.last_motor_physics_revision_advance_s: float | None = None
        self.latest_auxiliary_status: dict[str, object] = {}
        self.safety_transition_history: list[dict[str, object]] = []
        self.safety_handshake_events: list[dict[str, object]] = []
        self._last_safety_signature: tuple[tuple[str, str], ...] | None = None
        self._last_safety_json_signature: tuple[object, ...] | None = None
        self.lift_recovery_events: list[dict[str, object]] = []
        # Keep only bounded aggregate contact evidence for the active recovery
        # interval.  A pass therefore cannot substitute commanded velocities
        # or geometric clearance for physical cleaning-pavement contact.
        self.recovery_contact_window: dict[str, dict[str, object]] | None = None
        self.enable = self.create_publisher(Bool, f"{ROOT}/command/enable", 10)
        self.reset_ground = self.create_publisher(
            Float64, f"{ROOT}/command/reset_ground_volume_l", 10
        )
        self.reset_tank = self.create_publisher(
            Float64, f"{ROOT}/command/reset_tank_mass_kg", 10
        )
        self.filter_blockage = self.create_publisher(
            Float64, f"{ROOT}/command/filter_blockage_fraction", 10
        )
        self.service_drain = self.create_publisher(
            Bool, f"{ROOT}/command/service_drain_open", 10
        )
        self.cleaning = self.create_publisher(
            JointTrajectory, "/cleaning_controller/joint_trajectory", 10
        )
        self.brush = self.create_publisher(
            Float64MultiArray, "/safety/command/brush", 10
        )
        self.pump = self.create_publisher(
            Float64MultiArray, "/safety/command/pump", 10
        )
        self.drive = self.create_publisher(Twist, "/cmd_vel_gate", 10)
        self.main_power = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/main_power", 10
        )
        self.estop = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/emergency_stop", 10
        )
        self.estop_reset = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/emergency_stop_reset", 10
        )
        self.heartbeat = self.create_publisher(Empty, "/safety/control_heartbeat", 10)
        self.motor_fault_reset = self.create_publisher(
            Bool,
            f"/model/tzcup_formal_sanitation_vehicle/cleaning_motors/command/reset_faults",
            10,
        )
        self.create_subscription(String, f"{ROOT}/status_json", self._on_status, 50)
        self.create_subscription(
            Float64,
            "/model/tzcup_formal_sanitation_vehicle/payload/wastewater_mass_kg/applied",
            self._on_applied,
            50,
        )
        self.create_subscription(Odometry, "/odom/unfiltered", self._on_odom, 50)
        self.create_subscription(Clock, "/clock", self._on_clock, 50)
        self.create_subscription(
            DiagnosticArray, "/safety/status", self._on_safety_status, 100
        )
        # The compact product safety stream is the readiness/permit authority.
        # The larger DiagnosticArray remains optional forensic detail because
        # a large sample can be starved after the preflight subscriber exits,
        # while this bounded stream is independently accepted at 20 Hz.
        self.create_subscription(
            String, "/safety/status_json", self._on_safety_status_json, 1
        )
        self.create_subscription(
            Bool, "/safety/actuators_enabled", self._on_actuator_permit, 10
        )
        self.create_subscription(
            Float64MultiArray,
            "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/telemetry_snapshot",
            self._on_motor_status,
            100,
        )
        self.create_subscription(
            String,
            "/formal_vehicle/auxiliary/status_json",
            self._on_auxiliary_status,
            100,
        )
        for name, topic in (
            ("squeegee", "/cleaning/squeegee/contact"),
            ("left_side_brush", "/cleaning/left_side_brush/contact"),
            ("right_side_brush", "/cleaning/right_side_brush/contact"),
            ("central_roller", "/cleaning/central_roller/contact"),
        ):
            self.create_subscription(
                Contacts,
                topic,
                lambda message, contact_name=name: self._on_recovery_contact(
                    contact_name, message
                ),
                100,
            )

    def _on_status(self, message: String) -> None:
        self.status = json.loads(message.data)
        self.status_samples.append(dict(self.status))

    def _on_applied(self, message: Float64) -> None:
        self.applied_mass = float(message.data)

    def _on_odom(self, message: Odometry) -> None:
        self.odom = message

    def _on_clock(self, message: Clock) -> None:
        self.sim_time_s = message.clock.sec + message.clock.nanosec * 1e-9

    def _on_safety_status(self, message: DiagnosticArray) -> None:
        status = next(
            (entry for entry in message.status if entry.name == "whole_vehicle_safety"),
            None,
        )
        if status is None:
            return
        values = {entry.key: entry.value for entry in status.values}
        self.safety_state = values.get("state", status.message)
        self.safety_permit = (
            values.get("safety_inputs_permit_actuators") == "true"
        )
        self.safety_active_reasons = values.get("active_reasons", "")
        transition_keys = (
            "state",
            "safety_inputs_permit_actuators",
            "managed_controllers_active",
            "position_hold_ready",
            "active_reasons",
            "manual_estop_active",
            "front_bumper_contact",
            "front_bumper_available",
            "rear_bumper_contact",
            "rear_bumper_available",
            "safety_relay_enabled",
            "safety_relay_available",
            "bms_fault_active",
            "bms_fault_available",
            "cleaning_motor_fault_active",
            "cleaning_motor_fault_available",
            "traction_permitted",
            "traction_permit_available",
            "heartbeat_fresh",
            "command_fresh",
            "command_valid",
        )
        signature = tuple((key, values.get(key, "")) for key in transition_keys)
        if signature != self._last_safety_signature:
            self._last_safety_signature = signature
            self.safety_transition_history.append(
                {
                    "arrival_monotonic_s": time.monotonic(),
                    "sim_time_s": self.sim_time_s,
                    "safety": {key: values.get(key, "") for key in transition_keys},
                    "physics_update_stale": self.latest_motor_status.get(
                        "physics_update_stale"
                    ),
                    "motor_fault_active": self.latest_motor_status.get(
                        "fault_active"
                    ),
                    "auxiliary_bumper_inputs": self.latest_auxiliary_status.get(
                        "bumper_inputs", {}
                    ),
                    "cleaning_motor": dict(self.latest_motor_status),
                    "auxiliary": dict(self.latest_auxiliary_status),
                }
            )

    def _on_safety_status_json(self, message: String) -> None:
        values = json.loads(message.data)
        self.safety_status_json_count += 1
        self.safety_state = str(values.get("state", ""))
        self.safety_json_permit = bool(
            values.get("safety_inputs_permit_actuators", False)
        )
        self.safety_permit = self.safety_json_permit
        self.safety_active_reasons = str(values.get("active_reasons", ""))
        signature = (
            self.safety_state,
            self.safety_permit,
            bool(values.get("managed_controllers_active", False)),
            self.safety_active_reasons,
            int(values.get("unsafe_generation", 0)),
            int(values.get("consumed_unsafe_generation", 0)),
        )
        if signature != self._last_safety_json_signature:
            self._last_safety_json_signature = signature
            self.safety_transition_history.append(
                {
                    "arrival_monotonic_s": time.monotonic(),
                    "sim_time_s": self.sim_time_s,
                    "source": "bounded_status_json",
                    "safety": dict(values),
                    "physics_update_stale": self.latest_motor_status.get(
                        "physics_update_stale"
                    ),
                    "motor_fault_active": self.latest_motor_status.get(
                        "fault_active"
                    ),
                }
            )

    def _on_actuator_permit(self, message: Bool) -> None:
        # This is the actual safety-manager output wired to controllers, not
        # merely the status stream's declaration that inputs are permissive.
        self.actuator_permit = bool(message.data)

    def _on_motor_status(self, message: Float64MultiArray) -> None:
        decoded = decode_cleaning_motor_telemetry(message.data)
        now_s = time.monotonic()
        telemetry_sequence = int(decoded["telemetry_sequence"])
        sequence = int(decoded["physics_update_sequence"])
        if (
            self.last_motor_telemetry_sequence is not None
            and telemetry_sequence <= self.last_motor_telemetry_sequence
        ):
            raise ValueError(
                "cleaning motor telemetry_sequence is not strictly monotonic"
            )
        self.last_motor_telemetry_sequence = telemetry_sequence
        (
            self.last_motor_physics_update_sequence,
            self.last_motor_physics_revision_advance_s,
        ) = update_physics_revision_watchdog(
            sequence=sequence,
            physics_stale=bool(decoded["physics_update_stale"]),
            last_sequence=self.last_motor_physics_update_sequence,
            last_advance_s=self.last_motor_physics_revision_advance_s,
            now_s=now_s,
        )
        self.latest_motor_status = decoded
        self.motor_status_samples.append(
            {"sim_time_s": self.sim_time_s, **self.latest_motor_status}
        )

    def _on_auxiliary_status(self, message: String) -> None:
        self.latest_auxiliary_status = json.loads(message.data)

    def _on_recovery_contact(self, name: str, message: Contacts) -> None:
        if self.recovery_contact_window is None:
            return
        evidence = self.recovery_contact_window[name]
        evidence["messages"] = int(evidence["messages"]) + 1
        if not message.contacts:
            return
        evidence["nonempty_messages"] = int(evidence["nonempty_messages"]) + 1
        pairs = evidence["collision_pairs"]
        assert isinstance(pairs, set)
        for contact in message.contacts:
            pairs.add(
                " <-> ".join(
                    sorted((contact.collision1.name, contact.collision2.name))
                )
            )

    def begin_recovery_contact_window(self) -> None:
        self.recovery_contact_window = {
            name: {"messages": 0, "nonempty_messages": 0, "collision_pairs": set()}
            for name in RECOVERY_CONTACT_COLLISIONS
        }

    def recovery_ground_contact_evidence(self) -> dict[str, dict[str, object]]:
        if self.recovery_contact_window is None:
            raise RuntimeError("recovery contact window was not started")
        result: dict[str, dict[str, object]] = {}
        for name, evidence in self.recovery_contact_window.items():
            pairs = evidence["collision_pairs"]
            assert isinstance(pairs, set)
            required_collision = RECOVERY_CONTACT_COLLISIONS[name]
            result[name] = {
                "message_count": int(evidence["messages"]),
                "nonempty_message_count": int(evidence["nonempty_messages"]),
                "collision_pairs": sorted(pairs),
                "ground_contact_observed": int(evidence["nonempty_messages"]) > 0
                and any(
                    required_collision in pair.lower() and "ground" in pair.lower()
                    for pair in pairs
                ),
            }
        return result

    def publish_reset(self, ground_l: float, tank_kg: float) -> None:
        self.enable.publish(Bool(data=False))
        self.reset_ground.publish(Float64(data=ground_l))
        self.reset_tank.publish(Float64(data=tank_kg))
        self.filter_blockage.publish(Float64(data=0.0))
        self.service_drain.publish(Bool(data=False))

    def publish_filter_blockage(self, fraction: float) -> None:
        self.filter_blockage.publish(Float64(data=max(0.0, min(1.0, fraction))))

    def publish_service_drain(self, open_: bool) -> None:
        self.service_drain.publish(Bool(data=open_))

    def publish_motor_fault_reset(self, requested: bool) -> None:
        self.motor_fault_reset.publish(Bool(data=requested))

    def publish_cleaning_pose(
        self, lift_m: float = 0.100, *, duration_s: float = 3.0
    ) -> None:
        point = JointTrajectoryPoint()
        # Physical zero is safe raised; the 100 mm positive downstroke is the
        # frozen ground-tangent working position.
        # With the corrected carrier/nozzle installation this places the
        # rubber blade within 2-6 mm and the intake opening within 4-8 mm.
        point.positions = [lift_m]
        duration_ns = max(1, int(round(duration_s * 1_000_000_000)))
        point.time_from_start.sec = duration_ns // 1_000_000_000
        point.time_from_start.nanosec = duration_ns % 1_000_000_000
        trajectory = JointTrajectory()
        trajectory.joint_names = ["cleaning_lift_joint"]
        trajectory.points = [point]
        self.cleaning.publish(trajectory)

    def command(
        self, *, brushes: bool, pump: bool, speed: float, enabled: bool = True
    ) -> None:
        self.main_power.publish(Bool(data=True))
        self.estop.publish(Bool(data=False))
        self.estop_reset.publish(Bool(data=True))
        self.heartbeat.publish(Empty())
        self.enable.publish(Bool(data=enabled))
        self.brush.publish(
            Float64MultiArray(data=[8.0, -8.0, 12.0] if brushes else [0.0, 0.0, 0.0])
        )
        self.pump.publish(Float64MultiArray(data=[20.0] if pump else [0.0]))
        twist = Twist()
        twist.linear.x = speed
        self.drive.publish(twist)

    def stop(self) -> None:
        self.enable.publish(Bool(data=False))
        self.service_drain.publish(Bool(data=False))
        self.brush.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
        self.pump.publish(Float64MultiArray(data=[0.0]))
        self.drive.publish(Twist())
        self.estop.publish(Bool(data=True))
        self.estop_reset.publish(Bool(data=False))
        self.main_power.publish(Bool(data=False))


def cycle_wall(node: Probe, duration_s: float, callback=None) -> None:
    """Brief wall-time cycling for startup/shutdown only, never acceptance physics."""
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        if callback is not None:
            callback()
        rclpy.spin_once(node, timeout_sec=0.05)


def wait_for_sim_condition(
    node: Probe,
    predicate,
    *,
    label: str,
    timeout_sim_s: float,
    hard_wall_s: float,
    callback=None,
) -> None:
    """Wait on simulation progress, using wall time only to detect a dead clock."""
    wall_started = time.monotonic()
    last_progress_wall = wall_started
    sim_started: float | None = None
    last_sim: float | None = None
    while True:
        if callback is not None:
            callback()
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return

        now = time.monotonic()
        current_sim = node.sim_time_s
        if current_sim is not None:
            if sim_started is None:
                sim_started = current_sim
            if last_sim is None or current_sim > last_sim + 1e-9:
                last_sim = current_sim
                last_progress_wall = now
            if current_sim - sim_started >= timeout_sim_s:
                raise RuntimeError(
                    f"{label} did not complete within {timeout_sim_s:.1f} simulated seconds: "
                    f"{node.status}"
                )
        if now - last_progress_wall >= SIM_CLOCK_STALL_WALL_S:
            raise RuntimeError(
                f"simulation clock stalled for {SIM_CLOCK_STALL_WALL_S:.1f} wall seconds "
                f"while waiting for {label}: {node.status}"
            )
        if now - wall_started >= hard_wall_s:
            raise RuntimeError(
                f"{label} exceeded the {hard_wall_s:.1f} wall-second safety limit: "
                f"{node.status}"
            )


def advance_sim_time(
    node: Probe,
    duration_sim_s: float,
    *,
    label: str,
    hard_wall_s: float,
    callback=None,
) -> None:
    phase_start: list[float | None] = [None]

    def elapsed() -> bool:
        if node.sim_time_s is None:
            return False
        if phase_start[0] is None:
            phase_start[0] = node.sim_time_s
        return node.sim_time_s - phase_start[0] >= duration_sim_s

    wait_for_sim_condition(
        node,
        elapsed,
        label=label,
        timeout_sim_s=duration_sim_s + 1.0,
        hard_wall_s=hard_wall_s,
        callback=callback,
    )


def wait_ready(node: Probe, timeout_s: float = 120.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        publishers = (
            node.enable,
            node.cleaning,
            node.brush,
            node.pump,
            node.drive,
            node.filter_blockage,
            node.service_drain,
            node.motor_fault_reset,
        )
        if (
            node.status is not None
            and node.safety_permit is not None
            and node.actuator_permit is not None
            and all(pub.get_subscription_count() > 0 for pub in publishers)
            and all(
                node.get_publishers_info_by_topic(topic)
                for topic in (
                    "/cleaning/squeegee/contact",
                    "/cleaning/left_side_brush/contact",
                    "/cleaning/right_side_brush/contact",
                    "/cleaning/central_roller/contact",
                )
            )
        ):
            return
    counts = {
        "enable": node.enable.get_subscription_count(),
        "cleaning": node.cleaning.get_subscription_count(),
        "brush": node.brush.get_subscription_count(),
        "pump": node.pump.get_subscription_count(),
        "drive": node.drive.get_subscription_count(),
        "filter_blockage": node.filter_blockage.get_subscription_count(),
        "service_drain": node.service_drain.get_subscription_count(),
        "motor_fault_reset": node.motor_fault_reset.get_subscription_count(),
    }
    raise RuntimeError(
        "water recovery topics, safety status or controller subscriptions did not "
        f"become ready: status_seen={node.status is not None}, "
        f"safety_status_seen={node.safety_permit is not None}, "
        f"actuator_permit_seen={node.actuator_permit is not None}, "
        f"subscription_counts={counts}, contact_publishers={{"
        f"squeegee={len(node.get_publishers_info_by_topic('/cleaning/squeegee/contact'))}, "
        f"left={len(node.get_publishers_info_by_topic('/cleaning/left_side_brush/contact'))}, "
        f"right={len(node.get_publishers_info_by_topic('/cleaning/right_side_brush/contact'))}, "
        f"roller={len(node.get_publishers_info_by_topic('/cleaning/central_roller/contact'))}}}"
    )


def reset_episode(node: Probe, ground_l: float, tank_kg: float) -> dict[str, object]:
    def reset_applied() -> bool:
        if node.status is None:
            return False
        return (
            abs(float(node.status["ground_volume_l"]) - ground_l) <= 1e-5
            and abs(float(node.status["tank_mass_kg"]) - tank_kg) <= 1e-5
        )

    wait_for_sim_condition(
        node,
        reset_applied,
        label="water and tank reset acknowledgement",
        timeout_sim_s=15.0,
        hard_wall_s=360.0,
        callback=lambda: node.publish_reset(ground_l, tank_kg),
    )
    if node.status is None:
        raise RuntimeError("missing status after reset")
    status = dict(node.status)
    if abs(float(status["ground_volume_l"]) - ground_l) > 1e-5:
        raise RuntimeError(f"ground reset was not applied: {status}")
    if abs(float(status["tank_mass_kg"]) - tank_kg) > 1e-5:
        raise RuntimeError(f"tank reset was not applied: {status}")
    return status


def wait_for_safety_permit_after_reset(node: Probe, *, scenario: str) -> None:
    """Re-establish the product safety handshake after the preflight handoff."""

    initial_status_count = node.safety_status_json_count
    wait_for_sim_condition(
        node,
        lambda: node.safety_status_json_count > initial_status_count
        and node.safety_json_permit is True
        and node.actuator_permit is True,
        label=f"{scenario} safety permit recovery after preflight handoff",
        timeout_sim_s=5.0,
        hard_wall_s=180.0,
        callback=lambda: node.command(brushes=False, pump=False, speed=0.0),
    )
    node.safety_handshake_events.append(
        {
            "scenario": scenario,
            "initial_status_json_count": initial_status_count,
            "confirmed_status_json_count": node.safety_status_json_count,
            "confirmed_permit": node.safety_json_permit,
            "confirmed_actuator_permit": node.actuator_permit,
            "confirmed_state": node.safety_state,
            "confirmed_active_reasons": node.safety_active_reasons,
            "sim_time_s": node.sim_time_s,
        }
    )


def lower_until_geometry_ready(node: Probe, timeout_sim_s: float = 60.0) -> None:
    if node.status is None or "cleaning_lift_position_m" not in node.status:
        raise RuntimeError("actual cleaning_lift_joint position is unavailable")
    if node.status.get("base_pose_available") is not True:
        raise RuntimeError(
            "physical base pose is unavailable: "
            f"source={node.status.get('base_pose_source', 'unreported')}"
        )
    if node.safety_permit is not True or node.actuator_permit is not True:
        raise RuntimeError(
            "cleaning lift cannot lower before the formal safety and controller permits are enabled: "
            f"state={node.safety_state}, reasons={node.safety_active_reasons}, "
            f"actuator_permit={node.actuator_permit}"
        )
    target_m = 0.100
    initial_position_m = float(node.status["cleaning_lift_position_m"])
    node.publish_cleaning_pose(
        target_m,
        duration_s=trajectory_duration_s(initial_position_m, target_m),
    )
    node.lift_recovery_events.append(
        {
            "event": "initial_target",
            "sim_time_s": node.sim_time_s,
            "actual_position_m": initial_position_m,
            "target_position_m": target_m,
            "duration_s": trajectory_duration_s(initial_position_m, target_m),
        }
    )
    supervisor = CleaningLiftRecoverySupervisor(target_position_m=target_m)

    def maintain_safe_lowering() -> None:
        node.command(brushes=False, pump=False, speed=0.0)
        status = node.status or {}
        actual_position = status.get("cleaning_lift_position_m")
        reissue = supervisor.observe(
            sim_time_s=node.sim_time_s,
            actual_position_m=(
                float(actual_position) if actual_position is not None else None
            ),
            safety_permit=node.safety_permit,
        )
        if supervisor.exhausted:
            raise RuntimeError(
                "cleaning lift exceeded the bounded safety-recovery reissue count: "
                f"position={actual_position}, state={node.safety_state}, "
                f"reasons={node.safety_active_reasons}"
            )
        if reissue is None:
            return
        node.publish_cleaning_pose(
            reissue.target_position_m, duration_s=reissue.duration_s
        )
        node.lift_recovery_events.append(
            {
                "event": "safety_recovery_reissue",
                "attempt": reissue.attempt,
                "sim_time_s": node.sim_time_s,
                "actual_position_m": reissue.actual_position_m,
                "target_position_m": reissue.target_position_m,
                "duration_s": reissue.duration_s,
            }
        )

    wait_for_sim_condition(
        node,
        lambda: node.status is not None
        and bool(node.status["squeegee_ready"])
        and bool(node.status["nozzle_ready"]),
        label="cleaning geometry ground envelope",
        timeout_sim_s=timeout_sim_s,
        hard_wall_s=1_200.0,
        callback=maintain_safe_lowering,
    )


def recover_latched_cleaning_motor_fault(node: Probe) -> dict[str, object]:
    """Perform the bounded product reset sequence after an intentional load gate."""

    if not bool(node.latest_motor_status.get("fault_active", False)):
        return {"required": False, "attempts": 0}
    initial = dict(node.latest_motor_status)
    failure: str | None = None
    for attempt in range(1, 3):
        def idle(reset: bool = False) -> None:
            node.publish_filter_blockage(0.0)
            node.command(brushes=False, pump=False, speed=0.0)
            node.publish_motor_fault_reset(reset)

        # The motor core intentionally rejects reset while any command is
        # non-idle.  First hold the post-safety outputs at zero, then pulse the
        # physical reset line and finally require the permit to recover.
        advance_sim_time(
            node,
            0.5,
            label=f"cleaning motor reset idle dwell attempt {attempt}",
            hard_wall_s=120.0,
            callback=lambda: idle(False),
        )
        advance_sim_time(
            node,
            0.25,
            label=f"cleaning motor reset pulse attempt {attempt}",
            hard_wall_s=120.0,
            callback=lambda: idle(True),
        )
        node.publish_motor_fault_reset(False)
        try:
            wait_for_sim_condition(
                node,
                lambda: not bool(
                    node.latest_motor_status.get("fault_active", True)
                ),
                label=f"cleaning motor fault clear attempt {attempt}",
                timeout_sim_s=3.0,
                hard_wall_s=120.0,
                callback=lambda: idle(False),
            )
            wait_for_sim_condition(
                node,
                lambda: node.safety_permit is True and node.actuator_permit is True,
                label=f"safety permit recovery after motor reset attempt {attempt}",
                timeout_sim_s=5.0,
                hard_wall_s=180.0,
                callback=lambda: idle(False),
            )
            return {
                "required": True,
                "attempts": attempt,
                "initial": initial,
                "terminal": dict(node.latest_motor_status),
            }
        except RuntimeError as error:
            failure = str(error)
    raise RuntimeError(
        "cleaning motor fault did not clear through the bounded two-attempt "
        f"idle/reset/permit sequence: {failure}; status={node.latest_motor_status}"
    )


def fail_on_cleaning_motor_fault(node: Probe, label: str) -> None:
    if not bool(node.latest_motor_status.get("fault_active", False)):
        return
    faults = [
        {"name": motor.get("name"), "fault": motor.get("fault")}
        for motor in node.latest_motor_status.get("motors", [])
        if bool(motor.get("protection_active", False))
    ]
    raise RuntimeError(f"{label}: cleaning motor fault is terminal: {faults}")


def wait_for_applied_mass(node: Probe, target_kg: float, timeout_sim_s: float = 5.0) -> None:
    wait_for_sim_condition(
        node,
        lambda: node.applied_mass is not None
        and abs(node.applied_mass - target_kg) <= 1e-5,
        label="dynamic payload applied-mass acknowledgement",
        timeout_sim_s=timeout_sim_s,
        hard_wall_s=300.0,
    )


def run_lift_diagnostic(node: Probe) -> dict[str, object]:
    """Exercise only safety-gated lift lowering and retain transition causality."""
    reset_episode(node, 0.0, 0.0)
    wait_for_safety_permit_after_reset(node, scenario="diagnostic")
    lower_until_geometry_ready(node, timeout_sim_s=35.0)
    advance_sim_time(
        node,
        2.0,
        label="post-lowering diagnostic observation",
        hard_wall_s=180.0,
        callback=lambda: node.command(brushes=False, pump=False, speed=0.0),
    )
    final = dict(node.status or {})
    checks = {
        "base_pose_available": bool(final["base_pose_available"]),
        "squeegee_ready": bool(final["squeegee_ready"]),
        "nozzle_ready": bool(final["nozzle_ready"]),
        "no_ground_water_transferred": abs(float(final["recovered_volume_l"]))
        <= 1e-9,
    }
    return {
        "scenario": "diagnostic",
        "checks": checks,
        "metrics": {
            "terminal_status": final,
            "lift_recovery_events": list(node.lift_recovery_events),
            "safety_handshake_events": list(node.safety_handshake_events),
            "safety_transition_history": list(node.safety_transition_history),
        },
        "passed": all(checks.values()),
    }


def reverse_to_water_start(node: Probe, timeout_sim_s: float = 12.0) -> dict[str, float]:
    """Pre-position the raised machine ahead of the immutable water footprint."""
    start_x = node.odom.pose.pose.position.x if node.odom is not None else math.nan

    def reverse_command() -> None:
        node.publish_cleaning_pose(lift_m=0.0)
        node.command(brushes=False, pump=False, speed=-0.05, enabled=False)

    wait_for_sim_condition(
        node,
        lambda: node.status is not None
        and float(node.status["nozzle_world_x"]) <= -0.62,
        label="raised reverse pre-position",
        timeout_sim_s=timeout_sim_s,
        hard_wall_s=300.0,
        callback=reverse_command,
    )
    node.command(brushes=False, pump=False, speed=0.0, enabled=False)
    advance_sim_time(
        node,
        0.5,
        label="raised reverse stop settling",
        hard_wall_s=90.0,
        callback=lambda: node.command(
            brushes=False, pump=False, speed=0.0, enabled=False
        ),
    )
    end_x = node.odom.pose.pose.position.x if node.odom is not None else math.nan
    return {
        "base_start_x_m": start_x,
        "base_end_x_m": end_x,
        "reverse_distance_m": start_x - end_x,
        "terminal_nozzle_world_x_m": float((node.status or {})["nozzle_world_x"]),
    }


def run_normal(node: Probe) -> dict[str, object]:
    initial = reset_episode(node, 2.88, 0.0)
    wait_for_safety_permit_after_reset(node, scenario="normal")
    initial_ground = float(initial["ground_volume_l"])

    # Negative gate 1: enabled water model without brush or pump.
    advance_sim_time(
        node,
        2.0,
        label="disabled-system negative gate",
        hard_wall_s=120.0,
        callback=lambda: node.command(brushes=False, pump=False, speed=0.0),
    )
    disabled_status = dict(node.status or {})

    # Negative gate 2: pump alone cannot recover water.
    advance_sim_time(
        node,
        2.0,
        label="pump-without-brush negative gate",
        hard_wall_s=120.0,
        callback=lambda: node.command(brushes=False, pump=True, speed=0.0),
    )
    pump_only_status = dict(node.status or {})

    # The first water strip is behind the initial nozzle pose.  With cleaning
    # raised and the pump disabled, physically reverse ahead of the unchanged
    # scene footprint; only then lower and start the recovery pass.
    preposition_ground = float(pump_only_status["ground_volume_l"])
    preposition = reverse_to_water_start(node)
    preposition_status = dict(node.status or {})

    # Lower the physical lift/float/pitch joints, then continuously command all
    # three brush actuators, pump rotor and the real safety-gated A300 plant.
    lower_until_geometry_ready(node)

    # Negative gate 3: a clogged physical strainer raises differential
    # pressure above the protection threshold and must stop liquid transfer.
    blocked_ground = float((node.status or {})["ground_volume_l"])
    advance_sim_time(
        node,
        2.0,
        label="blocked-filter fail-closed gate",
        hard_wall_s=120.0,
        callback=lambda: (
            node.publish_filter_blockage(1.0),
            node.command(brushes=True, pump=True, speed=0.0),
        ),
    )
    blocked_filter_status = dict(node.status or {})
    node.publish_filter_blockage(0.0)
    wait_for_sim_condition(
        node,
        lambda: node.status is not None
        and not bool(node.status.get("filter_protection_active", True)),
        label="filter service recovery",
        timeout_sim_s=3.0,
        hard_wall_s=120.0,
        callback=lambda: node.publish_filter_blockage(0.0),
    )
    motor_fault_recovery = {"required": False, "attempts": 0}
    if bool(node.latest_motor_status.get("fault_active", False)):
        faults = [
            {"name": motor.get("name"), "fault": motor.get("fault")}
            for motor in node.latest_motor_status.get("motors", [])
            if bool(motor.get("protection_active", False))
        ]
        raise RuntimeError(
            "blocked-filter gate must not use a cleaning-motor fault as its "
            f"negative result; motor_faults={faults}"
        )
    node.status_samples.clear()
    node.motor_status_samples.clear()
    start_odom = node.odom.pose.pose.position if node.odom is not None else None
    start_sim_time = node.sim_time_s
    def normal_pass_complete() -> bool:
        if bool(node.latest_motor_status.get("fault_active", False)):
            raise RuntimeError(
                "cleaning motor fault became terminal during normal pass: "
                f"{node.latest_motor_status}"
            )
        if node.status is None:
            return False
        recovered = float(node.status["recovered_volume_l"])
        nozzle_x = float(node.status["nozzle_world_x"])
        return recovered / initial_ground >= 0.955 and nozzle_x >= 1.87

    def normal_pass_command() -> None:
        # 0.05 m/s through a 2 mm x 0.6 m layer requires 3.60 L/min,
        # remaining below the derated 10.57 L/min pump limit with enough
        # dwell time to drain each finite strip instead of skipping its edge.
        node.command(brushes=True, pump=True, speed=0.05)

    node.begin_recovery_contact_window()
    wait_for_sim_condition(
        node,
        normal_pass_complete,
        label="24-column normal recovery pass",
        timeout_sim_s=NORMAL_PASS_TIMEOUT_SIM_S,
        hard_wall_s=NORMAL_PASS_HARD_WALL_S,
        callback=normal_pass_command,
    )
    node.stop()
    advance_sim_time(
        node,
        2.0,
        label="normal-pass stop settling",
        hard_wall_s=90.0,
        callback=node.stop,
    )
    final = dict(node.status or {})

    final_ground = float(final["ground_volume_l"])
    final_tank = float(final["tank_mass_kg"])
    wait_for_applied_mass(node, final_tank, timeout_sim_s=10.0)
    removed_l = initial_ground - final_ground
    tank_gain_kg = final_tank - float(initial["tank_mass_kg"])
    mass_error = abs(removed_l - tank_gain_kg) / max(removed_l, 1e-12)
    recovery_rate = removed_l / initial_ground
    max_flow = max((float(row["flow_l_min"]) for row in node.status_samples), default=0.0)
    ready_samples = [
        row for row in node.status_samples
        if all(bool(row.get(key)) for key in (
            "brush_ready", "squeegee_ready", "nozzle_ready", "pump_ready"
        ))
    ]
    ready_duty = len(ready_samples) / max(len(node.status_samples), 1)
    covered_columns = {
        min(23, max(0, int((float(row["nozzle_world_x"]) + 0.60) / 0.10)))
        for row in ready_samples
        if -0.60 <= float(row["nozzle_world_x"]) <= 1.80
        and abs(float(row["nozzle_world_y"])) <= 0.30
    }
    simultaneous_ready_seen = any(
        all(bool(row.get(key)) for key in (
            "brush_ready", "squeegee_ready", "nozzle_ready", "pump_ready"
        ))
        for row in node.status_samples
    )
    end_odom = node.odom.pose.pose.position if node.odom is not None else None
    travel_m = None
    if start_odom is not None and end_odom is not None:
        travel_m = math.hypot(end_odom.x - start_odom.x, end_odom.y - start_odom.y)
    side_brush_duty = side_brush_duty_metrics(node.motor_status_samples)
    central_roller_duty = central_roller_duty_metrics(node.motor_status_samples)
    recovery_contacts = node.recovery_ground_contact_evidence()

    checks = {
        "finite_initial_ground_water": initial_ground > 0.0,
        "disabled_system_recovery_is_zero": abs(
            float(disabled_status["ground_volume_l"]) - initial_ground
        ) <= 1e-6,
        "raised_mechanism_is_not_recovery_ready": not bool(
            disabled_status["squeegee_ready"]
        ) and (
            float(disabled_status["squeegee_blade_clearance_m"]) > 0.012
            or float(disabled_status["intake_clearance_m"]) > 0.012
        ),
        "pump_without_brush_recovery_is_zero": abs(
            float(pump_only_status["ground_volume_l"]) - initial_ground
        ) <= 1e-6,
        "blocked_filter_stops_recovery": abs(
            float(blocked_filter_status["ground_volume_l"]) - blocked_ground
        ) <= 1e-6,
        "blocked_filter_trips_pressure_protection": bool(
            blocked_filter_status["filter_protection_active"]
        ) and float(blocked_filter_status["filter_differential_pressure_kpa"]) >= 35.0,
        "raised_disabled_reverse_does_not_recover": abs(
            float(preposition_status["ground_volume_l"]) - preposition_ground
        ) <= 1e-6 and preposition["reverse_distance_m"] >= 0.15,
        "all_physical_proxy_conditions_seen": simultaneous_ready_seen,
        "squeegee_blade_has_ground_contact_during_recovery": bool(
            recovery_contacts["squeegee"]["ground_contact_observed"]
        ),
        "brush_disks_have_ground_contact_during_recovery": all(
            bool(recovery_contacts[name]["ground_contact_observed"])
            for name in ("left_side_brush", "right_side_brush", "central_roller")
        ),
        "lowered_blade_clearance_is_physical": -0.004 <= float(
            final["squeegee_blade_clearance_m"]
        ) <= 0.012,
        "lowered_intake_gap_is_physical": -0.002 <= float(
            final["intake_clearance_m"]
        ) <= 0.012,
        "vehicle_physically_advanced": travel_m is not None and travel_m >= 2.35,
        "ready_duty_cycle_at_least_0_90": ready_duty >= 0.90,
        "nozzle_covered_all_24_water_columns": len(covered_columns) == 24,
        "recovery_rate_at_least_0_95": recovery_rate >= 0.95,
        "pump_flow_within_rated_derated_limit": max_flow <= PUMP_LIMIT_L_MIN + 0.05,
        "ground_to_tank_mass_error_at_most_0_01": mass_error <= 0.01,
        "plugin_reported_mass_error_at_most_0_01": float(
            final["mass_balance_error_fraction"]
        ) <= 0.01,
        "dynamic_payload_applied_matches_tank": node.applied_mass is not None
        and abs(node.applied_mass - final_tank) <= 1e-5,
        "visual_water_fraction_matches_ground_state": abs(
            float(final["visual_remaining_fraction"]) - final_ground / initial_ground
        ) <= 1e-6 and int(final["water_visual_count"]) == 24
        and bool(final["visual_layout_ready"]),
        "normal_episode_did_not_overflow": not bool(final["tank_full"]),
        "side_brush_motor_samples_present": all(
            metrics["sample_count"] >= 20
            and metrics["commanded_sample_count"] >= 20
            and metrics["steady_sample_count"] >= 20
            for metrics in side_brush_duty.values()
        ),
        "side_brush_steady_current_within_0_75_a_continuous_rating": all(
            metrics["steady_peak_current_a"] <= 0.75 + 1e-6
            for metrics in side_brush_duty.values()
        ),
        "side_brush_over_rating_contiguous_at_most_1_s": all(
            metrics["maximum_contiguous_over_rated_s"] <= 1.0 + 1e-6
            for metrics in side_brush_duty.values()
        ),
        "side_brush_peak_temperature_below_60_c": all(
            metrics["peak_temperature_c"] < 60.0
            for metrics in side_brush_duty.values()
        ),
        "side_brush_steady_p05_speed_ratio_at_least_0_80": all(
            metrics["p05_tracking_ratio"] >= 0.80 - 1e-6
            for metrics in side_brush_duty.values()
        ),
        "side_brush_direction_matches_command": all(
            bool(metrics["direction_matches_all_steady_samples"])
            for metrics in side_brush_duty.values()
        ),
        "side_brush_low_speed_contiguous_at_most_1_s": all(
            metrics["maximum_contiguous_low_speed_s"] <= 1.0 + 1e-6
            for metrics in side_brush_duty.values()
        ),
        "side_brush_fault_free_throughout_normal_pass": all(
            bool(metrics["fault_free_all_samples"])
            for metrics in side_brush_duty.values()
        ),
        "side_brush_telemetry_fields_finite": all(
            bool(metrics["all_fields_finite"])
            for metrics in side_brush_duty.values()
        ),
        "central_roller_motor_samples_present": (
            central_roller_duty["sample_count"] >= 20
            and central_roller_duty["commanded_sample_count"] >= 20
            and central_roller_duty["steady_sample_count"] >= 20
        ),
        "central_roller_steady_current_within_0_75_a_continuous_rating": (
            central_roller_duty["steady_peak_current_a"] <= 0.75 + 1e-6
        ),
        "central_roller_over_rating_contiguous_at_most_1_s": (
            central_roller_duty["maximum_contiguous_over_rated_s"] <= 1.0 + 1e-6
        ),
        "central_roller_peak_temperature_below_60_c": (
            central_roller_duty["peak_temperature_c"] < 60.0
        ),
        "central_roller_steady_p05_speed_ratio_at_least_0_80": (
            central_roller_duty["p05_tracking_ratio"] >= 0.80 - 1e-6
        ),
        "central_roller_direction_matches_command": bool(
            central_roller_duty["direction_matches_all_steady_samples"]
        ),
        "central_roller_low_speed_contiguous_at_most_1_s": (
            central_roller_duty["maximum_contiguous_low_speed_s"] <= 1.0 + 1e-6
        ),
        "central_roller_fault_free_throughout_normal_pass": bool(
            central_roller_duty["fault_free_all_samples"]
        ),
        "central_roller_telemetry_fields_finite": bool(
            central_roller_duty["all_fields_finite"]
        ),
    }
    return {
        "scenario": "normal_recovery",
        "checks": checks,
        "initial": initial,
        "disabled_system_terminal": disabled_status,
        "pump_only_terminal": pump_only_status,
        "blocked_filter_terminal": blocked_filter_status,
        "raised_reverse_terminal": preposition_status,
        "final": final,
        "metrics": {
            "initial_ground_volume_l": initial_ground,
            "final_ground_volume_l": final_ground,
            "ground_removed_l": removed_l,
            "tank_mass_gain_kg": tank_gain_kg,
            "dynamic_payload_applied_mass_kg": node.applied_mass,
            "recovery_rate": recovery_rate,
            "mass_balance_error_fraction": mass_error,
            "maximum_observed_flow_l_min": max_flow,
            "rated_derated_flow_limit_l_min": PUMP_LIMIT_L_MIN,
            "vehicle_xy_travel_m": travel_m,
            "simulated_elapsed_s": (
                node.sim_time_s - start_sim_time
                if node.sim_time_s is not None and start_sim_time is not None
                else None
            ),
            "all_conditions_ready_duty_cycle": ready_duty,
            "nozzle_covered_column_count": len(covered_columns),
            "nozzle_covered_columns": sorted(covered_columns),
            "status_samples": len(node.status_samples),
            "preposition": preposition,
            "terminal_squeegee_blade_clearance_m": float(
                final["squeegee_blade_clearance_m"]
            ),
            "terminal_intake_clearance_m": float(final["intake_clearance_m"]),
            "terminal_cleaning_lift_position_m": float(
                final["cleaning_lift_position_m"]
            ),
            "terminal_base_world_z_m": float(final["base_world_z_m"]),
            "terminal_base_world_roll_rad": float(final["base_world_roll_rad"]),
            "terminal_base_world_pitch_rad": float(final["base_world_pitch_rad"]),
            "terminal_base_pose_available": bool(final["base_pose_available"]),
            "terminal_base_pose_source": str(final["base_pose_source"]),
            "lift_recovery_events": list(node.lift_recovery_events),
            "safety_handshake_events": list(node.safety_handshake_events),
            "motor_fault_recovery": motor_fault_recovery,
            "side_brush_duty": side_brush_duty,
            "central_roller_duty": central_roller_duty,
            "recovery_ground_contact_evidence": recovery_contacts,
            "safety_handshake_events": list(node.safety_handshake_events),
        },
        "passed": all(checks.values()),
    }


def run_full(node: Probe) -> dict[str, object]:
    # Keep the same 56.4 mL headroom used by the original full-tank case while
    # deriving the starting mass from the final expanded-URDF payload limit.
    initial_tank = TANK_CAPACITY_KG - 0.0564
    initial = reset_episode(node, 0.40, initial_tank)
    wait_for_safety_permit_after_reset(node, scenario="full")
    lower_until_geometry_ready(node)
    node.status_samples.clear()
    node.motor_status_samples.clear()

    def full_tank_command() -> None:
        fail_on_cleaning_motor_fault(node, "full-tank active recovery")
        node.command(brushes=True, pump=True, speed=0.065)

    def full_stationary_command() -> None:
        fail_on_cleaning_motor_fault(node, "full-tank stationary observation")
        node.command(brushes=True, pump=True, speed=0.0)

    def tank_full_without_motor_fault() -> bool:
        if bool(node.latest_motor_status.get("fault_active", False)):
            raise RuntimeError(
                "cleaning motor fault became terminal during full-tank pass: "
                f"{node.latest_motor_status}"
            )
        return node.status is not None and bool(node.status["tank_full"])

    wait_for_sim_condition(
        node,
        tank_full_without_motor_fault,
        label="real wastewater tank-full state",
        timeout_sim_s=FULL_TANK_TIMEOUT_SIM_S,
        hard_wall_s=FULL_TANK_HARD_WALL_S,
        callback=full_tank_command,
    )
    at_full = dict(node.status or {})
    ground_at_full = float(at_full["ground_volume_l"])
    tank_at_full = float(at_full["tank_mass_kg"])
    advance_sim_time(
        node,
        2.0,
        label="post-full fail-closed observation",
        hard_wall_s=120.0,
        callback=full_stationary_command,
    )
    terminal = dict(node.status or {})
    wait_for_applied_mass(node, float(terminal["tank_mass_kg"]), timeout_sim_s=10.0)
    applied_at_full = node.applied_mass
    applied_before_service_drain = applied_at_full

    # A drain request while recovery actuators remain active must be rejected
    # by the physical valve interlock.  The request is retained in telemetry,
    # while the actual valve state and drained volume remain unchanged.
    advance_sim_time(
        node,
        1.0,
        label="active-recovery service-drain interlock",
        hard_wall_s=90.0,
        callback=lambda: (
            full_tank_command(),
            node.publish_service_drain(True),
        ),
    )
    drain_interlock_terminal = dict(node.status or {})
    node.publish_service_drain(False)

    # The physical low-point service port must reduce both the simulated tank
    # mass and the vehicle dynamic payload.  Splash / receiving-vessel CFD is
    # intentionally outside the contest-required recovery model.
    advance_sim_time(
        node,
        2.0,
        label="wastewater service drain",
        hard_wall_s=120.0,
        callback=lambda: (
            node.stop(),
            node.publish_service_drain(True),
        ),
    )
    node.publish_service_drain(False)
    advance_sim_time(
        node,
        0.2,
        label="service drain valve close",
        hard_wall_s=60.0,
        callback=lambda: node.publish_service_drain(False),
    )
    after_drain = dict(node.status or {})
    drain_permit_seen = any(
        bool(sample.get("service_drain_permitted"))
        for sample in node.status_samples
    )
    wait_for_applied_mass(node, float(after_drain["tank_mass_kg"]), timeout_sim_s=10.0)
    applied_after_service_drain = node.applied_mass
    node.stop()

    removed_l = float(initial["ground_volume_l"]) - ground_at_full
    tank_gain = tank_at_full - initial_tank
    error = abs(removed_l - tank_gain) / max(removed_l, 1e-12)
    post_full_ground_delta = ground_at_full - float(terminal["ground_volume_l"])
    service_drain_tank_mass_reduction_kg = float(terminal["tank_mass_kg"]) - float(
        after_drain["tank_mass_kg"]
    )
    service_drain_reported_mass_kg = float(after_drain["service_drained_volume_l"])
    service_drain_payload_mass_reduction_kg = (
        applied_before_service_drain - applied_after_service_drain
        if applied_before_service_drain is not None
        and applied_after_service_drain is not None
        else None
    )
    side_brush_duty = side_brush_duty_metrics(node.motor_status_samples)
    central_roller_duty = central_roller_duty_metrics(node.motor_status_samples)
    checks = {
        "tank_reaches_full": bool(at_full["tank_full"]),
        "full_case_blade_and_intake_geometry_ready": bool(
            at_full["squeegee_ready"]
        ) and bool(at_full["nozzle_ready"]),
        "tank_mass_clamped_to_capacity": abs(tank_at_full - TANK_CAPACITY_KG) <= 1e-5,
        "water_remains_when_tank_full": ground_at_full > 0.0,
        "full_tank_stops_ground_removal": abs(post_full_ground_delta) <= 1e-6,
        "full_tank_stops_flow": abs(float(terminal["flow_l_min"])) <= 1e-6,
        "full_case_mass_error_at_most_0_01": error <= 0.01,
        "dynamic_payload_applied_matches_full_tank": node.applied_mass is not None
        and applied_at_full is not None
        and abs(applied_at_full - tank_at_full) <= 1e-5,
        "active_recovery_blocks_service_drain": bool(
            drain_interlock_terminal["service_drain_requested_open"]
        ) and not bool(drain_interlock_terminal["service_drain_open"])
        and not bool(drain_interlock_terminal["service_drain_permitted"])
        and abs(
            float(drain_interlock_terminal["tank_mass_kg"])
            - float(terminal["tank_mass_kg"])
        ) <= 1e-6
        and abs(float(drain_interlock_terminal["service_drained_volume_l"]))
        <= 1e-6,
        "service_drain_reduces_tank_mass": float(after_drain["tank_mass_kg"])
        < float(terminal["tank_mass_kg"]) - 0.25,
        "service_drain_stationary_interlock_permitted": drain_permit_seen,
        "service_drain_reports_removed_volume": float(
            after_drain["service_drained_volume_l"]
        ) >= 0.30,
        # The service-drain path is part of the physical liquid ledger, not
        # merely a UI indicator: the tank reduction, reported discharged
        # volume (water is 1 kg/L here), and DynamicPayloadSystem reduction
        # must all agree after the drain valve closes.
        "service_drain_tank_mass_matches_reported_volume": abs(
            service_drain_tank_mass_reduction_kg - service_drain_reported_mass_kg
        ) <= 0.01,
        "service_drain_payload_mass_matches_tank_reduction": (
            service_drain_payload_mass_reduction_kg is not None
            and abs(
                service_drain_payload_mass_reduction_kg
                - service_drain_tank_mass_reduction_kg
            )
            <= 1e-5
        ),
        "service_drain_closes_and_updates_payload": not bool(
            after_drain["service_drain_open"]
        ) and not bool(after_drain["service_drain_permitted"])
        and node.applied_mass is not None
        and abs(node.applied_mass - float(after_drain["tank_mass_kg"])) <= 1e-5,
        "full_case_visual_fraction_matches_ground_state": abs(
            float(terminal["visual_remaining_fraction"])
            - float(terminal["ground_volume_l"]) / float(initial["ground_volume_l"])
        ) <= 1e-6 and int(terminal["water_visual_count"]) == 24
        and bool(terminal["visual_layout_ready"]),
        "side_brush_fault_free_throughout_full_case": all(
            bool(metrics["fault_free_all_samples"])
            for metrics in side_brush_duty.values()
        ),
        "central_roller_fault_free_throughout_full_case": bool(
            central_roller_duty["fault_free_all_samples"]
        ),
    }
    return {
        "scenario": "full_tank_fail_closed",
        "checks": checks,
        "initial": initial,
        "at_full": at_full,
        "terminal": terminal,
        "active_recovery_drain_interlock_terminal": drain_interlock_terminal,
        "after_service_drain": after_drain,
        "metrics": {
            "ground_removed_l": removed_l,
            "tank_mass_gain_kg": tank_gain,
            "dynamic_payload_applied_mass_kg": node.applied_mass,
            "dynamic_payload_applied_at_full_kg": applied_at_full,
            "service_drained_volume_l": float(
                after_drain["service_drained_volume_l"]
            ),
            "service_drain_tank_mass_reduction_kg": service_drain_tank_mass_reduction_kg,
            "service_drain_reported_mass_kg": service_drain_reported_mass_kg,
            "service_drain_payload_mass_reduction_kg": service_drain_payload_mass_reduction_kg,
            "mass_balance_error_fraction": error,
            "remaining_ground_volume_l": ground_at_full,
            "post_full_ground_delta_l": post_full_ground_delta,
            "at_full_cleaning_lift_position_m": float(
                at_full["cleaning_lift_position_m"]
            ),
            "at_full_base_world_z_m": float(at_full["base_world_z_m"]),
            "at_full_base_world_roll_rad": float(at_full["base_world_roll_rad"]),
            "at_full_base_world_pitch_rad": float(at_full["base_world_pitch_rad"]),
            "lift_recovery_events": list(node.lift_recovery_events),
            "side_brush_duty": side_brush_duty,
            "central_roller_duty": central_roller_duty,
        },
        "passed": all(checks.values()),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scenario", choices=("normal", "full", "diagnostic"), required=True
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument(
        "--runtime-binding",
        type=Path,
        help="Required by the formal all-scenarios runner; omitted only for non-canonical diagnostics.",
    )
    parser.add_argument("--preembedded-report", type=Path)
    parser.add_argument("--preembedded-world", type=Path)
    parser.add_argument("--preembedded-model-pose", default="0 0 0.005 0 0 0")
    args = parser.parse_args()
    runtime_evidence: tuple[dict[str, str], dict[str, object], dict[str, object]] | None = None
    if args.runtime_binding is not None:
        runtime_evidence = _bound_runtime_evidence(
            args.snapshot, args.session, args.runtime_binding
        )
    preembedded_world_binding: dict[str, object] | None = None
    if runtime_evidence is not None:
        if args.preembedded_report is None or args.preembedded_world is None:
            raise SystemExit(
                "formal water acceptance requires --preembedded-report and "
                "--preembedded-world"
            )
        source_binding, acceptance_session_binding, runtime_gate_binding = runtime_evidence
        runtime_closure_binding = runtime_gate_binding.get("runtime_closure_binding")
        if not isinstance(runtime_closure_binding, dict):
            raise ValueError("water runtime binding has no runtime closure binding")
        runtime_install_root = runtime_closure_binding.get("runtime_install_root")
        if not isinstance(runtime_install_root, str) or not runtime_install_root:
            raise ValueError("water runtime closure has no install root")
        preembedded_world_binding = validate_preembedded_sensor_world(
            report_path=args.preembedded_report,
            world_path=args.preembedded_world,
            expanded_urdf_path=REPOSITORY_ROOT
            / "reports/engineering/formal_competition_vehicle.urdf",
            acceptance_session={
                "started_epoch_ns": acceptance_session_binding["session_started_epoch_ns"],
                "session_manifest_sha256": acceptance_session_binding[
                    "session_manifest_sha256"
                ],
            },
            snapshot_identity=source_binding,
            expected_model_pose=args.preembedded_model_pose,
            expected_runtime_install_root=Path(runtime_install_root),
        )
    rclpy.init()
    node = Probe()
    try:
        wait_ready(node)
        if args.scenario == "normal":
            result = run_normal(node)
        elif args.scenario == "full":
            result = run_full(node)
        else:
            result = run_lift_diagnostic(node)
    except Exception as exc:  # preserve a machine-readable hard failure
        result = {
            "scenario": args.scenario,
            "passed": False,
            "checks": {},
            "error": f"{type(exc).__name__}: {exc}",
            "failure_diagnostics": {
                "last_water_status": dict(node.status or {}),
                "safety_state": node.safety_state,
                "safety_permit": node.safety_permit,
                "actuator_permit": node.actuator_permit,
                "safety_active_reasons": node.safety_active_reasons,
                "latest_cleaning_motor_status": dict(node.latest_motor_status),
                "latest_auxiliary_status": dict(node.latest_auxiliary_status),
                "safety_transition_history": list(node.safety_transition_history),
                "safety_handshake_events": list(node.safety_handshake_events),
                "lift_recovery_events": list(node.lift_recovery_events),
            },
        }
    finally:
        node.stop()
        cycle_wall(node, 0.2)
        node.destroy_node()
        rclpy.shutdown()
    result["schema_version"] = 1
    if runtime_evidence is not None:
        source_binding, acceptance_session_binding, runtime_gate_binding = runtime_evidence
        result["source_binding"] = source_binding
        result["acceptance_session_binding"] = acceptance_session_binding
        result["runtime_gate_binding"] = runtime_gate_binding
        result["preembedded_sensor_world_binding"] = preembedded_world_binding
    result["status"] = (
        "FORMAL_WATER_RECOVERY_SCENARIO_PASSED"
        if result["passed"]
        else "FAILED"
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
