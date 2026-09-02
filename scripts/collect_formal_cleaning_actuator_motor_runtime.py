#!/usr/bin/env python3
"""Exercise and collect the formal cleaning-motor Gazebo runtime.

The live scenario only publishes product commands. The stall is induced by
asking the P16 lift controller to move beyond its 100 mm physical travel stop;
joint state is observed, never written by this probe.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path
from typing import Any, Callable

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from formal_cleaning_motor_telemetry import (
    decode_cleaning_motor_telemetry,
    update_physics_revision_watchdog,
)


ROOT = "/model/tzcup_formal_sanitation_vehicle/cleaning_motors"
MOTOR_NAMES = (
    "left_side_brush",
    "right_side_brush",
    "central_roller",
    "cleaning_lift",
    "recovery_pump",
)
LIFT_TRAVEL_UPPER_M = 0.100
STALL_REFERENCE_M = 0.125
# The loaded P16 travel measured in the formal scene is much slower than its
# 4.8 mm/s no-load limit.  Keep the physical approach and fault latch separate.
LIFT_TRAVEL_APPROACH_M = 0.0995
LIFT_TRAVEL_APPROACH_TIMEOUT_S = 155.0
STALL_LATCH_TIMEOUT_S = 6.5
RECOVERY_LIFT_TARGET_M = 0.060
# This is a wall-clock observation budget, not simulated time.  The complete
# DART vehicle can run below real time while the loaded P16 is travelling: a
# real capture needed nearly 20 wall seconds to cover only 29.3 mm after reset
# at the reported 4.8 mm/s simulation velocity.  Keep 35 seconds so the full
# 40 mm reset stroke and the final idle sample are observable without relaxing
# either the physical stop or recovery-position criteria.
RECOVERY_LIFT_TIMEOUT_S = 35.0


def _snapshot_binding(path: Path) -> dict[str, str]:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    output = manifest["outputs"]["reports/engineering/formal_competition_vehicle.urdf"]
    source_inventory_sha256 = manifest["source_inventory_sha256"]
    return {
        "snapshot_manifest_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "source_inventory_sha256": str(source_inventory_sha256),
        "expanded_urdf_sha256": str(output["sha256"]),
    }


class Collector(Node):
    def __init__(self) -> None:
        super().__init__("formal_cleaning_actuator_motor_runtime_collector")
        self.samples: list[dict[str, Any]] = []
        self.latest: dict[str, Any] = {}
        self.phase = "startup"
        self.reset_publish_count = 0
        self.lift_trajectory_published = False
        self.last_motor_telemetry_sequence: int | None = None
        self.last_motor_physics_update_sequence: int | None = None
        self.last_motor_physics_revision_advance_s: float | None = None

        self.create_subscription(
            Float64MultiArray, ROOT + "/telemetry_snapshot", self._status, 50
        )
        self.create_subscription(Bool, ROOT + "/fault_active", lambda m: self._set("fault", bool(m.data)), 50)
        self.create_subscription(Float64MultiArray, ROOT + "/motor_current_a", lambda m: self._set("current_a", list(m.data)), 50)
        self.create_subscription(Float64MultiArray, ROOT + "/motor_temperature_c", lambda m: self._set("temperature_c", list(m.data)), 50)
        self.create_subscription(Float64MultiArray, ROOT + "/estimated_output_load", lambda m: self._set("output_load", list(m.data)), 50)
        self.create_subscription(Float64, ROOT + "/total_current_a", lambda m: self._set("total_current_a", float(m.data)), 50)
        self.create_subscription(Float64, ROOT + "/total_power_w", lambda m: self._set("total_power_w", float(m.data)), 50)
        self.create_subscription(Bool, "/safety/actuators_enabled", lambda m: self._set("safety_enabled", bool(m.data)), 50)
        self.create_subscription(JointState, "/joint_states", self._joint_state, 50)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/cleaning_controller/controller_state",
            self._lift_controller_state,
            50,
        )
        self.create_subscription(DiagnosticArray, "/safety/status", self._safety_status, 20)

        self.main_power = self.create_publisher(Bool, "/formal_vehicle/simulation/command/main_power", 10)
        self.estop = self.create_publisher(Bool, "/formal_vehicle/simulation/command/emergency_stop", 10)
        self.estop_reset = self.create_publisher(Bool, "/formal_vehicle/simulation/command/emergency_stop_reset", 10)
        self.heartbeat = self.create_publisher(Empty, "/safety/control_heartbeat", 10)
        self.brush = self.create_publisher(Float64MultiArray, "/safety/command/brush", 10)
        self.pump = self.create_publisher(Float64MultiArray, "/safety/command/pump", 10)
        self.lift = self.create_publisher(JointTrajectory, "/cleaning_controller/joint_trajectory", 10)
        self.reset = self.create_publisher(Bool, ROOT + "/command/reset_faults", 10)

    def _set(self, key: str, value: Any) -> None:
        self.latest[key] = value

    def _status(self, message: Float64MultiArray) -> None:
        status = decode_cleaning_motor_telemetry(message.data)
        now_s = time.monotonic()
        telemetry_sequence = int(status["telemetry_sequence"])
        sequence = int(status["physics_update_sequence"])
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
            physics_stale=bool(status["physics_update_stale"]),
            last_sequence=self.last_motor_physics_update_sequence,
            last_advance_s=self.last_motor_physics_revision_advance_s,
            now_s=now_s,
        )
        # The encoded snapshot is the single coherent motor sample.  Do not let
        # independently bridged scalar/vector topics combine fields from
        # different physics updates in the evidence record.
        motors = status["motors"]
        self.latest.update(
            {
                "status": status,
                "fault": bool(status["fault_active"]),
                "current_a": [float(row["current_a"]) for row in motors],
                "temperature_c": [float(row["temperature_c"]) for row in motors],
                "output_load": [float(row["estimated_output_load"]) for row in motors],
                "total_current_a": float(status["total_current_a"]),
                "total_power_w": float(status["total_power_w"]),
                # The mirror command is the actuator-side reference used by
                # the physical-stall validator.  It is intentionally separate
                # from the controller target recorded by _lift_controller_state.
                "lift_reference_m": float(motors[3]["command"]),
                "motor_lift_command_m": float(motors[3]["command"]),
                "lift_position_m": float(motors[3]["measured_position"]),
                "lift_velocity_m_s": float(motors[3]["measured_speed"]),
            }
        )
        self.samples.append(
            {
                "monotonic_s": time.monotonic(),
                "phase": self.phase,
                **self.latest,
            }
        )

    def _joint_state(self, message: JointState) -> None:
        for index, name in enumerate(message.name):
            if name != "cleaning_lift_joint":
                continue
            if index < len(message.position):
                self.latest["lift_position_m"] = float(message.position[index])
            if index < len(message.velocity):
                self.latest["lift_velocity_m_s"] = float(message.velocity[index])
            break

    def _lift_controller_state(self, message: JointTrajectoryControllerState) -> None:
        if message.reference.positions:
            # Keep the native controller target distinct from the motor mirror
            # command.  During a safety inhibit the mirror correctly holds the
            # carriage at its measured position, while a retraction may already
            # be armed in the controller for the eventual reset.
            self.latest["controller_lift_reference_m"] = float(message.reference.positions[0])

    def _safety_status(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name != "whole_vehicle_safety":
                continue
            self.latest["whole_vehicle_safety_state"] = status.message
            self.latest["whole_vehicle_safety_values"] = {
                item.key: item.value for item in status.values
            }

    def publish_safe_inputs(
        self,
        *,
        brush: tuple[float, float, float] = (0.0, 0.0, 0.0),
        pump: float = 0.0,
    ) -> None:
        self.main_power.publish(Bool(data=True))
        self.estop.publish(Bool(data=False))
        self.estop_reset.publish(Bool(data=True))
        self.heartbeat.publish(Empty())
        self.brush.publish(Float64MultiArray(data=list(brush)))
        self.pump.publish(Float64MultiArray(data=[pump]))

    def publish_lift_reference(self, target_m: float, duration_s: float) -> None:
        point = JointTrajectoryPoint()
        point.positions = [target_m]
        seconds = int(duration_s)
        point.time_from_start = Duration(
            sec=seconds,
            nanosec=int((duration_s - seconds) * 1_000_000_000),
        )
        trajectory = JointTrajectory()
        trajectory.joint_names = ["cleaning_lift_joint"]
        trajectory.points = [point]
        self.lift.publish(trajectory)
        self.lift_trajectory_published = True

    def publish_reset(self) -> None:
        self.reset.publish(Bool(data=True))
        self.reset_publish_count += 1


def _spin_phase(
    node: Collector,
    phase: str,
    duration_s: float,
    *,
    brush: tuple[float, float, float] = (0.0, 0.0, 0.0),
    pump: float = 0.0,
    predicate: Callable[[Collector], bool] | None = None,
) -> bool:
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("phase duration must be finite and positive")
    node.phase = phase
    deadline = time.monotonic() + duration_s
    while rclpy.ok() and time.monotonic() < deadline:
        node.publish_safe_inputs(brush=brush, pump=pump)
        rclpy.spin_once(node, timeout_sec=0.025)
        if predicate is not None and predicate(node):
            return True
    return predicate(node) if predicate is not None else True


def _motors(node: Collector) -> list[dict[str, Any]]:
    value = node.latest.get("status", {}).get("motors", [])
    return value if isinstance(value, list) else []


def _healthy_ready(node: Collector) -> bool:
    return (
        node.latest.get("safety_enabled") is True
        and node.latest.get("fault") is False
        and [row.get("name") for row in _motors(node)] == list(MOTOR_NAMES)
        and math.isfinite(float(node.latest.get("lift_position_m", math.nan)))
    )


def _stall_and_inhibit(node: Collector) -> bool:
    motors = _motors(node)
    return (
        len(motors) == 5
        and motors[3].get("fault") == "stall"
        and node.latest.get("fault") is True
        and node.latest.get("safety_enabled") is False
    )


def _near_lift_travel_stop(node: Collector) -> bool:
    return float(node.latest.get("lift_position_m", math.nan)) >= LIFT_TRAVEL_APPROACH_M


def _reset_recovered(node: Collector) -> bool:
    motors = _motors(node)
    return (
        len(motors) == 5
        and all(row.get("fault") == "none" for row in motors)
        and node.latest.get("fault") is False
        and node.latest.get("safety_enabled") is True
    )


def _recovery_reference_armed(node: Collector) -> bool:
    """Confirm a retraction is armed while the fault still blocks motion.

    Safety deliberately holds the motor-side mirror at the measured travel
    stop until the explicit reset succeeds.  That held mirror command must not
    be mistaken for the controller target which will take effect afterwards.
    """
    return (
        _stall_and_inhibit(node)
        and math.isclose(
            float(node.latest.get("controller_lift_reference_m", math.nan)),
            RECOVERY_LIFT_TARGET_M,
            abs_tol=0.002,
        )
    )


def _lift_retracted_and_idle(node: Collector) -> bool:
    motors = _motors(node)
    currents = node.latest.get("current_a", [])
    return (
        _reset_recovered(node)
        and len(currents) == len(MOTOR_NAMES)
        and all(abs(float(current)) <= 1.0e-6 for current in currents)
        and math.isclose(
            float(node.latest.get("lift_position_m", math.nan)),
            RECOVERY_LIFT_TARGET_M,
            abs_tol=0.002,
        )
        and abs(float(node.latest.get("lift_velocity_m_s", math.nan))) <= 0.0003
        and all(row.get("fault") == "none" for row in motors)
    )


def _bridge_graph(node: Collector) -> dict[str, dict[str, Any]]:
    graph: dict[str, dict[str, Any]] = {}
    for suffix in (
        "/motor_current_a",
        "/motor_temperature_c",
        "/estimated_output_load",
    ):
        topic = ROOT + suffix
        publishers = node.get_publishers_info_by_topic(topic)
        graph[topic] = {
            "publisher_count": len(publishers),
            "publishers": [
                {
                    "node_name": endpoint.node_name,
                    "node_namespace": endpoint.node_namespace,
                    "topic_type": endpoint.topic_type,
                }
                for endpoint in publishers
            ],
            "ros_subscription_count": node.count_subscribers(topic),
        }
    return graph


def run_live_scenario(node: Collector, startup_timeout_s: float) -> None:
    if not _spin_phase(node, "startup_idle", startup_timeout_s, predicate=_healthy_ready):
        raise TimeoutError("formal vehicle motor observer and safety permit did not become ready")

    # All five motors are driven through their normal controller paths. The
    # modest brush and pump speeds remain within continuous ratings.
    node.publish_lift_reference(0.060, 9.0)
    _spin_phase(
        node,
        "normal_load",
        5.0,
        brush=(2.0, -2.0, 3.0),
        pump=10.0,
    )

    # The reference exceeds the P16's 0.100 m physical travel. Gazebo's joint
    # limit stops the carriage while the controller continues demanding motion.
    node.publish_lift_reference(STALL_REFERENCE_M, 8.0)
    if not _spin_phase(
        node,
        "physical_travel_stop_stall",
        LIFT_TRAVEL_APPROACH_TIMEOUT_S,
        predicate=_near_lift_travel_stop,
    ):
        raise TimeoutError("physical lift did not reach its travel stop approach")
    if not _spin_phase(
        node,
        "physical_travel_stop_stall",
        STALL_LATCH_TIMEOUT_S,
        predicate=_stall_and_inhibit,
    ):
        raise TimeoutError("physical lift travel stop did not latch stall and inhibit the vehicle")
    # Preserve coherent post-stall telemetry in the physical-stop phase.  The
    # predicate above can observe the first fault update before its callback
    # has emitted a collector sample.
    _spin_phase(node, "physical_travel_stop_stall", 0.25)

    # The production thermal constants are retained; this phase demonstrates
    # idle cooling after stall, not a live overtemperature trip.
    _spin_phase(node, "idle_cooling", 4.0)

    # Arm the inward controller reference *before* clearing the native safety
    # inhibit.  The safety mirror must remain at the 0.100 m measured stop
    # while faulted; after reset it will consume this already-confirmed 0.060 m
    # target instead of briefly re-demanding the travel limit and latching a
    # second physical stall.
    node.publish_lift_reference(RECOVERY_LIFT_TARGET_M, 0.5)
    if not _spin_phase(
        node,
        "recovery_reference_arm",
        6.0,
        predicate=_recovery_reference_armed,
    ):
        raise TimeoutError("recovery lift reference was not armed before explicit reset")

    node.phase = "explicit_reset"
    reset_deadline = time.monotonic() + 8.0
    while rclpy.ok() and time.monotonic() < reset_deadline:
        node.publish_safe_inputs()
        node.publish_reset()
        rclpy.spin_once(node, timeout_sec=0.05)
        if _reset_recovered(node):
            break
    if not _reset_recovered(node):
        raise TimeoutError("idle cooled motor fault did not clear after explicit reset")
    # The 0.060 m reference was armed before reset and observed directly from
    # the controller state.  Do not race reset completion with a new command.
    if not _spin_phase(
        node,
        "recovery_retract",
        RECOVERY_LIFT_TIMEOUT_S,
        predicate=_lift_retracted_and_idle,
    ):
        raise TimeoutError("reset lift did not retract to a healthy idle state")
    # Do not return on the already-valid cached sample: the formal validator
    # requires fresh ``recovered_idle`` telemetry that proves the healthy
    # state persisted after the retraction predicate first became true.
    _spin_phase(node, "recovered_idle", 1.0)
    if not _lift_retracted_and_idle(node):
        raise RuntimeError("reset lift did not remain at a healthy idle state")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    parser.add_argument("--exercise-live", action="store_true")
    parser.add_argument("--snapshot-manifest", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise ValueError("duration must be finite and positive")

    rclpy.init()
    node = Collector()
    scenario_error: str | None = None
    bridge_graph: dict[str, dict[str, Any]] = {}
    try:
        if args.exercise_live:
            run_live_scenario(node, args.startup_timeout)
        else:
            deadline = time.monotonic() + args.duration
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
    except Exception as exc:  # Preserve a truthful machine-readable failure.
        scenario_error = str(exc)
    finally:
        bridge_graph = _bridge_graph(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    artifact: dict[str, Any] = {
        "schema_version": 2,
        "report_id": "tzcup_formal_cleaning_actuator_motor_runtime_capture_v2",
        "scenario": (
            "normal_load_physical_lift_travel_stop_idle_cool_reset"
            if args.exercise_live
            else "observation_only"
        ),
        "evidence_authority": "GAZEBO_PHYSICAL_JOINT_AND_POST_SAFETY_CONTROLLER_OBSERVATION",
        "joint_state_mutation_used": False,
        "production_motor_parameters_modified": False,
        "live_overtemperature_claimed": False,
        "thermal_protection_evidence": {
            "kind": "separate_core_unit_test",
            "test": "sanitation_gazebo_control/test/test_cleaning_actuator_motor_core.cc",
            "reason": "production thermal time constants are intentionally not shortened for live acceptance",
        },
        "physical_stall_mechanism": {
            "joint": "cleaning_lift_joint",
            "travel_upper_m": LIFT_TRAVEL_UPPER_M,
            "controller_reference_m": STALL_REFERENCE_M,
        },
        "lift_trajectory_published": node.lift_trajectory_published,
        "reset_publish_count": node.reset_publish_count,
        "cleaning_vector_bridge_graph": bridge_graph,
        "sample_count": len(node.samples),
        "samples": node.samples,
    }
    if args.snapshot_manifest is not None:
        artifact["source_binding"] = _snapshot_binding(args.snapshot_manifest)
    if scenario_error is not None:
        artifact["scenario_error"] = scenario_error
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if node.samples and scenario_error is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
