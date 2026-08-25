#!/usr/bin/env python3
"""Drive and machine-accept the formal vehicle's L1 water recovery proxy."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ROOT = "/model/tzcup_formal_sanitation_vehicle/water_recovery"
PUMP_LIMIT_L_MIN = 15.1 * 0.70
TANK_CAPACITY_KG = 9.7064
SIM_CLOCK_STALL_WALL_S = 45.0
NORMAL_PASS_TIMEOUT_SIM_S = 90.0
NORMAL_PASS_HARD_WALL_S = 2_400.0
FULL_TANK_TIMEOUT_SIM_S = 30.0
FULL_TANK_HARD_WALL_S = 900.0


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("formal_water_recovery_acceptance")
        self.status: dict[str, object] | None = None
        self.status_samples: list[dict[str, object]] = []
        self.applied_mass: float | None = None
        self.odom: Odometry | None = None
        self.sim_time_s: float | None = None
        self.enable = self.create_publisher(Bool, f"{ROOT}/command/enable", 10)
        self.reset_ground = self.create_publisher(
            Float64, f"{ROOT}/command/reset_ground_volume_l", 10
        )
        self.reset_tank = self.create_publisher(
            Float64, f"{ROOT}/command/reset_tank_mass_kg", 10
        )
        self.cleaning = self.create_publisher(
            JointTrajectory, "/cleaning_controller/joint_trajectory", 10
        )
        self.brush = self.create_publisher(
            Float64MultiArray, "/brush_controller/commands", 10
        )
        self.pump = self.create_publisher(
            Float64MultiArray, "/recovery_controller/commands", 10
        )
        self.drive = self.create_publisher(
            TwistStamped, "/base_controller/cmd_vel", 10
        )
        self.create_subscription(String, f"{ROOT}/status_json", self._on_status, 50)
        self.create_subscription(
            Float64,
            "/model/tzcup_formal_sanitation_vehicle/payload/wastewater_mass_kg/applied",
            self._on_applied,
            50,
        )
        self.create_subscription(Odometry, "/base_controller/odom", self._on_odom, 50)
        self.create_subscription(Clock, "/clock", self._on_clock, 50)

    def _on_status(self, message: String) -> None:
        self.status = json.loads(message.data)
        self.status_samples.append(dict(self.status))

    def _on_applied(self, message: Float64) -> None:
        self.applied_mass = float(message.data)

    def _on_odom(self, message: Odometry) -> None:
        self.odom = message

    def _on_clock(self, message: Clock) -> None:
        self.sim_time_s = message.clock.sec + message.clock.nanosec * 1e-9

    def publish_reset(self, ground_l: float, tank_kg: float) -> None:
        self.enable.publish(Bool(data=False))
        self.reset_ground.publish(Float64(data=ground_l))
        self.reset_tank.publish(Float64(data=tank_kg))

    def publish_cleaning_pose(self, lift_m: float = 0.025) -> None:
        point = JointTrajectoryPoint()
        # The frozen nominal working position is 25 mm on the 100 mm lift.
        # With the corrected carrier/nozzle installation this places the
        # rubber blade within 2-6 mm and the intake opening within 4-8 mm.
        point.positions = [lift_m, 0.0, 0.0]
        point.time_from_start.sec = 3
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            "cleaning_lift_joint",
            "squeegee_pitch_joint",
            "squeegee_float_joint",
        ]
        trajectory.points = [point]
        self.cleaning.publish(trajectory)

    def command(
        self, *, brushes: bool, pump: bool, speed: float, enabled: bool = True
    ) -> None:
        self.enable.publish(Bool(data=enabled))
        self.brush.publish(
            Float64MultiArray(data=[8.0, -8.0, 12.0] if brushes else [0.0, 0.0, 0.0])
        )
        self.pump.publish(Float64MultiArray(data=[20.0] if pump else [0.0]))
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = speed
        self.drive.publish(twist)

    def stop(self) -> None:
        self.enable.publish(Bool(data=False))
        self.brush.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
        self.pump.publish(Float64MultiArray(data=[0.0]))
        self.drive.publish(TwistStamped())


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


def wait_ready(node: Probe, timeout_s: float = 30.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        publishers = (
            node.enable,
            node.cleaning,
            node.brush,
            node.pump,
            node.drive,
        )
        if node.status is not None and all(pub.get_subscription_count() > 0 for pub in publishers):
            return
    raise RuntimeError("water recovery topics or controller subscriptions did not become ready")


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


def lower_until_geometry_ready(node: Probe, timeout_sim_s: float = 60.0) -> None:
    # Publish a short reliable burst, then let the 3 s trajectory complete.
    # Replacing it on every spin restarts interpolation and only approaches the
    # target asymptotically at low real-time factors.
    for _ in range(5):
        node.publish_cleaning_pose()
        node.command(brushes=False, pump=False, speed=0.0)
        rclpy.spin_once(node, timeout_sec=0.08)
    wait_for_sim_condition(
        node,
        lambda: node.status is not None
        and bool(node.status["squeegee_ready"])
        and bool(node.status["nozzle_ready"]),
        label="cleaning geometry ground envelope",
        timeout_sim_s=timeout_sim_s,
        hard_wall_s=1_200.0,
        callback=lambda: node.command(brushes=False, pump=False, speed=0.0),
    )


def wait_for_applied_mass(node: Probe, target_kg: float, timeout_sim_s: float = 5.0) -> None:
    wait_for_sim_condition(
        node,
        lambda: node.applied_mass is not None
        and abs(node.applied_mass - target_kg) <= 1e-5,
        label="dynamic payload applied-mass acknowledgement",
        timeout_sim_s=timeout_sim_s,
        hard_wall_s=300.0,
    )


def reverse_to_water_start(node: Probe, timeout_sim_s: float = 12.0) -> dict[str, float]:
    """Pre-position the raised machine ahead of the immutable water footprint."""
    start_x = node.odom.pose.pose.position.x if node.odom is not None else math.nan

    def reverse_command() -> None:
        node.publish_cleaning_pose(lift_m=0.10)
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
    # three brush actuators, the pump rotor and the real diff-drive controller.
    lower_until_geometry_ready(node)
    node.status_samples.clear()
    start_odom = node.odom.pose.pose.position if node.odom is not None else None
    start_sim_time = node.sim_time_s
    def normal_pass_complete() -> bool:
        if node.status is None:
            return False
        recovered = float(node.status["recovered_volume_l"])
        nozzle_x = float(node.status["nozzle_world_x"])
        return recovered / initial_ground >= 0.955 and nozzle_x >= 1.87

    def normal_pass_command() -> None:
        node.publish_cleaning_pose()
        # 0.05 m/s through a 2 mm x 0.6 m layer requires 3.60 L/min,
        # remaining below the derated 10.57 L/min pump limit with enough
        # dwell time to drain each finite strip instead of skipping its edge.
        node.command(brushes=True, pump=True, speed=0.05)

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
        1.0,
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
        "raised_disabled_reverse_does_not_recover": abs(
            float(preposition_status["ground_volume_l"]) - preposition_ground
        ) <= 1e-6 and preposition["reverse_distance_m"] >= 0.15,
        "all_physical_proxy_conditions_seen": simultaneous_ready_seen,
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
    }
    return {
        "scenario": "normal_recovery",
        "checks": checks,
        "initial": initial,
        "disabled_system_terminal": disabled_status,
        "pump_only_terminal": pump_only_status,
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
        },
        "passed": all(checks.values()),
    }


def run_full(node: Probe) -> dict[str, object]:
    initial_tank = 9.65
    initial = reset_episode(node, 0.40, initial_tank)
    lower_until_geometry_ready(node)
    node.status_samples.clear()

    def full_tank_command() -> None:
        node.publish_cleaning_pose()
        node.command(brushes=True, pump=True, speed=0.065)

    wait_for_sim_condition(
        node,
        lambda: node.status is not None and bool(node.status["tank_full"]),
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
        callback=lambda: node.command(brushes=True, pump=True, speed=0.0),
    )
    terminal = dict(node.status or {})
    wait_for_applied_mass(node, float(terminal["tank_mass_kg"]), timeout_sim_s=10.0)
    node.stop()

    removed_l = float(initial["ground_volume_l"]) - ground_at_full
    tank_gain = tank_at_full - initial_tank
    error = abs(removed_l - tank_gain) / max(removed_l, 1e-12)
    post_full_ground_delta = ground_at_full - float(terminal["ground_volume_l"])
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
        and abs(node.applied_mass - tank_at_full) <= 1e-5,
        "full_case_visual_fraction_matches_ground_state": abs(
            float(terminal["visual_remaining_fraction"])
            - float(terminal["ground_volume_l"]) / float(initial["ground_volume_l"])
        ) <= 1e-6 and int(terminal["water_visual_count"]) == 24
        and bool(terminal["visual_layout_ready"]),
    }
    return {
        "scenario": "full_tank_fail_closed",
        "checks": checks,
        "initial": initial,
        "at_full": at_full,
        "terminal": terminal,
        "metrics": {
            "ground_removed_l": removed_l,
            "tank_mass_gain_kg": tank_gain,
            "dynamic_payload_applied_mass_kg": node.applied_mass,
            "mass_balance_error_fraction": error,
            "remaining_ground_volume_l": ground_at_full,
            "post_full_ground_delta_l": post_full_ground_delta,
        },
        "passed": all(checks.values()),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=("normal", "full"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    try:
        wait_ready(node)
        result = run_normal(node) if args.scenario == "normal" else run_full(node)
    except Exception as exc:  # preserve a machine-readable hard failure
        result = {
            "scenario": args.scenario,
            "passed": False,
            "checks": {},
            "error": f"{type(exc).__name__}: {exc}",
        }
    finally:
        node.stop()
        cycle_wall(node, 0.2)
        node.destroy_node()
        rclpy.shutdown()
    result["schema_version"] = 1
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
