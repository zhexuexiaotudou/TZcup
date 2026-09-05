"""ROS-independent scoring for the formal vehicle mobility runtime gate."""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass
from typing import Any


WHEEL_JOINTS = (
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
)
WHEEL_RADIUS_M = 0.1625


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _projected_delta(start: Pose2D, end: Pose2D) -> tuple[float, float, float]:
    dx = end.x - start.x
    dy = end.y - start.y
    return (
        math.cos(start.yaw) * dx + math.sin(start.yaw) * dy,
        -math.sin(start.yaw) * dx + math.cos(start.yaw) * dy,
        math.atan2(math.sin(end.yaw - start.yaw), math.cos(end.yaw - start.yaw)),
    )


def evaluate_motion(raw: dict[str, Any]) -> dict[str, Any]:
    """Evaluate synchronized start/forward/stopped snapshots, failing closed."""
    ground_start = Pose2D(**raw["ground_truth"]["start"])
    ground_forward = Pose2D(**raw["ground_truth"]["forward_end"])
    ground_stopped = Pose2D(**raw["ground_truth"]["stopped_end"])
    odom_start = Pose2D(**raw["plant_odom"]["start"])
    odom_forward = Pose2D(**raw["plant_odom"]["forward_end"])
    odom_stopped = Pose2D(**raw["plant_odom"]["stopped_end"])

    gt_forward, gt_lateral, gt_yaw = _projected_delta(ground_start, ground_forward)
    gt_coast, gt_coast_lateral, gt_coast_yaw = _projected_delta(
        ground_forward, ground_stopped
    )
    odom_forward_m, odom_lateral, odom_yaw = _projected_delta(odom_start, odom_forward)
    odom_coast, _, odom_coast_yaw = _projected_delta(odom_forward, odom_stopped)
    coast_disagreement = abs(odom_coast - gt_coast)

    wheel_start = raw["wheel_state"]["start_positions_rad"]
    wheel_end = raw["wheel_state"]["forward_end_positions_rad"]
    wheel_travel = [abs(float(wheel_end[name]) - float(wheel_start[name])) * WHEEL_RADIUS_M for name in WHEEL_JOINTS]
    wheel_mean = statistics.fmean(wheel_travel)
    wheel_spread = max(wheel_travel) - min(wheel_travel)
    terminal_wheel_speed = max(abs(float(value)) for value in raw["wheel_state"]["stopped_velocities_rad_s"].values())
    terminal_odom_speed = math.hypot(
        float(raw["plant_odom"]["stopped_linear_velocity_mps"]["x"]),
        float(raw["plant_odom"]["stopped_linear_velocity_mps"]["y"]),
    )
    terminal_odom_angular_speed = abs(
        float(raw["plant_odom"]["stopped_angular_velocity_rad_s"])
    )

    checks = {
        "joint_state_broadcaster_active": raw["joint_state_broadcaster_state"] == "active",
        "safety_command_subscription": int(raw["command_subscription_count"]) == 1,
        "safety_actuator_enable_observed": int(raw["actuator_enabled_sample_count"]) > 0,
        "all_four_wheel_joints_observed": set(raw["wheel_state"]["observed_names"]) >= set(WHEEL_JOINTS),
        "ground_truth_forward_motion": 0.35 <= gt_forward <= 1.40,
        "plant_odometry_forward_motion": 0.35 <= odom_forward_m <= 1.40,
        "ground_truth_heading_straight": abs(gt_lateral) <= 0.12 and abs(gt_yaw) <= 0.12,
        "plant_odometry_heading_straight": abs(odom_lateral) <= 0.12 and abs(odom_yaw) <= 0.12,
        "odometry_matches_ground_truth": abs(odom_forward_m - gt_forward) <= 0.18,
        "wheel_rotation_matches_ground_truth": abs(wheel_mean - gt_forward) <= 0.22 and wheel_spread <= 0.12,
        "vehicle_stopped_after_zero_command": abs(gt_coast) <= 0.08 and abs(gt_coast_lateral) <= 0.05,
        "ground_truth_heading_stopped_after_zero_command": abs(gt_coast_yaw) <= 0.06,
        # Raw wheel-integrated odometry includes the A300 skid-steer tyre slip
        # seen during braking.  Keep the physical world-pose stop gate strict,
        # while accepting a bounded wheel-odom coast only when its terminal
        # speed is zero and it remains close to Gazebo ground truth.
        "plant_odometry_stopped_after_zero_command": abs(odom_coast) <= 0.15
        and coast_disagreement <= 0.10
        and terminal_odom_speed <= 0.03,
        "plant_odometry_heading_stopped_after_zero_command": abs(odom_coast_yaw)
        <= 0.12
        and terminal_odom_angular_speed <= 0.03,
        "wheel_joints_stopped_after_zero_command": terminal_wheel_speed <= 0.30,
    }
    metrics = {
        "ground_truth_forward_delta_m": gt_forward,
        "ground_truth_lateral_delta_m": gt_lateral,
        "ground_truth_yaw_delta_rad": gt_yaw,
        "plant_odom_forward_delta_m": odom_forward_m,
        "plant_odom_lateral_delta_m": odom_lateral,
        "plant_odom_yaw_delta_rad": odom_yaw,
        "forward_delta_disagreement_m": abs(odom_forward_m - gt_forward),
        "wheel_travel_m": dict(zip(WHEEL_JOINTS, wheel_travel, strict=True)),
        "wheel_mean_travel_m": wheel_mean,
        "wheel_travel_spread_m": wheel_spread,
        "ground_truth_coast_after_stop_m": gt_coast,
        "ground_truth_yaw_drift_after_stop_rad": gt_coast_yaw,
        "plant_odom_coast_after_stop_m": odom_coast,
        "plant_odom_yaw_drift_after_stop_rad": odom_coast_yaw,
        "stop_coast_disagreement_m": coast_disagreement,
        "terminal_plant_odom_speed_mps": terminal_odom_speed,
        "terminal_plant_odom_angular_speed_rad_s": terminal_odom_angular_speed,
        "terminal_max_wheel_speed_rad_s": terminal_wheel_speed,
    }
    return {"checks": checks, "metrics": metrics, "passed": all(checks.values())}


def evaluate_estop_stop(raw: dict[str, Any]) -> dict[str, Any]:
    """Score a stop asserted while the vehicle is physically in motion.

    The raw records intentionally contain both the final safety-manager command
    trace and independent Gazebo/plant/wheel endpoints.  A zero command by
    itself is never treated as proof that the simulated vehicle stopped.
    """

    phase = raw["estop"]
    ground_start = Pose2D(**phase["ground_truth"]["motion_start"])
    ground_trigger = Pose2D(**phase["ground_truth"]["trigger"])
    ground_end = Pose2D(**phase["ground_truth"]["stopped_end"])
    odom_trigger = Pose2D(**phase["plant_odom"]["trigger"])
    odom_end = Pose2D(**phase["plant_odom"]["stopped_end"])
    command_trace = phase["final_command_trace"]
    trigger_ns = int(phase["trigger_sim_time_ns"])
    motion_commands = [
        item for item in command_trace if int(item["sim_time_ns"]) < trigger_ns
    ]
    settled_commands = [
        item
        for item in command_trace
        if int(item["sim_time_ns"]) >= trigger_ns + 500_000_000
    ]
    if not settled_commands:
        raise ValueError("no final safety-manager samples after E-stop settling")
    if not motion_commands:
        raise ValueError("no final safety-manager samples before E-stop")

    travel_before_trigger, _, _ = _projected_delta(ground_start, ground_trigger)
    braking_distance, braking_lateral, braking_yaw = _projected_delta(
        ground_trigger, ground_end
    )
    odom_braking_distance, _, _ = _projected_delta(odom_trigger, odom_end)
    terminal_odom_speed = math.hypot(
        float(phase["plant_odom"]["stopped_linear_velocity_mps"]["x"]),
        float(phase["plant_odom"]["stopped_linear_velocity_mps"]["y"]),
    )
    terminal_wheel_speed = max(
        abs(float(value))
        for value in phase["wheel_state"]["stopped_velocities_rad_s"].values()
    )
    writer = phase["final_command_writer"]
    feedback = phase["emergency_stop_feedback_trace"]
    status_trace = phase["safety_status_trace"]
    feedback_or_status = any(
        item.get("active") is True and int(item["sim_time_ns"]) >= trigger_ns
        for item in feedback
    ) or any(
        int(item["sim_time_ns"]) >= trigger_ns
        and item.get("values", {}).get("manual_estop_active") == "true"
        for item in status_trace
    )
    checks = {
        "estop_asserted_during_physical_motion": travel_before_trigger >= 0.15,
        "final_safety_command_reached_one_mps_before_estop": max(
            abs(float(item["linear_x_mps"])) for item in motion_commands
        ) >= 0.98,
        "final_safety_command_has_one_expected_writer_and_input_subscriber": writer.get("writers") == [writer.get("expected_sole_writer")] and int(writer.get("input_subscription_count", 0)) == 1,
        "estop_feedback_or_manual_estop_status_observed": feedback_or_status,
        "final_safety_command_zero_after_estop": all(
            abs(float(item["linear_x_mps"])) <= 1e-4
            and abs(float(item["angular_z_rad_s"])) <= 1e-4
            for item in settled_commands
        ),
        # This is a Gazebo brake-distance regression bound, not a claim about
        # hardware-certified braking performance.
        "gazebo_estop_braking_distance_bounded": 0.0 <= braking_distance <= 0.25
        and abs(braking_lateral) <= 0.10
        and abs(braking_yaw) <= 0.10,
        "plant_odom_estop_braking_matches_ground_truth": abs(
            odom_braking_distance - braking_distance
        ) <= 0.12,
        "physical_vehicle_stopped_after_estop": terminal_odom_speed <= 0.03
        and terminal_wheel_speed <= 0.30,
    }
    metrics = {
        "estop_motion_before_trigger_m": travel_before_trigger,
        "gazebo_estop_braking_distance_m": braking_distance,
        "gazebo_estop_braking_lateral_m": braking_lateral,
        "gazebo_estop_braking_yaw_rad": braking_yaw,
        "plant_odom_estop_braking_distance_m": odom_braking_distance,
        "terminal_plant_odom_speed_after_estop_mps": terminal_odom_speed,
        "terminal_max_wheel_speed_after_estop_rad_s": terminal_wheel_speed,
        "final_zero_command_sample_count": len(settled_commands),
        "pre_estop_final_command_sample_count": len(motion_commands),
        "pre_estop_max_final_command_mps": max(
            abs(float(item["linear_x_mps"])) for item in motion_commands
        ),
        "estop_feedback_sample_count": len(feedback),
        "safety_status_sample_count": len(status_trace),
    }
    return {"checks": checks, "metrics": metrics, "passed": all(checks.values())}
