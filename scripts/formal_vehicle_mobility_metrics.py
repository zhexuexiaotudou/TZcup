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
    odom_start = Pose2D(**raw["controller_odom"]["start"])
    odom_forward = Pose2D(**raw["controller_odom"]["forward_end"])
    odom_stopped = Pose2D(**raw["controller_odom"]["stopped_end"])

    gt_forward, gt_lateral, gt_yaw = _projected_delta(ground_start, ground_forward)
    gt_coast, gt_coast_lateral, _ = _projected_delta(ground_forward, ground_stopped)
    odom_forward_m, odom_lateral, odom_yaw = _projected_delta(odom_start, odom_forward)
    odom_coast, _, _ = _projected_delta(odom_forward, odom_stopped)

    wheel_start = raw["wheel_state"]["start_positions_rad"]
    wheel_end = raw["wheel_state"]["forward_end_positions_rad"]
    wheel_travel = [abs(float(wheel_end[name]) - float(wheel_start[name])) * WHEEL_RADIUS_M for name in WHEEL_JOINTS]
    wheel_mean = statistics.fmean(wheel_travel)
    wheel_spread = max(wheel_travel) - min(wheel_travel)
    terminal_wheel_speed = max(abs(float(value)) for value in raw["wheel_state"]["stopped_velocities_rad_s"].values())
    terminal_odom_speed = math.hypot(
        float(raw["controller_odom"]["stopped_linear_velocity_mps"]["x"]),
        float(raw["controller_odom"]["stopped_linear_velocity_mps"]["y"]),
    )

    checks = {
        "base_controller_active": raw["controller_state"] == "active",
        "base_controller_command_subscription": int(raw["command_subscription_count"]) >= 1,
        "all_four_wheel_joints_observed": set(raw["wheel_state"]["observed_names"]) >= set(WHEEL_JOINTS),
        "ground_truth_forward_motion": 0.35 <= gt_forward <= 1.40,
        "controller_odometry_forward_motion": 0.35 <= odom_forward_m <= 1.40,
        "ground_truth_heading_straight": abs(gt_lateral) <= 0.12 and abs(gt_yaw) <= 0.12,
        "controller_odometry_heading_straight": abs(odom_lateral) <= 0.12 and abs(odom_yaw) <= 0.12,
        "odometry_matches_ground_truth": abs(odom_forward_m - gt_forward) <= 0.18,
        "wheel_rotation_matches_ground_truth": abs(wheel_mean - gt_forward) <= 0.22 and wheel_spread <= 0.12,
        "vehicle_stopped_after_zero_command": abs(gt_coast) <= 0.08 and abs(gt_coast_lateral) <= 0.05,
        "odometry_stopped_after_zero_command": abs(odom_coast) <= 0.08 and terminal_odom_speed <= 0.03,
        "wheel_joints_stopped_after_zero_command": terminal_wheel_speed <= 0.30,
    }
    metrics = {
        "ground_truth_forward_delta_m": gt_forward,
        "ground_truth_lateral_delta_m": gt_lateral,
        "ground_truth_yaw_delta_rad": gt_yaw,
        "controller_odom_forward_delta_m": odom_forward_m,
        "controller_odom_lateral_delta_m": odom_lateral,
        "controller_odom_yaw_delta_rad": odom_yaw,
        "forward_delta_disagreement_m": abs(odom_forward_m - gt_forward),
        "wheel_travel_m": dict(zip(WHEEL_JOINTS, wheel_travel, strict=True)),
        "wheel_mean_travel_m": wheel_mean,
        "wheel_travel_spread_m": wheel_spread,
        "ground_truth_coast_after_stop_m": gt_coast,
        "controller_odom_coast_after_stop_m": odom_coast,
        "terminal_controller_odom_speed_mps": terminal_odom_speed,
        "terminal_max_wheel_speed_rad_s": terminal_wheel_speed,
    }
    return {"checks": checks, "metrics": metrics, "passed": all(checks.values())}
