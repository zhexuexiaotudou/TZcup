#!/usr/bin/env python3
"""Run repeated low/tall Gazebo obstacle trials for AUTO-01 G2."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


WORLD = "sanitation_structured_world"
WORLD_TO_MAP_X = 8.0
ROBOT_HALF_LENGTH_M = 0.40
OBSTACLE_HALF_LENGTH_M = 0.15


def set_pose(name: str, x: float, y: float, z: float) -> None:
    request = (
        f"name: '{name}', position: {{x: {x}, y: {y}, z: {z}}}, "
        "orientation: {w: 1.0}"
    )
    last_result = None
    for attempt in range(1, 4):
        last_result = subprocess.run(
            [
                "gz",
                "service",
                "-s",
                f"/world/{WORLD}/set_pose",
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "8000",
                "--req",
                request,
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=12,
        )
        if (
            last_result.returncode == 0
            and "data: true" in last_result.stdout.lower()
        ):
            return
        if attempt < 3:
            time.sleep(0.25 * attempt)
    raise RuntimeError(
        f"SetEntityPose failed for {name} after 3 attempts: "
        f"rc={last_result.returncode} stdout={last_result.stdout!r} "
        f"stderr={last_result.stderr!r}"
    )


class Probe:
    def __init__(self) -> None:
        self.node = rclpy.create_node("auto01_g2_obstacle_probe")
        self.command = self.node.create_publisher(
            Twist, "/cmd_vel_nav", 20
        )
        self.monitor_states: list[int] = []
        self.robot_x_samples: list[float] = []
        self.forward_scan_minima: list[float] = []
        self.smoothed_commands: list[float] = []
        self.gated_commands: list[float] = []
        self.output_commands: list[float] = []
        self.sim_time_sec: float | None = None
        self.node.create_subscription(
            CollisionMonitorState,
            "/collision_monitor_state",
            lambda message: self.monitor_states.append(
                int(message.action_type)
            ),
            20,
        )
        self.node.create_subscription(
            Odometry,
            "/ground_truth/odom",
            lambda message: self.robot_x_samples.append(
                float(message.pose.pose.position.x)
            ),
            50,
        )
        self.node.create_subscription(LaserScan, "/scan", self._on_scan, 20)
        self.node.create_subscription(
            Clock,
            "/clock",
            lambda message: setattr(
                self,
                "sim_time_sec",
                float(message.clock.sec)
                + float(message.clock.nanosec) * 1.0e-9,
            ),
            20,
        )
        self.node.create_subscription(
            Twist,
            "/cmd_vel_smoothed",
            lambda message: self.smoothed_commands.append(
                float(message.linear.x)
            ),
            50,
        )
        self.node.create_subscription(
            Twist,
            "/cmd_vel_gate",
            lambda message: self.gated_commands.append(
                float(message.linear.x)
            ),
            50,
        )
        self.node.create_subscription(
            Twist,
            "/cmd_vel",
            lambda message: self.output_commands.append(
                float(message.linear.x)
            ),
            50,
        )

    def _on_scan(self, message: LaserScan) -> None:
        candidates = []
        for index, value in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if abs(angle) <= math.radians(12.0) and math.isfinite(value):
                candidates.append(float(value))
        if candidates:
            self.forward_scan_minima.append(min(candidates))

    def spin_for(self, seconds: float, speed: float = 0.0) -> None:
        message = Twist()
        message.linear.x = speed
        clock_deadline = time.monotonic() + 30.0
        while self.sim_time_sec is None and time.monotonic() < clock_deadline:
            self.command.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.04)
        if self.sim_time_sec is None:
            raise RuntimeError("timed out waiting for Gazebo /clock")
        target_sim_time = self.sim_time_sec + seconds
        wall_deadline = time.monotonic() + max(30.0, seconds * 30.0)
        while (
            self.sim_time_sec < target_sim_time
            and time.monotonic() < wall_deadline
        ):
            self.command.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.04)
        if self.sim_time_sec < target_sim_time:
            raise RuntimeError(
                f"Gazebo clock advanced only to {self.sim_time_sec:.3f}; "
                f"target was {target_sim_time:.3f}"
            )

    def run_trial(
        self, kind: str, index: int, obstacle_x_map: float, obstacle_y: float
    ) -> dict:
        set_pose("auto01_low_obstacle", 25.0, 17.0, 0.175)
        set_pose("auto01_tall_obstacle", 25.0, 16.0, 0.55)
        set_pose("sanitation_vehicle", -8.0, 0.0, 0.18)
        self.spin_for(0.6, 0.0)
        if kind == "low":
            set_pose(
                "auto01_low_obstacle",
                obstacle_x_map - WORLD_TO_MAP_X,
                obstacle_y,
                0.175,
            )
        else:
            set_pose(
                "auto01_tall_obstacle",
                obstacle_x_map - WORLD_TO_MAP_X,
                obstacle_y,
                0.55,
            )
        self.spin_for(0.8, 0.0)
        self.monitor_states.clear()
        self.robot_x_samples.clear()
        self.forward_scan_minima.clear()
        self.smoothed_commands.clear()
        self.gated_commands.clear()
        self.output_commands.clear()
        self.spin_for(2.0, 0.25)
        self.spin_for(0.4, 0.0)
        max_robot_x = max(self.robot_x_samples, default=0.0)
        clearance = (
            obstacle_x_map
            - OBSTACLE_HALF_LENGTH_M
            - max_robot_x
            - ROBOT_HALF_LENGTH_M
        )
        triggered = any(
            state
            in (
                CollisionMonitorState.STOP,
                CollisionMonitorState.APPROACH,
            )
            for state in self.monitor_states
        )
        forward_lidar_minimum = min(
            self.forward_scan_minima, default=math.inf
        )
        collision = clearance <= 0.0
        return {
            "kind": kind,
            "trial": index,
            "obstacle_pose_map": [obstacle_x_map, obstacle_y],
            "collision_monitor_triggered": triggered,
            "forward_lidar_minimum_m": (
                forward_lidar_minimum
                if math.isfinite(forward_lidar_minimum)
                else None
            ),
            "max_robot_x_m": max_robot_x,
            "maximum_smoothed_command_mps": max(
                self.smoothed_commands, default=0.0
            ),
            "maximum_gated_command_mps": max(
                self.gated_commands, default=0.0
            ),
            "maximum_output_command_mps": max(
                self.output_commands, default=0.0
            ),
            "minimum_axis_clearance_m": clearance,
            "collision": collision,
            "pass": triggered and not collision,
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.trials < 1:
        raise ValueError("--trials must be positive")
    rclpy.init()
    probe = Probe()
    try:
        probe.spin_for(2.0, 0.0)
        results = []
        for index in range(args.trials):
            phase = 2.0 * math.pi * index / args.trials
            results.append(
                probe.run_trial(
                    "low",
                    index,
                    0.84 + 0.008 * math.sin(phase),
                    0.03 * math.cos(phase),
                )
            )
        for index in range(args.trials):
            phase = 2.0 * math.pi * index / args.trials
            results.append(
                probe.run_trial(
                    "tall",
                    index,
                    0.84 + 0.008 * math.sin(phase),
                    0.03 * math.cos(phase),
                )
            )
        probe.spin_for(0.5, 0.0)
    finally:
        probe.node.destroy_node()
        rclpy.shutdown()
    low = [item for item in results if item["kind"] == "low"]
    tall = [item for item in results if item["kind"] == "tall"]
    collision_count = sum(bool(item["collision"]) for item in results)
    false_safe_count = sum(
        not item["collision_monitor_triggered"] for item in tall
    )
    report = {
        "schema_version": 1,
        "stage": "AUTO-01",
        "attempt_id": "AUTO-01-G2-C3-OBSTACLE-R1",
        "architecture": "G2",
        "profile": "auto01_g2_v5_retracted",
        "trial_count_per_class": args.trials,
        "low_obstacle": {
            "pass_count": sum(bool(item["pass"]) for item in low),
            "collision_count": sum(bool(item["collision"]) for item in low),
            "monitor_trigger_count": sum(
                bool(item["collision_monitor_triggered"]) for item in low
            ),
        },
        "tall_obstacle": {
            "pass_count": sum(bool(item["pass"]) for item in tall),
            "collision_count": sum(bool(item["collision"]) for item in tall),
            "camera_envelope_protection_trigger_count": sum(
                bool(item["collision_monitor_triggered"]) for item in tall
            ),
        },
        "height_classification_false_safe_count": false_safe_count,
        "height_classification_false_safe_rate": false_safe_count / len(tall),
        "collision_count": collision_count,
        "gate_pass": (
            all(bool(item["pass"]) for item in results)
            and collision_count == 0
            and false_safe_count == 0
        ),
        "trials": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    return 0 if report["gate_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
