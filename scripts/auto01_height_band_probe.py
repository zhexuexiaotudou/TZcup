#!/usr/bin/env python3
"""Run repeated Gazebo low/tall obstacle tests against the AUTO-01 safety chain."""

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


WORLD = "sanitation_structured_world"
WORLD_TO_MAP_X = 8.0
ROBOT_HALF_LENGTH_M = 0.40
OBSTACLE_HALF_LENGTH_M = 0.15


def set_pose(name: str, x: float, y: float, z: float) -> None:
    request = (
        f"name: '{name}', position: {{x: {x}, y: {y}, z: {z}}}, "
        "orientation: {w: 1.0}"
    )
    result = subprocess.run(
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
            "5000",
            "--req",
            request,
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )
    if result.returncode != 0 or "data: true" not in result.stdout.lower():
        raise RuntimeError(
            f"SetEntityPose failed for {name}: rc={result.returncode} "
            f"stdout={result.stdout!r} stderr={result.stderr!r}"
        )


class Probe:
    def __init__(self) -> None:
        self.node = rclpy.create_node("auto01_height_band_probe")
        self.command = self.node.create_publisher(Twist, "/cmd_vel_smoothed", 20)
        self.ground_states: list[int] = []
        self.high_states: list[int] = []
        self.robot_x_samples: list[float] = []
        self.node.create_subscription(
            CollisionMonitorState,
            "/ground_collision_monitor_state",
            lambda message: self.ground_states.append(int(message.action_type)),
            20,
        )
        self.node.create_subscription(
            CollisionMonitorState,
            "/collision_monitor_state",
            lambda message: self.high_states.append(int(message.action_type)),
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

    def spin_for(self, seconds: float, speed: float = 0.0) -> None:
        end = time.monotonic() + seconds
        message = Twist()
        message.linear.x = speed
        while time.monotonic() < end:
            self.command.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.04)

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
        self.ground_states.clear()
        self.high_states.clear()
        self.robot_x_samples.clear()
        self.spin_for(1.0, 0.25)
        self.spin_for(0.4, 0.0)
        max_robot_x = max(self.robot_x_samples, default=0.0)
        clearance = (
            obstacle_x_map
            - OBSTACLE_HALF_LENGTH_M
            - max_robot_x
            - ROBOT_HALF_LENGTH_M
        )
        ground_triggered = any(
            state
            in (
                CollisionMonitorState.STOP,
                CollisionMonitorState.APPROACH,
            )
            for state in self.ground_states
        )
        high_triggered = any(
            state == CollisionMonitorState.STOP for state in self.high_states
        )
        return {
            "kind": kind,
            "trial": index,
            "obstacle_pose_map": [obstacle_x_map, obstacle_y],
            "ground_monitor_triggered": ground_triggered,
            "high_monitor_triggered": high_triggered,
            "max_robot_x_m": max_robot_x,
            "minimum_axis_clearance_m": clearance,
            "collision": clearance <= 0.0,
            "pass": (
                ground_triggered and not high_triggered and clearance > 0.0
                if kind == "low"
                else high_triggered and clearance > 0.0
            ),
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
                    0.86 + 0.015 * math.sin(phase),
                    0.06 * math.cos(phase),
                )
            )
        for index in range(args.trials):
            phase = 2.0 * math.pi * index / args.trials
            results.append(
                probe.run_trial(
                    "tall",
                    index,
                    0.89 + 0.015 * math.sin(phase),
                    0.32 + 0.025 * math.cos(phase),
                )
            )
        probe.spin_for(0.5, 0.0)
    finally:
        probe.node.destroy_node()
        rclpy.shutdown()
    low = [item for item in results if item["kind"] == "low"]
    tall = [item for item in results if item["kind"] == "tall"]
    collision_count = sum(bool(item["collision"]) for item in results)
    false_safe_count = sum(not item["high_monitor_triggered"] for item in tall)
    report = {
        "schema_version": 1,
        "stage": "AUTO-01",
        "attempt_id": "AUTO-01-G1-C3-HEIGHT-R1",
        "trial_count_per_class": args.trials,
        "low_obstacle": {
            "pass_count": sum(bool(item["pass"]) for item in low),
            "collision_count": sum(bool(item["collision"]) for item in low),
            "high_envelope_false_trigger_count": sum(
                bool(item["high_monitor_triggered"]) for item in low
            ),
        },
        "tall_obstacle": {
            "pass_count": sum(bool(item["pass"]) for item in tall),
            "collision_count": sum(bool(item["collision"]) for item in tall),
            "camera_envelope_trigger_count": sum(
                bool(item["high_monitor_triggered"]) for item in tall
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
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["gate_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
