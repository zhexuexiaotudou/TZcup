"""Run 20 deterministic obstacle interactions during a live coverage mission."""

import json
import math
import random
import subprocess
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .evaluation import yaw_from_quaternion


class DynamicObstacleProbe(Node):
    def __init__(self):
        super().__init__("dynamic_obstacle_probe")
        self.declare_parameter("output_path", "/tmp/dynamic_obstacle_report.json")
        self.declare_parameter("set_pose_script", "/stage4t/scripts/gz_set_dynamic_obstacle.sh")
        self.declare_parameter("trial_count", 20)
        self.declare_parameter("hold_sec", 4.0)
        self.pose = None; self.scan_minimum = math.inf; self.collision_count = 0; self.collision_active = False
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, 20)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 20)

    def on_truth(self, message):
        pose = message.pose.pose
        self.pose = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))

    def on_scan(self, message):
        finite = [value for value in message.ranges if math.isfinite(value)]
        if not finite: return
        minimum = min(finite); self.scan_minimum = min(self.scan_minimum, minimum)
        active = minimum < 0.12
        if active and not self.collision_active: self.collision_count += 1
        self.collision_active = active

    def pump(self, duration):
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def set_pose(self, world_x, world_y):
        result = subprocess.run(
            ["bash", str(self.get_parameter("set_pose_script").value), f"{world_x:.6f}", f"{world_y:.6f}"],
            capture_output=True, text=True, check=False,
        )
        return result.returncode == 0 and "true" in result.stdout.lower(), result.stdout[-500:], result.stderr[-500:]

    def run(self):
        self.pump(5.0); randomizer = random.Random(20260715); trials = []
        for seed in range(int(self.get_parameter("trial_count").value)):
            if self.pose is None: break
            x, y, yaw = self.pose; lateral = randomizer.uniform(-0.45, 0.45); distance = randomizer.uniform(1.2, 1.8)
            map_x = x + distance * math.cos(yaw) - lateral * math.sin(yaw)
            map_y = y + distance * math.sin(yaw) + lateral * math.cos(yaw)
            # Ground-truth calibration is world -> map translation (+8, 0).
            set_success, stdout, stderr = self.set_pose(map_x - 8.0, map_y)
            start_pose = self.pose; start_collisions = self.collision_count; self.scan_minimum = math.inf
            self.pump(float(self.get_parameter("hold_sec").value))
            end_pose = self.pose
            progress = math.hypot(end_pose[0] - start_pose[0], end_pose[1] - start_pose[1]) if start_pose and end_pose else None
            interacted = self.scan_minimum < 3.0
            collision_free = self.collision_count == start_collisions
            trials.append({
                "seed": seed, "target_map_xy": [map_x, map_y], "set_pose_success": set_success,
                "minimum_lidar_range_m": self.scan_minimum if math.isfinite(self.scan_minimum) else None,
                "interaction_observed": interacted, "collision_free": collision_free,
                "vehicle_progress_m": progress, "stdout_tail": stdout, "stderr_tail": stderr,
                "valid": bool(set_success and interacted and collision_free),
            })
        self.set_pose(25.0, 18.0)
        valid = sum(trial["valid"] for trial in trials)
        report = {
            "schema_version": 1, "requested_trial_count": int(self.get_parameter("trial_count").value),
            "completed_trial_count": len(trials), "dynamic_obstacle_valid_trials": valid,
            "collision_count": self.collision_count, "trials": trials,
            "success": len(trials) >= 20 and valid >= 20 and self.collision_count == 0,
        }
        output = Path(str(self.get_parameter("output_path").value)); output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return report["success"]


def main(args=None):
    rclpy.init(args=args); node = DynamicObstacleProbe()
    try: passed = node.run()
    finally: node.destroy_node(); rclpy.shutdown()
    if not passed: raise SystemExit(2)
