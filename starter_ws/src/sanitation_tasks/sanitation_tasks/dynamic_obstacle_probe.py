"""Run 20 deterministic obstacle interactions during a live coverage mission."""

import json
import math
import random
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .evaluation import yaw_from_quaternion
from .dynamic_geometry import crossing_targets, path_heading, remaining_path_length


class DynamicObstacleProbe(Node):
    def __init__(self):
        super().__init__("dynamic_obstacle_probe")
        self.declare_parameter("output_path", "/tmp/dynamic_obstacle_report.json")
        self.declare_parameter("set_pose_script", "/stage4t/scripts/gz_set_dynamic_obstacle.sh")
        self.declare_parameter("trial_count", 20)
        self.declare_parameter("hold_sec", 2.0)
        self.declare_parameter("world_name", "sanitation_structured_world")
        self.declare_parameter("model_name", "dynamic_pedestrian_box")
        self.declare_parameter("service_timeout_ms", 3000)
        self.declare_parameter("state_wait_timeout_sec", 120.0)
        self.declare_parameter("minimum_remaining_path_m", 3.0)
        self.declare_parameter("minimum_progress_between_trials_m", 0.5)
        self.declare_parameter("resume_observation_sec", 2.0)
        self.declare_parameter("crossing_half_width_m", 1.0)
        self.declare_parameter("crossing_steps", 9)
        self.declare_parameter("world_to_map_x", 8.0)
        service_name = (
            f"/world/{self.get_parameter('world_name').value}/set_pose"
        )
        self.set_pose_client = self.create_client(SetEntityPose, service_name)
        self.coverage_state = None
        self.component_state = {}
        self.current_path = []
        self.pose = None; self.vehicle_speed_m_s = None
        self.scan_minimum = math.inf; self.collision_count = 0; self.collision_active = False
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, 20)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 20)
        self.create_subscription(String, "/coverage/state", self.on_state, 20)
        self.create_subscription(
            String, "/coverage/component_state", self.on_component_state, 20
        )
        self.create_subscription(NavPath, "/coverage/current_path", self.on_path, 20)

    def on_state(self, message):
        self.coverage_state = message.data

    def on_component_state(self, message):
        try:
            self.component_state = json.loads(message.data)
        except json.JSONDecodeError:
            self.component_state = {'decode_error': True, 'raw': message.data}

    def on_path(self, message):
        self.current_path = [
            (pose.pose.position.x, pose.pose.position.y) for pose in message.poses
        ]

    def on_truth(self, message):
        pose = message.pose.pose
        self.pose = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
        twist = message.twist.twist
        self.vehicle_speed_m_s = math.hypot(twist.linear.x, twist.linear.y)

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
        timeout_sec = float(self.get_parameter("service_timeout_ms").value) / 1000.0
        if not self.set_pose_client.wait_for_service(timeout_sec=timeout_sec):
            return False, "", "set_pose ROS-Gazebo bridge unavailable"
        request = SetEntityPose.Request()
        request.entity.name = str(self.get_parameter("model_name").value)
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(world_x)
        request.pose.position.y = float(world_y)
        request.pose.position.z = 0.55
        request.pose.orientation.w = 1.0
        future = self.set_pose_client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            return False, "", "set_pose request timed out"
        response = future.result()
        success = bool(response and response.success)
        return success, f"ros_gz set_pose success={str(success).lower()}", ""

    def run(self):
        self.pump(5.0); randomizer = random.Random(20260715); trials = []
        preflight = []
        for name, target in (
            ("park", (25.0, 18.0)),
            ("test", (-6.0, 0.0)),
            ("park_return", (25.0, 18.0)),
        ):
            success, stdout, stderr = self.set_pose(*target)
            preflight.append({
                "step": name, "target_world_xy": target, "success": success,
                "stdout_tail": stdout, "stderr_tail": stderr,
            })
            if not success:
                return self.write_report(trials, preflight, "gazebo_set_pose_preflight_failed")
        last_component = None
        last_injection_position = None
        terminal_states = {"COMPLETED", "FAILED", "RECOVERY"}
        for seed in range(int(self.get_parameter("trial_count").value)):
            remaining = 0.0
            spacing_from_previous = None
            current_component = None
            wait_deadline = time.monotonic() + float(
                self.get_parameter("state_wait_timeout_sec").value
            )
            while rclpy.ok() and time.monotonic() < wait_deadline:
                remaining = remaining_path_length(
                    self.pose[:2], self.current_path
                ) if self.pose else 0.0
                current_component = (
                    self.component_state.get("kind"),
                    self.component_state.get("index"),
                )
                same_component = current_component == last_component
                if same_component and self.pose and last_injection_position:
                    spacing_from_previous = math.dist(
                        self.pose[:2], last_injection_position
                    )
                spacing_ready = (
                    not same_component
                    or spacing_from_previous is not None
                    and spacing_from_previous >= float(
                        self.get_parameter("minimum_progress_between_trials_m").value
                    )
                )
                if (
                    self.coverage_state == "EXECUTING_SWATH"
                    and remaining >= float(
                        self.get_parameter("minimum_remaining_path_m").value
                    )
                    and spacing_ready
                ):
                    break
                if self.coverage_state in terminal_states:
                    break
                self.pump(0.1)
            if self.coverage_state != "EXECUTING_SWATH" or self.pose is None:
                break
            position = self.pose[:2]
            current_component = (
                self.component_state.get("kind"),
                self.component_state.get("index"),
            )
            heading = path_heading(position, self.current_path)
            distance = randomizer.uniform(1.2, 1.8)
            targets = crossing_targets(
                position, heading, distance,
                float(self.get_parameter("crossing_half_width_m").value),
                int(self.get_parameter("crossing_steps").value),
            )
            move_results = []
            start_collisions = self.collision_count; self.scan_minimum = math.inf
            step_sec = float(self.get_parameter("hold_sec").value) / len(targets)
            interaction_start = time.monotonic()
            injection_vehicle_speed = self.vehicle_speed_m_s
            for map_x, map_y in targets:
                set_success, stdout, stderr = self.set_pose(
                    map_x - float(self.get_parameter("world_to_map_x").value),
                    map_y,
                )
                move_results.append({
                    "target_map_xy": [map_x, map_y],
                    "elapsed_sec": time.monotonic() - interaction_start,
                    "set_pose_success": set_success,
                    "stdout_tail": stdout,
                    "stderr_tail": stderr,
                })
                if not set_success:
                    break
                self.pump(step_sec)
            park_success, park_stdout, park_stderr = self.set_pose(25.0, 18.0)
            pose_before_resume = self.pose
            resume_start = time.monotonic(); recovery_time = None; progress = None
            resume_timeout = float(
                self.get_parameter("resume_observation_sec").value
            )
            while rclpy.ok() and time.monotonic() - resume_start < resume_timeout:
                self.pump(0.1)
                end_pose = self.pose
                progress = math.hypot(
                    end_pose[0] - pose_before_resume[0],
                    end_pose[1] - pose_before_resume[1],
                ) if pose_before_resume and end_pose else None
                if progress is not None and progress >= 0.10:
                    recovery_time = time.monotonic() - resume_start
                    break
            interacted = self.scan_minimum < 3.0
            collision_free = self.collision_count == start_collisions
            resumed = progress is not None and progress >= 0.10
            set_success = len(move_results) == len(targets) and all(
                item["set_pose_success"] for item in move_results
            )
            corridor_center = (
                position[0] + distance * math.cos(heading),
                position[1] + distance * math.sin(heading),
            )
            corridor_distance = min(
                (math.dist(target, corridor_center) for target in targets),
                default=math.inf,
            )
            trials.append({
                "seed": seed,
                "coverage_component": self.component_state,
                "component_key": list(current_component),
                "spacing_from_previous_interaction_m": spacing_from_previous,
                "minimum_progress_between_trials_m": float(
                    self.get_parameter("minimum_progress_between_trials_m").value
                ),
                "remaining_path_m_at_injection": remaining_path_length(
                    position, self.current_path
                ),
                "path_heading_rad": heading,
                "moving_obstacle_trajectory": move_results,
                "set_pose_success": set_success,
                "obstacle_in_path_corridor": bool(
                    corridor_distance <= (
                        2.0 * float(self.get_parameter("crossing_half_width_m").value)
                        / max(int(self.get_parameter("crossing_steps").value) - 1, 1)
                    )
                ),
                "path_corridor_center_distance_m": corridor_distance,
                "minimum_lidar_range_m": self.scan_minimum if math.isfinite(self.scan_minimum) else None,
                "interaction_observed": interacted, "collision_free": collision_free,
                "mission_progress_resumed": resumed, "vehicle_progress_m": progress,
                "injection_vehicle_speed_m_s": injection_vehicle_speed,
                "resumed_vehicle_speed_m_s": self.vehicle_speed_m_s,
                "recovery_time_sec": recovery_time,
                "resume_observation_limit_sec": resume_timeout,
                "response_classification": (
                    "interaction_collision_free_and_resumed"
                    if interacted and collision_free and resumed
                    else "invalid_or_incomplete_interaction"
                ),
                "park_success": park_success, "park_stdout_tail": park_stdout,
                "park_stderr_tail": park_stderr,
                "valid": bool(set_success and park_success and interacted and collision_free and resumed),
            })
            last_component = current_component
            last_injection_position = position
        self.set_pose(25.0, 18.0)
        return self.write_report(trials, preflight, None)

    def write_report(self, trials, preflight, failure_reason):
        valid = sum(trial["valid"] for trial in trials)
        report = {
            "schema_version": 1, "requested_trial_count": int(self.get_parameter("trial_count").value),
            "completed_trial_count": len(trials), "dynamic_obstacle_valid_trials": valid,
            "world_name": str(self.get_parameter("world_name").value),
            "model_name": str(self.get_parameter("model_name").value),
            "set_pose_preflight": preflight, "failure_reason": failure_reason,
            "collision_count": self.collision_count, "trials": trials,
            "success": failure_reason is None and len(trials) >= 20 and valid >= 20 and self.collision_count == 0,
        }
        output = Path(str(self.get_parameter("output_path").value)); output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return report["success"]


def main(args=None):
    rclpy.init(args=args); node = DynamicObstacleProbe()
    try: passed = node.run()
    finally: node.destroy_node(); rclpy.shutdown()
    if not passed: raise SystemExit(2)
