#!/usr/bin/env python3
"""Drive the formal vehicle forward, stop it, and score physical motion evidence."""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from formal_vehicle_mobility_metrics import WHEEL_JOINTS, evaluate_motion, quaternion_yaw

MODEL_NAME = "tzcup_formal_sanitation_vehicle"


def _protobuf_scalar(block: str, field: str, default: float = 0.0) -> float:
    match = re.search(rf"^\s*{re.escape(field)}:\s*([-+0-9.eE]+)\s*$", block, re.MULTILINE)
    return float(match.group(1)) if match else default


def _named_pose_block(message: str, name: str) -> str:
    marker = f'name: "{name}"'
    marker_index = message.find(marker)
    if marker_index < 0:
        raise RuntimeError(f"Gazebo pose/info did not contain model {name}")
    start = message.rfind("pose {", 0, marker_index)
    if start < 0:
        raise RuntimeError(f"Gazebo pose/info block for {name} is malformed")
    depth = 0
    for index in range(start + len("pose "), len(message)):
        if message[index] == "{":
            depth += 1
        elif message[index] == "}":
            depth -= 1
            if depth == 0:
                return message[start : index + 1]
    raise RuntimeError(f"Gazebo pose/info block for {name} is unterminated")


def read_gazebo_ground_truth() -> dict[str, float]:
    """Read one named model pose directly from Gazebo Transport, preserving names."""
    executable = shutil.which("gz")
    if executable is None:
        vendor = Path("/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz")
        if vendor.exists():
            executable = str(vendor)
    if executable is None:
        raise RuntimeError("Gazebo CLI not found")
    result = subprocess.run(
        [executable, "topic", "-e", "-t", "/world/formal_vehicle_validation/pose/info", "-n", "1"],
        check=True,
        capture_output=True,
        text=True,
        timeout=10.0,
    )
    block = _named_pose_block(result.stdout, MODEL_NAME)
    position_match = re.search(r"position\s*\{(?P<body>.*?)\}", block, re.DOTALL)
    orientation_match = re.search(r"orientation\s*\{(?P<body>.*?)\}", block, re.DOTALL)
    if position_match is None or orientation_match is None:
        raise RuntimeError("Gazebo model pose has no position or orientation")
    position = position_match.group("body")
    orientation = orientation_match.group("body")
    return {
        "x": _protobuf_scalar(position, "x"),
        "y": _protobuf_scalar(position, "y"),
        "yaw": quaternion_yaw(
            _protobuf_scalar(orientation, "x"),
            _protobuf_scalar(orientation, "y"),
            _protobuf_scalar(orientation, "z"),
            _protobuf_scalar(orientation, "w", 1.0),
        ),
    }


class MobilityProbe(Node):
    def __init__(self) -> None:
        super().__init__(
            "formal_vehicle_mobility_runtime_probe",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
            automatically_declare_parameters_from_overrides=True,
        )
        self.command = self.create_publisher(TwistStamped, "/base_controller/cmd_vel", 10)
        self.create_subscription(Odometry, "/base_controller/odom", self._odom, 50)
        self.create_subscription(JointState, "/joint_states", self._joints, 50)
        self.controller_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.latest_odom: dict[str, Any] | None = None
        self.latest_wheels: dict[str, dict[str, float]] | None = None
        self.odom_samples = 0
        self.joint_samples = 0

    def _odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        self.latest_odom = {
            "pose": {
                "x": pose.position.x,
                "y": pose.position.y,
                "yaw": quaternion_yaw(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            },
            "linear_velocity_mps": {"x": twist.linear.x, "y": twist.linear.y},
            "angular_velocity_rad_s": twist.angular.z,
        }
        self.odom_samples += 1

    def _joints(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        if all(name in positions and name in velocities for name in WHEEL_JOINTS):
            self.latest_wheels = {
                "positions": {name: float(positions[name]) for name in WHEEL_JOINTS},
                "velocities": {name: float(velocities[name]) for name in WHEEL_JOINTS},
            }
            self.joint_samples += 1

    def publish_velocity(self, speed: float) -> None:
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.twist.linear.x = speed
        self.command.publish(command)

    def spin_for(self, duration: float, speed: float, rate_hz: float = 20.0) -> dict[str, Any]:
        wall_start = time.monotonic()
        wall_deadline = wall_start + max(30.0, duration * 15.0)
        sim_start = self.get_clock().now().nanoseconds
        period = 1.0 / rate_hz
        simulated = 0.0
        while simulated < duration:
            if time.monotonic() >= wall_deadline:
                raise TimeoutError(f"simulation clock did not advance {duration} s during mobility command")
            self.publish_velocity(speed)
            rclpy.spin_once(self, timeout_sec=period)
            simulated = (self.get_clock().now().nanoseconds - sim_start) / 1e9
        return {
            "requested_simulated_s": duration,
            "measured_simulated_s": simulated,
            "measured_wall_s": time.monotonic() - wall_start,
            "use_sim_time": bool(self.get_parameter("use_sim_time").value),
        }

    def snapshot(self) -> tuple[dict[str, Any], dict[str, Any], dict[str, dict[str, float]]]:
        if self.latest_odom is None or self.latest_wheels is None:
            raise RuntimeError("mobility evidence streams are incomplete")
        return (
            read_gazebo_ground_truth(),
            json.loads(json.dumps(self.latest_odom)),
            json.loads(json.dumps(self.latest_wheels)),
        )


def _wait_for_ready(node: MobilityProbe, timeout: float) -> str:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        ready = (
            node.latest_odom is not None
            and node.latest_wheels is not None
            and node.command.get_subscription_count() >= 1
            and node.controller_client.wait_for_service(timeout_sec=0.0)
        )
        if ready:
            future = node.controller_client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            if future.done() and future.result() is not None:
                states = {item.name: item.state for item in future.result().controller}
                if states.get("base_controller") == "active":
                    return "active"
        time.sleep(0.05)
    raise TimeoutError(
        "timed out waiting for active base controller, ground truth pose, controller odom, "
        "all four wheel joints and command subscription"
    )


def run(output: Path, timeout: float, forward_speed: float, forward_duration: float) -> dict[str, Any]:
    rclpy.init()
    node = MobilityProbe()
    try:
        controller_state = _wait_for_ready(node, timeout)
        settle_timing = node.spin_for(1.0, 0.0)
        ground_start, odom_start, wheels_start = node.snapshot()
        forward_timing = node.spin_for(forward_duration, forward_speed)
        ground_forward, odom_forward, wheels_forward = node.snapshot()
        stopped_timing = node.spin_for(3.0, 0.0)
        ground_stopped, odom_stopped, wheels_stopped = node.snapshot()
        raw = {
            "controller_state": controller_state,
            "command_subscription_count": node.command.get_subscription_count(),
            "ground_truth": {
                "start": ground_start,
                "forward_end": ground_forward,
                "stopped_end": ground_stopped,
            },
            "controller_odom": {
                "start": odom_start["pose"],
                "forward_end": odom_forward["pose"],
                "stopped_end": odom_stopped["pose"],
                "stopped_linear_velocity_mps": odom_stopped["linear_velocity_mps"],
                "stopped_angular_velocity_rad_s": odom_stopped["angular_velocity_rad_s"],
            },
            "wheel_state": {
                "observed_names": list(WHEEL_JOINTS),
                "start_positions_rad": wheels_start["positions"],
                "forward_end_positions_rad": wheels_forward["positions"],
                "stopped_velocities_rad_s": wheels_stopped["velocities"],
            },
        }
        evaluation = evaluate_motion(raw)
        report = {
            "report_id": "tzcup_formal_vehicle_mobility_runtime_v1",
            "status": "FORMAL_VEHICLE_FORWARD_STOP_RUNTIME_PASSED" if evaluation["passed"] else "FORMAL_VEHICLE_FORWARD_STOP_RUNTIME_FAILED",
            "command": {
                "topic": "/base_controller/cmd_vel",
                "forward_speed_mps": forward_speed,
                "forward_duration_s": forward_duration,
                "zero_command_duration_s": 3.0,
                "timing_source": "/clock via rclpy use_sim_time",
                "settle_timing": settle_timing,
                "forward_timing": forward_timing,
                "zero_timing": stopped_timing,
            },
            "sample_counts": {
                "gazebo_ground_truth_pose": 3,
                "controller_odometry": node.odom_samples,
                "joint_states": node.joint_samples,
            },
            "raw_evidence": raw,
            **evaluation,
            "claim_boundary": "This proves commanded straight-ahead physical motion and stopping in Gazebo using independent world pose, diff-drive odometry, and all four wheel joints; it does not prove path tracking, obstacle avoidance, or real-vehicle braking distance.",
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(report, sort_keys=True))
        if not evaluation["passed"]:
            raise SystemExit(1)
        return report
    finally:
        for _ in range(10):
            node.publish_velocity(0.0)
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--forward-speed", type=float, default=0.25)
    parser.add_argument("--forward-duration", type=float, default=4.0)
    args = parser.parse_args()
    run(args.output, args.timeout, args.forward_speed, args.forward_duration)


if __name__ == "__main__":
    main()
