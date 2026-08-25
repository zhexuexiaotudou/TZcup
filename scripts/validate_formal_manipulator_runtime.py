#!/usr/bin/env python3
"""Execute and measure the formal UR5e and Robotiq controllers in Gazebo."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"
MIMIC_RELATIONS = {
    "robotiq_85_right_knuckle_joint": -1.0,
    "robotiq_85_left_inner_knuckle_joint": 1.0,
    "robotiq_85_right_inner_knuckle_joint": -1.0,
    "robotiq_85_left_finger_tip_joint": -1.0,
    "robotiq_85_right_finger_tip_joint": 1.0,
}
ARM_WAYPOINTS = [
    ([0.00, 0.00, 0.00, 0.00, 0.00, 0.00], 2),
    ([0.12, -0.20, 0.30, -0.15, 0.10, -0.10], 5),
    ([-0.10, -0.25, 0.40, -0.20, -0.15, 0.15], 8),
    ([0.00, 0.00, 0.00, 0.00, 0.00, 0.00], 12),
]
GRIPPER_WAYPOINTS = [([0.00], 2), ([0.65], 5), ([0.00], 8), ([0.20], 10)]


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("formal_manipulator_runtime_probe")
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.gripper = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        self.samples: list[dict[str, float]] = []
        self.latest: dict[str, float] = {}
        self.subscription = self.create_subscription(JointState, "/joint_states", self._on_state, 20)

    def _on_state(self, message: JointState) -> None:
        current = {name: float(position) for name, position in zip(message.name, message.position)}
        self.latest.update(current)
        observed = {name: self.latest[name] for name in ARM_JOINTS + [GRIPPER_JOINT] if name in self.latest}
        if observed:
            observed["wall_time_s"] = time.time()
            self.samples.append(observed)

    def execute(self, client: ActionClient, joints: list[str], waypoints, timeout_s: float) -> dict:
        if not client.wait_for_server(timeout_sec=timeout_s):
            raise RuntimeError(f"action server unavailable: {client._action_name}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        for positions, seconds in waypoints:
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(sec=seconds)
            goal.trajectory.points.append(point)
        position_tolerance = 0.01 if joints == [GRIPPER_JOINT] else 0.02
        path_tolerance = 0.04 if joints == [GRIPPER_JOINT] else 0.08
        goal.path_tolerance = [JointTolerance(name=name, position=path_tolerance) for name in joints]
        goal.goal_tolerance = [JointTolerance(name=name, position=position_tolerance) for name in joints]
        goal.goal_time_tolerance = Duration(sec=2)
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_s)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"trajectory rejected: {client._action_name}")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_s)
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError(f"trajectory timed out: {client._action_name}")
        result = wrapped.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"trajectory failed on {client._action_name}: {result.error_code} {result.error_string}"
            )
        deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        target = waypoints[-1][0]
        missing = [name for name in joints if name not in self.latest]
        if missing:
            raise RuntimeError("joint states missing: " + ", ".join(missing))
        errors = {name: abs(self.latest[name] - expected) for name, expected in zip(joints, target)}
        if max(errors.values()) > position_tolerance:
            raise RuntimeError(f"terminal tracking error exceeds tolerance: {errors}")
        return {
            "action": client._action_name,
            "accepted": True,
            "result_error_code": int(result.error_code),
            "terminal_error_rad": errors,
            "terminal_max_error_rad": max(errors.values()),
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    try:
        state_deadline = time.monotonic() + args.timeout
        while rclpy.ok() and not all(name in node.latest for name in ARM_JOINTS + [GRIPPER_JOINT]):
            if time.monotonic() >= state_deadline:
                raise RuntimeError("commanded joint states did not become available")
            rclpy.spin_once(node, timeout_sec=0.1)
        arm_result = node.execute(node.arm, ARM_JOINTS, ARM_WAYPOINTS, args.timeout)
        gripper_result = node.execute(node.gripper, [GRIPPER_JOINT], GRIPPER_WAYPOINTS, args.timeout)
        ranges = {}
        for name in ARM_JOINTS + [GRIPPER_JOINT]:
            values = [sample[name] for sample in node.samples if name in sample]
            ranges[name] = max(values) - min(values)
        insufficient = [name for name in ARM_JOINTS if ranges[name] < 0.08]
        if insufficient or ranges[GRIPPER_JOINT] < 0.55:
            raise RuntimeError(f"insufficient measured motion; arm={insufficient}, ranges={ranges}")
        mimic_observed = sorted(set(MIMIC_RELATIONS) & set(node.latest))
        mimic_errors = {}
        for name in mimic_observed:
            mimic_errors[name] = abs(node.latest[name] - MIMIC_RELATIONS[name] * node.latest[GRIPPER_JOINT])
        if mimic_errors and max(mimic_errors.values()) > 0.02:
            raise RuntimeError(f"mimic state error exceeds tolerance: {mimic_errors}")
        report = {
            "report_id": "tzcup_formal_manipulator_runtime_v1",
            "status": "UR5E_AND_ROBOTIQ_GAZEBO_TRAJECTORY_EXECUTION_PASSED",
            "physics_engine": "gz-physics-bullet-featherstone-plugin",
            "arm": arm_result,
            "gripper": gripper_result,
            "measured_joint_range_rad": ranges,
            "joint_state_sample_count": len(node.samples),
            "mimic_relations_declared": MIMIC_RELATIONS,
            "mimic_joint_states_observed": mimic_observed,
            "mimic_terminal_error_rad": mimic_errors,
            "claim_boundary": (
                "This proves controller activation, six-axis rigid-body motion, master gripper motion and "
                "terminal tracking in Gazebo. It does not replace MoveIt self-collision planning, motor thermal/current "
                "models, cable-flex analysis or real-hardware validation."
            ),
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
