#!/usr/bin/env python3
"""Drive and measure every non-base formal cleaning/storage actuator in Gazebo."""

from __future__ import annotations

import argparse
import json
import time
from collections import defaultdict
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


POSITION_TARGETS = {
    "cleaning_lift_joint": 0.025,
    "squeegee_pitch_joint": 0.12,
    "squeegee_float_joint": 0.010,
    "dry_bin_lid_joint": 0.0,
    "dry_deposit_gate_joint": 1.05,
    "wastewater_lid_joint": 0.30,
}
VELOCITY_TARGETS = {
    "left_side_brush_joint": 8.0,
    "right_side_brush_joint": -8.0,
    "central_roller_joint": 12.0,
    "recovery_pump_joint": 20.0,
}
CONTACT_TOPICS = (
    "/cleaning/suction_nozzle/contact",
    "/storage/dry_deposit/contact",
    "/safety/front_bumper/contact",
    "/safety/rear_bumper/contact",
)


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("formal_function_positions_runtime_probe")
        self.positions: dict[str, list[float]] = defaultdict(list)
        self.velocities: dict[str, list[float]] = defaultdict(list)
        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self._joint_state, 20)
        self.cleaning = self.create_publisher(JointTrajectory, "/cleaning_controller/joint_trajectory", 1)
        self.storage = self.create_publisher(JointTrajectory, "/storage_controller/joint_trajectory", 1)
        self.brush = self.create_publisher(Float64MultiArray, "/brush_controller/commands", 1)
        self.recovery = self.create_publisher(Float64MultiArray, "/recovery_controller/commands", 1)

    def _joint_state(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                self.positions[name].append(float(msg.position[index]))
            if index < len(msg.velocity):
                self.velocities[name].append(float(msg.velocity[index]))

    @staticmethod
    def trajectory(joints: list[str], positions: list[float], seconds: int = 2) -> JointTrajectory:
        msg = JointTrajectory()
        msg.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=seconds)
        msg.points = [point]
        return msg

    def ready(self) -> bool:
        expected = set(POSITION_TARGETS) | set(VELOCITY_TARGETS)
        return (
            all(pub.get_subscription_count() > 0 for pub in (self.cleaning, self.storage, self.brush, self.recovery))
            and expected <= set(self.positions)
            and all(self.get_publishers_info_by_topic(topic) for topic in CONTACT_TOPICS)
        )

    def publish_targets(self) -> None:
        self.cleaning.publish(self.trajectory(
            ["cleaning_lift_joint", "squeegee_pitch_joint", "squeegee_float_joint"],
            [POSITION_TARGETS[name] for name in ("cleaning_lift_joint", "squeegee_pitch_joint", "squeegee_float_joint")],
        ))
        self.storage.publish(self.trajectory(
            ["dry_bin_lid_joint", "dry_deposit_gate_joint", "wastewater_lid_joint"],
            [POSITION_TARGETS[name] for name in ("dry_bin_lid_joint", "dry_deposit_gate_joint", "wastewater_lid_joint")],
        ))
        self.brush.publish(Float64MultiArray(data=[8.0, -8.0, 12.0]))
        self.recovery.publish(Float64MultiArray(data=[20.0]))

    def stop_rotors(self) -> None:
        self.brush.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
        self.recovery.publish(Float64MultiArray(data=[0.0]))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and not node.ready() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
    if not node.ready():
        counts = {
            "cleaning": node.cleaning.get_subscription_count(),
            "storage": node.storage.get_subscription_count(),
            "brush": node.brush.get_subscription_count(),
            "recovery": node.recovery.get_subscription_count(),
            "contact_topic_publishers": {
                topic: len(node.get_publishers_info_by_topic(topic)) for topic in CONTACT_TOPICS
            },
        }
        raise SystemExit(
            "formal function readiness timeout: "
            + json.dumps({"controller_subscriptions": counts, "joint_state_names": sorted(node.positions)})
        )
    node.publish_targets()
    # gz_ros2_control's stable position loop intentionally uses a conservative
    # 0.1 proportional gain.  Give the 100 mm lift and damped service lids
    # enough real simulation time to traverse instead of judging a two-second
    # trajectory by its first few controller cycles.
    sample_deadline = time.monotonic() + 48.0
    while rclpy.ok() and time.monotonic() < sample_deadline:
        rclpy.spin_once(node, timeout_sec=0.10)
    node.stop_rotors()
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)

    measured: dict[str, dict[str, float | int | bool]] = {}
    failures: list[str] = []
    for name, target in POSITION_TARGETS.items():
        values = node.positions.get(name, [])
        terminal = values[-1] if values else None
        minimum_error = min((abs(value - target) for value in values), default=None)
        passed = minimum_error is not None and minimum_error <= 0.025
        measured[name] = {
            "target": target,
            "terminal": terminal,
            "range": max(values) - min(values) if values else 0.0,
            "minimum_target_error": minimum_error,
            "samples": len(values),
            "passed": passed,
        }
        if not passed:
            failures.append(name)
    for name, target in VELOCITY_TARGETS.items():
        values = node.velocities.get(name, [])
        peak = max(values, key=abs) if values else None
        passed = peak is not None and abs(peak - target) <= 1.5
        measured[name] = {
            "target_velocity": target,
            "peak_velocity": peak,
            "samples": len(values),
            "passed": passed,
        }
        if not passed:
            failures.append(name)

    report = {
        "report_id": "tzcup_formal_function_positions_runtime_v1",
        "status": "FORMAL_CLEANING_STORAGE_AND_RECOVERY_ACTUATORS_PASSED" if not failures else "FAILED",
        "controller_count": 4,
        "actuated_joint_count": len(measured),
        "contact_topic_publishers": {
            topic: len(node.get_publishers_info_by_topic(topic)) for topic in CONTACT_TOPICS
        },
        "measured": measured,
        "failures": failures,
        "claim_boundary": "This proves controller-to-joint motion for cleaning, storage and pump positions; it does not prove hydraulic recovery efficiency, brush wear or debris pickup.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(report, ensure_ascii=False))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
