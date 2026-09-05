#!/usr/bin/env python3
"""Collect evaluator-only pedestrian and contact truth without controlling the robot."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String


class EnvironmentTruthCollector(Node):
    """Observe environment truth in a process with no command publishers/actions."""

    def __init__(self, *, timeout_s: float) -> None:
        super().__init__("formal_dynamic_environment_truth_collector")
        self.timeout_s = timeout_s
        self.started = time.monotonic()
        self.done = False
        self.status_samples: list[dict] = []
        self.active_pedestrian_count = 0
        self.status_error_count = 0
        self.front_contact_sample_count = 0
        self.rear_contact_sample_count = 0
        self.front_collision_sample_count = 0
        self.rear_collision_sample_count = 0
        self.create_subscription(
            String,
            "/scenario/environment/pedestrian_driver/status",
            self._pedestrian_status,
            20,
        )
        self.create_subscription(
            Contacts,
            "/safety/front_bumper/contact",
            self._front_contact,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Contacts,
            "/safety/rear_bumper/contact",
            self._rear_contact,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self._tick)

    def _pedestrian_status(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            self.status_error_count += 1
            return
        state = value.get("state")
        if isinstance(state, str) and state.startswith("ERROR_"):
            self.status_error_count += 1
        elapsed = value.get("schedule_elapsed_s")
        if state != "ACTIVE" or not isinstance(elapsed, (int, float)):
            return
        count = value.get("pedestrian_count")
        if not isinstance(count, int):
            self.status_error_count += 1
            return
        self.active_pedestrian_count = count
        self.status_samples.append(
            {
                "observation_ros_time_ns": self.get_clock().now().nanoseconds,
                "schedule_elapsed_s": float(elapsed),
                "pedestrian_count": count,
            }
        )

    def _front_contact(self, message: Contacts) -> None:
        self.front_contact_sample_count += 1
        self.front_collision_sample_count += int(bool(message.contacts))

    def _rear_contact(self, message: Contacts) -> None:
        self.rear_contact_sample_count += 1
        self.rear_collision_sample_count += int(bool(message.contacts))

    def _tick(self) -> None:
        if time.monotonic() - self.started >= self.timeout_s:
            self.done = True

    def telemetry(self) -> dict:
        return {
            "schema_version": 1,
            "collector_role": "evaluator_only_no_robot_control",
            "active_pedestrian_count": self.active_pedestrian_count,
            "pedestrian_status_samples": self.status_samples,
            "pedestrian_status_error_count": self.status_error_count,
            "collision_count": (
                self.front_collision_sample_count + self.rear_collision_sample_count
            ),
            "topic_sample_counts": {
                "/scenario/environment/pedestrian_driver/status": len(
                    self.status_samples
                ),
                "/safety/front_bumper/contact": self.front_contact_sample_count,
                "/safety/rear_bumper/contact": self.rear_contact_sample_count,
            },
            "evaluator_truth_topics_subscribed": [
                "/scenario/environment/pedestrian_driver/status",
                "/safety/front_bumper/contact",
                "/safety/rear_bumper/contact",
            ],
            "control_topics_published": [],
            "product_actions_created": [],
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=330.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.timeout <= 0.0:
        raise ValueError("timeout must be positive")
    rclpy.init()
    node = EnvironmentTruthCollector(timeout_s=args.timeout)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        value = node.telemetry()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
