#!/usr/bin/env python3
"""Fail-closed readiness probe for the formal 200 x 100 m campus runtime."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import tempfile
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool


REQUIRED_NODES = {
    "/robot_state_publisher",
    "/controller_manager",
    "/formal_legacy_topic_adapter",
    "/whole_vehicle_safety_manager",
    "/map_server",
    "/amcl",
    "/coverage_server",
}
LIFECYCLE_NODES = (
    "map_server",
    "amcl",
    "keepout_filter_mask_server",
    "keepout_costmap_filter_info_server",
    "speed_filter_mask_server",
    "speed_costmap_filter_info_server",
    "coverage_server",
)
ACTIVE_CONTROLLERS = {
    "joint_state_broadcaster",
    "arm_controller",
    "gripper_controller",
    "cleaning_controller",
    "storage_controller",
    "service_controller",
}
INACTIVE_CONTROLLERS = {"brush_controller", "recovery_controller"}
REQUIRED_TOPIC_TYPES = {
    "/map": "nav_msgs/msg/OccupancyGrid",
    "/scan": "sensor_msgs/msg/LaserScan",
    "/sensors/lidar_3d/points": "sensor_msgs/msg/PointCloud2",
    "/odom": "nav_msgs/msg/Odometry",
    "/emergency_stop": "std_msgs/msg/Bool",
}


class CampusRuntimeProbe(Node):
    def __init__(self) -> None:
        super().__init__("formal_campus_runtime_probe")
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.sample_counts = {name: 0 for name in REQUIRED_TOPIC_TYPES}
        self.estop_values: list[bool] = []
        self.create_subscription(
            OccupancyGrid, "/map", lambda _: self._sample("/map"), map_qos
        )
        self.create_subscription(
            LaserScan, "/scan", lambda _: self._sample("/scan"), sensor_qos
        )
        self.create_subscription(
            PointCloud2,
            "/sensors/lidar_3d/points",
            lambda _: self._sample("/sensors/lidar_3d/points"),
            sensor_qos,
        )
        self.create_subscription(
            Odometry, "/odom", lambda _: self._sample("/odom"), sensor_qos
        )
        self.create_subscription(Bool, "/emergency_stop", self._estop, 10)
        self.controller_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.lifecycle_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in LIFECYCLE_NODES
        }

    def _sample(self, topic: str) -> None:
        self.sample_counts[topic] += 1

    def _estop(self, message: Bool) -> None:
        self.sample_counts["/emergency_stop"] += 1
        self.estop_values.append(bool(message.data))

    def node_names(self) -> set[str]:
        result = set()
        for name, namespace in self.get_node_names_and_namespaces():
            result.add(f"/{name}" if namespace == "/" else f"{namespace}/{name}")
        return result

    def topic_types(self) -> dict[str, list[str]]:
        return {name: types for name, types in self.get_topic_names_and_types()}

    def controller_states(self) -> dict[str, str]:
        if not self.controller_client.wait_for_service(timeout_sec=0.05):
            return {}
        future = self.controller_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        response = future.result() if future.done() else None
        if response is None:
            return {}
        return {controller.name: controller.state for controller in response.controller}

    def lifecycle_states(self) -> dict[str, str]:
        states: dict[str, str] = {}
        for name, client in self.lifecycle_clients.items():
            if not client.wait_for_service(timeout_sec=0.02):
                continue
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            response = future.result() if future.done() else None
            if response is not None:
                states[name] = response.current_state.label
        return states


def _atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=path.parent, delete=False
    ) as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    temporary.replace(path)


def validate(timeout_s: float, output: Path) -> dict:
    started = time.monotonic()
    rclpy.init()
    probe = CampusRuntimeProbe()
    nodes: set[str] = set()
    topics: dict[str, list[str]] = {}
    controllers: dict[str, str] = {}
    lifecycle: dict[str, str] = {}
    try:
        while time.monotonic() - started < timeout_s:
            rclpy.spin_once(probe, timeout_sec=0.25)
            nodes = probe.node_names()
            topics = probe.topic_types()
            controllers = probe.controller_states()
            lifecycle = probe.lifecycle_states()
            node_gate = REQUIRED_NODES <= nodes
            topic_gate = all(
                expected in topics.get(name, [])
                for name, expected in REQUIRED_TOPIC_TYPES.items()
            )
            sample_gate = all(probe.sample_counts[name] > 0 for name in REQUIRED_TOPIC_TYPES)
            controller_gate = all(
                controllers.get(name) == "active" for name in ACTIVE_CONTROLLERS
            ) and all(
                controllers.get(name) == "inactive" for name in INACTIVE_CONTROLLERS
            )
            lifecycle_gate = all(
                lifecycle.get(name) == "active" for name in LIFECYCLE_NODES
            )
            estop_gate = bool(probe.estop_values) and all(probe.estop_values)
            if all(
                (
                    node_gate,
                    topic_gate,
                    sample_gate,
                    controller_gate,
                    lifecycle_gate,
                    estop_gate,
                )
            ):
                break
            time.sleep(0.5)
    finally:
        elapsed = time.monotonic() - started
        probe.destroy_node()
        rclpy.shutdown()

    gates = {
        "required_nodes_discovered": REQUIRED_NODES <= nodes,
        "required_topic_types_discovered": all(
            expected in topics.get(name, [])
            for name, expected in REQUIRED_TOPIC_TYPES.items()
        ),
        "required_topic_samples_received": all(
            probe.sample_counts[name] > 0 for name in REQUIRED_TOPIC_TYPES
        ),
        "required_controllers_in_safe_states": all(
            controllers.get(name) == "active" for name in ACTIVE_CONTROLLERS
        )
        and all(controllers.get(name) == "inactive" for name in INACTIVE_CONTROLLERS),
        "map_filter_and_coverage_lifecycle_active": all(
            lifecycle.get(name) == "active" for name in LIFECYCLE_NODES
        ),
        "initial_estop_remained_asserted": bool(probe.estop_values)
        and all(probe.estop_values),
    }
    payload = {
        "report_id": "tzcup_formal_campus_runtime_readiness_v1",
        "status": "PASSED" if all(gates.values()) else "BLOCKED",
        "elapsed_s": round(elapsed, 3),
        "ros_domain_id": int(os.environ.get("ROS_DOMAIN_ID", "0")),
        "ros_automatic_discovery_range": os.environ.get(
            "ROS_AUTOMATIC_DISCOVERY_RANGE", "SYSTEM_DEFAULT"
        ),
        "gates": gates,
        "required_nodes": sorted(REQUIRED_NODES),
        "discovered_required_nodes": sorted(REQUIRED_NODES & nodes),
        "controller_states": controllers,
        "lifecycle_states": lifecycle,
        "required_topic_types": REQUIRED_TOPIC_TYPES,
        "discovered_required_topic_types": {
            name: topics.get(name, []) for name in REQUIRED_TOPIC_TYPES
        },
        "topic_sample_counts": probe.sample_counts,
        "estop_samples": {
            "count": len(probe.estop_values),
            "all_asserted": bool(probe.estop_values) and all(probe.estop_values),
        },
        "claim_boundary": (
            "Single-host WSL launch readiness only. The probe never clears the "
            "E-stop, commands motion, or consumes evaluator-private truth."
        ),
    }
    _atomic_json(output, payload)
    return payload


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = validate(args.timeout, args.output)
    print(json.dumps(report, sort_keys=True))
    return 0 if report["status"] == "PASSED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
