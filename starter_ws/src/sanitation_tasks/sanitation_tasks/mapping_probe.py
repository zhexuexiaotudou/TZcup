import csv
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node

from .evaluation import normalize_angle, yaw_from_quaternion


DEFAULT_ROUTE = [
    [0.0, 0.0], [15.0, 0.0], [15.0, 7.0], [8.0, 8.0],
    [-6.0, 8.0], [-6.0, -8.0], [8.0, -8.0], [15.0, -8.0],
    [0.0, -8.0], [0.0, 0.0],
]


class MappingProbe(Node):
    """Ground-truth feedback route driver used to gather an actual SLAM map."""

    def __init__(self):
        super().__init__("mapping_probe")
        self.declare_parameter("output_path", "/tmp/mapping_probe.json")
        self.declare_parameter("trajectory_path", "/tmp/mapping_trajectory.csv")
        self.declare_parameter("route_json", json.dumps(DEFAULT_ROUTE))
        self.declare_parameter("route_file", "")
        self.declare_parameter("feedback_topic", "/odom")
        self.declare_parameter("command_topic", "/cmd_vel_gate")
        self.declare_parameter("max_linear_speed", 0.30)
        self.declare_parameter("max_angular_speed", 0.25)
        self.declare_parameter("waypoint_tolerance", 0.25)
        self.declare_parameter("timeout_sec", 300.0)
        route_file = self.get_parameter("route_file").value
        self.route = json.loads(Path(route_file).read_text(encoding="utf-8")) if route_file else json.loads(self.get_parameter("route_json").value)
        self.index = 0
        self.pose = None
        self.samples = []
        self.truth_samples = []
        self.map_snapshots = []
        self.publisher = self.create_publisher(
            Twist, self.get_parameter("command_topic").value, 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("feedback_topic").value),
            self._feedback,
            20,
        )
        self.create_subscription(Odometry, "/ground_truth/odom", self._truth, 20)
        self.create_subscription(OccupancyGrid, "/map", self._map, 10)

    def _feedback(self, message):
        pose = message.pose.pose
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.pose = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation), stamp)
        self.samples.append(self.pose)

    def _truth(self, message):
        pose = message.pose.pose
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.truth_samples.append(
            (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation), stamp)
        )

    def _map(self, message):
        data = message.data
        known = sum(value >= 0 for value in data)
        occupied = sum(value >= 65 for value in data)
        free = sum(0 <= value < 25 for value in data)
        self.map_snapshots.append({
            "stamp_sec": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
            "width_cells": message.info.width,
            "height_cells": message.info.height,
            "resolution_m": message.info.resolution,
            "known_cells": known,
            "free_cells": free,
            "occupied_cells": occupied,
            "unknown_cells": len(data) - known,
            "known_area_m2": known * message.info.resolution ** 2,
            "span_x_m": message.info.width * message.info.resolution,
            "span_y_m": message.info.height * message.info.resolution,
        })

    def stop(self):
        self.publisher.publish(Twist())

    def run(self):
        started = time.monotonic()
        while rclpy.ok() and time.monotonic() - started < self.get_parameter("timeout_sec").value:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is None:
                continue
            if self.index >= len(self.route):
                break
            x, y, yaw, _stamp = self.pose
            target_x, target_y = self.route[self.index]
            distance = math.hypot(target_x - x, target_y - y)
            if distance <= self.get_parameter("waypoint_tolerance").value:
                self.index += 1
                continue
            target_yaw = math.atan2(target_y - y, target_x - x)
            yaw_error = normalize_angle(target_yaw - yaw)
            command = Twist()
            max_angular = self.get_parameter("max_angular_speed").value
            command.angular.z = max(-max_angular, min(max_angular, 1.8 * yaw_error))
            if abs(yaw_error) < 0.75:
                speed = min(self.get_parameter("max_linear_speed").value, 0.20 + 0.35 * distance)
                command.linear.x = speed * max(0.15, math.cos(yaw_error))
            self.publisher.publish(command)
            # A rclpy Rate can deadlock here because this probe deliberately
            # drives its own single-threaded spin loop. Wall sleep keeps command
            # publication deterministic while callbacks are serviced above.
            time.sleep(0.05)
        self.stop()
        completed = self.index >= len(self.route)
        report = {
            "success": completed and bool(self.map_snapshots),
            "route_completed": completed,
            "waypoints_completed": self.index,
            "waypoint_count": len(self.route),
            "trajectory_sample_count": len(self.samples),
            "map_update_count": len(self.map_snapshots),
            "final_map": self.map_snapshots[-1] if self.map_snapshots else None,
            "route": self.route,
            "feedback_topic": str(self.get_parameter("feedback_topic").value),
            "ground_truth_used_for_control": str(self.get_parameter("feedback_topic").value) == "/ground_truth/odom",
            "ground_truth_evaluation_sample_count": len(self.truth_samples),
        }
        with open(self.get_parameter("trajectory_path").value, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["stamp_sec", "x_m", "y_m", "yaw_rad"])
            writer.writerows((stamp, x, y, yaw) for x, y, yaw, stamp in self.truth_samples)
        with open(self.get_parameter("output_path").value, "w", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        return completed


def main(args=None):
    rclpy.init(args=args)
    node = MappingProbe()
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not success:
        raise SystemExit(2)
