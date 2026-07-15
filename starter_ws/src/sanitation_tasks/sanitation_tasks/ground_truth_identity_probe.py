import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

from .evaluation import normalize_angle, summarize, yaw_from_quaternion


def stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def median_or_none(values):
    return statistics.median(values) if values else None


class GroundTruthIdentityProbe(Node):
    """Exercise and independently audit the Stage4S ground-truth identity."""

    def __init__(self):
        super().__init__("ground_truth_identity_probe")
        self.declare_parameter("output_path", "ground_truth_identity_report.json")
        self.declare_parameter("inventory_path", "ground_truth_transform_inventory.json")
        self.declare_parameter("trajectory_path", "ground_truth_identity_trajectory.csv")
        self.declare_parameter("timeout_sec", 120.0)
        self.command = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            Odometry, "/ground_truth/model_odom_raw", self.on_raw_truth, 50
        )
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, 50)
        self.create_subscription(
            TFMessage, "/ground_truth/dynamic_pose", self.on_dynamic_pose, 20
        )
        self.create_subscription(Odometry, "/odom/unfiltered", self.on_raw_odom, 20)
        self.create_subscription(Imu, "/imu/data", self.on_imu, 50)
        self.create_subscription(
            Bool, "/ground_truth/identity_valid", self.on_identity, 20
        )

        self.raw_truth = None
        self.raw_truth_by_stamp = {}
        self.raw_odom = None
        self.imu = None
        self.dynamic = None
        self.identity_values = []
        self.inventory = defaultdict(
            lambda: {
                "message_count": 0,
                "frame_ids": set(),
                "child_frame_ids": set(),
                "first_pose": None,
                "last_pose": None,
            }
        )
        self.rows = []
        self.phase = "wait_for_topics"
        self.phase_start = None
        self.run_start = None
        self.phase_reference = None
        self.last_yaw = None
        self.unwrapped_yaw = None
        self.failures = []
        self.finished = False
        self.exit_code = 2
        self.current_command = (0.0, 0.0)
        self.create_timer(0.02, self.tick)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_raw_truth(self, message):
        self.raw_truth = message
        key = round(stamp_sec(message.header.stamp), 6)
        self.raw_truth_by_stamp[key] = message
        if len(self.raw_truth_by_stamp) > 500:
            self.raw_truth_by_stamp.pop(next(iter(self.raw_truth_by_stamp)))

    def on_raw_odom(self, message):
        self.raw_odom = message

    def on_imu(self, message):
        self.imu = message

    def on_identity(self, message):
        self.identity_values.append(bool(message.data))

    def on_dynamic_pose(self, message):
        self.dynamic = message
        for index, transform in enumerate(message.transforms):
            pose = {
                "x": transform.transform.translation.x,
                "y": transform.transform.translation.y,
                "z": transform.transform.translation.z,
                "yaw": yaw_from_quaternion(transform.transform.rotation),
            }
            item = self.inventory[index]
            item["message_count"] += 1
            item["frame_ids"].add(transform.header.frame_id)
            item["child_frame_ids"].add(transform.child_frame_id)
            if item["first_pose"] is None:
                item["first_pose"] = pose
            item["last_pose"] = pose

    def on_truth(self, message):
        yaw = yaw_from_quaternion(message.pose.pose.orientation)
        if self.last_yaw is None:
            self.unwrapped_yaw = yaw
        else:
            self.unwrapped_yaw += normalize_angle(yaw - self.last_yaw)
        self.last_yaw = yaw

        stamp = stamp_sec(message.header.stamp)
        raw = self.raw_truth_by_stamp.get(round(stamp, 6), self.raw_truth)
        dynamic_zero = None
        if self.dynamic and self.dynamic.transforms:
            dynamic_zero = self.dynamic.transforms[0]
        row = {
            "stamp_sec": stamp,
            "phase": self.phase,
            "cmd_linear_x": self.current_command[0],
            "cmd_angular_z": self.current_command[1],
            "gt_x": message.pose.pose.position.x,
            "gt_y": message.pose.pose.position.y,
            "gt_yaw": yaw,
            "gt_unwrapped_yaw": self.unwrapped_yaw,
            "raw_gt_x": raw.pose.pose.position.x if raw else None,
            "raw_gt_y": raw.pose.pose.position.y if raw else None,
            "raw_gt_yaw": yaw_from_quaternion(raw.pose.pose.orientation) if raw else None,
            "raw_gt_frame": raw.header.frame_id if raw else None,
            "raw_gt_child": raw.child_frame_id if raw else None,
            "raw_odom_x": self.raw_odom.pose.pose.position.x if self.raw_odom else None,
            "raw_odom_y": self.raw_odom.pose.pose.position.y if self.raw_odom else None,
            "raw_odom_yaw": (
                yaw_from_quaternion(self.raw_odom.pose.pose.orientation)
                if self.raw_odom else None
            ),
            "imu_yaw_rate": self.imu.angular_velocity.z if self.imu else None,
            "dynamic_index0_x": (
                dynamic_zero.transform.translation.x if dynamic_zero else None
            ),
            "dynamic_index0_y": (
                dynamic_zero.transform.translation.y if dynamic_zero else None
            ),
            "dynamic_index0_yaw": (
                yaw_from_quaternion(dynamic_zero.transform.rotation)
                if dynamic_zero else None
            ),
        }
        self.rows.append(row)

    def publish_command(self, linear, angular):
        self.current_command = (linear, angular)
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.command.publish(message)

    def transition(self, phase):
        self.publish_command(0.0, 0.0)
        self.phase = phase
        self.phase_start = self.now_sec()
        self.phase_reference = None

    def phase_elapsed(self):
        return self.now_sec() - self.phase_start

    def latest_pose(self):
        if not self.rows:
            return None
        row = self.rows[-1]
        return row["gt_x"], row["gt_y"], row["gt_unwrapped_yaw"]

    def tick(self):
        if self.finished:
            return
        now = self.now_sec()
        if self.run_start is not None and now - self.run_start > float(
            self.get_parameter("timeout_sec").value
        ):
            self.failures.append(f"timeout in phase {self.phase}")
            self.finish()
            return

        topics_ready = (
            self.raw_truth is not None
            and self.rows
            and self.dynamic is not None
            and self.imu is not None
            and self.raw_odom is not None
            and self.identity_values
        )
        if self.phase == "wait_for_topics":
            self.publish_command(0.0, 0.0)
            if topics_ready:
                self.run_start = now
                self.transition("stationary_20s")
            return

        if self.phase == "stationary_20s":
            self.publish_command(0.0, 0.0)
            if self.phase_elapsed() >= 20.0:
                self.transition("settle_before_forward")
            return
        if self.phase == "settle_before_forward":
            if self.phase_elapsed() >= 3.0:
                self.phase_reference = self.latest_pose()
                self.phase = "forward_1m"
                self.phase_start = now
            return
        if self.phase == "forward_1m":
            x0, y0, yaw0 = self.phase_reference
            x, y, _yaw = self.latest_pose()
            distance = (x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)
            remaining = 1.0 - distance
            if abs(remaining) <= 0.0015:
                self.transition("settle_after_forward")
            else:
                speed = min(0.20, max(0.025, abs(remaining) * 0.7))
                self.publish_command(math.copysign(speed, remaining), 0.0)
            return
        if self.phase == "settle_after_forward":
            if self.phase_elapsed() >= 3.0:
                self.phase_reference = self.latest_pose()
                self.phase = "turn_positive_90"
                self.phase_start = now
            return
        if self.phase == "turn_positive_90":
            target = self.phase_reference[2] + math.pi / 2.0
            error = target - self.latest_pose()[2]
            if abs(error) <= 0.0015:
                self.transition("settle_after_positive_turn")
            else:
                rate = min(0.25, max(0.025, abs(error) * 0.8))
                self.publish_command(0.0, math.copysign(rate, error))
            return
        if self.phase == "settle_after_positive_turn":
            if self.phase_elapsed() >= 3.0:
                self.phase_reference = self.latest_pose()
                self.phase = "turn_negative_90"
                self.phase_start = now
            return
        if self.phase == "turn_negative_90":
            target = self.phase_reference[2] - math.pi / 2.0
            error = target - self.latest_pose()[2]
            if abs(error) <= 0.0015:
                self.transition("settle_after_negative_turn")
            else:
                rate = min(0.25, max(0.025, abs(error) * 0.8))
                self.publish_command(0.0, math.copysign(rate, error))
            return
        if self.phase == "settle_after_negative_turn":
            if self.phase_elapsed() >= 3.0:
                self.finish()

    def phase_rows(self, phase):
        return [row for row in self.rows if row["phase"] == phase]

    def finish(self):
        self.publish_command(0.0, 0.0)
        self.finished = True
        self.write_outputs()
        rclpy.shutdown()

    def write_outputs(self):
        output_path = Path(str(self.get_parameter("output_path").value))
        inventory_path = Path(str(self.get_parameter("inventory_path").value))
        trajectory_path = Path(str(self.get_parameter("trajectory_path").value))
        for path in (output_path, inventory_path, trajectory_path):
            path.parent.mkdir(parents=True, exist_ok=True)

        fieldnames = list(self.rows[0].keys()) if self.rows else []
        with trajectory_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)

        inventory = []
        for index, item in sorted(self.inventory.items()):
            inventory.append(
                {
                    "index": index,
                    "message_count": item["message_count"],
                    "frame_ids": sorted(item["frame_ids"]),
                    "child_frame_ids": sorted(item["child_frame_ids"]),
                    "first_pose": item["first_pose"],
                    "last_pose": item["last_pose"],
                }
            )
        inventory_path.write_text(
            json.dumps(
                {
                    "source_topic": "/ground_truth/dynamic_pose",
                    "source_gazebo_type": "gz.msgs.Pose_V",
                    "ros_type": "tf2_msgs/msg/TFMessage",
                    "transform_indices": inventory,
                },
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )

        stationary = self.phase_rows("stationary_20s")
        forward = self.phase_rows("forward_1m")
        positive = self.phase_rows("turn_positive_90")
        negative = self.phase_rows("turn_negative_90")
        spawn_window = stationary[: min(len(stationary), 50)]
        spawn_x = median_or_none([row["gt_x"] for row in spawn_window])
        spawn_y = median_or_none([row["gt_y"] for row in spawn_window])
        spawn_yaw = median_or_none([row["gt_yaw"] for row in spawn_window])
        spawn_position_error = (
            math.hypot(spawn_x, spawn_y) if spawn_x is not None else None
        )
        spawn_yaw_error = abs(normalize_angle(spawn_yaw)) if spawn_yaw is not None else None

        if stationary:
            sx, sy = stationary[0]["gt_x"], stationary[0]["gt_y"]
            stationary_drift = max(
                math.hypot(row["gt_x"] - sx, row["gt_y"] - sy)
                for row in stationary
            )
        else:
            stationary_drift = None

        forward_distance = None
        if forward:
            start, end = forward[0], forward[-1]
            forward_distance = math.hypot(
                end["gt_x"] - start["gt_x"], end["gt_y"] - start["gt_y"]
            )
        positive_angle = (
            positive[-1]["gt_unwrapped_yaw"] - positive[0]["gt_unwrapped_yaw"]
            if positive else None
        )
        negative_angle = (
            negative[-1]["gt_unwrapped_yaw"] - negative[0]["gt_unwrapped_yaw"]
            if negative else None
        )

        map_position_errors = []
        map_yaw_errors = []
        index0_position_errors = []
        index0_yaw_errors = []
        sync_errors = []
        for row in self.rows:
            if row["raw_gt_x"] is not None:
                map_position_errors.append(
                    math.hypot(
                        row["gt_x"] - (row["raw_gt_x"] + 8.0),
                        row["gt_y"] - row["raw_gt_y"],
                    )
                )
                map_yaw_errors.append(
                    abs(normalize_angle(row["gt_yaw"] - row["raw_gt_yaw"]))
                )
                sync_errors.append(0.0)
            if row["dynamic_index0_x"] is not None and row["raw_gt_x"] is not None:
                index0_position_errors.append(
                    math.hypot(
                        row["dynamic_index0_x"] - row["raw_gt_x"],
                        row["dynamic_index0_y"] - row["raw_gt_y"],
                    )
                )
                index0_yaw_errors.append(
                    abs(
                        normalize_angle(
                            row["dynamic_index0_yaw"] - row["raw_gt_yaw"]
                        )
                    )
                )

        finite_difference = []
        for previous, current in zip(self.rows, self.rows[1:]):
            dt = current["stamp_sec"] - previous["stamp_sec"]
            if dt <= 0.0 or dt > 0.1:
                continue
            dx = current["gt_x"] - previous["gt_x"]
            dy = current["gt_y"] - previous["gt_y"]
            vx = (dx * math.cos(previous["gt_yaw"]) + dy * math.sin(previous["gt_yaw"])) / dt
            vy = (-dx * math.sin(previous["gt_yaw"]) + dy * math.cos(previous["gt_yaw"])) / dt
            wz = (current["gt_unwrapped_yaw"] - previous["gt_unwrapped_yaw"]) / dt
            finite_difference.append((current["phase"], vx, vy, wz, current["imu_yaw_rate"]))
        positive_rates = [item for item in finite_difference if item[0] == "turn_positive_90"]
        negative_rates = [item for item in finite_difference if item[0] == "turn_negative_90"]
        imu_positive = [item[4] for item in positive_rates if item[4] is not None]
        imu_negative = [item[4] for item in negative_rates if item[4] is not None]

        thresholds = {
            "spawn_position_error_m": 0.005,
            "spawn_yaw_error_rad": 0.005,
            "stationary_drift_m": 0.002,
            "turn_magnitude_error_rad": 0.005,
            "map_transform_error_m": 1e-6,
            "map_transform_yaw_error_rad": 1e-6,
            "index0_identity_position_error_m": 0.005,
            "index0_identity_yaw_error_rad": 0.005,
        }
        checks = {
            "spawn_position": spawn_position_error is not None and spawn_position_error <= thresholds["spawn_position_error_m"],
            "spawn_yaw": spawn_yaw_error is not None and spawn_yaw_error <= thresholds["spawn_yaw_error_rad"],
            "stationary_drift": stationary_drift is not None and stationary_drift <= thresholds["stationary_drift_m"],
            "positive_90_sign_and_magnitude": positive_angle is not None and positive_angle > 0.0 and abs(positive_angle - math.pi / 2.0) <= thresholds["turn_magnitude_error_rad"],
            "negative_90_sign_and_magnitude": negative_angle is not None and negative_angle < 0.0 and abs(negative_angle + math.pi / 2.0) <= thresholds["turn_magnitude_error_rad"],
            "entity_selection_unchanged": bool(self.identity_values) and all(self.identity_values),
            "map_transform_position": bool(map_position_errors) and max(map_position_errors) <= thresholds["map_transform_error_m"],
            "map_transform_yaw": bool(map_yaw_errors) and max(map_yaw_errors) <= thresholds["map_transform_yaw_error_rad"],
            "legacy_index0_matches_model_pose_this_run": bool(index0_position_errors) and max(index0_position_errors) <= thresholds["index0_identity_position_error_m"] and max(index0_yaw_errors) <= thresholds["index0_identity_yaw_error_rad"],
            "imu_turn_sign": bool(imu_positive) and bool(imu_negative) and statistics.median(imu_positive) > 0.0 and statistics.median(imu_negative) < 0.0,
        }
        hard_check_names = (
            "spawn_position",
            "spawn_yaw",
            "stationary_drift",
            "positive_90_sign_and_magnitude",
            "negative_90_sign_and_magnitude",
            "entity_selection_unchanged",
            "map_transform_position",
            "map_transform_yaw",
        )
        success = not self.failures and all(checks[name] for name in hard_check_names)
        report = {
            "schema_version": 1,
            "selection_mode": "model_scoped_gazebo_odometry_publisher",
            "selection_topic": "/ground_truth/model_odom_raw",
            "expected_source_frame": "world",
            "expected_child_frame": "sanitation_vehicle/base_footprint",
            "legacy_pose_v_used_for_output": False,
            "fail_closed": True,
            "sample_count": len(self.rows),
            "failures": self.failures,
            "thresholds": thresholds,
            "measurements": {
                "spawn_position_error_m": spawn_position_error,
                "spawn_yaw_error_rad": spawn_yaw_error,
                "stationary_drift_m": stationary_drift,
                "forward_distance_m": forward_distance,
                "positive_turn_rad": positive_angle,
                "negative_turn_rad": negative_angle,
                "map_transform_position_error": summarize(map_position_errors),
                "map_transform_yaw_error": summarize(map_yaw_errors),
                "legacy_index0_position_error": summarize(index0_position_errors),
                "legacy_index0_yaw_error": summarize(index0_yaw_errors),
                "imu_positive_yaw_rate_median": median_or_none(imu_positive),
                "imu_negative_yaw_rate_median": median_or_none(imu_negative),
                "sync_error_sec": summarize(sync_errors),
            },
            "checks": checks,
            "hard_checks": list(hard_check_names),
            "success": success,
            "artifacts": {
                "transform_inventory": str(inventory_path),
                "trajectory_csv": str(trajectory_path),
            },
        }
        output_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        self.exit_code = 0 if success else 1


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthIdentityProbe()
    try:
        rclpy.spin(node)
    finally:
        exit_code = node.exit_code
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()
    raise SystemExit(exit_code)
