"""Run one fixed-time or IMU-feedback yaw trial and save every raw sample."""

import csv
import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .evaluation import normalize_angle, yaw_from_quaternion
from .transient_analysis import analyze_transient_samples


class TransientResponseRunner(Node):
    def __init__(self):
        super().__init__("transient_response_runner")
        self.declare_parameter("output_path", "/tmp/angular_rate_trial.json")
        self.declare_parameter("csv_path", "/tmp/angular_rate_trial.csv")
        self.declare_parameter("trial_id", "trial")
        self.declare_parameter("trial_type", "fixed_time")
        self.declare_parameter("thermal_state", "cold")
        self.declare_parameter("angular_rate", 0.25)
        self.declare_parameter("fixed_duration_sec", 8.0)
        self.declare_parameter("target_heading_rad", math.pi / 2.0)
        self.declare_parameter("heading_tolerance_rad", math.radians(1.0))
        self.declare_parameter("timeout_sec", 90.0)
        self.publisher = self.create_publisher(Twist, "/cmd_vel_gate", 20)
        self.create_subscription(Twist, "/cmd_vel", self.on_output, 50)
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, 50)
        self.create_subscription(Odometry, "/odom/unfiltered", self.on_raw, 50)
        self.create_subscription(Odometry, "/odom", self.on_ekf, 50)
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.output_command = Twist()
        self.raw = self.ekf = self.imu = None
        self.rows = []
        self.phase = "waiting"
        self.phase_start = None
        self.run_start = None
        self.last_imu_stamp = None
        self.feedback_heading = 0.0
        self.stable_since = None
        self.requested_rate = 0.0
        self.finished = False
        self.exit_code = 2
        self.warmup = [0.10, -0.10, 0.25, -0.25, 0.35, -0.35, 0.45, -0.45, 0.60, -0.60]
        self.warmup_index = 0
        self.create_timer(0.02, self.tick)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    @staticmethod
    def stamp(message):
        return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1.0e-9

    def on_output(self, message): self.output_command = message
    def on_raw(self, message): self.raw = message
    def on_ekf(self, message): self.ekf = message

    def on_imu(self, message):
        if self.phase == "trial":
            stamp = self.stamp(message)
            if self.last_imu_stamp is not None:
                dt = stamp - self.last_imu_stamp
                if 0.0 < dt <= 0.25:
                    self.feedback_heading += message.angular_velocity.z * dt
            self.last_imu_stamp = stamp
        self.imu = message

    def on_truth(self, message):
        if self.phase != "trial" or None in (self.raw, self.ekf, self.imu):
            return
        self.rows.append({
            "stamp_sec": self.stamp(message),
            "trial_active": True,
            "cmd_requested_angular": self.requested_rate,
            "cmd_output_angular": self.output_command.angular.z,
            "gt_yaw": yaw_from_quaternion(message.pose.pose.orientation),
            "gt_angular_z": message.twist.twist.angular.z,
            "raw_yaw": yaw_from_quaternion(self.raw.pose.pose.orientation),
            "raw_angular_z": self.raw.twist.twist.angular.z,
            "imu_yaw": yaw_from_quaternion(self.imu.orientation),
            "imu_yaw_rate": self.imu.angular_velocity.z,
            "ekf_yaw": yaw_from_quaternion(self.ekf.pose.pose.orientation),
            "ekf_angular_z": self.ekf.twist.twist.angular.z,
        })

    def publish(self, angular):
        self.requested_rate = float(angular)
        message = Twist(); message.angular.z = self.requested_rate
        self.publisher.publish(message)

    def ready(self): return None not in (self.raw, self.ekf, self.imu)

    def begin_phase(self, phase):
        self.phase = phase; self.phase_start = self.now_sec()
        if phase == "trial":
            self.rows = []; self.feedback_heading = 0.0; self.last_imu_stamp = None; self.stable_since = None

    def tick(self):
        if self.finished: return
        now = self.now_sec()
        if self.run_start is None:
            self.publish(0.0)
            if self.ready():
                self.run_start = now
                self.begin_phase("warmup" if self.get_parameter("thermal_state").value == "hot" else "pre_rest")
            return
        if now - self.run_start > float(self.get_parameter("timeout_sec").value):
            self.finish(False, "timeout"); return
        elapsed = now - self.phase_start
        if self.phase == "warmup":
            command_time = 0.75
            slot = int(elapsed / command_time)
            if slot >= len(self.warmup):
                self.publish(0.0); self.begin_phase("pre_rest"); return
            self.publish(self.warmup[slot]); return
        if self.phase == "pre_rest":
            self.publish(0.0)
            if elapsed >= 1.5: self.begin_phase("trial")
            return
        if self.phase == "trial":
            rate = float(self.get_parameter("angular_rate").value)
            if self.get_parameter("trial_type").value == "fixed_time":
                self.publish(rate)
                if elapsed >= float(self.get_parameter("fixed_duration_sec").value):
                    self.publish(0.0); self.begin_phase("post_rest")
            else:
                target = float(self.get_parameter("target_heading_rad").value)
                error = target - self.feedback_heading
                tolerance = float(self.get_parameter("heading_tolerance_rad").value)
                if abs(error) <= tolerance:
                    self.publish(0.0)
                    if self.stable_since is None: self.stable_since = now
                    if now - self.stable_since >= 0.5: self.begin_phase("post_rest")
                else:
                    self.stable_since = None
                    limit = abs(rate)
                    command = math.copysign(min(limit, max(0.06, 1.2 * abs(error))), error)
                    self.publish(command)
            return
        if self.phase == "post_rest":
            self.publish(0.0)
            if elapsed >= 2.0: self.finish(True, None)

    def finish(self, complete, error):
        if self.finished: return
        self.finished = True; self.publish(0.0)
        rate = float(self.get_parameter("angular_rate").value)
        metrics = analyze_transient_samples(self.rows, rate)
        report = {
            "schema_version": 1,
            "trial_id": str(self.get_parameter("trial_id").value),
            "trial_type": str(self.get_parameter("trial_type").value),
            "thermal_state": str(self.get_parameter("thermal_state").value),
            "feedback_source": "imu_angular_velocity_integral" if self.get_parameter("trial_type").value == "closed_loop_heading" else None,
            "ground_truth_used_for_control": False,
            "target_heading_rad": float(self.get_parameter("target_heading_rad").value) if self.get_parameter("trial_type").value == "closed_loop_heading" else None,
            "feedback_heading_rad": self.feedback_heading if self.get_parameter("trial_type").value == "closed_loop_heading" else None,
            "runner_complete": complete,
            "error": error,
            **metrics,
        }
        if self.get_parameter("trial_type").value == "closed_loop_heading" and metrics.get("complete"):
            target = float(self.get_parameter("target_heading_rad").value)
            report["ground_truth_heading_error_deg"] = math.degrees(
                abs(normalize_angle(metrics["ground_truth_yaw_delta_rad"] - target))
            )
            report["imu_feedback_heading_error_deg"] = math.degrees(
                abs(normalize_angle(self.feedback_heading - target))
            )
        output = Path(str(self.get_parameter("output_path").value)); output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        csv_path = Path(str(self.get_parameter("csv_path").value)); csv_path.parent.mkdir(parents=True, exist_ok=True)
        if self.rows:
            with csv_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(self.rows[0])); writer.writeheader(); writer.writerows(self.rows)
        self.exit_code = 0 if complete and metrics.get("complete") else 2
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args); node = TransientResponseRunner()
    try: rclpy.spin(node)
    finally:
        code = node.exit_code
        if rclpy.ok(): rclpy.shutdown()
        node.destroy_node()
    raise SystemExit(code)
