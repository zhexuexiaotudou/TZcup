"""Record raw, corrected and EKF covariance evidence without altering raw topics."""

import json
import math
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def covariance_summary(values):
    diagonal = [float(values[index * 6 + index]) for index in range(6)] if len(values) == 36 else [float(values[index * 3 + index]) for index in range(3)]
    finite = all(math.isfinite(value) for value in values)
    return {
        "diagonal": diagonal,
        "all_zero": all(value == 0.0 for value in values),
        "has_zero_diagonal": any(value == 0.0 for value in diagonal),
        "has_negative_diagonal": any(value < 0.0 for value in diagonal),
        "all_finite": finite,
        "unusual": (not finite) or any(value < -1.0 for value in diagonal),
    }


class CovarianceAudit(Node):
    def __init__(self):
        super().__init__("covariance_audit")
        self.declare_parameter("output_path", "/tmp/measurement_covariance_report.json")
        self.declare_parameter("duration_sec", 10.0)
        self.samples = {name: [] for name in ("raw_wheel_odom", "corrected_wheel_odom", "raw_imu", "corrected_imu", "ekf")}
        self.create_subscription(Odometry, "/odom/unfiltered", lambda msg: self.on_odom("raw_wheel_odom", msg), 50)
        self.create_subscription(Odometry, "/measurements/wheel_odom", lambda msg: self.on_odom("corrected_wheel_odom", msg), 50)
        self.create_subscription(Odometry, "/odom", lambda msg: self.on_odom("ekf", msg), 50)
        self.create_subscription(Imu, "/imu/data", lambda msg: self.on_imu("raw_imu", msg), qos_profile_sensor_data)
        self.create_subscription(Imu, "/measurements/imu", lambda msg: self.on_imu("corrected_imu", msg), qos_profile_sensor_data)

    @staticmethod
    def stamp(message):
        return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1.0e-9

    def on_odom(self, name, message):
        if len(self.samples[name]) < 1000:
            self.samples[name].append({
                "stamp": self.stamp(message),
                "frame": message.header.frame_id,
                "child_frame": message.child_frame_id,
                "pose": covariance_summary(message.pose.covariance),
                "twist": covariance_summary(message.twist.covariance),
            })

    def on_imu(self, name, message):
        if len(self.samples[name]) < 1000:
            self.samples[name].append({
                "stamp": self.stamp(message),
                "frame": message.header.frame_id,
                "orientation": covariance_summary(message.orientation_covariance),
                "angular_velocity": covariance_summary(message.angular_velocity_covariance),
                "linear_acceleration": covariance_summary(message.linear_acceleration_covariance),
            })

    def run(self):
        deadline = time.monotonic() + float(self.get_parameter("duration_sec").value)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        topics = {}
        for name, samples in self.samples.items():
            stamps = [sample["stamp"] for sample in samples]
            duration = stamps[-1] - stamps[0] if len(stamps) >= 2 else 0.0
            topics[name] = {
                "sample_count": len(samples),
                "frame": samples[-1]["frame"] if samples else None,
                "child_frame": samples[-1].get("child_frame") if samples else None,
                "first_timestamp_sec": stamps[0] if stamps else None,
                "last_timestamp_sec": stamps[-1] if stamps else None,
                "rate_hz": (len(stamps) - 1) / duration if duration > 0.0 else None,
                "covariance": {key: value for key, value in samples[-1].items() if key not in {"stamp", "frame", "child_frame"}} if samples else None,
            }
        required = ("corrected_wheel_odom", "corrected_imu", "ekf")
        corrected_nonzero = all(
            topics[name]["sample_count"] > 0
            and not any(summary["all_zero"] for summary in topics[name]["covariance"].values())
            for name in required
        )
        report = {
            "schema_version": 1,
            "raw_topics_preserved": topics["raw_wheel_odom"]["sample_count"] > 0 and topics["raw_imu"]["sample_count"] > 0,
            "corrected_topics_present": topics["corrected_wheel_odom"]["sample_count"] > 0 and topics["corrected_imu"]["sample_count"] > 0,
            "corrected_covariance_nonzero": corrected_nonzero,
            "parameter_source": "sanitation_tasks/config/measurement_covariance.yaml",
            "topics": topics,
            "pass": corrected_nonzero,
        }
        output = Path(str(self.get_parameter("output_path").value))
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return report["pass"]


def main(args=None):
    rclpy.init(args=args)
    node = CovarianceAudit()
    try:
        passed = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not passed:
        raise SystemExit(2)
