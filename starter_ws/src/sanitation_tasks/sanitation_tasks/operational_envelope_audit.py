"""Audit requested and final commands against one named operational profile."""

import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def command_violation(linear, angular, max_linear, max_angular, tolerance=1.0e-6):
    return abs(linear) > max_linear + tolerance or abs(angular) > max_angular + tolerance


class OperationalEnvelopeAudit(Node):
    def __init__(self):
        super().__init__("operational_envelope_audit")
        self.declare_parameter("output_path", "/tmp/operational_envelope_report.json")
        self.declare_parameter("profile_name", "localization_coverage")
        self.declare_parameter("max_linear_velocity", 0.45)
        self.declare_parameter("max_angular_velocity", 0.35)
        self.declare_parameter("duration_sec", 15.0)
        self.requested = []
        self.actual = []
        self.create_subscription(Twist, "/cmd_vel_gate", lambda message: self.record(self.requested, message), 50)
        self.create_subscription(Twist, "/cmd_vel", lambda message: self.record(self.actual, message), 50)

    def record(self, target, message):
        target.append((self.now_sec(), float(message.linear.x), float(message.angular.z)))

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def run(self):
        deadline = time.monotonic() + float(self.get_parameter("duration_sec").value)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        max_linear = float(self.get_parameter("max_linear_velocity").value)
        max_angular = float(self.get_parameter("max_angular_velocity").value)
        requested_violations = sum(command_violation(linear, angular, max_linear, max_angular) for _stamp, linear, angular in self.requested)
        actual_violations = sum(command_violation(linear, angular, max_linear, max_angular) for _stamp, linear, angular in self.actual)
        report = {
            "schema_version": 1,
            "profile_name": str(self.get_parameter("profile_name").value),
            "max_linear_velocity": max_linear,
            "max_angular_velocity": max_angular,
            "requested_sample_count": len(self.requested),
            "actual_sample_count": len(self.actual),
            "requested_cmd_limit_violations": requested_violations,
            "actual_cmd_limit_violations": actual_violations,
            "observed_actual_max_linear_velocity": max((abs(row[1]) for row in self.actual), default=None),
            "observed_actual_max_angular_velocity": max((abs(row[2]) for row in self.actual), default=None),
            "profile_pass": bool(self.actual and actual_violations == 0),
        }
        output = Path(str(self.get_parameter("output_path").value)); output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return report["profile_pass"]


def main(args=None):
    rclpy.init(args=args); node = OperationalEnvelopeAudit()
    try: passed = node.run()
    finally: node.destroy_node(); rclpy.shutdown()
    if not passed: raise SystemExit(2)
