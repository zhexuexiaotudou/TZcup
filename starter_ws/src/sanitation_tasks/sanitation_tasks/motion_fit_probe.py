import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .evaluation import normalize_angle, yaw_from_quaternion


def unwrap(values):
    if not values:
        return []
    output = [values[0]]
    for value in values[1:]:
        output.append(output[-1] + normalize_angle(value - output[-1]))
    return output


class MotionFitProbe(Node):
    def __init__(self):
        super().__init__("motion_fit_probe")
        self.declare_parameter("mode", "radius")
        self.declare_parameter("output_path", "motion_fit_probe.json")
        self.declare_parameter("drive_wheel_radius", 0.14)
        self.declare_parameter("drive_wheel_separation", 0.80)
        self.declare_parameter("wheel_mu_longitudinal", 1.0)
        self.declare_parameter("wheel_mu_lateral", 1.0)
        self.declare_parameter("slip_compliance_longitudinal", 0.0)
        self.declare_parameter("slip_compliance_lateral", 0.0)
        self.declare_parameter("enable_wheel_slip", False)
        self.mode = str(self.get_parameter("mode").value)
        if self.mode == "radius":
            self.actions = [
                ("stationary", 3.0, 0.0, 0.0),
                ("forward_5m", 25.0, 0.20, 0.0),
                ("rest", 3.0, 0.0, 0.0),
            ]
        elif self.mode == "separation":
            turn_duration = 2.0 * math.pi / 0.25
            self.actions = [
                ("stationary", 3.0, 0.0, 0.0),
                ("positive_360", turn_duration, 0.0, 0.25),
                ("rest_positive", 3.0, 0.0, 0.0),
                ("negative_360", turn_duration, 0.0, -0.25),
                ("rest_negative", 3.0, 0.0, 0.0),
            ]
        elif self.mode == "friction":
            self.actions = [
                ("stationary", 3.0, 0.0, 0.0),
                ("positive_360_high_speed", 2.0 * math.pi / 0.60, 0.0, 0.60),
                ("rest", 3.0, 0.0, 0.0),
            ]
        else:
            raise ValueError(f"unsupported fit mode: {self.mode}")
        self.publisher = self.create_publisher(Twist, "/cmd_vel_gate", 20)
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, 50)
        self.create_subscription(Odometry, "/odom/unfiltered", self.on_raw, 50)
        self.raw = None
        self.rows = []
        self.action_index = -1
        self.action_start = None
        self.current_action = None
        self.finished = False
        self.exit_code = 2
        self.create_timer(0.02, self.tick)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_raw(self, message):
        self.raw = message

    def on_truth(self, message):
        if self.current_action is None or self.raw is None:
            return
        self.rows.append(
            {
                "action": self.current_action[0],
                "gt_x": message.pose.pose.position.x,
                "gt_y": message.pose.pose.position.y,
                "gt_yaw": yaw_from_quaternion(message.pose.pose.orientation),
                "raw_x": self.raw.pose.pose.position.x,
                "raw_y": self.raw.pose.pose.position.y,
                "raw_yaw": yaw_from_quaternion(self.raw.pose.pose.orientation),
            }
        )

    def command(self, linear, angular):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.publisher.publish(message)

    def advance(self):
        self.action_index += 1
        if self.action_index >= len(self.actions):
            self.get_logger().info("fit probe schedule complete")
            self.finish()
            return
        self.current_action = self.actions[self.action_index]
        self.action_start = self.now_sec()
        self.get_logger().info(
            f"fit action start: {self.current_action[0]} "
            f"duration={self.current_action[1]:.3f}s sim_time={self.action_start:.3f}"
        )

    def tick(self):
        if self.finished:
            return
        if self.current_action is None:
            self.command(0.0, 0.0)
            if self.raw is not None and self.rows == []:
                self.advance()
            return
        if self.now_sec() - self.action_start >= self.current_action[1]:
            self.command(0.0, 0.0)
            self.advance()
            return
        self.command(self.current_action[2], self.current_action[3])

    def subset(self, action):
        return [row for row in self.rows if row["action"] == action]

    def turn_metrics(self, action, expected):
        rows = self.subset(action)
        gt = unwrap([row["gt_yaw"] for row in rows])
        raw = unwrap([row["raw_yaw"] for row in rows])
        gt_delta = gt[-1] - gt[0]
        raw_delta = raw[-1] - raw[0]
        return {
            "expected_yaw_rad": expected,
            "gt_yaw_rad": gt_delta,
            "raw_yaw_rad": raw_delta,
            "body_yaw_error_deg": math.degrees(abs(gt_delta - expected)),
            "raw_vs_truth_error_deg": math.degrees(abs(raw_delta - gt_delta)),
        }

    def finish(self):
        self.finished = True
        self.command(0.0, 0.0)
        report = {
            "schema_version": 1,
            "mode": self.mode,
            "drive_wheel_radius": float(self.get_parameter("drive_wheel_radius").value),
            "drive_wheel_separation": float(self.get_parameter("drive_wheel_separation").value),
            "sample_count": len(self.rows),
            "contact_parameters": {
                "wheel_mu_longitudinal": float(self.get_parameter("wheel_mu_longitudinal").value),
                "wheel_mu_lateral": float(self.get_parameter("wheel_mu_lateral").value),
                "slip_compliance_longitudinal": float(self.get_parameter("slip_compliance_longitudinal").value),
                "slip_compliance_lateral": float(self.get_parameter("slip_compliance_lateral").value),
                "enable_wheel_slip": bool(self.get_parameter("enable_wheel_slip").value),
            },
            "complete": False,
        }
        if self.mode == "radius":
            rows = self.subset("forward_5m")
            if len(rows) >= 2:
                heading = rows[0]["gt_yaw"]
                gt_dx = rows[-1]["gt_x"] - rows[0]["gt_x"]
                gt_dy = rows[-1]["gt_y"] - rows[0]["gt_y"]
                raw_dx = rows[-1]["raw_x"] - rows[0]["raw_x"]
                raw_dy = rows[-1]["raw_y"] - rows[0]["raw_y"]
                gt_distance = gt_dx * math.cos(heading) + gt_dy * math.sin(heading)
                raw_distance = raw_dx * math.cos(heading) + raw_dy * math.sin(heading)
                report.update(
                    {
                        "expected_distance_m": 5.0,
                        "gt_distance_m": gt_distance,
                        "raw_distance_m": raw_distance,
                        "body_distance_error_pct": abs(gt_distance - 5.0) / 5.0 * 100.0,
                        "raw_vs_truth_error_pct": abs(raw_distance - gt_distance) / max(abs(gt_distance), 1e-9) * 100.0,
                        "complete": True,
                    }
                )
        elif self.mode == "separation":
            positive = self.turn_metrics("positive_360", 2.0 * math.pi)
            negative = self.turn_metrics("negative_360", -2.0 * math.pi)
            report.update(
                {
                    "positive": positive,
                    "negative": negative,
                    "mean_body_yaw_error_deg": (
                        positive["body_yaw_error_deg"] + negative["body_yaw_error_deg"]
                    ) / 2.0,
                    "mean_raw_vs_truth_error_deg": (
                        positive["raw_vs_truth_error_deg"] + negative["raw_vs_truth_error_deg"]
                    ) / 2.0,
                    "left_right_asymmetry_deg": math.degrees(
                        abs(abs(positive["gt_yaw_rad"]) - abs(negative["gt_yaw_rad"]))
                    ),
                    "complete": True,
                }
            )
        else:
            high_speed = self.turn_metrics(
                "positive_360_high_speed", 2.0 * math.pi
            )
            report.update(
                {
                    "high_speed_positive": high_speed,
                    "objective_body_yaw_error_deg": high_speed["body_yaw_error_deg"],
                    "body_tracking_gate_deg": 18.0,
                    "body_tracking_pass": high_speed["body_yaw_error_deg"] <= 18.0,
                    "complete": True,
                }
            )
        output = Path(str(self.get_parameter("output_path").value))
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        self.exit_code = 0 if report["complete"] else 1
        self.get_logger().info(
            f"fit report written: {output}; complete={report['complete']}"
        )
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MotionFitProbe()
    try:
        rclpy.spin(node)
    finally:
        exit_code = node.exit_code
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()
    raise SystemExit(exit_code)
