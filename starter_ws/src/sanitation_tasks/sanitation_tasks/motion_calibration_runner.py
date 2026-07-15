import csv
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String

from .evaluation import normalize_angle, summarize, yaw_from_quaternion


WHEELS = (
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
)


def stamp_sec(message):
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def unwrap(values):
    if not values:
        return []
    output = [values[0]]
    for value in values[1:]:
        output.append(output[-1] + normalize_angle(value - output[-1]))
    return output


def endpoint_delta(rows, x_key, y_key):
    if len(rows) < 2 or rows[0][x_key] is None or rows[-1][x_key] is None:
        return None
    return rows[-1][x_key] - rows[0][x_key], rows[-1][y_key] - rows[0][y_key]


@dataclass(frozen=True)
class Action:
    segment: str
    action: str
    duration: float
    linear: float
    angular: float
    include_in_metrics: bool = True


def build_schedule():
    segments = []

    def add(name, actions):
        segments.extend(
            Action(name, action_name, duration, linear, angular)
            for action_name, duration, linear, angular in actions
        )
        segments.append(Action(name, f"rest_after_{name}", 3.0, 0.0, 0.0, False))

    add("stationary_20s", [("stationary", 20.0, 0.0, 0.0)])
    add("forward_5m_0p20", [("line", 5.0 / 0.20, 0.20, 0.0)])
    add("forward_5m_0p45", [("line", 5.0 / 0.45, 0.45, 0.0)])
    add("reverse_5m_0p20", [("line", 5.0 / 0.20, -0.20, 0.0)])
    add("turn_positive_360_0p25", [("turn", 2.0 * math.pi / 0.25, 0.0, 0.25)])
    add("turn_negative_360_0p25", [("turn", 2.0 * math.pi / 0.25, 0.0, -0.25)])
    add("turn_positive_360_0p60", [("turn", 2.0 * math.pi / 0.60, 0.0, 0.60)])
    add("circle_r1_left", [("circle", 2.0 * math.pi / 0.20, 0.20, 0.20)])
    add("circle_r1_right", [("circle", 2.0 * math.pi / 0.20, 0.20, -0.20)])
    add("circle_r2_left", [("circle", 2.0 * math.pi / 0.15, 0.30, 0.15)])
    add("circle_r2_right", [("circle", 2.0 * math.pi / 0.15, 0.30, -0.15)])
    rectangle = []
    for index, length in enumerate((10.0, 5.0, 10.0, 5.0), start=1):
        rectangle.append((f"side_{index}", length / 0.30, 0.30, 0.0))
        rectangle.append((f"corner_{index}", (math.pi / 2.0) / 0.25, 0.0, 0.25))
    add("rectangle_10x5", rectangle)
    add(
        "figure_eight_r2",
        [
            ("left_loop", 2.0 * math.pi / 0.15, 0.30, 0.15),
            ("right_loop", 2.0 * math.pi / 0.15, 0.30, -0.15),
        ],
    )
    return segments


def build_ablation_schedule():
    """Compact but complete A/B/C/D action set, identical for every candidate."""
    segments = []

    def add(name, actions):
        segments.extend(Action(name, *action) for action in actions)
        segments.append(Action(name, f"rest_after_{name}", 1.0, 0.0, 0.0, False))

    add("stationary_3s", [("stationary", 3.0, 0.0, 0.0)])
    add("ablation_straight", [("line", 5.0, 0.20, 0.0)])
    add("turn_positive_360_0p25", [("turn", 2.0 * math.pi / 0.25, 0.0, 0.25)])
    add("turn_negative_360_0p25", [("turn", 2.0 * math.pi / 0.25, 0.0, -0.25)])
    for rate in (0.35, 0.45, 0.60):
        label = str(rate).replace(".", "p")
        for direction, sign in (("positive", 1.0), ("negative", -1.0)):
            name = f"turn_{direction}_step_{label}"
            add(name, [("turn_step", 5.0, 0.0, sign * rate)])
    add("circle_r1_left", [("circle", 2.0 * math.pi / 0.20, 0.20, 0.20)])
    add("circle_r1_right", [("circle", 2.0 * math.pi / 0.20, 0.20, -0.20)])
    rectangle = []
    for index, length in enumerate((2.0, 1.0, 2.0, 1.0), start=1):
        rectangle.append((f"side_{index}", length / 0.30, 0.30, 0.0))
        rectangle.append((f"corner_{index}", (math.pi / 2.0) / 0.25, 0.0, 0.25))
    add("rectangle_2x1", rectangle)
    add("figure_eight_r1", [
        ("left_loop", 2.0 * math.pi / 0.20, 0.20, 0.20),
        ("right_loop", 2.0 * math.pi / 0.20, 0.20, -0.20),
    ])
    return segments


class MotionCalibrationRunner(Node):
    def __init__(self):
        super().__init__("motion_calibration_runner")
        self.declare_parameter("output_dir", "motion_calibration")
        self.declare_parameter("timeout_margin_sec", 90.0)
        self.declare_parameter("drive_wheel_radius", 0.14)
        self.declare_parameter("drive_wheel_separation", 0.80)
        self.declare_parameter("calibration_label", "baseline")
        self.declare_parameter("schedule_profile", "stage4s")
        self.declare_parameter("random_seed", 0)
        self.cmd_publisher = self.create_publisher(Twist, "/cmd_vel_gate", 20)
        self.marker_publisher = self.create_publisher(
            String, "/calibration/segment_marker", 20
        )
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, 50)
        self.create_subscription(Odometry, "/odom/unfiltered", self.on_raw_odom, 50)
        self.create_subscription(Odometry, "/odom", self.on_ekf, 50)
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 50)
        self.create_subscription(Twist, "/cmd_vel", self.on_output_command, 50)

        self.schedule = (
            build_ablation_schedule()
            if self.get_parameter("schedule_profile").value == "stage4t_ablation"
            else build_schedule()
        )
        self.expected_duration = sum(action.duration for action in self.schedule)
        self.action_index = -1
        self.action_start = None
        self.run_start = None
        self.current_action = None
        self.raw_odom = None
        self.ekf = None
        self.imu = None
        self.joints = None
        self.output_command = Twist()
        self.rows = []
        self.finished = False
        self.exit_code = 2
        self.create_timer(0.02, self.tick)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_raw_odom(self, message):
        self.raw_odom = message

    def on_ekf(self, message):
        self.ekf = message

    def on_imu(self, message):
        self.imu = message

    def on_joints(self, message):
        self.joints = message

    def on_output_command(self, message):
        self.output_command = message

    def on_truth(self, message):
        if self.current_action is None:
            return
        joint_positions = {}
        if self.joints:
            joint_positions = dict(zip(self.joints.name, self.joints.position))
        raw = self.raw_odom
        ekf = self.ekf
        imu = self.imu
        gt_stamp = stamp_sec(message)
        row = {
            "stamp_sec": gt_stamp,
            "segment": self.current_action.segment,
            "action": self.current_action.action,
            "include_in_metrics": self.current_action.include_in_metrics,
            "cmd_requested_linear": self.current_action.linear,
            "cmd_requested_angular": self.current_action.angular,
            "cmd_output_linear": self.output_command.linear.x,
            "cmd_output_angular": self.output_command.angular.z,
            "gt_x": message.pose.pose.position.x,
            "gt_y": message.pose.pose.position.y,
            "gt_yaw": yaw_from_quaternion(message.pose.pose.orientation),
            "gt_linear_x": message.twist.twist.linear.x,
            "gt_angular_z": message.twist.twist.angular.z,
            "raw_x": raw.pose.pose.position.x if raw else None,
            "raw_y": raw.pose.pose.position.y if raw else None,
            "raw_yaw": yaw_from_quaternion(raw.pose.pose.orientation) if raw else None,
            "raw_linear_x": raw.twist.twist.linear.x if raw else None,
            "raw_angular_z": raw.twist.twist.angular.z if raw else None,
            "raw_sync_error_sec": abs(gt_stamp - stamp_sec(raw)) if raw else None,
            "ekf_x": ekf.pose.pose.position.x if ekf else None,
            "ekf_y": ekf.pose.pose.position.y if ekf else None,
            "ekf_yaw": yaw_from_quaternion(ekf.pose.pose.orientation) if ekf else None,
            "ekf_sync_error_sec": abs(gt_stamp - stamp_sec(ekf)) if ekf else None,
            "imu_yaw": yaw_from_quaternion(imu.orientation) if imu else None,
            "imu_yaw_rate": imu.angular_velocity.z if imu else None,
            "imu_sync_error_sec": abs(gt_stamp - stamp_sec(imu)) if imu else None,
        }
        for wheel in WHEELS:
            row[f"joint_{wheel}"] = joint_positions.get(wheel)
        self.rows.append(row)

    def publish_command(self, linear, angular):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.cmd_publisher.publish(message)

    def publish_marker(self, event, action):
        payload = {
            "event": event,
            "segment": action.segment,
            "action": action.action,
            "include_in_metrics": action.include_in_metrics,
            "sim_time_sec": self.now_sec(),
        }
        self.marker_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    def ready(self):
        return all(
            value is not None
            for value in (self.raw_odom, self.ekf, self.imu, self.joints)
        )

    def advance(self):
        if self.current_action is not None:
            self.publish_marker("end", self.current_action)
        self.action_index += 1
        if self.action_index >= len(self.schedule):
            self.publish_command(0.0, 0.0)
            self.finish()
            return
        self.current_action = self.schedule[self.action_index]
        self.action_start = self.now_sec()
        self.publish_marker("start", self.current_action)

    def tick(self):
        if self.finished:
            return
        now = self.now_sec()
        if self.run_start is None:
            self.publish_command(0.0, 0.0)
            if self.ready():
                self.run_start = now
                self.advance()
            return
        if now - self.run_start > self.expected_duration + float(
            self.get_parameter("timeout_margin_sec").value
        ):
            self.exit_code = 3
            self.finish()
            return
        if now - self.action_start >= self.current_action.duration:
            self.advance()
            return
        self.publish_command(self.current_action.linear, self.current_action.angular)

    def finish(self):
        if self.finished:
            return
        self.finished = True
        self.publish_command(0.0, 0.0)
        if self.exit_code == 2:
            self.exit_code = 0
        self.write_outputs()
        rclpy.shutdown()

    def segment_actions(self, segment):
        return [
            action
            for action in self.schedule
            if action.segment == segment and action.include_in_metrics
        ]

    def analyze_segment(self, name, rows):
        rows = [row for row in rows if row["include_in_metrics"]]
        if len(rows) < 2:
            return {"segment": name, "sample_count": len(rows), "complete": False}
        times = [row["stamp_sec"] for row in rows]
        gt_yaw = unwrap([row["gt_yaw"] for row in rows])
        raw_yaw = unwrap([row["raw_yaw"] for row in rows])
        ekf_yaw = unwrap([row["ekf_yaw"] for row in rows])
        expected_distance = sum(
            action.linear * action.duration for action in self.segment_actions(name)
        )
        expected_yaw = sum(
            action.angular * action.duration for action in self.segment_actions(name)
        )
        gt_dx = rows[-1]["gt_x"] - rows[0]["gt_x"]
        gt_dy = rows[-1]["gt_y"] - rows[0]["gt_y"]
        heading = rows[0]["gt_yaw"]
        gt_projected = gt_dx * math.cos(heading) + gt_dy * math.sin(heading)
        raw_delta = endpoint_delta(rows, "raw_x", "raw_y")
        ekf_delta = endpoint_delta(rows, "ekf_x", "ekf_y")
        raw_projected = (
            raw_delta[0] * math.cos(heading) + raw_delta[1] * math.sin(heading)
            if raw_delta else None
        )
        gt_path_length = 0.0
        lateral_velocities = []
        gt_linear_velocities = []
        gt_angular_velocities = []
        imu_integral = 0.0
        requested_command_integral = 0.0
        actual_output_command_integral = 0.0
        ekf_errors = []
        for index in range(1, len(rows)):
            dt = times[index] - times[index - 1]
            if dt <= 0.0 or dt > 0.2:
                continue
            requested_command_integral += 0.5 * (
                rows[index - 1]["cmd_requested_angular"]
                + rows[index]["cmd_requested_angular"]
            ) * dt
            actual_output_command_integral += 0.5 * (
                rows[index - 1]["cmd_output_angular"]
                + rows[index]["cmd_output_angular"]
            ) * dt
            dx = rows[index]["gt_x"] - rows[index - 1]["gt_x"]
            dy = rows[index]["gt_y"] - rows[index - 1]["gt_y"]
            gt_path_length += math.hypot(dx, dy)
            yaw = rows[index - 1]["gt_yaw"]
            vx = (dx * math.cos(yaw) + dy * math.sin(yaw)) / dt
            vy = (-dx * math.sin(yaw) + dy * math.cos(yaw)) / dt
            wz = (gt_yaw[index] - gt_yaw[index - 1]) / dt
            gt_linear_velocities.append(vx)
            lateral_velocities.append(vy)
            gt_angular_velocities.append(wz)
            if rows[index]["imu_yaw_rate"] is not None:
                imu_integral += rows[index]["imu_yaw_rate"] * dt
            if rows[index]["ekf_x"] is not None:
                ekf_errors.append(
                    math.hypot(
                        rows[index]["ekf_x"] - rows[index]["gt_x"],
                        rows[index]["ekf_y"] - rows[index]["gt_y"],
                    )
                )
        gt_yaw_delta = gt_yaw[-1] - gt_yaw[0]
        raw_yaw_delta = raw_yaw[-1] - raw_yaw[0]
        ekf_yaw_delta = ekf_yaw[-1] - ekf_yaw[0]
        circle_radius = (
            gt_path_length / abs(gt_yaw_delta) if abs(gt_yaw_delta) > 0.25 else None
        )
        joint_deltas = {}
        for wheel in WHEELS:
            key = f"joint_{wheel}"
            values = [row[key] for row in rows if row[key] is not None]
            joint_deltas[wheel] = values[-1] - values[0] if len(values) >= 2 else None
        wheel_values = [abs(value) for value in joint_deltas.values() if value is not None]
        wheel_consistency = (
            (max(wheel_values) - min(wheel_values)) / max(wheel_values)
            if wheel_values and max(wheel_values) > 1e-9 else 0.0
        )
        raw_distance_error_pct = (
            abs(raw_projected - gt_projected) / max(abs(gt_projected), 1e-9) * 100.0
            if raw_projected is not None and abs(expected_distance) > 0.1 else None
        )
        body_distance_error_pct = (
            abs(gt_projected - expected_distance) / abs(expected_distance) * 100.0
            if abs(expected_distance) > 0.1 else None
        )
        body_yaw_error_deg = (
            math.degrees(abs(gt_yaw_delta - expected_yaw))
            if abs(expected_yaw) > 0.1 else None
        )
        raw_yaw_error_deg = (
            math.degrees(abs(raw_yaw_delta - gt_yaw_delta))
            if abs(expected_yaw) > 0.1 else None
        )
        imu_yaw_error_deg = (
            math.degrees(abs(imu_integral - gt_yaw_delta))
            if abs(expected_yaw) > 0.1 else None
        )
        gt_closure = math.hypot(gt_dx, gt_dy)
        raw_closure = math.hypot(*raw_delta) if raw_delta else None
        ekf_closure = math.hypot(*ekf_delta) if ekf_delta else None
        ekf_closure_vector_error = (
            math.hypot(ekf_delta[0] - gt_dx, ekf_delta[1] - gt_dy)
            if ekf_delta else None
        )
        return {
            "segment": name,
            "sample_count": len(rows),
            "duration_sec": times[-1] - times[0],
            "complete": True,
            "expected_distance_m": expected_distance,
            "expected_yaw_rad": expected_yaw,
            "requested_command_integral_rad": requested_command_integral,
            "actual_output_command_integral_rad": actual_output_command_integral,
            "gt_projected_distance_m": gt_projected,
            "gt_path_length_m": gt_path_length,
            "gt_yaw_delta_rad": gt_yaw_delta,
            "ground_truth_yaw_delta_rad": gt_yaw_delta,
            "raw_projected_distance_m": raw_projected,
            "raw_yaw_delta_rad": raw_yaw_delta,
            "imu_integrated_yaw_rad": imu_integral,
            "ekf_yaw_delta_rad": ekf_yaw_delta,
            "circle_radius_m": circle_radius,
            "body_distance_error_pct": body_distance_error_pct,
            "body_yaw_error_deg": body_yaw_error_deg,
            "raw_distance_error_pct": raw_distance_error_pct,
            "raw_yaw_error_deg": raw_yaw_error_deg,
            "imu_yaw_error_deg": imu_yaw_error_deg,
            "gt_closure_m": gt_closure,
            "raw_closure_m": raw_closure,
            "ekf_closure_m": ekf_closure,
            "ekf_closure_vector_error_m": ekf_closure_vector_error,
            "gt_linear_velocity": summarize(gt_linear_velocities),
            "gt_angular_velocity": summarize([abs(value) for value in gt_angular_velocities]),
            "lateral_slip_velocity": summarize([abs(value) for value in lateral_velocities]),
            "ekf_xy_error": summarize(ekf_errors),
            "wheel_joint_delta_rad": joint_deltas,
            "four_wheel_rotation_spread_ratio": wheel_consistency,
            "sync_error_sec": {
                "raw_odom": summarize([row["raw_sync_error_sec"] for row in rows if row["raw_sync_error_sec"] is not None]),
                "imu": summarize([row["imu_sync_error_sec"] for row in rows if row["imu_sync_error_sec"] is not None]),
                "ekf": summarize([row["ekf_sync_error_sec"] for row in rows if row["ekf_sync_error_sec"] is not None]),
            },
        }

    def write_outputs(self):
        output_dir = Path(str(self.get_parameter("output_dir").value))
        output_dir.mkdir(parents=True, exist_ok=True)
        trajectory_path = output_dir / "motion_calibration_trajectory.csv"
        if self.rows:
            with trajectory_path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(self.rows[0].keys()))
                writer.writeheader()
                writer.writerows(self.rows)

        segment_names = []
        for action in self.schedule:
            if action.include_in_metrics and action.segment not in segment_names:
                segment_names.append(action.segment)
        results = [
            self.analyze_segment(
                name,
                [row for row in self.rows if row["segment"] == name],
            )
            for name in segment_names
        ]
        by_name = {result["segment"]: result for result in results}
        if self.get_parameter("schedule_profile").value == "stage4t_ablation":
            line_names = ("ablation_straight",)
            turn_names = ("turn_positive_360_0p25", "turn_negative_360_0p25")
            circle_expected = {"circle_r1_left": 1.0, "circle_r1_right": 1.0}
            rectangle_name = "rectangle_2x1"
            figure_name = "figure_eight_r1"
        else:
            line_names = ("forward_5m_0p20", "forward_5m_0p45", "reverse_5m_0p20")
            turn_names = (
                "turn_positive_360_0p25",
                "turn_negative_360_0p25",
                "turn_positive_360_0p60",
            )
            circle_expected = {
                "circle_r1_left": 1.0,
                "circle_r1_right": 1.0,
                "circle_r2_left": 2.0,
                "circle_r2_right": 2.0,
            }
            rectangle_name = "rectangle_10x5"
            figure_name = "figure_eight_r2"
        body_command_tracking_valid = (
            all(by_name[name].get("body_distance_error_pct", math.inf) <= 5.0 for name in line_names)
            and all(by_name[name].get("body_yaw_error_deg", math.inf) <= 18.0 for name in turn_names)
            and all(
                abs(by_name[name].get("circle_radius_m", math.inf) - radius) / radius <= 0.15
                for name, radius in circle_expected.items()
            )
        )
        raw_odom_valid = (
            all(by_name[name].get("raw_distance_error_pct", math.inf) <= 1.0 for name in line_names)
            and all(by_name[name].get("raw_yaw_error_deg", math.inf) <= 2.0 for name in turn_names[:2])
        )
        imu_valid = all(
            by_name[name].get("imu_yaw_error_deg", math.inf) <= 1.0
            for name in turn_names[:2]
        )
        rectangle_error = by_name[rectangle_name].get(
            "ekf_closure_vector_error_m", math.inf
        )
        figure_error = by_name[figure_name].get(
            "ekf_closure_vector_error_m", math.inf
        )
        ekf_valid = rectangle_error <= 0.10 and figure_error <= 0.15
        layer_checks = {
            "ground_truth_valid": True,
            "body_command_tracking_valid": body_command_tracking_valid,
            "raw_odom_valid": raw_odom_valid,
            "imu_valid": imu_valid,
            "ekf_valid": ekf_valid,
        }
        layer_order = (
            ("layer_0_ground_truth", "ground_truth_valid"),
            ("layer_1_body_command_tracking", "body_command_tracking_valid"),
            ("layer_2_raw_wheel_odom", "raw_odom_valid"),
            ("layer_3_imu", "imu_valid"),
            ("layer_4_ekf", "ekf_valid"),
        )
        first_failed = next(
            (layer for layer, key in layer_order if not layer_checks[key]), None
        )
        recommendations = {
            "layer_0_ground_truth": "Stop. Repair ground-truth identity before any dynamics work.",
            "layer_1_body_command_tracking": "Fit drive radius/separation first; scan contact friction/slip only if command tracking remains invalid.",
            "layer_2_raw_wheel_odom": "Fit drive_wheel_radius then drive_wheel_separation against ground truth.",
            "layer_3_imu": "Audit IMU axis, covariance, bias and integration before EKF tuning.",
            "layer_4_ekf": "Run EKF A/B/C/D ablation with the same calibrated motion data.",
            None: "Proceed to parameter validation and EKF ablation.",
        }
        fault_report = {
            "schema_version": 1,
            "first_failed_layer": first_failed,
            **layer_checks,
            "recommended_next_action": recommendations[first_failed],
        }
        (output_dir / "fault_isolation_report.json").write_text(
            json.dumps(fault_report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        report = {
            "schema_version": 1,
            "baseline_commit": "413b6ebfb16d40e00a820c1dcf8cb5c87c90e566",
            "random_seed": int(self.get_parameter("random_seed").value),
            "schedule_profile": str(self.get_parameter("schedule_profile").value),
            "use_sim_time": bool(self.get_parameter("use_sim_time").value),
            "calibration_label": self.get_parameter("calibration_label").value,
            "vehicle_dynamics": {
                "drive_wheel_radius": self.get_parameter("drive_wheel_radius").value,
                "drive_wheel_separation": self.get_parameter("drive_wheel_separation").value,
            },
            "segment_count": len(results),
            "all_segments_complete": len(results) == len(segment_names) and all(result.get("complete") for result in results),
            "expected_duration_sec": self.expected_duration,
            "sample_count": len(self.rows),
            "thresholds": {
                "body_line_tracking_error_pct": 5.0,
                "body_turn_tracking_error_deg": 18.0,
                "body_circle_radius_error_pct": 15.0,
                "raw_5m_distance_error_pct": 1.0,
                "raw_low_speed_360_yaw_error_deg": 2.0,
                "imu_low_speed_360_yaw_error_deg": 1.0,
                "rectangle_ekf_closure_error_m": 0.10,
                "figure_eight_ekf_closure_error_m": 0.15,
            },
            "segments": results,
            "fault_isolation": fault_report,
            "realistic_motion_calibration_pass": all(layer_checks.values()),
            "experiment_completed": self.exit_code == 0,
        }
        (output_dir / "motion_calibration_report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

        flat_fields = (
            "segment",
            "sample_count",
            "duration_sec",
            "expected_distance_m",
            "expected_yaw_rad",
            "requested_command_integral_rad",
            "actual_output_command_integral_rad",
            "gt_projected_distance_m",
            "gt_path_length_m",
            "gt_yaw_delta_rad",
            "ground_truth_yaw_delta_rad",
            "raw_projected_distance_m",
            "raw_yaw_delta_rad",
            "imu_integrated_yaw_rad",
            "ekf_yaw_delta_rad",
            "circle_radius_m",
            "body_distance_error_pct",
            "body_yaw_error_deg",
            "raw_distance_error_pct",
            "raw_yaw_error_deg",
            "imu_yaw_error_deg",
            "gt_closure_m",
            "raw_closure_m",
            "ekf_closure_m",
            "ekf_closure_vector_error_m",
            "four_wheel_rotation_spread_ratio",
        )
        with (output_dir / "motion_calibration_segments.csv").open(
            "w", encoding="utf-8", newline=""
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=flat_fields)
            writer.writeheader()
            for result in results:
                writer.writerow({key: result.get(key) for key in flat_fields})

        if self.rows:
            fig, axes = plt.subplots(2, 2, figsize=(13, 10), constrained_layout=True)
            axes[0, 0].plot([row["gt_x"] for row in self.rows], [row["gt_y"] for row in self.rows], label="Gazebo truth", linewidth=2)
            axes[0, 0].plot([row["raw_x"] for row in self.rows], [row["raw_y"] for row in self.rows], label="raw wheel odom", alpha=0.8)
            axes[0, 0].plot([row["ekf_x"] for row in self.rows], [row["ekf_y"] for row in self.rows], label="EKF", alpha=0.8)
            axes[0, 0].set_title("Open-loop calibration trajectory")
            axes[0, 0].set_xlabel("x (m)")
            axes[0, 0].set_ylabel("y (m)")
            axes[0, 0].axis("equal")
            axes[0, 0].legend()
            times = [row["stamp_sec"] - self.rows[0]["stamp_sec"] for row in self.rows]
            xy_raw = [math.hypot(row["raw_x"] - row["gt_x"], row["raw_y"] - row["gt_y"]) for row in self.rows]
            xy_ekf = [math.hypot(row["ekf_x"] - row["gt_x"], row["ekf_y"] - row["gt_y"]) for row in self.rows]
            axes[0, 1].plot(times, xy_raw, label="raw odom")
            axes[0, 1].plot(times, xy_ekf, label="EKF")
            axes[0, 1].set_title("XY error against truth")
            axes[0, 1].set_xlabel("simulation time (s)")
            axes[0, 1].set_ylabel("error (m)")
            axes[0, 1].legend()
            axes[1, 0].plot(times, [row["cmd_requested_linear"] for row in self.rows], label="requested vx")
            axes[1, 0].plot(times, [row["gt_linear_x"] for row in self.rows], label="truth vx", alpha=0.8)
            axes[1, 0].set_title("Linear command tracking")
            axes[1, 0].set_xlabel("simulation time (s)")
            axes[1, 0].set_ylabel("m/s")
            axes[1, 0].legend()
            axes[1, 1].plot(times, [row["cmd_requested_angular"] for row in self.rows], label="requested wz")
            axes[1, 1].plot(times, [row["gt_angular_z"] for row in self.rows], label="truth wz", alpha=0.8)
            axes[1, 1].plot(times, [row["imu_yaw_rate"] for row in self.rows], label="IMU wz", alpha=0.6)
            axes[1, 1].set_title("Angular command tracking")
            axes[1, 1].set_xlabel("simulation time (s)")
            axes[1, 1].set_ylabel("rad/s")
            axes[1, 1].legend()
            fig.savefig(output_dir / "motion_calibration_trajectory.png", dpi=160)
            plt.close(fig)


def main(args=None):
    rclpy.init(args=args)
    node = MotionCalibrationRunner()
    try:
        rclpy.spin(node)
    finally:
        exit_code = node.exit_code
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()
    raise SystemExit(exit_code)
