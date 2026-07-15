import csv
import json
import math
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from .evaluation import (
    assert_comparable_frames,
    normalize_angle,
    summarize,
    synchronize_samples,
    yaw_from_quaternion,
)


class LocalizationEvaluator(Node):
    def __init__(self):
        super().__init__("localization_evaluator")
        self.declare_parameter("output_path", "/tmp/localization_report.json")
        self.declare_parameter("csv_path", "/tmp/localization_trajectory.csv")
        self.declare_parameter("plot_path", "/tmp/localization_error.png")
        self.declare_parameter("duration_sec", 120.0)
        self.declare_parameter("sync_tolerance_sec", 0.05)
        self.declare_parameter("rmse_threshold_m", 0.05)
        self.estimates = []
        self.truths = []
        self.particle_spreads = []
        self.tf_stamps = []
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._estimate, 20)
        self.create_subscription(Odometry, "/ground_truth/odom", self._truth, 20)
        self.create_subscription(PoseArray, "/particle_cloud", self._particles, 10)
        self.create_subscription(TFMessage, "/tf", self._tf, 50)

    def _estimate(self, message):
        assert_comparable_frames(message.header.frame_id, "map_gt", {("map", "map_gt")})
        pose = message.pose.pose
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.estimates.append((stamp, pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)))

    def _truth(self, message):
        assert_comparable_frames("map", message.header.frame_id, {("map", "map_gt")})
        pose = message.pose.pose
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.truths.append((stamp, pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)))

    def _particles(self, message):
        if not message.poses:
            return
        mean_x = sum(pose.position.x for pose in message.poses) / len(message.poses)
        mean_y = sum(pose.position.y for pose in message.poses) / len(message.poses)
        spread = math.sqrt(sum((pose.position.x - mean_x) ** 2 + (pose.position.y - mean_y) ** 2 for pose in message.poses) / len(message.poses))
        self.particle_spreads.append((len(message.poses), spread))

    def _tf(self, message):
        for transform in message.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                self.tf_stamps.append(transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9)

    def run(self):
        started = time.monotonic()
        duration = self.get_parameter("duration_sec").value
        while rclpy.ok() and time.monotonic() - started < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        pairs, dropped = synchronize_samples(
            self.estimates, self.truths, self.get_parameter("sync_tolerance_sec").value
        )
        rows, xy_errors, yaw_errors, sync_errors = [], [], [], []
        for estimate, truth, sync_error in pairs:
            xy_error = math.hypot(estimate[1] - truth[1], estimate[2] - truth[2])
            yaw_error = abs(normalize_angle(estimate[3] - truth[3]))
            xy_errors.append(xy_error)
            yaw_errors.append(yaw_error)
            sync_errors.append(sync_error)
            rows.append((*estimate, *truth[1:], xy_error, yaw_error, sync_error))
        with open(self.get_parameter("csv_path").value, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["stamp_sec", "estimate_x_m", "estimate_y_m", "estimate_yaw_rad", "truth_x_m", "truth_y_m", "truth_yaw_rad", "xy_error_m", "yaw_error_rad", "sync_error_sec"])
            writer.writerows(rows)
        figure, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
        if rows:
            axes[0].plot([row[1] for row in rows], [row[2] for row in rows], label="AMCL map")
            axes[0].plot([row[4] for row in rows], [row[5] for row in rows], label="Gazebo map_gt")
            axes[1].plot([row[0] - rows[0][0] for row in rows], xy_errors, label="XY error")
        axes[0].set_aspect("equal"); axes[0].set_title("Same-frame trajectories"); axes[0].legend()
        axes[1].axhline(self.get_parameter("rmse_threshold_m").value, color="red", linestyle="--", label="0.05 m gate")
        axes[1].set_title("Synchronized localization error"); axes[1].set_xlabel("simulation time (s)"); axes[1].set_ylabel("m"); axes[1].legend()
        figure.savefig(self.get_parameter("plot_path").value, dpi=160)
        plt.close(figure)
        xy_summary = summarize(xy_errors)
        tf_gaps = [current - previous for previous, current in zip(self.tf_stamps, self.tf_stamps[1:]) if current >= previous]
        report = {
            "schema_version": 1,
            "estimate_frame": "map",
            "truth_frame": "map_gt",
            "alignment": "world_to_map translation (+8, 0), zero yaw; map and map_gt axes explicitly coincident",
            "sample_count": len(pairs),
            "estimate_sample_count": len(self.estimates),
            "truth_sample_count": len(self.truths),
            "dropped_estimate_count": dropped,
            "sync_error_sec": summarize(sync_errors),
            "xy_error_m": xy_summary,
            "yaw_error_rad": summarize(yaw_errors),
            "particle_filter": {
                "update_count": len(self.particle_spreads),
                "particle_count_min": min((item[0] for item in self.particle_spreads), default=None),
                "particle_count_max": max((item[0] for item in self.particle_spreads), default=None),
                "spread_m": summarize([item[1] for item in self.particle_spreads]),
                "degenerate_update_count": sum(item[1] < 1.0e-4 for item in self.particle_spreads),
            },
            "tf_continuity": {
                "map_to_odom_sample_count": len(self.tf_stamps),
                "gap_sec": summarize(tf_gaps),
                "continuous": bool(self.tf_stamps and (max(tf_gaps, default=0.0) <= 0.5)),
            },
            "competition_localization_pass": bool(len(pairs) >= 20 and xy_summary["rmse"] is not None and xy_summary["rmse"] <= self.get_parameter("rmse_threshold_m").value),
        }
        with open(self.get_parameter("output_path").value, "w", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2); stream.write("\n")
        return report["competition_localization_pass"]


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationEvaluator()
    try:
        success = node.run()
    finally:
        node.destroy_node(); rclpy.shutdown()
    if not success:
        raise SystemExit(2)
