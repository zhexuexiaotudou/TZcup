import csv
import json
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.msg import ParticleCloud
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

from .evaluation import (
    assert_comparable_frames,
    summarize,
    synchronize_samples,
    yaw_from_quaternion,
)
from .localization_metrics import (
    apply_se2,
    load_map_calibration,
    PARTICLE_CLOUD_TYPE,
    particle_statistics,
    particle_topic_type_pass,
    pose_error,
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
        self.declare_parameter("map_frame_calibration", "")
        self.declare_parameter("localization_backend", "amcl")
        self.declare_parameter("estimate_topic", "/amcl_pose")
        self.declare_parameter("require_particle_instrumentation", True)
        calibration_path = str(self.get_parameter("map_frame_calibration").value)
        if not calibration_path:
            raise ValueError("map_frame_calibration is required for Stage4U evaluation")
        self.calibration = load_map_calibration(calibration_path)
        self.estimates = []
        self.truths = []
        self.particle_updates = []
        self.tf_stamps = []
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("estimate_topic").value),
            self._estimate,
            20,
        )
        self.create_subscription(Odometry, "/ground_truth/odom", self._truth, 20)
        particle_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            ParticleCloud, "/particle_cloud", self._particles, particle_qos
        )
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
        particles = [
            (
                particle.pose.position.x,
                particle.pose.position.y,
                yaw_from_quaternion(particle.pose.orientation),
                particle.weight,
            )
            for particle in message.particles
        ]
        self.particle_updates.append(particle_statistics(particles))

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
        rows = []
        map_xy_errors, map_yaw_errors = [], []
        absolute_xy_errors, absolute_yaw_errors = [], []
        calibrated_absolute_xy_errors, calibrated_absolute_yaw_errors = [], []
        sync_errors = []
        map_to_gt = self.calibration["T_map_gt_map"]
        gt_to_map = self.calibration["T_map_map_gt"]
        for estimate, truth, sync_error in pairs:
            estimate_map = (estimate[1], estimate[2], estimate[3])
            truth_map_gt = (truth[1], truth[2], truth[3])
            truth_map = apply_se2(gt_to_map, truth_map_gt)
            estimate_map_gt = apply_se2(map_to_gt, estimate_map)
            map_error = pose_error(estimate_map, truth_map)
            # Deliberately uncalibrated: this is the historical direct map/map_gt
            # subtraction, retained only as a separately labelled diagnostic.
            absolute_error = pose_error(estimate_map, truth_map_gt)
            calibrated_absolute_error = pose_error(estimate_map_gt, truth_map_gt)
            map_xy_errors.append(map_error["xy_m"])
            map_yaw_errors.append(map_error["yaw_rad"])
            absolute_xy_errors.append(absolute_error["xy_m"])
            absolute_yaw_errors.append(absolute_error["yaw_rad"])
            calibrated_absolute_xy_errors.append(calibrated_absolute_error["xy_m"])
            calibrated_absolute_yaw_errors.append(calibrated_absolute_error["yaw_rad"])
            sync_errors.append(sync_error)
            rows.append(
                (
                    *estimate,
                    *truth_map,
                    *truth_map_gt,
                    map_error["xy_m"],
                    map_error["yaw_rad"],
                    absolute_error["xy_m"],
                    absolute_error["yaw_rad"],
                    sync_error,
                )
            )
        with open(self.get_parameter("csv_path").value, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow([
                "stamp_sec", "estimate_map_x_m", "estimate_map_y_m", "estimate_map_yaw_rad",
                "truth_map_x_m", "truth_map_y_m", "truth_map_yaw_rad",
                "truth_map_gt_x_m", "truth_map_gt_y_m", "truth_map_gt_yaw_rad",
                "map_relative_xy_error_m", "map_relative_yaw_error_rad",
                "uncalibrated_absolute_xy_error_m", "uncalibrated_absolute_yaw_error_rad",
                "sync_error_sec",
            ])
            writer.writerows(rows)
        figure, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
        if rows:
            axes[0].plot([row[1] for row in rows], [row[2] for row in rows], label="AMCL map")
            axes[0].plot([row[4] for row in rows], [row[5] for row in rows], label="calibrated truth in map")
            axes[1].plot([row[0] - rows[0][0] for row in rows], map_xy_errors, label="map-relative XY error")
        axes[0].set_aspect("equal"); axes[0].set_title("Frozen-calibration trajectories"); axes[0].legend()
        axes[1].axhline(self.get_parameter("rmse_threshold_m").value, color="red", linestyle="--", label="0.05 m gate")
        axes[1].set_title("Synchronized localization error"); axes[1].set_xlabel("simulation time (s)"); axes[1].set_ylabel("m"); axes[1].legend()
        figure.savefig(self.get_parameter("plot_path").value, dpi=160)
        plt.close(figure)
        map_xy_summary = summarize(map_xy_errors)
        tf_gaps = [current - previous for previous, current in zip(self.tf_stamps, self.tf_stamps[1:]) if current >= previous]
        valid_particles = [item for item in self.particle_updates if item.get("valid")]
        publisher_info = self.get_publishers_info_by_topic("/particle_cloud")
        observed_types = sorted({item.topic_type for item in publisher_info})
        topic_type_pass = particle_topic_type_pass(observed_types) or bool(valid_particles)
        particle_required = bool(
            self.get_parameter("require_particle_instrumentation").value
        )
        particle_instrumentation_pass = bool(valid_particles and topic_type_pass)
        calibration_fit = self.calibration.get("fit_residual", {})
        report = {
            "schema_version": 2,
            "estimate_frame": "map",
            "truth_frame": "map_gt",
            "localization_backend": str(self.get_parameter("localization_backend").value),
            "estimate_topic": str(self.get_parameter("estimate_topic").value),
            "map_calibration": self.calibration,
            "sample_count": len(pairs),
            "estimate_sample_count": len(self.estimates),
            "truth_sample_count": len(self.truths),
            "dropped_estimate_count": dropped,
            "sync_error_sec": summarize(sync_errors),
            "map_relative_localization_error": {
                "xy_m": map_xy_summary,
                "yaw_rad": summarize(map_yaw_errors),
                "definition": "AMCL(map) minus truth transformed into map with frozen T_map_map_gt",
            },
            "map_georeferencing_error": {
                "fit_residual": calibration_fit,
                "definition": "one-time fixed-anchor map calibration residual; never fit per trial",
            },
            "absolute_world_error": {
                "xy_m": summarize(absolute_xy_errors),
                "yaw_rad": summarize(absolute_yaw_errors),
                "calibrated": False,
                "definition": "historical direct AMCL(map) minus canonical map_gt diagnostic",
            },
            "calibrated_absolute_world_error": {
                "xy_m": summarize(calibrated_absolute_xy_errors),
                "yaw_rad": summarize(calibrated_absolute_yaw_errors),
                "definition": "T_map_gt_map(AMCL map pose) minus canonical map_gt truth",
            },
            # Compatibility aliases now explicitly refer to map-relative error.
            "xy_error_m": map_xy_summary,
            "yaw_error_rad": summarize(map_yaw_errors),
            "particle_filter": {
                "expected_topic_type": PARTICLE_CLOUD_TYPE,
                "observed_topic_types": observed_types,
                "topic_type_validation_pass": topic_type_pass,
                "subscription_qos": {"history": "keep_last", "depth": 10, "reliability": "best_effort", "durability": "volatile"},
                "publisher_qos": [
                    {
                        "topic_type": item.topic_type,
                        "history": str(item.qos_profile.history),
                        "depth": item.qos_profile.depth,
                        "reliability": str(item.qos_profile.reliability),
                        "durability": str(item.qos_profile.durability),
                    }
                    for item in publisher_info
                ],
                "update_count": len(self.particle_updates),
                "valid_update_count": len(valid_particles),
                "invalid_update_count": len(self.particle_updates) - len(valid_particles),
                "particle_count_min": min((item["count"] for item in valid_particles), default=None),
                "particle_count_max": max((item["count"] for item in valid_particles), default=None),
                "spread_m": summarize([item["spread_m"] for item in valid_particles]),
                "effective_sample_size": summarize([item["effective_sample_size"] for item in valid_particles]),
                "effective_sample_ratio": summarize([item["effective_sample_ratio"] for item in valid_particles]),
                "max_normalized_weight": summarize([item["max_normalized_weight"] for item in valid_particles]),
                "normalized_entropy": summarize([item["normalized_entropy"] for item in valid_particles]),
                "weighted_mean_last": valid_particles[-1]["weighted_mean"] if valid_particles else None,
                "weighted_covariance_xyyaw_last": valid_particles[-1]["weighted_covariance_xyyaw"] if valid_particles else None,
                "degenerate_update_count": sum(bool(item.get("degenerate", True)) for item in self.particle_updates),
                "invalid_reasons": [item.get("reason") for item in self.particle_updates if not item.get("valid")],
                "particle_instrumentation_pass": particle_instrumentation_pass,
                "particle_instrumentation_required": particle_required,
                "particle_instrumentation_applicable": particle_required,
            },
            "tf_continuity": {
                "map_to_odom_sample_count": len(self.tf_stamps),
                "gap_sec": summarize(tf_gaps),
                "continuous": bool(self.tf_stamps and (max(tf_gaps, default=0.0) <= 0.5)),
            },
            "competition_localization_pass": bool(
                len(pairs) >= 20
                and map_xy_summary["rmse"] is not None
                and map_xy_summary["rmse"] <= self.get_parameter("rmse_threshold_m").value
                and (particle_instrumentation_pass or not particle_required)
            ),
        }
        with open(self.get_parameter("output_path").value, "w", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2); stream.write("\n")
        return report["competition_localization_pass"]


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationEvaluator()
    try:
        success = node.run()
    except ExternalShutdownException:
        success = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not success:
        raise SystemExit(2)
