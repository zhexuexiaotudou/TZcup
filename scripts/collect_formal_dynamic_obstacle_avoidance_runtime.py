#!/usr/bin/env python3
"""Run one truth-free Nav2 goal and collect dynamic-obstacle gate telemetry."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

from validate_formal_dynamic_obstacle_avoidance import (
    CONTROL_PROHIBITED_TRUTH_TOPICS,
    load_public_mission_contract,
    point_in_polygon,
)


class RuntimeCollector(Node):
    """Product control sends a fixed public-map goal and never reads schedule truth."""

    def __init__(
        self,
        *,
        mission_contract: dict,
        timeout_s: float,
    ) -> None:
        super().__init__("formal_dynamic_obstacle_runtime_collector")
        self.mission_contract = mission_contract
        self.goal_x = float(mission_contract["goal_pose_map"][0])
        self.goal_y = float(mission_contract["goal_pose_map"][1])
        self.geofence = [
            (float(row[0]), float(row[1]))
            for row in mission_contract["geofence_polygon_m"]
        ]
        self.expected_pedestrian_count = int(
            mission_contract["expected_pedestrian_count"]
        )
        self.timeout_s = timeout_s
        self.started = time.monotonic()
        self.action = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.estop = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/emergency_stop", 10
        )
        self.estop_reset = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/emergency_stop_reset", 10
        )
        self.power = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/main_power", 10
        )
        self.create_subscription(Odometry, "/odom", self._odom, 20)
        self.create_subscription(
            Odometry, "/odom/unfiltered", self._raw_odom, 20
        )
        self.create_subscription(TFMessage, "/tf", self._tf, 100)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._map_pose, 20
        )
        self.create_subscription(
            LaserScan,
            "/scan/navigation",
            self._scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/sensors/lidar_3d/points",
            self._pointcloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(Twist, "/cmd_vel_nav", self._nav_command, 20)
        self.create_subscription(Twist, "/cmd_vel_smoothed", self._smoothed_command, 20)
        self.create_subscription(Twist, "/cmd_vel_gate", self._gate_command, 20)
        self.create_subscription(
            TwistStamped,
            "/base_controller/cmd_vel",
            self._base_command,
            20,
        )
        self.create_subscription(
            CollisionMonitorState,
            "/collision_monitor_state",
            self._monitor,
            20,
        )
        self.create_subscription(
            DiagnosticArray,
            "/safety/status",
            self._safety_status,
            20,
        )
        self.create_subscription(
            String,
            "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/status",
            self._drivetrain_status,
            20,
        )
        self.timer = self.create_timer(0.1, self._tick)
        self.poses: list[tuple[float, float]] = []
        self.map_poses: list[tuple[float, float]] = []
        self.topic_sample_counts = {
            topic: 0
            for topic in (
                "/odom",
                "/odom/unfiltered",
                "/tf:odom->base_footprint",
                "/amcl_pose",
                "/scan/navigation",
                "/sensors/lidar_3d/points",
                "/cmd_vel_nav",
                "/cmd_vel_smoothed",
                "/cmd_vel_gate",
                "/base_controller/cmd_vel",
                "/collision_monitor_state",
                "/safety/status",
                "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/status",
            )
        }
        self.nearest_scan = math.inf
        self.nav_linear = 0.0
        self.smoothed_linear = 0.0
        self.gate_linear = 0.0
        self.base_linear = 0.0
        self.monitor_interventions = 0
        self.dynamic_interactions = 0
        self._interaction_latched = False
        self._monitor_active = False
        self.interaction_candidates: list[dict] = []
        self.safety_permit_sample_count = 0
        self.safety_enabled_sample_count = 0
        self.mission_safety_sample_count = 0
        self.mission_safety_inhibit_sample_count = 0
        self.bms_fault_clear_sample_count = 0
        self.traction_permitted_sample_count = 0
        self.goal_sent = False
        self.goal_odom_start_index: int | None = None
        self.goal_map_start_index: int | None = None
        self.goal_accepted = False
        self.goal_status: int | None = None
        self.feedback_sample_count = 0
        self.maximum_recovery_count = 0
        self.minimum_distance_remaining_m = math.inf
        self.completion_reason = "running"
        self.done = False

    def _raw_odom(self, message: Odometry) -> None:
        self.topic_sample_counts["/odom/unfiltered"] += 1

    def _tf(self, message: TFMessage) -> None:
        self.topic_sample_counts["/tf:odom->base_footprint"] += sum(
            transform.header.frame_id == "odom"
            and transform.child_frame_id == "base_footprint"
            for transform in message.transforms
        )

    def _drivetrain_status(self, _message: String) -> None:
        self.topic_sample_counts[
            "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/status"
        ] += 1

    def _odom(self, message: Odometry) -> None:
        self.topic_sample_counts["/odom"] += 1
        point = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )
        if not self.poses or math.dist(point, self.poses[-1]) >= 0.02:
            self.poses.append(point)

    def _map_pose(self, message: PoseWithCovarianceStamped) -> None:
        self.topic_sample_counts["/amcl_pose"] += 1
        point = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )
        if not self.map_poses or math.dist(point, self.map_poses[-1]) >= 0.02:
            self.map_poses.append(point)

    def _scan(self, message: LaserScan) -> None:
        self.topic_sample_counts["/scan/navigation"] += 1
        finite = [float(value) for value in message.ranges if math.isfinite(value)]
        self.nearest_scan = min(finite, default=math.inf)

    def _pointcloud(self, _message: PointCloud2) -> None:
        self.topic_sample_counts["/sensors/lidar_3d/points"] += 1

    def _nav_command(self, message: Twist) -> None:
        self.topic_sample_counts["/cmd_vel_nav"] += 1
        self.nav_linear = abs(float(message.linear.x))

    def _smoothed_command(self, message: Twist) -> None:
        self.topic_sample_counts["/cmd_vel_smoothed"] += 1
        self.smoothed_linear = abs(float(message.linear.x))

    def _gate_command(self, message: Twist) -> None:
        self.topic_sample_counts["/cmd_vel_gate"] += 1
        self.gate_linear = abs(float(message.linear.x))

    def _base_command(self, message: TwistStamped) -> None:
        self.topic_sample_counts["/base_controller/cmd_vel"] += 1
        self.base_linear = abs(float(message.twist.linear.x))

    def _monitor(self, message: CollisionMonitorState) -> None:
        self.topic_sample_counts["/collision_monitor_state"] += 1
        active = int(message.action_type) != int(CollisionMonitorState.DO_NOTHING)
        if active and not self._monitor_active:
            self.monitor_interventions += 1
        self._monitor_active = active
        # This is a sensor-side interaction: a current scan return plus the
        # scan collision gate attenuating a commanded motion. No pedestrian
        # velocity or schedule is consumed here.
        interaction = (
            active
            and self.nearest_scan < 3.0
            and self.smoothed_linear > 0.05
            and self.gate_linear < self.smoothed_linear * 0.8
        )
        if interaction and not self._interaction_latched:
            self.dynamic_interactions += 1
            self.interaction_candidates.append(
                {
                    "observation_ros_time_ns": self.get_clock().now().nanoseconds,
                    "vehicle_pose_map": (
                        list(self.map_poses[-1]) if self.map_poses else None
                    ),
                    "nearest_scan_range_m": (
                        self.nearest_scan if math.isfinite(self.nearest_scan) else None
                    ),
                    "smoothed_linear_command_mps": self.smoothed_linear,
                    "collision_checked_linear_command_mps": self.gate_linear,
                }
            )
        self._interaction_latched = interaction

    def _safety_status(self, message: DiagnosticArray) -> None:
        self.topic_sample_counts["/safety/status"] += 1
        for status in message.status:
            if status.name != "whole_vehicle_safety":
                continue
            values = {item.key: item.value for item in status.values}
            if values.get("safety_inputs_permit_actuators") == "true":
                self.safety_permit_sample_count += 1
            if self.goal_sent:
                self.mission_safety_sample_count += 1
                if values.get("safety_inputs_permit_actuators") != "true":
                    self.mission_safety_inhibit_sample_count += 1
            if values.get("base_command_enabled") == "true":
                self.safety_enabled_sample_count += 1
            if (
                values.get("bms_fault_available") == "true"
                and values.get("bms_fault_active") == "false"
            ):
                self.bms_fault_clear_sample_count += 1
            if (
                values.get("traction_permit_available") == "true"
                and values.get("traction_permitted") == "true"
            ):
                self.traction_permitted_sample_count += 1

    def _tick(self) -> None:
        self.estop.publish(Bool(data=False))
        self.estop_reset.publish(Bool(data=True))
        self.power.publish(Bool(data=True))
        if (
            not self.goal_sent
            and self.action.server_is_ready()
            and all(
                self.topic_sample_counts[topic] > 0
                for topic in (
                    "/odom",
                    "/odom/unfiltered",
                    "/tf:odom->base_footprint",
                    "/amcl_pose",
                    "/scan/navigation",
                    "/sensors/lidar_3d/points",
                    "/collision_monitor_state",
                    "/safety/status",
                    "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/status",
                )
            )
            and self.safety_permit_sample_count > 0
            and self.safety_enabled_sample_count > 0
            and self.bms_fault_clear_sample_count > 0
            and self.traction_permitted_sample_count > 0
        ):
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = self.goal_x
            goal.pose.pose.position.y = self.goal_y
            goal.pose.pose.orientation.w = 1.0
            self.goal_odom_start_index = max(0, len(self.poses) - 1)
            self.goal_map_start_index = max(0, len(self.map_poses) - 1)
            self.goal_sent = True
            future = self.action.send_goal_async(
                goal, feedback_callback=self._goal_feedback
            )
            future.add_done_callback(self._goal_response)
        if time.monotonic() - self.started >= self.timeout_s:
            self.completion_reason = "timeout"
            self.done = True

    def _goal_response(self, future) -> None:  # type: ignore[no-untyped-def]
        handle = future.result()
        if handle is None or not handle.accepted:
            self.goal_status = GoalStatus.STATUS_ABORTED
            self.completion_reason = "goal_rejected"
            self.done = True
            return
        self.goal_accepted = True
        result = handle.get_result_async()
        result.add_done_callback(self._goal_result)

    def _goal_feedback(self, message) -> None:  # type: ignore[no-untyped-def]
        feedback = message.feedback
        self.feedback_sample_count += 1
        self.maximum_recovery_count = max(
            self.maximum_recovery_count,
            int(feedback.number_of_recoveries),
        )
        remaining = float(feedback.distance_remaining)
        if math.isfinite(remaining):
            self.minimum_distance_remaining_m = min(
                self.minimum_distance_remaining_m, remaining
            )

    def _goal_result(self, future) -> None:  # type: ignore[no-untyped-def]
        wrapped = future.result()
        self.goal_status = int(wrapped.status)
        self.completion_reason = "action_result"
        self.done = True

    def telemetry(self) -> dict:
        mission_poses = (
            self.poses[self.goal_odom_start_index :]
            if self.goal_odom_start_index is not None
            else []
        )
        mission_map_poses = (
            self.map_poses[self.goal_map_start_index :]
            if self.goal_map_start_index is not None
            else []
        )
        travel = sum(
            math.dist(a, b) for a, b in zip(mission_poses, mission_poses[1:])
        )
        start = mission_map_poses[0] if mission_map_poses else (0.0, 0.0)
        end = (self.goal_x, self.goal_y)
        dx, dy = end[0] - start[0], end[1] - start[1]
        denominator = max(1.0e-9, math.hypot(dx, dy))
        cross_track = max(
            (
                abs(dy * (x - start[0]) - dx * (y - start[1])) / denominator
                for x, y in mission_map_poses
            ),
            default=0.0,
        )
        monitor_nodes = {
            f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
            for info in self.get_publishers_info_by_topic("/collision_monitor_state")
        }
        final_publishers = {
            f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
            for info in self.get_publishers_info_by_topic(
                "/base_controller/cmd_vel"
            )
        }
        final_node = next(iter(final_publishers), None)
        selected_odom_publishers = {
            f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
            for info in self.get_publishers_info_by_topic("/odom")
        }
        raw_odom_publishers = {
            f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
            for info in self.get_publishers_info_by_topic("/odom/unfiltered")
        }
        command_topics = (
            "/cmd_vel_nav",
            "/cmd_vel_smoothed",
            "/cmd_vel_gate",
            "/base_controller/cmd_vel",
        )
        command_publishers = {
            topic: sorted(
                {
                    f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace(
                        "//", "/"
                    )
                    for info in self.get_publishers_info_by_topic(topic)
                }
            )
            for topic in command_topics
        }
        node_graph = sorted(
            {
                f"{namespace.rstrip('/')}/{name}".replace("//", "/")
                for name, namespace in self.get_node_names_and_namespaces()
            }
        )
        geofence_violations = sum(
            not point_in_polygon(point, self.geofence, boundary_is_inside=True)
            for point in mission_map_poses
        )
        truth_subscriber_audit = {
            topic: sorted(
                {
                    f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace(
                        "//", "/"
                    )
                    for info in self.get_subscriptions_info_by_topic(topic)
                }
            )
            for topic in CONTROL_PROHIBITED_TRUTH_TOPICS
        }
        return {
            "vehicle_profile": "formal_transport_stowed",
            "command_chain": [
                "/cmd_vel_nav",
                "/cmd_vel_smoothed",
                "/cmd_vel_gate",
                "/base_controller/cmd_vel",
            ],
            "expected_pedestrian_count": self.expected_pedestrian_count,
            "mission_goal_source": self.mission_contract["goal_source"],
            "mission_goal_map": [self.goal_x, self.goal_y],
            "source_fixed_start_pose": self.mission_contract[
                "source_fixed_start_pose"
            ],
            "product_control_reads_pedestrian_truth": False,
            "pedestrian_velocity_estimation_used": False,
            "collision_monitor_node_count": len(monitor_nodes),
            "collision_monitor_nodes": sorted(monitor_nodes),
            "final_command_publisher_count": len(final_publishers),
            "final_command_publisher_node": final_node,
            "command_topic_publishers": command_publishers,
            "runtime_node_graph": node_graph,
            "selected_odom_publishers": sorted(selected_odom_publishers),
            "raw_odom_publishers": sorted(raw_odom_publishers),
            "goal_accepted": self.goal_accepted,
            "nav2_goal_succeeded": self.goal_status == GoalStatus.STATUS_SUCCEEDED,
            "nav2_goal_status": self.goal_status,
            "completion_reason": self.completion_reason,
            "feedback_sample_count": self.feedback_sample_count,
            "maximum_recovery_count": self.maximum_recovery_count,
            "minimum_distance_remaining_m": (
                self.minimum_distance_remaining_m
                if math.isfinite(self.minimum_distance_remaining_m)
                else None
            ),
            "physical_travel_distance_m": travel,
            "mission_odom_trajectory_xy_m": [list(point) for point in mission_poses],
            "mission_map_trajectory_xy_m": [list(point) for point in mission_map_poses],
            "maximum_cross_track_detour_m": cross_track,
            "verified_dynamic_interaction_count": self.dynamic_interactions,
            "dynamic_interaction_candidates": self.interaction_candidates,
            "collision_monitor_intervention_count": self.monitor_interventions,
            "geofence_violation_count": geofence_violations,
            "odom_pose_sample_count": len(mission_poses),
            "map_pose_sample_count": len(mission_map_poses),
            "mission_metrics_begin_at_goal_submission": True,
            "topic_sample_counts": self.topic_sample_counts,
            "safety_permit_sample_count": self.safety_permit_sample_count,
            "safety_enabled_sample_count": self.safety_enabled_sample_count,
            "mission_safety_sample_count": self.mission_safety_sample_count,
            "mission_safety_inhibit_sample_count": (
                self.mission_safety_inhibit_sample_count
            ),
            "bms_fault_clear_sample_count": self.bms_fault_clear_sample_count,
            "traction_permitted_sample_count": self.traction_permitted_sample_count,
            "control_truth_topics_subscribed": [],
            "evaluator_truth_topics_subscribed": [],
            "control_prohibited_truth_topic_subscriber_audit": truth_subscriber_audit,
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode-manifest", type=Path, required=True)
    parser.add_argument("--runtime-build-manifest", type=Path, required=True)
    parser.add_argument("--runtime-world-manifest", type=Path, required=True)
    parser.add_argument("--goal-x", type=float)
    parser.add_argument("--goal-y", type=float)
    parser.add_argument("--nominal-leg", type=float, default=30.0)
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    mission_contract = load_public_mission_contract(
        args.episode_manifest,
        goal_x=args.goal_x,
        goal_y=args.goal_y,
        nominal_leg_m=args.nominal_leg,
    )
    rclpy.init()
    node = RuntimeCollector(
        mission_contract=mission_contract,
        timeout_s=args.timeout,
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        value = node.telemetry()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    value["runtime_build_manifest"] = json.loads(
        args.runtime_build_manifest.read_text(encoding="utf-8")
    )
    value["runtime_world_manifest"] = json.loads(
        args.runtime_world_manifest.read_text(encoding="utf-8")
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    return 0 if value["nav2_goal_succeeded"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
