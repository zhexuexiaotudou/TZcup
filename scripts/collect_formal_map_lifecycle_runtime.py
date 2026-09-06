#!/usr/bin/env python3
"""Collect live, truth-free evidence for mapping or saved-map cleaning.

The collector is an observer of the product ROS graph.  Its only publishers
are the three operator safety commands required by the simulation safety input
adapter; actuator commands still have to traverse collision_monitor and
whole_vehicle_safety_manager.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray
import yaml

from sanitation_formal_campus_integration.saved_map_coverage_core import (
    ProductCoverageTelemetry,
    coverage_execution_passed,
    load_product_mission_geometry,
)
from sanitation_formal_campus_integration.map_lifecycle_core import (
    hard_restart_record_valid,
)
from sanitation_formal_campus_integration.runtime_evidence_core import (
    COMMAND_CHAIN_RECEIPT_REORDER_TOLERANCE_S,
    COMMAND_CHAIN_TOPICS,
    EXPECTED_COMMAND_TOPIC_PUBLISHER,
    first_nonzero_chain_is_ordered,
)


def _nodes_for_endpoints(endpoints) -> list[str]:  # type: ignore[no-untyped-def]
    return sorted({
        f"{item.node_namespace.rstrip('/')}/{item.node_name}".replace("//", "/")
        for item in endpoints
    })


def _hashes_valid(root: Path) -> bool:
    try:
        manifest = json.loads(
            (root / "map_lifecycle_manifest.json").read_text(encoding="utf-8")
        )
        occupancy_name = manifest.get("occupancy_map")
        if occupancy_name != "occupancy.yaml":
            return False
        metadata = yaml.safe_load(
            (root / occupancy_name).read_text(encoding="utf-8")
        )
    except (OSError, json.JSONDecodeError, yaml.YAMLError):
        return False
    image_name = metadata.get("image") if isinstance(metadata, dict) else None
    if (
        image_name != "occupancy.pgm"
        or Path(image_name).is_absolute()
        or Path(image_name).name != image_name
        or "/" in image_name
        or "\\" in image_name
    ):
        return False
    required = {
        occupancy_name,
        image_name,
        "mission_geometry.yaml",
        "materialization_contract.yaml",
        "geofence_keepout.yaml",
        "geofence_keepout.pgm",
        "neutral_speed.yaml",
        "neutral_speed.pgm",
    }
    hashes = manifest.get("sha256")
    resolved_root = root.resolve()
    return isinstance(hashes, dict) and set(hashes) == required and all(
        isinstance(expected, str)
        and len(expected) == 64
        and Path(name).name == name
        and "/" not in name
        and "\\" not in name
        and not (root / name).is_symlink()
        and (root / name).is_file()
        and (root / name).resolve().parent == resolved_root
        and hashlib.sha256((root / name).read_bytes()).hexdigest() == expected
        for name, expected in hashes.items()
    )


class Collector(Node):
    def __init__(
        self,
        *,
        mode: str,
        map_root: Path,
        timeout_sec: float,
        restart_record: Path | None,
        mission_geometry: Path | None,
        coverage_report: Path | None,
    ) -> None:
        super().__init__("formal_map_lifecycle_runtime_collector")
        self.mode = mode
        self.map_root = map_root
        self.timeout_sec = timeout_sec
        self.restart_record = restart_record
        self.coverage_report = coverage_report
        self.started = time.monotonic()
        self.done = False
        self.completion_reason = "running"
        self.sealed_manifest_seen_at: float | None = None
        self.coverage_terminal_seen_at: float | None = None
        self.coverage_state: dict = {}
        self.coverage_telemetry = (
            ProductCoverageTelemetry(load_product_mission_geometry(mission_geometry))
            if mission_geometry is not None
            else None
        )
        self.estop = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/emergency_stop", 10
        )
        self.estop_reset = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/emergency_stop_reset", 10
        )
        self.power = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/main_power", 10
        )
        self.action = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.create_subscription(TFMessage, "/tf", self._tf, 100)
        self.create_subscription(Odometry, "/odom", self._odom, 50)
        self.create_subscription(OccupancyGrid, "/map", self._map, 10)
        # Gazebo lidar publishers are BEST_EFFORT.  A default RELIABLE
        # subscription is incompatible and silently produces zero samples.
        self.create_subscription(
            LaserScan, "/scan", self._scan, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan,
            "/scan/navigation",
            self._filtered_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(Twist, "/cmd_vel_nav", self._cmd_nav, 20)
        self.create_subscription(
            Twist, "/cmd_vel_smoothed", self._cmd_smoothed, 20
        )
        self.create_subscription(Twist, "/cmd_vel_gate", self._cmd_gate, 20)
        self.create_subscription(
            TwistStamped, "/base_controller/cmd_vel", self._cmd_base, 20
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl, 20
        )
        self.create_subscription(
            CollisionMonitorState,
            "/collision_monitor_state",
            self._collision_state,
            20,
        )
        self.create_subscription(
            MarkerArray,
            "/collision_monitor/collision_points_marker",
            self._collision_points,
            20,
        )
        self.create_subscription(
            DiagnosticArray, "/safety/status", self._safety, 20
        )
        self.create_subscription(
            String, "/formal_mapping/lifecycle_status", self._lifecycle, 10
        )
        self.create_subscription(
            String, "/formal_saved_map_coverage/state", self._coverage_status, 10
        )
        self.create_subscription(Bool, "/brush_enabled", self._brush_state, 20)
        self.create_subscription(Log, "/rosout", self._rosout, 50)
        robot_description_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String,
            "/robot_description",
            self._robot_description,
            robot_description_qos,
        )
        self.timer = self.create_timer(0.1, self._tick)
        self.map_samples = 0
        self.map_known_cells_max = 0
        self.odom_samples = 0
        self.odom_contract_samples = 0
        self.odom_first_xy: tuple[float, float] | None = None
        self.odom_last_xy: tuple[float, float] | None = None
        self.amcl_samples = 0
        self.collision_state_samples = 0
        self.collision_action_counts: dict[int, int] = {}
        self.collision_marker_samples = 0
        self.collision_points_max = 0
        self.collision_points_inside_transport_max = 0
        self.collision_point_min_radius_m = math.inf
        self.scan_samples = 0
        self.scan_finite_ranges_max = 0
        self.scan_close_ranges_max = 0
        self.scan_min_finite_range_m = math.inf
        self.filtered_scan_samples = 0
        self.filtered_scan_finite_ranges_max = 0
        self.filtered_scan_close_ranges_max = 0
        self.filtered_scan_min_finite_range_m = math.inf
        self.command_samples = {
            "/cmd_vel_nav": 0,
            "/cmd_vel_smoothed": 0,
            "/cmd_vel_gate": 0,
            "/base_controller/cmd_vel": 0,
        }
        self.nonzero_command_samples = dict.fromkeys(self.command_samples, 0)
        # The evidence chain is keyed to the first *nonzero* receipt after
        # this collector starts.  Safety's normal zero heartbeat must never be
        # mistaken for product motion or allowed to establish this order.
        self.first_nonzero_command_received_s: dict[str, float | None] = {
            topic: None for topic in self.command_samples
        }
        self.last_nonzero_command_received_s: dict[str, float | None] = {
            topic: None for topic in self.command_samples
        }
        # An active-chain sample is a status received after a nonzero safety
        # base output that itself says the manager permits base commands.  This
        # uses the manager's own configured freshness decision rather than a
        # duplicate collector timeout and deliberately excludes later legal
        # idle-stop diagnostics between frontier goals.
        self.active_command_chain_safety_sample_count = 0
        self.active_command_chain_command_timeout_count = 0
        self.safety_samples = 0
        self.safety_permit_samples = 0
        self.safety_base_enabled_samples = 0
        self.safety_active_reason_counts: dict[str, int] = {}
        self.safety_latest_values: dict[str, str] = {}
        self.odom_tf_stamps: list[float] = []
        self.nav2_ready_seen = False
        self.slam_odom_failures_after_ready = 0
        self.lifecycle_status: dict = {}
        self.robot_description_samples = 0
        self.robot_description_sha256: str | None = None

    def _robot_description(self, message: String) -> None:
        self.robot_description_samples += 1
        self.robot_description_sha256 = hashlib.sha256(
            message.data.encode("utf-8")
        ).hexdigest()

    def _tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if (
                transform.header.frame_id == "odom"
                and transform.child_frame_id == "base_footprint"
            ):
                stamp = (
                    float(transform.header.stamp.sec)
                    + float(transform.header.stamp.nanosec) * 1.0e-9
                )
                if not self.odom_tf_stamps or stamp > self.odom_tf_stamps[-1]:
                    self.odom_tf_stamps.append(stamp)

    def _odom(self, message: Odometry) -> None:
        self.odom_samples += 1
        xy = (message.pose.pose.position.x, message.pose.pose.position.y)
        if self.odom_first_xy is None:
            self.odom_first_xy = xy
        self.odom_last_xy = xy
        if self.coverage_telemetry is not None:
            self.coverage_telemetry.observe_odom(*xy)
        if (
            message.header.frame_id == "odom"
            and message.child_frame_id == "base_footprint"
        ):
            self.odom_contract_samples += 1

    def _scan(self, message: LaserScan) -> None:
        self.scan_samples += 1
        finite = [value for value in message.ranges if math.isfinite(value)]
        self.scan_finite_ranges_max = max(self.scan_finite_ranges_max, len(finite))
        self.scan_close_ranges_max = max(
            self.scan_close_ranges_max,
            sum(value <= 2.0 for value in finite),
        )
        if finite:
            self.scan_min_finite_range_m = min(
                self.scan_min_finite_range_m, min(finite)
            )

    def _filtered_scan(self, message: LaserScan) -> None:
        self.filtered_scan_samples += 1
        finite = [value for value in message.ranges if math.isfinite(value)]
        self.filtered_scan_finite_ranges_max = max(
            self.filtered_scan_finite_ranges_max, len(finite)
        )
        self.filtered_scan_close_ranges_max = max(
            self.filtered_scan_close_ranges_max,
            sum(value <= 2.0 for value in finite),
        )
        if finite:
            self.filtered_scan_min_finite_range_m = min(
                self.filtered_scan_min_finite_range_m, min(finite)
            )

    def _record_command(self, topic: str, linear: float, angular: float) -> None:
        self.command_samples[topic] += 1
        if abs(linear) > 1.0e-4 or abs(angular) > 1.0e-4:
            self.nonzero_command_samples[topic] += 1
            received_s = time.monotonic() - self.started
            if self.first_nonzero_command_received_s[topic] is None:
                self.first_nonzero_command_received_s[topic] = received_s
            self.last_nonzero_command_received_s[topic] = received_s

    def _cmd_nav(self, message: Twist) -> None:
        self._record_command(
            "/cmd_vel_nav", message.linear.x, message.angular.z
        )

    def _cmd_smoothed(self, message: Twist) -> None:
        self._record_command(
            "/cmd_vel_smoothed", message.linear.x, message.angular.z
        )

    def _cmd_gate(self, message: Twist) -> None:
        self._record_command(
            "/cmd_vel_gate", message.linear.x, message.angular.z
        )

    def _cmd_base(self, message: TwistStamped) -> None:
        self._record_command(
            "/base_controller/cmd_vel",
            message.twist.linear.x,
            message.twist.angular.z,
        )

    def _map(self, message: OccupancyGrid) -> None:
        self.map_samples += 1
        self.map_known_cells_max = max(
            self.map_known_cells_max,
            sum(value >= 0 for value in message.data),
        )

    def _amcl(self, message: PoseWithCovarianceStamped) -> None:
        self.amcl_samples += 1
        if self.coverage_telemetry is not None:
            self.coverage_telemetry.observe_map_pose(
                message.pose.pose.position.x, message.pose.pose.position.y
            )

    def _brush_state(self, message: Bool) -> None:
        if self.coverage_telemetry is not None:
            self.coverage_telemetry.set_brush(message.data)

    def _coverage_status(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(value, dict):
            return
        self.coverage_state = value
        if value.get("state") in {"COMPLETED", "FAILED", "STOPPED"}:
            self.coverage_terminal_seen_at = time.monotonic()

    def _collision_state(self, message: CollisionMonitorState) -> None:
        self.collision_state_samples += 1
        action = int(message.action_type)
        self.collision_action_counts[action] = (
            self.collision_action_counts.get(action, 0) + 1
        )

    def _collision_points(self, message: MarkerArray) -> None:
        points = [point for marker in message.markers for point in marker.points]
        self.collision_marker_samples += 1
        self.collision_points_max = max(self.collision_points_max, len(points))
        self.collision_points_inside_transport_max = max(
            self.collision_points_inside_transport_max,
            sum(
                -0.540 <= point.x <= 0.620
                and -0.675 <= point.y <= 0.675
                for point in points
            ),
        )
        if points:
            self.collision_point_min_radius_m = min(
                self.collision_point_min_radius_m,
                min(math.hypot(point.x, point.y) for point in points),
            )

    def _safety(self, message: DiagnosticArray) -> None:
        self.safety_samples += 1
        for status in message.status:
            if status.name != "whole_vehicle_safety":
                continue
            values = {item.key: item.value for item in status.values}
            self.safety_latest_values = values
            reasons = [
                item.strip()
                for item in values.get("active_reasons", "").split(",")
                if item.strip()
            ]
            if (
                self.first_nonzero_command_received_s["/base_controller/cmd_vel"]
                is not None
                and values.get("base_command_enabled") == "true"
            ):
                self.active_command_chain_safety_sample_count += 1
                self.active_command_chain_command_timeout_count += sum(
                    reason.lower() == "command_timeout" for reason in reasons
                )
            for reason in reasons:
                self.safety_active_reason_counts[reason] = (
                    self.safety_active_reason_counts.get(reason, 0) + 1
                )
            if values.get("safety_inputs_permit_actuators") == "true":
                self.safety_permit_samples += 1
            if values.get("base_command_enabled") == "true":
                self.safety_base_enabled_samples += 1

    def _lifecycle(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(value, dict):
            self.lifecycle_status = value

    def _rosout(self, message: Log) -> None:
        if not self.nav2_ready_seen or "slam_toolbox" not in message.name:
            return
        text = message.msg.lower()
        if "odom" in text and any(
            token in text for token in ("failed", "failure", "could not", "cannot")
        ):
            self.slam_odom_failures_after_ready += 1

    def _tick(self) -> None:
        # A continuous operator heartbeat, not an actuator or safety bypass.
        self.estop.publish(Bool(data=False))
        self.estop_reset.publish(Bool(data=True))
        self.power.publish(Bool(data=True))
        self.nav2_ready_seen = self.nav2_ready_seen or self.action.server_is_ready()
        elapsed = time.monotonic() - self.started
        if self.mode == "mapping" and (
            self.map_root / "map_lifecycle_manifest.json"
        ).is_file():
            if self.sealed_manifest_seen_at is None:
                # The manager atomically seals the files before its next
                # latched ready-status publication.  Keep the live graph up
                # briefly so the collector captures that final state.
                self.sealed_manifest_seen_at = time.monotonic()
            elif time.monotonic() - self.sealed_manifest_seen_at >= 3.0:
                self.completion_reason = "sealed_map_manifest_observed"
                self.done = True
        elif (
            self.mode == "cleaning"
            and self.coverage_terminal_seen_at is not None
            and self.coverage_report is not None
            and self.coverage_report.is_file()
            and time.monotonic() - self.coverage_terminal_seen_at >= 3.0
        ):
            self.completion_reason = "coverage_action_terminal_observed"
            self.done = True
        elif elapsed >= self.timeout_sec:
            self.completion_reason = "timeout"
            self.done = True

    def _minimum_tf_rate(self) -> float:
        # Use simulation stamps, so a low Gazebo real-time factor cannot turn a
        # healthy 50 Hz transform into a false low-rate measurement.
        rates = [
            1.0 / delta
            for left, right in zip(self.odom_tf_stamps, self.odom_tf_stamps[1:])
            if math.isfinite(delta := right - left) and 0.0 < delta <= 0.5
        ]
        return min(rates, default=0.0)

    def telemetry(self) -> dict:
        collision_nodes = _nodes_for_endpoints(
            self.get_publishers_info_by_topic("/collision_monitor_state")
        )
        gate_nodes = _nodes_for_endpoints(
            self.get_publishers_info_by_topic("/cmd_vel_gate")
        )
        base_command_nodes = _nodes_for_endpoints(
            self.get_publishers_info_by_topic("/base_controller/cmd_vel")
        )
        command_topic_publishers = {
            topic: _nodes_for_endpoints(self.get_publishers_info_by_topic(topic))
            for topic in COMMAND_CHAIN_TOPICS
        }
        command_chain_publishers_attributed = all(
            command_topic_publishers[topic] == [expected]
            for topic, expected in EXPECTED_COMMAND_TOPIC_PUBLISHER.items()
        )
        odom_nodes = _nodes_for_endpoints(
            self.get_publishers_info_by_topic("/odom")
        )
        runtime_nodes = sorted(
            f"{namespace.rstrip('/')}/{name}".replace("//", "/")
            for name, namespace in self.get_node_names_and_namespaces()
        )
        tf_rate = self._minimum_tf_rate()
        if self.odom_first_xy is None or self.odom_last_xy is None:
            odom_displacement = 0.0
        else:
            odom_displacement = math.hypot(
                self.odom_last_xy[0] - self.odom_first_xy[0],
                self.odom_last_xy[1] - self.odom_first_xy[1],
            )
        command_chain_first_nonzero_s = {
            topic: self.first_nonzero_command_received_s[topic]
            for topic in COMMAND_CHAIN_TOPICS
        }
        command_chain_last_nonzero_s = {
            topic: self.last_nonzero_command_received_s[topic]
            for topic in COMMAND_CHAIN_TOPICS
        }
        command_chain_ordered = first_nonzero_chain_is_ordered(
            command_chain_first_nonzero_s
        )
        command_chain_live = (
            command_chain_ordered
            and command_chain_publishers_attributed
            and all(
                self.nonzero_command_samples[topic] > 0
                for topic in COMMAND_CHAIN_TOPICS
            )
        )
        common = (
            collision_nodes == ["/collision_monitor"]
            and gate_nodes == ["/collision_monitor"]
            and base_command_nodes == ["/whole_vehicle_safety_manager"]
            and self.map_samples > 0
            and self.odom_contract_samples > 0
            and self.safety_permit_samples > 0
            and self.safety_base_enabled_samples > 0
            and self.robot_description_samples > 0
        )
        cleaning_ready = (
            self.mode == "cleaning"
            and self.nav2_ready_seen
            and self.amcl_samples > 0
            and "/amcl" in runtime_nodes
            and "/coverage_server" in runtime_nodes
            and "/formal_saved_map_coverage_lifecycle_manager" in runtime_nodes
        )
        coverage_execution: dict = {}
        if self.coverage_report is not None:
            try:
                candidate = json.loads(
                    self.coverage_report.read_text(encoding="utf-8")
                )
            except (OSError, json.JSONDecodeError):
                candidate = {}
            if isinstance(candidate, dict):
                coverage_execution = candidate
        product_coverage = (
            self.coverage_telemetry.report()
            if self.coverage_telemetry is not None
            else {}
        )
        coverage_terminal_passed = (
            coverage_execution_passed(coverage_execution)
            and self.coverage_state.get("state") == "COMPLETED"
            and float(product_coverage.get("trajectory_total_distance_m", 0.0)) > 0.0
            and float(product_coverage.get("brush_enabled_distance_m", 0.0)) > 0.0
            and int(product_coverage.get("brush_state_sample_count", 0)) >= 2
            and int(product_coverage.get("brush_state_transitions", 0)) >= 2
            and product_coverage.get("brush_disabled_on_exit") is True
            and float(product_coverage.get("estimated_coverage_fraction", 0.0))
            >= 0.95
        )
        restart_verified = False
        restart_value: dict = {}
        if self.restart_record is not None:
            try:
                candidate = json.loads(
                    self.restart_record.read_text(encoding="utf-8")
                )
            except (OSError, json.JSONDecodeError):
                candidate = {}
            if isinstance(candidate, dict):
                restart_value = candidate
                restart_verified = hard_restart_record_valid(
                    candidate, self.map_root
                )
        passed = (
            common
            and (
                self.mode == "cleaning"
                or (
                    len(odom_nodes) == 1
                    and tf_rate >= 10.0
                    and odom_displacement >= 0.10
                    and self.slam_odom_failures_after_ready == 0
                    and self.lifecycle_status.get("ready") is True
                    and command_chain_live
                    and self.filtered_scan_samples > 0
                    and self.collision_state_samples > 0
                    and self.active_command_chain_safety_sample_count > 0
                    and self.active_command_chain_command_timeout_count == 0
                )
            )
            and (self.mode != "cleaning" or cleaning_ready)
            and (self.mode != "cleaning" or restart_verified)
            and (self.mode != "cleaning" or coverage_terminal_passed)
        )
        return {
            "schema_version": 1,
            "mode": self.mode,
            "passed": passed,
            "completion_reason": self.completion_reason,
            "truth_used_for_control": False,
            "control_truth_topics_subscribed": [],
            "operator_safety_commands_published": [
                "/formal_vehicle/simulation/command/emergency_stop",
                "/formal_vehicle/simulation/command/emergency_stop_reset",
                "/formal_vehicle/simulation/command/main_power",
            ],
            "robot_description_sample_count": self.robot_description_samples,
            "robot_description_sha256": self.robot_description_sha256,
            "collision_monitor_node_count": len(collision_nodes),
            "collision_monitor_nodes": collision_nodes,
            "cmd_vel_gate_publisher_count": len(gate_nodes),
            "cmd_vel_gate_publishers": gate_nodes,
            "base_command_publisher_count": len(base_command_nodes),
            "base_command_publishers": base_command_nodes,
            "command_topic_publishers": command_topic_publishers,
            "command_chain_publishers_attributed": command_chain_publishers_attributed,
            # /odom and odom->base_footprint are produced by the same one
            # ros2_control base broadcaster.  The frame contract is checked on
            # every odometry sample and the transform itself is sampled below.
            "odom_tf_publisher_count": len(odom_nodes),
            "odom_publishers": odom_nodes,
            "odom_message_frame_contract_samples": self.odom_contract_samples,
            "odom_tf_sample_count": len(self.odom_tf_stamps),
            "odom_tf_min_rate_hz": tf_rate,
            "odom_displacement_m": odom_displacement,
            "slam_map_observed": self.map_samples > 0,
            "slam_map_sample_count": self.map_samples,
            "slam_map_known_cells_max": self.map_known_cells_max,
            "slam_odom_failures_after_ready": self.slam_odom_failures_after_ready,
            "safety_status_sample_count": self.safety_samples,
            "safety_permit_sample_count": self.safety_permit_samples,
            "safety_base_enabled_sample_count": self.safety_base_enabled_samples,
            "safety_active_reason_counts": self.safety_active_reason_counts,
            "safety_latest_values": self.safety_latest_values,
            "scan_sample_count": self.scan_samples,
            "scan_finite_ranges_max": self.scan_finite_ranges_max,
            "scan_close_ranges_le_2m_max": self.scan_close_ranges_max,
            "scan_min_finite_range_m": (
                self.scan_min_finite_range_m
                if math.isfinite(self.scan_min_finite_range_m)
                else None
            ),
            "filtered_scan_sample_count": self.filtered_scan_samples,
            "filtered_scan_finite_ranges_max": (
                self.filtered_scan_finite_ranges_max
            ),
            "filtered_scan_close_ranges_le_2m_max": (
                self.filtered_scan_close_ranges_max
            ),
            "filtered_scan_min_finite_range_m": (
                self.filtered_scan_min_finite_range_m
                if math.isfinite(self.filtered_scan_min_finite_range_m)
                else None
            ),
            "collision_monitor_state_sample_count": self.collision_state_samples,
            "collision_monitor_action_counts": {
                str(key): value
                for key, value in sorted(self.collision_action_counts.items())
            },
            "collision_marker_sample_count": self.collision_marker_samples,
            "collision_points_max": self.collision_points_max,
            "collision_points_inside_transport_max": (
                self.collision_points_inside_transport_max
            ),
            "collision_point_min_radius_m": (
                self.collision_point_min_radius_m
                if math.isfinite(self.collision_point_min_radius_m)
                else None
            ),
            "command_topic_sample_counts": self.command_samples,
            "nonzero_command_topic_sample_counts": self.nonzero_command_samples,
            "command_chain_first_nonzero_received_s": command_chain_first_nonzero_s,
            "command_chain_last_nonzero_received_s": command_chain_last_nonzero_s,
            "command_chain_first_nonzero_ordered": command_chain_ordered,
            "command_chain_live": command_chain_live,
            "command_chain_receipt_reorder_tolerance_s": (
                COMMAND_CHAIN_RECEIPT_REORDER_TOLERANCE_S
            ),
            "active_command_chain_window_definition": (
                "status_after_first_nonzero_base_output_with_"
                "base_command_enabled_true"
            ),
            "active_command_chain_safety_sample_count": (
                self.active_command_chain_safety_sample_count
            ),
            "active_command_chain_command_timeout_count": (
                self.active_command_chain_command_timeout_count
            ),
            "nav2_action_ready": self.nav2_ready_seen,
            "localization_backend": "amcl" if self.mode == "cleaning" else "slam_toolbox",
            "amcl_pose_sample_count": self.amcl_samples,
            "saved_map_sha256_verified": (
                _hashes_valid(self.map_root) if self.mode == "cleaning" else False
            ),
            "world_derived_map_fallback": False,
            "cleaning_stack_ready": cleaning_ready,
            "coverage_server_ready": "/coverage_server" in runtime_nodes,
            "coverage_action_terminal_passed": coverage_terminal_passed,
            "coverage_execution_report": coverage_execution,
            "coverage_state": self.coverage_state,
            **product_coverage,
            "hard_restart_verified": restart_verified,
            "hard_restart_record": restart_value,
            "lifecycle_status": self.lifecycle_status,
            "runtime_node_graph": runtime_nodes,
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("mapping", "cleaning"), required=True)
    parser.add_argument("--map-root", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=14400.0)
    parser.add_argument("--restart-record", type=Path)
    parser.add_argument("--mission-geometry", type=Path)
    parser.add_argument("--coverage-report", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.mode == "cleaning" and args.restart_record is None:
        parser.error("--restart-record is required in cleaning mode")
    if args.mode == "cleaning" and (
        args.mission_geometry is None or args.coverage_report is None
    ):
        parser.error(
            "--mission-geometry and --coverage-report are required in cleaning mode"
        )
    rclpy.init()
    node = Collector(
        mode=args.mode,
        map_root=args.map_root,
        timeout_sec=args.timeout,
        restart_record=args.restart_record,
        mission_geometry=args.mission_geometry,
        coverage_report=args.coverage_report,
    )
    value: dict | None = None
    last_live_snapshot: dict | None = None
    next_snapshot = time.monotonic()
    interrupted = False
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= next_snapshot:
                last_live_snapshot = node.telemetry()
                next_snapshot = time.monotonic() + 5.0
        value = node.telemetry()
    except (KeyboardInterrupt, ExternalShutdownException):
        interrupted = True
        node.completion_reason = "interrupted_fail_closed"
        if rclpy.ok():
            try:
                value = node.telemetry()
            except Exception:  # the ROS context may be shutting down concurrently
                value = last_live_snapshot
        else:
            value = last_live_snapshot
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    if value is None:
        # This can only occur if shutdown raced the first graph snapshot.  Do
        # not lose the evidence file or invent live observations.
        value = {
            "schema_version": 1,
            "mode": args.mode,
            "passed": False,
            "completion_reason": "interrupted_before_first_snapshot_fail_closed",
            "truth_used_for_control": False,
            "control_truth_topics_subscribed": [],
        }
    elif interrupted:
        value["passed"] = False
        value["completion_reason"] = "interrupted_fail_closed"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0 if value["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
