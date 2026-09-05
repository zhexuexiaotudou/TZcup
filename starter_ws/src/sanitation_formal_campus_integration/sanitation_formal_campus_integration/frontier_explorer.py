"""Sensor-map frontier exploration through Nav2, without direct velocity control."""

from __future__ import annotations

import json
import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .map_lifecycle_core import (
    goal_tangent_yaw,
    load_campus_map_contract,
    select_frontier_goal,
)
from .frontier_runtime_core import (
    active_goal_timed_out,
    bounded_action_server_ready,
    goal_response_timed_out,
    progress_deadline_after_feedback,
)


class FormalFrontierExplorer(Node):
    """Expand SLAM known space while Nav2 performs obstacle-aware motion."""

    def __init__(self) -> None:
        super().__init__("formal_frontier_explorer")
        self.declare_parameter("episode_manifest", "")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("navigate_action", "/navigate_to_pose")
        self.declare_parameter("planning_period_sec", 2.0)
        self.declare_parameter("sample_spacing_m", 0.50)
        self.declare_parameter("max_consecutive_failures", 20)
        self.declare_parameter("action_discovery_timeout_sec", 0.1)
        self.declare_parameter("goal_response_timeout_sec", 5.0)
        self.declare_parameter("goal_execution_timeout_sec", 900.0)
        self.declare_parameter("goal_progress_timeout_sec", 120.0)
        self.declare_parameter("cancel_timeout_sec", 5.0)
        self._contract = load_campus_map_contract(
            str(self.get_parameter("episode_manifest").value)
        )
        self._map: OccupancyGrid | None = None
        self._odom_seen = False
        self._goal_active = False
        self._map_ready = False
        self._failures = 0
        self._previous: list[tuple[float, float]] = []
        self._goals_requested = 0
        self._goals_succeeded = 0
        self._goal_request_sequence = 0
        self._pending_goal_request_id: int | None = None
        self._goal_response_deadline_monotonic: float | None = None
        self._timed_out_goal_request_ids: set[int] = set()
        self._active_goal_handle = None
        self._goal_execution_deadline_monotonic: float | None = None
        self._goal_progress_deadline_monotonic: float | None = None
        self._best_distance_remaining_m = math.inf
        self._cancel_future = None
        self._cancel_response_deadline_monotonic: float | None = None
        self._cancel_result_deadline_monotonic: float | None = None
        self._cancel_reason: str | None = None
        self._terminal_blocked = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            map_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            20,
        )
        self.create_subscription(Bool, "/formal_mapping/map_ready", self._on_ready, 10)
        self._status = self.create_publisher(String, "/formal_mapping/explorer_status", 10)
        self._client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigate_action").value),
        )
        self.create_timer(
            float(self.get_parameter("planning_period_sec").value), self._plan
        )
        self.create_timer(
            0.1,
            self._watch_goal,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def _publish(self, state: str, **values) -> None:  # type: ignore[no-untyped-def]
        message = String()
        message.data = json.dumps(
            {
                "state": state,
                "world_truth_used_for_control": False,
                "direct_velocity_control": False,
                "planner": "nav2_navigate_to_pose",
                **values,
            },
            sort_keys=True,
        )
        self._status.publish(message)

    def _on_map(self, message: OccupancyGrid) -> None:
        self._map = message

    def _on_odom(self, message: Odometry) -> None:
        self._odom_seen = True

    def _map_position(self) -> tuple[float, float] | None:
        """Read the robot position directly in map, including nonidentity map->odom."""
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("base_frame").value),
                Time(),
            )
        except TransformException as exc:
            self._publish("waiting_for_map_frame_pose", detail=str(exc))
            return None
        translation = transform.transform.translation
        return float(translation.x), float(translation.y)

    def _on_ready(self, message: Bool) -> None:
        self._map_ready = bool(message.data)
        if self._map_ready:
            self._publish("map_quality_gate_complete")

    def _positive_timeout(self, parameter_name: str) -> float:
        value = float(self.get_parameter(parameter_name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{parameter_name} must be finite and positive")
        return value

    def _block(self, state: str, **values) -> None:  # type: ignore[no-untyped-def]
        self._terminal_blocked = True
        self._publish(state, terminal=True, **values)

    def _request_cancel(self, reason: str) -> None:
        if self._active_goal_handle is None or self._cancel_future is not None:
            self._block("frontier_cancel_unavailable", reason=reason)
            return
        self._cancel_reason = reason
        self._cancel_future = self._active_goal_handle.cancel_goal_async()
        self._cancel_response_deadline_monotonic = (
            time.monotonic() + self._positive_timeout("cancel_timeout_sec")
        )
        self._publish("frontier_cancel_requested", reason=reason)

    def _watch_goal(self) -> None:
        now = time.monotonic()
        if (
            self._terminal_blocked
            and not self._timed_out_goal_request_ids
            and self._cancel_future is None
            and self._cancel_result_deadline_monotonic is None
        ):
            return
        if goal_response_timed_out(
            pending_request_id=self._pending_goal_request_id,
            deadline_monotonic=self._goal_response_deadline_monotonic,
            now_monotonic=now,
        ):
            timed_out_request = self._pending_goal_request_id
            assert timed_out_request is not None
            self._timed_out_goal_request_ids.add(timed_out_request)
            self._pending_goal_request_id = None
            self._goal_response_deadline_monotonic = None
            self._failures += 1
            self._block(
                "frontier_goal_response_timeout",
                request_id=timed_out_request,
                failures=self._failures,
            )
            return
        if self._cancel_future is not None:
            if self._cancel_future.done():
                response = self._cancel_future.result()
                self._cancel_future = None
                self._cancel_response_deadline_monotonic = None
                if response is None or not response.goals_canceling:
                    self._block(
                        "frontier_cancel_rejected", reason=self._cancel_reason
                    )
                    return
                self._cancel_result_deadline_monotonic = (
                    now + self._positive_timeout("cancel_timeout_sec")
                )
                self._publish(
                    "frontier_cancel_accepted", reason=self._cancel_reason
                )
            elif active_goal_timed_out(
                goal_active=True,
                deadline_monotonic=self._cancel_response_deadline_monotonic,
                now_monotonic=now,
            ):
                self._block("frontier_cancel_response_timeout", reason=self._cancel_reason)
                self._cancel_future = None
                self._cancel_response_deadline_monotonic = None
            return
        if active_goal_timed_out(
            goal_active=self._goal_active,
            deadline_monotonic=self._cancel_result_deadline_monotonic,
            now_monotonic=now,
        ):
            self._block("frontier_cancel_result_timeout", reason=self._cancel_reason)
            self._cancel_result_deadline_monotonic = None
            return
        if self._cancel_result_deadline_monotonic is not None:
            return
        if self._active_goal_handle is None:
            return
        if active_goal_timed_out(
            goal_active=self._goal_active,
            deadline_monotonic=self._goal_execution_deadline_monotonic,
            now_monotonic=now,
        ):
            self._request_cancel("execution_timeout")
            return
        if active_goal_timed_out(
            goal_active=self._goal_active,
            deadline_monotonic=self._goal_progress_deadline_monotonic,
            now_monotonic=now,
        ):
            self._request_cancel("progress_timeout")

    def _plan(self) -> None:
        if (
            self._terminal_blocked
            or self._map_ready
            or self._goal_active
            or self._map is None
            or not self._odom_seen
        ):
            return
        if self._failures >= int(self.get_parameter("max_consecutive_failures").value):
            self._publish("blocked_excessive_nav2_failures", failures=self._failures)
            return
        message = self._map
        map_position = self._map_position()
        if map_position is None:
            return
        orientation = message.info.origin.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        target = select_frontier_goal(
            message.data,
            width=message.info.width,
            height=message.info.height,
            resolution=message.info.resolution,
            origin_x=message.info.origin.position.x,
            origin_y=message.info.origin.position.y,
            origin_yaw=yaw,
            geofence=self._contract.geofence,
            robot_x=map_position[0],
            robot_y=map_position[1],
            previous_goals=self._previous[-100:],
            sample_spacing_m=float(self.get_parameter("sample_spacing_m").value),
        )
        if target is None:
            self._publish("waiting_for_reachable_frontier")
            return
        if not bounded_action_server_ready(
            self._client,
            timeout_sec=float(
                self.get_parameter("action_discovery_timeout_sec").value
            ),
        ):
            self._publish("waiting_for_nav2_action")
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = target[0]
        goal.pose.pose.position.y = target[1]
        target_yaw = goal_tangent_yaw(
            map_position[0], map_position[1], target[0], target[1]
        )
        goal.pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(target_yaw / 2.0)
        self._goal_active = True
        self._goal_request_sequence += 1
        request_id = self._goal_request_sequence
        self._pending_goal_request_id = request_id
        response_timeout = self._positive_timeout("goal_response_timeout_sec")
        self._goal_response_deadline_monotonic = time.monotonic() + response_timeout
        self._previous.append(target)
        self._goals_requested += 1
        sent = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        sent.add_done_callback(
            lambda future, expected_request_id=request_id: self._on_goal_response(
                future, expected_request_id
            )
        )
        self._publish(
            "frontier_goal_requested",
            x=target[0],
            y=target[1],
            yaw=target_yaw,
            robot_map_x=map_position[0],
            robot_map_y=map_position[1],
            goals_requested=self._goals_requested,
            goals_succeeded=self._goals_succeeded,
        )
        self.get_logger().info(
            "Frontier goal %d requested in map: x=%.3f y=%.3f yaw=%.3f"
            % (self._goals_requested, target[0], target[1], target_yaw)
        )

    def _on_goal_response(  # type: ignore[no-untyped-def]
        self, future, request_id: int
    ) -> None:
        if request_id in self._timed_out_goal_request_ids:
            self._timed_out_goal_request_ids.remove(request_id)
            try:
                handle = future.result()
            except Exception as exc:
                self._goal_active = False
                self._block(
                    "frontier_late_goal_response_error",
                    request_id=request_id,
                    detail=str(exc),
                )
                return
            if handle is not None and handle.accepted:
                self._active_goal_handle = handle
                result = handle.get_result_async()
                result.add_done_callback(self._on_result)
                self._request_cancel("late_accept_after_response_timeout")
            else:
                self._goal_active = False
                self._publish("frontier_late_goal_rejected", request_id=request_id)
            return
        if request_id != self._pending_goal_request_id:
            return
        self._pending_goal_request_id = None
        self._goal_response_deadline_monotonic = None
        try:
            handle = future.result()
        except Exception as exc:
            self._goal_active = False
            self._failures += 1
            self._publish(
                "frontier_goal_response_error",
                detail=str(exc),
                failures=self._failures,
            )
            return
        if handle is None or not handle.accepted:
            self._goal_active = False
            self._failures += 1
            self._publish("frontier_goal_rejected", failures=self._failures)
            return
        self._active_goal_handle = handle
        now = time.monotonic()
        self._goal_execution_deadline_monotonic = (
            now + self._positive_timeout("goal_execution_timeout_sec")
        )
        self._goal_progress_deadline_monotonic = (
            now + self._positive_timeout("goal_progress_timeout_sec")
        )
        self._best_distance_remaining_m = math.inf
        result = handle.get_result_async()
        result.add_done_callback(self._on_result)

    def _on_feedback(self, message) -> None:  # type: ignore[no-untyped-def]
        if self._active_goal_handle is None or self._cancel_future is not None:
            return
        remaining = float(message.feedback.distance_remaining)
        if not math.isfinite(remaining) or remaining < 0.0:
            return
        best, refreshed = progress_deadline_after_feedback(
            previous_best_distance_m=self._best_distance_remaining_m,
            distance_remaining_m=remaining,
            now_monotonic=time.monotonic(),
            timeout_sec=self._positive_timeout("goal_progress_timeout_sec"),
        )
        self._best_distance_remaining_m = best
        if refreshed is not None:
            self._goal_progress_deadline_monotonic = refreshed

    def _on_result(self, future) -> None:  # type: ignore[no-untyped-def]
        try:
            wrapped = future.result()
        except Exception as exc:
            self._goal_active = False
            self._active_goal_handle = None
            self._block("frontier_goal_result_error", detail=str(exc))
            return
        self._goal_active = False
        self._active_goal_handle = None
        self._goal_execution_deadline_monotonic = None
        self._goal_progress_deadline_monotonic = None
        self._cancel_future = None
        self._cancel_response_deadline_monotonic = None
        self._cancel_result_deadline_monotonic = None
        cancel_reason = self._cancel_reason
        self._cancel_reason = None
        if int(wrapped.status) == 4:  # action_msgs/GoalStatus.STATUS_SUCCEEDED
            self._failures = 0
            self._goals_succeeded += 1
            self._publish(
                "frontier_goal_reached",
                goals_requested=self._goals_requested,
                goals_succeeded=self._goals_succeeded,
            )
            self.get_logger().info(
                "Frontier goal %d succeeded; requesting the next frontier"
                % self._goals_succeeded
            )
        else:
            self._failures += 1
            self._publish(
                "frontier_goal_failed",
                status=int(wrapped.status),
                failures=self._failures,
                cancel_reason=cancel_reason,
            )


def main(args=None) -> None:  # type: ignore[no-untyped-def]
    rclpy.init(args=args)
    node = FormalFrontierExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
