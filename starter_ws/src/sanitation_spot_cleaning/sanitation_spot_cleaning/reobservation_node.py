"""ROS adapter for bounded active re-observation through Nav2."""

from __future__ import annotations

import json
import math

from sanitation_perception.grid_safety import footprint_costmap_clear, keepout_clear

from .node import PRODUCT_FOOTPRINT_XY
from .reobservation_orchestrator import (
    ProductReobservationOrchestrator,
    ReobservationRequest,
    ReobservationSafety,
    ReobservationState,
)


def main() -> None:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.msg import CollisionMonitorState
    from nav_msgs.msg import OccupancyGrid
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger

    class ProductReobservationNode(Node):
        def __init__(self) -> None:
            super().__init__("product_reobservation")
            self.declare_parameter("maximum_localization_age_s", 0.5)
            self.declare_parameter("maximum_localization_covariance_trace", 0.25)
            self.declare_parameter("transition_timeout_s", 15.0)
            self.declare_parameter("navigation_timeout_s", 60.0)
            self.declare_parameter("fresh_verdict_timeout_s", 4.0)
            self.core = ProductReobservationOrchestrator(maximum_reobserve_count=2)
            self.requests: dict[str, ReobservationRequest] = {}
            self.handled: set[str] = set()
            self.current_request: ReobservationRequest | None = None
            self.selected_pose = None
            self.latest_verdicts: dict[str, dict] = {}
            self.coverage_state = "UNKNOWN"
            self.coverage_state_before_pause = "UNKNOWN"
            self.spot_clean_busy = True
            self.localized_pose = None
            self.localization_received_ns = 0
            self.emergency_stopped = True
            self.collision_clear = False
            self.global_costmap = None
            self.keepout_mask = None
            self.phase_started_ns = self._now_ns()
            self.pause_request_pending = False
            self.pause_service_accepted = False
            self.resume_request_pending = False
            self.resume_service_accepted = False
            self.navigation_pending = False
            self.navigation_goal_handle = None
            self.operation_id = 0

            self.candidate_publisher = self.create_publisher(
                String, "/active_observation/candidate", 20
            )
            self.state_publisher = self.create_publisher(
                String, "/reobserve/state", 20
            )
            self.pause_client = self.create_client(
                Trigger, "/coverage/control/pause"
            )
            self.resume_client = self.create_client(
                Trigger, "/coverage/control/resume"
            )
            self.navigate_client = ActionClient(
                self, NavigateToPose, "/navigate_to_pose"
            )
            latched = QoSProfile(depth=1)
            latched.reliability = ReliabilityPolicy.RELIABLE
            latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(
                String,
                "/perception/product/reobserve_requests",
                self._on_request,
                20,
            )
            self.create_subscription(
                String,
                "/perception/product/action_verdicts",
                self._on_verdict,
                20,
            )
            self.create_subscription(
                String,
                "/active_observation/pose_plan",
                self._on_pose_plan,
                20,
            )
            self.create_subscription(
                String, "/coverage/state", self._on_coverage_state, 20
            )
            self.create_subscription(
                String, "/spot_clean/state", self._on_spot_clean_state, 20
            )
            self.create_subscription(
                PoseWithCovarianceStamped,
                "/localization/fused_pose",
                self._on_localization,
                20,
            )
            self.create_subscription(
                Bool, "/emergency_stop", self._on_emergency_stop, 20
            )
            self.create_subscription(
                CollisionMonitorState,
                "collision_monitor_state",
                self._on_collision_state,
                20,
            )
            self.create_subscription(
                OccupancyGrid,
                "/global_costmap/costmap",
                self._on_costmap,
                10,
            )
            self.create_subscription(
                OccupancyGrid,
                "/keepout_filter_mask",
                self._on_keepout_mask,
                latched,
            )
            self.create_timer(0.05, self._step)
            self.create_timer(0.2, self._publish_state)

        def _now_ns(self) -> int:
            return int(self.get_clock().now().nanoseconds)

        def _on_request(self, message) -> None:
            try:
                payload = json.loads(message.data)
                request = ReobservationRequest(
                    request_id=str(payload["request_id"]),
                    track_uuid=str(payload["track_uuid"]),
                    target_uuid=str(payload["target_uuid"]),
                    stamp_ns=int(payload["stamp_ns"]),
                    x_m=float(payload["x_m"]),
                    y_m=float(payload["y_m"]),
                    covariance_trace=float(payload["covariance_trace"]),
                    class_id=str(payload["class_id"]),
                    target_size_m=float(payload["target_size_m"]),
                    reobserve_count=int(payload["reobserve_count"]),
                    source_backend=str(payload["source_backend"]),
                )
            except (KeyError, TypeError, ValueError):
                return
            lowered_source = request.source_backend.lower()
            if any(token in lowered_source for token in (
                "ground_truth", "gazebo_registry", "evaluation_registry"
            )):
                self.get_logger().error("GT control violation rejected")
                return
            if request.request_id not in self.handled:
                self.requests[request.request_id] = request

        def _on_verdict(self, message) -> None:
            try:
                payload = json.loads(message.data)
                self.latest_verdicts[str(payload["target_uuid"])] = payload
            except (KeyError, TypeError, ValueError):
                return
            self._consume_verdict()

        def _on_pose_plan(self, message) -> None:
            if self.core.state != ReobservationState.WAITING_POSE:
                return
            try:
                payload = json.loads(message.data)
            except (TypeError, ValueError):
                return
            if self.current_request is None or str(payload.get("candidate_id")) != self.current_request.request_id:
                return
            accepted = payload.get("accepted") is True and isinstance(
                payload.get("pose"), dict
            )
            if accepted:
                pose_record = payload["pose"]
                try:
                    selected = PoseStamped()
                    selected.header.frame_id = "map"
                    selected.header.stamp = self.get_clock().now().to_msg()
                    selected.pose.position.x = float(pose_record["x"])
                    selected.pose.position.y = float(pose_record["y"])
                    yaw = float(pose_record["yaw"])
                    selected.pose.orientation.z = math.sin(yaw * 0.5)
                    selected.pose.orientation.w = math.cos(yaw * 0.5)
                    self.selected_pose = selected
                except (KeyError, TypeError, ValueError):
                    accepted = False
            if not self.core.acknowledge_pose(
                accepted, self._safety(path_available=accepted)
            ):
                self._recover_or_finish("observation_pose_rejected")
                return
            self.coverage_state_before_pause = self.coverage_state
            self.phase_started_ns = self._now_ns()

        def _on_coverage_state(self, message) -> None:
            self.coverage_state = str(message.data)

        def _on_spot_clean_state(self, message) -> None:
            try:
                payload = json.loads(message.data)
                self.spot_clean_busy = payload.get("current_target_uuid") is not None
            except (TypeError, ValueError):
                self.spot_clean_busy = True

        def _on_localization(self, message) -> None:
            self.localized_pose = message
            self.localization_received_ns = self._now_ns()

        def _on_emergency_stop(self, message) -> None:
            self.emergency_stopped = bool(message.data)

        def _on_collision_state(self, message) -> None:
            self.collision_clear = int(message.action_type) == int(
                getattr(message, "DO_NOTHING", 0)
            )

        def _on_costmap(self, message) -> None:
            self.global_costmap = message

        def _on_keepout_mask(self, message) -> None:
            self.keepout_mask = message

        def _safety(self, *, path_available: bool) -> ReobservationSafety:
            age_s = (
                (self._now_ns() - self.localization_received_ns) / 1e9
                if self.localization_received_ns else math.inf
            )
            covariance = (
                self.localized_pose.pose.covariance
                if self.localized_pose is not None else ()
            )
            covariance_trace = (
                float(covariance[0]) + float(covariance[7])
                if len(covariance) >= 8 else math.inf
            )
            request = self.current_request
            target_clear = bool(request) and keepout_clear(
                self.keepout_mask, request.x_m, request.y_m
            )
            pose = self.selected_pose.pose if self.selected_pose is not None else None
            yaw = (
                2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
                if pose is not None else 0.0
            )
            footprint_clear = bool(pose) and footprint_costmap_clear(
                self.global_costmap,
                pose.position.x,
                pose.position.y,
                yaw,
                PRODUCT_FOOTPRINT_XY,
            )
            return ReobservationSafety(
                emergency_stopped=self.emergency_stopped,
                collision_clear=self.collision_clear,
                localization_healthy=(
                    age_s <= float(self.get_parameter("maximum_localization_age_s").value)
                    and covariance_trace <= float(self.get_parameter(
                        "maximum_localization_covariance_trace"
                    ).value)
                ),
                keepout_clear=bool(target_clear and footprint_clear),
                path_available=bool(path_available),
            )

        def _begin_request(self) -> None:
            if self.spot_clean_busy or self.coverage_state in {"UNKNOWN", "PAUSED", "PAUSING"}:
                return
            pending = [
                request for key, request in self.requests.items()
                if key not in self.handled
            ]
            if not pending:
                return
            request = min(pending, key=lambda item: (item.stamp_ns, item.request_id))
            if not self.core.submit(request):
                self.handled.add(request.request_id)
                return
            self.current_request = request
            self.operation_id += 1
            self.selected_pose = None
            self.phase_started_ns = self._now_ns()
            self.candidate_publisher.publish(String(data=json.dumps({
                "candidate_id": request.request_id,
                "x_m": request.x_m,
                "y_m": request.y_m,
                "target_size_m": request.target_size_m,
                "class_id": request.class_id,
                "covariance_trace": request.covariance_trace,
                "ground_truth_pose_used": False,
            }, sort_keys=True)))

        def _request_pause(self) -> None:
            if self.pause_request_pending or not self.pause_client.service_is_ready():
                return
            self.pause_request_pending = True
            operation_id = self.operation_id
            self.pause_client.call_async(Trigger.Request()).add_done_callback(
                lambda future: self._pause_response(future, operation_id)
            )

        def _pause_response(self, future, operation_id: int) -> None:
            if (
                operation_id != self.operation_id
                or self.current_request is None
                or self.core.state != ReobservationState.WAITING_SAFE_PAUSE
            ):
                return
            self.pause_request_pending = False
            try:
                self.pause_service_accepted = bool(future.result().success)
            except Exception:
                self.pause_service_accepted = False
            if not self.pause_service_accepted and self.core.state == ReobservationState.WAITING_SAFE_PAUSE:
                self.core.acknowledge_pause(False)
                self._recover_or_finish("coverage_pause_failed")

        def _begin_navigation(self) -> None:
            if self.navigation_pending or self.selected_pose is None:
                return
            if not self.navigate_client.server_is_ready():
                return
            if not self.core.acknowledge_pause(True):
                self._recover_or_finish("coverage_pause_not_acknowledged")
                return
            goal = NavigateToPose.Goal()
            goal.pose = self.selected_pose
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            self.navigation_pending = True
            self.phase_started_ns = self._now_ns()
            operation_id = self.operation_id
            self.navigate_client.send_goal_async(goal).add_done_callback(
                lambda future: self._navigation_goal_response(
                    future, operation_id
                )
            )

        def _navigation_goal_response(self, future, operation_id: int) -> None:
            if operation_id != self.operation_id:
                return
            try:
                handle = future.result()
                if not handle.accepted:
                    self._navigation_complete(False, operation_id)
                    return
                self.navigation_goal_handle = handle
                handle.get_result_async().add_done_callback(
                    lambda result_future: self._navigation_result(
                        result_future, operation_id
                    )
                )
            except Exception:
                self._navigation_complete(False, operation_id)

        def _navigation_result(self, future, operation_id: int) -> None:
            if operation_id != self.operation_id:
                return
            try:
                succeeded = future.result().status == GoalStatus.STATUS_SUCCEEDED
            except Exception:
                succeeded = False
            self._navigation_complete(succeeded, operation_id)

        def _navigation_complete(
            self, succeeded: bool, operation_id: int | None = None
        ) -> None:
            if (
                operation_id is not None and operation_id != self.operation_id
            ) or self.core.state != ReobservationState.NAVIGATING:
                return
            self.navigation_pending = False
            self.navigation_goal_handle = None
            if not self.core.acknowledge_navigation(
                succeeded, self._safety(path_available=True)
            ):
                self._recover_or_finish("navigation_failed")
                return
            self.phase_started_ns = self._now_ns()
            self._consume_verdict()

        def _consume_verdict(self) -> None:
            if (
                self.core.state != ReobservationState.WAITING_FRESH_VERDICT
                or self.current_request is None
            ):
                return
            payload = self.latest_verdicts.get(self.current_request.target_uuid)
            if not payload:
                return
            if self.core.observe_verdict(
                stamp_ns=int(payload.get("stamp_ns", 0)),
                verdict=str(payload.get("verdict", "")),
            ):
                self.phase_started_ns = self._now_ns()

        def _request_resume(self) -> None:
            if self.resume_request_pending or not self.resume_client.service_is_ready():
                return
            self.resume_request_pending = True
            operation_id = self.operation_id
            self.resume_client.call_async(Trigger.Request()).add_done_callback(
                lambda future: self._resume_response(future, operation_id)
            )

        def _resume_response(self, future, operation_id: int) -> None:
            if (
                operation_id != self.operation_id
                or self.current_request is None
                or self.core.state not in {
                    ReobservationState.WAITING_RESUME,
                    ReobservationState.DEFERRED,
                }
            ):
                return
            self.resume_request_pending = False
            try:
                self.resume_service_accepted = bool(future.result().success)
            except Exception:
                self.resume_service_accepted = False
            if not self.resume_service_accepted and self.core.state in {
                ReobservationState.WAITING_RESUME,
                ReobservationState.DEFERRED,
            }:
                self.core.acknowledge_resume(False)
                self._finish_current()

        def _coverage_restored(self) -> bool:
            return (
                self.coverage_state == self.coverage_state_before_pause
                and self.coverage_state not in {"UNKNOWN", "PAUSED", "PAUSING"}
            )

        def _recover_or_finish(self, reason: str) -> None:
            if self.core.state not in {
                ReobservationState.DEFERRED,
                ReobservationState.FAILED,
            }:
                self.core.defer(reason)
            if self.core.coverage_paused or self.coverage_state == "PAUSED":
                self.phase_started_ns = self._now_ns()
                self.resume_request_pending = False
                self.resume_service_accepted = False
            else:
                self._finish_current()

        def _finish_current(self) -> None:
            if self.current_request is not None:
                self.handled.add(self.current_request.request_id)
            self.current_request = None
            self.selected_pose = None
            self.pause_request_pending = False
            self.pause_service_accepted = False
            self.resume_request_pending = False
            self.resume_service_accepted = False
            self.navigation_pending = False
            self.navigation_goal_handle = None
            self.phase_started_ns = self._now_ns()

        def _timed_out(self, parameter: str) -> bool:
            return (self._now_ns() - self.phase_started_ns) / 1e9 > float(
                self.get_parameter(parameter).value
            )

        def _step(self) -> None:
            state = self.core.state
            if self.current_request is None:
                self._begin_request()
                return
            if state in {
                ReobservationState.WAITING_SAFE_PAUSE,
                ReobservationState.NAVIGATING,
            }:
                safety = self._safety(path_available=True)
                reason = self.core.safety_reason(safety)
                if reason is not None:
                    if state == ReobservationState.WAITING_SAFE_PAUSE:
                        self.core.acknowledge_pause(False)
                    else:
                        if self.navigation_goal_handle is not None:
                            self.navigation_goal_handle.cancel_goal_async()
                        self.core.acknowledge_navigation(False, safety)
                    self._recover_or_finish(f"runtime_safety_changed:{reason}")
                    return
            if state == ReobservationState.WAITING_POSE:
                if self._timed_out("transition_timeout_s"):
                    self.core.defer("observation_pose_timeout")
                    self._recover_or_finish("observation_pose_timeout")
                return
            if state == ReobservationState.WAITING_SAFE_PAUSE:
                self._request_pause()
                if self.pause_service_accepted and self.coverage_state == "PAUSED":
                    self._begin_navigation()
                elif self._timed_out("transition_timeout_s"):
                    self.core.acknowledge_pause(False)
                    self._recover_or_finish("coverage_pause_timeout")
                return
            if state == ReobservationState.NAVIGATING:
                if not self.navigation_pending:
                    self._begin_navigation()
                if self._timed_out("navigation_timeout_s"):
                    if self.navigation_goal_handle is not None:
                        self.navigation_goal_handle.cancel_goal_async()
                    self._navigation_complete(False, self.operation_id)
                return
            if state == ReobservationState.WAITING_FRESH_VERDICT:
                self._consume_verdict()
                if self._timed_out("fresh_verdict_timeout_s"):
                    self.core.defer("fresh_verdict_timeout")
                    self._recover_or_finish("fresh_verdict_timeout")
                return
            if state in {ReobservationState.WAITING_RESUME, ReobservationState.DEFERRED}:
                if not self.core.coverage_paused and self.coverage_state != "PAUSED":
                    self._finish_current()
                    return
                self._request_resume()
                if self.resume_service_accepted and self._coverage_restored():
                    self.core.acknowledge_resume(True)
                    self._finish_current()
                elif self._timed_out("transition_timeout_s"):
                    self.core.acknowledge_resume(False)
                    self._finish_current()

        def _publish_state(self) -> None:
            payload = self.core.snapshot()
            payload.update({
                "busy": self.current_request is not None,
                "coverage_state": self.coverage_state,
                "queued_request_count": sum(
                    key not in self.handled for key in self.requests
                ),
            })
            self.state_publisher.publish(
                String(data=json.dumps(payload, sort_keys=True))
            )

    rclpy.init()
    node = ProductReobservationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
