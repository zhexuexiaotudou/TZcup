"""ROS adapter for the fail-closed product spot-cleaning state machine."""

from __future__ import annotations

import json
import math

from sanitation_perception.camera_frustum_model import FrustumSweep
from sanitation_perception.grid_safety import footprint_costmap_clear, keepout_clear

from .product_orchestrator import (
    ProductCleanState,
    ProductSafety,
    ProductSpotCleanOrchestrator,
    ProductTarget,
)


PRODUCT_FOOTPRINT_XY = (
    (0.82, 0.66),
    (0.82, -0.66),
    (-0.575, -0.66),
    (-0.575, 0.66),
)


def approach_pose_xyyaw(
    robot_xy: tuple[float, float],
    target_xy: tuple[float, float],
    brush_forward_offset_m: float,
) -> tuple[float, float, float]:
    """Place the base so the physical brush centre, not the base, meets a target."""
    delta_x = float(target_xy[0]) - float(robot_xy[0])
    delta_y = float(target_xy[1]) - float(robot_xy[1])
    yaw = math.atan2(delta_y, delta_x) if math.hypot(delta_x, delta_y) > 1e-9 else 0.0
    return (
        float(target_xy[0]) - float(brush_forward_offset_m) * math.cos(yaw),
        float(target_xy[1]) - float(brush_forward_offset_m) * math.sin(yaw),
        yaw,
    )


def frustum_record_contains(record: dict, x_m: float, y_m: float) -> bool:
    """Validate a persisted online-camera sweep before using it as absence evidence."""
    try:
        sweep = FrustumSweep(
            sweep_id=str(record["sweep_id"]),
            mission_id=str(record["mission_id"]),
            stamp_ns=int(record["stamp_ns"]),
            camera_frame_id=str(record["camera_frame_id"]),
            image_frame_id=str(record["image_frame_id"]),
            camera_x_m=float(record["camera_x_m"]),
            camera_y_m=float(record["camera_y_m"]),
            camera_yaw_rad=float(record["camera_yaw_rad"]),
            horizontal_fov_rad=float(record["horizontal_fov_rad"]),
            minimum_range_m=float(record["minimum_range_m"]),
            maximum_range_m=float(record["maximum_range_m"]),
        )
        sweep.validate()
    except (KeyError, TypeError, ValueError):
        return False
    return sweep.contains(float(x_m), float(y_m))


def main() -> None:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from nav2_msgs.msg import CollisionMonitorState
    from nav_msgs.msg import OccupancyGrid
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sanitation_perception_interfaces.msg import CleaningEvent, GarbageTargetArray
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger

    class ProductSpotCleaningNode(Node):
        def __init__(self) -> None:
            super().__init__("product_spot_cleaning")
            self.declare_parameter("brush_forward_offset_m", 0.55)
            self.declare_parameter("brush_run_duration_s", 1.5)
            self.declare_parameter("maximum_localization_age_s", 0.5)
            self.declare_parameter("maximum_localization_covariance_trace", 0.25)
            self.declare_parameter("maximum_perception_health_age_s", 1.0)
            self.declare_parameter("transition_timeout_s", 15.0)
            self.declare_parameter("navigation_timeout_s", 60.0)
            self.declare_parameter("post_clean_timeout_s", 8.0)
            self.declare_parameter("planner_id", "GridBasedForward")

            self.core = ProductSpotCleanOrchestrator()
            self.targets: dict[str, object] = {}
            self.dynamic_map: dict | None = None
            self.coverage_state = "UNKNOWN"
            self.coverage_state_before_pause = "UNKNOWN"
            self.localized_pose = None
            self.localization_received_ns = 0
            self.perception_health: dict = {}
            self.perception_health_received_ns = 0
            self.emergency_stopped = True
            self.collision_clear = False
            self.global_costmap = None
            self.keepout_mask = None
            self.current_message = None
            self.current_goal_pose: tuple[float, float, float] | None = None
            self.handled: set[str] = set()
            self.phase_started_ns = self._now_ns()
            self.brush_deadline_ns = 0
            self.last_sweep_stamp_ns = 0
            self.path_request_pending = False
            self.pause_request_pending = False
            self.pause_service_accepted = False
            self.resume_request_pending = False
            self.resume_service_accepted = False
            self.abort_recovery = False
            self.navigation_pending = False
            self.navigation_goal_handle = None
            self.deferred_event_sent = False
            self.reobservation_busy = True
            self.operation_id = 0

            self.event_publisher = self.create_publisher(
                CleaningEvent, "/garbage/cleaning_events", 20
            )
            self.state_publisher = self.create_publisher(
                String, "/spot_clean/state", 20
            )
            self.brush_publisher = self.create_publisher(
                Bool, "/brush_enabled", 20
            )
            self.pause_client = self.create_client(
                Trigger, "/coverage/control/pause"
            )
            self.resume_client = self.create_client(
                Trigger, "/coverage/control/resume"
            )
            self.path_client = ActionClient(
                self, ComputePathToPose, "/compute_path_to_pose"
            )
            self.navigate_client = ActionClient(
                self, NavigateToPose, "/navigate_to_pose"
            )

            latched = QoSProfile(depth=1)
            latched.reliability = ReliabilityPolicy.RELIABLE
            latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(
                GarbageTargetArray,
                "/perception/product/targets",
                self._on_targets,
                20,
            )
            self.create_subscription(
                String,
                "/perception/product/dynamic_trash_map",
                self._on_dynamic_map,
                20,
            )
            self.create_subscription(
                String, "/perception/product/health", self._on_health, 20
            )
            self.create_subscription(
                String, "/coverage/state", self._on_coverage_state, 20
            )
            self.create_subscription(
                String, "/reobserve/state", self._on_reobservation_state, 20
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

        @staticmethod
        def _stamp_ns(stamp) -> int:
            return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

        def _on_targets(self, message) -> None:
            for target in message.targets:
                source = str(target.source_backend).lower()
                if any(token in source for token in (
                    "ground_truth", "gazebo_registry", "evaluation_registry"
                )):
                    self.get_logger().error("GT control violation rejected")
                    continue
                self.targets[str(target.uuid)] = target

        def _on_dynamic_map(self, message) -> None:
            try:
                payload = json.loads(message.data)
            except (TypeError, ValueError):
                return
            self.dynamic_map = payload
            self._observe_post_clean(payload)

        def _on_health(self, message) -> None:
            try:
                self.perception_health = json.loads(message.data)
                self.perception_health_received_ns = self._now_ns()
            except (TypeError, ValueError):
                self.perception_health = {}

        def _on_coverage_state(self, message) -> None:
            self.coverage_state = str(message.data)

        def _on_reobservation_state(self, message) -> None:
            try:
                self.reobservation_busy = json.loads(message.data).get("busy") is True
            except (TypeError, ValueError):
                self.reobservation_busy = True

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

        def _target_core(self, target) -> ProductTarget:
            covariance = target.map_pose.covariance
            return ProductTarget(
                uuid=str(target.uuid),
                class_id=str(target.class_id),
                target_type=str(target.target_type).upper(),
                track_state=str(target.track_state),
                confidence=float(target.confidence),
                observation_count=int(target.observation_count),
                covariance_trace=float(covariance[0]) + float(covariance[7]),
                source_backend=str(target.source_backend),
                in_keepout=bool(target.in_keepout),
            )

        def _goal_for_target(self, target) -> tuple[float, float, float] | None:
            if self.localized_pose is None:
                return None
            robot = self.localized_pose.pose.pose.position
            target_position = target.map_pose.pose.position
            return approach_pose_xyyaw(
                (robot.x, robot.y),
                (target_position.x, target_position.y),
                float(self.get_parameter("brush_forward_offset_m").value),
            )

        def _safety(self, target, *, path_available: bool) -> ProductSafety:
            now_ns = self._now_ns()
            localization_age_s = (
                (now_ns - self.localization_received_ns) / 1e9
                if self.localization_received_ns else math.inf
            )
            perception_age_s = (
                (now_ns - self.perception_health_received_ns) / 1e9
                if self.perception_health_received_ns else math.inf
            )
            covariance = (
                self.localized_pose.pose.covariance
                if self.localized_pose is not None else ()
            )
            localization_covariance = (
                float(covariance[0]) + float(covariance[7])
                if len(covariance) >= 8 else math.inf
            )
            goal = self.current_goal_pose or self._goal_for_target(target)
            footprint_clear = bool(goal) and footprint_costmap_clear(
                self.global_costmap, goal[0], goal[1], goal[2], PRODUCT_FOOTPRINT_XY
            )
            target_position = target.map_pose.pose.position
            keepout_is_clear = (
                not bool(target.in_keepout)
                and keepout_clear(
                    self.keepout_mask, target_position.x, target_position.y
                )
            )
            observation_stamp_ns = self._stamp_ns(target.last_seen)
            observation_age_s = (
                max(0.0, (now_ns - observation_stamp_ns) / 1e9)
                if observation_stamp_ns > 0 else math.inf
            )
            return ProductSafety(
                emergency_stopped=self.emergency_stopped,
                collision_clear=self.collision_clear,
                localization_healthy=(
                    localization_age_s
                    <= float(self.get_parameter("maximum_localization_age_s").value)
                    and localization_covariance
                    <= float(self.get_parameter(
                        "maximum_localization_covariance_trace"
                    ).value)
                ),
                perception_healthy=(
                    perception_age_s
                    <= float(self.get_parameter("maximum_perception_health_age_s").value)
                    and self.perception_health.get("state") == "ACTIVE"
                    and self.perception_health.get(
                        "perception_spot_clean_allowed"
                    ) is True
                ),
                keepout_clear=bool(keepout_is_clear and footprint_clear),
                path_available=bool(path_available),
                observation_age_s=observation_age_s,
            )

        def _pose_message(self, pose: tuple[float, float, float]) -> PoseStamped:
            message = PoseStamped()
            message.header.frame_id = "map"
            message.header.stamp = self.get_clock().now().to_msg()
            message.pose.position.x = float(pose[0])
            message.pose.position.y = float(pose[1])
            message.pose.orientation.z = math.sin(float(pose[2]) * 0.5)
            message.pose.orientation.w = math.cos(float(pose[2]) * 0.5)
            return message

        def _prepare_next_target(self) -> None:
            if self.reobservation_busy or self.coverage_state in {
                "UNKNOWN", "PAUSED", "PAUSING"
            }:
                return
            candidates = [
                item for item in self.targets.values()
                if str(item.track_state) == "CONFIRMED"
                and str(item.uuid) not in self.handled
            ]
            if not candidates:
                return
            target = min(candidates, key=lambda item: self._stamp_ns(item.first_seen))
            self.operation_id += 1
            self.current_message = target
            self.current_goal_pose = self._goal_for_target(target)
            self.phase_started_ns = self._now_ns()
            safety = self._safety(target, path_available=True)
            if self.current_goal_pose is None or not all((
                not safety.emergency_stopped,
                safety.collision_clear,
                safety.localization_healthy,
                safety.perception_healthy,
                safety.keepout_clear,
                safety.observation_age_s <= self.core.maximum_observation_age_s,
            )):
                self.core.submit(self._target_core(target), safety)
                self._defer_current("foundation_safety_rejected")
                return
            self._request_path()

        def _request_path(self) -> None:
            if self.path_request_pending or self.current_goal_pose is None:
                return
            if not self.path_client.server_is_ready():
                return
            self.path_request_pending = True
            self.phase_started_ns = self._now_ns()
            goal = ComputePathToPose.Goal()
            goal.goal = self._pose_message(self.current_goal_pose)
            goal.use_start = False
            goal.planner_id = str(self.get_parameter("planner_id").value)
            operation_id = self.operation_id
            self.path_client.send_goal_async(goal).add_done_callback(
                lambda future: self._path_goal_response(future, operation_id)
            )

        def _path_goal_response(self, future, operation_id: int) -> None:
            if operation_id != self.operation_id:
                return
            try:
                handle = future.result()
                if not handle.accepted:
                    self._path_complete(False, operation_id)
                    return
                handle.get_result_async().add_done_callback(
                    lambda result_future: self._path_result(
                        result_future, operation_id
                    )
                )
            except Exception:
                self._path_complete(False, operation_id)

        def _path_result(self, future, operation_id: int) -> None:
            if operation_id != self.operation_id:
                return
            try:
                response = future.result()
                available = (
                    response.status == GoalStatus.STATUS_SUCCEEDED
                    and len(response.result.path.poses) >= 2
                )
            except Exception:
                available = False
            self._path_complete(available, operation_id)

        def _path_complete(
            self, available: bool, operation_id: int | None = None
        ) -> None:
            if operation_id is not None and operation_id != self.operation_id:
                return
            self.path_request_pending = False
            target = self.current_message
            if target is None:
                return
            if not self.core.submit(
                self._target_core(target),
                self._safety(target, path_available=available),
            ):
                self._defer_current("path_or_submit_rejected")
                return
            self._publish_event("scheduled", cleaned_fraction=0.0)
            self.coverage_state_before_pause = self.coverage_state
            self.phase_started_ns = self._now_ns()

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
                or self.current_message is None
                or self.core.state != ProductCleanState.WAITING_SAFE_PAUSE
            ):
                return
            self.pause_request_pending = False
            try:
                self.pause_service_accepted = bool(future.result().success)
            except Exception:
                self.pause_service_accepted = False
            if not self.pause_service_accepted:
                self.core.acknowledge_coverage_pause(False)
                self._defer_current("coverage_pause_failed")

        def _begin_approach(self) -> None:
            if self.navigation_pending or self.current_goal_pose is None:
                return
            if not self.navigate_client.server_is_ready():
                if self._timed_out("transition_timeout_s"):
                    self.core.acknowledge_coverage_pause(True)
                    self.core.acknowledge_approach(False)
                    self._defer_current("navigate_to_pose_server_timeout")
                return
            if not self.core.acknowledge_coverage_pause(True):
                self._defer_current("coverage_pause_not_acknowledged")
                return
            self._publish_event("approaching", cleaned_fraction=0.0)
            goal = NavigateToPose.Goal()
            goal.pose = self._pose_message(self.current_goal_pose)
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
            ) or self.core.state != ProductCleanState.APPROACHING:
                return
            self.navigation_pending = False
            self.navigation_goal_handle = None
            if not self.core.acknowledge_approach(succeeded):
                self._defer_current("approach_failed")
                return
            self._pre_clean_verify()

        def _pre_clean_verify(self) -> None:
            target = self.current_message
            latest = self.targets.get(str(target.uuid)) if target is not None else None
            if target is None:
                self._defer_current("target_missing_before_clean")
                return
            self._publish_event("pre_clean_verify", cleaned_fraction=0.0)
            still_present = latest is not None and str(latest.track_state) not in {
                "LOST", "REJECTED", "CLEANED"
            }
            identity_stable = (
                latest is not None and str(latest.class_id) == str(target.class_id)
            )
            confidence_healthy = (
                latest is not None
                and float(latest.confidence) >= self.core.minimum_confidence
                and int(latest.observation_count) >= self.core.minimum_observations
            )
            action_verifier_accepts = str(target.cleaning_policy).strip() not in {
                "", "none", "unsupported"
            }
            safety = self._safety(latest or target, path_available=True)
            if not self.core.pre_clean_verify(
                target_still_present=still_present,
                identity_stable=identity_stable,
                class_confidence_healthy=confidence_healthy,
                action_verifier_accepts=action_verifier_accepts,
                safety=safety,
            ):
                self._defer_current("pre_clean_verification_failed")
                return
            self._start_clean_cycle()

        def _start_clean_cycle(self) -> None:
            self._publish_event("cleaning", cleaned_fraction=0.0)
            self.brush_deadline_ns = self._now_ns() + int(
                float(self.get_parameter("brush_run_duration_s").value) * 1e9
            )
            self.phase_started_ns = self._now_ns()

        def _finish_clean_cycle(self) -> None:
            self.brush_deadline_ns = 0
            if not self.core.acknowledge_cleaning(True):
                self._defer_current("cleaning_actuator_failed")
                return
            self._publish_event("post_verify_pending", cleaned_fraction=0.0)
            self.phase_started_ns = self._now_ns()
            self.last_sweep_stamp_ns = 0

        def _observe_post_clean(self, payload: dict) -> None:
            if self.core.state != ProductCleanState.POST_CLEAN_VERIFY:
                return
            regions = payload.get("observed_regions") or []
            if not regions or self.current_message is None:
                return
            sweep = regions[-1]
            sweep_stamp_ns = int(sweep.get("stamp_ns", 0))
            if sweep_stamp_ns <= self.last_sweep_stamp_ns:
                return
            self.last_sweep_stamp_ns = sweep_stamp_ns
            target = self.current_message
            x_m = float(target.map_pose.pose.position.x)
            y_m = float(target.map_pose.pose.position.y)
            in_fov = frustum_record_contains(sweep, x_m, y_m)
            records = payload.get("targets") or []
            record = next(
                (item for item in records if str(item.get("uuid")) == str(target.uuid)),
                None,
            )
            detected = bool(
                record and int(record.get("last_seen_stamp_ns", 0)) >= sweep_stamp_ns
            )
            if str(target.target_type).upper() == "AREA":
                if not in_fov:
                    return
                before = max(float(target.size.x), 1e-9)
                after = (
                    max(0.0, float((record.get("estimated_size_m") or [0.0])[0]))
                    if detected else 0.0
                )
                outcome = self.core.observe_area_post_clean(
                    remaining_ratio=after / before
                )
                if outcome == "RECLEAN":
                    self._publish_event("reclean_queued", cleaned_fraction=0.0)
                    self._publish_event("approaching", cleaned_fraction=0.0)
                    self._publish_event("pre_clean_verify", cleaned_fraction=0.0)
                    self._start_clean_cycle()
                elif outcome == "CLEANED":
                    self._verified_cleaned(1.0 - min(1.0, after / before))
                else:
                    self._defer_current("area_residual_after_retry")
                return
            if self.core.observe_discrete_post_clean(
                target_in_camera_fov=in_fov, detected=detected
            ):
                self._verified_cleaned(1.0)

        def _verified_cleaned(self, cleaned_fraction: float) -> None:
            self._publish_event("cleaned", cleaned_fraction=cleaned_fraction)
            self.resume_request_pending = False
            self.resume_service_accepted = False
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
            valid_state = (
                self.abort_recovery
                and self.core.state in {
                    ProductCleanState.DEFERRED,
                    ProductCleanState.FAILED,
                }
            ) or self.core.state == ProductCleanState.WAITING_RESUME
            if (
                operation_id != self.operation_id
                or self.current_message is None
                or not valid_state
            ):
                return
            self.resume_request_pending = False
            try:
                self.resume_service_accepted = bool(future.result().success)
            except Exception:
                self.resume_service_accepted = False
            if not self.resume_service_accepted:
                if self.abort_recovery:
                    self.core.acknowledge_abort_resume(False)
                else:
                    self.core.acknowledge_coverage_resume(False)
                self._finish_current()

        def _coverage_restored(self) -> bool:
            return (
                self.coverage_state == self.coverage_state_before_pause
                and self.coverage_state not in {"UNKNOWN", "PAUSED", "PAUSING"}
            )

        def _defer_current(self, reason: str) -> None:
            self.brush_deadline_ns = 0
            if not self.deferred_event_sent and self.current_message is not None:
                self._publish_event("deferred", cleaned_fraction=0.0)
                self.deferred_event_sent = True
            self.get_logger().warning(f"spot clean deferred: {reason}")
            if self.core.coverage_paused or self.coverage_state == "PAUSED":
                self.abort_recovery = True
                self.resume_request_pending = False
                self.resume_service_accepted = False
                self.phase_started_ns = self._now_ns()
            else:
                self._finish_current()

        def _finish_current(self) -> None:
            if self.current_message is not None:
                self.handled.add(str(self.current_message.uuid))
            self.current_message = None
            self.current_goal_pose = None
            self.path_request_pending = False
            self.pause_request_pending = False
            self.pause_service_accepted = False
            self.resume_request_pending = False
            self.resume_service_accepted = False
            self.abort_recovery = False
            self.navigation_pending = False
            self.navigation_goal_handle = None
            self.deferred_event_sent = False
            self.brush_deadline_ns = 0
            self.phase_started_ns = self._now_ns()

        def _publish_event(self, result: str, *, cleaned_fraction: float) -> None:
            if self.current_message is None:
                return
            target = self.current_message
            event = CleaningEvent()
            event.header.stamp = self.get_clock().now().to_msg()
            event.header.frame_id = "map"
            event.event_id = f"{target.uuid}:{result}:{self._now_ns()}"
            event.target_uuid = str(target.uuid)
            event.class_id = str(target.class_id)
            event.cleaning_policy = str(target.cleaning_policy)
            event.result = str(result)
            event.cleaned_fraction = float(cleaned_fraction)
            event.brush_enabled_during_event = bool(self.core.brush_enabled)
            event.in_keepout = bool(target.in_keepout)
            event.source_backend = "product_spot_cleaning"
            self.event_publisher.publish(event)

        def _timed_out(self, parameter: str) -> bool:
            return (self._now_ns() - self.phase_started_ns) / 1e9 > float(
                self.get_parameter(parameter).value
            )

        def _step(self) -> None:
            state = self.core.state
            self.brush_publisher.publish(Bool(data=bool(self.core.brush_enabled)))
            if self.current_message is None:
                self._prepare_next_target()
                return
            if state in {
                ProductCleanState.WAITING_SAFE_PAUSE,
                ProductCleanState.APPROACHING,
                ProductCleanState.CLEANING,
            }:
                reason = self.core.motion_safety_reason(
                    self._safety(self.current_message, path_available=True)
                )
                if reason is not None:
                    if state == ProductCleanState.WAITING_SAFE_PAUSE:
                        self.core.acknowledge_coverage_pause(False)
                    elif state == ProductCleanState.APPROACHING:
                        if self.navigation_goal_handle is not None:
                            self.navigation_goal_handle.cancel_goal_async()
                        self.core.acknowledge_approach(False)
                    else:
                        self.core.acknowledge_cleaning(False)
                    self._defer_current(f"runtime_safety_changed:{reason}")
                    return
            if state == ProductCleanState.IDLE:
                self._request_path()
                if self._timed_out("transition_timeout_s"):
                    target = self.current_message
                    if target is not None:
                        self.core.submit(
                            self._target_core(target),
                            self._safety(target, path_available=False),
                        )
                    self._defer_current("compute_path_server_timeout")
                return
            if self.path_request_pending and self._timed_out("transition_timeout_s"):
                self._path_complete(False, self.operation_id)
                return
            if state == ProductCleanState.WAITING_SAFE_PAUSE:
                self._request_pause()
                if self.pause_service_accepted and self.coverage_state == "PAUSED":
                    self._begin_approach()
                elif self._timed_out("transition_timeout_s"):
                    self.core.acknowledge_coverage_pause(False)
                    self._defer_current("coverage_pause_timeout")
                return
            if state == ProductCleanState.APPROACHING:
                if self.navigation_pending and self._timed_out("navigation_timeout_s"):
                    if self.navigation_goal_handle is not None:
                        self.navigation_goal_handle.cancel_goal_async()
                    self._navigation_complete(False, self.operation_id)
                return
            if state == ProductCleanState.CLEANING:
                if self.brush_deadline_ns and self._now_ns() >= self.brush_deadline_ns:
                    self._finish_clean_cycle()
                return
            if state == ProductCleanState.POST_CLEAN_VERIFY:
                if self._timed_out("post_clean_timeout_s"):
                    self._defer_current("post_clean_camera_evidence_timeout")
                return
            if state == ProductCleanState.WAITING_RESUME:
                self._request_resume()
                if self.resume_service_accepted and self._coverage_restored():
                    self.core.acknowledge_coverage_resume(True)
                    self._finish_current()
                elif self._timed_out("transition_timeout_s"):
                    self.core.acknowledge_coverage_resume(False)
                    self._finish_current()
                return
            if self.abort_recovery:
                self._request_resume()
                if self.resume_service_accepted and self._coverage_restored():
                    self.core.acknowledge_abort_resume(True)
                    self._finish_current()
                elif self._timed_out("transition_timeout_s"):
                    self.core.acknowledge_abort_resume(False)
                    self._finish_current()

        def _publish_state(self) -> None:
            payload = self.core.snapshot()
            payload.update({
                "coverage_state": self.coverage_state,
                "queued_target_count": sum(
                    str(item.track_state) == "CONFIRMED"
                    and str(item.uuid) not in self.handled
                    for item in self.targets.values()
                ),
                "current_target_uuid": (
                    str(self.current_message.uuid)
                    if self.current_message is not None else None
                ),
                "ground_truth_control_allowed": False,
                "reobservation_busy": self.reobservation_busy,
            })
            self.state_publisher.publish(
                String(data=json.dumps(payload, sort_keys=True))
            )

    rclpy.init()
    node = ProductSpotCleaningNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.brush_publisher.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
