"""ROS adapter that gates planner paths before Nav2 FollowPath execution."""

from __future__ import annotations

import math
import time

from .formal_trajectory_core import FormalTrajectoryGate, PathPose


CONTROL_INPUT_TOPICS = (
    "/active_cleaning/trajectory",
    "/active_cleaning/cancel",
    "/safety/actuators_enabled",
)
NAVIGATION_ACTION = "/follow_path"


def node_class():
    """Load ROS dependencies and expose the node for real transport tests."""
    import rclpy
    from action_msgs.msg import GoalStatus
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav2_msgs.action import FollowPath
    from nav_msgs.msg import Path
    from rclpy.action import ActionClient
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool

    class FormalTrajectoryExecutor(Node):
        def __init__(self) -> None:
            super().__init__("formal_active_cleaning_trajectory_executor")
            self.declare_parameter("mission_geometry", "")
            self.declare_parameter("path_topic", CONTROL_INPUT_TOPICS[0])
            self.declare_parameter("cancel_topic", CONTROL_INPUT_TOPICS[1])
            self.declare_parameter("safety_permit_topic", CONTROL_INPUT_TOPICS[2])
            self.declare_parameter("follow_path_action", NAVIGATION_ACTION)
            self.declare_parameter("status_topic", "/active_cleaning/executor_status")
            self.declare_parameter("max_segment_length", 0.50)
            self.declare_parameter("max_path_length", 10000.0)
            self.declare_parameter("max_pose_count", 100000)
            self.declare_parameter("max_safety_age_sec", 0.50)
            self.declare_parameter("controller_id", "FollowPath")
            self.declare_parameter("goal_checker_id", "general_goal_checker")
            for name, default in (
                ("goal_response_timeout_sec", 5.0),
                ("execution_timeout_sec", 1800.0),
                ("cancel_timeout_sec", 5.0),
            ):
                self.declare_parameter(name, default)
                value = float(self.get_parameter(name).value)
                if not math.isfinite(value) or value <= 0:
                    raise RuntimeError(f"{name} must be finite and positive")
                setattr(self, "_" + name, value)
            self._gate = FormalTrajectoryGate.from_mission_geometry(
                str(self.get_parameter("mission_geometry").value),
                max_segment_length=float(
                    self.get_parameter("max_segment_length").value
                ),
                max_path_length=float(self.get_parameter("max_path_length").value),
                max_pose_count=int(self.get_parameter("max_pose_count").value),
            )
            self._max_safety_age = float(
                self.get_parameter("max_safety_age_sec").value
            )
            if not math.isfinite(self._max_safety_age) or self._max_safety_age <= 0:
                raise RuntimeError("max_safety_age_sec must be finite and positive")
            self._safety_permitted = False
            self._last_safety_time: float | None = None
            self._goal_handle = None
            self._goal_pending = False
            self._cancel_requested = False
            self._cancel_sent = False
            self._goal_started = None
            self._cancel_started = None
            self._fatal_reason = ""
            self._terminal_confirmed = True
            self._request_stamp = ""
            self._state = "BLOCKED"
            self._reason = "safety_permit_not_received"
            self._last_path_length = 0.0
            self._action_client = ActionClient(
                self,
                FollowPath,
                str(self.get_parameter("follow_path_action").value),
            )
            latched = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._status_publisher = self.create_publisher(
                DiagnosticArray,
                str(self.get_parameter("status_topic").value),
                latched,
            )
            self.create_subscription(
                Path,
                str(self.get_parameter("path_topic").value),
                self._on_path,
                10,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("cancel_topic").value),
                self._on_cancel,
                10,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("safety_permit_topic").value),
                self._on_safety,
                latched,
            )
            self.create_timer(
                0.10, self._watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME)
            )
            self._publish_status()

        def _safety_is_fresh_and_permitted(self) -> bool:
            if self._last_safety_time is None or not self._safety_permitted:
                return False
            age = time.monotonic() - self._last_safety_time
            return 0.0 <= age <= self._max_safety_age

        def _on_safety(self, message: Bool) -> None:
            self._safety_permitted = bool(message.data)
            self._last_safety_time = time.monotonic()
            if not self._safety_permitted:
                self._fail_closed("safety_inhibited")
            elif (self._goal_handle is None and not self._goal_pending
                  and not self._fatal_reason and self._state == "BLOCKED"):
                self._state = "IDLE"
                self._reason = "ready"
                self._publish_status()

        def _on_cancel(self, message: Bool) -> None:
            if message.data:
                self._fail_closed("operator_or_planner_cancel")

        def _on_path(self, message: Path) -> None:
            if self._fatal_reason:
                self._publish_status()
                return
            if self._goal_handle is not None or self._goal_pending:
                # A second request must not overwrite the active goal's status.
                self.get_logger().error("trajectory ignored: executor_busy")
                return
            self._request_stamp = f"{message.header.stamp.sec}:{message.header.stamp.nanosec}"
            if not self._safety_is_fresh_and_permitted():
                self._reject("safety_not_permitted_or_stale")
                return
            if not self._action_client.server_is_ready():
                self._reject("follow_path_server_unavailable")
                return
            poses = tuple(
                PathPose(
                    x=float(item.pose.position.x),
                    y=float(item.pose.position.y),
                    quaternion=(
                        float(item.pose.orientation.x),
                        float(item.pose.orientation.y),
                        float(item.pose.orientation.z),
                        float(item.pose.orientation.w),
                    ),
                    frame_id=item.header.frame_id,
                )
                for item in message.poses
            )
            decision = self._gate.validate(
                path_frame_id=message.header.frame_id,
                poses=poses,
            )
            self._last_path_length = decision.path_length
            if not decision.accepted:
                self._reject(decision.reason)
                return
            goal = FollowPath.Goal()
            goal.path = message
            goal.controller_id = str(self.get_parameter("controller_id").value)
            goal.goal_checker_id = str(self.get_parameter("goal_checker_id").value)
            self._goal_pending = True
            self._cancel_requested = False
            self._cancel_sent = False
            self._cancel_started = None
            self._goal_started = time.monotonic()
            self._terminal_confirmed = False
            self._state = "SUBMITTING"
            self._reason = "validated_path"
            self._publish_status()
            try:
                future = self._action_client.send_goal_async(goal)
                future.add_done_callback(self._on_goal_response)
            except Exception as exc:
                # A transport exception does not prove the server never got it.
                self._latch_failure(f"goal_submission_exception:{type(exc).__name__}")

        def _on_goal_response(self, future) -> None:
            try:
                handle = future.result()
            except Exception as exc:  # ROS future transports executor exceptions.
                self._latch_failure(f"goal_response_exception:{type(exc).__name__}")
                return
            self._goal_pending = False
            if not handle.accepted:
                self._goal_handle = None
                self._terminal_confirmed = True
                self._state = "FAILED"
                self._reason = self._fatal_reason or "follow_path_goal_rejected"
                self._publish_status()
                return
            self._goal_handle = handle
            # Subscribe BEFORE canceling: cancellation acknowledgement is not
            # terminal evidence. Late accepted goals must also be drained.
            try:
                handle.get_result_async().add_done_callback(self._on_result)
            except Exception as exc:
                self._latch_failure(f"result_subscription_exception:{type(exc).__name__}")
            if self._goal_handle is None:
                return  # An already-resolved result callback ran synchronously.
            if self._cancel_requested or self._fatal_reason or not self._safety_is_fresh_and_permitted():
                self._cancel_active_goal("cancel_before_goal_acceptance")
                return
            self._state = "EXECUTING"
            self._reason = "follow_path_active"
            self._publish_status()

        def _on_result(self, future) -> None:
            try:
                wrapped = future.result()
                status = int(wrapped.status)
                error_code = int(getattr(wrapped.result, "error_code", 0))
            except Exception as exc:  # ROS future transports executor exceptions.
                self._latch_failure(f"result_exception:{type(exc).__name__}")
                self._cancel_active_goal(self._fatal_reason)
                return
            if status not in {
                GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_ABORTED,
            }:
                self._latch_failure(f"nonterminal_result_status:{status}")
                self._cancel_active_goal(self._fatal_reason)
                return
            self._goal_handle = None
            self._terminal_confirmed = True
            was_canceled = self._cancel_requested
            self._cancel_requested = False
            self._cancel_started = None
            if self._fatal_reason:
                self._state = "FAILED"
                self._reason = self._fatal_reason
            elif was_canceled:
                self._state = "CANCELED"
                self._reason = "cancel_requested_goal_terminated"
            elif status == GoalStatus.STATUS_SUCCEEDED and error_code == 0:
                self._state = "SUCCEEDED"
                self._reason = "follow_path_succeeded"
            elif status == GoalStatus.STATUS_CANCELED:
                self._state = "CANCELED"
                self._reason = "follow_path_canceled"
            else:
                self._state = "FAILED"
                self._reason = f"follow_path_failed:status={status}:error={error_code}"
            self._publish_status()

        def _reject(self, reason: str) -> None:
            self._state = "REJECTED"
            self._reason = reason
            self.get_logger().error(f"trajectory rejected: {reason}")
            self._publish_status()

        def _cancel_active_goal(self, reason: str) -> None:
            self._cancel_requested = True
            if self._cancel_sent:
                return
            self._state = "CANCELING"
            self._reason = self._fatal_reason or reason
            if self._goal_handle is not None:
                self._cancel_sent = True
                self._cancel_started = time.monotonic()
                try:
                    handle = self._goal_handle
                    handle.cancel_goal_async().add_done_callback(
                        lambda future: self._on_cancel_response(future, handle)
                    )
                except Exception as exc:
                    self._latch_failure(f"cancel_exception:{type(exc).__name__}")
            self._publish_status()

        def _on_cancel_response(self, future, handle) -> None:
            if self._terminal_confirmed or self._goal_handle is not handle:
                return
            try:
                response = future.result()
                own_id = bytes(self._goal_handle.goal_id.uuid)
                accepted = response.return_code == 0 and any(
                    bytes(goal.goal_id.uuid) == own_id
                    for goal in response.goals_canceling
                )
            except Exception as exc:
                self._latch_failure(f"cancel_response_exception:{type(exc).__name__}")
                return
            if not accepted:
                self._latch_failure("cancel_not_acknowledged_for_active_goal")

        def _latch_failure(self, reason: str) -> None:
            self._fatal_reason = self._fatal_reason or reason
            self._state = "FAILED"
            self._reason = self._fatal_reason
            self._publish_status()

        def _fail_closed(self, reason: str) -> None:
            self._cancel_requested = True
            if self._goal_handle is not None:
                self._cancel_active_goal(reason)
            elif self._goal_pending:
                self._state = "CANCEL_PENDING_ACCEPTANCE"
                self._reason = reason
                self._publish_status()
            else:
                self._state = "FAILED" if self._fatal_reason else "BLOCKED"
                self._reason = self._fatal_reason or reason
                self._publish_status()

        def _watchdog(self) -> None:
            now = time.monotonic()
            if self._goal_pending and now - self._goal_started >= self._goal_response_timeout_sec:
                self._cancel_requested = True
                self._latch_failure("goal_response_timeout")
            if self._goal_handle is not None and not self._cancel_sent:
                if now - self._goal_started >= self._execution_timeout_sec:
                    self._latch_failure("execution_timeout")
                    self._cancel_active_goal(self._fatal_reason)
            if self._cancel_started is not None and now - self._cancel_started >= self._cancel_timeout_sec:
                self._latch_failure("cancel_terminal_timeout")
            if not self._safety_is_fresh_and_permitted():
                self._fail_closed("safety_not_permitted_or_stale")
            self._publish_status()

        def _publish_status(self) -> None:
            status = DiagnosticStatus()
            status.name = "formal_active_cleaning_trajectory_executor"
            status.hardware_id = "nav2_follow_path_safety_chain"
            status.level = (
                DiagnosticStatus.OK
                if self._state in {"IDLE", "EXECUTING", "SUCCEEDED"}
                else DiagnosticStatus.ERROR
            )
            status.message = self._state
            status.values = [
                KeyValue(key="reason", value=self._reason),
                KeyValue(key="terminal_confirmed", value=str(self._terminal_confirmed).lower()),
                KeyValue(key="restart_required", value=str(bool(self._fatal_reason)).lower()),
                KeyValue(key="request_stamp", value=self._request_stamp),
                KeyValue(
                    key="safety_fresh_and_permitted",
                    value=str(self._safety_is_fresh_and_permitted()).lower(),
                ),
                KeyValue(key="nav2_action", value=NAVIGATION_ACTION),
                KeyValue(key="command_chain", value="FollowPath_to_cmd_vel_gate"),
                KeyValue(key="path_length_m", value=f"{self._last_path_length:.6f}"),
            ]
            message = DiagnosticArray()
            message.header.stamp = self.get_clock().now().to_msg()
            message.status = [status]
            self._status_publisher.publish(message)

        def destroy_node(self) -> bool:
            if not rclpy.ok():
                return True
            self._fail_closed("node_shutdown")
            self._action_client.destroy()
            return super().destroy_node()

    return FormalTrajectoryExecutor


def main() -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init()
    node = None
    try:
        node = node_class()()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
