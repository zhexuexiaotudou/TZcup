"""ROS planner that turns product belief into validated global trajectories."""

from __future__ import annotations

import json
import math
import time

from .formal_observation_core import PublicPlanningMap
from .formal_policy_core import FormalRuntimePolicyCore, runtime_task_config
from .models import KnownTarget, Pose2D


CONTROL_INPUT_TOPICS = (
    "/active_cleaning/ground_dirt_belief",
    "/active_cleaning/garbage_targets",
    "/active_cleaning/observation_ready",
    "/active_cleaning/executor_status",
    "/active_cleaning/grasp_result",
)


def build_grasp_request(
    *,
    target_id: str,
    frame_id: str,
    pose: tuple[float, float, float, float, float, float, float],
    size_m: tuple[float, float, float],
    confidence: float,
) -> str:
    """Serialize a truth-free 3-D perception target for the grasp executor."""

    if not target_id.strip() or not frame_id.strip():
        raise ValueError("target_id and frame_id must be non-empty")
    values = (*pose, *size_m, confidence)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("grasp target geometry must be finite")
    if any(value < 0.020 or value > 0.040 for value in size_m):
        raise ValueError("grasp target dimensions are outside the 30 mm cube tolerance")
    if not 0.50 <= confidence <= 1.0:
        raise ValueError("grasp target confidence is outside the accepted range")
    quaternion_norm = math.sqrt(sum(value * value for value in pose[3:]))
    if quaternion_norm < 1.0e-9 or abs(quaternion_norm - 1.0) > 0.02:
        raise ValueError("grasp target quaternion is not normalized")
    return json.dumps(
        {
            "schema_version": 2,
            "target_id": target_id,
            "frame_id": frame_id,
            "pose": {
                "x_m": pose[0],
                "y_m": pose[1],
                "z_m": pose[2],
                "qx": pose[3],
                "qy": pose[4],
                "qz": pose[5],
                "qw": pose[6],
            },
            "size_m": list(size_m),
            # The random physical material is intentionally not inferable from
            # colour. The bin scale verifies the allowed mass class after the
            # drop; no evaluator truth enters the product grasp request.
            "material": "unknown",
            "confidence": confidence,
            "truth_used": False,
        },
        sort_keys=True,
    )


def main() -> None:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sanitation_perception_interfaces.msg import GarbageTargetArray
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformException, TransformListener

    class FormalPolicyPlanner(Node):
        def __init__(self) -> None:
            super().__init__("formal_active_cleaning_policy_planner")
            for name, default in (
                ("occupancy_map", ""),
                ("mission_geometry", ""),
                ("materialization_contract", ""),
                ("policy_checkpoint", ""),
                ("belief_topic", CONTROL_INPUT_TOPICS[0]),
                ("targets_topic", CONTROL_INPUT_TOPICS[1]),
                ("ready_topic", CONTROL_INPUT_TOPICS[2]),
                ("executor_status_topic", CONTROL_INPUT_TOPICS[3]),
                ("grasp_result_topic", CONTROL_INPUT_TOPICS[4]),
                ("path_topic", "/active_cleaning/trajectory"),
                ("grasp_request_topic", "/active_cleaning/grasp_request"),
                ("cleaning_request_topic", "/active_cleaning/cleaning_requested"),
                ("mission_complete_topic", "/active_cleaning/mission_complete"),
                ("status_topic", "/active_cleaning/planner_status"),
                ("odometry_topic", "/odom"),
                ("map_frame", "map"),
                ("base_frame", "base_link"),
            ):
                self.declare_parameter(name, default)
            self.declare_parameter("planning_resolution_m", 1.0)
            self.declare_parameter("sensing_radius_m", 10.0)
            self.declare_parameter("sensing_fov_rad", math.radians(87.0))
            self.declare_parameter("episode_seed", 7)
            self.declare_parameter("maximum_input_age_sec", 1.5)
            self.declare_parameter("planning_period_sec", 0.5)
            self.declare_parameter("maximum_task_distance_m", 0.0)

            public_map = PublicPlanningMap.load(
                str(self.get_parameter("occupancy_map").value),
                str(self.get_parameter("mission_geometry").value),
                str(self.get_parameter("materialization_contract").value),
            )
            config = runtime_task_config(
                public_map,
                str(self.get_parameter("mission_geometry").value),
                planning_resolution_m=float(
                    self.get_parameter("planning_resolution_m").value
                ),
                sensing_radius_m=float(self.get_parameter("sensing_radius_m").value),
                sensing_fov_rad=float(self.get_parameter("sensing_fov_rad").value),
            )
            self._core = FormalRuntimePolicyCore(
                public_map,
                config,
                str(self.get_parameter("policy_checkpoint").value),
                maximum_task_distance_m=float(
                    self.get_parameter("maximum_task_distance_m").value
                ),
            )
            self._core.reset(episode_seed=int(self.get_parameter("episode_seed").value))
            self._map_frame = str(self.get_parameter("map_frame").value)
            self._base_frame = str(self.get_parameter("base_frame").value)
            self._maximum_age = float(self.get_parameter("maximum_input_age_sec").value)
            if not math.isfinite(self._maximum_age) or self._maximum_age <= 0:
                raise RuntimeError("maximum_input_age_sec must be finite and positive")

            latched = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._path_publisher = self.create_publisher(
                Path, str(self.get_parameter("path_topic").value), 10
            )
            self._grasp_publisher = self.create_publisher(
                String, str(self.get_parameter("grasp_request_topic").value), 10
            )
            self._cleaning_publisher = self.create_publisher(
                Bool, str(self.get_parameter("cleaning_request_topic").value), latched
            )
            self._complete_publisher = self.create_publisher(
                Bool, str(self.get_parameter("mission_complete_topic").value), latched
            )
            self._status_publisher = self.create_publisher(
                DiagnosticArray, str(self.get_parameter("status_topic").value), latched
            )
            self.create_subscription(
                OccupancyGrid,
                str(self.get_parameter("belief_topic").value),
                self._on_belief,
                latched,
            )
            self.create_subscription(
                GarbageTargetArray,
                str(self.get_parameter("targets_topic").value),
                self._on_targets,
                10,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("ready_topic").value),
                self._on_ready,
                latched,
            )
            self.create_subscription(
                DiagnosticArray,
                str(self.get_parameter("executor_status_topic").value),
                self._on_executor_status,
                latched,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("grasp_result_topic").value),
                self._on_grasp_result,
                10,
            )
            self.create_subscription(
                Odometry,
                str(self.get_parameter("odometry_topic").value),
                self._on_odometry,
                20,
            )
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._belief: tuple[int, ...] | None = None
            self._belief_time: float | None = None
            self._targets: tuple[KnownTarget, ...] = ()
            self._target_grasp_observations: dict[str, dict[str, object]] = {}
            self._targets_time: float | None = None
            self._observation_ready = False
            self._ready_time: float | None = None
            self._last_odom_xy: tuple[float, float] | None = None
            self._task_distance = 0.0
            self._step_index = 0
            self._busy = False
            self._executor_seen_active = False
            self._pending_grasp: str | None = None
            self._mission_complete = False
            self._returning_home = False
            self._task_distance_at_completion: float | None = None
            self._cleaning_requested = False
            self._state = "BLOCKED"
            self._reason = "awaiting_product_inputs"
            self.create_timer(
                float(self.get_parameter("planning_period_sec").value), self._plan
            )
            self._publish_status()

        def _on_belief(self, message: OccupancyGrid) -> None:
            if message.header.frame_id != self._map_frame:
                self._block("belief_frame_mismatch")
                return
            expected = self._core.public_map.width * self._core.public_map.height
            if len(message.data) != expected:
                self._block("belief_size_mismatch")
                return
            self._belief = tuple(int(value) for value in message.data)
            self._belief_time = time.monotonic()

        def _on_targets(self, message: GarbageTargetArray) -> None:
            if message.header.frame_id != self._map_frame:
                self._block("target_frame_mismatch")
                return
            self._targets = tuple(
                KnownTarget(
                    target_id=item.uuid,
                    x=float(item.map_pose.pose.position.x),
                    y=float(item.map_pose.pose.position.y),
                    cleared=item.track_state.upper() in {"CLEARED", "IN_BIN"},
                    attempts=0,
                )
                for item in message.targets
            )
            self._target_grasp_observations = {
                str(item.uuid): {
                    "frame_id": str(item.header.frame_id or message.header.frame_id),
                    "pose": (
                        float(item.map_pose.pose.position.x),
                        float(item.map_pose.pose.position.y),
                        float(item.map_pose.pose.position.z),
                        float(item.map_pose.pose.orientation.x),
                        float(item.map_pose.pose.orientation.y),
                        float(item.map_pose.pose.orientation.z),
                        float(item.map_pose.pose.orientation.w),
                    ),
                    "size_m": (
                        float(item.size.x),
                        float(item.size.y),
                        float(item.size.z),
                    ),
                    "confidence": float(item.confidence),
                }
                for item in message.targets
            }
            self._targets_time = time.monotonic()

        def _on_ready(self, message: Bool) -> None:
            self._observation_ready = bool(message.data)
            self._ready_time = time.monotonic()
            if not message.data:
                self._block("product_observation_not_ready")

        def _on_odometry(self, message: Odometry) -> None:
            point = (
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
            )
            if not all(math.isfinite(value) for value in point):
                return
            if self._last_odom_xy is not None:
                increment = math.dist(self._last_odom_xy, point)
                if increment < 2.0:
                    self._task_distance += increment
            self._last_odom_xy = point

        def _on_executor_status(self, message: DiagnosticArray) -> None:
            rows = [row for row in message.status if row.name == "formal_active_cleaning_trajectory_executor"]
            if not rows:
                return
            state = rows[-1].message
            if state in {"SUBMITTING", "EXECUTING", "CANCELING"}:
                self._executor_seen_active = True
                self._busy = True
            elif self._busy and (
                self._executor_seen_active
                or state in {"REJECTED", "FAILED", "CANCELED"}
            ) and state in {"IDLE", "SUCCEEDED", "REJECTED", "FAILED", "CANCELED", "BLOCKED"}:
                self._busy = False
                self._executor_seen_active = False
                self._set_cleaning(False)
                self._state = "IDLE" if state == "SUCCEEDED" else "REPLAN"
                self._reason = f"executor_{state.lower()}"
                self._publish_status()

        def _on_grasp_result(self, message: String) -> None:
            try:
                value = json.loads(message.data)
                target_id = str(value["target_id"])
                verified = value["verified_in_bin"] is True
            except (json.JSONDecodeError, KeyError, TypeError, ValueError):
                self._block("invalid_grasp_result")
                return
            if self._pending_grasp != target_id:
                self._block("unexpected_grasp_result_target")
                return
            self._core.mark_grasp_result(target_id, verified_in_bin=verified)
            self._pending_grasp = None
            self._busy = False
            self._state = "IDLE" if verified else "REPLAN"
            self._reason = "grasp_verified_in_bin" if verified else "grasp_not_verified"
            self._publish_status()

        def _inputs_fresh(self) -> bool:
            now = time.monotonic()
            stamps = (self._belief_time, self._targets_time, self._ready_time)
            return (
                self._observation_ready
                and all(stamp is not None for stamp in stamps)
                and all(0.0 <= now - float(stamp) <= self._maximum_age for stamp in stamps)
            )

        def _map_pose(self) -> Pose2D | None:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._map_frame,
                    self._base_frame,
                    Time(),
                )
            except TransformException:
                return None
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
            )
            values = (translation.x, translation.y, yaw)
            return Pose2D(*values) if all(math.isfinite(value) for value in values) else None

        def _plan(self) -> None:
            # This is also the coordinator watchdog heartbeat.  If the planner
            # dies, the cleaning actuator coordinator times this request out
            # and commands lift/brush/pump safe.
            self._cleaning_publisher.publish(Bool(data=self._cleaning_requested))
            if self._mission_complete or self._busy:
                return
            if not self._inputs_fresh() or self._belief is None:
                self._block("product_inputs_stale_or_not_ready")
                return
            pose = self._map_pose()
            if pose is None:
                self._block("map_to_base_transform_unavailable")
                return
            observation = self._core.observation(
                belief_values=self._belief,
                pose=pose,
                targets=self._targets,
                step_index=self._step_index,
                task_distance=self._task_distance,
            )
            uncleared = [item for item in observation.belief.known_targets if not item.cleared]
            dirty = any(value > 0.0 for value in observation.belief.known_ground_dirt)
            task_cleaning_complete = (
                observation.observed_ratio >= 0.95 and not uncleared and not dirty
            )
            if task_cleaning_complete:
                if not self._returning_home:
                    self._returning_home = True
                    self._task_distance_at_completion = self._task_distance
                return_decision = self._core.return_home(observation)
                if return_decision.kind == "trajectory":
                    stamp = self.get_clock().now().to_msg()
                    path = Path()
                    path.header.stamp = stamp
                    path.header.frame_id = self._map_frame
                    for item in return_decision.trajectory:
                        pose_message = PoseStamped()
                        pose_message.header = path.header
                        pose_message.pose.position.x = item.x
                        pose_message.pose.position.y = item.y
                        pose_message.pose.orientation.z = math.sin(item.yaw / 2.0)
                        pose_message.pose.orientation.w = math.cos(item.yaw / 2.0)
                        path.poses.append(pose_message)
                    self._set_cleaning(False)
                    self._path_publisher.publish(path)
                    self._busy = True
                    self._executor_seen_active = False
                    self._state = "RETURNING_HOME"
                    self._reason = return_decision.reason
                    self._publish_status(observation.observed_ratio)
                    return
                if return_decision.kind != "home_reached":
                    self._block(return_decision.reason)
                    return
                self._mission_complete = True
                self._state = "COMPLETE"
                self._reason = "task_complete_and_fixed_start_pose_reached"
                self._set_cleaning(False)
                self._complete_publisher.publish(Bool(data=True))
                self._publish_status(observation.observed_ratio)
                return
            decision = self._core.decide(observation)
            self._step_index += 1
            if decision.kind == "grasp":
                target = next(
                    item for item in self._targets if item.target_id == decision.grasp_target_id
                )
                self._pending_grasp = target.target_id
                self._busy = True
                self._set_cleaning(False)
                observation = self._target_grasp_observations.get(target.target_id)
                if observation is None:
                    self._pending_grasp = None
                    self._busy = False
                    self._block("grasp_target_has_no_3d_perception_observation")
                    return
                try:
                    request = build_grasp_request(
                        target_id=target.target_id,
                        frame_id=str(observation["frame_id"]),
                        pose=observation["pose"],  # type: ignore[arg-type]
                        size_m=observation["size_m"],  # type: ignore[arg-type]
                        confidence=float(observation["confidence"]),
                    )
                except (KeyError, TypeError, ValueError) as exc:
                    self._pending_grasp = None
                    self._busy = False
                    self._block(f"invalid_3d_grasp_target:{exc}")
                    return
                self._grasp_publisher.publish(String(data=request))
                self._state = "WAITING_GRASP"
                self._reason = decision.reason
            elif decision.kind == "trajectory":
                stamp = self.get_clock().now().to_msg()
                path = Path()
                path.header.stamp = stamp
                path.header.frame_id = self._map_frame
                for item in decision.trajectory:
                    pose_message = PoseStamped()
                    pose_message.header = path.header
                    pose_message.pose.position.x = item.x
                    pose_message.pose.position.y = item.y
                    pose_message.pose.orientation.z = math.sin(item.yaw / 2.0)
                    pose_message.pose.orientation.w = math.cos(item.yaw / 2.0)
                    path.poses.append(pose_message)
                self._set_cleaning(decision.clean_ground)
                self._path_publisher.publish(path)
                self._busy = True
                self._executor_seen_active = False
                self._state = "WAITING_EXECUTOR"
                self._reason = decision.reason
            else:
                self._state = "BLOCKED"
                self._reason = decision.reason
            self._publish_status(decision.observed_ratio)

        def _block(self, reason: str) -> None:
            self._state = "BLOCKED"
            self._reason = reason
            self._set_cleaning(False)
            self._publish_status()

        def _set_cleaning(self, requested: bool) -> None:
            self._cleaning_requested = bool(requested)
            self._cleaning_publisher.publish(Bool(data=self._cleaning_requested))

        def _publish_status(self, observed_ratio: float = 0.0) -> None:
            status = DiagnosticStatus()
            status.name = "formal_active_cleaning_policy_planner"
            status.hardware_id = "frozen_truth_free_q_policy"
            status.level = DiagnosticStatus.OK if self._state in {
                "IDLE", "WAITING_EXECUTOR", "WAITING_GRASP", "RETURNING_HOME", "COMPLETE"
            } else DiagnosticStatus.ERROR
            status.message = self._state
            status.values = [
                KeyValue(key="reason", value=self._reason),
                KeyValue(key="truth_used_for_control", value="false"),
                KeyValue(key="product_inputs_fresh", value=str(self._inputs_fresh()).lower()),
                KeyValue(key="observed_ratio", value=f"{observed_ratio:.6f}"),
                KeyValue(
                    key="task_distance_m_excluding_return",
                    value=f"{(self._task_distance_at_completion if self._task_distance_at_completion is not None else self._task_distance):.6f}",
                ),
                KeyValue(
                    key="return_distance_m",
                    value=f"{(0.0 if self._task_distance_at_completion is None else max(0.0, self._task_distance - self._task_distance_at_completion)):.6f}",
                ),
                KeyValue(key="returning_home", value=str(self._returning_home).lower()),
                KeyValue(key="pending_grasp", value=self._pending_grasp or ""),
            ]
            message = DiagnosticArray()
            message.header.stamp = self.get_clock().now().to_msg()
            message.status = [status]
            self._status_publisher.publish(message)

    rclpy.init()
    node = None
    try:
        node = FormalPolicyPlanner()
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
