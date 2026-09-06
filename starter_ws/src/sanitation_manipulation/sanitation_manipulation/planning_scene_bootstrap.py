"""Persistent, fail-closed bootstrap for the configured MoveIt ground box.

This node is deliberately separate from the perceived-cube lifecycle in
``formal_grasp_executor``.  It injects only the configured static ground box,
then continuously verifies a read-only ``GetPlanningScene`` snapshot.  A
reset, missing object, unknown revision, or unavailable robot-link TF changes
the latched readiness topic to false before any reinjection attempt.
"""

from __future__ import annotations

from pathlib import Path
import time
from typing import Any

from .planning_scene_core import (
    PlanningSceneReadback,
    SceneObjectReadback,
    load_planning_scene_config,
    next_scene_revision,
    parse_scene_revision,
    planning_virtual_joint_from_srdf,
    validate_scene_readback,
)


def main() -> None:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, PlanningScene, PlanningSceneComponents
    from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
    from rcl_interfaces.srv import GetParameters
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from shape_msgs.msg import SolidPrimitive
    from std_msgs.msg import Bool
    from tf2_ros import Buffer, TransformException, TransformListener

    class PlanningSceneBootstrap(Node):
        def __init__(self) -> None:
            super().__init__("moveit_planning_scene_bootstrap")
            default_config = str(
                Path(get_package_share_directory("sanitation_manipulation"))
                / "config"
                / "bin_and_scene.yaml"
            )
            self.declare_parameter("config_file", default_config)
            self.declare_parameter("base_frame", "base_link")
            self.declare_parameter("apply_planning_scene_service", "/apply_planning_scene")
            self.declare_parameter("get_planning_scene_service", "/get_planning_scene")
            self.declare_parameter("move_group_node", "/move_group")
            self.declare_parameter("planning_scene_ready_topic", "/manipulation/planning_scene_ready")
            self.declare_parameter("status_topic", "/manipulation/planning_scene_bootstrap_status")
            self.declare_parameter("poll_period_sec", 0.5)
            self.declare_parameter("service_timeout_sec", 5.0)
            self._config = load_planning_scene_config(
                str(self.get_parameter("config_file").value)
            )
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._ready_pub = self.create_publisher(
                Bool, str(self.get_parameter("planning_scene_ready_topic").value), qos
            )
            self._status_pub = self.create_publisher(
                DiagnosticArray, str(self.get_parameter("status_topic").value), 10
            )
            # _tick waits for asynchronous MoveIt responses. Its timer must
            # not re-enter, while client completion callbacks must remain
            # schedulable by the multi-threaded executor.
            self._timer_callback_group = MutuallyExclusiveCallbackGroup()
            self._service_callback_group = ReentrantCallbackGroup()
            self._apply = self.create_client(
                ApplyPlanningScene,
                str(self.get_parameter("apply_planning_scene_service").value),
                callback_group=self._service_callback_group,
            )
            self._get = self.create_client(
                GetPlanningScene,
                str(self.get_parameter("get_planning_scene_service").value),
                callback_group=self._service_callback_group,
            )
            move_group = str(self.get_parameter("move_group_node").value).rstrip("/")
            self._semantic = self.create_client(
                GetParameters,
                f"{move_group}/get_parameters",
                callback_group=self._service_callback_group,
            )
            self._tf_buffer = Buffer()
            # Jazzy TransformListener owns a ReentrantCallbackGroup for /tf
            # and /tf_static. Keep it in this executor (rather than adding
            # this node to another executor) and reject an incompatible API.
            self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
            self._tf_listener_group = self._tf_listener.group
            if not isinstance(self._tf_listener_group, ReentrantCallbackGroup):
                raise RuntimeError("transform_listener_requires_reentrant_callback_group")
            self._ready = False
            self._reason = "bootstrap_not_yet_verified"
            self._last_revision = 0
            self._publish_ready(False, self._reason)
            self._timer = self.create_timer(
                float(self.get_parameter("poll_period_sec").value),
                self._tick,
                callback_group=self._timer_callback_group,
            )

        def _publish_ready(self, ready: bool, reason: str) -> None:
            changed = ready != self._ready or reason != self._reason
            self._ready = ready
            self._reason = reason
            self._ready_pub.publish(Bool(data=ready))
            if not changed:
                return
            status = DiagnosticStatus()
            status.name = "moveit_planning_scene_bootstrap"
            status.hardware_id = "moveit_world_ground_contract"
            status.level = DiagnosticStatus.OK if ready else DiagnosticStatus.ERROR
            status.message = "READY" if ready else "NOT_READY"
            status.values = [
                KeyValue(key="reason", value=reason),
                KeyValue(key="scene_revision_prefix", value=self._config.scene_revision_prefix),
                KeyValue(key="ground_id", value=self._config.ground.object_id),
                KeyValue(key="ground_frame", value=self._config.ground.frame_id),
                KeyValue(key="ground_top_height_m", value=str(self._config.ground.top_height_m)),
            ]
            message = DiagnosticArray()
            message.header.stamp = self.get_clock().now().to_msg()
            message.status = [status]
            self._status_pub.publish(message)

        def _wait(self, future: Any, label: str) -> Any:
            deadline = time.monotonic() + float(self.get_parameter("service_timeout_sec").value)
            while rclpy.ok() and not future.done():
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"{label}_timeout")
                time.sleep(0.01)
            if not future.done():
                raise RuntimeError(f"{label}_interrupted")
            return future.result()

        def _require_link_tf(self) -> None:
            base = str(self.get_parameter("base_frame").value)
            frames = (self._config.ground.frame_id, *self._config.required_robot_links)
            for frame in frames:
                try:
                    # Lookup, rather than a string-only URDF assertion, makes
                    # unavailable TF a hard runtime failure.
                    self._tf_buffer.lookup_transform(base, frame, rclpy.time.Time())
                except TransformException as exc:
                    raise RuntimeError(f"required_robot_or_ground_tf_unavailable:{frame}") from exc

        def _require_actual_planning_frame(self) -> None:
            """Read the live MoveIt semantic model, never infer map from TF."""
            if not self._semantic.wait_for_service(timeout_sec=0.1):
                raise RuntimeError("move_group_get_parameters_service_unavailable")
            request = GetParameters.Request()
            request.names = ["robot_description_semantic"]
            response = self._wait(
                self._semantic.call_async(request), "get_move_group_robot_description_semantic"
            )
            if response is None or len(response.values) != 1:
                raise RuntimeError("move_group_robot_description_semantic_missing")
            name, planning_frame, child_link = planning_virtual_joint_from_srdf(
                response.values[0].string_value
            )
            if (
                planning_frame != self._config.planning_frame_id
                or name != self._config.planning_virtual_joint_name
                or child_link != self._config.planning_virtual_joint_child_link
            ):
                raise RuntimeError("configured_ground_frame_does_not_match_moveit_planning_frame")

        def _ground_collision(self) -> CollisionObject:
            ground = self._config.ground
            item = CollisionObject()
            item.id = ground.object_id
            item.header.frame_id = ground.frame_id
            item.operation = CollisionObject.ADD
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = list(ground.size_m)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = ground.pose_xyz_m
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = ground.pose_xyzw
            item.primitives = [primitive]
            item.primitive_poses = [pose]
            return item

        def _ground_acm(self, current: Any) -> Any:
            """Preserve existing ACM entries but make ground wheel-only."""
            names = list(current.entry_names)
            for link in (self._config.ground.object_id, *self._config.ground.allowed_contact_links):
                if link not in names:
                    names.append(link)
            prior: set[tuple[str, str]] = set()
            for row, first in enumerate(current.entry_names):
                if row >= len(current.entry_values):
                    continue
                for column, enabled in enumerate(current.entry_values[row].enabled):
                    if enabled and column < len(current.entry_names):
                        prior.add((first, current.entry_names[column]))
            ground = self._config.ground.object_id
            wheel_links = set(self._config.ground.allowed_contact_links)
            matrix = type(current)()
            matrix.entry_names = names
            matrix.default_entry_names = list(current.default_entry_names)
            matrix.default_entry_values = list(current.default_entry_values)
            matrix.entry_values = []
            for first in names:
                row = AllowedCollisionEntry()
                row.enabled = []
                for second in names:
                    enabled = (first, second) in prior or (second, first) in prior
                    if ground in (first, second):
                        other = second if first == ground else first
                        enabled = other in wheel_links
                    row.enabled.append(enabled)
                matrix.entry_values.append(row)
            return matrix

        def _apply_ground(self, revision: str) -> None:
            if not self._apply.wait_for_service(timeout_sec=0.1):
                raise RuntimeError("apply_planning_scene_service_unavailable")
            if not self._get.wait_for_service(timeout_sec=0.1):
                raise RuntimeError("get_planning_scene_service_unavailable")
            current_request = GetPlanningScene.Request()
            current_request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            current = self._wait(self._get.call_async(current_request), "get_current_allowed_collision_matrix")
            if current is None:
                raise RuntimeError("get_current_allowed_collision_matrix_empty")
            scene = PlanningScene()
            scene.name = revision
            scene.is_diff = True
            scene.robot_state.is_diff = True
            # Only a configured *world* object is created here.  Required
            # robot links are verified through TF and remain in the URDF.
            scene.world.collision_objects = [self._ground_collision()]
            scene.allowed_collision_matrix = self._ground_acm(current.scene.allowed_collision_matrix)
            request = ApplyPlanningScene.Request()
            request.scene = scene
            response = self._wait(self._apply.call_async(request), "apply_configured_ground")
            if response is None or response.success is not True:
                raise RuntimeError("apply_configured_ground_failed")

        @staticmethod
        def _shape_name(value: int, primitive: Any) -> str:
            if value == primitive.BOX:
                return "BOX"
            return f"UNKNOWN_{value}"

        def _readback(self) -> PlanningSceneReadback:
            if not self._get.wait_for_service(timeout_sec=0.1):
                raise RuntimeError("get_planning_scene_service_unavailable")
            request = GetPlanningScene.Request()
            request.components.components = (
                PlanningSceneComponents.SCENE_SETTINGS
                | PlanningSceneComponents.ROBOT_STATE
                | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            )
            response = self._wait(self._get.call_async(request), "get_planning_scene")
            if response is None:
                raise RuntimeError("get_planning_scene_empty_response")
            objects: list[SceneObjectReadback] = []
            for item in response.scene.world.collision_objects:
                if not item.primitives or not item.primitive_poses:
                    continue
                primitive, pose = item.primitives[0], item.primitive_poses[0]
                objects.append(
                    SceneObjectReadback(
                        object_id=item.id,
                        frame_id=item.header.frame_id,
                        shape_type=self._shape_name(primitive.type, primitive),
                        dimensions_m=tuple(float(value) for value in primitive.dimensions),
                        pose_xyz_m=(float(pose.position.x), float(pose.position.y), float(pose.position.z)),
                        pose_xyzw=(
                            float(pose.orientation.x),
                            float(pose.orientation.y),
                            float(pose.orientation.z),
                            float(pose.orientation.w),
                        ),
                    )
                )
            revision = response.scene.name.strip() or None
            pairs: list[tuple[str, str]] = []
            acm = response.scene.allowed_collision_matrix
            for row, first in enumerate(acm.entry_names):
                if row >= len(acm.entry_values):
                    continue
                for column, enabled in enumerate(acm.entry_values[row].enabled):
                    if enabled and column < len(acm.entry_names):
                        pairs.append((first, acm.entry_names[column]))
            return PlanningSceneReadback(
                revision=revision,
                world_objects=tuple(objects),
                allowed_collision_pairs=tuple(pairs),
            )

        def _tick(self) -> None:
            try:
                self._require_actual_planning_frame()
                self._require_link_tf()
                readback = self._readback()
                try:
                    self._last_revision = validate_scene_readback(self._config, readback)
                except (RuntimeError, ValueError):
                    # A reset/lost object is published as not-ready before the
                    # only permitted repair (re-applying the configured box).
                    self._publish_ready(False, "planning_scene_readback_not_ready")
                    observed = parse_scene_revision(self._config, readback.revision)
                    base_revision = max(self._last_revision, observed or 0)
                    source = (
                        f"{self._config.scene_revision_prefix}:{base_revision}"
                        if base_revision else None
                    )
                    self._apply_ground(next_scene_revision(self._config, source))
                    self._require_link_tf()
                    self._last_revision = validate_scene_readback(self._config, self._readback())
                self._publish_ready(True, "configured_ground_readback_verified")
            except Exception as exc:
                self._publish_ready(False, str(exc))

    rclpy.init()
    node = PlanningSceneBootstrap()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
