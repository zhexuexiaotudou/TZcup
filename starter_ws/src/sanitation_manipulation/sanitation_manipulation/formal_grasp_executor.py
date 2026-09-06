"""Target-conditioned MoveIt grasp executor for the formal vehicle.

The executor accepts only a truth-free perception track carrying a 3-D pose,
measured dimensions and material. MoveIt ``MoveGroup``/IK and Cartesian-path
interfaces generate collision-checked motion. MTC is not claimed. Physical
dual-finger contact, attachment, wrist re-observation and dry-bin mass/count
remain independent fail-closed acceptance gates.
"""

from __future__ import annotations

import hashlib
import json
import math
import threading
import time
from typing import Any

from .formal_grasp_core import (
    ARM_JOINTS,
    GRIPPER_JOINT,
    STORAGE_JOINTS,
    TRANSPORT,
    DryBinSample,
    GraspRequest,
    ParkingObservation,
    TargetGeometry,
    ToolPose,
    build_target_conditioned_waypoints,
    material_for_measured_mass,
    validate_wrist_recheck,
    verify_bin_increment,
)
from .planning_scene_core import (
    PlanningSceneReadback,
    SceneObjectReadback,
    load_planning_scene_config,
    next_scene_revision,
    validate_scene_readback,
)


def main() -> None:
    import rclpy
    from action_msgs.msg import GoalStatus
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointTolerance
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Pose, PoseStamped
    from moveit_msgs.action import ExecuteTrajectory, MoveGroup
    from moveit_msgs.msg import (
        AttachedCollisionObject,
        CollisionObject,
        Constraints,
        JointConstraint,
        MoveItErrorCodes,
        OrientationConstraint,
        PlanningScene,
        PositionConstraint,
        PlanningSceneComponents,
    )
    from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene, GetPositionIK
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive
    from std_msgs.msg import Bool, Empty, String
    from tf2_geometry_msgs import do_transform_pose_stamped
    from tf2_ros import Buffer, TransformException, TransformListener
    from trajectory_msgs.msg import JointTrajectoryPoint

    class FormalGraspExecutor(Node):
        def __init__(self) -> None:
            super().__init__("formal_physical_grasp_executor")
            defaults = (
                ("request_topic", "/active_cleaning/grasp_request"),
                ("wrist_recheck_topic", "/perception/wrist/grasp_recheck"),
                ("result_topic", "/active_cleaning/grasp_result"),
                ("status_topic", "/manipulation/formal_grasp_status"),
                ("base_motion_inhibit_topic", "/manipulation/base_motion_inhibited"),
                ("safety_permit_topic", "/safety/actuators_enabled"),
                ("odometry_topic", "/odom"),
                ("dual_contact_topic", "/manipulation/gripper/dual_contact"),
                ("grasp_state_topic", "/manipulation/grasp/state"),
                (
                    "dry_bin_status_topic",
                    "/model/tzcup_formal_sanitation_vehicle/dry_bin/observed_status_json",
                ),
                ("attach_topic", "/manipulation/grasp/attach"),
                ("detach_topic", "/manipulation/grasp/detach"),
                ("base_frame", "base_link"),
                ("planning_group", "manipulator"),
                ("tool_link", "tool0"),
                ("move_group_action", "/move_action"),
                ("execute_trajectory_action", "/execute_trajectory"),
                ("compute_ik_service", "/compute_ik"),
                ("cartesian_path_service", "/compute_cartesian_path"),
                ("apply_planning_scene_service", "/apply_planning_scene"),
                ("get_planning_scene_service", "/get_planning_scene"),
                ("planning_scene_ready_topic", "/manipulation/planning_scene_ready"),
            )
            for name, value in defaults:
                self.declare_parameter(name, value)
            self.declare_parameter("maximum_input_age_sec", 0.50)
            self.declare_parameter("pick_window_tolerance_m", 0.075)
            self.declare_parameter("maximum_linear_speed_m_s", 0.015)
            self.declare_parameter("maximum_angular_speed_rad_s", 0.025)
            self.declare_parameter("action_timeout_sec", 60.0)
            self.declare_parameter("planning_time_sec", 8.0)
            self.declare_parameter("planning_attempts", 8)
            self.declare_parameter("velocity_scaling", 0.18)
            self.declare_parameter("acceleration_scaling", 0.15)
            self.declare_parameter("cartesian_max_step_m", 0.005)
            self.declare_parameter("minimum_cartesian_fraction", 0.98)
            self.declare_parameter("wrist_recheck_timeout_sec", 5.0)
            self.declare_parameter("wrist_maximum_age_sec", 0.50)
            self.declare_parameter("contact_maximum_age_sec", 0.15)
            self.declare_parameter("bin_settle_timeout_sec", 12.0)
            self.declare_parameter("bin_monitor_startup_timeout_sec", 30.0)
            self.declare_parameter("planning_scene_config_file", "")
            scene_config_file = str(self.get_parameter("planning_scene_config_file").value)
            if not scene_config_file:
                raise RuntimeError("planning_scene_config_file_required")
            self._scene_contract = load_planning_scene_config(scene_config_file)

            latched = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._result = self.create_publisher(
                String, str(self.get_parameter("result_topic").value), 10
            )
            self._status = self.create_publisher(
                DiagnosticArray, str(self.get_parameter("status_topic").value), 10
            )
            self._base_inhibit = self.create_publisher(
                Bool,
                str(self.get_parameter("base_motion_inhibit_topic").value),
                latched,
            )
            self._attach = self.create_publisher(
                Empty, str(self.get_parameter("attach_topic").value), 10
            )
            self._detach = self.create_publisher(
                Empty, str(self.get_parameter("detach_topic").value), 10
            )
            self.create_subscription(
                String, str(self.get_parameter("request_topic").value), self._on_request, 10
            )
            self.create_subscription(
                String,
                str(self.get_parameter("wrist_recheck_topic").value),
                self._on_wrist_recheck,
                10,
            )
            self.create_subscription(
                Bool, str(self.get_parameter("safety_permit_topic").value), self._on_safety, 10
            )
            self.create_subscription(
                Odometry, str(self.get_parameter("odometry_topic").value), self._on_odometry, 20
            )
            self.create_subscription(
                Bool, str(self.get_parameter("dual_contact_topic").value), self._on_dual_contact, 50
            )
            self.create_subscription(
                Bool, str(self.get_parameter("grasp_state_topic").value), self._on_grasp_state, 10
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("planning_scene_ready_topic").value),
                self._on_planning_scene_ready,
                latched,
            )
            self.create_subscription(
                String, str(self.get_parameter("dry_bin_status_topic").value), self._on_bin_status, 50
            )
            self.create_subscription(JointState, "/joint_states", self._on_joints, 50)

            self._move_group = ActionClient(
                self, MoveGroup, str(self.get_parameter("move_group_action").value)
            )
            self._execute = ActionClient(
                self,
                ExecuteTrajectory,
                str(self.get_parameter("execute_trajectory_action").value),
            )
            self._gripper = ActionClient(
                self,
                FollowJointTrajectory,
                "/gripper_controller/follow_joint_trajectory",
            )
            self._storage = ActionClient(
                self,
                FollowJointTrajectory,
                "/storage_controller/follow_joint_trajectory",
            )
            self._compute_ik = self.create_client(
                GetPositionIK, str(self.get_parameter("compute_ik_service").value)
            )
            self._cartesian = self.create_client(
                GetCartesianPath,
                str(self.get_parameter("cartesian_path_service").value),
            )
            self._apply_scene = self.create_client(
                ApplyPlanningScene,
                str(self.get_parameter("apply_planning_scene_service").value),
            )
            self._get_scene = self.create_client(
                GetPlanningScene,
                str(self.get_parameter("get_planning_scene_service").value),
            )
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._lock = threading.RLock()
            self._busy = False
            self._safety_permitted = False
            self._safety_time: float | None = None
            self._odometry: tuple[float, float] | None = None
            self._odometry_time: float | None = None
            self._dual_contact = False
            self._dual_contact_time: float | None = None
            self._grasp_attached: bool | None = None
            # No request may issue an arm, gripper, storage, or MoveIt scene
            # command before the independent bootstrap has read back the
            # configured ground.  A transient-local false after a scene reset
            # also aborts waits before another command can be issued.
            self._planning_scene_ready = False
            self._latest_bin: DryBinSample | None = None
            self._bin_samples: list[DryBinSample] = []
            self._joints: dict[str, float] = {}
            self._wrist_rechecks: dict[str, tuple[GraspRequest, float]] = {}
            self._motion_inhibited = False
            self._state = "IDLE"
            self._reason = "awaiting_perceived_target"
            self._publish_base_inhibit(False)
            self._publish_status()

        def _on_safety(self, message: Bool) -> None:
            with self._lock:
                self._safety_permitted = bool(message.data)
                self._safety_time = time.monotonic()

        def _on_odometry(self, message: Odometry) -> None:
            with self._lock:
                self._odometry = (
                    float(message.twist.twist.linear.x),
                    float(message.twist.twist.angular.z),
                )
                self._odometry_time = time.monotonic()

        def _on_dual_contact(self, message: Bool) -> None:
            with self._lock:
                self._dual_contact = bool(message.data)
                self._dual_contact_time = time.monotonic()

        def _on_grasp_state(self, message: Bool) -> None:
            with self._lock:
                self._grasp_attached = bool(message.data)

        def _on_planning_scene_ready(self, message: Bool) -> None:
            with self._lock:
                self._planning_scene_ready = bool(message.data)

        def _on_bin_status(self, message: String) -> None:
            try:
                sample = DryBinSample.from_json(message.data)
            except ValueError as exc:
                self.get_logger().error(str(exc))
                return
            with self._lock:
                self._latest_bin = sample
                self._bin_samples.append(sample)
                self._bin_samples = self._bin_samples[-200:]

        def _on_joints(self, message: JointState) -> None:
            with self._lock:
                self._joints.update(
                    {name: float(value) for name, value in zip(message.name, message.position)}
                )

        def _on_wrist_recheck(self, message: String) -> None:
            try:
                request = GraspRequest.from_json(message.data)
            except ValueError as exc:
                self.get_logger().warning(f"rejected wrist recheck: {exc}")
                return
            with self._lock:
                self._wrist_rechecks[request.target_id] = (request, time.monotonic())

        def _on_request(self, message: String) -> None:
            try:
                request = GraspRequest.from_json(message.data)
            except ValueError as exc:
                self._publish_result("", False, "invalid_perception_request", {"error": str(exc)})
                return
            with self._lock:
                if self._busy:
                    self._publish_result(request.target_id, False, "manipulator_busy", {"retryable": True})
                    return
                self._busy = True
            threading.Thread(
                target=self._execute_request,
                args=(request,),
                name=f"formal-grasp-{request.target_id}",
                daemon=True,
            ).start()

        def _publish_base_inhibit(self, inhibited: bool) -> None:
            with self._lock:
                self._motion_inhibited = inhibited
            self._base_inhibit.publish(Bool(data=inhibited))

        def _safety_ok(self) -> bool:
            with self._lock:
                age = None if self._safety_time is None else time.monotonic() - self._safety_time
                maximum = float(self.get_parameter("maximum_input_age_sec").value)
                return bool(self._safety_permitted and age is not None and 0.0 <= age <= maximum)

        def _require_safety(self) -> None:
            if not self._safety_ok():
                raise RuntimeError("safety_permit_missing_stale_or_inhibited")
            with self._lock:
                planning_scene_ready = self._planning_scene_ready
            if not planning_scene_ready:
                raise RuntimeError("planning_scene_not_ready_or_ground_readback_missing")

        def _wait_future(self, future: Any, description: str, *, check_safety: bool = True) -> Any:
            deadline = time.monotonic() + float(self.get_parameter("action_timeout_sec").value)
            while rclpy.ok() and not future.done():
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"{description}_timeout")
                if check_safety:
                    self._require_safety()
                time.sleep(0.01)
            if not future.done():
                raise RuntimeError(f"{description}_interrupted")
            return future.result()

        @staticmethod
        def _pose_message(pose: ToolPose | TargetGeometry) -> Pose:
            message = Pose()
            message.position.x = pose.x_m
            message.position.y = pose.y_m
            message.position.z = pose.z_m
            message.orientation.x = pose.qx
            message.orientation.y = pose.qy
            message.orientation.z = pose.qz
            message.orientation.w = pose.qw
            return message

        def _pose_stamped(self, pose: ToolPose) -> PoseStamped:
            message = PoseStamped()
            message.header.frame_id = str(self.get_parameter("base_frame").value)
            message.header.stamp = self.get_clock().now().to_msg()
            message.pose = self._pose_message(pose)
            return message

        def _request_in_base(self, request: GraspRequest) -> tuple[GraspRequest, float]:
            base_frame = str(self.get_parameter("base_frame").value)
            source = PoseStamped()
            source.header.frame_id = request.frame_id
            source.pose = self._pose_message(request.geometry)
            try:
                transform = self._tf_buffer.lookup_transform(
                    base_frame, request.frame_id, rclpy.time.Time()
                )
                transformed = do_transform_pose_stamped(source, transform)
            except TransformException as exc:
                raise RuntimeError("target_to_base_transform_unavailable") from exc
            stamp_s = transform.header.stamp.sec + transform.header.stamp.nanosec / 1.0e9
            clock_s = self.get_clock().now().nanoseconds / 1.0e9
            transform_age = 0.0 if stamp_s <= 0.0 else max(0.0, clock_s - stamp_s)
            pose = transformed.pose
            geometry = TargetGeometry(
                frame_id=base_frame,
                x_m=float(pose.position.x),
                y_m=float(pose.position.y),
                z_m=float(pose.position.z),
                qx=float(pose.orientation.x),
                qy=float(pose.orientation.y),
                qz=float(pose.orientation.z),
                qw=float(pose.orientation.w),
                size_x_m=request.geometry.size_x_m,
                size_y_m=request.geometry.size_y_m,
                size_z_m=request.geometry.size_z_m,
                material=request.geometry.material,
            )
            return GraspRequest(request.target_id, geometry, request.confidence), transform_age

        def _parking_observation(
            self, request_base: GraspRequest, transform_age: float
        ) -> ParkingObservation:
            with self._lock:
                if self._odometry is None or self._odometry_time is None:
                    raise RuntimeError("odometry_unavailable")
                linear, angular = self._odometry
                odometry_age = time.monotonic() - self._odometry_time
            return ParkingObservation(
                target_x_base_m=request_base.geometry.x_m,
                target_y_base_m=request_base.geometry.y_m,
                linear_speed_m_s=linear,
                angular_speed_rad_s=angular,
                transform_age_s=transform_age,
                odometry_age_s=odometry_age,
            )

        def _wait_wrist_recheck(self, original_base: GraspRequest) -> GraspRequest:
            deadline = time.monotonic() + float(self.get_parameter("wrist_recheck_timeout_sec").value)
            maximum_age = float(self.get_parameter("wrist_maximum_age_sec").value)
            reason = "wrist_near_field_recheck_unavailable"
            while time.monotonic() < deadline:
                self._require_safety()
                with self._lock:
                    sample = self._wrist_rechecks.get(original_base.target_id)
                if sample is not None and 0.0 <= time.monotonic() - sample[1] <= maximum_age:
                    refined_base, _ = self._request_in_base(sample[0])
                    valid, reason = validate_wrist_recheck(original_base, refined_base)
                    if valid:
                        return refined_base
                    raise RuntimeError(reason)
                time.sleep(0.02)
            raise RuntimeError(reason)

        def _wait_moveit_interfaces(self) -> None:
            action_interfaces = (
                (self._move_group, "move_group_action_server_unavailable"),
                (self._execute, "execute_trajectory_action_server_unavailable"),
            )
            service_interfaces = (
                (self._compute_ik, "compute_ik_service_unavailable"),
                (self._cartesian, "cartesian_path_service_unavailable"),
                (self._apply_scene, "apply_planning_scene_service_unavailable"),
                (self._get_scene, "get_planning_scene_service_unavailable"),
            )
            for client, reason in action_interfaces:
                if not client.wait_for_server(timeout_sec=5.0):
                    raise RuntimeError(reason)
            for client, reason in service_interfaces:
                if not client.wait_for_service(timeout_sec=5.0):
                    raise RuntimeError(reason)

        def _validate_ik(self, pose: ToolPose, label: str) -> None:
            self._require_safety()
            request = GetPositionIK.Request()
            request.ik_request.group_name = str(self.get_parameter("planning_group").value)
            request.ik_request.ik_link_name = str(self.get_parameter("tool_link").value)
            request.ik_request.pose_stamped = self._pose_stamped(pose)
            request.ik_request.avoid_collisions = True
            request.ik_request.attempts = int(self.get_parameter("planning_attempts").value)
            request.ik_request.timeout.sec = int(float(self.get_parameter("planning_time_sec").value))
            request.ik_request.robot_state.is_diff = True
            response = self._wait_future(
                self._compute_ik.call_async(request), f"{label}_collision_checked_ik"
            )
            if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
                code = None if response is None else int(response.error_code.val)
                raise RuntimeError(f"{label}_collision_checked_ik_failed_{code}")

        def _moveit_goal(self, constraints: Constraints, label: str) -> dict[str, Any]:
            self._require_safety()
            goal = MoveGroup.Goal()
            request = goal.request
            request.group_name = str(self.get_parameter("planning_group").value)
            request.num_planning_attempts = int(self.get_parameter("planning_attempts").value)
            request.allowed_planning_time = float(self.get_parameter("planning_time_sec").value)
            request.max_velocity_scaling_factor = float(self.get_parameter("velocity_scaling").value)
            request.max_acceleration_scaling_factor = float(self.get_parameter("acceleration_scaling").value)
            request.start_state.is_diff = True
            request.goal_constraints = [constraints]
            goal.planning_options.plan_only = False
            goal.planning_options.replan = True
            goal.planning_options.replan_attempts = 2
            handle = self._wait_future(
                self._move_group.send_goal_async(goal), f"{label}_move_group_goal_response"
            )
            if handle is None or not handle.accepted:
                raise RuntimeError(f"{label}_move_group_goal_rejected")
            wrapped = self._wait_future(handle.get_result_async(), f"{label}_move_group")
            if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f"{label}_move_group_action_failed")
            if wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(f"{label}_move_group_error_{int(wrapped.result.error_code.val)}")
            points = len(wrapped.result.planned_trajectory.joint_trajectory.points)
            if points < 1:
                raise RuntimeError(f"{label}_move_group_returned_empty_trajectory")
            return {
                "step": label,
                "planner": "MoveGroup",
                "collision_checked": True,
                "trajectory_points": points,
                "planning_time_sec": float(wrapped.result.planning_time),
            }

        def _moveit_joint(self, target: tuple[float, ...], label: str) -> dict[str, Any]:
            constraints = Constraints()
            constraints.name = label
            constraints.joint_constraints = [
                JointConstraint(
                    joint_name=name,
                    position=value,
                    tolerance_above=0.03,
                    tolerance_below=0.03,
                    weight=1.0,
                )
                for name, value in zip(ARM_JOINTS, target)
            ]
            return self._moveit_goal(constraints, label)

        def _moveit_pose(self, pose: ToolPose, label: str) -> dict[str, Any]:
            self._validate_ik(pose, label)
            constraints = Constraints()
            constraints.name = label
            position = PositionConstraint()
            position.header.frame_id = str(self.get_parameter("base_frame").value)
            position.link_name = str(self.get_parameter("tool_link").value)
            region = SolidPrimitive()
            region.type = SolidPrimitive.BOX
            region.dimensions = [0.004, 0.004, 0.004]
            position.constraint_region.primitives = [region]
            position.constraint_region.primitive_poses = [self._pose_message(pose)]
            position.weight = 1.0
            orientation = OrientationConstraint()
            orientation.header.frame_id = position.header.frame_id
            orientation.link_name = position.link_name
            orientation.orientation = self._pose_message(pose).orientation
            orientation.absolute_x_axis_tolerance = 0.035
            orientation.absolute_y_axis_tolerance = 0.035
            orientation.absolute_z_axis_tolerance = 0.060
            orientation.weight = 1.0
            constraints.position_constraints = [position]
            constraints.orientation_constraints = [orientation]
            result = self._moveit_goal(constraints, label)
            result["target_pose"] = pose.__dict__
            result["ik_validated"] = True
            return result

        def _execute_robot_trajectory(self, trajectory: Any, label: str) -> None:
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = trajectory
            handle = self._wait_future(
                self._execute.send_goal_async(goal), f"{label}_execute_goal_response"
            )
            if handle is None or not handle.accepted:
                raise RuntimeError(f"{label}_execute_rejected")
            wrapped = self._wait_future(handle.get_result_async(), f"{label}_execute")
            if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f"{label}_execute_action_failed")
            if wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(f"{label}_execute_error_{int(wrapped.result.error_code.val)}")

        def _cartesian_move(self, pose: ToolPose, label: str) -> dict[str, Any]:
            self._validate_ik(pose, label)
            request = GetCartesianPath.Request()
            request.header.frame_id = str(self.get_parameter("base_frame").value)
            request.group_name = str(self.get_parameter("planning_group").value)
            request.link_name = str(self.get_parameter("tool_link").value)
            request.start_state.is_diff = True
            request.waypoints = [self._pose_message(pose)]
            request.max_step = float(self.get_parameter("cartesian_max_step_m").value)
            request.jump_threshold = 0.0
            request.avoid_collisions = True
            request.max_velocity_scaling_factor = float(self.get_parameter("velocity_scaling").value)
            request.max_acceleration_scaling_factor = float(self.get_parameter("acceleration_scaling").value)
            response = self._wait_future(
                self._cartesian.call_async(request), f"{label}_cartesian_plan"
            )
            minimum = float(self.get_parameter("minimum_cartesian_fraction").value)
            if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
                code = None if response is None else int(response.error_code.val)
                raise RuntimeError(f"{label}_cartesian_error_{code}")
            if float(response.fraction) < minimum:
                raise RuntimeError(f"{label}_cartesian_fraction_{float(response.fraction):.6f}")
            if not response.solution.joint_trajectory.points:
                raise RuntimeError(f"{label}_cartesian_returned_empty_trajectory")
            self._execute_robot_trajectory(response.solution, label)
            return {
                "step": label,
                "planner": "GetCartesianPath+ExecuteTrajectory",
                "collision_checked": True,
                "ik_validated": True,
                "cartesian_fraction": float(response.fraction),
                "target_pose": pose.__dict__,
                "trajectory_points": len(response.solution.joint_trajectory.points),
            }

        def _target_collision_id(self, target_id: str) -> str:
            digest = hashlib.sha256(target_id.encode("utf-8")).hexdigest()[:16]
            return f"perceived_cube_{digest}"

        def _read_scene_contract(self, label: str) -> tuple[int, str]:
            """Read a complete scene contract; executor never bootstraps it."""
            request = GetPlanningScene.Request()
            request.components.components = (
                PlanningSceneComponents.SCENE_SETTINGS
                | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            )
            response = self._wait_future(
                self._get_scene.call_async(request), label
            )
            if response is None:
                raise RuntimeError(f"{label}_empty")
            objects: list[SceneObjectReadback] = []
            for item in response.scene.world.collision_objects:
                if not item.primitives or not item.primitive_poses:
                    continue
                primitive, pose = item.primitives[0], item.primitive_poses[0]
                shape = "BOX" if primitive.type == SolidPrimitive.BOX else f"UNKNOWN_{primitive.type}"
                objects.append(
                    SceneObjectReadback(
                        object_id=item.id,
                        frame_id=item.header.frame_id,
                        shape_type=shape,
                        dimensions_m=tuple(float(value) for value in primitive.dimensions),
                        pose_xyz_m=(float(pose.position.x), float(pose.position.y), float(pose.position.z)),
                        pose_xyzw=(
                            float(pose.orientation.x), float(pose.orientation.y),
                            float(pose.orientation.z), float(pose.orientation.w),
                        ),
                    )
                )
            pairs: list[tuple[str, str]] = []
            acm = response.scene.allowed_collision_matrix
            for row, first in enumerate(acm.entry_names):
                if row >= len(acm.entry_values):
                    continue
                for column, enabled in enumerate(acm.entry_values[row].enabled):
                    if enabled and column < len(acm.entry_names):
                        pairs.append((first, acm.entry_names[column]))
            scene_name = response.scene.name.strip()
            if not scene_name:
                raise RuntimeError(f"{label}_revision_missing")
            try:
                revision = validate_scene_readback(
                    self._scene_contract,
                    PlanningSceneReadback(
                        revision=scene_name,
                        world_objects=tuple(objects),
                        allowed_collision_pairs=tuple(pairs),
                    ),
                )
            except ValueError as exc:
                raise RuntimeError(f"{label}_scene_contract_invalid") from exc
            return revision, scene_name

        def _apply_scene_diff(self, scene: PlanningScene, label: str) -> None:
            self._require_safety()
            # All writers own a monotonic revision rather than leaving a
            # default empty name in cube diffs.  This preserves the bootstrap
            # provenance instead of silently reverting it after each
            # perceived-cube world/attached lifecycle update.
            if not self._get_scene.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("get_planning_scene_service_unavailable")
            current_revision, current_name = self._read_scene_contract(
                f"{label}_get_current_planning_scene"
            )
            scene.name = next_scene_revision(self._scene_contract, current_name)
            scene.is_diff = True
            scene.robot_state.is_diff = True
            request = ApplyPlanningScene.Request()
            request.scene = scene
            response = self._wait_future(self._apply_scene.call_async(request), label)
            if response is None or response.success is not True:
                raise RuntimeError(f"{label}_failed")
            advanced, _ = self._read_scene_contract(f"{label}_verify_applied_scene")
            if advanced != current_revision + 1:
                raise RuntimeError(f"{label}_revision_not_monotonic_after_apply")

        def _world_collision(self, request: GraspRequest, *, remove: bool) -> CollisionObject:
            item = CollisionObject()
            item.header.frame_id = str(self.get_parameter("base_frame").value)
            item.id = self._target_collision_id(request.target_id)
            item.operation = CollisionObject.REMOVE if remove else CollisionObject.ADD
            if not remove:
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = list(request.geometry.size_m)
                item.primitives = [primitive]
                item.primitive_poses = [self._pose_message(request.geometry)]
            return item

        def _set_world_target(self, request: GraspRequest, *, present: bool) -> None:
            scene = PlanningScene()
            scene.world.collision_objects = [self._world_collision(request, remove=not present)]
            self._apply_scene_diff(scene, "apply_perceived_target_collision_scene")

        def _set_attached_target(self, request: GraspRequest, *, attached: bool) -> None:
            scene = PlanningScene()
            item = AttachedCollisionObject()
            item.link_name = str(self.get_parameter("tool_link").value)
            item.touch_links = [
                "tool0",
                "robotiq_85_base_link",
                "robotiq_85_left_finger_link",
                "robotiq_85_right_finger_link",
                "robotiq_85_left_finger_tip_link",
                "robotiq_85_right_finger_tip_link",
            ]
            item.object = self._world_collision(request, remove=not attached)
            item.object.operation = CollisionObject.ADD if attached else CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects = [item]
            self._apply_scene_diff(scene, "apply_attached_perceived_target_scene")

        def _controller_trajectory(
            self,
            client: Any,
            joints: tuple[str, ...],
            target: tuple[float, ...],
            duration_s: int,
            tolerance: float,
            label: str,
        ) -> dict[str, Any]:
            self._require_safety()
            if not client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError(f"{label}_action_server_unavailable")
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(joints)
            point = JointTrajectoryPoint()
            point.positions = list(target)
            point.time_from_start = Duration(sec=duration_s)
            goal.trajectory.points = [point]
            goal.path_tolerance = [JointTolerance(name=name, position=tolerance) for name in joints]
            goal.goal_tolerance = [JointTolerance(name=name, position=tolerance) for name in joints]
            goal.goal_time_tolerance = Duration(sec=3)
            handle = self._wait_future(client.send_goal_async(goal), f"{label}_goal_response")
            if handle is None or not handle.accepted:
                raise RuntimeError(f"{label}_trajectory_rejected")
            wrapped = self._wait_future(handle.get_result_async(), f"{label}_trajectory")
            if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f"{label}_trajectory_failed")
            if wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError(f"{label}_controller_error_{int(wrapped.result.error_code)}")
            with self._lock:
                terminal = {name: self._joints.get(name) for name in joints}
            return {"step": label, "target": dict(zip(joints, target)), "terminal": terminal}

        def _common_live_contact(self) -> bool:
            with self._lock:
                now = time.monotonic()
                maximum = float(self.get_parameter("contact_maximum_age_sec").value)
                return bool(
                    self._dual_contact_time is not None
                    and now - self._dual_contact_time <= maximum
                    and self._dual_contact
                )

        def _close_contact_attach(self) -> dict[str, Any]:
            self._require_safety()
            if not self._gripper.wait_for_server(timeout_sec=5.0):
                raise RuntimeError("gripper_action_server_unavailable")
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = [GRIPPER_JOINT]
            point = JointTrajectoryPoint()
            point.positions = [0.57]
            point.time_from_start = Duration(sec=20)
            goal.trajectory.points = [point]
            goal.path_tolerance = [JointTolerance(name=GRIPPER_JOINT, position=0.50)]
            goal.goal_tolerance = [JointTolerance(name=GRIPPER_JOINT, position=0.50)]
            goal.goal_time_tolerance = Duration(sec=3)
            handle = self._wait_future(
                self._gripper.send_goal_async(goal), "contact_close_goal_response"
            )
            if handle is None or not handle.accepted:
                raise RuntimeError("contact_close_trajectory_rejected")
            result_future = handle.get_result_async()
            deadline = time.monotonic() + float(self.get_parameter("action_timeout_sec").value)
            contacted = False
            attached = False
            while rclpy.ok() and not result_future.done():
                self._require_safety()
                if time.monotonic() >= deadline:
                    raise TimeoutError("dual_finger_contact_timeout")
                contacted = self._common_live_contact()
                with self._lock:
                    gripper_position = self._joints.get(GRIPPER_JOINT, 0.0)
                if contacted and gripper_position >= 0.20:
                    self._attach.publish(Empty())
                    attach_deadline = time.monotonic() + 2.0
                    while time.monotonic() < attach_deadline:
                        self._require_safety()
                        with self._lock:
                            if self._grasp_attached is True:
                                break
                        time.sleep(0.01)
                    with self._lock:
                        attached = self._grasp_attached is True
                    cancel = self._wait_future(handle.cancel_goal_async(), "contact_close_cancel")
                    if cancel is None or not cancel.goals_canceling:
                        raise RuntimeError("contact_close_could_not_hold")
                    break
                time.sleep(0.005)
            wrapped = self._wait_future(result_future, "contact_close_result")
            if not contacted:
                raise RuntimeError("gripper_closed_without_dual_finger_object_contact")
            if wrapped is None or wrapped.status != GoalStatus.STATUS_CANCELED:
                raise RuntimeError("contact_close_not_canceled_at_physical_contact")
            return {
                "step": "PICK_CONTACT_ATTACH",
                "dual_finger_common_collision": True,
                "identity_free_contact_signal": True,
                "attachment_state_ack_observed": attached,
            }

        def _wait_attached(self, expected: bool, timeout_s: float, reason: str) -> None:
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                self._require_safety()
                with self._lock:
                    if self._grasp_attached is expected:
                        return
                time.sleep(0.01)
            raise RuntimeError(reason)

        def _wait_bin_increment(
            self, baseline: DryBinSample
        ) -> tuple[float, list[DryBinSample]]:
            deadline = time.monotonic() + float(self.get_parameter("bin_settle_timeout_sec").value)
            reason = "no_post_release_bin_samples"
            while time.monotonic() < deadline:
                self._require_safety()
                with self._lock:
                    samples = tuple(self._bin_samples)
                verified, reason, mass = verify_bin_increment(baseline, samples)
                if verified and mass is not None:
                    return mass, list(samples[-8:])
                time.sleep(0.05)
            raise RuntimeError(reason)

        def _wait_bin_baseline(self) -> DryBinSample:
            deadline = time.monotonic() + float(
                self.get_parameter("bin_monitor_startup_timeout_sec").value
            )
            while time.monotonic() < deadline:
                self._require_safety()
                with self._lock:
                    baseline = self._latest_bin
                if baseline is not None and baseline.sensor_ready:
                    return baseline
                time.sleep(0.05)
            raise RuntimeError("dry_bin_monitor_unavailable")

        def _execute_request(self, request: GraspRequest) -> None:
            release_commanded = False
            physical_grasp_phase_entered = False
            safe_transport_restored = False
            scene_target: GraspRequest | None = None
            evidence: dict[str, Any] = {
                "truth_used_for_control": False,
                "simulator_entity_identity_in_request": False,
                "planning_backend": "MoveGroup_action_GetPositionIK_GetCartesianPath",
                "moveit_task_constructor_used": False,
                "sequence": [],
            }
            try:
                self._set_state("VALIDATING", "checking_safety_parking_bin_and_moveit")
                self._require_safety()
                request_base, transform_age = self._request_in_base(request)
                parking = self._parking_observation(request_base, transform_age)
                parked, parking_reason = parking.validate(
                    position_tolerance_m=float(self.get_parameter("pick_window_tolerance_m").value),
                    maximum_linear_speed_m_s=float(self.get_parameter("maximum_linear_speed_m_s").value),
                    maximum_angular_speed_rad_s=float(self.get_parameter("maximum_angular_speed_rad_s").value),
                    maximum_age_s=float(self.get_parameter("maximum_input_age_sec").value),
                )
                evidence["perceived_target"] = {
                    "target_id": request.target_id,
                    "pose_base": request_base.geometry.__dict__,
                    "size_m": request.geometry.size_m,
                    "material": request.geometry.material,
                    "confidence": request.confidence,
                }
                evidence["parking"] = {
                    "valid": parked,
                    "reason": parking_reason,
                    "target_x_base_m": parking.target_x_base_m,
                    "target_y_base_m": parking.target_y_base_m,
                    "linear_speed_m_s": parking.linear_speed_m_s,
                    "angular_speed_rad_s": parking.angular_speed_rad_s,
                }
                if not parked:
                    raise RuntimeError(parking_reason)
                baseline = self._wait_bin_baseline()
                with self._lock:
                    initially_attached = self._grasp_attached
                    self._bin_samples = []
                    self._wrist_rechecks.pop(request.target_id, None)
                if baseline.full:
                    raise RuntimeError("dry_bin_full")
                if initially_attached is True:
                    raise RuntimeError("grasp_transport_attached_at_task_start")
                if initially_attached is None:
                    self._detach.publish(Empty())
                    time.sleep(0.25)
                    evidence["initial_detach"] = {
                        "commanded_safe_idle": True,
                        "state_was_transition_only_and_unknown": True,
                    }
                else:
                    evidence["initial_detach"] = {
                        "commanded_safe_idle": False,
                        "detached_state_observed": True,
                    }
                evidence["baseline_bin"] = baseline.__dict__
                self._wait_moveit_interfaces()

                # The safety manager must consume this latched topic and gate
                # base commands while leaving arm/storage authority available.
                self._publish_base_inhibit(True)
                self._set_state("EXECUTING", "target_conditioned_moveit_task")
                evidence["sequence"].append(self._moveit_joint(TRANSPORT, "TRANSPORT"))
                waypoints = build_target_conditioned_waypoints(request_base.geometry)
                scene_target = request_base
                self._set_world_target(scene_target, present=True)
                evidence["sequence"].append(
                    self._moveit_pose(waypoints.pregrasp, "TARGET_CONDITIONED_PREGRASP")
                )
                evidence["sequence"].append(
                    self._controller_trajectory(
                        self._gripper, (GRIPPER_JOINT,), (0.0,), 3, 0.08, "GRIPPER_OPEN"
                    )
                )

                self._set_state("EXECUTING", "awaiting_wrist_near_field_recheck")
                refined = self._wait_wrist_recheck(request_base)
                evidence["wrist_near_field_recheck"] = {
                    "accepted": True,
                    "pose_base": refined.geometry.__dict__,
                    "source_topic": str(self.get_parameter("wrist_recheck_topic").value),
                }
                self._set_world_target(scene_target, present=False)
                scene_target = refined
                self._set_world_target(scene_target, present=True)
                waypoints = build_target_conditioned_waypoints(refined.geometry)
                evidence["sequence"].append(
                    self._moveit_pose(waypoints.pregrasp, "WRIST_REFINED_PREGRASP")
                )

                # The perceived cube is removed from the world collision set
                # only for the short contact approach. Vehicle/environment
                # collisions remain enabled; after physical contact it is
                # represented as an attached collision object.
                self._set_world_target(scene_target, present=False)
                # Contact can occur at any point during the final Cartesian
                # approach.  Enter the non-retryable physical phase before
                # commanding that motion, so a partial approach failure can
                # never be mistaken for a safe pre-contact failure.
                physical_grasp_phase_entered = True
                evidence["sequence"].append(
                    self._cartesian_move(waypoints.pick, "LINEAR_CONTACT_APPROACH")
                )
                evidence["sequence"].append(self._close_contact_attach())
                self._set_attached_target(scene_target, attached=True)
                evidence["sequence"].append(
                    self._cartesian_move(waypoints.lift, "LINEAR_COLLISION_CHECKED_LIFT")
                )
                with self._lock:
                    attached_after_lift = self._grasp_attached is True
                contact_after_lift = self._common_live_contact()
                if not attached_after_lift and not contact_after_lift:
                    raise RuntimeError("physical_hold_not_observed_after_lift")
                evidence["physical_hold_after_lift"] = {
                    "attachment_state_ack": attached_after_lift,
                    "persistent_dual_finger_contact": contact_after_lift,
                }
                evidence["sequence"].append(
                    self._controller_trajectory(
                        self._storage, STORAGE_JOINTS, (1.05,), 4, 0.12, "DRY_DEPOSIT_GATE_OPEN"
                    )
                )
                evidence["sequence"].append(
                    self._moveit_pose(waypoints.deposit, "COLLISION_CHECKED_DEPOSIT")
                )
                with self._lock:
                    attached_before_deposit = self._grasp_attached is True
                    self._bin_samples = []
                contact_before_deposit = self._common_live_contact()
                if not attached_before_deposit and not contact_before_deposit:
                    raise RuntimeError("physical_hold_not_observed_before_deposit")
                self._set_attached_target(scene_target, attached=False)
                self._detach.publish(Empty())
                release_commanded = True
                if attached_before_deposit:
                    self._wait_attached(False, 2.0, "physical_detachment_not_acknowledged")
                evidence["sequence"].append(
                    {
                        "step": "DETACH_OVER_DRY_BIN",
                        "state_transition_ack_required": attached_before_deposit,
                        "physical_release_requires_bin_increment": True,
                    }
                )
                evidence["sequence"].append(
                    self._controller_trajectory(
                        self._gripper, (GRIPPER_JOINT,), (0.0,), 3, 0.50, "GRIPPER_RELEASE"
                    )
                )
                measured_mass, stable_samples = self._wait_bin_increment(baseline)
                measured_material = material_for_measured_mass(measured_mass)
                if measured_material is None:
                    raise RuntimeError("post_deposit_mass_not_a_permitted_material")
                evidence["dry_bin_verification"] = {
                    "stable_sample_count": len(stable_samples),
                    "measured_increment_kg": measured_mass,
                    "pre_grasp_material": "unknown",
                    "post_deposit_material_from_load_increment": measured_material,
                    "contained_object_count": stable_samples[-1].contained_object_count,
                    "physical_monitor_confirmed": True,
                    "dynamic_payload_increment_confirmed": True,
                }
                evidence["sequence"].append(
                    self._moveit_pose(waypoints.retreat, "COLLISION_CHECKED_BIN_RETREAT")
                )
                evidence["sequence"].append(
                    self._moveit_joint(TRANSPORT, "RETURN_TRANSPORT")
                )
                evidence["sequence"].append(
                    self._controller_trajectory(
                        self._storage, STORAGE_JOINTS, (0.0,), 4, 0.12, "DRY_DEPOSIT_GATE_CLOSED"
                    )
                )
                safe_transport_restored = True
                self._publish_base_inhibit(False)
                evidence["base_motion_inhibit"] = {
                    "topic": str(self.get_parameter("base_motion_inhibit_topic").value),
                    "held_during_arm_task": True,
                    "released_only_after_transport_and_gate_close": True,
                }
                evidence["contact_attachment_detachment_complete"] = True
                self._set_state("SUCCEEDED", "physical_cube_verified_in_bin")
                self._publish_result(request.target_id, True, "physical_cube_verified_in_bin", evidence)
            except Exception as exc:
                evidence["error"] = str(exc)
                evidence["physical_grasp_phase_entered"] = physical_grasp_phase_entered
                evidence["release_commanded"] = release_commanded
                evidence["retryable_without_operator"] = False
                with self._lock:
                    evidence["attached_at_failure"] = self._grasp_attached
                if scene_target is not None and release_commanded:
                    try:
                        self._set_attached_target(scene_target, attached=False)
                    except Exception as scene_exc:
                        evidence["scene_cleanup_error"] = str(scene_exc)
                if (release_commanded or not physical_grasp_phase_entered) and self._safety_ok():
                    recovery: list[dict[str, Any]] = []
                    try:
                        if scene_target is not None and not physical_grasp_phase_entered:
                            self._set_world_target(scene_target, present=False)
                        recovery.append(
                            self._moveit_joint(TRANSPORT, "FAILURE_RETURN_TRANSPORT")
                        )
                        recovery.append(
                            self._controller_trajectory(
                                self._storage,
                                STORAGE_JOINTS,
                                (0.0,),
                                4,
                                0.12,
                                "FAILURE_DRY_GATE_CLOSED",
                            )
                        )
                        safe_transport_restored = True
                        self._publish_base_inhibit(False)
                        evidence["failure_recovery"] = {
                            "completed": True,
                            "sequence": recovery,
                        }
                        # A post-release failure may already have changed the
                        # physical bin payload.  Only pre-contact failures are
                        # safe for the bounded second attempt.
                        evidence["retryable_without_operator"] = (
                            not physical_grasp_phase_entered and not release_commanded
                        )
                    except Exception as recovery_exc:
                        evidence["failure_recovery"] = {
                            "completed": False,
                            "error": str(recovery_exc),
                            "sequence": recovery,
                        }
                elif not release_commanded:
                    evidence["failure_recovery"] = {
                        "completed": False,
                        "reason": "physical_hold_or_nontransport_arm_preserved_for_operator_recovery",
                    }
                evidence["base_motion_inhibit"] = {
                    "topic": str(self.get_parameter("base_motion_inhibit_topic").value),
                    "currently_inhibited": not safe_transport_restored and self._motion_inhibited,
                    "operator_reset_required": not safe_transport_restored and self._motion_inhibited,
                }
                self._set_state("FAILED", str(exc))
                self._publish_result(request.target_id, False, str(exc), evidence)
            finally:
                with self._lock:
                    self._busy = False

        def _set_state(self, state: str, reason: str) -> None:
            with self._lock:
                self._state = state
                self._reason = reason
            self._publish_status()

        def _publish_result(
            self, target_id: str, verified: bool, reason: str, evidence: dict[str, Any]
        ) -> None:
            payload = {
                "schema_version": 2,
                "target_id": target_id,
                "verified_in_bin": verified is True,
                "reason": reason,
                "evidence": evidence,
            }
            self._result.publish(String(data=json.dumps(payload, sort_keys=True)))

        def _publish_status(self) -> None:
            status = DiagnosticStatus()
            status.name = "formal_physical_grasp_executor"
            status.hardware_id = "ur5e_2f85_movegroup_physical_bin_chain"
            status.level = (
                DiagnosticStatus.OK
                if self._state in {"IDLE", "VALIDATING", "EXECUTING", "SUCCEEDED"}
                else DiagnosticStatus.ERROR
            )
            status.message = self._state
            status.values = [
                KeyValue(key="reason", value=self._reason),
                KeyValue(key="busy", value=str(self._busy).lower()),
                KeyValue(key="safety_permitted", value=str(self._safety_ok()).lower()),
                KeyValue(key="base_motion_inhibited", value=str(self._motion_inhibited).lower()),
                KeyValue(
                    key="planning_scene_ready",
                    value=str(self._planning_scene_ready).lower(),
                ),
                KeyValue(key="planning_backend", value="MoveGroup+GetPositionIK+GetCartesianPath"),
                KeyValue(key="moveit_task_constructor_used", value="false"),
                KeyValue(key="truth_used_for_control", value="false"),
            ]
            message = DiagnosticArray()
            message.header.stamp = self.get_clock().now().to_msg()
            message.status = [status]
            self._status.publish(message)

    rclpy.init()
    node = FormalGraspExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
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


if __name__ == "__main__":
    main()
