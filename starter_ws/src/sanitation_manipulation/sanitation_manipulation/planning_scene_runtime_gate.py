"""Live-only, ground-specific MoveIt collision and persistence gate.

This gate consumes a truth-free perceived request and sends no trajectory or
controller command. Its optional ground-removal diff runs only while the
manipulator is idle and preserves robot state, ACM, and all other world
objects while proving that the bootstrap restores the configured ground.
"""

from __future__ import annotations

import hashlib
import json
import time
from typing import Any

from .formal_grasp_core import GraspRequest, ToolPose, build_target_conditioned_waypoints
from .planning_scene_core import (
    PlanningSceneReadback,
    SceneObjectReadback,
    load_planning_scene_config,
    next_scene_revision,
    validate_scene_readback,
)


def main() -> None:
    import rclpy
    from geometry_msgs.msg import Pose, PoseStamped
    from moveit_msgs.msg import CollisionObject, MoveItErrorCodes, PlanningScene, PlanningSceneComponents, RobotState
    from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene, GetPositionIK, GetStateValidity
    from rcl_interfaces.msg import Log
    from rclpy.node import Node
    from rclpy.qos import qos_profile_rosout_default
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive
    from tf2_ros import Buffer, TransformException, TransformListener

    class GroundCollisionRuntimeGate(Node):
        _BODYWORK_SERVICE_DOOR_JOINTS = (
            "bodywork_power_service_door_hinge_joint",
            "bodywork_power_service_door_latch_joint",
            "bodywork_compute_service_door_hinge_joint",
            "bodywork_compute_service_door_latch_joint",
            "bodywork_wet_service_door_hinge_joint",
            "bodywork_wet_service_door_latch_joint",
            "bodywork_rear_dry_service_door_hinge_joint",
            "bodywork_rear_dry_service_door_latch_joint",
        )

        def __init__(self) -> None:
            super().__init__("moveit_ground_runtime_gate")
            self.declare_parameter("config_file", "")
            self.declare_parameter("request_json", "")
            self.declare_parameter("base_frame", "base_link")
            self.declare_parameter("planning_group", "manipulator")
            self.declare_parameter("tool_link", "tool0")
            self.declare_parameter("timeout_sec", 10.0)
            self.declare_parameter("max_map_tf_age_sec", 0.50)
            self.declare_parameter("map_tf_pose_tolerance_m", 0.002)
            self.declare_parameter("allow_ground_removal_test", False)
            config_file = str(self.get_parameter("config_file").value)
            if not config_file:
                raise ValueError("config_file is required")
            self.config = load_planning_scene_config(config_file)
            raw_request = str(self.get_parameter("request_json").value)
            if not raw_request:
                raise ValueError("request_json must be a live truth-free perceived grasp request")
            self.request = GraspRequest.from_json(raw_request)
            self.base_frame = str(self.get_parameter("base_frame").value)
            if self.request.frame_id != self.base_frame:
                raise ValueError("runtime gate request must already be transformed into base_frame")
            self._ik = self.create_client(GetPositionIK, "/compute_ik")
            self._cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
            self._check_state_validity = self.create_client(GetStateValidity, "/check_state_validity")
            self._scene = self.create_client(GetPlanningScene, "/get_planning_scene")
            self._apply = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
            self._door_missing_joint_warning_count = 0
            self._door_missing_joint_warning_summaries: list[dict[str, str]] = []
            self._moveit_error_summaries: list[dict[str, str]] = []
            # This observer does not modify robot state. The eight passive
            # service-door joints intentionally remain off shared
            # /joint_states and use evaluator-only feedback instead.
            self._rosout = self.create_subscription(
                Log, "/rosout", self._on_rosout, qos_profile_rosout_default
            )

        @staticmethod
        def _log_summary(message: Log) -> dict[str, str]:
            return {
                "logger": str(message.name),
                "level": str(int(message.level)),
                "message": " ".join(str(message.msg).split())[:512],
            }

        @classmethod
        def _is_known_bodywork_door_missing_joint_warning(cls, message: Log) -> bool:
            if not (Log.WARN <= int(message.level) < Log.ERROR):
                return False
            text = str(message.msg).lower()
            missing_state_terms = ("missing", "not known", "not found", "not available")
            return (
                "joint" in text
                and any(term in text for term in missing_state_terms)
                and any(joint in text for joint in cls._BODYWORK_SERVICE_DOOR_JOINTS)
            )

        @staticmethod
        def _is_moveit_error(message: Log) -> bool:
            if int(message.level) < Log.ERROR:
                return False
            text = f"{message.name} {message.msg}".lower()
            return any(token in text for token in (
                "moveit", "move_group", "planning_scene", "robot_model_loader", "kinematics", "ompl",
            ))

        def _on_rosout(self, message: Log) -> None:
            if self._is_known_bodywork_door_missing_joint_warning(message):
                self._door_missing_joint_warning_count += 1
                summary = self._log_summary(message)
                if summary not in self._door_missing_joint_warning_summaries:
                    self._door_missing_joint_warning_summaries.append(summary)
            # The known warning is only annotated, never a blanket filter.
            # Any other MoveIt error stays visible and fails this gate.
            if self._is_moveit_error(message):
                summary = self._log_summary(message)
                if summary not in self._moveit_error_summaries:
                    self._moveit_error_summaries.append(summary)

        def _rosout_report(self) -> dict[str, Any]:
            return {
                "bodywork_service_door_missing_joint_warning": {
                    "observed": self._door_missing_joint_warning_count > 0,
                    "count": self._door_missing_joint_warning_count,
                    "message_summaries": self._door_missing_joint_warning_summaries,
                },
                "other_moveit_error_summaries": self._moveit_error_summaries,
            }

        def _wait(self, future: Any, label: str) -> Any:
            deadline = time.monotonic() + float(self.get_parameter("timeout_sec").value)
            while rclpy.ok() and not future.done():
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"{label}_timeout")
                rclpy.spin_once(self, timeout_sec=0.05)
            if not future.done():
                raise RuntimeError(f"{label}_interrupted")
            return future.result()

        @staticmethod
        def _pose_message(pose: ToolPose) -> Pose:
            result = Pose()
            result.position.x, result.position.y, result.position.z = pose.x_m, pose.y_m, pose.z_m
            result.orientation.x, result.orientation.y = pose.qx, pose.qy
            result.orientation.z, result.orientation.w = pose.qz, pose.qw
            return result

        def _require_interfaces(self) -> None:
            for client, label in (
                (self._scene, "get_planning_scene"),
                (self._apply, "apply_planning_scene"),
                (self._ik, "compute_ik"),
                (self._cartesian, "compute_cartesian_path"),
                (self._check_state_validity, "check_state_validity"),
            ):
                if not client.wait_for_service(timeout_sec=1.0):
                    raise RuntimeError(f"{label}_unavailable")

        def _readback(self) -> tuple[int, PlanningSceneReadback]:
            request = GetPlanningScene.Request()
            request.components.components = (
                PlanningSceneComponents.SCENE_SETTINGS | PlanningSceneComponents.ROBOT_STATE
                | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            )
            response = self._wait(self._scene.call_async(request), "get_planning_scene")
            objects: list[SceneObjectReadback] = []
            for item in response.scene.world.collision_objects:
                if not item.primitives or not item.primitive_poses:
                    continue
                primitive, pose = item.primitives[0], item.primitive_poses[0]
                shape = "BOX" if primitive.type == primitive.BOX else f"UNKNOWN_{primitive.type}"
                objects.append(SceneObjectReadback(
                    object_id=item.id, frame_id=item.header.frame_id, shape_type=shape,
                    dimensions_m=tuple(float(value) for value in primitive.dimensions),
                    pose_xyz_m=(float(pose.position.x), float(pose.position.y), float(pose.position.z)),
                    pose_xyzw=(float(pose.orientation.x), float(pose.orientation.y), float(pose.orientation.z), float(pose.orientation.w)),
                ))
            pairs: list[tuple[str, str]] = []
            acm = response.scene.allowed_collision_matrix
            for row, first in enumerate(acm.entry_names):
                if row >= len(acm.entry_values):
                    continue
                for column, enabled in enumerate(acm.entry_values[row].enabled):
                    if enabled and column < len(acm.entry_names):
                        pairs.append((first, acm.entry_names[column]))
            readback = PlanningSceneReadback(
                response.scene.name.strip() or None, tuple(objects), tuple(pairs)
            )
            revision = validate_scene_readback(self.config, readback)
            self._require_fresh_map_virtual_joint_state(response.scene.robot_state)
            return revision, readback

        def _require_fresh_map_virtual_joint_state(self, robot_state: Any) -> None:
            """Bind MoveIt's mobile virtual joint to a fresh localization TF.

            This is read-only.  It rejects an absent virtual joint, stale map
            transform, or a planning-scene robot state inconsistent with the
            localization chain; it never publishes shared joint states.
            """
            multi = robot_state.multi_dof_joint_state
            name = self.config.planning_virtual_joint_name
            if name not in multi.joint_names:
                raise RuntimeError("planning_virtual_joint_missing_from_robot_state")
            index = list(multi.joint_names).index(name)
            if index >= len(multi.transforms):
                raise RuntimeError("planning_virtual_joint_transform_missing")
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.config.planning_frame_id,
                    self.config.planning_virtual_joint_child_link,
                    rclpy.time.Time(),
                )
            except TransformException as exc:
                raise RuntimeError("fresh_map_to_base_footprint_tf_unavailable") from exc
            stamp = rclpy.time.Time.from_msg(transform.header.stamp)
            now = self.get_clock().now()
            age = (now - stamp).nanoseconds / 1.0e9
            if stamp.nanoseconds <= 0 or age < 0.0 or age > float(self.get_parameter("max_map_tf_age_sec").value):
                raise RuntimeError("map_to_base_footprint_tf_stale")
            actual = multi.transforms[index]
            expected = transform.transform
            tolerance = float(self.get_parameter("map_tf_pose_tolerance_m").value)
            if any(abs(a - b) > tolerance for a, b in zip(
                (actual.translation.x, actual.translation.y, actual.translation.z),
                (expected.translation.x, expected.translation.y, expected.translation.z),
            )):
                raise RuntimeError("planning_virtual_joint_pose_disagrees_with_map_tf")
            # q and -q are the same physical orientation.
            orientation_dot = sum(a * b for a, b in zip(
                (actual.rotation.x, actual.rotation.y, actual.rotation.z, actual.rotation.w),
                (expected.rotation.x, expected.rotation.y, expected.rotation.z, expected.rotation.w),
            ))
            if abs(abs(orientation_dot) - 1.0) > tolerance:
                raise RuntimeError("planning_virtual_joint_orientation_disagrees_with_map_tf")

        def _ik_solution(self, pose: ToolPose, *, avoid_collisions: bool) -> tuple[int, Any]:
            request = GetPositionIK.Request()
            request.ik_request.group_name = str(self.get_parameter("planning_group").value)
            request.ik_request.ik_link_name = str(self.get_parameter("tool_link").value)
            stamped = PoseStamped()
            stamped.header.frame_id = self.base_frame
            stamped.pose = self._pose_message(pose)
            request.ik_request.pose_stamped = stamped
            request.ik_request.avoid_collisions = avoid_collisions
            response = self._wait(self._ik.call_async(request), "get_position_ik")
            return int(response.error_code.val), response.solution

        def _cartesian_result(self, pose: ToolPose) -> tuple[int, float]:
            request = GetCartesianPath.Request()
            request.header.frame_id = self.base_frame
            request.group_name = str(self.get_parameter("planning_group").value)
            request.link_name = str(self.get_parameter("tool_link").value)
            request.start_state.is_diff = True
            request.waypoints = [self._pose_message(pose)]
            request.max_step = 0.005
            request.jump_threshold = 0.0
            request.avoid_collisions = True
            response = self._wait(self._cartesian.call_async(request), "collision_checked_cartesian")
            return int(response.error_code.val), float(response.fraction)

        def _state_validity(self, robot_state: Any) -> tuple[bool, list[dict[str, str]]]:
            request = GetStateValidity.Request()
            request.robot_state = robot_state
            request.group_name = str(self.get_parameter("planning_group").value)
            response = self._wait(self._check_state_validity.call_async(request), "get_state_validity")
            # The ROS 2 GetStateValidity request has no contacts/max_contacts
            # fields. MoveIt's state-validation capability enables contacts
            # server-side; the response contacts below are the attributable
            # ground evidence required by this gate.
            contacts = [{"body_1": contact.contact_body_1, "body_2": contact.contact_body_2} for contact in response.contacts]
            return bool(response.valid), contacts

        def _ground_contacts(self, robot_state: Any) -> list[dict[str, str]]:
            valid, contacts = self._state_validity(robot_state)
            ground = self.config.ground.object_id
            arm_links = set(self.config.expected_arm_contact_links)
            if valid or not any(ground in contact.values() and arm_links & set(contact.values()) for contact in contacts):
                raise RuntimeError("below_ground_state_not_explicitly_colliding_with_ground")
            return contacts

        def _frozen_negative_state(self) -> RobotState:
            state = RobotState()
            state.joint_state = JointState()
            state.joint_state.name = list(self.config.negative_joint_names)
            state.joint_state.position = list(self.config.negative_joint_positions)
            state.is_diff = True
            return state

        def _apply_diff(self, scene: PlanningScene, *, current_revision: str, label: str) -> None:
            scene.name = next_scene_revision(self.config, current_revision)
            scene.is_diff = True
            scene.robot_state.is_diff = True
            request = ApplyPlanningScene.Request()
            request.scene = scene
            response = self._wait(self._apply.call_async(request), label)
            if response is None or response.success is not True:
                raise RuntimeError(f"{label}_failed")

        def _perceived_cube_diff(self, *, add: bool) -> PlanningScene:
            scene = PlanningScene()
            item = CollisionObject()
            item.header.frame_id = self.base_frame
            digest = hashlib.sha256(self.request.target_id.encode("utf-8")).hexdigest()[:16]
            item.id = f"perceived_cube_{digest}"
            item.operation = CollisionObject.ADD if add else CollisionObject.REMOVE
            if add:
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = list(self.request.geometry.size_m)
                item.primitives = [primitive]
                item.primitive_poses = [self._pose_message(ToolPose(
                    self.request.geometry.x_m, self.request.geometry.y_m, self.request.geometry.z_m,
                    self.request.geometry.qx, self.request.geometry.qy, self.request.geometry.qz, self.request.geometry.qw,
                ))]
            scene.world.collision_objects = [item]
            return scene

        @staticmethod
        def _non_ground_scene_contract(
            readback: PlanningSceneReadback, ground_id: str
        ) -> tuple[set[SceneObjectReadback], set[tuple[str, str]]]:
            objects = {
                item for item in readback.world_objects
                if item.object_id != ground_id
            }
            acm_pairs = {
                tuple(sorted(pair)) for pair in readback.allowed_collision_pairs
                if ground_id not in pair
            }
            return objects, acm_pairs

        def _exercise_ground_removal(
            self, current_revision: str, before: PlanningSceneReadback
        ) -> tuple[int, PlanningSceneReadback]:
            if self.get_parameter("allow_ground_removal_test").value is not True:
                raise RuntimeError("allow_ground_removal_test=true is required while manipulator is idle")
            removal = PlanningScene()
            # Never send an empty full-scene replacement: that would clear
            # MoveIt's robot state, SRDF-derived ACM and unrelated world
            # objects. This diff removes only ground, so the bootstrap can
            # prove persistent restoration without destroying scene state.
            removal.name = next_scene_revision(self.config, current_revision)
            removal.is_diff = True
            removal.robot_state.is_diff = True
            item = CollisionObject()
            item.id = self.config.ground.object_id
            item.header.frame_id = self.config.ground.frame_id
            item.operation = CollisionObject.REMOVE
            removal.world.collision_objects = [item]
            request = ApplyPlanningScene.Request()
            request.scene = removal
            response = self._wait(self._apply.call_async(request), "exercise_ground_removal_diff")
            if response is None or response.success is not True:
                raise RuntimeError("exercise_ground_removal_diff_failed")
            baseline = int(current_revision.rsplit(":", 1)[1]) + 1
            deadline = time.monotonic() + float(self.get_parameter("timeout_sec").value)
            while time.monotonic() < deadline:
                try:
                    revision, restored = self._readback()
                    if revision > baseline:
                        ground = self.config.ground.object_id
                        if self._non_ground_scene_contract(before, ground) != self._non_ground_scene_contract(restored, ground):
                            raise RuntimeError("ground_removal_changed_non_ground_world_or_acm")
                        return revision, restored
                except ValueError:
                    pass
                rclpy.spin_once(self, timeout_sec=0.05)
            raise RuntimeError("bootstrap_did_not_restore_ground_after_removal")

        def _assert_advanced_ground_readback(self, previous: int) -> int:
            revision, _ = self._readback()
            if revision <= previous:
                raise RuntimeError("planning_scene_revision_did_not_increment")
            return revision

        def run(self) -> dict[str, Any]:
            self._require_interfaces()
            initial_number, initial = self._readback()
            waypoints = build_target_conditioned_waypoints(self.request.geometry)
            positive = (waypoints.pregrasp, waypoints.pick, waypoints.lift, waypoints.deposit)
            positive_solutions = {name: self._ik_solution(pose, avoid_collisions=True) for name, pose in zip(
                ("pregrasp", "pick", "lift", "deposit"), positive
            )}
            positive_ik = {name: result[0] for name, result in positive_solutions.items()}
            if any(code != MoveItErrorCodes.SUCCESS for code in positive_ik.values()):
                raise RuntimeError(f"normal_collision_checked_ik_failed:{positive_ik}")
            positive_state_validity = {
                name: self._state_validity(solution)[0]
                for name, (_, solution) in positive_solutions.items()
            }
            if not all(positive_state_validity.values()):
                raise RuntimeError(f"normal_anchor_state_invalid:{positive_state_validity}")
            positive_cartesian_code, positive_cartesian_fraction = self._cartesian_result(waypoints.pick)
            if positive_cartesian_code != MoveItErrorCodes.SUCCESS or positive_cartesian_fraction < 0.98:
                raise RuntimeError("normal_collision_checked_cartesian_pick_failed")
            # This frozen state is independently known to be reachable and to
            # put an arm link at z=-0.463656072 in base_footprint coordinates.
            # It avoids the ambiguous "IK failed" inference entirely.
            contacts = self._ground_contacts(self._frozen_negative_state())
            restore_revision, after_restore = self._exercise_ground_removal(
                initial.revision or "", initial
            )
            self._apply_diff(self._perceived_cube_diff(add=True), current_revision=after_restore.revision or "", label="apply_perceived_cube_world_diff")
            cube_add_revision = self._assert_advanced_ground_readback(restore_revision)
            _, after_cube_add = self._readback()
            self._apply_diff(self._perceived_cube_diff(add=False), current_revision=after_cube_add.revision or "", label="remove_perceived_cube_world_diff")
            cube_remove_revision = self._assert_advanced_ground_readback(cube_add_revision)
            return {
                "runtime_gate": "moveit_ground_collision", "passed": True,
                "executor_or_controller_commands_sent": False, "truth_used_for_control": False,
                "normal_collision_checked_ik": positive_ik,
                "normal_anchor_state_valid": positive_state_validity,
                "normal_pick_cartesian_fraction": positive_cartesian_fraction,
                "negative_joint_state": dict(zip(self.config.negative_joint_names, self.config.negative_joint_positions)),
                "below_ground_ground_contacts": contacts,
                "scene_revisions": {
                    "initial": initial_number, "after_ground_restore": restore_revision,
                    "after_perceived_cube_add": cube_add_revision, "after_perceived_cube_remove": cube_remove_revision,
                },
                "ground_removal_preserved_non_ground_world_and_acm": True,
                "ground_removal_used_robot_state_diff_only": True,
                "rosout_observation": self._rosout_report(),
            }

    rclpy.init()
    node: GroundCollisionRuntimeGate | None = None
    try:
        node = GroundCollisionRuntimeGate()
        print(json.dumps(node.run(), sort_keys=True))
    except Exception as exc:
        result: dict[str, Any] = {
            "runtime_gate": "moveit_ground_collision", "passed": False, "error": str(exc),
        }
        if node is not None:
            result["rosout_observation"] = node._rosout_report()
        print(json.dumps(result, sort_keys=True))
        raise SystemExit(1) from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
