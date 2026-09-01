#!/usr/bin/env python3
"""Machine-accept the formal vehicle's enforced actuator interlock."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path

from formal_runtime_gate_binding import load_binding
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from ros_gz_interfaces.msg import Contact, Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectoryPoint


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SNAPSHOT = ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
DEFAULT_SESSION = ROOT / "artifacts/formal_final_acceptance_session.json"


def _source_binding(snapshot_path: Path) -> dict[str, str]:
    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    outputs = snapshot.get("outputs", {})
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf", {})
    source_hash = snapshot.get("source_inventory_sha256")
    urdf_hash = urdf.get("sha256") if isinstance(urdf, dict) else None
    if not isinstance(source_hash, str) or not source_hash:
        raise ValueError("snapshot has no source_inventory_sha256")
    if not isinstance(urdf_hash, str) or not urdf_hash:
        raise ValueError("snapshot has no expanded URDF sha256")
    return {
        "snapshot_manifest_sha256": hashlib.sha256(snapshot_path.read_bytes()).hexdigest(),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": urdf_hash,
    }


def _bound_runtime_evidence(
    snapshot_path: Path, session_path: Path, binding_path: Path
) -> dict[str, object]:
    """Reject interlock evidence detached from the current formal session."""

    source_binding = _source_binding(snapshot_path)
    session = json.loads(session_path.read_text(encoding="utf-8"))
    if not isinstance(session, dict) or session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise ValueError("formal acceptance session must be RUNNING")
    started_epoch_ns = session.get("started_epoch_ns")
    if not isinstance(started_epoch_ns, int) or started_epoch_ns <= 0:
        raise ValueError("formal acceptance session start time is invalid")
    binding = load_binding(binding_path)
    bound_session = binding.get("acceptance_session_binding")
    if not isinstance(bound_session, dict):
        raise ValueError("runtime binding has no acceptance-session binding")
    if bound_session.get("snapshot") != source_binding:
        raise ValueError("runtime binding snapshot differs from interlock source binding")
    if (
        bound_session.get("session_manifest_sha256")
        != hashlib.sha256(session_path.read_bytes()).hexdigest()
        or bound_session.get("session_started_epoch_ns") != started_epoch_ns
    ):
        raise ValueError("runtime binding session differs from interlock session")
    return binding


SWITCHED_CONTROLLERS = {
    "brush_controller",
    "recovery_controller",
}
HELD_CONTROLLERS = {
    "cleaning_controller",
    "arm_controller",
    "gripper_controller",
    "storage_controller",
}
ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
HELD_JOINT_THRESHOLDS = {
    "cleaning_lift_joint": 0.002,
    **{joint: 0.01 for joint in ARM_JOINTS},
    "robotiq_85_left_knuckle_joint": 0.005,
    "dry_deposit_gate_joint": 0.01,
}
STATUS_JSON_KEYS = {
    "schema_version",
    "state",
    "safety_inputs_permit_actuators",
    "actuators_enabled",
    "managed_controllers_active",
    "active_reasons",
    "unsafe_generation",
    "consumed_unsafe_generation",
    "status_publish_count",
    "maximum_timer_gap_sec",
    "publish_thread_error",
}
ALLOWED_ADDITIONAL_STATUS_REASONS = {"manipulator_base_inhibit"}
MAXIMUM_STATUS_SAMPLE_GAP_SEC = 0.25
MAXIMUM_PERMIT_SAMPLE_GAP_SEC = 0.25
MINIMUM_PHASE_DURATION_SEC = 1.25


def _endpoint_fqn(namespace: str, name: str) -> str:
    namespace = namespace.rstrip("/")
    return f"{namespace}/{name}" if namespace else f"/{name}"


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("whole_vehicle_actuator_interlock_acceptance")
        self.brush_samples: list[tuple[float, tuple[float, ...]]] = []
        self.pump_samples: list[tuple[float, tuple[float, ...]]] = []
        self.base_samples: list[tuple[float, tuple[float, float]]] = []
        self.permit_samples: list[tuple[float, bool]] = []
        self.joint_samples: list[tuple[float, dict[str, float]]] = []
        self.status_samples: list[tuple[float, dict]] = []
        self.status_parse_errors: list[str] = []
        self.latest_joints: dict[str, float] = {}
        self.estop = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/emergency_stop", 10
        )
        self.relay = self.create_publisher(Bool, "/safety/relay_enabled", 10)
        self.bms_fault = self.create_publisher(
            Bool, "/formal_vehicle/power/bms_fault", 10
        )
        self.traction_permitted = self.create_publisher(
            Bool, "/formal_vehicle/power/traction_permitted", 10
        )
        self.heartbeat = self.create_publisher(Empty, "/safety/control_heartbeat", 10)
        self.front = self.create_publisher(
            Contacts, "/safety/front_bumper/contact", 10
        )
        self.rear = self.create_publisher(
            Contacts, "/safety/rear_bumper/contact", 10
        )
        self.brush_input = self.create_publisher(
            Float64MultiArray, "/safety/command/brush", 10
        )
        self.pump_input = self.create_publisher(
            Float64MultiArray, "/safety/command/pump", 10
        )
        self.base_input = self.create_publisher(Twist, "/cmd_vel_gate", 10)
        self.create_subscription(
            TwistStamped,
            "/base_controller/cmd_vel",
            lambda message: self.base_samples.append(
                (
                    time.monotonic(),
                    (float(message.twist.linear.x), float(message.twist.angular.z)),
                )
            ),
            50,
        )
        self.create_subscription(
            Float64MultiArray,
            "/brush_controller/commands",
            lambda message: self.brush_samples.append(
                (time.monotonic(), tuple(message.data))
            ),
            50,
        )
        self.create_subscription(
            Float64MultiArray,
            "/recovery_controller/commands",
            lambda message: self.pump_samples.append(
                (time.monotonic(), tuple(message.data))
            ),
            50,
        )
        self.create_subscription(
            Bool,
            "/safety/actuators_enabled",
            lambda message: self.permit_samples.append(
                (time.monotonic(), bool(message.data))
            ),
            50,
        )
        self.create_subscription(JointState, "/joint_states", self._on_joints, 50)
        self.create_subscription(String, "/safety/status_json", self._on_status, 50)
        self.list_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.arm_action = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
        self.trajectory_actions = {
            "/cleaning_controller/follow_joint_trajectory": ActionClient(
                self,
                FollowJointTrajectory,
                "/cleaning_controller/follow_joint_trajectory",
            ),
            "/arm_controller/follow_joint_trajectory": self.arm_action,
            "/gripper_controller/follow_joint_trajectory": ActionClient(
                self,
                FollowJointTrajectory,
                "/gripper_controller/follow_joint_trajectory",
            ),
            "/storage_controller/follow_joint_trajectory": ActionClient(
                self,
                FollowJointTrajectory,
                "/storage_controller/follow_joint_trajectory",
            ),
        }

    def _on_joints(self, message: JointState) -> None:
        self.latest_joints.update(
            {
                name: float(position)
                for name, position in zip(message.name, message.position)
            }
        )
        observed = {
            joint: self.latest_joints[joint]
            for joint in HELD_JOINT_THRESHOLDS
            if joint in self.latest_joints
        }
        if observed:
            self.joint_samples.append((time.monotonic(), observed))

    def _on_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise TypeError("status_json payload must be an object")
        except (json.JSONDecodeError, TypeError) as exc:
            self.status_parse_errors.append(f"{type(exc).__name__}: {exc}")
            return
        self.status_samples.append((time.monotonic(), payload))

    def publish_inputs(
        self,
        *,
        estop: bool,
        collision: bool = False,
        rear_collision: bool = False,
        relay_enabled: bool = True,
        send_heartbeat: bool = True,
    ) -> None:
        self.estop.publish(Bool(data=estop))
        self.relay.publish(Bool(data=relay_enabled))
        self.bms_fault.publish(Bool(data=False))
        self.traction_permitted.publish(Bool(data=True))
        if send_heartbeat:
            self.heartbeat.publish(Empty())
        self.front.publish(Contacts(contacts=[Contact()] if collision else []))
        self.rear.publish(
            Contacts(contacts=[Contact()] if rear_collision else [])
        )
        self.brush_input.publish(Float64MultiArray(data=[8.0, -8.0, 12.0]))
        self.pump_input.publish(Float64MultiArray(data=[20.0]))
        base = Twist()
        base.linear.x = 0.20
        base.angular.z = 0.10
        self.base_input.publish(base)

    def drive_phase(
        self,
        *,
        estop: bool,
        duration_sec: float,
        collision: bool = False,
        rear_collision: bool = False,
        relay_enabled: bool = True,
        send_heartbeat: bool = True,
    ) -> float:
        start = time.monotonic()
        deadline = start + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_inputs(
                estop=estop,
                collision=collision,
                rear_collision=rear_collision,
                relay_enabled=relay_enabled,
                send_heartbeat=send_heartbeat,
            )
            rclpy.spin_once(self, timeout_sec=0.02)
        return start

    def wait_future(
        self,
        future,
        *,
        estop: bool,
        timeout_sec: float,
        collision: bool = False,
        rear_collision: bool = False,
        relay_enabled: bool = True,
        send_heartbeat: bool = True,
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            self.publish_inputs(
                estop=estop,
                collision=collision,
                rear_collision=rear_collision,
                relay_enabled=relay_enabled,
                send_heartbeat=send_heartbeat,
            )
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise RuntimeError("ROS future timed out")

    def wait_for_action_server_healthy(
        self, action_name: str, timeout_sec: float
    ) -> ActionClient:
        """Discover an action server without starving the safety heartbeat."""

        client = self.trajectory_actions[action_name]
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_inputs(estop=False)
            remaining = deadline - time.monotonic()
            if client.wait_for_server(timeout_sec=min(0.10, max(0.0, remaining))):
                return client
        raise RuntimeError(
            f"held-controller FollowJointTrajectory action unavailable: {action_name}"
        )

    def wait_for_service_with_inputs(
        self,
        client,
        *,
        label: str,
        timeout_sec: float,
        estop: bool = False,
        collision: bool = False,
        rear_collision: bool = False,
        relay_enabled: bool = True,
        send_heartbeat: bool = True,
    ) -> None:
        """Discover a service while continuously preserving the active phase."""

        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_inputs(
                estop=estop,
                collision=collision,
                rear_collision=rear_collision,
                relay_enabled=relay_enabled,
                send_heartbeat=send_heartbeat,
            )
            remaining = deadline - time.monotonic()
            if client.wait_for_service(
                timeout_sec=min(0.10, max(0.0, remaining))
            ):
                return
        raise RuntimeError(f"{label} unavailable")

    def start_live_arm_goal(self, timeout_sec: float):
        missing = sorted(set(ARM_JOINTS) - set(self.latest_joints))
        if missing:
            raise RuntimeError(f"arm joint states unavailable: {missing}")
        self.wait_for_action_server_healthy(
            "/arm_controller/follow_joint_trajectory", timeout_sec
        )
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        target = [self.latest_joints[joint] for joint in ARM_JOINTS]
        target[0] += 0.20 if target[0] < 2.5 else -0.20
        point = JointTrajectoryPoint()
        point.positions = target
        point.time_from_start = Duration(sec=5)
        goal.trajectory.points = [point]
        send_future = self.arm_action.send_goal_async(goal)
        self.wait_future(send_future, estop=False, timeout_sec=timeout_sec)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("live arm trajectory goal was rejected")
        return handle.get_result_async()

    def start_all_live_position_goals(self, timeout_sec: float) -> dict[str, dict]:
        """Start one slow, bounded goal on every held position controller."""

        required = set(ARM_JOINTS) | {
            "cleaning_lift_joint",
            "robotiq_85_left_knuckle_joint",
            "dry_deposit_gate_joint",
        }
        missing = sorted(required - set(self.latest_joints))
        if missing:
            raise RuntimeError(f"held-controller joint states unavailable: {missing}")
        arm_target = [self.latest_joints[joint] for joint in ARM_JOINTS]
        arm_target[0] += 0.20 if arm_target[0] < 2.5 else -0.20
        cleaning = self.latest_joints["cleaning_lift_joint"]
        gripper = self.latest_joints["robotiq_85_left_knuckle_joint"]
        storage = self.latest_joints["dry_deposit_gate_joint"]
        specifications = {
            "/arm_controller/follow_joint_trajectory": (
                list(ARM_JOINTS),
                arm_target,
                {"shoulder_pan_joint": 0.005},
            ),
            "/cleaning_controller/follow_joint_trajectory": (
                ["cleaning_lift_joint"],
                [cleaning + 0.02 if cleaning <= 0.07 else cleaning - 0.02],
                {"cleaning_lift_joint": 0.00025},
            ),
            "/gripper_controller/follow_joint_trajectory": (
                ["robotiq_85_left_knuckle_joint"],
                [gripper + 0.20 if gripper <= 0.50 else gripper - 0.20],
                {"robotiq_85_left_knuckle_joint": 0.01},
            ),
            "/storage_controller/follow_joint_trajectory": (
                ["dry_deposit_gate_joint"],
                [storage + 0.40 if storage <= 1.10 else storage - 0.40],
                {"dry_deposit_gate_joint": 0.02},
            ),
        }
        for action_name in specifications:
            self.wait_for_action_server_healthy(action_name, timeout_sec)

        initial_positions = {
            joint: self.latest_joints[joint]
            for joint in required
        }
        send_futures = {}
        for action_name, (joints, targets, motion_thresholds) in specifications.items():
            client = self.trajectory_actions[action_name]
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = joints
            point = JointTrajectoryPoint()
            point.positions = targets
            point.time_from_start = Duration(sec=5)
            goal.trajectory.points = [point]
            send_futures[action_name] = client.send_goal_async(goal)

        goals: dict[str, dict] = {}
        for action_name, (joints, targets, motion_thresholds) in specifications.items():
            send_future = send_futures[action_name]
            self.wait_future(send_future, estop=False, timeout_sec=timeout_sec)
            handle = send_future.result()
            if handle is None or not handle.accepted:
                raise RuntimeError(f"held-controller live goal was rejected: {action_name}")
            goals[action_name] = {
                "future": handle.get_result_async(),
                "joints": tuple(joints),
                "initial_positions": {
                    joint: initial_positions[joint] for joint in joints
                },
                "target_positions": dict(zip(joints, targets)),
                "motion_thresholds": motion_thresholds,
            }
        return goals

    def wait_for_all_live_position_goal_motion(
        self, goals: dict[str, dict], timeout_sec: float
    ) -> dict[str, dict]:
        """Prove every accepted goal physically started before an inhibit."""

        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            completed = sorted(
                action_name
                for action_name, evidence in goals.items()
                if evidence["future"].done()
            )
            if completed:
                raise RuntimeError(
                    "held-controller goals completed before live motion was proven: "
                    f"{completed}"
                )
            motion = {
                action_name: {
                    joint: abs(
                        self.latest_joints.get(joint, initial) - initial
                    )
                    for joint, initial in evidence["initial_positions"].items()
                }
                for action_name, evidence in goals.items()
            }
            if all(
                all(
                    motion[action_name].get(joint, 0.0) >= threshold
                    for joint, threshold in evidence["motion_thresholds"].items()
                )
                for action_name, evidence in goals.items()
            ):
                return motion
            self.publish_inputs(estop=False)
            rclpy.spin_once(self, timeout_sec=0.02)
        raise RuntimeError(
            "not every held-controller goal produced measured motion before inhibit"
        )

    def wait_for_position_goal_cancellations(
        self,
        goals: dict[str, dict],
        *,
        label: str,
        timeout_sec: float,
        estop: bool = False,
        collision: bool = False,
        rear_collision: bool = False,
        relay_enabled: bool = True,
        send_heartbeat: bool = True,
    ) -> dict[str, int]:
        """Keep the trigger asserted until every live goal reports canceled."""

        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if all(evidence["future"].done() for evidence in goals.values()):
                break
            self.publish_inputs(
                estop=estop,
                collision=collision,
                rear_collision=rear_collision,
                relay_enabled=relay_enabled,
                send_heartbeat=send_heartbeat,
            )
            rclpy.spin_once(self, timeout_sec=0.02)
        pending = sorted(
            action_name
            for action_name, evidence in goals.items()
            if not evidence["future"].done()
        )
        if pending:
            raise RuntimeError(
                f"held-controller goals did not finish during {label}: {pending}"
            )
        statuses: dict[str, int] = {}
        for action_name, evidence in goals.items():
            result = evidence["future"].result()
            if result is None or result.status != GoalStatus.STATUS_CANCELED:
                status = None if result is None else result.status
                raise RuntimeError(
                    f"active held-controller goal was not {label}-canceled: "
                    f"action={action_name}, status={status}"
                )
            statuses[action_name] = int(result.status)
        return statuses

    def controller_states(
        self,
        timeout_sec: float,
        *,
        estop: bool = False,
        collision: bool = False,
        rear_collision: bool = False,
        relay_enabled: bool = True,
        send_heartbeat: bool = True,
    ) -> dict[str, str]:
        input_arguments = {
            "estop": estop,
            "collision": collision,
            "rear_collision": rear_collision,
            "relay_enabled": relay_enabled,
            "send_heartbeat": send_heartbeat,
        }
        self.wait_for_service_with_inputs(
            self.list_client,
            label="/controller_manager/list_controllers",
            timeout_sec=timeout_sec,
            **input_arguments,
        )
        future = self.list_client.call_async(ListControllers.Request())
        self.wait_future(future, timeout_sec=timeout_sec, **input_arguments)
        response = future.result()
        if response is None:
            raise RuntimeError("list_controllers timed out")
        return {controller.name: controller.state for controller in response.controller}

    def assert_single_gateway_writer(self) -> None:
        expected_node = "/whole_vehicle_safety_manager"
        for topic in (
            "/base_controller/cmd_vel",
            "/brush_controller/commands",
            "/recovery_controller/commands",
            "/cleaning_controller/joint_trajectory",
            "/arm_controller/joint_trajectory",
            "/gripper_controller/joint_trajectory",
            "/storage_controller/joint_trajectory",
        ):
            publishers = self.get_publishers_info_by_topic(topic)
            writers = sorted(
                _endpoint_fqn(item.node_namespace, item.node_name)
                for item in publishers
            )
            if writers != [expected_node]:
                raise RuntimeError(
                    f"{topic} must have exactly one safety-manager writer; "
                    f"observed={writers}"
                )

    def assert_single_safety_input_writers(self) -> dict[str, str]:
        expected = {
            "/formal_vehicle/simulation/command/emergency_stop": self.get_fully_qualified_name(),
            "/emergency_stop": "/formal_auxiliary_bridge",
            "/safety/relay_enabled": self.get_fully_qualified_name(),
            "/formal_vehicle/power/bms_fault": self.get_fully_qualified_name(),
            "/formal_vehicle/power/traction_permitted": self.get_fully_qualified_name(),
            "/safety/control_heartbeat": self.get_fully_qualified_name(),
            "/safety/front_bumper/contact": self.get_fully_qualified_name(),
            "/safety/rear_bumper/contact": self.get_fully_qualified_name(),
            (
                "/model/tzcup_formal_sanitation_vehicle/"
                "cleaning_motors/fault_active"
            ): "/cleaning_actuator_scalar_bridge",
        }
        evidence: dict[str, str] = {}
        for topic, expected_node in expected.items():
            publishers = self.get_publishers_info_by_topic(topic)
            writers = sorted(
                _endpoint_fqn(item.node_namespace, item.node_name)
                for item in publishers
            )
            if writers != [expected_node]:
                raise RuntimeError(
                    f"{topic} must have exactly one attributed safety-input writer; "
                    f"expected={expected_node}, observed={writers}"
                )
            evidence[topic] = expected_node
        return evidence

    def assert_standard_trajectory_actions_available(self) -> None:
        # rclpy does not expose the CLI's aggregate action-graph query on Node.
        # A typed ActionClient both verifies the standard interface type and
        # performs a live discovery handshake with each controller server.
        for action_name in self.trajectory_actions:
            self.wait_for_action_server_healthy(action_name, 3.0)


def _settled(samples, phase_start, settle_sec=0.5):
    return [values for stamp, values in samples if stamp >= phase_start + settle_sec]


def _zero(values) -> bool:
    return all(math.isclose(value, 0.0, abs_tol=1.0e-12) for value in values)


def _periodic_window_evidence(
    stamps: list[float],
    *,
    window_start: float,
    window_end: float,
    maximum_gap_sec: float,
    label: str,
) -> dict[str, float]:
    if len(stamps) < 3:
        raise RuntimeError(f"fewer than three settled {label} samples")
    first_gap = stamps[0] - window_start
    adjacent_gaps = [
        current - previous for previous, current in zip(stamps, stamps[1:])
    ]
    final_gap = window_end - stamps[-1]
    maximum_gap = max([first_gap, final_gap, *adjacent_gaps])
    if maximum_gap > maximum_gap_sec:
        raise RuntimeError(
            f"{label} sample stream did not cover the complete settled window: "
            f"first_gap_sec={first_gap:.6f}, final_gap_sec={final_gap:.6f}, "
            f"maximum_gap_sec={maximum_gap:.6f}"
        )
    return {
        "first_arrival_gap_sec": first_gap,
        "final_arrival_gap_sec": final_gap,
        "maximum_arrival_gap_sec": maximum_gap,
    }


def _status_reason_evidence(
    node: Probe,
    *,
    phase_start: float,
    settle_sec: float,
    target_reason: str | None,
    safety_inputs_permit_actuators: bool,
) -> dict:
    if node.status_parse_errors:
        raise RuntimeError(
            f"invalid /safety/status_json samples: {node.status_parse_errors}"
        )
    window_start = phase_start + settle_sec
    window_end = time.monotonic()
    samples = [
        (stamp, payload)
        for stamp, payload in node.status_samples
        if window_start <= stamp <= window_end
    ]
    if len(samples) < 3:
        raise RuntimeError(
            "fewer than three settled /safety/status_json samples in phase: "
            f"target_reason={target_reason}, observed={len(samples)}"
        )
    permit_samples = [
        (stamp, permitted)
        for stamp, permitted in node.permit_samples
        if window_start <= stamp <= window_end
    ]
    if len(permit_samples) < 3:
        raise RuntimeError(
            "fewer than three settled /safety/actuators_enabled samples in phase: "
            f"target_reason={target_reason}, observed={len(permit_samples)}"
        )
    if any(
        permitted is not safety_inputs_permit_actuators
        for _, permitted in permit_samples
    ):
        raise RuntimeError(
            "/safety/actuators_enabled samples disagree with the expected permit: "
            f"target_reason={target_reason}, expected={safety_inputs_permit_actuators}"
        )

    allowed_reasons = set(ALLOWED_ADDITIONAL_STATUS_REASONS)
    if target_reason is not None:
        allowed_reasons.add(target_reason)
    observed_reason_sets: set[tuple[str, ...]] = set()
    publish_counts: list[int] = []
    maximum_timer_gaps: list[float] = []
    unsafe_generations: list[int] = []
    consumed_unsafe_generations: list[int] = []
    for _, payload in samples:
        if set(payload) != STATUS_JSON_KEYS:
            raise RuntimeError(
                "/safety/status_json schema mismatch: "
                f"expected={sorted(STATUS_JSON_KEYS)}, observed={sorted(payload)}"
            )
        if payload["schema_version"] != 1:
            raise RuntimeError("unsupported /safety/status_json schema_version")
        reason_text = payload["active_reasons"]
        if not isinstance(reason_text, str):
            raise RuntimeError("/safety/status_json active_reasons must be a string")
        reasons = tuple(reason for reason in reason_text.split(",") if reason)
        reason_set = set(reasons)
        if target_reason is not None and target_reason not in reason_set:
            raise RuntimeError(
                f"target safety reason absent: target={target_reason}, "
                f"observed={sorted(reason_set)}"
            )
        unexpected = reason_set - allowed_reasons
        if unexpected:
            raise RuntimeError(
                f"unattributed safety reasons during {target_reason or 'healthy'} "
                f"phase: {sorted(unexpected)}"
            )
        expected_permit = safety_inputs_permit_actuators
        if payload["safety_inputs_permit_actuators"] is not expected_permit:
            raise RuntimeError(
                "safety input permit disagrees with attributed status reason"
            )
        if payload["actuators_enabled"] is not expected_permit:
            raise RuntimeError("effective actuator permit disagrees with safety status")
        if payload["managed_controllers_active"] is not expected_permit:
            raise RuntimeError("managed controller state disagrees with safety status")
        expected_states = (
            {"ENABLED", "BASE_COMMAND_STOPPED"}
            if expected_permit
            else {"INHIBITED"}
        )
        if payload["state"] not in expected_states:
            raise RuntimeError(
                f"unexpected safety state: expected={sorted(expected_states)}, "
                f"observed={payload['state']}"
            )
        if payload["publish_thread_error"] != "none":
            raise RuntimeError(
                "whole-vehicle safety status publisher reported an internal error"
            )
        count = payload["status_publish_count"]
        if type(count) is not int:
            raise RuntimeError("status_publish_count must be an integer")
        maximum_timer_gap = payload["maximum_timer_gap_sec"]
        if (
            isinstance(maximum_timer_gap, bool)
            or not isinstance(maximum_timer_gap, (int, float))
            or not math.isfinite(float(maximum_timer_gap))
            or float(maximum_timer_gap) < 0.0
            or float(maximum_timer_gap) > MAXIMUM_STATUS_SAMPLE_GAP_SEC
        ):
            raise RuntimeError(
                "maximum_timer_gap_sec is invalid or exceeds the acceptance limit: "
                f"observed={maximum_timer_gap}"
            )
        unsafe_generation = payload["unsafe_generation"]
        consumed_generation = payload["consumed_unsafe_generation"]
        if (
            type(unsafe_generation) is not int
            or type(consumed_generation) is not int
            or unsafe_generation < 0
            or consumed_generation < 0
            or consumed_generation > unsafe_generation
        ):
            raise RuntimeError(
                "unsafe generation counters are invalid: "
                f"unsafe={unsafe_generation}, consumed={consumed_generation}"
            )
        publish_counts.append(count)
        maximum_timer_gaps.append(float(maximum_timer_gap))
        unsafe_generations.append(unsafe_generation)
        consumed_unsafe_generations.append(consumed_generation)
        observed_reason_sets.add(tuple(sorted(reason_set)))

    nonconsecutive = [
        (previous, current)
        for previous, current in zip(publish_counts, publish_counts[1:])
        if current != previous + 1
    ]
    if nonconsecutive:
        raise RuntimeError(
            "non-consecutive /safety/status_json publication sequence: "
            f"{nonconsecutive}"
        )
    if any(
        current < previous
        for previous, current in zip(unsafe_generations, unsafe_generations[1:])
    ) or any(
        current < previous
        for previous, current in zip(
            consumed_unsafe_generations, consumed_unsafe_generations[1:]
        )
    ):
        raise RuntimeError("unsafe generation counters moved backwards")
    if any(
        current < previous
        for previous, current in zip(maximum_timer_gaps, maximum_timer_gaps[1:])
    ):
        raise RuntimeError("maximum_timer_gap_sec moved backwards")
    if (
        not safety_inputs_permit_actuators
        and consumed_unsafe_generations[-1] != unsafe_generations[-1]
    ):
        raise RuntimeError(
            "the inhibited settled window ended before the unsafe generation "
            "was consumed"
        )
    status_window = _periodic_window_evidence(
        [stamp for stamp, _ in samples],
        window_start=window_start,
        window_end=window_end,
        maximum_gap_sec=MAXIMUM_STATUS_SAMPLE_GAP_SEC,
        label="/safety/status_json",
    )
    permit_window = _periodic_window_evidence(
        [stamp for stamp, _ in permit_samples],
        window_start=window_start,
        window_end=window_end,
        maximum_gap_sec=MAXIMUM_PERMIT_SAMPLE_GAP_SEC,
        label="/safety/actuators_enabled",
    )
    return {
        "target_reason": target_reason,
        "allowed_additional_reasons": sorted(ALLOWED_ADDITIONAL_STATUS_REASONS),
        "sample_count": len(samples),
        "actuator_permit_sample_count": len(permit_samples),
        "first_status_publish_count": publish_counts[0],
        "last_status_publish_count": publish_counts[-1],
        "settled_window_duration_sec": window_end - window_start,
        "maximum_arrival_gap_sec": status_window["maximum_arrival_gap_sec"],
        "status_window_continuity": status_window,
        "permit_window_continuity": permit_window,
        "maximum_timer_gap_sec": maximum_timer_gaps[-1],
        "unsafe_generation": unsafe_generations[-1],
        "consumed_unsafe_generation": consumed_unsafe_generations[-1],
        "observed_active_reason_sets": [
            list(reasons) for reasons in sorted(observed_reason_sets)
        ],
        "safety_inputs_permit_actuators": safety_inputs_permit_actuators,
    }


def _assert_controller_states(states: dict[str, str], *, permitted: bool) -> None:
    managed = SWITCHED_CONTROLLERS | HELD_CONTROLLERS
    missing = sorted(managed - set(states))
    expected = {
        name: "active" if permitted or name in HELD_CONTROLLERS else "inactive"
        for name in managed
    }
    wrong = {
        name: states.get(name)
        for name in sorted(managed)
        if states.get(name) != expected[name]
    }
    if missing or wrong:
        raise RuntimeError(
            f"managed controller state mismatch: expected={expected}, "
            f"missing={missing}, wrong={wrong}"
        )


def _joint_hold_evidence(
    node: Probe,
    phase_start: float,
    settle_sec: float = 0.25,
    reference_positions: dict[str, float] | None = None,
) -> dict:
    settled = [
        values
        for stamp, values in node.joint_samples
        if stamp >= phase_start + settle_sec
    ]
    if not settled:
        raise RuntimeError("no joint states observed in the inhibited hold window")
    evidence = {}
    for joint, threshold in HELD_JOINT_THRESHOLDS.items():
        values = [sample[joint] for sample in settled if joint in sample]
        if not values:
            raise RuntimeError(f"joint absent from hold evidence: {joint}")
        drift = max(values) - min(values)
        reference = (
            reference_positions.get(joint)
            if reference_positions is not None
            else values[0]
        )
        if reference is None:
            raise RuntimeError(f"joint absent from inhibit reference: {joint}")
        max_abs_from_reference = max(abs(value - reference) for value in values)
        if drift > threshold:
            raise RuntimeError(
                f"inhibited joint drift exceeds threshold: joint={joint}, "
                f"drift={drift}, threshold={threshold}"
            )
        if max_abs_from_reference > threshold:
            raise RuntimeError(
                f"inhibited joint moved away from trigger position: joint={joint}, "
                f"max_abs_from_reference={max_abs_from_reference}, "
                f"threshold={threshold}, trigger_reference={reference}, "
                f"first={values[0]}, last={values[-1]}, range={drift}, "
                f"sample_count={len(values)}"
            )
        evidence[joint] = {
            "trigger_reference": reference,
            "first": values[0],
            "last": values[-1],
            "range": drift,
            "max_abs_from_trigger": max_abs_from_reference,
            "threshold": threshold,
        }
    return evidence


def _run_hard_interlock_phase(
    node: Probe,
    *,
    label: str,
    target_reason: str,
    duration_sec: float,
    timeout_sec: float,
    settle_sec: float = 0.5,
    collision: bool = False,
    rear_collision: bool = False,
    relay_enabled: bool = True,
    send_heartbeat: bool = True,
) -> dict:
    position_goals = node.start_all_live_position_goals(timeout_sec)
    measured_motion = node.wait_for_all_live_position_goal_motion(
        position_goals, timeout_sec
    )
    pre_trigger_joints = dict(node.latest_joints)

    phase_start = node.drive_phase(
        estop=False,
        collision=collision,
        rear_collision=rear_collision,
        relay_enabled=relay_enabled,
        send_heartbeat=send_heartbeat,
        duration_sec=duration_sec,
    )
    settled = {
        "base": _settled(node.base_samples, phase_start, settle_sec),
        "brush": _settled(node.brush_samples, phase_start, settle_sec),
        "pump": _settled(node.pump_samples, phase_start, settle_sec),
    }
    for actuator, samples in settled.items():
        if not samples or not all(_zero(values) for values in samples):
            raise RuntimeError(
                f"{actuator} output was not continuously zero during {label}"
            )

    status_evidence = _status_reason_evidence(
        node,
        phase_start=phase_start,
        settle_sec=settle_sec,
        target_reason=target_reason,
        safety_inputs_permit_actuators=False,
    )

    states = node.controller_states(
        timeout_sec,
        collision=collision,
        rear_collision=rear_collision,
        relay_enabled=relay_enabled,
        send_heartbeat=send_heartbeat,
    )
    _assert_controller_states(states, permitted=False)
    position_goal_statuses = node.wait_for_position_goal_cancellations(
        position_goals,
        label=label,
        timeout_sec=timeout_sec,
        collision=collision,
        rear_collision=rear_collision,
        relay_enabled=relay_enabled,
        send_heartbeat=send_heartbeat,
    )
    hold_evidence = _joint_hold_evidence(
        node,
        phase_start,
        settle_sec,
        reference_positions=pre_trigger_joints,
    )

    managed = SWITCHED_CONTROLLERS | HELD_CONTROLLERS
    return {
        "trigger": {
            "front_bumper_contact": collision,
            "rear_bumper_contact": rear_collision,
            "safety_relay_enabled": relay_enabled,
            "heartbeat_published": send_heartbeat,
        },
        "settle_sec": settle_sec,
        "settled_zero_sample_counts": {
            actuator: len(samples) for actuator, samples in settled.items()
        },
        "managed_controller_states": {
            name: states[name] for name in sorted(managed)
        },
        "position_goal_motion_from_start": measured_motion,
        "pre_trigger_joint_positions": pre_trigger_joints,
        "position_goal_result_statuses": position_goal_statuses,
        "inhibited_joint_hold_evidence": hold_evidence,
        "safety_status_evidence": status_evidence,
        "all_position_controller_goals_moved_before_trigger": True,
        "all_position_controller_goals_canceled": True,
        "all_held_joints_stable_after_cancel": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--phase-duration", type=float, default=1.5)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    args = parser.parse_args()
    if args.phase_duration < MINIMUM_PHASE_DURATION_SEC:
        raise SystemExit(
            "--phase-duration must be at least 1.25 s so the 1.0 s heartbeat "
            "settling window still contains at least three 20 Hz samples"
        )
    runtime_binding = _bound_runtime_evidence(
        args.snapshot, args.session, args.runtime_binding
    )

    rclpy.init()
    node = Probe()
    try:
        locked_start = node.drive_phase(
            estop=False,
            relay_enabled=False,
            duration_sec=args.phase_duration,
        )
        locked_brush = _settled(node.brush_samples, locked_start)
        locked_pump = _settled(node.pump_samples, locked_start)
        locked_base = _settled(node.base_samples, locked_start)
        if not locked_base or not all(_zero(values) for values in locked_base):
            raise RuntimeError("base output was not continuously zero while inhibited")
        if not locked_brush or not all(_zero(values) for values in locked_brush):
            raise RuntimeError("brush output was not continuously zero while inhibited")
        if not locked_pump or not all(_zero(values) for values in locked_pump):
            raise RuntimeError("pump output was not continuously zero while inhibited")
        locked_status_evidence = _status_reason_evidence(
            node,
            phase_start=locked_start,
            settle_sec=0.5,
            target_reason="safety_relay_disabled",
            safety_inputs_permit_actuators=False,
        )
        locked_states = node.controller_states(
            args.timeout, estop=False, relay_enabled=False
        )
        _assert_controller_states(locked_states, permitted=False)
        node.assert_single_gateway_writer()
        safety_input_writer_evidence = node.assert_single_safety_input_writers()

        enabled_start = node.drive_phase(estop=False, duration_sec=args.phase_duration)
        enabled_brush = _settled(node.brush_samples, enabled_start)
        enabled_pump = _settled(node.pump_samples, enabled_start)
        enabled_base = _settled(node.base_samples, enabled_start)
        if (0.20, 0.10) not in enabled_base:
            raise RuntimeError("enabled base command was not forwarded")
        if (8.0, -8.0, 12.0) not in enabled_brush:
            raise RuntimeError("enabled brush command was not forwarded")
        if (20.0,) not in enabled_pump:
            raise RuntimeError("enabled pump command was not forwarded")
        enabled_status_evidence = _status_reason_evidence(
            node,
            phase_start=enabled_start,
            settle_sec=0.5,
            target_reason=None,
            safety_inputs_permit_actuators=True,
        )
        enabled_states = node.controller_states(args.timeout)
        _assert_controller_states(enabled_states, permitted=True)
        node.assert_standard_trajectory_actions_available()

        front_collision_evidence = _run_hard_interlock_phase(
            node,
            label="front bumper contact",
            target_reason="front_bumper_contact",
            duration_sec=args.phase_duration,
            timeout_sec=args.timeout,
            collision=True,
        )

        front_recovery_start = node.drive_phase(
            estop=False, duration_sec=args.phase_duration
        )
        front_recovery_status = _status_reason_evidence(
            node,
            phase_start=front_recovery_start,
            settle_sec=0.5,
            target_reason=None,
            safety_inputs_permit_actuators=True,
        )
        _assert_controller_states(node.controller_states(args.timeout), permitted=True)

        rear_collision_evidence = _run_hard_interlock_phase(
            node,
            label="rear bumper contact",
            target_reason="rear_bumper_contact",
            duration_sec=args.phase_duration,
            timeout_sec=args.timeout,
            rear_collision=True,
        )

        rear_recovery_start = node.drive_phase(
            estop=False, duration_sec=args.phase_duration
        )
        rear_recovery_status = _status_reason_evidence(
            node,
            phase_start=rear_recovery_start,
            settle_sec=0.5,
            target_reason=None,
            safety_inputs_permit_actuators=True,
        )
        _assert_controller_states(node.controller_states(args.timeout), permitted=True)

        relay_disabled_evidence = _run_hard_interlock_phase(
            node,
            label="safety relay false",
            target_reason="safety_relay_disabled",
            duration_sec=args.phase_duration,
            timeout_sec=args.timeout,
            relay_enabled=False,
        )

        relay_recovery_start = node.drive_phase(
            estop=False, duration_sec=args.phase_duration
        )
        relay_recovery_status = _status_reason_evidence(
            node,
            phase_start=relay_recovery_start,
            settle_sec=0.5,
            target_reason=None,
            safety_inputs_permit_actuators=True,
        )
        _assert_controller_states(node.controller_states(args.timeout), permitted=True)

        heartbeat_timeout_evidence = _run_hard_interlock_phase(
            node,
            label="heartbeat timeout",
            target_reason="heartbeat_timeout",
            duration_sec=args.phase_duration,
            timeout_sec=args.timeout,
            settle_sec=1.0,
            send_heartbeat=False,
        )

        heartbeat_recovery_start = node.drive_phase(
            estop=False, duration_sec=args.phase_duration
        )
        heartbeat_recovery_status = _status_reason_evidence(
            node,
            phase_start=heartbeat_recovery_start,
            settle_sec=0.5,
            target_reason=None,
            safety_inputs_permit_actuators=True,
        )
        _assert_controller_states(node.controller_states(args.timeout), permitted=True)
        position_goals = node.start_all_live_position_goals(args.timeout)
        emergency_stop_motion = node.wait_for_all_live_position_goal_motion(
            position_goals, args.timeout
        )
        pre_estop_joints = dict(node.latest_joints)

        relock_start = node.drive_phase(estop=True, duration_sec=args.phase_duration)
        relock_brush = _settled(node.brush_samples, relock_start)
        relock_pump = _settled(node.pump_samples, relock_start)
        relock_base = _settled(node.base_samples, relock_start)
        if not relock_base or not all(_zero(values) for values in relock_base):
            raise RuntimeError("base did not return to zero after re-lock")
        if not relock_brush or not all(_zero(values) for values in relock_brush):
            raise RuntimeError("brush did not return to zero after re-lock")
        if not relock_pump or not all(_zero(values) for values in relock_pump):
            raise RuntimeError("pump did not return to zero after re-lock")
        relock_status_evidence = _status_reason_evidence(
            node,
            phase_start=relock_start,
            settle_sec=0.5,
            target_reason="manual_estop",
            safety_inputs_permit_actuators=False,
        )
        relock_states = node.controller_states(args.timeout, estop=True)
        _assert_controller_states(relock_states, permitted=False)
        position_goal_statuses = node.wait_for_position_goal_cancellations(
            position_goals,
            label="emergency stop",
            timeout_sec=args.timeout,
            estop=True,
        )
        hold_evidence = _joint_hold_evidence(
            node,
            relock_start,
            reference_positions=pre_estop_joints,
        )
        post_estop_joints = dict(node.latest_joints)

        report = {
            "report_id": "tzcup_whole_vehicle_actuator_interlock_v1",
            "status": "WHOLE_VEHICLE_ACTUATOR_INTERLOCK_PASSED",
            "checks": {
                "locked_base_zero": True,
                "locked_brush_zero": True,
                "locked_pump_zero": True,
                "locked_velocity_controllers_inactive": True,
                "locked_position_controllers_active": True,
                "managed_command_topics_have_single_gateway_writer": True,
                "safety_input_topics_have_single_attributed_writer": True,
                "healthy_bms_fault_false_and_traction_permitted_inputs": True,
                "status_json_samples_continuous_and_source_attributed": True,
                "initial_inhibit_attributed_to_safety_relay_disabled": True,
                "enabled_brush_forwarded": True,
                "enabled_pump_forwarded": True,
                "enabled_base_forwarded": True,
                "enabled_managed_controllers_active": True,
                "standard_trajectory_actions_available_when_enabled": True,
                "collision_zeros_base_brush_and_pump": True,
                "collision_velocity_controllers_inactive": True,
                "active_arm_goal_canceled_on_collision": True,
                "active_cleaning_goal_canceled_on_collision": True,
                "active_gripper_goal_canceled_on_collision": True,
                "active_storage_goal_canceled_on_collision": True,
                "all_held_controller_goals_moved_before_collision": True,
                "all_held_controller_goals_canceled_on_collision": True,
                "held_joints_stable_after_collision": True,
                "rear_collision_zeros_base_brush_and_pump": True,
                "rear_collision_velocity_controllers_inactive": True,
                "active_arm_goal_canceled_on_rear_collision": True,
                "active_cleaning_goal_canceled_on_rear_collision": True,
                "active_gripper_goal_canceled_on_rear_collision": True,
                "active_storage_goal_canceled_on_rear_collision": True,
                "all_held_controller_goals_moved_before_rear_collision": True,
                "all_held_controller_goals_canceled_on_rear_collision": True,
                "held_joints_stable_after_rear_collision": True,
                "relay_false_zeros_base_brush_and_pump": True,
                "relay_false_velocity_controllers_inactive": True,
                "active_arm_goal_canceled_on_relay_false": True,
                "active_cleaning_goal_canceled_on_relay_false": True,
                "active_gripper_goal_canceled_on_relay_false": True,
                "active_storage_goal_canceled_on_relay_false": True,
                "all_held_controller_goals_moved_before_relay_false": True,
                "all_held_controller_goals_canceled_on_relay_false": True,
                "held_joints_stable_after_relay_false": True,
                "heartbeat_timeout_zeros_base_brush_and_pump": True,
                "heartbeat_timeout_velocity_controllers_inactive": True,
                "active_arm_goal_canceled_on_heartbeat_timeout": True,
                "active_cleaning_goal_canceled_on_heartbeat_timeout": True,
                "active_gripper_goal_canceled_on_heartbeat_timeout": True,
                "active_storage_goal_canceled_on_heartbeat_timeout": True,
                "all_held_controller_goals_moved_before_heartbeat_timeout": True,
                "all_held_controller_goals_canceled_on_heartbeat_timeout": True,
                "held_joints_stable_after_heartbeat_timeout": True,
                "active_arm_goal_canceled_on_relock": True,
                "active_cleaning_goal_canceled_on_relock": True,
                "active_gripper_goal_canceled_on_relock": True,
                "active_storage_goal_canceled_on_relock": True,
                "all_held_controller_goals_moved_before_relock": True,
                "all_held_controller_goals_canceled_on_relock": True,
                "held_joint_drift_within_threshold": True,
                "relock_brush_zero": True,
                "relock_pump_zero": True,
                "relock_base_zero": True,
                "relock_velocity_controllers_inactive": True,
                "relock_position_controllers_active": True,
                "final_physical_estop_attributed_to_manual_estop": True,
            },
            "sample_counts": {
                "base": len(node.base_samples),
                "brush": len(node.brush_samples),
                "pump": len(node.pump_samples),
                "permit": len(node.permit_samples),
                "status_json": len(node.status_samples),
            },
            "managed_controllers": sorted(SWITCHED_CONTROLLERS | HELD_CONTROLLERS),
            "safety_input_writer_evidence": safety_input_writer_evidence,
            "published_healthy_safety_inputs": {
                "bms_fault": False,
                "traction_permitted": True,
            },
            "safety_status_evidence": {
                "initial_relay_disabled": locked_status_evidence,
                "initial_enabled": enabled_status_evidence,
                "front_bumper_recovery": front_recovery_status,
                "rear_bumper_recovery": rear_recovery_status,
                "safety_relay_recovery": relay_recovery_status,
                "heartbeat_recovery": heartbeat_recovery_status,
                "final_physical_emergency_stop": relock_status_evidence,
            },
            "pre_estop_joint_positions": pre_estop_joints,
            "post_estop_joint_positions": post_estop_joints,
            "emergency_stop_position_goal_motion_from_start": emergency_stop_motion,
            "emergency_stop_position_goal_statuses": position_goal_statuses,
            "inhibited_joint_hold_evidence": hold_evidence,
            "hard_interlock_evidence": {
                "front_bumper_contact": front_collision_evidence,
                "rear_bumper_contact": rear_collision_evidence,
                "safety_relay_false": relay_disabled_evidence,
                "heartbeat_timeout": heartbeat_timeout_evidence,
            },
            "runtime_gate_binding": runtime_binding,
            "acceptance_session_binding": runtime_binding[
                "acceptance_session_binding"
            ],
            "runtime_closure_binding": runtime_binding["runtime_closure_binding"],
            "claim_boundary": (
                "This proves ROS command-path enforcement in the formal Gazebo "
                "controller graph. It is not real-hardware STO or safety-PLC evidence."
            ),
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(report, ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
