#!/usr/bin/env python3
"""Run and score one contact-gated physical cube pick-and-deposit episode."""

from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Any, Callable

import rclpy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros_gz_interfaces.msg import Contacts, Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String
from trajectory_msgs.msg import JointTrajectoryPoint


WORLD = "formal_cube_manipulation"
CUBE = "material_cube"
WRIST = "ur5e_wrist_3_link"
ARM_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"
STORAGE_JOINTS = ["dry_deposit_gate_joint"]
MATERIALS = {
    "paperboard": {"density_kg_m3": 700.0, "mass_kg": 0.0189},
    "PP": {"density_kg_m3": 900.0, "mass_kg": 0.0243},
    "PET": {"density_kg_m3": 1380.0, "mass_kg": 0.03726},
    "aluminum": {"density_kg_m3": 2700.0, "mass_kg": 0.0729},
}

# These joint targets were solved against the production UR5e chain.  The
# The vehicle parks with a random cube in the open right-side manipulation
# window, beyond the body sill and brush envelope.  The side pick keeps the
# arm outside the cowl.  The deposit target is a bent-elbow
# branch above the arm-side hopper; a 60 degree tool yaw aligns the fingers
# with the clear bore and leaves the cube a physical gravity drop.
PREGRASP = [-1.48278161, -0.44199397, 1.21471947, -2.34352182, -1.57079633, -3.05357794]
PICK = [-1.48278161, 0.10260211, 0.80254082, -2.47593926, -1.57079633, -3.05357794]
DEPOSIT = [-0.30233498, -1.56960444, -0.73057657, -2.40638971, 1.56851193, 0.60324332]
CUBE_INITIAL = (0.300, -0.950, 0.017)
BIN_FLOOR_SUPPORT_Z_M = 0.469
BIN_FLOOR_SUPPORT_TOLERANCE_M = 0.020
DRY_BIN_STATUS_TOPIC = "/model/tzcup_formal_sanitation_vehicle/dry_bin/status_json"
SAFETY_PERMIT_TOPIC = "/safety/actuators_enabled"
DRY_BIN_MASS_TOLERANCE_KG = 1e-5
BIN_SUPPORT_COLLISION_TOKENS = (
    "dry_floor_collision",
    "dry_bin_front_panel_collision",
    "dry_bin_rear_panel_collision",
    "dry_bin_left_panel_collision",
    "dry_bin_right_panel_collision",
)


def _gz_executable() -> str:
    executable = shutil.which("gz")
    if executable:
        return executable
    vendor = Path("/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz")
    if vendor.exists():
        return str(vendor)
    raise RuntimeError("Gazebo CLI not found")


def _pose_block(message: str, name: str) -> str:
    marker = f'name: "{name}"'
    marker_index = message.find(marker)
    if marker_index < 0:
        raise RuntimeError(f"Gazebo pose stream did not contain {name}")
    start = message.rfind("pose {", 0, marker_index)
    depth = 0
    for index in range(start + len("pose "), len(message)):
        if message[index] == "{":
            depth += 1
        elif message[index] == "}":
            depth -= 1
            if depth == 0:
                return message[start:index + 1]
    raise RuntimeError(f"unterminated pose block for {name}")


def _scalar(block: str, field: str, default: float = 0.0) -> float:
    match = re.search(rf"^\s*{re.escape(field)}:\s*([-+0-9.eE]+)\s*$", block, re.MULTILINE)
    return float(match.group(1)) if match else default


def read_gazebo_poses(
    *names: str,
    keepalive: Callable[[], None] | None = None,
) -> dict[str, dict[str, float]]:
    command = [_gz_executable(), "topic", "-e", "-t", f"/world/{WORLD}/pose/info", "-n", "1"]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        if keepalive is not None:
            keepalive()
        deadline = time.monotonic() + 10.0
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                process.kill()
                stdout, stderr = process.communicate()
                raise subprocess.TimeoutExpired(command, 10.0, output=stdout, stderr=stderr)
            try:
                stdout, stderr = process.communicate(timeout=min(0.05, remaining))
                break
            except subprocess.TimeoutExpired:
                if keepalive is not None:
                    keepalive()
    except BaseException:
        if process.poll() is None:
            process.kill()
            process.communicate()
        raise
    if keepalive is not None:
        keepalive()
    if process.returncode:
        raise subprocess.CalledProcessError(process.returncode, command, stdout, stderr)
    poses: dict[str, dict[str, float]] = {}
    for name in names:
        block = _pose_block(stdout, name)
        position_match = re.search(r"position\s*\{(?P<body>.*?)\}", block, re.DOTALL)
        if position_match is None:
            raise RuntimeError(f"pose for {name} has no position")
        position = position_match.group("body")
        poses[name] = {axis: _scalar(position, axis) for axis in ("x", "y", "z")}
    return poses


def read_generated_sdf_inertial_mass(material: str) -> dict[str, Any]:
    """Convert the exact spawn xacro through sdformat and read its inertial mass."""
    model = Path(get_package_share_directory("sanitation_manipulation")) / "urdf" / "material_cube.urdf.xacro"
    expanded = subprocess.run(
        ["xacro", str(model), f"material:={material}"],
        check=True,
        capture_output=True,
        text=True,
        timeout=10.0,
    ).stdout
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", encoding="utf-8") as handle:
        handle.write(expanded)
        handle.flush()
        sdf = subprocess.run(
            [_gz_executable(), "sdf", "-p", handle.name],
            check=True,
            capture_output=True,
            text=True,
            timeout=10.0,
        ).stdout
    mass_match = re.search(r"<inertial>.*?<mass>([^<]+)</mass>", sdf, re.DOTALL)
    if mass_match is None:
        raise RuntimeError("generated cube SDF has no inertial mass")
    return {
        "mass_kg": float(mass_match.group(1)),
        "source": "sdformat conversion of the exact material_cube spawn xacro",
        "model": CUBE,
        "link": "cube_link",
    }


def distance(a: dict[str, float], b: dict[str, float]) -> float:
    return math.sqrt(sum((a[axis] - b[axis]) ** 2 for axis in ("x", "y", "z")))


def validate_dry_bin_status(
    status: dict[str, Any] | None,
    *,
    expected_count: int,
    expected_mass_kg: float,
    label: str,
) -> dict[str, Any]:
    """Fail closed on the bridged, observation-only dry-bin instrument state."""
    if not isinstance(status, dict):
        raise RuntimeError(f"{label} dry-bin status_json was not observed through ROS: {status}")
    if status.get("sensor_ready") is not True:
        raise RuntimeError(f"{label} dry-bin sensor is not ready: {status}")
    try:
        count = int(status["contained_object_count"])
        mass_kg = float(status["contained_mass_kg"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"{label} dry-bin status schema is incomplete: {status}") from exc
    if count != expected_count:
        raise RuntimeError(
            f"{label} dry-bin object count mismatch: measured={count}, expected={expected_count}"
        )
    if abs(mass_kg - expected_mass_kg) > DRY_BIN_MASS_TOLERANCE_KG:
        raise RuntimeError(
            f"{label} dry-bin mass mismatch: measured={mass_kg}, expected={expected_mass_kg}, "
            f"tolerance={DRY_BIN_MASS_TOLERANCE_KG}"
        )
    if status.get("full") is not False:
        raise RuntimeError(f"{label} dry-bin unexpectedly reports full: {status}")
    return dict(status)


class PickPlaceProbe(Node):
    def __init__(self) -> None:
        super().__init__(
            "formal_cube_pick_place_runtime_probe",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
            automatically_declare_parameters_from_overrides=True,
        )
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.gripper = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        self.storage = ActionClient(self, FollowJointTrajectory, "/storage_controller/follow_joint_trajectory")
        self.estop_command = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/emergency_stop", 10
        )
        self.estop_reset_command = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/emergency_stop_reset", 10
        )
        self.main_power_command = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/main_power", 10
        )
        self.safe_zero_command = self.create_publisher(Twist, "/cmd_vel_gate", 10)
        self.attach = self.create_publisher(Empty, "/manipulation/grasp/attach", 10)
        self.detach = self.create_publisher(Empty, "/manipulation/grasp/detach", 10)
        self.set_pose = self.create_client(SetEntityPose, f"/world/{WORLD}/set_pose")
        self.create_subscription(JointState, "/joint_states", self._on_joints, 50)
        self.create_subscription(Contacts, "/manipulation/gripper/left_contact", self._on_left, 50)
        self.create_subscription(Contacts, "/manipulation/gripper/right_contact", self._on_right, 50)
        self.create_subscription(Contacts, "/manipulation/cube/contact", self._on_cube, 50)
        self.create_subscription(Contacts, "/storage/dry_deposit/contact", self._on_chute, 50)
        self.create_subscription(Contacts, "/storage/dry_bin/floor_contact", self._on_floor, 50)
        self.create_subscription(Bool, "/manipulation/grasp/state", self._on_grasp_state, 10)
        self.create_subscription(Bool, SAFETY_PERMIT_TOPIC, self._on_safety_permit, 10)
        self.create_subscription(String, DRY_BIN_STATUS_TOPIC, self._on_dry_bin_status, 50)
        self.latest: dict[str, float] = {}
        self.samples: list[dict[str, float]] = []
        self.left_cube_contacts = 0
        self.right_cube_contacts = 0
        self.chute_cube_contacts = 0
        self.bin_support_contacts = 0
        self.bin_support_collision_names: set[str] = set()
        self.all_after_release_contact_count = 0
        self.all_after_release_collision_names: set[str] = set()
        self.bin_support_first_sim_s: float | None = None
        self.bin_support_last_sim_s: float | None = None
        self.released_to_bin = False
        self.left_last_wall_s: float | None = None
        self.right_last_wall_s: float | None = None
        self.grasp_state: bool | None = None
        self.grasp_state_events: list[dict[str, Any]] = []
        self.initialization_set_pose_calls = 0
        self.task_set_pose_calls = 0
        self.task_phase_started = False
        self.attach_command_count = 0
        self.detach_command_count = 0
        self.payload_command_count = 0
        self.dry_bin_status: dict[str, Any] | None = None
        self.dry_bin_status_after_release: dict[str, Any] | None = None
        self.dry_bin_status_sample_count = 0
        self.safety_permitted = False
        self.safety_permit_event_count = 0
        self.create_timer(0.05, self._operator_heartbeat)

    def _operator_heartbeat(self) -> None:
        # These are physical operator controls and a zero vehicle request.  The
        # whole-vehicle safety manager remains the only permit publisher.
        self.estop_command.publish(Bool(data=False))
        self.estop_reset_command.publish(Bool(data=True))
        self.main_power_command.publish(Bool(data=True))
        self.safe_zero_command.publish(Twist())

    def pump_operator_controls(self) -> None:
        """Keep operator inputs fresh while a Gazebo CLI query owns the thread."""
        self._operator_heartbeat()
        rclpy.spin_once(self, timeout_sec=0.0)

    def _on_safety_permit(self, message: Bool) -> None:
        self.safety_permitted = bool(message.data)
        self.safety_permit_event_count += 1

    @staticmethod
    def _message_has_cube(message: Contacts) -> bool:
        for contact in message.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if any(CUBE in name for name in names):
                return True
        return False

    def _on_joints(self, message: JointState) -> None:
        self.latest.update({name: float(value) for name, value in zip(message.name, message.position)})
        observed = {name: self.latest[name] for name in ARM_JOINTS + [GRIPPER_JOINT] if name in self.latest}
        if observed:
            self.samples.append(observed)

    def _on_left(self, message: Contacts) -> None:
        if self._message_has_cube(message):
            self.left_cube_contacts += 1
            self.left_last_wall_s = time.monotonic()

    def _on_right(self, message: Contacts) -> None:
        if self._message_has_cube(message):
            self.right_cube_contacts += 1
            self.right_last_wall_s = time.monotonic()

    def _on_chute(self, message: Contacts) -> None:
        if self._message_has_cube(message):
            self.chute_cube_contacts += 1

    def _on_cube(self, message: Contacts) -> None:
        if not self.released_to_bin:
            return
        for contact in message.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if not any(CUBE in name for name in names):
                continue
            self.all_after_release_contact_count += 1
            self.all_after_release_collision_names.update(name for name in names if CUBE not in name)
            supports = [
                name for name in names
                if "tzcup_formal_sanitation_vehicle" in name and "robotiq_85" not in name
                and any(token in name for token in BIN_SUPPORT_COLLISION_TOKENS)
            ]
            if supports:
                self.bin_support_contacts += 1
                self.bin_support_collision_names.update(supports)
                sim_s = self.get_clock().now().nanoseconds / 1e9
                if self.bin_support_first_sim_s is None:
                    self.bin_support_first_sim_s = sim_s
                self.bin_support_last_sim_s = sim_s

    def _on_floor(self, message: Contacts) -> None:
        if not self.released_to_bin or not self._message_has_cube(message):
            return
        self.bin_support_contacts += 1
        self.bin_support_collision_names.add(
            "sensor-filtered:dry_bin_link_fixed_joint_lump__dry_floor_collision_collision"
        )
        for contact in message.contacts:
            self.bin_support_collision_names.update((contact.collision1.name, contact.collision2.name))
        sim_s = self.get_clock().now().nanoseconds / 1e9
        if self.bin_support_first_sim_s is None:
            self.bin_support_first_sim_s = sim_s
        self.bin_support_last_sim_s = sim_s

    def _on_grasp_state(self, message: Bool) -> None:
        self.grasp_state = bool(message.data)
        self.grasp_state_events.append({"attached": self.grasp_state, "wall_time_s": time.time()})

    def _on_dry_bin_status(self, message: String) -> None:
        status = json.loads(message.data)
        if not isinstance(status, dict):
            raise RuntimeError(f"dry-bin status_json is not an object: {status}")
        self.dry_bin_status = status
        self.dry_bin_status_sample_count += 1
        if self.released_to_bin:
            self.dry_bin_status_after_release = status

    def spin_for(self, wall_seconds: float) -> None:
        deadline = time.monotonic() + wall_seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, deadline - time.monotonic()))

    def spin_sim_for(self, simulated_seconds: float, wall_timeout_s: float) -> float:
        start_ns = self.get_clock().now().nanoseconds
        deadline = time.monotonic() + wall_timeout_s
        measured = 0.0
        while rclpy.ok() and measured < simulated_seconds:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"simulation clock advanced only {measured:.3f}s while waiting for {simulated_seconds:.3f}s"
                )
            rclpy.spin_once(self, timeout_sec=0.05)
            measured = (self.get_clock().now().nanoseconds - start_ns) / 1e9
        return measured

    def wait_until(self, predicate, timeout_s: float, description: str) -> None:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(description)
            rclpy.spin_once(self, timeout_sec=0.05)

    def command_detach(self, *, initialization: bool) -> None:
        if self.task_phase_started and initialization:
            raise RuntimeError("initialization detach is forbidden after task start")
        self.detach.publish(Empty())
        self.detach_command_count += 1
        self.spin_for(0.35)

    def command_attach_after_contact(self) -> dict[str, Any]:
        now = time.monotonic()
        poses = read_gazebo_poses(CUBE, WRIST, keepalive=self.pump_operator_controls)
        cube = poses[CUBE]
        wrist = poses[WRIST]
        horizontal_offset_m = math.hypot(cube["x"] - wrist["x"], cube["y"] - wrist["y"])
        vertical_offset_m = wrist["z"] - cube["z"]
        gate = {
            "left_cube_contact_count": self.left_cube_contacts,
            "right_cube_contact_count": self.right_cube_contacts,
            "left_contact_age_s": None if self.left_last_wall_s is None else now - self.left_last_wall_s,
            "right_contact_age_s": None if self.right_last_wall_s is None else now - self.right_last_wall_s,
            "gripper_joint_rad": self.latest.get(GRIPPER_JOINT),
            "cube_pose_m": cube,
            "wrist_pose_m": wrist,
            "horizontal_offset_m": horizontal_offset_m,
            "vertical_offset_m": vertical_offset_m,
        }
        if (
            self.left_cube_contacts <= 0
            or self.right_cube_contacts <= 0
            or self.left_last_wall_s is None
            or self.right_last_wall_s is None
            or now - self.left_last_wall_s > 0.15
            or now - self.right_last_wall_s > 0.15
            or self.latest.get(GRIPPER_JOINT, 0.0) < 0.20
            or horizontal_offset_m > 0.050
            or not 0.12 < vertical_offset_m < 0.21
        ):
            raise RuntimeError(f"physical grasp gate not satisfied: {gate}")
        self.attach.publish(Empty())
        self.attach_command_count += 1
        # The transport state topic is diagnostic only; the acceptance truth is
        # the cube's measured rigid motion with the wrist during the lift.
        self.spin_for(0.15)
        gate["attach_permitted"] = True
        gate["state_ack_observed"] = self.grasp_state is True
        return gate

    def close_and_attach(self, timeout_s: float) -> tuple[dict[str, Any], dict[str, Any]]:
        """Close slowly and attach at the instant both live contacts satisfy the gate."""
        if not self.gripper.wait_for_server(timeout_sec=timeout_s):
            raise RuntimeError("gripper action server unavailable")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [GRIPPER_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [0.57]
        point.time_from_start = Duration(sec=20)
        goal.trajectory.points = [point]
        goal.path_tolerance = [JointTolerance(name=GRIPPER_JOINT, position=0.50)]
        goal.goal_tolerance = [JointTolerance(name=GRIPPER_JOINT, position=0.50)]
        goal.goal_time_tolerance = Duration(sec=3)
        send_future = self.gripper.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_s)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("gripper close trajectory rejected")
        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout_s
        grasp_gate = None
        cancel_requested = False
        while rclpy.ok() and not result_future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError("contact-gated gripper close timed out")
            rclpy.spin_once(self, timeout_sec=0.01)
            now = time.monotonic()
            live_dual_contact = (
                self.left_last_wall_s is not None
                and self.right_last_wall_s is not None
                and now - self.left_last_wall_s <= 0.15
                and now - self.right_last_wall_s <= 0.15
                and self.latest.get(GRIPPER_JOINT, 0.0) >= 0.20
            )
            if live_dual_contact and grasp_gate is None:
                grasp_gate = self.command_attach_after_contact()
                cancel_future = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
                cancel_response = cancel_future.result()
                if cancel_response is None or not cancel_response.goals_canceling:
                    raise RuntimeError("gripper close could not be stopped at the contact-gated hold position")
                cancel_requested = True
        wrapped = result_future.result()
        if wrapped is None:
            raise TimeoutError("gripper close returned no action result")
        result = wrapped.result
        if grasp_gate is None:
            raise RuntimeError(
                "gripper completed without simultaneous dual-finger cube contact and geometry gate; "
                f"left={self.left_cube_contacts}, right={self.right_cube_contacts}, "
                f"joint={self.latest.get(GRIPPER_JOINT)}"
            )
        canceled_at_contact = cancel_requested and wrapped.status == GoalStatus.STATUS_CANCELED
        if not canceled_at_contact:
            raise RuntimeError(
                f"contact-gated close did not stop as canceled after attach: status={wrapped.status}, "
                f"result={result.error_code} {result.error_string}"
            )
        action = {
            "action": self.gripper._action_name,
            "result_error_code": int(result.error_code),
            "action_status": int(wrapped.status),
            "target_rad": {GRIPPER_JOINT: 0.57},
            "terminal_rad": {GRIPPER_JOINT: self.latest.get(GRIPPER_JOINT)},
            "attach_during_active_close": True,
            "canceled_to_hold_contact_position": canceled_at_contact,
        }
        return action, grasp_gate

    def initialize_cube(self) -> None:
        if self.task_phase_started:
            self.task_set_pose_calls += 1
            raise RuntimeError("SetEntityPose is evaluator-only and forbidden after task start")
        if not self.set_pose.wait_for_service(timeout_sec=20.0):
            raise RuntimeError("SetEntityPose evaluator service unavailable")
        request = SetEntityPose.Request()
        request.entity.name = CUBE
        request.entity.type = Entity.MODEL
        request.pose.position.x, request.pose.position.y, request.pose.position.z = CUBE_INITIAL
        request.pose.orientation.w = 1.0
        future = self.set_pose.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("initial cube pose reset failed")
        self.initialization_set_pose_calls += 1
        self.spin_for(0.75)

    def execute(
        self,
        client: ActionClient,
        joints: list[str],
        target: list[float],
        trajectory_s: int,
        timeout_s: float,
        tolerance: float,
    ) -> dict[str, Any]:
        if not client.wait_for_server(timeout_sec=timeout_s):
            raise RuntimeError(f"action server unavailable: {client._action_name}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = target
        point.time_from_start = Duration(sec=trajectory_s)
        goal.trajectory.points = [point]
        goal.path_tolerance = [JointTolerance(name=name, position=tolerance) for name in joints]
        goal.goal_tolerance = [JointTolerance(name=name, position=tolerance) for name in joints]
        goal.goal_time_tolerance = Duration(sec=3)
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_s)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"trajectory rejected: {client._action_name}")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_s)
        wrapped = result_future.result()
        if wrapped is None:
            raise TimeoutError(f"trajectory timed out: {client._action_name}")
        result = wrapped.result
        terminal = {name: self.latest.get(name) for name in joints}
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"trajectory did not succeed on {client._action_name}: "
                f"status={wrapped.status}; terminal={terminal}"
            )
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"trajectory failed on {client._action_name}: {result.error_code} {result.error_string}; "
                f"terminal={terminal}"
            )
        return {
            "action": client._action_name,
            "result_error_code": int(result.error_code),
            "target_rad": dict(zip(joints, target)),
            "terminal_rad": terminal,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--material", choices=sorted(MATERIALS), default="PET")
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args()
    report: dict[str, Any] = {
        "report_id": "tzcup_formal_physical_cube_pick_place_v1",
        "status": "FAILED",
        "passed": False,
        "material": args.material,
    }
    rclpy.init()
    node = PickPlaceProbe()
    try:
        node.wait_until(
            lambda: all(name in node.latest for name in ARM_JOINTS + [GRIPPER_JOINT]),
            args.timeout,
            "commanded joint states did not become available",
        )
        node.wait_until(
            lambda: node.dry_bin_status is not None,
            args.timeout,
            f"dry-bin status did not arrive on ROS topic {DRY_BIN_STATUS_TOPIC}",
        )
        node.wait_until(
            lambda: node.safety_permitted,
            args.timeout,
            f"whole-vehicle actuator permit did not become true on {SAFETY_PERMIT_TOPIC}",
        )
        baseline_dry_bin_status = validate_dry_bin_status(
            node.dry_bin_status,
            expected_count=0,
            expected_mass_kg=0.0,
            label="pre-task",
        )

        # DetachableJoint has no initial-detached option.  This release and the
        # one pose reset below are strictly environment initialization.  Once
        # task_phase_started flips, this verifier exposes no teleport/delete path.
        node.command_detach(initialization=True)
        node.initialize_cube()
        initial_pose = read_gazebo_poses(CUBE, keepalive=node.pump_operator_controls)[CUBE]
        node.task_phase_started = True

        actions = []
        actions.append(node.execute(node.arm, ARM_JOINTS, PREGRASP, 7, args.timeout, 0.10))
        actions.append(node.execute(node.gripper, [GRIPPER_JOINT], [0.0], 3, args.timeout, 0.08))
        actions.append(node.execute(node.arm, ARM_JOINTS, PICK, 7, args.timeout, 0.10))
        pick_pose = read_gazebo_poses(CUBE, WRIST, keepalive=node.pump_operator_controls)
        close_action, grasp_gate = node.close_and_attach(args.timeout)
        actions.append(close_action)
        attached_pose = read_gazebo_poses(CUBE, WRIST, keepalive=node.pump_operator_controls)
        attached_offset_m = distance(attached_pose[CUBE], attached_pose[WRIST])

        actions.append(node.execute(node.arm, ARM_JOINTS, PREGRASP, 7, args.timeout, 0.12))
        lifted_pose = read_gazebo_poses(CUBE, WRIST, keepalive=node.pump_operator_controls)
        lifted_offset_m = distance(lifted_pose[CUBE], lifted_pose[WRIST])
        lift_m = lifted_pose[CUBE]["z"] - attached_pose[CUBE]["z"]
        if lift_m < 0.20 or abs(lifted_offset_m - attached_offset_m) > 0.012:
            raise RuntimeError(
                f"cube did not remain rigidly held during lift: lift={lift_m}, "
                f"offsets={attached_offset_m},{lifted_offset_m}"
            )

        actions.append(node.execute(node.storage, STORAGE_JOINTS, [1.05], 4, args.timeout, 0.12))
        actions.append(node.execute(node.arm, ARM_JOINTS, DEPOSIT, 12, args.timeout, 0.20))
        release_pose = read_gazebo_poses(CUBE, WRIST, keepalive=node.pump_operator_controls)
        release_cube = release_pose[CUBE]
        if not (-0.30 < release_cube["x"] < -0.10 and -0.04 < release_cube["y"] < 0.11 and release_cube["z"] > 1.08):
            raise RuntimeError(f"held cube did not reach the open dry hopper: {release_cube}")

        node.released_to_bin = True
        node.command_detach(initialization=False)
        actions.append(node.execute(node.gripper, [GRIPPER_JOINT], [0.0], 3, args.timeout, 0.50))
        settle_sim_s = node.spin_sim_for(3.0, args.timeout)
        settled_a = read_gazebo_poses(CUBE, keepalive=node.pump_operator_controls)[CUBE]
        stable_window_sim_s = node.spin_sim_for(1.0, args.timeout)
        settled_b = read_gazebo_poses(CUBE, keepalive=node.pump_operator_controls)[CUBE]
        settling_delta_m = distance(settled_a, settled_b)
        total_settled_sim_s = settle_sim_s + stable_window_sim_s
        in_bin = (
            -0.445 < settled_b["x"] < 0.035
            and -0.017 < settled_b["y"] < 0.337
            and abs(settled_b["z"] - BIN_FLOOR_SUPPORT_Z_M) <= BIN_FLOOR_SUPPORT_TOLERANCE_M
        )
        if not in_bin or settling_delta_m > 0.005:
            raise RuntimeError(
                f"cube was not stably retained in dry bin: pose={settled_b}, delta={settling_delta_m}"
            )
        support_span_sim_s = (
            0.0
            if node.bin_support_first_sim_s is None or node.bin_support_last_sim_s is None
            else node.bin_support_last_sim_s - node.bin_support_first_sim_s
        )
        if node.bin_support_contacts <= 0 or support_span_sim_s < 0.5:
            raise RuntimeError(
                "settled cube has no persistent measured load-bearing contact with dry floor/walls: "
                f"count={node.bin_support_contacts}, span={support_span_sim_s}, "
                f"collisions={sorted(node.bin_support_collision_names)}"
            )

        material = MATERIALS[args.material]
        node.wait_until(
            lambda: (
                node.dry_bin_status_after_release is not None
                and node.dry_bin_status_after_release.get("contained_object_count") == 1
            ),
            args.timeout,
            "dry-bin ROS status did not count the released physical cube; "
            f"settled_pose={settled_b}, latest_status={node.dry_bin_status_after_release}",
        )
        post_release_dry_bin_status = validate_dry_bin_status(
            node.dry_bin_status_after_release,
            expected_count=1,
            expected_mass_kg=material["mass_kg"],
            label="post-release",
        )

        ranges = {}
        for name in ARM_JOINTS + [GRIPPER_JOINT]:
            values = [sample[name] for sample in node.samples if name in sample]
            ranges[name] = max(values) - min(values)
        insufficient = [name for name in ARM_JOINTS if ranges.get(name, 0.0) < 0.05]
        if insufficient:
            raise RuntimeError(f"not all six axes physically moved: {insufficient}, ranges={ranges}")

        sdf_inertial = read_generated_sdf_inertial_mass(args.material)
        if abs(sdf_inertial["mass_kg"] - material["mass_kg"]) > 1e-8:
            raise RuntimeError(f"generated SDF inertial mass mismatch: {sdf_inertial}, expected={material}")
        report.update({
            "status": "PHYSICAL_CONTACT_GATED_PICK_LIFT_DEPOSIT_PASSED",
            "passed": True,
            "physics_engine": "gz-physics-dartsim-plugin",
            "safety_readiness": {
                "permit_topic": SAFETY_PERMIT_TOPIC,
                "permit_observed": node.safety_permitted,
                "permit_event_count": node.safety_permit_event_count,
                "operator_controls_only": True,
                "synthetic_permit_published": False,
            },
            "cube": {
                "edge_m": 0.03,
                **material,
                "initial_pose_m": initial_pose,
                "pick_pose_m": pick_pose[CUBE],
                "attached_pose_m": attached_pose[CUBE],
                "lifted_pose_m": lifted_pose[CUBE],
                "release_pose_m": release_cube,
                "settled_pose_m": settled_b,
                "lift_m": lift_m,
                "settling_delta_m": settling_delta_m,
                "settled_sim_duration_s": total_settled_sim_s,
                "stable_window_sim_duration_s": stable_window_sim_s,
                "bin_floor_support_z_m": BIN_FLOOR_SUPPORT_Z_M,
                "bin_floor_support_tolerance_m": BIN_FLOOR_SUPPORT_TOLERANCE_M,
                "present_after_deposit": True,
                "stable_inside_dry_bin": True,
            },
            "grasp_gate": grasp_gate,
            "grasp_state_events": node.grasp_state_events,
            "attachment_constraint_proof": {
                "transport_state_ack_observed": any(event["attached"] for event in node.grasp_state_events),
                "truth_source": "Gazebo ground-truth cube and wrist poses before and after physical lift",
                "cube_wrist_offset_before_lift_m": attached_offset_m,
                "cube_wrist_offset_after_lift_m": lifted_offset_m,
                "offset_change_m": abs(lifted_offset_m - attached_offset_m),
                "cube_lift_m": lift_m,
                "constraint_proven_by_rigid_motion_not_ack": True,
            },
            "chute_cube_contact_count": node.chute_cube_contacts,
            "bin_load_bearing_contact": {
                "support_contact_count": node.bin_support_contacts,
                "vehicle_collision_names": sorted(node.bin_support_collision_names),
                "support_contact_span_sim_s": support_span_sim_s,
                "persistent_support_observed": True,
                "all_contact_count_after_release": node.all_after_release_contact_count,
                "all_other_collision_names_after_release": sorted(node.all_after_release_collision_names),
            },
            "dry_bin_monitor": {
                "ros_topic": DRY_BIN_STATUS_TOPIC,
                "transport": "Gazebo-to-ROS bridge",
                "sensor_ready": post_release_dry_bin_status["sensor_ready"],
                "contained_object_count": post_release_dry_bin_status["contained_object_count"],
                "contained_mass_kg": post_release_dry_bin_status["contained_mass_kg"],
                "mass_tolerance_kg": DRY_BIN_MASS_TOLERANCE_KG,
                "full": post_release_dry_bin_status["full"],
                "baseline_status": baseline_dry_bin_status,
                "post_release_status": post_release_dry_bin_status,
                "post_release_sample_observed": True,
                "monitor_is_observation_only": True,
            },
            "actions": actions,
            "measured_joint_range_rad": ranges,
            "inventory_mass": {
                "physical_cube_count": 1,
                "physical_material_mass_kg": material["mass_kg"],
                "generated_sdf_inertial": sdf_inertial,
                "dynamic_dry_payload_command_count": node.payload_command_count,
                "dynamic_dry_payload_added_kg": 0.0,
                "effective_new_inventory_mass_kg": material["mass_kg"],
                "double_count_prevented": True,
                "aggregation_or_reserve_payload_substitution": False,
            },
            "evaluator_interface_audit": {
                "initialization_detach_commands": 1,
                "initialization_set_pose_calls": node.initialization_set_pose_calls,
                "task_set_pose_calls": node.task_set_pose_calls,
                "delete_entity_calls": 0,
                "task_attach_commands": node.attach_command_count,
                "task_detach_commands": node.detach_command_count - 1,
                "set_pose_or_remove_after_task_start": False,
                "physical_cube_deleted_after_deposit": False,
            },
            "claim_boundary": (
                "DetachableJoint supplies the post-contact holding constraint only after live left and right "
                "Robotiq fingertip contacts and measured closure. The cube remains a physical Gazebo body in "
                "the dry bin, so its material mass loads the vehicle exactly once; no reserve payload command "
                "is published. Initialization detach/reset are evaluator setup and are forbidden after task start."
            ),
        })
    except Exception as exc:
        report["failure"] = {"type": type(exc).__name__, "message": str(exc)}
        report["runtime_counters"] = {
            "left_cube_contacts": node.left_cube_contacts,
            "right_cube_contacts": node.right_cube_contacts,
            "chute_cube_contacts": node.chute_cube_contacts,
            "bin_support_contacts": node.bin_support_contacts,
            "all_after_release_contact_count": node.all_after_release_contact_count,
            "all_after_release_collision_names": sorted(node.all_after_release_collision_names),
            "attach_commands": node.attach_command_count,
            "detach_commands": node.detach_command_count,
            "initialization_set_pose_calls": node.initialization_set_pose_calls,
            "task_set_pose_calls": node.task_set_pose_calls,
            "payload_commands": node.payload_command_count,
            "dry_bin_status_samples": node.dry_bin_status_sample_count,
            "latest_dry_bin_status": node.dry_bin_status,
            "latest_dry_bin_status_after_release": node.dry_bin_status_after_release,
            "safety_permitted": node.safety_permitted,
            "safety_permit_events": node.safety_permit_event_count,
        }
        raise
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False))
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
