#!/usr/bin/env python3
"""Execute and measure the formal UR5e and Robotiq controllers in Gazebo."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from formal_runtime_gate_binding import load_binding


ARM_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"
MIMIC_RELATIONS = {
    "robotiq_85_right_knuckle_joint": -1.0,
    "robotiq_85_left_inner_knuckle_joint": 1.0,
    "robotiq_85_right_inner_knuckle_joint": -1.0,
    "robotiq_85_left_finger_tip_joint": -1.0,
    "robotiq_85_right_finger_tip_joint": 1.0,
}
FOLLOWER_EFFORT_LIMIT_NM = 12.0
# A small float/simulator reporting allowance; it is not an additional command limit.
FOLLOWER_EFFORT_TOLERANCE_NM = 0.05
ARM_WAYPOINTS = [
    ([0.00, 0.00, 0.00, 0.00, 0.00, 0.00], 2),
    ([0.12, -0.20, 0.30, -0.15, 0.10, -0.10], 5),
    ([-0.10, -0.25, 0.40, -0.20, -0.15, 0.15], 8),
    ([0.00, 0.00, 0.00, 0.00, 0.00, 0.00], 12),
]
GRIPPER_WAYPOINTS = [([0.00], 2), ([0.65], 5), ([0.00], 8), ([0.20], 10)]
ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SESSION = ROOT / "artifacts/formal_final_acceptance_session.json"


def aligned_joint_state_measurements(
    names: list[str],
    positions: list[float],
    velocities: list[float],
    efforts: list[float],
) -> dict[str, dict[str, float]]:
    """Build a complete name-keyed JointState snapshot or reject it.

    ``sensor_msgs/JointState`` fields are parallel arrays. Never use ``zip``
    here: it would silently discard a named joint when one field is shorter.
    """

    expected_length = len(names)
    field_lengths = {
        "position": len(positions),
        "velocity": len(velocities),
        "effort": len(efforts),
    }
    short_fields = [
        field for field, length in field_lengths.items() if length != expected_length
    ]
    if short_fields:
        raise ValueError(
            "JointState arrays must align with name; "
            f"name={expected_length}, "
            + ", ".join(f"{field}={field_lengths[field]}" for field in short_fields)
        )
    if len(set(names)) != expected_length:
        raise ValueError("JointState has duplicate joint names")
    measurements: dict[str, dict[str, float]] = {}
    for index, name in enumerate(names):
        position = float(positions[index])
        velocity = float(velocities[index])
        effort = float(efforts[index])
        if not all(math.isfinite(value) for value in (position, velocity, effort)):
            raise ValueError(f"JointState has non-finite measurement for {name}")
        measurements[name] = {
            "position_rad": position,
            "velocity_rad_s": velocity,
            "effort_nm": effort,
        }
    return measurements


def snapshot_binding(path: Path) -> dict[str, str]:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    output = manifest.get("outputs", {}).get(
        "reports/engineering/formal_competition_vehicle.urdf"
    )
    source_hash = manifest.get("source_inventory_sha256")
    if not isinstance(output, dict) or not isinstance(output.get("sha256"), str):
        raise RuntimeError("snapshot manifest has no expanded formal vehicle URDF hash")
    if not isinstance(source_hash, str) or not source_hash:
        raise RuntimeError("snapshot manifest has no source inventory hash")
    return {
        "snapshot_manifest_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": output["sha256"],
    }


def bound_runtime_evidence(
    snapshot_path: Path, session_path: Path, binding_path: Path
) -> tuple[dict[str, str], dict[str, object], dict[str, object]]:
    """Reject reports not bound to the active content-verified runtime/session."""

    source = snapshot_binding(snapshot_path)
    session = json.loads(session_path.read_text(encoding="utf-8"))
    if not isinstance(session, dict) or session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise ValueError("formal acceptance session must be RUNNING")
    started_epoch_ns = session.get("started_epoch_ns")
    if not isinstance(started_epoch_ns, int) or started_epoch_ns <= 0:
        raise ValueError("formal acceptance session start time is invalid")
    if started_epoch_ns > time.time_ns():
        raise ValueError("formal acceptance session is future dated")
    binding = load_binding(binding_path)
    bound_session = binding.get("acceptance_session_binding")
    if not isinstance(bound_session, dict):
        raise ValueError("runtime binding has no acceptance-session binding")
    if bound_session.get("snapshot") != source:
        raise ValueError("runtime binding snapshot differs from manipulator source binding")
    if (
        bound_session.get("session_manifest_sha256")
        != hashlib.sha256(session_path.read_bytes()).hexdigest()
        or bound_session.get("session_started_epoch_ns") != started_epoch_ns
        or bound_session.get("session_status_at_gate")
        != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
    ):
        raise ValueError("runtime binding session differs from manipulator session")
    verified_epoch_ns = binding.get("verified_epoch_ns")
    now_ns = time.time_ns()
    if (
        not isinstance(verified_epoch_ns, int)
        or verified_epoch_ns < started_epoch_ns
        or verified_epoch_ns > now_ns
    ):
        raise ValueError("runtime binding timestamp is outside the active acceptance session")
    binding_mtime_ns = binding_path.stat().st_mtime_ns
    if binding_mtime_ns < started_epoch_ns or binding_mtime_ns > now_ns:
        raise ValueError("runtime binding file timestamp is outside the active acceptance session")
    closure = binding.get("runtime_closure_binding")
    if not isinstance(closure, dict):
        raise ValueError("runtime binding has no runtime closure identity")
    if closure.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED":
        raise ValueError("runtime binding closure is not VERIFIED")
    for key in ("manifest_sha256", "closure_sha256"):
        value = closure.get(key)
        if (
            not isinstance(value, str)
            or len(value) != 64
            or any(character not in "0123456789abcdef" for character in value)
        ):
            raise ValueError(f"runtime binding closure has invalid {key}")
    if closure.get("symbolic_link_count") != 0:
        raise ValueError("runtime binding closure contains symbolic links")
    return source, bound_session, binding


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("formal_manipulator_runtime_probe")
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.gripper = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        self.samples: list[dict[str, object]] = []
        self.latest: dict[str, float] = {}
        self.latest_velocity: dict[str, float] = {}
        self.latest_effort: dict[str, float] = {}
        self.joint_state_error: str | None = None
        self.subscription = self.create_subscription(JointState, "/joint_states", self._on_state, 20)

    def _on_state(self, message: JointState) -> None:
        if self.joint_state_error is not None:
            return
        try:
            current = aligned_joint_state_measurements(
                list(message.name),
                list(message.position),
                list(message.velocity),
                list(message.effort),
            )
        except ValueError as exc:
            self.joint_state_error = f"invalid /joint_states measurement: {exc}"
            return
        tracked_joints = ARM_JOINTS + [GRIPPER_JOINT] + list(MIMIC_RELATIONS)
        observed = {
            name: current[name] for name in tracked_joints if name in current
        }
        if observed:
            self.latest.update(
                {name: measurement["position_rad"] for name, measurement in observed.items()}
            )
            self.latest_velocity.update(
                {name: measurement["velocity_rad_s"] for name, measurement in observed.items()}
            )
            self.latest_effort.update(
                {name: measurement["effort_nm"] for name, measurement in observed.items()}
            )
            self.samples.append({"wall_time_s": time.time(), "joints": observed})

    def _raise_if_joint_state_invalid(self) -> None:
        if self.joint_state_error is not None:
            raise RuntimeError(self.joint_state_error)

    def joint_measurements(self, name: str) -> list[dict[str, float]]:
        self._raise_if_joint_state_invalid()
        return [
            sample["joints"][name]  # type: ignore[index]
            for sample in self.samples
            if name in sample["joints"]  # type: ignore[operator]
        ]

    def execute(self, client: ActionClient, joints: list[str], waypoints, timeout_s: float) -> dict:
        self._raise_if_joint_state_invalid()
        if not client.wait_for_server(timeout_sec=timeout_s):
            raise RuntimeError(f"action server unavailable: {client._action_name}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        for positions, seconds in waypoints:
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(sec=seconds)
            goal.trajectory.points.append(point)
        position_tolerance = 0.01 if joints == [GRIPPER_JOINT] else 0.02
        path_tolerance = 0.04 if joints == [GRIPPER_JOINT] else 0.08
        goal.path_tolerance = [JointTolerance(name=name, position=path_tolerance) for name in joints]
        goal.goal_tolerance = [JointTolerance(name=name, position=position_tolerance) for name in joints]
        goal.goal_time_tolerance = Duration(sec=2)
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_s)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"trajectory rejected: {client._action_name}")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_s)
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError(f"trajectory timed out: {client._action_name}")
        result = wrapped.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"trajectory failed on {client._action_name}: {result.error_code} {result.error_string}"
            )
        deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        self._raise_if_joint_state_invalid()
        target = waypoints[-1][0]
        missing = [name for name in joints if name not in self.latest]
        if missing:
            raise RuntimeError("joint states missing: " + ", ".join(missing))
        errors = {name: abs(self.latest[name] - expected) for name, expected in zip(joints, target)}
        if max(errors.values()) > position_tolerance:
            raise RuntimeError(f"terminal tracking error exceeds tolerance: {errors}")
        return {
            "action": client._action_name,
            "accepted": True,
            "result_error_code": int(result.error_code),
            "terminal_error_rad": errors,
            "terminal_max_error_rad": max(errors.values()),
        }

    def follower_runtime_evidence(self) -> dict[str, dict[str, float | int | str]]:
        """Return live /joint_states evidence for every physical gripper follower."""

        self._raise_if_joint_state_invalid()
        evidence: dict[str, dict[str, float | int | str]] = {}
        master_terminal = self.latest[GRIPPER_JOINT]
        for name, multiplier in MIMIC_RELATIONS.items():
            measurements = self.joint_measurements(name)
            synchronized_measurements = [
                (sample["joints"][name], sample["joints"][GRIPPER_JOINT])  # type: ignore[index]
                for sample in self.samples
                if name in sample["joints"] and GRIPPER_JOINT in sample["joints"]  # type: ignore[operator]
            ]
            if (
                not measurements
                or not synchronized_measurements
                or name not in self.latest
                or name not in self.latest_velocity
                or name not in self.latest_effort
            ):
                raise RuntimeError(f"mimic follower lacks live joint-state evidence: {name}")
            positions = [measurement["position_rad"] for measurement in measurements]
            velocities = [measurement["velocity_rad_s"] for measurement in measurements]
            efforts = [measurement["effort_nm"] for measurement in measurements]
            motion_range = max(positions) - min(positions)
            velocity_range = max(velocities) - min(velocities)
            peak_abs_velocity = max(abs(value) for value in velocities)
            peak_abs_effort = max(abs(value) for value in efforts)
            max_noncontact_tracking_error = max(
                abs(follower["position_rad"] - multiplier * master["position_rad"])
                for follower, master in synchronized_measurements
            )
            expected_terminal = multiplier * master_terminal
            terminal_error = abs(self.latest[name] - expected_terminal)
            if (
                motion_range < 0.55
                or terminal_error > 0.02
                or peak_abs_effort > FOLLOWER_EFFORT_LIMIT_NM + FOLLOWER_EFFORT_TOLERANCE_NM
            ):
                raise RuntimeError(
                    "mimic follower runtime tracking failed: "
                    f"{name}; range={motion_range}, terminal_error={terminal_error}, "
                    f"peak_abs_effort_nm={peak_abs_effort}"
                )
            evidence[name] = {
                "status": "LIVE_JOINT_STATE_TRACKED",
                "sample_count": len(measurements),
                "first_position_rad": positions[0],
                "last_sample_position_rad": positions[-1],
                "terminal_position_rad": self.latest[name],
                "motion_range_rad": motion_range,
                "velocity_range_rad_s": velocity_range,
                "peak_abs_velocity_rad_s": peak_abs_velocity,
                "peak_abs_effort_nm": peak_abs_effort,
                "observed_effort_limit_nm": FOLLOWER_EFFORT_LIMIT_NM,
                "effort_tolerance_nm": FOLLOWER_EFFORT_TOLERANCE_NM,
                "effort_within_observed_limit": True,
                "master_multiplier": multiplier,
                "expected_terminal_position_rad": expected_terminal,
                "terminal_error_rad": terminal_error,
                "max_noncontact_tracking_error_rad": max_noncontact_tracking_error,
            }
        return evidence


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--snapshot-manifest", type=Path, required=True)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument(
        "--physics-engine",
        default="gz-physics-dartsim-plugin",
        help="Gazebo physics plugin selected by the launch under test",
    )
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()
    source_binding, acceptance_session_binding, runtime_gate_binding = bound_runtime_evidence(
        args.snapshot_manifest, args.session, args.runtime_binding
    )
    gate_started_epoch_ns = time.time_ns()
    rclpy.init()
    node = Probe()
    try:
        state_deadline = time.monotonic() + args.timeout
        while rclpy.ok() and not all(name in node.latest for name in ARM_JOINTS + [GRIPPER_JOINT]):
            node._raise_if_joint_state_invalid()
            if time.monotonic() >= state_deadline:
                raise RuntimeError("commanded joint states did not become available")
            rclpy.spin_once(node, timeout_sec=0.1)
        node._raise_if_joint_state_invalid()
        arm_result = node.execute(node.arm, ARM_JOINTS, ARM_WAYPOINTS, args.timeout)
        gripper_result = node.execute(node.gripper, [GRIPPER_JOINT], GRIPPER_WAYPOINTS, args.timeout)
        ranges = {}
        for name in ARM_JOINTS + [GRIPPER_JOINT]:
            values = [measurement["position_rad"] for measurement in node.joint_measurements(name)]
            if not values:
                raise RuntimeError(f"joint lacks live position evidence: {name}")
            ranges[name] = max(values) - min(values)
        insufficient = [name for name in ARM_JOINTS if ranges[name] < 0.08]
        if insufficient or ranges[GRIPPER_JOINT] < 0.55:
            raise RuntimeError(f"insufficient measured motion; arm={insufficient}, ranges={ranges}")
        follower_evidence = node.follower_runtime_evidence()
        mimic_observed = sorted(follower_evidence)
        mimic_errors = {
            name: float(follower_evidence[name]["terminal_error_rad"])
            for name in mimic_observed
        }
        report = {
            "report_id": "tzcup_formal_manipulator_runtime_v2",
            "status": "UR5E_AND_ROBOTIQ_GAZEBO_TRAJECTORY_EXECUTION_PASSED",
            "gate_started_epoch_ns": gate_started_epoch_ns,
            "created_epoch_ns": time.time_ns(),
            "physics_engine": args.physics_engine,
            "arm": arm_result,
            "gripper": gripper_result,
            "measured_joint_range_rad": ranges,
            "joint_state_sample_count": len(node.samples),
            "mimic_relations_declared": MIMIC_RELATIONS,
            "mimic_joint_states_observed": mimic_observed,
            "mimic_terminal_error_rad": mimic_errors,
            "mimic_follower_runtime_evidence": follower_evidence,
            "follower_effort_observation_limit_nm": FOLLOWER_EFFORT_LIMIT_NM,
            "follower_effort_observation_tolerance_nm": FOLLOWER_EFFORT_TOLERANCE_NM,
            "all_followers_observed_effort_within_limit": all(
                follower["effort_within_observed_limit"]
                for follower in follower_evidence.values()
            ),
            "follower_evidence_source": "live /joint_states from the running Gazebo controller graph",
            "source_binding": source_binding,
            "acceptance_session_binding": acceptance_session_binding,
            "runtime_gate_binding": runtime_gate_binding,
            "runtime_identity": runtime_gate_binding["runtime_closure_binding"],
            "claim_boundary": (
                "This proves an active-session, frozen-runtime Gazebo execution: controller activation, six-axis "
                "rigid-body motion, master gripper motion, and all five physical follower states measured live on "
                "/joint_states. Follower velocity and effort metrics are measured JointState observations (Nm for "
                "these revolute joints), not plugin JointForceCmd values or a proof of commanded actuator force. "
                "The non-contact tracking error is the maximum live position deviation from the declared mimic ratio "
                "over synchronized JointState samples. Snapshot data only binds source identity; it is not used as a "
                "static-URDF substitute for follower motion. It does not replace MoveIt self-collision planning, motor "
                "thermal/current models, cable-flex analysis or real-hardware validation."
            ),
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
