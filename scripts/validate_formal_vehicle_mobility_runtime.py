#!/usr/bin/env python3
"""Drive the formal vehicle forward, stop it, and score physical motion evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path
from typing import Any

import rclpy
from controller_manager_msgs.srv import ListControllers
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty

from formal_vehicle_mobility_metrics import WHEEL_JOINTS, evaluate_motion, quaternion_yaw
from formal_runtime_gate_binding import load_binding
from gazebo_ground_truth import read_named_model_pose

MODEL_NAME = "tzcup_formal_sanitation_vehicle"
DEFAULT_CLOCK_STALL_TIMEOUT_S = 20.0
DEFAULT_PHASE_HARD_TIMEOUT_S = 600.0
ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SNAPSHOT = ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
DEFAULT_SESSION = ROOT / "artifacts/formal_final_acceptance_session.json"


def _source_binding(snapshot_path: Path) -> dict[str, str]:
    """Extract the immutable vehicle identity from the canonical snapshot."""

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
) -> tuple[dict[str, str], dict[str, object], dict[str, object]]:
    """Reject a mobility result that is not from the active frozen final run."""

    source_binding = _source_binding(snapshot_path)
    session = json.loads(session_path.read_text(encoding="utf-8"))
    if not isinstance(session, dict) or session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise ValueError("formal acceptance session must be RUNNING")
    started_epoch_ns = session.get("started_epoch_ns")
    if not isinstance(started_epoch_ns, int) or started_epoch_ns <= 0:
        raise ValueError("formal acceptance session start time is invalid")
    binding = load_binding(binding_path)
    bound_session = binding["acceptance_session_binding"]
    if not isinstance(bound_session, dict):
        raise ValueError("runtime binding has no acceptance-session binding")
    if bound_session.get("snapshot") != source_binding:
        raise ValueError("runtime binding snapshot differs from mobility source binding")
    if (
        bound_session.get("session_manifest_sha256")
        != hashlib.sha256(session_path.read_bytes()).hexdigest()
        or bound_session.get("session_started_epoch_ns") != started_epoch_ns
    ):
        raise ValueError("runtime binding session differs from mobility session")
    return source_binding, bound_session, binding


class SimulationClockProgressWatchdog:
    """Fail on a stalled simulation clock while tolerating a low real-time factor."""

    def __init__(
        self,
        initial_sim_ns: int,
        initial_wall_s: float,
        stall_timeout_s: float,
        hard_timeout_s: float,
    ) -> None:
        if stall_timeout_s <= 0.0:
            raise ValueError("clock stall timeout must be positive")
        if hard_timeout_s <= 0.0:
            raise ValueError("phase hard timeout must be positive")
        if hard_timeout_s <= stall_timeout_s:
            raise ValueError("phase hard timeout must exceed clock stall timeout")
        self.initial_wall_s = initial_wall_s
        self.last_progress_wall_s = initial_wall_s
        self.last_sim_ns = initial_sim_ns
        self.stall_timeout_s = stall_timeout_s
        self.hard_timeout_s = hard_timeout_s

    def observe(self, sim_ns: int, wall_s: float) -> None:
        wall_elapsed = wall_s - self.initial_wall_s
        if wall_elapsed >= self.hard_timeout_s:
            raise TimeoutError(
                f"mobility phase exceeded the {self.hard_timeout_s:.1f} s hard wall limit"
            )
        if sim_ns < self.last_sim_ns:
            raise TimeoutError("simulation clock moved backwards during mobility command")
        if sim_ns > self.last_sim_ns:
            self.last_sim_ns = sim_ns
            self.last_progress_wall_s = wall_s
            return
        stalled_for = wall_s - self.last_progress_wall_s
        if stalled_for >= self.stall_timeout_s:
            raise TimeoutError(
                "simulation clock made no progress for "
                f"{stalled_for:.1f} s during mobility command"
            )


def read_gazebo_ground_truth() -> dict[str, float]:
    """Read one named model pose directly from Gazebo Transport, preserving names."""
    pose = read_named_model_pose(
        world_name="formal_vehicle_validation", model_name=MODEL_NAME
    )
    return {key: pose[key] for key in ("x", "y", "yaw")}


class MobilityProbe(Node):
    def __init__(self) -> None:
        super().__init__(
            "formal_vehicle_mobility_runtime_probe",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
            automatically_declare_parameters_from_overrides=True,
        )
        self.command = self.create_publisher(Twist, "/cmd_vel_gate", 10)
        self.main_power = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/main_power", 10
        )
        self.estop = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/emergency_stop", 10
        )
        self.estop_reset = self.create_publisher(
            Bool, "/formal_vehicle/simulation/command/emergency_stop_reset", 10
        )
        self.heartbeat = self.create_publisher(Empty, "/safety/control_heartbeat", 10)
        self.create_subscription(Odometry, "/odom/unfiltered", self._odom, 50)
        self.create_subscription(JointState, "/joint_states", self._joints, 50)
        self.create_subscription(
            Bool, "/safety/actuators_enabled", self._actuator_enabled, 20
        )
        self.create_subscription(
            DiagnosticArray, "/safety/status", self._safety_status, 20
        )
        self.controller_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.latest_odom: dict[str, Any] | None = None
        self.latest_wheels: dict[str, dict[str, float]] | None = None
        self.odom_samples = 0
        self.joint_samples = 0
        self.actuator_enabled_samples = 0
        self.latest_safety_status: dict[str, str] = {}

    def _actuator_enabled(self, message: Bool) -> None:
        if message.data:
            self.actuator_enabled_samples += 1

    def _safety_status(self, message: DiagnosticArray) -> None:
        for status in message.status:
            self.latest_safety_status = {
                item.key: item.value for item in status.values
            }

    def _odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        self.latest_odom = {
            "pose": {
                "x": pose.position.x,
                "y": pose.position.y,
                "yaw": quaternion_yaw(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            },
            "linear_velocity_mps": {"x": twist.linear.x, "y": twist.linear.y},
            "angular_velocity_rad_s": twist.angular.z,
        }
        self.odom_samples += 1

    def _joints(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        if all(name in positions and name in velocities for name in WHEEL_JOINTS):
            self.latest_wheels = {
                "positions": {name: float(positions[name]) for name in WHEEL_JOINTS},
                "velocities": {name: float(velocities[name]) for name in WHEEL_JOINTS},
            }
            self.joint_samples += 1

    def publish_velocity(self, speed: float) -> None:
        self.main_power.publish(Bool(data=True))
        self.estop.publish(Bool(data=False))
        self.estop_reset.publish(Bool(data=True))
        self.heartbeat.publish(Empty())
        command = Twist()
        command.linear.x = speed
        self.command.publish(command)

    def spin_for(
        self,
        duration: float,
        speed: float,
        rate_hz: float = 20.0,
        clock_stall_timeout_s: float = DEFAULT_CLOCK_STALL_TIMEOUT_S,
        hard_wall_timeout_s: float = DEFAULT_PHASE_HARD_TIMEOUT_S,
    ) -> dict[str, Any]:
        if duration < 0.0:
            raise ValueError("simulated duration cannot be negative")
        if rate_hz <= 0.0:
            raise ValueError("command publication rate must be positive")
        wall_start = time.monotonic()
        sim_start = self.get_clock().now().nanoseconds
        watchdog = SimulationClockProgressWatchdog(
            sim_start,
            wall_start,
            clock_stall_timeout_s,
            hard_wall_timeout_s,
        )
        period = 1.0 / rate_hz
        simulated = 0.0
        while simulated < duration:
            watchdog.observe(self.get_clock().now().nanoseconds, time.monotonic())
            self.publish_velocity(speed)
            rclpy.spin_once(self, timeout_sec=period)
            sim_now = self.get_clock().now().nanoseconds
            wall_now = time.monotonic()
            watchdog.observe(sim_now, wall_now)
            simulated = (sim_now - sim_start) / 1e9
        measured_wall_s = time.monotonic() - wall_start
        return {
            "requested_simulated_s": duration,
            "measured_simulated_s": simulated,
            "measured_wall_s": measured_wall_s,
            "observed_realtime_factor": simulated / measured_wall_s if measured_wall_s > 0.0 else None,
            "clock_stall_timeout_s": clock_stall_timeout_s,
            "hard_wall_timeout_s": hard_wall_timeout_s,
            "use_sim_time": bool(self.get_parameter("use_sim_time").value),
        }

    def snapshot(self) -> tuple[dict[str, Any], dict[str, Any], dict[str, dict[str, float]]]:
        if self.latest_odom is None or self.latest_wheels is None:
            raise RuntimeError("mobility evidence streams are incomplete")
        return (
            read_gazebo_ground_truth(),
            json.loads(json.dumps(self.latest_odom)),
            json.loads(json.dumps(self.latest_wheels)),
        )


def _wait_for_ready(node: MobilityProbe, timeout: float) -> str:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        node.publish_velocity(0.0)
        ready = (
            node.latest_odom is not None
            and node.latest_wheels is not None
            and node.command.get_subscription_count() >= 1
            and node.main_power.get_subscription_count() >= 2
            and node.controller_client.wait_for_service(timeout_sec=0.0)
        )
        if ready:
            future = node.controller_client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            if future.done() and future.result() is not None:
                states = {item.name: item.state for item in future.result().controller}
                if (
                    states.get("joint_state_broadcaster") == "active"
                    and node.actuator_enabled_samples > 0
                ):
                    return "active"
        time.sleep(0.05)
    raise TimeoutError(
        "timed out waiting for the safety command chain, enabled A300 plant odometry, "
        "active joint-state broadcaster and all four wheel joints; "
        f"odom_seen={node.latest_odom is not None}, "
        f"wheels_seen={node.latest_wheels is not None}, "
        f"command_subscriptions={node.command.get_subscription_count()}, "
        f"main_power_subscriptions={node.main_power.get_subscription_count()}, "
        f"controller_service={node.controller_client.service_is_ready()}, "
        f"actuator_enabled_samples={node.actuator_enabled_samples}, "
        f"odom_samples={node.odom_samples}, joint_samples={node.joint_samples}, "
        f"safety_status={node.latest_safety_status}"
    )


def run(
    output: Path,
    timeout: float,
    forward_speed: float,
    forward_duration: float,
    source_binding: dict[str, str],
    acceptance_session_binding: dict[str, object],
    runtime_gate_binding: dict[str, object],
    clock_stall_timeout: float = DEFAULT_CLOCK_STALL_TIMEOUT_S,
    phase_hard_timeout: float = DEFAULT_PHASE_HARD_TIMEOUT_S,
) -> dict[str, Any]:
    rclpy.init()
    node = MobilityProbe()
    try:
        joint_state_broadcaster_state = _wait_for_ready(node, timeout)
        settle_timing = node.spin_for(
            1.0,
            0.0,
            clock_stall_timeout_s=clock_stall_timeout,
            hard_wall_timeout_s=phase_hard_timeout,
        )
        ground_start, odom_start, wheels_start = node.snapshot()
        forward_timing = node.spin_for(
            forward_duration,
            forward_speed,
            clock_stall_timeout_s=clock_stall_timeout,
            hard_wall_timeout_s=phase_hard_timeout,
        )
        ground_forward, odom_forward, wheels_forward = node.snapshot()
        stopped_timing = node.spin_for(
            3.0,
            0.0,
            clock_stall_timeout_s=clock_stall_timeout,
            hard_wall_timeout_s=phase_hard_timeout,
        )
        ground_stopped, odom_stopped, wheels_stopped = node.snapshot()
        raw = {
            "joint_state_broadcaster_state": joint_state_broadcaster_state,
            "command_subscription_count": node.command.get_subscription_count(),
            "actuator_enabled_sample_count": node.actuator_enabled_samples,
            "ground_truth": {
                "start": ground_start,
                "forward_end": ground_forward,
                "stopped_end": ground_stopped,
            },
            "plant_odom": {
                "start": odom_start["pose"],
                "forward_end": odom_forward["pose"],
                "stopped_end": odom_stopped["pose"],
                "stopped_linear_velocity_mps": odom_stopped["linear_velocity_mps"],
                "stopped_angular_velocity_rad_s": odom_stopped["angular_velocity_rad_s"],
            },
            "wheel_state": {
                "observed_names": list(WHEEL_JOINTS),
                "start_positions_rad": wheels_start["positions"],
                "forward_end_positions_rad": wheels_forward["positions"],
                "stopped_velocities_rad_s": wheels_stopped["velocities"],
            },
        }
        evaluation = evaluate_motion(raw)
        report = {
            "report_id": "tzcup_formal_a300_drivetrain_runtime_v1",
            "status": "FORMAL_A300_DRIVETRAIN_FORWARD_STOP_RUNTIME_PASSED" if evaluation["passed"] else "FORMAL_A300_DRIVETRAIN_FORWARD_STOP_RUNTIME_FAILED",
            "command": {
                "product_input_topic": "/cmd_vel_gate",
                "safety_output_topic": "/base_controller/cmd_vel",
                "plant_odometry_topic": "/odom/unfiltered",
                "forward_speed_mps": forward_speed,
                "forward_duration_s": forward_duration,
                "zero_command_duration_s": 3.0,
                "timing_source": "/clock via rclpy use_sim_time",
                "settle_timing": settle_timing,
                "forward_timing": forward_timing,
                "zero_timing": stopped_timing,
            },
            "sample_counts": {
                "gazebo_ground_truth_pose": 3,
                "plant_odometry": node.odom_samples,
                "joint_states": node.joint_samples,
            },
            "source_binding": source_binding,
            "acceptance_session_binding": acceptance_session_binding,
            "runtime_gate_binding": runtime_gate_binding,
            "raw_evidence": raw,
            **evaluation,
            "claim_boundary": "This proves commanded straight-ahead physical motion and stopping in Gazebo through the product Twist gate, sole whole-vehicle safety writer, typed A300 adapter, effort plant, independent world pose, raw plant odometry and all four wheel joints. It does not prove path tracking, obstacle avoidance, hardware-correlated traction or real-vehicle braking distance.",
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(report, sort_keys=True))
        if not evaluation["passed"]:
            raise SystemExit(1)
        return report
    finally:
        for _ in range(10):
            node.publish_velocity(0.0)
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--forward-speed", type=float, default=0.25)
    parser.add_argument("--forward-duration", type=float, default=4.0)
    parser.add_argument(
        "--clock-stall-timeout",
        type=float,
        default=DEFAULT_CLOCK_STALL_TIMEOUT_S,
        help="Fail when /clock makes no progress for this many wall seconds.",
    )
    parser.add_argument(
        "--phase-hard-timeout",
        type=float,
        default=DEFAULT_PHASE_HARD_TIMEOUT_S,
        help="Absolute wall-time ceiling for each simulated-time phase.",
    )
    args = parser.parse_args()
    source_binding, acceptance_session_binding, runtime_gate_binding = _bound_runtime_evidence(
        args.snapshot, args.session, args.runtime_binding
    )
    run(
        args.output,
        args.timeout,
        args.forward_speed,
        args.forward_duration,
        source_binding,
        acceptance_session_binding,
        runtime_gate_binding,
        args.clock_stall_timeout,
        args.phase_hard_timeout,
    )


if __name__ == "__main__":
    main()
