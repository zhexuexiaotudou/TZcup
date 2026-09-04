#!/usr/bin/env python3
"""Machine-accept the formal vehicle's physical ground-dirt sweep."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import qos_profile_clock
from std_msgs.msg import Bool, Empty, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from formal_cleaning_lift_recovery_core import trajectory_duration_s
from formal_runtime_gate_binding import load_binding


ROOT = "/model/tzcup_formal_sanitation_vehicle/ground_dirt"
VEHICLE = "tzcup_formal_sanitation_vehicle"
SIM_CLOCK_STALL_WALL_S = 45.0
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SNAPSHOT = REPOSITORY_ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
DEFAULT_SESSION = REPOSITORY_ROOT / "artifacts/formal_final_acceptance_session.json"
REPORT_ID = "tzcup_formal_ground_dirt_physical_cleaning_v1"
CLEANING_LIFT_WORK_READY_M = 0.095
CLEANING_LIFT_POSE_TIMEOUT_SIM_S = 45.0


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
) -> tuple[dict[str, object], dict[str, object]]:
    """Reject ground-dirt evidence detached from the current frozen run."""

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
        raise ValueError("runtime binding snapshot differs from ground-dirt source binding")
    if (
        bound_session.get("session_manifest_sha256")
        != hashlib.sha256(session_path.read_bytes()).hexdigest()
        or bound_session.get("session_started_epoch_ns") != started_epoch_ns
        or Path(str(bound_session.get("session_manifest", ""))).resolve()
        != session_path.resolve()
    ):
        raise ValueError("runtime binding session differs from ground-dirt session")
    return binding, bound_session


class Probe(Node):
    def __init__(self, world: str) -> None:
        super().__init__("formal_ground_dirt_acceptance")
        self.status: dict[str, object] | None = None
        self.samples: list[dict[str, object]] = []
        self.sim_time_s: float | None = None
        self.task_started = False
        self.initialization_set_pose_calls = 0
        self.task_set_pose_calls = 0
        self.enable = self.create_publisher(Bool, f"{ROOT}/command/enable", 10)
        self.cleaning = self.create_publisher(
            JointTrajectory, "/cleaning_controller/joint_trajectory", 10
        )
        self.brush = self.create_publisher(
            Float64MultiArray, "/safety/command/brush", 10
        )
        self.drive = self.create_publisher(Twist, "/cmd_vel_gate", 10)
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
        self.set_pose = self.create_client(SetEntityPose, f"/world/{world}/set_pose")
        self.create_subscription(String, f"{ROOT}/status_json", self._on_status, 50)
        self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_clock)

    def _on_status(self, message: String) -> None:
        self.status = json.loads(message.data)
        self.samples.append(dict(self.status))

    def _on_clock(self, message: Clock) -> None:
        self.sim_time_s = message.clock.sec + message.clock.nanosec * 1e-9

    def command_pose(self, lift_m: float) -> None:
        point = JointTrajectoryPoint()
        point.positions = [lift_m]
        current_m = (
            float(self.status["lift_position_m"])
            if self.status is not None and "lift_position_m" in self.status
            else 0.0
        )
        duration_ns = int(round(trajectory_duration_s(current_m, lift_m) * 1_000_000_000))
        point.time_from_start.sec = duration_ns // 1_000_000_000
        point.time_from_start.nanosec = duration_ns % 1_000_000_000
        trajectory = JointTrajectory()
        trajectory.joint_names = ["cleaning_lift_joint"]
        trajectory.points = [point]
        self.cleaning.publish(trajectory)

    def command(self, *, enabled: bool, brushes: bool, speed: float) -> None:
        self.main_power.publish(Bool(data=True))
        self.estop.publish(Bool(data=False))
        self.estop_reset.publish(Bool(data=True))
        self.heartbeat.publish(Empty())
        self.enable.publish(Bool(data=enabled))
        self.brush.publish(
            Float64MultiArray(
                data=[8.0, -8.0, 12.0] if brushes else [0.0, 0.0, 0.0]
            )
        )
        twist = Twist()
        twist.linear.x = speed
        self.drive.publish(twist)

    def stop(self) -> None:
        self.command(enabled=False, brushes=False, speed=0.0)

    def initialize_vehicle(self, x: float, y: float, yaw: float) -> None:
        if self.task_started:
            self.task_set_pose_calls += 1
            raise RuntimeError("SetEntityPose is evaluator-only and forbidden after task start")
        if not self.set_pose.wait_for_service(timeout_sec=20.0):
            raise RuntimeError("SetEntityPose evaluator service unavailable")
        request = SetEntityPose.Request()
        request.entity.name = VEHICLE
        request.entity.type = Entity.MODEL
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = 0.005
        request.pose.orientation.z = math.sin(yaw / 2.0)
        request.pose.orientation.w = math.cos(yaw / 2.0)
        future = self.set_pose.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("formal vehicle initialization pose failed")
        self.initialization_set_pose_calls += 1


def wait_condition(
    node: Probe,
    predicate,
    *,
    label: str,
    timeout_sim_s: float,
    hard_wall_s: float,
    callback=None,
) -> None:
    wall_started = time.monotonic()
    last_progress_wall = wall_started
    sim_started: float | None = None
    last_sim: float | None = None
    while True:
        if callback is not None:
            callback()
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
        now = time.monotonic()
        if node.sim_time_s is not None:
            if sim_started is None:
                sim_started = node.sim_time_s
            if last_sim is None or node.sim_time_s > last_sim + 1e-9:
                last_sim = node.sim_time_s
                last_progress_wall = now
            if node.sim_time_s - sim_started >= timeout_sim_s:
                raise RuntimeError(f"{label} exceeded simulated timeout: {node.status}")
        if now - last_progress_wall >= SIM_CLOCK_STALL_WALL_S:
            raise RuntimeError(f"simulation clock stalled while waiting for {label}")
        if now - wall_started >= hard_wall_s:
            raise RuntimeError(f"{label} exceeded wall timeout: {node.status}")


def advance(node: Probe, seconds: float, label: str, callback) -> None:
    started: list[float | None] = [None]

    def done() -> bool:
        if node.sim_time_s is None:
            return False
        if started[0] is None:
            started[0] = node.sim_time_s
        return node.sim_time_s - started[0] >= seconds

    wait_condition(
        node,
        done,
        label=label,
        timeout_sim_s=seconds + 1.0,
        hard_wall_s=max(180.0, seconds * 30.0),
        callback=callback,
    )


def wait_ready(node: Probe) -> None:
    deadline = time.monotonic() + 45.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (
            node.status is not None
            and bool(node.status.get("cell_layout_ready"))
            and node.enable.get_subscription_count() > 0
            and node.cleaning.get_subscription_count() > 0
            and node.brush.get_subscription_count() > 0
            and node.drive.get_subscription_count() > 0
        ):
            return
    raise RuntimeError(f"ground dirt plugin or controllers did not become ready: {node.status}")


def set_work_pose(node: Probe, lift_m: float) -> None:
    for _ in range(5):
        node.command_pose(lift_m)
        node.command(enabled=False, brushes=False, speed=0.0)
        rclpy.spin_once(node, timeout_sec=0.08)
    predicate = (
        (
            lambda: node.status is not None
            and float(node.status["lift_position_m"]) >= CLEANING_LIFT_WORK_READY_M
        )
        if lift_m > 0.05
        else (lambda: node.status is not None and float(node.status["lift_position_m"]) <= 0.04)
    )
    wait_condition(
        node,
        predicate,
        label=f"cleaning lift {lift_m:.3f} m",
        timeout_sim_s=CLEANING_LIFT_POSE_TIMEOUT_SIM_S,
        hard_wall_s=360.0,
        callback=lambda: node.command(enabled=False, brushes=False, speed=0.0),
    )


def gz_pose_snapshot(world: str, names: list[str]) -> dict[str, bool]:
    executable = shutil.which("gz") or "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz"
    result = subprocess.run(
        [executable, "topic", "-e", "-t", f"/world/{world}/pose/info", "-n", "1"],
        check=True,
        capture_output=True,
        text=True,
        timeout=15.0,
    )
    return {name: f'name: "{name}"' in result.stdout for name in names}


def run(node: Probe, setup: dict[str, object]) -> dict[str, object]:
    patch = setup["patch"]
    pose = patch["pose"]
    yaw = float(pose["yaw_rad"])
    half_length = float(patch["size_m"][0]) / 2.0
    # Put the central roller 0.10 m before the first cell centre.  SetPose is
    # evaluator-only initialization; all accepted cleaning after task start is
    # caused by diff-drive motion and real brush joint states.
    base_local_x = -half_length - 0.155 - 0.10
    base_x = float(pose["x_m"]) + math.cos(yaw) * base_local_x
    base_y = float(pose["y_m"]) + math.sin(yaw) * base_local_x
    initial_area = float(setup["initial_area_m2"])
    litter_ids = list(setup["rigid_litter_ids"])

    def reset_initialization() -> None:
        node.stop()
        node.initialize_vehicle(base_x, base_y, yaw)
        advance(
            node,
            0.5,
            "initialization settling",
            lambda: node.command(enabled=False, brushes=False, speed=0.0),
        )

    # Disabled negative gate: lowered and rotating while physically moving.
    reset_initialization()
    set_work_pose(node, 0.100)
    advance(
        node,
        5.0,
        "disabled negative gate",
        lambda: (node.command_pose(0.100), node.command(enabled=False, brushes=True, speed=0.05)),
    )
    disabled = dict(node.status or {})

    # Raised negative gate: enabled and rotating while physically moving.
    reset_initialization()
    set_work_pose(node, 0.0)
    advance(
        node,
        5.0,
        "raised negative gate",
        lambda: node.command(enabled=True, brushes=True, speed=0.05),
    )
    raised = dict(node.status or {})

    # Stopped negative gate: lowered and enabled, but every brush is stationary.
    reset_initialization()
    set_work_pose(node, 0.100)
    advance(
        node,
        5.0,
        "stopped-brush negative gate",
        lambda: (node.command_pose(0.100), node.command(enabled=True, brushes=False, speed=0.05)),
    )
    stopped = dict(node.status or {})

    reset_initialization()
    before_litter = gz_pose_snapshot(str(setup["world_name"]), litter_ids)
    node.task_started = True
    node.samples.clear()

    wait_condition(
        node,
        lambda: node.status is not None and float(node.status["cleaned_fraction"]) >= 0.45,
        label="physical partial dirt pass",
        timeout_sim_s=35.0,
        hard_wall_s=900.0,
        callback=lambda: (node.command_pose(0.100), node.command(enabled=True, brushes=True, speed=0.06)),
    )
    node.command(enabled=True, brushes=True, speed=0.0)
    advance(
        node,
        0.5,
        "partial pass stop",
        lambda: node.command(enabled=True, brushes=True, speed=0.0),
    )
    partial = dict(node.status or {})

    wait_condition(
        node,
        lambda: node.status is not None and float(node.status["cleaned_fraction"]) >= 0.95,
        label="physical 95 percent dirt pass",
        timeout_sim_s=40.0,
        hard_wall_s=1_200.0,
        callback=lambda: (node.command_pose(0.100), node.command(enabled=True, brushes=True, speed=0.06)),
    )
    node.stop()
    advance(node, 1.0, "final stop", node.stop)
    final = dict(node.status or {})
    after_litter = gz_pose_snapshot(str(setup["world_name"]), litter_ids)

    balance = abs(
        float(final["initial_area_m2"])
        - float(final["cleaned_area_m2"])
        - float(final["remaining_area_m2"])
    )
    ready_samples = [sample for sample in node.samples if bool(sample.get("roller_ready"))]
    checks = {
        "random_generator_cell_area_matches_truth": abs(
            float(final["initial_area_m2"]) - initial_area
        ) <= 1e-9,
        "disabled_gate_removes_zero_area": abs(float(disabled["cleaned_area_m2"])) <= 1e-9,
        "raised_gate_removes_zero_area": abs(float(raised["cleaned_area_m2"])) <= 1e-9,
        "stopped_gate_removes_zero_area": abs(float(stopped["cleaned_area_m2"])) <= 1e-9,
        "partial_pass_is_strictly_partial": 0.40 <= float(partial["cleaned_fraction"]) < 0.95,
        "physical_sweep_reaches_95_percent": float(final["cleaned_fraction"]) >= 0.95,
        "area_mass_conservation_exact": balance <= 1e-9
        and float(final["area_balance_error_m2"]) <= 1e-9,
        "real_joint_and_world_pose_ready_samples_seen": bool(ready_samples)
        and any(abs(float(sample["roller_velocity_rad_s"])) >= 2.0 for sample in ready_samples)
        and any(abs(float(sample["roller_world_x"])) > 1.0 for sample in ready_samples),
        "no_task_set_pose_after_start": node.task_set_pose_calls == 0,
        "all_rigid_litter_models_remain": all(before_litter.values())
        and all(after_litter.values())
        and before_litter.keys() == after_litter.keys(),
        "plugin_reports_no_rigid_litter_mutation": int(
            final["rigid_litter_entities_modified"]
        ) == 0,
    }
    return {
        "schema_version": 1,
        "report_id": REPORT_ID,
        "status": "FORMAL_GROUND_DIRT_PHYSICAL_CLEANING_PASSED"
        if all(checks.values())
        else "FAILED",
        "passed": all(checks.values()),
        "checks": checks,
        "setup": setup,
        "disabled_terminal": disabled,
        "raised_terminal": raised,
        "stopped_terminal": stopped,
        "partial_terminal": partial,
        "final": final,
        "metrics": {
            "initial_area_m2": float(final["initial_area_m2"]),
            "partial_cleaned_fraction": float(partial["cleaned_fraction"]),
            "final_cleaned_fraction": float(final["cleaned_fraction"]),
            "area_balance_error_m2": balance,
            "ready_sample_count": len(ready_samples),
            "initialization_set_pose_calls": node.initialization_set_pose_calls,
            "task_set_pose_calls": node.task_set_pose_calls,
            "rigid_litter_count": len(litter_ids),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--setup", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    args = parser.parse_args()
    setup = json.loads(args.setup.read_text(encoding="utf-8"))
    runtime_binding, acceptance_session_binding = _bound_runtime_evidence(
        args.snapshot, args.session, args.runtime_binding
    )
    rclpy.init()
    node = Probe(str(setup["world_name"]))
    try:
        wait_ready(node)
        result = run(node, setup)
    except Exception as exc:
        result = {
            "schema_version": 1,
            "status": "FAILED",
            "passed": False,
            "checks": {},
            "error": f"{type(exc).__name__}: {exc}",
        }
    finally:
        node.stop()
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()
    result["report_id"] = REPORT_ID
    result["runtime_gate_binding"] = runtime_binding
    result["acceptance_session_binding"] = acceptance_session_binding
    result["runtime_closure_binding"] = runtime_binding["runtime_closure_binding"]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
