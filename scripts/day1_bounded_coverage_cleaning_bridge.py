#!/usr/bin/env python3
"""Bridge coverage brush intent to the real cleaning actuators.

This node never publishes vehicle commands and never subscribes to evaluator
truth. It keeps the operator safety heartbeat alive, lowers the physical
cleaning lift after the safety permit is active, and mirrors /brush_enabled
onto the formal brush motor command. Ground-dirt status is recorded as
evaluation evidence only.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Empty, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from day1_bounded_coverage_readiness import precoverage_actuator_readiness


DIRT_STATUS_TOPIC = (
    "/model/tzcup_formal_sanitation_vehicle/ground_dirt/status_json"
)


def _finite_number(value: object, default: float = 0.0) -> float:
    return (
        float(value)
        if isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
        else default
    )


def summarize_status_samples(
    samples: list[dict[str, object]],
) -> dict[str, object]:
    """Return deterministic actuator and dirt metrics from parsed status rows."""
    if not samples:
        return {
            "sample_count": 0,
            "initial": None,
            "terminal": None,
            "cleaned_cell_delta": 0,
            "cleaned_area_delta_m2": 0.0,
            "all_three_ready_sample_count": 0,
            "roller_ready_sample_count": 0,
            "minimum_lift_position_m": None,
            "maximum_roller_velocity_rad_s": 0.0,
        }
    first = samples[0]
    last = samples[-1]
    all_ready = 0
    roller_ready = 0
    minimum_lift = math.inf
    maximum_roller = 0.0
    for sample in samples:
        if all(bool(sample.get(key)) for key in (
            "left_ready", "right_ready", "roller_ready"
        )):
            all_ready += 1
        if bool(sample.get("roller_ready")):
            roller_ready += 1
        minimum_lift = min(
            minimum_lift, _finite_number(sample.get("lift_position_m"))
        )
        maximum_roller = max(
            maximum_roller,
            abs(_finite_number(sample.get("roller_velocity_rad_s"))),
        )
    initial_cells = int(first.get("cleaned_cell_count", 0))
    terminal_cells = int(last.get("cleaned_cell_count", 0))
    initial_area = _finite_number(first.get("cleaned_area_m2"))
    terminal_area = _finite_number(last.get("cleaned_area_m2"))
    return {
        "sample_count": len(samples),
        "initial": first,
        "terminal": last,
        "cleaned_cell_delta": terminal_cells - initial_cells,
        "cleaned_area_delta_m2": terminal_area - initial_area,
        "all_three_ready_sample_count": all_ready,
        "roller_ready_sample_count": roller_ready,
        "minimum_lift_position_m": (
            minimum_lift if math.isfinite(minimum_lift) else None
        ),
        "maximum_roller_velocity_rad_s": maximum_roller,
    }


class BoundedCoverageCleaningBridge(Node):
    def __init__(self, output_dir: Path, stop_file: Path) -> None:
        super().__init__("day1_bounded_coverage_cleaning_bridge")
        self.output_dir = output_dir
        self.stop_file = stop_file
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.started_wall = time.monotonic()
        self.sim_time_s: float | None = None
        self.permitted = False
        self.permit_observed_ever = False
        self.brush_enabled = False
        self.brush_command_enabled = False
        self.lift_requested = False
        self.stop_requested = False
        self.fused_pose_messages = 0
        self.status_samples: list[dict[str, object]] = []
        self.events: list[dict[str, object]] = []
        self.pub = {
            "power": self.create_publisher(
                Bool, "/formal_vehicle/simulation/command/main_power", 10
            ),
            "estop": self.create_publisher(
                Bool, "/formal_vehicle/simulation/command/emergency_stop", 10
            ),
            "reset": self.create_publisher(
                Bool,
                "/formal_vehicle/simulation/command/emergency_stop_reset",
                10,
            ),
            "heartbeat": self.create_publisher(
                Empty, "/safety/control_heartbeat", 10
            ),
            "brush": self.create_publisher(
                Float64MultiArray, "/safety/command/brush", 10
            ),
            "lift": self.create_publisher(
                JointTrajectory, "/cleaning_controller/joint_trajectory", 10
            ),
            "dirt_enable": self.create_publisher(
                Bool,
                "/model/tzcup_formal_sanitation_vehicle/ground_dirt/command/enable",
                10,
            ),
            "fused_pose": self.create_publisher(
                PoseWithCovarianceStamped, "/localization/fused_pose", 20
            ),
        }
        self.create_subscription(
            Clock, "/clock", self._on_clock, qos_profile_sensor_data
        )
        self.create_subscription(
            Bool, "/safety/actuators_enabled", self._on_permit, 10
        )
        self.create_subscription(Bool, "/brush_enabled", self._on_brush, 20)
        self.create_subscription(String, DIRT_STATUS_TOPIC, self._on_dirt, 50)
        self.create_subscription(
            Odometry,
            "/localization/fused_odom",
            self._on_fused_odom,
            qos_profile_sensor_data,
        )
        self.create_timer(0.05, self._tick)

    def _event(self, kind: str, data: object) -> None:
        self.events.append({
            "wall_s": time.monotonic() - self.started_wall,
            "sim_s": self.sim_time_s,
            "kind": kind,
            "data": data,
        })

    def _on_clock(self, message: Clock) -> None:
        self.sim_time_s = (
            float(message.clock.sec) + float(message.clock.nanosec) * 1e-9
        )

    def _on_permit(self, message: Bool) -> None:
        if bool(message.data) != self.permitted:
            self._event("permit", bool(message.data))
        self.permitted = bool(message.data)
        if self.permitted:
            self.permit_observed_ever = True

    def _on_brush(self, message: Bool) -> None:
        enabled = bool(message.data)
        if enabled != self.brush_enabled:
            self._event("brush_intent", enabled)
        self.brush_enabled = enabled

    def _on_fused_odom(self, message: Odometry) -> None:
        output = PoseWithCovarianceStamped()
        output.header = message.header
        output.pose = message.pose
        self.pub["fused_pose"].publish(output)
        self.fused_pose_messages += 1

    def _on_dirt(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(payload, dict):
            return
        sample = {
            key: payload.get(key)
            for key in (
                "enabled",
                "cell_layout_ready",
                "cell_count",
                "cleaned_cell_count",
                "initial_area_m2",
                "cleaned_area_m2",
                "remaining_area_m2",
                "cleaned_fraction",
                "area_balance_error_m2",
                "lift_position_m",
                "left_velocity_rad_s",
                "right_velocity_rad_s",
                "roller_velocity_rad_s",
                "left_ready",
                "right_ready",
                "roller_ready",
                "left_clearance_m",
                "right_clearance_m",
                "roller_clearance_m",
                "sim_time_s",
                "rigid_litter_entities_modified",
            )
        }
        self.status_samples.append(sample)
        if len(self.status_samples) > 20000:
            del self.status_samples[:1000]
        finish_ready = self._finish_ready()
        if finish_ready:
            ready_path = self.output_dir / "cleaning_bridge_ready.json"
            if not ready_path.exists():
                readiness = precoverage_actuator_readiness(
                    self.status_samples, self.permitted
                )
                ready_payload = {
                    "schema_version": 1,
                    "ready": True,
                    "ready_scope": "precoverage_work_pose",
                    "permitted": self.permitted,
                    "brush_intent": self.brush_enabled,
                    "readiness": readiness,
                    "status": sample,
                    "wall_s": time.monotonic() - self.started_wall,
                }
                temporary = ready_path.with_suffix(".json.tmp")
                temporary.write_text(
                    json.dumps(ready_payload, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
                )
                temporary.replace(ready_path)
                self._event("cleaning_actuators_ready", sample)

    def _finish_ready(self) -> bool:
        return bool(
            precoverage_actuator_readiness(
                self.status_samples, self.permitted
            )["ready"]
        )

    def _publish_lift_request(self) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = ["cleaning_lift_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.10]
        point.time_from_start.sec = 20
        point.time_from_start.nanosec = 900000000
        trajectory.points = [point]
        self.pub["lift"].publish(trajectory)
        self.lift_requested = True
        self._event("cleaning_lift_requested", 0.10)

    def _tick(self) -> None:
        if self.stop_file.exists():
            self.stop_requested = True
        now = time.monotonic()
        self.pub["power"].publish(Bool(data=True))
        self.pub["estop"].publish(Bool(data=self.stop_requested))
        self.pub["reset"].publish(Bool(data=not self.stop_requested))
        self.pub["heartbeat"].publish(Empty())
        self.pub["dirt_enable"].publish(Bool(data=True))
        command_enabled = self.brush_enabled and not self.stop_requested
        if command_enabled != self.brush_command_enabled:
            self._event("brush_command", command_enabled)
            self.brush_command_enabled = command_enabled
        self.pub["brush"].publish(
            Float64MultiArray(
                data=[8.0, -8.0, 12.0] if command_enabled else [0.0, 0.0, 0.0]
            )
        )
        if (
            self.permitted
            and not self.lift_requested
            and self.pub["lift"].get_subscription_count() > 0
        ):
            self._publish_lift_request()

    def finalize(self, reason: str) -> dict[str, object]:
        # Publish a bounded release state after the coverage process stops.
        for _ in range(10):
            self.pub["brush"].publish(
                Float64MultiArray(data=[0.0, 0.0, 0.0])
            )
            self.pub["estop"].publish(Bool(data=True))
            self.pub["reset"].publish(Bool(data=False))
            self.pub["dirt_enable"].publish(Bool(data=False))
            rclpy.spin_once(self, timeout_sec=0.05)
        report = {
            "schema_version": 1,
            "artifact_kind": "day1_bounded_coverage_cleaning_bridge",
            "reason": reason,
            "ground_truth_subscribed": False,
            "vehicle_command_publishers": [],
            "fused_pose_adapter_messages": self.fused_pose_messages,
            "brush_command_on": [8.0, -8.0, 12.0],
            "brush_command_off": [0.0, 0.0, 0.0],
            "permit_observed": self.permit_observed_ever,
            "permit_terminal": self.permitted,
            "lift_requested": self.lift_requested,
            "brush_disabled_on_exit": True,
            "dirt_system_disabled_on_exit": True,
            **summarize_status_samples(self.status_samples),
            "events": self.events,
        }
        output = self.output_dir / "cleaning_bridge.json"
        temporary = output.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary.replace(output)
        (self.output_dir / "cleaning_bridge_timeline.jsonl").write_text(
            "".join(
                json.dumps(row, sort_keys=True) + "\n" for row in self.events
            ),
            encoding="utf-8",
        )
        return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--stop-file", type=Path, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = BoundedCoverageCleaningBridge(args.output_dir, args.stop_file)

    def stop_handler(_signum, _frame):  # type: ignore[no-untyped-def]
        node.stop_requested = True

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    reason = "stop_file"
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        reason = "signal"
    finally:
        node.finalize(reason)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
