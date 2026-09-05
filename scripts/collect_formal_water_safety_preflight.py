#!/usr/bin/env python3
"""Record and gate the formal water scenario's safety telemetry before motion."""

from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Float64MultiArray, String

from formal_cleaning_motor_telemetry import (
    decode_cleaning_motor_telemetry,
    update_physics_revision_watchdog,
)


MOTOR_ROOT = "/model/tzcup_formal_sanitation_vehicle/cleaning_motors"
COMMAND_PERIOD_S = 0.05
POLL_PERIOD_S = 0.005


class SafetyPreflight(Node):
    def __init__(self) -> None:
        super().__init__("formal_water_safety_preflight")
        self._state_lock = threading.RLock()
        self._callback_group = ReentrantCallbackGroup()
        self.started_monotonic_s = time.monotonic()
        self.events: list[dict[str, object]] = []
        self.fault_active: bool | None = None
        self.physics_update_stale: bool | None = None
        self.safety_state: str | None = None
        self.safety_permit: bool | None = None
        self.active_reasons: str | None = None
        self.last_fault_arrival_s: float | None = None
        self.last_status_arrival_s: float | None = None
        self.last_telemetry_sequence: int | None = None
        self.last_physics_update_sequence: int | None = None
        self.last_physics_revision_advance_s: float | None = None
        self.last_safety_arrival_s: float | None = None
        self.last_actuator_permit_arrival_s: float | None = None
        self.actuator_permit: bool | None = None

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
        self.drive = self.create_publisher(Twist, "/cmd_vel_gate", 10)
        self.brush = self.create_publisher(
            Float64MultiArray, "/safety/command/brush", 10
        )
        self.pump = self.create_publisher(
            Float64MultiArray, "/safety/command/pump", 10
        )
        latest_sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        latest_reliable_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_reliable_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            f"{MOTOR_ROOT}/fault_active",
            self._on_fault,
            latest_sensor_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Float64MultiArray,
            f"{MOTOR_ROOT}/telemetry_snapshot",
            self._on_motor_status,
            latest_sensor_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            "/formal_vehicle/auxiliary/critical_safety_status_json",
            self._on_critical_status,
            latest_reliable_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            "/safety/status_json",
            self._on_safety_status_json,
            latest_reliable_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Bool,
            "/safety/actuators_enabled",
            self._on_actuator_permit,
            latched_reliable_qos,
            callback_group=self._callback_group,
        )

    def _arrival(self, source: str, **values: object) -> float:
        monotonic_s = time.monotonic()
        with self._state_lock:
            self.events.append(
                {
                    "arrival_monotonic_s": monotonic_s,
                    "arrival_relative_s": monotonic_s
                    - self.started_monotonic_s,
                    "arrival_unix_ns": time.time_ns(),
                    "source": source,
                    **values,
                }
            )
        return monotonic_s

    def _on_fault(self, message: Bool) -> None:
        with self._state_lock:
            self.fault_active = bool(message.data)
            self.last_fault_arrival_s = self._arrival(
                "fault", fault_active=self.fault_active
            )

    def _on_motor_status(self, message: Float64MultiArray) -> None:
        payload = decode_cleaning_motor_telemetry(message.data)
        now_s = time.monotonic()
        with self._state_lock:
            telemetry_sequence = int(payload["telemetry_sequence"])
            sequence = int(payload["physics_update_sequence"])
            if (
                self.last_telemetry_sequence is not None
                and telemetry_sequence <= self.last_telemetry_sequence
            ):
                raise ValueError(
                    "cleaning motor telemetry_sequence is not strictly monotonic"
                )
            self.last_telemetry_sequence = telemetry_sequence
            self.physics_update_stale = bool(payload["physics_update_stale"])
            (
                self.last_physics_update_sequence,
                self.last_physics_revision_advance_s,
            ) = update_physics_revision_watchdog(
                sequence=sequence,
                physics_stale=self.physics_update_stale,
                last_sequence=self.last_physics_update_sequence,
                last_advance_s=self.last_physics_revision_advance_s,
                now_s=now_s,
            )
            self.last_status_arrival_s = self._arrival(
                "motor_status",
                fault_active=bool(payload["fault_active"]),
                physics_update_stale=self.physics_update_stale,
                command_fresh=bool(payload["command_fresh"]),
                telemetry_sequence=telemetry_sequence,
                physics_update_sequence=sequence,
            )

    def _on_critical_status(self, message: String) -> None:
        payload = json.loads(message.data)
        if payload.get("schema_version") != 1:
            raise ValueError("unsupported critical safety status schema")
        self._arrival(
            "critical_status",
            critical_publish_count=int(payload.get("publish_count", 0)),
            critical_maximum_gap_sec=float(
                payload.get("maximum_gap_sec", float("inf"))
            ),
            critical_thread_error=payload.get("thread_error"),
            relay_enabled=bool(payload.get("relay_enabled", False)),
            front_bumper_available=bool(
                payload.get("front_bumper_available", False)
            ),
            rear_bumper_available=bool(
                payload.get("rear_bumper_available", False)
            ),
        )

    def _on_safety_status_json(self, message: String) -> None:
        values = json.loads(message.data)
        if values.get("schema_version") != 1:
            raise ValueError("unsupported whole-vehicle safety status schema")
        with self._state_lock:
            self.safety_state = str(values.get("state", ""))
            self.safety_permit = bool(
                values.get("safety_inputs_permit_actuators", False)
            )
            self.active_reasons = values.get("active_reasons", "")
            self.last_safety_arrival_s = self._arrival(
                "safety",
                safety_state=self.safety_state,
                safety_permit=self.safety_permit,
                actuators_enabled=bool(values.get("actuators_enabled", False)),
                managed_controllers_active=bool(
                    values.get("managed_controllers_active", False)
                ),
                active_reasons=self.active_reasons,
                status_publish_count=int(values.get("status_publish_count", 0)),
                maximum_timer_gap_sec=float(
                    values.get("maximum_timer_gap_sec", float("inf"))
                ),
                publish_thread_error=values.get(
                    "publish_thread_error", "missing"
                ),
            )

    def _on_actuator_permit(self, message: Bool) -> None:
        with self._state_lock:
            self.actuator_permit = bool(message.data)
            self.last_actuator_permit_arrival_s = self._arrival(
                "actuator_permit", actuator_permit=self.actuator_permit
            )

    def publish_safe_stationary_command(self) -> None:
        self.main_power.publish(Bool(data=True))
        self.estop.publish(Bool(data=False))
        self.estop_reset.publish(Bool(data=True))
        self.heartbeat.publish(Empty())
        self.drive.publish(Twist())
        self.brush.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
        self.pump.publish(Float64MultiArray(data=[0.0]))

    def healthy_and_fresh(self, now_s: float) -> bool:
        with self._state_lock:
            arrivals = (
                self.last_fault_arrival_s,
                self.last_status_arrival_s,
                self.last_safety_arrival_s,
                self.last_actuator_permit_arrival_s,
            )
            return (
                all(
                    arrival is not None and now_s - arrival <= 0.15
                    for arrival in arrivals
                )
                and self.fault_active is False
                and self.physics_update_stale is False
                and self.safety_state == "ENABLED"
                and self.safety_permit is True
                and self.actuator_permit is True
                and self.active_reasons == ""
            )

    def event_snapshot(self) -> list[dict[str, object]]:
        with self._state_lock:
            return list(self.events)

    def state_snapshot(self) -> dict[str, object]:
        with self._state_lock:
            return {
                "fault_active": self.fault_active,
                "physics_update_stale": self.physics_update_stale,
                "safety_state": self.safety_state,
                "safety_permit": self.safety_permit,
                "active_reasons": self.active_reasons,
            }


def summarize_window(
    events: list[dict[str, object]], start_s: float, end_s: float
) -> tuple[dict[str, object], dict[str, bool]]:
    window = [
        event
        for event in events
        if start_s <= float(event["arrival_monotonic_s"]) <= end_s
    ]
    faults = [event for event in window if event["source"] == "fault"]
    motor_status = [
        event for event in window if event["source"] == "motor_status"
    ]
    safety = [event for event in window if event["source"] == "safety"]
    actuator_permits = [
        event for event in window if event["source"] == "actuator_permit"
    ]
    auxiliary = [
        event for event in window if event["source"] == "critical_status"
    ]
    source_metrics: dict[str, dict[str, float | int]] = {}
    for source in (
        "fault",
        "motor_status",
        "safety",
        "actuator_permit",
        "critical_status",
    ):
        source_times = [
            float(event["arrival_monotonic_s"])
            for event in window
            if event["source"] == source
        ]
        source_gaps = [
            later - earlier
            for earlier, later in zip(source_times, source_times[1:])
        ]
        source_metrics[source] = {
            "count": len(source_times),
            "rate_hz": (
                (len(source_times) - 1) / (source_times[-1] - source_times[0])
                if len(source_times) >= 2
                and source_times[-1] > source_times[0]
                else 0.0
            ),
            "maximum_gap_s": max(source_gaps, default=float("inf")),
        }
    fault_times = [float(event["arrival_monotonic_s"]) for event in faults]
    fault_gaps = [
        later - earlier for earlier, later in zip(fault_times, fault_times[1:])
    ]
    fault_rate_hz = (
        (len(fault_times) - 1) / (fault_times[-1] - fault_times[0])
        if len(fault_times) >= 2 and fault_times[-1] > fault_times[0]
        else 0.0
    )
    maximum_fault_gap_s = max(fault_gaps, default=float("inf"))
    safety_times = [float(event["arrival_monotonic_s"]) for event in safety]
    safety_gaps = [
        later - earlier for earlier, later in zip(safety_times, safety_times[1:])
    ]
    safety_rate_hz = (
        (len(safety_times) - 1) / (safety_times[-1] - safety_times[0])
        if len(safety_times) >= 2 and safety_times[-1] > safety_times[0]
        else 0.0
    )
    maximum_safety_gap_s = max(safety_gaps, default=float("inf"))
    unavailable_reasons = [
        reason
        for event in safety
        for reason in str(event["active_reasons"]).split(",")
        if reason.endswith("_unavailable")
    ]
    critical_rate_hz = 0.0
    if len(auxiliary) >= 2:
        critical_elapsed_s = (
            float(auxiliary[-1]["arrival_monotonic_s"])
            - float(auxiliary[0]["arrival_monotonic_s"])
        )
        if critical_elapsed_s > 0.0:
            critical_rate_hz = (
                int(auxiliary[-1]["critical_publish_count"])
                - int(auxiliary[0]["critical_publish_count"])
            ) / critical_elapsed_s
    metrics = {
        "window_duration_s": end_s - start_s,
        "event_count": len(window),
        "collector_source_metrics": source_metrics,
        "fault_sample_count": len(faults),
        "motor_status_sample_count": len(motor_status),
        "safety_sample_count": len(safety),
        "auxiliary_status_sample_count": len(auxiliary),
        "safety_status_rate_hz": safety_rate_hz,
        "maximum_safety_status_gap_s": maximum_safety_gap_s,
        "maximum_reported_timer_gap_s": max(
            (
                float(event["maximum_timer_gap_sec"])
                for event in safety
            ),
            default=float("inf"),
        ),
        "bumper_unavailable_count": sum(
            reason
            in {"front_bumper_unavailable", "rear_bumper_unavailable"}
            for reason in unavailable_reasons
        ),
        "relay_unavailable_count": unavailable_reasons.count(
            "safety_relay_unavailable"
        ),
        "publish_thread_errors": sorted(
            {str(event["publish_thread_error"]) for event in safety}
        ),
        "critical_producer_rate_hz": critical_rate_hz,
        "critical_producer_maximum_gap_s": max(
            (
                float(event["critical_maximum_gap_sec"])
                for event in auxiliary
            ),
            default=float("inf"),
        ),
        "critical_producer_thread_errors": sorted(
            {str(event["critical_thread_error"]) for event in auxiliary}
        ),
        "fault_rate_hz": fault_rate_hz,
        "maximum_fault_gap_s": maximum_fault_gap_s,
        "fault_true_count": sum(bool(event["fault_active"]) for event in faults),
        "physics_update_stale_true_count": sum(
            bool(event["physics_update_stale"]) for event in motor_status
        ),
    }
    checks = {
        "fault_topic_rate_at_least_18_hz": fault_rate_hz >= 18.0,
        "fault_topic_maximum_gap_at_most_0_075_s": maximum_fault_gap_s <= 0.075,
        "fault_zero_true_in_stable_window": bool(faults)
        and all(not bool(event["fault_active"]) for event in faults),
        "physics_update_zero_stale_in_stable_window": bool(motor_status)
        and all(not bool(event["physics_update_stale"]) for event in motor_status),
        "safety_status_rate_between_18_and_22_hz": 18.0
        <= safety_rate_hz
        <= 22.0,
        "safety_status_maximum_gap_below_0_075_s": maximum_safety_gap_s
        < 0.075,
        "collector_high_rate_sources_fair_and_fresh": all(
            source_metrics[source]["count"] > 0
            and float(source_metrics[source]["maximum_gap_s"]) < 0.075
            for source in (
                "fault",
                "motor_status",
                "safety",
                "actuator_permit",
            )
        ),
        "reported_timer_maximum_gap_below_0_075_s": bool(safety)
        and all(float(event["maximum_timer_gap_sec"]) < 0.075 for event in safety),
        "zero_bumper_or_relay_unavailable": not any(
            reason
            in {
                "front_bumper_unavailable",
                "rear_bumper_unavailable",
                "safety_relay_unavailable",
            }
            for reason in unavailable_reasons
        ),
        "publish_thread_error_null": bool(safety)
        and all(event["publish_thread_error"] == "none" for event in safety),
        "critical_producer_rate_between_18_and_22_hz": 18.0
        <= critical_rate_hz
        <= 22.0,
        "critical_producer_maximum_gap_below_0_075_s": bool(auxiliary)
        and all(
            float(event["critical_maximum_gap_sec"]) < 0.075
            for event in auxiliary
        ),
        "critical_producer_thread_error_null": bool(auxiliary)
        and all(event["critical_thread_error"] is None for event in auxiliary),
        "critical_snapshot_inputs_available": bool(auxiliary)
        and all(
            bool(event["relay_enabled"])
            and bool(event["front_bumper_available"])
            and bool(event["rear_bumper_available"])
            for event in auxiliary
        ),
        "safety_permit_enabled_continuous_window": bool(safety)
        and all(
            event["safety_state"] == "ENABLED"
            and bool(event["safety_permit"])
            and event["active_reasons"] == ""
            for event in safety
        ),
        # The status stream's declared permit cannot substitute for the
        # actual safety-manager output wired to the controller chain.
        "actuator_permission_enabled_continuous_window": bool(actuator_permits)
        and all(bool(event["actuator_permit"]) for event in actuator_permits),
    }
    return metrics, checks


def pump_executor(
    executor: MultiThreadedExecutor,
    stop_event: threading.Event,
    errors: list[BaseException],
) -> None:
    """Continuously submit ready callbacks until teardown requests a stop."""

    try:
        while not stop_event.is_set():
            executor.spin_once(timeout_sec=0.02)
    except BaseException as error:
        errors.append(error)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stable-duration-s", type=float, default=5.0)
    parser.add_argument("--timeout-s", type=float, default=180.0)
    parser.add_argument("--inject-estop-edge", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = SafetyPreflight()
    # A dedicated pump and four callback workers keep each depth-one evidence
    # stream serviceable even while the other 20 Hz streams are continuously
    # ready.  State mutation is serialized by the recorder lock.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_stop = threading.Event()
    executor_errors: list[BaseException] = []
    executor_thread = threading.Thread(
        target=pump_executor,
        args=(executor, executor_stop, executor_errors),
        name="formal_water_preflight_callbacks",
        daemon=True,
    )
    executor_thread.start()
    stable_start_s: float | None = None
    failure: str | None = None
    deadline_s = time.monotonic() + args.timeout_s
    next_command_s = time.monotonic()
    try:
        while time.monotonic() < deadline_s:
            if executor_errors or not executor_thread.is_alive():
                error = executor_errors[0] if executor_errors else None
                raise RuntimeError("preflight_callback_executor_failed") from error
            now_s = time.monotonic()
            if now_s >= next_command_s:
                node.publish_safe_stationary_command()
                next_command_s += COMMAND_PERIOD_S
                if next_command_s <= now_s:
                    next_command_s = now_s + COMMAND_PERIOD_S
            if node.healthy_and_fresh(now_s):
                if stable_start_s is None:
                    stable_start_s = now_s
                if now_s - stable_start_s >= args.stable_duration_s:
                    break
            else:
                stable_start_s = None
            time.sleep(POLL_PERIOD_S)
        else:
            failure = "safety telemetry did not remain continuously healthy"
    except Exception as exc:
        failure = f"{type(exc).__name__}: {exc}"

    stable_finished_s = time.monotonic()
    edge_metrics: dict[str, object] = {}
    edge_checks: dict[str, bool] = {}
    if failure is None and args.inject_estop_edge:
        injected_s = time.monotonic()
        node.estop.publish(Bool(data=True))
        edge_deadline_s = injected_s + 1.0
        stopped_arrival_s: float | None = None
        while time.monotonic() < edge_deadline_s:
            with node._state_lock:
                stopped = (
                    node.actuator_permit is False
                    and node.last_actuator_permit_arrival_s is not None
                    and node.last_actuator_permit_arrival_s >= injected_s
                )
                arrival_s = node.last_actuator_permit_arrival_s
            if stopped:
                stopped_arrival_s = arrival_s
                break
            time.sleep(POLL_PERIOD_S)
        stop_latency_s = (
            stopped_arrival_s - injected_s
            if stopped_arrival_s is not None
            else float("inf")
        )
        edge_metrics = {
            "estop_injected_monotonic_s": injected_s,
            "permit_false_arrival_monotonic_s": stopped_arrival_s,
            "danger_edge_stop_latency_s": stop_latency_s,
        }
        edge_checks = {
            "real_ros_danger_edge_stop_below_0_050_s": stop_latency_s < 0.05
        }
    if stable_start_s is None:
        metrics: dict[str, object] = {"window_duration_s": 0.0}
        checks: dict[str, bool] = {}
    else:
        events = node.event_snapshot()
        metrics, checks = summarize_window(
            events, stable_start_s, stable_finished_s
        )
        checks["stable_window_at_least_requested_duration"] = (
            stable_finished_s - stable_start_s >= args.stable_duration_s
        )
        metrics.update(edge_metrics)
        checks.update(edge_checks)
    passed = failure is None and bool(checks) and all(checks.values())
    result = {
        "schema_version": 1,
        "status": "FORMAL_WATER_SAFETY_PREFLIGHT_PASSED" if passed else "FAILED",
        "passed": passed,
        "requested_stable_duration_s": args.stable_duration_s,
        "checks": checks,
        "metrics": metrics,
        "failure": failure,
        "last_state": node.state_snapshot(),
        "events": node.event_snapshot(),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in result.items() if key != "events"}, indent=2))
    executor_stop.set()
    executor_thread.join(timeout=2.0)
    if executor_thread.is_alive():
        raise RuntimeError("preflight_callback_executor_join_timeout")
    if executor_errors:
        raise RuntimeError("preflight_callback_executor_failed") from executor_errors[0]
    executor.shutdown(timeout_sec=2.0)
    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
