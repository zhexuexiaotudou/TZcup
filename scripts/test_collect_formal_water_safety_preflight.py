"""Deterministic checks for the formal water safety soak summary."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray, String

from collect_formal_water_safety_preflight import MOTOR_ROOT, SafetyPreflight
from collect_formal_water_safety_preflight import pump_executor
from collect_formal_water_safety_preflight import summarize_window


ROOT = Path(__file__).resolve().parents[1]


def _events(
    *,
    bad_reason: str = "",
    thread_error: str = "none",
    actuator_permit: bool = True,
    reported_timer_gap: float = 0.05,
    critical_timer_gap: float = 0.05,
):
    events = []
    for index in range(1301):
        arrival = index * 0.05
        events.extend(
            [
                {
                    "arrival_monotonic_s": arrival,
                    "source": "fault",
                    "fault_active": False,
                },
                {
                    "arrival_monotonic_s": arrival,
                    "source": "motor_status",
                    "physics_update_stale": False,
                },
                {
                    "arrival_monotonic_s": arrival,
                    "source": "safety",
                    "safety_state": "ENABLED",
                    "safety_permit": True,
                    "active_reasons": bad_reason,
                    "maximum_timer_gap_sec": reported_timer_gap,
                    "publish_thread_error": thread_error,
                },
                {
                    "arrival_monotonic_s": arrival,
                    "source": "actuator_permit",
                    "actuator_permit": actuator_permit,
                },
                {
                    "arrival_monotonic_s": arrival,
                    "source": "critical_status",
                    "critical_publish_count": index,
                    "critical_maximum_gap_sec": critical_timer_gap,
                    "critical_thread_error": None,
                    "relay_enabled": True,
                    "front_bumper_available": True,
                    "rear_bumper_available": True,
                },
            ]
        )
    return events


def test_sixty_five_second_twenty_hz_soak_passes_strict_metrics():
    metrics, checks = summarize_window(_events(), 0.0, 65.0)

    assert all(checks.values())
    assert metrics["safety_status_rate_hz"] == 20.0
    assert metrics["maximum_safety_status_gap_s"] < 0.075
    assert metrics["bumper_unavailable_count"] == 0
    assert metrics["relay_unavailable_count"] == 0
    assert metrics["publish_thread_errors"] == ["none"]
    assert metrics["critical_producer_rate_hz"] == 20.0
    assert metrics["critical_producer_thread_errors"] == ["None"]
    assert all(
        metrics["collector_source_metrics"][source]["count"] == 1301
        for source in ("fault", "motor_status", "safety", "actuator_permit")
    )
    assert checks["collector_high_rate_sources_fair_and_fresh"]


def test_unavailable_or_thread_error_fails_closed():
    _, unavailable = summarize_window(
        _events(bad_reason="front_bumper_unavailable"), 0.0, 65.0
    )
    _, failed_thread = summarize_window(
        _events(thread_error="RuntimeError"), 0.0, 65.0
    )

    assert unavailable["zero_bumper_or_relay_unavailable"] is False
    assert unavailable["safety_permit_enabled_continuous_window"] is False
    assert failed_thread["publish_thread_error_null"] is False


def test_scoped_continuity_ignores_pre_window_cumulative_timer_spikes():
    _, checks = summarize_window(
        _events(reported_timer_gap=0.120152, critical_timer_gap=0.101178),
        0.0,
        65.0,
    )

    assert all(checks.values())


def test_critical_safety_arrival_gap_fails_even_when_cumulative_metric_is_low():
    events = _events()
    for event in events:
        if event["source"] == "critical_status" and event["arrival_monotonic_s"] == 5.0:
            event["arrival_monotonic_s"] = 5.03
            break

    _, checks = summarize_window(events, 0.0, 65.0)

    assert checks["collector_high_rate_sources_fair_and_fresh"] is False


def test_critical_safety_stream_must_cover_the_complete_window():
    events = [
        event
        for event in _events()
        if not (
            event["source"] == "critical_status"
            and event["arrival_monotonic_s"] > 64.9
        )
    ]

    _, checks = summarize_window(events, 0.0, 65.0)

    assert checks["collector_high_rate_sources_fair_and_fresh"] is False


def test_declared_safety_permit_cannot_substitute_for_disabled_actuators():
    _, checks = summarize_window(_events(actuator_permit=False), 0.0, 65.0)

    assert checks["safety_permit_enabled_continuous_window"] is True
    assert checks["actuator_permission_enabled_continuous_window"] is False


def test_missing_or_starved_high_rate_source_fails_fairness_gate():
    events = [
        event for event in _events() if event["source"] != "safety"
    ]

    metrics, checks = summarize_window(events, 0.0, 65.0)

    assert metrics["collector_source_metrics"]["safety"]["count"] == 0
    assert checks["collector_high_rate_sources_fair_and_fresh"] is False


def test_callback_executor_does_not_starve_safety_under_interleaved_load():
    rclpy.init()
    collector = SafetyPreflight()
    producer = Node("formal_water_preflight_stress_producer")
    fault = producer.create_publisher(Bool, f"{MOTOR_ROOT}/fault_active", 10)
    motor = producer.create_publisher(
        Float64MultiArray, f"{MOTOR_ROOT}/telemetry_snapshot", 100
    )
    latched = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    permit = producer.create_publisher(
        Bool, "/safety/actuators_enabled", latched
    )
    safety = producer.create_publisher(String, "/safety/status_json", 1)
    critical = producer.create_publisher(
        String,
        "/formal_vehicle/auxiliary/critical_safety_status_json",
        1,
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(collector)
    executor.add_node(producer)
    executor_stop = threading.Event()
    executor_errors: list[BaseException] = []
    spin_thread = threading.Thread(
        target=pump_executor,
        args=(executor, executor_stop, executor_errors),
        daemon=True,
    )
    spin_thread.start()
    try:
        status = String(
            data=json.dumps(
                {
                    "schema_version": 1,
                    "state": "ENABLED",
                    "safety_inputs_permit_actuators": True,
                    "actuators_enabled": True,
                    "managed_controllers_active": True,
                    "active_reasons": "",
                    "status_publish_count": 1,
                    "maximum_timer_gap_sec": 0.05,
                    "publish_thread_error": "none",
                }
            )
        )
        critical_status = String(
            data=json.dumps(
                {
                    "schema_version": 1,
                    "publish_count": 1,
                    "maximum_gap_sec": 0.05,
                    "thread_error": None,
                    "relay_enabled": True,
                    "front_bumper_available": True,
                    "rear_bumper_available": True,
                }
            )
        )
        # Five streams at 40 Hz produce 200 callbacks/s, four times
        # times the old one-spin-per-20ms collector could service.  Keep the
        # pressure on for 32.6 seconds so this regression crosses the original
        # approximately 32.5-second starvation cycle instead of validating
        # only a short scheduling burst.
        for index in range(1304):
            values = [0.0] * 63
            values[:8] = [
                1.0,
                float(index + 1),
                float(index + 1),
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
            ]
            for motor_index in range(5):
                base = 8 + motor_index * 11
                values[base + 7] = 1.0
                values[base + 10] = 1.0 if motor_index == 3 else 0.0
            fault.publish(Bool(data=False))
            motor.publish(Float64MultiArray(data=values))
            permit.publish(Bool(data=True))
            safety.publish(status)
            critical.publish(critical_status)
            time.sleep(0.025)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            counts = {
                source: sum(
                    event["source"] == source
                    for event in collector.event_snapshot()
                )
                for source in (
                    "fault",
                    "motor_status",
                    "safety",
                    "actuator_permit",
                    "critical_status",
                )
            }
            if all(count >= 1000 for count in counts.values()):
                break
            time.sleep(0.01)
        # Discovery may legitimately consume the first few seconds, but no
        # source may lag the others once the graph is matched.
        assert all(count >= 1000 for count in counts.values()), counts
        assert (
            max(counts.values()) - min(counts.values())
        ) / max(counts.values()) < 0.01, counts
        snapshot = collector.event_snapshot()
        for source in (
            "fault",
            "motor_status",
            "safety",
            "actuator_permit",
            "critical_status",
        ):
            arrivals = [
                float(event["arrival_monotonic_s"])
                for event in snapshot
                if event["source"] == source
            ]
            # This is a deliberately saturated 160-callback/s scheduling
            # stress, so its purpose is to exclude multi-second starvation;
            # summarize_window separately enforces the real 75 ms acceptance
            # gate at the product's 20 Hz contract.
            assert max(
                later - earlier
                for earlier, later in zip(arrivals, arrivals[1:])
            ) < 0.5, source
    finally:
        executor_stop.set()
        spin_thread.join(timeout=2.0)
        assert not spin_thread.is_alive()
        assert executor_errors == []
        executor.shutdown(timeout_sec=2.0)
        executor.remove_node(collector)
        executor.remove_node(producer)
        collector.destroy_node()
        producer.destroy_node()
        rclpy.shutdown()


def test_safety_publish_loops_catch_up_immediately_after_an_overrun():
    for relative in (
        "starter_ws/src/sanitation_safety/sanitation_safety/"
        "simulation_safety_inputs.py",
        "starter_ws/src/sanitation_safety/sanitation_safety/"
        "whole_vehicle_safety_manager.py",
    ):
        source = (ROOT / relative).read_text(encoding="utf-8")
        assert "deadline = now + period" not in source
        assert "if deadline <= now:\n                    deadline = now" in source
