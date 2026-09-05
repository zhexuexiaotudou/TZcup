#!/usr/bin/env python3
"""Collect and finalize the strict typed cleaning-motor transport diagnostic."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import time
from pathlib import Path
from typing import Any


TYPED_TOPIC = (
    "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/telemetry_snapshot"
)
STATUS_TOPIC = "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/status_json"
EXPECTED_CHECKS = {
    "all_snapshots_parse_as_63_finite_values",
    "first_frame_below_0_5_s",
    "maximum_gap_at_most_75_ms",
    "no_burst_gap_below_20_ms",
    "physics_revision_advances",
    "physics_revision_never_moves_backwards",
    "physics_revision_stagnation_below_0_75_s",
    "rate_18_to_22_hz",
    "raw_trace_contains_every_received_frame",
    "ros_status_json_has_zero_publishers",
    "steady_samples_not_physics_stale",
    "telemetry_sequence_strictly_increasing",
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _read_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def _regular(path: Path, label: str) -> Path:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{label} must be a regular non-symlink file: {path}")
    return path.resolve()


def _write_fresh_json(path: Path, payload: dict[str, Any]) -> None:
    if path.exists() or path.is_symlink():
        raise ValueError(f"refusing stale output: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_suffix(path.suffix + f".pending.{os.getpid()}")
    if pending.exists() or pending.is_symlink():
        raise ValueError(f"refusing stale pending output: {pending}")
    pending.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    pending.replace(path)


def collect(output: Path, trace: Path, duration_s: float) -> int:
    if not 5.0 <= duration_s <= 60.0:
        raise ValueError("duration-s must be between 5 and 60 seconds")
    if output.resolve() == trace.resolve():
        raise ValueError("output and trace must be distinct")
    for path in (output, trace):
        if path.exists() or path.is_symlink():
            raise ValueError(f"refusing stale output: {path}")

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray

    from formal_cleaning_motor_telemetry import (
        VALUE_COUNT,
        decode_cleaning_motor_telemetry,
    )

    rclpy.init()
    node = Node("formal_typed_cleaning_motor_diagnostic")
    started = time.monotonic()
    samples: list[dict[str, Any]] = []
    raw_frames: list[dict[str, Any]] = []
    errors: list[str] = []

    def callback(message: Float64MultiArray) -> None:
        arrival = time.monotonic()
        data = [float(value) for value in message.data]
        frame: dict[str, Any] = {
            "arrival_since_start_s": arrival - started,
            "data": data,
        }
        raw_frames.append(frame)
        try:
            decoded = decode_cleaning_motor_telemetry(data)
            samples.append(
                {
                    "arrival_s": arrival,
                    "value_count": len(data),
                    "telemetry_sequence": decoded["telemetry_sequence"],
                    "physics_update_sequence": decoded["physics_update_sequence"],
                    "physics_update_stale": decoded["physics_update_stale"],
                    "fault_active": decoded["fault_active"],
                }
            )
        except Exception as error:  # retained verbatim in the evidence report
            text = f"{type(error).__name__}: {error}"
            frame["decode_error"] = text
            errors.append(text)

    node.create_subscription(Float64MultiArray, TYPED_TOPIC, callback, 100)
    deadline = started + duration_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)

    trace.parent.mkdir(parents=True, exist_ok=True)
    trace_pending = trace.with_suffix(trace.suffix + f".pending.{os.getpid()}")
    trace_pending.write_text(
        "".join(
            json.dumps(frame, separators=(",", ":")) + "\n" for frame in raw_frames
        ),
        encoding="utf-8",
    )
    trace_pending.replace(trace)

    arrivals = [float(sample["arrival_s"]) for sample in samples]
    gaps = [right - left for left, right in zip(arrivals, arrivals[1:])]
    telemetry_sequences = [int(sample["telemetry_sequence"]) for sample in samples]
    physics_sequences = [int(sample["physics_update_sequence"]) for sample in samples]
    physics_advance_count = sum(
        right > left for left, right in zip(physics_sequences, physics_sequences[1:])
    )
    maximum_stagnant = 0.0
    stagnant_start: float | None = None
    for index in range(1, len(samples)):
        if physics_sequences[index] == physics_sequences[index - 1]:
            if stagnant_start is None:
                stagnant_start = arrivals[index - 1]
            maximum_stagnant = max(
                maximum_stagnant, arrivals[index] - stagnant_start
            )
        else:
            stagnant_start = None

    status_publishers = node.get_publishers_info_by_topic(STATUS_TOPIC)
    checks = {
        "first_frame_below_0_5_s": bool(samples)
        and arrivals[0] - started < 0.5,
        "rate_18_to_22_hz": len(gaps) > 20
        and 18.0 <= len(gaps) / sum(gaps) <= 22.0,
        "maximum_gap_at_most_75_ms": bool(gaps) and max(gaps) <= 0.075,
        "no_burst_gap_below_20_ms": bool(gaps) and min(gaps) >= 0.020,
        "all_snapshots_parse_as_63_finite_values": bool(samples)
        and not errors
        and len(raw_frames) == len(samples)
        and all(sample["value_count"] == VALUE_COUNT for sample in samples),
        "raw_trace_contains_every_received_frame": len(raw_frames) == len(samples),
        "telemetry_sequence_strictly_increasing": len(samples) > 1
        and all(
            right > left
            for left, right in zip(telemetry_sequences, telemetry_sequences[1:])
        ),
        "physics_revision_never_moves_backwards": len(samples) > 1
        and all(
            right >= left
            for left, right in zip(physics_sequences, physics_sequences[1:])
        ),
        "physics_revision_advances": physics_advance_count > 0,
        "physics_revision_stagnation_below_0_75_s": maximum_stagnant < 0.75,
        "steady_samples_not_physics_stale": bool(samples)
        and all(sample["physics_update_stale"] is False for sample in samples),
        "ros_status_json_has_zero_publishers": len(status_publishers) == 0,
    }
    report = {
        "schema_version": 1,
        "status": (
            "FORMAL_TYPED_CLEANING_MOTOR_DIAG_PASSED"
            if all(checks.values())
            else "FAILED"
        ),
        "passed": all(checks.values()),
        "checks": checks,
        "metrics": {
            "sample_count": len(samples),
            "raw_trace_frame_count": len(raw_frames),
            "first_frame_s": arrivals[0] - started if samples else math.inf,
            "observed_hz": len(gaps) / sum(gaps) if gaps else 0.0,
            "minimum_gap_s": min(gaps) if gaps else math.inf,
            "maximum_gap_s": max(gaps) if gaps else math.inf,
            "physics_advance_count": physics_advance_count,
            "maximum_physics_stagnation_s": maximum_stagnant,
            "first_telemetry_sequence": telemetry_sequences[0] if samples else None,
            "last_telemetry_sequence": telemetry_sequences[-1] if samples else None,
            "first_physics_sequence": physics_sequences[0] if samples else None,
            "last_physics_sequence": physics_sequences[-1] if samples else None,
            "ros_status_json_publisher_count": len(status_publishers),
            "decode_errors": errors,
        },
    }
    _write_fresh_json(output, report)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report["passed"] else 1


def finalize(
    collector_report: Path,
    launch_log: Path,
    launch_audit: Path,
    gz_topic_info: Path,
    ros_topic_info: Path,
    output: Path,
) -> int:
    paths = [
        _regular(collector_report, "collector report"),
        _regular(launch_log, "launch log"),
        _regular(launch_audit, "launch audit"),
        _regular(gz_topic_info, "Gazebo topic info"),
        _regular(ros_topic_info, "ROS topic info"),
    ]
    if len(set(paths)) != len(paths):
        raise ValueError("typed diagnostic inputs must be distinct")
    report = _read_object(collector_report)
    audit = _read_object(launch_audit)
    launch = launch_log.read_text(encoding="utf-8", errors="replace")
    gz_info = gz_topic_info.read_text(encoding="utf-8", errors="replace")
    ros_info = ros_topic_info.read_text(encoding="utf-8", errors="replace")
    tagged = re.findall(r"gz_publish_failed topic=(\S+) count=(\d+)", launch)
    node_shared = re.findall(
        r"^.*NodeShared::Publish\(\) Error:.*$", launch, re.MULTILINE
    )
    checks = report.get("checks")
    transport_checks = {
        "launch_log_audit_passed": audit.get("passed") is True,
        "zero_nodeshared_publish_errors": not node_shared,
        "zero_topic_tagged_publish_failures": not tagged,
        "gazebo_topic_type_is_double_v": "gz.msgs.Double_V" in gz_info,
        "ros_topic_type_is_float64_multi_array": (
            "std_msgs/msg/Float64MultiArray" in ros_info
        ),
        "ros_typed_topic_has_one_publisher": "Publisher count: 1" in ros_info,
    }
    passed = (
        report.get("status") == "FORMAL_TYPED_CLEANING_MOTOR_DIAG_PASSED"
        and report.get("passed") is True
        and isinstance(checks, dict)
        and set(checks) == EXPECTED_CHECKS
        and all(checks.get(name) is True for name in EXPECTED_CHECKS)
        and all(transport_checks.values())
    )
    final_report = dict(report)
    final_report["status"] = (
        "FORMAL_TYPED_CLEANING_MOTOR_DIAG_PASSED" if passed else "FAILED"
    )
    final_report["passed"] = passed
    final_report["transport_audit"] = {
        "passed": all(transport_checks.values()),
        "checks": transport_checks,
        "node_shared_publish_errors": node_shared,
        "topic_tagged_publish_failures": [
            {"topic": topic, "count": int(count)} for topic, count in tagged
        ],
        "launch_log": str(launch_log.resolve()),
        "launch_log_sha256": _sha256(launch_log),
        "launch_audit_json": str(launch_audit.resolve()),
        "launch_audit_sha256": _sha256(launch_audit),
        "gazebo_topic_info": str(gz_topic_info.resolve()),
        "gazebo_topic_info_sha256": _sha256(gz_topic_info),
        "ros_topic_info": str(ros_topic_info.resolve()),
        "ros_topic_info_sha256": _sha256(ros_topic_info),
    }
    _write_fresh_json(output, final_report)
    return 0 if passed else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--collect", action="store_true")
    mode.add_argument("--finalize", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trace", type=Path)
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--collector-report", type=Path)
    parser.add_argument("--launch-log", type=Path)
    parser.add_argument("--launch-audit", type=Path)
    parser.add_argument("--gz-topic-info", type=Path)
    parser.add_argument("--ros-topic-info", type=Path)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.collect:
        if args.trace is None:
            raise SystemExit("--collect requires --trace")
        return collect(args.output, args.trace, args.duration_s)
    required = {
        "collector_report": args.collector_report,
        "launch_log": args.launch_log,
        "launch_audit": args.launch_audit,
        "gz_topic_info": args.gz_topic_info,
        "ros_topic_info": args.ros_topic_info,
    }
    missing = [f"--{name.replace('_', '-')}" for name, value in required.items() if value is None]
    if missing:
        raise SystemExit("--finalize requires " + ", ".join(missing))
    return finalize(output=args.output, **required)  # type: ignore[arg-type]


if __name__ == "__main__":
    raise SystemExit(main())
