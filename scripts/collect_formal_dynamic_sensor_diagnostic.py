#!/usr/bin/env python3
"""Collect non-formal sensor evidence from the dynamic UserCommands spawn path."""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path
from typing import Any

import rclpy

from collect_formal_vehicle_sensor_runtime import (
    ACTIVE_CONTROLLERS,
    INACTIVE_CONTROLLERS,
    Probe,
)
from formal_vehicle_sensor_runtime_contract import validate_runtime_contract


def collect(output: Path, timeout_s: float) -> dict[str, Any]:
    rclpy.init()
    node = Probe()
    states: dict[str, str] = {}
    controller_plane_ready = False
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.10)
            states = node.controller_states()
            controllers_ready = all(
                states.get(name) == "active" for name in ACTIVE_CONTROLLERS
            ) and all(
                states.get(name) == "inactive" for name in INACTIVE_CONTROLLERS
            )
            if controllers_ready and node.robot_description:
                controller_plane_ready = True
                node.start_topic_subscriptions()
                break
        next_progress = time.monotonic()
        while (
            controller_plane_ready
            and time.monotonic() < deadline
            and node.pending_topics()
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
            retired = node.retire_ready_subscriptions()
            now = time.monotonic()
            if retired or now >= next_progress:
                print(
                    "dynamic_sensor_diagnostic_progress "
                    f"retired={len(retired)} pending={len(node.pending_topics())} "
                    f"pending_topics={','.join(node.pending_topics())}",
                    flush=True,
                )
                next_progress = now + 5.0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    observed_hz = node.observed_hz()
    runtime_contract = validate_runtime_contract(
        node.samples, node.metadata, observed_hz
    )
    passed_checks = {
        "diagnostic_is_explicitly_not_formal_evidence": True,
        "controller_plane_ready_before_sensor_subscriptions": controller_plane_ready,
        "required_controllers_safe": all(
            states.get(name) == "active" for name in ACTIVE_CONTROLLERS
        ) and all(
            states.get(name) == "inactive" for name in INACTIVE_CONTROLLERS
        ),
        "all_sensor_subscriptions_retired_after_bounded_evidence": (
            not node.pending_topics()
        ),
        "runtime_sensor_contract_passed": runtime_contract["passed"],
    }
    passed = all(passed_checks.values())
    report = {
        "report_id": "tzcup_dynamic_spawn_sensor_diagnostic_v1",
        "status": (
            "DYNAMIC_SPAWN_SENSOR_DIAGNOSTIC_PASSED"
            if passed
            else "DYNAMIC_SPAWN_SENSOR_DIAGNOSTIC_BLOCKED"
        ),
        "passed": passed,
        "formal_eligible": False,
        "spawn_mode": "dynamic_usercommands",
        "claim_boundary": (
            "Diagnostic-only A/B evidence. It may isolate dynamic spawn transport "
            "from the preembedded control crash but cannot satisfy the formal "
            "session, snapshot, FOV, frozen-runtime, or preembedded-world bindings."
        ),
        "passed_checks": passed_checks,
        "controller_states": states,
        "sample_counts": node.samples,
        "observed_source_timestamp_hz": observed_hz,
        "observed_interfaces": node.metadata,
        "pending_topics": list(node.pending_topics()),
        "runtime_sensor_contract": runtime_contract,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + f".pending.{os.getpid()}")
    temporary.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(output)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()
    report = collect(args.output, args.timeout)
    print(json.dumps(report, sort_keys=True))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
