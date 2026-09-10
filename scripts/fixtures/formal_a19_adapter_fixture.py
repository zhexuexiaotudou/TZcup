#!/usr/bin/env python3
"""Test-only A19 line-protocol adapter; formal producer paths reject this file."""

from __future__ import annotations

import json
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any


PROTOCOL = "tzcup.formal_a19.adapter.v1"
nonce = os.environ["TZCUP_FORMAL_A19_NONCE"]
contract = json.loads(Path(os.environ["TZCUP_FORMAL_A19_CONTRACT"]).read_text(encoding="utf-8"))
sequence = 0
profile = "nominal"
running = False
lock = threading.Lock()


def emit(payload: dict[str, Any]) -> None:
    global sequence
    with lock:
        payload.update(protocol=PROTOCOL, nonce=nonce, adapter_sequence=sequence)
        sequence += 1
        sys.stdout.write(json.dumps(payload, separators=(",", ":")) + "\n")
        sys.stdout.flush()


def sample_loop() -> None:
    period = float(contract["stability_gates"]["sample_period_s"])
    while running:
        emit({
            "type": "sample",
            "profile": profile,
            "components": {name: "OPERATIONAL" for name in contract["required_pipeline_components"]},
            "metrics": {
                "crash_count": 0,
                "deadlock_count": 0,
                "queue_growth_count": 0,
                "unexpected_model_reload_count": 0,
                "persistent_tf_failure_count": 0,
                "unrecoverable_watchdog_event_count": 0,
                "unsafe_cleaning_action_count": 0,
                "localization_xy_error_m": 0.01,
                "memory_rss_bytes": 100000000,
            },
        })
        time.sleep(period)


for line in sys.stdin:
    command = json.loads(line)
    if command.get("protocol") != PROTOCOL or command.get("nonce") != nonce:
        raise SystemExit(2)
    command_type = command.get("type")
    if command_type == "start":
        running = True
        emit({
            "type": "hello",
            "command_id": command["command_id"],
            "components": contract["required_pipeline_components"],
            "profiles": [row["profile"] for row in contract["profile_schedule"]],
            "faults": [row["fault"] for row in contract["fault_schedule"]],
            "product_pid": 1, "product_pgid": 1,
            "operator_control": {"initial_arm_commands": 1, "post_start_commands": 0},
        })
        threading.Thread(target=sample_loop, daemon=True).start()
    elif command_type == "set_profile":
        profile = command["profile"]
        configured = {"sensor_latency_ms": 0.0, "sensor_dropout_probability": 0.0, "wheel_slip_ratio": 0.0, "actuator_gain": 1.0}
        emit({"type": "profile_activated", "command_id": command["command_id"], "profile": profile, "configured_values": configured, "readback": {"product_pid": 1, "product_pgid": 1, "physical_readback": {"wheel_slip_ratio": {"configured": configured["wheel_slip_ratio"], "observed": True}, "actuator_gain": {"configured": configured["actuator_gain"], "observed": True}}}})
    elif command_type == "inject_fault":
        expectation = contract["fault_expectations"][command["fault"]]
        emit({
            "type": "fault_injected", "command_id": command["command_id"],
            "fault": command["fault"], "profile": command["profile"],
            "parameters": command["parameters"],
        })
        emit({
            "type": "fault_state", "fault": command["fault"], "state": expectation["state"],
            "safety_state": "STOPPED" if expectation["requires_global_safety_stop"] else "RUNNING", "pending_clean_outcome": "CANCELLED" if expectation["requires_cleaning_inhibit"] else None,
            "perception_health": "DEGRADED" if expectation["requires_perception_degraded"] else "OPERATIONAL", "nav2_operational": True,
            "watchdog_operational": True, "unsafe_cleaning_action_count": 0,
            "brake_latency_s": 0.01 if expectation["requires_global_safety_stop"] else None,
            "injection_readback": {"fixture": True},
        })
        time.sleep(0.001)
        emit({
            "type": "fault_state", "fault": command["fault"], "state": "RECOVERED",
            "safety_state": "RUNNING", "coverage_state": "RESUMED",
            "recovery_readback": {"fixture": True},
        })
    elif command_type == "shutdown":
        running = False
        emit({"type": "complete", "command_id": command["command_id"], "reason": command["reason"]})
        break
    else:
        raise SystemExit(2)

raise SystemExit(0)
