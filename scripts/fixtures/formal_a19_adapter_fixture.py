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


def fault_readbacks(command: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    fault, parameters = command["fault"], command["parameters"]
    schema = contract["fault_readback_contract"][fault]
    kind, outcome = schema["injection_readback_kind"], schema["expected_outcome"]
    if kind == "sensor_drop":
        suffix = "rgb" if fault == "rgb_freeze" else "depth"
        injection = {"fault": fault, "observed": True, "dropped_channels": [f"front_{suffix}"], "counts": {f"front_{suffix}": {"input": 1, "output": 0, "dropped": 1, "mutated": 0}}}
        recovery = {"fault": fault, "observed": True, "forwarded_channels": [f"front_{suffix}"], "counts": {f"front_{suffix}": {"input": 2, "output": 1, "dropped": 1, "mutated": 0}}}
    elif kind == "sensor_mutation":
        mutation = {"fault": fault, "channel": "front_rgb"}
        mutation.update({
            "timestamp_skew": {"action": "timestamp_shifted", "before": [1, 0], "after": [1, 250000000]},
            "camera_info_mismatch": {"action": "camera_info_width_changed", "before": 640, "after": 648},
            "tf_unavailable": {"action": "frame_rewritten_to_unavailable", "before": "camera", "after": "formal_a19_missing_camera"},
            "invalid_depth": {"action": "depth_payload_invalidated", "after_nonzero_bytes": 0, "size_bytes": 8},
        }[fault])
        injection = {"fault": fault, "observed": True, "mutation": mutation, "parameters": parameters}
        recovery = {"fault": fault, "observed": True, "forwarded_channels": ["front_info" if fault == "camera_info_mismatch" else ("front_depth" if fault == "invalid_depth" else "front_rgb")], "counts": {"front": {"input": 2, "output": 2, "dropped": 0, "mutated": 1}}}
    elif kind == "product_consumer":
        effect = {"fault": fault, "observed": True, "expected_outcome": outcome}
        if fault == "proposal_flood": effect.update(input_proposal_count=1, output_proposal_count=parameters["proposals_per_frame"])
        if fault == "proposal_dropout": effect.update(input_proposal_count=1, output_proposal_count=0)
        if fault in {"action_verifier_failure", "reobserve_timeout"}: effect["target_id"] = "fixture-target"
        if fault == "action_verifier_failure": effect.update(original_verified=True, final_verified=False)
        source = "post_clean_verification" if fault == "action_verifier_failure" else "perception"
        def product(active: str) -> dict[str, Any]:
            values = {"formal_a19_fault": active, "formal_a19_fault_events": 1, "formal_a19_fault_effect": effect}
            return {"fault": fault, "observed": True, "source": source, "active_fault": active, "events": 1, "effect": effect, "rows": [{"values": values}]}
        injection, recovery = product(fault), product("")
    elif kind == "model_provider":
        values = {"fault": fault, "active": True, "observed": True, "inference_path_triggered": True}
        if fault == "cuda_provider_failure": values.update(requested_provider="CUDAExecutionProvider", available_providers=["CUDAExecutionProvider"], selected_provider="CUDAExecutionProvider", session_providers=["CUDAExecutionProvider"])
        if fault == "model_hash_mismatch": values.update(actual_sha256="a" * 64, expected_sha256="b" * 64)
        if fault == "corrupt_model": values.update(shadow_only=True, loader_error="invalid protobuf", original_sha256="a" * 64, shadow_sha256="b" * 64)
        if fault == "sustained_slow_inference": values.update(observed_delay_ms=parameters["latency_ms"], requested_latency_ms=parameters["latency_ms"])
        injection = {"message": "a19_fault_active", "level": 2, "values": values}
        recovery = {"message": "a19_fault_recovered", "level": 0, "values": {"fault": fault, "cleared": True, "trigger_was_observed": True, "original_model_hashes_restored": {"dosod": True, "edgesam": True}}}
    elif kind == "nav2_lifecycle":
        injection = {"fault": fault, "observed": True, "expected_outcome": outcome, "planner_state_before": 3, "planner_state_after": 2, "compute_path_server_ready": False}
        recovery = {"fault": fault, "observed": True, "planner_state_before": 2, "planner_state_after": 3, "compute_path_server_ready": True}
    else:
        pose = {"x": 0.5, "y": 0.0, "z": 0.0}
        common = {"fault": fault, "observed": True, "model": "walker", "world": "world", "pose_source_type": "native gz.msgs.Pose_V", "scan_source_topic": "/scan/navigation", "set_pose_service": "/world/world/set_pose"}
        injection = {**common, "expected_outcome": outcome, "set_pose_success_count": 1, "nearest_navigation_scan_m": 0.5, "native_pose": pose, "target_pose": pose}
        recovery = {**common, "restore_set_pose_success_count": 1, "post_restore_navigation_scan_m": 1.0, "native_position_error_m": 0.0, "native_pose": pose, "original_pose": pose}
    return injection, recovery


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
        configured = contract["profile_expectations"][profile]
        command_speed, torque = [2.0, 3.0, 2.0, 3.0], [4.0, 5.0, 4.0, 5.0]
        physical = {"source": "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/status", "wheel_slip_ratio": configured["wheel_slip_ratio"], "actuator_gain": configured["actuator_gain"], "commanded_wheel_speed_rad_s": command_speed, "effective_wheel_speed_rad_s": [value / (1.0 - configured["wheel_slip_ratio"]) for value in command_speed], "unscaled_wheel_torque_nm": torque, "applied_wheel_torque_nm": [value * configured["actuator_gain"] for value in torque], "measured_wheel_speed_rad_s": [1.0, 1.5, 1.0, 1.5]}
        emit({"type": "profile_activated", "command_id": command["command_id"], "profile": profile, "configured_values": configured, "readback": {"profile": profile, "sensor_latency_ms": configured["sensor_latency_ms"], "sensor_dropout_probability": configured["sensor_dropout_probability"], "product_pid": 1, "product_pgid": 1, "physical_readback": physical}})
    elif command_type == "inject_fault":
        expectation = contract["fault_expectations"][command["fault"]]
        injection_readback, recovery_readback = fault_readbacks(command)
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
            "injection_readback": injection_readback,
        })
        time.sleep(0.001)
        emit({
            "type": "fault_state", "fault": command["fault"], "state": "RECOVERED",
            "safety_state": "RUNNING", "coverage_state": "RESUMED",
            "recovery_readback": recovery_readback,
        })
    elif command_type == "shutdown":
        running = False
        emit({"type": "complete", "command_id": command["command_id"], "reason": command["reason"]})
        break
    else:
        raise SystemExit(2)

raise SystemExit(0)
