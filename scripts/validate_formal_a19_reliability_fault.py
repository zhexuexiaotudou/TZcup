#!/usr/bin/env python3
"""Independently validate canonical A19 raw evidence and publish one receipt."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import stat
import tempfile
from pathlib import Path
from statistics import median
from typing import Any, Mapping, Sequence

from formal_runtime_gate_binding import build_binding
from produce_formal_a19_reliability_fault import (
    DEFAULT_CONTRACT,
    PROTOCOL,
    RAW_RECEIPT,
    ROOT,
    _adapter_file_identities,
    _git_identity,
    validate_contract,
)


PASSED = "FORMAL_A19_TWO_HOUR_RELIABILITY_FAULT_PASSED"
BLOCKED = "FORMAL_A19_TWO_HOUR_RELIABILITY_FAULT_BLOCKED"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _identity(path: Path) -> tuple[int, int, int, int, int]:
    row = path.lstat()
    return row.st_dev, row.st_ino, row.st_size, row.st_mtime_ns, row.st_mode


def _normalized(path: Path) -> Path:
    if ".." in Path(str(path)).parts:
        raise ValueError(f"parent traversal is forbidden: {path}")
    return Path(os.path.normpath(os.path.abspath(str(path))))


def _no_symlink_ancestor(path: Path) -> None:
    current = path
    while True:
        if stat.S_ISLNK(current.lstat().st_mode):
            raise ValueError(f"symlink is forbidden: {current}")
        if current.parent == current:
            return
        current = current.parent


def _safe_root(path: Path, parent: Path) -> Path:
    parent, path = _normalized(parent), _normalized(path)
    if not parent.is_dir() or not path.is_dir():
        raise ValueError("evidence parent/root must be existing directories")
    try:
        path.relative_to(parent)
    except ValueError as exc:
        raise ValueError(f"evidence root escapes authorized parent: {path}") from exc
    _no_symlink_ancestor(path)
    path.resolve(strict=True).relative_to(parent.resolve(strict=True))
    return path


def _safe_file(path: Path, root: Path) -> Path:
    root, path = _normalized(root), _normalized(path)
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"path escapes root: {path}") from exc
    _no_symlink_ancestor(path)
    path.resolve(strict=True).relative_to(root.resolve(strict=True))
    if not stat.S_ISREG(path.lstat().st_mode):
        raise ValueError(f"not a regular file: {path}")
    return path


def _stable_bytes(path: Path, root: Path) -> bytes:
    path = _safe_file(path, root)
    before = _identity(path)
    data = path.read_bytes()
    if before != _identity(path):
        raise ValueError(f"file changed while read: {path}")
    return data


def _stable_json(path: Path, root: Path) -> tuple[dict[str, Any], str]:
    data = _stable_bytes(path, root)
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"JSON root is not an object: {path}")
    return value, hashlib.sha256(data).hexdigest()


def _strict_number(value: Any, label: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(float(value)):
        raise ValueError(f"{label} must be a finite number")
    return float(value)


def _strict_int(value: Any, label: str) -> int:
    if type(value) is not int:
        raise ValueError(f"{label} must be an integer")
    return value


def _percentile(values: Sequence[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[max(0, math.ceil(len(ordered) * fraction) - 1)]


def _raw_reference(receipt: Mapping[str, Any], name: str, evidence_root: Path) -> tuple[Path, bytes]:
    raw = receipt.get("raw_evidence")
    row = raw.get(name) if isinstance(raw, Mapping) else None
    if not isinstance(row, Mapping) or set(row) - {"path", "sha256", "size_bytes", "line_count"}:
        raise ValueError(f"raw evidence reference is invalid: {name}")
    relative = row.get("path")
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute() or ".." in Path(relative).parts:
        raise ValueError(f"raw evidence path is invalid: {name}")
    path = _safe_file(evidence_root / relative, evidence_root)
    data = _stable_bytes(path, evidence_root)
    if hashlib.sha256(data).hexdigest() != row.get("sha256") or len(data) != row.get("size_bytes"):
        raise ValueError(f"raw evidence digest/size drifted: {name}")
    if "line_count" in row and data.count(b"\n") != row["line_count"]:
        raise ValueError(f"raw evidence line count drifted: {name}")
    return path, data


def _parse_events(data: bytes) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for index, line in enumerate(data.splitlines()):
        try:
            row = json.loads(line.decode("utf-8"))
        except (UnicodeError, json.JSONDecodeError) as exc:
            raise ValueError(f"adapter event line {index} is invalid JSON") from exc
        if not isinstance(row, dict) or "parse_error" in row or not isinstance(row.get("adapter"), dict):
            raise ValueError(f"adapter event line {index} is malformed")
        if row.get("producer_sequence") != index:
            raise ValueError("producer event sequence is not contiguous")
        _strict_int(row.get("received_epoch_ns"), "received_epoch_ns")
        _strict_int(row.get("received_monotonic_ns"), "received_monotonic_ns")
        events.append(row)
    if not events:
        raise ValueError("adapter event stream is empty")
    monotonic = [row["received_monotonic_ns"] for row in events]
    if any(after <= before for before, after in zip(monotonic, monotonic[1:])):
        raise ValueError("producer event monotonic timestamps are not strictly increasing")
    return events


def _command_map(receipt: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    rows = receipt.get("issued_commands")
    if not isinstance(rows, list) or not rows:
        raise ValueError("raw receipt has no producer-issued commands")
    result: dict[str, dict[str, Any]] = {}
    previous = -1
    for row in rows:
        if not isinstance(row, dict) or not isinstance(row.get("command"), dict):
            raise ValueError("producer-issued command row is invalid")
        _strict_int(row.get("sent_epoch_ns"), "command sent_epoch_ns")
        sent = _strict_int(row.get("sent_monotonic_ns"), "command sent_monotonic_ns")
        if sent <= previous:
            raise ValueError("producer-issued command timestamps are not increasing")
        previous = sent
        command = row["command"]
        command_id = command.get("command_id")
        if not isinstance(command_id, str) or not command_id or command_id in result:
            raise ValueError("producer-issued command id is missing or duplicated")
        result[command_id] = row
    return result


def _json_value(value: Any) -> Any:
    if not isinstance(value, str):
        return value
    try:
        return json.loads(value)
    except json.JSONDecodeError:
        return value


def _product_telemetry_matches(readback: Mapping[str, Any]) -> bool:
    for row in readback.get("rows", []):
        values = row.get("values") if isinstance(row, Mapping) else None
        if not isinstance(values, Mapping):
            continue
        if (
            _json_value(values.get("formal_a19_fault")) == readback.get("active_fault")
            and _json_value(values.get("formal_a19_fault_events")) == readback.get("events")
            and _json_value(values.get("formal_a19_fault_effect")) == readback.get("effect")
        ):
            return True
    return False


def _pose(readback: Mapping[str, Any], name: str) -> tuple[float, float, float] | None:
    value = readback.get(name)
    if not isinstance(value, Mapping):
        return None
    try:
        return tuple(_strict_number(value.get(axis), f"{name}.{axis}") for axis in ("x", "y", "z"))
    except ValueError:
        return None


def _validate_fault_readbacks(
    fault: str, injection: Any, recovery: Any, scheduled: Mapping[str, Any],
    readback_contract: Mapping[str, Any],
) -> list[str]:
    errors: list[str] = []
    if not isinstance(injection, Mapping) or not injection:
        return ["fault has no non-empty independently observed injection readback"]
    if not isinstance(recovery, Mapping) or not recovery:
        return ["fault recovery has no non-empty independently observed readback"]
    kind = readback_contract.get("injection_readback_kind")
    recovery_kind = readback_contract.get("recovery_readback_kind")
    outcome = readback_contract.get("expected_outcome")
    parameters = scheduled["parameters"]

    if kind in {"sensor_drop", "sensor_mutation", "product_consumer", "nav2_lifecycle", "dynamic_obstacle"}:
        if injection.get("fault") != fault or injection.get("observed") is not True:
            errors.append("injection readback fault identity/observed flag is invalid")
    if recovery_kind != "model_provider_cleared" and (
        recovery.get("fault") != fault or recovery.get("observed") is not True
    ):
        errors.append("recovery readback fault identity/observed flag is invalid")

    if kind == "sensor_drop":
        suffix = "rgb" if fault == "rgb_freeze" else "depth"
        channels, counts = injection.get("dropped_channels"), injection.get("counts")
        if not isinstance(channels, list) or not channels or any(not isinstance(x, str) or not x.endswith(suffix) for x in channels):
            errors.append("sensor drop readback has no matching dropped channel")
        if not isinstance(counts, Mapping):
            errors.append("sensor drop readback has no channel counters")
    elif kind == "sensor_mutation":
        mutation = injection.get("mutation")
        actions = {
            "timestamp_skew": "timestamp_shifted", "camera_info_mismatch": "camera_info_width_changed",
            "tf_unavailable": "frame_rewritten_to_unavailable", "invalid_depth": "depth_payload_invalidated",
        }
        if not isinstance(mutation, Mapping) or mutation.get("fault") != fault or mutation.get("action") != actions.get(fault) or injection.get("parameters") != parameters:
            errors.append("sensor mutation readback does not match the scheduled fault")
        elif fault == "timestamp_skew" and mutation.get("before") == mutation.get("after"):
            errors.append("timestamp mutation did not change the stamp")
        elif fault == "camera_info_mismatch" and not (
            type(mutation.get("before")) is int and type(mutation.get("after")) is int and mutation["after"] > mutation["before"]
        ):
            errors.append("CameraInfo mutation did not increase width")
        elif fault == "tf_unavailable" and not str(mutation.get("after", "")).startswith("formal_a19_missing_"):
            errors.append("TF mutation did not bind an unavailable frame")
        elif fault == "invalid_depth" and not (
            mutation.get("after_nonzero_bytes") == 0 and type(mutation.get("size_bytes")) is int and mutation["size_bytes"] > 0
        ):
            errors.append("depth mutation did not invalidate a non-empty payload")
    elif kind == "product_consumer":
        expected_source = "post_clean_verification" if fault == "action_verifier_failure" else "perception"
        effect = injection.get("effect")
        if (
            injection.get("source") != expected_source or injection.get("active_fault") != fault
            or type(injection.get("events")) is not int or injection["events"] <= 0
            or not isinstance(effect, Mapping) or effect.get("fault") != fault
            or effect.get("observed") is not True or effect.get("expected_outcome") != outcome
            or not _product_telemetry_matches(injection)
        ):
            errors.append("product consumer readback is not bound to the real fault effect telemetry")
        elif fault == "proposal_flood" and not (
            type(effect.get("input_proposal_count")) is int and effect["input_proposal_count"] > 0
            and effect.get("output_proposal_count") == parameters["proposals_per_frame"]
            and effect["output_proposal_count"] > effect["input_proposal_count"]
        ):
            errors.append("proposal flood did not prove real output expansion")
        elif fault == "proposal_dropout" and not (
            type(effect.get("input_proposal_count")) is int and effect["input_proposal_count"] > 0
            and effect.get("output_proposal_count") == 0
        ):
            errors.append("proposal dropout did not prove real output removal")
        elif fault in {"action_verifier_failure", "reobserve_timeout"} and not isinstance(effect.get("target_id"), str):
            errors.append("product consumer fault has no real target identity")
        if (
            recovery.get("source") != expected_source or recovery.get("active_fault") != ""
            or type(recovery.get("events")) is not int or recovery["events"] < injection.get("events", 0)
            or recovery.get("effect") != effect or not _product_telemetry_matches(recovery)
        ):
            errors.append("product consumer recovery is not bound to cleared live telemetry")
    elif kind == "model_provider":
        values = injection.get("values")
        decoded = {key: _json_value(value) for key, value in values.items()} if isinstance(values, Mapping) else {}
        if (
            injection.get("message") != "a19_fault_active" or injection.get("level") != 2
            or decoded.get("fault") != fault or decoded.get("observed") is not True
            or decoded.get("active") is not True or decoded.get("inference_path_triggered") is not True
        ):
            errors.append("model/provider fault was not triggered in the real inference path")
        elif fault == "cuda_provider_failure" and not (
            decoded.get("requested_provider") == "CUDAExecutionProvider"
            and decoded.get("selected_provider") == "CUDAExecutionProvider"
            and "CUDAExecutionProvider" in decoded.get("available_providers", [])
            and "CUDAExecutionProvider" in decoded.get("session_providers", [])
        ):
            errors.append("CUDA provider failure lacks an actually selected CUDA provider")
        elif fault == "model_hash_mismatch" and not (
            isinstance(decoded.get("actual_sha256"), str) and len(decoded["actual_sha256"]) == 64
            and isinstance(decoded.get("expected_sha256"), str) and len(decoded["expected_sha256"]) == 64
            and decoded["actual_sha256"] != decoded["expected_sha256"]
        ):
            errors.append("DOSOD model hash mismatch was not independently demonstrated")
        elif fault == "corrupt_model" and not (
            decoded.get("shadow_only") is True and isinstance(decoded.get("loader_error"), str) and decoded["loader_error"]
            and decoded.get("original_sha256") != decoded.get("shadow_sha256")
        ):
            errors.append("EdgeSAM corrupt shadow was not rejected by the real loader")
        elif fault == "sustained_slow_inference":
            try:
                if _strict_number(decoded.get("observed_delay_ms"), "observed delay") < _strict_number(parameters.get("latency_ms"), "requested delay"):
                    errors.append("slow inference did not reach the scheduled delay")
            except ValueError as exc:
                errors.append(str(exc))
        recovered_values = recovery.get("values")
        recovered = {key: _json_value(value) for key, value in recovered_values.items()} if isinstance(recovered_values, Mapping) else {}
        if (
            recovery.get("message") != "a19_fault_recovered" or recovery.get("level") != 0
            or recovered.get("fault") != fault or recovered.get("cleared") is not True
            or recovered.get("trigger_was_observed") is not True
            or recovered.get("original_model_hashes_restored") != {"dosod": True, "edgesam": True}
        ):
            errors.append("model/provider recovery lacks trigger-bound model restoration")
    elif kind == "nav2_lifecycle":
        if not (injection.get("expected_outcome") == outcome and injection.get("planner_state_before") == 3 and injection.get("planner_state_after") == 2 and injection.get("compute_path_server_ready") is False):
            errors.append("Nav2 injection lacks active-to-inactive action-server readback")
        if not (recovery.get("planner_state_before") == 2 and recovery.get("planner_state_after") == 3 and recovery.get("compute_path_server_ready") is True):
            errors.append("Nav2 recovery lacks inactive-to-active action-server readback")
    elif kind == "dynamic_obstacle":
        native, target = _pose(injection, "native_pose"), _pose(injection, "target_pose")
        try:
            scan = _strict_number(injection.get("nearest_navigation_scan_m"), "navigation scan")
        except ValueError:
            scan = math.inf
        if (
            injection.get("expected_outcome") != outcome or injection.get("pose_source_type") != "native gz.msgs.Pose_V"
            or injection.get("scan_source_topic") != "/scan/navigation"
            or type(injection.get("set_pose_success_count")) is not int or injection["set_pose_success_count"] <= 0
            or native is None or target is None or math.dist(native, target) > 0.05
            or scan > float(parameters["minimum_block_distance_m"]) + 0.2
        ):
            errors.append("dynamic obstacle injection lacks SetEntityPose, native Pose_V and scan agreement")
        restored, original = _pose(recovery, "native_pose"), _pose(recovery, "original_pose")
        try:
            scan = _strict_number(recovery.get("post_restore_navigation_scan_m"), "post-restore scan")
            position_error = _strict_number(recovery.get("native_position_error_m"), "restore position error")
        except ValueError:
            scan, position_error = math.inf, math.inf
        if (
            recovery.get("pose_source_type") != "native gz.msgs.Pose_V" or recovery.get("scan_source_topic") != "/scan/navigation"
            or type(recovery.get("restore_set_pose_success_count")) is not int or recovery["restore_set_pose_success_count"] <= 0
            or restored is None or original is None or math.dist(restored, original) > 0.05
            or position_error > 0.05 or not math.isfinite(scan)
            or injection.get("model") != recovery.get("model") or injection.get("world") != recovery.get("world")
        ):
            errors.append("dynamic obstacle recovery lacks real return pose and fresh scan agreement")
    else:
        errors.append("fault readback kind is unsupported")

    if recovery_kind == "sensor_forwarding":
        channels = recovery.get("forwarded_channels")
        if not isinstance(channels, list) or not channels or not isinstance(recovery.get("counts"), Mapping):
            errors.append("sensor recovery did not prove resumed forwarding")
    return errors


def _validate_profile_readback(
    acknowledgement: Mapping[str, Any], command: Mapping[str, Any],
    hello: Mapping[str, Any], expected: Mapping[str, Any],
) -> list[str]:
    profile = acknowledgement.get("profile")
    errors: list[str] = []
    if acknowledgement.get("command_id") != command.get("command_id") or profile != command.get("profile"):
        return ["profile activation does not acknowledge the matching producer command"]
    configured, readback = acknowledgement.get("configured_values"), acknowledgement.get("readback")
    if not isinstance(configured, Mapping) or not isinstance(readback, Mapping):
        return ["profile activation has no full configured/readback record"]
    if configured != expected:
        errors.append(f"profile {profile} configured values differ from the frozen contract")
    if readback.get("profile") != profile or any(readback.get(field) != configured.get(field) for field in ("sensor_latency_ms", "sensor_dropout_probability")):
        errors.append(f"profile {profile} sensor readback drifted")
    if readback.get("product_pid") != hello.get("product_pid") or readback.get("product_pgid") != hello.get("product_pgid"):
        errors.append("profile activation restarted or replaced the product process")
    physical = readback.get("physical_readback")
    required_physical = {
        "source", "wheel_slip_ratio", "actuator_gain", "commanded_wheel_speed_rad_s",
        "effective_wheel_speed_rad_s", "unscaled_wheel_torque_nm",
        "applied_wheel_torque_nm", "measured_wheel_speed_rad_s",
    }
    if not isinstance(physical, Mapping) or set(physical) != required_physical:
        return errors + ["profile activation lacks complete physical readback"]
    if (
        physical.get("source") != "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/status"
        or physical.get("wheel_slip_ratio") != configured.get("wheel_slip_ratio")
        or physical.get("actuator_gain") != configured.get("actuator_gain")
    ):
        errors.append(f"profile {profile} physical fields drifted")
    arrays = [physical.get(name) for name in (
        "commanded_wheel_speed_rad_s", "effective_wheel_speed_rad_s",
        "unscaled_wheel_torque_nm", "applied_wheel_torque_nm", "measured_wheel_speed_rad_s",
    )]
    if any(not isinstance(values, list) or len(values) != 4 or any(type(value) not in (int, float) or not math.isfinite(float(value)) for value in values) for values in arrays):
        return errors + [f"profile {profile} lacks four-wheel finite plant readback"]
    command, effective, unscaled, applied, _ = arrays
    slip, gain = float(configured["wheel_slip_ratio"]), float(configured["actuator_gain"])
    if any(not math.isclose(float(actual), float(requested) / (1.0 - slip), abs_tol=1e-7) for requested, actual in zip(command, effective)):
        errors.append(f"profile {profile} wheel-slip plant readback drifted")
    if any(not math.isclose(float(actual), float(requested) * gain, abs_tol=1e-7) for requested, actual in zip(unscaled, applied)):
        errors.append(f"profile {profile} actuator-gain plant readback drifted")
    if profile != "nominal" and (not any(abs(float(value)) > 1e-6 for value in command) or not any(abs(float(value)) > 1e-6 for value in unscaled)):
        errors.append(f"non-nominal profile {profile} lacks live physical actuation")
    return errors


def _validate_semantics(receipt: Mapping[str, Any], events: Sequence[dict[str, Any]], contract: Mapping[str, Any]) -> tuple[list[str], dict[str, Any]]:
    errors: list[str] = []
    metrics: dict[str, Any] = {}
    producer = receipt.get("producer")
    nonce = producer.get("nonce") if isinstance(producer, Mapping) else None
    adapters = [row["adapter"] for row in events]
    for index, event in enumerate(adapters):
        if event.get("protocol") != PROTOCOL or event.get("nonce") != nonce or event.get("adapter_sequence") != index:
            errors.append(f"adapter protocol/nonce/sequence mismatch at event {index}")
            break
    hello = [event for event in adapters if event.get("type") == "hello"]
    expected_faults = [row["fault"] for row in contract["fault_schedule"]]
    expected_profiles = [row["profile"] for row in contract["profile_schedule"]]
    if len(hello) != 1 or hello[0].get("components") != contract["required_pipeline_components"] or hello[0].get("faults") != expected_faults or hello[0].get("profiles") != expected_profiles:
        errors.append("adapter hello does not bind the exact component/profile/fault inventory")
    elif (not isinstance(hello[0].get("product_pid"), int) or not isinstance(hello[0].get("product_pgid"), int)
          or hello[0].get("operator_control") != {"initial_arm_commands": 1, "post_start_commands": 0}):
        errors.append("adapter did not prove one product identity and startup-only operator arm")
    try:
        commands = _command_map(receipt)
    except ValueError as exc:
        errors.append(str(exc))
        commands = {}
    typed_commands: dict[str, list[dict[str, Any]]] = {}
    for row in commands.values():
        command = row["command"]
        typed_commands.setdefault(str(command.get("type")), []).append(row)
        if command.get("protocol") != PROTOCOL or command.get("nonce") != nonce:
            errors.append("producer-issued command has wrong protocol or nonce")
    expected_command_counts = {"start": 1, "set_profile": 4, "inject_fault": 18, "shutdown": 1}
    if {name: len(typed_commands.get(name, [])) for name in expected_command_counts} != expected_command_counts or set(typed_commands) != set(expected_command_counts):
        errors.append("producer-issued command inventory is incomplete or has extras")
    if hello and typed_commands.get("start") and hello[0].get("command_id") != typed_commands["start"][0]["command"].get("command_id"):
        errors.append("adapter hello does not acknowledge the producer start command")
    execution = receipt.get("execution")
    if not isinstance(execution, Mapping):
        errors.append("execution receipt is missing")
        execution = {}
    try:
        started = _strict_int(execution.get("started_monotonic_ns"), "execution start")
        ended = _strict_int(execution.get("ended_monotonic_ns"), "execution end")
        duration = _strict_number(execution.get("monotonic_duration_s"), "execution duration")
        recomputed = (ended - started) / 1e9
        if abs(duration - recomputed) > 1e-6:
            errors.append("execution monotonic duration does not recompute")
    except ValueError as exc:
        errors.append(str(exc)); started = ended = 0; duration = 0.0
    minimum_duration = float(contract["stability_gates"]["minimum_duration_s"])
    if duration < minimum_duration:
        errors.append(f"real monotonic duration is below {minimum_duration:g} seconds")
    if execution.get("exit_code") != 0 or execution.get("zero_survivors") is not True or execution.get("survivor_pids") != [] or execution.get("signals_sent") != [] or execution.get("shell") is not False:
        errors.append("launch command did not finish with zero exit, zero survivors and no forced cleanup")
    if receipt.get("capture_complete") is not True or receipt.get("producer_errors") != []:
        errors.append("canonical producer did not complete cleanly")
    sample_rows = [row for row in events if row["adapter"].get("type") == "sample"]
    samples = [row["adapter"] for row in sample_rows]
    gates = contract["stability_gates"]
    if len(samples) < int(gates["minimum_sample_count"]):
        errors.append("raw time series has too few samples")
    sample_times = [row["received_monotonic_ns"] for row in sample_rows]
    max_gap = 0.0
    if sample_times:
        max_gap = max(((b - a) / 1e9 for a, b in zip(sample_times, sample_times[1:])), default=0.0)
        span = (sample_times[-1] - sample_times[0]) / 1e9
        if span + float(gates["maximum_sample_gap_s"]) < minimum_duration:
            errors.append("raw time series does not span the required real duration")
        if max_gap > float(gates["maximum_sample_gap_s"]):
            errors.append("raw time series cadence has a gap above the contract")
    else:
        span = 0.0
    required_components = set(contract["required_pipeline_components"])
    localization_errors: list[float] = []
    memory: list[tuple[int, float]] = []
    counters = {
        "crash_count", "deadlock_count", "queue_growth_count",
        "unexpected_model_reload_count", "persistent_tf_failure_count",
        "unrecoverable_watchdog_event_count", "unsafe_cleaning_action_count",
    }
    profile_times: dict[str, list[int]] = {name: [] for name in expected_profiles}
    for index, (envelope, sample) in enumerate(zip(sample_rows, samples)):
        components = sample.get("components")
        sample_metrics = sample.get("metrics")
        profile = sample.get("profile")
        if not isinstance(components, dict) or set(components) != required_components:
            errors.append(f"sample {index} has incomplete product components"); continue
        if any(value not in {"OPERATIONAL", "DEGRADED", "STOPPED"} for value in components.values()):
            errors.append(f"sample {index} has invalid component health")
        if profile not in profile_times:
            errors.append(f"sample {index} has invalid profile")
        else:
            profile_times[profile].append(envelope["received_monotonic_ns"])
        if not isinstance(sample_metrics, dict):
            errors.append(f"sample {index} has no metrics"); continue
        for counter in counters:
            if sample_metrics.get(counter) != gates[counter]:
                errors.append(f"sample {index} violates {counter}")
        try:
            localization_errors.append(_strict_number(sample_metrics.get("localization_xy_error_m"), "localization error"))
            memory.append((envelope["received_monotonic_ns"], _strict_number(sample_metrics.get("memory_rss_bytes"), "memory_rss_bytes")))
        except ValueError as exc:
            errors.append(f"sample {index}: {exc}")
    profile_durations: dict[str, float] = {}
    for row in contract["profile_schedule"]:
        values = profile_times[row["profile"]]
        observed = (max(values) - min(values)) / 1e9 if values else 0.0
        profile_durations[row["profile"]] = observed
        if observed < float(row["minimum_observed_duration_s"]):
            errors.append(f"profile {row['profile']} has insufficient observed duration")
    if localization_errors:
        rmse = math.sqrt(sum(value * value for value in localization_errors) / len(localization_errors))
        p95 = _percentile(localization_errors, 0.95)
        if rmse > float(gates["maximum_localization_xy_rmse_m"]): errors.append("localization RMSE exceeds A19 gate")
        if p95 > float(gates["maximum_localization_xy_p95_m"]): errors.append("localization P95 exceeds A19 gate")
    else:
        rmse = p95 = math.inf
    memory_growth = math.inf
    if memory:
        window_ns = int(float(gates["memory_comparison_window_s"]) * 1e9)
        first = [value for when, value in memory if when <= memory[0][0] + window_ns]
        last = [value for when, value in memory if when >= memory[-1][0] - window_ns]
        if first and last and median(first) > 0:
            memory_growth = (median(last) - median(first)) / median(first)
            if memory_growth > float(gates["maximum_memory_growth_ratio"]):
                errors.append("memory growth exceeds A19 gate")
    fault_results: list[dict[str, Any]] = []
    timing = contract["fault_timing"]
    evidence = contract["required_fault_evidence"]
    expectations = contract.get("fault_expectations")
    if not isinstance(expectations, Mapping):
        errors.append("fault-specific expectations are missing")
        expectations = {}
    for scheduled in contract["fault_schedule"]:
        fault = scheduled["fault"]
        expectation = expectations.get(fault)
        command_rows = [row for row in commands.values() if row["command"].get("type") == "inject_fault" and row["command"].get("fault") == fault]
        injected = [event for event in adapters if event.get("type") == "fault_injected" and event.get("fault") == fault]
        states = [(envelope, envelope["adapter"]) for envelope in events if envelope["adapter"].get("type") == "fault_state" and envelope["adapter"].get("fault") == fault]
        fault_errors: list[str] = []
        if not isinstance(expectation, Mapping):
            fault_errors.append("fault-specific expectation is missing")
            expectation = {}
        if len(command_rows) != 1 or len(injected) != 1:
            fault_errors.append("requires exactly one producer command and injection acknowledgement")
        else:
            command_row = command_rows[0]; command = command_row["command"]
            acknowledgement = injected[0]
            if acknowledgement.get("command_id") != command.get("command_id") or acknowledgement.get("parameters") != scheduled["parameters"] or command.get("parameters") != scheduled["parameters"] or command.get("profile") != scheduled["profile"]:
                fault_errors.append("command/ack parameters or profile drifted")
            planned = started + int(float(scheduled["offset_s"]) * 1e9)
            lateness = (command_row["sent_monotonic_ns"] - planned) / 1e9
            if lateness < -0.25 or lateness > float(timing["maximum_injection_lateness_s"]):
                fault_errors.append("injection command missed its producer-owned schedule")
        stopped = [(row, event) for row, event in states if event.get("state") == expectation.get("state")]
        recovered = [(row, event) for row, event in states if event.get("state") == "RECOVERED"]
        if len(stopped) != 1 or len(recovered) != 1:
            fault_errors.append("requires exactly one STOPPED then one RECOVERED event")
        elif stopped[0][0]["received_monotonic_ns"] >= recovered[0][0]["received_monotonic_ns"]:
            fault_errors.append("RECOVERED does not follow STOPPED")
        else:
            stop_event, recover_event = stopped[0][1], recovered[0][1]
            injected_envelope = next((row for row in events if row["adapter"] is injected[0]), None) if injected else None
            if injected_envelope is not None:
                stop_latency = (stopped[0][0]["received_monotonic_ns"] - injected_envelope["received_monotonic_ns"]) / 1e9
                recovery_latency = (recovered[0][0]["received_monotonic_ns"] - stopped[0][0]["received_monotonic_ns"]) / 1e9
                if stop_latency < 0 or stop_latency > float(timing["maximum_safe_stop_latency_s"]): fault_errors.append("safe STOPPED latency exceeds contract")
                if recovery_latency < 0 or recovery_latency > float(timing["maximum_recovery_latency_s"]): fault_errors.append("RECOVERED latency exceeds contract")
            fault_errors.extend(_validate_fault_readbacks(
                fault, stop_event.get("injection_readback"), recover_event.get("recovery_readback"),
                scheduled, contract["fault_readback_contract"][fault],
            ))
            if expectation.get("requires_global_safety_stop") is True:
                if stop_event.get("safety_state") != evidence["required_safety_state"]:
                    fault_errors.append("fault did not force safety STOPPED")
                if expectation.get("requires_cleaning_inhibit") is True and stop_event.get("pending_clean_outcome") not in evidence["unsafe_pending_clean_outcomes"]:
                    fault_errors.append("unsafe pending clean was not cancelled/deferred")
            elif stop_event.get("safety_state") != "RUNNING":
                fault_errors.append("non-stopping fault did not retain independently observed running safety")
            if expectation.get("requires_perception_degraded") is True and stop_event.get("perception_health") not in evidence["perception_health_states"]:
                fault_errors.append("perception health did not degrade/error")
            if stop_event.get("nav2_operational") is not True or stop_event.get("watchdog_operational") is not True:
                fault_errors.append("Safety/Nav2/Watchdog did not remain operational")
            if stop_event.get("unsafe_cleaning_action_count") != 0:
                fault_errors.append("unsafe cleaning action occurred during fault")
            if expectation.get("requires_global_safety_stop") is True:
                try:
                    brake = _strict_number(stop_event.get("brake_latency_s"), "brake latency")
                    if brake > float(gates["maximum_estop_brake_latency_s"]): fault_errors.append("brake latency exceeds A19 gate")
                except ValueError as exc:
                    fault_errors.append(str(exc))
            if recover_event.get("coverage_state") not in evidence["required_recovered_coverage_states"] or recover_event.get("safety_state") != "RUNNING":
                fault_errors.append("safe Coverage recovery was not observed")
        if fault_errors:
            errors.extend(f"{fault}: {message}" for message in fault_errors)
        fault_results.append({"fault": fault, "passed": not fault_errors, "errors": fault_errors})
    profile_acks = [event for event in adapters if event.get("type") == "profile_activated"]
    if [event.get("profile") for event in profile_acks] != expected_profiles:
        errors.append("profile activation acknowledgements are incomplete or reordered")
    elif typed_commands.get("set_profile"):
        profile_commands = typed_commands["set_profile"]
        for acknowledgement, command_row in zip(profile_acks, profile_commands):
            command = command_row["command"]
            errors.extend(_validate_profile_readback(
                acknowledgement, command, hello[0] if hello else {},
                contract["profile_expectations"].get(acknowledgement.get("profile"), {}),
            ))
    if any(event.get("type") == "profile_unsupported" for event in adapters):
        errors.append("A19 run encountered an unsupported physical profile")
    complete = [event for event in adapters if event.get("type") == "complete"]
    if len(complete) != 1 or complete[0].get("reason") != "formal_duration_complete":
        errors.append("adapter did not emit one formal-duration completion event")
    elif typed_commands.get("shutdown") and complete[0].get("command_id") != typed_commands["shutdown"][0]["command"].get("command_id"):
        errors.append("adapter completion does not acknowledge the producer shutdown command")
    metrics.update(
        duration_s=duration, sample_count=len(samples), sample_span_s=span,
        maximum_sample_gap_s=max_gap, memory_growth_ratio=memory_growth,
        localization_xy_rmse_m=rmse, localization_xy_p95_m=p95,
        profile_observed_duration_s=profile_durations,
        fault_count=len(fault_results), passed_fault_count=sum(row["passed"] for row in fault_results),
    )
    return sorted(set(errors)), {"metrics": metrics, "faults": fault_results}


def validate(
    report_path: Path, snapshot_path: Path, session_path: Path, closure_path: Path,
    evidence_root: Path, contract_path: Path = DEFAULT_CONTRACT,
    repository_root: Path = ROOT, evidence_parent: Path | None = None,
) -> dict[str, Any]:
    errors: list[str] = []
    repository_root = repository_root.resolve()
    evidence_parent = evidence_parent or repository_root / "artifacts"
    try:
        evidence_root = _safe_root(evidence_root, evidence_parent)
        root_identity = _identity(evidence_root)
        receipt, receipt_sha = _stable_json(report_path, evidence_root)
        contract, contract_sha = _stable_json(contract_path, repository_root)
        validate_contract(contract)
        if receipt.get("receipt_id") != RAW_RECEIPT:
            errors.append("raw receipt identity is invalid")
        if receipt.get("status") != "FORMAL_A19_RAW_CAPTURE_COMPLETE":
            errors.append("raw receipt is not a complete canonical capture")
        contract_ref = receipt.get("contract")
        if not isinstance(contract_ref, Mapping) or contract_ref.get("sha256") != contract_sha or contract_ref.get("contract_id") != contract.get("contract_id"):
            errors.append("raw receipt is not bound to the current A19 contract")
        producer_ref = receipt.get("producer")
        producer_path = repository_root / "scripts/produce_formal_a19_reliability_fault.py"
        if not isinstance(producer_ref, Mapping) or producer_ref.get("path") != str(producer_path.resolve()) or producer_ref.get("sha256") != _sha256(producer_path) or producer_ref.get("protocol") != PROTOCOL:
            errors.append("raw receipt is not bound to the current canonical producer")
        if receipt.get("source_binding") != _git_identity(repository_root):
            errors.append("raw receipt source commit/tree is not current")
        binding_path, binding_data = _raw_reference(receipt, "runtime_gate_binding", evidence_root)
        binding = json.loads(binding_data.decode("utf-8"))
        if binding != receipt.get("runtime_gate_binding"):
            errors.append("runtime binding sidecar differs from raw receipt")
        current_binding = build_binding(
            repository_root=repository_root, install_root=Path(binding["runtime_closure_binding"]["runtime_install_root"]),
            closure_manifest=closure_path, session_path=session_path, snapshot_path=snapshot_path,
        )
        if current_binding["acceptance_session_binding"] != binding.get("acceptance_session_binding") or current_binding["runtime_closure_binding"] != binding.get("runtime_closure_binding"):
            errors.append("current session/snapshot/runtime closure differs from raw receipt")
        execution = receipt.get("execution")
        if not isinstance(execution, Mapping) or not isinstance(execution.get("argv"), list):
            errors.append("raw receipt has no executable argv identity")
        else:
            current_adapter_files = _adapter_file_identities(
                execution["argv"], repository_root,
                Path(binding["runtime_closure_binding"]["runtime_install_root"]),
            )
            if execution.get("argv_file_identities") != current_adapter_files:
                errors.append("adapter executable/code identity drifted after capture")
        _, event_data = _raw_reference(receipt, "adapter_events_jsonl", evidence_root)
        _raw_reference(receipt, "adapter_stderr_log", evidence_root)
        events = _parse_events(event_data)
        semantic_errors, detail = _validate_semantics(receipt, events, contract)
        errors.extend(semantic_errors)
        if _identity(evidence_root) != root_identity:
            errors.append("evidence root identity changed during validation")
        _, final_receipt_sha = _stable_json(report_path, evidence_root)
        if final_receipt_sha != receipt_sha:
            errors.append("raw receipt changed before final reread")
    except (KeyError, OSError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
        errors.append(str(exc))
        receipt = {}
        contract = {}
        receipt_sha = ""
        detail = {"metrics": {}, "faults": []}
    passed = not errors
    return {
        "schema_version": 3,
        "report_id": "tzcup_formal_a19_reliability_fault_validation_v3",
        "status": PASSED if passed else BLOCKED,
        "passed": passed,
        "errors": sorted(set(errors)),
        "contract_id": contract.get("contract_id"),
        "raw_receipt_sha256": receipt_sha,
        "source_binding": receipt.get("source_binding"),
        "runtime_gate_binding": receipt.get("runtime_gate_binding"),
        "metrics": detail["metrics"],
        "faults": detail["faults"],
        "raw_evidence": receipt.get("raw_evidence"),
        "claim_boundary": contract.get("claim_boundary"),
    }


def _safe_output(path: Path, root: Path) -> None:
    root, path = _normalized(root), _normalized(path)
    try:
        path.parent.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"output escapes repository root: {path}") from exc
    _no_symlink_ancestor(path.parent)
    path.parent.resolve(strict=True).relative_to(root.resolve(strict=True))
    if path.exists() or path.is_symlink():
        raise ValueError(f"refusing to overwrite retained output: {path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--acceptance-session", type=Path, required=True)
    parser.add_argument("--runtime-closure", type=Path, required=True)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--repository-root", type=Path, default=ROOT)
    args = parser.parse_args()
    try:
        _safe_output(args.output, args.repository_root)
        result = validate(
            args.report, args.snapshot, args.acceptance_session, args.runtime_closure,
            args.evidence_root, args.contract, args.repository_root,
        )
        descriptor, temporary = tempfile.mkstemp(prefix=".a19-", suffix=".pending", dir=args.output.parent)
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(json.dumps(result, indent=2, sort_keys=True) + "\n")
            stream.flush(); os.fsync(stream.fileno())
        Path(temporary).replace(args.output)
        persisted = json.loads(args.output.read_text(encoding="utf-8"))
        if persisted != result:
            raise ValueError("output changed before final reread")
    except (OSError, ValueError) as exc:
        print(json.dumps({"status": "INVALID", "error": str(exc)})); return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["passed"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
