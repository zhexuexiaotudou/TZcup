#!/usr/bin/env python3
"""Run the real product graph behind a fail-closed A19 sensor-fault proxy.

This adapter never turns a producer command into evidence by echoing it.  Its
sensor faults alter messages consumed by the real PC perception node; product
faults drive that node's real model/provider hooks and await diagnostics.
Faults without a product-side injection point are reported UNSUPPORTED before
the expensive soak starts.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Mapping, Sequence


PROTOCOL = "tzcup.formal_a19.adapter.v1"
SUPPORTED_FAULTS = frozenset({
    "rgb_freeze",
    "depth_freeze",
    "timestamp_skew",
    "camera_info_mismatch",
    "tf_unavailable",
    "invalid_depth",
    "proposal_flood",
    "proposal_dropout",
    "classifier_exception",
    "classifier_timeout",
    "action_verifier_failure",
    "reobserve_timeout",
    "cuda_provider_failure",
    "model_hash_mismatch",
    "corrupt_model",
    "sustained_slow_inference",
    "nav2_path_unavailable",
    "dynamic_obstacle_blocks_observation",
})
MODEL_PROVIDER_FAULTS = frozenset({
    "cuda_provider_failure", "model_hash_mismatch", "corrupt_model",
    "sustained_slow_inference",
})
SENSOR_PROXY_FAULTS = frozenset({
    "rgb_freeze", "depth_freeze", "timestamp_skew", "camera_info_mismatch",
    "tf_unavailable", "invalid_depth",
})
MODEL_PROVIDER_FAULT_TOPIC = "/formal_a19/perception_fault"
PROFILE_CONFIG = "starter_ws/src/sanitation_tasks/config/sim2real_fault_profiles.yaml"
PROFILE_NAMES = ("nominal", "transport_stress", "wet_surface", "degraded_drive")
DRIVETRAIN_PROFILE_TOPIC = "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/profile"
DRIVETRAIN_STATUS_TOPIC = "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/status"
PROXY_TOPICS = {
    "front_rgb": (
        "/sensors/front_rgbd/depth/image_rect_raw/image",
        "/formal_a19/proxy/front_rgb",
    ),
    "front_depth": (
        "/sensors/front_rgbd/depth/image_rect_raw/depth_image",
        "/formal_a19/proxy/front_depth",
    ),
    "front_info": (
        "/sensors/front_rgbd/depth/image_rect_raw/camera_info",
        "/formal_a19/proxy/front_camera_info",
    ),
    "wrist_rgb": (
        "/sensors/wrist_rgbd/depth/image_rect_raw/image",
        "/formal_a19/proxy/wrist_rgb",
    ),
    "wrist_depth": (
        "/sensors/wrist_rgbd/depth/image_rect_raw/depth_image",
        "/formal_a19/proxy/wrist_depth",
    ),
    "wrist_info": (
        "/sensors/wrist_rgbd/depth/image_rect_raw/camera_info",
        "/formal_a19/proxy/wrist_camera_info",
    ),
    "rear_left_rgb": (
        "/sensors/rear_left_fisheye/image_raw",
        "/formal_a19/proxy/rear_left_rgb",
    ),
    "rear_right_rgb": (
        "/sensors/rear_right_fisheye/image_raw",
        "/formal_a19/proxy/rear_right_rgb",
    ),
}
PRODUCT_TOPIC_OVERRIDES = {
    "perception_front_rgb_topic": PROXY_TOPICS["front_rgb"][1],
    "perception_front_depth_topic": PROXY_TOPICS["front_depth"][1],
    "perception_front_camera_info_topic": PROXY_TOPICS["front_info"][1],
    "perception_wrist_rgb_topic": PROXY_TOPICS["wrist_rgb"][1],
    "perception_wrist_depth_topic": PROXY_TOPICS["wrist_depth"][1],
    "perception_wrist_camera_info_topic": PROXY_TOPICS["wrist_info"][1],
    "perception_rear_left_rgb_topic": PROXY_TOPICS["rear_left_rgb"][1],
    "perception_rear_right_rgb_topic": PROXY_TOPICS["rear_right_rgb"][1],
}


class AdapterError(RuntimeError):
    """The real product adapter cannot produce truthful A19 evidence."""


def load_profile_settings(path: Path) -> dict[str, dict[str, float]]:
    """Load the existing profile contract; do not duplicate its values in code."""
    try:
        import yaml
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (ImportError, OSError, UnicodeError, ValueError) as exc:
        raise AdapterError(f"cannot read A19 profile configuration: {exc}") from exc
    rows = value.get("profiles") if isinstance(value, Mapping) else None
    if not isinstance(rows, Mapping) or tuple(rows) != PROFILE_NAMES:
        raise AdapterError("A19 profile configuration must retain the four frozen profiles")
    result: dict[str, dict[str, float]] = {}
    for name in PROFILE_NAMES:
        row = rows.get(name)
        if not isinstance(row, Mapping):
            raise AdapterError(f"A19 profile {name} is malformed")
        parsed: dict[str, float] = {}
        for field in ("sensor_latency_ms", "sensor_dropout_probability", "wheel_slip_ratio", "actuator_gain"):
            value = row.get(field)
            if type(value) not in (int, float) or not math.isfinite(float(value)):
                raise AdapterError(f"A19 profile {name}.{field} is invalid")
            parsed[field] = float(value)
        if parsed["sensor_latency_ms"] < 0 or not 0 <= parsed["sensor_dropout_probability"] < 1:
            raise AdapterError(f"A19 profile {name} has invalid sensor settings")
        if not 0 <= parsed["wheel_slip_ratio"] < 1 or not 0 < parsed["actuator_gain"] <= 1:
            raise AdapterError(f"A19 profile {name} has invalid drive settings")
        result[name] = parsed
    return result


def physical_profile_readback_from_status(
    payload: str | Mapping[str, Any], settings: Mapping[str, float]
) -> dict[str, Any]:
    """Accept only the native plant's command-to-output profile telemetry."""
    try:
        row = json.loads(payload) if isinstance(payload, str) else dict(payload)
        profile = row["profile"]
        slip_ratio = float(profile["wheel_slip_ratio"])
        actuator_gain = float(profile["actuator_gain"])
        command = [float(value) for value in row["commanded_wheel_speed_rad_s"]]
        effective = [float(value) for value in row["effective_wheel_speed_rad_s"]]
        unscaled_torque = [float(value) for value in row["unscaled_wheel_torque_nm"]]
        applied_torque = [float(value) for value in row["applied_wheel_torque_nm"]]
        measured_speed = [float(value) for value in row["measured_wheel_speed_rad_s"]]
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise AdapterError(f"malformed native drivetrain profile readback: {exc}") from exc
    arrays = (command, effective, unscaled_torque, applied_torque, measured_speed)
    if any(len(values) != 4 or not all(math.isfinite(value) for value in values) for values in arrays):
        raise AdapterError("native drivetrain profile readback requires four finite wheel values")
    expected_slip = float(settings["wheel_slip_ratio"])
    expected_gain = float(settings["actuator_gain"])
    if not math.isclose(slip_ratio, expected_slip, abs_tol=1e-9) or not math.isclose(actuator_gain, expected_gain, abs_tol=1e-9):
        raise AdapterError("native drivetrain profile values do not match the requested product profile")
    slip_scale = 1.0 / (1.0 - slip_ratio)
    if any(not math.isclose(actual, requested * slip_scale, abs_tol=1e-7) for requested, actual in zip(command, effective)):
        raise AdapterError("native wheel-end command does not reflect requested slip ratio")
    if any(not math.isclose(actual, requested * actuator_gain, abs_tol=1e-7) for requested, actual in zip(unscaled_torque, applied_torque)):
        raise AdapterError("native actuator output does not reflect requested gain")
    if (slip_ratio != 0.0 or actuator_gain != 1.0) and (
        not any(abs(value) > 1e-6 for value in command)
        or not any(abs(value) > 1e-6 for value in unscaled_torque)
    ):
        raise AdapterError("non-nominal profile has no live wheel command and actuator output readback")
    return {
        "source": DRIVETRAIN_STATUS_TOPIC,
        "wheel_slip_ratio": slip_ratio,
        "actuator_gain": actuator_gain,
        "commanded_wheel_speed_rad_s": command,
        "effective_wheel_speed_rad_s": effective,
        "unscaled_wheel_torque_nm": unscaled_torque,
        "applied_wheel_torque_nm": applied_torque,
        "measured_wheel_speed_rad_s": measured_speed,
    }


def validate_fault_expectations(contract: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    rows = contract.get("fault_expectations")
    if not isinstance(rows, Mapping) or set(rows) != SUPPORTED_FAULTS:
        raise AdapterError("A19 contract must declare one fault-specific expectation per supported fault")
    result: dict[str, dict[str, Any]] = {}
    for fault in SUPPORTED_FAULTS:
        row = rows[fault]
        if not isinstance(row, Mapping) or row.get("state") not in {"STOPPED", "DEGRADED"}:
            raise AdapterError(f"A19 fault expectation is malformed: {fault}")
        for key in ("requires_global_safety_stop", "requires_perception_degraded", "requires_cleaning_inhibit"):
            if type(row.get(key)) is not bool:
                raise AdapterError(f"A19 fault expectation {fault}.{key} must be boolean")
        if row["state"] == "STOPPED" and not row["requires_global_safety_stop"]:
            raise AdapterError(f"A19 STOPPED expectation must require a real safety stop: {fault}")
        result[fault] = dict(row)
    return result


def parse_product_argv(raw: str) -> list[str]:
    try:
        value = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise AdapterError(f"product argv is not valid JSON: {exc}") from exc
    if not isinstance(value, list) or not value or any(
        not isinstance(item, str) or not item for item in value
    ):
        raise AdapterError("product argv must be a non-empty JSON string array")
    executable = Path(value[0])
    resolved = executable if executable.is_absolute() else Path(shutil.which(value[0]) or "")
    if not str(resolved) or not resolved.is_file():
        raise AdapterError(f"product executable is unavailable: {value[0]}")
    joined = "\0".join(value)
    required = (
        "ros2",
        "launch",
        "sanitation_product_demo_integration",
        "product_demo.launch.py",
        "gui:=false",
    )
    if any(token not in value for token in required):
        raise AdapterError("product argv must launch the headless canonical product_demo graph")
    for name, topic in PRODUCT_TOPIC_OVERRIDES.items():
        if f"{name}:={topic}" not in value:
            raise AdapterError(f"product argv does not bind the A19 proxy output: {name}")
    if any("fixture" in token.lower() for token in value):
        raise AdapterError("product argv may not reference a fixture")
    if "eval" in joined or "bash -c" in joined or "sh -c" in joined:
        raise AdapterError("product argv may not use a shell evaluator")
    return value


def launch_argument(argv: Sequence[str], name: str) -> str:
    prefix = f"{name}:="
    values = [item[len(prefix):] for item in argv if item.startswith(prefix)]
    if len(values) != 1 or not values[0]:
        raise AdapterError(f"product argv must bind exactly one {name}:= value")
    return values[0]


def frozen_obstacle_target(argv: Sequence[str]) -> tuple[str, str, tuple[float, float, float]]:
    """Use frozen schedule/start inputs; never consume a Gazebo truth topic."""
    schedule_path = Path(launch_argument(argv, "pedestrian_schedule"))
    manifest_path = Path(launch_argument(argv, "episode_manifest"))
    try:
        schedule = json.loads(schedule_path.read_text(encoding="utf-8"))
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        model = schedule["pedestrians"][0]["object_id"]
        world = schedule["world_name"]
        start = manifest["source_fixed_start_pose"]
        x, y, yaw = (float(start[index]) for index in range(3))
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError, IndexError) as exc:
        raise AdapterError(f"cannot bind dynamic obstacle to frozen product inputs: {exc}") from exc
    if not isinstance(model, str) or not model or not isinstance(world, str) or not world:
        raise AdapterError("frozen pedestrian schedule has no model/world identity")
    return world, model, (x, y, yaw)


def contract_capabilities(contract: Mapping[str, Any]) -> tuple[list[str], list[str]]:
    rows = contract.get("fault_schedule")
    if not isinstance(rows, list):
        raise AdapterError("A19 contract fault_schedule is missing")
    faults = [row.get("fault") for row in rows if isinstance(row, Mapping)]
    if len(faults) != len(rows) or any(not isinstance(item, str) for item in faults):
        raise AdapterError("A19 contract fault schedule is malformed")
    supported = [item for item in faults if item in SUPPORTED_FAULTS]
    unsupported = [item for item in faults if item not in SUPPORTED_FAULTS]
    return supported, unsupported


def _positive_number(value: Any, label: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(float(value)) or float(value) <= 0:
        raise AdapterError(f"{label} must be a positive finite number")
    return float(value)


def validate_fault_parameters(fault: str, parameters: Mapping[str, Any]) -> None:
    if fault not in SUPPORTED_FAULTS:
        raise AdapterError(f"UNSUPPORTED product fault: {fault}")
    if not isinstance(parameters, Mapping):
        raise AdapterError("fault parameters must be an object")
    if fault in {"rgb_freeze", "depth_freeze", "tf_unavailable"}:
        _positive_number(parameters.get("duration_s"), f"{fault}.duration_s")
    elif fault == "timestamp_skew":
        _positive_number(parameters.get("duration_s"), "timestamp_skew.duration_s")
        _positive_number(parameters.get("skew_ms"), "timestamp_skew.skew_ms")
    elif fault == "camera_info_mismatch":
        _positive_number(parameters.get("duration_s"), "camera_info_mismatch.duration_s")
        _positive_number(parameters.get("width_delta_px"), "camera_info_mismatch.width_delta_px")
    elif fault == "invalid_depth":
        _positive_number(parameters.get("duration_s"), "invalid_depth.duration_s")
        if parameters.get("invalid_fraction") != 1.0:
            raise AdapterError("invalid_depth currently supports only an observed 1.0 invalid fraction")
    elif fault == "proposal_flood":
        _positive_number(parameters.get("proposals_per_frame"), "proposal_flood.proposals_per_frame")
        _positive_number(parameters.get("duration_s"), "proposal_flood.duration_s")
    elif fault == "proposal_dropout":
        if parameters.get("drop_probability") != 1.0:
            raise AdapterError("proposal_dropout currently supports only an observed 1.0 drop probability")
        _positive_number(parameters.get("duration_s"), "proposal_dropout.duration_s")
    elif fault == "classifier_exception":
        _positive_number(parameters.get("exception_count"), "classifier_exception.exception_count")
    elif fault in {"classifier_timeout", "reobserve_timeout"}:
        _positive_number(parameters.get("timeout_s"), f"{fault}.timeout_s")
        _positive_number(parameters.get("occurrences"), f"{fault}.occurrences")
    elif fault == "action_verifier_failure":
        _positive_number(parameters.get("reject_count"), "action_verifier_failure.reject_count")
    elif fault == "cuda_provider_failure":
        if parameters.get("provider") != "CUDAExecutionProvider":
            raise AdapterError("cuda_provider_failure requires CUDAExecutionProvider")
        _positive_number(parameters.get("duration_s"), "cuda_provider_failure.duration_s")
    elif fault == "model_hash_mismatch":
        if parameters.get("model") != "dosod" or parameters.get("mismatch_count") != 1:
            raise AdapterError("model_hash_mismatch requires dosod and mismatch_count=1")
    elif fault == "corrupt_model":
        if parameters.get("model") != "edgesam" or int(parameters.get("corrupt_bytes", 0)) <= 0:
            raise AdapterError("corrupt_model requires edgesam and positive corrupt_bytes")
    elif fault == "sustained_slow_inference":
        _positive_number(parameters.get("latency_ms"), "sustained_slow_inference.latency_ms")
        _positive_number(parameters.get("duration_s"), "sustained_slow_inference.duration_s")
    elif fault == "nav2_path_unavailable":
        _positive_number(parameters.get("duration_s"), "nav2_path_unavailable.duration_s")
    elif fault == "dynamic_obstacle_blocks_observation":
        _positive_number(parameters.get("duration_s"), "dynamic_obstacle_blocks_observation.duration_s")
        _positive_number(parameters.get("minimum_block_distance_m"), "dynamic_obstacle_blocks_observation.minimum_block_distance_m")


def _shift_stamp(stamp: Any, skew_ms: float) -> None:
    total = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    total += int(skew_ms * 1_000_000)
    stamp.sec, stamp.nanosec = divmod(total, 1_000_000_000)


def transform_sensor_message(
    channel: str, message: Any, fault: str | None, parameters: Mapping[str, Any]
) -> tuple[Any | None, dict[str, Any] | None]:
    """Return the actually forwarded message and an independently inspectable mutation."""
    if fault is None:
        return message, None
    validate_fault_parameters(fault, parameters)
    if fault == "rgb_freeze" and channel.endswith("rgb"):
        return None, {"action": "dropped", "channel": channel}
    if fault == "depth_freeze" and channel.endswith("depth"):
        return None, {"action": "dropped", "channel": channel}
    target = copy.deepcopy(message)
    if fault == "timestamp_skew" and channel.endswith("rgb"):
        before = (int(target.header.stamp.sec), int(target.header.stamp.nanosec))
        _shift_stamp(target.header.stamp, float(parameters["skew_ms"]))
        after = (int(target.header.stamp.sec), int(target.header.stamp.nanosec))
        return target, {"action": "timestamp_shifted", "channel": channel, "before": before, "after": after}
    if fault == "camera_info_mismatch" and channel.endswith("info"):
        before = int(target.width)
        target.width = before + int(parameters["width_delta_px"])
        return target, {"action": "camera_info_width_changed", "channel": channel, "before": before, "after": int(target.width)}
    if fault == "tf_unavailable" and channel.endswith(("rgb", "depth")):
        before = str(target.header.frame_id)
        target.header.frame_id = f"formal_a19_missing_{before}"
        return target, {"action": "frame_rewritten_to_unavailable", "channel": channel, "before": before, "after": target.header.frame_id}
    if fault == "invalid_depth" and channel.endswith("depth"):
        before_nonzero = sum(byte != 0 for byte in bytes(target.data))
        target.data = bytes(len(target.data))
        return target, {"action": "depth_payload_invalidated", "channel": channel, "before_nonzero_bytes": before_nonzero, "after_nonzero_bytes": 0, "size_bytes": len(target.data)}
    return target, None


class SensorFaultController:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._active: tuple[str, dict[str, Any]] | None = None
        self._baseline: dict[str, dict[str, int]] = {}
        self._recovery_baseline: dict[str, dict[str, int]] = {}
        self._recovery_fault: str | None = None
        self._counts = {
            channel: {"input": 0, "output": 0, "dropped": 0, "mutated": 0}
            for channel in PROXY_TOPICS
        }
        self._last_mutation: dict[str, Any] | None = None
        self._profile = "nominal"
        self._profile_settings: dict[str, float] = {
            "sensor_latency_ms": 0.0, "sensor_dropout_probability": 0.0,
            "wheel_slip_ratio": 0.0, "actuator_gain": 1.0,
        }
        self._profile_baseline = copy.deepcopy(self._counts)

    def begin(self, fault: str, parameters: Mapping[str, Any]) -> None:
        validate_fault_parameters(fault, parameters)
        with self._lock:
            if self._active is not None:
                raise AdapterError("another sensor fault is already active")
            self._active = (fault, dict(parameters))
            self._baseline = copy.deepcopy(self._counts)
            self._last_mutation = None

    def apply(self, channel: str, message: Any) -> Any | None:
        with self._lock:
            self._counts[channel]["input"] += 1
            fault, parameters = self._active or (None, {})
            profile = self._profile
            settings = dict(self._profile_settings)
            sequence = self._counts[channel]["input"]
        delay_s = settings["sensor_latency_ms"] / 1000.0
        if delay_s:
            time.sleep(delay_s)
        probability = settings["sensor_dropout_probability"]
        drop_modulus = int(round(1.0 / probability)) if probability else 0
        if drop_modulus and sequence % drop_modulus == 0:
            with self._lock:
                self._counts[channel]["dropped"] += 1
                self._last_mutation = {"fault": fault, "profile": profile, "action": "profile_dropped", "channel": channel, "drop_modulus": drop_modulus}
            return None
        with self._lock:
            forwarded, mutation = transform_sensor_message(channel, message, fault, parameters)
            if forwarded is None:
                self._counts[channel]["dropped"] += 1
            else:
                self._counts[channel]["output"] += 1
            if mutation is not None:
                if forwarded is not None:
                    self._counts[channel]["mutated"] += 1
                self._last_mutation = {"fault": fault, **mutation}
            return forwarded

    def set_profile(self, profile: str, settings: Mapping[str, float]) -> None:
        if profile not in PROFILE_NAMES:
            raise AdapterError(f"unknown A19 profile: {profile}")
        with self._lock:
            self._profile = profile
            self._profile_settings = dict(settings)
            self._profile_baseline = copy.deepcopy(self._counts)

    def profile_readback(self) -> dict[str, Any] | None:
        with self._lock:
            changed = {channel: {key: counts[key] - self._profile_baseline[channel][key] for key in counts} for channel, counts in self._counts.items()}
            ingress = sum(row["input"] for row in changed.values())
            egress = sum(row["output"] for row in changed.values())
            if ingress <= 0 or egress <= 0:
                return None
            probability = self._profile_settings["sensor_dropout_probability"]
            return {"profile": self._profile, "observed": True, "ingress_messages": ingress, "egress_messages": egress, "dropped_messages": sum(row["dropped"] for row in changed.values()), "sensor_latency_ms": self._profile_settings["sensor_latency_ms"], "sensor_dropout_probability": probability, "drop_modulus": int(round(1.0 / probability)) if probability else None, "channel_counts": changed}

    def readback(self) -> dict[str, Any] | None:
        with self._lock:
            if self._active is None:
                return None
            fault, parameters = self._active
            if fault in {"rgb_freeze", "depth_freeze"}:
                suffix = "rgb" if fault == "rgb_freeze" else "depth"
                rows = []
                for channel, counts in self._counts.items():
                    if not channel.endswith(suffix):
                        continue
                    before = self._baseline[channel]
                    if counts["input"] > before["input"] and counts["output"] == before["output"]:
                        rows.append(channel)
                if rows:
                    return {"fault": fault, "observed": True, "dropped_channels": sorted(rows), "counts": copy.deepcopy(self._counts)}
                return None
            if self._last_mutation and self._last_mutation.get("fault") == fault:
                return {"fault": fault, "observed": True, "mutation": copy.deepcopy(self._last_mutation), "parameters": dict(parameters)}
            return None

    def clear(self) -> dict[str, Any]:
        with self._lock:
            if self._active is None:
                raise AdapterError("no sensor fault is active")
            fault = self._active[0]
            self._active = None
            self._recovery_fault = fault
            self._recovery_baseline = copy.deepcopy(self._counts)
            return {"fault": fault, "cleared": True, "counts": copy.deepcopy(self._counts)}

    def recovery_readback(self) -> dict[str, Any] | None:
        with self._lock:
            fault = self._recovery_fault
            if fault is None:
                return None
            if fault not in SENSOR_PROXY_FAULTS:
                return None
            if fault in {"rgb_freeze", "timestamp_skew", "tf_unavailable"}:
                suffixes = ("rgb",)
            elif fault in {"depth_freeze", "invalid_depth"}:
                suffixes = ("depth",)
            elif fault == "camera_info_mismatch":
                suffixes = ("info",)
            else:
                return None
            recovered = [
                channel
                for channel, counts in self._counts.items()
                if channel.endswith(suffixes)
                and counts["input"] > self._recovery_baseline[channel]["input"]
                and counts["output"] > self._recovery_baseline[channel]["output"]
            ]
            if not recovered:
                return None
            self._recovery_fault = None
            return {"fault": fault, "observed": True, "forwarded_channels": sorted(recovered), "counts": copy.deepcopy(self._counts)}

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {"active_fault": None if self._active is None else self._active[0], "counts": copy.deepcopy(self._counts), "last_mutation": copy.deepcopy(self._last_mutation)}

    def queue_growth_count(self) -> int:
        with self._lock:
            return sum(
                int(row["input"] != row["output"] + row["dropped"])
                for row in self._counts.values()
            )


def process_group_rss_bytes(process_group_id: int) -> int:
    if os.name != "posix" or not Path("/proc").is_dir():
        raise AdapterError("process RSS evidence requires Linux /proc")
    page_size = os.sysconf("SC_PAGE_SIZE")
    total = 0
    for item in Path("/proc").iterdir():
        if not item.name.isdigit():
            continue
        try:
            fields = (item / "stat").read_text(encoding="utf-8").split()
            if len(fields) <= 4 or int(fields[4]) != process_group_id:
                continue
            resident_pages = int((item / "statm").read_text(encoding="utf-8").split()[1])
            total += resident_pages * page_size
        except (OSError, UnicodeError, ValueError, IndexError):
            continue
    return total


def process_group_command_pids(process_group_id: int, token: str) -> set[int]:
    if os.name != "posix" or not Path("/proc").is_dir():
        raise AdapterError("process identity evidence requires Linux /proc")
    result: set[int] = set()
    for item in Path("/proc").iterdir():
        if not item.name.isdigit():
            continue
        try:
            fields = (item / "stat").read_text(encoding="utf-8").split()
            command = (item / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8")
            if len(fields) > 4 and int(fields[4]) == process_group_id and token in command:
                result.add(int(item.name))
        except (OSError, UnicodeError, ValueError):
            continue
    return result


class JsonEmitter:
    def __init__(self, nonce: str) -> None:
        self.nonce = nonce
        self.sequence = 0
        self.lock = threading.Lock()

    def emit(self, payload: Mapping[str, Any]) -> None:
        with self.lock:
            row = dict(payload)
            row.update(protocol=PROTOCOL, nonce=self.nonce, adapter_sequence=self.sequence)
            self.sequence += 1
            sys.stdout.write(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n")
            sys.stdout.flush()


def _run_ros_adapter(args: argparse.Namespace, contract: Mapping[str, Any]) -> int:
    try:
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from lifecycle_msgs.msg import State, Transition
        from lifecycle_msgs.srv import ChangeState, GetState
        from nav2_msgs.action import ComputePathToPose
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from ros_gz_interfaces.msg import Entity
        from ros_gz_interfaces.srv import SetEntityPose
        from sanitation_perception_interfaces.msg import GarbageTargetArray
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import Bool, Float64MultiArray, String
    except ImportError as exc:
        raise AdapterError(f"ROS product runtime imports are unavailable: {exc}") from exc

    nonce = os.environ.get("TZCUP_FORMAL_A19_NONCE", "")
    if not nonce or os.environ.get("TZCUP_FORMAL_A19_PROTOCOL") != PROTOCOL:
        raise AdapterError("producer protocol environment is missing")
    emitter = JsonEmitter(nonce)
    supported, unsupported = contract_capabilities(contract)
    fault_expectations = validate_fault_expectations(contract)
    profiles = load_profile_settings(args.repository_root / PROFILE_CONFIG)
    profile_config_sha256 = hashlib.sha256((args.repository_root / PROFILE_CONFIG).read_bytes()).hexdigest()

    class Probe(Node):
        def __init__(self) -> None:
            super().__init__("formal_a19_real_product_adapter")
            self.controller = SensorFaultController()
            self.lock = threading.Lock()
            self.last: dict[str, tuple[float, Any]] = {}
            self.unsafe_cleaning_action_count = 0
            self.cleaning_requested = False
            self.actuators_enabled = False
            self.product_started = False
            self.last_activity = time.monotonic()
            self.tf_failure_since: float | None = None
            self.initial_perception_pids: set[int] | None = None
            self.product_fault_baseline: dict[str, int] = {}
            self.model_provider_fault_events: dict[tuple[str, str], dict[str, Any]] = {}
            self.raw_odom_pose: tuple[float, float, float] | None = None
            self.nearest_scan_m = math.inf
            self.navigation_scan_at = 0.0
            self.blocker: dict[str, Any] | None = None
            self.planner_change = self.create_client(ChangeState, "/planner_server/change_state")
            self.planner_state = self.create_client(GetState, "/planner_server/get_state")
            self.compute_path = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
            self.set_pose_clients: dict[str, Any] = {}
            self.create_timer(0.1, self._keep_dynamic_blocker)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.operator = self.create_publisher(Bool, "/product_demo/operator_start", latched)
            self.fault_control = self.create_publisher(String, "/formal_a19/fault_control", latched)
            self.model_provider_fault = self.create_publisher(
                String, MODEL_PROVIDER_FAULT_TOPIC, 10
            )
            self.drivetrain_profile = self.create_publisher(
                Float64MultiArray, DRIVETRAIN_PROFILE_TOPIC, 10
            )
            message_types = {
                "front_rgb": Image, "front_depth": Image, "front_info": CameraInfo,
                "wrist_rgb": Image, "wrist_depth": Image, "wrist_info": CameraInfo,
                "rear_left_rgb": Image, "rear_right_rgb": Image,
            }
            self.proxy_publishers = {}
            for channel, (source, target) in PROXY_TOPICS.items():
                message_type = message_types[channel]
                publisher = self.create_publisher(message_type, target, qos_profile_sensor_data)
                self.proxy_publishers[channel] = publisher
                self.create_subscription(
                    message_type, source,
                    lambda message, name=channel: self._proxy(name, message),
                    qos_profile_sensor_data,
                )
            self.create_subscription(String, "/safety/status_json", lambda m: self._remember("safety", m.data), 20)
            self.create_subscription(String, DRIVETRAIN_STATUS_TOPIC, lambda m: self._remember("drivetrain", m.data), 20)
            self.create_subscription(Bool, "/safety/actuators_enabled", self._on_actuators, 20)
            self.create_subscription(Bool, "/active_cleaning/cleaning_requested", self._on_cleaning, 20)
            self.create_subscription(Float64MultiArray, "/brush_controller/commands", self._on_cleaning_output, 20)
            self.create_subscription(Float64MultiArray, "/recovery_controller/commands", self._on_cleaning_output, 20)
            self.create_subscription(DiagnosticArray, "/perception/open_vocab/diagnostics", lambda m: self._diagnostic("perception", m), latched)
            self.create_subscription(DiagnosticArray, "/active_cleaning/observation_status", lambda m: self._diagnostic("observation", m), latched)
            self.create_subscription(DiagnosticArray, "/active_cleaning/planner_status", lambda m: self._diagnostic("coverage", m), latched)
            self.create_subscription(DiagnosticArray, "/active_cleaning/executor_status", lambda m: self._diagnostic("nav2", m), latched)
            self.create_subscription(DiagnosticArray, "/active_cleaning/cleaning_status", lambda m: self._diagnostic("spot_cleaning", m), 10)
            self.create_subscription(DiagnosticArray, "/manipulation/formal_grasp_status", lambda m: self._diagnostic("post_clean_verification", m), 10)
            self.create_subscription(GarbageTargetArray, "/perception/garbage/targets", lambda m: self._remember("tracking", len(m.targets)), 10)
            self.create_subscription(OccupancyGrid, "/active_cleaning/ground_dirt_belief", lambda m: self._remember("dynamic_trash_map", len(m.data)), latched)
            self.create_subscription(Odometry, "/odom", lambda m: self._remember("odom", (float(m.pose.pose.position.x), float(m.pose.pose.position.y))), 20)
            self.create_subscription(Odometry, "/odom/unfiltered", self._on_raw_odom, 20)
            from sensor_msgs.msg import LaserScan
            self.create_subscription(LaserScan, "/scan/navigation", self._on_scan, qos_profile_sensor_data)

        def _proxy(self, channel: str, message: Any) -> None:
            forwarded = self.controller.apply(channel, message)
            if forwarded is not None:
                self.proxy_publishers[channel].publish(forwarded)

        def _remember(self, name: str, value: Any) -> None:
            with self.lock:
                self.last[name] = (time.monotonic(), value)
                self.last_activity = time.monotonic()

        def _on_raw_odom(self, message: Any) -> None:
            position, orientation = message.pose.pose.position, message.pose.pose.orientation
            yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y), 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
            self.raw_odom_pose = (float(position.x), float(position.y), yaw)
            self._remember("raw_odom", (float(position.x), float(position.y)))

        def _on_scan(self, message: Any) -> None:
            values = [float(value) for value in message.ranges if math.isfinite(float(value))]
            if values:
                self.nearest_scan_m = min(values)
            self.navigation_scan_at = time.monotonic()
            self._remember("navigation_scan", self.nearest_scan_m)

        def _wait_future(self, future: Any, timeout_s: float, label: str) -> Any:
            deadline = time.monotonic() + timeout_s
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            if not future.done() or future.result() is None:
                raise AdapterError(f"timed out waiting for {label}")
            return future.result()

        def _planner_state_id(self) -> int:
            if not self.planner_state.wait_for_service(timeout_sec=2.0):
                raise AdapterError("planner_server get_state service unavailable")
            return int(self._wait_future(self.planner_state.call_async(GetState.Request()), 3.0, "planner state").current_state.id)

        def set_planner_active(self, active: bool) -> dict[str, Any]:
            if not self.planner_change.wait_for_service(timeout_sec=2.0):
                raise AdapterError("planner_server change_state service unavailable")
            before = self._planner_state_id()
            request = ChangeState.Request(); request.transition.id = Transition.TRANSITION_ACTIVATE if active else Transition.TRANSITION_DEACTIVATE
            if not self._wait_future(self.planner_change.call_async(request), 3.0, "planner lifecycle transition").success:
                raise AdapterError("planner lifecycle transition was rejected")
            expected = State.PRIMARY_STATE_ACTIVE if active else State.PRIMARY_STATE_INACTIVE
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                after, action_ready = self._planner_state_id(), self.compute_path.server_is_ready()
                if after == expected and action_ready is active:
                    return {"planner_state_before": before, "planner_state_after": after, "compute_path_server_ready": action_ready}
                time.sleep(0.05)
            raise AdapterError("planner lifecycle transition had no independent action-server readback")

        def begin_dynamic_blocker(self, *, world: str, model: str, start: tuple[float, float, float], minimum_distance_m: float) -> None:
            from gazebo_ground_truth import read_named_model_pose
            if not math.isfinite(self.nearest_scan_m) or time.monotonic() - self.navigation_scan_at > 1.0:
                raise AdapterError("fresh navigation scan is required before dynamic blocker injection")
            client = self.set_pose_clients.get(world)
            if client is None:
                client = self.create_client(SetEntityPose, f"/world/{world}/set_pose"); self.set_pose_clients[world] = client
            if not client.wait_for_service(timeout_sec=5.0):
                raise AdapterError("Gazebo SetEntityPose service unavailable for A19 blocker")
            try:
                original_pose = read_named_model_pose(world_name=world, model_name=model, timeout_s=5.0)
            except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                raise AdapterError(f"cannot independently read dynamic blocker origin: {exc}") from exc
            self.blocker = {
                "world": world, "model": model, "start": start,
                "minimum_distance_m": minimum_distance_m, "success_count": 0,
                "restore_success_count": 0, "original_pose": original_pose,
                "baseline_scan_m": self.nearest_scan_m, "active": True,
            }
            self._set_dynamic_blocker()

        def _set_dynamic_blocker(self) -> None:
            blocker = self.blocker
            if blocker is None or not blocker["active"] or self.raw_odom_pose is None:
                return
            ox, oy, oyaw = self.raw_odom_pose; sx, sy, syaw = blocker["start"]
            x = sx + math.cos(syaw) * ox - math.sin(syaw) * oy
            y = sy + math.sin(syaw) * ox + math.cos(syaw) * oy
            distance = float(blocker["minimum_distance_m"])
            x += math.cos(syaw + oyaw) * distance; y += math.sin(syaw + oyaw) * distance
            blocker["target_pose"] = {"x": x, "y": y, "z": 0.0, "yaw": syaw + oyaw}
            request = SetEntityPose.Request(); request.entity.name = str(blocker["model"]); request.entity.type = Entity.MODEL
            request.pose.position.x = x; request.pose.position.y = y; request.pose.position.z = 0.0
            request.pose.orientation.z = math.sin((syaw + oyaw) / 2.0); request.pose.orientation.w = math.cos((syaw + oyaw) / 2.0)
            future = self.set_pose_clients[str(blocker["world"])].call_async(request)
            def recorded(done: Any) -> None:
                try:
                    if done.result() is not None and done.result().success: blocker["success_count"] += 1
                except Exception:
                    pass
            future.add_done_callback(recorded)

        def _keep_dynamic_blocker(self) -> None:
            self._set_dynamic_blocker()

        def dynamic_blocker_readback(self) -> dict[str, Any] | None:
            blocker = self.blocker
            nearest = self.value("navigation_scan", 1.0)
            if blocker is None or int(blocker["success_count"]) <= 0 or nearest is None or float(nearest) > float(blocker["minimum_distance_m"]) + 0.2:
                return None
            try:
                from gazebo_ground_truth import read_named_model_pose
                native_pose = read_named_model_pose(world_name=str(blocker["world"]), model_name=str(blocker["model"]), timeout_s=5.0)
            except (OSError, RuntimeError, subprocess.SubprocessError):
                return None
            target = blocker.get("target_pose")
            if not isinstance(target, Mapping) or math.dist((native_pose["x"], native_pose["y"], native_pose["z"]), (target["x"], target["y"], target["z"])) > 0.05:
                return None
            return {"observed": True, "set_pose_success_count": int(blocker["success_count"]), "nearest_navigation_scan_m": float(nearest), "native_pose": native_pose, "target_pose": dict(target), "model": blocker["model"], "world": blocker["world"]}

        def clear_dynamic_blocker(self) -> dict[str, Any]:
            blocker = self.blocker
            if blocker is None: raise AdapterError("dynamic blocker is not active")
            blocker["active"] = False
            blocker["restore_started_at"] = time.monotonic()
            original = blocker["original_pose"]
            request = SetEntityPose.Request(); request.entity.name = str(blocker["model"]); request.entity.type = Entity.MODEL
            request.pose.position.x = float(original["x"]); request.pose.position.y = float(original["y"]); request.pose.position.z = float(original["z"])
            request.pose.orientation.z = math.sin(float(original["yaw"]) / 2.0); request.pose.orientation.w = math.cos(float(original["yaw"]) / 2.0)
            future = self.set_pose_clients[str(blocker["world"])].call_async(request)
            def recorded(done: Any) -> None:
                try:
                    if done.result() is not None and done.result().success: blocker["restore_success_count"] += 1
                except Exception:
                    pass
            future.add_done_callback(recorded)
            return {"restore_requested": True, "model": blocker["model"]}

        def dynamic_blocker_recovery_readback(self) -> dict[str, Any] | None:
            blocker = self.blocker
            if blocker is None or blocker["active"] or int(blocker["restore_success_count"]) <= 0:
                return None
            restored_at = float(blocker["restore_started_at"])
            if self.navigation_scan_at <= restored_at:
                return None
            try:
                from gazebo_ground_truth import read_named_model_pose
                native_pose = read_named_model_pose(world_name=str(blocker["world"]), model_name=str(blocker["model"]), timeout_s=5.0)
            except (OSError, RuntimeError, subprocess.SubprocessError):
                return None
            original = blocker["original_pose"]
            pose_error = math.dist((native_pose["x"], native_pose["y"], native_pose["z"]), (original["x"], original["y"], original["z"]))
            scan = self.value("navigation_scan", 1.0)
            if pose_error > 0.05 or scan is None or float(scan) + 0.05 < float(blocker["baseline_scan_m"]):
                return None
            readback = {"observed": True, "restore_set_pose_success_count": int(blocker["restore_success_count"]), "native_pose": native_pose, "original_pose": original, "native_position_error_m": pose_error, "post_restore_navigation_scan_m": float(scan), "model": blocker["model"], "world": blocker["world"]}
            self.blocker = None
            return readback

        def _diagnostic(self, name: str, message: Any) -> None:
            rows = [{
                "name": row.name,
                "level": int(row.level),
                "message": row.message,
                "values": {item.key: item.value for item in row.values},
            } for row in message.status]
            if name == "perception":
                now = time.monotonic()
                failed = any(row["message"] in {"map_tf_missing", "stale_map_tf_rejected"} for row in rows)
                succeeded = any(row["message"] in {"rgbd_product_ok", "dosod_product_ok"} for row in rows)
                with self.lock:
                    if failed and self.tf_failure_since is None:
                        self.tf_failure_since = now
                    elif succeeded:
                        self.tf_failure_since = None
                    for row in rows:
                        if row["message"] not in {"a19_fault_active", "a19_fault_recovered"}:
                            continue
                        fault = row["values"].get("fault")
                        if isinstance(fault, str):
                            self.model_provider_fault_events[(fault, row["message"])] = dict(row)
            self._remember(name, rows)

        def _on_actuators(self, message: Any) -> None:
            with self.lock:
                self.actuators_enabled = bool(message.data)
                if self.cleaning_requested and not self.actuators_enabled:
                    # A request while inhibited is safely blocked, not unsafe actuation.
                    pass

        def _on_cleaning(self, message: Any) -> None:
            with self.lock:
                self.cleaning_requested = bool(message.data)

        def _unsafe_output(self, nonzero: bool) -> None:
            with self.lock:
                if nonzero and not self.actuators_enabled:
                    self.unsafe_cleaning_action_count += 1

        def _on_cleaning_output(self, message: Any) -> None:
            self._unsafe_output(any(abs(float(value)) > 1e-9 for value in message.data))

        def value(self, name: str, maximum_age_s: float = 3.0) -> Any | None:
            with self.lock:
                row = self.last.get(name)
            if row is None or time.monotonic() - row[0] > maximum_age_s:
                return None
            return row[1]

        def command_operator(self, armed: bool) -> None:
            self.operator.publish(Bool(data=armed))

        def command_drive_profile(self, settings: Mapping[str, float]) -> None:
            self.drivetrain_profile.publish(Float64MultiArray(data=[
                float(settings["wheel_slip_ratio"]), float(settings["actuator_gain"]),
            ]))

        def physical_profile_readback(self, settings: Mapping[str, float]) -> dict[str, Any] | None:
            raw = self.value("drivetrain", 1.0)
            if raw is None:
                return None
            try:
                return physical_profile_readback_from_status(raw, settings)
            except AdapterError:
                return None

        def safety_state(self) -> str:
            if self.safety_stopped():
                return "STOPPED"
            if self.safety_running():
                return "RUNNING"
            return "UNKNOWN"

        @staticmethod
        def _fault_values(rows: Any) -> tuple[str, int] | None:
            if not isinstance(rows, list):
                return None
            for row in rows:
                values = row.get("values") if isinstance(row, Mapping) else None
                if not isinstance(values, Mapping):
                    continue
                try:
                    fault = json.loads(str(values.get("formal_a19_fault", '""')))
                    events = json.loads(str(values.get("formal_a19_fault_events", "0")))
                except json.JSONDecodeError:
                    fault = str(values.get("formal_a19_fault", ""))
                    try:
                        events = int(str(values.get("formal_a19_fault_events", "0")))
                    except ValueError:
                        continue
                if isinstance(fault, str) and type(events) is int:
                    return fault, events
            return None

        def command_product_fault(self, fault: str, parameters: Mapping[str, Any], active: bool) -> None:
            self.fault_control.publish(String(data=json.dumps({
                "fault": fault, "parameters": dict(parameters), "active": active,
            }, sort_keys=True, separators=(",", ":"))))

        def command_model_provider_fault(self, action: str, fault: str, parameters: Mapping[str, Any]) -> None:
            self.model_provider_fault.publish(String(data=json.dumps({
                "action": action, "fault": fault, "parameters": dict(parameters),
            }, sort_keys=True)))

        def model_provider_fault_readback(self, fault: str, message: str) -> dict[str, Any] | None:
            with self.lock:
                row = self.model_provider_fault_events.get((fault, message))
            return copy.deepcopy(row) if row is not None else None

        def product_fault_readback(self, fault: str, *, recovered: bool = False) -> dict[str, Any] | None:
            source = "post_clean_verification" if fault == "action_verifier_failure" else "perception"
            rows = self.value(source, 2.0)
            parsed = self._fault_values(rows)
            if parsed is None:
                return None
            active_fault, events = parsed
            if recovered:
                if active_fault:
                    return None
            elif active_fault != fault or events <= self.product_fault_baseline.get(fault, 0):
                return None
            return {"fault": fault, "observed": True, "source": source, "active_fault": active_fault, "events": events, "rows": rows}

        def safety_stopped(self) -> bool:
            raw = self.value("safety", 1.0)
            try:
                value = json.loads(raw)
            except (TypeError, json.JSONDecodeError):
                return False
            return value.get("actuators_enabled") is False and value.get("state") in {"INHIBITED", "BASE_COMMAND_STOPPED"}

        def safety_running(self) -> bool:
            raw = self.value("safety", 1.0)
            try:
                value = json.loads(raw)
            except (TypeError, json.JSONDecodeError):
                return False
            return value.get("actuators_enabled") is True and value.get("state") == "ENABLED"

        def perception_degraded(self) -> bool:
            rows = self.value("observation", 1.0) or self.value("perception", 1.0)
            return bool(rows) and any(int(row["level"]) >= 2 for row in rows)

        def cleaning_safe(self) -> bool:
            rows = self.value("spot_cleaning", 1.0)
            if not rows:
                return False
            row = rows[-1]
            return row["message"] == "SAFE_INHIBIT" and row["values"].get("safety_permitted") == "false"

        def components(self) -> dict[str, str]:
            mapping = {
                "Coverage": "coverage", "Perception": "perception", "Tracking": "tracking",
                "DynamicTrashMap": "dynamic_trash_map", "Spot Cleaning": "spot_cleaning",
                "Post-Clean Verification": "post_clean_verification",
            }
            return {name: ("OPERATIONAL" if self.value(source) is not None else "DEGRADED") for name, source in mapping.items()}

        def coverage_running(self) -> bool:
            rows = self.value("coverage", 1.0)
            operational = {"IDLE", "WAITING_EXECUTOR", "WAITING_GRASP", "RETURNING_HOME", "COMPLETE"}
            return bool(rows) and any(row["message"] in operational and int(row["level"]) == 0 for row in rows)

        def localization_error(self) -> float:
            filtered, raw = self.value("odom", 2.0), self.value("raw_odom", 2.0)
            if filtered is None or raw is None:
                raise AdapterError("fresh filtered and raw physical odometry are required")
            return math.dist(filtered, raw)

        def deadlock_count(self) -> int:
            with self.lock:
                return int(time.monotonic() - self.last_activity > 5.0)

        def persistent_tf_failure_count(self) -> int:
            with self.lock:
                return int(
                    self.tf_failure_since is not None
                    and time.monotonic() - self.tf_failure_since > 2.5
                )

        def unexpected_model_reload_count(self) -> int:
            current = process_group_command_pids(os.getpgrp(), "pc_open_vocab_product_adapter")
            if self.initial_perception_pids is None:
                self.initial_perception_pids = current
                return 0
            return int(current != self.initial_perception_pids)

    rclpy.init()
    probe = Probe()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(probe)
    spin = threading.Thread(target=executor.spin, name="a19-ros-probe", daemon=True)
    spin.start()
    product: subprocess.Popen[bytes] | None = None
    product_log = None
    sampling = threading.Event()
    profile = "nominal"
    sampler: threading.Thread | None = None
    initial_operator_arm_commands = 0

    def wait_for(predicate, timeout: float, label: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if product is not None and product.poll() is not None:
                raise AdapterError(f"product graph exited during {label}: {product.returncode}")
            if predicate():
                return
            time.sleep(0.05)
        raise AdapterError(f"timed out waiting for independent {label} readback")

    def sample_loop() -> None:
        period = float(contract["stability_gates"]["sample_period_s"])
        next_at = time.monotonic()
        while sampling.is_set():
            try:
                components = probe.components()
                crash = int(product is None or product.poll() is not None)
                metrics = {
                    "crash_count": crash,
                    "deadlock_count": probe.deadlock_count(),
                    "queue_growth_count": probe.controller.queue_growth_count(),
                    "unexpected_model_reload_count": probe.unexpected_model_reload_count(),
                    "persistent_tf_failure_count": probe.persistent_tf_failure_count(),
                    "unrecoverable_watchdog_event_count": int(probe.value("safety", 1.0) is None),
                    "unsafe_cleaning_action_count": probe.unsafe_cleaning_action_count,
                    "localization_xy_error_m": probe.localization_error(),
                    "memory_rss_bytes": process_group_rss_bytes(os.getpgrp()),
                }
                emitter.emit({"type": "sample", "profile": profile, "components": components, "metrics": metrics, "sensor_proxy": probe.controller.snapshot()})
            except Exception as exc:  # evidence must show observation failure, not invent zeros
                emitter.emit({"type": "adapter_error", "stage": "sample", "error": str(exc)})
                sampling.clear()
                return
            next_at += period
            time.sleep(max(0.0, next_at - time.monotonic()))

    def start_sampling() -> None:
        nonlocal sampler
        if sampling.is_set():
            return
        sampling.set()
        sampler = threading.Thread(target=sample_loop, name="a19-sampler", daemon=True)
        sampler.start()

    def stop_product() -> None:
        nonlocal product_log
        sampling.clear()
        if sampler is not None:
            sampler.join(timeout=2.0)
        if product is not None and product.poll() is None:
            product.send_signal(signal.SIGINT)
            try:
                product.wait(timeout=20.0)
            except subprocess.TimeoutExpired:
                product.terminate()
                try:
                    product.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    product.kill(); product.wait()
        if product_log is not None:
            product_log.flush(); os.fsync(product_log.fileno()); product_log.close(); product_log = None

    def start_product() -> dict[str, Any]:
        nonlocal product, product_log, initial_operator_arm_commands
        argv = parse_product_argv(args.product_argv_json)
        base_log = Path(args.product_log).resolve()
        log_path = base_log
        if log_path.exists() or log_path.is_symlink():
            raise AdapterError(f"refusing stale product log: {log_path}")
        log_path.parent.mkdir(parents=True, exist_ok=True)
        product_log = log_path.open("xb")
        product = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=product_log, stderr=subprocess.STDOUT, cwd=args.repository_root)
        wait_for(lambda: all(value == "OPERATIONAL" for value in probe.components().values()), args.startup_timeout_s, "product component readiness")
        probe.controller.set_profile("nominal", profiles["nominal"])
        # The product graph has an explicit human/operator arm gate.  This is
        # startup only; fault injection and recovery never toggle it.
        probe.command_operator(True); initial_operator_arm_commands += 1
        wait_for(probe.safety_running, args.readback_timeout_s, "armed safety")
        probe.command_drive_profile(profiles["nominal"])
        wait_for(lambda: probe.controller.profile_readback() is not None, args.readback_timeout_s, "nominal profile ingress")
        wait_for(lambda: probe.physical_profile_readback(profiles["nominal"]) is not None, args.readback_timeout_s, "nominal physical profile")
        readback = probe.controller.profile_readback()
        physical_readback = probe.physical_profile_readback(profiles["nominal"])
        assert readback is not None and physical_readback is not None
        return {"product_pid": product.pid, "product_pgid": os.getpgid(product.pid), "product_log": str(log_path), "frozen_product_argv_sha256": hashlib.sha256("\0".join(argv).encode("utf-8")).hexdigest(), "profile_config": PROFILE_CONFIG, "profile_config_sha256": profile_config_sha256, "proxy_readback": readback, "physical_readback": physical_readback, "operator_control": {"initial_arm_commands": initial_operator_arm_commands, "post_start_commands": 0}}

    try:
        for line in sys.stdin:
            command = json.loads(line)
            if command.get("protocol") != PROTOCOL or command.get("nonce") != nonce:
                raise AdapterError("producer command protocol or nonce mismatch")
            kind = command.get("type")
            if kind == "start":
                if unsupported:
                    emitter.emit({
                        "type": "capability_blocked",
                        "command_id": command.get("command_id"),
                        "supported_faults": supported,
                        "unsupported_faults": unsupported,
                        "reason": "product_fault_injection_points_missing",
                    })
                    raise AdapterError("formal A19 contract contains unsupported product faults")
                start_readback = start_product()
                probe.product_started = True
                emitter.emit({"type": "hello", "command_id": command["command_id"], "components": contract["required_pipeline_components"], "profiles": [row["profile"] for row in contract["profile_schedule"]], "faults": [row["fault"] for row in contract["fault_schedule"]], "product_pid": product.pid, "product_pgid": os.getpgid(product.pid), "sensor_proxy_topics": PROXY_TOPICS, "initial_profile_readback": start_readback, "operator_control": start_readback["operator_control"]})
                start_sampling()
            elif kind == "set_profile":
                requested = command.get("profile")
                if requested not in profiles:
                    raise AdapterError(f"UNSUPPORTED live product profile: {requested}")
                profile = requested
                probe.controller.set_profile(profile, profiles[profile])
                probe.command_drive_profile(profiles[profile])
                wait_for(lambda: probe.controller.profile_readback() is not None, args.readback_timeout_s, f"{profile} profile ingress")
                wait_for(lambda: probe.physical_profile_readback(profiles[profile]) is not None, args.readback_timeout_s, f"{profile} physical profile")
                readback = probe.controller.profile_readback()
                physical_readback = probe.physical_profile_readback(profiles[profile])
                assert readback is not None and physical_readback is not None and product is not None
                readback.update({"product_pid": product.pid, "product_pgid": os.getpgid(product.pid), "physical_readback": physical_readback})
                emitter.emit({"type": "profile_activated", "command_id": command["command_id"], "profile": profile, "configured_values": profiles[profile], "readback": readback})
            elif kind == "inject_fault":
                fault = str(command.get("fault"))
                parameters = command.get("parameters")
                if not isinstance(parameters, Mapping):
                    raise AdapterError("fault parameters are not an object")
                started = time.monotonic()
                validate_fault_parameters(fault, parameters)
                expectation = fault_expectations[fault]
                nav2_fault = fault == "nav2_path_unavailable"
                dynamic_obstacle_fault = fault == "dynamic_obstacle_blocks_observation"
                if fault in SENSOR_PROXY_FAULTS:
                    probe.controller.begin(fault, parameters)
                elif fault in MODEL_PROVIDER_FAULTS:
                    probe.command_model_provider_fault("begin", fault, parameters)
                elif nav2_fault:
                    nav2_injection = probe.set_planner_active(False)
                elif dynamic_obstacle_fault:
                    world, model, start = frozen_obstacle_target(parse_product_argv(args.product_argv_json))
                    probe.begin_dynamic_blocker(world=world, model=model, start=start, minimum_distance_m=float(parameters["minimum_block_distance_m"]))
                else:
                    prior = probe.product_fault_readback(fault)
                    probe.product_fault_baseline[fault] = 0 if prior is None else int(prior["events"])
                    probe.command_product_fault(fault, parameters, True)
                emitter.emit({"type": "fault_injected", "command_id": command["command_id"], "fault": fault, "profile": command["profile"], "parameters": dict(parameters)})
                wait_for(
                    lambda: (
                        probe.controller.readback() is not None
                        if fault in SENSOR_PROXY_FAULTS
                        else (
                            probe.model_provider_fault_readback(
                                fault, "a19_fault_active"
                            ) is not None
                            if fault in MODEL_PROVIDER_FAULTS
                            else (nav2_injection is not None if nav2_fault else (probe.dynamic_blocker_readback() is not None if dynamic_obstacle_fault else probe.product_fault_readback(fault) is not None))
                        )
                    ),
                    args.readback_timeout_s,
                    f"{fault} injection",
                )
                if expectation["requires_global_safety_stop"]:
                    wait_for(probe.safety_stopped, args.readback_timeout_s, f"{fault} safety stop")
                    wait_for(probe.cleaning_safe, args.readback_timeout_s, f"{fault} cleaning inhibit")
                if expectation["requires_perception_degraded"]:
                    wait_for(probe.perception_degraded, args.readback_timeout_s, f"{fault} perception degradation")
                brake_latency_s = time.monotonic() - started if expectation["requires_global_safety_stop"] else None
                readback = (
                    probe.controller.readback()
                    if fault in SENSOR_PROXY_FAULTS
                    else (
                        probe.model_provider_fault_readback(fault, "a19_fault_active")
                        if fault in MODEL_PROVIDER_FAULTS
                        else (nav2_injection if nav2_fault else (probe.dynamic_blocker_readback() if dynamic_obstacle_fault else probe.product_fault_readback(fault)))
                    )
                )
                emitter.emit({"type": "fault_state", "fault": fault, "state": expectation["state"], "safety_state": probe.safety_state(), "pending_clean_outcome": "DEFERRED" if probe.cleaning_safe() else None, "perception_health": "DEGRADED" if probe.perception_degraded() else "OPERATIONAL", "nav2_operational": probe.value("nav2") is not None, "watchdog_operational": probe.value("safety", 1.0) is not None, "unsafe_cleaning_action_count": probe.unsafe_cleaning_action_count, "brake_latency_s": brake_latency_s, "injection_readback": readback, "cleaning_inhibit_readback": probe.value("spot_cleaning")})
                # The wrist fault must remain active through the product's
                # existing bounded re-observation wait; clearing it at the
                # first dropped message would merely defer the same request.
                duration = float(
                    parameters.get(
                        "duration_s",
                        parameters.get("timeout_s", 0.0)
                        if fault == "reobserve_timeout" else 0.0,
                    )
                )
                time.sleep(max(0.0, duration - (time.monotonic() - started)))
                if fault in SENSOR_PROXY_FAULTS:
                    probe.controller.clear()
                if fault in MODEL_PROVIDER_FAULTS:
                    probe.command_model_provider_fault("clear", fault, parameters)
                elif nav2_fault:
                    recovery_readback = probe.set_planner_active(True)
                elif dynamic_obstacle_fault:
                    recovery_readback = probe.clear_dynamic_blocker()
                elif fault not in SENSOR_PROXY_FAULTS:
                    probe.command_product_fault(fault, parameters, False)
                recovered: dict[str, Any] = {}
                def sensor_recovered() -> bool:
                    readback = (
                        probe.controller.recovery_readback()
                        if fault in SENSOR_PROXY_FAULTS
                        else (
                            probe.model_provider_fault_readback(
                                fault, "a19_fault_recovered"
                            )
                            if fault in MODEL_PROVIDER_FAULTS
                            else (recovery_readback if nav2_fault else (probe.dynamic_blocker_recovery_readback() if dynamic_obstacle_fault else probe.product_fault_readback(fault, recovered=True)))
                        )
                    )
                    if readback is None:
                        return False
                    recovered.update(readback)
                    return True
                wait_for(sensor_recovered, args.readback_timeout_s, f"{fault} sensor recovery")
                if expectation["requires_global_safety_stop"]:
                    wait_for(probe.safety_running, args.readback_timeout_s, f"{fault} safety recovery")
                wait_for(probe.coverage_running, args.readback_timeout_s, f"{fault} coverage recovery")
                emitter.emit({"type": "fault_state", "fault": fault, "state": "RECOVERED", "safety_state": probe.safety_state(), "coverage_state": "RUNNING", "recovery_readback": recovered})
            elif kind == "shutdown":
                stop_product()
                emitter.emit({"type": "complete", "command_id": command["command_id"], "reason": command["reason"]})
                return 0
            else:
                raise AdapterError(f"unknown producer command: {kind}")
        raise AdapterError("producer command stream ended without shutdown")
    finally:
        stop_product()
        executor.shutdown()
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin.join(timeout=2.0)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--product-argv-json", required=True)
    parser.add_argument("--product-log", required=True)
    parser.add_argument("--startup-timeout-s", type=float, default=240.0)
    parser.add_argument("--readback-timeout-s", type=float, default=15.0)
    parser.add_argument("--capabilities-json", action="store_true")
    args = parser.parse_args(argv)
    try:
        contract_path = Path(os.environ.get("TZCUP_FORMAL_A19_CONTRACT", ""))
        if not contract_path.is_file():
            raise AdapterError("TZCUP_FORMAL_A19_CONTRACT is unavailable")
        contract = json.loads(contract_path.read_text(encoding="utf-8"))
        supported, unsupported = contract_capabilities(contract)
        if args.capabilities_json:
            print(json.dumps({"supported_faults": supported, "unsupported_faults": unsupported}, sort_keys=True))
            return 0 if not unsupported else 4
        parse_product_argv(args.product_argv_json)
        return _run_ros_adapter(args, contract)
    except (AdapterError, OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"formal A19 product adapter refused: {exc}", file=sys.stderr)
        return 78


if __name__ == "__main__":
    raise SystemExit(main())
