#!/usr/bin/env python3
"""Run the real product graph behind a fail-closed A19 sensor-fault proxy.

This adapter never turns a producer command into evidence by echoing it.  The
six supported input faults are applied to ROS messages consumed by the real PC
perception node and are acknowledged only after the proxy observes the effect.
Faults without a product-side injection point are reported UNSUPPORTED before
the expensive soak starts.
"""

from __future__ import annotations

import argparse
import copy
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
})
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
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
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
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.operator = self.create_publisher(Bool, "/product_demo/operator_start", latched)
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
            self.create_subscription(Odometry, "/odom/unfiltered", lambda m: self._remember("raw_odom", (float(m.pose.pose.position.x), float(m.pose.pose.position.y))), 20)

        def _proxy(self, channel: str, message: Any) -> None:
            forwarded = self.controller.apply(channel, message)
            if forwarded is not None:
                self.proxy_publishers[channel].publish(forwarded)

        def _remember(self, name: str, value: Any) -> None:
            with self.lock:
                self.last[name] = (time.monotonic(), value)
                self.last_activity = time.monotonic()

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
                argv = parse_product_argv(args.product_argv_json)
                log_path = Path(args.product_log).resolve()
                if log_path.exists() or log_path.is_symlink():
                    raise AdapterError(f"refusing stale product log: {log_path}")
                log_path.parent.mkdir(parents=True, exist_ok=True)
                product_log = log_path.open("xb")
                product = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=product_log, stderr=subprocess.STDOUT, cwd=args.repository_root)
                wait_for(lambda: all(value == "OPERATIONAL" for value in probe.components().values()), args.startup_timeout_s, "product component readiness")
                for _ in range(12):
                    probe.command_operator(True); time.sleep(0.1)
                wait_for(probe.safety_running, args.readback_timeout_s, "armed safety")
                probe.product_started = True
                emitter.emit({"type": "hello", "command_id": command["command_id"], "components": contract["required_pipeline_components"], "profiles": [row["profile"] for row in contract["profile_schedule"]], "faults": [row["fault"] for row in contract["fault_schedule"]], "product_pid": product.pid, "sensor_proxy_topics": PROXY_TOPICS})
                sampling.set()
                sampler = threading.Thread(target=sample_loop, name="a19-sampler", daemon=True)
                sampler.start()
            elif kind == "set_profile":
                requested = command.get("profile")
                if requested != "nominal":
                    emitter.emit({"type": "profile_unsupported", "command_id": command.get("command_id"), "profile": requested, "reason": "sim2real profile has no live drivetrain/sensor parameter consumer"})
                    raise AdapterError(f"UNSUPPORTED live product profile: {requested}")
                profile = requested
                emitter.emit({"type": "profile_activated", "command_id": command["command_id"], "profile": profile, "readback": {"sensor_latency_ms": 0, "sensor_dropout_probability": 0.0, "wheel_slip_ratio": 0.0, "actuator_gain": 1.0}})
            elif kind == "inject_fault":
                fault = str(command.get("fault"))
                parameters = command.get("parameters")
                if not isinstance(parameters, Mapping):
                    raise AdapterError("fault parameters are not an object")
                started = time.monotonic()
                probe.controller.begin(fault, parameters)
                emitter.emit({"type": "fault_injected", "command_id": command["command_id"], "fault": fault, "profile": command["profile"], "parameters": dict(parameters)})
                for _ in range(10):
                    probe.command_operator(False); time.sleep(0.05)
                wait_for(lambda: probe.controller.readback() is not None, args.readback_timeout_s, f"{fault} injection")
                wait_for(probe.safety_stopped, args.readback_timeout_s, f"{fault} safety stop")
                brake_latency_s = time.monotonic() - started
                wait_for(probe.cleaning_safe, args.readback_timeout_s, f"{fault} cleaning inhibit")
                wait_for(probe.perception_degraded, args.readback_timeout_s, f"{fault} perception degradation")
                readback = probe.controller.readback()
                emitter.emit({"type": "fault_state", "fault": fault, "state": "STOPPED", "safety_state": "STOPPED", "pending_clean_outcome": "DEFERRED", "perception_health": "DEGRADED", "nav2_operational": probe.value("nav2") is not None, "watchdog_operational": probe.value("safety", 1.0) is not None, "unsafe_cleaning_action_count": probe.unsafe_cleaning_action_count, "brake_latency_s": brake_latency_s, "injection_readback": readback, "cleaning_inhibit_readback": probe.value("spot_cleaning")})
                duration = float(parameters.get("duration_s", 0.0))
                time.sleep(max(0.0, duration - (time.monotonic() - started)))
                probe.controller.clear()
                for _ in range(12):
                    probe.command_operator(True); time.sleep(0.1)
                recovered: dict[str, Any] = {}
                def sensor_recovered() -> bool:
                    readback = probe.controller.recovery_readback()
                    if readback is None:
                        return False
                    recovered.update(readback)
                    return True
                wait_for(sensor_recovered, args.readback_timeout_s, f"{fault} sensor recovery")
                wait_for(probe.safety_running, args.readback_timeout_s, f"{fault} safety recovery")
                wait_for(probe.coverage_running, args.readback_timeout_s, f"{fault} coverage recovery")
                emitter.emit({"type": "fault_state", "fault": fault, "state": "RECOVERED", "safety_state": "RUNNING", "coverage_state": "RUNNING", "recovery_readback": recovered})
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
