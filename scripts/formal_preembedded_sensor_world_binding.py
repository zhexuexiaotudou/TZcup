#!/usr/bin/env python3
"""Fail-closed identity binding for a preembedded formal sensor world."""

from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


CONTROLLER_RUNTIME_RELATIVE = Path(
    "share/sanitation_vehicle_description/config/formal_vehicle_controllers.yaml"
)
FORMAL_CONTROLLER_FILENAME = "gz_ros2_control-system"
FORMAL_CONTROLLER_NAME = "gz_ros2_control::GazeboSimROS2ControlPlugin"


class PreembeddedWorldBindingError(RuntimeError):
    """The generated world is not the current session-bound formal source."""


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise PreembeddedWorldBindingError(f"cannot read {label}: {error}") from error
    if not isinstance(value, dict):
        raise PreembeddedWorldBindingError(f"{label} root must be an object")
    return value


def _normalize_pose(value: str) -> str:
    try:
        values = tuple(float(item) for item in value.split())
    except ValueError as error:
        raise PreembeddedWorldBindingError("model initial pose is not numeric") from error
    if len(values) != 6 or not all(math.isfinite(item) for item in values):
        raise PreembeddedWorldBindingError("model initial pose must have six finite values")
    values = tuple(0.0 if abs(item) < 1e-12 else item for item in values)
    return " ".join(f"{item:.12g}" for item in values)


def _expanded_urdf_contract(urdf_path: Path) -> tuple[int, str]:
    try:
        root = ET.parse(urdf_path).getroot()
    except ET.ParseError as error:
        raise PreembeddedWorldBindingError(f"expanded URDF is invalid XML: {error}") from error
    count = sum(1 for _ in root.findall("gazebo/sensor"))
    if count <= 0:
        raise PreembeddedWorldBindingError("expanded URDF has no gazebo sensor contract")
    model_name = (root.get("name") or "").strip()
    if not model_name:
        raise PreembeddedWorldBindingError("expanded URDF robot name is missing")
    return count, model_name


def validate_preembedded_sensor_world(
    *,
    report_path: Path,
    world_path: Path,
    expanded_urdf_path: Path,
    acceptance_session: dict[str, Any],
    snapshot_identity: dict[str, str],
    expected_model_pose: str,
    expected_runtime_install_root: Path,
) -> dict[str, Any]:
    """Validate immutable generated-world identity before any ROS subscription."""

    for path, label in ((report_path, "preembedded report"), (world_path, "preembedded world")):
        if not path.is_file():
            raise PreembeddedWorldBindingError(f"missing {label}: {path}")
    report = _json_object(report_path, "preembedded report")
    session_started = acceptance_session.get("started_epoch_ns")
    if not isinstance(session_started, int) or session_started <= 0:
        raise PreembeddedWorldBindingError("acceptance session start is invalid")
    if report_path.stat().st_mtime_ns < session_started or world_path.stat().st_mtime_ns < session_started:
        raise PreembeddedWorldBindingError("preembedded world evidence predates acceptance session")
    if report.get("status") != "FORMAL_PREEMBEDDED_SENSOR_WORLD_READY" or report.get("passed") is not True:
        raise PreembeddedWorldBindingError("preembedded world report is not ready")
    if report.get("spawn_mode") != "preembedded_before_gazebo_sensors_system":
        raise PreembeddedWorldBindingError("preembedded world report has wrong spawn mode")
    if Path(str(report.get("output_world", ""))).resolve() != world_path.resolve():
        raise PreembeddedWorldBindingError("preembedded report points at another world")
    if report.get("output_world_sha256") != _sha256(world_path):
        raise PreembeddedWorldBindingError("preembedded world SHA-256 differs from its report")
    if not expected_runtime_install_root.is_absolute():
        raise PreembeddedWorldBindingError(
            "expected frozen runtime install root must be absolute"
        )
    try:
        runtime_install_root = expected_runtime_install_root.resolve(strict=True)
    except OSError as error:
        raise PreembeddedWorldBindingError(
            "frozen runtime install root is missing"
        ) from error
    controller_binding = report.get("controller_runtime_binding")
    if not isinstance(controller_binding, dict):
        raise PreembeddedWorldBindingError(
            "preembedded report has no controller runtime binding"
        )
    recorded_runtime_install_root = Path(
        str(controller_binding.get("runtime_install_root", ""))
    )
    if not recorded_runtime_install_root.is_absolute():
        raise PreembeddedWorldBindingError(
            "controller binding runtime install root must be absolute"
        )
    if str(recorded_runtime_install_root.resolve()) != str(recorded_runtime_install_root):
        raise PreembeddedWorldBindingError(
            "controller binding runtime install root is not canonical"
        )
    if recorded_runtime_install_root.resolve() != runtime_install_root:
        raise PreembeddedWorldBindingError(
            "controller binding uses another runtime install root"
        )
    expected_controller = (
        runtime_install_root / CONTROLLER_RUNTIME_RELATIVE
    ).resolve()
    controller_path = Path(
        str(controller_binding.get("resolved_controller_config", ""))
    )
    if not controller_path.is_absolute() or controller_path.resolve() != expected_controller:
        raise PreembeddedWorldBindingError(
            "controller binding does not use the frozen runtime package artifact"
        )
    if controller_path.is_symlink() or not controller_path.is_file():
        raise PreembeddedWorldBindingError(
            "bound controller config is missing, non-regular or symbolic"
        )
    if controller_binding.get("controller_config_relative_to_install") != (
        CONTROLLER_RUNTIME_RELATIVE.as_posix()
    ):
        raise PreembeddedWorldBindingError(
            "controller config relative path differs from the runtime contract"
        )
    controller_hash = _sha256(controller_path)
    if controller_binding.get("controller_config_sha256") != controller_hash:
        raise PreembeddedWorldBindingError(
            "controller config SHA-256 differs from its preembedded report"
        )
    try:
        world_root = ET.parse(world_path).getroot()
    except ET.ParseError as error:
        raise PreembeddedWorldBindingError(
            f"preembedded world is invalid XML: {error}"
        ) from error
    expected_count, expected_model_name = _expanded_urdf_contract(expanded_urdf_path)
    if report.get("model_name") != expected_model_name:
        raise PreembeddedWorldBindingError(
            "preembedded report model name differs from expanded URDF"
        )
    world = world_root.find("world")
    if world is None:
        raise PreembeddedWorldBindingError("preembedded SDF has no world")
    matching_models = [
        model for model in world.findall("model")
        if model.get("name") == expected_model_name
    ]
    if len(matching_models) != 1:
        raise PreembeddedWorldBindingError(
            "preembedded world must contain exactly one expanded-URDF model"
        )
    control_candidates = [
        plugin
        for plugin in matching_models[0].findall(".//plugin")
        if plugin.get("filename") == FORMAL_CONTROLLER_FILENAME
        or plugin.get("name") == FORMAL_CONTROLLER_NAME
    ]
    plugins = [
        plugin
        for plugin in control_candidates
        if plugin.get("filename") == FORMAL_CONTROLLER_FILENAME
        and plugin.get("name") == FORMAL_CONTROLLER_NAME
    ]
    if (
        len(control_candidates) != 1
        or len(plugins) != 1
        or len(plugins[0].findall("parameters")) != 1
    ):
        raise PreembeddedWorldBindingError(
            "preembedded world must contain exactly one complete bound "
            "gz_ros2_control plugin and no competing control authority"
        )
    world_controller = (plugins[0].findtext("parameters") or "").strip()
    if world_controller != str(expected_controller):
        raise PreembeddedWorldBindingError(
            "preembedded world controller parameter differs from frozen runtime"
        )
    source_world_path = Path(str(report.get("source_world", "")))
    if not source_world_path.is_file():
        raise PreembeddedWorldBindingError("preembedded report source world is missing")
    if report.get("source_world_sha256") != _sha256(source_world_path):
        raise PreembeddedWorldBindingError("preembedded report source-world SHA-256 mismatch")
    if Path(str(report.get("vehicle_urdf", ""))).resolve() != expanded_urdf_path.resolve():
        raise PreembeddedWorldBindingError("preembedded report uses another expanded URDF")
    actual_urdf_hash = _sha256(expanded_urdf_path)
    if report.get("vehicle_urdf_sha256") != actual_urdf_hash:
        raise PreembeddedWorldBindingError("preembedded report expanded-URDF SHA-256 mismatch")
    if snapshot_identity.get("expanded_urdf_sha256") != actual_urdf_hash:
        raise PreembeddedWorldBindingError("snapshot does not match current expanded URDF")
    if report.get("sensor_count") != expected_count:
        raise PreembeddedWorldBindingError(
            f"preembedded sensor count mismatch: expected {expected_count}"
        )
    if _normalize_pose(str(report.get("model_initial_pose", ""))) != _normalize_pose(expected_model_pose):
        raise PreembeddedWorldBindingError("preembedded model initial pose differs from launch contract")
    return {
        "preembedded_report_path": str(report_path.resolve()),
        "preembedded_report_sha256": _sha256(report_path),
        "preembedded_world_path": str(world_path.resolve()),
        "preembedded_world_sha256": _sha256(world_path),
        "source_world_path": str(source_world_path.resolve()),
        "source_world_sha256": report.get("source_world_sha256"),
        "source_urdf_path": str(expanded_urdf_path.resolve()),
        "source_urdf_sha256": actual_urdf_hash,
        "runtime_install_root": str(runtime_install_root),
        "controller_config_path": str(expected_controller),
        "controller_config_sha256": controller_hash,
        "sensor_count": expected_count,
        "model_name": expected_model_name,
        "spawn_mode": report["spawn_mode"],
        "model_initial_pose": _normalize_pose(expected_model_pose),
        "acceptance_session_sha256": str(acceptance_session["session_manifest_sha256"]),
        "snapshot": dict(snapshot_identity),
    }
