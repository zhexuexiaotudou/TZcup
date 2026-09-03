#!/usr/bin/env python3
"""Fail-closed identity binding for the formal preembedded grasp world."""

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
VEHICLE_MODEL_NAME = "tzcup_formal_sanitation_vehicle"
CUBE_MODEL_NAME = "material_cube"
VEHICLE_INITIAL_POSE = "0 0 0.005 0 0 0"
CUBE_INITIAL_POSE = "0.300 -0.950 0.017 0 0 0"
SPAWN_MODE = "preembedded_before_gazebo_sensors_system"


class PreembeddedGraspWorldBindingError(RuntimeError):
    """The grasp contact world is not the current source-bound formal world."""


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _normalized_pose(value: str) -> str:
    try:
        values = tuple(float(item) for item in value.split())
    except ValueError as error:
        raise PreembeddedGraspWorldBindingError("model pose is not numeric") from error
    if len(values) != 6 or not all(math.isfinite(item) for item in values):
        raise PreembeddedGraspWorldBindingError("model pose must have six finite values")
    values = tuple(0.0 if abs(item) < 1e-12 else item for item in values)
    return " ".join(f"{item:.12g}" for item in values)


def _regular_file(path: Path, label: str, *, not_before_ns: int | None = None) -> Path:
    if path.is_symlink() or not path.is_file():
        raise PreembeddedGraspWorldBindingError(
            f"{label} must be a regular non-symbolic file: {path}"
        )
    resolved = path.resolve(strict=True)
    if not_before_ns is not None and path.stat().st_mtime_ns < not_before_ns:
        raise PreembeddedGraspWorldBindingError(
            f"{label} predates the acceptance session"
        )
    return resolved


def _require_path_hash(
    payload: dict[str, Any], path_key: str, hash_key: str, expected: Path, label: str
) -> str:
    reported = Path(str(payload.get(path_key, "")))
    if reported.resolve() != expected:
        raise PreembeddedGraspWorldBindingError(f"{label} path differs from its bound source")
    actual_hash = _sha256(expected)
    if payload.get(hash_key) != actual_hash:
        raise PreembeddedGraspWorldBindingError(f"{label} SHA-256 differs from its bound source")
    return actual_hash


def _controller_binding_valid(report: dict[str, Any], runtime_install_root: Path) -> tuple[Path, str]:
    binding = report.get("controller_runtime_binding")
    if not isinstance(binding, dict):
        raise PreembeddedGraspWorldBindingError("grasp report has no controller runtime binding")
    expected = runtime_install_root / CONTROLLER_RUNTIME_RELATIVE
    controller = _regular_file(
        Path(str(binding.get("resolved_controller_config", ""))), "controller config"
    )
    if (
        Path(str(binding.get("runtime_install_root", ""))).resolve() != runtime_install_root
        or controller != expected
        or binding.get("controller_config_relative_to_install")
        != CONTROLLER_RUNTIME_RELATIVE.as_posix()
    ):
        raise PreembeddedGraspWorldBindingError(
            "controller binding differs from the frozen runtime contract"
        )
    controller_hash = _sha256(controller)
    if binding.get("controller_config_sha256") != controller_hash:
        raise PreembeddedGraspWorldBindingError(
            "controller config SHA-256 differs from the grasp report"
        )
    return controller, controller_hash


def validate_preembedded_grasp_world(
    *,
    report_path: Path,
    world_path: Path,
    vehicle_urdf_path: Path,
    cube_urdf_path: Path,
    source_world_path: Path,
    acceptance_session: dict[str, Any],
    snapshot_identity: dict[str, str],
    expected_runtime_install_root: Path,
    expected_vehicle_pose: str = VEHICLE_INITIAL_POSE,
    expected_cube_pose: str = CUBE_INITIAL_POSE,
) -> dict[str, Any]:
    """Validate both preloaded grasp models before and after final execution."""

    session_started = acceptance_session.get("started_epoch_ns")
    session_hash = acceptance_session.get("session_manifest_sha256")
    if not isinstance(session_started, int) or session_started <= 0:
        raise PreembeddedGraspWorldBindingError("acceptance session start is invalid")
    if not isinstance(session_hash, str) or len(session_hash) != 64:
        raise PreembeddedGraspWorldBindingError("acceptance session digest is invalid")

    report_path = _regular_file(report_path, "preembedded grasp report", not_before_ns=session_started)
    world_path = _regular_file(world_path, "preembedded grasp world", not_before_ns=session_started)
    vehicle_urdf_path = _regular_file(
        vehicle_urdf_path, "preembedded vehicle URDF", not_before_ns=session_started
    )
    cube_urdf_path = _regular_file(
        cube_urdf_path, "preembedded cube URDF", not_before_ns=session_started
    )
    source_world_path = _regular_file(source_world_path, "frozen grasp source world")
    if not expected_runtime_install_root.is_absolute():
        raise PreembeddedGraspWorldBindingError("frozen runtime install root must be absolute")
    if (
        expected_runtime_install_root.is_symlink()
        or not expected_runtime_install_root.is_dir()
    ):
        raise PreembeddedGraspWorldBindingError("frozen runtime install root is not regular")
    runtime_install_root = expected_runtime_install_root.resolve(strict=True)
    _regular_file(runtime_install_root / "setup.bash", "frozen runtime setup")

    try:
        report = json.loads(report_path.read_text(encoding="utf-8"))
        root = ET.parse(world_path).getroot()
    except (UnicodeError, json.JSONDecodeError, ET.ParseError) as error:
        raise PreembeddedGraspWorldBindingError(
            f"invalid preembedded grasp evidence: {error}"
        ) from error
    if not isinstance(report, dict):
        raise PreembeddedGraspWorldBindingError("preembedded grasp report root must be an object")
    additional = report.get("additional_model")
    if not isinstance(additional, dict):
        raise PreembeddedGraspWorldBindingError("preembedded grasp report has no cube model binding")
    if (
        report.get("status") != "FORMAL_PREEMBEDDED_SENSOR_WORLD_READY"
        or report.get("passed") is not True
        or report.get("formal_eligible") is not True
        or report.get("spawn_mode") != SPAWN_MODE
        or report.get("model_name") != VEHICLE_MODEL_NAME
        or additional.get("model_name") != CUBE_MODEL_NAME
    ):
        raise PreembeddedGraspWorldBindingError("preembedded grasp report has an invalid spawn/model contract")

    report_hash = _sha256(report_path)
    world_hash = _require_path_hash(
        report, "output_world", "output_world_sha256", world_path, "preembedded grasp world"
    )
    vehicle_hash = _require_path_hash(
        report, "vehicle_urdf", "vehicle_urdf_sha256", vehicle_urdf_path, "vehicle URDF"
    )
    cube_hash = _require_path_hash(
        additional, "urdf", "urdf_sha256", cube_urdf_path, "cube URDF"
    )
    source_world_hash = _require_path_hash(
        report, "source_world", "source_world_sha256", source_world_path, "source world"
    )
    vehicle_pose = _normalized_pose(str(report.get("model_initial_pose", "")))
    cube_pose = _normalized_pose(str(additional.get("model_initial_pose", "")))
    if vehicle_pose != _normalized_pose(expected_vehicle_pose):
        raise PreembeddedGraspWorldBindingError("vehicle initial pose differs from launch contract")
    if cube_pose != _normalized_pose(expected_cube_pose):
        raise PreembeddedGraspWorldBindingError("cube initial pose differs from launch contract")

    controller_path, controller_hash = _controller_binding_valid(report, runtime_install_root)

    world = root.find("world")
    if world is None:
        raise PreembeddedGraspWorldBindingError("preembedded grasp SDF has no world")
    vehicle_models = [model for model in world.findall("model") if model.get("name") == VEHICLE_MODEL_NAME]
    cube_models = [model for model in world.findall("model") if model.get("name") == CUBE_MODEL_NAME]
    if len(vehicle_models) != 1 or len(cube_models) != 1:
        raise PreembeddedGraspWorldBindingError(
            "preembedded grasp world must contain exactly one vehicle and one material cube"
        )
    if _normalized_pose(vehicle_models[0].findtext("pose") or "") != vehicle_pose:
        raise PreembeddedGraspWorldBindingError("world vehicle pose differs from grasp report")
    if _normalized_pose(cube_models[0].findtext("pose") or "") != cube_pose:
        raise PreembeddedGraspWorldBindingError("world cube pose differs from grasp report")
    control_candidates = [
        plugin
        for plugin in vehicle_models[0].findall(".//plugin")
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
        or (plugins[0].findtext("parameters") or "").strip() != str(controller_path)
    ):
        raise PreembeddedGraspWorldBindingError(
            "preembedded grasp world has no single bound gz_ros2_control authority"
        )
    return {
        "preembedded_report_path": str(report_path),
        "preembedded_report_sha256": report_hash,
        "preembedded_world_path": str(world_path),
        "preembedded_world_sha256": world_hash,
        "vehicle_urdf_path": str(vehicle_urdf_path),
        "vehicle_urdf_sha256": vehicle_hash,
        "cube_urdf_path": str(cube_urdf_path),
        "cube_urdf_sha256": cube_hash,
        "source_world_path": str(source_world_path),
        "source_world_sha256": source_world_hash,
        "controller_config_path": str(controller_path),
        "controller_config_sha256": controller_hash,
        "runtime_install_root": str(runtime_install_root),
        "vehicle_model_name": VEHICLE_MODEL_NAME,
        "cube_model_name": CUBE_MODEL_NAME,
        "vehicle_initial_pose": vehicle_pose,
        "cube_initial_pose": cube_pose,
        "spawn_mode": SPAWN_MODE,
        "acceptance_session_sha256": session_hash,
        "snapshot": dict(snapshot_identity),
    }
