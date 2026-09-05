#!/usr/bin/env python3
"""Fail-closed geometry and interface checks for the formal vehicle register."""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTER = ROOT / "config" / "high_fidelity_vehicle" / "formal_vehicle_component_register.yaml"
DEFAULT_URDF = ROOT / "reports" / "engineering" / "formal_competition_vehicle.urdf"
DEFAULT_CONTROLLERS = ROOT / "starter_ws" / "src" / "sanitation_vehicle_description" / "config" / "formal_vehicle_controllers.yaml"
DEFAULT_LAUNCH = ROOT / "starter_ws" / "src" / "sanitation_vehicle_description" / "launch" / "formal_vehicle_sim.launch.py"
DEFAULT_BRIDGE_CONFIGS = (
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_vehicle_description"
    / "config"
    / "formal_high_bandwidth_sensor_bridge.yaml",
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_vehicle_description"
    / "config"
    / "formal_visual_sensor_bridge.yaml",
)


class ComponentRegisterError(ValueError):
    pass


Matrix = tuple[tuple[float, float, float, float], ...]

REQUIRED_EXPLICIT_FUNCTIONAL_COMPONENTS = {
    "front_left_a300_motor",
    "front_right_a300_motor",
    "rear_left_a300_motor",
    "rear_right_a300_motor",
    "front_left_a300_encoder",
    "front_right_a300_encoder",
    "rear_left_a300_encoder",
    "rear_right_a300_encoder",
    "left_a300_fixed_beam",
    "right_a300_fixed_beam",
    "left_a300_fixed_spacer",
    "right_a300_fixed_spacer",
    "left_pololu_4694_encoder",
    "right_pololu_4694_encoder",
    "central_pololu_4694_encoder",
    "zed_f9p_receiver_enclosure",
    "zed_f9p_receiver_module",
    "a300_left_battery_pack",
    "a300_right_battery_pack",
    "a300_left_battery_bms",
    "a300_right_battery_bms",
    "charge_port_housing",
    "charge_receptacle",
    "charge_port_door",
    "charge_connector_lock",
    "emergency_stop_housing",
    "emergency_stop_6mm_plunger",
    "wastewater_drain_pipe",
    "wastewater_ball_valve_body",
    "wastewater_ball_valve_ball",
    "wastewater_valve_actuator",
    "wastewater_service_cap",
    "wastewater_hose_coupling",
    "wrist_rgbd_machined_bracket",
    "wrist_rgbd_camera_housing",
    "front_rgbd_ir_left_optical_datum",
    "front_rgbd_ir_right_optical_datum",
    "wrist_rgbd_ir_left_optical_datum",
    "wrist_rgbd_ir_right_optical_datum",
    "main_power_isolator_housing",
    "main_power_isolator_handle",
    "main_power_contactor_housing",
    "main_power_contactor_armature",
    "squeegee_preload_spring_pack",
    "power_service_door_hinge",
    "power_service_door_panel",
    "power_service_door_latch",
    "compute_service_door_hinge",
    "compute_service_door_panel",
    "compute_service_door_latch",
    "wet_service_door_hinge",
    "wet_service_door_panel",
    "wet_service_door_latch",
    "rear_dry_service_door_hinge",
    "rear_dry_service_door_panel",
    "rear_dry_service_door_latch",
    "dry_deposit_presence_sensor",
    "dry_bin_latch_base",
    "dry_bin_latch_handle",
    "dry_bin_latch_keeper",
    "wastewater_lid_latch_base",
    "wastewater_lid_latch_handle",
    "wastewater_lid_latch_keeper",
}


def _identity() -> Matrix:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _multiply(left: Matrix, right: Matrix) -> Matrix:
    return tuple(
        tuple(sum(left[row][index] * right[index][column] for index in range(4)) for column in range(4))
        for row in range(4)
    )


def _inverse_rigid(transform: Matrix) -> Matrix:
    rotation_t = tuple(tuple(transform[column][row] for column in range(3)) for row in range(3))
    translation = tuple(transform[row][3] for row in range(3))
    return tuple(
        rotation_t[row]
        + (-sum(rotation_t[row][index] * translation[index] for index in range(3)),)
        for row in range(3)
    ) + ((0.0, 0.0, 0.0, 1.0),)


def _transform(xyz: Iterable[float], rpy: Iterable[float]) -> Matrix:
    x, y, z = xyz
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y),
        (-sp, cp * sr, cp * cr, z),
        (0.0, 0.0, 0.0, 1.0),
    )


def _numbers(value: str | None) -> tuple[float, float, float]:
    values = tuple(float(item) for item in (value or "0 0 0").split())
    if len(values) != 3:
        raise ComponentRegisterError(f"URDF pose vector must have three values: {value!r}")
    return values


def _joint_origin(joint: ET.Element) -> Matrix:
    origin = joint.find("origin")
    if origin is None:
        return _identity()
    return _transform(_numbers(origin.get("xyz")), _numbers(origin.get("rpy")))


def _world_transforms(root: ET.Element, joints: dict[str, ET.Element]) -> dict[str, Matrix]:
    child_links = {joint.find("child").get("link") for joint in joints.values()}
    roots = [link.get("name") for link in root.findall("link") if link.get("name") not in child_links]
    if len(roots) != 1:
        raise ComponentRegisterError(f"expanded URDF must have exactly one root for FK: {roots}")
    transforms = {roots[0]: _identity()}
    pending = list(joints.values())
    while pending:
        remaining: list[ET.Element] = []
        progress = False
        for joint in pending:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in transforms:
                remaining.append(joint)
                continue
            transforms[child] = _multiply(transforms[parent], _joint_origin(joint))
            progress = True
        if not progress:
            unresolved = sorted(joint.get("name") for joint in remaining)
            raise ComponentRegisterError(f"unable to resolve URDF FK tree: {unresolved}")
        pending = remaining
    return transforms


def _rotation_error_rad(expected: Matrix, actual: Matrix) -> float:
    relative = _multiply(_inverse_rigid(expected), actual)
    cosine = max(-1.0, min(1.0, (sum(relative[index][index] for index in range(3)) - 1.0) / 2.0))
    return math.acos(cosine)


_BRIDGE_ARGUMENT = re.compile(
    r"(?P<topic>/[^\s@\"']+)@(?P<ros>[^\s\[\]\"']+)(?P<direction>[\[\]])(?P<gz>[^\s,\"']+)"
)
_LITERAL_REMAP = re.compile(
    r"\(\s*[\"'](?P<source>/[^\"']+)[\"']\s*,\s*[\"'](?P<target>/[^\"']+)[\"']\s*,?\s*\)"
)
_CPP_GZ_TO_ROS_ENDPOINT = re.compile(
    r"constexpr\s+GazeboToRosEndpoint\s*<\s*"
    r"(?P<ros>[A-Za-z0-9_:]+)\s*,\s*(?P<gz>[A-Za-z0-9_:]+)\s*>\s*"
    r"[A-Za-z0-9_]+\s*\{\s*\"(?P<topic>/[^\"]+)\"\s*\}\s*;",
    re.MULTILINE,
)
_CPP_STRING_TOPIC_ROOT = re.compile(
    r"std::string\s+(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*\{\s*"
    r'"(?P<topic>/[^"]+)"\s*\}',
    re.MULTILINE,
)
_CPP_GZ_PUBLISH_ENDPOINT = re.compile(
    r"Advertise\s*<\s*gz::msgs::(?P<gz_type>[A-Za-z_][A-Za-z0-9_]*)\s*>\s*\(\s*"
    r"this->(?P<root>[A-Za-z_][A-Za-z0-9_]*)\s*\+\s*"
    r'"(?P<suffix>/[^"]+)"\s*\)',
    re.MULTILINE,
)


def _bridge_contracts(
    launch_text: str, bridge_config_paths: Iterable[Path] = ()
) -> dict[str, tuple[str, str]]:
    contracts: dict[str, tuple[str, str]] = {}
    remappings = {
        match.group("source"): match.group("target")
        for match in _LITERAL_REMAP.finditer(launch_text)
    }
    for match in _BRIDGE_ARGUMENT.finditer(launch_text):
        source_topic = match.group("topic")
        topic = remappings.get(source_topic, source_topic)
        contract = (match.group("ros"), match.group("gz"))
        if topic in contracts and contracts[topic] != contract:
            raise ComponentRegisterError(f"launch declares conflicting bridge types for {topic}")
        contracts[topic] = contract
    for config_path in bridge_config_paths:
        rows = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        if not isinstance(rows, list):
            raise ComponentRegisterError(
                f"bridge config root must be a list: {config_path}"
            )
        for row in rows:
            if not isinstance(row, dict):
                raise ComponentRegisterError(
                    f"bridge config entries must be mappings: {config_path}"
                )
            if row.get("direction") not in {"GZ_TO_ROS", "BIDIRECTIONAL"}:
                continue
            topic = row.get("ros_topic_name")
            ros_type = row.get("ros_type_name")
            gz_type = row.get("gz_type_name")
            if not all(isinstance(value, str) and value for value in (topic, ros_type, gz_type)):
                raise ComponentRegisterError(
                    f"bridge config entry lacks a typed ROS topic: {config_path}"
                )
            contract = (ros_type, gz_type)
            if topic in contracts and contracts[topic] != contract:
                raise ComponentRegisterError(
                    f"bridge configs declare conflicting types for {topic}"
                )
            contracts[topic] = contract
    return contracts


def _launch_nodes(launch_text: str) -> list[tuple[str, str, str]]:
    """Return literal launch Nodes as (package, executable, name)."""
    nodes: list[tuple[str, str, str]] = []
    tree = ast.parse(launch_text)
    for call in (
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
    ):
        values: dict[str, str] = {}
        for keyword in call.keywords:
            if (
                keyword.arg in {"package", "executable", "name"}
                and isinstance(keyword.value, ast.Constant)
                and isinstance(keyword.value.value, str)
            ):
                values[keyword.arg] = keyword.value.value
        if {"package", "executable", "name"} <= values.keys():
            nodes.append((values["package"], values["executable"], values["name"]))
    return nodes


def _launch_node_remappings(
    launch_text: str,
) -> list[tuple[tuple[str, str, str], dict[str, str]]]:
    """Return literal remaps scoped to their exact launch Node."""
    results: list[tuple[tuple[str, str, str], dict[str, str]]] = []
    tree = ast.parse(launch_text)
    for call in (
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
    ):
        values: dict[str, str] = {}
        remappings: dict[str, str] = {}
        for keyword in call.keywords:
            if (
                keyword.arg in {"package", "executable", "name"}
                and isinstance(keyword.value, ast.Constant)
                and isinstance(keyword.value.value, str)
            ):
                values[keyword.arg] = keyword.value.value
            elif keyword.arg == "remappings" and isinstance(
                keyword.value, (ast.List, ast.Tuple)
            ):
                for item in keyword.value.elts:
                    try:
                        source, target = ast.literal_eval(item)
                    except (ValueError, TypeError, SyntaxError):
                        continue
                    if isinstance(source, str) and isinstance(target, str):
                        remappings[source] = target
        if {"package", "executable", "name"} <= values.keys():
            results.append(
                ((values["package"], values["executable"], values["name"]), remappings)
            )
    return results


def _cpp_gz_to_ros_endpoints(source: Path) -> list[tuple[str, str, str]]:
    """Return compiled typed custom endpoints as (topic, ROS type, GZ type)."""
    endpoints: list[tuple[str, str, str]] = []
    for match in _CPP_GZ_TO_ROS_ENDPOINT.finditer(source.read_text(encoding="utf-8")):
        ros_type = match.group("ros").replace("::", "/")
        gz_type = match.group("gz").replace("gz::msgs::", "gz.msgs.")
        endpoints.append((match.group("topic"), ros_type, gz_type))
    return endpoints


def _cpp_gz_publish_endpoints(source: Path) -> list[tuple[str, str]]:
    """Return literal Gazebo publishers as (topic, GZ type)."""
    source_text = source.read_text(encoding="utf-8")
    topic_roots = {
        match.group("name"): match.group("topic")
        for match in _CPP_STRING_TOPIC_ROOT.finditer(source_text)
    }
    endpoints: list[tuple[str, str]] = []
    for match in _CPP_GZ_PUBLISH_ENDPOINT.finditer(source_text):
        topic_root = topic_roots.get(match.group("root"))
        if topic_root is None:
            continue
        endpoints.append(
            (
                topic_root + match.group("suffix"),
                f"gz.msgs.{match.group('gz_type')}",
            )
        )
    return endpoints


def _python_topic_endpoints(source: Path) -> set[tuple[str, str, str]]:
    """Return literal ROS topic endpoints as (direction, topic, message class)."""
    endpoints: set[tuple[str, str, str]] = set()
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    parameter_defaults: dict[str, str] = {}
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "declare_parameter"
            and len(node.args) >= 2
            and isinstance(node.args[0], ast.Constant)
            and isinstance(node.args[0].value, str)
            and isinstance(node.args[1], ast.Constant)
            and isinstance(node.args[1].value, str)
        ):
            parameter_defaults[node.args[0].value] = node.args[1].value

    def topic_value(node: ast.expr) -> str | None:
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            return node.value
        if (
            isinstance(node, ast.Attribute)
            and node.attr == "value"
            and isinstance(node.value, ast.Call)
            and isinstance(node.value.func, ast.Attribute)
            and node.value.func.attr == "get_parameter"
            and node.value.args
            and isinstance(node.value.args[0], ast.Constant)
            and isinstance(node.value.args[0].value, str)
        ):
            return parameter_defaults.get(node.value.args[0].value)
        return None

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        method = node.func.attr
        if method not in {"create_publisher", "create_subscription"} or len(node.args) < 2:
            continue
        topic = topic_value(node.args[1])
        if topic is None:
            continue
        message_node = node.args[0]
        if isinstance(message_node, ast.Name):
            message_class = message_node.id
        elif isinstance(message_node, ast.Attribute):
            message_class = message_node.attr
        else:
            continue
        direction = "publisher" if method == "create_publisher" else "subscription"
        endpoints.add((direction, topic, message_class))
    return endpoints


def _product_python_sources() -> list[Path]:
    sources: list[Path] = []
    for source in (ROOT / "starter_ws" / "src").rglob("*.py"):
        relative_parts = source.relative_to(ROOT / "starter_ws" / "src").parts
        if "test" in relative_parts or "launch" in relative_parts:
            continue
        sources.append(source)
    return sources


def _product_cpp_sources() -> list[Path]:
    sources: list[Path] = []
    for suffix in ("*.cc", "*.cpp"):
        for source in (ROOT / "starter_ws" / "src").rglob(suffix):
            relative_parts = source.relative_to(ROOT / "starter_ws" / "src").parts
            if "test" in relative_parts:
                continue
            sources.append(source)
    return sources


def validate(
    register_path: Path = DEFAULT_REGISTER,
    urdf_path: Path = DEFAULT_URDF,
    controllers_path: Path = DEFAULT_CONTROLLERS,
    launch_path: Path = DEFAULT_LAUNCH,
    bridge_config_paths: Iterable[Path] | None = None,
) -> dict:
    register = yaml.safe_load(register_path.read_text(encoding="utf-8"))
    controllers = yaml.safe_load(controllers_path.read_text(encoding="utf-8"))
    root = ET.parse(urdf_path).getroot()
    launch_text = launch_path.read_text(encoding="utf-8")
    link_elements = {link.get("name"): link for link in root.findall("link")}
    links = set(link_elements)
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    joint_by_child = {
        joint.find("child").get("link"): joint for joint in joints.values()
    }
    parent_by_child = {
        joint.find("child").get("link"): joint.find("parent").get("link")
        for joint in joints.values()
    }
    transforms = _world_transforms(root, joints)
    if bridge_config_paths is None:
        bridge_config_paths = (
            DEFAULT_BRIDGE_CONFIGS
            if launch_path.resolve() == DEFAULT_LAUNCH.resolve()
            else ()
        )
    bridge_contracts = _bridge_contracts(launch_text, bridge_config_paths)
    launch_nodes = _launch_nodes(launch_text)
    launch_node_remappings = _launch_node_remappings(launch_text)
    urdf_sensor_topics = {
        topic.text.strip()
        for topic in root.findall(".//sensor/topic")
        if topic.text and topic.text.strip()
    }

    errors: list[str] = []

    def is_descendant(child: str, ancestor: str) -> bool:
        seen: set[str] = set()
        cursor = child
        while cursor in parent_by_child and cursor not in seen:
            seen.add(cursor)
            cursor = parent_by_child[cursor]
            if cursor == ancestor:
                return True
        return child == ancestor

    coordinate_contract = register.get("coordinate_contract", {})
    default_reference = coordinate_contract.get("default_reference")
    position_tolerance = coordinate_contract.get("position_tolerance_m")
    orientation_tolerance = coordinate_contract.get("orientation_tolerance_rad")
    if register.get("schema_version", 0) < 6:
        errors.append("component register schema_version must be at least 6")
    if default_reference not in links:
        errors.append(f"coordinate_contract.default_reference missing from URDF: {default_reference}")
    if not isinstance(position_tolerance, (int, float)) or position_tolerance <= 0:
        errors.append("coordinate_contract.position_tolerance_m must be positive")
    if not isinstance(orientation_tolerance, (int, float)) or orientation_tolerance <= 0:
        errors.append("coordinate_contract.orientation_tolerance_rad must be positive")

    def validate_pose(item: dict, label: str, link: str) -> None:
        reference = item.get("coordinate_reference", default_reference)
        xyz = item.get("xyz_m")
        rpy = item.get("rpy_rad")
        if reference not in links:
            errors.append(f"{label}.coordinate_reference missing from URDF: {reference}")
            return
        if link not in links:
            return
        if not is_descendant(link, reference):
            errors.append(f"{label} coordinate link does not descend from {reference}: {link}")
            return
        if not isinstance(xyz, list) or len(xyz) != 3 or not all(isinstance(value, (int, float)) for value in xyz):
            errors.append(f"{label} requires numeric three-element xyz_m")
            return
        if not isinstance(rpy, list) or len(rpy) != 3 or not all(isinstance(value, (int, float)) for value in rpy):
            errors.append(f"{label} requires numeric three-element rpy_rad")
            return
        actual = _multiply(_inverse_rigid(transforms[reference]), transforms[link])
        expected = _transform(xyz, rpy)
        position_error = math.sqrt(sum((actual[index][3] - xyz[index]) ** 2 for index in range(3)))
        rotation_error = _rotation_error_rad(expected, actual)
        if isinstance(position_tolerance, (int, float)) and position_error > position_tolerance:
            errors.append(
                f"{label} FK position error {position_error:.9f} m exceeds {position_tolerance:.9f} m "
                f"for {reference}->{link}"
            )
        if isinstance(orientation_tolerance, (int, float)) and rotation_error > orientation_tolerance:
            errors.append(
                f"{label} FK orientation error {rotation_error:.9f} rad exceeds {orientation_tolerance:.9f} rad "
                f"for {reference}->{link}"
            )
        if item.get("dynamic_position") and "coordinate_reference" not in item:
            errors.append(f"{label} dynamic position requires an explicit coordinate_reference")

    topic_contracts = register.get("topic_contracts", {})
    validated_topic_contracts: set[str] = set()
    validated_single_writer_contracts: set[str] = set()
    validated_gazebo_only_diagnostics: set[str] = set()
    endpoint_cache = {
        source.resolve(): _python_topic_endpoints(source)
        for source in _product_python_sources()
    }
    cpp_bridge_endpoint_cache = {
        source.resolve(): _cpp_gz_to_ros_endpoints(source)
        for source in _product_cpp_sources()
    }
    cpp_gz_publish_endpoint_cache = {
        source.resolve(): _cpp_gz_publish_endpoints(source)
        for source in _product_cpp_sources()
    }
    native_runtime_bindings: set[tuple[str, str, str, str]] = set()
    for candidate_contract in topic_contracts.values():
        if candidate_contract.get("transport") != "gazebo_native_bridge":
            continue
        binding = (
            candidate_contract.get("source_path"),
            candidate_contract.get("bridge_package"),
            candidate_contract.get("bridge_executable"),
            candidate_contract.get("writer_node"),
        )
        if all(isinstance(value, str) and value for value in binding):
            native_runtime_bindings.add(binding)

    def resolved_native_publishers(endpoint: tuple[str, str, str]) -> list[str]:
        publishers: list[str] = []
        for source_path, package, executable, writer_node in sorted(
            native_runtime_bindings
        ):
            candidate_source = (ROOT / source_path).resolve()
            expected_node = (package, executable, writer_node)
            remapping_instances = [
                remappings
                for node, remappings in launch_node_remappings
                if node == expected_node
            ]
            if len(remapping_instances) != 1:
                continue
            remappings = remapping_instances[0]
            resolved_endpoints = {
                (remappings.get(local_topic, local_topic), ros_type, gz_type)
                for local_topic, ros_type, gz_type in cpp_bridge_endpoint_cache.get(
                    candidate_source, []
                )
            }
            if endpoint in resolved_endpoints:
                publishers.append(
                    f"{source_path}::{package}/{executable}/{writer_node}"
                )
        return publishers

    for contract_id, contract in topic_contracts.items():
        topic = contract.get("ros_topic")
        ros_type = contract.get("ros_type")
        transport = contract.get("transport", "gazebo_bridge")
        gz_type = contract.get("gz_type")
        sensor_base_topic = contract.get("sensor_base_topic")
        if transport == "gazebo_only_diagnostic":
            allowed_fields = {
                "transport",
                "direction",
                "single_writer",
                "writer_system",
                "gz_topic",
                "gz_type",
                "source_path",
            }
            unexpected_fields = sorted(set(contract) - allowed_fields)
            if unexpected_fields:
                errors.append(
                    f"Gazebo-only diagnostic contract {contract_id} has forbidden fields: "
                    f"{unexpected_fields}"
                )
            gz_topic = contract.get("gz_topic")
            source_path = contract.get("source_path")
            writer_system = contract.get("writer_system")
            direction = contract.get("direction")
            if not all(
                isinstance(value, str) and value
                for value in (gz_topic, gz_type, source_path, writer_system)
            ):
                errors.append(
                    f"Gazebo-only diagnostic contract {contract_id} requires gz_topic, gz_type, "
                    "source_path and writer_system"
                )
                continue
            if direction != "publisher":
                errors.append(
                    f"Gazebo-only diagnostic contract {contract_id} must declare publisher direction"
                )
            if contract.get("single_writer") is not True:
                errors.append(
                    f"Gazebo-only diagnostic contract {contract_id} must declare single_writer true"
                )
            source = ROOT / source_path
            if not source.is_file():
                errors.append(
                    f"Gazebo-only diagnostic contract {contract_id} source missing: {source_path}"
                )
                continue
            try:
                source.resolve().relative_to((ROOT / "starter_ws" / "src").resolve())
            except ValueError:
                errors.append(
                    f"Gazebo-only diagnostic contract {contract_id} source must remain under "
                    f"starter_ws/src: {source_path}"
                )
            if source.suffix not in {".cc", ".cpp"}:
                errors.append(
                    f"Gazebo-only diagnostic contract {contract_id} source must be C++: {source_path}"
                )
            source_text = source.read_text(encoding="utf-8")
            if re.search(rf"\bclass\s+{re.escape(writer_system)}\b", source_text) is None:
                errors.append(
                    f"Gazebo-only diagnostic contract {contract_id} writer system "
                    f"{writer_system} missing from {source_path}"
                )
            endpoint = (gz_topic, gz_type)
            source_endpoints = cpp_gz_publish_endpoint_cache.get(source.resolve(), [])
            if source_endpoints.count(endpoint) != 1:
                errors.append(
                    f"Gazebo-only diagnostic contract {contract_id} exact Gazebo publisher "
                    f"missing or duplicated in {source_path}: {endpoint}"
                )
            publishers = sorted(
                str(candidate.relative_to(ROOT)).replace("\\", "/")
                for candidate, endpoints in cpp_gz_publish_endpoint_cache.items()
                if endpoint in endpoints
            )
            if publishers != [source_path]:
                errors.append(
                    f"Gazebo-only diagnostic contract {contract_id} expected only "
                    f"{source_path}, found {publishers}"
                )
            elif (
                direction == "publisher"
                and contract.get("single_writer") is True
                and source_endpoints.count(endpoint) == 1
            ):
                validated_single_writer_contracts.add(contract_id)
            if gz_topic in bridge_contracts:
                errors.append(
                    f"Gazebo-only diagnostic contract {contract_id} must not be bridged to ROS: "
                    f"{gz_topic}"
                )
            matching_plugins = [
                plugin
                for plugin in root.findall(".//gazebo/plugin")
                if (plugin.get("name") or "").rsplit("::", 1)[-1] == writer_system
            ]
            if len(matching_plugins) != 1:
                errors.append(
                    f"Gazebo-only diagnostic contract {contract_id} expected exactly one URDF "
                    f"writer system {writer_system}, found {len(matching_plugins)}"
                )
            else:
                plugin = matching_plugins[0]
                expected_filename = f"lib{writer_system}.so"
                if plugin.get("filename") != expected_filename:
                    errors.append(
                        f"Gazebo-only diagnostic contract {contract_id} writer plugin filename "
                        f"must be {expected_filename}, found {plugin.get('filename')}"
                    )
                enabled = plugin.find("status_json_enabled")
                if enabled is None or (enabled.text or "").strip().lower() != "true":
                    errors.append(
                        f"Gazebo-only diagnostic contract {contract_id} status_json_enabled "
                        "must be true in the formal URDF"
                    )
            validated_topic_contracts.add(contract_id)
            validated_gazebo_only_diagnostics.add(contract_id)
            continue
        if not all(isinstance(value, str) and value for value in (topic, ros_type)):
            errors.append(f"topic contract {contract_id} requires ros_topic and ros_type")
            continue
        if transport == "ros_native":
            source_path = contract.get("source_path")
            if not isinstance(source_path, str) or not source_path:
                errors.append(
                    f"native topic contract {contract_id} requires source_path"
                )
                continue
            source = ROOT / source_path
            if not source.is_file():
                errors.append(
                    f"native topic contract {contract_id} source missing: {source_path}"
                )
                continue
            source_text = source.read_text(encoding="utf-8")
            message_class = ros_type.rsplit("/", 1)[-1]
            direction = contract.get("direction")
            if direction is not None and direction not in {"publisher", "subscription"}:
                errors.append(
                    f"native topic contract {contract_id} has invalid direction: {direction}"
                )
            expected_endpoint = (direction, topic, message_class)
            source_endpoints = endpoint_cache.get(source.resolve(), set())
            if direction is not None and expected_endpoint not in source_endpoints:
                errors.append(
                    f"native topic contract {contract_id} exact {direction} endpoint missing "
                    f"from {source_path}: {(topic, ros_type)}"
                )
            elif direction is None and (topic not in source_text or message_class not in source_text):
                errors.append(
                    f"native topic contract {contract_id} not implemented by "
                    f"{source_path}: {(topic, ros_type)}"
                )
            if contract.get("single_writer"):
                writer_node = contract.get("writer_node")
                if direction != "publisher":
                    errors.append(
                        f"single-writer topic contract {contract_id} must declare publisher direction"
                    )
                if not isinstance(writer_node, str) or not writer_node:
                    errors.append(
                        f"single-writer topic contract {contract_id} requires writer_node"
                    )
                publishers = sorted(
                    str(candidate.relative_to(ROOT)).replace("\\", "/")
                    for candidate, endpoints in endpoint_cache.items()
                    if ("publisher", topic, message_class) in endpoints
                )
                if publishers != [source_path]:
                    errors.append(
                        f"single-writer topic contract {contract_id} expected only "
                        f"{source_path}, found {publishers}"
                    )
                else:
                    validated_single_writer_contracts.add(contract_id)
            if gz_type is not None or sensor_base_topic is not None:
                errors.append(
                    f"native topic contract {contract_id} cannot declare Gazebo sensor fields"
                )
            validated_topic_contracts.add(contract_id)
            continue
        if transport == "gazebo_native_bridge":
            source_path = contract.get("source_path")
            package = contract.get("bridge_package")
            executable = contract.get("bridge_executable")
            writer_node = contract.get("writer_node")
            direction = contract.get("direction")
            if not all(
                isinstance(value, str) and value
                for value in (source_path, package, executable, writer_node)
            ):
                errors.append(
                    f"native Gazebo bridge contract {contract_id} requires source_path, "
                    "bridge_package, bridge_executable and writer_node"
                )
                continue
            source = ROOT / source_path
            if not source.is_file():
                errors.append(
                    f"native Gazebo bridge contract {contract_id} source missing: {source_path}"
                )
                continue
            expected_node = (package, executable, writer_node)
            node_instances = [node for node in launch_nodes if node == expected_node]
            node_remappings = [
                remappings
                for node, remappings in launch_node_remappings
                if node == expected_node
            ]
            if len(node_instances) != 1:
                errors.append(
                    f"native Gazebo bridge contract {contract_id} expected exactly one launch "
                    f"node {expected_node}, found {len(node_instances)}"
                )
            source_endpoints = cpp_bridge_endpoint_cache.get(source.resolve(), [])
            scoped_remappings = node_remappings[0] if len(node_remappings) == 1 else {}
            resolved_source_endpoints = [
                (scoped_remappings.get(local_topic, local_topic), local_ros_type, local_gz_type)
                for local_topic, local_ros_type, local_gz_type in source_endpoints
            ]
            endpoint = (topic, ros_type, gz_type)
            if direction != "publisher":
                errors.append(
                    f"native Gazebo bridge contract {contract_id} must declare publisher direction"
                )
            if resolved_source_endpoints.count(endpoint) != 1:
                errors.append(
                    f"native Gazebo bridge contract {contract_id} exact GZ->ROS endpoint "
                    f"missing or duplicated in {source_path}: {endpoint}"
                )
            publishers = resolved_native_publishers(endpoint)
            expected_publisher = (
                f"{source_path}::{package}/{executable}/{writer_node}"
            )
            if contract.get("single_writer"):
                if direction != "publisher":
                    errors.append(
                        f"single-writer topic contract {contract_id} must declare publisher direction"
                    )
                if publishers != [expected_publisher]:
                    errors.append(
                        f"single-writer topic contract {contract_id} expected only "
                        f"{source_path}, found {publishers}"
                    )
                elif len(node_instances) == 1 and resolved_source_endpoints.count(endpoint) == 1:
                    validated_single_writer_contracts.add(contract_id)
            validated_topic_contracts.add(contract_id)
            continue
        if transport != "gazebo_bridge":
            errors.append(
                f"topic contract {contract_id} has unsupported transport: {transport}"
            )
            continue
        if not isinstance(gz_type, str) or not gz_type:
            errors.append(
                f"Gazebo topic contract {contract_id} requires gz_type"
            )
            continue
        actual_types = bridge_contracts.get(topic)
        if actual_types is None:
            errors.append(f"topic contract {contract_id} missing exact bridge: {topic}")
        elif actual_types != (ros_type, gz_type):
            errors.append(
                f"topic contract {contract_id} type mismatch for {topic}: "
                f"launch={actual_types}, register={(ros_type, gz_type)}"
            )
        if sensor_base_topic is not None and sensor_base_topic not in urdf_sensor_topics:
            errors.append(
                f"topic contract {contract_id} sensor_base_topic missing from URDF sensors: {sensor_base_topic}"
            )
        validated_topic_contracts.add(contract_id)

    def validate_topic_references(contract_ids: object, label: str) -> None:
        if not isinstance(contract_ids, list) or not contract_ids:
            errors.append(f"{label} requires at least one topic_contract")
            return
        for contract_id in contract_ids:
            if contract_id not in topic_contracts:
                errors.append(f"{label} references unknown topic contract: {contract_id}")
            elif contract_id in validated_gazebo_only_diagnostics:
                errors.append(
                    f"{label} references diagnostic-only topic contract: {contract_id}"
                )

    manager_types = controllers.get("controller_manager", {}).get("ros__parameters", {})
    physical_interfaces = register.get("physical_actuation_interfaces", {})

    def controller_joints(name: str) -> set[str]:
        params = controllers.get(name, {}).get("ros__parameters", {})
        values: list[str] = []
        for key in ("joints", "left_wheel_names", "right_wheel_names"):
            values.extend(params.get(key, []) or [])
        return set(values)

    def interface_joints(name: str) -> set[str]:
        if name in manager_types:
            return controller_joints(name)
        interface = physical_interfaces.get(name, {})
        return set(interface.get("driven_joints", []))

    for interface_name, interface in physical_interfaces.items():
        source_relative = interface.get("source_path")
        source = ROOT / str(source_relative)
        if not source.is_file():
            errors.append(
                f"physical actuation interface {interface_name} source missing: "
                f"{source_relative}"
            )
            continue
        source_text = source.read_text(encoding="utf-8")
        for token in (
            interface.get("authority"),
            interface.get("physical_command_component"),
        ):
            if not isinstance(token, str) or token.rsplit(".", 1)[-1] not in source_text:
                errors.append(
                    f"physical actuation interface {interface_name} source lacks {token}"
                )
        if interface.get("publishes_tf") is not False:
            errors.append(
                f"physical actuation interface {interface_name} must not publish TF"
            )

    checked_sensors: list[str] = []
    required_camera_contracts = {
        "intel_d435_front": {
            "front_rgbd_rgb",
            "front_rgbd_depth",
            "front_rgbd_camera_info",
            "front_rgbd_ir_left",
            "front_rgbd_ir_left_camera_info",
            "front_rgbd_ir_right",
            "front_rgbd_ir_right_camera_info",
        },
        "intel_d435_wrist": {
            "wrist_rgbd_rgb",
            "wrist_rgbd_depth",
            "wrist_rgbd_camera_info",
            "wrist_rgbd_ir_left",
            "wrist_rgbd_ir_left_camera_info",
            "wrist_rgbd_ir_right",
            "wrist_rgbd_ir_right_camera_info",
        },
        "rear_left_fisheye": {"rear_left_image", "rear_left_camera_info"},
        "rear_right_fisheye": {"rear_right_image", "rear_right_camera_info"},
    }
    for sensor in register.get("sensor_installations", []):
        sensor_id = sensor["id"]
        parent = sensor["parent_link"]
        mount = sensor["mount_link"]
        child = sensor["sensor_link"]
        for field, name in (("parent_link", parent), ("mount_link", mount), ("sensor_link", child)):
            if name not in links:
                errors.append(f"{sensor_id}.{field} missing from URDF: {name}")
        if mount in links and parent in links and not is_descendant(mount, parent):
            errors.append(f"{sensor_id} mount load path does not descend from {parent}: {mount}")
        if child in links and mount in links and not is_descendant(child, mount):
            errors.append(f"{sensor_id} sensor load path does not descend from {mount}: {child}")
        for field, name in (("mount_link", mount), ("sensor_link", child)):
            if name in links:
                element = link_elements[name]
                if not element.findall("visual") or not element.findall("collision"):
                    errors.append(
                        f"{sensor_id}.{field} must have visible and collidable physical geometry"
                    )
        for child_name, expected_parent, label in (
            (mount, parent, "mount"),
            (child, mount, "sensor"),
        ):
            joint = joint_by_child.get(child_name)
            actual_parent = (
                joint.find("parent").get("link") if joint is not None else None
            )
            if actual_parent != expected_parent:
                errors.append(
                    f"{sensor_id} {label} joint must directly connect "
                    f"{expected_parent}->{child_name}, found {actual_parent}->{child_name}"
                )
        if not sensor.get("connection"):
            errors.append(f"{sensor_id} has no mechanical connection description")
        validate_topic_references(sensor.get("topic_contracts"), sensor_id)
        missing_camera_contracts = required_camera_contracts.get(sensor_id, set()) - set(
            sensor.get("topic_contracts", [])
        )
        if missing_camera_contracts:
            errors.append(
                f"{sensor_id} is missing required RGB/depth/CameraInfo contracts: "
                f"{sorted(missing_camera_contracts)}"
            )
        if sensor_id == "intel_d435_wrist":
            if mount != "wrist_rgbd_mount_link" or child != "wrist_rgbd_link":
                errors.append(
                    "wrist D435 must use distinct bracket and camera housing links"
                )
            bracket_joint = joints.get("wrist_rgbd_bracket_joint")
            camera_joint = joints.get("wrist_rgbd_mount_joint")
            if bracket_joint is None or (
                bracket_joint.find("parent").get("link"),
                bracket_joint.find("child").get("link"),
            ) != ("tool0", "wrist_rgbd_mount_link"):
                errors.append("wrist RGBD bracket must attach directly to tool0")
            if camera_joint is None or (
                camera_joint.find("parent").get("link"),
                camera_joint.find("child").get("link"),
            ) != ("wrist_rgbd_mount_link", "wrist_rgbd_link"):
                errors.append("wrist D435 housing must attach directly to its bracket")
        validate_pose(sensor, sensor_id, child)
        checked_sensors.append(sensor_id)

    checked_subassemblies: list[str] = []
    checked_actuator_links: set[str] = set()
    for assembly in register.get("mechanical_subassemblies", []):
        assembly_id = assembly["id"]
        parent = assembly["parent_link"]
        roots = assembly.get("root_links", [assembly.get("root_link")])
        if parent not in links:
            errors.append(f"{assembly_id}.parent_link missing from URDF: {parent}")
        for child in roots:
            if child not in links:
                errors.append(f"{assembly_id}.root_link missing from URDF: {child}")
            elif parent in links and not is_descendant(child, parent):
                errors.append(f"{assembly_id} load path does not descend from {parent}: {child}")
        if not assembly.get("connection"):
            errors.append(f"{assembly_id} has no mechanical connection description")
        expected_assembly_roots = {
            "dry_storage": ("storage_system_mount_link", {"dry_bin_link"}),
            "wastewater_storage": (
                "storage_system_mount_link",
                {"wastewater_tank_link"},
            ),
            "wrist_rgbd_installation": ("tool0", {"wrist_rgbd_mount_link"}),
            "bodywork_service_access": (
                "base_link",
                {
                    "bodywork_power_service_door_hinge_bracket_link",
                    "bodywork_compute_service_door_hinge_bracket_link",
                    "bodywork_wet_service_door_hinge_bracket_link",
                    "bodywork_rear_dry_service_door_hinge_bracket_link",
                },
            ),
        }
        if assembly_id in expected_assembly_roots:
            expected_parent, expected_roots = expected_assembly_roots[assembly_id]
            if parent != expected_parent or set(roots) != expected_roots:
                errors.append(
                    f"{assembly_id} must match the direct URDF parent/root chain"
                )
        for joint_name in assembly.get("driven_joints", []):
            if joint_name not in joints:
                errors.append(f"{assembly_id}.driven_joint missing from URDF: {joint_name}")
        passive_joints = set(assembly.get("passive_joints", []))
        driven_joints = set(assembly.get("driven_joints", []))
        for joint_name in passive_joints:
            if joint_name not in joints:
                errors.append(f"{assembly_id}.passive_joint missing from URDF: {joint_name}")
        if passive_joints & driven_joints:
            errors.append(
                f"{assembly_id} cannot classify one joint as both driven and passive: "
                f"{sorted(passive_joints & driven_joints)}"
            )
        actuator_links = list(assembly.get("actuator_links", []))
        if assembly.get("actuator_link"):
            actuator_links.append(assembly["actuator_link"])
        for actuator_link in actuator_links:
            if actuator_link not in links:
                errors.append(
                    f"{assembly_id}.actuator_link missing from URDF: {actuator_link}"
                )
            elif parent in links and not is_descendant(actuator_link, parent):
                errors.append(
                    f"{assembly_id} actuator load path does not descend from "
                    f"{parent}: {actuator_link}"
                )
            elif (
                not link_elements[actuator_link].findall("visual")
                or not link_elements[actuator_link].findall("collision")
            ):
                errors.append(
                    f"{assembly_id}.actuator_link must have visible and collidable physical geometry: "
                    f"{actuator_link}"
                )
            checked_actuator_links.add(actuator_link)
        checked_subassemblies.append(assembly_id)

    checked_positions: list[str] = []
    position_ids: set[str] = set()
    required_camera_position_contracts = {
        "forward_perception": required_camera_contracts["intel_d435_front"],
        "rear_left_perception": required_camera_contracts["rear_left_fisheye"],
        "rear_right_perception": required_camera_contracts["rear_right_fisheye"],
        "grasp_observation": required_camera_contracts["intel_d435_wrist"],
    }
    required_datum_physical_links = {
        "warning_and_work_lighting": ("bodywork_lighting_link", None),
        "front_contact_safety": ("bodywork_lower_tub_link", "front_bumper_collision"),
        "rear_contact_safety": ("bodywork_lower_tub_link", "rear_bumper_collision"),
    }
    for position in register.get("functional_positions", []):
        position_id = position["id"]
        if position_id in position_ids:
            errors.append(f"duplicate functional position id: {position_id}")
        position_ids.add(position_id)
        link = position["link"]
        parent = position["parent_link"]
        if link not in links:
            errors.append(f"{position_id}.link missing from URDF: {link}")
        if parent not in links:
            errors.append(f"{position_id}.parent_link missing from URDF: {parent}")
        if link in links and parent in links and not is_descendant(link, parent):
            errors.append(f"{position_id} load path does not descend from {parent}: {link}")
        if position_id in required_datum_physical_links:
            expected_physical_link, expected_collision = required_datum_physical_links[position_id]
            physical_link = position.get("physical_link")
            if physical_link != expected_physical_link:
                errors.append(
                    f"{position_id}.physical_link must explicitly map its datum to {expected_physical_link}"
                )
            elif physical_link not in links or not is_descendant(physical_link, parent):
                errors.append(f"{position_id}.physical_link has no valid URDF load path")
            else:
                physical_element = link_elements[physical_link]
                if not physical_element.findall("visual") or not physical_element.findall("collision"):
                    errors.append(
                        f"{position_id}.physical_link must have visible and collidable geometry"
                    )
                if expected_collision is not None:
                    collision_names = {
                        collision.get("name") for collision in physical_element.findall("collision")
                    }
                    if position.get("physical_collision") != expected_collision or expected_collision not in collision_names:
                        errors.append(
                            f"{position_id}.physical_collision must bind {expected_collision}"
                        )
        physical_position_link = position.get("physical_link", link)
        if physical_position_link in links:
            physical_element = link_elements[physical_position_link]
            if not physical_element.findall("visual") or not physical_element.findall("collision"):
                errors.append(
                    f"{position_id} physical link must have visible and collidable geometry: "
                    f"{physical_position_link}"
                )
        if not position.get("function"):
            errors.append(f"{position_id} has no function definition")
        validate_pose(position, position_id, link)
        required_joints = position.get("required_joints", [])
        for joint_name in required_joints:
            if joint_name not in joints:
                errors.append(f"{position_id}.required_joint missing from URDF: {joint_name}")
        required_passive_joints = position.get("required_passive_joints", [])
        for joint_name in required_passive_joints:
            if joint_name not in joints:
                errors.append(
                    f"{position_id}.required_passive_joint missing from URDF: {joint_name}"
                )
        passive_actuation_types = {
            "passive_manual_latch",
            "passive_spring_damper",
            "passive_service_interlock",
        }
        passive_actuation = (
            position.get("passive_actuation")
            if required_joints
            else position.get("actuation")
        )
        if required_passive_joints and passive_actuation not in passive_actuation_types:
            errors.append(
                f"{position_id} passive joints require an explicit passive actuation type"
            )
        interface = position.get("interface")
        if required_joints and not interface:
            errors.append(f"{position_id} has actuators but no controller interface")
        if interface:
            if interface not in manager_types and interface not in physical_interfaces:
                errors.append(
                    f"{position_id}.interface not declared by controller manager "
                    f"or physical actuation registry: {interface}"
                )
            missing = set(required_joints) - interface_joints(interface)
            if missing:
                errors.append(f"{position_id}.interface {interface} does not command {sorted(missing)}")
        if position_id == "wastewater_drain":
            if set(required_joints) != {"wastewater_drain_valve_joint"}:
                errors.append(
                    "wastewater_drain must register wastewater_drain_valve_joint as powered"
                )
            if "wastewater_drain_valve_joint" in required_passive_joints:
                errors.append(
                    "wastewater_drain_valve_joint cannot be registered as passive"
                )
            if interface != "service_controller":
                errors.append(
                    "wastewater_drain powered valve must use service_controller"
                )
            if position.get("actuation") != "powered_position_joint":
                errors.append(
                    "wastewater_drain must declare powered_position_joint actuation"
                )
            if position.get("actuator_link") != "wastewater_drain_valve_actuator_link":
                errors.append(
                    "wastewater_drain must register its physical actuator link"
                )
        if position_id == "bodywork_service_access":
            expected_hinges = {
                "bodywork_power_service_door_hinge_joint": (0.0, 1.745329252),
                "bodywork_compute_service_door_hinge_joint": (-1.745329252, 0.0),
                "bodywork_wet_service_door_hinge_joint": (-1.745329252, 0.0),
                "bodywork_rear_dry_service_door_hinge_joint": (-1.745329252, 0.0),
            }
            expected_latches = {
                "bodywork_power_service_door_latch_joint",
                "bodywork_compute_service_door_latch_joint",
                "bodywork_wet_service_door_latch_joint",
                "bodywork_rear_dry_service_door_latch_joint",
            }
            if set(required_passive_joints) != set(expected_hinges) | expected_latches:
                errors.append(
                    "bodywork service access must register four hinges and four latches"
                )
            if position.get("lock_state") != "all_latches_zero_rad":
                errors.append("bodywork service access must define the locked zero state")
            for joint_name, expected_limits in expected_hinges.items():
                joint = joints.get(joint_name)
                limit = joint.find("limit") if joint is not None else None
                axis = joint.find("axis") if joint is not None else None
                if (
                    joint is None
                    or joint.get("type") != "revolute"
                    or axis is None
                    or axis.get("xyz") != "0 0 1"
                    or limit is None
                    or not math.isclose(
                        float(limit.get("lower", "nan")), expected_limits[0], abs_tol=1e-9
                    )
                    or not math.isclose(
                        float(limit.get("upper", "nan")), expected_limits[1], abs_tol=1e-9
                    )
                ):
                    errors.append(
                        f"{joint_name} must be a vertically hinged, mechanically limited door"
                    )
            ros2_control_joints = {
                item.get("name"): item for item in root.findall(".//ros2_control/joint")
            }
            for joint_name in set(expected_hinges) | expected_latches:
                joint = ros2_control_joints.get(joint_name)
                if (
                    joint is None
                    or joint.find("command_interface") is not None
                    or joint.find("state_interface[@name='position']") is None
                ):
                    errors.append(
                        f"{joint_name} must be passive state-only service hardware"
                    )
        if position_id == "dry_deposition":
            if position.get("presence_sensor_link") != "dry_deposit_presence_sensor_link":
                errors.append(
                    "dry_deposition must register its physical passage-presence sensor link"
                )
            if position.get("presence_measurement") != "physical_chute_contact_event":
                errors.append(
                    "dry_deposition must define the chute-contact presence measurement"
                )
        if position_id in {"dry_storage", "wet_storage"}:
            expected_service_joints = {
                "dry_storage": {"dry_bin_lid_joint", "dry_bin_latch_joint"},
                "wet_storage": {"wastewater_lid_joint", "wastewater_lid_latch_joint"},
            }[position_id]
            if set(required_passive_joints) != expected_service_joints:
                errors.append(
                    f"{position_id} must register its lid hinge and manual latch joint"
                )
            if position.get("lock_state") != "lid_zero_rad_and_latch_zero_rad":
                errors.append(f"{position_id} must define the closed zero-radian lock state")
            if position.get("service_sequence") != "rotate_latch_to_released_stop_then_open_lid":
                errors.append(f"{position_id} must define latch-before-lid service sequencing")
            ros2_control_joints = {
                item.get("name"): item for item in root.findall(".//ros2_control/joint")
            }
            for joint_name in expected_service_joints:
                joint = joints.get(joint_name)
                limit = joint.find("limit") if joint is not None else None
                axis = joint.find("axis") if joint is not None else None
                expected_upper = (
                    1.221730476 if joint_name.endswith("latch_joint") else 1.570796
                )
                if (
                    joint is None
                    or joint.get("type") != "revolute"
                    or axis is None
                    or axis.get("xyz") != "0 1 0"
                    or limit is None
                    or not math.isclose(float(limit.get("lower", "nan")), 0.0, abs_tol=1e-9)
                    or not math.isclose(
                        float(limit.get("upper", "nan")), expected_upper, abs_tol=1e-9
                    )
                ):
                    errors.append(
                        f"{joint_name} must be a mechanically limited manual service joint"
                    )
                control_joint = ros2_control_joints.get(joint_name)
                if (
                    control_joint is None
                    or control_joint.find("command_interface") is not None
                    or control_joint.find("state_interface[@name='position']") is None
                ):
                    errors.append(
                        f"{joint_name} must be passive state-only service hardware"
                    )
        required_topic_contracts = position.get("required_topic_contracts")
        if required_topic_contracts is not None:
            validate_topic_references(required_topic_contracts, position_id)
        if (
            position_id == "dry_deposition"
            and "dry_deposit_contact" not in (required_topic_contracts or [])
        ):
            errors.append(
                "dry_deposition presence sensor must require dry_deposit_contact"
            )
        missing_camera_contracts = required_camera_position_contracts.get(
            position_id, set()
        ) - set(required_topic_contracts or [])
        if missing_camera_contracts:
            errors.append(
                f"{position_id} is missing required RGB/depth/CameraInfo contracts: "
                f"{sorted(missing_camera_contracts)}"
            )
        if "visible" not in position:
            errors.append(f"{position_id} does not declare product visibility")
        actuator_links = list(position.get("actuator_links", []))
        if position.get("actuator_link"):
            actuator_links.append(position["actuator_link"])
        for actuator_link in actuator_links:
            if actuator_link not in links:
                errors.append(
                    f"{position_id}.actuator_link missing from URDF: {actuator_link}"
                )
            elif parent in links and not is_descendant(actuator_link, parent):
                errors.append(
                    f"{position_id} actuator load path does not descend from "
                    f"{parent}: {actuator_link}"
                )
            elif (
                not link_elements[actuator_link].findall("visual")
                or not link_elements[actuator_link].findall("collision")
            ):
                errors.append(
                    f"{position_id}.actuator_link must have visible and collidable physical geometry: "
                    f"{actuator_link}"
                )
            checked_actuator_links.add(actuator_link)
        checked_positions.append(position_id)

    checked_components: list[str] = []
    component_ids: set[str] = set()
    for position in register.get("functional_positions", []):
        position_id = position["id"]
        for component in position.get("components", []):
            component_id = component.get("id")
            label = f"{position_id}.components.{component_id}"
            if not isinstance(component_id, str) or not component_id:
                errors.append(f"{position_id} component requires a non-empty id")
                continue
            if component_id in component_ids:
                errors.append(f"duplicate functional component id: {component_id}")
            component_ids.add(component_id)
            link = component.get("link")
            parent = component.get("parent_link")
            joint_name = component.get("joint")
            if link not in links:
                errors.append(f"{label}.link missing from URDF: {link}")
            if parent not in links:
                errors.append(f"{label}.parent_link missing from URDF: {parent}")
            if link in links and parent in links and not is_descendant(link, parent):
                errors.append(
                    f"{label} load path does not descend from {parent}: {link}"
                )
            if joint_name not in joints:
                errors.append(f"{label}.joint missing from URDF: {joint_name}")
            else:
                joint = joints[joint_name]
                actual_parent = joint.find("parent").get("link")
                actual_child = joint.find("child").get("link")
                if (actual_parent, actual_child) != (parent, link):
                    errors.append(
                        f"{label}.joint endpoints mismatch: "
                        f"{(actual_parent, actual_child)} != {(parent, link)}"
                    )
            if "visible" not in component:
                errors.append(f"{label} does not declare product visibility")
            if link in links and "optical_datum" not in str(component_id):
                element = link_elements[link]
                if not element.findall("visual") or not element.findall("collision"):
                    errors.append(
                        f"{label} must have visible and collidable physical geometry"
                    )
            validate_pose(component, label, link)
            checked_components.append(component_id)

    missing_explicit_components = sorted(
        REQUIRED_EXPLICIT_FUNCTIONAL_COMPONENTS - component_ids
    )
    if missing_explicit_components:
        errors.append(
            "required explicit functional components are missing: "
            f"{missing_explicit_components}"
        )

    semantics = register.get("navigation_sensor_semantics", {})
    mapping_semantics = semantics.get("mapping", {})
    obstacle_semantics = semantics.get("obstacle_perception", {})
    expected_mapping = {
        "dimensionality": "2d_occupancy_grid",
        "primary_sensor_installation": "hokuyo_utm30lx",
        "topic_contract": "lidar_2d_scan",
        "product_pipeline": "slam_toolbox_utm_scan",
    }
    expected_obstacle = {
        "dimensionality": "3d_pointcloud",
        "primary_sensor_installation": "livox_mid360",
        "topic_contract": "lidar_3d_pointcloud",
        "product_pipeline": "mid360_obstacle_layer",
        "mapping_claim": "prohibited",
    }
    if mapping_semantics != expected_mapping:
        errors.append("navigation mapping semantics must declare UTM 2D occupancy mapping")
    if obstacle_semantics != expected_obstacle:
        errors.append("MID360 semantics must be 3D obstacle perception with mapping prohibited")
    sensors_by_id = {
        sensor.get("id"): sensor for sensor in register.get("sensor_installations", [])
    }
    positions_by_id = {
        position.get("id"): position
        for position in register.get("functional_positions", [])
    }
    if sensors_by_id.get("hokuyo_utm30lx", {}).get("role") != (
        "2d_occupancy_mapping_localization_and_obstacle_lidar"
    ):
        errors.append("UTM role must explicitly own 2D occupancy mapping")
    if sensors_by_id.get("livox_mid360", {}).get("role") != (
        "3d_obstacle_perception_lidar"
    ):
        errors.append("MID360 role cannot claim mapping")
    if positions_by_id.get("mapping_2d", {}).get("function") != (
        "utm30lx_2d_occupancy_mapping"
    ):
        errors.append("mapping_2d functional position must use UTM-30LX")
    if positions_by_id.get("obstacle_perception_3d", {}).get("function") != (
        "mid360_3d_obstacle_perception"
    ):
        errors.append("obstacle_perception_3d must be MID360 perception only")
    if "mapping_3d" in positions_by_id:
        errors.append("mapping_3d is prohibited by the formal navigation semantics")
    emergency_stop = positions_by_id.get("emergency_stop", {})
    emergency_components = {
        component.get("id"): component
        for component in emergency_stop.get("components", [])
    }
    plunger = emergency_components.get("emergency_stop_6mm_plunger", {})
    plunger_joint = joints.get("emergency_stop_plunger_joint")
    plunger_limit = plunger_joint.find("limit") if plunger_joint is not None else None
    if (
        emergency_stop.get("plunger_travel_m") != 0.006
        or plunger.get("travel_m") != 0.006
        or plunger_limit is None
        or float(plunger_limit.get("upper", "nan")) != 0.006
    ):
        errors.append("emergency stop plunger must preserve the explicit 0.006 m travel")

    if errors:
        raise ComponentRegisterError("; ".join(errors))
    return {
        "register_id": register["register_id"],
        "status": "COMPONENT_REGISTER_URDF_FK_AND_INTERFACES_VALID",
        "urdf_sha256": hashlib.sha256(urdf_path.read_bytes()).hexdigest(),
        "coordinate_reference": default_reference,
        "position_tolerance_m": position_tolerance,
        "orientation_tolerance_rad": orientation_tolerance,
        "sensor_installation_count": len(checked_sensors),
        "mechanical_subassembly_count": len(checked_subassemblies),
        "actuator_link_count": len(checked_actuator_links),
        "functional_position_count": len(checked_positions),
        "functional_component_count": len(checked_components),
        "topic_contract_count": len(validated_topic_contracts),
        "product_topic_contract_count": len(
            validated_topic_contracts - validated_gazebo_only_diagnostics
        ),
        "gazebo_only_diagnostic_count": len(validated_gazebo_only_diagnostics),
        "single_writer_topic_count": len(validated_single_writer_contracts),
        "checked_sensor_installations": checked_sensors,
        "checked_mechanical_subassemblies": checked_subassemblies,
        "checked_actuator_links": sorted(checked_actuator_links),
        "checked_functional_positions": checked_positions,
        "checked_functional_components": checked_components,
        "checked_topic_contracts": sorted(validated_topic_contracts),
        "checked_product_topic_contracts": sorted(
            validated_topic_contracts - validated_gazebo_only_diagnostics
        ),
        "checked_gazebo_only_diagnostics": sorted(validated_gazebo_only_diagnostics),
        "checked_single_writer_topics": sorted(validated_single_writer_contracts),
        "top_protrusion_name": register["external_identity"]["top_protrusion_name"],
        "claim_boundary": register["claim_boundary"],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--register", type=Path, default=DEFAULT_REGISTER)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--controllers", type=Path, default=DEFAULT_CONTROLLERS)
    parser.add_argument("--launch", type=Path, default=DEFAULT_LAUNCH)
    parser.add_argument("--bridge-config", type=Path, action="append")
    parser.add_argument("--write-report", type=Path)
    args = parser.parse_args()
    result = validate(
        args.register,
        args.urdf,
        args.controllers,
        args.launch,
        args.bridge_config,
    )
    if args.write_report:
        args.write_report.parent.mkdir(parents=True, exist_ok=True)
        args.write_report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
