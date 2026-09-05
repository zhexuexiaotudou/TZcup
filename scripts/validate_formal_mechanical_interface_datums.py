#!/usr/bin/env python3
"""Fail-closed static audit for the formal vehicle mechanical datum crosswalk.

The audit intentionally works on an already-expanded URDF at zero joint
position.  It does not invoke ROS, Gazebo, Docker, WSL, CAD, or any physical
hardware tool.  A passing result is therefore not a manufacturing release.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Iterable

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CROSSWALK = (
    ROOT / "config" / "high_fidelity_vehicle" / "formal_mechanical_interface_datums.yaml"
)


class MechanicalDatumValidationError(ValueError):
    """Raised when the snapshot-bound static mechanical crosswalk is invalid."""


Matrix = tuple[tuple[float, float, float, float], ...]


def _load_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise MechanicalDatumValidationError(f"YAML root must be a mapping: {path}")
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _identity() -> Matrix:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _multiply(left: Matrix, right: Matrix) -> Matrix:
    return tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(4))
            for column in range(4)
        )
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
    x, y, z = (float(value) for value in xyz)
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y),
        (-sp, cp * sr, cp * cr, z),
        (0.0, 0.0, 0.0, 1.0),
    )


def _numbers(raw: str | None) -> tuple[float, float, float]:
    values = tuple(float(item) for item in (raw or "0 0 0").split())
    if len(values) != 3:
        raise MechanicalDatumValidationError(f"URDF pose must have three values: {raw!r}")
    return values


def _joint_origin(joint: ET.Element) -> Matrix:
    origin = joint.find("origin")
    if origin is None:
        return _identity()
    return _transform(_numbers(origin.get("xyz")), _numbers(origin.get("rpy")))


def _world_transforms(root: ET.Element) -> tuple[str, dict[str, ET.Element], dict[str, Matrix]]:
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    if None in joints:
        raise MechanicalDatumValidationError("expanded URDF has unnamed joint")
    child_links = {joint.find("child").get("link") for joint in joints.values()}
    roots = [link.get("name") for link in root.findall("link") if link.get("name") not in child_links]
    if len(roots) != 1 or not roots[0]:
        raise MechanicalDatumValidationError(f"expanded URDF must have one FK root, got {roots}")
    poses = {roots[0]: _identity()}
    pending = list(joints.values())
    while pending:
        remaining: list[ET.Element] = []
        progressed = False
        for joint in pending:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in poses:
                remaining.append(joint)
                continue
            poses[child] = _multiply(poses[parent], _joint_origin(joint))
            progressed = True
        if not progressed:
            unresolved = sorted(joint.get("name") for joint in remaining)
            raise MechanicalDatumValidationError(f"URDF FK cannot resolve joints: {unresolved}")
        pending = remaining
    return roots[0], joints, poses


def _rotation_error_rad(expected: Matrix, actual: Matrix) -> float:
    relative = _multiply(_inverse_rigid(expected), actual)
    cosine = max(-1.0, min(1.0, (sum(relative[index][index] for index in range(3)) - 1.0) / 2.0))
    return math.acos(cosine)


def _pose_from_mapping(value: dict[str, Any], label: str) -> Matrix:
    xyz = value.get("xyz_m")
    rpy = value.get("rpy_rad")
    if not isinstance(xyz, list) or len(xyz) != 3 or not isinstance(rpy, list) or len(rpy) != 3:
        raise MechanicalDatumValidationError(f"{label} must contain xyz_m and rpy_rad vectors")
    return _transform(xyz, rpy)


def _translation_error_m(expected: Matrix, actual: Matrix) -> float:
    return math.sqrt(sum((expected[index][3] - actual[index][3]) ** 2 for index in range(3)))


def _by_id(rows: Any, identifier: str, label: str) -> dict[str, Any]:
    if not isinstance(rows, list):
        raise MechanicalDatumValidationError(f"{label} must be a list")
    matches = [row for row in rows if isinstance(row, dict) and row.get("id") == identifier]
    if len(matches) != 1:
        raise MechanicalDatumValidationError(f"{label} must contain exactly one {identifier!r}")
    return matches[0]


def _assert_expected(actual: dict[str, Any], expected: dict[str, Any], label: str) -> None:
    if not isinstance(expected, dict) or not expected:
        raise MechanicalDatumValidationError(f"{label} expected binding must be a non-empty mapping")
    for key, value in expected.items():
        if key.endswith("_contains"):
            source_key = key.removesuffix("_contains")
            collection = actual.get(source_key)
            if not isinstance(collection, list) or value not in collection:
                raise MechanicalDatumValidationError(f"{label}.{source_key} must contain {value!r}")
        elif actual.get(key) != value:
            raise MechanicalDatumValidationError(
                f"{label}.{key} expected {value!r}, got {actual.get(key)!r}"
            )


def _functional_component(position: dict[str, Any], component_id: str) -> dict[str, Any]:
    return _by_id(position.get("components"), component_id, f"functional position {position.get('id')} components")


def _validate_snapshot(crosswalk: dict[str, Any], root: Path) -> dict[str, str]:
    snapshot = crosswalk.get("source_snapshot")
    if not isinstance(snapshot, dict) or snapshot.get("algorithm") != "sha256":
        raise MechanicalDatumValidationError("source_snapshot must use sha256")
    inputs = snapshot.get("inputs")
    if not isinstance(inputs, list) or len(inputs) != 3:
        raise MechanicalDatumValidationError("source_snapshot must contain exactly layout/register/URDF inputs")
    actual_hashes: dict[str, str] = {}
    paths: set[str] = set()
    for item in inputs:
        if not isinstance(item, dict):
            raise MechanicalDatumValidationError("source_snapshot inputs must be mappings")
        path_text = item.get("path")
        expected = item.get("sha256")
        if not isinstance(path_text, str) or not path_text or Path(path_text).is_absolute():
            raise MechanicalDatumValidationError("source snapshot path must be a non-empty relative path")
        if not isinstance(expected, str) or len(expected) != 64 or any(char not in "0123456789abcdef" for char in expected.lower()):
            raise MechanicalDatumValidationError(f"source snapshot digest is invalid: {path_text}")
        if path_text in paths:
            raise MechanicalDatumValidationError(f"source snapshot path is duplicated: {path_text}")
        paths.add(path_text)
        actual = _sha256(root / path_text)
        actual_hashes[path_text] = actual
        if actual != expected.lower():
            raise MechanicalDatumValidationError(
                f"snapshot hash mismatch for {path_text}: expected {expected.lower()}, got {actual}"
            )
    required = {
        "config/high_fidelity_vehicle/formal_vehicle_layout.yaml",
        "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml",
        "reports/engineering/formal_competition_vehicle.urdf",
    }
    if paths != required:
        raise MechanicalDatumValidationError(f"source snapshot paths must be {sorted(required)}")
    return actual_hashes


def validate(
    crosswalk_path: Path = DEFAULT_CROSSWALK,
    *,
    root: Path = ROOT,
) -> dict[str, Any]:
    """Validate the current source snapshot and every declared datum/interface."""
    root = root.resolve()
    crosswalk = _load_yaml(crosswalk_path)
    if crosswalk.get("schema_version") != 1:
        raise MechanicalDatumValidationError("unsupported crosswalk schema_version")
    if crosswalk.get("status") != "STATIC_DERIVED_SNAPSHOT_BOUND_NOT_MANUFACTURING_RELEASE":
        raise MechanicalDatumValidationError("crosswalk must retain the static non-release status")
    release = crosswalk.get("manufacturing_release")
    if not isinstance(release, dict) or release.get("released") is not False or release.get("status") != "NOT_A_MANUFACTURING_RELEASE":
        raise MechanicalDatumValidationError("crosswalk must explicitly remain outside manufacturing release")
    hashes = _validate_snapshot(crosswalk, root)

    coordinate = crosswalk.get("coordinate_contract")
    if not isinstance(coordinate, dict) or coordinate.get("nominal_joint_configuration") != "zero":
        raise MechanicalDatumValidationError("crosswalk must use zero joint configuration")
    root_frame = coordinate.get("root_frame")
    position_tolerance = coordinate.get("static_position_tolerance_m")
    orientation_tolerance = coordinate.get("static_orientation_tolerance_rad")
    if not isinstance(root_frame, str) or not root_frame:
        raise MechanicalDatumValidationError("crosswalk root_frame is required")
    if not isinstance(position_tolerance, (int, float)) or position_tolerance <= 0:
        raise MechanicalDatumValidationError("static_position_tolerance_m must be positive")
    if not isinstance(orientation_tolerance, (int, float)) or orientation_tolerance <= 0:
        raise MechanicalDatumValidationError("static_orientation_tolerance_rad must be positive")

    layout = _load_yaml(root / "config" / "high_fidelity_vehicle" / "formal_vehicle_layout.yaml")
    register = _load_yaml(root / "config" / "high_fidelity_vehicle" / "formal_vehicle_component_register.yaml")
    if layout.get("robot", {}).get("root_frame") != root_frame:
        raise MechanicalDatumValidationError("layout root frame differs from crosswalk root frame")
    if register.get("coordinate_contract", {}).get("default_reference") != root_frame:
        raise MechanicalDatumValidationError("component register reference differs from crosswalk root frame")
    if abs(float(register["coordinate_contract"]["position_tolerance_m"]) - float(position_tolerance)) > 1e-12:
        raise MechanicalDatumValidationError("crosswalk position tolerance differs from component register")
    if abs(float(register["coordinate_contract"]["orientation_tolerance_rad"]) - float(orientation_tolerance)) > 1e-12:
        raise MechanicalDatumValidationError("crosswalk orientation tolerance differs from component register")

    urdf_root = ET.parse(root / "reports" / "engineering" / "formal_competition_vehicle.urdf").getroot()
    observed_root, joints, poses = _world_transforms(urdf_root)
    if observed_root != root_frame:
        raise MechanicalDatumValidationError(f"URDF root {observed_root!r} differs from {root_frame!r}")

    datums = crosswalk.get("datum_catalog")
    if not isinstance(datums, list) or not datums:
        raise MechanicalDatumValidationError("datum_catalog must be a non-empty list")
    datum_ids: set[str] = set()
    datum_links: set[str] = set()
    positions = register.get("functional_positions")
    observed_datums: dict[str, dict[str, Any]] = {}
    for datum in datums:
        if not isinstance(datum, dict):
            raise MechanicalDatumValidationError("datum catalog entries must be mappings")
        datum_id = datum.get("datum_id")
        link = datum.get("link")
        if not isinstance(datum_id, str) or not datum_id or datum_id in datum_ids:
            raise MechanicalDatumValidationError(f"datum id must be unique: {datum_id!r}")
        if not isinstance(link, str) or not link or link in datum_links:
            raise MechanicalDatumValidationError(f"datum link must be unique: {link!r}")
        datum_ids.add(datum_id)
        datum_links.add(link)
        if link not in poses:
            raise MechanicalDatumValidationError(f"datum {datum_id} link is absent from URDF: {link}")
        expected_pose = _pose_from_mapping(datum.get("expected_root_pose", {}), f"datum {datum_id}")
        actual_pose = poses[link]
        position_error = _translation_error_m(expected_pose, actual_pose)
        orientation_error = _rotation_error_rad(expected_pose, actual_pose)
        if position_error > float(position_tolerance):
            raise MechanicalDatumValidationError(f"datum {datum_id} FK position error {position_error:.9g} m")
        if orientation_error > float(orientation_tolerance):
            raise MechanicalDatumValidationError(f"datum {datum_id} FK orientation error {orientation_error:.9g} rad")
        layout_frame = datum.get("layout_installation_frame")
        if layout_frame is not None:
            layout_pose = layout.get("installation_frames", {}).get(layout_frame)
            if not isinstance(layout_pose, dict):
                raise MechanicalDatumValidationError(f"datum {datum_id} layout frame is absent: {layout_frame!r}")
            layout_matrix = _pose_from_mapping(layout_pose, f"layout frame {layout_frame}")
            if _translation_error_m(expected_pose, layout_matrix) > float(position_tolerance) or _rotation_error_rad(expected_pose, layout_matrix) > float(orientation_tolerance):
                raise MechanicalDatumValidationError(f"datum {datum_id} differs from layout frame {layout_frame}")
        position_id = datum.get("functional_position")
        if position_id is not None:
            position = _by_id(positions, position_id, "functional positions")
            if position.get("link") != link:
                raise MechanicalDatumValidationError(f"datum {datum_id} functional position link differs")
            position_matrix = _pose_from_mapping(position, f"functional position {position_id}")
            if _translation_error_m(expected_pose, position_matrix) > float(position_tolerance) or _rotation_error_rad(expected_pose, position_matrix) > float(orientation_tolerance):
                raise MechanicalDatumValidationError(f"datum {datum_id} differs from functional position {position_id}")
        component_binding = datum.get("functional_component")
        if component_binding is not None:
            if not isinstance(component_binding, dict):
                raise MechanicalDatumValidationError(f"datum {datum_id} functional_component must be a mapping")
            owner = _by_id(positions, component_binding.get("owner_position"), "functional positions")
            component = _functional_component(owner, component_binding.get("component_id"))
            if component.get("link") != link:
                raise MechanicalDatumValidationError(f"datum {datum_id} functional component link differs")
            component_matrix = _pose_from_mapping(component, f"functional component {component_binding.get('component_id')}")
            if _translation_error_m(expected_pose, component_matrix) > float(position_tolerance) or _rotation_error_rad(expected_pose, component_matrix) > float(orientation_tolerance):
                raise MechanicalDatumValidationError(f"datum {datum_id} differs from its functional component")
        observed_datums[datum_id] = {
            "link": link,
            "xyz_m": [round(actual_pose[index][3], 9) for index in range(3)],
            "position_error_m": round(position_error, 12),
            "orientation_error_rad": round(orientation_error, 12),
        }

    assemblies = register.get("mechanical_subassemblies")
    interfaces = crosswalk.get("interface_crosswalk")
    if not isinstance(interfaces, list) or not interfaces:
        raise MechanicalDatumValidationError("interface_crosswalk must be a non-empty list")
    interface_ids: set[str] = set()
    for interface in interfaces:
        if not isinstance(interface, dict):
            raise MechanicalDatumValidationError("interface crosswalk entries must be mappings")
        interface_id = interface.get("interface_id")
        if not isinstance(interface_id, str) or not interface_id or interface_id in interface_ids:
            raise MechanicalDatumValidationError(f"interface id must be unique: {interface_id!r}")
        interface_ids.add(interface_id)
        chain = interface.get("datum_chain")
        if not isinstance(chain, list) or len(chain) < 2 or any(item not in datum_ids for item in chain):
            raise MechanicalDatumValidationError(f"interface {interface_id} has an invalid datum chain")
        if len(set(chain)) != len(chain):
            raise MechanicalDatumValidationError(f"interface {interface_id} repeats a datum")
        assembly_binding = interface.get("mechanical_subassembly")
        if not isinstance(assembly_binding, dict):
            raise MechanicalDatumValidationError(f"interface {interface_id} lacks mechanical subassembly binding")
        assembly = _by_id(assemblies, assembly_binding.get("id"), "mechanical subassemblies")
        _assert_expected(assembly, assembly_binding.get("expected"), f"interface {interface_id} mechanical subassembly")
        position_binding = interface.get("functional_position")
        if position_binding is not None:
            if not isinstance(position_binding, dict):
                raise MechanicalDatumValidationError(f"interface {interface_id} functional position binding must be a mapping")
            position = _by_id(positions, position_binding.get("id"), "functional positions")
            _assert_expected(position, position_binding.get("expected"), f"interface {interface_id} functional position")
        joint_assertions = interface.get("urdf_joints")
        if not isinstance(joint_assertions, list) or not joint_assertions:
            raise MechanicalDatumValidationError(f"interface {interface_id} must bind one or more URDF joints")
        for assertion in joint_assertions:
            if not isinstance(assertion, dict):
                raise MechanicalDatumValidationError(f"interface {interface_id} URDF joint assertion must be a mapping")
            name = assertion.get("name")
            joint = joints.get(name)
            if joint is None:
                raise MechanicalDatumValidationError(f"interface {interface_id} URDF joint is absent: {name!r}")
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent != assertion.get("parent_link") or child != assertion.get("child_link"):
                raise MechanicalDatumValidationError(f"interface {interface_id} URDF joint relation differs: {name}")

    required_interfaces = {
        "chassis_top_plate_to_arm_base",
        "chassis_top_plate_to_sensor_tower",
        "chassis_to_cleaning_head",
        "chassis_top_plate_to_dry_bin",
        "chassis_top_plate_to_wastewater_tank",
        "chassis_to_charge_receptacle",
        "wastewater_tank_to_drain_hose",
    }
    if interface_ids != required_interfaces:
        raise MechanicalDatumValidationError(f"interface ids must be {sorted(required_interfaces)}")
    return {
        "crosswalk_id": crosswalk.get("crosswalk_id"),
        "status": crosswalk["status"],
        "manufacturing_release": False,
        "coordinate_reference": root_frame,
        "nominal_joint_configuration": "zero",
        "source_snapshot_sha256": hashes,
        "datum_count": len(datum_ids),
        "interface_count": len(interface_ids),
        "checked_interfaces": sorted(interface_ids),
        "observed_datums": observed_datums,
        "claim_boundary": "static_zero_joint_snapshot_crosswalk_only",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT, help="repository root")
    parser.add_argument("--crosswalk", type=Path, default=DEFAULT_CROSSWALK)
    parser.add_argument("--output", type=Path, help="optional JSON evidence path")
    args = parser.parse_args()
    try:
        result = validate(args.crosswalk, root=args.root)
    except (MechanicalDatumValidationError, ET.ParseError, OSError, yaml.YAMLError) as error:
        print(json.dumps({"valid": False, "error": str(error)}, indent=2, sort_keys=True))
        return 1
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
