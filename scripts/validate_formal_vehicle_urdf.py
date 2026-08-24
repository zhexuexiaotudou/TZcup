#!/usr/bin/env python3
"""Validate the formal competition-vehicle layout and an expanded URDF.

The validator intentionally separates deterministic XML/layout checks from gates
that require mesh, CAD, MoveIt or Gazebo scans.  It never converts a pending
simulation gate into a pass based on declared YAML alone.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml

from validate_pre_urdf_readiness import (
    ContractError as PreUrdfContractError,
    load_contract,
    validate_budget_csvs,
    validate_contract,
)


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LAYOUT = ROOT / "config" / "high_fidelity_vehicle" / "formal_vehicle_layout.yaml"
DEFAULT_CONTRACT = ROOT / "config" / "high_fidelity_vehicle" / "pre_urdf_contract.yaml"
DEFAULT_URDF = ROOT / "reports" / "engineering" / "formal_competition_vehicle.urdf"
ROS_NAME_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
NUMBER_SPLIT_RE = re.compile(r"[\s,]+")
EPSILON = 1e-12


class FormalVehicleValidationError(ValueError):
    """Raised when deterministic evidence cannot support the formal URDF."""


def _load_yaml(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise FormalVehicleValidationError(f"{path} root must be a mapping")
    return data


def _number(value: Any, field: str, *, positive: bool = False, allow_zero: bool = True) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise FormalVehicleValidationError(f"{field} must be numeric") from exc
    if not math.isfinite(number):
        raise FormalVehicleValidationError(f"{field} must be finite")
    if positive and (number < 0.0 or (number == 0.0 and not allow_zero)):
        qualifier = "non-negative" if allow_zero else "positive"
        raise FormalVehicleValidationError(f"{field} must be {qualifier}")
    return number


def _vector(value: Any, field: str, length: int = 3) -> list[float]:
    if isinstance(value, str):
        raw = [token for token in NUMBER_SPLIT_RE.split(value.strip()) if token]
    else:
        raw = value
    if not isinstance(raw, (list, tuple)) or len(raw) != length:
        raise FormalVehicleValidationError(f"{field} must contain {length} numbers")
    return [_number(item, f"{field}[{index}]") for index, item in enumerate(raw)]


def _unique(values: list[str], field: str) -> None:
    if len(values) != len(set(values)):
        duplicates = sorted({name for name in values if values.count(name) > 1})
        raise FormalVehicleValidationError(f"duplicate {field}: {', '.join(duplicates)}")


def _box(entry: dict[str, Any], field: str) -> tuple[list[float], list[float]]:
    lower = _vector(entry.get("min_xyz_m"), f"{field}.min_xyz_m")
    upper = _vector(entry.get("max_xyz_m"), f"{field}.max_xyz_m")
    if any(high <= low for low, high in zip(lower, upper)):
        raise FormalVehicleValidationError(f"{field} has an empty or inverted envelope")
    return lower, upper


def _inside(inner: tuple[list[float], list[float]], outer: tuple[list[float], list[float]]) -> bool:
    return all(
        outer[0][axis] - 1e-9 <= inner[0][axis]
        and inner[1][axis] <= outer[1][axis] + 1e-9
        for axis in range(3)
    )


def _csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def validate_layout(
    layout_path: Path = DEFAULT_LAYOUT,
    contract_path: Path = DEFAULT_CONTRACT,
) -> dict[str, Any]:
    """Validate layout inputs and budgets without claiming URDF/simulation success."""

    layout = _load_yaml(layout_path)
    if layout.get("schema_version") != 1:
        raise FormalVehicleValidationError("unsupported formal layout schema_version")
    robot = layout.get("robot", {})
    if robot.get("name") != "tzcup_formal_sanitation_vehicle":
        raise FormalVehicleValidationError("formal robot name is not frozen to the project contract")
    if robot.get("root_frame") != "base_footprint":
        raise FormalVehicleValidationError("formal robot root frame must be base_footprint")

    try:
        contract = load_contract(contract_path)
        pre_result = validate_contract(contract)
        validate_budget_csvs(pre_result, contract_path.parent)
    except PreUrdfContractError as exc:
        raise FormalVehicleValidationError(f"pre-URDF contract failed: {exc}") from exc

    required_frames = set(contract["frame_contract"]["required_frames"])
    installation = layout.get("installation_frames", {})
    missing_installations = sorted(required_frames - set(installation))
    if missing_installations:
        raise FormalVehicleValidationError(
            "required installation frames missing: " + ", ".join(missing_installations)
        )
    for name, pose in installation.items():
        if not ROS_NAME_RE.fullmatch(str(name)):
            raise FormalVehicleValidationError(f"invalid installation frame name: {name}")
        _vector(pose.get("xyz_m"), f"installation_frames.{name}.xyz_m")
        _vector(pose.get("rpy_rad"), f"installation_frames.{name}.rpy_rad")
        if not isinstance(pose.get("pending"), bool):
            raise FormalVehicleValidationError(f"installation frame {name} lacks explicit pending flag")
        if pose["pending"] and not pose.get("pending_reason"):
            raise FormalVehicleValidationError(f"pending installation frame {name} lacks a reason")

    components = layout.get("component_envelopes", {})
    if not components:
        raise FormalVehicleValidationError("component_envelopes cannot be empty")
    component_boxes: dict[str, tuple[list[float], list[float]]] = {}
    for name, entry in components.items():
        component_boxes[name] = _box(entry, f"component_envelopes.{name}")
        if not isinstance(entry.get("pending"), bool):
            raise FormalVehicleValidationError(f"component envelope {name} lacks explicit pending flag")
        if entry["pending"] and not entry.get("pending_reason"):
            raise FormalVehicleValidationError(f"pending component envelope {name} lacks a reason")

    envelopes = layout.get("vehicle_envelopes", {})
    stowed = _box(envelopes.get("transport_stowed", {}), "vehicle_envelopes.transport_stowed")
    operating = _box(envelopes.get("arm_operating", {}), "vehicle_envelopes.arm_operating")
    if not _inside(stowed, operating):
        raise FormalVehicleValidationError("transport envelope must fit inside arm-operating envelope")
    outside = sorted(name for name, bounds in component_boxes.items() if not _inside(bounds, stowed))
    if outside:
        raise FormalVehicleValidationError(
            "component envelopes exceed the transport envelope: " + ", ".join(outside)
        )
    stowed_size = [stowed[1][axis] - stowed[0][axis] for axis in range(3)]
    declared_size = [
        _number(envelopes.get("configured_max_transport_length_m"), "transport length", positive=True, allow_zero=False),
        _number(envelopes.get("configured_max_transport_width_m"), "transport width", positive=True, allow_zero=False),
        _number(envelopes.get("configured_max_transport_height_m"), "transport height", positive=True, allow_zero=False),
    ]
    if any(abs(value - expected) > 1e-6 for value, expected in zip(stowed_size, declared_size)):
        raise FormalVehicleValidationError("declared transport dimensions differ from the envelope")

    storage = layout.get("storage", {})
    dry = storage.get("dry_bin", {})
    dry_dimensions = _vector(dry.get("internal_dimensions_m"), "storage.dry_bin.internal_dimensions_m")
    dry_geometric_l = math.prod(dry_dimensions) * 1000.0
    dry_declared_l = _number(dry.get("geometric_volume_l"), "dry-bin geometric volume", positive=True, allow_zero=False)
    dry_usable_fraction = _number(dry.get("usable_fraction"), "dry-bin usable fraction", positive=True, allow_zero=False)
    dry_usable_l = _number(dry.get("usable_volume_l"), "dry-bin usable volume", positive=True, allow_zero=False)
    if dry_usable_fraction > 1.0:
        raise FormalVehicleValidationError("dry-bin usable fraction cannot exceed one")
    if abs(dry_geometric_l - dry_declared_l) > 1e-6:
        raise FormalVehicleValidationError("dry-bin dimensions do not match geometric volume")
    if abs(dry_declared_l * dry_usable_fraction - dry_usable_l) > 1e-5:
        raise FormalVehicleValidationError("dry-bin usable volume does not match geometry and fraction")
    dry_required_l = float(contract["competition_requirements"]["dry_bin_usable_min_l"])
    if dry_usable_l + 1e-9 < dry_required_l:
        raise FormalVehicleValidationError("dry-bin usable volume is below 40 L")

    wet = storage.get("wastewater_tank", {})
    wet_dimensions = _vector(wet.get("internal_dimensions_m"), "storage.wastewater_tank.internal_dimensions_m")
    install_limit_l = math.prod(wet_dimensions) * 1000.0
    declared_install_limit_l = _number(
        wet.get("geometric_installation_limit_l"),
        "wastewater installation limit",
        positive=True,
        allow_zero=False,
    )
    if abs(install_limit_l - declared_install_limit_l) > 1e-6:
        raise FormalVehicleValidationError("wastewater tank dimensions do not match installation limit")
    if wet.get("cog_limit_l") is not None:
        _number(wet["cog_limit_l"], "wastewater CoG limit", positive=True, allow_zero=False)
    if wet.get("final_capacity_pending") is not True or wet.get("cog_limit_l") is not None:
        raise FormalVehicleValidationError(
            "wastewater capacity must remain pending until a full-vehicle CoG scan supplies its limit"
        )

    capacity_rows = {
        row["capacity_id"]: row
        for row in _csv_rows(contract_path.parent / "capacity_budget.csv")
    }
    mass_rows = {
        row["row_id"]: row
        for row in _csv_rows(contract_path.parent / "mass_budget.csv")
    }
    mass_limit_l = float(capacity_rows["wastewater_mass_limited_nominal"]["volume_l"])
    design_cap_l = float(capacity_rows["wastewater_design_cap"]["volume_l"])
    preliminary_nominal_l = min(mass_limit_l, design_cap_l, install_limit_l)
    wet_usable_fraction = _number(wet.get("usable_fraction"), "wastewater usable fraction", positive=True, allow_zero=False)
    if wet_usable_fraction > 1.0:
        raise FormalVehicleValidationError("wastewater usable fraction cannot exceed one")
    preliminary_usable_l = preliminary_nominal_l * wet_usable_fraction
    fixed_payload_kg = float(mass_rows["fixed_payload"]["mass_kg"])
    dry_trash_kg = float(mass_rows["worst_case_dry_trash"]["mass_kg"])
    design_payload_limit_kg = float(contract["mass_capacity_budget"]["payload_design_limit_kg"])
    water_density_kg_l = float(contract["mass_capacity_budget"]["wastewater"]["density_kg_l"])
    payload_at_usable_fill_kg = fixed_payload_kg + dry_trash_kg + preliminary_usable_l * water_density_kg_l
    if payload_at_usable_fill_kg > design_payload_limit_kg + 1e-9:
        raise FormalVehicleValidationError("usable wastewater fill exceeds the design payload limit")

    cleaning = layout.get("cleaning_geometry", {})
    working_width_m = _number(
        cleaning.get("effective_working_width_m"),
        "effective cleaning width",
        positive=True,
        allow_zero=False,
    )
    required_width_m = max(
        float(contract["competition_requirements"]["cleaning_width_min_m"]),
        _number(cleaning.get("required_minimum_width_m"), "required cleaning width", positive=True, allow_zero=False),
    )
    if working_width_m + 1e-9 < required_width_m:
        raise FormalVehicleValidationError("effective cleaning width is below 0.6 m")

    arm = layout.get("arm_envelopes", {})
    arm_stowed = _box(arm.get("stowed", {}), "arm_envelopes.stowed")
    arm_deployed = _box(arm.get("deployed", {}), "arm_envelopes.deployed")
    if not _inside(arm_stowed, stowed):
        raise FormalVehicleValidationError("stowed arm envelope exceeds transport envelope")
    if not _inside(arm_deployed, operating):
        raise FormalVehicleValidationError("deployed arm envelope exceeds operating envelope")
    if not arm.get("stowed", {}).get("pending") or not arm.get("deployed", {}).get("pending"):
        raise FormalVehicleValidationError("arm envelopes cannot be final before the exact swept-volume scan")

    sensor_contracts = {item["id"]: item for item in contract["sensor_contracts"]}
    sensor_layout = layout.get("sensor_layout", [])
    sensor_ids = [str(item.get("id", "")) for item in sensor_layout]
    sensor_frames = [str(item.get("frame", "")) for item in sensor_layout]
    _unique(sensor_ids, "sensor layout ids")
    _unique(sensor_frames, "sensor layout frames")
    missing_sensors = sorted(set(sensor_contracts) - set(sensor_ids))
    if missing_sensors:
        raise FormalVehicleValidationError("sensor layout missing: " + ", ".join(missing_sensors))
    passed_fov: list[str] = []
    pending_fov: list[str] = []
    for sensor in sensor_layout:
        sensor_id = str(sensor["id"])
        if sensor["frame"] != sensor_contracts[sensor_id]["frame"]:
            raise FormalVehicleValidationError(f"sensor {sensor_id} frame differs from the pre-URDF contract")
        _vector(sensor.get("xyz_m"), f"sensor_layout.{sensor_id}.xyz_m")
        forward = _vector(sensor.get("forward_xyz"), f"sensor_layout.{sensor_id}.forward_xyz")
        norm = math.sqrt(sum(value * value for value in forward))
        if abs(norm - 1.0) > 0.02:
            raise FormalVehicleValidationError(f"sensor {sensor_id} forward vector is not normalized")
        minimum = _number(sensor.get("minimum_clear_fraction"), f"sensor {sensor_id} minimum clear fraction", positive=True)
        if minimum > 1.0:
            raise FormalVehicleValidationError(f"sensor {sensor_id} minimum clear fraction exceeds one")
        if sensor.get("pending"):
            if sensor.get("approximate_clear_fraction") is not None or not sensor.get("pending_reason"):
                raise FormalVehicleValidationError(f"pending sensor {sensor_id} must be unscored and explain why")
            pending_fov.append(sensor_id)
        else:
            clear = _number(sensor.get("approximate_clear_fraction"), f"sensor {sensor_id} clear fraction", positive=True)
            if clear > 1.0 or clear + 1e-9 < minimum:
                raise FormalVehicleValidationError(f"sensor {sensor_id} fails the approximate FOV clearance gate")
            passed_fov.append(sensor_id)

    policy = layout.get("validation_policy", {})
    approved_pending = [str(item) for item in policy.get("approved_pending_gates", [])]
    _unique(approved_pending, "approved pending gates")
    if not approved_pending:
        raise FormalVehicleValidationError("approved_pending_gates cannot be empty")
    if policy.get("require_inertial_on_every_physical_link") is not True:
        raise FormalVehicleValidationError("every physical formal URDF link must retain an inertial")
    inertial_exempt = [str(item) for item in policy.get("inertial_exempt_virtual_frames", [])]
    _unique(inertial_exempt, "inertial-exempt virtual frames")
    if inertial_exempt != [robot["root_frame"]]:
        raise FormalVehicleValidationError("only the virtual root frame may be exempt from inertial data")

    passed_checks = [
        "pre_urdf_contract_and_four_budget_csvs",
        "required_installation_frames_have_numeric_initial_poses",
        "declared_component_envelopes_fit_transport_envelope",
        "dry_bin_geometry_and_usable_volume_at_least_40_l",
        "mass_installation_and_design_caps_bound_preliminary_wastewater_fill",
        "payload_limit_at_preliminary_usable_wastewater_fill",
        "effective_cleaning_width_at_least_0_6_m",
        "declared_arm_stowed_and_operating_envelopes_are_bounded",
        "static_sensor_layout_approximate_fov_clearance",
    ]
    return {
        "layout_id": layout["layout_id"],
        "robot_name": robot["name"],
        "status": "LAYOUT_CONTRACT_VALID_URDF_AND_SIMULATION_GATES_PENDING",
        "passed_deterministic_checks": passed_checks,
        "urdf_validation": {"evaluated": False, "passed": False},
        "counts": {
            "installation_frames": len(installation),
            "component_envelopes": len(components),
            "sensor_layouts": len(sensor_layout),
            "approximate_fov_passed": len(passed_fov),
            "approximate_fov_pending": len(pending_fov),
        },
        "transport_envelope_m": {
            "length": round(stowed_size[0], 6),
            "width": round(stowed_size[1], 6),
            "height": round(stowed_size[2], 6),
        },
        "dry_bin_usable_l": round(dry_usable_l, 6),
        "preliminary_wastewater": {
            "mass_limit_l": round(mass_limit_l, 6),
            "installation_limit_l": round(install_limit_l, 6),
            "cog_limit_l": None,
            "nominal_before_usable_fraction_l": round(preliminary_nominal_l, 6),
            "usable_l": round(preliminary_usable_l, 6),
            "final_capacity_frozen": False,
        },
        "payload": {
            "design_limit_kg": round(design_payload_limit_kg, 6),
            "at_preliminary_usable_fill_kg": round(payload_at_usable_fill_kg, 6),
            "remaining_margin_kg": round(design_payload_limit_kg - payload_at_usable_fill_kg, 6),
        },
        "effective_cleaning_width_m": round(working_width_m, 6),
        "approximate_fov_passed_sensors": passed_fov,
        "approximate_fov_pending_sensors": pending_fov,
        "pending_external_gates": approved_pending,
        "claim_boundary": (
            "Numeric initial layout and deterministic budgets pass. Expanded URDF inertials/names/limits "
            "must pass separately; final CoG, wastewater capacity, mesh swept volumes, exact occlusion, "
            "ground contact and Gazebo sensor visibility remain fail-closed."
        ),
    }


def _attribute_numbers(element: ET.Element, attribute: str, field: str, count: int) -> list[float]:
    if attribute not in element.attrib:
        raise FormalVehicleValidationError(f"{field} is missing")
    return _vector(element.attrib[attribute], field, count)


def _rotation_from_rpy(rpy: list[float]) -> list[list[float]]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _matmul(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3)]
        for row in range(3)
    ]


def _rotate(rotation: list[list[float]], vector: list[float]) -> list[float]:
    return [sum(rotation[row][column] * vector[column] for column in range(3)) for row in range(3)]


def _compose_pose(
    parent: tuple[list[float], list[list[float]]],
    child: tuple[list[float], list[list[float]]],
) -> tuple[list[float], list[list[float]]]:
    shifted = _rotate(parent[1], child[0])
    return (
        [parent[0][axis] + shifted[axis] for axis in range(3)],
        _matmul(parent[1], child[1]),
    )


def _rotation_distance(left: list[list[float]], right: list[list[float]]) -> float:
    # angle(R_left^T R_right), clamped for round-off.
    relative_trace = sum(
        left[row][column] * right[row][column]
        for row in range(3)
        for column in range(3)
    )
    cosine = max(-1.0, min(1.0, (relative_trace - 1.0) / 2.0))
    return math.acos(cosine)


def _joint_origin_pose(joint: ET.Element) -> tuple[list[float], list[list[float]]]:
    origin = joint.find("origin")
    if origin is None:
        return [0.0, 0.0, 0.0], _rotation_from_rpy([0.0, 0.0, 0.0])
    xyz = _vector(origin.attrib.get("xyz", "0 0 0"), f"joint {joint.attrib.get('name')} origin xyz")
    rpy = _vector(origin.attrib.get("rpy", "0 0 0"), f"joint {joint.attrib.get('name')} origin rpy")
    return xyz, _rotation_from_rpy(rpy)


def _world_link_poses(
    joints: list[ET.Element], root_name: str
) -> dict[str, tuple[list[float], list[list[float]]]]:
    identity = ([0.0, 0.0, 0.0], _rotation_from_rpy([0.0, 0.0, 0.0]))
    poses: dict[str, tuple[list[float], list[list[float]]]] = {root_name: identity}
    remaining = list(joints)
    while remaining:
        progressed = False
        for joint in list(remaining):
            parent = joint.find("parent").attrib["link"]
            if parent not in poses:
                continue
            child = joint.find("child").attrib["link"]
            # Zero joint position is the formal installation reference. Joint
            # motion is handled later by CAD/MoveIt swept-volume gates.
            poses[child] = _compose_pose(poses[parent], _joint_origin_pose(joint))
            remaining.remove(joint)
            progressed = True
        if not progressed:
            unresolved = ", ".join(sorted(joint.attrib.get("name", "") for joint in remaining))
            raise FormalVehicleValidationError(
                "URDF joint graph cannot be resolved from the root; possible cycle: " + unresolved
            )
    return poses


def _validate_static_frame_poses(
    joints: list[ET.Element], layout: dict[str, Any]
) -> dict[str, Any]:
    policy = layout["validation_policy"]
    position_tolerance = _number(
        policy.get("static_frame_position_tolerance_m"),
        "static frame position tolerance",
        positive=True,
        allow_zero=False,
    )
    rotation_tolerance = _number(
        policy.get("static_frame_rotation_tolerance_rad"),
        "static frame rotation tolerance",
        positive=True,
        allow_zero=False,
    )
    poses = _world_link_poses(joints, layout["robot"]["root_frame"])
    checked: list[str] = []
    pending: list[str] = []
    mismatches: list[str] = []
    for name, expected in layout["installation_frames"].items():
        if expected["pending"]:
            pending.append(name)
            continue
        if name not in poses:
            raise FormalVehicleValidationError(f"installation frame {name} has no resolved URDF pose")
        expected_position = _vector(expected["xyz_m"], f"installation_frames.{name}.xyz_m")
        expected_rotation = _rotation_from_rpy(
            _vector(expected["rpy_rad"], f"installation_frames.{name}.rpy_rad")
        )
        actual_position, actual_rotation = poses[name]
        position_error = math.sqrt(
            sum((actual_position[axis] - expected_position[axis]) ** 2 for axis in range(3))
        )
        rotation_error = _rotation_distance(expected_rotation, actual_rotation)
        if position_error > position_tolerance or rotation_error > rotation_tolerance:
            mismatches.append(
                f"installation frame {name} differs from layout "
                f"(position_error={position_error:.6f} m, rotation_error={rotation_error:.6f} rad)"
            )
        else:
            checked.append(name)
    if mismatches:
        raise FormalVehicleValidationError("; ".join(mismatches))
    return {
        "checked_count": len(checked),
        "pending_count": len(pending),
        "checked_frames": checked,
        "pending_frames": pending,
        "position_tolerance_m": position_tolerance,
        "rotation_tolerance_rad": rotation_tolerance,
    }


def _validate_inertial(link: ET.Element, inertial_exempt: set[str]) -> float:
    name = link.attrib.get("name", "")
    inertial = link.find("inertial")
    if inertial is None:
        if name in inertial_exempt:
            return 0.0
        raise FormalVehicleValidationError(f"link {name} has no inertial")
    if name in inertial_exempt:
        raise FormalVehicleValidationError(f"virtual frame {name} must not carry inertial data")
    mass_node = inertial.find("mass")
    if mass_node is None:
        raise FormalVehicleValidationError(f"link {name} inertial has no mass")
    mass = _number(mass_node.attrib.get("value"), f"link {name} mass", positive=True, allow_zero=False)
    origin = inertial.find("origin")
    if origin is not None:
        if "xyz" in origin.attrib:
            _attribute_numbers(origin, "xyz", f"link {name} inertial origin xyz", 3)
        if "rpy" in origin.attrib:
            _attribute_numbers(origin, "rpy", f"link {name} inertial origin rpy", 3)
    inertia = inertial.find("inertia")
    if inertia is None:
        raise FormalVehicleValidationError(f"link {name} inertial has no inertia matrix")
    values = {
        key: _number(inertia.attrib.get(key), f"link {name} inertia {key}")
        for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
    }
    ixx, ixy, ixz = values["ixx"], values["ixy"], values["ixz"]
    iyy, iyz, izz = values["iyy"], values["iyz"], values["izz"]
    leading_minor_2 = ixx * iyy - ixy * ixy
    determinant = (
        ixx * (iyy * izz - iyz * iyz)
        - ixy * (ixy * izz - iyz * ixz)
        + ixz * (ixy * iyz - iyy * ixz)
    )
    # Sylvester's criterion is scale-independent. Virtual REP/optical frames
    # deliberately use tiny but strictly positive inertias, so an absolute
    # engineering epsilon would incorrectly reject a mathematically valid SPD
    # matrix solely because its SI magnitude is small.
    if ixx <= 0.0 or leading_minor_2 <= 0.0 or determinant <= 0.0:
        raise FormalVehicleValidationError(f"link {name} inertia matrix is not positive definite")
    if ixx + iyy < izz - 1e-9 or ixx + izz < iyy - 1e-9 or iyy + izz < ixx - 1e-9:
        raise FormalVehicleValidationError(f"link {name} inertia violates physical triangle inequalities")
    return mass


def _validate_joint(joint: ET.Element, links: set[str], limit_types: set[str]) -> None:
    name = joint.attrib.get("name", "")
    joint_type = joint.attrib.get("type", "")
    if joint_type not in {"fixed", "revolute", "continuous", "prismatic", "floating", "planar"}:
        raise FormalVehicleValidationError(f"joint {name} has unsupported type {joint_type}")
    parent = joint.find("parent")
    child = joint.find("child")
    if parent is None or child is None:
        raise FormalVehicleValidationError(f"joint {name} must define parent and child")
    parent_name = parent.attrib.get("link", "")
    child_name = child.attrib.get("link", "")
    if parent_name not in links or child_name not in links:
        raise FormalVehicleValidationError(f"joint {name} references an unknown link")
    if parent_name == child_name:
        raise FormalVehicleValidationError(f"joint {name} connects a link to itself")
    origin = joint.find("origin")
    if origin is not None:
        if "xyz" in origin.attrib:
            _attribute_numbers(origin, "xyz", f"joint {name} origin xyz", 3)
        if "rpy" in origin.attrib:
            _attribute_numbers(origin, "rpy", f"joint {name} origin rpy", 3)
    if joint_type in {"revolute", "continuous", "prismatic"}:
        axis = joint.find("axis")
        if axis is None:
            raise FormalVehicleValidationError(f"joint {name} has no axis")
        axis_values = _attribute_numbers(axis, "xyz", f"joint {name} axis", 3)
        if math.sqrt(sum(value * value for value in axis_values)) <= EPSILON:
            raise FormalVehicleValidationError(f"joint {name} axis is zero")
    if joint_type in limit_types:
        limit = joint.find("limit")
        if limit is None:
            raise FormalVehicleValidationError(f"joint {name} has no limit")
        effort = _number(limit.attrib.get("effort"), f"joint {name} effort", positive=True, allow_zero=False)
        velocity = _number(limit.attrib.get("velocity"), f"joint {name} velocity", positive=True, allow_zero=False)
        if effort <= 0.0 or velocity <= 0.0:  # explicit for readable failures
            raise FormalVehicleValidationError(f"joint {name} effort and velocity must be positive")
        if joint_type in {"revolute", "prismatic"}:
            lower = _number(limit.attrib.get("lower"), f"joint {name} lower limit")
            upper = _number(limit.attrib.get("upper"), f"joint {name} upper limit")
            if upper <= lower:
                raise FormalVehicleValidationError(f"joint {name} has inverted or empty limits")


def validate_expanded_urdf(
    urdf_path: Path,
    layout_path: Path = DEFAULT_LAYOUT,
    contract_path: Path = DEFAULT_CONTRACT,
) -> dict[str, Any]:
    """Validate an already-expanded URDF and combine it with layout evidence."""

    layout_result = validate_layout(layout_path, contract_path)
    layout = _load_yaml(layout_path)
    contract = load_contract(contract_path)
    if not urdf_path.is_file():
        raise FormalVehicleValidationError(f"expanded URDF does not exist: {urdf_path}")
    raw = urdf_path.read_text(encoding="utf-8")
    forbidden = [str(token) for token in layout["validation_policy"]["forbidden_symbolic_tokens"]]
    found = [token for token in forbidden if token.lower() in raw.lower()]
    if found:
        raise FormalVehicleValidationError("expanded URDF contains symbolic placeholder tokens: " + ", ".join(found))
    try:
        root = ET.fromstring(raw)
    except ET.ParseError as exc:
        raise FormalVehicleValidationError(f"expanded URDF is not valid XML: {exc}") from exc
    if root.tag != "robot":
        raise FormalVehicleValidationError("expanded URDF root must be <robot>")
    if root.attrib.get("name") != layout["robot"]["name"]:
        raise FormalVehicleValidationError("expanded URDF robot name differs from the layout contract")

    links_nodes = root.findall("link")
    joints_nodes = root.findall("joint")
    link_names = [link.attrib.get("name", "") for link in links_nodes]
    joint_names = [joint.attrib.get("name", "") for joint in joints_nodes]
    if not link_names:
        raise FormalVehicleValidationError("expanded URDF contains no links")
    _unique(link_names, "link names")
    _unique(joint_names, "joint names")
    for name in link_names + joint_names:
        if not ROS_NAME_RE.fullmatch(name):
            raise FormalVehicleValidationError(f"invalid URDF name: {name}")
    link_set = set(link_names)
    required_frames = set(contract["frame_contract"]["required_frames"])
    missing_frames = sorted(required_frames - link_set)
    if missing_frames:
        raise FormalVehicleValidationError("required URDF frames missing: " + ", ".join(missing_frames))
    required_joints = {item["name"] for item in contract["joint_contract"]}
    missing_joints = sorted(required_joints - set(joint_names))
    if missing_joints:
        raise FormalVehicleValidationError("required URDF joints missing: " + ", ".join(missing_joints))
    sensor_frames = {item["frame"] for item in contract["sensor_contracts"]}
    missing_sensor_frames = sorted(sensor_frames - link_set)
    if missing_sensor_frames:
        raise FormalVehicleValidationError("required sensor frames missing: " + ", ".join(missing_sensor_frames))

    inertial_exempt = set(layout["validation_policy"]["inertial_exempt_virtual_frames"])
    unknown_exempt = sorted(inertial_exempt - link_set)
    if unknown_exempt:
        raise FormalVehicleValidationError(
            "inertial-exempt virtual frames missing from URDF: " + ", ".join(unknown_exempt)
        )
    total_mass = sum(_validate_inertial(link, inertial_exempt) for link in links_nodes)
    limit_types = set(layout["validation_policy"]["required_joint_limits"])
    for joint in joints_nodes:
        _validate_joint(joint, link_set, limit_types)

    children: dict[str, str] = {}
    for joint in joints_nodes:
        child = joint.find("child").attrib["link"]  # validated above
        if child in children:
            raise FormalVehicleValidationError(
                f"link {child} has multiple parent joints: {children[child]} and {joint.attrib['name']}"
            )
        children[child] = joint.attrib["name"]
    roots = sorted(link_set - set(children))
    if roots != [layout["robot"]["root_frame"]]:
        raise FormalVehicleValidationError(
            "expanded URDF must be a single tree rooted at base_footprint; roots: " + ", ".join(roots)
        )
    frame_pose_result = _validate_static_frame_poses(joints_nodes, layout)
    try:
        reported_urdf_path = urdf_path.resolve().relative_to(ROOT.resolve()).as_posix()
    except ValueError:
        reported_urdf_path = urdf_path.resolve().as_posix()

    result = dict(layout_result)
    result["status"] = "FORMAL_URDF_DETERMINISTIC_CHECKS_PASSED_EXTERNAL_GATES_PENDING"
    result["urdf_validation"] = {
        "evaluated": True,
        "passed": True,
        "path": reported_urdf_path,
        "link_count": len(link_names),
        "joint_count": len(joint_names),
        "sensor_frame_count": len(sensor_frames),
        "total_empty_vehicle_mass_kg": round(total_mass, 6),
        "all_physical_links_have_positive_mass_and_positive_definite_physical_inertia": True,
        "massless_virtual_frames": sorted(inertial_exempt),
        "all_required_frames_joints_and_sensors_present": True,
        "joint_limits_and_tree_topology_valid": True,
        "symbolic_placeholders_absent": True,
        "static_frame_pose_consistency": frame_pose_result,
    }
    result["passed_deterministic_checks"] = list(result["passed_deterministic_checks"]) + [
        "expanded_urdf_xml_and_robot_identity",
        "unique_link_and_joint_names",
        "all_physical_link_mass_and_inertia",
        "required_frames_joints_and_sensor_frames",
        "joint_limits_axes_and_tree_topology",
        "no_symbolic_placeholders_in_expanded_urdf",
        "non_pending_static_frame_poses_match_layout",
    ]
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--layout", type=Path, default=DEFAULT_LAYOUT)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--urdf", type=Path, help="Path to an expanded URDF (not Xacro)")
    parser.add_argument("--layout-only", action="store_true", help="Validate only numeric layout/budgets")
    parser.add_argument("--expect-report", type=Path)
    parser.add_argument("--write-report", type=Path)
    args = parser.parse_args()
    if args.layout_only and args.urdf:
        parser.error("--layout-only and --urdf are mutually exclusive")
    urdf_path = args.urdf
    if not args.layout_only and urdf_path is None:
        urdf_path = DEFAULT_URDF
    result = (
        validate_layout(args.layout, args.contract)
        if args.layout_only
        else validate_expanded_urdf(urdf_path, args.layout, args.contract)
    )
    if args.expect_report:
        expected = json.loads(args.expect_report.read_text(encoding="utf-8"))
        if result != expected:
            raise FormalVehicleValidationError("committed formal-layout report differs from validation")
    if args.write_report:
        args.write_report.parent.mkdir(parents=True, exist_ok=True)
        args.write_report.write_text(
            json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
