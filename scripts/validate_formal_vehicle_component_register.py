#!/usr/bin/env python3
"""Fail-closed checks for the formal vehicle's named mechanical interfaces."""

from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTER = ROOT / "config" / "high_fidelity_vehicle" / "formal_vehicle_component_register.yaml"
DEFAULT_URDF = ROOT / "reports" / "engineering" / "formal_competition_vehicle.urdf"
DEFAULT_CONTROLLERS = ROOT / "starter_ws" / "src" / "sanitation_vehicle_description" / "config" / "formal_vehicle_controllers.yaml"


class ComponentRegisterError(ValueError):
    pass


def validate(
    register_path: Path = DEFAULT_REGISTER,
    urdf_path: Path = DEFAULT_URDF,
    controllers_path: Path = DEFAULT_CONTROLLERS,
) -> dict:
    register = yaml.safe_load(register_path.read_text(encoding="utf-8"))
    controllers = yaml.safe_load(controllers_path.read_text(encoding="utf-8"))
    urdf_text = urdf_path.read_text(encoding="utf-8")
    root = ET.parse(urdf_path).getroot()
    links = {link.attrib["name"] for link in root.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    parent_by_child = {
        joint.find("child").attrib["link"]: joint.find("parent").attrib["link"]
        for joint in joints.values()
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

    manager_types = controllers.get("controller_manager", {}).get("ros__parameters", {})

    def controller_joints(name: str) -> set[str]:
        params = controllers.get(name, {}).get("ros__parameters", {})
        values: list[str] = []
        for key in ("joints", "left_wheel_names", "right_wheel_names"):
            values.extend(params.get(key, []) or [])
        return set(values)
    checked_sensors: list[str] = []
    for sensor in register.get("sensor_installations", []):
        sensor_id = sensor["id"]
        parent = sensor["parent_link"]
        mount = sensor["mount_link"]
        child = sensor["sensor_link"]
        for field, name in (("parent_link", parent), ("mount_link", mount), ("sensor_link", child)):
            if name not in links:
                errors.append(f"{sensor_id}.{field} missing from URDF: {name}")
        if mount != child and parent_by_child.get(mount) != parent:
            errors.append(
                f"{sensor_id} mount load path is {parent_by_child.get(mount)!r}->{mount}, expected {parent}->{mount}"
            )
        if mount != child and parent_by_child.get(child) != mount:
            errors.append(
                f"{sensor_id} sensor load path is {parent_by_child.get(child)!r}->{child}, expected {mount}->{child}"
            )
        if not sensor.get("connection"):
            errors.append(f"{sensor_id} has no mechanical connection description")
        if not sensor.get("topic"):
            errors.append(f"{sensor_id} has no simulation topic")
        checked_sensors.append(sensor_id)

    checked_subassemblies: list[str] = []
    for assembly in register.get("mechanical_subassemblies", []):
        assembly_id = assembly["id"]
        parent = assembly["parent_link"]
        roots = assembly.get("root_links", [assembly.get("root_link")])
        if parent not in links:
            errors.append(f"{assembly_id}.parent_link missing from URDF: {parent}")
        for child in roots:
            if child not in links:
                errors.append(f"{assembly_id}.root_link missing from URDF: {child}")
        if not assembly.get("connection"):
            errors.append(f"{assembly_id} has no mechanical connection description")
        for joint_name in assembly.get("driven_joints", []):
            if joint_name not in joints:
                errors.append(f"{assembly_id}.driven_joint missing from URDF: {joint_name}")
        checked_subassemblies.append(assembly_id)

    checked_positions: list[str] = []
    position_ids: set[str] = set()
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
        if not position.get("function"):
            errors.append(f"{position_id} has no function definition")
        xyz = position.get("xyz_m")
        if position.get("dynamic_position"):
            if xyz is not None:
                errors.append(f"{position_id} dynamic position must use xyz_m: null")
        elif not isinstance(xyz, list) or len(xyz) != 3:
            errors.append(f"{position_id} requires a three-element xyz_m")
        required_joints = position.get("required_joints", [])
        for joint_name in required_joints:
            if joint_name not in joints:
                errors.append(f"{position_id}.required_joint missing from URDF: {joint_name}")
        interface = position.get("interface")
        if required_joints and not interface:
            errors.append(f"{position_id} has actuators but no controller interface")
        if interface:
            if interface not in manager_types:
                errors.append(f"{position_id}.interface not declared by controller manager: {interface}")
            missing = set(required_joints) - controller_joints(interface)
            if missing:
                errors.append(f"{position_id}.interface {interface} does not command {sorted(missing)}")
        for topic in position.get("required_topics", []):
            if topic not in urdf_text:
                errors.append(f"{position_id}.required_topic missing from URDF: {topic}")
        if "visible" not in position:
            errors.append(f"{position_id} does not declare product visibility")
        checked_positions.append(position_id)

    if errors:
        raise ComponentRegisterError("; ".join(errors))
    return {
        "register_id": register["register_id"],
        "status": "COMPONENT_REGISTER_AND_MECHANICAL_LOAD_PATHS_VALID",
        "sensor_installation_count": len(checked_sensors),
        "mechanical_subassembly_count": len(checked_subassemblies),
        "functional_position_count": len(checked_positions),
        "checked_sensor_installations": checked_sensors,
        "checked_mechanical_subassemblies": checked_subassemblies,
        "checked_functional_positions": checked_positions,
        "top_protrusion_name": register["external_identity"]["top_protrusion_name"],
        "claim_boundary": register["claim_boundary"],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--register", type=Path, default=DEFAULT_REGISTER)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--write-report", type=Path)
    args = parser.parse_args()
    result = validate(args.register, args.urdf)
    if args.write_report:
        args.write_report.parent.mkdir(parents=True, exist_ok=True)
        args.write_report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
