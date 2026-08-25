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


class ComponentRegisterError(ValueError):
    pass


def validate(register_path: Path = DEFAULT_REGISTER, urdf_path: Path = DEFAULT_URDF) -> dict:
    register = yaml.safe_load(register_path.read_text(encoding="utf-8"))
    root = ET.parse(urdf_path).getroot()
    links = {link.attrib["name"] for link in root.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    parent_by_child = {
        joint.find("child").attrib["link"]: joint.find("parent").attrib["link"]
        for joint in joints.values()
    }

    errors: list[str] = []
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

    if errors:
        raise ComponentRegisterError("; ".join(errors))
    return {
        "register_id": register["register_id"],
        "status": "COMPONENT_REGISTER_AND_MECHANICAL_LOAD_PATHS_VALID",
        "sensor_installation_count": len(checked_sensors),
        "mechanical_subassembly_count": len(checked_subassemblies),
        "checked_sensor_installations": checked_sensors,
        "checked_mechanical_subassemblies": checked_subassemblies,
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
