#!/usr/bin/env python3
"""Fail-closed checks for offline A300 power and service hardware design."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config" / "high_fidelity_vehicle" / "a300_power_and_service_hardware_contract.yaml"
DRIVETRAIN_CONTRACT = ROOT / "config" / "high_fidelity_vehicle" / "a300_drivetrain_realism_contract.yaml"


class A300PowerServiceContractError(ValueError):
    pass


def _close(actual: object, expected: float, tolerance: float = 1e-9) -> bool:
    return isinstance(actual, (int, float)) and math.isclose(float(actual), expected, abs_tol=tolerance)


def _assembly_mass(links: list[dict]) -> float:
    return sum(float(link["mass_kg"]) for link in links)


DEFAULT_URDF = ROOT / "reports" / "engineering" / "formal_competition_vehicle.urdf"


def _mass(link: ET.Element) -> float:
    node = link.find("./inertial/mass")
    return float(node.attrib["value"]) if node is not None else 0.0


def validate(contract_path: Path = DEFAULT_CONTRACT, urdf_path: Path = DEFAULT_URDF) -> dict:
    contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
    drivetrain = yaml.safe_load(DRIVETRAIN_CONTRACT.read_text(encoding="utf-8"))
    errors: list[str] = []

    if contract.get("status") != "RUNTIME_INTEGRATED_PENDING_FULL_ACCEPTANCE":
        errors.append("power/service hardware integration status is not frozen")
    truth = contract.get("truth_boundary", {})
    official = truth.get("official_public_facts", {})
    expected_official = {
        "nominal_voltage_v": 25.6,
        "capacity_40ah_wh": 1024.0,
        "capacity_40ah_pack_count": 2,
        "a300_40ah_vehicle_mass_kg": 78.5,
        "a300_80ah_vehicle_mass_kg": 93.5,
    }
    for key, expected in expected_official.items():
        if not _close(official.get(key), expected):
            errors.append(f"official A300 power boundary changed: {key}")
    inferred = truth.get("derived_not_published_single_pack_mass", {})
    if inferred.get("source_class") != "official_whole_vehicle_difference_engineering_inference":
        errors.append("single-pack mass must remain an engineering inference")
    if not _close(inferred.get("vehicle_mass_difference_kg"), 15.0):
        errors.append("40-to-80 Ah whole-vehicle mass difference must remain 15 kg")
    if not _close(inferred.get("inferred_mass_per_pack_kg"), 7.5):
        errors.append("single-pack differential inference must remain 7.5 kg")
    if "clearpath_publishes_7_5kg_single_pack_mass" not in truth.get("forbidden_claims", []):
        errors.append("contract must forbid treating inferred pack mass as published")

    transfer = contract.get("a300_curb_mass_transfer", {})
    unchanged = transfer.get("unchanged_allocations_kg", {})
    recomputed = (
        float(transfer.get("revised_chassis_and_internal_structure_excluding_batteries_kg", 0.0))
        + float(transfer.get("explicit_two_battery_assemblies_kg", 0.0))
        + sum(float(value) for value in unchanged.values())
    )
    if not _close(recomputed, 78.5) or not _close(transfer.get("recomputed_total_kg"), 78.5):
        errors.append("explicit batteries must preserve the published 78.5 kg curb mass")
    if transfer.get("double_counting_forbidden") is not True:
        errors.append("battery mass double-counting must be forbidden")
    drivetrain_allocations = drivetrain.get("mass_partition", {}).get("allocations_kg", {})
    if not _close(drivetrain_allocations.get("two_battery_assemblies"), 15.0):
        errors.append("drivetrain mass contract does not transfer 15 kg into explicit batteries")
    if not _close(
        drivetrain_allocations.get("chassis_and_internal_structure_excluding_batteries"), 35.5
    ):
        errors.append("drivetrain chassis mass was not reduced by the two battery packs")

    battery = contract.get("battery_and_bms_design", {})
    assemblies = battery.get("assemblies", [])
    if len(assemblies) != 2:
        errors.append("40 Ah A300 design requires exactly two battery assemblies")
    pack_boxes: list[tuple[float, float, float, float, float, float]] = []
    for item in assemblies:
        if item.get("pack_joint_type") != "fixed" or item.get("bms_joint_type") != "fixed":
            errors.append(f"battery and BMS joints must be fixed: {item.get('id')}")
        expected_mass = float(item.get("pack_cells_case_mass_kg", 0.0)) + float(item.get("bms_mass_kg", 0.0))
        if not _close(expected_mass, 7.5) or not _close(item.get("assembly_mass_kg"), 7.5):
            errors.append(f"battery/BMS suballocation must total inferred 7.5 kg: {item.get('id')}")
        xyz = item.get("xyz_m", [])
        size = item.get("envelope_m", [])
        if len(xyz) != 3 or len(size) != 3:
            errors.append(f"battery envelope is incomplete: {item.get('id')}")
            continue
        bounds = tuple(
            value
            for center, extent in zip(xyz, size)
            for value in (float(center) - float(extent) / 2.0, float(center) + float(extent) / 2.0)
        )
        pack_boxes.append(bounds)
        if bounds[0] < -0.43 or bounds[1] > 0.43 or bounds[2] < -0.189 or bounds[3] > 0.189 or bounds[4] < 0.0 or bounds[5] > 0.22956:
            errors.append(f"battery envelope leaves the locked chassis envelope: {item.get('id')}")
    if len(pack_boxes) == 2:
        x_overlap = min(pack_boxes[0][1], pack_boxes[1][1]) - max(pack_boxes[0][0], pack_boxes[1][0])
        y_overlap = min(pack_boxes[0][3], pack_boxes[1][3]) - max(pack_boxes[0][2], pack_boxes[1][2])
        z_overlap = min(pack_boxes[0][5], pack_boxes[1][5]) - max(pack_boxes[0][4], pack_boxes[1][4])
        if x_overlap > 0 and y_overlap > 0 and z_overlap > 0:
            errors.append("left and right battery envelopes overlap")
    if battery.get("bms_mass_source_class") != "engineering_suballocation_inside_inferred_pack_mass":
        errors.append("BMS mass must remain a non-official suballocation")
    fail_safe = set(battery.get("fail_safe_rules", []))
    if "either_pack_fault_inhibits_complete_traction_system" not in fail_safe:
        errors.append("either-pack fault must inhibit the whole drivetrain")
    if "charge_connected_inhibits_traction" not in fail_safe:
        errors.append("charge-connected state must inhibit traction")

    charge = contract.get("charge_interface_design", {})
    charge_links = charge.get("links", [])
    if not _close(_assembly_mass(charge_links), charge.get("assembly_mass_kg", -1.0)):
        errors.append("charge-interface link masses do not match assembly mass")
    charge_joints = {link.get("name"): link for link in charge_links}
    door = charge_joints.get("charge_port_door_link", {})
    lock = charge_joints.get("charge_connector_lock_link", {})
    if door.get("joint_type") != "revolute" or not _close(door.get("upper_rad"), 1.92):
        errors.append("charge door must be a bounded revolute joint")
    if lock.get("joint_type") != "prismatic" or not _close(lock.get("upper_m"), 0.006):
        errors.append("charge connector lock must be a physical bounded prismatic latch")
    charge_interfaces = {
        item.get("topic"): item
        for item in charge.get("implemented_interfaces", [])
    }
    charge_contact = charge_interfaces.get(
        "/formal_vehicle/service/raw/charge_plug_contact", {}
    )
    if (
        charge_contact.get("type") != "ros_gz_interfaces/msg/Contacts"
        or charge_contact.get("direction") != "subscription"
    ):
        errors.append("charge plug presence must use the raw Contacts subscription")
    if "/formal_vehicle/power/charge_plug_present" in charge_interfaces:
        errors.append("synthetic charge_plug_present Boolean is prohibited")
    charge_joint_state = charge_interfaces.get("/joint_states", {})
    if set(charge_joint_state.get("joints", [])) != {
        "charge_port_door_hinge_joint",
        "charge_connector_lock_joint",
    }:
        errors.append("charge door and connector lock must come from joint_states")
    permit_rule = str(charge.get("permit_rule", ""))
    for phrase in ("open charge door", "plug contact", "engaged lock", "stationary", "traction inhibited", "no BMS fault"):
        if phrase not in permit_rule:
            errors.append(f"charge permit rule missing: {phrase}")

    drain = contract.get("wastewater_drain_valve_design", {})
    drain_links = drain.get("links", [])
    if not _close(_assembly_mass(drain_links), drain.get("assembly_mass_kg", -1.0)):
        errors.append("drain-valve link masses do not match assembly mass")
    required_drain_links = {
        "wastewater_drain_pipe_link", "wastewater_drain_valve_body_link",
        "wastewater_drain_valve_ball_link", "wastewater_drain_valve_actuator_link",
        "wastewater_drain_service_cap_link", "wastewater_drain_coupling_link",
    }
    if {link.get("name") for link in drain_links} != required_drain_links:
        errors.append("drain physical link chain is incomplete")
    valve = next((link for link in drain_links if link.get("name") == "wastewater_drain_valve_ball_link"), {})
    if valve.get("joint_type") != "revolute" or not _close(valve.get("upper_rad"), math.pi / 2, 1e-8):
        errors.append("drain valve ball must have a 90 degree physical joint")
    if drain.get("actuation") != "24V_spring_return_normally_closed_engineering_candidate":
        errors.append("drain valve must fail closed through a spring-return actuator")
    if not _close(drain.get("fail_safe_position_rad"), 0.0):
        errors.append("drain valve fail-safe position must be closed")
    drain_interfaces = {
        item.get("topic"): item
        for item in drain.get("implemented_interfaces", [])
    }
    drain_contact = drain_interfaces.get(
        "/formal_vehicle/service/raw/drain_hose_contact", {}
    )
    if (
        drain_contact.get("type") != "ros_gz_interfaces/msg/Contacts"
        or drain_contact.get("direction") != "subscription"
    ):
        errors.append("drain hose presence must use the raw Contacts subscription")
    drain_joint_state = drain_interfaces.get("/joint_states", {})
    if "wastewater_drain_service_cap_joint" not in drain_joint_state.get("joints", []):
        errors.append("drain service cap state must come from joint_states")
    for phrase in ("stationary", "cleaning disabled", "recovery pump stopped", "connected hose"):
        if phrase not in str(drain.get("permit_rule", "")):
            errors.append(f"drain permit rule missing: {phrase}")
    if "no CFD claim" not in str(drain.get("hydraulic_model_boundary", "")):
        errors.append("simplified drain hydraulics must explicitly forbid a CFD claim")

    required_tests = set(contract.get("future_tests", []))
    if len(required_tests) < 15 or "integrated_vehicle_references_physical_power_and_service_hardware" not in required_tests:
        errors.append("future physical and safety test plan is incomplete")

    if not urdf_path.is_file():
        errors.append(f"expanded integrated URDF is missing: {urdf_path}")
    else:
        root = ET.parse(urdf_path).getroot()
        links = {node.attrib["name"]: node for node in root.findall("link")}
        joints = {node.attrib["name"]: node for node in root.findall("joint")}
        required_links = {
            "a300_left_battery_pack_link", "a300_left_battery_bms_link",
            "a300_right_battery_pack_link", "a300_right_battery_bms_link",
            "charge_port_housing_link", "charge_port_door_link",
            "charge_receptacle_link", "charge_connector_lock_link",
            "emergency_stop_housing_link", "emergency_stop_plunger_link",
            *required_drain_links,
        }
        missing = sorted(required_links - links.keys())
        if missing:
            errors.append(f"integrated physical links missing: {missing}")

        curb_parts = {
            "base_link": 35.5,
            "payload_deck_link": 4.0,
            "left_suspension_beam_spacer_link": 0.5,
            "right_suspension_beam_spacer_link": 0.5,
            "left_suspension_beam_link": 2.5,
            "right_suspension_beam_link": 2.5,
            # Each 2.00 kg drive-module allocation is represented by a
            # 1.92 kg motor body plus a separate 0.08 kg encoder end cap.
            "front_left_motor_link": 1.92,
            "front_right_motor_link": 1.92,
            "rear_left_motor_link": 1.92,
            "rear_right_motor_link": 1.92,
            "front_left_encoder_link": 0.08,
            "front_right_encoder_link": 0.08,
            "rear_left_encoder_link": 0.08,
            "rear_right_encoder_link": 0.08,
            "front_left_wheel_link": 2.5,
            "front_right_wheel_link": 2.5,
            "rear_left_wheel_link": 2.5,
            "rear_right_wheel_link": 2.5,
            "a300_left_battery_pack_link": 7.35,
            "a300_left_battery_bms_link": 0.15,
            "a300_right_battery_pack_link": 7.35,
            "a300_right_battery_bms_link": 0.15,
        }
        actual_curb = 0.0
        for name, expected in curb_parts.items():
            if name not in links:
                continue
            actual = _mass(links[name])
            actual_curb += actual
            if not _close(actual, expected, 1e-8):
                errors.append(f"integrated curb allocation changed: {name}={actual}")
        if not _close(actual_curb, 78.5, 1e-8):
            errors.append(f"integrated A300 curb mass is {actual_curb}, expected 78.5")

        joint_expectations = {
            "charge_port_door_hinge_joint": ("revolute", 0.0, 1.92),
            "charge_connector_lock_joint": ("prismatic", 0.0, 0.006),
            "emergency_stop_plunger_joint": ("prismatic", 0.0, 0.006),
            "wastewater_drain_valve_joint": ("revolute", 0.0, math.pi / 2.0),
            "wastewater_drain_service_cap_joint": ("revolute", 0.0, 2.4),
        }
        for name, (kind, lower, upper) in joint_expectations.items():
            node = joints.get(name)
            if node is None:
                errors.append(f"integrated service joint missing: {name}")
                continue
            limit = node.find("limit")
            if node.attrib.get("type") != kind or limit is None:
                errors.append(f"integrated service joint type/limit invalid: {name}")
                continue
            if not _close(float(limit.attrib["lower"]), lower, 1e-8) or not _close(float(limit.attrib["upper"]), upper, 1e-8):
                errors.append(f"integrated service joint range invalid: {name}")

        control_text = (ROOT / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/control_interfaces.xacro").read_text(encoding="utf-8")
        if 'name="wastewater_drain_valve_joint"' not in control_text:
            errors.append("powered drain valve is missing from ros2_control")
        for passive in ("charge_port_door_hinge_joint", "charge_connector_lock_joint", "emergency_stop_plunger_joint", "wastewater_drain_service_cap_joint"):
            if f'<xacro:hf_position_joint name="{passive}"' in control_text:
                errors.append(f"passive service joint is incorrectly commanded: {passive}")

        mesh_root = ROOT / "starter_ws/src/sanitation_vehicle_description/meshes/project/service"
        if len(list(mesh_root.glob("*.stl"))) < 14:
            errors.append("manufactured service visual mesh set is incomplete")

    if errors:
        raise A300PowerServiceContractError("; ".join(errors))
    return {
        "contract_id": contract["contract_id"],
        "status": "A300_POWER_AND_SERVICE_HARDWARE_INTEGRATED_STATIC_VALID",
        "battery_pack_count": len(assemblies),
        "inferred_pack_mass_kg": inferred["inferred_mass_per_pack_kg"],
        "explicit_battery_mass_kg": transfer["explicit_two_battery_assemblies_kg"],
        "a300_curb_mass_kg": recomputed,
        "charge_link_count": len(charge_links),
        "drain_link_count": len(drain_links),
        "runtime_integrated": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--write-report", type=Path)
    args = parser.parse_args()
    result = validate(args.contract, args.urdf)
    if args.write_report:
        args.write_report.parent.mkdir(parents=True, exist_ok=True)
        args.write_report.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
