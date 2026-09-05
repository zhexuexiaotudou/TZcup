#!/usr/bin/env python3
"""Fail-closed static validation for receiver and encoder hardware details."""

from __future__ import annotations

import argparse
import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config/high_fidelity_vehicle/encoder_feedback_realism_contract.yaml"
DEFAULT_REGISTER = ROOT / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml"
DEFAULT_PLATFORM = ROOT / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/a300_platform.xacro"
DEFAULT_SENSORS = ROOT / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/sensor_suite.xacro"
DEFAULT_CLEANING = ROOT / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/cleaning_mechanism.xacro"
DEFAULT_NODE = ROOT / "starter_ws/src/sanitation_vehicle_description/scripts/formal_encoder_feedback_publisher.py"
DEFAULT_LAUNCH = ROOT / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"


class EncoderFeedbackContractError(ValueError):
    pass


def validate(
    contract_path: Path = DEFAULT_CONTRACT,
    register_path: Path = DEFAULT_REGISTER,
    platform_path: Path = DEFAULT_PLATFORM,
    sensors_path: Path = DEFAULT_SENSORS,
    cleaning_path: Path = DEFAULT_CLEANING,
    node_path: Path = DEFAULT_NODE,
    launch_path: Path = DEFAULT_LAUNCH,
) -> dict:
    contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
    register = yaml.safe_load(register_path.read_text(encoding="utf-8"))
    platform = platform_path.read_text(encoding="utf-8")
    sensors = sensors_path.read_text(encoding="utf-8")
    cleaning = cleaning_path.read_text(encoding="utf-8")
    node = node_path.read_text(encoding="utf-8")
    launch = launch_path.read_text(encoding="utf-8")
    ast.parse(node, filename=str(node_path))
    ET.parse(platform_path)
    ET.parse(sensors_path)
    ET.parse(cleaning_path)

    errors: list[str] = []
    if contract.get("schema_version") != 1:
        errors.append("encoder contract schema_version must be 1")

    a300 = contract.get("a300_wheel_encoders", {})
    a300_quantizer = a300.get("simulation_quantizer", {})
    if a300.get("public_source", {}).get("published_encoder_resolution") is not False:
        errors.append("A300 public encoder resolution must remain explicitly unpublished")
    if a300_quantizer.get("counts_per_wheel_revolution") != 4096:
        errors.append("A300 engineering simulation quantizer must remain 4096 counts/rev")
    if "not_clearpath_specification" not in str(a300_quantizer.get("source_class", "")):
        errors.append("A300 simulation quantizer must not be presented as a Clearpath specification")

    pololu = contract.get("pololu_4694_encoders", {})
    if pololu.get("motor_shaft_counts_per_revolution") != 64:
        errors.append("Pololu 4694 motor encoder must remain 64 CPR")
    if pololu.get("nominal_gear_ratio") != 70:
        errors.append("selected Pololu 4694 nominal gear ratio must remain 70:1")
    if pololu.get("nominal_output_counts_per_revolution") != 4480:
        errors.append("Pololu output quantizer must equal 64 CPR times 70:1")

    required_platform = [
        "${name}_encoder_link",
        "${name}_encoder_mount_joint",
        "a300_encoder_cap.stl",
        '<mass value="1.92"/>',
        '<mass value="0.080"/>',
    ]
    required_sensors = [
        "zed_f9p_receiver_enclosure_link",
        "zed_f9p_module_reference_link",
        "zed_f9p_receiver_enclosure_mount_joint",
        "zed_f9p_module_reference_joint",
        'size="0.017 0.022 0.0024"',
    ]
    required_cleaning = [
        "${side}_side_brush_encoder_link",
        "central_roller_encoder_link",
        "pololu_37d_encoder_cap.stl",
        'mass="0.115"',
        'mass="0.015"',
    ]
    for label, source, tokens in (
        ("A300 platform", platform, required_platform),
        ("GNSS receiver", sensors, required_sensors),
        ("Pololu cleaning encoders", cleaning, required_cleaning),
    ):
        missing = [token for token in tokens if token not in source]
        if missing:
            errors.append(f"{label} is missing explicit hardware tokens: {missing}")

    expected_node_constants = {
        "A300_SIM_COUNTS_PER_WHEEL_REVOLUTION = 4096",
        "POLOLU_COUNTS_PER_OUTPUT_REVOLUTION = 64 * 70",
        '"/formal_vehicle/encoders/a300/counts"',
        '"/formal_vehicle/encoders/a300/joint_states"',
        '"/formal_vehicle/encoders/cleaning/counts"',
        '"/formal_vehicle/encoders/cleaning/joint_states"',
    }
    if missing := sorted(token for token in expected_node_constants if token not in node):
        errors.append(f"encoder publisher is missing contract constants/topics: {missing}")
    if "formal_encoder_feedback_publisher.py" not in launch:
        errors.append("formal vehicle launch does not start the encoder feedback publisher")

    topic_contracts = register.get("topic_contracts", {})
    expected_contract_ids = {
        "a300_encoder_counts",
        "a300_encoder_joint_states",
        "cleaning_encoder_counts",
        "cleaning_encoder_joint_states",
    }
    if missing := sorted(expected_contract_ids - set(topic_contracts)):
        errors.append(f"component register is missing encoder topic contracts: {missing}")
    sensors_by_id = {item.get("id"): item for item in register.get("sensor_installations", [])}
    receiver = sensors_by_id.get("ublox_zed_f9p_receiver", {})
    if receiver.get("sensor_link") != "zed_f9p_module_reference_link":
        errors.append("component register does not bind the explicit ZED-F9P receiver module")

    mesh_paths = [
        ROOT / "starter_ws/src/sanitation_vehicle_description/meshes/generated/platform/zed_f9p_receiver_enclosure.stl",
        ROOT / "starter_ws/src/sanitation_vehicle_description/meshes/generated/platform/zed_f9p_module_reference.stl",
        ROOT / "starter_ws/src/sanitation_vehicle_description/meshes/generated/platform/a300_encoder_cap.stl",
        ROOT / "starter_ws/src/sanitation_vehicle_description/meshes/project/cleaning/pololu_37d_encoder_cap.stl",
    ]
    missing_meshes = [str(path.relative_to(ROOT)) for path in mesh_paths if not path.is_file() or path.stat().st_size < 84]
    if missing_meshes:
        errors.append(f"receiver/encoder mesh assets missing or empty: {missing_meshes}")

    if errors:
        raise EncoderFeedbackContractError("; ".join(errors))
    return {
        "status": "FORMAL_RECEIVER_AND_ENCODER_HARDWARE_STATIC_VALID",
        "a300_encoder_link_count": len(a300.get("physical_links", [])),
        "pololu_encoder_link_count": len(pololu.get("physical_links", [])),
        "encoder_topic_contract_count": len(expected_contract_ids),
        "a300_hardware_resolution_pending": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    args = parser.parse_args()
    print(yaml.safe_dump(validate(contract_path=args.contract), sort_keys=False).strip())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

