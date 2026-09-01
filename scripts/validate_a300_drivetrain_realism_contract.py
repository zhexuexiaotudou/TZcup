#!/usr/bin/env python3
"""Validate the integrated A300 drivetrain single-authority contract fail closed."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config" / "high_fidelity_vehicle" / "a300_drivetrain_realism_contract.yaml"
LOCKED_CLEARPATH_COMMIT = "b0f6d920422ad302372a1c65e31d61648da884ed"
INTEGRATED_STATUS = "VEHICLE_INTEGRATION_IMPLEMENTED_RUNTIME_REVALIDATION_PENDING"


class A300DrivetrainContractError(ValueError):
    pass


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _close(actual: object, expected: float, tolerance: float = 1e-9) -> bool:
    return isinstance(actual, (int, float)) and math.isclose(float(actual), expected, abs_tol=tolerance)


def validate(contract_path: Path = DEFAULT_CONTRACT, root: Path = ROOT) -> dict:
    contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
    errors: list[str] = []

    if contract.get("status") != INTEGRATED_STATUS:
        errors.append("contract must identify integration with runtime revalidation pending")
    upstream = contract.get("upstream_lock", {})
    if upstream.get("commit") != LOCKED_CLEARPATH_COMMIT:
        errors.append("Clearpath source must remain locked to b0f6d920")
    if upstream.get("license") != "BSD-3-Clause":
        errors.append("vendored Clearpath source license must be BSD-3-Clause")
    required_upstream_files = {"description", "motor", "beam", "wheel", "wheel_chain", "control"}
    if set(upstream.get("source_files", {})) != required_upstream_files:
        errors.append("locked upstream file set is incomplete")

    published = contract.get("published_platform_boundaries", {})
    exact_boundaries = {
        "robot_mass_kg": 78.5,
        "payload_limit_kg": 101.5,
        "wheelbase_m": 0.512,
        "physical_wheel_radius_from_locked_description_m": 0.1651,
        "odometry_control_radius_from_locked_control_m": 0.1625,
        "tire_center_track_locked_description_m": 0.562,
        "maximum_vehicle_speed_mps": 2.0,
        "nominal_bus_voltage_v": 25.6,
        "continuous_current_per_motor_a": 17.0,
        "continuous_battery_current_a": 60.0,
        "aggregate_motor_output_power_w": 1080.0,
        "command_timeout_s": 0.5,
        "outdoor_wheel_mass_kg_each": 2.5,
        "outdoor_wheel_width_m": 0.1143,
    }
    if published.get("source_class") != "official_public":
        errors.append("published boundary section must be classified official_public")
    for key, expected in exact_boundaries.items():
        if not _close(published.get(key), expected):
            errors.append(f"published A300 boundary changed: {key}")
    if published.get("suspension_system_present") is not False:
        errors.append("A300 must not be represented as having a suspension system")

    chain = contract.get("future_rigid_body_chain", {})
    per_side = chain.get("per_side_links", [])
    if [item.get("role") for item in per_side] != ["suspension_spacer", "suspension_beam"]:
        errors.append("fixed spacer/structural-beam chain missing")
    if any(item.get("joint_type") != "fixed" for item in per_side):
        errors.append("A300 spacer and structural beam joints must be fixed")
    per_wheel = chain.get("per_wheel_links", {})
    if per_wheel.get("motor_joint_type") != "fixed" or per_wheel.get("wheel_joint_type") != "continuous":
        errors.append("motor must be fixed to beam and wheel must be continuous")
    if chain.get("no_compliance_joint_allowed") is not True:
        errors.append("contract must forbid invented suspension compliance")
    left_y = 0.192 + 0.0159 + 0.0095 + 0.0655
    if not _close(per_wheel.get("resulting_tire_center_y_abs_m"), left_y):
        errors.append("wheel-chain transforms do not resolve to the locked tire center")

    mesh_roles: list[str] = []
    for mesh in contract.get("vendored_meshes", []):
        mesh_roles.append(mesh.get("role", ""))
        path = root / mesh.get("path", "")
        if not path.is_file():
            errors.append(f"vendored drivetrain mesh missing: {mesh.get('path')}")
        elif _sha256(path) != mesh.get("sha256"):
            errors.append(f"vendored drivetrain mesh hash mismatch: {mesh.get('role')}")
    expected_mesh_roles = {"motor", "suspension_beam", "suspension_spacer", "outdoor_left", "outdoor_right"}
    if set(mesh_roles) != expected_mesh_roles:
        errors.append("vendored drivetrain mesh role set is incomplete")

    mass = contract.get("mass_partition", {})
    allocations = mass.get("allocations_kg", {})
    calculated_mass = sum(float(value) for value in allocations.values()) if allocations else 0.0
    if mass.get("source_class") != "engineering_allocation_preserving_official_total":
        errors.append("unpublished component masses must remain engineering allocations")
    if not _close(calculated_mass, 78.5) or not _close(mass.get("allocation_sum_kg"), 78.5):
        errors.append("A300 link mass allocation must conserve the published 78.5 kg total")
    warning = str(mass.get("warning", "")).lower()
    if "not official" not in warning or "measurement" not in warning:
        errors.append("mass truth boundary must require future measurement")

    plant = contract.get("plant_model", {})
    source_paths = [plant.get(key) for key in (
        "core_header", "core_source", "gazebo_candidate_source", "typed_command_adapter_source",
        "test_source", "candidate_compile_project"
    )]
    if not all(isinstance(path, str) and (root / path).is_file() for path in source_paths):
        errors.append("offline drivetrain core, candidate plugin or test source missing")
    required_effects = set(plant.get("required_effects", []))
    expected_effects = {
        "torque_speed_saturation", "per_motor_continuous_current_limit",
        "aggregate_battery_current_limit", "aggregate_output_power_limit",
        "resistive_brake_response_delay_and_ramp", "command_timeout",
        "emergency_stop", "any_motor_fault_global_inhibit", "invalid_input_fail_safe",
    }
    if required_effects != expected_effects:
        errors.append("plant effect contract is incomplete")
    engineering = plant.get("engineering_parameters_not_official", {})
    if not engineering or any(not isinstance(value, (int, float)) for value in engineering.values()):
        errors.append("unpublished motor/brake parameters must be explicit engineering values")
    boundary = plant.get("integration_boundary", {})
    if boundary.get("active_in_cmake") is not True:
        errors.append("validated drivetrain candidate must be wired into its package build")
    if any(boundary.get(key) is not True for key in
           ("loaded_by_current_vehicle", "current_runtime_behavior_changed")):
        errors.append("integrated plant must declare current vehicle runtime wiring")

    cmake = (root / "starter_ws" / "src" / "sanitation_gazebo_control" / "CMakeLists.txt").read_text(encoding="utf-8")
    for required_cmake_token in (
        "add_library(A300DrivetrainPlantSystem", "src/A300DrivetrainPlantCore.cc",
        "src/A300DrivetrainPlantSystem.cc", "a300_drivetrain_command_adapter",
    ):
        if required_cmake_token not in cmake:
            errors.append(f"candidate drivetrain package wiring missing: {required_cmake_token}")
    vehicle = root / "starter_ws" / "src" / "sanitation_vehicle_description"
    integration = root / "starter_ws" / "src" / "sanitation_formal_campus_integration"
    localization = root / "starter_ws" / "src" / "sanitation_localization"
    runtime_xacro = (vehicle / "urdf" / "formal_competition_vehicle.urdf.xacro").read_text(
        encoding="utf-8"
    )
    controller_yaml = (vehicle / "config" / "formal_vehicle_controllers.yaml").read_text(
        encoding="utf-8"
    )
    control_xacro = (
        vehicle / "urdf" / "high_fidelity" / "control_interfaces.xacro"
    ).read_text(encoding="utf-8")
    vehicle_launch = (vehicle / "launch" / "formal_vehicle_sim.launch.py").read_text(
        encoding="utf-8"
    )
    campus_launch = (integration / "launch" / "formal_campus.launch.py").read_text(
        encoding="utf-8"
    )
    topic_adapter = (
        integration / "sanitation_formal_campus_integration" / "topic_adapter.py"
    ).read_text(encoding="utf-8")
    fusion = yaml.safe_load(
        (localization / "config" / "formal_fusion.yaml").read_text(encoding="utf-8")
    )

    if runtime_xacro.count("libA300DrivetrainPlantSystem.so") != 1:
        errors.append("A300 drivetrain plant must be loaded exactly once by the formal vehicle")
    for required_token in (
        "front_left_wheel_joint", "front_right_wheel_joint",
        "rear_left_wheel_joint", "rear_right_wheel_joint",
        "<odometry_topic>/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/odom</odometry_topic>",
        "<odometry_frame_id>odom</odometry_frame_id>",
        "<odometry_child_frame_id>base_footprint</odometry_child_frame_id>",
    ):
        if required_token not in runtime_xacro:
            errors.append(f"formal plant plugin wiring missing: {required_token}")
    if "base_controller:" in controller_yaml or "DiffDriveController" in controller_yaml:
        errors.append("legacy diff-drive controller must be absent")
    for wheel in (
        "front_left_wheel_joint", "front_right_wheel_joint",
        "rear_left_wheel_joint", "rear_right_wheel_joint",
    ):
        if f'<xacro:hf_state_only_joint name="{wheel}"/>' not in control_xacro:
            errors.append(f"wheel must expose state only to ros2_control: {wheel}")
    wheel_block = control_xacro.split("A300DrivetrainPlantSystem", 1)[-1].split(
        "Gazebo must start", 1
    )[0]
    if "command_interface" in wheel_block or "hf_velocity_joint" in wheel_block:
        errors.append("wheel ros2_control block must not contain command interfaces")
    for required_token in (
        'executable="a300_drivetrain_command_adapter"',
        'name="a300_drivetrain_bridge"',
        '"/odom/unfiltered"',
        '"start_localization"',
    ):
        if required_token not in vehicle_launch:
            errors.append(f"formal vehicle launch wiring missing: {required_token}")
    if '"base_controller"' in vehicle_launch:
        errors.append("formal vehicle launch must not spawn base_controller")
    if "formal_campus_base_controller_spawner" in campus_launch:
        errors.append("formal campus launch must not retain a base controller spawner")
    for retired_token in (
        "relay_legacy_base_odometry",
        "publish_selected_odom",
        "base_controller/odom",
        "odom/unfiltered",
    ):
        if retired_token in topic_adapter:
            errors.append(
                f"sensor topic adapter must not contain an odometry relay: {retired_token}"
            )
    if "publish_selected_odom" in campus_launch or "relay_legacy_base_odometry" in campus_launch:
        errors.append("formal campus must not expose retired odometry relay controls")
    local = fusion.get("local_ekf", {}).get("ros__parameters", {})
    if (
        local.get("odom0") != "/odom/unfiltered"
        or local.get("world_frame") != "odom"
        or local.get("base_link_frame") != "base_footprint"
        or local.get("publish_tf") is not True
    ):
        errors.append("local EKF must uniquely select raw plant odometry and publish odom TF")
    plan = boundary.get("final_single_authority_plan", {})
    required_plan = {
        "diff_drive_controller_retained": False,
        "plant_raw_odometry_ros_topic": "/odom/unfiltered",
        "plant_publishes_tf": False,
        "selected_odometry_topic": "/odom",
        "selected_odometry_publisher": "/local_ekf",
        "odom_to_base_tf_publisher": "/local_ekf",
        "local_ekf_base_link_frame": "base_footprint",
        "legacy_topic_adapter_odometry_relay_present": False,
    }
    for key, expected in required_plan.items():
        if plan.get(key) != expected:
            errors.append(f"single-authority plan mismatch: {key}")

    if errors:
        raise A300DrivetrainContractError("; ".join(errors))
    return {
        "contract_id": contract["contract_id"],
        "status": "A300_DRIVETRAIN_VEHICLE_INTEGRATION_STATIC_VALID",
        "upstream_commit": upstream["commit"],
        "mesh_count": len(mesh_roles),
        "published_mass_kg": published["robot_mass_kg"],
        "allocated_mass_kg": calculated_mass,
        "runtime_integrated": True,
        "runtime_revalidation_pending": True,
        "truth_boundary": "unpublished internal motor and fixed-link parameters remain engineering allocations",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--write-report", type=Path)
    args = parser.parse_args()
    result = validate(args.contract)
    if args.write_report:
        args.write_report.parent.mkdir(parents=True, exist_ok=True)
        args.write_report.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
