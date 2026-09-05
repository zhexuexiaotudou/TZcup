#!/usr/bin/env python3
"""Validate the source-level cleaning actuator motor realism contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "config/high_fidelity_vehicle/cleaning_actuator_motor_realism_contract.yaml"


def validate() -> dict:
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    core = (ROOT / "starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorMotorCore.cc").read_text(encoding="utf-8")
    plugin = (ROOT / "starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorMotorSystem.cc").read_text(encoding="utf-8")
    xacro = (ROOT / "starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro").read_text(encoding="utf-8")
    cleaning = (ROOT / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/cleaning_mechanism.xacro").read_text(encoding="utf-8")
    interfaces = (ROOT / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/control_interfaces.xacro").read_text(encoding="utf-8")
    launch = (ROOT / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py").read_text(encoding="utf-8")
    scalar_bridge = (ROOT / "starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorScalarNativeBridge.cc").read_text(encoding="utf-8")
    vector_bridge = (ROOT / "starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorVectorBridge.cc").read_text(encoding="utf-8")
    safety = (ROOT / "starter_ws/src/sanitation_safety/sanitation_safety/whole_vehicle_safety_core.py").read_text(encoding="utf-8")
    core_test = (ROOT / "starter_ws/src/sanitation_gazebo_control/test/test_cleaning_actuator_motor_core.cc").read_text(encoding="utf-8")
    runner = (ROOT / "scripts/run_formal_cleaning_actuator_motor_runtime.sh").read_text(encoding="utf-8")

    checks = {
        "five_actuators_ordered": len(contract["actuator_order"]) == 5,
        "pololu_public_limits": (
            contract["actuators"]["pololu_4694"]["no_load_speed_rpm"] == 140.0
            and contract["actuators"]["pololu_4694"]["stall_current_a"] == 3.0
            and contract["actuators"]["pololu_4694"]["rated_current_a"] == 0.75
        ),
        "p16_public_limits": (
            contract["actuators"]["actuonix_p16"]["no_load_speed_m_s"] == 0.0048
            and contract["actuators"]["actuonix_p16"]["stall_current_a"] == 1.0
            and contract["actuators"]["actuonix_p16"]["maximum_load_n"] == 300.0
        ),
        "jabsco_public_limits": (
            contract["actuators"]["jabsco_q402j_118s_3a"]["rated_current_a"] == 6.0
            and contract["actuators"]["jabsco_q402j_118s_3a"]["stall_current_a"] == 10.0
            and contract["actuators"]["jabsco_q402j_118s_3a"]["thermal_cutout"] is True
        ),
        "core_has_thermal_and_stall_latches": all(
            token in core
            for token in (
                "latched_stall_", "latched_overtemperature_",
                "thermal_time_constant_s", "estimated_output_load",
            )
        ),
        "observer_never_writes_joint_commands": all(
            token not in plugin
            for token in (
                "components::JointForceCmd", "components::JointVelocityCmd",
                "components::JointPositionCmd", "CreateComponent(",
            )
        ),
        "plugin_loaded": "libCleaningActuatorMotorSystem.so" in xacro,
        "real_joint_limits": all(
            token in cleaning + interfaces
            for token in ("14.660766", "3.040473", "0.0048", "300.0")
        ),
        "runtime_bridge_and_mirror": (
            all(
                token in launch
                for token in (
                    "cleaning_actuator_command_mirror",
                    'executable="cleaning_actuator_scalar_native_bridge"',
                    'name="cleaning_actuator_scalar_bridge"',
                    'executable="cleaning_actuator_vector_bridge"',
                    'name="cleaning_actuator_motor_bridge"',
                )
            )
            and "NativeBridgeSupport(\"cleaning_actuator_scalar_native_bridge\")" in scalar_bridge
            and all(
                token in scalar_bridge
                for token in (
                    "RosToGazeboEndpoint<std_msgs::msg::Float64, gz::msgs::Double>",
                    "RosToGazeboEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean>",
                    "GazeboToRosEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean>",
                    "GazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double>",
                    "cleaning_motors/command/lift_position",
                    "cleaning_motors/command/enable",
                    "cleaning_motors/command/reset_faults",
                    "cleaning_motors/fault_active",
                    "cleaning_motors/total_current_a",
                    "cleaning_motors/total_power_w",
                )
            )
            and "cleaning_motors/motor_temperature_c" in vector_bridge
            and "cleaning_motors/motor_current_a" in vector_bridge
            and "cleaning_motors/estimated_output_load" in vector_bridge
            and "cleaning_motors/telemetry_snapshot" in vector_bridge
            and "cleaning_motors/status_json@std_msgs/msg/String" not in launch
        ),
        "whole_vehicle_fail_closed": all(
            token in safety
            for token in (
                "CLEANING_MOTOR_FAULT_UNAVAILABLE",
                "CLEANING_MOTOR_FAULT_ACTIVE",
                "set_cleaning_motor_fault",
            )
        ),
        "live_runner_uses_physical_travel_stop_and_frozen_overlay": all(
            token in runner
            for token in (
                "FORMAL_VEHICLE_RUNTIME_WS",
                "--exercise-live",
                "--snapshot-manifest",
                "formal_vehicle_sim.launch.py",
            )
        ),
        "thermal_core_test_keeps_production_parameters": all(
            token in core_test
            for token in (
                "CleaningActuatorMotorCore thermalCore(parameters)",
                "CleaningMotorFault::kOvertemperature",
                "hot overtemperature latch must reject",
                "idle motor must cool below",
            )
        ) and contract["thermal_acceptance"]["production_parameters_modified"] is False,
        "thermal_live_boundary_truthful": (
            contract["thermal_acceptance"]["live_overtemperature_required"] is False
            and "must not shorten production thermal time constants"
            in contract["thermal_acceptance"]["boundary"]
        ),
        "runtime_acceptance_pending_truthful": (
            contract["runtime_acceptance"]["status"]
            == "pending_no_gazebo_run_in_this_change"
        ),
    }
    failed = [name for name, passed in checks.items() if not passed]
    return {
        "schema_version": 1,
        "contract_id": contract["contract_id"],
        "passed": not failed,
        "checks": checks,
        "failed_checks": failed,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = validate()
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
