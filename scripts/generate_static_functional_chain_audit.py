#!/usr/bin/env python3
"""Produce a fail-closed, source-only audit of the formal vehicle functions.

This is deliberately not an acceptance runner.  It follows the declared
Xacro/URDF, controller, Gazebo-system, launch and runtime-validator paths, but
does not start ROS, Gazebo, Docker or WSL.  A missing source dependency, or an
explicitly isolated payload path, is reported as ``BLOCKED`` rather than being
promoted by a matching name.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from validate_s100p_mechanical_electrical_evidence import (
    BLOCKED_STATUS as S100P_BLOCKED_STATUS,
    DEFAULT as S100P_EVIDENCE_DEFAULT,
    validate as validate_s100p_evidence,
)


ROOT = Path(__file__).resolve().parents[1]
REPORT_ID = "tzcup_formal_vehicle_static_functional_chain_audit_v1"
VALID_STATUSES = {"STATIC_CLOSED", "BLOCKED"}
REQUIRED_ITEM_IDS = (
    "mobility_forward_and_brake",
    "six_axis_arm_and_gripper",
    "cube_pick_and_rear_dry_bin_containment",
    "dry_garbage_increases_vehicle_mass",
    "ground_dirt_coverage",
    "brush_squeegee_water_to_wastewater_tank",
    "sensor_single_line_lidar",
    "sensor_mid360",
    "sensor_front_rgbd",
    "sensor_rear_fisheyes",
    "sensor_end_effector_stereo",
    "sensor_gnss",
    "sensor_wheel_speed",
)
SUPPLEMENTAL_ITEM_IDS = (
    "s100p_installation_and_low_voltage_power",
    "obstacle_avoidance_chain",
)


@dataclass(frozen=True)
class Source:
    path: str
    text: str


def _read(root: Path, path: str) -> Source:
    target = root / path
    try:
        return Source(path, target.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return Source(path, "")


def _line(source: Source, token: str) -> int | None:
    for number, value in enumerate(source.text.splitlines(), start=1):
        if token in value:
            return number
    return None


def _evidence(source: Source, token: str) -> dict[str, object]:
    return {"path": source.path, "line": _line(source, token), "token": token}


def _check(source: Source, *tokens: str) -> bool:
    return bool(source.text) and all(token in source.text for token in tokens)


def _item(
    item_id: str,
    requirement: str,
    checks: Iterable[tuple[bool, Source, str]],
    *,
    blocked_reason: str | None = None,
    physical_semantics: str,
    placeholder_indicators: Iterable[str] = (),
) -> dict[str, object]:
    checks = tuple(checks)
    closed = blocked_reason is None and all(passed for passed, _, _ in checks)
    return {
        "id": item_id,
        "requirement": requirement,
        "status": "STATIC_CLOSED" if closed else "BLOCKED",
        "checks": [
            {"passed": passed, "evidence": _evidence(source, token)}
            for passed, source, token in checks
        ],
        "blocked_reason": blocked_reason
        or (None if closed else "required static evidence is absent"),
        "physical_semantics": physical_semantics,
        "placeholder_indicators": list(placeholder_indicators),
    }


def _s100p_evidence_conclusion(root: Path) -> dict[str, object]:
    """Read and validate the S100P evidence contract without touching hardware."""
    path = root / S100P_EVIDENCE_DEFAULT.relative_to(ROOT)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        validate_s100p_evidence(payload, root)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return {
            "path": str(path.relative_to(root)).replace("\\", "/"),
            "validator": "validate_s100p_mechanical_electrical_evidence.py",
            "validator_complete": False,
            "status": "INVALID_OR_MISSING_EVIDENCE_CONTRACT",
            "acceptance": {},
            "blocked_gates": [],
            "error": str(exc),
        }
    acceptance = payload.get("acceptance")
    return {
        "path": str(path.relative_to(root)).replace("\\", "/"),
        "validator": "validate_s100p_mechanical_electrical_evidence.py",
        "validator_complete": True,
        "status": payload.get("status"),
        "acceptance": acceptance,
        "blocked_gates": payload.get("blocked_gates"),
        "error": None,
    }


def audit(root: Path = ROOT) -> dict[str, object]:
    """Build the audit result without executing a simulator or shell command."""

    vehicle = _read(root, "starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro")
    control = _read(root, "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/control_interfaces.xacro")
    storage = _read(root, "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/storage_system.xacro")
    material_cube = _read(root, "starter_ws/src/sanitation_manipulation/urdf/material_cube.urdf.xacro")
    sensors = _read(root, "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/sensor_suite.xacro")
    controllers = _read(root, "starter_ws/src/sanitation_vehicle_description/config/formal_vehicle_controllers.yaml")
    sim_launch = _read(root, "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py")
    cube_launch = _read(root, "starter_ws/src/sanitation_manipulation/launch/formal_cube_pick_place.launch.py")
    sensor_bridge = _read(root, "starter_ws/src/sanitation_vehicle_description/config/formal_high_bandwidth_sensor_bridge.yaml")
    drivetrain = _read(root, "starter_ws/src/sanitation_gazebo_control/src/A300DrivetrainPlantSystem.cc")
    dry_bin = _read(root, "starter_ws/src/sanitation_gazebo_control/src/DryBinMonitorSystem.cc")
    ground_dirt = _read(root, "starter_ws/src/sanitation_gazebo_control/src/GroundDirtCleaningSystem.cc")
    water = _read(root, "starter_ws/src/sanitation_gazebo_control/src/WaterRecoverySystem.cc")
    payload = _read(root, "starter_ws/src/sanitation_gazebo_control/src/DynamicPayloadSystem.cc")
    dry_accounting_contract = _read(root, "config/high_fidelity_vehicle/dry_payload_accounting_contract.yaml")
    twenty_cube_launch = _read(root, "starter_ws/src/sanitation_manipulation/launch/formal_20_cube_pick_place.launch.py")
    water_world = _read(root, "starter_ws/src/sanitation_vehicle_description/worlds/formal_vehicle_validation.sdf")
    run_cube = _read(root, "scripts/run_formal_cube_pick_place_runtime.sh")
    run_dirt = _read(root, "scripts/run_formal_ground_dirt_cleaning_runtime.sh")
    run_water = _read(root, "scripts/run_formal_water_recovery_runtime.sh")
    mobility_metrics = _read(root, "scripts/formal_vehicle_mobility_metrics.py")
    gazebo_cmake = _read(root, "starter_ws/src/sanitation_gazebo_control/CMakeLists.txt")
    platform = _read(root, "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/a300_platform.xacro")
    component_register = _read(root, "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml")
    s100_launch = _read(root, "starter_ws/src/sanitation_perception/launch/formal_s100p_open_vocab.launch.py")
    nav_launch = _read(root, "starter_ws/src/sanitation_navigation/launch/navigation.launch.py")
    nav_config = _read(root, "starter_ws/src/sanitation_navigation/config/nav2.yaml")
    velocity_gate = _read(root, "starter_ws/src/sanitation_safety/sanitation_safety/velocity_gate.py")
    dynamic_obstacle_audit = _read(root, "scripts/audit_formal_dynamic_obstacle_readiness.py")
    s100p_evidence = _s100p_evidence_conclusion(root)
    s100p_evidence_source = _read(root, str(s100p_evidence["path"]))

    items = [
        _item(
            "mobility_forward_and_brake",
            "A ROS velocity command reaches all four wheel effort joints and zero/E-stop has a physical stop path.",
            (
                (_check(vehicle, 'filename="libA300DrivetrainPlantSystem.so"', "front_left_wheel_joint", "rear_right_wheel_joint"), vehicle, 'filename="libA300DrivetrainPlantSystem.so"'),
                (_check(vehicle, "<command_topic>/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/cmd_vel</command_topic>", "<emergency_stop_topic>"), vehicle, "<command_topic>"),
                (_check(drivetrain, "JointForceCmd", "resistive_brake_active", "OnEmergencyStop"), drivetrain, "resistive_brake_active"),
                (_check(gazebo_cmake, "add_library(A300DrivetrainPlantSystem SHARED"), gazebo_cmake, "add_library(A300DrivetrainPlantSystem SHARED"),
                (_check(sim_launch, "a300_drivetrain_command_adapter", "a300_drivetrain_bridge", "/odom/unfiltered"), sim_launch, "a300_drivetrain_bridge"),
                (_check(mobility_metrics, "ground_truth_forward_motion", "vehicle_stopped_after_zero_command"), mobility_metrics, "vehicle_stopped_after_zero_command"),
            ),
            physical_semantics="Four physical wheel joints receive the effort-plant command; braking is an explicit plant state rather than a controller-only label.",
        ),
        _item(
            "six_axis_arm_and_gripper",
            "Six commanded UR5e joints and one physical Robotiq actuator are controller-backed and launched with the cube scenario.",
            (
                (_check(control, "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"), control, "shoulder_pan_joint"),
                (_check(controllers, "arm_controller:", "gripper_controller:", "robotiq_85_left_knuckle_joint"), controllers, "gripper_controller:"),
                (_check(vehicle, 'filename="libGripperMimicEffortSystem.so"', "<master_joint>robotiq_85_left_knuckle_joint</master_joint>"), vehicle, "libGripperMimicEffortSystem.so"),
                (_check(gazebo_cmake, "add_library(GripperMimicEffortSystem SHARED"), gazebo_cmake, "add_library(GripperMimicEffortSystem SHARED"),
                (_check(cube_launch, '"arm_controller"', '"gripper_controller"', "formal_cube_manipulation"), cube_launch, "formal_cube_manipulation"),
            ),
            physical_semantics="Six arm position interfaces plus one gripper actuator drive named URDF joints; the passive finger linkage has a dedicated Gazebo effort system.",
        ),
        _item(
            "cube_pick_and_rear_dry_bin_containment",
            "A material-specific 30 mm rigid cube launch and finger/deposition/floor contact bridges reach the physical dry-bin monitor.",
            (
                (_check(material_cube, '<xacro:property name="edge" value="0.030"/>', '<collision name="cube_collision">', '<mass value="${mass}"/>'), material_cube, 'name="edge" value="0.030"'),
                (_check(cube_launch, "material_cube.urdf.xacro", "dry_deposit_contact", "dry_bin_floor_contact"), cube_launch, "dry_deposit_contact"),
                (_check(storage, 'joint name="dry_bin_mount_joint"', 'origin xyz="-0.205 ${dry_center_y} 0.200"'), storage, "dry_bin_mount_joint"),
                (_check(dry_bin, "contained_object_count", "physical_contained_mass_kg", 'name != "material_cube"'), dry_bin, "physical_contained_mass_kg"),
                (_check(gazebo_cmake, "add_library(DryBinMonitorSystem SHARED"), gazebo_cmake, "add_library(DryBinMonitorSystem SHARED"),
                (_check(run_cube, "dry_bin/status_json", "validate_formal_cube_pick_place_runtime.py"), run_cube, "validate_formal_cube_pick_place_runtime.py"),
            ),
            physical_semantics="The cube carries density-derived inertial mass and collision geometry; monitor evidence is tied to deposit and bin-floor contacts, not a symbolic counter.",
        ),
        _item(
            "dry_garbage_increases_vehicle_mass",
            "A newly contained physical cube must remain an independent rigid body, transfer its load through the rear-bin contact path, and never also enter the aggregate dry inertia ledger.",
            (
                (_check(dry_bin, "physical_resident", "resident_rigid_body_count", "resident_rigid_body_mass_kg", "independent_rigid_bodies_contact"), dry_bin, "resident_rigid_body_mass_kg"),
                (_check(payload, "dryAccountingMode", "physicalResidentDry", "dryAggregateInputRejected", "independent_rigid_bodies_contact"), payload, "dryAggregateInputRejected"),
                (_check(vehicle, '<dry_accounting_mode>$(arg dry_accounting_mode)</dry_accounting_mode>', '<xacro:arg name="dry_accounting_mode" default="physical_resident"/>'), vehicle, "dry_accounting_mode"),
                (_check(twenty_cube_launch, '"dry_accounting_mode": "physical_resident"'), twenty_cube_launch, "dry_accounting_mode"),
                (_check(dry_accounting_contract, "exactly_one_mode_required: true", "physical_resident_forbids_nonzero_aggregate_dry_mass: true", "aggregate_forbids_physical_resident_dry_bodies: true"), dry_accounting_contract, "exactly_one_mode_required"),
            ),
            physical_semantics="Physical-resident dry payloads remain separate rigid bodies and the aggregate inertia path rejects a simultaneous dry-mass input.",
        ),
        _item(
            "ground_dirt_coverage",
            "A lowered rotating brush sweeps a source-owned dirt grid and publishes a measurable coverage state through its dedicated runtime validator.",
            (
                (_check(vehicle, 'filename="libGroundDirtCleaningSystem.so"'), vehicle, "libGroundDirtCleaningSystem.so"),
                (_check(ground_dirt, "left_side_brush_joint", "central_roller_joint", "cleaned_fraction", "sideBrushRadiusM"), ground_dirt, "cleaned_fraction"),
                (_check(gazebo_cmake, "add_library(GroundDirtCleaningSystem SHARED"), gazebo_cmake, "add_library(GroundDirtCleaningSystem SHARED"),
                (_check(run_dirt, "prepare_formal_ground_dirt_runtime.py", "formal_vehicle_sim.launch.py", "ground_dirt/command/enable", "ground_dirt/status_json", "validate_formal_ground_dirt_cleaning_runtime.py"), run_dirt, "ground_dirt/status_json"),
            ),
            physical_semantics="Ground-dirt state is derived from the lowered, rotating side brushes and central roller rather than a controller command alone.",
        ),
        _item(
            "brush_squeegee_water_to_wastewater_tank",
            "Side brushes, central roller, lift, compliant squeegee and pump joint states gate recoverable-water transfer into the wastewater tank mass ledger.",
            (
                (_check(vehicle, 'filename="libWaterRecoverySystem.so"', "<initial_tank_mass_kg>"), vehicle, "libWaterRecoverySystem.so"),
                (_check(water, "left_side_brush_joint", "central_roller_joint", "squeegee_float_joint", "recovery_pump_joint", "PublishPayload"), water, "PublishPayload"),
                (_check(water_world, 'model name="formal_recoverable_water_patch"'), water_world, "formal_recoverable_water_patch"),
                (_check(payload, "waterMassKg", "ApplyCompositeInertial", "waterTopic"), payload, "waterMassKg"),
                (_check(gazebo_cmake, "add_library(WaterRecoverySystem SHARED", "add_library(DynamicPayloadSystem SHARED"), gazebo_cmake, "add_library(WaterRecoverySystem SHARED"),
                (_check(sim_launch, "water_recovery/tank_mass_kg", "water_recovery/command/enable"), sim_launch, "water_recovery/tank_mass_kg"),
                (_check(run_water, "validate_formal_water_recovery_runtime.py", "finalize_formal_water_recovery_acceptance.py"), run_water, "validate_formal_water_recovery_runtime.py"),
            ),
            physical_semantics="The modeled water patch is gated by physical cleaning-joint state and publishes recovered tank mass into the composite payload path.",
        ),
    ]

    sensor_specs = {
        "single_line_lidar": (sensors, "utm30lx", sim_launch, "/sensors/lidar_2d/scan@sensor_msgs/msg/LaserScan"),
        "mid360": (sensors, "sensor name=\"mid360\"", sensor_bridge, "/sensors/lidar_3d/points"),
        "front_rgbd": (sensors, "name=\"front_rgbd\"", sensor_bridge, "/sensors/front_rgbd/depth/image_rect_raw/image"),
        "rear_fisheyes": (sensors, "rear_left_fisheye", sensor_bridge, "/sensors/rear_right_fisheye/image_raw"),
        "end_effector_stereo": (sensors, "name=\"wrist_rgbd\"", sensor_bridge, "/sensors/wrist_rgbd/infra2/image_rect_raw"),
        "gnss": (sensors, "sensor name=\"zed_f9p\"", sim_launch, "/sensors/gnss/fix@sensor_msgs/msg/NavSatFix"),
        "wheel_speed": (control, "front_left_wheel_joint", drivetrain, "JointVelocity"),
    }
    for sensor_id, (definition, definition_token, transport, transport_token) in sensor_specs.items():
        extra = ()
        if sensor_id == "wheel_speed":
            extra = ((_check(sim_launch, "joint_state_broadcaster"), sim_launch, "joint_state_broadcaster"),)
        items.append(
            _item(
                f"sensor_{sensor_id}",
                f"{sensor_id} has a concrete model definition and a declared transport/control observation path.",
                ((_check(definition, definition_token), definition, definition_token), (_check(transport, transport_token), transport, transport_token), *extra),
                physical_semantics="A named Gazebo/URDF sensor or measured wheel-state interface is bound to a declared observation transport; this does not prove data is emitted at runtime.",
            )
        )

    supplemental_items = [
        _item(
            "s100p_installation_and_low_voltage_power",
            "S100P compute has a collision-bearing mount/enclosure and an explicitly declared isolated low-voltage power branch, without treating a provisional board envelope as completed hardware integration.",
            (
                (_check(platform, 'name="s100_cabinet_roof_mount_joint"', 'name="s100_compute_enclosure_mount_joint"', 'name="s100_official_external_envelope_collision"'), platform, 'name="s100_cabinet_roof_mount_joint"'),
                (_check(platform, 'name="power_distribution_box_link"', 'name="isolated_dc_dc_module_link"', "Explicit protected power path"), platform, "Explicit protected power path"),
                (_check(component_register, "low_voltage_power_branch", "isolated_sensor_and_compute_dc_conversion", "s100_compute_enclosure_link"), component_register, "low_voltage_power_branch"),
                (_check(s100_launch, "hobot_dosod", "mono_edgesam", "open_vocab_product_adapter"), s100_launch, "open_vocab_product_adapter"),
                (s100p_evidence["validator_complete"] is True and s100p_evidence["status"] == S100P_BLOCKED_STATUS, s100p_evidence_source, S100P_BLOCKED_STATUS),
            ),
            blocked_reason="The validated S100P mechanical/electrical evidence contract remains BLOCKED: board boundary/hole datums, mass/CoM, connector coordinates/keepouts, thermal airflow, J1 pinout/polarity, real harness/protection and installed power-on/runtime are unresolved. No static source can promote this to installed-and-powered hardware acceptance.",
            physical_semantics="Mount plate, enclosure, collision volumes, PDU and isolated DC/DC are modeled; the board datum is an epsilon-mass provisional envelope, not measured S100P CAD or a verified electrical installation.",
            placeholder_indicators=(
                "prior RDK S100 121 x 120 x 52.4 mm envelope only as a provisional collision placeholder",
                "external_envelope_frozen_mass_thermal_connectors_and_live_board_pending",
            ),
        ),
        _item(
            "obstacle_avoidance_chain",
            "Navigation consumes UTM scan and MID-360 point cloud in costmaps/collision monitor and gates the resulting command before the drivetrain interface.",
            (
                (_check(nav_config, "observation_sources: scan mid360", "cmd_vel_out_topic: /cmd_vel_gate", "use_collision_detection: true"), nav_config, "observation_sources: scan mid360"),
                (_check(nav_launch, "package='nav2_collision_monitor'", "executable='velocity_gate'", "start_velocity_gate"), nav_launch, "package='nav2_collision_monitor'"),
                (_check(velocity_gate, '"/cmd_vel_gate"', '"/cmd_vel"', '"/emergency_stop"'), velocity_gate, '"/cmd_vel_gate"'),
                (_check(dynamic_obstacle_audit, "utm_and_mid360_feed_collision_monitor", "single_final_command_writer_contract"), dynamic_obstacle_audit, "utm_and_mid360_feed_collision_monitor"),
            ),
            physical_semantics="The source-level path contains obstacle observations, collision-monitor command gating and emergency-stop gating; response timing, false negatives and collision-free motion remain runtime evidence.",
        ),
    ]

    blocked = [item["id"] for item in items if item["status"] == "BLOCKED"]
    supplemental_blocked = [item["id"] for item in supplemental_items if item["status"] == "BLOCKED"]
    static_closed_count = sum(item["status"] == "STATIC_CLOSED" for item in items)
    return {
        "report_id": REPORT_ID,
        "audit_mode": "static_source_only",
        "execution_prohibited": ["WSL", "Gazebo", "Docker", "ROS runtime", "data collection"],
        "status": "BLOCKED" if blocked else "STATIC_CLOSED",
        "status_scope": "core_13_item_legacy_static_functional_chain_only",
        "items": items,
        "blocked_items": blocked,
        "required_item_count": len(REQUIRED_ITEM_IDS),
        "static_closed_count": static_closed_count,
        "supplemental_items": supplemental_items,
        "s100p_mechanical_electrical_evidence": s100p_evidence,
        "supplemental_blocked_items": supplemental_blocked,
        "expanded_required_item_count": len(REQUIRED_ITEM_IDS) + len(SUPPLEMENTAL_ITEM_IDS),
        "expanded_static_closed_count": static_closed_count + sum(item["status"] == "STATIC_CLOSED" for item in supplemental_items),
        "expanded_scope_status": "BLOCKED" if blocked or supplemental_blocked else "STATIC_CLOSED",
        "expanded_scope_runtime_accepted": False,
        "expanded_scope_boundary": "Expanded scope includes provisional S100P installation/power and static obstacle-avoidance topology; it remains source-only and never accepts runtime behavior.",
        "runtime_accepted": False,
        "fresh_gazebo_runtime_required": True,
        "runtime_boundary": "STATIC_CLOSED proves source-level continuity only; every actuator, sensor and mass effect still requires fresh Gazebo runtime evidence.",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path, default=ROOT / "reports/engineering/static_functional_chain_audit.json")
    args = parser.parse_args()
    report = audit(args.root.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{report['status']}: wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
