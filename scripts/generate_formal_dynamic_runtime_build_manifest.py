#!/usr/bin/env python3
"""Bind the dynamic-obstacle runtime install to this checkout's source bytes."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


SOURCE_INSTALL_BINDINGS = {
    (
        "starter_ws/src/sanitation_formal_campus_integration/launch/"
        "formal_campus_map_lifecycle.launch.py"
    ):
        "share/sanitation_formal_campus_integration/launch/formal_campus_map_lifecycle.launch.py",
    (
        "starter_ws/src/sanitation_formal_campus_integration/"
        "sanitation_formal_campus_integration/map_lifecycle_core.py"
    ):
        "lib/python*/site-packages/sanitation_formal_campus_integration/map_lifecycle_core.py",
    (
        "starter_ws/src/sanitation_formal_campus_integration/config/"
        "formal_utm30lx_self_filter.yaml"
    ):
        "share/sanitation_formal_campus_integration/config/formal_utm30lx_self_filter.yaml",
    (
        "starter_ws/src/sanitation_formal_campus_integration/"
        "sanitation_formal_campus_integration/formal_scan_self_filter.py"
    ):
        "lib/python*/site-packages/sanitation_formal_campus_integration/formal_scan_self_filter.py",
    (
        "starter_ws/src/sanitation_formal_campus_integration/"
        "sanitation_formal_campus_integration/scan_self_filter_core.py"
    ):
        (
            "lib/python*/site-packages/sanitation_formal_campus_integration/"
            "scan_self_filter_core.py"
        ),
    "starter_ws/src/sanitation_navigation/config/nav2.yaml":
        "share/sanitation_navigation/config/nav2.yaml",
    "starter_ws/src/sanitation_navigation/launch/navigation.launch.py":
        "share/sanitation_navigation/launch/navigation.launch.py",
    "starter_ws/src/sanitation_formal_campus_integration/launch/formal_campus.launch.py":
        "share/sanitation_formal_campus_integration/launch/formal_campus.launch.py",
    (
        "starter_ws/src/sanitation_formal_campus_integration/"
        "sanitation_formal_campus_integration/topic_adapter.py"
    ):
        "lib/python*/site-packages/sanitation_formal_campus_integration/topic_adapter.py",
    "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py":
        "share/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py",
    "starter_ws/src/sanitation_vehicle_description/config/formal_vehicle_controllers.yaml":
        "share/sanitation_vehicle_description/config/formal_vehicle_controllers.yaml",
    "starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro":
        "share/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro",
    (
        "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/"
        "control_interfaces.xacro"
    ):
        "share/sanitation_vehicle_description/urdf/high_fidelity/control_interfaces.xacro",
    (
        "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/"
        "manipulator_stack.xacro"
    ):
        "share/sanitation_vehicle_description/urdf/high_fidelity/manipulator_stack.xacro",
    "starter_ws/src/sanitation_localization/launch/formal_localization_fusion.launch.py":
        "share/sanitation_localization/launch/formal_localization_fusion.launch.py",
    "starter_ws/src/sanitation_localization/config/formal_fusion.yaml":
        "share/sanitation_localization/config/formal_fusion.yaml",
    "starter_ws/src/sanitation_safety/sanitation_safety/whole_vehicle_safety_manager.py":
        "lib/python*/site-packages/sanitation_safety/whole_vehicle_safety_manager.py",
    "starter_ws/src/sanitation_safety/sanitation_safety/whole_vehicle_safety_core.py":
        "lib/python*/site-packages/sanitation_safety/whole_vehicle_safety_core.py",
    "starter_ws/src/sanitation_safety/sanitation_safety/simulation_safety_inputs.py":
        "lib/python*/site-packages/sanitation_safety/simulation_safety_inputs.py",
    "starter_ws/src/sanitation_safety/sanitation_safety/service_drain_core.py":
        "lib/python*/site-packages/sanitation_safety/service_drain_core.py",
    "starter_ws/src/sanitation_safety/sanitation_safety/service_drain_manager.py":
        "lib/python*/site-packages/sanitation_safety/service_drain_manager.py",
    "starter_ws/src/sanitation_power_system/sanitation_power_system/a300_bms_core.py":
        "lib/python*/site-packages/sanitation_power_system/a300_bms_core.py",
    "starter_ws/src/sanitation_power_system/sanitation_power_system/a300_bms_node.py":
        "lib/python*/site-packages/sanitation_power_system/a300_bms_node.py",
    "starter_ws/src/sanitation_power_system/config/a300_40ah_bms.yaml":
        "share/sanitation_power_system/config/a300_40ah_bms.yaml",
    "starter_ws/src/sanitation_campus_scenario/sanitation_campus_scenario/pedestrian_driver.py":
        "lib/python*/site-packages/sanitation_campus_scenario/pedestrian_driver.py",
}

SOURCE_ONLY_RUNTIME_FILES = (
    "starter_ws/src/sanitation_formal_campus_integration/setup.py",
    "scripts/collect_formal_map_lifecycle_runtime.py",
    "scripts/run_formal_dynamic_obstacle_avoidance.sh",
    "scripts/collect_formal_dynamic_obstacle_avoidance_runtime.py",
    "scripts/collect_formal_dynamic_environment_runtime.py",
    "scripts/validate_formal_dynamic_obstacle_avoidance.py",
    "scripts/generate_formal_dynamic_runtime_build_manifest.py",
    "scripts/prepare_formal_dynamic_obstacle_schedule.py",
    "scripts/prepare_formal_dynamic_runtime_world.py",
    "scripts/run_formal_runtime_isolation.sh",
    "scripts/formal_source_bound_preflight.sh",
    "scripts/run_r065_public_modeling_session.sh",
    "scripts/publish_r065_public_modeling_receipt.py",
    "scripts/run_r065_w1_dynamic_footprint_live.sh",
    "scripts/run_r065_w2_moveit_ground_live.sh",
    "scripts/collect_r065_w2_live_grasp_request.py",
    "starter_ws/src/sanitation_gazebo_control/include/sanitation_gazebo_control/A300DrivetrainPlantCore.hh",
    "starter_ws/src/sanitation_gazebo_control/src/A300DrivetrainPlantCore.cc",
    "starter_ws/src/sanitation_gazebo_control/src/A300DrivetrainPlantSystem.cc",
    "starter_ws/src/sanitation_gazebo_control/src/A300DrivetrainCommandAdapter.cc",
    "starter_ws/src/sanitation_gazebo_control/src/A300DrivetrainNativeBridge.cc",
    "starter_ws/src/sanitation_gazebo_control/src/WaterRecoverySystem.cc",
    "starter_ws/src/sanitation_safety/setup.py",
    "starter_ws/src/sanitation_power_system/setup.py",
)

REQUIRED_PLUGIN_LIBRARIES = (
    "libSanitationMissionControl.so",
    "libDynamicPayloadSystem.so",
    "libWaterRecoverySystem.so",
    "libDryBinMonitorSystem.so",
    "libGripperContactGateSystem.so",
    "libGroundDirtCleaningSystem.so",
    "libCleaningActuatorMotorSystem.so",
    "libA300DrivetrainPlantSystem.so",
    "libGripperMimicEffortSystem.so",
    "libFormalAuxiliaryVisualSystem.so",
    "libServiceDoorSystem.so",
    "libSqueegeeComplianceSystem.so",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def generate_manifest(repository_root: Path, install_root: Path) -> dict:
    bindings = []
    for source_relative, installed_pattern in SOURCE_INSTALL_BINDINGS.items():
        source = repository_root / source_relative
        matches = sorted(install_root.glob(installed_pattern))
        if not source.is_file():
            raise FileNotFoundError(f"runtime source is missing: {source}")
        if len(matches) != 1:
            raise FileNotFoundError(
                f"expected one installed runtime file for {installed_pattern}, got {matches}"
            )
        source_sha = _sha256(source)
        installed_sha = _sha256(matches[0])
        bindings.append(
            {
                "source": source_relative,
                "installed_relative": matches[0].relative_to(install_root).as_posix(),
                "source_sha256": source_sha,
                "installed_sha256": installed_sha,
                "matches": source_sha == installed_sha,
            }
        )
    libraries = {
        name: (install_root / "lib" / name).is_file()
        for name in REQUIRED_PLUGIN_LIBRARIES
    }
    source_only = []
    for relative in SOURCE_ONLY_RUNTIME_FILES:
        path = repository_root / relative
        if not path.is_file():
            raise FileNotFoundError(f"runtime orchestration source is missing: {path}")
        source_only.append({"source": relative, "source_sha256": _sha256(path)})
    passed = all(item["matches"] for item in bindings) and all(libraries.values())
    return {
        "schema_version": 1,
        "current_source_build_completed": passed,
        "repository_root": str(repository_root.resolve()),
        "runtime_install_root": str(install_root.resolve()),
        "source_install_bindings": bindings,
        "source_only_runtime_files": source_only,
        "required_plugin_libraries": libraries,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--install-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    value = generate_manifest(args.repository_root, args.install_root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(args.output)
    return 0 if value["current_source_build_completed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
