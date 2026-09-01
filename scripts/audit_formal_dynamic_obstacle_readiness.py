#!/usr/bin/env python3
"""Audit static readiness for the formal eight-pedestrian avoidance gate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import yaml

from generate_formal_dynamic_runtime_build_manifest import (
    REQUIRED_PLUGIN_LIBRARIES,
    generate_manifest,
)
from prepare_formal_dynamic_obstacle_schedule import materialize_schedule
from prepare_formal_dynamic_runtime_world import prepare_runtime_world


def _dependencies(package_xml: Path) -> set[str]:
    root = ET.parse(package_xml).getroot()
    return {
        (child.text or "").strip()
        for child in root
        if child.tag in {"depend", "exec_depend"} and (child.text or "").strip()
    }


def audit(
    repository_root: Path,
    install_root: Path,
    episode_root: Path,
) -> dict:
    nav2_paths = (
        repository_root / "starter_ws/src/sanitation_navigation/config/nav2.yaml",
        repository_root
        / "starter_ws/src/sanitation_navigation/config/nav2_auto12.yaml",
    )
    nav2_profiles = [yaml.safe_load(path.read_text(encoding="utf-8")) for path in nav2_paths]
    lifecycle_source = (
        repository_root
        / "starter_ws/src/sanitation_formal_campus_integration/launch/"
        "formal_campus_map_lifecycle.launch.py"
    ).read_text(encoding="utf-8")
    campus_source = (
        repository_root
        / "starter_ws/src/sanitation_formal_campus_integration/launch/"
        "formal_campus.launch.py"
    ).read_text(encoding="utf-8")
    product_collector = (
        repository_root / "scripts/collect_formal_dynamic_obstacle_avoidance_runtime.py"
    ).read_text(encoding="utf-8")
    evaluator_collector = (
        repository_root / "scripts/collect_formal_dynamic_environment_runtime.py"
    ).read_text(encoding="utf-8")
    runner_source = (
        repository_root / "scripts/run_formal_dynamic_obstacle_avoidance.sh"
    ).read_text(encoding="utf-8")
    episode_manifest = episode_root / "public/episode_manifest.json"
    public_world = episode_root / "public/world.sdf"
    base_schedule = episode_root / "environment/pedestrian_schedule.json"

    try:
        build = generate_manifest(repository_root, install_root)
        build_error = None
    except (FileNotFoundError, ValueError) as exc:
        build_error = str(exc)
        build = {
            "current_source_build_completed": False,
            "source_install_bindings": [],
            "source_only_runtime_files": [],
            "required_plugin_libraries": {
                name: (install_root / "lib" / name).is_file()
                for name in REQUIRED_PLUGIN_LIBRARIES
            },
            "error": build_error,
        }
    with tempfile.TemporaryDirectory(prefix="tzcup_dynamic_static_") as directory:
        temp = Path(directory)
        schedule = materialize_schedule(
            episode_manifest=episode_manifest,
            public_world=public_world,
            base_schedule=base_schedule,
            seed=81422,
            nominal_leg_m=30.0,
        )
        runtime_world = prepare_runtime_world(public_world, temp / "world.sdf")

    monitor_sources_valid = all(
        profile["collision_monitor"]["ros__parameters"].get(
            "observation_sources"
        )
        == ["scan", "mid360"]
        and profile["collision_monitor"]["ros__parameters"]["scan"].get("type")
        == "scan"
        and profile["collision_monitor"]["ros__parameters"]["mid360"].get(
            "type"
        )
        == "pointcloud"
        and profile["collision_monitor"]["ros__parameters"]["mid360"].get(
            "topic"
        )
        == "/sensors/lidar_3d/points"
        for profile in nav2_profiles
    )
    costmap_sources_valid = all(
        set(
            profile[name][name]["ros__parameters"]["obstacle_layer"][
                "observation_sources"
            ].split()
        )
        == {"scan", "mid360"}
        for profile in nav2_profiles
        for name in ("local_costmap", "global_costmap")
    )
    product_forbidden = (
        "/scenario/environment/pedestrian_driver/status",
        "/safety/front_bumper/contact",
        "/safety/rear_bumper/contact",
        "ros_gz_interfaces.msg",
    )
    evaluator_control_forbidden = (
        "NavigateToPose",
        "ActionClient",
        "create_publisher",
        "/cmd_vel",
    )
    required_packages = {
        "sanitation_formal_campus_integration",
        "sanitation_vehicle_description",
        "sanitation_gazebo_control",
        "sanitation_gazebo_auxiliary",
        "sanitation_navigation",
        "sanitation_localization",
        "sanitation_power_system",
        "sanitation_safety",
        "sanitation_campus_scenario",
        "sanitation_manipulation",
        "sanitation_coverage",
    }
    checks = {
        "fresh_overlay_matches_checkout": build["current_source_build_completed"]
        is True,
        "all_vehicle_plugin_libraries_present": bool(
            build["required_plugin_libraries"]
        )
        and all(build["required_plugin_libraries"].values()),
        "eight_public_world_walkers_preserved": runtime_world[
            "pedestrian_model_count"
        ]
        == 8
        and runtime_world["world_preserved_except_contact_system"] is True,
        "randomized_schedule_has_eight_walkers_and_three_crossings": len(
            schedule["pedestrians"]
        )
        == 8
        and schedule["acceptance_environment"][
            "mission_corridor_crossing_count"
        ]
        >= 3,
        "environment_schedule_is_evaluator_only": schedule["access"]
        == "environment_driver_only_not_robot_control"
        and schedule["acceptance_environment"][
            "product_control_access_prohibited"
        ]
        is True,
        "product_collector_reads_no_environment_truth": not any(
            marker in product_collector for marker in product_forbidden
        ),
        "evaluator_collector_has_no_control_surface": not any(
            marker in evaluator_collector for marker in evaluator_control_forbidden
        ),
        "canonical_filtered_scan_reaches_all_2d_consumers": (
            'canonical_scan = "/scan/navigation"' in lifecycle_source
            and 'nav2["amcl"]["ros__parameters"]["scan_topic"] = canonical_scan'
            in lifecycle_source
            and 'nav2["collision_monitor"]["ros__parameters"]["scan"]["topic"] = canonical_scan'
            in lifecycle_source
        ),
        "utm_and_mid360_feed_both_costmaps": costmap_sources_valid,
        "utm_and_mid360_feed_collision_monitor": monitor_sources_valid,
        "single_collision_monitor_contract": (
            'package="nav2_collision_monitor"' not in lifecycle_source
            and "Jazzy nav2_bringup above owns collision_monitor" in lifecycle_source
        ),
        "single_final_command_writer_contract": (
            '"command_input_topic": "/cmd_vel_gate"' in campus_source
            and '"base_command_output_topic": "/base_controller/cmd_vel"'
            in campus_source
            and '"enable_safety_manager": "false"' in campus_source
        ),
        "navigation_declares_collision_monitor_dependency": (
            "nav2_collision_monitor"
            in _dependencies(
                repository_root
                / "starter_ws/src/sanitation_navigation/package.xml"
            )
        ),
        "runner_requires_complete_project_overlay": all(
            package in runner_source for package in required_packages
        ),
        "runner_uses_split_product_and_evaluator_collectors": (
            "collect_formal_dynamic_obstacle_avoidance_runtime.py" in runner_source
            and "collect_formal_dynamic_environment_runtime.py" in runner_source
            and '--environment-telemetry "${environment_telemetry}"'
            in runner_source
        ),
    }
    passed = all(checks.values())
    return {
        "schema_version": 1,
        "report_id": "tzcup_formal_dynamic_obstacle_static_readiness_v1",
        "status": (
            "FORMAL_DYNAMIC_OBSTACLE_STATIC_READINESS_PASSED"
            if passed
            else "FORMAL_DYNAMIC_OBSTACLE_STATIC_READINESS_BLOCKED"
        ),
        "passed": passed,
        "checks": checks,
        "blockers": [name for name, value in checks.items() if not value],
        "runtime_build_manifest": build,
        "runtime_build_manifest_error": build_error,
        "admitted_episode": {
            "episode_manifest": str(episode_manifest.resolve()),
            "public_world": str(public_world.resolve()),
            "base_schedule": str(base_schedule.resolve()),
            "walker_ids": runtime_world["pedestrian_model_ids"],
            "static_audit_seed": 81422,
        },
        "dynamic_runtime_executed": False,
        "pending_dynamic_prerequisites": [
            "final frozen vehicle snapshot and RUNNING acceptance session",
            "qualified first-task saved-map lifecycle artifact",
            "non-symlink merged overlay with hashable installed Python modules",
            "serial Gazebo/Nav2 eight-pedestrian runtime",
        ],
        "claim_boundary": (
            "PASS proves source, package, overlay, sensor, command ownership and "
            "truth-isolation readiness only. It does not claim that Gazebo was "
            "started, a Nav2 goal completed, an avoidance detour occurred, or "
            "collision-free dynamic acceptance passed."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--install-root", type=Path, required=True)
    parser.add_argument("--episode-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = audit(
        args.repository_root.resolve(),
        args.install_root.resolve(),
        args.episode_root.resolve(),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(args.output)
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
