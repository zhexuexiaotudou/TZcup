#!/usr/bin/env python3
"""Machine audit for AUTO-01 geometry candidates."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_coverage"))

from sanitation_coverage.mission_geometry import compile_mission_geometry  # noqa: E402
from stage5br6w_profile import (  # noqa: E402
    materialize_mission,
    materialize_nav2,
    navigation_footprint,
)


STAGE4W_CLEANABLE_AREA_M2 = 14.997536062726478


def polygon_radius(points: list[list[float]]) -> float:
    return max(math.hypot(float(x), float(y)) for x, y in points)


def audit(profile_path: Path, nav2_path: Path, mission_path: Path) -> dict:
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    architecture = profile.get("architecture", "G1")
    base_nav2 = yaml.safe_load(nav2_path.read_text(encoding="utf-8"))
    base_mission = yaml.safe_load(mission_path.read_text(encoding="utf-8"))
    materialized_nav2 = materialize_nav2(base_nav2, profile)
    materialized_mission = materialize_mission(base_mission, profile)
    geometry = compile_mission_geometry(materialized_mission)
    navigation = navigation_footprint(profile)
    ratio = geometry["cleanable_area_m2"] / STAGE4W_CLEANABLE_AREA_M2
    checks = {
        "profile_is_opt_in": profile.get("opt_in_only") is True,
        "production_default_unchanged": profile.get(
            "production_default_unchanged"
        )
        is True,
        "local_costmap_uses_navigation_footprint": json.loads(
            materialized_nav2["local_costmap"]["local_costmap"][
                "ros__parameters"
            ]["footprint"]
        )
        == navigation,
        "global_costmap_uses_navigation_footprint": json.loads(
            materialized_nav2["global_costmap"]["global_costmap"][
                "ros__parameters"
            ]["footprint"]
        )
        == navigation,
        "coverage_uses_navigation_footprint": materialized_mission[
            "robot_footprint"
        ]
        == navigation,
        "headland_clearance_valid": geometry["headland_clearance_valid"] is True,
        "cleanable_area_ratio_at_least_0_90": ratio >= 0.90,
    }
    high_radius = None
    if architecture == "G1":
        high = profile["high_overhang"]
        band_min, band_max = map(float, high["z_band_base_link_m"])
        monitor_name = high["collision_monitor"]["name"]
        monitor = materialized_nav2["collision_monitor"]["ros__parameters"]
        ground_monitor = materialized_nav2["ground_collision_monitor"][
            "ros__parameters"
        ]
        ground = profile["ground_obstacles"]
        monitor_points = json.loads(monitor[monitor_name]["points"])
        scan_filter = materialized_nav2["scan_self_filter"]["ros__parameters"]
        checks.update(
            {
                "high_overhang_monitor_enabled": (
                    monitor_name in monitor["polygons"]
                    and monitor[monitor_name]["enabled"] is True
                    and monitor[monitor_name]["action_type"] == "stop"
                    and monitor[monitor_name]["min_points"] == 1
                ),
                "high_overhang_polygon_exact": monitor_points
                == high["collision_polygon_xy_m"],
                "navigation_scan_self_filter_matches_camera_envelope": (
                    scan_filter["input_topic"] == "/scan"
                    and scan_filter["output_topic"] == "/scan/navigation"
                    and float(scan_filter["mask_min_x_m"])
                    == min(
                        point[0]
                        for point in high["collision_polygon_xy_m"]
                    )
                    and float(scan_filter["mask_max_x_m"])
                    == max(
                        point[0]
                        for point in high["collision_polygon_xy_m"]
                    )
                ),
                "ground_monitor_uses_scan_and_ground_footprint": (
                    ground_monitor["polygons"] == ["FootprintApproach"]
                    and ground_monitor["observation_sources"]
                    == ["scan", "ground_cloud"]
                    and ground_monitor["ground_cloud"]["topic"]
                    == ground["pointcloud_topic"]
                    and [
                        ground_monitor["ground_cloud"]["min_height"],
                        ground_monitor["ground_cloud"]["max_height"],
                    ]
                    == ground["z_band_base_link_m"]
                ),
                "high_monitor_uses_height_filtered_pointcloud": (
                    monitor["observation_sources"]
                    == ["high_overhang_cloud"]
                    and monitor["high_overhang_cloud"]["topic"]
                    == high["collision_monitor"]["topic"]
                    and float(
                        monitor["high_overhang_cloud"]["min_height"]
                    )
                    == band_min
                    and float(
                        monitor["high_overhang_cloud"]["max_height"]
                    )
                    == band_max
                ),
            }
        )
        high_radius = polygon_radius(high["collision_polygon_xy_m"])
    elif architecture == "G2":
        camera = profile["camera_mechanical_reconstruction"]
        aabb_min = [float(value) for value in camera["rotated_aabb_min_m"]]
        aabb_max = [float(value) for value in camera["rotated_aabb_max_m"]]
        min_x = min(point[0] for point in navigation)
        max_x = max(point[0] for point in navigation)
        min_y = min(point[1] for point in navigation)
        max_y = max(point[1] for point in navigation)
        monitor = materialized_nav2["collision_monitor"]["ros__parameters"]
        cloud = monitor.get("ground_cloud", {})
        ground = profile["ground_obstacles"]
        self_filter = materialized_nav2["pointcloud_self_filter"][
            "ros__parameters"
        ]
        expected_filter = ground["self_filter"]
        checks.update(
            {
                "camera_xy_aabb_inside_navigation_footprint": (
                    min_x <= aabb_min[0] <= aabb_max[0] <= max_x
                    and min_y <= aabb_min[1] <= aabb_max[1] <= max_y
                ),
                "camera_above_lidar_plane": (
                    aabb_min[2] > float(camera["lidar_plane_base_link_m"])
                    and camera["lidar_plane_intersects_camera"] is False
                ),
                "camera_within_mount_height": aabb_max[2]
                <= float(camera["maximum_mount_height_m"]),
                "camera_collision_free": camera[
                    "collision_free_from_body_bumper_arm_and_brush"
                ]
                is True,
                "single_monitor_fuses_lidar_and_rgbd": (
                    monitor["polygons"] == ["FootprintApproach"]
                    and monitor["observation_sources"]
                    == ["scan", "ground_cloud"]
                    and cloud["topic"] == ground["pointcloud_topic"]
                    and [
                        cloud["min_height"],
                        cloud["max_height"],
                    ]
                    == ground["z_band_base_link_m"]
                    and float(monitor["source_timeout"]) == 5.0
                ),
                "pointcloud_self_filter_exact": (
                    self_filter["input_topic"]
                    == ground["raw_pointcloud_topic"]
                    and self_filter["output_topic"]
                    == ground["pointcloud_topic"]
                    and self_filter["output_frame"]
                    == expected_filter["output_frame"]
                    and self_filter["mask_min_xyz_m"]
                    == expected_filter["self_mask_min_xyz_m"]
                    and self_filter["mask_max_xyz_m"]
                    == expected_filter["self_mask_max_xyz_m"]
                    and self_filter["sampling_stride"]
                    == expected_filter["sampling_stride"]
                ),
            }
        )
    else:
        raise ValueError(f"unsupported AUTO-01 architecture: {architecture}")

    return {
        "schema_version": 1,
        "stage": "AUTO-01",
        "attempt_id": profile["attempt_id"],
        "architecture": architecture,
        "profile": profile["profile"],
        "checks": checks,
        "all_offline_checks_pass": all(checks.values()),
        "metrics": {
            "stage4w_baseline_cleanable_area_m2": STAGE4W_CLEANABLE_AREA_M2,
            "candidate_cleanable_area_m2": geometry["cleanable_area_m2"],
            "cleanable_area_ratio": ratio,
            "navigation_footprint_radius_m": polygon_radius(navigation),
            "high_overhang_radius_m": high_radius,
            "configured_headland_width_m": geometry[
                "configured_headland_width_m"
            ],
            "required_headland_width_m": geometry[
                "required_headland_width_m"
            ],
            "swath_conflict_count": None,
            "legal_staging_pose_count": None,
        },
        "runtime_gates_pending": [
            "cold_start_3_of_3",
            "nav2_parameter_services_ready_within_60_s",
            "swath_conflict_count_zero",
            "legal_staging_pose_per_component",
            "seed0_full_formal_gate",
            "low_obstacle_30_trials",
            "tall_obstacle_30_trials",
            "height_classification_false_safe_zero",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        type=Path,
        default=ROOT
        / "starter_ws/src/sanitation_navigation/config/auto01_g1_height_banded.yaml",
    )
    parser.add_argument(
        "--nav2",
        type=Path,
        default=ROOT / "starter_ws/src/sanitation_navigation/config/nav2.yaml",
    )
    parser.add_argument(
        "--mission",
        type=Path,
        default=ROOT
        / "starter_ws/src/sanitation_tasks/config/demo_area.yaml",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = audit(args.profile, args.nav2, args.mission)
    payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8", newline="\n")
    print(payload, end="")
    return 0 if report["all_offline_checks_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
