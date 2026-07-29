#!/usr/bin/env python3
"""Audit real AUTO-01 runtime parameters and seed0 Coverage evidence."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import yaml


def parameters(path: Path) -> dict:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    return next(iter(document.values())).get("ros__parameters", {})


def polygon(value) -> list[list[float]]:
    if isinstance(value, str):
        value = json.loads(value)
    return [[float(axis) for axis in point] for point in value]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stage", default="AUTO-01")
    args = parser.parse_args()
    profile = yaml.safe_load(args.profile.read_text(encoding="utf-8"))
    architecture = profile.get("architecture", "G1")
    expected_navigation = polygon(profile["navigation_footprint_xy_m"])
    local = parameters(args.trial / "runtime_local_costmap_params.yaml")
    global_ = parameters(args.trial / "runtime_global_costmap_params.yaml")
    coverage = json.loads(
        (args.trial / "coverage_report.json").read_text(encoding="utf-8")
    )
    expected_radius = max(math.hypot(*point) for point in expected_navigation)
    preflight = coverage.get("route_selection") or coverage.get(
        "transit_to_start", {}
    ).get("preflight", {})
    checks = {
        "local_costmap_navigation_footprint_exact": polygon(
            local.get("footprint", [])
        )
        == expected_navigation,
        "global_costmap_navigation_footprint_exact": polygon(
            global_.get("footprint", [])
        )
        == expected_navigation,
        "coverage_navigation_radius_exact": abs(
            float(coverage["mission_geometry"]["footprint_radius_m"])
            - expected_radius
        )
        <= 1e-9,
        "coverage_profile_headland_exact": coverage["mission_geometry"][
            "configured_headland_width_m"
        ]
        == float(profile["coverage_overrides"]["headland_width_m"]),
        "cleanable_area_ratio_at_least_0_90": (
            float(coverage["mission_geometry"]["cleanable_area_m2"])
            / 14.997536062726478
        )
        >= 0.90,
        "swath_conflict_count_zero": coverage.get(
            "swath_exclusion_intersection_count"
        )
        == 0,
        "legal_staging_candidate_exists": any(
            candidate.get("staging_inside_operation_polygon")
            and candidate.get("target_cost_and_keepout_clear")
            and candidate.get("success")
            for candidate in preflight.get("candidates", [])
        ),
        "local_published_footprint_observed": (
            args.trial / "runtime_local_published_footprint.yaml"
        ).stat().st_size
        > 0,
        "global_published_footprint_observed": (
            args.trial / "runtime_global_published_footprint.yaml"
        ).stat().st_size
        > 0,
    }

    high_radius = None
    if architecture == "G1":
        expected_high = polygon(
            profile["high_overhang"]["collision_polygon_xy_m"]
        )
        collision_evidence = json.loads(
            (
                args.trial / "runtime_collision_monitor_selected_params.json"
            ).read_text(encoding="utf-8")
        )
        collision = collision_evidence["high_monitor"]["parameters"]
        ground_collision = collision_evidence["ground_monitor"]["parameters"]
        scan_filter = collision_evidence["scan_self_filter"]["parameters"]
        high_name = profile["high_overhang"]["collision_monitor"]["name"]
        high_runtime = collision.get(high_name, {})
        checks.update(
            {
                "costmaps_use_self_filtered_scan": (
                    local.get("obstacle_layer", {}).get("scan", {}).get("topic")
                    == "/scan/navigation"
                    and global_.get("obstacle_layer", {})
                    .get("scan", {})
                    .get("topic")
                    == "/scan/navigation"
                ),
                "collision_monitor_ground_footprint_topic": ground_collision.get(
                    "FootprintApproach", {}
                ).get("footprint_topic")
                == "/local_costmap/published_footprint",
                "collision_monitor_ground_scan_chain": (
                    ground_collision.get("polygons") == ["FootprintApproach"]
                    and ground_collision.get("observation_sources")
                    == ["scan", "ground_cloud"]
                    and ground_collision.get("scan", {}).get("topic")
                    == "/scan/navigation"
                    and ground_collision.get("ground_cloud", {}).get("topic")
                    == profile["ground_obstacles"]["pointcloud_topic"]
                    and float(
                        ground_collision.get("ground_cloud", {}).get(
                            "min_height", -1
                        )
                    )
                    == float(profile["ground_obstacles"]["z_band_base_link_m"][0])
                    and float(
                        ground_collision.get("ground_cloud", {}).get(
                            "max_height", -1
                        )
                    )
                    == float(profile["ground_obstacles"]["z_band_base_link_m"][1])
                    and ground_collision.get("cmd_vel_out_topic")
                    == "/cmd_vel_ground_safe"
                    and float(ground_collision.get("source_timeout", 0.0))
                    == 5.0
                ),
                "collision_monitor_high_overhang_enabled": (
                    high_name in collision.get("polygons", [])
                    and high_runtime.get("enabled") is True
                    and high_runtime.get("action_type") == "stop"
                    and int(high_runtime.get("min_points", 0)) == 1
                ),
                "collision_monitor_high_overhang_polygon_exact": polygon(
                    high_runtime.get("points", [])
                )
                == expected_high,
                "collision_monitor_high_overhang_height_band_exact": (
                    collision.get("observation_sources")
                    == ["high_overhang_cloud"]
                    and collision.get("high_overhang_cloud", {}).get("topic")
                    == profile["high_overhang"]["collision_monitor"]["topic"]
                    and float(
                        collision.get("high_overhang_cloud", {}).get(
                            "min_height", -1
                        )
                    )
                    == float(profile["high_overhang"]["z_band_base_link_m"][0])
                    and float(
                        collision.get("high_overhang_cloud", {}).get(
                            "max_height", -1
                        )
                    )
                    == float(profile["high_overhang"]["z_band_base_link_m"][1])
                    and float(collision.get("source_timeout", 0.0)) == 5.0
                ),
                "scan_self_filter_matches_camera_envelope": (
                    scan_filter.get("input_topic") == "/scan"
                    and scan_filter.get("output_topic") == "/scan/navigation"
                    and float(scan_filter.get("laser_origin_x_m", -1))
                    == float(
                        profile["high_overhang"][
                            "lidar_origin_x_base_link_m"
                        ]
                    )
                    and float(scan_filter.get("mask_min_x_m", -1))
                    == min(point[0] for point in expected_high)
                    and float(scan_filter.get("mask_max_x_m", -1))
                    == max(point[0] for point in expected_high)
                    and float(scan_filter.get("mask_min_y_m", -1))
                    == min(point[1] for point in expected_high)
                    and float(scan_filter.get("mask_max_y_m", -1))
                    == max(point[1] for point in expected_high)
                ),
            }
        )
        high_radius = max(math.hypot(*point) for point in expected_high)
    elif architecture == "G2":
        collision_evidence = json.loads(
            (
                args.trial / "runtime_collision_monitor_g2_params.json"
            ).read_text(encoding="utf-8")
        )
        collision = collision_evidence["parameters"]
        self_filter = collision_evidence["pointcloud_self_filter"][
            "parameters"
        ]
        camera = profile["camera_mechanical_reconstruction"]
        aabb_min = [float(value) for value in camera["rotated_aabb_min_m"]]
        aabb_max = [float(value) for value in camera["rotated_aabb_max_m"]]
        ground = profile["ground_obstacles"]
        cloud = collision.get("ground_cloud", {})
        expected_filter = ground["self_filter"]
        checks.update(
            {
                "costmaps_keep_unmasked_scan": (
                    local.get("obstacle_layer", {}).get("scan", {}).get("topic")
                    in ("/scan", "scan")
                    and global_.get("obstacle_layer", {})
                    .get("scan", {})
                    .get("topic")
                    in ("/scan", "scan")
                ),
                "single_collision_monitor_uses_full_footprint": (
                    collision.get("polygons") == ["FootprintApproach"]
                    and collision.get("FootprintApproach", {}).get(
                        "footprint_topic"
                    )
                    == "/local_costmap/published_footprint"
                    and collision.get("cmd_vel_out_topic") == "/cmd_vel_gate"
                ),
                "single_monitor_fuses_scan_and_rgbd": (
                    collision.get("observation_sources")
                    == ["scan", "ground_cloud"]
                    and cloud.get("topic") == ground["pointcloud_topic"]
                    and float(cloud.get("min_height", -1))
                    == float(ground["z_band_base_link_m"][0])
                    and float(cloud.get("max_height", -1))
                    == float(ground["z_band_base_link_m"][1])
                    and float(collision.get("source_timeout", 0.0)) == 5.0
                ),
                "camera_xy_aabb_inside_navigation_footprint": (
                    min(point[0] for point in expected_navigation)
                    <= aabb_min[0]
                    <= aabb_max[0]
                    <= max(point[0] for point in expected_navigation)
                    and min(point[1] for point in expected_navigation)
                    <= aabb_min[1]
                    <= aabb_max[1]
                    <= max(point[1] for point in expected_navigation)
                ),
                "camera_above_lidar_plane": (
                    aabb_min[2] > float(camera["lidar_plane_base_link_m"])
                    and camera["lidar_plane_intersects_camera"] is False
                ),
                "camera_within_mount_height": aabb_max[2]
                <= float(camera["maximum_mount_height_m"]),
                "actual_gazebo_verification_camera_frame_observed": (
                    args.trial / "verification_camera_sample.yaml"
                ).is_file()
                and (
                    args.trial / "verification_camera_sample.yaml"
                ).stat().st_size
                > 0,
                "runtime_pointcloud_self_filter_exact": (
                    self_filter.get("input_topic")
                    == ground["raw_pointcloud_topic"]
                    and self_filter.get("output_topic")
                    == ground["pointcloud_topic"]
                    and self_filter.get("output_frame")
                    == expected_filter["output_frame"]
                    and self_filter.get("mask_min_xyz_m")
                    == expected_filter["self_mask_min_xyz_m"]
                    and self_filter.get("mask_max_xyz_m")
                    == expected_filter["self_mask_max_xyz_m"]
                    and int(self_filter.get("sampling_stride", 0))
                    == int(expected_filter["sampling_stride"])
                ),
            }
        )
    else:
        raise ValueError(f"unsupported AUTO-01 architecture: {architecture}")

    report = {
        "schema_version": 1,
        "stage": args.stage,
        "attempt_id": profile["attempt_id"],
        "architecture": architecture,
        "profile": profile["profile"],
        "checks": checks,
        "runtime_geometry_gate_pass": all(checks.values()),
        "metrics": {
            "navigation_footprint_radius_m": expected_radius,
            "high_overhang_radius_m": high_radius,
            "cleanable_area_m2": coverage["mission_geometry"][
                "cleanable_area_m2"
            ],
            "cleanable_area_ratio": coverage["mission_geometry"][
                "cleanable_area_m2"
            ]
            / 14.997536062726478,
            "swath_conflict_count": coverage.get(
                "swath_exclusion_intersection_count"
            ),
            "component_count": coverage.get("component_count"),
            "empirical_coverage": coverage.get("empirical_metrics", {}).get(
                "coverage_rate"
            ),
            "collision_count": coverage.get("collision_count"),
            "keepout_violation_count": coverage.get(
                "keepout_violation_sample_count"
            ),
            "localization_rmse_m": coverage.get(
                "localization_regression_during_coverage", {}
            ).get("rmse_m"),
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    return 0 if report["runtime_geometry_gate_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
