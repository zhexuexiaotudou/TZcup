#!/usr/bin/env python3
"""Derive and materialize opt-in navigation/camera-envelope profiles."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml


def derive_candidate_footprint(mechanics: dict, margin_m: float = 0.03) -> list[list[float]]:
    v4 = mechanics["camera_results"]["V4"]
    production = mechanics.get("production_nav2_footprint_xy_m")
    if production is None:
        raise ValueError("mechanics report must expose production_nav2_footprint_xy_m")
    min_x = min(min(point[0] for point in production), float(v4["camera_aabb_min_m"][0])) - margin_m
    max_x = max(max(point[0] for point in production), float(v4["camera_aabb_max_m"][0])) + margin_m
    min_y = min(min(point[1] for point in production), float(v4["camera_aabb_min_m"][1])) - margin_m
    max_y = max(max(point[1] for point in production), float(v4["camera_aabb_max_m"][1])) + margin_m
    return [[max_x, max_y], [max_x, min_y], [min_x, min_y], [min_x, max_y]]


def _footprint_string(points: list[list[float]]) -> str:
    return json.dumps(points, separators=(",", ":"))


def navigation_footprint(profile: dict) -> list[list[float]]:
    points = profile.get("navigation_footprint_xy_m", profile.get("footprint_xy_m"))
    if not isinstance(points, list) or len(points) < 3:
        raise ValueError("profile must define navigation_footprint_xy_m or footprint_xy_m")
    return points


def materialize_nav2(base: dict, profile: dict) -> dict:
    points = navigation_footprint(profile)
    encoded = _footprint_string(points)
    base["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"] = encoded
    base["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"] = encoded
    high_overhang = profile.get("high_overhang")
    if high_overhang:
        monitor = base["collision_monitor"]["ros__parameters"]
        high = high_overhang["collision_monitor"]
        ground = profile["ground_obstacles"]
        ground_lower, ground_upper = ground["z_band_base_link_m"]
        name = str(high["name"])
        ground_monitor = dict(monitor)
        ground_monitor["source_timeout"] = 5.0
        ground_monitor["cmd_vel_out_topic"] = "/cmd_vel_ground_safe"
        ground_monitor["state_topic"] = "ground_collision_monitor_state"
        ground_monitor["observation_sources"] = ["scan", "ground_cloud"]
        ground_monitor["scan"] = dict(ground_monitor["scan"])
        ground_monitor["scan"]["topic"] = "/scan/navigation"
        ground_monitor["ground_cloud"] = {
            "type": "pointcloud",
            "topic": str(ground["pointcloud_topic"]),
            "min_height": float(ground_lower),
            "max_height": float(ground_upper),
            "min_range": 0.05,
            "enabled": True,
        }
        base["ground_collision_monitor"] = {"ros__parameters": ground_monitor}
        for costmap_name in ("local_costmap", "global_costmap"):
            obstacle_layer = base[costmap_name][costmap_name]["ros__parameters"][
                "obstacle_layer"
            ]
            obstacle_layer["scan"] = dict(obstacle_layer["scan"])
            obstacle_layer["scan"]["topic"] = "/scan/navigation"
        mask = high_overhang["collision_polygon_xy_m"]
        base["scan_self_filter"] = {
            "ros__parameters": {
                "use_sim_time": True,
                "input_topic": "/scan",
                "output_topic": "/scan/navigation",
                "laser_origin_x_m": float(high_overhang["lidar_origin_x_base_link_m"]),
                "mask_min_x_m": min(float(point[0]) for point in mask),
                "mask_max_x_m": max(float(point[0]) for point in mask),
                "mask_min_y_m": min(float(point[1]) for point in mask),
                "mask_max_y_m": max(float(point[1]) for point in mask),
            }
        }
        monitor["cmd_vel_in_topic"] = "/cmd_vel_ground_safe"
        monitor["source_timeout"] = 5.0
        monitor["polygons"] = [name]
        monitor[name] = {
            "type": "polygon",
            "action_type": str(high["action_type"]),
            "points": _footprint_string(high_overhang["collision_polygon_xy_m"]),
            "min_points": int(high["min_points"]),
            "visualize": False,
            "enabled": True,
        }
        monitor.pop("FootprintApproach", None)
        lower, upper = high_overhang["z_band_base_link_m"]
        monitor["observation_sources"] = ["high_overhang_cloud"]
        monitor.pop("scan", None)
        monitor["high_overhang_cloud"] = {
            "type": "pointcloud",
            "topic": str(high["topic"]),
            "min_height": float(lower),
            "max_height": float(upper),
            "min_range": 0.05,
            "enabled": True,
        }
    elif profile.get("ground_obstacles"):
        # G2 keeps every physical component inside the navigation footprint.
        # A single footprint-approach monitor can therefore fuse lidar and
        # height-aware RGB-D without a second command stage or scan masking.
        monitor = base["collision_monitor"]["ros__parameters"]
        ground = profile["ground_obstacles"]
        lower, upper = ground["z_band_base_link_m"]
        monitor["source_timeout"] = 5.0
        monitor["observation_sources"] = ["scan", "ground_cloud"]
        monitor["ground_cloud"] = {
            "type": "pointcloud",
            "topic": str(ground["pointcloud_topic"]),
            "min_height": float(lower),
            "max_height": float(upper),
            "min_range": 0.05,
            "enabled": True,
        }
        self_filter = ground.get("self_filter")
        if self_filter:
            base["pointcloud_self_filter"] = {
                "ros__parameters": {
                    "use_sim_time": True,
                    "input_topic": str(ground["raw_pointcloud_topic"]),
                    "output_topic": str(ground["pointcloud_topic"]),
                    "output_frame": str(self_filter["output_frame"]),
                    "mask_min_xyz_m": [
                        float(value)
                        for value in self_filter["self_mask_min_xyz_m"]
                    ],
                    "mask_max_xyz_m": [
                        float(value)
                        for value in self_filter["self_mask_max_xyz_m"]
                    ],
                    "sampling_stride": int(
                        self_filter["sampling_stride"]
                    ),
                }
            }
    return base


def materialize_mission(base: dict, profile: dict) -> dict:
    overrides = profile["coverage_overrides"]
    base["robot_footprint"] = navigation_footprint(profile)
    base["robot_width_m"] = float(overrides["robot_width_m"])
    base["headland"]["width_m"] = float(overrides["headland_width_m"])
    base["staging_offset_m"] = float(overrides["staging_offset_m"])
    base["profile"] = profile["profile"]
    return base


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-nav2", type=Path, required=True)
    parser.add_argument("--base-mission", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--nav2-output", type=Path, required=True)
    parser.add_argument("--mission-output", type=Path, required=True)
    args = parser.parse_args()
    profile = yaml.safe_load(args.profile.read_text(encoding="utf-8"))
    nav2 = materialize_nav2(yaml.safe_load(args.base_nav2.read_text(encoding="utf-8")), profile)
    mission = materialize_mission(yaml.safe_load(args.base_mission.read_text(encoding="utf-8")), profile)
    args.nav2_output.parent.mkdir(parents=True, exist_ok=True)
    args.mission_output.parent.mkdir(parents=True, exist_ok=True)
    args.nav2_output.write_text(yaml.safe_dump(nav2, sort_keys=False), encoding="utf-8")
    args.mission_output.write_text(yaml.safe_dump(mission, sort_keys=False), encoding="utf-8")


if __name__ == "__main__":
    main()
