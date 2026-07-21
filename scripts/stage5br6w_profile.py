#!/usr/bin/env python3
"""Derive and materialize the opt-in Stage5BR6W V4 engineering footprint."""

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


def materialize_nav2(base: dict, profile: dict) -> dict:
    points = profile["footprint_xy_m"]
    encoded = _footprint_string(points)
    base["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"] = encoded
    base["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"] = encoded
    return base


def materialize_mission(base: dict, profile: dict) -> dict:
    overrides = profile["coverage_overrides"]
    base["robot_footprint"] = profile["footprint_xy_m"]
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
