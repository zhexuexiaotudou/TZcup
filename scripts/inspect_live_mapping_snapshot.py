#!/usr/bin/env python3
"""Inspect a recorded HMI mapping snapshot without starting ROS or Gazebo."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
INTEGRATION_SOURCE = (
    REPOSITORY_ROOT
    / "starter_ws"
    / "src"
    / "sanitation_formal_campus_integration"
)
if str(INTEGRATION_SOURCE) not in sys.path:
    sys.path.insert(0, str(INTEGRATION_SOURCE))

from sanitation_formal_campus_integration.map_lifecycle_core import (  # noqa: E402
    _inside,
    load_campus_map_contract,
    select_frontier_goal,
)


def _number(value: Any, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be numeric") from exc
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def _integer(value: Any, label: str) -> int:
    number = _number(value, label)
    if not number.is_integer():
        raise ValueError(f"{label} must be an integer")
    return int(number)


def _mapping_fields(
    telemetry: dict[str, Any],
) -> tuple[list[int], int, int, float, float, float, float, float, float]:
    try:
        grid = telemetry["visualization"]["occupancy_grid"]
        pose = telemetry["vehicle"]["estimated_pose_map"]
    except (KeyError, TypeError) as exc:
        raise ValueError("dashboard telemetry lacks visualization.occupancy_grid or vehicle.estimated_pose_map") from exc
    if not isinstance(grid, dict) or not isinstance(pose, list) or len(pose) < 2:
        raise ValueError("dashboard occupancy grid or map pose is malformed")
    width = _integer(grid.get("width"), "occupancy width")
    height = _integer(grid.get("height"), "occupancy height")
    resolution = _number(grid.get("resolution"), "occupancy resolution")
    origin = grid.get("origin")
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise ValueError("occupancy dimensions and resolution must be positive")
    if not isinstance(origin, list) or len(origin) < 2:
        raise ValueError("occupancy origin must contain x and y")
    raw_data = grid.get("data")
    if not isinstance(raw_data, list) or len(raw_data) != width * height:
        raise ValueError("occupancy data length must equal width * height")
    data = [_integer(value, "occupancy cell") for value in raw_data]
    return (
        data,
        width,
        height,
        resolution,
        _number(origin[0], "occupancy origin x"),
        _number(origin[1], "occupancy origin y"),
        _number(grid.get("origin_yaw", 0.0), "occupancy origin yaw"),
        _number(pose[0], "vehicle map x"),
        _number(pose[1], "vehicle map y"),
    )


def _inside_raster(
    *, x: float, y: float, width: int, height: int, resolution: float,
    origin_x: float, origin_y: float, origin_yaw: float,
) -> bool:
    dx, dy = x - origin_x, y - origin_y
    cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    return 0.0 <= local_x < width * resolution and 0.0 <= local_y < height * resolution


def _known_free_inside_geofence(
    *, data: list[int], width: int, height: int, resolution: float,
    origin_x: float, origin_y: float, origin_yaw: float,
    geofence: tuple[tuple[float, float], ...],
) -> int:
    cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)
    count = 0
    for row in range(height):
        local_y = (row + 0.5) * resolution
        for column in range(width):
            if not 0 <= data[row * width + column] <= 25:
                continue
            local_x = (column + 0.5) * resolution
            x = origin_x + cosine * local_x - sine * local_y
            y = origin_y + sine * local_x + cosine * local_y
            if _inside(x, y, geofence):
                count += 1
    return count


def inspect_snapshot(telemetry_path: Path, episode_manifest_path: Path) -> dict[str, Any]:
    """Return a compact, JSON-serializable diagnosis of one recorded map."""
    telemetry = json.loads(telemetry_path.read_text(encoding="utf-8"))
    if not isinstance(telemetry, dict):
        raise ValueError("dashboard telemetry root must be an object")
    contract = load_campus_map_contract(episode_manifest_path)
    (
        data, width, height, resolution, origin_x, origin_y, origin_yaw,
        robot_x, robot_y,
    ) = _mapping_fields(telemetry)
    known = sum(cell >= 0 for cell in data)
    free = sum(0 <= cell <= 25 for cell in data)
    occupied = sum(cell > 25 for cell in data)
    unknown = len(data) - known
    common = {
        "schema_version": 1,
        "input": {
            "dashboard_telemetry": str(telemetry_path),
            "episode_manifest": str(episode_manifest_path),
        },
        "map": {
            "width": width,
            "height": height,
            "resolution_m": resolution,
            "origin": [origin_x, origin_y, origin_yaw],
            "cells": {"known": known, "free": free, "occupied": occupied, "unknown": unknown},
        },
        "robot_pose_map": [robot_x, robot_y],
        "robot_inside_raster": _inside_raster(
            x=robot_x, y=robot_y, width=width, height=height,
            resolution=resolution, origin_x=origin_x, origin_y=origin_y,
            origin_yaw=origin_yaw,
        ),
    }
    downsample_stride = _integer(
        telemetry["visualization"]["occupancy_grid"].get("downsample_stride", 1),
        "occupancy downsample stride",
    )
    if downsample_stride < 1:
        raise ValueError("occupancy downsample stride must be positive")
    common["input"]["dashboard_projection"] = {
        "downsample_stride": downsample_stride,
        "source_dimensions": telemetry["visualization"]["occupancy_grid"].get(
            "source_dimensions"
        ),
    }
    # The browser state is intentionally a bounded visualization projection,
    # not the OccupancyGrid consumed by the production frontier explorer.
    # Applying the production selector to a coarsened raster can only diagnose
    # that projection; it must not make a production safety conclusion.
    if downsample_stride > 1:
        common["status"] = "DASHBOARD_PROJECTION_NOT_PRODUCTION_INPUT"
        common["frontier_goal_map"] = None
        common["actionable_causes"] = [
            "dashboard_projection_not_production_input"
        ]
        return common
    geometry_inputs = {
        "data": data, "width": width, "height": height,
        "resolution": resolution, "origin_x": origin_x, "origin_y": origin_y,
        "origin_yaw": origin_yaw, "geofence": contract.geofence,
    }
    selector_inputs = {
        **geometry_inputs,
        "robot_x": robot_x, "robot_y": robot_y,
    }
    goal = select_frontier_goal(**selector_inputs)
    common["frontier_goal_map"] = None if goal is None else [goal[0], goal[1]]
    if goal is not None:
        common["status"] = "FRONTIER_GOAL_AVAILABLE"
        return common

    causes: list[str] = []
    if not common["robot_inside_raster"]:
        causes.append("robot_outside_occupancy_raster")
    free_inside = _known_free_inside_geofence(**geometry_inputs)
    common["map"]["known_free_cells_inside_geofence"] = free_inside
    if free_inside == 0:
        causes.append("no_known_free_cell_inside_geofence")
    else:
        # These calls deliberately reuse the production selector with relaxed
        # policy knobs; they do not reimplement its reachability algorithm.
        loose_goal = select_frontier_goal(
            **selector_inputs,
            clearance_m=0.0,
            frontier_standoff_m=0.0,
            min_goal_distance_m=0.0,
        )
        footprint_goal = select_frontier_goal(
            **selector_inputs,
            min_goal_distance_m=0.0,
        )
        if loose_goal is None:
            causes.append("no_reachable_frontier_adjacent_safe_candidate")
        elif footprint_goal is None:
            causes.append("no_footprint_clear_frontier_candidate")
        else:
            causes.append("no_goal_after_exact_selector_policy")
    common["status"] = "NO_FRONTIER_GOAL"
    common["actionable_causes"] = causes
    return common


def _output_path(value: str) -> Path:
    path = Path(value).resolve()
    try:
        path.relative_to(REPOSITORY_ROOT)
    except ValueError as exc:
        raise ValueError("--output must remain below the repository root") from exc
    if not path.parent.is_dir():
        raise ValueError("--output parent directory must already exist")
    return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dashboard_telemetry", type=Path)
    parser.add_argument("episode_manifest", type=Path)
    parser.add_argument("--output", help="Optional JSON output path below this repository")
    args = parser.parse_args(argv)
    report = inspect_snapshot(args.dashboard_telemetry, args.episode_manifest)
    payload = json.dumps(report, sort_keys=True, separators=(",", ":"))
    print(payload)
    if args.output:
        _output_path(args.output).write_text(payload + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
