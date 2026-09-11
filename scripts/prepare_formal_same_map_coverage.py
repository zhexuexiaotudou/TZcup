#!/usr/bin/env python3
"""Prepare truthful FullCoverage probe/server configs from the saved-map mission."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path

import yaml


class PreparationError(RuntimeError):
    pass


def _object(path: Path) -> dict:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise PreparationError(f"object expected: {path}")
    return value


def _continuous_cleaning_lane_spacing(sweeps: dict) -> tuple[float, dict]:
    """Return a fail-closed spacing from the actual transverse brush union.

    The outer side-brush envelope and a declared vehicle working width are not
    continuous cleaning bands.  A lane pitch larger than the widest continuous
    tool interval leaves repeated uncleaned strips.  The central roller is the
    only continuous transverse band in the checked formal mechanism, so retain
    a 20 mm overlap instead of treating the 1.32 m declaration as a swath.
    """
    try:
        left = sweeps["left_side_brush"]
        right = sweeps["right_side_brush"]
        roller = sweeps["central_roller"]
        intervals = {
            "left_side_brush": [float(left["center_xy_m"][1]) - float(left["radius_m"]),
                                float(left["center_xy_m"][1]) + float(left["radius_m"])],
            "right_side_brush": [float(right["center_xy_m"][1]) - float(right["radius_m"]),
                                 float(right["center_xy_m"][1]) + float(right["radius_m"])],
            "central_roller": [float(roller["center_xy_m"][1]) - float(roller["width_m"]) / 2.0,
                               float(roller["center_xy_m"][1]) + float(roller["width_m"]) / 2.0],
        }
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        raise PreparationError("motion profile lacks finite transverse brush intervals") from exc
    if any(not all(math.isfinite(value) for value in interval) or interval[0] >= interval[1]
           for interval in intervals.values()):
        raise PreparationError("motion profile has invalid transverse brush intervals")
    ordered = sorted(intervals.values())
    gaps = [[round(first[1], 6), round(second[0], 6)] for first, second in zip(ordered, ordered[1:])
            if second[0] > first[1]]
    continuous_width = intervals["central_roller"][1] - intervals["central_roller"][0]
    spacing = round(continuous_width - 0.020, 6)
    if spacing <= 0.0:
        raise PreparationError("continuous roller band cannot retain required overlap")
    return spacing, {"tool_intervals_y_m": intervals, "interior_gaps_y_m": gaps,
                     "continuous_band_width_m": round(continuous_width, 6),
                     "conservative_lane_spacing_m": spacing}


def prepare(mission_path: Path, motion_profile_path: Path) -> tuple[dict, dict]:
    mission = _object(mission_path)
    profile = _object(motion_profile_path)
    sweeps = profile.get("mechanism_sweeps")
    footprints = profile.get("motion_footprints")
    if not isinstance(sweeps, dict) or not isinstance(footprints, dict):
        raise PreparationError("motion profile lacks cleaning geometry")
    transverse = sweeps.get("transverse_union")
    cleaning = footprints.get("cleaning_deployed")
    if not isinstance(transverse, dict) or not isinstance(cleaning, dict):
        raise PreparationError("motion profile lacks deployed cleaning footprint")
    declared_width = float(transverse.get("declared_effective_cleaning_width_m", 0.0))
    lane_spacing, transverse_geometry = _continuous_cleaning_lane_spacing(sweeps)
    footprint = cleaning.get("footprint_xy_m")
    if declared_width <= 0.0 or not isinstance(footprint, list) or len(footprint) < 3:
        raise PreparationError("invalid cleaning width/footprint")
    points = [[float(value) for value in point] for point in footprint]
    if any(len(point) != 2 or not all(math.isfinite(value) for value in point) for point in points):
        raise PreparationError("invalid deployed cleaning footprint vertices")
    radius = max(math.hypot(*point) for point in points)
    safety_margin = 0.10
    # Keep the declared envelope for clearance.  Only swath pitch is narrowed
    # to the observed continuous cleaning band.
    expected_clearance = math.ceil((radius + safety_margin + declared_width / 2.0) * 100.0) / 100.0
    truth = mission.get("truth_boundary")
    if not isinstance(truth, dict) or truth.get("dirt_truth_used") is not False:
        raise PreparationError("saved-map mission has no dirt-truth prohibition")
    coverage = mission.get("saved_occupancy_coverage")
    if not isinstance(coverage, dict) or coverage.get("source") != "saved_slam_occupancy_only":
        raise PreparationError("saved-map mission has no sealed SLAM coverage geometry")
    geometry_name = coverage.get("geometry")
    geometry_hash = coverage.get("sha256")
    if not isinstance(geometry_name, str) or Path(geometry_name).name != geometry_name or not isinstance(geometry_hash, str):
        raise PreparationError("saved-map coverage geometry reference is invalid")
    geometry_path = mission_path.parent / geometry_name
    if not geometry_path.is_file() or hashlib.sha256(geometry_path.read_bytes()).hexdigest() != geometry_hash:
        raise PreparationError("saved-map coverage geometry hash differs")
    geometry = _object(geometry_path)
    outer = geometry.get("planning_outer_polygon")
    exclusions = geometry.get("planning_hole_polygons")
    if (geometry.get("source") != "saved_slam_occupancy_only" or not isinstance(outer, list)
            or not isinstance(exclusions, list)):
        raise PreparationError("saved-map coverage geometry is not shared by the mission")
    try:
        headland = float(geometry["planning_clearance_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise PreparationError("saved-map coverage clearance is invalid") from exc
    if (not math.isfinite(headland) or not math.isclose(headland, expected_clearance, abs_tol=1e-9)
            or geometry.get("obstacle_inflation_m") != headland
            or mission.get("headland") != {"enabled": True, "width_m": headland}):
        raise PreparationError("saved-map coverage clearance is not shared by the mission")
    try:
        free_cells = int(geometry["reachable_cleanable_cells"])
        resolution = float(geometry["resolution_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise PreparationError("saved-map coverage cells are invalid") from exc
    if free_cells <= 0 or not math.isfinite(resolution) or resolution <= 0.0:
        raise PreparationError("saved-map coverage cells are invalid")
    probe = {
        **mission,
        "mode": "coverage",
        "route_mode": "AREA_FILL",
        "coverage_planner_profile": "SKID_STEER_OPTIMIZED",
        "operation_width_m": lane_spacing,
        "planning_swath_spacing_m": lane_spacing,
        "declared_effective_cleaning_width_m": declared_width,
        "transverse_cleaning_geometry": transverse_geometry,
        "route_type": "BOUSTROPHEDON",
        "path_type": "DUBIN",
        "allow_overlap": True,
        "outer_polygon": outer,
        "keepout_polygons": exclusions,
        "exclusion_polygons": exclusions,
        "coverage_geometry_sha256": geometry_hash,
        "coverage_geometry": {"sha256": geometry_hash, "planning_clearance_m": headland,
            "resolution_m": resolution, "free_cell_count": free_cells,
            "cleanable_area_m2": free_cells * resolution * resolution},
        "headland": {"enabled": False, "width_m": 0.0},
        "safety_margin_m": safety_margin,
        "staging_offset_m": headland,
        "optimized_staging_offset_m": headland,
        "robot_footprint": points,
        "world_to_map_translation": [0.0, 0.0],
        "empirical_coverage_threshold": 0.98,
        "empirical_repeat_rate_threshold": 1.0,
        "empirical_swath_lateral_p95_threshold_m": 0.08,
        "evaluation_brush_dropout": {"enabled": False},
    }
    server = {
        "coverage_server": {"ros__parameters": {
            "use_sim_time": True,
            "action_server_result_timeout": 30.0,
            "coordinates_in_cartesian_frame": True,
            "robot_width": max(y for _, y in points) - min(y for _, y in points),
            "operation_width": lane_spacing,
            "min_turning_radius": 0.40,
            "linear_curv_change": 200.0,
            "default_headland_width": 0.0,
            "default_headland_type": "CONSTANT",
            "default_allow_overlap": True,
            "default_swath_type": "COVERAGE",
            "default_swath_angle_type": "SET_ANGLE",
            "default_swath_angle": 0.0,
            "default_route_type": "BOUSTROPHEDON",
            "default_path_type": "DUBIN",
            "default_path_continuity_type": "DISCONTINUOUS",
            "default_turn_point_distance": 0.10,
            "max_turn_angular_velocity": 0.60,
        }}
    }
    return probe, server


def _write(path: Path, value: dict) -> None:
    if path.exists():
        raise PreparationError(f"refusing to overwrite retained config: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_suffix(path.suffix + f".pending.{os.getpid()}")
    pending.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    pending.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--motion-profile", type=Path, required=True)
    parser.add_argument("--probe-output", type=Path, required=True)
    parser.add_argument("--server-output", type=Path, required=True)
    args = parser.parse_args()
    try:
        probe, server = prepare(args.mission, args.motion_profile)
        _write(args.probe_output, probe)
        _write(args.server_output, server)
    except (PreparationError, OSError, UnicodeError, yaml.YAMLError, ValueError) as exc:
        print(json.dumps({"status": "INVALID", "error": str(exc)}, indent=2))
        return 2
    print(json.dumps({"status": "READY", "probe": str(args.probe_output),
                      "server": str(args.server_output)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
