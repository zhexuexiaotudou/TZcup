#!/usr/bin/env python3
"""Prepare truthful FullCoverage probe/server configs from the saved-map mission."""

from __future__ import annotations

import argparse
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
    width = float(transverse.get("declared_effective_cleaning_width_m", 0.0))
    footprint = cleaning.get("footprint_xy_m")
    if width <= 0.0 or not isinstance(footprint, list) or len(footprint) < 3:
        raise PreparationError("invalid cleaning width/footprint")
    points = [[float(value) for value in point] for point in footprint]
    if any(len(point) != 2 or not all(math.isfinite(value) for value in point) for point in points):
        raise PreparationError("invalid deployed cleaning footprint vertices")
    radius = max(math.hypot(*point) for point in points)
    safety_margin = 0.10
    headland = math.ceil((radius + safety_margin + width / 2.0) * 100.0) / 100.0
    truth = mission.get("truth_boundary")
    if not isinstance(truth, dict) or truth.get("dirt_truth_used") is not False:
        raise PreparationError("saved-map mission has no dirt-truth prohibition")
    probe = {
        **mission,
        "mode": "coverage",
        "route_mode": "AREA_FILL",
        "coverage_planner_profile": "SKID_STEER_OPTIMIZED",
        "operation_width_m": width,
        "planning_swath_spacing_m": round(width * 0.80, 6),
        "route_type": "BOUSTROPHEDON",
        "path_type": "DUBIN",
        "allow_overlap": True,
        "exclusion_polygons": mission.get("exclusion_polygons", []),
        "headland": {"enabled": True, "width_m": headland},
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
            "operation_width": width,
            "min_turning_radius": 0.40,
            "linear_curv_change": 200.0,
            "default_headland_width": headland,
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
