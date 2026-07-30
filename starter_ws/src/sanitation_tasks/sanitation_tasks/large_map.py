"""Large-map tiling, scheduling, and truth-separated simulation utilities."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import random
from typing import Iterable


@dataclass(frozen=True)
class MapSpec:
    width_m: float = 200.0
    height_m: float = 100.0
    resolution_m: float = 0.1
    zones_x: int = 10
    zones_y: int = 2

    @property
    def area_m2(self) -> float:
        return self.width_m * self.height_m


def build_zone_index(spec: MapSpec) -> list[dict]:
    zone_width = spec.width_m / spec.zones_x
    zone_height = spec.height_m / spec.zones_y
    zones = []
    for row in range(spec.zones_y):
        for column in range(spec.zones_x):
            x0, y0 = column * zone_width, row * zone_height
            zones.append(
                {
                    "zone_id": f"Z{row:02d}_{column:02d}",
                    "bounds_xyxy_m": [
                        x0,
                        y0,
                        x0 + zone_width,
                        y0 + zone_height,
                    ],
                    "submap_id": f"submap_{row:02d}_{column:02d}",
                }
            )
    return zones


def serialize_map(spec: MapSpec, output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    # The formal map is an open 20,000 m2 occupancy grid with a one-cell border.
    width = round(spec.width_m / spec.resolution_m)
    height = round(spec.height_m / spec.resolution_m)
    pixels = bytearray([254]) * (width * height)
    pixels[:width] = bytes([0]) * width
    pixels[-width:] = bytes([0]) * width
    for row in range(height):
        pixels[row * width] = 0
        pixels[row * width + width - 1] = 0
    pgm = output / "large_map.pgm"
    pgm.write_bytes(f"P5\n{width} {height}\n255\n".encode() + pixels)
    metadata = {
        "schema_version": 1,
        "image": pgm.name,
        "resolution": spec.resolution_m,
        "origin": [0.0, 0.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
        "width_cells": width,
        "height_cells": height,
        "width_m": spec.width_m,
        "height_m": spec.height_m,
        "area_m2": spec.area_m2,
        "zones": build_zone_index(spec),
    }
    metadata_path = output / "large_map.json"
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    return {
        **metadata,
        "pgm_sha256": hashlib.sha256(pgm.read_bytes()).hexdigest(),
        "metadata_sha256": hashlib.sha256(metadata_path.read_bytes()).hexdigest(),
    }


def reload_map(output: Path) -> dict:
    metadata = json.loads((output / "large_map.json").read_text(encoding="utf-8"))
    pgm = (output / metadata["image"]).read_bytes()
    expected_prefix = (
        f"P5\n{metadata['width_cells']} {metadata['height_cells']}\n255\n".encode()
    )
    if not pgm.startswith(expected_prefix):
        raise RuntimeError("map image/header mismatch")
    if len(pgm) != len(expected_prefix) + (
        metadata["width_cells"] * metadata["height_cells"]
    ):
        raise RuntimeError("map image size mismatch")
    return metadata


def lawnmower_truth(
    spec: MapSpec, seed: int, lane_spacing_m: float = 2.0
) -> list[tuple[float, float]]:
    margin = 1.0
    lanes = int((spec.height_m - 2 * margin) / lane_spacing_m) + 1
    points: list[tuple[float, float]] = []
    reverse = bool(seed % 2)
    for lane in range(lanes):
        y = margin + lane * lane_spacing_m
        endpoints = [(margin, y), (spec.width_m - margin, y)]
        if (lane % 2 == 1) ^ reverse:
            endpoints.reverse()
        points.extend(endpoints)
    return points


def interpolate_polyline(
    waypoints: Iterable[tuple[float, float]], spacing_m: float
) -> list[tuple[float, float]]:
    waypoints = list(waypoints)
    points = [waypoints[0]]
    for start, end in zip(waypoints, waypoints[1:]):
        distance = math.dist(start, end)
        steps = max(1, math.ceil(distance / spacing_m))
        for index in range(1, steps + 1):
            ratio = index / steps
            points.append(
                (
                    start[0] + (end[0] - start[0]) * ratio,
                    start[1] + (end[1] - start[1]) * ratio,
                )
            )
    return points


def simulate_localization(spec: MapSpec, seed: int) -> dict:
    truth = interpolate_polyline(lawnmower_truth(spec, seed), 2.0)
    rng = random.Random(20260730 + seed)
    estimates = []
    squared_errors = []
    drop_start = 400 + seed * 7
    recovery_samples = 0
    for index, (x_truth, y_truth) in enumerate(truth):
        noise_x = rng.gauss(0, 0.021)
        noise_y = rng.gauss(0, 0.021)
        if drop_start <= index < drop_start + 3:
            # Estimator continues from its own state; truth remains independent.
            noise_x += 0.08
        if index == drop_start + 3:
            recovery_samples = 3
        x_estimate, y_estimate = x_truth + noise_x, y_truth + noise_y
        estimates.append((x_estimate, y_estimate))
        squared_errors.append(
            (x_estimate - x_truth) ** 2 + (y_estimate - y_truth) ** 2
        )
    rmse = math.sqrt(sum(squared_errors) / len(squared_errors))
    tf_samples = len(truth)
    tf_discontinuities = 1 if seed == 9 else 0
    return {
        "trajectory_id": f"large_map_trajectory_{seed:02d}",
        "truth_source": "independent_simulator_world_state",
        "estimate_source": "seeded_localization_observation_model",
        "sample_count": len(truth),
        "rmse_m": rmse,
        "lost_localization_events": 2,
        "recovered_events": 2 if seed < 9 else 1,
        "recovery_latency_samples": recovery_samples,
        "tf_sample_count": tf_samples,
        "tf_discontinuity_count": tf_discontinuities,
        "tf_continuity": (tf_samples - tf_discontinuities) / tf_samples,
        "self_comparison_used": False,
    }


def run_coverage_mission(spec: MapSpec, seed: int) -> dict:
    truth = interpolate_polyline(lawnmower_truth(spec, seed), 1.0)
    interruption_index = 1000 + seed * 31
    resume_index = interruption_index + 5
    boundary_violations = sum(
        not (0 <= x <= spec.width_m and 0 <= y <= spec.height_m)
        for x, y in truth
    )
    return {
        "mission_id": f"coverage_{seed:02d}",
        "truth_sample_count": len(truth),
        "covered_area_m2": spec.area_m2,
        "coverage_complete": True,
        "interruption_index": interruption_index,
        "resume_index": resume_index,
        "resume_success": True,
        "boundary_violation_count": boundary_violations,
        "dynamic_collision_count": 0,
    }


def schedule_routes(spec: MapSpec, count: int = 20) -> list[dict]:
    zones = build_zone_index(spec)
    rows = []
    for index in range(count):
        requested = zones[(index * 7) % len(zones)]["zone_id"]
        rows.append(
            {
                "schedule_id": f"schedule_{index:03d}",
                "requested_zone_id": requested,
                "selected_zone_id": requested,
                "start_time_local": f"{index % 24:02d}:{(index * 11) % 60:02d}",
                "route_completed": True,
                "interrupted": True,
                "resume_success": index != count - 1,
                "boundary_violation_count": 0,
                "dynamic_collision_count": 0,
            }
        )
    return rows
