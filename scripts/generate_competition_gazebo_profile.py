#!/usr/bin/env python3
"""Generate deterministic competition-scale Gazebo runtime assets.

The 200 x 100 m occupancy grid represents the complete competition envelope.
The live mission is deliberately one representative zone so the full
start/pause/resume/stop/finish cycle can be demonstrated in minutes.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


WIDTH_M = 200.0
HEIGHT_M = 100.0
RESOLUTION_M = 0.1
WORLD_TO_MAP = (100.0, 50.0)
DEMO_ZONE = (6.0, 45.5, 18.0, 54.5)


def _paint_rectangle(
    pixels: bytearray,
    width: int,
    height: int,
    bounds: tuple[float, float, float, float],
    value: int = 0,
) -> None:
    x0, y0, x1, y1 = bounds
    c0 = max(0, int(x0 / RESOLUTION_M))
    c1 = min(width - 1, int(x1 / RESOLUTION_M))
    r0 = max(0, int(y0 / RESOLUTION_M))
    r1 = min(height - 1, int(y1 / RESOLUTION_M))
    for row in range(r0, r1 + 1):
        start = row * width + c0
        pixels[start : row * width + c1 + 1] = bytes([value]) * (c1 - c0 + 1)


def _zones() -> list[dict]:
    return [
        {
            "zone_id": f"Z{row:02d}_{column:02d}",
            "bounds_xyxy_m": [
                column * 20.0,
                row * 50.0,
                (column + 1) * 20.0,
                (row + 1) * 50.0,
            ],
        }
        for row in range(2)
        for column in range(10)
    ]


def _write_pgm(path: Path, width: int, height: int, pixels: bytearray) -> str:
    payload = f"P5\n{width} {height}\n255\n".encode() + pixels
    path.write_bytes(payload)
    return hashlib.sha256(payload).hexdigest()


def _write_map_yaml(path: Path, image: str, mode: str = "trinary") -> None:
    path.write_text(
        "\n".join(
            [
                f"image: {image}",
                f"mode: {mode}",
                f"resolution: {RESOLUTION_M}",
                "origin: [0.0, 0.0, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                "",
            ]
        ),
        encoding="utf-8",
    )


def generate(output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    width = round(WIDTH_M / RESOLUTION_M)
    height = round(HEIGHT_M / RESOLUTION_M)

    occupancy = bytearray([254]) * (width * height)
    _paint_rectangle(occupancy, width, height, (0.0, 0.0, 0.2, HEIGHT_M))
    _paint_rectangle(occupancy, width, height, (WIDTH_M - 0.2, 0.0, WIDTH_M, HEIGHT_M))
    _paint_rectangle(occupancy, width, height, (0.0, 0.0, WIDTH_M, 0.2))
    _paint_rectangle(occupancy, width, height, (0.0, HEIGHT_M - 0.2, WIDTH_M, HEIGHT_M))

    # Static collision geometry from sanitation_campus_large.sdf, transformed
    # from centered Gazebo world coordinates into the positive map frame.
    for center_x in (40.0, 100.0, 160.0):
        _paint_rectangle(
            occupancy, width, height,
            (center_x - 9.0, 88.5, center_x + 9.0, 97.5),
        )
    _paint_rectangle(occupancy, width, height, (142.0, 3.0, 170.0, 11.0))

    map_sha = _write_pgm(output / "competition_map.pgm", width, height, occupancy)
    _write_map_yaml(output / "competition_map.yaml", "competition_map.pgm")

    # Keepout repeats the surveyed static envelope. The speed mask is zero,
    # meaning no extra percentage reduction beyond the 1.0 m/s safety gate.
    keepout_sha = _write_pgm(output / "competition_keepout.pgm", width, height, occupancy)
    _write_map_yaml(output / "competition_keepout.yaml", "competition_keepout.pgm", "scale")
    speed = bytearray([254]) * (width * height)
    speed_sha = _write_pgm(output / "competition_speed.pgm", width, height, speed)
    _write_map_yaml(output / "competition_speed.yaml", "competition_speed.pgm", "scale")

    x0, y0, x1, y1 = DEMO_ZONE
    mission = output / "competition_zone_auto12.yaml"
    mission.write_text(
        f"""frame_id: map
mission_id: competition_live_zone_Z01_00
mode: coverage
profile: auto12_efficiency_v1
scope: representative_live_zone_on_full_competition_map
full_map_area_m2: {WIDTH_M * HEIGHT_M:.1f}
live_zone_area_m2: {(x1 - x0) * (y1 - y0):.1f}
robot_width_m: 1.32
operation_width_m: 1.32
min_turning_radius_m: 0.75
route_type: BOUSTROPHEDON
path_type: DUBIN
allow_overlap: true
expected_components: 7
outer_polygon:
  - [{x0}, {y0}]
  - [{x1}, {y0}]
  - [{x1}, {y1}]
  - [{x0}, {y1}]
exclusion_polygons: []
keepout_polygons: []
static_obstacles: []
world_to_map_translation: [{WORLD_TO_MAP[0]}, {WORLD_TO_MAP[1]}]
headland:
  enabled: true
  width_m: 1.80
safety_margin_m: 0.10
staging_offset_m: 1.50
robot_footprint:
  - [0.72, 0.66]
  - [0.72, -0.66]
  - [-0.58, -0.66]
  - [-0.58, 0.66]
""",
        encoding="utf-8",
    )

    manifest = {
        "schema_version": 1,
        "profile": "competition_gazebo_auto12",
        "truth_level": "LIVE_REPRESENTATIVE_ZONE_ON_FULL_SCALE_MAP",
        "full_map": {
            "width_m": WIDTH_M,
            "height_m": HEIGHT_M,
            "area_m2": WIDTH_M * HEIGHT_M,
            "resolution_m": RESOLUTION_M,
            "cells": [width, height],
            "zone_count": 20,
            "zones": _zones(),
            "occupancy_sha256": map_sha,
            "keepout_sha256": keepout_sha,
            "speed_sha256": speed_sha,
        },
        "live_demonstration": {
            "zone_id": "Z01_00",
            "bounds_xyxy_m": list(DEMO_ZONE),
            "area_m2": (x1 - x0) * (y1 - y0),
            "complete_operator_cycle": ["start", "pause", "resume", "stop", "finish"],
        },
        "vehicle_candidate": {
            "cleaning_width_m": 1.32,
            "brush_center_y_m": 0.52,
            "max_cleaning_speed_m_s": 1.0,
            "theoretical_peak_efficiency_m2_h": 4752.0,
            "offline_mean_effective_efficiency_m2_h": 4205.8,
        },
        "competition_truth": {
            "simulation_competition_matrix_pass": False,
            "real_domain_pass": False,
            "j6_toolchain_pass": False,
            "final_competition_evidence_complete": False,
            "remaining_blockers": [
                "learned five-class perception and spot-clean loop not integrated live",
                "real-domain replay and hardware evidence unavailable",
                "J6 toolchain and target-device runtime unavailable",
                "full 20000 m2 Gazebo cleaning endurance run not executed",
            ],
        },
    }
    (output / "competition_profile_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(generate(args.output), ensure_ascii=False))


if __name__ == "__main__":
    main()
