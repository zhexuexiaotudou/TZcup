#!/usr/bin/env python3
"""Materialize eight random walkers with mission-corridor interactions.

This is environment/evaluator orchestration, not a product planner.  The fixed
mission leg comes only from the public episode manifest; pedestrian routes are
generated afterwards and are never supplied to Nav2 or the safety controller.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import random
from typing import Any
import xml.etree.ElementTree as ET

from validate_formal_dynamic_obstacle_avoidance import (
    load_public_mission_contract,
    point_in_polygon,
)


def _read_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def _numbers(text: str | None, *, count: int) -> list[float]:
    values = [float(value) for value in (text or "").split()]
    if len(values) < count:
        raise ValueError(f"expected at least {count} numeric values, got {text!r}")
    return values


def static_exclusions_from_public_world(
    world_path: Path,
) -> list[tuple[float, float, float]]:
    """Return conservative XY bounding circles for public static assets."""

    root = ET.parse(world_path).getroot()
    exclusions: list[tuple[float, float, float]] = []
    for model in root.findall(".//world/model"):
        if not model.attrib.get("name", "").startswith("asset_"):
            continue
        pose = _numbers(model.findtext("pose"), count=2)
        radii: list[float] = []
        for collision in model.findall("./link/collision"):
            box = collision.find("./geometry/box/size")
            cylinder = collision.find("./geometry/cylinder/radius")
            if box is not None:
                size = _numbers(box.text, count=2)
                radii.append(math.hypot(size[0], size[1]) / 2.0)
            elif cylinder is not None:
                radii.append(float(cylinder.text or "0"))
        if radii:
            exclusions.append((pose[0], pose[1], max(radii)))
    return exclusions


def walker_ids_from_public_world(world_path: Path) -> list[str]:
    root = ET.parse(world_path).getroot()
    return sorted(
        model.attrib.get("name", "")
        for model in root.findall(".//world/model")
        if model.attrib.get("name", "").startswith("walker_")
    )


def _route_clear(
    first: tuple[float, float],
    second: tuple[float, float],
    exclusions: list[tuple[float, float, float]],
    polygon: list[tuple[float, float]],
    *,
    pedestrian_radius_m: float,
    margin_m: float = 0.75,
) -> bool:
    distance = math.dist(first, second)
    sample_count = max(2, math.ceil(distance / 0.25) + 1)
    for index in range(sample_count):
        ratio = index / (sample_count - 1)
        point = (
            first[0] + ratio * (second[0] - first[0]),
            first[1] + ratio * (second[1] - first[1]),
        )
        if not point_in_polygon(point, polygon, boundary_is_inside=True):
            return False
        if any(
            math.hypot(point[0] - x, point[1] - y)
            < pedestrian_radius_m + radius + margin_m
            for x, y, radius in exclusions
        ):
            return False
    return True


def materialize_schedule(
    *,
    episode_manifest: Path,
    public_world: Path,
    base_schedule: Path,
    seed: int,
    nominal_leg_m: float,
    crossing_count: int = 3,
) -> dict[str, Any]:
    mission = load_public_mission_contract(
        episode_manifest, nominal_leg_m=nominal_leg_m
    )
    schedule = _read_object(base_schedule)
    if schedule.get("access") != "environment_driver_only_not_robot_control":
        raise ValueError("base schedule lacks environment-only access boundary")
    pedestrians = schedule.get("pedestrians")
    if not isinstance(pedestrians, list) or len(pedestrians) != 8:
        raise ValueError("formal dynamic acceptance requires exactly eight walkers")
    scheduled_ids = [str(row.get("object_id", "")) for row in pedestrians]
    world_walker_ids = walker_ids_from_public_world(public_world)
    if len(set(scheduled_ids)) != 8:
        raise ValueError("pedestrian schedule object IDs must be unique")
    if sorted(scheduled_ids) != world_walker_ids:
        raise ValueError(
            "pedestrian schedule IDs must exactly match the eight public world walkers"
        )
    if crossing_count < 1 or crossing_count > len(pedestrians):
        raise ValueError("crossing_count is outside the pedestrian count")

    rng = random.Random(seed)
    # Pedestrian SetEntityPose commands use Gazebo/source-world coordinates,
    # while the product goal and AMCL telemetry use the saved-map local frame.
    start_x, start_y, _ = mission["source_fixed_start_pose"]
    goal_x, goal_y, _ = mission["goal_pose_source_world"]
    dx, dy = goal_x - start_x, goal_y - start_y
    length = math.hypot(dx, dy)
    tangent = (dx / length, dy / length)
    normal = (-tangent[1], tangent[0])
    polygon = [
        tuple(row) for row in mission["geofence_polygon_source_world"]
    ]
    exclusions = static_exclusions_from_public_world(public_world)

    # Random candidates stay within the central 60% of the fixed public leg.
    # Selecting them never reads the base pedestrian routes.
    fractions = [0.20 + 0.60 * index / 80.0 for index in range(81)]
    rng.shuffle(fractions)
    selected: list[tuple[float, float, float]] = []
    for fraction in fractions:
        center = (start_x + fraction * dx, start_y + fraction * dy)
        half_span = rng.uniform(3.5, 5.5)
        first = (
            center[0] - normal[0] * half_span,
            center[1] - normal[1] * half_span,
        )
        second = (
            center[0] + normal[0] * half_span,
            center[1] + normal[1] * half_span,
        )
        radius = float(pedestrians[len(selected)].get("radius_m", 0.25))
        if not _route_clear(
            first,
            second,
            exclusions,
            polygon,
            pedestrian_radius_m=radius,
        ):
            continue
        if any(math.hypot(center[0] - x, center[1] - y) < 4.0 for x, y, _ in selected):
            continue
        selected.append((center[0], center[1], half_span))
        if len(selected) == crossing_count:
            break
    if len(selected) != crossing_count:
        raise ValueError("could not place enough obstacle-free mission crossings")

    crossing_ids: list[str] = []
    for pedestrian, (center_x, center_y, half_span) in zip(
        pedestrians, selected
    ):
        speed = rng.uniform(0.45, 0.80)
        first = (
            center_x - normal[0] * half_span,
            center_y - normal[1] * half_span,
        )
        second = (
            center_x + normal[0] * half_span,
            center_y + normal[1] * half_span,
        )
        if rng.random() < 0.5:
            first, second = second, first
        duration = math.dist(first, second) / speed
        pedestrian["speed_mps"] = round(speed, 4)
        pedestrian["waypoints"] = [
            [0.0, round(first[0], 6), round(first[1], 6)],
            [round(duration, 4), round(second[0], 6), round(second[1], 6)],
            [round(2.0 * duration, 4), round(first[0], 6), round(first[1], 6)],
        ]
        crossing_ids.append(str(pedestrian["object_id"]))

    mission_digest = hashlib.sha256(
        json.dumps(mission, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    schedule["acceptance_environment"] = {
        "schema_version": 1,
        "seed": seed,
        "randomized_each_run_unless_seed_pinned": True,
        "mission_contract_sha256": mission_digest,
        "goal_source": mission["goal_source"],
        "goal_pose_map": mission["goal_pose_map"],
        "goal_pose_source_world": mission["goal_pose_source_world"],
        "mission_corridor_crossing_count": crossing_count,
        "mission_corridor_crossing_ids": crossing_ids,
        "pedestrian_model_ids": world_walker_ids,
        "product_control_access_prohibited": True,
    }
    return schedule


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode-manifest", type=Path, required=True)
    parser.add_argument("--public-world", type=Path, required=True)
    parser.add_argument("--base-schedule", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--nominal-leg", type=float, default=30.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    value = materialize_schedule(
        episode_manifest=args.episode_manifest,
        public_world=args.public_world,
        base_schedule=args.base_schedule,
        seed=args.seed,
        nominal_leg_m=args.nominal_leg,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
