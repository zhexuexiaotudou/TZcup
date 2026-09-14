#!/usr/bin/env python3
"""Search a deterministic, obstacle-free short functional-smoke route."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any

from prepare_dynamic_avoidance_single_run import (
    FUNCTIONAL_SMOKE_MODE,
    corridor_fraction_range,
    freeze_route_from_schedule,
    protocol_mode,
    scheduled_crossing_count,
    validate_protocol,
)
from prepare_formal_dynamic_obstacle_schedule import (
    MIN_CROSSING_CENTER_SEPARATION_M,
    _require_all_pedestrian_paths_clear,
    materialize_schedule,
    static_exclusions_from_public_world,
)
from validate_formal_dynamic_obstacle_avoidance import (
    load_public_mission_contract,
    point_in_polygon,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_object(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file() or path.is_symlink():
        raise ValueError(f"{label} must be a regular file: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"{label} is not valid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{label} JSON root must be an object")
    return value


def corridor_capacity(
    *,
    nominal_leg_m: float,
    fraction_start: float,
    fraction_end: float,
    crossing_count: int,
) -> dict[str, Any]:
    span_m = nominal_leg_m * (fraction_end - fraction_start)
    required_span_m = MIN_CROSSING_CENTER_SEPARATION_M * (crossing_count - 1)
    return {
        "available_centerline_span_m": span_m,
        "required_center_span_m": required_span_m,
        "geometrically_feasible": span_m + 1.0e-12 >= required_span_m,
    }


def _route_samples(
    waypoints: list[list[float]],
) -> list[tuple[float, float]]:
    first, second = waypoints[0], waypoints[1]
    first_xy = (float(first[1]), float(first[2]))
    second_xy = (float(second[1]), float(second[2]))
    distance = math.dist(first_xy, second_xy)
    sample_count = max(2, math.ceil(distance / 0.25) + 1)
    return [
        (
            first_xy[0] + index / (sample_count - 1) * (second_xy[0] - first_xy[0]),
            first_xy[1] + index / (sample_count - 1) * (second_xy[1] - first_xy[1]),
        )
        for index in range(sample_count)
    ]


def _materialization_proof(
    *,
    schedule: dict[str, Any],
    route_manifest: dict[str, Any],
    episode_manifest: Path,
    public_world: Path,
    nominal_leg_m: float,
    fraction_start: float,
    fraction_end: float,
) -> dict[str, Any]:
    environment = schedule["acceptance_environment"]
    crossing_ids = environment["mission_corridor_crossing_ids"]
    selected = next(
        row
        for row in schedule["pedestrians"]
        if row["object_id"] == crossing_ids[0]
    )
    waypoints = selected["waypoints"]
    mission = load_public_mission_contract(
        episode_manifest,
        nominal_leg_m=nominal_leg_m,
    )
    exclusions = static_exclusions_from_public_world(public_world)
    samples = _route_samples(waypoints)
    radius = float(selected["radius_m"])
    minimum_static_surface_clearance_m = min(
        math.hypot(x - asset_x, y - asset_y) - asset_radius - radius
        for x, y in samples
        for asset_x, asset_y, asset_radius in exclusions
    )
    polygon = [
        (float(row[0]), float(row[1]))
        for row in mission["geofence_polygon_source_world"]
    ]
    if not all(point_in_polygon(point, polygon, boundary_is_inside=True) for point in samples):
        raise ValueError("selected route leaves the public geofence")
    _require_all_pedestrian_paths_clear(schedule["pedestrians"])
    first_xy = (float(waypoints[0][1]), float(waypoints[0][2]))
    second_xy = (float(waypoints[1][1]), float(waypoints[1][2]))
    center_xy = (
        (first_xy[0] + second_xy[0]) / 2.0,
        (first_xy[1] + second_xy[1]) / 2.0,
    )
    start = mission["source_fixed_start_pose"]
    mission_x_m = math.cos(start[2]) * (center_xy[0] - start[0]) + math.sin(
        start[2]
    ) * (center_xy[1] - start[1])
    heading_delta_y = float(waypoints[1][2]) - float(waypoints[0][2])
    route_heading = "positive_y" if heading_delta_y > 0.0 else "negative_y"
    static_interaction = route_manifest["static_interaction_materialization"]
    return {
        "selected_object_id": crossing_ids[0],
        "crossing_count": len(crossing_ids),
        "route_source_world": waypoints,
        "route_surface_length_m": math.dist(first_xy, second_xy),
        "route_heading": route_heading,
        "route_heading_source_world": [0.0, 1.0 if heading_delta_y > 0.0 else -1.0],
        "center_source_world": list(center_xy),
        "center_mission_x_m": mission_x_m,
        "minimum_static_surface_clearance_m": minimum_static_surface_clearance_m,
        "required_static_surface_clearance_m": 0.75,
        "inside_public_geofence": True,
        "complete_schedule_peer_gate_passed": True,
        "corridor_fraction_range": [fraction_start, fraction_end],
        "corridor_centerline_span_m": nominal_leg_m
        * (fraction_end - fraction_start),
        "static_interaction_materialization": static_interaction,
    }


def search_route(
    *,
    episode_manifest: Path,
    public_world: Path,
    base_schedule: Path,
    protocol: dict[str, Any],
    protocol_sha256: str,
    input_sha256: dict[str, str],
    max_seed_candidates: int,
    corridor_ranges: list[tuple[str, float, float]],
) -> dict[str, Any]:
    validate_protocol(protocol)
    if protocol_mode(protocol) != FUNCTIONAL_SMOKE_MODE:
        raise ValueError("functional route search requires the smoke protocol")
    target_crossings = scheduled_crossing_count(protocol)
    declared_range = corridor_fraction_range(protocol)
    if not corridor_ranges:
        corridor_ranges = [("declared", *declared_range)]
    base_seed = int(protocol["schedule"]["seed"])
    nominal_leg_m = float(protocol["schedule"]["nominal_leg_m"])

    capacity_evidence: list[dict[str, Any]] = []
    for crossing_count in (3, 2, target_crossings):
        for label, start, end in corridor_ranges:
            proof = corridor_capacity(
                nominal_leg_m=nominal_leg_m,
                fraction_start=start,
                fraction_end=end,
                crossing_count=crossing_count,
            )
            capacity_evidence.append(
                {
                    "crossing_count": crossing_count,
                    "corridor_id": label,
                    "fraction_range": [start, end],
                    **proof,
                }
            )

    attempted = 0
    failures: list[dict[str, Any]] = []
    candidate_evidence: list[dict[str, Any]] = []
    selected_result: dict[str, Any] | None = None
    for corridor_id, fraction_start, fraction_end in corridor_ranges:
        capacity = corridor_capacity(
            nominal_leg_m=nominal_leg_m,
            fraction_start=fraction_start,
            fraction_end=fraction_end,
            crossing_count=target_crossings,
        )
        if not capacity["geometrically_feasible"]:
            continue
        for offset in range(max_seed_candidates):
            seed = base_seed + offset
            attempted += 1
            try:
                candidate_protocol = copy.deepcopy(protocol)
                candidate_protocol["schedule"]["seed"] = seed
                candidate_protocol_sha256 = hashlib.sha256(
                    (
                        json.dumps(candidate_protocol, indent=2, sort_keys=True)
                        + "\n"
                    ).encode("utf-8")
                ).hexdigest()
                schedule = materialize_schedule(
                    episode_manifest=episode_manifest,
                    public_world=public_world,
                    base_schedule=base_schedule,
                    seed=seed,
                    nominal_leg_m=nominal_leg_m,
                    crossing_count=target_crossings,
                    corridor_fraction_start=fraction_start,
                    corridor_fraction_end=fraction_end,
                )
                canonical_schedule = (
                    json.dumps(schedule, indent=2, sort_keys=True) + "\n"
                ).encode("utf-8")
                route_manifest = freeze_route_from_schedule(
                    schedule=schedule,
                    episode_manifest=episode_manifest,
                    protocol=candidate_protocol,
                    protocol_sha256=candidate_protocol_sha256,
                    schedule_sha256=hashlib.sha256(canonical_schedule).hexdigest(),
                    input_sha256=input_sha256,
                    declared_epoch_ns=1,
                )
                proof = _materialization_proof(
                    schedule=schedule,
                    route_manifest=route_manifest,
                    episode_manifest=episode_manifest,
                    public_world=public_world,
                    nominal_leg_m=nominal_leg_m,
                    fraction_start=fraction_start,
                    fraction_end=fraction_end,
                )
                static_interaction = proof["static_interaction_materialization"]
                candidate_evidence.append(
                    {
                        "seed": seed,
                        "corridor_id": corridor_id,
                        "corridor_fraction_range": [
                            fraction_start,
                            fraction_end,
                        ],
                        "crossing_mission_x_m": proof["center_mission_x_m"],
                        "route_heading": proof["route_heading"],
                        "trigger_time_from_schedule_start_s": route_manifest[
                            "selected_obstacle"
                        ]["trigger_time_from_schedule_start_s"],
                        "passed": static_interaction["passed"],
                        "selected_interaction_sample": static_interaction[
                            "selected_interaction_sample"
                        ],
                    }
                )
                if not static_interaction["passed"]:
                    failures.append(
                        {
                            "seed": seed,
                            "corridor_id": corridor_id,
                            "crossing_count": target_crossings,
                            "error": static_interaction["failure_reason"],
                            "route_heading": proof["route_heading"],
                            "crossing_mission_x_m": proof["center_mission_x_m"],
                            "trigger_time_from_schedule_start_s": (
                                route_manifest["selected_obstacle"][
                                    "trigger_time_from_schedule_start_s"
                                ]
                            ),
                        }
                    )
                    continue
            except (KeyError, StopIteration, TypeError, ValueError) as exc:
                failures.append(
                    {
                        "seed": seed,
                        "corridor_id": corridor_id,
                        "crossing_count": target_crossings,
                        "error": str(exc),
                    }
                )
                continue
            if selected_result is None:
                selected_result = {
                    "schema_version": 1,
                    "kind": "tzcup_dynamic_avoidance_functional_route_search",
                    "status": "FUNCTIONAL_SMOKE_ROUTE_MATERIALIZED",
                    "mode": FUNCTIONAL_SMOKE_MODE,
                    "protocol_id": candidate_protocol["protocol_id"],
                    "input_protocol_sha256": protocol_sha256,
                    "selected_protocol_sha256": candidate_protocol_sha256,
                    "protocol_sha256": candidate_protocol_sha256,
                    "input_sha256": input_sha256,
                    "nominal_leg_m": nominal_leg_m,
                    "selected_seed": seed,
                    "selected_protocol_schedule": candidate_protocol["schedule"],
                    "selected_corridor_id": corridor_id,
                    "selected_corridor_fraction_range": [
                        fraction_start,
                        fraction_end,
                    ],
                    "selected_crossing_count": target_crossings,
                    "selected_route_heading": proof["route_heading"],
                    "selected_trigger_time_from_schedule_start_s": route_manifest[
                        "selected_obstacle"
                    ]["trigger_time_from_schedule_start_s"],
                    "route_materialization": proof,
                    "predeclared_route_manifest": route_manifest,
                    "capacity_evidence": capacity_evidence,
                    "attempted_candidate_count": attempted,
                    "candidate_evidence": candidate_evidence,
                    "retained_failure_count": len(failures),
                    "retained_failure_examples": failures[:20],
                    "claim_boundary": (
                        "Static route materialization against the frozen public "
                        "episode, world, and offline source hash. It does not claim "
                        "Gazebo execution, collision-free runtime behavior, or the "
                        "official >=95% rate."
                    ),
                }

    if selected_result is not None:
        selected_result["attempted_candidate_count"] = attempted
        selected_result["candidate_evidence"] = candidate_evidence
        selected_result["retained_failure_count"] = len(failures)
        selected_result["retained_failure_examples"] = failures[:20]
        return selected_result

    return {
        "schema_version": 1,
        "kind": "tzcup_dynamic_avoidance_functional_route_search",
        "status": "NO_FUNCTIONAL_SMOKE_ROUTE_MATERIALIZED",
        "mode": FUNCTIONAL_SMOKE_MODE,
        "protocol_id": protocol["protocol_id"],
        "protocol_sha256": protocol_sha256,
        "input_sha256": input_sha256,
        "nominal_leg_m": nominal_leg_m,
        "selected_seed": None,
        "selected_corridor_id": None,
        "selected_corridor_fraction_range": None,
        "selected_crossing_count": target_crossings,
        "capacity_evidence": capacity_evidence,
        "attempted_candidate_count": attempted,
        "candidate_evidence": candidate_evidence,
        "retained_failure_count": len(failures),
        "retained_failure_examples": failures[:20],
        "claim_boundary": (
            "Static route search exhausted the declared candidate family without "
            "materializing a valid functional-smoke route. Gazebo must not start."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, required=True)
    parser.add_argument("--episode-manifest", type=Path, required=True)
    parser.add_argument("--public-world", type=Path, required=True)
    parser.add_argument("--base-schedule", type=Path, required=True)
    parser.add_argument("--maximum-seed-candidates", type=int, default=256)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.maximum_seed_candidates < 1:
        raise SystemExit("maximum seed candidates must be positive")
    if args.output.exists():
        raise SystemExit(f"fresh route-search output required: {args.output}")
    protocol = _read_object(args.protocol, "protocol")
    for path, label in (
        (args.episode_manifest, "episode manifest"),
        (args.public_world, "public world"),
        (args.base_schedule, "base schedule"),
    ):
        if not path.is_file() or path.is_symlink():
            raise SystemExit(f"{label} must be a regular file: {path}")
    result = search_route(
        episode_manifest=args.episode_manifest,
        public_world=args.public_world,
        base_schedule=args.base_schedule,
        protocol=protocol,
        protocol_sha256=_sha256(args.protocol),
        input_sha256={
            "episode_manifest": _sha256(args.episode_manifest),
            "public_world": _sha256(args.public_world),
            "base_schedule": _sha256(args.base_schedule),
        },
        max_seed_candidates=args.maximum_seed_candidates,
        corridor_ranges=[
            ("declared", *corridor_fraction_range(protocol)),
        ],
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    pending = args.output.with_suffix(args.output.suffix + f".pending.{os.getpid()}")
    pending.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    pending.replace(args.output)
    print(args.output)
    return 0 if result["status"] == "FUNCTIONAL_SMOKE_ROUTE_MATERIALIZED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
