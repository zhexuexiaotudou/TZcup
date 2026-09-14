#!/usr/bin/env python3
"""Freeze one deterministic dynamic-obstacle route before a live trial."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import time
from typing import Any

from validate_formal_dynamic_obstacle_avoidance import (
    load_public_mission_contract,
)


ROUTE_MANIFEST_KIND = "tzcup_predeclared_dynamic_avoidance_obstacle_route"
ROUTE_SELECTION = "first_acceptance_environment_mission_corridor_crossing"
OFFICIAL_MODE = "OFFICIAL_SINGLE_RUN"
FUNCTIONAL_SMOKE_MODE = "FUNCTIONAL_SMOKE_NOT_OFFICIAL_95"
SUPPORTED_MODES = (OFFICIAL_MODE, FUNCTIONAL_SMOKE_MODE)
OFFICIAL_CROSSING_COUNT = 3
FUNCTIONAL_SMOKE_CROSSING_COUNT = 1


def protocol_mode(protocol: dict[str, Any]) -> str:
    return str(protocol.get("mode", OFFICIAL_MODE))


def scheduled_crossing_count(protocol: dict[str, Any]) -> int:
    schedule = protocol.get("schedule")
    value = (
        schedule.get("mission_corridor_crossing_count")
        if isinstance(schedule, dict)
        else None
    )
    expected = (
        FUNCTIONAL_SMOKE_CROSSING_COUNT
        if protocol_mode(protocol) == FUNCTIONAL_SMOKE_MODE
        else OFFICIAL_CROSSING_COUNT
    )
    if isinstance(value, bool) or not isinstance(value, int) or value != expected:
        raise ValueError(
            f"{protocol_mode(protocol)} requires {expected} mission-corridor crossings"
        )
    return value


def corridor_fraction_range(protocol: dict[str, Any]) -> tuple[float, float]:
    schedule = protocol.get("schedule")
    raw = (
        schedule.get("corridor_fraction_range")
        if isinstance(schedule, dict)
        else None
    )
    if (
        not isinstance(raw, list)
        or len(raw) != 2
        or any(isinstance(value, bool) for value in raw)
        or not all(isinstance(value, (int, float)) for value in raw)
    ):
        raise ValueError("schedule.corridor_fraction_range must contain two numbers")
    start, end = (float(raw[0]), float(raw[1]))
    if not math.isfinite(start) or not math.isfinite(end):
        raise ValueError("corridor fraction range must be finite")
    if not 0.0 <= start < end <= 1.0:
        raise ValueError("corridor fraction range must satisfy 0 <= start < end <= 1")
    return start, end


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


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_schedule_bytes(schedule: dict[str, Any]) -> bytes:
    return (json.dumps(schedule, indent=2, sort_keys=True) + "\n").encode("utf-8")


def validate_protocol(protocol: dict[str, Any]) -> None:
    if protocol.get("schema_version") != 1:
        raise ValueError("protocol schema_version must equal 1")
    if not isinstance(protocol.get("protocol_id"), str) or not protocol[
        "protocol_id"
    ]:
        raise ValueError("protocol_id is required")
    mode = protocol_mode(protocol)
    if mode not in SUPPORTED_MODES:
        raise ValueError("unsupported protocol mode")
    official = protocol.get("official_metric")
    if not isinstance(official, dict):
        raise ValueError("official_metric is required")
    if official.get("threshold") != 0.95:
        raise ValueError("official dynamic-avoidance threshold must equal 0.95")
    if official.get("status") != "NOT_MEASURED":
        raise ValueError("offline preparation must retain NOT_MEASURED status")
    if official.get("single_run_can_prove_threshold") is not False:
        raise ValueError("protocol must deny a single-run threshold claim")
    schedule = protocol.get("schedule")
    if not isinstance(schedule, dict):
        raise ValueError("schedule contract is required")
    seed = schedule.get("seed")
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ValueError("schedule.seed must be an integer")
    nominal_leg = schedule.get("nominal_leg_m")
    if (
        isinstance(nominal_leg, bool)
        or not isinstance(nominal_leg, (int, float))
        or not math.isfinite(float(nominal_leg))
        or float(nominal_leg) < 5.0
    ):
        raise ValueError("schedule.nominal_leg_m must be finite and at least 5 m")
    if mode == FUNCTIONAL_SMOKE_MODE and not math.isclose(
        float(nominal_leg), 6.0, rel_tol=0.0, abs_tol=1.0e-9
    ):
        raise ValueError(
            "functional-smoke nominal_leg_m must equal the declared 6.0 m"
        )
    scheduled_crossing_count(protocol)
    corridor_fraction_range(protocol)
    if schedule.get("route_selection") != ROUTE_SELECTION:
        raise ValueError("unsupported obstacle route selection")
    if schedule.get("route_declaration_required_before_run_start") is not True:
        raise ValueError("route declaration must precede the live run")
    obstacle = protocol.get("obstacle")
    if not isinstance(obstacle, dict):
        raise ValueError("obstacle contract is required")
    radius = obstacle.get("radius_m")
    if (
        isinstance(radius, bool)
        or not isinstance(radius, (int, float))
        or not math.isfinite(float(radius))
        or float(radius) <= 0.0
    ):
        raise ValueError("obstacle.radius_m must be positive and finite")
    minimum_distance = obstacle.get("minimum_distance")
    if not isinstance(minimum_distance, dict):
        raise ValueError("obstacle.minimum_distance is required")
    threshold = minimum_distance.get("threshold_m")
    if (
        isinstance(threshold, bool)
        or not isinstance(threshold, (int, float))
        or not math.isfinite(float(threshold))
        or float(threshold) <= 0.0
    ):
        raise ValueError("minimum-distance threshold must be positive and finite")
    envelope = minimum_distance.get("vehicle_envelope_radius_m")
    if (
        isinstance(envelope, bool)
        or not isinstance(envelope, (int, float))
        or not math.isfinite(float(envelope))
        or float(envelope) <= 0.0
    ):
        raise ValueError("vehicle envelope radius must be positive and finite")
    denominator = protocol.get("denominator")
    if not isinstance(denominator, dict) or denominator.get(
        "single_run_denominator"
    ) != 1:
        raise ValueError("single-run denominator must equal 1")
    if denominator.get("post_launch_exclusions") != "none":
        raise ValueError("a started single run cannot be excluded after launch")


def _source_to_map(
    point: tuple[float, float], source_start: tuple[float, float, float]
) -> tuple[float, float]:
    dx, dy = point[0] - source_start[0], point[1] - source_start[1]
    cosine, sine = math.cos(source_start[2]), math.sin(source_start[2])
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def _route_waypoints(row: dict[str, Any]) -> list[list[float]]:
    raw = row.get("waypoints")
    if not isinstance(raw, list) or len(raw) < 2:
        raise ValueError("selected obstacle route needs at least two waypoints")
    waypoints: list[list[float]] = []
    for item in raw:
        if not isinstance(item, list) or len(item) != 3:
            raise ValueError("obstacle waypoint must be [time_s, x_m, y_m]")
        try:
            waypoint = [float(value) for value in item]
        except (TypeError, ValueError) as exc:
            raise ValueError("obstacle waypoint is not numeric") from exc
        if not all(math.isfinite(value) for value in waypoint):
            raise ValueError("obstacle waypoint must be finite")
        waypoints.append(waypoint)
    if waypoints[0][0] != 0.0 or any(
        second[0] <= first[0] for first, second in zip(waypoints, waypoints[1:])
    ):
        raise ValueError("obstacle waypoint times must start at zero and increase")
    return waypoints


def first_centerline_crossing_time(route_map: list[list[float]]) -> float:
    """Return the first time a mission-local route reaches y=0."""

    for first, second in zip(route_map, route_map[1:]):
        first_y, second_y = float(first[2]), float(second[2])
        if first_y == 0.0:
            return float(first[0])
        if second_y == 0.0:
            return float(second[0])
        if first_y * second_y < 0.0:
            ratio = -first_y / (second_y - first_y)
            return float(first[0]) + ratio * (float(second[0]) - float(first[0]))
    raise ValueError("selected obstacle route never crosses the mission centerline")


def freeze_route_from_schedule(
    *,
    schedule: dict[str, Any],
    episode_manifest: Path,
    protocol: dict[str, Any],
    protocol_sha256: str,
    schedule_sha256: str,
    input_sha256: dict[str, str],
    declared_epoch_ns: int | None = None,
) -> dict[str, Any]:
    """Build the immutable pre-run route declaration from the exact schedule."""

    validate_protocol(protocol)
    if schedule.get("access") != "environment_driver_only_not_robot_control":
        raise ValueError("schedule lacks the evaluator-only access boundary")
    environment = schedule.get("acceptance_environment")
    if not isinstance(environment, dict):
        raise ValueError("schedule lacks acceptance_environment")
    seed = protocol["schedule"]["seed"]
    if environment.get("seed") != seed:
        raise ValueError("schedule seed differs from the protocol seed")
    crossing_ids = environment.get("mission_corridor_crossing_ids")
    expected_crossing_count = scheduled_crossing_count(protocol)
    if (
        not isinstance(crossing_ids, list)
        or len(crossing_ids) != expected_crossing_count
        or not all(isinstance(value, str) and value for value in crossing_ids)
    ):
        raise ValueError(
            "schedule mission-corridor crossing IDs differ from the protocol"
        )
    if environment.get("mission_corridor_crossing_count") != expected_crossing_count:
        raise ValueError("schedule mission-corridor crossing count differs from protocol")
    selected_id = crossing_ids[0]
    pedestrians = schedule.get("pedestrians")
    if not isinstance(pedestrians, list) or len(pedestrians) != 8:
        raise ValueError("single-run protocol requires eight scheduled walkers")
    matches = [
        row
        for row in pedestrians
        if isinstance(row, dict) and row.get("object_id") == selected_id
    ]
    if len(matches) != 1:
        raise ValueError("selected mission-corridor walker is not unique")
    selected = matches[0]
    route_source = _route_waypoints(selected)
    radius = selected.get("radius_m")
    if (
        isinstance(radius, bool)
        or not isinstance(radius, (int, float))
        or not math.isclose(
            float(radius),
            float(protocol["obstacle"]["radius_m"]),
            rel_tol=0.0,
            abs_tol=1.0e-9,
        )
    ):
        raise ValueError("selected walker radius differs from the protocol")
    mission = load_public_mission_contract(
        episode_manifest,
        nominal_leg_m=float(protocol["schedule"]["nominal_leg_m"]),
    )
    source_start = tuple(float(value) for value in mission["source_fixed_start_pose"])
    route_map = [
        [waypoint[0], *_source_to_map((waypoint[1], waypoint[2]), source_start)]
        for waypoint in route_source
    ]
    trigger_time_s = first_centerline_crossing_time(route_map)
    declared_ns = time.time_ns() if declared_epoch_ns is None else int(
        declared_epoch_ns
    )
    if declared_ns <= 0:
        raise ValueError("declared_epoch_ns must be positive")
    return {
        "schema_version": 1,
        "kind": ROUTE_MANIFEST_KIND,
        "mode": protocol_mode(protocol),
        "protocol_id": protocol["protocol_id"],
        "protocol_sha256": protocol_sha256,
        "declared_epoch_ns": declared_ns,
        "schedule_seed": seed,
        "schedule_sha256": schedule_sha256,
        "input_sha256": input_sha256,
        "source_fixed_start_pose": list(source_start),
        "selected_obstacle": {
            "object_id": selected_id,
            "radius_m": float(radius),
            "speed_mps": float(selected["speed_mps"]),
            "route_source_world": route_source,
            "route_map": route_map,
            "trigger_time_from_schedule_start_s": trigger_time_s,
            "trigger_type": protocol["obstacle"]["trigger"]["type"],
        },
        "claim_boundary": (
            "This file predeclares the deterministic obstacle route and trigger "
            "before the live trial. It is not evidence that the route executed, "
            "that avoidance succeeded, or that the >=95% rate was measured."
        ),
    }


def materialize_frozen_schedule(
    *,
    episode_manifest: Path,
    public_world: Path,
    base_schedule: Path,
    protocol: dict[str, Any],
) -> dict[str, Any]:
    from prepare_formal_dynamic_obstacle_schedule import materialize_schedule

    return materialize_schedule(
        episode_manifest=episode_manifest,
        public_world=public_world,
        base_schedule=base_schedule,
        seed=int(protocol["schedule"]["seed"]),
        nominal_leg_m=float(protocol["schedule"]["nominal_leg_m"]),
        crossing_count=scheduled_crossing_count(protocol),
        corridor_fraction_start=corridor_fraction_range(protocol)[0],
        corridor_fraction_end=corridor_fraction_range(protocol)[1],
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, required=True)
    parser.add_argument("--episode-manifest", type=Path, required=True)
    parser.add_argument("--public-world", type=Path, required=True)
    parser.add_argument("--base-schedule", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"fresh predeclared route output required: {args.output}")
    protocol = _read_object(args.protocol, "protocol")
    validate_protocol(protocol)
    for path, label in (
        (args.episode_manifest, "episode manifest"),
        (args.public_world, "public world"),
        (args.base_schedule, "base schedule"),
    ):
        if not path.is_file() or path.is_symlink():
            raise SystemExit(f"{label} must be a regular file: {path}")
    schedule = materialize_frozen_schedule(
        episode_manifest=args.episode_manifest,
        public_world=args.public_world,
        base_schedule=args.base_schedule,
        protocol=protocol,
    )
    schedule_bytes = _canonical_schedule_bytes(schedule)
    manifest = freeze_route_from_schedule(
        schedule=schedule,
        episode_manifest=args.episode_manifest,
        protocol=protocol,
        protocol_sha256=_sha256(args.protocol),
        schedule_sha256=hashlib.sha256(schedule_bytes).hexdigest(),
        input_sha256={
            "episode_manifest": _sha256(args.episode_manifest),
            "public_world": _sha256(args.public_world),
            "base_schedule": _sha256(args.base_schedule),
        },
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    pending = args.output.with_suffix(args.output.suffix + f".pending.{os.getpid()}")
    pending.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    pending.replace(args.output)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
