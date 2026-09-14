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
STATIC_VEHICLE_MAX_SPEED_MPS = 0.45
STATIC_SENSOR_INTERACTION_RANGE_M = 3.0
STATIC_COLLISION_MONITOR_TIME_BEFORE_COLLISION_S = 1.2
STATIC_EVALUATOR_STATUS_SAMPLE_PERIOD_S = 0.1
STATIC_MAXIMUM_STATUS_ALIGNMENT_S = 0.5
STATIC_PREDICTED_ENVELOPE_MARGIN_M = 0.05
STATIC_PROOF_TIME_STEP_S = 0.01


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


def _first_centerline_crossing(
    route_map: list[list[float]],
) -> tuple[float, float]:
    """Return the first mission-local centerline crossing time and x."""

    for first, second in zip(route_map, route_map[1:]):
        first_y, second_y = float(first[2]), float(second[2])
        if first_y == 0.0:
            return float(first[0]), float(first[1])
        if second_y == 0.0:
            return float(second[0]), float(second[1])
        if first_y * second_y < 0.0:
            ratio = -first_y / (second_y - first_y)
            return (
                float(first[0]) + ratio * (float(second[0]) - float(first[0])),
                float(first[1]) + ratio * (float(second[1]) - float(first[1])),
            )
    raise ValueError("selected obstacle route never crosses the mission centerline")


def first_centerline_crossing_time(route_map: list[list[float]]) -> float:
    """Return the first time a mission-local route reaches y=0."""

    return _first_centerline_crossing(route_map)[0]


def _interpolate_route_point(
    route_map: list[list[float]], elapsed_s: float
) -> tuple[float, float]:
    period_s = float(route_map[-1][0])
    if period_s <= 0.0:
        raise ValueError("route period must be positive")
    phase = elapsed_s % period_s
    for first, second in zip(route_map, route_map[1:]):
        if phase <= float(second[0]):
            duration = float(second[0]) - float(first[0])
            if duration <= 0.0:
                raise ValueError("route times must increase")
            ratio = (phase - float(first[0])) / duration
            return (
                float(first[1]) + ratio * (float(second[1]) - float(first[1])),
                float(first[2]) + ratio * (float(second[2]) - float(first[2])),
            )
    raise AssertionError("validated route interpolation fell through")


def static_interaction_materialization(
    *,
    route_map: list[list[float]],
    walker_radius_m: float,
    walker_speed_mps: float,
    vehicle_envelope_radius_m: float,
    minimum_surface_clearance_m: float,
    trigger_window_s: float,
) -> dict[str, Any]:
    """Prove an evaluator-sampleable interaction against the frozen route.

    The proof uses the maximum commanded speed, the configured approach
    lookahead, and the evaluator status period. It is intentionally
    conservative about route geometry: the selected interaction sample may be
    earlier than the centerline crossing, but it must already have positive
    surface clearance and a predicted envelope encounter inside the monitor's
    approach horizon.
    """

    if walker_radius_m <= 0.0 or vehicle_envelope_radius_m <= 0.0:
        raise ValueError("static interaction radii must be positive")
    if walker_speed_mps <= 0.0:
        raise ValueError("static interaction walker speed must be positive")
    if minimum_surface_clearance_m <= 0.0 or trigger_window_s <= 0.0:
        raise ValueError("static interaction thresholds must be positive")
    period_s = float(route_map[-1][0])
    if period_s <= 0.0:
        raise ValueError("static interaction route period must be positive")

    trigger_time_s, crossing_x_m = _first_centerline_crossing(route_map)
    collision_distance_m = walker_radius_m + vehicle_envelope_radius_m
    required_predicted_distance_m = (
        collision_distance_m - STATIC_PREDICTED_ENVELOPE_MARGIN_M
    )
    start_s = max(0.0, trigger_time_s - trigger_window_s)
    end_s = min(period_s, trigger_time_s + trigger_window_s)
    sample_count = max(
        1,
        int(math.floor((end_s - start_s) / STATIC_PROOF_TIME_STEP_S + 1.0e-9)),
    )
    selected: dict[str, Any] | None = None
    for index in range(sample_count + 1):
        candidate_s = min(end_s, start_s + index * STATIC_PROOF_TIME_STEP_S)
        walker_x_m, walker_y_m = _interpolate_route_point(route_map, candidate_s)
        vehicle_x_m = STATIC_VEHICLE_MAX_SPEED_MPS * candidate_s
        center_distance_m = math.hypot(
            walker_x_m - vehicle_x_m, walker_y_m
        )
        surface_gap_m = (
            center_distance_m - walker_radius_m - vehicle_envelope_radius_m
        )
        if center_distance_m > STATIC_SENSOR_INTERACTION_RANGE_M:
            continue
        if surface_gap_m < minimum_surface_clearance_m:
            continue
        horizon_sample_count = int(
            math.ceil(
                STATIC_COLLISION_MONITOR_TIME_BEFORE_COLLISION_S
                / STATIC_PROOF_TIME_STEP_S
            )
        )
        predicted_minimum_center_distance_m = min(
            math.hypot(
                _interpolate_route_point(
                    route_map,
                    candidate_s + step * STATIC_PROOF_TIME_STEP_S,
                )[0]
                - STATIC_VEHICLE_MAX_SPEED_MPS
                * (candidate_s + step * STATIC_PROOF_TIME_STEP_S),
                _interpolate_route_point(
                    route_map,
                    candidate_s + step * STATIC_PROOF_TIME_STEP_S,
                )[1],
            )
            for step in range(horizon_sample_count + 1)
        )
        if (
            predicted_minimum_center_distance_m
            > required_predicted_distance_m
        ):
            continue
        phase_s = candidate_s % STATIC_EVALUATOR_STATUS_SAMPLE_PERIOD_S
        status_alignment_s = min(
            phase_s,
            STATIC_EVALUATOR_STATUS_SAMPLE_PERIOD_S - phase_s,
        )
        if status_alignment_s > STATIC_MAXIMUM_STATUS_ALIGNMENT_S:
            continue
        selected = {
            "interaction_sample_schedule_elapsed_s": candidate_s,
            "interaction_sample_trigger_delta_s": abs(
                candidate_s - trigger_time_s
            ),
            "interaction_sample_vehicle_mission_x_m": vehicle_x_m,
            "interaction_sample_walker_x_m": walker_x_m,
            "interaction_sample_walker_y_m": walker_y_m,
            "interaction_sample_center_distance_m": center_distance_m,
            "interaction_sample_surface_gap_m": surface_gap_m,
            "interaction_sample_sensor_range_limit_m": (
                STATIC_SENSOR_INTERACTION_RANGE_M
            ),
            "interaction_sample_status_alignment_s": status_alignment_s,
            "predicted_minimum_center_distance_within_horizon_m": (
                predicted_minimum_center_distance_m
            ),
            "predicted_envelope_intervention_distance_m": (
                required_predicted_distance_m
            ),
        }
        break

    passed = selected is not None
    proof = {
        "schema_version": 1,
        "status": (
            "STATIC_OBSTACLE_INTERACTION_SAMPLEABLE"
            if passed
            else "STATIC_OBSTACLE_INTERACTION_NOT_SAMPLEABLE"
        ),
        "passed": passed,
        "route_centerline_crossing_time_s": trigger_time_s,
        "route_centerline_crossing_x_m": crossing_x_m,
        "trigger_window_s": trigger_window_s,
        "vehicle_max_speed_mps": STATIC_VEHICLE_MAX_SPEED_MPS,
        "vehicle_arrival_at_crossing_s": (
            crossing_x_m / STATIC_VEHICLE_MAX_SPEED_MPS
        ),
        "collision_monitor_time_before_collision_s": (
            STATIC_COLLISION_MONITOR_TIME_BEFORE_COLLISION_S
        ),
        "evaluator_status_sample_period_s": (
            STATIC_EVALUATOR_STATUS_SAMPLE_PERIOD_S
        ),
        "maximum_evaluator_status_alignment_s": (
            STATIC_MAXIMUM_STATUS_ALIGNMENT_S
        ),
        "minimum_surface_clearance_m": minimum_surface_clearance_m,
        "selected_interaction_sample": selected,
        "claim_boundary": (
            "Static geometry and clock-sampling proof only. Live sensor "
            "detection, collision-monitor behavior, path execution, and task "
            "completion remain runtime measurements."
        ),
    }
    if not passed:
        proof["failure_reason"] = (
            "no sample inside the trigger window simultaneously had positive "
            "surface clearance, sensor-range observability, a collision-monitor "
            "approach horizon, and evaluator status alignment"
        )
    return proof


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
    static_interaction = static_interaction_materialization(
        route_map=route_map,
        walker_radius_m=float(radius),
        walker_speed_mps=float(selected["speed_mps"]),
        vehicle_envelope_radius_m=float(
            protocol["obstacle"]["minimum_distance"][
                "vehicle_envelope_radius_m"
            ]
        ),
        minimum_surface_clearance_m=float(
            protocol["obstacle"]["minimum_distance"]["threshold_m"]
        ),
        trigger_window_s=float(protocol["obstacle"]["trigger"]["window_s"]),
    )
    if protocol_mode(protocol) == FUNCTIONAL_SMOKE_MODE and not static_interaction[
        "passed"
    ]:
        raise ValueError(
            "selected obstacle route has no evaluator-sampleable static interaction"
        )
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
        "static_interaction_materialization": static_interaction,
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
