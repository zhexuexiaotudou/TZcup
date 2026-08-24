"""ROS-independent pedestrian schedule interpolation."""

from __future__ import annotations

import math
import json
from pathlib import Path
from typing import Sequence

from .generator import GenerationError


def load_schedule(path: str | Path) -> dict:
    try:
        schedule = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise GenerationError(f"cannot load pedestrian schedule: {exc}") from exc
    if schedule.get("schema_version") != 1:
        raise GenerationError("pedestrian schedule schema_version must equal 1")
    if schedule.get("access") != "environment_driver_only_not_robot_control":
        raise GenerationError("pedestrian schedule access boundary is missing")
    if not isinstance(schedule.get("world_name"), str) or not schedule["world_name"]:
        raise GenerationError("pedestrian schedule world_name is missing")
    pedestrians = schedule.get("pedestrians")
    if not isinstance(pedestrians, list):
        raise GenerationError("pedestrian schedule pedestrians must be a list")
    names: set[str] = set()
    for pedestrian in pedestrians:
        name = pedestrian.get("object_id")
        if not isinstance(name, str) or not name or name in names:
            raise GenerationError("pedestrian object_id must be unique and non-empty")
        names.add(name)
        interpolate_loop(pedestrian.get("waypoints", ()), 0.0)
    return schedule


def interpolate_loop(waypoints: Sequence[Sequence[float]], elapsed_s: float) -> tuple[float, float, float]:
    if len(waypoints) < 2:
        raise GenerationError("pedestrian schedule needs at least two waypoints")
    if elapsed_s < 0 or not math.isfinite(elapsed_s):
        raise GenerationError("elapsed_s must be finite and non-negative")
    parsed = [tuple(float(v) for v in row) for row in waypoints]
    if any(len(row) != 3 for row in parsed):
        raise GenerationError("waypoints must contain [time_s, x_m, y_m]")
    if parsed[0][0] != 0.0 or any(b[0] <= a[0] for a, b in zip(parsed, parsed[1:])):
        raise GenerationError("waypoint times must start at zero and strictly increase")
    duration = parsed[-1][0]
    time_s = elapsed_s % duration
    for first, second in zip(parsed, parsed[1:]):
        if time_s <= second[0]:
            ratio = (time_s - first[0]) / (second[0] - first[0])
            x = first[1] + ratio * (second[1] - first[1])
            y = first[2] + ratio * (second[2] - first[2])
            yaw = math.atan2(second[2] - first[2], second[1] - first[1])
            return x, y, yaw
    raise AssertionError("validated schedule interpolation fell through")
