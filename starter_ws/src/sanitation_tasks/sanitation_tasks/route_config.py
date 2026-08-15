"""ROS-independent route configuration validation."""

from __future__ import annotations

import json
import math
from pathlib import Path


def load_waypoints(waypoints_json: str, waypoints_file: str = ""):
    payload = (
        json.loads(Path(waypoints_file).read_text(encoding="utf-8"))
        if waypoints_file
        else json.loads(waypoints_json)
    )
    if not isinstance(payload, list) or not payload:
        raise ValueError("navigation waypoints must be a non-empty list")
    waypoints = []
    for index, row in enumerate(payload):
        if not isinstance(row, list) or len(row) != 3:
            raise ValueError(f"waypoint {index} must be [x, y, yaw]")
        values = tuple(float(value) for value in row)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"waypoint {index} contains a non-finite value")
        waypoints.append(values)
    return waypoints
