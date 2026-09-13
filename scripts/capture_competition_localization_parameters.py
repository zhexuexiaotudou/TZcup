#!/usr/bin/env python3
"""Capture and verify effective localization parameters through ROS services."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess
from typing import Callable


SCHEMA_VERSION = 1
EXPECTED_PARAMETERS = {
    "/local_ekf": {
        "publish_tf": True,
        "world_frame": "odom",
        "odom_frame": "odom",
        "base_link_frame": "base_footprint",
    },
    "/global_ekf": {
        "frequency": 50.0,
        "publish_tf": True,
        "map_frame": "map",
        "odom_frame": "odom",
        "base_link_frame": "base_footprint",
        "world_frame": "map",
        "odom1": "/odometry/gps",
        "pose0": "/amcl_pose",
        "transform_timeout": 0.0,
    },
    "/amcl": {
        "tf_broadcast": False,
        "global_frame_id": "map",
        "odom_frame_id": "odom",
    },
    "/navsat_transform": {
        "broadcast_utm_transform": False,
        "broadcast_cartesian_transform": False,
        "use_odometry_yaw": False,
        "zero_altitude": True,
    },
}


def parse_parameter_value(raw: str, expected: object) -> object:
    """Parse the scalar printed by ros2 param get and coerce to expected type."""
    matches = re.findall(r"value is:\s*(.*?)\s*$", raw, flags=re.MULTILINE)
    token = (matches[-1] if matches else raw.strip().splitlines()[-1]).strip()
    token = token.strip("\"'")
    if isinstance(expected, bool):
        if token.lower() not in {"true", "false"}:
            raise ValueError(f"expected boolean parameter response, got {token!r}")
        return token.lower() == "true"
    if isinstance(expected, float):
        return float(token)
    if isinstance(expected, int):
        return int(token)
    return token


def capture(
    runner: Callable[..., subprocess.CompletedProcess[str]],
    *,
    timeout_sec: float,
) -> dict[str, object]:
    """Query every required parameter and retain raw evidence for each response."""
    nodes: dict[str, object] = {}
    all_expected = True
    for node, parameters in EXPECTED_PARAMETERS.items():
        values: dict[str, object] = {}
        for name, expected in parameters.items():
            error = ""
            try:
                completed = runner(
                    ["ros2", "param", "get", node, name],
                    capture_output=True,
                    text=True,
                    timeout=timeout_sec,
                    check=False,
                )
                raw = completed.stdout.strip()
                actual = (
                    parse_parameter_value(raw, expected)
                    if completed.returncode == 0
                    else None
                )
                error = completed.stderr.strip() if completed.returncode else ""
            except Exception as exc:  # pragma: no cover - subprocess failure path
                raw = ""
                actual = None
                error = f"{type(exc).__name__}: {exc}"
            matched = actual == expected
            all_expected &= matched
            values[name] = {
                "expected": expected,
                "actual": actual,
                "matched": matched,
                "raw": raw,
                "error": error,
                "command": ["ros2", "param", "get", node, name],
            }
        nodes[node] = values
    return {
        "schema_version": SCHEMA_VERSION,
        "all_expected": all_expected,
        "nodes": nodes,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"fresh output required: {args.output}")
    if not args.timeout_sec > 0:
        raise SystemExit("--timeout-sec must be positive")

    report = capture(subprocess.run, timeout_sec=args.timeout_sec)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0 if report["all_expected"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
