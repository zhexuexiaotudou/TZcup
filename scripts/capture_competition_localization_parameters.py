#!/usr/bin/env python3
"""Capture and verify effective localization parameters through ROS services."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess
import time
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
STABILIZER_EXPECTED_PARAMETERS = {
    "tau_sec": 1.5,
    "max_filter_dt_sec": 0.1,
    "max_gap_sec": 0.5,
    "input_tf_topic": "/localization/raw_map_odom",
}


def expected_parameters_for_owner(map_odom_owner: str) -> dict[str, dict[str, object]]:
    if map_odom_owner not in {"/global_ekf", "/map_odom_stabilizer"}:
        raise ValueError(f"unsupported map->odom owner: {map_odom_owner}")
    parameters = {
        node: dict(values) for node, values in EXPECTED_PARAMETERS.items()
    }
    if map_odom_owner == "/map_odom_stabilizer":
        parameters["/map_odom_stabilizer"] = dict(
            STABILIZER_EXPECTED_PARAMETERS
        )
    return parameters


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
    expected_parameters: dict[str, dict[str, object]] | None = None,
    timeout_sec: float,
    attempts: int = 3,
    spin_time_sec: float = 2.0,
    discovery_timeout_sec: int = 5,
    retry_delay_sec: float = 0.5,
    sleeper: Callable[[float], None] = time.sleep,
) -> dict[str, object]:
    """Query every required parameter and retain raw evidence for each response."""
    if attempts < 1:
        raise ValueError("attempts must be at least one")
    parameters_by_node = (
        EXPECTED_PARAMETERS if expected_parameters is None else expected_parameters
    )
    nodes: dict[str, object] = {}
    all_expected = True
    for node, parameters in parameters_by_node.items():
        values: dict[str, object] = {}
        for name, expected in parameters.items():
            command = [
                "ros2",
                "param",
                "get",
                "--spin-time",
                str(spin_time_sec),
                "--timeout",
                str(discovery_timeout_sec),
                node,
                name,
            ]
            raw = ""
            actual = None
            error = ""
            attempt_rows: list[dict[str, object]] = []
            for attempt in range(1, attempts + 1):
                attempt_raw = ""
                attempt_actual = None
                attempt_error = ""
                returncode = None
                try:
                    completed = runner(
                        command,
                        capture_output=True,
                        text=True,
                        timeout=timeout_sec,
                        check=False,
                    )
                    returncode = completed.returncode
                    attempt_raw = completed.stdout.strip()
                    attempt_actual = (
                        parse_parameter_value(attempt_raw, expected)
                        if completed.returncode == 0
                        else None
                    )
                    attempt_error = (
                        completed.stderr.strip() if completed.returncode else ""
                    )
                except Exception as exc:  # pragma: no cover - subprocess failure path
                    attempt_error = f"{type(exc).__name__}: {exc}"
                attempt_rows.append(
                    {
                        "attempt": attempt,
                        "returncode": returncode,
                        "raw": attempt_raw,
                        "actual": attempt_actual,
                        "error": attempt_error,
                    }
                )
                raw = attempt_raw
                actual = attempt_actual
                error = attempt_error
                if returncode == 0 and actual == expected:
                    break
                if attempt < attempts:
                    sleeper(retry_delay_sec)
            matched = actual == expected
            all_expected &= matched
            values[name] = {
                "expected": expected,
                "actual": actual,
                "matched": matched,
                "raw": raw,
                "error": error,
                "command": command,
                "attempt_count": len(attempt_rows),
                "attempts": attempt_rows,
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
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    parser.add_argument("--attempts", type=int, default=3)
    parser.add_argument("--spin-time-sec", type=float, default=2.0)
    parser.add_argument("--discovery-timeout-sec", type=int, default=5)
    parser.add_argument("--retry-delay-sec", type=float, default=0.5)
    parser.add_argument(
        "--map-odom-owner",
        choices=("/global_ekf", "/map_odom_stabilizer"),
        default="/global_ekf",
    )
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"fresh output required: {args.output}")
    if not args.timeout_sec > 0:
        raise SystemExit("--timeout-sec must be positive")
    if args.attempts < 1:
        raise SystemExit("--attempts must be at least one")
    if args.spin_time_sec < 0 or args.discovery_timeout_sec <= 0:
        raise SystemExit("discovery timing arguments are invalid")
    if args.retry_delay_sec < 0:
        raise SystemExit("--retry-delay-sec must be nonnegative")

    report = capture(
        subprocess.run,
        expected_parameters=expected_parameters_for_owner(args.map_odom_owner),
        timeout_sec=args.timeout_sec,
        attempts=args.attempts,
        spin_time_sec=args.spin_time_sec,
        discovery_timeout_sec=args.discovery_timeout_sec,
        retry_delay_sec=args.retry_delay_sec,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0 if report["all_expected"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
