#!/usr/bin/env python3
"""Pure cleaning-brush telemetry metrics for the formal water runtime gate."""

from __future__ import annotations

import math
from typing import Any


def _number(value: object) -> float:
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def _maximum_contiguous_duration(
    rows: list[dict[str, Any]], predicate
) -> float:
    maximum = 0.0
    start: float | None = None
    previous: float | None = None
    for row in rows:
        stamp = row["stamp"]
        if predicate(row):
            if start is None:
                start = stamp
            previous = stamp
            maximum = max(maximum, previous - start)
        else:
            start = None
            previous = None
    return maximum


def _cleaning_motor_duty_metrics(
    samples: list[dict[str, object]],
    *,
    motor_commands_rad_s: dict[str, float],
    rated_current_a: float = 0.75,
    startup_transient_limit_s: float = 1.0,
    minimum_tracking_ratio: float = 0.80,
) -> dict[str, dict[str, float | int | bool]]:
    """Measure same-update signed tracking/current/fault behavior.

    Only samples whose signed velocity command magnitude matches the formal
    command for that motor enter tracking/current calculations.  The first
    second is an explicit motor-start transient; every later sample is steady
    state.  Fault freedom is checked over every captured scenario sample.
    """

    result: dict[str, dict[str, float | int | bool]] = {}
    for name, expected_command_rad_s in motor_commands_rad_s.items():
        rows: list[dict[str, Any]] = []
        for sample in samples:
            stamp = _number(sample.get("sim_time_s"))
            motors = sample.get("motors", [])
            if not isinstance(motors, list):
                motors = []
            motor = next(
                (
                    candidate
                    for candidate in motors
                    if isinstance(candidate, dict) and candidate.get("name") == name
                ),
                None,
            )
            if motor is None:
                continue
            command = _number(motor.get("command"))
            measured_speed = _number(motor.get("measured_speed"))
            current = _number(motor.get("current_a"))
            temperature = _number(motor.get("temperature_c"))
            finite = all(
                math.isfinite(value)
                for value in (stamp, command, measured_speed, current, temperature)
            )
            rows.append(
                {
                    "stamp": stamp,
                    "command": command,
                    "measured_speed": measured_speed,
                    "current": current,
                    "temperature": temperature,
                    "finite": finite,
                    "fault_free": (
                        finite
                        and sample.get("physics_update_stale") is False
                        and sample.get("fault_active") is False
                        and motor.get("protection_active") is False
                        and motor.get("fault") == "none"
                    ),
                }
            )
        rows.sort(key=lambda row: row["stamp"] if math.isfinite(row["stamp"]) else math.inf)
        commanded = [
            row
            for row in rows
            if row["finite"]
            and abs(abs(row["command"]) - expected_command_rad_s) <= 0.05
        ]
        steady_start = (
            commanded[0]["stamp"] + startup_transient_limit_s
            if commanded
            else math.inf
        )
        steady = [row for row in commanded if row["stamp"] >= steady_start]
        for row in commanded:
            row["tracking_ratio"] = abs(row["measured_speed"]) / max(
                abs(row["command"]), 1e-12
            )
            row["direction_matches"] = row["command"] * row["measured_speed"] > 0.0

        currents = [row["current"] for row in commanded]
        steady_currents = [row["current"] for row in steady]
        ratios = [row["tracking_ratio"] for row in steady]
        result[name] = {
            "sample_count": len(rows),
            "commanded_sample_count": len(commanded),
            "steady_sample_count": len(steady),
            "rated_current_a": rated_current_a,
            "expected_command_rad_s": expected_command_rad_s,
            "startup_transient_limit_s": startup_transient_limit_s,
            "minimum_tracking_ratio": minimum_tracking_ratio,
            "all_fields_finite": bool(rows) and all(row["finite"] for row in rows),
            "fault_free_all_samples": bool(rows) and all(row["fault_free"] for row in rows),
            "direction_matches_all_steady_samples": bool(steady) and all(
                row["direction_matches"] for row in steady
            ),
            "peak_current_a": max(currents, default=math.inf),
            "p95_current_a": _percentile(currents, 0.95),
            "steady_peak_current_a": max(steady_currents, default=math.inf),
            "maximum_contiguous_over_rated_s": _maximum_contiguous_duration(
                commanded, lambda row: row["current"] > rated_current_a + 1e-9
            ),
            "p05_tracking_ratio": _percentile(ratios, 0.05),
            "p50_tracking_ratio": _percentile(ratios, 0.50),
            "maximum_contiguous_low_speed_s": _maximum_contiguous_duration(
                steady,
                lambda row: row["tracking_ratio"] < minimum_tracking_ratio - 1e-9,
            ),
            "peak_temperature_c": max(
                (row["temperature"] for row in rows), default=math.inf
            ),
        }
    return result


def side_brush_duty_metrics(
    samples: list[dict[str, object]],
    *,
    rated_current_a: float = 0.75,
    expected_command_rad_s: float = 8.0,
    startup_transient_limit_s: float = 1.0,
    minimum_tracking_ratio: float = 0.80,
) -> dict[str, dict[str, float | int | bool]]:
    """Return duty metrics for both formal side-brush motors."""

    return _cleaning_motor_duty_metrics(
        samples,
        motor_commands_rad_s={
            "left_side_brush": expected_command_rad_s,
            "right_side_brush": expected_command_rad_s,
        },
        rated_current_a=rated_current_a,
        startup_transient_limit_s=startup_transient_limit_s,
        minimum_tracking_ratio=minimum_tracking_ratio,
    )


def central_roller_duty_metrics(
    samples: list[dict[str, object]],
    *,
    rated_current_a: float = 0.75,
    expected_command_rad_s: float = 12.0,
    startup_transient_limit_s: float = 1.0,
    minimum_tracking_ratio: float = 0.80,
) -> dict[str, float | int | bool]:
    """Return duty metrics for the independently driven central roller."""

    return _cleaning_motor_duty_metrics(
        samples,
        motor_commands_rad_s={"central_roller": expected_command_rad_s},
        rated_current_a=rated_current_a,
        startup_transient_limit_s=startup_transient_limit_s,
        minimum_tracking_ratio=minimum_tracking_ratio,
    )["central_roller"]
