#!/usr/bin/env python3
"""Fixed-layout decoder for atomic cleaning-actuator telemetry snapshots."""

from __future__ import annotations

import math
from collections.abc import Sequence


SCHEMA_VERSION = 1
HEADER_COUNT = 8
FIELDS_PER_MOTOR = 11
MOTOR_NAMES = (
    "left_side_brush",
    "right_side_brush",
    "central_roller",
    "cleaning_lift",
    "recovery_pump",
)
VALUE_COUNT = HEADER_COUNT + len(MOTOR_NAMES) * FIELDS_PER_MOTOR
RATED_CURRENT_A = (0.75, 0.75, 0.75, 0.50, 6.0)
FAULT_NAMES = {
    0: "none",
    1: "command_timeout",
    2: "stall",
    3: "overtemperature",
    4: "invalid_input",
}


class CleaningMotorTelemetryError(ValueError):
    """Raised when a typed snapshot violates its frozen schema."""


def update_physics_revision_watchdog(
    *,
    sequence: int,
    physics_stale: bool,
    last_sequence: int | None,
    last_advance_s: float | None,
    now_s: float,
    timeout_s: float = 0.75,
) -> tuple[int | None, float | None]:
    """Start physics liveness timing only after the first healthy revision."""
    if last_sequence is not None and sequence < last_sequence:
        raise ValueError("cleaning motor physics revision moved backwards")
    if last_sequence is None:
        if sequence == 0 or physics_stale:
            return None, None
        return sequence, now_s
    if sequence > last_sequence:
        return sequence, now_s
    if last_advance_s is None or now_s - last_advance_s >= timeout_s:
        raise ValueError("cleaning motor physics revision stalled for >=0.75 s")
    return last_sequence, last_advance_s


def _finite(values: Sequence[float]) -> list[float]:
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise CleaningMotorTelemetryError("telemetry snapshot contains non-finite values")
    return result


def _integer(value: float, name: str, *, minimum: int = 0) -> int:
    integer = int(value)
    if value != float(integer) or integer < minimum:
        raise CleaningMotorTelemetryError(f"{name} must be an integer >= {minimum}")
    return integer


def _boolean(value: float, name: str) -> bool:
    integer = _integer(value, name)
    if integer not in (0, 1):
        raise CleaningMotorTelemetryError(f"{name} must be encoded as 0 or 1")
    return bool(integer)


def decode_cleaning_motor_telemetry(values: Sequence[float]) -> dict[str, object]:
    data = _finite(values)
    if len(data) != VALUE_COUNT:
        raise CleaningMotorTelemetryError(
            f"telemetry snapshot length {len(data)} != frozen {VALUE_COUNT}"
        )
    schema = _integer(data[0], "schema_version", minimum=1)
    if schema != SCHEMA_VERSION:
        raise CleaningMotorTelemetryError(
            f"unsupported telemetry schema {schema}, expected {SCHEMA_VERSION}"
        )
    telemetry_sequence = _integer(data[1], "telemetry_sequence", minimum=1)
    physics_sequence = _integer(data[2], "physics_update_sequence")
    if telemetry_sequence > 2**53 or physics_sequence > 2**53:
        raise CleaningMotorTelemetryError("sequence exceeds exact double range")
    physics_stale = _boolean(data[3], "physics_update_stale")
    command_fresh = _boolean(data[4], "command_fresh")
    aggregate_fault = _boolean(data[5], "fault_active")
    if physics_sequence == 0 and not (
        physics_stale and aggregate_fault and not command_fresh
    ):
        raise CleaningMotorTelemetryError(
            "physics revision zero is only valid for fail-closed startup"
        )
    if data[6] < 0.0 or data[7] < 0.0:
        raise CleaningMotorTelemetryError("total current and power must be non-negative")
    motors: list[dict[str, object]] = []
    for index, name in enumerate(MOTOR_NAMES):
        base = HEADER_COUNT + index * FIELDS_PER_MOTOR
        fault_code = _integer(data[base + 9], f"{name}.fault")
        if fault_code not in FAULT_NAMES:
            raise CleaningMotorTelemetryError(f"{name}.fault enum {fault_code} is invalid")
        protection_active = _boolean(data[base + 8], f"{name}.protection_active")
        if protection_active != (fault_code != 0):
            raise CleaningMotorTelemetryError(
                f"{name} protection flag disagrees with fault enum"
            )
        position_actuator = _boolean(
            data[base + 10], f"{name}.position_actuator"
        )
        if position_actuator != (index == 3):
            raise CleaningMotorTelemetryError(
                f"{name} position actuator encoding violates frozen layout"
            )
        if data[base + 3] < 0.0 or data[base + 5] < 0.0 or data[base + 6] < 0.0:
            raise CleaningMotorTelemetryError(
                f"{name} current, power and estimated load must be non-negative"
            )
        if not -50.0 <= data[base + 4] <= 200.0:
            raise CleaningMotorTelemetryError(
                f"{name} temperature is outside the physical telemetry range"
            )
        if physics_sequence > 0 and data[base + 7] <= 0.0:
            raise CleaningMotorTelemetryError(
                f"{name} speed_limit must be positive after physics startup"
            )
        motors.append(
            {
                "name": name,
                "control_mode": (
                    "position"
                    if position_actuator
                    else "velocity"
                ),
                "command": data[base + 0],
                "measured_position": data[base + 1],
                "measured_speed": data[base + 2],
                "current_a": data[base + 3],
                "temperature_c": data[base + 4],
                "electrical_power_w": data[base + 5],
                "estimated_output_load": data[base + 6],
                "speed_limit": data[base + 7],
                "current_above_rating": data[base + 3] > RATED_CURRENT_A[index],
                "protection_active": protection_active,
                "fault": FAULT_NAMES[fault_code],
            }
        )
    expected_fault = physics_stale or any(
        bool(motor["protection_active"]) for motor in motors
    )
    if aggregate_fault != expected_fault:
        raise CleaningMotorTelemetryError(
            "aggregate fault disagrees with stale/protection state"
        )
    total_current = data[6]
    total_power = data[7]
    if not math.isclose(
        total_current,
        sum(float(motor["current_a"]) for motor in motors),
        rel_tol=1e-12,
        abs_tol=1e-12,
    ):
        raise CleaningMotorTelemetryError("total current disagrees with motor sum")
    if not math.isclose(
        total_power,
        sum(float(motor["electrical_power_w"]) for motor in motors),
        rel_tol=1e-12,
        abs_tol=1e-12,
    ):
        raise CleaningMotorTelemetryError("total power disagrees with motor sum")
    return {
        "schema_version": schema,
        "telemetry_sequence": telemetry_sequence,
        "physics_update_sequence": physics_sequence,
        "physics_update_stale": physics_stale,
        "command_fresh": command_fresh,
        "fault_active": aggregate_fault,
        "total_current_a": total_current,
        "total_power_w": total_power,
        "motors": motors,
    }
