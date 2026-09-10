"""Decoder for the frozen atomic cleaning-motor telemetry snapshot."""

from __future__ import annotations

import math


MOTOR_NAMES = (
    "left_side_brush",
    "right_side_brush",
    "central_roller",
    "cleaning_lift",
    "recovery_pump",
)
RATED_CURRENT_A = (0.75, 0.75, 0.75, 0.50, 6.0)
FAULT_NAMES = {
    0: "none",
    1: "command_timeout",
    2: "stall",
    3: "overtemperature",
    4: "invalid_input",
}


def decode_cleaning_motor_snapshot(values) -> dict:
    """Return compact JSON-safe diagnostics from the 63-value v1 layout."""
    data = [float(value) for value in values]
    if len(data) != 63 or not all(math.isfinite(value) for value in data):
        raise ValueError("cleaning motor snapshot must contain 63 finite values")

    def integer(index: int, name: str, minimum: int = 0) -> int:
        result = int(data[index])
        if data[index] != float(result) or result < minimum:
            raise ValueError(f"{name} must be an integer >= {minimum}")
        return result

    def boolean(index: int, name: str) -> bool:
        result = integer(index, name)
        if result not in (0, 1):
            raise ValueError(f"{name} must be encoded as 0 or 1")
        return bool(result)

    schema = integer(0, "schema_version", 1)
    if schema != 1:
        raise ValueError(f"unsupported cleaning motor schema {schema}")
    telemetry_sequence = integer(1, "telemetry_sequence", 1)
    physics_sequence = integer(2, "physics_update_sequence")
    if telemetry_sequence > 2**53 or physics_sequence > 2**53:
        raise ValueError("cleaning motor sequence exceeds exact double range")
    physics_stale = boolean(3, "physics_update_stale")
    command_fresh = boolean(4, "command_fresh")
    aggregate_fault = boolean(5, "fault_active")
    if physics_sequence == 0 and not (
        physics_stale and aggregate_fault and not command_fresh
    ):
        raise ValueError("physics revision zero is only valid for fail-closed startup")
    if data[6] < 0.0 or data[7] < 0.0:
        raise ValueError("cleaning motor totals must be non-negative")

    motors = []
    for motor_index, motor_name in enumerate(MOTOR_NAMES):
        base = 8 + motor_index * 11
        protection_active = boolean(base + 8, f"{motor_name}.protection_active")
        fault_code = integer(base + 9, f"{motor_name}.fault")
        if fault_code not in FAULT_NAMES:
            raise ValueError(f"{motor_name}.fault enum {fault_code} is invalid")
        if protection_active != (fault_code != 0):
            raise ValueError(f"{motor_name} protection disagrees with fault")
        position_actuator = boolean(base + 10, f"{motor_name}.position_actuator")
        if position_actuator != (motor_index == 3):
            raise ValueError(f"{motor_name} position actuator violates v1 layout")
        if data[base + 3] < 0.0 or data[base + 5] < 0.0 or data[base + 6] < 0.0:
            raise ValueError(f"{motor_name} current, power and load must be non-negative")
        if not -50.0 <= data[base + 4] <= 200.0:
            raise ValueError(f"{motor_name} temperature is outside physical range")
        if physics_sequence > 0 and data[base + 7] <= 0.0:
            raise ValueError(f"{motor_name} speed limit must be positive")
        motors.append(
            {
                "name": motor_name,
                "command": data[base],
                "measured_position": data[base + 1],
                "measured_speed": data[base + 2],
                "current_a": data[base + 3],
                "temperature_c": data[base + 4],
                "electrical_power_w": data[base + 5],
                "estimated_output_load": data[base + 6],
                "speed_limit": data[base + 7],
                "current_above_rating": data[base + 3] > RATED_CURRENT_A[motor_index],
                "protection_active": protection_active,
                "fault": FAULT_NAMES[fault_code],
            }
        )
    if aggregate_fault != (
        physics_stale or any(motor["protection_active"] for motor in motors)
    ):
        raise ValueError("aggregate cleaning motor fault is inconsistent")
    if not math.isclose(
        data[6],
        sum(float(motor["current_a"]) for motor in motors),
        rel_tol=1e-12,
        abs_tol=1e-12,
    ):
        raise ValueError("total current disagrees with motor sum")
    if not math.isclose(
        data[7],
        sum(float(motor["electrical_power_w"]) for motor in motors),
        rel_tol=1e-12,
        abs_tol=1e-12,
    ):
        raise ValueError("total power disagrees with motor sum")
    return {
        "schema_version": schema,
        "telemetry_sequence": telemetry_sequence,
        "physics_update_sequence": physics_sequence,
        "physics_update_stale": physics_stale,
        "command_fresh": command_fresh,
        "fault_active": aggregate_fault,
        "total_current_a": data[6],
        "total_power_w": data[7],
        "motors": motors,
    }
