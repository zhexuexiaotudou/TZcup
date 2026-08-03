"""Estimated-heading control helpers for skid-steer in-place rotation."""

import math


def normalized_yaw_error(target_yaw: float, current_yaw: float) -> float:
    return math.atan2(
        math.sin(float(target_yaw) - float(current_yaw)),
        math.cos(float(target_yaw) - float(current_yaw)),
    )


def bounded_angular_command(
    error_rad: float,
    *,
    maximum_rad_s: float = 0.60,
    minimum_rad_s: float = 0.12,
    gain: float = 1.5,
    tolerance_rad: float = 0.06,
) -> float:
    error = float(error_rad)
    if abs(error) <= tolerance_rad:
        return 0.0
    magnitude = min(maximum_rad_s, max(minimum_rad_s, gain * abs(error)))
    return math.copysign(magnitude, error)
