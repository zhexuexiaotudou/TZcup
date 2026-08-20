"""ROS-independent transform-continuity helpers."""

from __future__ import annotations

import math


def wrapped_angle_delta(first: float, second: float) -> float:
    return math.atan2(math.sin(second - first), math.cos(second - first))


def transform_jump(
    previous: tuple[float, float, float],
    current: tuple[float, float, float],
    *,
    translation_threshold_m: float,
    yaw_threshold_rad: float,
) -> dict[str, float | bool]:
    translation = math.hypot(current[0] - previous[0], current[1] - previous[1])
    yaw = abs(wrapped_angle_delta(previous[2], current[2]))
    return {
        "translation_m": translation,
        "yaw_rad": yaw,
        "exceeds_diagnostic_threshold": (
            translation > translation_threshold_m or yaw > yaw_threshold_rad
        ),
    }
