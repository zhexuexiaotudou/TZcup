"""Estimated-pose control helpers for short skid-steer translations."""

import math

from .skid_steer_rotation import normalized_yaw_error


def translation_errors(current, goal, travel_yaw):
    dx = float(goal[0]) - float(current[0])
    dy = float(goal[1]) - float(current[1])
    cosine = math.cos(float(travel_yaw))
    sine = math.sin(float(travel_yaw))
    along = dx * cosine + dy * sine
    cross = -dx * sine + dy * cosine
    yaw = normalized_yaw_error(float(travel_yaw), float(current[2]))
    return along, cross, yaw, math.hypot(dx, dy)


def bounded_translation_command(
    along_m,
    cross_m,
    yaw_error_rad,
    *,
    maximum_linear_m_s=0.55,
    maximum_angular_rad_s=0.60,
):
    linear = max(-0.30, min(maximum_linear_m_s, 1.2 * float(along_m)))
    angular = 1.8 * float(yaw_error_rad) + 1.4 * float(cross_m)
    angular = max(-maximum_angular_rad_s, min(maximum_angular_rad_s, angular))
    if abs(yaw_error_rad) > 0.35:
        linear = 0.0
    return linear, angular
