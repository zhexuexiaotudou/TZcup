"""Pure helpers for semantic cleaning and Ackermann telemetry."""

import math


SCHEMA = "tzcup.gazebo_cleaning_telemetry.v2"


def resolve_cleanable_polygon(config):
    """Use an explicit scored polygon before legacy headland inference."""
    explicit = config.get("cleanable_outer_polygon")
    if explicit:
        return [(float(point[0]), float(point[1])) for point in explicit]
    outer = [(float(point[0]), float(point[1])) for point in config.get("outer_polygon", [])]
    if len(outer) != 4:
        return outer
    inset = float(config.get("headland", {}).get("width_m", 0.0))
    min_x = min(point[0] for point in outer) + inset
    max_x = max(point[0] for point in outer) - inset
    min_y = min(point[1] for point in outer) + inset
    max_y = max(point[1] for point in outer) - inset
    if min_x >= max_x or min_y >= max_y:
        return outer
    return [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)]


def classify_motion_state(state: str, brush_enabled: bool) -> str:
    state = str(state).upper()
    if state.startswith("REPAIR_") or state == "REPAIR_SWATH":
        return "repair"
    if brush_enabled and state == "EXECUTING_SWATH":
        return "cleaning"
    return "transit"


def virtual_steering_angle(left_rad, right_rad):
    """Recover the bicycle-model steering angle from the two road-wheel angles."""
    left = float(left_rad)
    right = float(right_rad)
    if abs(left) < 1e-6 and abs(right) < 1e-6:
        return 0.0
    if abs(left) < 1e-6 or abs(right) < 1e-6:
        return 0.5 * (left + right)
    cot_sum = 1.0 / math.tan(left) + 1.0 / math.tan(right)
    if abs(cot_sum) < 1e-9:
        return 0.5 * (left + right)
    return math.atan(2.0 / cot_sum)


def measured_motion(linear_x_mps, yaw_rate_rad_s, stop_threshold_mps=0.02):
    """Classify measured gear and curvature without using simulation truth."""
    linear = float(linear_x_mps)
    yaw_rate = float(yaw_rate_rad_s)
    if abs(linear) <= float(stop_threshold_mps):
        return {"gear": "STOP", "curvature_1pm": 0.0, "turning_radius_m": None}
    curvature = yaw_rate / linear
    radius = abs(1.0 / curvature) if abs(curvature) > 1e-6 else None
    return {
        "gear": "FORWARD" if linear > 0.0 else "REVERSE",
        "curvature_1pm": curvature,
        "turning_radius_m": radius,
    }


def decimate_xy(points, limit=240):
    if limit <= 0:
        raise ValueError("limit must be positive")
    step = max(1, len(points) // limit)
    return [[float(point.x), float(point.y)] for point in points[::step]][:limit + 1]


def validate_telemetry_v2(payload):
    if payload.get("schema") != SCHEMA:
        raise ValueError("telemetry schema must be v2")
    paths = payload.get("paths")
    required = {
        "planned_swaths", "planned_connectors", "planned_repairs",
        "current_component", "actual_cleaning", "actual_transit", "actual_repair",
        "blocked_intervals", "planned_ackermann_forward",
        "planned_ackermann_reverse", "actual_forward", "actual_reverse",
    }
    if not isinstance(paths, dict) or not required.issubset(paths):
        raise ValueError("semantic path layers are incomplete")
    if not isinstance(payload.get("blocked_intervals"), list):
        raise ValueError("blocked_intervals must be a list")
    if not isinstance(payload.get("deferred_swaths"), list):
        raise ValueError("deferred_swaths must be a list")
    steering = payload.get("steering")
    if not isinstance(steering, dict) or not {
        "front_left_rad", "front_right_rad", "virtual_rad", "configured_min_radius_m"
    }.issubset(steering):
        raise ValueError("Ackermann steering telemetry is incomplete")
    motion = payload.get("motion")
    if not isinstance(motion, dict) or motion.get("gear") not in {
        "FORWARD", "REVERSE", "STOP"
    }:
        raise ValueError("measured motion telemetry is incomplete")
    return True
