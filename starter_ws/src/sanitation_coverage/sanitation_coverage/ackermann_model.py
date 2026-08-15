"""Canonical Ackermann product vehicle model constants and formulas.

This module is the single source of truth for the frozen Ackermann geometry
used by the Xacro vehicle expansion, the Nav2 profile, the coverage planner,
the demo task geometry, the inventory generator and the fast CI tests.  Every
value is traceable to the physical chassis described in
``sanitation_vehicle.urdf.xacro`` (base_length=1.15, base_width=0.72, wheel
radius=0.14, front/rear axle x=+/-0.38, front/rear track 0.80).  The product
footprint also contains the two deployed side brushes; omitting them would
make Nav2 and the collision monitor narrower than the physical machine.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Frozen physical geometry (matches the Xacro argument defaults)
# ---------------------------------------------------------------------------

BASE_LENGTH_M = 1.15
BASE_WIDTH_M = 0.72
BASE_HEIGHT_M = 0.28
WHEEL_RADIUS_M = 0.14
WHEEL_WIDTH_M = 0.08
AXLE_X_M = 0.38
TRACK_M = 0.80
WHEEL_POS_Z_M = -0.02
WHEEL_HALF_LENGTH_M = WHEEL_RADIUS_M  # x half-extent in the ground plane
WHEEL_HALF_WIDTH_M = WHEEL_WIDTH_M / 2.0  # y half-extent in the ground plane
BRUSH_RADIUS_M = 0.14
BRUSH_CENTER_Y_M = 0.52
LEGACY_BRUSH_FORWARD_X_M = 0.58
ACKERMANN_BRUSH_FORWARD_X_M = 0.68
OPERATION_WIDTH_M = 2.0 * (BRUSH_CENTER_Y_M + BRUSH_RADIUS_M)

# Chassis collision wheel-well geometry (Xacro ``sanitation_wheel_well``):
# a centre spine plus front/rear stubs that leave the wheel wells open.
CHASSIS_SPINE_X_HALF_M = 0.22
CHASSIS_SPINE_Y_HALF_M = BASE_WIDTH_M / 2.0
CHASSIS_STUB_X_RANGE_M = (0.22, 0.575)
CHASSIS_STUB_Y_HALF_M = 0.26

# Wheelbase: distance between the front and the rear axle.
WHEELBASE_M = 2.0 * AXLE_X_M  # 0.76

# ---------------------------------------------------------------------------
# Frozen steering design
# ---------------------------------------------------------------------------

# Bicycle-equivalent virtual steering limit (degrees and radians).
MAX_VIRTUAL_STEERING_DEG = 28.0
MAX_VIRTUAL_STEERING_RAD = math.radians(MAX_VIRTUAL_STEERING_DEG)

# Physical symmetric steering joint limit, including the >= 2 degree margin
# over the inner wheel angle required by the frozen virtual steering angle.
PHYSICAL_STEERING_LIMIT_DEG = 38.5
PHYSICAL_STEERING_LIMIT_RAD = math.radians(PHYSICAL_STEERING_LIMIT_DEG)


def minimum_radius_m() -> float:
    """Bicycle-equivalent minimum radius: R = L / tan(delta_max)."""
    return WHEELBASE_M / math.tan(MAX_VIRTUAL_STEERING_RAD)


def inner_wheel_angle_rad() -> float:
    """Ackermann inner wheel angle at the frozen minimum radius."""
    radius = minimum_radius_m()
    return math.atan2(WHEELBASE_M, radius - TRACK_M / 2.0)


def outer_wheel_angle_rad() -> float:
    """Ackermann outer wheel angle at the frozen minimum radius."""
    radius = minimum_radius_m()
    return math.atan2(WHEELBASE_M, radius + TRACK_M / 2.0)


def plugin_steering_clamp_rad() -> float:
    """Gazebo Ackermann plugin clamp producing the frozen minimum radius.

    Gazebo's Ackermann plugin applies its steering limit through an
    angular-velocity clamp whose bicycle-equivalent relation is
    ``R = L / sin(limit)``.  Passing the physical 28 deg virtual angle blindly
    would therefore yield a smaller radius than the frozen design, so the
    plugin clamp is computed from the frozen radius instead.
    """
    return math.asin(WHEELBASE_M / minimum_radius_m())


def plugin_steering_clamp_deg() -> float:
    return math.degrees(plugin_steering_clamp_rad())


def curvature_from_virtual_steering(virtual_steering_rad: float) -> float:
    """Path curvature kappa = tan(delta_virtual) / L."""
    return math.tan(virtual_steering_rad) / WHEELBASE_M


def radius_from_virtual_steering(virtual_steering_rad: float) -> float:
    """Instantaneous turning radius R = L / tan(delta_virtual)."""
    tangent = math.tan(virtual_steering_rad)
    if abs(tangent) < 1e-12:
        return math.inf
    return WHEELBASE_M / tangent


def virtual_steering_from_radius(radius_m: float) -> float:
    if radius_m is None or not math.isfinite(float(radius_m)) or float(radius_m) <= 0.0:
        return 0.0
    return math.atan2(WHEELBASE_M, float(radius_m))


@dataclass(frozen=True)
class AckermannWheelPose:
    """Wheel centre pose in the base_footprint frame."""

    side: str
    axle_x_m: float
    y_m: float
    steered: bool


def wheel_poses() -> tuple[AckermannWheelPose, ...]:
    half_track = TRACK_M / 2.0
    return (
        AckermannWheelPose("front_left", AXLE_X_M, half_track, True),
        AckermannWheelPose("front_right", AXLE_X_M, -half_track, True),
        AckermannWheelPose("rear_left", -AXLE_X_M, half_track, False),
        AckermannWheelPose("rear_right", -AXLE_X_M, -half_track, False),
    )


def honest_footprint_polygon() -> list[list[float]]:
    """Honest 2-D hull of chassis, swept wheels and deployed side brushes.

    Each wheel cylinder has its axis along the vehicle Y axis, so its ground
    footprint is a 0.28 x 0.08 rectangle centred on the wheel. The returned
    hull includes the complete front-wheel sweep through the physical joint
    limit, not just the straight-wheel rectangle.
    """
    half_length = BASE_LENGTH_M / 2.0
    half_width = BASE_WIDTH_M / 2.0
    max_x = max(
        half_length,
        *(abs(pose.axle_x_m) + WHEEL_HALF_LENGTH_M for pose in wheel_poses()),
    )
    swept_half_y = (
        WHEEL_HALF_LENGTH_M * math.sin(PHYSICAL_STEERING_LIMIT_RAD)
        + WHEEL_HALF_WIDTH_M * math.cos(PHYSICAL_STEERING_LIMIT_RAD)
    )
    max_x = max(max_x, ACKERMANN_BRUSH_FORWARD_X_M + BRUSH_RADIUS_M)
    min_x = -half_length
    max_y = max(
        half_width,
        TRACK_M / 2.0 + swept_half_y,
        BRUSH_CENTER_Y_M + BRUSH_RADIUS_M,
    )
    return [
        [max_x, max_y],
        [max_x, -max_y],
        [min_x, -max_y],
        [min_x, max_y],
    ]


def honest_footprint_radius_m() -> float:
    return max(
        math.hypot(x, y) for x, y in honest_footprint_polygon()
    )


def swept_turning_radius_m() -> float:
    """Outer radius of the complete product footprint at maximum steering."""
    radius = minimum_radius_m()
    rear_axle_x = -AXLE_X_M
    return max(
        math.hypot(x - rear_axle_x, y - radius)
        for x, y in honest_footprint_polygon()
    )


def _rotated_wheel_bbox(
    axle_x_m: float, y_m: float, steering_rad: float
) -> tuple[float, float, float, float]:
    """Axis-aligned bounding box of the steered wheel rectangle."""
    cosine = math.cos(steering_rad)
    sine = math.sin(steering_rad)
    xs = []
    ys = []
    for hx in (-WHEEL_HALF_LENGTH_M, WHEEL_HALF_LENGTH_M):
        for hy in (-WHEEL_HALF_WIDTH_M, WHEEL_HALF_WIDTH_M):
            xs.append(axle_x_m + hx * cosine - hy * sine)
            ys.append(y_m + hx * sine + hy * cosine)
    return min(xs), max(xs), min(ys), max(ys)


def _rect_to_rect_gap(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> float:
    """Minimum distance between two axis-aligned rectangles (0 = touch)."""
    x1, x2, y1, y2 = first
    x3, x4, y3, y4 = second
    gap_x = max(0.0, max(x1 - x4, x3 - x2))
    gap_y = max(0.0, max(y1 - y4, y3 - y2))
    return math.hypot(gap_x, gap_y)


def _chassis_boxes() -> tuple[tuple[float, float, float, float], ...]:
    half = CHASSIS_SPINE_X_HALF_M
    width = CHASSIS_SPINE_Y_HALF_M
    stub_x1, stub_x2 = CHASSIS_STUB_X_RANGE_M
    stub_y = CHASSIS_STUB_Y_HALF_M
    return (
        (-half, half, -width, width),
        (stub_x1, stub_x2, -stub_y, stub_y),
        (-stub_x2, -stub_x1, -stub_y, stub_y),
    )


def clearance_wheel_to_chassis(steering_rad: float) -> float:
    """Minimum wheel/body clearance over the full steering range."""
    clearances = []
    for pose in wheel_poses():
        wheel_box = _rotated_wheel_bbox(
            pose.axle_x_m, pose.y_m, steering_rad if pose.steered else 0.0
        )
        for chassis_box in _chassis_boxes():
            clearances.append(
                _rect_to_rect_gap(wheel_box, chassis_box)
            )
    return min(clearances)


def clearance_wheel_to_brush(steering_rad: float) -> float:
    """Minimum wheel/brush horizontal clearance (Ackermann brush position)."""
    brush_radius = BRUSH_RADIUS_M
    brush_x = ACKERMANN_BRUSH_FORWARD_X_M
    brush_positions = ((brush_x, BRUSH_CENTER_Y_M), (brush_x, -BRUSH_CENTER_Y_M))
    clearances = []
    for pose in wheel_poses():
        wheel_box = _rotated_wheel_bbox(
            pose.axle_x_m, pose.y_m, steering_rad if pose.steered else 0.0
        )
        x1, x2, y1, y2 = wheel_box
        for bx, by in brush_positions:
            nearest_x = min(max(bx, x1), x2)
            nearest_y = min(max(by, y1), y2)
            clearances.append(math.hypot(bx - nearest_x, by - nearest_y) - brush_radius)
    return min(clearances)


def sampled_steering_clearances() -> dict[str, float]:
    angles = (-PHYSICAL_STEERING_LIMIT_RAD, -MAX_VIRTUAL_STEERING_RAD, 0.0,
              MAX_VIRTUAL_STEERING_RAD, PHYSICAL_STEERING_LIMIT_RAD)
    return {
        "sampled_steering_deg": [math.degrees(angle) for angle in angles],
        "minimum_wheel_to_chassis_clearance_m": min(
            clearance_wheel_to_chassis(angle) for angle in angles
        ),
        "minimum_wheel_to_brush_clearance_m": min(
            clearance_wheel_to_brush(angle) for angle in angles
        ),
        "minimum_chassis_clearance_required_m": 0.0,
    }


def summary() -> dict[str, object]:
    return {
        "wheelbase_m": WHEELBASE_M,
        "track_width_m": TRACK_M,
        "wheel_radius_m": WHEEL_RADIUS_M,
        "wheel_width_m": WHEEL_WIDTH_M,
        "front_rear_axle_x_m": AXLE_X_M,
        "base_length_m": BASE_LENGTH_M,
        "base_width_m": BASE_WIDTH_M,
        "operation_width_m": OPERATION_WIDTH_M,
        "brush_radius_m": BRUSH_RADIUS_M,
        "brush_center_y_m": BRUSH_CENTER_Y_M,
        "brush_forward_x_m": ACKERMANN_BRUSH_FORWARD_X_M,
        "max_virtual_steering_deg": MAX_VIRTUAL_STEERING_DEG,
        "max_virtual_steering_rad": MAX_VIRTUAL_STEERING_RAD,
        "minimum_radius_m": minimum_radius_m(),
        "inner_wheel_angle_deg": math.degrees(inner_wheel_angle_rad()),
        "outer_wheel_angle_deg": math.degrees(outer_wheel_angle_rad()),
        "physical_steering_limit_deg": PHYSICAL_STEERING_LIMIT_DEG,
        "plugin_steering_clamp_deg": plugin_steering_clamp_deg(),
        "plugin_steering_clamp_rad": plugin_steering_clamp_rad(),
        "footprint_polygon": honest_footprint_polygon(),
        "footprint_radius_m": honest_footprint_radius_m(),
        "swept_turning_radius_m": swept_turning_radius_m(),
        "clearances": sampled_steering_clearances(),
    }
