"""Small dependency-free geometry helpers for the research environment."""

from __future__ import annotations

import math
from typing import Iterable, Sequence

from .models import Point2D, Polygon2D, Pose2D


EPSILON = 1.0e-9


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_in_polygon(point: Point2D, polygon: Polygon2D) -> bool:
    """Return true for interior points and points on the polygon boundary."""
    x, y = point
    inside = False
    for index, current in enumerate(polygon):
        previous = polygon[index - 1]
        x1, y1 = previous
        x2, y2 = current
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        if abs(cross) <= EPSILON and min(x1, x2) - EPSILON <= x <= max(x1, x2) + EPSILON and min(y1, y2) - EPSILON <= y <= max(y1, y2) + EPSILON:
            return True
        if (y1 > y) != (y2 > y):
            intersection_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < intersection_x:
                inside = not inside
    return inside


def distance_to_segment(point: Point2D, start: Point2D, end: Point2D) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= EPSILON:
        return distance(point, start)
    ratio = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared
    ratio = min(1.0, max(0.0, ratio))
    projection = (start[0] + ratio * dx, start[1] + ratio * dy)
    return distance(point, projection)


def _orientation(a: Point2D, b: Point2D, c: Point2D) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def segments_intersect(a: Point2D, b: Point2D, c: Point2D, d: Point2D) -> bool:
    o1 = _orientation(a, b, c)
    o2 = _orientation(a, b, d)
    o3 = _orientation(c, d, a)
    o4 = _orientation(c, d, b)
    if ((o1 > EPSILON and o2 < -EPSILON) or (o1 < -EPSILON and o2 > EPSILON)) and (
        (o3 > EPSILON and o4 < -EPSILON) or (o3 < -EPSILON and o4 > EPSILON)
    ):
        return True
    return any(
        abs(value) <= EPSILON and distance_to_segment(point, start, end) <= EPSILON
        for value, point, start, end in (
            (o1, c, a, b),
            (o2, d, a, b),
            (o3, a, c, d),
            (o4, b, c, d),
        )
    )


def distance_between_segments(a: Point2D, b: Point2D, c: Point2D, d: Point2D) -> float:
    if segments_intersect(a, b, c, d):
        return 0.0
    return min(
        distance_to_segment(a, c, d),
        distance_to_segment(b, c, d),
        distance_to_segment(c, a, b),
        distance_to_segment(d, a, b),
    )


def distance_segment_to_polygon_boundary(
    start: Point2D, end: Point2D, polygon: Polygon2D
) -> float:
    return min(
        distance_between_segments(start, end, polygon[index - 1], polygon[index])
        for index in range(len(polygon))
    )


def distance_to_polygon(point: Point2D, polygon: Polygon2D) -> float:
    if point_in_polygon(point, polygon):
        return 0.0
    return min(
        distance_to_segment(point, polygon[index - 1], polygon[index])
        for index in range(len(polygon))
    )


def distance_to_polygon_boundary(point: Point2D, polygon: Polygon2D) -> float:
    """Unsigned distance to a polygon edge, including for interior points."""
    return min(
        distance_to_segment(point, polygon[index - 1], polygon[index])
        for index in range(len(polygon))
    )


def sample_segment(start: Pose2D, end: Pose2D, spacing: float) -> tuple[Pose2D, ...]:
    length = distance((start.x, start.y), (end.x, end.y))
    if length <= EPSILON:
        return (start,)
    count = max(1, int(math.ceil(length / spacing)))
    yaw_delta = wrap_angle(end.yaw - start.yaw)
    return tuple(
        Pose2D(
            start.x + (end.x - start.x) * index / count,
            start.y + (end.y - start.y) * index / count,
            wrap_angle(start.yaw + yaw_delta * index / count),
        )
        for index in range(count + 1)
    )


def polyline_length(points: Sequence[Pose2D]) -> float:
    return sum(
        distance((previous.x, previous.y), (current.x, current.y))
        for previous, current in zip(points, points[1:])
    )


def validate_ackermann_path(
    points: Sequence[Pose2D],
    *,
    max_curvature: float,
    heading_tolerance: float = 0.18,
) -> tuple[bool, str]:
    """Reject lateral motion, in-place rotation, and excessive curvature."""
    if not points:
        return True, "wait"
    for previous, current in zip(points, points[1:]):
        ds = distance((previous.x, previous.y), (current.x, current.y))
        dyaw = wrap_angle(current.yaw - previous.yaw)
        if ds <= EPSILON:
            if abs(dyaw) > EPSILON:
                return False, "in_place_rotation_forbidden"
            continue
        chord_yaw = math.atan2(current.y - previous.y, current.x - previous.x)
        midpoint_yaw = wrap_angle(previous.yaw + 0.5 * dyaw)
        heading_error = abs(wrap_angle(chord_yaw - midpoint_yaw))
        reverse_heading_error = abs(
            wrap_angle(chord_yaw - midpoint_yaw - math.pi)
        )
        if min(heading_error, reverse_heading_error) > heading_tolerance:
            return False, "body_lateral_motion_forbidden"
        if abs(dyaw) / ds > max_curvature * 1.05 + EPSILON:
            return False, "curvature_limit_exceeded"
    return True, "ok"


def _sample_turn_then_line(
    start: Pose2D,
    goal: Point2D,
    radius: float,
    turn_sign: int,
    spacing: float,
) -> tuple[Pose2D, ...] | None:
    center = (
        start.x - turn_sign * radius * math.sin(start.yaw),
        start.y + turn_sign * radius * math.cos(start.yaw),
    )
    dx = goal[0] - center[0]
    dy = goal[1] - center[1]
    center_distance = math.hypot(dx, dy)
    if center_distance <= radius + EPSILON:
        return None
    phi = math.atan2(dy, dx)
    alpha = math.acos(radius / center_distance)
    start_radial = wrap_angle(start.yaw - turn_sign * math.pi / 2.0)
    candidates: list[tuple[float, tuple[Pose2D, ...]]] = []
    for tangent_angle in (phi + alpha, phi - alpha):
        tangent = (
            center[0] + radius * math.cos(tangent_angle),
            center[1] + radius * math.sin(tangent_angle),
        )
        line_yaw = math.atan2(goal[1] - tangent[1], goal[0] - tangent[0])
        tangent_yaw = wrap_angle(tangent_angle + turn_sign * math.pi / 2.0)
        if abs(wrap_angle(line_yaw - tangent_yaw)) > 1.0e-5:
            continue
        if turn_sign > 0:
            sweep = (tangent_angle - start_radial) % (2.0 * math.pi)
        else:
            sweep = (start_radial - tangent_angle) % (2.0 * math.pi)
        arc_length = radius * sweep
        arc_count = max(1, int(math.ceil(arc_length / spacing)))
        path = []
        for index in range(arc_count + 1):
            radial = start_radial + turn_sign * sweep * index / arc_count
            path.append(
                Pose2D(
                    center[0] + radius * math.cos(radial),
                    center[1] + radius * math.sin(radial),
                    wrap_angle(radial + turn_sign * math.pi / 2.0),
                )
            )
        straight_length = distance(tangent, goal)
        straight_count = max(1, int(math.ceil(straight_length / spacing)))
        for index in range(1, straight_count + 1):
            ratio = index / straight_count
            path.append(
                Pose2D(
                    tangent[0] + (goal[0] - tangent[0]) * ratio,
                    tangent[1] + (goal[1] - tangent[1]) * ratio,
                    line_yaw,
                )
            )
        candidates.append((arc_length + straight_length, tuple(path)))
    if not candidates:
        return None
    return min(candidates, key=lambda item: item[0])[1]


def ackermann_path_to_point(
    start: Pose2D,
    goal: Point2D,
    *,
    min_turn_radius: float,
    spacing: float,
) -> tuple[Pose2D, ...]:
    """Create a forward-only one-arc-plus-line path to a point.

    The final heading is intentionally free because the RL action is a global
    reference trajectory, not a direct steering command.
    """
    if distance((start.x, start.y), goal) <= spacing * 0.25:
        return (start,)
    paths = [
        path
        for sign in (-1, 1)
        if (path := _sample_turn_then_line(start, goal, min_turn_radius, sign, spacing))
    ]
    if not paths:
        raise ValueError("goal lies inside both minimum-turn circles")
    return min(paths, key=polyline_length)


def curvature_limited_reference_path_for_skid_steer(
    start: Pose2D,
    goal: Point2D,
    *,
    min_turn_radius: float,
    spacing: float,
) -> tuple[Pose2D, ...]:
    """Build a curvature-limited reference path for the A300 skid-steer base.

    The retained Ackermann-named helper is a compatibility implementation
    detail.  This wrapper emits only a map-frame reference path: it neither
    models nor commands physical steering joints.
    """
    return ackermann_path_to_point(
        start,
        goal,
        min_turn_radius=min_turn_radius,
        spacing=spacing,
    )


def cells_within_path(
    centers: Sequence[Point2D],
    path: Sequence[Pose2D],
    radius: float,
) -> set[int]:
    if not path:
        return set()
    result: set[int] = set()
    segments: Iterable[tuple[Point2D, Point2D]] = (
        ((a.x, a.y), (b.x, b.y)) for a, b in zip(path, path[1:])
    )
    segment_list = tuple(segments)
    for index, center in enumerate(centers):
        if distance(center, (path[0].x, path[0].y)) <= radius:
            result.add(index)
            continue
        if any(distance_to_segment(center, start, end) <= radius for start, end in segment_list):
            result.add(index)
    return result
