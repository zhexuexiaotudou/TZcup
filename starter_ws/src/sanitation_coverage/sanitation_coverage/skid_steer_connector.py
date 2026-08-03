"""Rotate-translate-rotate connectors suitable for a skid-steer chassis."""

import math

from .coverage_components import ComponentType, CoverageComponent


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _point_in_polygon(point, polygon) -> bool:
    x, y = point
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / ((y2 - y1) or 1e-12) + x1:
            inside = not inside
        previous = current
    return inside


def _segment_inside(start, end, polygon, sample_m=0.05) -> bool:
    count = max(1, int(math.ceil(math.dist(start, end) / sample_m)))
    return all(
        _point_in_polygon(
            (start[0] + (end[0] - start[0]) * i / count,
             start[1] + (end[1] - start[1]) * i / count),
            polygon,
        )
        for i in range(count + 1)
    )


def plan_skid_steer_connector(
    connector_id: str,
    start: tuple[float, float],
    start_yaw: float,
    goal: tuple[float, float],
    goal_yaw: float,
    safe_polygon: list[tuple[float, float]],
    allow_backup: bool = True,
) -> tuple[CoverageComponent, ...]:
    """Build an RTR connector, or a bounded BACKUP fallback when forward is unsafe."""
    if not safe_polygon:
        raise ValueError("safe_polygon is required")
    delta = (goal[0] - start[0], goal[1] - start[1])
    distance = math.hypot(*delta)
    travel_yaw = math.atan2(delta[1], delta[0]) if distance else goal_yaw
    reverse = False
    if allow_backup and abs(normalize_angle(travel_yaw - start_yaw)) > math.pi / 2:
        reverse = True
        travel_yaw = normalize_angle(travel_yaw + math.pi)
    if not _segment_inside(start, goal, safe_polygon):
        raise ValueError("connector translation leaves the footprint-safe polygon")
    components = []
    first_turn = normalize_angle(travel_yaw - start_yaw)
    if abs(first_turn) > 1e-3:
        components.append(CoverageComponent(
            f"{connector_id}-rotate-in", ComponentType.ROTATE, (start,), False, "TURN",
            {"delta_yaw_rad": first_turn, "target_yaw_rad": travel_yaw},
        ))
    if distance > 1e-4:
        kind = ComponentType.BACKUP if reverse else ComponentType.SHIFT
        components.append(CoverageComponent(
            f"{connector_id}-translate", kind, (start, goal), False,
            "BACKUP" if reverse else "SHIFT", {"travel_yaw_rad": travel_yaw},
        ))
    physical_arrival_yaw = normalize_angle(travel_yaw + (math.pi if reverse else 0.0))
    final_turn = normalize_angle(goal_yaw - physical_arrival_yaw)
    if abs(final_turn) > 1e-3:
        components.append(CoverageComponent(
            f"{connector_id}-rotate-out", ComponentType.ROTATE, (goal,), False, "TURN",
            {"delta_yaw_rad": final_turn, "target_yaw_rad": goal_yaw},
        ))
    return tuple(components)
