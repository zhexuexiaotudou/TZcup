"""Order swaths into a stable adjacent boustrophedon route."""

from dataclasses import dataclass
import math
from typing import Iterable


Point = tuple[float, float]
Swath = tuple[Point, Point]


@dataclass(frozen=True)
class RoutedSwaths:
    swaths: tuple[Swath, ...]
    connector_distance_m: float
    start_distance_m: float
    ordering: str


def _normal_projection(swath: Swath, heading: float) -> float:
    center = ((swath[0][0] + swath[1][0]) / 2, (swath[0][1] + swath[1][1]) / 2)
    return -center[0] * math.sin(heading) + center[1] * math.cos(heading)


def _candidate(swaths: list[Swath], start: Point, label: str) -> RoutedSwaths:
    choices = []
    for first_reversed in (False, True):
        routed = []
        for index, swath in enumerate(swaths):
            reverse = first_reversed if index % 2 == 0 else not first_reversed
            routed.append((swath[1], swath[0]) if reverse else swath)
        start_distance = math.dist(start, routed[0][0])
        connector_distance = sum(
            math.dist(left[1], right[0]) for left, right in zip(routed, routed[1:])
        )
        choices.append(RoutedSwaths(
            tuple(routed), connector_distance, start_distance,
            f"{label}:{'reverse_first' if first_reversed else 'forward_first'}",
        ))
    return min(choices, key=lambda item: (item.start_distance_m + item.connector_distance_m, item.ordering))


def route_oriented_swaths(swaths: Iterable[Swath], start: Point) -> RoutedSwaths:
    normalized = [
        ((float(a[0]), float(a[1])), (float(b[0]), float(b[1]))) for a, b in swaths
    ]
    if not normalized:
        return RoutedSwaths((), 0.0, 0.0, "empty")
    if any(a == b for a, b in normalized):
        raise ValueError("degenerate swaths cannot be routed")
    longest = max(normalized, key=lambda swath: math.dist(*swath))
    heading = math.atan2(longest[1][1] - longest[0][1], longest[1][0] - longest[0][0])
    direction = (math.cos(heading), math.sin(heading))
    # Fields2Cover may encode neighboring swaths with alternating endpoint
    # order already. Normalize that source order before applying our own
    # boustrophedon alternation, otherwise connectors can cross the whole field.
    aligned = []
    for swath in normalized:
        vector = (swath[1][0] - swath[0][0], swath[1][1] - swath[0][1])
        aligned.append(
            swath if vector[0] * direction[0] + vector[1] * direction[1] >= 0.0
            else (swath[1], swath[0])
        )
    ascending = sorted(aligned, key=lambda swath: _normal_projection(swath, heading))
    candidates = [
        _candidate(ascending, start, "ascending"),
        _candidate(list(reversed(ascending)), start, "descending"),
    ]
    return min(
        candidates,
        key=lambda item: (item.start_distance_m + item.connector_distance_m, item.ordering),
    )
