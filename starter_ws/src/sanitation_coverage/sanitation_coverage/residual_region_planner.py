"""Connected-component residual repair without cross-island brush-on stitching."""

from dataclasses import dataclass
import math
from typing import Iterable


Point = tuple[float, float]


@dataclass(frozen=True)
class ResidualPlan:
    regions: tuple[tuple[Point, ...], ...]
    swaths: tuple[tuple[Point, Point], ...]
    total_length_m: float
    truncated: bool


def connected_residual_regions(points: Iterable[Point], resolution: float) -> list[list[Point]]:
    remaining = {(round(float(x), 6), round(float(y), 6)) for x, y in points}
    regions = []
    threshold = resolution * 1.45
    while remaining:
        seed = remaining.pop()
        region = [seed]
        queue = [seed]
        while queue:
            current = queue.pop()
            neighbors = [point for point in remaining if math.dist(current, point) <= threshold]
            for point in neighbors:
                remaining.remove(point)
                region.append(point)
                queue.append(point)
        regions.append(sorted(region))
    return sorted(regions, key=lambda region: (-len(region), region[0]))


def _region_swaths(region: list[Point], resolution: float, brush_width: float):
    min_x, max_x = min(x for x, _ in region), max(x for x, _ in region)
    min_y, max_y = min(y for _, y in region), max(y for _, y in region)
    horizontal = (max_x - min_x) >= (max_y - min_y)
    axis = 1 if horizontal else 0
    # Cluster adjacent residual rows/columns into one brush-width corridor.
    # The old one-grid-row-per-swath rule produced many almost coincident
    # repair paths and exhausted the bounded repair budget without covering
    # the next residual island.
    ordered = sorted(region, key=lambda point: (point[axis], point[1 - axis]))
    groups: list[list[Point]] = []
    for point in ordered:
        if not groups or point[axis] - groups[-1][0][axis] > brush_width + 1e-9:
            groups.append([point])
        else:
            groups[-1].append(point)
    swaths = []
    for index, values in enumerate(groups):
        if horizontal:
            y = (min(point[1] for point in values) + max(point[1] for point in values)) / 2.0
            segment = ((min(p[0] for p in values) - brush_width / 2, y),
                       (max(p[0] for p in values) + brush_width / 2, y))
        else:
            x = (min(point[0] for point in values) + max(point[0] for point in values)) / 2.0
            segment = ((x, min(p[1] for p in values) - brush_width / 2),
                       (x, max(p[1] for p in values) + brush_width / 2))
        swaths.append(segment if index % 2 == 0 else (segment[1], segment[0]))
    return swaths


def plan_residual_regions(
    missed_points: Iterable[Point],
    resolution: float,
    brush_width: float,
    primary_length_m: float,
    max_ratio: float = 0.10,
) -> ResidualPlan:
    regions = connected_residual_regions(missed_points, resolution)
    budget = max(0.0, primary_length_m * max_ratio)
    selected = []
    used = 0.0
    truncated = False
    for region in regions:
        for swath in _region_swaths(region, resolution, brush_width):
            length = math.dist(*swath)
            if used + length > budget + 1e-9:
                truncated = True
                continue
            selected.append(swath)
            used += length
    return ResidualPlan(
        tuple(tuple(region) for region in regions), tuple(selected), used, truncated
    )
