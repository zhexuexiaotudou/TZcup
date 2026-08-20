"""Fail-closed sampling of authoritative Nav2 occupancy-grid safety layers."""

from __future__ import annotations

import math


def quaternion_yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def sample_occupancy_grid(grid, x_m: float, y_m: float) -> int | None:
    """Return the cell value at a map point, or ``None`` outside valid data."""
    if grid is None or float(grid.info.resolution) <= 0.0:
        return None
    origin = grid.info.origin
    yaw = quaternion_yaw(origin.orientation)
    dx = float(x_m) - float(origin.position.x)
    dy = float(y_m) - float(origin.position.y)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    column = int(math.floor(local_x / float(grid.info.resolution)))
    row = int(math.floor(local_y / float(grid.info.resolution)))
    width = int(grid.info.width)
    height = int(grid.info.height)
    if not (0 <= column < width and 0 <= row < height):
        return None
    index = row * width + column
    if not 0 <= index < len(grid.data):
        return None
    return int(grid.data[index])


def keepout_clear(grid, x_m: float, y_m: float) -> bool:
    """Only an explicitly clear keepout-mask cell is action-safe."""
    return sample_occupancy_grid(grid, x_m, y_m) == 0


def costmap_clear(
    grid,
    x_m: float,
    y_m: float,
    *,
    lethal_threshold: int = 99,
) -> bool:
    """Treat missing, unknown and lethal costmap cells as unsafe."""
    value = sample_occupancy_grid(grid, x_m, y_m)
    return value is not None and 0 <= value < int(lethal_threshold)


def _point_in_polygon(x_m: float, y_m: float, polygon) -> bool:
    inside = False
    previous_x, previous_y = polygon[-1]
    for current_x, current_y in polygon:
        if ((previous_y > y_m) != (current_y > y_m)) and (
            x_m
            < (current_x - previous_x)
            * (y_m - previous_y)
            / ((current_y - previous_y) or 1e-12)
            + previous_x
        ):
            inside = not inside
        previous_x, previous_y = current_x, current_y
    return inside


def footprint_costmap_clear(
    grid,
    x_m: float,
    y_m: float,
    yaw_rad: float,
    footprint_xy,
    *,
    lethal_threshold: int = 99,
) -> bool:
    """Require every sampled cell under the oriented footprint to be clear."""
    if grid is None or not footprint_xy:
        return False
    cosine = math.cos(float(yaw_rad))
    sine = math.sin(float(yaw_rad))
    polygon = [
        (
            float(x_m) + cosine * float(local_x) - sine * float(local_y),
            float(y_m) + sine * float(local_x) + cosine * float(local_y),
        )
        for local_x, local_y in footprint_xy
    ]
    minimum_x = min(point[0] for point in polygon)
    maximum_x = max(point[0] for point in polygon)
    minimum_y = min(point[1] for point in polygon)
    maximum_y = max(point[1] for point in polygon)
    spacing = max(float(grid.info.resolution) * 0.5, 0.01)
    rows = int(math.ceil((maximum_y - minimum_y) / spacing)) + 1
    columns = int(math.ceil((maximum_x - minimum_x) / spacing)) + 1
    samples = list(polygon) + [(float(x_m), float(y_m))]
    for row in range(rows):
        sample_y = min(maximum_y, minimum_y + row * spacing)
        for column in range(columns):
            sample_x = min(maximum_x, minimum_x + column * spacing)
            if _point_in_polygon(sample_x, sample_y, polygon):
                samples.append((sample_x, sample_y))
    return all(
        costmap_clear(
            grid,
            sample_x,
            sample_y,
            lethal_threshold=lethal_threshold,
        )
        for sample_x, sample_y in samples
    )
