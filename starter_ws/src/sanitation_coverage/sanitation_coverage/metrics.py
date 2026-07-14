import math


def segment_length(start, end):
    return math.hypot(end[0] - start[0], end[1] - start[1])


def path_length(points):
    return sum(segment_length(a, b) for a, b in zip(points, points[1:]))


def repair_degenerate_swaths(swaths, turns, nav_points, tolerance=1.0e-3):
    """Repair OpenNav path-component end points from adjacent turn boundaries.

    OpenNav Coverage currently emits the correct dense nav_path and turn paths,
    but the end point of each swath in PathComponents can equal its start point.
    The first point of the following turn is the corresponding swath end; the
    final dense-path point closes the last swath.
    """
    if not swaths or not nav_points:
        return swaths, False
    if all(segment_length(start, end) > tolerance for start, end in swaths):
        return swaths, False
    if len(turns) != len(swaths) - 1 or any(not turn for turn in turns):
        return swaths, False
    repaired = []
    for index, (start, _end) in enumerate(swaths):
        end = turns[index][0] if index < len(turns) else nav_points[-1]
        repaired.append((start, end))
    return repaired, True


def point_in_polygon(x, y, polygon):
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y):
            crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing_x:
                inside = not inside
        previous = current
    return inside


def point_segment_distance(x, y, start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator == 0.0:
        return math.hypot(x - start[0], y - start[1])
    projection = ((x - start[0]) * dx + (y - start[1]) * dy) / denominator
    projection = min(1.0, max(0.0, projection))
    closest_x = start[0] + projection * dx
    closest_y = start[1] + projection * dy
    return math.hypot(x - closest_x, y - closest_y)


def raster_coverage_metrics(polygon, swaths, width, resolution=0.10):
    min_x = min(point[0] for point in polygon)
    max_x = max(point[0] for point in polygon)
    min_y = min(point[1] for point in polygon)
    max_y = max(point[1] for point in polygon)
    target = covered = repeated = passes = 0
    y = min_y + resolution / 2.0
    while y < max_y:
        x = min_x + resolution / 2.0
        while x < max_x:
            if point_in_polygon(x, y, polygon):
                target += 1
                count = sum(
                    point_segment_distance(x, y, start, end) <= width / 2.0
                    for start, end in swaths
                )
                passes += count
                covered += count >= 1
                repeated += count >= 2
            x += resolution
        y += resolution
    cell_area = resolution * resolution
    target_area = target * cell_area
    covered_area = covered * cell_area
    repeated_area = repeated * cell_area
    return {
        "resolution_m": resolution,
        "target_area_m2": target_area,
        "covered_area_m2": covered_area,
        "missed_area_m2": max(0.0, target_area - covered_area),
        "repeated_area_m2": repeated_area,
        "coverage_rate": covered / target if target else 0.0,
        "miss_rate": (target - covered) / target if target else 0.0,
        "repeat_rate": repeated / target if target else 0.0,
        "gross_swept_area_m2": passes * cell_area,
    }
