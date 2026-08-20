import bisect
import math
import statistics


def segment_length(start, end):
    return math.hypot(end[0] - start[0], end[1] - start[1])


def path_length(points):
    return sum(segment_length(a, b) for a, b in zip(points, points[1:]))


def path_heading_variation(headings):
    """Return total absolute wrapped heading change along a sampled path."""
    return sum(
        abs(
            math.atan2(
                math.sin(float(second) - float(first)),
                math.cos(float(second) - float(first)),
            )
        )
        for first, second in zip(headings, headings[1:])
    )


def projected_path_progress(points, position):
    """Arc distance of the closest projected point on a polyline."""
    if len(points) < 2:
        return 0.0
    best_distance = math.inf
    best_progress = 0.0
    cumulative = 0.0
    px, py = map(float, position[:2])
    for start, end in zip(points, points[1:]):
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        length_sq = dx * dx + dy * dy
        length = math.sqrt(length_sq)
        fraction = 0.0 if length_sq <= 1e-12 else min(
            1.0,
            max(
                0.0,
                ((px - start[0]) * dx + (py - start[1]) * dy) / length_sq,
            ),
        )
        closest_x = float(start[0]) + fraction * dx
        closest_y = float(start[1]) + fraction * dy
        distance = math.hypot(px - closest_x, py - closest_y)
        if distance < best_distance:
            best_distance = distance
            best_progress = cumulative + fraction * length
        cumulative += length
    return best_progress


def split_path_at_curvature_reversals(points, headings, *, tolerance_rad=1e-4):
    """Split a sampled path where signed curvature changes direction.

    Dubins CCC paths can pass close to an earlier branch of the same path.
    Giving each curvature primitive to a stateful path follower separately
    preserves path topology without changing the collision-checked geometry.
    The boundary pose is intentionally shared by adjacent sections.
    """
    if len(points) != len(headings):
        raise ValueError("points and headings must have the same length")
    if len(points) < 2:
        return [(list(points), list(headings))] if points else []

    boundaries = [0]
    active_class = None
    for edge_index, (first, second) in enumerate(
        zip(headings, headings[1:])
    ):
        delta = math.atan2(
            math.sin(float(second) - float(first)),
            math.cos(float(second) - float(first)),
        )
        curvature_class = (
            1 if delta > tolerance_rad
            else -1 if delta < -tolerance_rad
            else 0
        )
        if active_class is not None and curvature_class != active_class:
            boundary = edge_index
            if boundary > boundaries[-1]:
                boundaries.append(boundary)
        active_class = curvature_class
    if boundaries[-1] != len(points) - 1:
        boundaries.append(len(points) - 1)
    return [
        (
            list(points[start : end + 1]),
            list(headings[start : end + 1]),
        )
        for start, end in zip(boundaries, boundaries[1:])
        if end > start
    ]


def semantic_path_distances(timed_samples):
    """Split executed base motion by the brush state at both segment ends."""
    brush_on = 0.0
    brush_off = 0.0
    transition = 0.0
    for first, second in zip(timed_samples, timed_samples[1:]):
        distance = segment_length(first[1:3], second[1:3])
        first_brush = bool(first[4])
        second_brush = bool(second[4])
        if first_brush and second_brush:
            brush_on += distance
        elif not first_brush and not second_brush:
            brush_off += distance
        else:
            transition += distance
    return {
        "brush_on_distance_m": brush_on,
        "brush_off_distance_m": brush_off,
        "brush_transition_distance_m": transition,
        "total_distance_m": brush_on + brush_off + transition,
    }


def point_line_distance(x, y, start, end):
    """Perpendicular distance to an infinite swath centreline."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    denominator = math.hypot(dx, dy)
    if denominator == 0.0:
        return math.hypot(x - start[0], y - start[1])
    return abs(dy * x - dx * y + end[0] * start[1] - end[1] * start[0]) / denominator


def swath_absolute_cross_track_errors(
    timed_samples, swaths, brush_forward_offset_m=0.55
):
    """Return absolute brush-centre error to the nearest planned swath line."""
    errors = []
    for sample in timed_samples:
        if not bool(sample[4]) or sample[5] != "EXECUTING_SWATH":
            continue
        yaw = float(sample[3])
        brush_x = float(sample[1]) + brush_forward_offset_m * math.cos(yaw)
        brush_y = float(sample[2]) + brush_forward_offset_m * math.sin(yaw)
        if swaths:
            errors.append(min(
                point_line_distance(brush_x, brush_y, start, end)
                for start, end in swaths
            ))
    return errors


def swath_lateral_errors(timed_samples, swaths, brush_forward_offset_m=0.55):
    """Backward-compatible name for absolute planned-line cross-track error."""
    return swath_absolute_cross_track_errors(
        timed_samples, swaths, brush_forward_offset_m
    )


def _signed_line_offset(x, y, start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    denominator = math.hypot(dx, dy)
    if denominator == 0.0:
        return None
    return (dx * (y - start[1]) - dy * (x - start[0])) / denominator


def _line_projection_fraction(x, y, start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator == 0.0:
        return None
    return ((x - start[0]) * dx + (y - start[1]) * dy) / denominator


def swath_straightness_errors(
    timed_samples,
    swaths,
    brush_forward_offset_m=0.55,
    endpoint_fraction=0.10,
    minimum_group_samples=3,
):
    """Measure weave within each executed straight swath independently.

    The central 80 percent excludes entry/exit settling and measures the steady
    straight-line phase.  A constant lateral offset is deliberately removed
    per continuous swath run.
    Absolute map alignment is reported separately by
    :func:`swath_absolute_cross_track_errors` and localization metrics.  This
    prevents RTK/map-frame bias from being misreported as a disorderly coverage
    path while still retaining the planned-line diagnostic.
    """
    groups = []
    current = []
    for sample in timed_samples:
        if bool(sample[4]) and sample[5] == "EXECUTING_SWATH":
            yaw = float(sample[3])
            current.append((
                float(sample[1]) + brush_forward_offset_m * math.cos(yaw),
                float(sample[2]) + brush_forward_offset_m * math.sin(yaw),
            ))
        elif current:
            groups.append(current)
            current = []
    if current:
        groups.append(current)

    errors = []
    for group in groups:
        if len(group) < minimum_group_samples or not swaths:
            continue
        selected = min(
            swaths,
            key=lambda swath: statistics.median(
                point_line_distance(x, y, *swath) for x, y in group
            ),
        )
        interior_offsets = []
        for x, y in group:
            projection = _line_projection_fraction(x, y, *selected)
            offset = _signed_line_offset(x, y, *selected)
            if (
                projection is not None
                and offset is not None
                and endpoint_fraction <= projection <= 1.0 - endpoint_fraction
            ):
                interior_offsets.append(offset)
        if len(interior_offsets) < minimum_group_samples:
            continue
        centre = statistics.median(interior_offsets)
        errors.extend(abs(offset - centre) for offset in interior_offsets)
    return errors


def summarize_distances(values):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return {
            'sample_count': 0,
            'mean_m': None,
            'median_m': None,
            'rmse_m': None,
            'p95_m': None,
            'max_m': None,
        }
    p95_index = min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)
    return {
        'sample_count': len(ordered),
        'mean_m': statistics.fmean(ordered),
        'median_m': statistics.median(ordered),
        'rmse_m': math.sqrt(sum(value * value for value in ordered) / len(ordered)),
        'p95_m': ordered[p95_index],
        'max_m': ordered[-1],
    }


def synchronized_pose_errors(estimates, truths, tolerance_sec=0.05):
    """Interpolate truth at estimate ROS stamps and return XY/yaw errors.

    The callback arrival order is deliberately ignored: comparing the latest
    estimate against the latest truth turns transport latency into apparent
    localization error while the vehicle is moving.  Ground truth is used only
    here in the evaluator.  Linear interpolation avoids converting a bounded
    sensor timestamp phase offset into distance error; no truth sample or
    interpolated pose is published back to the production graph.
    """
    ordered_truths = sorted(truths, key=lambda sample: float(sample[0]))
    truth_times = [float(sample[0]) for sample in ordered_truths]
    xy_errors = []
    yaw_errors = []
    sync_errors = []
    dropped = 0
    for estimate in estimates:
        stamp = float(estimate[0])
        index = bisect.bisect_left(truth_times, stamp)
        if index < len(ordered_truths) and abs(truth_times[index] - stamp) <= 1e-9:
            truth_x = float(ordered_truths[index][1])
            truth_y = float(ordered_truths[index][2])
            truth_yaw = float(ordered_truths[index][3])
            support_error = 0.0
        elif index == 0 or index >= len(ordered_truths):
            dropped += 1
            continue
        else:
            left = ordered_truths[index - 1]
            right = ordered_truths[index]
            left_stamp = float(left[0])
            right_stamp = float(right[0])
            span = right_stamp - left_stamp
            if span <= 0.0 or span > 2.0 * tolerance_sec:
                dropped += 1
                continue
            fraction = (stamp - left_stamp) / span
            truth_x = float(left[1]) + fraction * (
                float(right[1]) - float(left[1])
            )
            truth_y = float(left[2]) + fraction * (
                float(right[2]) - float(left[2])
            )
            yaw_span = math.atan2(
                math.sin(float(right[3]) - float(left[3])),
                math.cos(float(right[3]) - float(left[3])),
            )
            truth_yaw = float(left[3]) + fraction * yaw_span
            support_error = min(stamp - left_stamp, right_stamp - stamp)
        if support_error > tolerance_sec:
            dropped += 1
            continue
        xy_errors.append(math.hypot(
            float(estimate[1]) - truth_x,
            float(estimate[2]) - truth_y,
        ))
        yaw_errors.append(abs(math.atan2(
            math.sin(float(estimate[3]) - truth_yaw),
            math.cos(float(estimate[3]) - truth_yaw),
        )))
        sync_errors.append(support_error)
    return xy_errors, yaw_errors, sync_errors, dropped


def synchronized_xy_errors(estimates, truths, tolerance_sec=0.05):
    """Backward-compatible XY-only view of :func:`synchronized_pose_errors`."""
    xy_errors, _yaw_errors, sync_errors, dropped = synchronized_pose_errors(
        estimates, truths, tolerance_sec
    )
    return xy_errors, sync_errors, dropped


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


def point_in_cleanable_area(x, y, outer_polygon, exclusion_polygons=()):
    return point_in_polygon(x, y, outer_polygon) and not any(
        point_in_polygon(x, y, polygon) for polygon in exclusion_polygons
    )


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


def raster_coverage_metrics(
    polygon, swaths, width, resolution=0.10, exclusion_polygons=()
):
    min_x = min(point[0] for point in polygon)
    max_x = max(point[0] for point in polygon)
    min_y = min(point[1] for point in polygon)
    max_y = max(point[1] for point in polygon)
    target = covered = repeated = passes = 0
    y = min_y + resolution / 2.0
    while y < max_y:
        x = min_x + resolution / 2.0
        while x < max_x:
            if point_in_cleanable_area(x, y, polygon, exclusion_polygons):
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


def empirical_swept_metrics(
    polygon, timed_points, width, resolution=0.10, exclusion_polygons=()
):
    """Rasterize actual brush-on poses, with continuous motion counted once.

    A cell becomes a repeated-cleaning cell only after the footprint leaves it
    for at least five samples and later returns. This avoids labeling every
    high-rate adjacent pose on one straight pass as overlap.
    """
    min_x = min(point[0] for point in polygon)
    max_x = max(point[0] for point in polygon)
    min_y = min(point[1] for point in polygon)
    max_y = max(point[1] for point in polygon)
    cells = {}
    rows = max(0, int(math.ceil((max_y - min_y) / resolution)))
    columns = max(0, int(math.ceil((max_x - min_x) / resolution)))
    for iy in range(rows):
        y = min_y + (iy + 0.5) * resolution
        for ix in range(columns):
            x = min_x + (ix + 0.5) * resolution
            if point_in_cleanable_area(x, y, polygon, exclusion_polygons):
                cells[(ix, iy)] = (x, y)
    visited = set()
    repeated = set()
    last_seen = {}
    radius_cells = int(math.ceil(width / 2.0 / resolution))
    for index, (_stamp, x, y) in enumerate(timed_points):
        center_ix = int((x - min_x) / resolution)
        center_iy = int((y - min_y) / resolution)
        for iy in range(center_iy - radius_cells, center_iy + radius_cells + 1):
            for ix in range(center_ix - radius_cells, center_ix + radius_cells + 1):
                cell_index = (ix, iy)
                if cell_index not in cells:
                    continue
                cell_x, cell_y = cells[cell_index]
                if math.hypot(cell_x - x, cell_y - y) <= width / 2.0:
                    if cell_index in last_seen and index - last_seen[cell_index] > 5:
                        repeated.add(cell_index)
                    visited.add(cell_index)
                    last_seen[cell_index] = index
    cell_area = resolution * resolution
    target_area = len(cells) * cell_area
    covered_area = len(visited) * cell_area
    repeated_area = len(repeated) * cell_area
    duration = (
        timed_points[-1][0] - timed_points[0][0] if len(timed_points) >= 2 else 0.0
    )
    return {
        "resolution_m": resolution,
        "target_area_m2": target_area,
        "covered_area_m2": covered_area,
        "missed_area_m2": max(0.0, target_area - covered_area),
        "repeated_area_m2": repeated_area,
        "coverage_rate": len(visited) / len(cells) if cells else 0.0,
        "miss_rate": (len(cells) - len(visited)) / len(cells) if cells else 0.0,
        "repeat_rate": len(repeated) / len(cells) if cells else 0.0,
        "brush_on_duration_sec": max(0.0, duration),
        "metric_basis": "gazebo_ground_truth_cleaning_footprint_brush_on",
    }


def uncovered_cell_centers(
    polygon, timed_points, width, resolution=0.10, exclusion_polygons=()
):
    """Return cleanable raster cells not reached by the actual brush center."""
    min_x = min(point[0] for point in polygon)
    max_x = max(point[0] for point in polygon)
    min_y = min(point[1] for point in polygon)
    max_y = max(point[1] for point in polygon)
    rows = max(0, int(math.ceil((max_y - min_y) / resolution)))
    columns = max(0, int(math.ceil((max_x - min_x) / resolution)))
    missed = []
    radius = width / 2.0
    for iy in range(rows):
        y = min_y + (iy + 0.5) * resolution
        for ix in range(columns):
            x = min_x + (ix + 0.5) * resolution
            if not point_in_cleanable_area(x, y, polygon, exclusion_polygons):
                continue
            if not any(math.hypot(x - px, y - py) <= radius for _, px, py in timed_points):
                missed.append((x, y))
    return missed


def horizontal_repair_segments(missed_cells, cleanable_polygon, width):
    """Create bounded horizontal repair lines for sparse missed-cell rows."""
    if not missed_cells:
        return []
    min_x = min(point[0] for point in cleanable_polygon)
    max_x = max(point[0] for point in cleanable_polygon)
    rows = {}
    for x, y in missed_cells:
        rows.setdefault(round(y, 6), []).append(x)
    row_groups = []
    for y, xs in sorted(rows.items()):
        if row_groups and y - row_groups[-1][-1][0] <= width * 0.75:
            row_groups[-1].append((y, xs))
        else:
            row_groups.append([(y, xs)])
    segments = []
    reverse = False
    margin = width / 2.0
    for group in row_groups:
        y = sum(item[0] for item in group) / len(group)
        xs = [x for _, values in group for x in values]
        start_x = max(min_x, min(xs) - margin)
        end_x = min(max_x, max(xs) + margin)
        start, end = (start_x, y), (end_x, y)
        if reverse:
            start, end = end, start
        segments.append((start, end))
        reverse = not reverse
    return segments
