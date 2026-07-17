import math


def polygon_area(polygon):
    return abs(sum(
        x1 * y2 - x2 * y1
        for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1])
    )) / 2.0


def _point_in_polygon(x, y, polygon):
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if ((y1 > y) != (y2 > y)) and (
            x < (x2 - x1) * (y - y1) / ((y2 - y1) or 1e-12) + x1
        ):
            inside = not inside
        previous = current
    return inside


def exclusion_clearance(point, exclusions):
    """Distance to footprint-inflated exclusions; zero means invalid staging."""
    if any(_point_in_polygon(point[0], point[1], polygon) for polygon in exclusions):
        return 0.0
    distances = []
    for polygon in exclusions:
        for start, end in zip(polygon, polygon[1:] + polygon[:1]):
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            scale = dx * dx + dy * dy
            projection = 0.0 if scale == 0 else (
                (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
            ) / scale
            projection = min(1.0, max(0.0, projection))
            distances.append(math.hypot(
                point[0] - start[0] - projection * dx,
                point[1] - start[1] - projection * dy,
            ))
    return min(distances) if distances else None


def target_grid_viable(values, lethal_threshold=99):
    """Fail closed unless costmap and keepout mask confirm a safe target."""
    cost = values.get('global_costmap_cost')
    keepout = values.get('keepout_mask_value')
    return bool(
        values.get('global_costmap_received')
        and values.get('keepout_mask_received')
        and cost is not None
        and int(cost) < int(lethal_threshold)
        and keepout is not None
        and int(keepout) == 0
    )


def rectangle_polygon(center, size, margin):
    half_x = float(size[0]) / 2.0 + margin
    half_y = float(size[1]) / 2.0 + margin
    x, y = map(float, center)
    return [
        (x - half_x, y - half_y),
        (x + half_x, y - half_y),
        (x + half_x, y + half_y),
        (x - half_x, y + half_y),
    ]


def _bounds(polygon):
    return (
        min(point[0] for point in polygon),
        min(point[1] for point in polygon),
        max(point[0] for point in polygon),
        max(point[1] for point in polygon),
    )


def _clip_rectangle(polygon, outer):
    x1, y1, x2, y2 = _bounds(polygon)
    ox1, oy1, ox2, oy2 = _bounds(outer)
    clipped = (max(x1, ox1), max(y1, oy1), min(x2, ox2), min(y2, oy2))
    if clipped[0] >= clipped[2] or clipped[1] >= clipped[3]:
        return None
    x1, y1, x2, y2 = clipped
    return [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]


def _inset_rectangle(polygon, inset):
    x1, y1, x2, y2 = _bounds(polygon)
    if x1 + inset >= x2 - inset or y1 + inset >= y2 - inset:
        raise ValueError('headland removes the complete operation polygon')
    return [
        (x1 + inset, y1 + inset),
        (x2 - inset, y1 + inset),
        (x2 - inset, y2 - inset),
        (x1 + inset, y2 - inset),
    ]


def _expand_rectangle(polygon, margin):
    x1, y1, x2, y2 = _bounds(polygon)
    return [
        (x1 - margin, y1 - margin),
        (x2 + margin, y1 - margin),
        (x2 + margin, y2 + margin),
        (x1 - margin, y2 + margin),
    ]


def _overlap(first, second):
    return not (
        first[2] < second[0] or second[2] < first[0]
        or first[3] < second[1] or second[3] < first[1]
    )


def merge_axis_aligned_exclusions(polygons):
    bounds = [_bounds(polygon) for polygon in polygons]
    changed = True
    while changed:
        changed = False
        merged = []
        while bounds:
            current = bounds.pop(0)
            index = next(
                (i for i, item in enumerate(bounds) if _overlap(current, item)),
                None,
            )
            if index is None:
                merged.append(current)
                continue
            other = bounds.pop(index)
            bounds.insert(0, (
                min(current[0], other[0]), min(current[1], other[1]),
                max(current[2], other[2]), max(current[3], other[3]),
            ))
            changed = True
        bounds = merged
    return [
        [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
        for x1, y1, x2, y2 in bounds
    ]


def compile_mission_geometry(config):
    outer = [tuple(map(float, point)) for point in config['outer_polygon']]
    exclusions = [
        [tuple(map(float, point)) for point in polygon]
        for polygon in config.get('exclusion_polygons', [])
    ]
    exclusions.extend(
        [tuple(map(float, point)) for point in polygon]
        for polygon in config.get('keepout_polygons', [])
    )
    footprint = [tuple(map(float, point)) for point in config['robot_footprint']]
    footprint_radius = max(math.hypot(x, y) for x, y in footprint)
    safety_margin = float(config.get('safety_margin_m', 0.0))
    operation_width = float(config.get('operation_width_m', 0.0))
    configured_headland = float(config.get('headland', {}).get('width_m', 0.0))
    required_headland = footprint_radius + safety_margin + operation_width / 2.0
    margin = safety_margin + footprint_radius
    translation = tuple(map(float, config.get('world_to_map_translation', [0.0, 0.0])))
    compiled_obstacles = []
    ignored_obstacles = []
    for obstacle in config.get('static_obstacles', []):
        center = tuple(map(float, obstacle['center']))
        source_frame = obstacle.get('frame_id', 'map')
        if source_frame == 'world':
            center = (center[0] + translation[0], center[1] + translation[1])
        polygon = _clip_rectangle(
            rectangle_polygon(center, obstacle['size'], margin), outer
        )
        record = {
            'name': obstacle.get('name'),
            'source_frame': source_frame,
            'map_center': center,
            'size': tuple(map(float, obstacle['size'])),
        }
        if polygon is None:
            ignored_obstacles.append({
                **record, 'reason': 'inflated_bounds_outside_outer_polygon'
            })
        else:
            exclusions.append(polygon)
            compiled_obstacles.append({**record, 'inflated_polygon': polygon})
    exclusions = merge_axis_aligned_exclusions(exclusions)
    base_excluded_area = sum(polygon_area(polygon) for polygon in exclusions)
    cleanable_outer = _inset_rectangle(outer, configured_headland)
    cleanable_exclusions = merge_axis_aligned_exclusions([
        clipped
        for polygon in exclusions
        if (clipped := _clip_rectangle(
            _expand_rectangle(polygon, configured_headland), cleanable_outer
        )) is not None
    ])
    cleanable_area = max(
        0.0,
        polygon_area(cleanable_outer)
        - sum(polygon_area(polygon) for polygon in cleanable_exclusions),
    )
    excluded_area = polygon_area(outer) - cleanable_area
    return {
        'outer_polygon': outer,
        'exclusion_polygons': exclusions,
        'cleanable_outer_polygon': cleanable_outer,
        'cleanable_exclusion_polygons': cleanable_exclusions,
        'outer_area_m2': polygon_area(outer),
        'base_excluded_area_m2': base_excluded_area,
        'excluded_area_m2': excluded_area,
        'cleanable_area_m2': cleanable_area,
        'footprint_radius_m': footprint_radius,
        'safety_margin_m': safety_margin,
        'configured_headland_width_m': configured_headland,
        'required_headland_width_m': required_headland,
        'headland_clearance_valid': configured_headland >= required_headland,
        'world_to_map_translation': translation,
        'compiled_static_obstacles': compiled_obstacles,
        'ignored_static_obstacles': ignored_obstacles,
    }
