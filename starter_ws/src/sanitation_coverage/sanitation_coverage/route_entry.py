import math


def apply_lateral_affine(swaths, angle_deg, scale=1.0, offset_m=0.0):
    """Apply an offline map-normal calibration to desired brush swaths."""
    angle = math.radians(float(angle_deg))
    normal = (-math.sin(angle), math.cos(angle))
    calibrated = []
    for start, end in swaths:
        points = []
        for point in (start, end):
            projection = point[0] * normal[0] + point[1] * normal[1]
            correction = float(scale) * projection + float(offset_m) - projection
            points.append((
                point[0] + correction * normal[0],
                point[1] + correction * normal[1],
            ))
        calibrated.append(tuple(points))
    return calibrated


def segment_heading(start, end):
    if start == end:
        raise ValueError('a single or degenerate point cannot define heading')
    return math.atan2(end[1] - start[1], end[0] - start[0])


def brush_center_to_base_swath(start, end, forward_offset_m, extension_m):
    """Convert a desired brush-center swath to the commanded base path."""
    length = math.dist(start, end)
    if length <= 1e-9:
        return start, end
    unit_x = (end[0] - start[0]) / length
    unit_y = (end[1] - start[1]) / length
    return (
        (
            start[0] - (forward_offset_m + extension_m) * unit_x,
            start[1] - (forward_offset_m + extension_m) * unit_y,
        ),
        (
            end[0] + (extension_m - forward_offset_m) * unit_x,
            end[1] + (extension_m - forward_offset_m) * unit_y,
        ),
    )


def oriented_pose(point, heading):
    return {'x': float(point[0]), 'y': float(point[1]), 'yaw': float(heading)}


def transit_pose(current_point, staging):
    """Give a staging goal the explicit approach heading, never implicit yaw 0."""
    return oriented_pose(
        (staging['x'], staging['y']),
        segment_heading(current_point, (staging['x'], staging['y'])),
    )


def entry_points(current_point, first_component, spacing_m=0.05, lead_in_m=0.20):
    """Dense brush-off entry ending on the first swath's explicit heading."""
    swath = first_component['points']
    heading = segment_heading(swath[0], swath[1])
    target = (
        swath[0][0] + lead_in_m * math.cos(heading),
        swath[0][1] + lead_in_m * math.sin(heading),
    )
    distance = math.dist(current_point, swath[0])
    count = max(2, int(math.ceil(distance / spacing_m)) + 1)
    approach = [
        (
            current_point[0] + (swath[0][0] - current_point[0]) * index / (count - 1),
            current_point[1] + (swath[0][1] - current_point[1]) * index / (count - 1),
        )
        for index in range(count)
    ]
    return approach + [target]


def reverse_components(components):
    return [
        {**component, 'points': list(reversed(component['points']))}
        for component in reversed(components)
    ]


def staging_pose(first_component, offset_m):
    points = first_component['points']
    heading = segment_heading(points[0], points[1])
    return oriented_pose(
        (
            points[0][0] - offset_m * math.cos(heading),
            points[0][1] - offset_m * math.sin(heading),
        ),
        heading,
    )


def route_candidates(components, staging_offset_m):
    if not components:
        return []
    forward = components
    reverse = reverse_components(components)
    return [
        {
            'direction': 'forward',
            'components': forward,
            'staging_pose': staging_pose(forward[0], staging_offset_m),
        },
        {
            'direction': 'reverse',
            'components': reverse,
            'staging_pose': staging_pose(reverse[0], staging_offset_m),
        },
    ]
