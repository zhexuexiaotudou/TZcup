import math


def segment_heading(start, end):
    if start == end:
        raise ValueError('a single or degenerate point cannot define heading')
    return math.atan2(end[1] - start[1], end[0] - start[0])


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
