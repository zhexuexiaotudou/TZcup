import math


def remaining_path_length(position, points):
    if not points:
        return 0.0
    nearest = min(
        range(len(points)), key=lambda index: math.dist(position, points[index])
    )
    return math.dist(position, points[nearest]) + sum(
        math.dist(start, end)
        for start, end in zip(points[nearest:], points[nearest + 1:])
    )


def path_heading(position, points):
    if len(points) < 2:
        raise ValueError('a dynamic trial needs a path with at least two points')
    nearest = min(
        range(len(points) - 1), key=lambda index: math.dist(position, points[index])
    )
    start, end = points[nearest], points[nearest + 1]
    return math.atan2(end[1] - start[1], end[0] - start[0])


def crossing_targets(position, heading, ahead_m, half_width_m, steps):
    if steps < 2:
        raise ValueError('a moving crossing needs at least two poses')
    center_x = position[0] + ahead_m * math.cos(heading)
    center_y = position[1] + ahead_m * math.sin(heading)
    return [
        (
            center_x - lateral * math.sin(heading),
            center_y + lateral * math.cos(heading),
        )
        for lateral in (
            -half_width_m + 2.0 * half_width_m * index / (steps - 1)
            for index in range(steps)
        )
    ]
