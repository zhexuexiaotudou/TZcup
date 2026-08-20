"""Deterministic frontier extraction and map-extent metrics.

The helpers are ROS-independent so the exploration policy can be regression
tested without treating a synthetic map as runtime acceptance evidence.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Iterable, Sequence


UNKNOWN = -1


@dataclass(frozen=True)
class GridGeometry:
    width: int
    height: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    origin_yaw_rad: float = 0.0


@dataclass(frozen=True)
class FrontierGoal:
    grid_x: int
    grid_y: int
    world_x_m: float
    world_y_m: float
    yaw_rad: float
    frontier_cell_count: int
    information_gain_m: float
    distance_m: float
    score: float
    preference_distance_m: float | None = None
    raw_world_x_m: float | None = None
    raw_world_y_m: float | None = None


def reverse_escape_goal(
    robot_xy_m: tuple[float, float],
    robot_yaw_rad: float,
    *,
    distance_m: float,
    allowed_bounds_xyxy_m: tuple[float, float, float, float] | None = None,
    boundary_margin_m: float = 0.0,
) -> FrontierGoal | None:
    """Build a straight reverse goal that preserves the chassis heading."""
    distance = max(0.0, float(distance_m))
    if distance <= 1.0e-9:
        return None
    world_x = robot_xy_m[0] - distance * math.cos(robot_yaw_rad)
    world_y = robot_xy_m[1] - distance * math.sin(robot_yaw_rad)
    if allowed_bounds_xyxy_m is not None:
        min_x, min_y, max_x, max_y = allowed_bounds_xyxy_m
        margin = max(0.0, float(boundary_margin_m))
        if not (
            min_x + margin <= world_x <= max_x - margin
            and min_y + margin <= world_y <= max_y - margin
        ):
            return None
    return FrontierGoal(
        grid_x=-1,
        grid_y=-1,
        world_x_m=world_x,
        world_y_m=world_y,
        yaw_rad=robot_yaw_rad,
        frontier_cell_count=0,
        information_gain_m=0.0,
        distance_m=distance,
        score=-distance,
    )


def frontier_goal_exclusion_centers(
    goal: FrontierGoal,
    geometry: GridGeometry | None,
) -> tuple[tuple[float, float], ...]:
    """Return both the commanded endpoint and its source frontier location.

    Ackermann heading limits can turn a distant raw frontier into a short local
    arc endpoint.  Cooling only that endpoint allows the same unreachable raw
    frontier to be selected again with a slightly different local arc, which
    can produce an indefinitely successful but zero-map-gain loop.
    """
    endpoint = (float(goal.world_x_m), float(goal.world_y_m))
    if goal.raw_world_x_m is not None and goal.raw_world_y_m is not None:
        raw_frontier = (
            float(goal.raw_world_x_m),
            float(goal.raw_world_y_m),
        )
    elif geometry is not None and goal.grid_x >= 0 and goal.grid_y >= 0:
        raw_frontier = grid_to_world(goal.grid_x, goal.grid_y, geometry)
    else:
        return (endpoint,)
    if math.hypot(
        raw_frontier[0] - endpoint[0], raw_frontier[1] - endpoint[1]
    ) <= 1.0e-9:
        return (endpoint,)
    return (endpoint, raw_frontier)


def lane_shift_connector_goals(
    robot_pose: tuple[float, float, float],
    target_y_m: float,
    *,
    candidate_distances_m: Iterable[float],
    allowed_bounds_xyxy_m: tuple[float, float, float, float] | None = None,
    boundary_margin_m: float = 0.0,
) -> list[FrontierGoal]:
    """Build bounds-safe, map-checkable staging poses for a sweep lane shift.

    The poses are derived only from the current fused pose and the active
    bounds-derived sweep direction.  The caller must still reject every pose
    whose online costmap footprint is not traversable before sending it to
    Nav2's collision-checking planner.
    """
    robot_x, robot_y, _ = robot_pose
    delta_y = float(target_y_m) - robot_y
    if abs(delta_y) <= 1.0e-9:
        return []
    direction = 1.0 if delta_y > 0.0 else -1.0
    yaw = math.copysign(math.pi / 2.0, direction)
    goals = []
    seen_distances = set()
    for requested_distance in candidate_distances_m:
        distance = min(abs(delta_y), max(0.0, float(requested_distance)))
        rounded_distance = round(distance, 9)
        if distance <= 1.0e-9 or rounded_distance in seen_distances:
            continue
        seen_distances.add(rounded_distance)
        world_y = robot_y + direction * distance
        if allowed_bounds_xyxy_m is not None:
            min_x, min_y, max_x, max_y = allowed_bounds_xyxy_m
            margin = max(0.0, float(boundary_margin_m))
            if not (
                min_x + margin <= robot_x <= max_x - margin
                and min_y + margin <= world_y <= max_y - margin
            ):
                continue
        goals.append(FrontierGoal(
            grid_x=-1,
            grid_y=-1,
            world_x_m=robot_x,
            world_y_m=world_y,
            yaw_rad=yaw,
            frontier_cell_count=0,
            information_gain_m=0.0,
            distance_m=distance,
            score=-distance,
        ))
    return goals


def sweep_staging_goals(
    robot_pose: tuple[float, float, float],
    target_xy_m: tuple[float, float],
    *,
    candidate_distances_m: Iterable[float],
    allowed_bounds_xyxy_m: tuple[float, float, float, float] | None = None,
    boundary_margin_m: float = 0.0,
) -> list[FrontierGoal]:
    """Build short bounds-derived staging goals toward a sweep anchor.

    The caller must validate every endpoint against the live costmap before
    sending it to Nav2. No world obstacle geometry or ground-truth pose enters
    this construction.
    """
    robot_x, robot_y, _ = robot_pose
    delta_x = float(target_xy_m[0]) - robot_x
    delta_y = float(target_xy_m[1]) - robot_y
    target_distance = math.hypot(delta_x, delta_y)
    if target_distance <= 1.0e-9:
        return []
    heading = math.atan2(delta_y, delta_x)
    goals = []
    seen_distances = set()
    for requested_distance in candidate_distances_m:
        distance = min(target_distance, max(0.0, float(requested_distance)))
        rounded_distance = round(distance, 9)
        if distance <= 1.0e-9 or rounded_distance in seen_distances:
            continue
        seen_distances.add(rounded_distance)
        world_x = robot_x + distance * math.cos(heading)
        world_y = robot_y + distance * math.sin(heading)
        if allowed_bounds_xyxy_m is not None:
            min_x, min_y, max_x, max_y = allowed_bounds_xyxy_m
            margin = max(0.0, float(boundary_margin_m))
            if not (
                min_x + margin <= world_x <= max_x - margin
                and min_y + margin <= world_y <= max_y - margin
            ):
                continue
        goals.append(FrontierGoal(
            grid_x=-1,
            grid_y=-1,
            world_x_m=world_x,
            world_y_m=world_y,
            yaw_rad=heading,
            frontier_cell_count=0,
            information_gain_m=0.0,
            distance_m=distance,
            score=-distance,
        ))
    return goals


def sweep_anchor_is_behind_chassis(
    robot_pose: tuple[float, float, float],
    target_xy_m: tuple[float, float],
) -> bool:
    """Return whether the sweep anchor lies in the chassis rear half-plane."""
    error = sweep_anchor_heading_error_rad(robot_pose, target_xy_m)
    return error is not None and abs(error) > math.pi / 2.0


def sweep_anchor_heading_error_rad(
    robot_pose: tuple[float, float, float],
    target_xy_m: tuple[float, float],
) -> float | None:
    """Return signed shortest heading error from the chassis to an anchor."""
    robot_x, robot_y, robot_yaw = robot_pose
    delta_x = float(target_xy_m[0]) - float(robot_x)
    delta_y = float(target_xy_m[1]) - float(robot_y)
    if math.hypot(delta_x, delta_y) <= 1.0e-9:
        return None
    target_heading = math.atan2(delta_y, delta_x)
    return math.atan2(
        math.sin(target_heading - float(robot_yaw)),
        math.cos(target_heading - float(robot_yaw)),
    )


def straight_staging_path_poses(
    robot_pose: tuple[float, float, float],
    goal: FrontierGoal,
    *,
    sample_spacing_m: float,
) -> list[tuple[float, float, float]]:
    """Sample the complete straight staging corridor for costmap checks."""
    spacing = float(sample_spacing_m)
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("sample_spacing_m must be finite and positive")
    start_x, start_y, _ = robot_pose
    delta_x = float(goal.world_x_m) - float(start_x)
    delta_y = float(goal.world_y_m) - float(start_y)
    distance = math.hypot(delta_x, delta_y)
    if distance <= 1.0e-9:
        return [(float(start_x), float(start_y), float(goal.yaw_rad))]
    sample_count = max(1, int(math.ceil(distance / spacing)))
    return [
        (
            float(start_x) + delta_x * index / sample_count,
            float(start_y) + delta_y * index / sample_count,
            float(goal.yaw_rad),
        )
        for index in range(sample_count + 1)
    ]


def frontier_sweep_targets(
    required_bounds_xyxy_m: tuple[float, float, float, float],
    robot_xy_m: tuple[float, float],
    robot_yaw_rad: float,
    *,
    sensor_range_m: float,
    lane_overlap_m: float,
    boundary_margin_m: float,
) -> list[tuple[float, float]]:
    """Derive a deterministic serpentine mission from bounds and sensor range.

    The returned points are ranking preferences, not navigation goals.  The
    explorer still drives only to frontiers extracted from the live occupancy
    grid.  Keeping a preference until its endpoint is reached prevents the
    largest connected frontier from making an Ackermann vehicle shuttle along
    its current heading forever.
    """
    min_x, min_y, max_x, max_y = (
        float(value) for value in required_bounds_xyxy_m
    )
    if max_x <= min_x or max_y <= min_y:
        raise ValueError("required bounds must have positive area")
    sensor_range = max(0.0, float(sensor_range_m))
    overlap = max(0.0, min(sensor_range, float(lane_overlap_m)))
    margin = max(0.0, float(boundary_margin_m))
    if max_x - min_x <= 2.0 * margin or max_y - min_y <= 2.0 * margin:
        return []

    # These are map-coverage anchors at the required envelope, not chassis
    # waypoints.  The live frontier and sensor range keep the vehicle inside
    # the goal boundary margin while the scan reaches each anchor.
    x0, x1 = min_x, max_x
    y0, y1 = min_y, max_y

    usable_height = y1 - y0
    maximum_lane_spacing = max(1.0e-6, 2.0 * (sensor_range - overlap))
    lane_intervals = max(1, int(math.ceil(usable_height / maximum_lane_spacing)))
    lane_spacing = usable_height / lane_intervals
    lanes = [y0 + index * lane_spacing for index in range(lane_intervals + 1)]
    start_index = min(
        range(len(lanes)), key=lambda index: abs(lanes[index] - robot_xy_m[1])
    )
    # Cover the nearest strip first, continue toward the nearer outer edge,
    # then cross once to finish the remaining half of the mission.
    north_distance = abs(y1 - robot_xy_m[1])
    south_distance = abs(robot_xy_m[1] - y0)
    if north_distance <= south_distance:
        lane_order = list(range(start_index, len(lanes))) + list(
            range(start_index - 1, -1, -1)
        )
    else:
        lane_order = list(range(start_index, -1, -1)) + list(
            range(start_index + 1, len(lanes))
        )

    first_x = x1 if math.cos(robot_yaw_rad) >= 0.0 else x0
    second_x = x0 if first_x == x1 else x1
    targets: list[tuple[float, float]] = []
    for lane_position, lane_index in enumerate(lane_order):
        same_side = first_x if lane_position % 2 == 0 else second_x
        opposite_side = second_x if lane_position % 2 == 0 else first_x
        lane_y = lanes[lane_index]
        targets.extend(((same_side, lane_y), (opposite_side, lane_y)))
    return targets


def frontier_sweep_target_axis(
    targets: Sequence[tuple[float, float]], target_index: int
) -> str:
    """Classify a sweep anchor as a horizontal pass or vertical lane shift."""
    index = int(target_index)
    if not (0 <= index < len(targets)):
        raise IndexError("sweep target index is out of range")
    if index == 0:
        return "horizontal"
    previous = targets[index - 1]
    target = targets[index]
    if abs(target[0] - previous[0]) <= 1.0e-9 and abs(
        target[1] - previous[1]
    ) > 1.0e-9:
        return "vertical"
    return "horizontal"


def vertical_sweep_anchor_reached(
    mapped_envelope_bounds_xyxy_m: Sequence[float] | None,
    *,
    previous_y_m: float,
    target_y_m: float,
    radius_m: float,
) -> bool:
    """Check a lane-shift anchor against the live map's vertical envelope."""
    if mapped_envelope_bounds_xyxy_m is None:
        return False
    if len(mapped_envelope_bounds_xyxy_m) != 4:
        raise ValueError("mapped envelope bounds must contain four values")
    radius = max(0.0, float(radius_m))
    if float(target_y_m) > float(previous_y_m):
        return float(mapped_envelope_bounds_xyxy_m[3]) >= float(target_y_m) - radius
    return float(mapped_envelope_bounds_xyxy_m[1]) <= float(target_y_m) + radius


def prune_timed_exclusions(
    records: Iterable[tuple[float, float, float]],
    *,
    now_monotonic: float,
) -> tuple[list[tuple[float, float, float]], list[tuple[float, float]]]:
    """Drop expired navigation exclusions and return active XY points."""
    active = [
        (float(x), float(y), float(expires))
        for x, y, expires in records
        if float(expires) > float(now_monotonic)
    ]
    return active, [(x, y) for x, y, _ in active]


def next_adaptive_goal_distance(
    current_distance_m: float,
    success_streak: int,
    *,
    succeeded: bool,
    minimum_distance_m: float,
    maximum_distance_m: float,
    successes_per_growth: int,
    growth_step_m: float,
) -> tuple[float, int]:
    """Grow frontier strides after success and contract on any failure."""
    minimum = max(0.0, float(minimum_distance_m))
    maximum = max(minimum, float(maximum_distance_m))
    current = min(maximum, max(minimum, float(current_distance_m)))
    if not succeeded:
        return max(minimum, current * 0.5), 0
    streak = int(success_streak) + 1
    threshold = max(1, int(successes_per_growth))
    if streak < threshold:
        return current, streak
    return min(maximum, current + max(0.0, float(growth_step_m))), 0


def next_no_progress_frontier_state(
    current_streak: int,
    *,
    map_area_before_m2: float,
    map_area_after_m2: float,
    minimum_gain_m2: float,
    successes_before_staging: int,
    successes_before_raw_exclusion: int,
) -> tuple[int, str | None, float]:
    """Classify successful motion by whether it actually expanded the map.

    Nav2 success only proves that the commanded endpoint was reached. It is
    not exploration progress. Repeated short Ackermann arcs can succeed near
    the same raw frontier while the known map area stays constant. The caller
    periodically requests a known-free staging step while preserving the raw
    frontier. A longer streak requests raw-frontier cooling as well.
    """
    before = float(map_area_before_m2)
    after = float(map_area_after_m2)
    minimum_gain = float(minimum_gain_m2)
    if not all(math.isfinite(value) for value in (before, after, minimum_gain)):
        raise ValueError("map areas and minimum gain must be finite")
    if minimum_gain < 0.0:
        raise ValueError("minimum gain must be non-negative")
    staging_limit = int(successes_before_staging)
    raw_limit = int(successes_before_raw_exclusion)
    if staging_limit < 1:
        raise ValueError("successes_before_staging must be positive")
    if raw_limit < staging_limit:
        raise ValueError("raw exclusion limit must be at least staging limit")

    gain = after - before
    if gain + 1.0e-9 >= minimum_gain:
        return 0, None, gain
    streak = max(0, int(current_streak)) + 1
    if streak >= raw_limit:
        return 0, "raw_and_staging", gain
    if streak % staging_limit == 0:
        return streak, "staging", gain
    return streak, None, gain


def no_progress_recovery_action_for_sweep(
    recovery_action: str | None,
    *,
    sweep_axis: str | None,
) -> str | None:
    """Preserve a reachable frontier while traversing a known sweep lane.

    A horizontal bounds-derived sweep necessarily crosses already observed
    space on its way to the opposite anchor. Successful low-gain motion there
    is expected transit, not proof that the raw frontier is stale. Cooling that
    frontier can leave no ranked goal and trigger a reverse escape that exactly
    cancels the forward progress. Navigation failures still use the independent
    failed-goal exclusion path; only successful low-gain raw exclusion is
    suppressed while a horizontal sweep anchor is active.
    """
    if recovery_action == "raw_and_staging" and sweep_axis == "horizontal":
        return "staging"
    return recovery_action


def _index(x: int, y: int, geometry: GridGeometry) -> int:
    return y * geometry.width + x


def _neighbors4(x: int, y: int, geometry: GridGeometry):
    for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        nx, ny = x + dx, y + dy
        if 0 <= nx < geometry.width and 0 <= ny < geometry.height:
            yield nx, ny


def _neighbors8(x: int, y: int, geometry: GridGeometry):
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nx, ny = x + dx, y + dy
            if 0 <= nx < geometry.width and 0 <= ny < geometry.height:
                yield nx, ny


def grid_to_world(x: int, y: int, geometry: GridGeometry) -> tuple[float, float]:
    local_x = (x + 0.5) * geometry.resolution_m
    local_y = (y + 0.5) * geometry.resolution_m
    cosine = math.cos(geometry.origin_yaw_rad)
    sine = math.sin(geometry.origin_yaw_rad)
    return (
        geometry.origin_x_m + cosine * local_x - sine * local_y,
        geometry.origin_y_m + sine * local_x + cosine * local_y,
    )


def world_disk_is_traversable(
    data: Sequence[int],
    geometry: GridGeometry,
    world_xy_m: tuple[float, float],
    *,
    radius_m: float,
    maximum_cost: int,
) -> bool:
    """Check a world-frame disk against a nav occupancy/cost grid."""
    if len(data) != geometry.width * geometry.height:
        raise ValueError("grid data length does not match geometry")
    dx = world_xy_m[0] - geometry.origin_x_m
    dy = world_xy_m[1] - geometry.origin_y_m
    cosine = math.cos(geometry.origin_yaw_rad)
    sine = math.sin(geometry.origin_yaw_rad)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    center_x = int(math.floor(local_x / geometry.resolution_m))
    center_y = int(math.floor(local_y / geometry.resolution_m))
    radius_cells = int(math.ceil(max(0.0, radius_m) / geometry.resolution_m))
    for offset_y in range(-radius_cells, radius_cells + 1):
        for offset_x in range(-radius_cells, radius_cells + 1):
            if math.hypot(offset_x, offset_y) * geometry.resolution_m > radius_m:
                continue
            x = center_x + offset_x
            y = center_y + offset_y
            if not (0 <= x < geometry.width and 0 <= y < geometry.height):
                return False
            value = int(data[_index(x, y, geometry)])
            if value < 0 or value > int(maximum_cost):
                return False
    return True


def world_disk_has_known_cell(
    data: Sequence[int],
    geometry: GridGeometry,
    world_xy_m: tuple[float, float],
    *,
    radius_m: float,
) -> bool:
    """Return whether live mapping has observed any cell near a world point."""
    if len(data) != geometry.width * geometry.height:
        raise ValueError("grid data length does not match geometry")
    dx = world_xy_m[0] - geometry.origin_x_m
    dy = world_xy_m[1] - geometry.origin_y_m
    cosine = math.cos(geometry.origin_yaw_rad)
    sine = math.sin(geometry.origin_yaw_rad)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    center_x = int(math.floor(local_x / geometry.resolution_m))
    center_y = int(math.floor(local_y / geometry.resolution_m))
    radius_cells = int(math.ceil(max(0.0, radius_m) / geometry.resolution_m))
    for offset_y in range(-radius_cells, radius_cells + 1):
        for offset_x in range(-radius_cells, radius_cells + 1):
            if math.hypot(offset_x, offset_y) * geometry.resolution_m > radius_m:
                continue
            x = center_x + offset_x
            y = center_y + offset_y
            if not (0 <= x < geometry.width and 0 <= y < geometry.height):
                continue
            if int(data[_index(x, y, geometry)]) != UNKNOWN:
                return True
    return False


def frontier_clusters(
    data: Sequence[int],
    geometry: GridGeometry,
    *,
    free_max: int = 25,
    minimum_cells: int = 5,
    connection_radius_cells: int = 1,
) -> list[list[tuple[int, int]]]:
    """Return clustered free cells adjacent to unknown space.

    A multi-cell connection radius joins angular lidar samples separated by a
    small raster gap without changing which cells qualify as frontiers.
    """
    expected = geometry.width * geometry.height
    if len(data) != expected:
        raise ValueError(f"grid data length {len(data)} != {expected}")
    frontier = set()
    for y in range(geometry.height):
        for x in range(geometry.width):
            value = int(data[_index(x, y, geometry)])
            if 0 <= value <= free_max and any(
                int(data[_index(nx, ny, geometry)]) == UNKNOWN
                for nx, ny in _neighbors4(x, y, geometry)
            ):
                frontier.add((x, y))

    clusters = []
    while frontier:
        start = frontier.pop()
        queue = deque([start])
        cluster = [start]
        while queue:
            x, y = queue.popleft()
            radius = max(1, int(connection_radius_cells))
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if dx == 0 and dy == 0:
                        continue
                    neighbor = (x + dx, y + dy)
                    if neighbor in frontier:
                        frontier.remove(neighbor)
                        queue.append(neighbor)
                        cluster.append(neighbor)
        if len(cluster) >= minimum_cells:
            clusters.append(sorted(cluster, key=lambda cell: (cell[1], cell[0])))
    return sorted(clusters, key=lambda cells: (-len(cells), cells[0]))


def _frontier_heading(
    cluster: Sequence[tuple[int, int]],
    data: Sequence[int],
    geometry: GridGeometry,
) -> float:
    dx_total = 0.0
    dy_total = 0.0
    for x, y in cluster:
        for nx, ny in _neighbors4(x, y, geometry):
            if int(data[_index(nx, ny, geometry)]) == UNKNOWN:
                dx_total += nx - x
                dy_total += ny - y
    if abs(dx_total) + abs(dy_total) <= 1e-9:
        return geometry.origin_yaw_rad
    return geometry.origin_yaw_rad + math.atan2(dy_total, dx_total)


def _ackermann_arc_endpoint(
    robot_xy_m: tuple[float, float],
    robot_yaw_rad: float,
    desired_heading_rad: float,
    *,
    maximum_heading_change_rad: float,
    minimum_turning_radius_m: float,
    minimum_goal_distance_m: float,
) -> tuple[float, float, float]:
    """Return a finite-radius local turn toward an off-heading frontier."""
    delta = math.atan2(
        math.sin(desired_heading_rad - robot_yaw_rad),
        math.cos(desired_heading_rad - robot_yaw_rad),
    )
    limit = max(0.0, float(maximum_heading_change_rad))
    delta = max(-limit, min(limit, delta))
    if abs(delta) <= 1.0e-9:
        return robot_xy_m[0], robot_xy_m[1], robot_yaw_rad
    radius = max(1.0e-6, float(minimum_turning_radius_m))
    turn_sign = 1.0 if delta > 0.0 else -1.0
    goal_yaw = robot_yaw_rad + delta
    world_x = robot_xy_m[0] + turn_sign * radius * (
        math.sin(goal_yaw) - math.sin(robot_yaw_rad)
    )
    world_y = robot_xy_m[1] - turn_sign * radius * (
        math.cos(goal_yaw) - math.cos(robot_yaw_rad)
    )
    # If a caller requires a longer progress step, extend on the tangent after
    # completing the minimum-radius arc. This remains forward-drivable.
    required = max(0.0, float(minimum_goal_distance_m))
    distance = math.hypot(world_x - robot_xy_m[0], world_y - robot_xy_m[1])
    if distance + 1.0e-9 < required:
        relative_x = world_x - robot_xy_m[0]
        relative_y = world_y - robot_xy_m[1]
        tangent_x = math.cos(goal_yaw)
        tangent_y = math.sin(goal_yaw)
        projection = relative_x * tangent_x + relative_y * tangent_y
        discriminant = max(
            0.0,
            projection * projection + required * required - distance * distance,
        )
        extension = max(0.0, -projection + math.sqrt(discriminant))
        world_x += extension * tangent_x
        world_y += extension * tangent_y
    return world_x, world_y, goal_yaw


def sweep_alignment_goal(
    robot_pose: tuple[float, float, float],
    target_xy_m: tuple[float, float],
    *,
    maximum_heading_change_rad: float,
    minimum_turning_radius_m: float,
    minimum_goal_distance_m: float,
    allowed_bounds_xyxy_m: tuple[float, float, float, float] | None = None,
    boundary_margin_m: float = 0.0,
) -> FrontierGoal | None:
    """Build one forward Ackermann arc that rotates toward a sweep anchor."""
    robot_x, robot_y, robot_yaw = robot_pose
    delta_x = float(target_xy_m[0]) - robot_x
    delta_y = float(target_xy_m[1]) - robot_y
    if math.hypot(delta_x, delta_y) <= 1.0e-9:
        return None
    desired_heading = math.atan2(delta_y, delta_x)
    world_x, world_y, goal_yaw = _ackermann_arc_endpoint(
        (robot_x, robot_y),
        robot_yaw,
        desired_heading,
        maximum_heading_change_rad=maximum_heading_change_rad,
        minimum_turning_radius_m=minimum_turning_radius_m,
        minimum_goal_distance_m=minimum_goal_distance_m,
    )
    distance = math.hypot(world_x - robot_x, world_y - robot_y)
    if distance <= 1.0e-9:
        return None
    if allowed_bounds_xyxy_m is not None:
        min_x, min_y, max_x, max_y = allowed_bounds_xyxy_m
        margin = max(0.0, float(boundary_margin_m))
        if not (
            min_x + margin <= world_x <= max_x - margin
            and min_y + margin <= world_y <= max_y - margin
        ):
            return None
    return FrontierGoal(
        grid_x=-1,
        grid_y=-1,
        world_x_m=world_x,
        world_y_m=world_y,
        yaw_rad=goal_yaw,
        frontier_cell_count=0,
        information_gain_m=0.0,
        distance_m=distance,
        score=-distance,
    )


def rank_frontiers(
    data: Sequence[int],
    geometry: GridGeometry,
    robot_xy_m: tuple[float, float],
    *,
    robot_yaw_rad: float | None = None,
    excluded_world_xy: Iterable[tuple[float, float]] = (),
    exclusion_radius_m: float = 2.0,
    minimum_goal_distance_m: float = 1.0,
    minimum_cells: int = 5,
    connection_radius_cells: int = 1,
    distance_weight: float = 0.25,
    allowed_bounds_xyxy_m: tuple[float, float, float, float] | None = None,
    boundary_margin_m: float = 0.0,
    goal_backoff_m: float = 0.0,
    maximum_goal_distance_m: float | None = None,
    maximum_goal_yaw_change_rad: float | None = None,
    minimum_goal_arc_yaw_change_rad: float = 0.15,
    minimum_turning_radius_m: float = 1.429,
    boundary_turn_buffer_m: float = 0.0,
    preferred_world_xy: tuple[float, float] | None = None,
) -> list[FrontierGoal]:
    goals = []
    excluded = list(excluded_world_xy)
    for cluster in frontier_clusters(
        data,
        geometry,
        minimum_cells=minimum_cells,
        connection_radius_cells=connection_radius_cells,
    ):
        cells_with_distance = []
        for cell in cluster:
            cell_world = grid_to_world(cell[0], cell[1], geometry)
            cells_with_distance.append((
                cell,
                math.hypot(
                    cell_world[0] - robot_xy_m[0],
                    cell_world[1] - robot_xy_m[1],
                ),
            ))
        def candidate_order(row):
            cell, raw_distance = row
            point = grid_to_world(cell[0], cell[1], geometry)
            heading = math.atan2(
                point[1] - robot_xy_m[1], point[0] - robot_xy_m[0]
            )
            heading_error = 0.0
            if robot_yaw_rad is not None:
                heading_error = abs(math.atan2(
                    math.sin(heading - robot_yaw_rad),
                    math.cos(heading - robot_yaw_rad),
                ))
            preference_distance = 0.0
            if preferred_world_xy is not None:
                preference_distance = math.hypot(
                    point[0] - preferred_world_xy[0],
                    point[1] - preferred_world_xy[1],
                )
            return (
                preference_distance,
                heading_error,
                -raw_distance,
                cell[1],
                cell[0],
            )

        selected = None
        for (grid_x, grid_y), raw_distance in sorted(
            cells_with_distance, key=candidate_order
        ):
            world_x, world_y = grid_to_world(grid_x, grid_y, geometry)
            backoff = min(
                max(0.0, float(goal_backoff_m)),
                max(0.0, raw_distance - float(minimum_goal_distance_m)),
            )
            if raw_distance > 1.0e-9 and backoff > 0.0:
                scale = (raw_distance - backoff) / raw_distance
                world_x = robot_xy_m[0] + scale * (world_x - robot_xy_m[0])
                world_y = robot_xy_m[1] + scale * (world_y - robot_xy_m[1])
            distance = math.hypot(
                world_x - robot_xy_m[0], world_y - robot_xy_m[1]
            )
            if (
                maximum_goal_distance_m is not None
                and distance > float(maximum_goal_distance_m)
            ):
                scale = float(maximum_goal_distance_m) / distance
                world_x = robot_xy_m[0] + scale * (world_x - robot_xy_m[0])
                world_y = robot_xy_m[1] + scale * (world_y - robot_xy_m[1])
                distance = float(maximum_goal_distance_m)
            goal_yaw = math.atan2(
                world_y - robot_xy_m[1], world_x - robot_xy_m[0]
            )
            desired_goal_yaw = goal_yaw
            synthesized_turn = False
            if robot_yaw_rad is not None and maximum_goal_yaw_change_rad is not None:
                yaw_delta = math.atan2(
                    math.sin(goal_yaw - robot_yaw_rad),
                    math.cos(goal_yaw - robot_yaw_rad),
                )
                yaw_limit = max(0.0, float(maximum_goal_yaw_change_rad))
                # A direct chord is only a physically honest Ackermann goal
                # while it is nearly aligned with the current chassis.  For
                # a material but still sub-limit heading change, synthesize
                # the same minimum-radius forward arc used for larger turns;
                # otherwise Hybrid-A* must invent a local maneuver and can
                # saturate the steering plant at the target-envelope edge.
                arc_threshold = max(
                    0.0, float(minimum_goal_arc_yaw_change_rad)
                )
                if abs(yaw_delta) > arc_threshold + 1.0e-9:
                    synthesized_turn = True
                    world_x, world_y, goal_yaw = _ackermann_arc_endpoint(
                        robot_xy_m,
                        robot_yaw_rad,
                        goal_yaw,
                        maximum_heading_change_rad=yaw_limit,
                        minimum_turning_radius_m=minimum_turning_radius_m,
                        minimum_goal_distance_m=minimum_goal_distance_m,
                    )
                    distance = math.hypot(
                        world_x - robot_xy_m[0], world_y - robot_xy_m[1]
                    )
            if allowed_bounds_xyxy_m is not None:
                min_x, min_y, max_x, max_y = allowed_bounds_xyxy_m
                margin = max(0.0, float(boundary_margin_m))
                if not (
                    min_x + margin <= world_x <= max_x - margin
                    and min_y + margin <= world_y <= max_y - margin
                ):
                    continue
                # Do not drive a forward segment all the way to the goal
                # envelope edge. Reserve one turning radius so the next
                # frontier can be reached without leaving the safety margin.
                if robot_yaw_rad is not None and not synthesized_turn:
                    turn_buffer = max(0.0, float(boundary_turn_buffer_m))
                    heading_x = math.cos(robot_yaw_rad)
                    heading_y = math.sin(robot_yaw_rad)
                    boundary_buffer_hit = (
                        (heading_x > 1.0e-6 and world_x > max_x - margin - turn_buffer)
                        or (heading_x < -1.0e-6 and world_x < min_x + margin + turn_buffer)
                        or (heading_y > 1.0e-6 and world_y > max_y - margin - turn_buffer)
                        or (heading_y < -1.0e-6 and world_y < min_y + margin + turn_buffer)
                    )
                    if boundary_buffer_hit and maximum_goal_yaw_change_rad is not None:
                        arc_x, arc_y, arc_yaw = _ackermann_arc_endpoint(
                            robot_xy_m,
                            robot_yaw_rad,
                            desired_goal_yaw,
                            maximum_heading_change_rad=float(
                                maximum_goal_yaw_change_rad
                            ),
                            minimum_turning_radius_m=minimum_turning_radius_m,
                            minimum_goal_distance_m=min(
                                0.40, float(minimum_goal_distance_m)
                            ),
                        )
                        arc_distance = math.hypot(
                            arc_x - robot_xy_m[0], arc_y - robot_xy_m[1]
                        )
                        if (
                            arc_distance >= 0.25
                            and min_x + margin <= arc_x <= max_x - margin
                            and min_y + margin <= arc_y <= max_y - margin
                        ):
                            world_x, world_y, goal_yaw = arc_x, arc_y, arc_yaw
                            distance = arc_distance
                            synthesized_turn = True
                        else:
                            continue
                    elif boundary_buffer_hit:
                        continue
            if distance + 1.0e-9 < minimum_goal_distance_m:
                continue
            raw_world = grid_to_world(grid_x, grid_y, geometry)
            if any(
                math.hypot(world_x - x, world_y - y) <= exclusion_radius_m
                or math.hypot(raw_world[0] - x, raw_world[1] - y)
                <= exclusion_radius_m
                for x, y in excluded
            ):
                continue
            preference_distance = None
            if preferred_world_xy is not None:
                preference_distance = math.hypot(
                    raw_world[0] - preferred_world_xy[0],
                    raw_world[1] - preferred_world_xy[1],
                )
            selected = (
                grid_x,
                grid_y,
                world_x,
                world_y,
                distance,
                goal_yaw,
                preference_distance,
                raw_world,
            )
            break
        if selected is None:
            continue
        (
            grid_x,
            grid_y,
            world_x,
            world_y,
            distance,
            goal_yaw,
            preference_distance,
            raw_world,
        ) = selected
        gain = len(cluster) * geometry.resolution_m
        goals.append(
            FrontierGoal(
                grid_x=grid_x,
                grid_y=grid_y,
                world_x_m=world_x,
                world_y_m=world_y,
                # Face from known free space toward the frontier. This remains
                # well-defined when sparse radial rays make the cluster's
                # unknown-neighbor vectors cancel at its arithmetic centroid.
                yaw_rad=goal_yaw,
                frontier_cell_count=len(cluster),
                information_gain_m=gain,
                distance_m=distance,
                score=gain - distance_weight * distance,
                preference_distance_m=preference_distance,
                raw_world_x_m=raw_world[0],
                raw_world_y_m=raw_world[1],
            )
        )
    if preferred_world_xy is not None:
        return sorted(
            goals,
            key=lambda goal: (
                float(goal.preference_distance_m or 0.0),
                -goal.score,
                -goal.frontier_cell_count,
                goal.distance_m,
            ),
        )
    return sorted(
        goals,
        key=lambda goal: (-goal.score, -goal.frontier_cell_count, goal.distance_m),
    )


def map_extent_metrics(
    data: Sequence[int],
    geometry: GridGeometry,
    *,
    required_bounds_xyxy_m: tuple[float, float, float, float] | None = None,
) -> dict:
    """Measure known cells and the continuous known-cell bounding envelope."""
    if len(data) != geometry.width * geometry.height:
        raise ValueError("grid data length does not match geometry")
    known_count = 0
    min_x = geometry.width
    max_x = -1
    min_y = geometry.height
    max_y = -1
    required_known_count = 0
    cell_area = geometry.resolution_m**2
    required_area = None
    required_bounds = required_bounds_xyxy_m
    if required_bounds is not None:
        rx0, ry0, rx1, ry1 = required_bounds
        if rx1 <= rx0 or ry1 <= ry0:
            raise ValueError("required bounds must have positive area")
        required_area = (rx1 - rx0) * (ry1 - ry0)
    for index, value in enumerate(data):
        if int(value) == UNKNOWN:
            continue
        x = index % geometry.width
        y = index // geometry.width
        known_count += 1
        min_x = min(min_x, x)
        max_x = max(max_x, x)
        min_y = min(min_y, y)
        max_y = max(max_y, y)
        if required_bounds is not None:
            world_x, world_y = grid_to_world(x, y, geometry)
            if rx0 <= world_x <= rx1 and ry0 <= world_y <= ry1:
                required_known_count += 1
    if known_count == 0:
        return {
            "known_cell_count": 0,
            "known_area_m2": 0.0,
            "mapped_envelope_area_m2": 0.0,
            "mapped_envelope_bounds_xyxy_m": None,
            "required_bounds_area_m2": None,
            "required_bounds_envelope_coverage_ratio": None,
            "required_bounds_known_area_m2": None,
            "required_bounds_known_coverage_ratio": None,
        }
    corners = [
        grid_to_world(x, y, geometry)
        for x, y in (
            (min_x, min_y), (max_x, min_y),
            (min_x, max_y), (max_x, max_y),
        )
    ]
    half = geometry.resolution_m / 2.0
    envelope = (
        min(point[0] for point in corners) - half,
        min(point[1] for point in corners) - half,
        max(point[0] for point in corners) + half,
        max(point[1] for point in corners) + half,
    )
    envelope_area = (envelope[2] - envelope[0]) * (envelope[3] - envelope[1])
    envelope_coverage = None
    required_known_area = None
    required_known_coverage = None
    if required_bounds_xyxy_m is not None:
        rx0, ry0, rx1, ry1 = required_bounds_xyxy_m
        intersection = max(0.0, min(envelope[2], rx1) - max(envelope[0], rx0)) * max(
            0.0, min(envelope[3], ry1) - max(envelope[1], ry0)
        )
        envelope_coverage = intersection / required_area
        required_known_area = min(required_area, required_known_count * cell_area)
        required_known_coverage = required_known_area / required_area
    return {
        "known_cell_count": known_count,
        "known_area_m2": known_count * cell_area,
        "mapped_envelope_area_m2": envelope_area,
        "mapped_envelope_bounds_xyxy_m": list(envelope),
        "required_bounds_area_m2": required_area,
        "required_bounds_envelope_coverage_ratio": envelope_coverage,
        "required_bounds_known_area_m2": required_known_area,
        "required_bounds_known_coverage_ratio": required_known_coverage,
    }


def mapping_completion_reached(
    metrics: dict,
    *,
    required_envelope_coverage_ratio: float = 1.0,
) -> bool:
    """Require continuous boundary reach plus enough actually known area."""
    envelope = metrics.get("required_bounds_envelope_coverage_ratio")
    required_area = metrics.get("required_bounds_area_m2")
    known_area = metrics.get("known_area_m2")
    if envelope is None or required_area is None or known_area is None:
        return False
    return (
        float(envelope) >= float(required_envelope_coverage_ratio)
        and float(known_area) >= float(required_area)
    )
