"""Ackermann connector planning: forward arcs, cusps and deferred swaths.

The Ackermann chassis cannot rotate in place, so connectors are forward or
reverse arcs executed through Nav2 FollowPath with the RPP controller.  The
planner order is:

1. collision-checked forward Dubins path,
2. forward U-turn (CSC with net 180 degree heading change),
3. forward teardrop (CSC with partial heading change),
4. bounded Reeds-Shepp-like three-point lattice search (may include reverse
   sections, each direction change split by an explicit CUSP_STOP),
5. Hybrid/Smac connector request (executed at runtime through Nav2), and
6. deferring the target swath (recorded, never cheated).

Every generated segment samples the honest footprint, enforces
abs(curvature) <= 1/R_min, stays inside the outer turning apron and outside
the keepouts, and records SE(2) poses/headings, direction, curvature,
steering reference, speed profile, brush-off state, collision-check result,
source/target swath ids and connector classification.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

from .ackermann_model import (
    WHEELBASE_M,
    minimum_radius_m,
    honest_footprint_polygon,
)
from .coverage_components import ComponentType, CoverageComponent


Pose = tuple[float, float, float]  # x, y, yaw
TRACKABLE_FORWARD_CONNECTOR_RADIUS_M = 1.8


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _point_in_polygon(point, polygon) -> bool:
    x, y = point
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (
            (y2 - y1) or 1e-12
        ) + x1:
            inside = not inside
        previous = current
    return inside


def _footprint_corners(pose: Pose) -> list[tuple[float, float]]:
    x, y, yaw = pose
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return [
        (
            x + px * cosine - py * sine,
            y + px * sine + py * cosine,
        )
        for px, py in honest_footprint_polygon()
    ]


def pose_feasible(
    pose: Pose,
    apron: list[tuple[float, float]],
    keepouts: list[list[tuple[float, float]]],
) -> bool:
    corners = _footprint_corners(pose)
    if not all(_point_in_polygon(corner, apron) for corner in corners):
        return False
    for keepout in keepouts:
        if any(_point_in_polygon(corner, keepout) for corner in corners):
            return False
    return True


def _arc_offset(start_pose: Pose, radius_m: float, turn_angle: float) -> Pose:
    """Pose after driving a circular arc with the given signed heading change.

    ``radius_m`` is the positive arc radius; ``turn_angle`` is signed
    (positive = left turn, negative = right turn) in the standard
    counterclockwise-positive map convention.
    """
    x, y, yaw = start_pose
    side = 1.0 if turn_angle >= 0.0 else -1.0
    radius = abs(radius_m)
    center_x = x - side * radius * math.sin(yaw)
    center_y = y + side * radius * math.cos(yaw)
    start_angle = yaw - side * math.pi / 2.0
    final_angle = start_angle + turn_angle
    end_x = center_x + radius * math.cos(final_angle)
    end_y = center_y + radius * math.sin(final_angle)
    return (end_x, end_y, normalize_angle(yaw + turn_angle))


def _sample_arc(
    start_pose: Pose,
    radius: float,
    signed_angle: float,
    step_rad: float = math.radians(4.0),
) -> list[Pose]:
    samples = []
    remaining = abs(signed_angle)
    travelled = 0.0
    current = start_pose
    while remaining > 1e-6:
        step = min(step_rad, remaining)
        current = _arc_offset(current, radius, math.copysign(step, signed_angle))
        samples.append(current)
        travelled += step
        remaining -= step
    return samples


def _sample_segment(
    start: Pose, end: Pose, spacing_m: float = 0.10
) -> list[Pose]:
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    count = max(1, int(math.ceil(length / spacing_m)))
    return [
        (
            start[0] + (end[0] - start[0]) * i / count,
            start[1] + (end[1] - start[1]) * i / count,
            end[2],
        )
        for i in range(1, count + 1)
    ]


def _mod2pi(angle: float) -> float:
    return angle % (2.0 * math.pi)


def _dubins_words(alpha: float, beta: float, distance: float):
    """Return normalized (word, t, p, q) Dubins candidates."""
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    candidates = []

    def add(word, t, p, q):
        if all(math.isfinite(value) for value in (t, p, q)):
            candidates.append((word, _mod2pi(t), p, _mod2pi(q)))

    p2 = 2.0 + distance * distance - 2.0 * cab + 2.0 * distance * (sa - sb)
    if p2 >= 0.0:
        tmp = math.atan2(cb - ca, distance + sa - sb)
        add("LSL", -alpha + tmp, math.sqrt(p2), beta - tmp)

    p2 = 2.0 + distance * distance - 2.0 * cab + 2.0 * distance * (sb - sa)
    if p2 >= 0.0:
        tmp = math.atan2(ca - cb, distance - sa + sb)
        add("RSR", alpha - tmp, math.sqrt(p2), -beta + tmp)

    p2 = -2.0 + distance * distance + 2.0 * cab + 2.0 * distance * (sa + sb)
    if p2 >= 0.0:
        p = math.sqrt(p2)
        tmp = math.atan2(-ca - cb, distance + sa + sb) - math.atan2(-2.0, p)
        add("LSR", -alpha + tmp, p, -beta + tmp)

    p2 = distance * distance - 2.0 + 2.0 * cab - 2.0 * distance * (sa + sb)
    if p2 >= 0.0:
        p = math.sqrt(p2)
        tmp = math.atan2(ca + cb, distance - sa - sb) - math.atan2(2.0, p)
        add("RSL", alpha - tmp, p, beta - tmp)

    tmp = (6.0 - distance * distance + 2.0 * cab + 2.0 * distance * (sa - sb)) / 8.0
    if abs(tmp) <= 1.0:
        p = _mod2pi(2.0 * math.pi - math.acos(tmp))
        t = _mod2pi(alpha - math.atan2(ca - cb, distance - sa + sb) + p / 2.0)
        add("RLR", t, p, alpha - beta - t + p)

    tmp = (6.0 - distance * distance + 2.0 * cab + 2.0 * distance * (-sa + sb)) / 8.0
    if abs(tmp) <= 1.0:
        p = _mod2pi(2.0 * math.pi - math.acos(tmp))
        t = _mod2pi(-alpha - math.atan2(ca - cb, distance + sa - sb) + p / 2.0)
        add("LRL", t, p, beta - alpha - t + p)
    return candidates


def _sample_dubins_candidate(
    start: Pose, word: str, parameters: tuple[float, float, float], radius: float
) -> list[Pose]:
    poses = [start]
    current = start
    for primitive, normalized_length in zip(word, parameters):
        if primitive == "S":
            length = normalized_length * radius
            end = (
                current[0] + length * math.cos(current[2]),
                current[1] + length * math.sin(current[2]),
                current[2],
            )
            samples = _sample_segment(current, end, spacing_m=0.08)
        else:
            signed_angle = normalized_length if primitive == "L" else -normalized_length
            samples = _sample_arc(
                current, radius, signed_angle, step_rad=math.radians(2.0)
            )
        poses.extend(samples)
        current = poses[-1]
    return poses


def _dubins_forward_search(
    start: Pose,
    goal: Pose,
    apron: list[tuple[float, float]],
    keepouts: list[list[tuple[float, float]]],
    *,
    radii: tuple[float, ...],
) -> list[Segment] | None:
    """Shortest collision-free forward Dubins path over the frozen radii."""
    best = None
    best_length = math.inf
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    for radius in radii:
        distance = math.hypot(dx, dy) / radius
        theta = math.atan2(dy, dx)
        alpha = _mod2pi(start[2] - theta)
        beta = _mod2pi(goal[2] - theta)
        for word, t, p, q in _dubins_words(alpha, beta, distance):
            poses = _sample_dubins_candidate(start, word, (t, p, q), radius)
            endpoint = poses[-1]
            if math.hypot(endpoint[0] - goal[0], endpoint[1] - goal[1]) > 0.02:
                continue
            if abs(normalize_angle(endpoint[2] - goal[2])) > math.radians(1.0):
                continue
            poses[-1] = goal
            if not all(pose_feasible(pose, apron, keepouts) for pose in poses):
                continue
            length = radius * (t + p + q)
            if length < best_length:
                best_length = length
                best = [Segment(
                    direction="FORWARD",
                    curvature=1.0 / radius,
                    poses=tuple(poses),
                    connector_class="FORWARD_DUBINS_TURN",
                )]
    return best


def plan_forward_dubins_path(
    start: Pose,
    goal: Pose,
    apron: list[tuple[float, float]],
    keepouts: list[list[tuple[float, float]]],
) -> list[Pose] | None:
    """Public footprint-checked forward-only transit path helper."""
    result = _dubins_forward_search(
        start,
        goal,
        apron,
        keepouts,
        radii=(minimum_radius_m(), 1.8, 2.5, 3.5, 5.0),
    )
    return list(result[0].poses) if result else None


def plan_reverse_dubins_path(
    start: Pose,
    goal: Pose,
    apron: list[tuple[float, float]],
    keepouts: list[list[tuple[float, float]]],
) -> list[Pose] | None:
    """Footprint-checked reverse-only path using the Dubins symmetry.

    A chassis reversing at yaw ``psi`` follows the same geometric curve as a
    forward vehicle at ``psi + pi``.  Convert only the headings back; the
    sampled positions and frozen turning-radius checks remain unchanged.
    """
    virtual_start = (start[0], start[1], normalize_angle(start[2] + math.pi))
    virtual_goal = (goal[0], goal[1], normalize_angle(goal[2] + math.pi))
    virtual_path = plan_forward_dubins_path(
        virtual_start, virtual_goal, apron, keepouts
    )
    if not virtual_path:
        return None
    reverse_path = [
        (x, y, normalize_angle(yaw - math.pi))
        for x, y, yaw in virtual_path
    ]
    reverse_path[0] = start
    reverse_path[-1] = goal
    return reverse_path


def _reverse_arc_samples(
    start_pose: Pose,
    radius: float,
    angle: float,
    step_rad: float = math.radians(4.0),
) -> list[Pose]:
    """Sample a reverse arc: chassis backs up along the arc.

    Reversing with steering to the left rotates the chassis clockwise (yaw
    decreases) around the centre that lies on the steering side of the
    chassis.  The chassis centre follows the same circle as forward motion
    but in the opposite traversal direction.
    """
    samples = []
    remaining = abs(angle)
    current = start_pose
    while remaining > 1e-6:
        step = min(step_rad, remaining)
        current = _reverse_arc_offset(
            current, radius, math.copysign(step, angle)
        )
        samples.append(current)
        remaining -= step
    return samples


def _reverse_arc_offset(start_pose: Pose, radius_m: float, steer_angle: float) -> Pose:
    """Chassis pose after reversing with the given signed steering angle."""
    x, y, yaw = start_pose
    side = 1.0 if steer_angle >= 0.0 else -1.0
    radius = abs(radius_m)
    center_x = x - side * radius * math.sin(yaw)
    center_y = y + side * radius * math.cos(yaw)
    yaw_change = -math.copysign(abs(steer_angle), steer_angle)
    start_phi = yaw - side * math.pi / 2.0
    final_phi = start_phi + yaw_change
    end_x = center_x + radius * math.cos(final_phi)
    end_y = center_y + radius * math.sin(final_phi)
    return (end_x, end_y, normalize_angle(yaw + yaw_change))


@dataclass(frozen=True)
class Segment:
    direction: str  # FORWARD or REVERSE
    curvature: float  # signed
    poses: tuple[Pose, ...]
    connector_class: str
    cusp_before: bool = False


def split_hybrid_path_by_direction(
    poses: list[Pose] | tuple[Pose, ...],
    *,
    distance_epsilon_m: float = 1e-4,
) -> list[dict]:
    """Split a Smac Hybrid path into constant-direction sections.

    Smac stores the vehicle heading in every pose.  A path edge is forward
    when its displacement has a non-negative projection on that heading and
    reverse otherwise.  Splitting at every sign change lets FollowPath track
    one direction at a time and gives the executor an explicit place to stop
    before changing gear instead of chasing a replanned cusp.
    """
    normalized = [
        (float(pose[0]), float(pose[1]), normalize_angle(float(pose[2])))
        for pose in poses
    ]
    if len(normalized) < 2:
        return []

    edge_directions: list[str | None] = []
    for start, end in zip(normalized, normalized[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        if math.hypot(dx, dy) <= distance_epsilon_m:
            edge_directions.append(None)
            continue
        projection = dx * math.cos(start[2]) + dy * math.sin(start[2])
        edge_directions.append("FORWARD" if projection >= 0.0 else "REVERSE")

    first_direction = next(
        (direction for direction in edge_directions if direction is not None),
        None,
    )
    if first_direction is None:
        return []
    previous = first_direction
    for index, direction in enumerate(edge_directions):
        if direction is None:
            edge_directions[index] = previous
        else:
            previous = direction

    sections: list[dict] = []
    section_start = 0
    section_direction = str(edge_directions[0])
    for edge_index, direction in enumerate(edge_directions[1:], start=1):
        if direction == section_direction:
            continue
        section_poses = normalized[section_start:edge_index + 1]
        sections.append({
            "direction": section_direction,
            "poses": section_poses,
            "cusp_before": bool(sections),
        })
        section_start = edge_index
        section_direction = str(direction)
    sections.append({
        "direction": section_direction,
        "poses": normalized[section_start:],
        "cusp_before": bool(sections),
    })
    return [section for section in sections if len(section["poses"]) >= 2]


def _segments_to_components(
    connector_id: str,
    segments: list[Segment],
    source_swath_id: str,
    target_swath_id: str,
    collision_checked: bool,
    connector_class: str,
) -> tuple[CoverageComponent, ...]:
    components: list[CoverageComponent] = []
    for index, segment in enumerate(segments):
        if segment.cusp_before:
            cusp_pose = segment.poses[0]
            components.append(
                CoverageComponent(
                    component_id=f"{connector_id}-cusp-{index:02d}",
                    kind=ComponentType.CUSP_STOP,
                    points=((cusp_pose[0], cusp_pose[1]),),
                    brush_enabled=False,
                    speed_profile="STOP",
                    metadata={
                        "direction": "STOP",
                        "target_speed_mps": 0.0,
                        "connector_class": connector_class,
                        "collision_checked": collision_checked,
                        "source_swath_id": source_swath_id,
                        "target_swath_id": target_swath_id,
                    },
                )
            )
        points = tuple((pose[0], pose[1]) for pose in segment.poses)
        steering_reference = math.degrees(
            math.atan2(WHEELBASE_M * segment.curvature, 1.0)
        )
        components.append(
            CoverageComponent(
                component_id=f"{connector_id}-{segment.direction.lower()}-{index:02d}",
                kind=(
                    ComponentType.FORWARD
                    if segment.direction == "FORWARD"
                    else ComponentType.REVERSE
                ),
                points=points,
                brush_enabled=False,
                speed_profile=segment.direction,
                metadata={
                    "direction": segment.direction,
                    "curvature": segment.curvature,
                    "steering_reference_deg": steering_reference,
                    "speed_profile": segment.direction,
                    "connector_class": connector_class,
                    "collision_checked": collision_checked,
                    "source_swath_id": source_swath_id,
                    "target_swath_id": target_swath_id,
                    "headings_rad": [pose[2] for pose in segment.poses],
                },
            )
        )
    return tuple(components)


def _csc_forward_search(
    start: Pose,
    goal: Pose,
    apron: list[tuple[float, float]],
    keepouts: list[list[tuple[float, float]]],
    *,
    radii: tuple[float, ...],
    alpha_step_deg: float = 5.0,
    require_uturn: bool,
) -> list[Segment] | None:
    """Bounded forward CSC (arc-straight-arc) search."""
    best = None
    best_length = math.inf
    for direction in (1.0, -1.0):
        for radius in radii:
            alpha1 = alpha_step_deg
            while alpha1 <= 360.0 - alpha_step_deg:
                turn1 = math.radians(direction * alpha1)
                heading_after_first = start[2] + turn1
                first_end = _arc_offset(start, radius, turn1)
                if require_uturn:
                    net = normalize_angle(goal[2] - start[2])
                    target_net = math.radians(180.0) * direction
                    if abs(normalize_angle(net - target_net)) > math.radians(1.0):
                        alpha1 += alpha_step_deg
                        continue
                    desired_alpha2 = math.pi - math.radians(alpha1)
                else:
                    desired_alpha2 = normalize_angle(
                        goal[2] - heading_after_first
                    ) * direction
                if desired_alpha2 <= 0.0 or desired_alpha2 > math.pi:
                    alpha1 += alpha_step_deg
                    continue
                second_start = _arc_offset(goal, radius, -desired_alpha2 * direction)
                delta = (
                    second_start[0] - first_end[0],
                    second_start[1] - first_end[1],
                )
                straight_length = math.hypot(*delta)
                tangent = (
                    math.cos(heading_after_first),
                    math.sin(heading_after_first),
                )
                projection = delta[0] * tangent[0] + delta[1] * tangent[1]
                lateral = abs(delta[0] * tangent[1] - delta[1] * tangent[0])
                if projection < 0.0 or lateral > 1e-3:
                    alpha1 += alpha_step_deg
                    continue
                first_samples = _sample_arc(start, radius, turn1)
                straight_start = first_samples[-1] if first_samples else start
                straight_end = (
                    second_start[0],
                    second_start[1],
                    heading_after_first,
                )
                straight_samples = _sample_segment(straight_start, straight_end)
                second_samples = _sample_arc(
                    second_start, radius, desired_alpha2 * direction
                )
                all_poses = (
                    [(start[0], start[1], start[2])]
                    + first_samples
                    + straight_samples
                    + second_samples
                    + [(goal[0], goal[1], goal[2])]
                )
                if not all(
                    pose_feasible(pose, apron, keepouts) for pose in all_poses
                ):
                    alpha1 += alpha_step_deg
                    continue
                length = straight_length + radius * math.radians(
                    alpha1 + math.degrees(desired_alpha2)
                )
                if length < best_length:
                    best_length = length
                    best = [
                        Segment(
                            direction="FORWARD",
                            curvature=direction / radius,
                            poses=tuple(all_poses),
                            connector_class=(
                                "FORWARD_U_TURN"
                                if require_uturn else "FORWARD_TEARDROP_TURN"
                            ),
                        )
                    ]
                alpha1 += alpha_step_deg
    return best


def _lattice_three_point_search(
    start: Pose,
    goal: Pose,
    apron: list[tuple[float, float]],
    keepouts: list[list[tuple[float, float]]],
    *,
    spacing_m: float = 0.20,
    heading_bins: int = 24,
    max_nodes: int = 20000,
) -> list[Segment] | None:
    """Bounded Reeds-Shepp-like lattice search over forward/reverse arcs."""
    if not pose_feasible(start, apron, keepouts):
        return None
    if not pose_feasible(goal, apron, keepouts):
        return None
    radius = minimum_radius_m()
    arc_length = 0.30
    arc_angle = arc_length / radius
    primitives = (
        ("FORWARD", "left", +arc_angle),
        ("FORWARD", "right", -arc_angle),
        ("REVERSE", "left", +arc_angle),
        ("REVERSE", "right", -arc_angle),
    )
    xs = sorted(
        {
            round(start[0] + i * spacing_m, 3)
            for i in range(-15, 16)
        }
        | {
            round(goal[0] + i * spacing_m, 3)
            for i in range(-15, 16)
        }
    )
    ys = sorted(
        {
            round(start[1] + i * spacing_m, 3)
            for i in range(-15, 16)
        }
        | {
            round(goal[1] + i * spacing_m, 3)
            for i in range(-15, 16)
        }
    )
    heading_step = 2.0 * math.pi / heading_bins
    start_index = None
    goal_index = None
    nodes: dict[tuple, Pose] = {}
    for xi, x in enumerate(xs):
        for yi, y in enumerate(ys):
            for hi in range(heading_bins):
                pose = (x, y, normalize_angle(hi * heading_step))
                if not pose_feasible(pose, apron, keepouts):
                    continue
                index = (xi, yi, hi)
                nodes[index] = pose
    def _nearest(target: Pose) -> tuple | None:
        best = None
        best_distance = math.inf
        for index, pose in nodes.items():
            distance = math.hypot(
                pose[0] - target[0], pose[1] - target[1]
            ) + 0.5 * abs(normalize_angle(pose[2] - target[2]))
            if distance < best_distance:
                best_distance = distance
                best = index
        return best

    start_index = _nearest(start)
    goal_index = _nearest(goal)
    if start_index is None or goal_index is None:
        return None
    infinity = math.inf
    distances = {index: infinity for index in nodes}
    previous: dict[tuple, tuple] = {}
    previous_direction: dict[tuple, str] = {}
    previous_curvature: dict[tuple, float] = {}
    visited: set[tuple] = set()
    distances[start_index] = 0.0
    queue = [(0.0, start_index)]
    expanded = 0
    while expanded < max_nodes:
        if not queue:
            break
        distance, current = heapq.heappop(queue)
        if current in visited or not math.isfinite(distance):
            continue
        if current == goal_index:
            break
        visited.add(current)
        expanded += 1
        current_pose = nodes[current]
        for direction, turn, angle in primitives:
            if direction == "REVERSE":
                child = _reverse_arc_offset(current_pose, radius, angle)
                primitive_samples = _reverse_arc_samples(
                    current_pose, radius, angle,
                    step_rad=min(abs(angle), math.radians(2.0)),
                )
            else:
                child = _arc_offset(current_pose, radius, angle)
                primitive_samples = _sample_arc(
                    current_pose, radius, angle,
                    step_rad=min(abs(angle), math.radians(2.0)),
                )
            if not all(
                pose_feasible(sample, apron, keepouts)
                for sample in primitive_samples
            ):
                continue
            xi = min(
                range(len(xs)),
                key=lambda i: abs(xs[i] - child[0]),
            )
            yi = min(
                range(len(ys)),
                key=lambda i: abs(ys[i] - child[1]),
            )
            hi = int(round(normalize_angle(child[2]) / heading_step)) % heading_bins
            child_index = (xi, yi, hi)
            if child_index not in nodes:
                continue
            if math.hypot(xs[xi] - child[0], ys[yi] - child[1]) > 0.12:
                continue
            cost = distances[current] + arc_length
            if direction != previous_direction.get(current):
                cost += 0.5
            if cost < distances[child_index]:
                distances[child_index] = cost
                previous[child_index] = current
                previous_direction[child_index] = direction
                previous_curvature[child_index] = (
                    (1.0 / radius) if turn == "left" else (-1.0 / radius)
                )
                heapq.heappush(queue, (cost, child_index))
    if goal_index not in visited:
        return None
    moves = []
    cursor = goal_index
    while cursor != start_index:
        parent = previous.get(cursor)
        if parent is None:
            return None
        moves.append(
            (
                nodes[cursor],
                previous_direction[cursor],
                previous_curvature[cursor],
            )
        )
        cursor = parent
    moves.reverse()
    segments: list[Segment] = []
    current_direction = None
    current_class = "REEDS_SHEPP_THREE_POINT_TURN"
    current_pose = start
    for chassis_pose, direction, curvature in moves:
        if direction != current_direction:
            current_direction = direction
            segments.append(
                Segment(
                    direction=direction,
                    curvature=curvature,
                    poses=(current_pose,),
                    connector_class=current_class,
                    cusp_before=len(segments) > 0,
                )
            )
        angle_step = math.copysign(arc_length / radius, curvature)
        if direction == "REVERSE":
            intermediates = _reverse_arc_samples(
                current_pose,
                radius,
                angle_step,
                step_rad=min(abs(angle_step), math.radians(2.0)),
            )
        else:
            intermediates = _sample_arc(
                current_pose,
                radius,
                angle_step,
                step_rad=min(abs(angle_step), math.radians(2.0)),
            )
        segments[-1] = Segment(
            direction=direction,
            curvature=curvature,
            poses=segments[-1].poses + tuple(intermediates) + (chassis_pose,),
            connector_class=current_class,
            cusp_before=segments[-1].cusp_before,
        )
        current_pose = chassis_pose
    return segments


def plan_ackermann_connector(
    connector_id: str,
    start: tuple[float, float],
    start_yaw: float,
    goal: tuple[float, float],
    goal_yaw: float,
    apron: list[tuple[float, float]],
    keepouts: list[list[tuple[float, float]]],
    source_swath_id: str,
    target_swath_id: str,
) -> tuple[CoverageComponent, ...] | None:
    """Plan the best connector, or None when every option is infeasible."""
    start_pose: Pose = (start[0], start[1], start_yaw)
    goal_pose: Pose = (goal[0], goal[1], goal_yaw)
    # The frozen 1.429 m physical limit is safe for collision checking but the
    # Gazebo steering plant cannot repeatedly track a near-closed maximum-lock
    # arc without >0.4 m cross-track error. Keep transit able to use the true
    # limit, while operation connectors use a conservative validated radius.
    radii = (TRACKABLE_FORWARD_CONNECTOR_RADIUS_M, 2.0, 2.5, 3.5, 5.0)
    dubins = _dubins_forward_search(
        start_pose, goal_pose, apron, keepouts, radii=radii
    )
    if dubins:
        return _segments_to_components(
            connector_id, dubins, source_swath_id, target_swath_id,
            collision_checked=True, connector_class="FORWARD_DUBINS_TURN",
        )
    uturn = _csc_forward_search(
        start_pose, goal_pose, apron, keepouts,
        radii=radii, require_uturn=True,
    )
    if uturn:
        return _segments_to_components(
            connector_id, uturn, source_swath_id, target_swath_id,
            collision_checked=True, connector_class="FORWARD_U_TURN",
        )
    teardrop = _csc_forward_search(
        start_pose, goal_pose, apron, keepouts,
        radii=radii, require_uturn=False,
    )
    if teardrop:
        return _segments_to_components(
            connector_id, teardrop, source_swath_id, target_swath_id,
            collision_checked=True, connector_class="FORWARD_TEARDROP_TURN",
        )
    three_point = _lattice_three_point_search(
        start_pose, goal_pose, apron, keepouts
    )
    if three_point:
        return _segments_to_components(
            connector_id, three_point, source_swath_id, target_swath_id,
            collision_checked=True,
            connector_class="REEDS_SHEPP_THREE_POINT_TURN",
        )
    return None


def deferred_swath_component(
    component_id: str,
    swath_id: str,
    points: tuple[tuple[float, float], ...],
) -> CoverageComponent:
    """Record an infeasible swath as deferred instead of cheating it."""
    return CoverageComponent(
        component_id=component_id,
        kind=ComponentType.DEFERRED_SWATH,
        points=points,
        brush_enabled=False,
        speed_profile="DEFERRED",
        metadata={
            "swath_id": swath_id,
            "deferred": True,
            "reason": "no feasible Ackermann connector within the turning apron",
            "collision_checked": True,
        },
    )


def hybrid_connector_request(
    connector_id: str,
    start: tuple[float, float],
    start_yaw: float,
    goal: tuple[float, float],
    goal_yaw: float,
    source_swath_id: str,
    target_swath_id: str,
) -> CoverageComponent:
    """Defer connector feasibility to the Smac Hybrid planner at runtime."""
    return CoverageComponent(
        component_id=connector_id,
        kind=ComponentType.OBSTACLE_BYPASS,
        points=((start[0], start[1]), (goal[0], goal[1])),
        brush_enabled=False,
        speed_profile="BYPASS",
        metadata={
            "direction": "HYBRID",
            "connector_class": "SMAC_HYBRID_CONNECTOR",
            "collision_checked": False,
            "deferred_to_nav2": True,
            "start_yaw_rad": start_yaw,
            "goal_yaw_rad": goal_yaw,
            "source_swath_id": source_swath_id,
            "target_swath_id": target_swath_id,
        },
    )
