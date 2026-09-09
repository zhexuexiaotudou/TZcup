"""Truth-free map lifecycle primitives for the formal campus product path."""

from __future__ import annotations

from array import array
from collections import deque
from dataclasses import dataclass
import datetime
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Sequence

import yaml

from .contract import CANONICAL_PLANNING_KINEMATIC_CONSTRAINT


REQUIRED_SAVED_MAP_SUPPORT_FILES = frozenset({
    "mission_geometry.yaml",
    "materialization_contract.yaml",
    "geofence_keepout.yaml",
    "geofence_keepout.pgm",
    "neutral_speed.yaml",
    "neutral_speed.pgm",
})
MAPPING_POSE_SOURCE = "wheel_imu_ekf_lidar_scan_matching_gnss_consistency"
MAXIMUM_SAVED_MAP_RESOLUTION_M = 0.10


class MapLifecycleError(RuntimeError):
    """Raised when a formal map artifact fails closed."""


def hard_restart_record_valid(record: dict, map_root: str | Path) -> bool:
    """Verify a separate saved-map process start against immutable map evidence."""
    root = Path(map_root)
    try:
        mapping_completion = datetime.datetime.fromisoformat(
            str(record["mapping_completion_wall_time"])
        )
        mapping_cleanup = datetime.datetime.fromisoformat(
            str(record["mapping_cleanup_wall_time"])
        )
        cleaning_start = datetime.datetime.fromisoformat(
            str(record["cleaning_start_wall_time"])
        )
        manifest_hash = hashlib.sha256(
            (root / "map_lifecycle_manifest.json").read_bytes()
        ).hexdigest()
        mapping_runtime_hash = hashlib.sha256(
            (root / "mapping_runtime.json").read_bytes()
        ).hexdigest()
        handoff_hash = hashlib.sha256(
            (root / "mapping_handoff_record.json").read_bytes()
        ).hexdigest()
    except (KeyError, OSError, ValueError):
        return False
    mapping_pids = {
        record.get("mapping_runner_pid"),
        record.get("mapping_launch_pid"),
        record.get("mapping_collector_pid"),
    }
    cleaning_pids = {
        record.get("cleaning_runner_pid"),
        record.get("cleaning_launch_pid"),
    }
    return (
        record.get("schema_version") == 2
        and record.get("mapping_stopped_before_cleaning") is True
        and record.get("mapping_process_count_before_cleaning") == 0
        and record.get("mapping_pid_alive_count_before_cleaning") == 0
        and record.get("mapping_runner_exit_code") == 0
        and record.get("restart_type") == "separate_process_hard_restart"
        and mapping_completion <= mapping_cleanup <= cleaning_start
        and len(mapping_pids) == 3
        and len(cleaning_pids) == 2
        and all(isinstance(pid, int) and pid > 0 for pid in mapping_pids)
        and all(isinstance(pid, int) and pid > 0 for pid in cleaning_pids)
        and mapping_pids.isdisjoint(cleaning_pids)
        and record.get("map_lifecycle_manifest_sha256") == manifest_hash
        and record.get("mapping_runtime_sha256") == mapping_runtime_hash
        and record.get("mapping_handoff_record_sha256") == handoff_hash
    )


@dataclass(frozen=True)
class CampusMapContract:
    episode_id: str
    map_id: str
    field_area_m2: float
    geofence: tuple[tuple[float, float], ...]
    source_geofence: tuple[tuple[float, float], ...]
    fixed_start_source: tuple[float, float, float]


@dataclass(frozen=True)
class GridObservation:
    observed_cells: int
    field_cells: int
    observed_area_m2: float
    field_sampled_area_m2: float
    observed_fraction: float
    passed: bool


def goal_tangent_yaw(
    robot_map_x: float,
    robot_map_y: float,
    target_map_x: float,
    target_map_y: float,
) -> float:
    """Return the map-frame heading from the robot toward a frontier goal."""
    values = (robot_map_x, robot_map_y, target_map_x, target_map_y)
    if not all(math.isfinite(value) for value in values):
        raise MapLifecycleError("frontier tangent inputs must be finite")
    dx = target_map_x - robot_map_x
    dy = target_map_y - robot_map_y
    if math.hypot(dx, dy) <= 1e-9:
        raise MapLifecycleError("frontier tangent requires a distinct target")
    return math.atan2(dy, dx)


def _polygon_area(points: Sequence[tuple[float, float]]) -> float:
    return abs(sum(
        x1 * y2 - x2 * y1
        for (x1, y1), (x2, y2) in zip(points, (*points[1:], points[0]))
    )) / 2.0


def _local_point(
    point: tuple[float, float], start: tuple[float, float, float]
) -> tuple[float, float]:
    dx, dy = point[0] - start[0], point[1] - start[1]
    cosine, sine = math.cos(start[2]), math.sin(start[2])
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def _polygon_from_manifest(field: dict[str, Any], key: str, frame_id: str) -> tuple[tuple[float, float], ...] | None:
    value = field.get(key)
    if value is None:
        return None
    if not isinstance(value, dict) or value.get("frame_id") != frame_id:
        raise MapLifecycleError(f"{key} has an invalid frame")
    raw = value.get("polygon_m")
    try:
        polygon = tuple((float(point[0]), float(point[1])) for point in raw)
    except (TypeError, ValueError, IndexError) as exc:
        raise MapLifecycleError(f"{key} has invalid points") from exc
    if len(polygon) < 3 or not all(math.isfinite(item) for point in polygon for item in point):
        raise MapLifecycleError(f"{key} has invalid points")
    return polygon


def _same_polygon(left: Sequence[tuple[float, float]], right: Sequence[tuple[float, float]]) -> bool:
    return len(left) == len(right) and all(math.isclose(ax, bx, abs_tol=1e-6) and math.isclose(ay, by, abs_tol=1e-6) for (ax, ay), (bx, by) in zip(left, right))


def load_campus_map_contract(path: str | Path) -> CampusMapContract:
    """Load public mission geometry, never evaluator/world object truth."""
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise MapLifecycleError("unable to read public episode manifest") from exc
    if not isinstance(value, dict) or value.get("profile") != "formal":
        raise MapLifecycleError("map lifecycle requires a formal public episode")
    field = value.get("field")
    start_value = value.get("vehicle_start_pose_source_world") or value.get("vehicle_start_pose_map")
    if not isinstance(field, dict) or not isinstance(start_value, dict):
        raise MapLifecycleError("formal field or fixed start is missing")
    if field.get("physical_boundary_walls") is not False:
        raise MapLifecycleError("formal lifecycle requires the frozen no-wall field")
    try:
        width = float(field["width_m"])
        height = float(field["height_m"])
        area = float(field["area_m2"])
        source = tuple(
            float(start_value[key]) for key in ("x_m", "y_m", "yaw_rad")
        )
        source_polygon = _polygon_from_manifest(field, "source_world_geofence", "source_world")
        if source_polygon is None:
            if field.get("geofence_frame") not in {"map", "source_world"}:
                raise MapLifecycleError("legacy geofence frame is invalid")
            source_polygon = tuple((float(point[0]), float(point[1])) for point in field["geofence_polygon_m"])
        else:
            if field.get("geofence_frame") != "source_world":
                raise MapLifecycleError("explicit source-world geofence has an ambiguous legacy frame")
            legacy = tuple((float(point[0]), float(point[1])) for point in field.get("geofence_polygon_m", ()))
            if legacy and not _same_polygon(legacy, source_polygon):
                raise MapLifecycleError("legacy and source-world geofences disagree")
            legacy_contract = field.get("legacy_geofence")
            if (
                not isinstance(legacy_contract, dict)
                or legacy_contract.get("field") != "geofence_polygon_m"
                or legacy_contract.get("frame_id") != "source_world"
                or not isinstance(legacy_contract.get("deprecation"), str)
                or not legacy_contract["deprecation"]
            ):
                raise MapLifecycleError("legacy geofence deprecation contract is invalid")
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        raise MapLifecycleError("formal field geometry is invalid") from exc
    if len(source_polygon) < 3 or len(source) != 3:
        raise MapLifecycleError("formal field polygon or fixed start is invalid")
    values = (width, height, area, *source, *(v for p in source_polygon for v in p))
    if not all(math.isfinite(item) for item in values):
        raise MapLifecycleError("formal field contains a non-finite value")
    # The frozen formal map is 200 x 100 m. Other randomized aspect-ratio maps
    # are separate generalization episodes and cannot masquerade as this gate.
    if abs(width - 200.0) > 1e-6 or abs(height - 100.0) > 1e-6:
        raise MapLifecycleError("formal baseline field must be exactly 200 x 100 m")
    if abs(area - 20_000.0) > 1e-3 or abs(_polygon_area(source_polygon) - area) > 1e-3:
        raise MapLifecycleError("formal field area must be exactly 20000 m2")
    expected_local = tuple(_local_point(point, source) for point in source_polygon)
    local = _polygon_from_manifest(field, "localization_map_geofence", "map")
    if local is None:
        local = expected_local
    elif not _same_polygon(local, expected_local):
        raise MapLifecycleError("localization geofence must apply the source transform exactly once")
    return CampusMapContract(
        episode_id=str(value.get("episode_id", "")),
        map_id=str(value.get("map_id", "")),
        field_area_m2=area,
        geofence=local,
        source_geofence=source_polygon,
        fixed_start_source=(source[0], source[1], source[2]),
    )


def _inside(x: float, y: float, polygon: Sequence[tuple[float, float]]) -> bool:
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y) and x < (
            (x2 - x1) * (y - y1) / (y2 - y1) + x1
        ):
            inside = not inside
        previous = current
    return inside


def assess_grid_observation(
    data: Sequence[int],
    *,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    geofence: Sequence[tuple[float, float]],
    threshold: float = 0.95,
) -> GridObservation:
    """Count known SLAM cells whose centers lie inside the configured field."""
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise MapLifecycleError("occupancy grid dimensions do not match its payload")
    if (
        not math.isfinite(resolution)
        or resolution <= 0.0
        or resolution > MAXIMUM_SAVED_MAP_RESOLUTION_M
    ):
        raise MapLifecycleError(
            "formal SLAM resolution must be in "
            f"(0, {MAXIMUM_SAVED_MAP_RESOLUTION_M:.2f}] m"
        )
    if not 0.0 < threshold <= 1.0:
        raise MapLifecycleError("observation threshold must be in (0, 1]")
    cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)
    # Cells outside the current OccupancyGrid extent are still unobserved
    # formal-field cells. Using only the overlap as denominator would let a
    # tiny locally complete map pass the 95% whole-campus gate.
    field_cells = max(1, round(_polygon_area(geofence) / (resolution * resolution)))
    observed_cells = 0
    for row in range(height):
        local_y = (row + 0.5) * resolution
        base = row * width
        for column in range(width):
            local_x = (column + 0.5) * resolution
            x = origin_x + cosine * local_x - sine * local_y
            y = origin_y + sine * local_x + cosine * local_y
            if _inside(x, y, geofence):
                if int(data[base + column]) >= 0:
                    observed_cells += 1
    if observed_cells == 0:
        raise MapLifecycleError("SLAM grid does not overlap the formal geofence")
    observed_cells = min(observed_cells, field_cells)
    cell_area = resolution * resolution
    fraction = observed_cells / field_cells
    return GridObservation(
        observed_cells=observed_cells,
        field_cells=field_cells,
        observed_area_m2=observed_cells * cell_area,
        field_sampled_area_m2=field_cells * cell_area,
        observed_fraction=fraction,
        passed=fraction + 1e-12 >= threshold,
    )


def select_frontier_goal(
    data: Sequence[int],
    *,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    geofence: Sequence[tuple[float, float]],
    robot_x: float,
    robot_y: float,
    previous_goals: Sequence[tuple[float, float]] = (),
    sample_spacing_m: float = 0.50,
    previous_goal_clearance_m: float = 1.0,
    clearance_m: float = 0.95,
    frontier_standoff_m: float = 1.20,
    seed_max_offset_m: float = 0.75,
    min_goal_distance_m: float = 1.0,
    max_search_radius_m: float = 14.0,
) -> tuple[float, float] | None:
    """Select a nearby, footprint-clear goal inside a reachable frontier."""
    if (
        width <= 2
        or height <= 2
        or len(data) != width * height
        or not math.isfinite(resolution)
        or resolution <= 0.0
    ):
        return None
    for name, value in (
        ("origin_x", origin_x),
        ("origin_y", origin_y),
        ("origin_yaw", origin_yaw),
        ("robot_x", robot_x),
        ("robot_y", robot_y),
    ):
        if not math.isfinite(value):
            raise MapLifecycleError(f"{name} must be finite")
    for name, value in (
        ("sample_spacing_m", sample_spacing_m),
        ("previous_goal_clearance_m", previous_goal_clearance_m),
        ("clearance_m", clearance_m),
        ("frontier_standoff_m", frontier_standoff_m),
        ("seed_max_offset_m", seed_max_offset_m),
        ("min_goal_distance_m", min_goal_distance_m),
        ("max_search_radius_m", max_search_radius_m),
    ):
        if not math.isfinite(value) or value < 0.0:
            raise MapLifecycleError(f"{name} must be finite and non-negative")
    if frontier_standoff_m < clearance_m:
        raise MapLifecycleError("frontier standoff must cover footprint clearance")

    cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)

    def global_world_position(row: int, column: int) -> tuple[float, float]:
        local_x = (column + 0.5) * resolution
        local_y = (row + 0.5) * resolution
        return (
            origin_x + cosine * local_x - sine * local_y,
            origin_y + sine * local_x + cosine * local_y,
        )

    # Nav2 mapping uses a 30 m rolling global window. Bound frontier work to
    # the same neighborhood so a full 200 x 100 m map never turns one timer
    # callback into a multi-million-cell allocation or scan.
    robot_dx, robot_dy = robot_x - origin_x, robot_y - origin_y
    robot_local_x = cosine * robot_dx + sine * robot_dy
    robot_local_y = -sine * robot_dx + cosine * robot_dy
    robot_column = math.floor(robot_local_x / resolution)
    robot_row = math.floor(robot_local_y / resolution)
    padding_m = max_search_radius_m + clearance_m + resolution
    padding_cells = math.ceil(padding_m / resolution)
    min_column = max(0, robot_column - padding_cells)
    max_column = min(width - 1, robot_column + padding_cells)
    min_row = max(0, robot_row - padding_cells)
    max_row = min(height - 1, robot_row + padding_cells)
    if min_column > max_column or min_row > max_row:
        return None
    local_width = max_column - min_column + 1
    local_height = max_row - min_row + 1
    local_size = local_width * local_height

    free = bytearray(local_size)
    prefix_width = local_width + 1
    blocked_prefix = array("I", [0]) * ((local_height + 1) * prefix_width)
    for local_row in range(local_height):
        global_row = min_row + local_row
        blocked_in_row = 0
        current_prefix = (local_row + 1) * prefix_width
        previous_prefix = local_row * prefix_width
        for local_column in range(local_width):
            global_column = min_column + local_column
            local_index = local_row * local_width + local_column
            world_x, world_y = global_world_position(global_row, global_column)
            is_free = (
                0 <= int(data[global_row * width + global_column]) <= 25
                and _inside(world_x, world_y, geofence)
            )
            free[local_index] = is_free
            blocked_in_row += not is_free
            blocked_prefix[current_prefix + local_column + 1] = (
                blocked_prefix[previous_prefix + local_column + 1]
                + blocked_in_row
            )

    # A square clearance query is a cheap, conservative envelope of the
    # yaw-invariant 0.95 m vehicle circle. The additional half cell ensures a
    # blocked raster cell touching the envelope cannot be missed by its center.
    clearance_cells = (
        0 if clearance_m == 0.0 else math.ceil(clearance_m / resolution + 0.5)
    )
    safe_cache = bytearray(local_size)

    def is_safe(local_index: int) -> bool:
        cached = safe_cache[local_index]
        if cached:
            return cached == 2
        local_row, local_column = divmod(local_index, local_width)
        row0, row1 = local_row - clearance_cells, local_row + clearance_cells
        column0 = local_column - clearance_cells
        column1 = local_column + clearance_cells
        safe = (
            row0 >= 0
            and column0 >= 0
            and row1 < local_height
            and column1 < local_width
        )
        if safe:
            top = row0 * prefix_width
            bottom = (row1 + 1) * prefix_width
            blocked = (
                blocked_prefix[bottom + column1 + 1]
                - blocked_prefix[top + column1 + 1]
                - blocked_prefix[bottom + column0]
                + blocked_prefix[top + column0]
            )
            safe = blocked == 0
        safe_cache[local_index] = 2 if safe else 1
        return safe

    def local_world_position(local_index: int) -> tuple[float, float]:
        local_row, local_column = divmod(local_index, local_width)
        return global_world_position(
            min_row + local_row, min_column + local_column
        )

    seed: int | None = None
    seed_offset = math.inf
    for local_index, is_free in enumerate(free):
        if not is_free:
            continue
        world_x, world_y = local_world_position(local_index)
        offset = math.hypot(world_x - robot_x, world_y - robot_y)
        if offset < seed_offset:
            seed, seed_offset = local_index, offset
    if seed is None or seed_offset > seed_max_offset_m:
        return None

    neighbours = ((-1, 0), (1, 0), (0, -1), (0, 1))
    seed_local_row, seed_local_column = divmod(seed, local_width)
    seed_global_row = min_row + seed_local_row
    seed_global_column = min_column + seed_local_column
    seed_touches_map_boundary = (
        seed_global_row - clearance_cells < 0
        or seed_global_column - clearance_cells < 0
        or seed_global_row + clearance_cells >= height
        or seed_global_column + clearance_cells >= width
    )
    if not is_safe(seed) and not seed_touches_map_boundary:
        return None

    # The robot may sit just outside slam_toolbox's still-growing raster. Walk
    # only the shortest free bootstrap band to the first footprint-safe cell;
    # after that, never propagate through a corridor the vehicle cannot fit.
    bootstrap_steps = array("i", [-1]) * local_size
    bootstrap_steps[seed] = 0
    queue: deque[int] = deque([seed])
    anchor: int | None = None
    while queue:
        current = queue.popleft()
        distance = bootstrap_steps[current]
        if is_safe(current):
            anchor = current
            break
        if distance >= clearance_cells:
            continue
        local_row, local_column = divmod(current, local_width)
        for dr, dc in neighbours:
            next_row, next_column = local_row + dr, local_column + dc
            if not (
                0 <= next_row < local_height
                and 0 <= next_column < local_width
            ):
                continue
            next_index = next_row * local_width + next_column
            if bootstrap_steps[next_index] < 0 and free[next_index]:
                bootstrap_steps[next_index] = distance + 1
                queue.append(next_index)
    if anchor is None:
        return None

    reachable_steps = array("i", [-1]) * local_size
    reachable_steps[anchor] = bootstrap_steps[anchor]
    queue = deque([anchor])
    while queue:
        current = queue.popleft()
        local_row, local_column = divmod(current, local_width)
        for dr, dc in neighbours:
            next_row, next_column = local_row + dr, local_column + dc
            if not (
                0 <= next_row < local_height
                and 0 <= next_column < local_width
            ):
                continue
            next_index = next_row * local_width + next_column
            if reachable_steps[next_index] < 0 and is_safe(next_index):
                reachable_steps[next_index] = reachable_steps[current] + 1
                queue.append(next_index)

    raw_frontiers: list[int] = []
    for local_index, is_free in enumerate(free):
        if not is_free:
            continue
        local_row, local_column = divmod(local_index, local_width)
        global_row = min_row + local_row
        global_column = min_column + local_column
        if not (0 < global_row < height - 1 and 0 < global_column < width - 1):
            continue
        if any(
            int(data[(global_row + dr) * width + global_column + dc]) < 0
            and _inside(
                *global_world_position(global_row + dr, global_column + dc),
                geofence,
            )
            for dr, dc in neighbours
        ):
            raw_frontiers.append(local_index)
    if not raw_frontiers:
        return None

    max_frontier_steps = math.ceil(frontier_standoff_m / resolution)
    frontier_steps = array("i", [-1]) * local_size
    queue = deque(raw_frontiers)
    for local_index in raw_frontiers:
        frontier_steps[local_index] = 0
    while queue:
        current = queue.popleft()
        distance = frontier_steps[current]
        if distance >= max_frontier_steps:
            continue
        local_row, local_column = divmod(current, local_width)
        for dr, dc in neighbours:
            next_row, next_column = local_row + dr, local_column + dc
            if not (
                0 <= next_row < local_height
                and 0 <= next_column < local_width
            ):
                continue
            next_index = next_row * local_width + next_column
            if free[next_index] and frontier_steps[next_index] < 0:
                frontier_steps[next_index] = distance + 1
                queue.append(next_index)

    stride = max(1, round(sample_spacing_m / resolution))
    candidates: list[tuple[int, tuple[float, float]]] = []
    sampled_candidates: list[tuple[int, tuple[float, float]]] = []
    for local_index, distance_steps in enumerate(reachable_steps):
        if (
            distance_steps < 0
            or frontier_steps[local_index] < 0
            or not is_safe(local_index)
        ):
            continue
        world_x, world_y = local_world_position(local_index)
        if math.hypot(world_x - robot_x, world_y - robot_y) > max_search_radius_m:
            continue
        if any(
            math.hypot(world_x - old_x, world_y - old_y)
            < previous_goal_clearance_m
            for old_x, old_y in previous_goals
        ):
            continue
        candidate = (distance_steps, (world_x, world_y))
        candidates.append(candidate)
        local_row, local_column = divmod(local_index, local_width)
        if not (min_row + local_row) % stride and not (
            min_column + local_column
        ) % stride:
            sampled_candidates.append(candidate)
    if not candidates:
        return None
    candidates = sampled_candidates or candidates
    distant = [
        candidate
        for candidate in candidates
        if candidate[0] * resolution >= min_goal_distance_m
    ]
    pool = distant or [candidate for candidate in candidates if candidate[0] > 0]
    if not pool:
        return None
    return min(pool, key=lambda candidate: candidate[0])[1]


def _write_pgm(path: Path, rows: list[bytearray]) -> None:
    height, width = len(rows), len(rows[0])
    path.write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii")
        + b"".join(bytes(row) for row in reversed(rows))
    )


def prepare_public_lifecycle_artifacts(
    contract: CampusMapContract, output_directory: str | Path, *, resolution: float = 0.25
) -> dict[str, Path]:
    """Create only geofence/support files; no world object geometry is consumed."""
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    margin = 2.0
    min_x = min(p[0] for p in contract.geofence) - margin
    min_y = min(p[1] for p in contract.geofence) - margin
    max_x = max(p[0] for p in contract.geofence) + margin
    max_y = max(p[1] for p in contract.geofence) + margin
    width = math.ceil((max_x - min_x) / resolution)
    height = math.ceil((max_y - min_y) / resolution)
    keepout = [bytearray(width) for _ in range(height)]
    speed = [bytearray([255] * width) for _ in range(height)]
    for row in range(height):
        y = min_y + (row + 0.5) * resolution
        for column in range(width):
            x = min_x + (column + 0.5) * resolution
            if not _inside(x, y, contract.geofence):
                keepout[row][column] = 0
            else:
                keepout[row][column] = 255
    keepout_image = output / "geofence_keepout.pgm"
    speed_image = output / "neutral_speed.pgm"
    _write_pgm(keepout_image, keepout)
    _write_pgm(speed_image, speed)
    metadata = {
        "resolution": resolution,
        "origin": [min_x, min_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
        "mode": "trinary",
    }
    keepout_yaml = output / "geofence_keepout.yaml"
    speed_yaml = output / "neutral_speed.yaml"
    keepout_yaml.write_text(yaml.safe_dump({"image": keepout_image.name, **metadata}, sort_keys=False), encoding="utf-8")
    speed_yaml.write_text(yaml.safe_dump({"image": speed_image.name, **metadata}, sort_keys=False), encoding="utf-8")
    mission = {
        "schema_version": 1,
        "mission_id": f"formal-lifecycle-{contract.episode_id}",
        "mode": "mapping_then_cleaning",
        "frame_id": "map",
        "kinematic_model": "four_wheel_skid_steer",
        "planning_kinematic_constraint": CANONICAL_PLANNING_KINEMATIC_CONSTRAINT,
        "physical_steering_claim": False,
        "outer_polygon": [list(point) for point in contract.geofence],
        "keepout_polygons": [],
        "vehicle_start_pose_map": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
        "source_fixed_start_pose": list(contract.fixed_start_source),
        "truth_boundary": {
            "world_geometry_used_for_product_map": False,
            "evaluator_truth_used": False,
            "dirt_truth_used": False,
        },
    }
    mission_path = output / "mission_geometry.yaml"
    mission_path.write_text(yaml.safe_dump(mission, sort_keys=False), encoding="utf-8")
    materialization = {
        "schema_version": 2,
        "map_id": contract.map_id,
        "map_source": "slam_toolbox_lidar_odometry",
        "world_geometry_used_for_product_map": False,
        "evaluator_truth_used": False,
        "dirt_truth_used": False,
        "mapping_ignores_dirt": True,
        "fixed_start_local_pose": [0.0, 0.0, 0.0],
        "geofence_area_m2": contract.field_area_m2,
        "resolution_contract": {
            "static_materializer": {"value_source": "formal_campus.launch.py:map_resolution", "purpose": "public_world_static_collision_raster"},
            "lifecycle_support_mask": {"resolution_m": resolution, "value_source": "prepare_public_lifecycle_artifacts(resolution)", "purpose": "public_geofence_support_mask"},
            "slam_occupancy": {"value_source": "saved_map_metadata", "maximum_accepted_resolution_m": MAXIMUM_SAVED_MAP_RESOLUTION_M, "purpose": "runtime_lidar_slam_occupancy"},
            "coverage_planning": {"value_source": "ProductCoverageTelemetry(raster_resolution_m)", "purpose": "saved_map_coverage_raster_planning"},
        },
    }
    materialization_path = output / "materialization_contract.yaml"
    materialization_path.write_text(yaml.safe_dump(materialization, sort_keys=False), encoding="utf-8")
    return {
        "keepout_map": keepout_yaml,
        "speed_map": speed_yaml,
        "mission_geometry": mission_path,
        "materialization_contract": materialization_path,
    }


def sha256(path: str | Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _artifact_basename(value: Any, *, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise MapLifecycleError(f"{label} is missing")
    candidate = Path(value)
    if (
        candidate.is_absolute()
        or candidate.name != value
        or "/" in value
        or "\\" in value
        or value in {".", ".."}
    ):
        raise MapLifecycleError(f"{label} must be a local artifact basename")
    return value


def validate_saved_map_artifact(
    artifact_directory: str | Path, contract: CampusMapContract
) -> dict[str, Any]:
    root = Path(artifact_directory)
    manifest_path = root / "map_lifecycle_manifest.json"
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise MapLifecycleError("saved-map lifecycle manifest is missing or invalid") from exc
    try:
        observed_fraction = float(manifest.get("observed_fraction", 0.0))
        quality_threshold = float(manifest.get("quality_threshold", 0.0))
        stable_samples = int(manifest.get("stable_gate_samples", 0))
    except (TypeError, ValueError) as exc:
        raise MapLifecycleError("saved map has invalid quality metadata") from exc
    if (
        manifest.get("schema_version") != 1
        or manifest.get("status") != "ready_for_localization_cleaning"
        or manifest.get("episode_id") != contract.episode_id
        or manifest.get("map_id") != contract.map_id
        or not math.isfinite(observed_fraction)
        or not math.isfinite(quality_threshold)
        or quality_threshold < 0.95
        or observed_fraction < quality_threshold
        or stable_samples < 3
        or manifest.get("fixed_start_verified") is not True
        or manifest.get("gnss_mapping_reference_observed") is not True
        or manifest.get("mapping_pose_source") != MAPPING_POSE_SOURCE
        or manifest.get("world_truth_used_for_control") is not False
        or manifest.get("mapping_ignored_dirt") is not True
    ):
        raise MapLifecycleError("saved map did not pass the formal lifecycle gate")
    occupancy_name = _artifact_basename(
        manifest.get("occupancy_map"), label="occupancy map"
    )
    if occupancy_name != "occupancy.yaml":
        raise MapLifecycleError("formal saved map must use occupancy.yaml")
    try:
        occupancy_metadata = yaml.safe_load(
            (root / occupancy_name).read_text(encoding="utf-8")
        )
    except (OSError, yaml.YAMLError) as exc:
        raise MapLifecycleError("saved occupancy metadata is missing or invalid") from exc
    if not isinstance(occupancy_metadata, dict):
        raise MapLifecycleError("saved occupancy metadata is invalid")
    image_name = _artifact_basename(
        occupancy_metadata.get("image"), label="occupancy image"
    )
    if image_name != "occupancy.pgm":
        raise MapLifecycleError("formal saved map must use occupancy.pgm")
    required_files = {
        occupancy_name,
        image_name,
        *REQUIRED_SAVED_MAP_SUPPORT_FILES,
    }
    hashes = manifest.get("sha256")
    if not isinstance(hashes, dict) or set(hashes) != required_files:
        raise MapLifecycleError("saved map hash seal is incomplete or contains extras")
    resolved_root = root.resolve()
    for filename, expected in hashes.items():
        _artifact_basename(filename, label="hashed artifact")
        candidate = root / filename
        if (
            not isinstance(expected, str)
            or len(expected) != 64
            or candidate.is_symlink()
            or not candidate.is_file()
            or candidate.resolve().parent != resolved_root
            or sha256(candidate) != expected
        ):
            raise MapLifecycleError(f"saved map integrity check failed: {filename}")
    return manifest
