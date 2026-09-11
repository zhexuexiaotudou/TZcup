"""Truth-free map lifecycle primitives for the formal campus product path."""

from __future__ import annotations

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
    "coverage_free_space.pgm",
    "coverage_geometry.yaml",
    "mission_geometry.yaml",
    "materialization_contract.yaml",
    "geofence_keepout.yaml",
    "geofence_keepout.pgm",
    "neutral_speed.yaml",
    "neutral_speed.pgm",
})
MAPPING_POSE_SOURCE = "wheel_imu_ekf_lidar_scan_matching_gnss_consistency"
MAXIMUM_SAVED_MAP_RESOLUTION_M = 0.10
# Frozen formal cleaning geometry: max deployed footprint radius (0.620, 0.695),
# a 0.10 m safety margin and half the 1.32 m effective brush width, rounded up.
FORMAL_COVERAGE_STATIC_OBSTACLE_INFLATION_M = math.ceil((math.hypot(0.620, 0.695) + 0.10 + 1.32 / 2.0) * 100.0) / 100.0
MAXIMUM_COVERAGE_RECTANGLES = 4096


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
) -> tuple[float, float] | None:
    """Select a known-free frontier; Nav2 remains responsible for its path."""
    if width <= 2 or height <= 2 or len(data) != width * height:
        return None
    stride = max(1, round(sample_spacing_m / resolution))
    cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)
    best: tuple[float, float] | None = None
    best_score = -1.0
    for row in range(1, height - 1, stride):
        for column in range(1, width - 1, stride):
            index = row * width + column
            if not 0 <= int(data[index]) <= 25:
                continue
            if not any(
                int(data[(row + dr) * width + column + dc]) < 0
                for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1))
            ):
                continue
            local_x, local_y = (column + 0.5) * resolution, (row + 0.5) * resolution
            x = origin_x + cosine * local_x - sine * local_y
            y = origin_y + sine * local_x + cosine * local_y
            if not _inside(x, y, geofence):
                continue
            if any(
                math.hypot(x - old_x, y - old_y) < previous_goal_clearance_m
                for old_x, old_y in previous_goals
            ):
                continue
            score = math.hypot(x - robot_x, y - robot_y)
            if score > best_score:
                best, best_score = (x, y), score
    return best


_PGM_ASCII_WHITESPACE = frozenset(b" \t\r\n\v\f")


def _encode_binary_pgm(rows: list[bytearray]) -> bytes:
    height, width = len(rows), len(rows[0])
    return (
        f"P5\n{width} {height}\n255\n".encode("ascii")
        + b"".join(bytes(row) for row in reversed(rows))
    )


def _write_pgm(path: Path, rows: list[bytearray]) -> None:
    path.write_bytes(_encode_binary_pgm(rows))


def parse_binary_pgm(data: bytes) -> tuple[int, int, list[bytearray]]:
    """Parse one P5 snapshot into map-order rows (lowest y first)."""
    tokens: list[bytes] = []
    index = 0
    while len(tokens) < 4:
        while index < len(data) and data[index] in _PGM_ASCII_WHITESPACE:
            index += 1
        if index < len(data) and data[index] == ord("#"):
            newline = data.find(b"\n", index)
            if newline < 0:
                raise MapLifecycleError("saved occupancy PGM has an invalid comment")
            index = newline + 1
            continue
        end = index
        while end < len(data) and data[end] not in _PGM_ASCII_WHITESPACE:
            end += 1
        if end == index:
            raise MapLifecycleError("saved occupancy PGM header is incomplete")
        tokens.append(data[index:end])
        index = end
    if tokens[0] != b"P5":
        raise MapLifecycleError("saved occupancy image must be a binary PGM")
    if any(not token.isdigit() for token in tokens[1:]):
        raise MapLifecycleError("saved occupancy PGM header is invalid")
    try:
        width, height, maximum = (int(item) for item in tokens[1:])
    except ValueError as exc:
        raise MapLifecycleError("saved occupancy PGM header is invalid") from exc
    if data[index:index + 2] == b"\r\n":
        index += 2
    elif index < len(data) and data[index] in _PGM_ASCII_WHITESPACE:
        index += 1
    else:
        raise MapLifecycleError("saved occupancy PGM has no binary-data separator")
    pixels = data[index:]
    if width <= 0 or height <= 0 or maximum != 255 or len(pixels) != width * height:
        raise MapLifecycleError("saved occupancy PGM dimensions are invalid")
    # PGM is top-to-bottom while occupancy-map coordinates start at origin_y.
    return width, height, [
        bytearray(pixels[row * width:(row + 1) * width])
        for row in range(height - 1, -1, -1)
    ]


def read_binary_pgm(path: Path) -> tuple[int, int, list[bytearray]]:
    """Read a binary PGM exactly once and return map-order rows."""
    return parse_binary_pgm(path.read_bytes())


def assess_saved_pgm_observation(
    occupancy_metadata: Any,
    occupancy_image: bytes,
    *,
    geofence: Sequence[tuple[float, float]],
    threshold: float,
) -> GridObservation:
    """Recompute formal-field observation from a sealed trinary SLAM PGM."""
    if not isinstance(occupancy_metadata, dict):
        raise MapLifecycleError("saved occupancy metadata is invalid")
    try:
        resolution = float(occupancy_metadata["resolution"])
        origin_x, origin_y, origin_yaw = (float(value) for value in occupancy_metadata["origin"])
        occupied_threshold = float(occupancy_metadata.get("occupied_thresh", 0.65))
        free_threshold = float(occupancy_metadata.get("free_thresh", 0.25))
        negate = occupancy_metadata.get("negate", 0)
    except (KeyError, TypeError, ValueError) as exc:
        raise MapLifecycleError("saved occupancy metadata is invalid") from exc
    if (
        isinstance(negate, bool)
        or negate not in {0, 1}
        or occupancy_metadata.get("mode", "trinary") != "trinary"
        or not 0.0 < resolution <= MAXIMUM_SAVED_MAP_RESOLUTION_M
        or not all(math.isfinite(value) for value in (
            origin_x, origin_y, origin_yaw, occupied_threshold, free_threshold,
        ))
        or abs(origin_yaw) > 1e-9
        or not 0.0 <= free_threshold < occupied_threshold <= 1.0
    ):
        raise MapLifecycleError("saved occupancy metadata is outside the formal contract")
    width, height, rows = parse_binary_pgm(occupancy_image)
    data: list[int] = []
    for row in rows:
        for pixel in row:
            probability = pixel / 255.0 if negate else (255 - pixel) / 255.0
            if probability > occupied_threshold:
                data.append(100)
            elif probability < free_threshold:
                data.append(0)
            else:
                # In trinary mode the interval including both thresholds is unknown.
                data.append(-1)
    return assess_grid_observation(
        data,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=origin_yaw,
        geofence=geofence,
        threshold=threshold,
    )


def _rectangles_from_mask(
    mask: Sequence[Sequence[bool]], *, resolution: float, origin_x: float, origin_y: float
) -> list[list[list[float]]]:
    """Tile a raster exactly with rectangles; no obstacle bounding-box shortcut."""
    active: dict[tuple[int, int], list[int]] = {}
    rectangles: list[tuple[int, int, int, int]] = []
    for row, values in enumerate(mask):
        runs: list[tuple[int, int]] = []
        column = 0
        while column < len(values):
            if not values[column]:
                column += 1
                continue
            start = column
            while column < len(values) and values[column]:
                column += 1
            runs.append((start, column))
        next_active: dict[tuple[int, int], list[int]] = {}
        for run in runs:
            previous = active.pop(run, None)
            next_active[run] = [run[0], previous[1] if previous else row, run[1], row + 1]
        rectangles.extend(tuple(value) for value in active.values())
        active = next_active
    rectangles.extend(tuple(value) for value in active.values())
    if len(rectangles) > MAXIMUM_COVERAGE_RECTANGLES:
        raise MapLifecycleError("saved occupancy free space is too fragmented for coverage planning")
    return [
        [
            [origin_x + left * resolution, origin_y + bottom * resolution],
            [origin_x + right * resolution, origin_y + bottom * resolution],
            [origin_x + right * resolution, origin_y + top * resolution],
            [origin_x + left * resolution, origin_y + top * resolution],
        ]
        for left, bottom, right, top in rectangles
    ]


def _mask_loops(mask: Sequence[Sequence[bool]], *, resolution: float, origin_x: float, origin_y: float) -> list[list[list[float]]]:
    """Trace exact union boundaries of raster cells; never emit adjacent boxes."""
    height, width = len(mask), len(mask[0])
    edges: dict[tuple[int, int], list[tuple[int, int]]] = {}
    def add(start: tuple[int, int], end: tuple[int, int]) -> None:
        edges.setdefault(start, []).append(end)
    for row, values in enumerate(mask):
        for column, value in enumerate(values):
            if not value:
                continue
            if row == 0 or not mask[row - 1][column]: add((column, row), (column + 1, row))
            if column == width - 1 or not mask[row][column + 1]: add((column + 1, row), (column + 1, row + 1))
            if row == height - 1 or not mask[row + 1][column]: add((column + 1, row + 1), (column, row + 1))
            if column == 0 or not mask[row][column - 1]: add((column, row + 1), (column, row))
    incoming: dict[tuple[int, int], int] = {}
    for starts in edges.values():
        for end in starts:
            incoming[end] = incoming.get(end, 0) + 1
    if any(len(ends) != 1 or incoming.get(vertex) != 1 for vertex, ends in edges.items()):
        raise MapLifecycleError("saved occupancy rings touch at a vertex or edge")
    loops: list[list[list[float]]] = []
    while edges:
        start = next(iter(edges))
        current, loop = start, [start]
        while True:
            choices = edges.get(current)
            if not choices:
                raise MapLifecycleError("saved occupancy boundary is not a closed polygon")
            following = choices.pop()
            if not choices:
                del edges[current]
            current = following
            if current == start:
                break
            loop.append(current)
        if len(loop) < 3:
            raise MapLifecycleError("saved occupancy boundary is degenerate")
        if len(set(loop)) != len(loop):
            raise MapLifecycleError("saved occupancy boundary self-intersects")
        compact = [
            point for index, point in enumerate(loop)
            if (point[0] - loop[index - 1][0]) * (loop[(index + 1) % len(loop)][1] - point[1])
            != (point[1] - loop[index - 1][1]) * (loop[(index + 1) % len(loop)][0] - point[0])
        ]
        if len(compact) < 3:
            raise MapLifecycleError("saved occupancy boundary is degenerate")
        loops.append([[origin_x + x * resolution, origin_y + y * resolution] for x, y in compact])
    if len(loops) > MAXIMUM_COVERAGE_RECTANGLES:
        raise MapLifecycleError("saved occupancy free space is too fragmented for coverage planning")
    vertices: set[tuple[float, float]] = set()
    for loop in loops:
        current = {tuple(point) for point in loop}
        if vertices.intersection(current):
            raise MapLifecycleError("saved occupancy rings touch at a vertex or edge")
        vertices.update(current)
    return loops


def _signed_area(points: Sequence[Sequence[float]]) -> float:
    return sum(
        point[0] * following[1] - following[0] * point[1]
        for point, following in zip(points, (*points[1:], points[0]))
    ) / 2.0


def _dilate_square(mask: Sequence[Sequence[bool]], radius: int) -> list[list[bool]]:
    """Conservatively inflate occupied/unknown cells in O(width*height)."""
    height, width = len(mask), len(mask[0])
    prefix = [[0] * (width + 1) for _ in range(height + 1)]
    for row, values in enumerate(mask, start=1):
        running = 0
        for column, value in enumerate(values, start=1):
            running += int(value)
            prefix[row][column] = prefix[row - 1][column] + running
    result = [[False] * width for _ in range(height)]
    for row in range(height):
        top, bottom = max(0, row - radius), min(height, row + radius + 1)
        for column in range(width):
            left, right = max(0, column - radius), min(width, column + radius + 1)
            result[row][column] = (
                prefix[bottom][right] - prefix[top][right]
                - prefix[bottom][left] + prefix[top][left]
            ) > 0
    return result


def materialize_saved_map_coverage_geometry(
    artifact_directory: str | Path,
    contract: CampusMapContract,
    *,
    obstacle_inflation_m: float = FORMAL_COVERAGE_STATIC_OBSTACLE_INFLATION_M,
) -> dict[str, Path]:
    """Derive reachable coverage cells only from the saved SLAM occupancy map."""
    root = Path(artifact_directory)
    if not math.isfinite(obstacle_inflation_m) or obstacle_inflation_m <= 0.0:
        raise MapLifecycleError("coverage obstacle inflation must be positive and finite")
    occupancy_path = _local_artifact_path(root, "occupancy.yaml", label="occupancy map")
    mission_path = _local_artifact_path(root, "mission_geometry.yaml", label="mission geometry")
    try:
        occupancy_bytes = _read_artifact_snapshot(root, occupancy_path.name, label="occupancy map")
        mission_bytes = _read_artifact_snapshot(root, mission_path.name, label="mission geometry")
        metadata = yaml.safe_load(occupancy_bytes)
        mission = yaml.safe_load(mission_bytes)
    except (OSError, yaml.YAMLError) as exc:
        raise MapLifecycleError("saved occupancy or mission geometry is missing") from exc
    if not isinstance(metadata, dict) or not isinstance(mission, dict):
        raise MapLifecycleError("saved occupancy or mission geometry is invalid")
    image_name = _artifact_basename(metadata.get("image"), label="occupancy image")
    if image_name != "occupancy.pgm":
        raise MapLifecycleError("formal saved map must use occupancy.pgm")
    try:
        resolution = float(metadata["resolution"])
        origin_x, origin_y, origin_yaw = (float(value) for value in metadata["origin"])
        occupied_threshold = float(metadata.get("occupied_thresh", 0.65))
        free_threshold = float(metadata.get("free_thresh", 0.25))
        negate = int(metadata.get("negate", 0))
    except (KeyError, TypeError, ValueError) as exc:
        raise MapLifecycleError("saved occupancy metadata is invalid") from exc
    if (
        not 0.0 < resolution <= MAXIMUM_SAVED_MAP_RESOLUTION_M
        or not all(math.isfinite(value) for value in (origin_x, origin_y, origin_yaw, occupied_threshold, free_threshold))
        or not 0.0 <= free_threshold < occupied_threshold <= 1.0
        or negate not in {0, 1}
        or metadata.get("mode", "trinary") != "trinary"
        or abs(origin_yaw) > 1e-9
    ):
        raise MapLifecycleError("saved occupancy metadata is outside the formal contract")
    if mission.get("outer_polygon") != [list(point) for point in contract.geofence]:
        raise MapLifecycleError("saved-map mission geofence differs from the formal contract")
    truth = mission.get("truth_boundary")
    if not isinstance(truth, dict) or any(
        truth.get(name) is not False
        for name in ("world_geometry_used_for_product_map", "evaluator_truth_used", "dirt_truth_used")
    ):
        raise MapLifecycleError("saved-map coverage geometry violates truth isolation")
    image_bytes = _read_artifact_snapshot(root, image_name, label="occupancy image")
    width, height, pixels = parse_binary_pgm(image_bytes)
    blocked = [[False] * width for _ in range(height)]
    for row in range(height):
        y = origin_y + (row + 0.5) * resolution
        for column in range(width):
            x = origin_x + (column + 0.5) * resolution
            occupancy = pixels[row][column] / 255.0 if negate else (255 - pixels[row][column]) / 255.0
            blocked[row][column] = (
                not _inside(x, y, contract.geofence)
                or occupancy >= free_threshold
            )
    inflation_cells = math.ceil(obstacle_inflation_m / resolution)
    inflated = _dilate_square(blocked, inflation_cells)
    # Occupancy outside the exported PGM is unknown/blocked, never free margin.
    for row in range(height):
        for column in range(width):
            if row < inflation_cells or column < inflation_cells or row >= height - inflation_cells or column >= width - inflation_cells:
                inflated[row][column] = True
    start_column = math.floor((-origin_x) / resolution)
    start_row = math.floor((-origin_y) / resolution)
    if not (0 <= start_row < height and 0 <= start_column < width) or inflated[start_row][start_column]:
        raise MapLifecycleError("fixed saved-map start is not reachable after obstacle inflation")
    reachable = [[False] * width for _ in range(height)]
    pending = [(start_row, start_column)]
    reachable[start_row][start_column] = True
    while pending:
        row, column = pending.pop()
        for next_row, next_column in ((row - 1, column), (row + 1, column), (row, column - 1), (row, column + 1)):
            if (
                0 <= next_row < height and 0 <= next_column < width
                and not inflated[next_row][next_column] and not reachable[next_row][next_column]
            ):
                reachable[next_row][next_column] = True
                pending.append((next_row, next_column))
    reachable_cells = sum(sum(row) for row in reachable)
    if reachable_cells == 0:
        raise MapLifecycleError("saved occupancy has no reachable cleanable free space")
    free_space_path = _local_artifact_path(
        root, "coverage_free_space.pgm", label="coverage free-space map", must_exist=False
    )
    free_space_bytes = _encode_binary_pgm(
        [bytearray(255 if value else 0 for value in row) for row in reachable]
    )
    free_space_path.write_bytes(free_space_bytes)
    # Collision/telemetry consumers treat every keepout as solid, so use an
    # exact disjoint raster decomposition rather than complement boundary rings.
    keepouts = _rectangles_from_mask(
        [[_inside(origin_x + (column + 0.5) * resolution, origin_y + (row + 0.5) * resolution, contract.geofence) and not reachable[row][column]
          for column in range(width)] for row in range(height)],
        resolution=resolution, origin_x=origin_x, origin_y=origin_y,
    )
    reachable_loops = _mask_loops(reachable, resolution=resolution, origin_x=origin_x, origin_y=origin_y)
    planning_outer = [loop for loop in reachable_loops if _signed_area(loop) > 0.0]
    planning_holes = [loop for loop in reachable_loops if _signed_area(loop) < 0.0]
    if len(planning_outer) != 1 or len(planning_outer) + len(planning_holes) != len(reachable_loops):
        raise MapLifecycleError("reachable saved occupancy is not one valid outer region with holes")
    if any(
        not _inside(
            sum(point[0] for point in hole) / len(hole),
            sum(point[1] for point in hole) / len(hole),
            planning_outer[0],
        )
        for hole in planning_holes
    ):
        raise MapLifecycleError("reachable saved occupancy hole is outside its planning outer region")
    geometry_path = _local_artifact_path(
        root, "coverage_geometry.yaml", label="coverage geometry", must_exist=False
    )
    geometry = {
        "schema_version": 1,
        "source": "saved_slam_occupancy_only",
        "occupancy_map": occupancy_path.name,
        "occupancy_map_sha256": hashlib.sha256(occupancy_bytes).hexdigest(),
        "occupancy_image": image_name,
        "occupancy_image_sha256": hashlib.sha256(image_bytes).hexdigest(),
        "free_space_map": free_space_path.name,
        "free_space_map_sha256": hashlib.sha256(free_space_bytes).hexdigest(),
        "resolution_m": resolution,
        "origin": [origin_x, origin_y, origin_yaw],
        "obstacle_inflation_m": obstacle_inflation_m,
        "planning_clearance_m": obstacle_inflation_m,
        "planning_clearance_preapplied": True,
        "planning_clearance_derivation": "ceil_cm(hypot(0.620,0.695)+0.10+1.32/2)",
        "reachable_cleanable_cells": reachable_cells,
        "reachable_cleanable_area_m2": reachable_cells * resolution * resolution,
        "planning_polygons": reachable_loops,
        "planning_outer_polygon": planning_outer[0],
        "planning_hole_polygons": planning_holes,
        "keepout_polygons": keepouts,
        "world_truth_used_for_product_map": False,
    }
    geometry_bytes = yaml.safe_dump(geometry, sort_keys=False).encode("utf-8")
    geometry_path.write_bytes(geometry_bytes)
    mission.update({
        "keepout_polygons": keepouts,
        "exclusion_polygons": keepouts,
        "saved_occupancy_coverage": {
            "geometry": geometry_path.name,
            "sha256": hashlib.sha256(geometry_bytes).hexdigest(),
            "free_space_map": free_space_path.name,
            "source": "saved_slam_occupancy_only",
            "planning_clearance_preapplied": True,
        },
    })
    mission_path.write_bytes(yaml.safe_dump(mission, sort_keys=False).encode("utf-8"))
    return {"coverage_geometry": geometry_path, "coverage_free_space": free_space_path, "mission_geometry": mission_path}


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
        "headland": {"enabled": True, "width_m": FORMAL_COVERAGE_STATIC_OBSTACLE_INFLATION_M},
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


def _local_artifact_path(
    root: Path, value: Any, *, label: str, must_exist: bool = True
) -> Path:
    """Resolve one fixed sibling without allowing links or path traversal."""
    name = _artifact_basename(value, label=label)
    candidate = root / name
    if (
        candidate.is_symlink()
        or (must_exist and not candidate.is_file())
        or candidate.resolve(strict=False).parent != root.resolve()
    ):
        raise MapLifecycleError(f"{label} must be a regular local artifact")
    return candidate


def _read_artifact_snapshot(root: Path, value: Any, *, label: str) -> bytes:
    """Read one checked artifact exactly once for both parsing and hashing."""
    try:
        return _local_artifact_path(root, value, label=label).read_bytes()
    except OSError as exc:
        raise MapLifecycleError(f"{label} is missing or unreadable") from exc


def validate_saved_map_artifact(
    artifact_directory: str | Path, contract: CampusMapContract
) -> dict[str, Any]:
    root = Path(artifact_directory)
    try:
        manifest = json.loads(_read_artifact_snapshot(
            root, "map_lifecycle_manifest.json", label="saved-map lifecycle manifest"
        ))
    except (MapLifecycleError, json.JSONDecodeError) as exc:
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
    image_name = "occupancy.pgm"
    required_files = {
        occupancy_name,
        image_name,
        *REQUIRED_SAVED_MAP_SUPPORT_FILES,
    }
    hashes = manifest.get("sha256")
    if not isinstance(hashes, dict) or set(hashes) != required_files:
        raise MapLifecycleError("saved map hash seal is incomplete or contains extras")
    snapshots: dict[str, bytes] = {}
    for filename, expected in hashes.items():
        try:
            snapshot = _read_artifact_snapshot(root, filename, label="hashed artifact")
        except MapLifecycleError as exc:
            raise MapLifecycleError(f"saved map integrity check failed: {filename}") from exc
        if not isinstance(expected, str) or hashlib.sha256(snapshot).hexdigest() != expected:
            raise MapLifecycleError(f"saved map integrity check failed: {filename}")
        snapshots[filename] = snapshot
    try:
        occupancy_metadata = yaml.safe_load(snapshots[occupancy_name])
    except yaml.YAMLError as exc:
        raise MapLifecycleError("saved occupancy metadata is missing or invalid") from exc
    if not isinstance(occupancy_metadata, dict):
        raise MapLifecycleError("saved occupancy metadata is invalid")
    sealed_image_name = _artifact_basename(
        occupancy_metadata.get("image"), label="occupancy image"
    )
    if sealed_image_name != image_name:
        raise MapLifecycleError("formal saved map must use occupancy.pgm")
    pgm_observation = assess_saved_pgm_observation(
        occupancy_metadata,
        snapshots[image_name],
        geofence=contract.geofence,
        threshold=quality_threshold,
    )
    if (
        not pgm_observation.passed
        or not math.isclose(observed_fraction, pgm_observation.observed_fraction, abs_tol=1e-12)
        or manifest.get("saved_pgm_observed_fraction") != pgm_observation.observed_fraction
        or manifest.get("saved_pgm_observed_cells") != pgm_observation.observed_cells
        or manifest.get("saved_pgm_field_cells") != pgm_observation.field_cells
    ):
        raise MapLifecycleError("saved occupancy PGM observation does not match the lifecycle manifest")
    try:
        mission = yaml.safe_load(snapshots["mission_geometry.yaml"])
        coverage = mission["saved_occupancy_coverage"]
        geometry_name = _artifact_basename(coverage["geometry"], label="coverage geometry")
        free_name = _artifact_basename(coverage["free_space_map"], label="coverage free-space map")
        geometry = yaml.safe_load(snapshots[geometry_name])
        _, _, free_rows = parse_binary_pgm(snapshots[free_name])
    except (yaml.YAMLError, KeyError, TypeError, MapLifecycleError) as exc:
        raise MapLifecycleError("saved occupancy coverage geometry is missing or invalid") from exc
    free_cell_count = sum(pixel == 255 for row in free_rows for pixel in row)
    binary_free_map = all(pixel in (0, 255) for row in free_rows for pixel in row)
    if (
        geometry_name != "coverage_geometry.yaml"
        or free_name != "coverage_free_space.pgm"
        or coverage.get("sha256") != hashes.get(geometry_name)
        or not isinstance(geometry, dict)
        or geometry.get("source") != "saved_slam_occupancy_only"
        or geometry.get("occupancy_map") != occupancy_name
        or geometry.get("occupancy_image") != image_name
        or geometry.get("occupancy_map_sha256") != hashes.get(occupancy_name)
        or geometry.get("occupancy_image_sha256") != hashes.get(image_name)
        or geometry.get("free_space_map") != free_name
        or geometry.get("free_space_map_sha256") != hashes.get(free_name)
        or geometry.get("planning_clearance_m") != FORMAL_COVERAGE_STATIC_OBSTACLE_INFLATION_M
        or geometry.get("obstacle_inflation_m") != FORMAL_COVERAGE_STATIC_OBSTACLE_INFLATION_M
        or geometry.get("planning_clearance_preapplied") is not True
        or coverage.get("planning_clearance_preapplied") is not True
        or not isinstance(geometry.get("planning_clearance_derivation"), str)
        or mission.get("headland") != {"enabled": True, "width_m": FORMAL_COVERAGE_STATIC_OBSTACLE_INFLATION_M}
        or geometry.get("world_truth_used_for_product_map") is not False
        or mission.get("keepout_polygons") != geometry.get("keepout_polygons")
        or mission.get("exclusion_polygons") != geometry.get("keepout_polygons")
        or not isinstance(geometry.get("planning_outer_polygon"), list)
        or len(geometry["planning_outer_polygon"]) < 3
        or not isinstance(geometry.get("planning_hole_polygons"), list)
        or not isinstance(geometry.get("reachable_cleanable_cells"), int)
        or geometry["reachable_cleanable_cells"] <= 0
        or not binary_free_map
        or free_cell_count != geometry["reachable_cleanable_cells"]
    ):
        raise MapLifecycleError("saved occupancy coverage geometry violates the formal contract")
    return manifest
