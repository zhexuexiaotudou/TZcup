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
    "mission_geometry.yaml",
    "materialization_contract.yaml",
    "geofence_keepout.yaml",
    "geofence_keepout.pgm",
    "neutral_speed.yaml",
    "neutral_speed.pgm",
})
MAPPING_POSE_SOURCE = "wheel_imu_ekf_lidar_scan_matching_gnss_consistency"


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


def load_campus_map_contract(path: str | Path) -> CampusMapContract:
    """Load public mission geometry, never evaluator/world object truth."""
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise MapLifecycleError("unable to read public episode manifest") from exc
    if not isinstance(value, dict) or value.get("profile") != "formal":
        raise MapLifecycleError("map lifecycle requires a formal public episode")
    field = value.get("field")
    start_value = value.get("vehicle_start_pose_map")
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
        source_polygon = tuple(
            (float(point[0]), float(point[1]))
            for point in field["geofence_polygon_m"]
        )
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
    local = tuple(_local_point(point, source) for point in source_polygon)
    return CampusMapContract(
        episode_id=str(value.get("episode_id", "")),
        map_id=str(value.get("map_id", "")),
        field_area_m2=area,
        geofence=local,
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
    if not math.isfinite(resolution) or resolution <= 0.0 or resolution > 0.10:
        raise MapLifecycleError("formal SLAM resolution must be in (0, 0.10] m")
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
