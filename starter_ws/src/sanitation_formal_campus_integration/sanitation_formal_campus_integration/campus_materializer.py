"""Materialize deployable campus maps from public episode inputs only."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Iterable
import xml.etree.ElementTree as ET

import yaml

from sanitation_formal_campus_integration.contract import (
    CANONICAL_PLANNING_KINEMATIC_CONSTRAINT,
    IntegrationContractError,
    formal_motion_values,
    load_yaml_mapping,
)


Point = tuple[float, float]
Polygon = list[Point]
EVALUATOR_ONLY_KEYS = {
    "generator_version",
    "runtime_environment",
    "seeds",
    "sensor_randomization",
    "truth_boundary",
}


@dataclass(frozen=True)
class GridSpec:
    """One shared map coordinate contract for every generated raster."""

    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int
    geofence: tuple[Point, ...]


@dataclass(frozen=True)
class StaticCollision:
    """Public collision geometry extracted from the scenario world."""

    name: str
    shape: str
    center: Point
    yaw: float
    size: tuple[float, float]


@dataclass(frozen=True)
class MaterializedCampusArtifacts:
    """Paths and map-frame values consumed by the integration launch."""

    occupancy_map: Path
    keepout_map: Path
    speed_map: Path
    mission_geometry: Path
    contract_report: Path
    start_pose: tuple[float, float, float]
    grid: GridSpec
    static_collision_count: int
    world_name: str


def _load_public_manifest(path: str | Path) -> dict[str, Any]:
    manifest_path = Path(path)
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise IntegrationContractError(
            f"unable to read public episode manifest: {manifest_path}"
        ) from exc
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1:
        raise IntegrationContractError("public episode manifest schema_version must be 1")
    leaked = sorted(EVALUATOR_ONLY_KEYS.intersection(manifest))
    if leaked:
        raise IntegrationContractError(
            f"evaluator-only manifest keys are prohibited: {', '.join(leaked)}"
        )
    required = {
        "episode_id",
        "profile",
        "field",
        "counts",
        "vehicle_start_pose_map",
    }
    missing = sorted(required.difference(manifest))
    if missing:
        raise IntegrationContractError(
            f"public episode manifest is incomplete: {', '.join(missing)}"
        )
    if manifest.get("profile") != "formal":
        raise IntegrationContractError("formal campus integration requires profile=formal")
    return manifest


def _pose_values(element: ET.Element | None) -> tuple[float, ...]:
    text = "" if element is None or element.text is None else element.text
    try:
        values = tuple(float(value) for value in text.split())
    except ValueError as exc:
        raise IntegrationContractError(f"invalid SDF pose: {text!r}") from exc
    if not values:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise IntegrationContractError(f"SDF pose must contain six finite values: {text!r}")
    return values


def _float_pair(text: str | None, label: str) -> tuple[float, float]:
    try:
        values = tuple(float(value) for value in (text or "").split())
    except ValueError as exc:
        raise IntegrationContractError(f"invalid {label} geometry") from exc
    if len(values) < 2 or not all(math.isfinite(value) and value > 0 for value in values[:2]):
        raise IntegrationContractError(f"invalid {label} geometry")
    return values[0], values[1]


def extract_static_collisions(world_path: str | Path) -> tuple[str, list[StaticCollision]]:
    """Read static deployable collision footprints without evaluator metadata."""
    try:
        root = ET.parse(Path(world_path)).getroot()
    except (OSError, ET.ParseError) as exc:
        raise IntegrationContractError(f"unable to read scenario world: {world_path}") from exc
    world = root.find("world")
    if world is None or not world.get("name"):
        raise IntegrationContractError("scenario SDF has no named world")
    collisions: list[StaticCollision] = []
    for model in world.findall("model"):
        if (model.findtext("static") or "false").strip().lower() != "true":
            continue
        model_name = model.get("name") or "unnamed_static_model"
        # The scenario driver moves these public environment actors with
        # SetEntityPose even though Gazebo requires their SDF bodies to be
        # declared static.  Their public ``walker_`` role is therefore the
        # authoritative dynamic marker; height is not a safe discriminator
        # because a legitimate bench or kerb may also have a ground-level
        # model origin.
        if model_name.startswith("walker_"):
            continue
        if not model.findall("./link/collision"):
            # Dirt is represented as visuals only.  Do not even parse its pose
            # while producing static navigation artifacts.
            continue
        model_pose = _pose_values(model.find("pose"))
        model_collision_count = 0
        for link in model.findall("link"):
            link_pose = _pose_values(link.find("pose"))
            for collision in link.findall("collision"):
                collision_pose = _pose_values(collision.find("pose"))
                center = (
                    model_pose[0] + math.cos(model_pose[5]) * (link_pose[0] + collision_pose[0])
                    - math.sin(model_pose[5]) * (link_pose[1] + collision_pose[1]),
                    model_pose[1] + math.sin(model_pose[5]) * (link_pose[0] + collision_pose[0])
                    + math.cos(model_pose[5]) * (link_pose[1] + collision_pose[1]),
                )
                yaw = model_pose[5] + link_pose[5] + collision_pose[5]
                geometry = collision.find("geometry")
                if geometry is None:
                    raise IntegrationContractError(
                        f"static collision has no geometry: {model_name}"
                    )
                box = geometry.find("box")
                cylinder = geometry.find("cylinder")
                if box is not None:
                    size = _float_pair(box.findtext("size"), "box")
                    shape = "box"
                elif cylinder is not None:
                    try:
                        radius = float(cylinder.findtext("radius") or "")
                    except ValueError as exc:
                        raise IntegrationContractError("invalid cylinder geometry") from exc
                    if not math.isfinite(radius) or radius <= 0:
                        raise IntegrationContractError("invalid cylinder geometry")
                    size = (2.0 * radius, 2.0 * radius)
                    shape = "cylinder"
                elif geometry.find("plane") is not None:
                    continue
                else:
                    raise IntegrationContractError(
                        f"unsupported static collision geometry: {model_name}"
                    )
                suffix = "" if model_collision_count == 0 else f"_{model_collision_count}"
                collisions.append(
                    StaticCollision(
                        name=f"{model_name}{suffix}",
                        shape=shape,
                        center=center,
                        yaw=yaw,
                        size=size,
                    )
                )
                model_collision_count += 1
    return world.get("name", ""), collisions


def _validated_grid(manifest: dict[str, Any], resolution: float) -> GridSpec:
    if not math.isfinite(resolution) or resolution <= 0 or resolution > 0.5:
        raise IntegrationContractError("map resolution must be in (0, 0.5] metres")
    field = manifest.get("field")
    if not isinstance(field, dict) or field.get("geofence_frame") != "map":
        raise IntegrationContractError("public geofence must use the map frame")
    raw_polygon = field.get("geofence_polygon_m")
    if not isinstance(raw_polygon, list) or len(raw_polygon) != 4:
        raise IntegrationContractError("public geofence must be a four-point rectangle")
    try:
        polygon = tuple((float(point[0]), float(point[1])) for point in raw_polygon)
    except (IndexError, TypeError, ValueError) as exc:
        raise IntegrationContractError("public geofence contains invalid points") from exc
    if not all(math.isfinite(value) for point in polygon for value in point):
        raise IntegrationContractError("public geofence contains non-finite points")
    xs = sorted({point[0] for point in polygon})
    ys = sorted({point[1] for point in polygon})
    if len(xs) != 2 or len(ys) != 2 or set(polygon) != {
        (xs[0], ys[0]),
        (xs[1], ys[0]),
        (xs[1], ys[1]),
        (xs[0], ys[1]),
    }:
        raise IntegrationContractError("public geofence must be axis-aligned and rectangular")
    width_m = xs[1] - xs[0]
    height_m = ys[1] - ys[0]
    if not math.isclose(float(field.get("width_m", -1)), width_m, abs_tol=1e-6):
        raise IntegrationContractError("public field width and geofence disagree")
    if not math.isclose(float(field.get("height_m", -1)), height_m, abs_tol=1e-6):
        raise IntegrationContractError("public field height and geofence disagree")
    # One outer cell on each side makes the physical geofence visible as a
    # lethal band instead of relying on undefined behavior outside the image.
    width = math.ceil(width_m / resolution) + 2
    height = math.ceil(height_m / resolution) + 2
    return GridSpec(
        origin_x=xs[0] - resolution,
        origin_y=ys[0] - resolution,
        resolution=resolution,
        width=width,
        height=height,
        geofence=polygon,
    )


def _collision_polygon(collision: StaticCollision, margin: float) -> Polygon:
    cx, cy = collision.center
    if collision.shape == "cylinder":
        radius = collision.size[0] / 2.0 + margin
        return [
            (
                cx + radius * math.cos(collision.yaw + 2.0 * math.pi * index / 16.0),
                cy + radius * math.sin(collision.yaw + 2.0 * math.pi * index / 16.0),
            )
            for index in range(16)
        ]
    half_x = collision.size[0] / 2.0 + margin
    half_y = collision.size[1] / 2.0 + margin
    cosine = math.cos(collision.yaw)
    sine = math.sin(collision.yaw)
    return [
        (cx + cosine * x - sine * y, cy + sine * x + cosine * y)
        for x, y in ((-half_x, -half_y), (half_x, -half_y), (half_x, half_y), (-half_x, half_y))
    ]


def _point_in_polygon(x: float, y: float, polygon: Iterable[Point]) -> bool:
    points = list(polygon)
    inside = False
    previous = points[-1]
    for current in points:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y) and x < (
            (x2 - x1) * (y - y1) / ((y2 - y1) or 1e-12) + x1
        ):
            inside = not inside
        previous = current
    return inside


def _distance_to_polygon(x: float, y: float, polygon: Polygon) -> float:
    distances = []
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        denominator = dx * dx + dy * dy
        scale = 0.0 if denominator == 0 else (
            (x - start[0]) * dx + (y - start[1]) * dy
        ) / denominator
        scale = min(1.0, max(0.0, scale))
        distances.append(math.hypot(x - start[0] - scale * dx, y - start[1] - scale * dy))
    return min(distances)


def _blank_grid(spec: GridSpec) -> list[bytearray]:
    return [bytearray(spec.width) for _ in range(spec.height)]


def _mark_polygon(grid: list[bytearray], spec: GridSpec, polygon: Polygon, value: int) -> None:
    conservative_pad = spec.resolution / math.sqrt(2.0)
    min_x = min(point[0] for point in polygon) - conservative_pad
    max_x = max(point[0] for point in polygon) + conservative_pad
    min_y = min(point[1] for point in polygon) - conservative_pad
    max_y = max(point[1] for point in polygon) + conservative_pad
    first_column = max(0, math.floor((min_x - spec.origin_x) / spec.resolution))
    last_column = min(spec.width - 1, math.floor((max_x - spec.origin_x) / spec.resolution))
    first_row = max(0, math.floor((min_y - spec.origin_y) / spec.resolution))
    last_row = min(spec.height - 1, math.floor((max_y - spec.origin_y) / spec.resolution))
    for row in range(first_row, last_row + 1):
        y = spec.origin_y + (row + 0.5) * spec.resolution
        for column in range(first_column, last_column + 1):
            x = spec.origin_x + (column + 0.5) * spec.resolution
            if (
                _point_in_polygon(x, y, polygon)
                or _distance_to_polygon(x, y, polygon) <= conservative_pad
            ):
                grid[row][column] = max(grid[row][column], value)


def _mark_geofence_boundary(
    grid: list[bytearray], spec: GridSpec, width_m: float, value: int
) -> None:
    min_x = min(point[0] for point in spec.geofence)
    max_x = max(point[0] for point in spec.geofence)
    min_y = min(point[1] for point in spec.geofence)
    max_y = max(point[1] for point in spec.geofence)
    for row in range(spec.height):
        y = spec.origin_y + (row + 0.5) * spec.resolution
        for column in range(spec.width):
            x = spec.origin_x + (column + 0.5) * spec.resolution
            inside_clear = (
                min_x + width_m < x < max_x - width_m
                and min_y + width_m < y < max_y - width_m
            )
            if not inside_clear:
                grid[row][column] = max(grid[row][column], value)


def _grid_value(grid: list[bytearray], spec: GridSpec, x: float, y: float) -> int | None:
    column = math.floor((x - spec.origin_x) / spec.resolution)
    row = math.floor((y - spec.origin_y) / spec.resolution)
    if not (0 <= column < spec.width and 0 <= row < spec.height):
        return None
    return int(grid[row][column])


def _write_pgm(path: Path, grid: list[bytearray]) -> None:
    height = len(grid)
    width = len(grid[0]) if grid else 0
    # Map values use 0=free and 100=restricted. PGM uses white=free because
    # every emitted YAML sets negate: 0.
    pixels = b"".join(
        bytes(255 - round(value * 255 / 100) for value in row)
        for row in reversed(grid)
    )
    path.write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + pixels)


def _write_map_yaml(path: Path, image_name: str, spec: GridSpec, mode: str) -> None:
    payload = {
        "image": image_name,
        "mode": mode,
        "resolution": spec.resolution,
        "origin": [spec.origin_x, spec.origin_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def _sha256(path: str | Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _safe_identifier(value: object) -> str:
    sanitized = re.sub(r"[^A-Za-z0-9_.-]+", "-", str(value)).strip("-.")
    if not sanitized:
        raise IntegrationContractError("episode_id cannot form a safe artifact identifier")
    return sanitized


def _validate_start_pose(
    manifest: dict[str, Any],
    pose_override: tuple[float, float, float] | None,
) -> tuple[float, float, float]:
    source = manifest.get("vehicle_start_pose_map")
    if not isinstance(source, dict):
        raise IntegrationContractError("public manifest has no vehicle_start_pose_map")
    try:
        pose = (
            float(source["x_m"]),
            float(source["y_m"]),
            float(source.get("yaw_rad", 0.0)),
        ) if pose_override is None else tuple(float(value) for value in pose_override)
    except (KeyError, TypeError, ValueError) as exc:
        raise IntegrationContractError("vehicle start pose is invalid") from exc
    if len(pose) != 3 or not all(math.isfinite(value) for value in pose):
        raise IntegrationContractError("vehicle start pose must contain three finite values")
    return pose


def materialize_campus_artifacts(
    episode_manifest_path: str | Path,
    world_path: str | Path,
    motion_profile_path: str | Path,
    output_directory: str | Path,
    *,
    resolution: float = 0.10,
    safety_margin_m: float = 0.15,
    slow_zone_width_m: float = 1.0,
    slow_zone_percent: int = 50,
    start_pose_override: tuple[float, float, float] | None = None,
) -> MaterializedCampusArtifacts:
    """Build map/filter/mission files with a single fail-closed contract."""
    if not math.isfinite(safety_margin_m) or safety_margin_m < 0:
        raise IntegrationContractError("safety margin must be finite and non-negative")
    if not math.isfinite(slow_zone_width_m) or slow_zone_width_m < 0:
        raise IntegrationContractError("slow zone width must be finite and non-negative")
    if not isinstance(slow_zone_percent, int) or not 1 <= slow_zone_percent <= 100:
        raise IntegrationContractError("slow zone percent must be an integer in [1, 100]")
    manifest = _load_public_manifest(episode_manifest_path)
    vehicle_contract = manifest.get("vehicle")
    if not isinstance(vehicle_contract, dict):
        raise IntegrationContractError("public manifest vehicle contract is missing")
    if vehicle_contract.get("included") is not False:
        raise IntegrationContractError(
            "public world must not include a proxy vehicle; the formal URDF is launched separately"
        )
    if vehicle_contract.get("urdf_claim") is not False:
        raise IntegrationContractError("public scenario cannot claim an embedded vehicle URDF")
    spec = _validated_grid(manifest, float(resolution))
    world_name, collisions = extract_static_collisions(world_path)
    if world_name != f"campus_{manifest['profile']}":
        raise IntegrationContractError(
            "public world name disagrees with the formal profile contract"
        )
    expected_count = manifest.get("counts", {}).get("static_assets")
    if not isinstance(expected_count, int) or expected_count < 0:
        raise IntegrationContractError("public static asset count is invalid")
    if len(collisions) != expected_count:
        raise IntegrationContractError(
            "public world static collision count disagrees with manifest: "
            f"world={len(collisions)}, manifest={expected_count}"
        )
    navigation_footprint, cleaning_width = formal_motion_values(motion_profile_path)
    profile = load_yaml_mapping(motion_profile_path)
    cleaning_footprint_raw = (
        profile.get("motion_footprints", {})
        .get("cleaning_deployed", {})
        .get("footprint_xy_m")
    )
    if not isinstance(cleaning_footprint_raw, list) or len(cleaning_footprint_raw) < 3:
        raise IntegrationContractError("formal cleaning footprint is missing or invalid")
    cleaning_footprint = [
        [float(value) for value in point] for point in cleaning_footprint_raw
    ]
    if not all(
        len(point) == 2 and all(math.isfinite(value) for value in point)
        for point in cleaning_footprint
    ):
        raise IntegrationContractError("formal cleaning footprint contains invalid points")
    footprint_radius = max(math.hypot(*point) for point in navigation_footprint)
    cleaning_footprint_radius = max(
        math.hypot(*point) for point in cleaning_footprint
    )
    keepout_inflation = footprint_radius + safety_margin_m
    # Coverage is performed with the deployed cleaning envelope, which is
    # wider than the transport footprint used by Nav2.  Its headland must meet
    # the coverage compiler's own clearance equation exactly.
    headland = cleaning_footprint_radius + safety_margin_m + cleaning_width / 2.0

    occupancy = _blank_grid(spec)
    keepout = _blank_grid(spec)
    speed = _blank_grid(spec)
    _mark_geofence_boundary(occupancy, spec, spec.resolution, 100)
    _mark_geofence_boundary(keepout, spec, keepout_inflation, 100)
    obstacle_polygons: list[Polygon] = []
    public_obstacles = []
    for collision in collisions:
        physical = _collision_polygon(collision, 0.0)
        inflated = _collision_polygon(collision, keepout_inflation)
        slow = _collision_polygon(
            collision, keepout_inflation + slow_zone_width_m
        )
        _mark_polygon(occupancy, spec, physical, 100)
        _mark_polygon(keepout, spec, inflated, 100)
        _mark_polygon(speed, spec, slow, slow_zone_percent)
        obstacle_polygons.append(inflated)
        public_obstacles.append(
            {
                "name": collision.name,
                "source": "public_world_static_collision",
                "shape": collision.shape,
                "center_map_m": list(collision.center),
                "yaw_rad": collision.yaw,
                "size_xy_m": list(collision.size),
                "inflated_keepout_polygon_m": [list(point) for point in inflated],
            }
        )
    # Never attach a speed limit to a lethal cell; this keeps filter semantics
    # unambiguous when asset inflation and slow zones overlap.
    for row in range(spec.height):
        for column in range(spec.width):
            if keepout[row][column]:
                speed[row][column] = 0

    start_pose = _validate_start_pose(manifest, start_pose_override)
    if _grid_value(occupancy, spec, start_pose[0], start_pose[1]) != 0:
        raise IntegrationContractError("vehicle start pose is outside the free occupancy map")
    if _grid_value(keepout, spec, start_pose[0], start_pose[1]) != 0:
        raise IntegrationContractError("vehicle start pose intersects a formal keepout region")

    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    occupancy_pgm = output / "occupancy.pgm"
    keepout_pgm = output / "keepout_mask.pgm"
    speed_pgm = output / "speed_mask.pgm"
    occupancy_yaml = output / "occupancy.yaml"
    keepout_yaml = output / "keepout_mask.yaml"
    speed_yaml = output / "speed_mask.yaml"
    mission_yaml = output / "mission_geometry.yaml"
    report_yaml = output / "materialization_contract.yaml"
    _write_pgm(occupancy_pgm, occupancy)
    _write_pgm(keepout_pgm, keepout)
    _write_pgm(speed_pgm, speed)
    _write_map_yaml(occupancy_yaml, occupancy_pgm.name, spec, "trinary")
    _write_map_yaml(keepout_yaml, keepout_pgm.name, spec, "scale")
    _write_map_yaml(speed_yaml, speed_pgm.name, spec, "scale")

    mission = {
        "schema_version": 1,
        "mission_id": f"formal-campus-{_safe_identifier(manifest['episode_id'])}",
        "mode": "coverage",
        "route_mode": "AREA_FILL",
        "frame_id": "map",
        "coverage_planner_profile": "SKID_STEER_OPTIMIZED",
        "kinematic_model": "four_wheel_skid_steer",
        "planning_kinematic_constraint": CANONICAL_PLANNING_KINEMATIC_CONSTRAINT,
        "physical_steering_claim": False,
        "operation_width_m": cleaning_width,
        "planning_swath_spacing_m": round(cleaning_width * 0.80, 6),
        "outer_polygon": [list(point) for point in spec.geofence],
        "exclusion_polygons": [],
        "keepout_polygons": [
            [list(point) for point in polygon] for polygon in obstacle_polygons
        ],
        "headland": {"enabled": True, "width_m": headland},
        "safety_margin_m": safety_margin_m,
        "world_to_map_translation": [0.0, 0.0],
        "robot_footprint": cleaning_footprint,
        "vehicle_start_pose_map": {
            "x_m": start_pose[0],
            "y_m": start_pose[1],
            "yaw_rad": start_pose[2],
        },
        "materialized_static_obstacles": public_obstacles,
        "truth_boundary": {
            "inputs": [
                "public/episode_manifest.json",
                "public/world.sdf",
                "formal_motion_cleaning_profile.yaml",
            ],
            "evaluator_truth_used": False,
            "dirt_truth_used": False,
        },
    }
    mission_yaml.write_text(yaml.safe_dump(mission, sort_keys=False), encoding="utf-8")
    report = {
        "schema_version": 1,
        "episode_id": manifest["episode_id"],
        "world_name": world_name,
        "source_sha256": {
            "public_episode_manifest": _sha256(episode_manifest_path),
            "public_world": _sha256(world_path),
            "formal_motion_profile": _sha256(motion_profile_path),
        },
        "grid": {
            "frame_id": "map",
            "origin": [spec.origin_x, spec.origin_y, 0.0],
            "resolution_m": spec.resolution,
            "width_cells": spec.width,
            "height_cells": spec.height,
        },
        "static_collision_count": len(collisions),
        "formal_navigation_footprint_radius_m": footprint_radius,
        "formal_cleaning_footprint_radius_m": cleaning_footprint_radius,
        "keepout_inflation_m": keepout_inflation,
        "slow_zone": {
            "additional_width_m": slow_zone_width_m,
            "speed_limit_percent": slow_zone_percent,
        },
        "vehicle_start_pose_map": mission["vehicle_start_pose_map"],
        "artifacts": {
            "occupancy_map": occupancy_yaml.name,
            "keepout_map": keepout_yaml.name,
            "speed_map": speed_yaml.name,
            "mission_geometry": mission_yaml.name,
        },
        "evaluator_truth_used": False,
        "dirt_truth_used": False,
    }
    report_yaml.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    return MaterializedCampusArtifacts(
        occupancy_map=occupancy_yaml,
        keepout_map=keepout_yaml,
        speed_map=speed_yaml,
        mission_geometry=mission_yaml,
        contract_report=report_yaml,
        start_pose=start_pose,
        grid=spec,
        static_collision_count=len(collisions),
        world_name=world_name,
    )
