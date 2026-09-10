"""ROS-independent evidence helpers for saved-map product coverage."""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Sequence

import yaml

from .map_lifecycle_core import (
    FORMAL_COVERAGE_STATIC_OBSTACLE_INFLATION_M,
    MapLifecycleError,
    REQUIRED_SAVED_MAP_SUPPORT_FILES,
    _artifact_basename,
    _read_artifact_snapshot,
    parse_binary_pgm,
)


FORMAL_OPERATION_WIDTH_M = 1.32
MAPPING_SAFE_SPEED_PROFILE = "mapping_safe"
DRY_CLEANING_SPEED_PROFILE = "dry_cleaning_competition_candidate"
WET_PUDDLE_SPEED_PROFILE = "wet_puddle_recovery"


@dataclass(frozen=True)
class FormalOperationSpeedProfile:
    """One explicit runtime speed profile, distinct from acceptance status."""

    name: str
    maximum_linear_speed_mps: float


FORMAL_MAX_LINEAR_SPEED_MPS = 0.45


class SavedMapCoverageError(RuntimeError):
    """Raised when product coverage evidence is incomplete or inconsistent."""


@dataclass(frozen=True)
class SavedMapCoverageGeometry:
    outer_polygon: tuple[tuple[float, float], ...]
    planning_outer_polygon: tuple[tuple[float, float], ...]
    planning_hole_polygons: tuple[tuple[tuple[float, float], ...], ...]
    keepout_polygons: tuple[tuple[tuple[float, float], ...], ...]
    free_cells: frozenset[tuple[int, int]]
    raster_resolution_m: float
    origin_x: float
    origin_y: float
    planning_clearance_m: float
    sha256: str


def load_formal_operation_speed_profile(
    path: str | Path, profile_name: str
) -> FormalOperationSpeedProfile:
    """Load the selectable dry/mapping runtime profile without awarding acceptance.

    Wet recovery remains a depth-segmented hydraulic mode, so it deliberately
    cannot be selected for this dry saved-map coverage executor.
    """
    try:
        document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        profiles = document["profiles"]
        profile = profiles[profile_name]
    except (OSError, yaml.YAMLError, KeyError, TypeError) as exc:
        raise SavedMapCoverageError("formal operation speed profile is missing or invalid") from exc
    if profile_name == WET_PUDDLE_SPEED_PROFILE:
        raise SavedMapCoverageError(
            "wet puddle recovery requires its depth-segmented hydraulic controller"
        )
    if profile_name not in {MAPPING_SAFE_SPEED_PROFILE, DRY_CLEANING_SPEED_PROFILE}:
        raise SavedMapCoverageError("unknown formal operation speed profile")
    speed_key = (
        "maximum_linear_speed_m_s"
        if profile_name == MAPPING_SAFE_SPEED_PROFILE
        else "target_linear_speed_m_s"
    )
    value = profile.get(speed_key) if isinstance(profile, dict) else None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SavedMapCoverageError("formal operation speed profile has no numeric speed")
    speed = float(value)
    if not math.isfinite(speed) or speed <= 0.0:
        raise SavedMapCoverageError("formal operation speed profile speed is invalid")
    expected = 0.45 if profile_name == MAPPING_SAFE_SPEED_PROFILE else 1.0
    if not math.isclose(speed, expected, abs_tol=1e-12):
        raise SavedMapCoverageError("formal operation speed profile changed its frozen speed")
    if profile_name == DRY_CLEANING_SPEED_PROFILE:
        if profile.get("enabled_for_formal_runtime") is not True:
            raise SavedMapCoverageError("dry cleaning runtime profile is not enabled")
        if profile.get("competition_efficiency_eligible") is not False:
            raise SavedMapCoverageError("dry cleaning profile cannot claim efficiency acceptance")
    return FormalOperationSpeedProfile(profile_name, speed)


def polygon_area(points: Sequence[tuple[float, float]]) -> float:
    if len(points) < 3:
        raise SavedMapCoverageError("coverage polygon requires at least three points")
    area = abs(sum(
        x1 * y2 - x2 * y1
        for (x1, y1), (x2, y2) in zip(points, (*points[1:], points[0]))
    )) / 2.0
    if not math.isfinite(area) or area <= 0.0:
        raise SavedMapCoverageError("coverage polygon area must be positive")
    return area


def point_in_polygon(
    x: float, y: float, polygon: Sequence[tuple[float, float]]
) -> bool:
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


def _polygon(raw: object, label: str) -> tuple[tuple[float, float], ...]:
    try:
        polygon = tuple((float(point[0]), float(point[1])) for point in raw)  # type: ignore[arg-type]
    except (TypeError, ValueError, IndexError) as exc:
        raise SavedMapCoverageError(f"{label} is missing or invalid") from exc
    if not all(math.isfinite(item) for point in polygon for item in point):
        raise SavedMapCoverageError(f"{label} contains non-finite coordinates")
    polygon_area(polygon)
    return polygon


def _sealed_snapshot(
    root: Path, name: str, hashes: dict[str, object], label: str
) -> bytes:
    try:
        snapshot = _read_artifact_snapshot(root, name, label=label)
    except MapLifecycleError as exc:
        raise SavedMapCoverageError(f"{label} is missing or invalid") from exc
    expected = hashes.get(name)
    if not isinstance(expected, str) or hashlib.sha256(snapshot).hexdigest() != expected:
        raise SavedMapCoverageError(f"{label} is not sealed by the lifecycle manifest")
    return snapshot


def load_product_mission_geometry(path: str | Path) -> SavedMapCoverageGeometry:
    mission_path = Path(path)
    root = mission_path.parent
    try:
        if _artifact_basename(mission_path.name, label="mission geometry") != "mission_geometry.yaml":
            raise SavedMapCoverageError("formal mission geometry must use mission_geometry.yaml")
        manifest_bytes = _read_artifact_snapshot(
            root, "map_lifecycle_manifest.json", label="saved-map lifecycle manifest"
        )
        manifest = json.loads(manifest_bytes)
        hashes = manifest["sha256"]
        required_files = {"occupancy.yaml", "occupancy.pgm", *REQUIRED_SAVED_MAP_SUPPORT_FILES}
        if not isinstance(hashes, dict) or set(hashes) != required_files:
            raise SavedMapCoverageError("saved-map lifecycle manifest hash seal is incomplete")
        mission_bytes = _sealed_snapshot(root, mission_path.name, hashes, "mission geometry")
        value = yaml.safe_load(mission_bytes)
        polygon = _polygon(value["outer_polygon"], "public mission geometry")
        coverage = value["saved_occupancy_coverage"]
        if not isinstance(coverage, dict):
            raise TypeError("saved occupancy coverage must be a mapping")
        geometry_name = _artifact_basename(coverage["geometry"], label="coverage geometry")
        free_name = _artifact_basename(coverage["free_space_map"], label="coverage free-space map")
        geometry_bytes = _sealed_snapshot(root, geometry_name, hashes, "coverage geometry")
        free_bytes = _sealed_snapshot(root, free_name, hashes, "coverage free-space map")
        geometry = yaml.safe_load(geometry_bytes)
    except SavedMapCoverageError:
        raise
    except (MapLifecycleError, OSError, json.JSONDecodeError, yaml.YAMLError, KeyError, TypeError, ValueError, IndexError) as exc:
        raise SavedMapCoverageError("public mission geometry is missing or invalid") from exc
    area = polygon_area(polygon)
    if abs(area - 20_000.0) > 1e-3:
        raise SavedMapCoverageError("formal saved-map coverage requires 20000 m2")
    truth = value.get("truth_boundary", {})
    if not isinstance(truth, dict) or any(
        truth.get(name) is not False
        for name in (
            "world_geometry_used_for_product_map",
            "evaluator_truth_used",
            "dirt_truth_used",
        )
    ):
        raise SavedMapCoverageError("product mission geometry violates truth isolation")
    if (
        geometry_name != "coverage_geometry.yaml"
        or free_name != "coverage_free_space.pgm"
        or coverage.get("sha256") != hashes[geometry_name]
        or coverage.get("planning_clearance_preapplied") is not True
        or not isinstance(geometry, dict)
        or geometry.get("source") != "saved_slam_occupancy_only"
        or geometry.get("occupancy_map") != "occupancy.yaml"
        or geometry.get("occupancy_image") != "occupancy.pgm"
        or geometry.get("occupancy_map_sha256") != hashes["occupancy.yaml"]
        or geometry.get("occupancy_image_sha256") != hashes["occupancy.pgm"]
        or geometry.get("free_space_map") != free_name
        or geometry.get("free_space_map_sha256") != hashes[free_name]
        or geometry.get("planning_clearance_preapplied") is not True
        or geometry.get("world_truth_used_for_product_map") is not False
    ):
        raise SavedMapCoverageError("saved occupancy coverage geometry is missing or invalid")
    planning_outer = _polygon(geometry.get("planning_outer_polygon"), "saved occupancy planning outer polygon")
    planning_holes = tuple(_polygon(item, "saved occupancy planning hole polygon") for item in geometry.get("planning_hole_polygons", ()))
    keepouts = tuple(_polygon(item, "saved occupancy keepout polygon") for item in geometry.get("keepout_polygons", ()))
    if value.get("keepout_polygons") != [list(map(list, item)) for item in keepouts]:
        raise SavedMapCoverageError("mission does not use its saved occupancy coverage geometry")
    if any(
        not point_in_polygon(
            sum(point[0] for point in hole) / len(hole),
            sum(point[1] for point in hole) / len(hole), planning_outer,
        )
        for hole in planning_holes
    ):
        raise SavedMapCoverageError("saved occupancy planning hole is outside its outer region")
    try:
        resolution = float(geometry["resolution_m"])
        origin_x, origin_y, origin_yaw = (float(item) for item in geometry["origin"])
    except (KeyError, TypeError, ValueError) as exc:
        raise SavedMapCoverageError("saved occupancy coverage raster metadata is invalid") from exc
    if not 0.0 < resolution <= 0.25 or not all(math.isfinite(item) for item in (origin_x, origin_y, origin_yaw)) or abs(origin_yaw) > 1e-9:
        raise SavedMapCoverageError("saved occupancy coverage raster metadata is invalid")
    try:
        width, height, rows = parse_binary_pgm(free_bytes)
    except Exception as exc:
        raise SavedMapCoverageError("saved coverage free-space PGM is invalid") from exc
    if any(pixel not in (0, 255) for row in rows for pixel in row):
        raise SavedMapCoverageError("saved coverage free-space PGM must be binary")
    free_cells = frozenset((column, row) for row in range(height) for column in range(width) if rows[row][column] == 255)
    expected_cells = geometry.get("reachable_cleanable_cells")
    if not isinstance(expected_cells, int) or expected_cells <= 0 or len(free_cells) != expected_cells:
        raise SavedMapCoverageError("saved occupancy coverage free-space count is invalid")
    try:
        clearance = float(geometry["planning_clearance_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise SavedMapCoverageError("saved occupancy planning clearance is invalid") from exc
    if (
        not math.isclose(clearance, FORMAL_COVERAGE_STATIC_OBSTACLE_INFLATION_M, abs_tol=1e-12)
        or geometry.get("obstacle_inflation_m") != clearance
    ):
        raise SavedMapCoverageError("saved occupancy planning clearance is invalid")
    return SavedMapCoverageGeometry(
        outer_polygon=polygon,
        planning_outer_polygon=planning_outer,
        planning_hole_polygons=planning_holes,
        keepout_polygons=keepouts,
        free_cells=free_cells,
        raster_resolution_m=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        planning_clearance_m=clearance,
        sha256=hashes[geometry_name],
    )


def validate_execution_parameters(
    operation_width_m: float,
    max_speed_mps: float,
    speed_profile: FormalOperationSpeedProfile,
) -> None:
    if not math.isclose(operation_width_m, FORMAL_OPERATION_WIDTH_M, abs_tol=1e-9):
        raise SavedMapCoverageError("formal operation width must be exactly 1.32 m")
    if not math.isclose(
        max_speed_mps, speed_profile.maximum_linear_speed_mps, abs_tol=1e-9
    ):
        raise SavedMapCoverageError("formal maximum linear speed disagrees with profile")
    expected_speeds = {
        MAPPING_SAFE_SPEED_PROFILE: FORMAL_MAX_LINEAR_SPEED_MPS,
        DRY_CLEANING_SPEED_PROFILE: 1.0,
    }
    if speed_profile.name not in expected_speeds or not math.isclose(
        speed_profile.maximum_linear_speed_mps,
        expected_speeds[speed_profile.name],
        abs_tol=1e-9,
    ):
        raise SavedMapCoverageError("formal execution profile is not an approved dry or mapping profile")


@dataclass
class ProductCoverageTelemetry:
    """Integrate estimated motion and brush sweep without simulator truth."""

    polygon: tuple[tuple[float, float], ...]
    keepout_polygons: tuple[tuple[tuple[float, float], ...], ...] = ()
    cleanable_cells: frozenset[tuple[int, int]] | None = None
    raster_origin_x: float | None = None
    raster_origin_y: float | None = None
    coverage_geometry_sha256: str | None = None
    planning_clearance_m: float | None = None
    operation_width_m: float = FORMAL_OPERATION_WIDTH_M
    operation_speed_profile: str = MAPPING_SAFE_SPEED_PROFILE
    maximum_linear_speed_mps: float = FORMAL_MAX_LINEAR_SPEED_MPS
    raster_resolution_m: float = 0.25
    total_distance_m: float = 0.0
    brush_enabled_distance_m: float = 0.0
    brush_state: bool = False
    brush_state_samples: int = 0
    brush_state_transitions: int = 0
    _last_odom_xy: tuple[float, float] | None = None
    _last_map_xy: tuple[float, float] | None = None
    _covered_cells: set[tuple[int, int]] = field(default_factory=set)

    def __post_init__(self) -> None:
        speed_profile = FormalOperationSpeedProfile(
            self.operation_speed_profile, self.maximum_linear_speed_mps
        )
        validate_execution_parameters(
            self.operation_width_m, self.maximum_linear_speed_mps, speed_profile
        )
        if not 0.0 < self.raster_resolution_m <= 0.25:
            raise SavedMapCoverageError("coverage evidence raster must be <=0.25 m")
        self._min_x = self.raster_origin_x if self.cleanable_cells is not None else min(point[0] for point in self.polygon)
        self._min_y = self.raster_origin_y if self.cleanable_cells is not None else min(point[1] for point in self.polygon)
        if self.cleanable_cells is not None:
            if self.raster_origin_x is None or self.raster_origin_y is None or not self.cleanable_cells:
                raise SavedMapCoverageError("saved occupancy coverage raster is incomplete")
            self._field_cells = len(self.cleanable_cells)
        else:
            self._field_cells = max(
                1,
                round(
                    polygon_area(self.polygon)
                    / (self.raster_resolution_m * self.raster_resolution_m)
                ),
            )

    @classmethod
    def from_mission_geometry(cls, geometry: SavedMapCoverageGeometry) -> "ProductCoverageTelemetry":
        return cls(
            polygon=geometry.outer_polygon,
            keepout_polygons=geometry.keepout_polygons,
            cleanable_cells=geometry.free_cells,
            raster_resolution_m=geometry.raster_resolution_m,
            raster_origin_x=geometry.origin_x,
            raster_origin_y=geometry.origin_y,
            coverage_geometry_sha256=geometry.sha256,
            planning_clearance_m=geometry.planning_clearance_m,
        )

    def set_brush(self, enabled: bool) -> None:
        self.brush_state_samples += 1
        enabled = bool(enabled)
        if enabled != self.brush_state:
            self.brush_state_transitions += 1
        self.brush_state = enabled

    def observe_odom(self, x: float, y: float) -> None:
        if not math.isfinite(x) or not math.isfinite(y):
            return
        current = (x, y)
        if self._last_odom_xy is not None:
            distance = math.dist(self._last_odom_xy, current)
            if distance <= 5.0:
                self.total_distance_m += distance
                if self.brush_state:
                    self.brush_enabled_distance_m += distance
        self._last_odom_xy = current

    def observe_map_pose(self, x: float, y: float) -> None:
        if not math.isfinite(x) or not math.isfinite(y):
            return
        current = (x, y)
        previous = self._last_map_xy or current
        distance = math.dist(previous, current)
        samples = max(1, math.ceil(distance / (self.raster_resolution_m / 2.0)))
        if self.brush_state:
            for index in range(samples + 1):
                ratio = index / samples
                self._mark_disk(
                    previous[0] + (current[0] - previous[0]) * ratio,
                    previous[1] + (current[1] - previous[1]) * ratio,
                )
        self._last_map_xy = current

    def _mark_disk(self, x: float, y: float) -> None:
        radius = self.operation_width_m / 2.0
        cell_radius = math.ceil(radius / self.raster_resolution_m)
        center_column = math.floor((x - self._min_x) / self.raster_resolution_m)
        center_row = math.floor((y - self._min_y) / self.raster_resolution_m)
        for row in range(center_row - cell_radius, center_row + cell_radius + 1):
            cy = self._min_y + (row + 0.5) * self.raster_resolution_m
            for column in range(
                center_column - cell_radius, center_column + cell_radius + 1
            ):
                cx = self._min_x + (column + 0.5) * self.raster_resolution_m
                if (
                    math.hypot(cx - x, cy - y) <= radius
                    and (
                        (column, row) in self.cleanable_cells
                        if self.cleanable_cells is not None
                        else point_in_polygon(cx, cy, self.polygon) and not any(
                            point_in_polygon(cx, cy, polygon)
                            for polygon in self.keepout_polygons
                        )
                    )
                ):
                    self._covered_cells.add((column, row))

    @property
    def estimated_coverage_fraction(self) -> float:
        return min(1.0, len(self._covered_cells) / self._field_cells)

    def report(self) -> dict:
        return {
            "trajectory_total_distance_m": self.total_distance_m,
            "brush_enabled_distance_m": self.brush_enabled_distance_m,
            "brush_state_transitions": self.brush_state_transitions,
            "brush_state_sample_count": self.brush_state_samples,
            "brush_state_source": "/brush_enabled_product_runtime",
            "brush_disabled_on_exit": not self.brush_state,
            "estimated_covered_cells": len(self._covered_cells),
            "estimated_field_cells": self._field_cells,
            "estimated_coverage_fraction": self.estimated_coverage_fraction,
            "coverage_raster_resolution_m": self.raster_resolution_m,
            "coverage_geometry_sha256": self.coverage_geometry_sha256,
            "coverage_planning_clearance_m": self.planning_clearance_m,
            "coverage_pose_source": "amcl_pose_product_estimate",
            "operation_speed_profile": self.operation_speed_profile,
            "maximum_linear_speed_mps": self.maximum_linear_speed_mps,
            "simulator_truth_used": False,
        }


def coverage_execution_passed(report: dict) -> bool:
    return (
        report.get("success") is True
        and report.get("terminal_state") == "COMPLETED"
        and report.get("ground_truth_used_for_control") is False
        and report.get("operation_width_m") == FORMAL_OPERATION_WIDTH_M
        and report.get("operation_speed_profile")
        in {MAPPING_SAFE_SPEED_PROFILE, DRY_CLEANING_SPEED_PROFILE}
        and report.get("maximum_linear_speed_mps")
        in {FORMAL_MAX_LINEAR_SPEED_MPS, 1.0}
        and int(report.get("planned_swath_count", 0)) > 0
        and report.get("completed_swath_count") == report.get("planned_swath_count")
        and isinstance(report.get("coverage_geometry_sha256"), str)
        and len(report["coverage_geometry_sha256"]) == 64
        and float(report.get("cleanable_area_m2", 0.0)) > 0.0
    )
