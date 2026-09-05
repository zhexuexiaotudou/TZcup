"""ROS-independent evidence helpers for saved-map product coverage."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any, Sequence

import yaml


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


def load_product_mission_geometry(path: str | Path) -> tuple[tuple[float, float], ...]:
    try:
        value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        raw = value["outer_polygon"]
        polygon = tuple((float(point[0]), float(point[1])) for point in raw)
    except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError, IndexError) as exc:
        raise SavedMapCoverageError("public mission geometry is missing or invalid") from exc
    if not all(math.isfinite(item) for point in polygon for item in point):
        raise SavedMapCoverageError("coverage polygon contains non-finite coordinates")
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
    return polygon


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
        self._min_x = min(point[0] for point in self.polygon)
        self._min_y = min(point[1] for point in self.polygon)
        self._field_cells = max(
            1,
            round(
                polygon_area(self.polygon)
                / (self.raster_resolution_m * self.raster_resolution_m)
            ),
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
                    and point_in_polygon(cx, cy, self.polygon)
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
    )
