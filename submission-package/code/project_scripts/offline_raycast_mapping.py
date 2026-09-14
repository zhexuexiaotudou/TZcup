#!/usr/bin/env python3
"""Deterministic offline 2D raycast mapping from frozen SDF collision geometry.

This tool never starts ROS or Gazebo. It rasterizes static collision geometry
only to simulate range observations, then reconstructs the saved PGM/YAML map
from an inverse sensor log-odds update.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from typing import Iterable, Sequence
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import yaml


sys.path.insert(0, str(Path(__file__).resolve().parent))
from verify_map_area import assess_map  # noqa: E402


OFFLINE_MAPPING_KIND = "OFFLINE_RAYCAST_MAPPING"
DEFAULT_RESOLUTION_M = 0.05
DEFAULT_SENSOR_HEIGHT_M = 1.1621
DEFAULT_RANGE_M = 29.0
DEFAULT_RAY_COUNT = 1080
DEFAULT_POSE_SPACING_M = 1.0
DEFAULT_MAP_MARGIN_M = 5.0
DEFAULT_VEHICLE_RADIUS_M = 0.70
LOG_ODDS_FREE = -0.40
LOG_ODDS_OCCUPIED = 0.85
LOG_ODDS_OCCUPIED_THRESHOLD = math.log(0.65 / 0.35)
LOG_ODDS_FREE_THRESHOLD = math.log(0.25 / 0.75)


class OfflineMappingError(RuntimeError):
    """Raised when the offline mapping contract cannot be satisfied."""


@dataclass(frozen=True)
class StaticCollision:
    """Projected static collision footprint at the 2D scan plane."""

    name: str
    shape: str
    center_x_m: float
    center_y_m: float
    half_x_m: float
    half_y_m: float
    yaw_rad: float
    min_z_m: float
    max_z_m: float


@dataclass(frozen=True)
class GridSpec:
    """Output grid geometry."""

    min_x_m: float
    min_y_m: float
    resolution_m: float
    width: int
    height: int

    @property
    def max_x_m(self) -> float:
        return self.min_x_m + self.width * self.resolution_m

    @property
    def max_y_m(self) -> float:
        return self.min_y_m + self.height * self.resolution_m


@dataclass(frozen=True)
class Pose:
    """Known vehicle pose on the deterministic route."""

    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class ScanResult:
    """Cells observed by one scan."""

    free_indices: np.ndarray
    occupied_indices: np.ndarray
    ranges_m: np.ndarray


def sha256_file(path: str | Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _pose_values(element: ET.Element | None) -> tuple[float, float, float, float, float, float]:
    text = "" if element is None or element.text is None else element.text
    try:
        values = tuple(float(value) for value in text.split())
    except ValueError as exc:
        raise OfflineMappingError(f"invalid SDF pose: {text!r}") from exc
    if not values:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise OfflineMappingError(f"SDF pose must contain six finite values: {text!r}")
    return values  # type: ignore[return-value]


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _pose_matrix(values: Sequence[float]) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = _rpy_matrix(values[3], values[4], values[5])
    transform[:3, 3] = values[:3]
    return transform


def _positive_float(text: str | None, label: str) -> float:
    try:
        value = float(text or "")
    except ValueError as exc:
        raise OfflineMappingError(f"invalid {label}") from exc
    if not math.isfinite(value) or value <= 0.0:
        raise OfflineMappingError(f"invalid {label}")
    return value


def extract_scan_plane_collisions(
    world_path: str | Path,
    *,
    sensor_height_m: float,
    z_tolerance_m: float = 1e-6,
) -> tuple[str, list[StaticCollision], int]:
    """Extract static collision footprints intersecting a horizontal scan plane."""

    if not math.isfinite(sensor_height_m):
        raise OfflineMappingError("sensor height must be finite")
    try:
        root = ET.parse(Path(world_path)).getroot()
    except (OSError, ET.ParseError) as exc:
        raise OfflineMappingError(f"unable to read SDF world: {world_path}") from exc
    world = root.find("world")
    if world is None or not world.get("name"):
        raise OfflineMappingError("SDF has no named world")

    collisions: list[StaticCollision] = []
    static_count = 0
    for model in world.findall("model"):
        if (model.findtext("static") or "").strip().lower() != "true":
            continue
        model_name = model.get("name") or "unnamed_static_model"
        if model_name.startswith("walker_"):
            continue
        if not model.findall("./link/collision"):
            continue
        static_count += 1
        model_transform = _pose_matrix(_pose_values(model.find("pose")))
        collision_index = 0
        for link in model.findall("link"):
            link_transform = model_transform @ _pose_matrix(_pose_values(link.find("pose")))
            for collision in link.findall("collision"):
                collision_transform = link_transform @ _pose_matrix(
                    _pose_values(collision.find("pose"))
                )
                geometry = collision.find("geometry")
                if geometry is None:
                    raise OfflineMappingError(f"static collision has no geometry: {model_name}")
                box = geometry.find("box")
                cylinder = geometry.find("cylinder")
                if box is not None:
                    values = tuple(float(value) for value in (box.findtext("size") or "").split())
                    if len(values) != 3 or not all(
                        math.isfinite(value) and value > 0.0 for value in values
                    ):
                        raise OfflineMappingError(f"invalid box geometry: {model_name}")
                    half_x, half_y, half_z = (value / 2.0 for value in values)
                    shape = "box"
                elif cylinder is not None:
                    radius = _positive_float(cylinder.findtext("radius"), "cylinder radius")
                    length = _positive_float(cylinder.findtext("length"), "cylinder length")
                    half_x = half_y = radius
                    half_z = length / 2.0
                    shape = "cylinder"
                else:
                    # The frozen large campus contains only static box, cylinder
                    # and the non-occupying asphalt/plane collision.
                    if geometry.find("plane") is not None:
                        continue
                    raise OfflineMappingError(
                        f"unsupported static collision geometry: {model_name}"
                    )

                rotation = collision_transform[:3, :3]
                z_extent = (
                    abs(rotation[2, 0]) * half_x
                    + abs(rotation[2, 1]) * half_y
                    + abs(rotation[2, 2]) * half_z
                )
                center_z = float(collision_transform[2, 3])
                min_z = center_z - z_extent
                max_z = center_z + z_extent
                if not (min_z - z_tolerance_m <= sensor_height_m <= max_z + z_tolerance_m):
                    continue

                yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
                if shape == "box":
                    cosine, sine = abs(math.cos(yaw)), abs(math.sin(yaw))
                    extent_x = cosine * half_x + sine * half_y
                    extent_y = sine * half_x + cosine * half_y
                else:
                    extent_x = extent_y = max(half_x, half_y)
                suffix = "" if collision_index == 0 else f"_{collision_index}"
                collisions.append(
                    StaticCollision(
                        name=f"{model_name}{suffix}",
                        shape=shape,
                        center_x_m=float(collision_transform[0, 3]),
                        center_y_m=float(collision_transform[1, 3]),
                        half_x_m=float(extent_x),
                        half_y_m=float(extent_y),
                        yaw_rad=yaw,
                        min_z_m=min_z,
                        max_z_m=max_z,
                    )
                )
                collision_index += 1
    collisions.sort(key=lambda item: item.name)
    return world.get("name", ""), collisions, static_count


def make_grid(
    *,
    field_min_x_m: float,
    field_min_y_m: float,
    field_width_m: float,
    field_height_m: float,
    margin_m: float,
    resolution_m: float,
) -> GridSpec:
    if resolution_m <= 0.0 or resolution_m > 0.05:
        raise OfflineMappingError("resolution must be in (0, 0.05] m")
    if margin_m < 0.0 or not math.isfinite(margin_m):
        raise OfflineMappingError("map margin must be finite and non-negative")
    width = round((field_width_m + 2.0 * margin_m) / resolution_m)
    height = round((field_height_m + 2.0 * margin_m) / resolution_m)
    if width <= 0 or height <= 0:
        raise OfflineMappingError("output grid dimensions must be positive")
    return GridSpec(
        min_x_m=field_min_x_m - margin_m,
        min_y_m=field_min_y_m - margin_m,
        resolution_m=resolution_m,
        width=width,
        height=height,
    )


def rasterize_collisions(
    collisions: Iterable[StaticCollision],
    grid: GridSpec,
) -> np.ndarray:
    """Rasterize scan-plane collision footprints."""

    occupancy = np.zeros((grid.height, grid.width), dtype=bool)
    for collision in collisions:
        first_column = max(
            0,
            math.floor(
                (collision.center_x_m - collision.half_x_m - grid.min_x_m)
                / grid.resolution_m
            ),
        )
        last_column = min(
            grid.width - 1,
            math.ceil(
                (collision.center_x_m + collision.half_x_m - grid.min_x_m)
                / grid.resolution_m
            ),
        )
        first_row = max(
            0,
            math.floor(
                (collision.center_y_m - collision.half_y_m - grid.min_y_m)
                / grid.resolution_m
            ),
        )
        last_row = min(
            grid.height - 1,
            math.ceil(
                (collision.center_y_m + collision.half_y_m - grid.min_y_m)
                / grid.resolution_m
            ),
        )
        if first_column > last_column or first_row > last_row:
            continue
        x = grid.min_x_m + (np.arange(first_column, last_column + 1) + 0.5) * grid.resolution_m
        y = grid.min_y_m + (np.arange(first_row, last_row + 1) + 0.5) * grid.resolution_m
        dx = x[None, :] - collision.center_x_m
        dy = y[:, None] - collision.center_y_m
        cosine, sine = math.cos(collision.yaw_rad), math.sin(collision.yaw_rad)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        if collision.shape == "box":
            inside = (
                (np.abs(local_x) <= collision.half_x_m + 1e-12)
                & (np.abs(local_y) <= collision.half_y_m + 1e-12)
            )
        elif collision.shape == "cylinder":
            radius = max(collision.half_x_m, collision.half_y_m)
            inside = local_x * local_x + local_y * local_y <= radius * radius + 1e-12
        else:
            raise OfflineMappingError(f"unsupported collision shape: {collision.shape}")
        occupancy[first_row : last_row + 1, first_column : last_column + 1] |= inside
    return occupancy


def _point_is_clear(
    occupancy: np.ndarray,
    grid: GridSpec,
    x_m: float,
    y_m: float,
    radius_m: float,
) -> bool:
    half_span = math.ceil(radius_m / grid.resolution_m)
    center_column = math.floor((x_m - grid.min_x_m) / grid.resolution_m)
    center_row = math.floor((y_m - grid.min_y_m) / grid.resolution_m)
    if not (
        half_span <= center_column < grid.width - half_span
        and half_span <= center_row < grid.height - half_span
    ):
        return False
    window = occupancy[
        center_row - half_span : center_row + half_span + 1,
        center_column - half_span : center_column + half_span + 1,
    ]
    return not bool(window.any())


def sample_polyline(points: Sequence[tuple[float, float]], spacing_m: float) -> list[Pose]:
    """Sample a polyline deterministically at no more than the requested cadence."""

    if not math.isfinite(spacing_m) or spacing_m <= 0.0:
        raise OfflineMappingError("pose spacing must be finite and positive")
    if len(points) < 2:
        raise OfflineMappingError("route must contain at least two points")
    poses: list[Pose] = []
    previous: tuple[float, float] | None = None
    for start, end in zip(points, points[1:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 0.0:
            raise OfflineMappingError("route segments must have positive length")
        yaw = math.atan2(dy, dx)
        count = max(1, math.ceil(length / spacing_m))
        for index in range(count):
            fraction = index / count
            point = (start[0] + fraction * dx, start[1] + fraction * dy)
            if previous != point:
                poses.append(Pose(point[0], point[1], yaw))
                previous = point
    poses.append(Pose(points[-1][0], points[-1][1], poses[-1].yaw_rad))
    if len(poses) < 2:
        raise OfflineMappingError("route produced fewer than two poses")
    if any(
        math.hypot(right.x_m - left.x_m, right.y_m - left.y_m) > spacing_m + 1e-9
        for left, right in zip(poses, poses[1:])
    ):
        raise OfflineMappingError("sampled route violates pose cadence")
    return poses


def validate_route(
    poses: Sequence[Pose],
    occupancy: np.ndarray,
    grid: GridSpec,
    *,
    vehicle_radius_m: float,
) -> None:
    for pose in poses:
        if not _point_is_clear(
            occupancy, grid, pose.x_m, pose.y_m, vehicle_radius_m
        ):
            raise OfflineMappingError(
                f"route pose intersects static geometry: "
                f"({pose.x_m:.3f}, {pose.y_m:.3f})"
            )


def cast_scan(
    occupancy: np.ndarray,
    grid: GridSpec,
    pose: Pose,
    *,
    angles_rad: np.ndarray,
    max_range_m: float,
    ray_step_m: float | None = None,
) -> ScanResult:
    """Cast one deterministic 2D range scan against the source geometry raster."""

    if occupancy.shape != (grid.height, grid.width):
        raise OfflineMappingError("occupancy shape does not match grid")
    if angles_rad.ndim != 1 or angles_rad.size == 0:
        raise OfflineMappingError("scan angles must be a non-empty 1D array")
    if max_range_m <= 0.0 or not math.isfinite(max_range_m):
        raise OfflineMappingError("scan range must be finite and positive")
    step = grid.resolution_m * 0.5 if ray_step_m is None else ray_step_m
    if step <= 0.0 or step > grid.resolution_m:
        raise OfflineMappingError("ray step must be in (0, resolution] m")

    distances = np.arange(0.0, max_range_m + step * 0.5, step, dtype=np.float64)
    cosine = np.cos(angles_rad)[:, None]
    sine = np.sin(angles_rad)[:, None]
    x = pose.x_m + cosine * distances[None, :]
    y = pose.y_m + sine * distances[None, :]
    columns = np.floor((x - grid.min_x_m) / grid.resolution_m).astype(np.int64)
    rows = np.floor((y - grid.min_y_m) / grid.resolution_m).astype(np.int64)
    valid = (
        (columns >= 0)
        & (columns < grid.width)
        & (rows >= 0)
        & (rows < grid.height)
    )
    flat = np.zeros(valid.shape, dtype=np.int64)
    flat[valid] = rows[valid] * grid.width + columns[valid]
    hits = np.zeros(valid.shape, dtype=bool)
    hits[valid] = occupancy.ravel()[flat[valid]]
    has_hit = hits.any(axis=1)
    first_hit = np.argmax(hits, axis=1)
    first_hit[~has_hit] = hits.shape[1]
    steps = np.arange(hits.shape[1], dtype=np.int64)[None, :]
    free_mask = valid & (steps < first_hit[:, None])
    occupied_mask = (
        valid
        & hits
        & has_hit[:, None]
        & (steps == first_hit[:, None])
    )
    ranges = np.full(angles_rad.shape, np.inf, dtype=np.float64)
    ranges[has_hit] = distances[first_hit[has_hit]]
    return ScanResult(
        free_indices=np.unique(flat[free_mask]),
        occupied_indices=np.unique(flat[occupied_mask]),
        ranges_m=ranges,
    )


def reconstruct_occupancy(
    source_occupancy: np.ndarray,
    grid: GridSpec,
    poses: Sequence[Pose],
    *,
    ray_count: int,
    max_range_m: float,
) -> tuple[np.ndarray, dict[str, object]]:
    """Reconstruct an occupancy map with inverse sensor log odds."""

    if ray_count < 4 or ray_count % 2:
        raise OfflineMappingError("ray count must be an even integer of at least four")
    angles = np.linspace(
        -math.pi,
        math.pi,
        ray_count,
        endpoint=False,
        dtype=np.float64,
    )
    log_odds = np.zeros(source_occupancy.size, dtype=np.float32)
    hit_observations = 0
    free_observations = 0
    finite_ranges = 0
    for pose in poses:
        scan = cast_scan(
            source_occupancy,
            grid,
            pose,
            angles_rad=angles,
            max_range_m=max_range_m,
        )
        log_odds[scan.free_indices] += LOG_ODDS_FREE
        log_odds[scan.occupied_indices] += LOG_ODDS_OCCUPIED
        free_observations += int(scan.free_indices.size)
        hit_observations += int(scan.occupied_indices.size)
        finite_ranges += int(np.isfinite(scan.ranges_m).sum())
    unknown = log_odds == 0.0
    occupied = log_odds >= LOG_ODDS_OCCUPIED_THRESHOLD
    free = log_odds <= LOG_ODDS_FREE_THRESHOLD
    conflict = occupied & free
    if conflict.any():
        raise OfflineMappingError("log-odds classification produced occupied/free conflicts")
    return (
        np.where(
            occupied,
            np.uint8(0),
            np.where(free, np.uint8(254), np.uint8(205)),
        ).reshape((grid.height, grid.width)),
        {
            "pose_count": len(poses),
            "ray_count": ray_count,
            "free_cell_updates": free_observations,
            "occupied_cell_updates": hit_observations,
            "finite_range_returns": finite_ranges,
        },
    )


def classification_counts(pixels: np.ndarray) -> dict[str, int]:
    if pixels.ndim != 2:
        raise OfflineMappingError("map pixels must be two-dimensional")
    occupied = int(np.count_nonzero(pixels == 0))
    free = int(np.count_nonzero(pixels == 254))
    unknown = int(np.count_nonzero(pixels == 205))
    if occupied + free + unknown != pixels.size:
        raise OfflineMappingError("map contains unsupported trinary pixel values")
    return {
        "occupied_cells": occupied,
        "free_cells": free,
        "unknown_cells": unknown,
        "known_cells": occupied + free,
    }


def _cell_centers(grid: GridSpec) -> tuple[np.ndarray, np.ndarray]:
    x = grid.min_x_m + (np.arange(grid.width, dtype=np.float64) + 0.5) * grid.resolution_m
    y = grid.min_y_m + (np.arange(grid.height, dtype=np.float64) + 0.5) * grid.resolution_m
    return x, y


def _binary_boundary(mask: np.ndarray) -> np.ndarray:
    neighbors = (
        np.roll(mask, 1, axis=0)
        & np.roll(mask, -1, axis=0)
        & np.roll(mask, 1, axis=1)
        & np.roll(mask, -1, axis=1)
    )
    return mask & ~neighbors


def _binary_dilate(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    if radius_cells < 0:
        raise OfflineMappingError("dilation radius must be non-negative")
    result = mask.copy()
    for _ in range(radius_cells):
        next_result = result.copy()
        next_result[1:, :] |= result[:-1, :]
        next_result[:-1, :] |= result[1:, :]
        next_result[:, 1:] |= result[:, :-1]
        next_result[:, :-1] |= result[:, 1:]
        result = next_result
    return result


def quality_metrics(
    reconstructed: np.ndarray,
    source_occupancy: np.ndarray,
    grid: GridSpec,
    *,
    field_bounds_m: tuple[float, float, float, float],
) -> dict[str, float | int | bool]:
    """Measure reconstruction quality against public source geometry."""

    if reconstructed.shape != source_occupancy.shape:
        raise OfflineMappingError("reconstructed and source grids must match")
    min_x, min_y, max_x, max_y = field_bounds_m
    x, y = _cell_centers(grid)
    field_mask = (
        (x[None, :] >= min_x)
        & (x[None, :] <= max_x)
        & (y[:, None] >= min_y)
        & (y[:, None] <= max_y)
    )
    occupied = reconstructed == 0
    free = reconstructed == 254
    known = reconstructed != 205
    source_boundary = _binary_boundary(source_occupancy) & field_mask
    boundary_matched = source_boundary & _binary_dilate(
        occupied,
        math.ceil(0.15 / grid.resolution_m),
    )
    source_free = (~source_occupancy) & field_mask
    source_occupied = source_occupancy & field_mask
    cell_area = grid.resolution_m * grid.resolution_m
    total_cells = int(reconstructed.size)
    known_cells = int(np.count_nonzero(known))
    field_cells = int(np.count_nonzero(field_mask))
    unknown_cells = total_cells - known_cells
    source_boundary_cells = int(np.count_nonzero(source_boundary))
    source_free_cells = int(np.count_nonzero(source_free))
    source_occupied_cells = int(np.count_nonzero(source_occupied))
    reconstructed_occupied_cells = int(np.count_nonzero(occupied))
    reconstructed_free_cells = int(np.count_nonzero(free))
    occupied_precision_denominator = max(1, reconstructed_occupied_cells)
    free_precision_denominator = max(1, reconstructed_free_cells)
    boundary_recall = (
        float(np.count_nonzero(boundary_matched)) / source_boundary_cells
        if source_boundary_cells
        else 1.0
    )
    source_free_recall = (
        float(np.count_nonzero(source_free & free)) / source_free_cells
        if source_free_cells
        else 1.0
    )
    metrics: dict[str, float | int | bool] = {
        "map_cells": total_cells,
        "known_cells": known_cells,
        "unknown_cells": unknown_cells,
        "known_area_m2": known_cells * cell_area,
        "unknown_fraction": unknown_cells / total_cells,
        "field_cells": field_cells,
        "field_known_cells": int(np.count_nonzero(known & field_mask)),
        "field_known_area_m2": float(np.count_nonzero(known & field_mask))
        * cell_area,
        "field_unknown_fraction": 1.0
        - float(np.count_nonzero(known & field_mask)) / field_cells,
        "source_field_area_m2": (source_free_cells + source_occupied_cells)
        * cell_area,
        "source_field_occupied_cells": source_occupied_cells,
        "source_field_free_cells": source_free_cells,
        "source_field_free_area_m2": source_free_cells * cell_area,
        "source_field_occupied_area_m2": source_occupied_cells * cell_area,
        "source_occupied_boundary_cells": source_boundary_cells,
        "source_occupied_boundary_recall": boundary_recall,
        "source_free_recall": source_free_recall,
        "occupied_precision": float(
            np.count_nonzero(occupied & source_occupied)
        )
        / occupied_precision_denominator,
        "free_precision": float(
            np.count_nonzero(free & ~source_occupancy)
        )
        / free_precision_denominator,
        "area_gate": known_cells * cell_area >= 20_000.0,
        "resolution_gate": grid.resolution_m <= 0.05,
        "unknown_fraction_gate": unknown_cells / total_cells <= 0.10,
        "source_free_recall_gate": source_free_recall >= 0.99,
        "source_occupied_boundary_recall_gate": boundary_recall >= 0.95,
    }
    metrics["quality_gate"] = all(
        bool(metrics[key])
        for key in (
            "area_gate",
            "resolution_gate",
            "unknown_fraction_gate",
            "source_free_recall_gate",
            "source_occupied_boundary_recall_gate",
        )
    )
    return metrics


def write_map_bundle(
    output_directory: Path,
    pixels_bottom_up: np.ndarray,
    grid: GridSpec,
) -> tuple[Path, Path]:
    output_directory.mkdir(parents=True, exist_ok=True)
    image_path = output_directory / "occupancy.pgm"
    yaml_path = output_directory / "occupancy.yaml"
    image = Image.fromarray(np.flipud(pixels_bottom_up), mode="L")
    image.save(image_path, format="PPM")
    yaml_path.write_text(
        yaml.safe_dump(
            {
                "image": image_path.name,
                "mode": "trinary",
                "resolution": grid.resolution_m,
                "origin": [grid.min_x_m, grid.min_y_m, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.25,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return yaml_path, image_path


def write_area_verification_report(
    output_directory: Path,
    yaml_path: Path,
) -> tuple[Path, dict[str, object], dict[str, object]]:
    """Write the same single-map wrapper produced by verify_map_area's CLI."""

    measured = assess_map(
        yaml_path,
        root=output_directory,
        minimum_area_m2=20_000.0,
    )
    report: dict[str, object] = {
        "schema_version": 1,
        "scan_root": ".",
        "minimum_known_area_m2": 20_000.0,
        "map_count": 1,
        "pass_count": 1 if measured["area_gate"] else 0,
        "pass": bool(measured["area_gate"]),
        "largest_known_area_map_yaml": measured["map_yaml"],
        "largest_known_area_m2": measured["known_area_m2"],
        "maps": [measured],
        "excluded_maps": [],
        "errors": [],
    }
    report_path = output_directory / "map_area_verification.json"
    report_path.write_text(
        json.dumps(report, ensure_ascii=True, indent=2) + "\n",
        encoding="utf-8",
    )
    return report_path, report, measured


def _route() -> list[tuple[float, float]]:
    return [
        (-98.0, 0.0),
        (-98.0, 48.5),
        (98.0, 48.5),
        (98.0, 16.0),
        (-98.0, 16.0),
        (-98.0, -16.0),
        (98.0, -16.0),
        (98.0, -48.5),
        (-98.0, -48.5),
    ]


def run_offline_mapping(
    *,
    world_path: Path,
    output_directory: Path,
    resolution_m: float = DEFAULT_RESOLUTION_M,
    sensor_height_m: float = DEFAULT_SENSOR_HEIGHT_M,
    max_range_m: float = DEFAULT_RANGE_M,
    ray_count: int = DEFAULT_RAY_COUNT,
    pose_spacing_m: float = DEFAULT_POSE_SPACING_M,
    map_margin_m: float = DEFAULT_MAP_MARGIN_M,
    vehicle_radius_m: float = DEFAULT_VEHICLE_RADIUS_M,
) -> dict[str, object]:
    started = time.perf_counter()
    output_directory.mkdir(parents=True, exist_ok=True)
    world_name, collisions, static_model_count = extract_scan_plane_collisions(
        world_path,
        sensor_height_m=sensor_height_m,
    )
    if world_name != "sanitation_campus_large":
        raise OfflineMappingError(f"unexpected source world: {world_name}")
    grid = make_grid(
        field_min_x_m=-100.0,
        field_min_y_m=-50.0,
        field_width_m=200.0,
        field_height_m=100.0,
        margin_m=map_margin_m,
        resolution_m=resolution_m,
    )
    source_occupancy = rasterize_collisions(collisions, grid)
    route_points = _route()
    poses = sample_polyline(route_points, pose_spacing_m)
    validate_route(
        poses,
        source_occupancy,
        grid,
        vehicle_radius_m=vehicle_radius_m,
    )
    reconstructed, observation_stats = reconstruct_occupancy(
        source_occupancy,
        grid,
        poses,
        ray_count=ray_count,
        max_range_m=max_range_m,
    )
    metrics = quality_metrics(
        reconstructed,
        source_occupancy,
        grid,
        field_bounds_m=(-100.0, -50.0, 100.0, 50.0),
    )
    yaml_path, image_path = write_map_bundle(output_directory, reconstructed, grid)
    area_report_path, area_report, measured_area = write_area_verification_report(
        output_directory,
        yaml_path,
    )
    route_length_m = sum(
        math.dist(left, right) for left, right in zip(route_points, route_points[1:])
    )
    wall_time_s = time.perf_counter() - started
    manifest = {
        "schema_version": 1,
        "mapping_kind": OFFLINE_MAPPING_KIND,
        "status": (
            "OFFLINE_RAYCAST_MAPPING_PASS_LIVE_SLAM_NOT_CLAIMED"
            if metrics["quality_gate"] and measured_area["area_gate"]
            else "OFFLINE_RAYCAST_MAPPING_FAIL"
        ),
        "claim_boundary": (
            "Deterministic offline 2D reconstruction from frozen SDF collision "
            "geometry and simulated known-pose range scans. This is not Gazebo "
            "live SLAM, not a physical mapping run, and not localization, "
            "navigation, map-reuse, or product acceptance."
        ),
        "source": {
            "world_path": "starter_ws/src/sanitation_worlds/worlds/sanitation_campus_large.sdf",
            "world_sha256": sha256_file(world_path),
            "world_name": world_name,
            "static_model_count": static_model_count,
            "scan_plane_collision_count": len(collisions),
            "scan_plane_collisions": [asdict(collision) for collision in collisions],
            "source_raster_role": "SIMULATION_ONLY_NOT_WRITTEN_AS_OUTPUT",
        },
        "sensor_model": {
            "type": "ideal_deterministic_2d_horizontal_range_sensor",
            "physical_sensor_reference": "Hokuyo UTM-30LX scan height",
            "sensor_height_m": sensor_height_m,
            "min_range_m": 0.0,
            "max_range_m": max_range_m,
            "ray_count": ray_count,
            "angular_span_deg": 360.0,
            "angular_resolution_deg": 360.0 / ray_count,
            "range_noise_stddev_m": 0.0,
            "no_return_model": "free_space_to_max_range",
            "ray_step_m": resolution_m * 0.5,
        },
        "pose_path": {
            "deterministic": True,
            "source": "fixed_source_world_serpentine",
            "points_source_world_m": [list(point) for point in route_points],
            "route_length_m": route_length_m,
            "pose_spacing_m": pose_spacing_m,
            "pose_count": len(poses),
            "first_pose": asdict(poses[0]),
            "last_pose": asdict(poses[-1]),
            "vehicle_collision_radius_m": vehicle_radius_m,
            "route_validation": "all_poses_clear_of_scan_plane_geometry",
        },
        "map": {
            "format": "PGM_YAML",
            "mode": "trinary",
            "frame_id": "map",
            "resolution_m": grid.resolution_m,
            "origin_m": [grid.min_x_m, grid.min_y_m, 0.0],
            "width_cells": grid.width,
            "height_cells": grid.height,
            "span_x_m": grid.width * grid.resolution_m,
            "span_y_m": grid.height * grid.resolution_m,
            "field_bounds_m": [-100.0, -50.0, 100.0, 50.0],
            "unknown_pixel": 205,
            "free_pixel": 254,
            "occupied_pixel": 0,
            "log_odds_free": LOG_ODDS_FREE,
            "log_odds_occupied": LOG_ODDS_OCCUPIED,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25,
            "occupancy_yaml_sha256": sha256_file(yaml_path),
            "occupancy_pgm_sha256": sha256_file(image_path),
        },
        "observation_statistics": {
            **observation_stats,
            **classification_counts(reconstructed),
        },
        "quality_metrics": metrics,
        "map_area_verifier": {
            "report_path": area_report_path.name,
            "report_sha256": sha256_file(area_report_path),
            "minimum_known_area_m2": 20_000.0,
            "known_area_m2": measured_area["known_area_m2"],
            "area_gate": measured_area["area_gate"],
            "map_yaml_sha256": measured_area["map_yaml_sha256"],
            "map_image_sha256": measured_area["map_image_sha256"],
        },
        "runtime": {
            "wall_time_seconds": wall_time_s,
            "deterministic_inputs": True,
        },
        "live_slam_evidence_still_required": [
            "Gazebo fresh first-map lifecycle with the real vehicle sensor graph",
            "wheel/IMU odometry plus LiDAR scan matching and GNSS consistency evidence",
            "SLAM Toolbox map save, complete restart, reload, and fixed-start relocalization",
            "Nav2 route completion and collision-free exploration of the same frozen world",
            "saved-map coverage planning, localization error, and map-reuse acceptance",
        ],
    }
    manifest_path = output_directory / "offline_raycast_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=True, indent=2, sort_keys=False) + "\n",
        encoding="utf-8",
    )
    seal_path = output_directory / "offline_raycast_seal.json"
    seal_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "mapping_kind": OFFLINE_MAPPING_KIND,
                "status": manifest["status"],
                "manifest_sha256": sha256_file(manifest_path),
                "occupancy_yaml_sha256": manifest["map"]["occupancy_yaml_sha256"],
                "occupancy_pgm_sha256": manifest["map"]["occupancy_pgm_sha256"],
                "source_world_sha256": manifest["source"]["world_sha256"],
            },
            ensure_ascii=True,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    if not metrics["quality_gate"] or not measured_area["area_gate"]:
        raise OfflineMappingError(
            "offline mapping quality gate failed: "
            f"quality={metrics['quality_gate']} area={measured_area['area_gate']}"
        )
    return manifest


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Build a deterministic offline 2D occupancy map from frozen SDF "
            "collision geometry. This command never starts ROS or Gazebo."
        )
    )
    parser.add_argument(
        "--world",
        type=Path,
        default=Path(
            "starter_ws/src/sanitation_worlds/worlds/sanitation_campus_large.sdf"
        ),
    )
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--resolution-m", type=float, default=DEFAULT_RESOLUTION_M)
    parser.add_argument("--sensor-height-m", type=float, default=DEFAULT_SENSOR_HEIGHT_M)
    parser.add_argument("--max-range-m", type=float, default=DEFAULT_RANGE_M)
    parser.add_argument("--ray-count", type=int, default=DEFAULT_RAY_COUNT)
    parser.add_argument("--pose-spacing-m", type=float, default=DEFAULT_POSE_SPACING_M)
    parser.add_argument("--map-margin-m", type=float, default=DEFAULT_MAP_MARGIN_M)
    parser.add_argument("--vehicle-radius-m", type=float, default=DEFAULT_VEHICLE_RADIUS_M)
    args = parser.parse_args(argv)
    try:
        manifest = run_offline_mapping(
            world_path=args.world,
            output_directory=args.output_directory,
            resolution_m=args.resolution_m,
            sensor_height_m=args.sensor_height_m,
            max_range_m=args.max_range_m,
            ray_count=args.ray_count,
            pose_spacing_m=args.pose_spacing_m,
            map_margin_m=args.map_margin_m,
            vehicle_radius_m=args.vehicle_radius_m,
        )
    except OfflineMappingError as exc:
        print(json.dumps({"status": "OFFLINE_RAYCAST_MAPPING_FAIL", "error": str(exc)}))
        return 2
    print(
        json.dumps(
            {
                "status": manifest["status"],
                "known_area_m2": manifest["quality_metrics"]["known_area_m2"],
                "unknown_fraction": manifest["quality_metrics"]["unknown_fraction"],
                "source_occupied_boundary_recall": manifest["quality_metrics"][
                    "source_occupied_boundary_recall"
                ],
                "runtime_seconds": manifest["runtime"]["wall_time_seconds"],
            },
            ensure_ascii=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
