"""ROS-independent product-observation bridge for formal active cleaning."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import yaml

from .models import BeliefSnapshot, KnownTarget


UNKNOWN = -1
OBSERVED_CLEAN = 1
MIN_DIRT_VALUE = 2


class FormalObservationError(RuntimeError):
    """Raised when a public map or product observation violates the contract."""


@dataclass(frozen=True)
class PublicPlanningMap:
    frame_id: str
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    traversable: tuple[bool, ...]
    outer_polygon: tuple[tuple[float, float], ...]
    keepout_polygons: tuple[tuple[tuple[float, float], ...], ...]

    @classmethod
    def load(
        cls,
        occupancy_yaml: str | Path,
        mission_geometry_yaml: str | Path,
        materialization_contract_yaml: str | Path,
    ) -> "PublicPlanningMap":
        occupancy_path = Path(occupancy_yaml).resolve()
        occupancy = _yaml_mapping(occupancy_path)
        mission = _yaml_mapping(Path(mission_geometry_yaml))
        materialization = _yaml_mapping(Path(materialization_contract_yaml))
        if (
            materialization.get("evaluator_truth_used") is not False
            or materialization.get("dirt_truth_used") is not False
        ):
            raise FormalObservationError("public map materialization boundary is invalid")
        if mission.get("frame_id") != "map":
            raise FormalObservationError("mission geometry must use the map frame")
        image_name = occupancy.get("image")
        if not isinstance(image_name, str) or not image_name:
            raise FormalObservationError("occupancy image path is missing")
        image_path = (occupancy_path.parent / image_name).resolve()
        width, height, pixels = _read_pgm(image_path)
        try:
            resolution = float(occupancy["resolution"])
            origin = occupancy["origin"]
            origin_x, origin_y = float(origin[0]), float(origin[1])
            occupied_threshold = float(occupancy["occupied_thresh"])
            negate = int(occupancy.get("negate", 0))
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            raise FormalObservationError("occupancy metadata is incomplete") from exc
        if resolution <= 0.0 or not all(
            math.isfinite(value) for value in (resolution, origin_x, origin_y)
        ):
            raise FormalObservationError("occupancy geometry is invalid")
        if negate not in (0, 1):
            raise FormalObservationError("occupancy negate must be zero or one")
        traversable_rows: list[bool] = []
        for image_row in range(height - 1, -1, -1):
            row_start = image_row * width
            for pixel in pixels[row_start : row_start + width]:
                occupancy_probability = (
                    pixel / 255.0 if negate else (255 - pixel) / 255.0
                )
                traversable_rows.append(occupancy_probability < occupied_threshold)
        outer = _polygon(mission.get("outer_polygon"), "outer_polygon")
        keepouts = tuple(
            _polygon(item, "keepout_polygon")
            for item in mission.get("keepout_polygons", ())
        )
        return cls(
            frame_id="map",
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            traversable=tuple(traversable_rows),
            outer_polygon=outer,
            keepout_polygons=keepouts,
        )

    def cell_index(self, x: float, y: float) -> int | None:
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        column = math.floor((x - self.origin_x) / self.resolution)
        row = math.floor((y - self.origin_y) / self.resolution)
        if column < 0 or column >= self.width or row < 0 or row >= self.height:
            return None
        return row * self.width + column

    def point_is_traversable(self, x: float, y: float) -> bool:
        index = self.cell_index(x, y)
        return index is not None and self.traversable[index]


@dataclass(frozen=True)
class ProductTargetObservation:
    target_id: str
    x: float
    y: float
    confidence: float
    source_backend: str
    track_state: str
    in_keepout: bool


@dataclass(frozen=True)
class MaskUpdate:
    accepted: bool
    reason: str
    observed_cells: int
    dirty_cells: int


class FormalObservationBridgeCore:
    """Accumulate only map-projected product observations into planner belief."""

    def __init__(self, planning_map: PublicPlanningMap, *, min_target_confidence: float):
        if not 0.0 <= min_target_confidence <= 1.0:
            raise ValueError("min_target_confidence must be in [0, 1]")
        self.map = planning_map
        self.min_target_confidence = float(min_target_confidence)
        cell_count = planning_map.width * planning_map.height
        self._observed = [False] * cell_count
        self._known_ground_dirt = [0.0] * cell_count
        self._targets: dict[str, KnownTarget] = {}

    def update_projected_mask(
        self,
        *,
        frame_id: str,
        width: int,
        height: int,
        encoding: str,
        step: int,
        data: bytes | bytearray | Sequence[int],
    ) -> MaskUpdate:
        """
        Merge a trinary map-grid image.

        Row zero is the occupancy-grid row at the public map origin. Value 0
        is unobserved, 1 is observed clean, and 2..255 is dirt confidence.
        """
        if frame_id != self.map.frame_id:
            return MaskUpdate(False, "frame_mismatch", 0, 0)
        if width != self.map.width or height != self.map.height:
            return MaskUpdate(False, "dimension_mismatch", 0, 0)
        if encoding not in {"mono8", "8UC1"}:
            return MaskUpdate(False, "encoding_mismatch", 0, 0)
        if step != width:
            return MaskUpdate(False, "non_contiguous_mask", 0, 0)
        values = bytes(data)
        if len(values) != width * height:
            return MaskUpdate(False, "payload_size_mismatch", 0, 0)
        observed_cells = 0
        dirty_cells = 0
        for index, value in enumerate(values):
            if not self.map.traversable[index] or value == 0:
                continue
            self._observed[index] = True
            observed_cells += 1
            if value == OBSERVED_CLEAN:
                self._known_ground_dirt[index] = 0.0
            else:
                confidence = (value - OBSERVED_CLEAN) / (255 - OBSERVED_CLEAN)
                self._known_ground_dirt[index] = confidence
                dirty_cells += 1
        return MaskUpdate(True, "accepted", observed_cells, dirty_cells)

    def replace_targets(
        self, targets: Iterable[ProductTargetObservation]
    ) -> tuple[ProductTargetObservation, ...]:
        accepted: list[ProductTargetObservation] = []
        next_targets: dict[str, KnownTarget] = {}
        for target in targets:
            backend = target.source_backend.strip().lower()
            if not target.target_id or backend in {"ground_truth", "evaluator"}:
                continue
            if target.track_state.upper() not in {
                "CONFIRMED",
                "QUEUED",
                "APPROACHING",
                "CLEANING",
                "CLEANED",
                "IN_BIN",
            }:
                continue
            if not all(
                math.isfinite(value)
                for value in (target.x, target.y, target.confidence)
            ):
                continue
            if not 0.0 <= target.confidence <= 1.0:
                continue
            if target.confidence < self.min_target_confidence or target.in_keepout:
                continue
            if not self.map.point_is_traversable(target.x, target.y):
                continue
            cleared = target.track_state.upper() in {"CLEARED", "IN_BIN"}
            next_targets[target.target_id] = KnownTarget(
                target_id=target.target_id,
                x=target.x,
                y=target.y,
                cleared=cleared,
                attempts=0,
            )
            accepted.append(target)
        self._targets = next_targets
        return tuple(accepted)

    def belief_snapshot(self) -> BeliefSnapshot:
        return BeliefSnapshot(
            width=self.map.width,
            height=self.map.height,
            origin=(self.map.origin_x, self.map.origin_y),
            resolution=self.map.resolution,
            traversable=self.map.traversable,
            observed=tuple(self._observed),
            known_ground_dirt=tuple(self._known_ground_dirt),
            known_targets=tuple(self._targets[key] for key in sorted(self._targets)),
        )

    def occupancy_grid_values(self) -> tuple[int, ...]:
        result: list[int] = []
        for free, observed, dirt in zip(
            self.map.traversable, self._observed, self._known_ground_dirt
        ):
            if not free or not observed:
                result.append(UNKNOWN)
            elif dirt <= 0.0:
                result.append(0)
            else:
                result.append(max(1, min(100, math.ceil(dirt * 100.0))))
        return tuple(result)


def _yaml_mapping(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise FormalObservationError(f"unable to read public map input: {path}") from exc
    if not isinstance(value, dict):
        raise FormalObservationError(f"expected YAML mapping: {path}")
    return value


def _polygon(value: Any, name: str) -> tuple[tuple[float, float], ...]:
    try:
        points = tuple((float(point[0]), float(point[1])) for point in value)
    except (TypeError, ValueError, IndexError) as exc:
        raise FormalObservationError(f"{name} is invalid") from exc
    if len(points) < 3 or not all(
        math.isfinite(coordinate) for point in points for coordinate in point
    ):
        raise FormalObservationError(f"{name} is invalid")
    return points


def _read_pgm(path: Path) -> tuple[int, int, bytes]:
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise FormalObservationError(f"unable to read occupancy image: {path}") from exc
    tokens: list[bytes] = []
    index = 0
    while len(tokens) < 4:
        while index < len(raw) and raw[index] in b" \t\r\n":
            index += 1
        if index < len(raw) and raw[index] == ord("#"):
            while index < len(raw) and raw[index] not in b"\r\n":
                index += 1
            continue
        start = index
        while index < len(raw) and raw[index] not in b" \t\r\n":
            index += 1
        if start == index:
            raise FormalObservationError("PGM header is incomplete")
        tokens.append(raw[start:index])
    try:
        magic, width_raw, height_raw, max_raw = tokens
        width, height, maximum = int(width_raw), int(height_raw), int(max_raw)
    except ValueError as exc:
        raise FormalObservationError("PGM header is invalid") from exc
    if magic != b"P5" or width <= 0 or height <= 0 or maximum != 255:
        raise FormalObservationError("only 8-bit binary PGM occupancy maps are supported")
    if index >= len(raw) or raw[index] not in b" \t\r\n":
        raise FormalObservationError("PGM header has no payload separator")
    if raw[index : index + 2] == b"\r\n":
        index += 2
    else:
        index += 1
    pixels = raw[index:]
    if len(pixels) != width * height:
        raise FormalObservationError("PGM payload size does not match dimensions")
    return width, height, pixels
