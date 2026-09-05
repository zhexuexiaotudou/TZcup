"""ROS-independent fail-closed validation for formal planner trajectories."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Sequence

import yaml


Point2D = tuple[float, float]
Polygon2D = tuple[Point2D, ...]


class FormalTrajectoryError(RuntimeError):
    """Raised when formal mission geometry is unavailable or malformed."""


@dataclass(frozen=True)
class PathPose:
    x: float
    y: float
    quaternion: tuple[float, float, float, float]
    frame_id: str


@dataclass(frozen=True)
class PathDecision:
    accepted: bool
    reason: str
    path_length: float
    pose_count: int


@dataclass(frozen=True)
class FormalTrajectoryGate:
    frame_id: str
    outer_polygon: Polygon2D
    keepout_polygons: tuple[Polygon2D, ...]
    max_segment_length: float = 0.50
    max_path_length: float = 10000.0
    max_pose_count: int = 100000

    @classmethod
    def from_mission_geometry(
        cls,
        path: str | Path,
        *,
        max_segment_length: float,
        max_path_length: float,
        max_pose_count: int,
    ) -> "FormalTrajectoryGate":
        try:
            value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as exc:
            raise FormalTrajectoryError("unable to read mission geometry") from exc
        if not isinstance(value, dict) or value.get("frame_id") != "map":
            raise FormalTrajectoryError("mission geometry must be a map-frame mapping")
        outer = _polygon(value.get("outer_polygon"), "outer_polygon")
        keepouts = tuple(
            _polygon(item, "keepout_polygon")
            for item in value.get("keepout_polygons", ())
        )
        return cls(
            frame_id="map",
            outer_polygon=outer,
            keepout_polygons=keepouts,
            max_segment_length=float(max_segment_length),
            max_path_length=float(max_path_length),
            max_pose_count=int(max_pose_count),
        )

    def __post_init__(self) -> None:
        if len(self.outer_polygon) < 3:
            raise ValueError("outer_polygon must contain at least three points")
        if not math.isfinite(self.max_segment_length) or self.max_segment_length <= 0:
            raise ValueError("max_segment_length must be finite and positive")
        if not math.isfinite(self.max_path_length) or self.max_path_length <= 0:
            raise ValueError("max_path_length must be finite and positive")
        if self.max_pose_count < 2:
            raise ValueError("max_pose_count must be at least two")

    def validate(self, *, path_frame_id: str, poses: Sequence[PathPose]) -> PathDecision:
        if path_frame_id != self.frame_id:
            return PathDecision(False, "path_frame_mismatch", 0.0, len(poses))
        if len(poses) < 2:
            return PathDecision(False, "path_too_short", 0.0, len(poses))
        if len(poses) > self.max_pose_count:
            return PathDecision(False, "path_pose_limit_exceeded", 0.0, len(poses))
        points: list[Point2D] = []
        for pose in poses:
            if pose.frame_id != self.frame_id:
                return PathDecision(False, "pose_frame_mismatch", 0.0, len(poses))
            values = (pose.x, pose.y, *pose.quaternion)
            if not all(math.isfinite(value) for value in values):
                return PathDecision(False, "non_finite_pose", 0.0, len(poses))
            norm = math.sqrt(sum(value * value for value in pose.quaternion))
            if not 0.99 <= norm <= 1.01:
                return PathDecision(False, "invalid_quaternion", 0.0, len(poses))
            point = (pose.x, pose.y)
            if not _point_strictly_inside(point, self.outer_polygon):
                return PathDecision(False, "outside_geofence", 0.0, len(poses))
            if any(_point_in_or_on_polygon(point, polygon) for polygon in self.keepout_polygons):
                return PathDecision(False, "inside_keepout", 0.0, len(poses))
            points.append(point)
        total = 0.0
        for first, second in zip(points, points[1:]):
            segment_length = math.dist(first, second)
            if segment_length <= 1.0e-9:
                return PathDecision(False, "duplicate_consecutive_pose", total, len(poses))
            if segment_length > self.max_segment_length + 1.0e-9:
                return PathDecision(False, "segment_too_long", total, len(poses))
            if _segment_crosses_polygon_boundary(first, second, self.outer_polygon):
                return PathDecision(False, "segment_crosses_geofence", total, len(poses))
            if any(
                _segment_intersects_polygon(first, second, polygon)
                for polygon in self.keepout_polygons
            ):
                return PathDecision(False, "segment_crosses_keepout", total, len(poses))
            total += segment_length
            if total > self.max_path_length + 1.0e-9:
                return PathDecision(False, "path_length_limit_exceeded", total, len(poses))
        return PathDecision(True, "accepted", total, len(poses))


def _polygon(value: Any, name: str) -> Polygon2D:
    try:
        polygon = tuple((float(point[0]), float(point[1])) for point in value)
    except (TypeError, ValueError, IndexError) as exc:
        raise FormalTrajectoryError(f"{name} is invalid") from exc
    if len(polygon) < 3 or not all(
        math.isfinite(value) for point in polygon for value in point
    ):
        raise FormalTrajectoryError(f"{name} is invalid")
    return polygon


def _cross(a: Point2D, b: Point2D, c: Point2D) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _point_on_segment(point: Point2D, first: Point2D, second: Point2D) -> bool:
    return abs(_cross(first, second, point)) <= 1.0e-9 and (
        min(first[0], second[0]) - 1.0e-9
        <= point[0]
        <= max(first[0], second[0]) + 1.0e-9
        and min(first[1], second[1]) - 1.0e-9
        <= point[1]
        <= max(first[1], second[1]) + 1.0e-9
    )


def _point_in_or_on_polygon(point: Point2D, polygon: Polygon2D) -> bool:
    inside = False
    for first, second in zip(polygon, polygon[1:] + polygon[:1]):
        if _point_on_segment(point, first, second):
            return True
        if (first[1] > point[1]) != (second[1] > point[1]):
            intersection_x = (
                (second[0] - first[0])
                * (point[1] - first[1])
                / (second[1] - first[1])
                + first[0]
            )
            if point[0] < intersection_x:
                inside = not inside
    return inside


def _point_strictly_inside(point: Point2D, polygon: Polygon2D) -> bool:
    if any(
        _point_on_segment(point, first, second)
        for first, second in zip(polygon, polygon[1:] + polygon[:1])
    ):
        return False
    return _point_in_or_on_polygon(point, polygon)


def _segments_intersect(a: Point2D, b: Point2D, c: Point2D, d: Point2D) -> bool:
    orientations = (_cross(a, b, c), _cross(a, b, d), _cross(c, d, a), _cross(c, d, b))
    if (
        orientations[0] * orientations[1] < -1.0e-12
        and orientations[2] * orientations[3] < -1.0e-12
    ):
        return True
    return (
        (abs(orientations[0]) <= 1.0e-9 and _point_on_segment(c, a, b))
        or (abs(orientations[1]) <= 1.0e-9 and _point_on_segment(d, a, b))
        or (abs(orientations[2]) <= 1.0e-9 and _point_on_segment(a, c, d))
        or (abs(orientations[3]) <= 1.0e-9 and _point_on_segment(b, c, d))
    )


def _segment_crosses_polygon_boundary(
    first: Point2D, second: Point2D, polygon: Polygon2D
) -> bool:
    return any(
        _segments_intersect(first, second, edge_first, edge_second)
        for edge_first, edge_second in zip(polygon, polygon[1:] + polygon[:1])
    )


def _segment_intersects_polygon(
    first: Point2D, second: Point2D, polygon: Polygon2D
) -> bool:
    if _point_in_or_on_polygon(first, polygon) or _point_in_or_on_polygon(second, polygon):
        return True
    return _segment_crosses_polygon_boundary(first, second, polygon)
