from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Callable, Iterable, Sequence


Point = tuple[float, float]
Polygon = Sequence[Point]


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class CandidateRegion:
    candidate_id: str
    center_xy_m: Point
    target_size_m: float
    class_id: str


@dataclass(frozen=True)
class VerificationCameraModel:
    width_px: int
    height_px: int
    horizontal_fov_rad: float
    mount_xyz_m: tuple[float, float, float]
    pitch_rad: float
    predicted_self_pixel_fraction: float
    predicted_target_self_overlap: float
    minimum_target_short_side_px: float = 12.0


@dataclass(frozen=True)
class PlannerConstraints:
    standoff_min_m: float = 0.65
    standoff_max_m: float = 1.35
    standoff_steps: int = 4
    arc_half_angle_rad: float = math.radians(70.0)
    arc_samples: int = 15
    minimum_clearance_m: float = 0.15
    maximum_covariance_trace: float = 0.03
    maximum_self_pixel_fraction: float = 0.05
    maximum_target_self_overlap: float = 0.05


@dataclass(frozen=True)
class PlannedObservationPose:
    pose: Pose2D
    expected_target_short_side_px: float
    expected_roi_xyxy: tuple[float, float, float, float]
    expected_self_pixel_fraction: float
    expected_target_self_overlap: float
    viewing_angle_rad: float
    standoff_m: float
    path_length_m: float
    turning_cost_rad: float
    clearance_m: float
    visibility_expected: bool
    score: float
    path: tuple[Pose2D, ...]

    def to_record(self) -> dict:
        payload = asdict(self)
        payload["path"] = [asdict(pose) for pose in self.path]
        return payload


def _point_in_polygon(point: Point, polygon: Polygon) -> bool:
    x, y = point
    inside = False
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        x1, y1 = first
        x2, y2 = second
        if (y1 > y) != (y2 > y):
            crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing_x:
                inside = not inside
    return inside


def _distance_to_segment(point: Point, first: Point, second: Point) -> float:
    px, py = point
    x1, y1 = first
    x2, y2 = second
    dx, dy = x2 - x1, y2 - y1
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.hypot(px - x1, py - y1)
    scale = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / length_squared))
    return math.hypot(px - (x1 + scale * dx), py - (y1 + scale * dy))


def _boundary_distance(point: Point, polygon: Polygon) -> float:
    return min(_distance_to_segment(point, polygon[index], polygon[(index + 1) % len(polygon)]) for index in range(len(polygon)))


def _angle_delta(first: float, second: float) -> float:
    return abs((first - second + math.pi) % (2.0 * math.pi) - math.pi)


def _path_length(path: Sequence[Pose2D]) -> float:
    return sum(math.hypot(second.x - first.x, second.y - first.y) for first, second in zip(path, path[1:]))


class ObservationPosePlanner:
    def __init__(self, constraints: PlannerConstraints = PlannerConstraints()):
        if constraints.arc_samples < 1 or constraints.standoff_steps < 1:
            raise ValueError("planner sampling counts must be positive")
        self.constraints = constraints

    def _clearance(self, point: Point, cleanable: Polygon, keepouts: Iterable[Polygon]) -> float | None:
        if not _point_in_polygon(point, cleanable):
            return None
        cleanable_clearance = _boundary_distance(point, cleanable)
        keepout_clearances = []
        for keepout in keepouts:
            if _point_in_polygon(point, keepout):
                return None
            keepout_clearances.append(_boundary_distance(point, keepout))
        return min([cleanable_clearance, *keepout_clearances]) if keepout_clearances else cleanable_clearance

    @staticmethod
    def _camera_projection(region: CandidateRegion, pose: Pose2D, camera: VerificationCameraModel) -> tuple[float, tuple[float, float, float, float], float, bool]:
        cosine, sine = math.cos(pose.yaw), math.sin(pose.yaw)
        camera_x = pose.x + cosine * camera.mount_xyz_m[0] - sine * camera.mount_xyz_m[1]
        camera_y = pose.y + sine * camera.mount_xyz_m[0] + cosine * camera.mount_xyz_m[1]
        dx, dy = region.center_xy_m[0] - camera_x, region.center_xy_m[1] - camera_y
        distance = max(math.hypot(dx, dy), 1e-6)
        bearing = math.atan2(dy, dx)
        viewing_angle = _angle_delta(bearing, pose.yaw)
        focal_x = camera.width_px / (2.0 * math.tan(camera.horizontal_fov_rad / 2.0))
        vertical_fov = 2.0 * math.atan(math.tan(camera.horizontal_fov_rad / 2.0) * camera.height_px / camera.width_px)
        focal_y = camera.height_px / (2.0 * math.tan(vertical_fov / 2.0))
        short_side = focal_x * region.target_size_m / distance
        target_elevation = -math.atan2(camera.mount_xyz_m[2], distance)
        relative_elevation = target_elevation - camera.pitch_rad
        center_x = camera.width_px / 2.0 + focal_x * math.tan((bearing - pose.yaw + math.pi) % (2.0 * math.pi) - math.pi)
        center_y = camera.height_px / 2.0 - focal_y * math.tan(relative_elevation)
        half = short_side / 2.0
        roi = (center_x - half, center_y - half, center_x + half, center_y + half)
        visible = (
            viewing_angle <= camera.horizontal_fov_rad / 2.0
            and abs(relative_elevation) <= vertical_fov / 2.0
            and roi[0] >= 0.0 and roi[1] >= 0.0
            and roi[2] < camera.width_px and roi[3] < camera.height_px
        )
        return short_side, roi, viewing_angle, visible

    def plan(
        self,
        *,
        region: CandidateRegion,
        covariance_trace: float,
        camera: VerificationCameraModel,
        cleanable_polygon: Polygon,
        keepout_polygons: Iterable[Polygon],
        current_pose: Pose2D,
        compute_path: Callable[[Pose2D], Sequence[Pose2D] | None],
    ) -> PlannedObservationPose | None:
        if covariance_trace > self.constraints.maximum_covariance_trace:
            return None
        if camera.predicted_self_pixel_fraction > self.constraints.maximum_self_pixel_fraction:
            return None
        if camera.predicted_target_self_overlap > self.constraints.maximum_target_self_overlap:
            return None
        keepouts = tuple(keepout_polygons)
        options = []
        distances = [
            self.constraints.standoff_min_m + index * (self.constraints.standoff_max_m - self.constraints.standoff_min_m) / max(self.constraints.standoff_steps - 1, 1)
            for index in range(self.constraints.standoff_steps)
        ]
        angles = [
            -self.constraints.arc_half_angle_rad + index * 2.0 * self.constraints.arc_half_angle_rad / max(self.constraints.arc_samples - 1, 1)
            for index in range(self.constraints.arc_samples)
        ]
        current_bearing = math.atan2(region.center_xy_m[1] - current_pose.y, region.center_xy_m[0] - current_pose.x)
        for standoff in distances:
            for offset in angles:
                radial = current_bearing + math.pi + offset
                x = region.center_xy_m[0] + standoff * math.cos(radial)
                y = region.center_xy_m[1] + standoff * math.sin(radial)
                yaw = math.atan2(region.center_xy_m[1] - y, region.center_xy_m[0] - x)
                pose = Pose2D(x, y, yaw)
                clearance = self._clearance((x, y), cleanable_polygon, keepouts)
                if clearance is None or clearance < self.constraints.minimum_clearance_m:
                    continue
                short_side, roi, viewing_angle, visible = self._camera_projection(region, pose, camera)
                if not visible or short_side < camera.minimum_target_short_side_px:
                    continue
                path = compute_path(pose)
                if not path:
                    continue
                path_tuple = tuple(path)
                length = _path_length(path_tuple)
                turning = _angle_delta(current_pose.yaw, pose.yaw)
                pixel_margin = min(short_side / camera.minimum_target_short_side_px, 4.0)
                score = 4.0 * pixel_margin + 2.0 * min(clearance, 1.0) - length - 0.25 * turning
                options.append(PlannedObservationPose(
                    pose=pose,
                    expected_target_short_side_px=short_side,
                    expected_roi_xyxy=roi,
                    expected_self_pixel_fraction=camera.predicted_self_pixel_fraction,
                    expected_target_self_overlap=camera.predicted_target_self_overlap,
                    viewing_angle_rad=viewing_angle,
                    standoff_m=standoff,
                    path_length_m=length,
                    turning_cost_rad=turning,
                    clearance_m=clearance,
                    visibility_expected=visible,
                    score=score,
                    path=path_tuple,
                ))
        return max(options, key=lambda item: (item.score, -item.path_length_m)) if options else None
