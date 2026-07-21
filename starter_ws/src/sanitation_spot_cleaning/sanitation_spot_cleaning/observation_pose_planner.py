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
    mount_rpy_rad: tuple[float, float, float] | None = None
    fx_px: float | None = None
    fy_px: float | None = None
    cx_px: float | None = None
    cy_px: float | None = None

    @property
    def effective_mount_rpy_rad(self) -> tuple[float, float, float]:
        # Legacy Stage5BR5 reports a conventional negative downward pitch,
        # while the Gazebo +X optical convention uses positive URDF Y rotation.
        return self.mount_rpy_rad or (0.0, -self.pitch_rad, 0.0)


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
    maximum_costmap_footprint_cost: float = 252.0
    require_polygon_checks: bool = False
    require_costmap_footprint_cost: bool = False
    require_pose_dependent_self_overlap: bool = False


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
    costmap_footprint_cost: float | None
    footprint_polygon_xy_m: tuple[Point, ...]
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


def _orientation(first: Point, second: Point, third: Point) -> float:
    return (second[0] - first[0]) * (third[1] - first[1]) - (second[1] - first[1]) * (third[0] - first[0])


def _segments_intersect(a: Point, b: Point, c: Point, d: Point, epsilon: float = 1e-9) -> bool:
    ab_c, ab_d = _orientation(a, b, c), _orientation(a, b, d)
    cd_a, cd_b = _orientation(c, d, a), _orientation(c, d, b)
    if ((ab_c > epsilon and ab_d < -epsilon) or (ab_c < -epsilon and ab_d > epsilon)) and (
        (cd_a > epsilon and cd_b < -epsilon) or (cd_a < -epsilon and cd_b > epsilon)
    ):
        return True
    return any(
        abs(value) <= epsilon and _distance_to_segment(point, first, second) <= epsilon
        for value, point, first, second in (
            (ab_c, c, a, b), (ab_d, d, a, b), (cd_a, a, c, d), (cd_b, b, c, d)
        )
    )


def _polygons_intersect(first: Polygon, second: Polygon) -> bool:
    if any(_point_in_polygon(point, second) for point in first):
        return True
    if any(_point_in_polygon(point, first) for point in second):
        return True
    return any(
        _segments_intersect(first[index], first[(index + 1) % len(first)], second[other], second[(other + 1) % len(second)])
        for index in range(len(first)) for other in range(len(second))
    )


def _transform_polygon(polygon: Polygon, pose: Pose2D) -> tuple[Point, ...]:
    cosine, sine = math.cos(pose.yaw), math.sin(pose.yaw)
    return tuple((pose.x + cosine * x - sine * y, pose.y + sine * x + cosine * y) for x, y in polygon)


def _polygon_clearance(polygon: Polygon, boundary: Polygon, keepouts: Sequence[Polygon]) -> float | None:
    if not polygon or any(not _point_in_polygon(point, boundary) for point in polygon):
        return None
    if any(_polygons_intersect(polygon, keepout) for keepout in keepouts):
        return None
    boundary_clearance = min(_boundary_distance(point, boundary) for point in polygon)
    keepout_clearance = [
        min(_boundary_distance(point, keepout) for point in polygon)
        for keepout in keepouts
    ]
    return min([boundary_clearance, *keepout_clearance]) if keepout_clearance else boundary_clearance


def _rotation_matrix_rpy(roll: float, pitch: float, yaw: float) -> tuple[tuple[float, float, float], ...]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _transpose_multiply(matrix: tuple[tuple[float, float, float], ...], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(sum(matrix[row][column] * vector[row] for row in range(3)) for column in range(3))


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
        world_delta = (region.center_xy_m[0] - camera_x, region.center_xy_m[1] - camera_y, -camera.mount_xyz_m[2])
        base_delta = (
            cosine * world_delta[0] + sine * world_delta[1],
            -sine * world_delta[0] + cosine * world_delta[1],
            world_delta[2],
        )
        camera_delta = _transpose_multiply(_rotation_matrix_rpy(*camera.effective_mount_rpy_rad), base_delta)
        forward, lateral, vertical = camera_delta
        focal_x = camera.fx_px or camera.width_px / (2.0 * math.tan(camera.horizontal_fov_rad / 2.0))
        focal_y = camera.fy_px or focal_x
        center_x = (camera.cx_px if camera.cx_px is not None else camera.width_px / 2.0) - focal_x * lateral / max(forward, 1e-6)
        center_y = (camera.cy_px if camera.cy_px is not None else camera.height_px / 2.0) - focal_y * vertical / max(forward, 1e-6)
        distance = max(math.sqrt(sum(value * value for value in camera_delta)), 1e-6)
        viewing_angle = math.atan2(math.hypot(lateral, vertical), max(forward, 1e-6))
        short_side = min(focal_x, focal_y) * region.target_size_m / max(forward, 1e-6)
        half = short_side / 2.0
        roi = (center_x - half, center_y - half, center_x + half, center_y + half)
        visible = (
            forward > 0.0
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
        candidate_footprint: Polygon = (),
        footprint_cost: Callable[[Pose2D, Polygon], float | None] | None = None,
        self_overlap_estimator: Callable[[Pose2D, tuple[float, float, float, float]], tuple[float, float]] | None = None,
    ) -> PlannedObservationPose | None:
        if covariance_trace > self.constraints.maximum_covariance_trace:
            return None
        if self.constraints.require_polygon_checks and len(candidate_footprint) < 3:
            return None
        if self.constraints.require_costmap_footprint_cost and footprint_cost is None:
            return None
        if self.constraints.require_pose_dependent_self_overlap and self_overlap_estimator is None:
            return None
        if self_overlap_estimator is None and camera.predicted_self_pixel_fraction > self.constraints.maximum_self_pixel_fraction:
            return None
        if self_overlap_estimator is None and camera.predicted_target_self_overlap > self.constraints.maximum_target_self_overlap:
            return None
        keepouts = tuple(keepout_polygons)
        options = []
        distances = [
            self.constraints.standoff_min_m + index * (self.constraints.standoff_max_m - self.constraints.standoff_min_m) / max(self.constraints.standoff_steps - 1, 1)
            for index in range(self.constraints.standoff_steps)
        ]
        angles = [0.0] if self.constraints.arc_samples == 1 else [
            -self.constraints.arc_half_angle_rad + index * 2.0 * self.constraints.arc_half_angle_rad / (self.constraints.arc_samples - 1)
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
                transformed_footprint = _transform_polygon(candidate_footprint, pose) if candidate_footprint else ()
                clearance = (
                    _polygon_clearance(transformed_footprint, cleanable_polygon, keepouts)
                    if transformed_footprint else self._clearance((x, y), cleanable_polygon, keepouts)
                )
                if clearance is None or clearance < self.constraints.minimum_clearance_m:
                    continue
                short_side, roi, viewing_angle, visible = self._camera_projection(region, pose, camera)
                if not visible or short_side < camera.minimum_target_short_side_px:
                    continue
                self_pixels, target_overlap = (
                    self_overlap_estimator(pose, roi) if self_overlap_estimator is not None
                    else (camera.predicted_self_pixel_fraction, camera.predicted_target_self_overlap)
                )
                if self_pixels > self.constraints.maximum_self_pixel_fraction or target_overlap > self.constraints.maximum_target_self_overlap:
                    continue
                cost = footprint_cost(pose, transformed_footprint) if footprint_cost is not None else None
                if self.constraints.require_costmap_footprint_cost and cost is None:
                    continue
                if cost is not None and cost > self.constraints.maximum_costmap_footprint_cost:
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
                    expected_self_pixel_fraction=self_pixels,
                    expected_target_self_overlap=target_overlap,
                    viewing_angle_rad=viewing_angle,
                    standoff_m=standoff,
                    path_length_m=length,
                    turning_cost_rad=turning,
                    clearance_m=clearance,
                    costmap_footprint_cost=cost,
                    footprint_polygon_xy_m=transformed_footprint,
                    visibility_expected=visible,
                    score=score,
                    path=path_tuple,
                ))
        return max(options, key=lambda item: (item.score, -item.path_length_m)) if options else None
