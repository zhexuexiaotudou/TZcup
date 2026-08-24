"""ROS-independent geometry for 30 mm cube detection and top grasps.

The module intentionally consumes plain XYZ samples and returns dataclasses.  A
ROS adapter may translate ``sensor_msgs/PointCloud2`` and ``geometry_msgs/Pose``
at the package boundary without making the algorithm depend on a robot URDF.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import Iterable, Sequence


Point3 = tuple[float, float, float]


@dataclass(frozen=True)
class CubeDetectorConfig:
    cube_edge_m: float = 0.030
    dimension_tolerance_m: float = 0.010
    min_height_m: float = 0.018
    max_height_m: float = 0.045
    ground_distance_m: float = 0.003
    max_ground_tilt_deg: float = 15.0
    cluster_tolerance_m: float = 0.014
    min_cluster_points: int = 12
    ransac_iterations: int = 160
    ransac_seed: int = 301
    obb_angle_step_deg: float = 2.0

    def __post_init__(self) -> None:
        positive = (
            self.cube_edge_m,
            self.dimension_tolerance_m,
            self.min_height_m,
            self.max_height_m,
            self.ground_distance_m,
            self.cluster_tolerance_m,
            self.obb_angle_step_deg,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError("cube detector distances and angle step must be positive")
        if self.min_height_m >= self.max_height_m:
            raise ValueError("min_height_m must be below max_height_m")
        if not 0.0 <= self.max_ground_tilt_deg < 90.0:
            raise ValueError("max_ground_tilt_deg must be in [0, 90)")
        if self.min_cluster_points < 3 or self.ransac_iterations < 1:
            raise ValueError("insufficient cluster or RANSAC sample count")


@dataclass(frozen=True)
class GroundPlane:
    normal: Point3
    offset_m: float
    inlier_count: int
    rms_m: float

    def signed_distance(self, point: Point3) -> float:
        return sum(a * b for a, b in zip(self.normal, point)) + self.offset_m

    def height_at(self, x_m: float, y_m: float) -> float:
        nx, ny, nz = self.normal
        if abs(nz) < 1e-9:
            raise ValueError("ground plane is vertical")
        return -(nx * x_m + ny * y_m + self.offset_m) / nz


@dataclass(frozen=True)
class CubeCandidate:
    center_m: Point3
    size_m: Point3
    yaw_rad: float
    point_count: int
    dimension_error_m: float


@dataclass(frozen=True)
class CubeDetectionResult:
    ground: GroundPlane
    candidates: tuple[CubeCandidate, ...]
    elevated_point_count: int
    rejected_cluster_count: int


@dataclass(frozen=True)
class Pose3D:
    position_m: Point3
    quaternion_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class TopGraspCandidate:
    target_id: str
    pregrasp_pose: Pose3D
    grasp_pose: Pose3D
    lift_pose: Pose3D
    opening_m: float
    yaw_rad: float
    score: float
    placeholder_geometry: bool = True


def _finite_points(points: Iterable[Sequence[float]]) -> list[Point3]:
    output: list[Point3] = []
    for point in points:
        if len(point) < 3:
            continue
        xyz = (float(point[0]), float(point[1]), float(point[2]))
        if all(math.isfinite(value) for value in xyz):
            output.append(xyz)
    return output


def _plane_from_three(a: Point3, b: Point3, c: Point3) -> tuple[Point3, float] | None:
    ab = tuple(b[index] - a[index] for index in range(3))
    ac = tuple(c[index] - a[index] for index in range(3))
    normal = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    magnitude = math.sqrt(sum(value * value for value in normal))
    if magnitude < 1e-10:
        return None
    normal = tuple(value / magnitude for value in normal)
    if normal[2] < 0.0:
        normal = tuple(-value for value in normal)
    offset = -sum(normal[index] * a[index] for index in range(3))
    return normal, offset


def fit_ground_ransac(
    points: Iterable[Sequence[float]], config: CubeDetectorConfig = CubeDetectorConfig()
) -> GroundPlane:
    """Fit a near-horizontal ground plane without NumPy, PCL, or ROS."""

    cloud = _finite_points(points)
    if len(cloud) < 3:
        raise ValueError("at least three finite points are required")
    rng = random.Random(config.ransac_seed)
    min_vertical = math.cos(math.radians(config.max_ground_tilt_deg))
    best: tuple[int, float, Point3, float, list[int]] | None = None
    for _ in range(config.ransac_iterations):
        indices = rng.sample(range(len(cloud)), 3)
        fitted = _plane_from_three(*(cloud[index] for index in indices))
        if fitted is None:
            continue
        normal, offset = fitted
        if normal[2] < min_vertical:
            continue
        distances = [
            abs(sum(normal[index] * point[index] for index in range(3)) + offset)
            for point in cloud
        ]
        inliers = [
            index
            for index, distance in enumerate(distances)
            if distance <= config.ground_distance_m
        ]
        if not inliers:
            continue
        squared_error = sum(distances[index] ** 2 for index in inliers)
        candidate = (len(inliers), -squared_error, normal, offset, inliers)
        if best is None or candidate[:2] > best[:2]:
            best = candidate
    if best is None or best[0] < 3:
        raise ValueError("no near-horizontal ground plane found")

    _, _, normal, offset, inliers = best
    # Refine only the offset.  Retaining the sampled normal keeps this dependency
    # free; downstream acceptance is tolerant to the millimetre-scale residual.
    offsets = sorted(
        -sum(normal[index] * cloud[row][index] for index in range(3))
        for row in inliers
    )
    offset = offsets[len(offsets) // 2]
    residuals = [
        sum(normal[index] * cloud[row][index] for index in range(3)) + offset
        for row in inliers
    ]
    rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    return GroundPlane(normal, offset, len(inliers), rms)


def _cluster_points(points: list[Point3], tolerance_m: float) -> list[list[Point3]]:
    if not points:
        return []
    cells: dict[tuple[int, int, int], list[int]] = {}
    for index, point in enumerate(points):
        cell = tuple(math.floor(value / tolerance_m) for value in point)
        cells.setdefault(cell, []).append(index)
    tolerance_squared = tolerance_m * tolerance_m
    unseen = set(range(len(points)))
    clusters: list[list[Point3]] = []
    while unseen:
        seed = min(unseen)
        unseen.remove(seed)
        queue = [seed]
        members = [seed]
        while queue:
            current = queue.pop()
            point = points[current]
            cell = tuple(math.floor(value / tolerance_m) for value in point)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        for neighbour in cells.get(
                            (cell[0] + dx, cell[1] + dy, cell[2] + dz), ()
                        ):
                            if neighbour not in unseen:
                                continue
                            other = points[neighbour]
                            distance_squared = sum(
                                (point[index] - other[index]) ** 2
                                for index in range(3)
                            )
                            if distance_squared <= tolerance_squared:
                                unseen.remove(neighbour)
                                queue.append(neighbour)
                                members.append(neighbour)
        clusters.append([points[index] for index in members])
    return clusters


def _minimum_xy_box(
    points: list[Point3], angle_step_deg: float
) -> tuple[float, float, float, float, float]:
    best: tuple[float, float, float, float, float, float] | None = None
    angle = 0.0
    while angle < math.pi / 2.0 - 1e-12:
        cosine, sine = math.cos(angle), math.sin(angle)
        rotated = [
            (cosine * point[0] + sine * point[1], -sine * point[0] + cosine * point[1])
            for point in points
        ]
        minimum_u = min(point[0] for point in rotated)
        maximum_u = max(point[0] for point in rotated)
        minimum_v = min(point[1] for point in rotated)
        maximum_v = max(point[1] for point in rotated)
        width, length = maximum_u - minimum_u, maximum_v - minimum_v
        area = width * length
        candidate = (area, angle, minimum_u, maximum_u, minimum_v, maximum_v)
        if best is None or candidate[0] < best[0]:
            best = candidate
        angle += math.radians(angle_step_deg)
    assert best is not None
    _, angle, minimum_u, maximum_u, minimum_v, maximum_v = best
    center_u = (minimum_u + maximum_u) / 2.0
    center_v = (minimum_v + maximum_v) / 2.0
    cosine, sine = math.cos(angle), math.sin(angle)
    center_x = cosine * center_u - sine * center_v
    center_y = sine * center_u + cosine * center_v
    return maximum_u - minimum_u, maximum_v - minimum_v, angle, center_x, center_y


class CubePointCloudDetector:
    def __init__(self, config: CubeDetectorConfig = CubeDetectorConfig()) -> None:
        self.config = config

    def detect(self, points: Iterable[Sequence[float]]) -> CubeDetectionResult:
        cloud = _finite_points(points)
        ground = fit_ground_ransac(cloud, self.config)
        elevated = [
            point
            for point in cloud
            if self.config.min_height_m
            <= ground.signed_distance(point)
            <= self.config.max_height_m
        ]
        candidates: list[CubeCandidate] = []
        rejected = 0
        for cluster in _cluster_points(elevated, self.config.cluster_tolerance_m):
            if len(cluster) < self.config.min_cluster_points:
                rejected += 1
                continue
            width, length, yaw, center_x, center_y = _minimum_xy_box(
                cluster, self.config.obb_angle_step_deg
            )
            height = max(ground.signed_distance(point) for point in cluster)
            dimensions = (width, length, height)
            errors = [abs(value - self.config.cube_edge_m) for value in dimensions]
            if max(errors) > self.config.dimension_tolerance_m:
                rejected += 1
                continue
            ground_z = ground.height_at(center_x, center_y)
            candidates.append(
                CubeCandidate(
                    center_m=(center_x, center_y, ground_z + height / 2.0),
                    size_m=dimensions,
                    yaw_rad=yaw,
                    point_count=len(cluster),
                    dimension_error_m=sum(errors) / 3.0,
                )
            )
        candidates.sort(key=lambda candidate: (candidate.center_m[0], candidate.center_m[1]))
        return CubeDetectionResult(
            ground=ground,
            candidates=tuple(candidates),
            elevated_point_count=len(elevated),
            rejected_cluster_count=rejected,
        )


def _top_down_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    # Rz(yaw) * Rx(pi): tool Z points down.  The exact tool convention remains
    # a placeholder until the real end-effector URDF and hand-eye calibration.
    return (math.cos(yaw_rad / 2.0), math.sin(yaw_rad / 2.0), 0.0, 0.0)


def generate_top_grasps(
    target_id: str,
    cube: CubeCandidate,
    approach_clearance_m: float = 0.080,
    lift_clearance_m: float = 0.120,
    finger_clearance_m: float = 0.006,
) -> tuple[TopGraspCandidate, ...]:
    """Generate symmetric top-grasp poses without solving robot-specific IK."""

    if not target_id:
        raise ValueError("target_id must be non-empty")
    if min(approach_clearance_m, lift_clearance_m, finger_clearance_m) <= 0.0:
        raise ValueError("grasp clearances must be positive")
    x_m, y_m, center_z = cube.center_m
    top_z = center_z + cube.size_m[2] / 2.0
    opening = max(cube.size_m[0], cube.size_m[1]) + 2.0 * finger_clearance_m
    score = max(0.0, 1.0 - cube.dimension_error_m / 0.030)
    output: list[TopGraspCandidate] = []
    for yaw in (cube.yaw_rad, cube.yaw_rad + math.pi / 2.0):
        quaternion = _top_down_quaternion(yaw)
        output.append(
            TopGraspCandidate(
                target_id=target_id,
                pregrasp_pose=Pose3D((x_m, y_m, top_z + approach_clearance_m), quaternion),
                grasp_pose=Pose3D((x_m, y_m, top_z), quaternion),
                lift_pose=Pose3D((x_m, y_m, top_z + lift_clearance_m), quaternion),
                opening_m=opening,
                yaw_rad=yaw,
                score=score,
            )
        )
    return tuple(output)
