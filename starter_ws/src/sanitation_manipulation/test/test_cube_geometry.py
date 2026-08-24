import math

import pytest

from sanitation_manipulation.cube_geometry import (
    CubeCandidate,
    CubeDetectorConfig,
    CubePointCloudDetector,
    fit_ground_ransac,
    generate_top_grasps,
)


def _ground_points(slope_x=0.0):
    return [
        (x * 0.04, y * 0.04, slope_x * x * 0.04)
        for x in range(-12, 13)
        for y in range(-8, 9)
    ]


def _cube_top(center_x, center_y, yaw_rad, slope_x=0.0, edge=0.030, height=None):
    height = edge if height is None else height
    output = []
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    for row in range(7):
        for column in range(7):
            u = -edge / 2.0 + row * edge / 6.0
            v = -edge / 2.0 + column * edge / 6.0
            x = center_x + cosine * u - sine * v
            y = center_y + sine * u + cosine * v
            output.append((x, y, slope_x * x + height))
    return output


def test_ransac_finds_tilted_ground_and_ignores_nonfinite_points():
    cloud = _ground_points(0.03) + _cube_top(0.35, 0.08, 0.2, 0.03)
    cloud += [(float("nan"), 0.0, 0.0), (0.0, float("inf"), 0.0)]
    plane = fit_ground_ransac(cloud)
    assert plane.inlier_count >= 400
    assert plane.rms_m < 1e-6
    assert plane.height_at(0.4, 0.0) == pytest.approx(0.012, abs=1e-5)


def test_cube_detector_recovers_rotated_30mm_target_and_rejects_large_object():
    cloud = _ground_points()
    cloud += _cube_top(0.35, 0.08, math.radians(28.0))
    cloud += _cube_top(-0.30, -0.12, 0.0, edge=0.060, height=0.030)
    result = CubePointCloudDetector().detect(cloud)
    assert len(result.candidates) == 1
    cube = result.candidates[0]
    assert cube.center_m == pytest.approx((0.35, 0.08, 0.015), abs=0.004)
    assert sorted(cube.size_m[:2]) == pytest.approx([0.030, 0.030], abs=0.003)
    assert cube.size_m[2] == pytest.approx(0.030, abs=0.003)
    assert cube.dimension_error_m < 0.003
    assert result.rejected_cluster_count == 1


def test_noise_cluster_below_minimum_count_is_rejected():
    cloud = _ground_points() + [(0.20 + index * 0.001, 0.0, 0.03) for index in range(5)]
    result = CubePointCloudDetector().detect(cloud)
    assert result.candidates == ()
    assert result.rejected_cluster_count == 1


def test_detector_configuration_rejects_invalid_ranges():
    with pytest.raises(ValueError):
        CubeDetectorConfig(min_height_m=0.05, max_height_m=0.04)
    with pytest.raises(ValueError):
        fit_ground_ransac([(0.0, 0.0, 0.0)])


def test_top_grasps_are_pose_candidates_not_robot_specific_ik():
    cube = CubeCandidate(
        center_m=(0.4, -0.1, 0.015),
        size_m=(0.03, 0.03, 0.03),
        yaw_rad=0.25,
        point_count=40,
        dimension_error_m=0.001,
    )
    grasps = generate_top_grasps("cube-7", cube)
    assert len(grasps) == 2
    assert all(row.placeholder_geometry for row in grasps)
    assert all(row.opening_m == pytest.approx(0.042) for row in grasps)
    assert grasps[0].pregrasp_pose.position_m[2] > grasps[0].grasp_pose.position_m[2]
    assert grasps[0].lift_pose.position_m[2] > grasps[0].pregrasp_pose.position_m[2]
    for row in grasps:
        assert sum(value * value for value in row.grasp_pose.quaternion_xyzw) == pytest.approx(1.0)
