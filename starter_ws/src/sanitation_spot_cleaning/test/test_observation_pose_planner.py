import math

import pytest

from sanitation_spot_cleaning.observation_pose_planner import (
    CandidateRegion,
    ObservationPosePlanner,
    PlannerConstraints,
    Pose2D,
    VerificationCameraModel,
)


REGION = CandidateRegion("candidate", (2.0, 0.0), 0.10, "metal_can")
CLEANABLE = ((-1.0, -3.0), (5.0, -3.0), (5.0, 3.0), (-1.0, 3.0))
CAMERA = VerificationCameraModel(640, 480, 1.50098, (0.67, 0.0, 0.48), math.radians(-50.0), 0.01, 0.01)
CURRENT = Pose2D(0.0, 0.0, 0.0)


def straight_path(goal):
    return (CURRENT, goal)


def test_planner_selects_reachable_visible_pose_without_gt_pose_passthrough():
    result = ObservationPosePlanner().plan(
        region=REGION,
        covariance_trace=0.002,
        camera=CAMERA,
        cleanable_polygon=CLEANABLE,
        keepout_polygons=(),
        current_pose=CURRENT,
        compute_path=straight_path,
    )
    assert result is not None
    assert result.pose.x != pytest.approx(REGION.center_xy_m[0])
    assert result.visibility_expected
    assert result.expected_target_short_side_px >= 12.0
    assert result.path_length_m > 0.0


def test_planner_rejects_keepout_covariance_self_overlap_and_no_path():
    planner = ObservationPosePlanner(PlannerConstraints(arc_samples=5, standoff_steps=2))
    full_keepout = ((-1.0, -3.0), (5.0, -3.0), (5.0, 3.0), (-1.0, 3.0))
    common = dict(region=REGION, camera=CAMERA, cleanable_polygon=CLEANABLE, current_pose=CURRENT)
    assert planner.plan(**common, covariance_trace=0.1, keepout_polygons=(), compute_path=straight_path) is None
    assert planner.plan(**common, covariance_trace=0.0, keepout_polygons=(full_keepout,), compute_path=straight_path) is None
    blocked_camera = VerificationCameraModel(640, 480, 1.50098, (0.67, 0.0, 0.48), math.radians(-50.0), 0.06, 0.0)
    assert planner.plan(**{**common, "camera": blocked_camera}, covariance_trace=0.0, keepout_polygons=(), compute_path=straight_path) is None
    assert planner.plan(**common, covariance_trace=0.0, keepout_polygons=(), compute_path=lambda _: None) is None


def test_planner_serializes_auditable_metrics():
    result = ObservationPosePlanner().plan(
        region=REGION,
        covariance_trace=0.0,
        camera=CAMERA,
        cleanable_polygon=CLEANABLE,
        keepout_polygons=(),
        current_pose=CURRENT,
        compute_path=straight_path,
    )
    record = result.to_record()
    assert set(record) >= {"expected_roi_xyxy", "expected_self_pixel_fraction", "path_length_m", "clearance_m", "path"}
    assert len(record["path"]) == 2
