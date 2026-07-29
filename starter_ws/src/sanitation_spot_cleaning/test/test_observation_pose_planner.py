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


def test_planner_fails_closed_when_target_center_is_in_keepout_or_outside_cleanable():
    planner = ObservationPosePlanner(PlannerConstraints(arc_samples=3, standoff_steps=1))
    target_keepout = ((1.8, -0.2), (2.2, -0.2), (2.2, 0.2), (1.8, 0.2))
    assert planner.plan(
        region=REGION,
        covariance_trace=0.0,
        camera=CAMERA,
        cleanable_polygon=CLEANABLE,
        keepout_polygons=(target_keepout,),
        current_pose=CURRENT,
        compute_path=straight_path,
    ) is None
    outside = CandidateRegion("outside", (8.0, 0.0), 0.10, "metal_can")
    assert planner.plan(
        region=outside,
        covariance_trace=0.0,
        camera=CAMERA,
        cleanable_polygon=CLEANABLE,
        keepout_polygons=(),
        current_pose=CURRENT,
        compute_path=straight_path,
    ) is None


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


def test_planner_compensates_camera_mount_yaw():
    yawed_camera = VerificationCameraModel(
        640,
        480,
        1.50098,
        (0.33, 0.28, 0.66),
        math.radians(-50.0),
        0.0,
        0.0,
        mount_rpy_rad=(0.0, math.radians(50.0), math.radians(90.0)),
    )
    result = ObservationPosePlanner(
        PlannerConstraints(standoff_min_m=0.85, standoff_max_m=0.85, standoff_steps=1, arc_samples=1)
    ).plan(
        region=REGION,
        covariance_trace=0.0,
        camera=yawed_camera,
        cleanable_polygon=CLEANABLE,
        keepout_polygons=(),
        current_pose=CURRENT,
        compute_path=straight_path,
    )
    assert result is not None
    assert result.visibility_expected
    assert result.pose.yaw == pytest.approx(-math.radians(90.0))


def test_projection_calibration_offsets_center_and_scales_short_side():
    baseline = ObservationPosePlanner._camera_projection(REGION, CURRENT, CAMERA)
    calibrated_camera = VerificationCameraModel(
        **{
            **CAMERA.__dict__,
            "projection_center_offset_px": (-13.0, 12.0),
            "projection_short_side_scale": 1.4,
        }
    )
    calibrated = ObservationPosePlanner._camera_projection(
        REGION, CURRENT, calibrated_camera
    )
    assert calibrated[0] == pytest.approx(baseline[0] * 1.4)
    assert calibrated[1][0] + calibrated[1][2] == pytest.approx(
        baseline[1][0] + baseline[1][2] - 26.0
    )
    assert calibrated[1][1] + calibrated[1][3] == pytest.approx(
        baseline[1][1] + baseline[1][3] + 24.0
    )


def test_projection_center_affine_maps_raw_center_before_offsets():
    baseline = ObservationPosePlanner._camera_projection(REGION, CURRENT, CAMERA)
    raw_center_x = (baseline[1][0] + baseline[1][2]) / 2.0
    raw_center_y = (baseline[1][1] + baseline[1][3]) / 2.0
    calibrated_camera = VerificationCameraModel(
        **{
            **CAMERA.__dict__,
            "projection_center_affine": (0.9, 0.1, 4.0, -0.2, 1.1, 8.0),
        }
    )
    calibrated = ObservationPosePlanner._camera_projection(
        REGION, CURRENT, calibrated_camera
    )
    calibrated_x = (calibrated[1][0] + calibrated[1][2]) / 2.0
    calibrated_y = (calibrated[1][1] + calibrated[1][3]) / 2.0
    assert calibrated_x == pytest.approx(0.9 * raw_center_x + 0.1 * raw_center_y + 4.0)
    assert calibrated_y == pytest.approx(-0.2 * raw_center_x + 1.1 * raw_center_y + 8.0)


def test_projection_roi_margin_does_not_change_predicted_target_short_side():
    baseline = ObservationPosePlanner._camera_projection(REGION, CURRENT, CAMERA)
    expanded_camera = VerificationCameraModel(
        **{**CAMERA.__dict__, "projection_roi_margin_px": 15.0}
    )
    expanded = ObservationPosePlanner._camera_projection(
        REGION, CURRENT, expanded_camera
    )
    assert expanded[0] == pytest.approx(baseline[0])
    assert expanded[1][0] == pytest.approx(baseline[1][0] - 15.0)
    assert expanded[1][2] == pytest.approx(baseline[1][2] + 15.0)


def test_class_short_side_correction_uses_only_runtime_geometry_features():
    baseline = ObservationPosePlanner._camera_projection(
        REGION, CURRENT, CAMERA
    )
    raw_center_x = (baseline[1][0] + baseline[1][2]) / 2.0
    raw_center_y = (baseline[1][1] + baseline[1][3]) / 2.0
    coefficients = (1.0, 0.2, -0.1, 0.3, 0.4)
    corrected_camera = VerificationCameraModel(
        **{
            **CAMERA.__dict__,
            "class_short_side_correction": (
                ("metal_can", *coefficients),
            ),
        }
    )
    corrected = ObservationPosePlanner._camera_projection(
        REGION, CURRENT, corrected_camera
    )
    expected_factor = (
        coefficients[0]
        + coefficients[1] * (raw_center_x - 320.0) / 200.0
        + coefficients[2] * (raw_center_y - 240.0) / 120.0
        + coefficients[3] * baseline[0] / 60.0
        + coefficients[4] * REGION.target_size_m / 0.30
    )
    assert corrected[0] == pytest.approx(baseline[0] * expected_factor)


def test_class_projection_calibration_uses_generic_class_not_truth_pose():
    calibrated_camera = VerificationCameraModel(
        **{
            **CAMERA.__dict__,
            "class_projection_calibration": (
                ("metal_can", -1.2, 9.4, 0.85),
            ),
        }
    )
    baseline = ObservationPosePlanner._camera_projection(REGION, CURRENT, CAMERA)
    calibrated = ObservationPosePlanner._camera_projection(
        REGION, CURRENT, calibrated_camera
    )
    assert calibrated[0] == pytest.approx(baseline[0] * 0.85)
    assert calibrated[1][0] + calibrated[1][2] == pytest.approx(
        baseline[1][0] + baseline[1][2] - 2.4
    )
