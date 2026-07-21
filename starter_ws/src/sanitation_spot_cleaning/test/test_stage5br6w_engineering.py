import importlib.util
import json
import math
from pathlib import Path

import yaml
import pytest

from sanitation_spot_cleaning.observation_pose_planner import (
    CandidateRegion,
    ObservationPosePlanner,
    PlannerConstraints,
    Pose2D,
    VerificationCameraModel,
)


ROOT = Path(__file__).resolve().parents[4]
SPEC = importlib.util.spec_from_file_location("stage5br6w_profile", ROOT / "scripts" / "stage5br6w_profile.py")
PROFILE_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROFILE_MODULE)


def test_v4_candidate_footprint_is_derived_from_aabb_and_production_envelope():
    mechanics = json.loads((ROOT / "artifacts/stage5br5_20260720_review/camera_mechanics_report.json").read_text(encoding="utf-8"))
    config = yaml.safe_load((ROOT / "starter_ws/src/sanitation_learning/config/stage5br5_active_observation.yaml").read_text(encoding="utf-8"))
    mechanics["production_nav2_footprint_xy_m"] = config["vehicle_collision_geometry"]["production_nav2_footprint_xy_m"]
    profile = yaml.safe_load((ROOT / "starter_ws/src/sanitation_navigation/config/stage5br6w_v4_candidate_footprint.yaml").read_text(encoding="utf-8"))
    derived = PROFILE_MODULE.derive_candidate_footprint(mechanics, profile["mount_bracket_safety_margin_m"])
    for actual, expected in zip(derived, profile["footprint_xy_m"]):
        assert actual == pytest.approx(expected)
    assert profile["production_default_unchanged"] is True


def test_engineering_planner_requires_polygon_costmap_and_pose_overlap_inputs():
    planner = ObservationPosePlanner(PlannerConstraints(
        arc_samples=3,
        standoff_steps=2,
        minimum_clearance_m=0.01,
        require_polygon_checks=True,
        require_costmap_footprint_cost=True,
        require_pose_dependent_self_overlap=True,
    ))
    camera = VerificationCameraModel(
        640, 480, 1.50098, (0.67, 0.34, 0.48), math.radians(50.0), 0.0, 0.0,
        mount_rpy_rad=(0.0, math.radians(50.0), 0.0), fx_px=344.0, fy_px=344.0, cx_px=320.0, cy_px=240.0,
    )
    common = dict(
        region=CandidateRegion("candidate", (2.0, 0.0), 0.12, "metal_can"),
        covariance_trace=0.0,
        camera=camera,
        cleanable_polygon=((-2.0, -3.0), (5.0, -3.0), (5.0, 3.0), (-2.0, 3.0)),
        keepout_polygons=(),
        current_pose=Pose2D(0.0, 0.0, 0.0),
        compute_path=lambda goal: (Pose2D(0.0, 0.0, 0.0), goal),
    )
    assert planner.plan(**common) is None
    result = planner.plan(
        **common,
        candidate_footprint=((0.74, 0.44), (0.74, -0.39), (-0.43, -0.39), (-0.43, 0.44)),
        footprint_cost=lambda _pose, polygon: 0.0 if len(polygon) == 4 else None,
        self_overlap_estimator=lambda _pose, roi: (0.0, 0.0 if roi[0] >= 0 else 1.0),
    )
    assert result is not None
    assert result.costmap_footprint_cost == 0.0
    assert len(result.footprint_polygon_xy_m) == 4


def test_engineering_planner_rejects_polygon_keepout_intersection_even_when_center_is_clear():
    planner = ObservationPosePlanner(PlannerConstraints(
        arc_samples=1, standoff_steps=1, standoff_min_m=1.0, standoff_max_m=1.0,
        minimum_clearance_m=0.0, require_polygon_checks=True,
    ))
    camera = VerificationCameraModel(640, 480, 1.50098, (0.0, 0.0, 0.4), 0.0, 0.0, 0.0)
    result = planner.plan(
        region=CandidateRegion("candidate", (2.0, 0.0), 0.2, "metal_can"),
        covariance_trace=0.0,
        camera=camera,
        cleanable_polygon=((-2.0, -3.0), (5.0, -3.0), (5.0, 3.0), (-2.0, 3.0)),
        keepout_polygons=(((0.90, -0.10), (1.10, -0.10), (1.10, 0.10), (0.90, 0.10)),),
        current_pose=Pose2D(0.0, 0.0, 0.0),
        compute_path=lambda goal: (Pose2D(0.0, 0.0, 0.0), goal),
        candidate_footprint=((0.2, 0.2), (0.2, -0.2), (-0.2, -0.2), (-0.2, 0.2)),
    )
    assert result is None
