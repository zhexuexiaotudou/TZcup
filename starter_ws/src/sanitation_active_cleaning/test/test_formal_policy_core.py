import json
import math

import pytest

from sanitation_active_cleaning.formal_observation_core import PublicPlanningMap
from sanitation_active_cleaning.formal_policy_core import FormalRuntimePolicyCore
from sanitation_active_cleaning.geometry import curvature_limited_reference_path_for_skid_steer
from sanitation_active_cleaning.models import KnownTarget, Pose2D, TaskConfig
from sanitation_active_cleaning.rl import QLearningPolicy


def _config():
    return TaskConfig.from_mapping(
        {
            "geofence": [[0, 0], [10, 0], [10, 6], [0, 6]],
            "start": {"x": 1.0, "y": 1.0, "yaw": 0.0},
            "grid_resolution": 1.0,
            "sensing_radius": 2.0,
            "sensing_fov_rad": math.pi,
            "cleaning_width": 1.0,
            "vehicle_radius": 0.1,
            "grasp_radius": 0.75,
            "min_turn_radius": 0.5,
            "path_sample_spacing": 0.2,
            "ground_dirt_count": 0,
            "discrete_target_count": 0,
            "pedestrian_count": 0,
            "max_steps": 40,
        }
    )


def _core(tmp_path):
    config = _config()
    policy = QLearningPolicy(config, seed=7)
    policy._row((0, 0, 0, 0, 2))
    checkpoint = tmp_path / "q.json"
    checkpoint.write_text(json.dumps(policy.checkpoint()), encoding="utf-8")
    public = PublicPlanningMap(
        frame_id="map",
        width=20,
        height=12,
        resolution=0.5,
        origin_x=0.0,
        origin_y=0.0,
        traversable=(True,) * 240,
        outer_polygon=((0, 0), (10, 0), (10, 6), (0, 6)),
        keepout_polygons=(),
    )
    return FormalRuntimePolicyCore(
        public, config, checkpoint, maximum_task_distance_m=100.0
    )


def test_product_belief_is_downsampled_without_changing_exact_observed_ratio(tmp_path):
    core = _core(tmp_path)
    values = [-1] * 240
    for index in range(120):
        values[index] = 0
    observation = core.observation(
        belief_values=values,
        pose=Pose2D(1.0, 1.0, 0.0),
        targets=(),
        step_index=3,
        task_distance=4.5,
    )
    assert observation.observed_ratio == 0.5
    assert 0 < sum(observation.belief.observed) < len(observation.belief.observed)
    assert observation.task_distance == 4.5


def test_runtime_policy_emits_grasp_request_and_requires_verified_result(tmp_path):
    core = _core(tmp_path)
    core.reset(episode_seed=11)
    target = KnownTarget("cube-1", 1.3, 0.05, False, 0)
    observation = core.observation(
        belief_values=[0] * 240,
        pose=Pose2D(1.0, 1.0, 0.0),
        targets=(target,),
        step_index=0,
        task_distance=0.0,
    )
    decision = core.decide(observation)
    assert decision.kind == "grasp"
    assert decision.grasp_target_id == "cube-1"

    core.mark_grasp_result("cube-1", verified_in_bin=True)
    cleared_observation = core.observation(
        belief_values=[0] * 240,
        pose=Pose2D(1.0, 1.0, 0.0),
        targets=(target,),
        step_index=1,
        task_distance=0.0,
    )
    assert cleared_observation.belief.known_targets[0].cleared is True


def test_runtime_policy_parks_target_in_physical_side_grasp_window(tmp_path):
    core = _core(tmp_path)
    core.reset(episode_seed=11)
    target = KnownTarget("cube-far", 8.0, 4.5, False, 0)
    observation = core.observation(
        belief_values=[0] * 240,
        pose=Pose2D(1.0, 1.0, 0.0),
        targets=(target,),
        step_index=0,
        task_distance=0.0,
    )
    decision = core.decide(observation)
    assert decision.kind == "trajectory"
    assert decision.reason == "navigate_to_physical_grasp_window"
    endpoint = decision.trajectory[-1]
    delta_x = target.x - endpoint.x
    delta_y = target.y - endpoint.y
    body_x = math.cos(endpoint.yaw) * delta_x + math.sin(endpoint.yaw) * delta_y
    body_y = -math.sin(endpoint.yaw) * delta_x + math.cos(endpoint.yaw) * delta_y
    assert body_x == pytest.approx(0.300, abs=0.10)
    assert body_y == pytest.approx(-0.950, abs=0.10)


def test_target_near_vehicle_but_outside_arm_window_never_grasps(tmp_path):
    core = _core(tmp_path)
    core.reset(episode_seed=11)
    target = KnownTarget("cube-wrong-side", 1.2, 1.0, False, 0)
    observation = core.observation(
        belief_values=[0] * 240,
        pose=Pose2D(1.0, 1.0, 0.0),
        targets=(target,),
        step_index=0,
        task_distance=0.0,
    )
    decision = core.decide(observation)
    assert decision.kind != "grasp"


def test_return_home_is_separate_from_cleaning_and_requires_fixed_start_pose(tmp_path):
    core = _core(tmp_path)
    core.reset(episode_seed=11)
    far = core.observation(
        belief_values=[0] * 240,
        pose=Pose2D(8.0, 4.0, math.pi),
        targets=(),
        step_index=20,
        task_distance=30.0,
    )
    decision = core.return_home(far)
    assert decision.kind == "trajectory"
    assert decision.clean_ground is False
    assert math.dist(
        (decision.trajectory[-1].x, decision.trajectory[-1].y),
        (core.config.start.x, core.config.start.y),
    ) < 0.01

    at_home = core.observation(
        belief_values=[0] * 240,
        pose=core.config.start,
        targets=(),
        step_index=21,
        task_distance=31.0,
    )
    assert core.return_home(at_home).kind == "home_reached"


def test_product_trajectory_never_exceeds_full_coverage_distance_budget(tmp_path):
    core = _core(tmp_path)
    core.maximum_task_distance_m = 1.1
    core.reset(episode_seed=11)
    target = KnownTarget("cube-far", 8.0, 4.5, False, 0)
    observation = core.observation(
        belief_values=[0] * 240,
        pose=Pose2D(1.0, 1.0, 0.0),
        targets=(target,),
        step_index=0,
        task_distance=1.0,
    )
    decision = core.decide(observation)
    assert decision.kind == "wait"
    assert decision.reason == "full_coverage_distance_budget_exceeded"


def test_canonical_skid_steer_reference_wrapper_retains_path_only_contract():
    path = curvature_limited_reference_path_for_skid_steer(
        Pose2D(0.0, 0.0, 0.0),
        (2.0, 1.0),
        min_turn_radius=0.70,
        spacing=0.20,
    )
    assert len(path) >= 2
    assert math.dist((path[0].x, path[0].y), (0.0, 0.0)) < 1.0e-9
    assert path[0].yaw == pytest.approx(0.0)
    assert math.dist((path[-1].x, path[-1].y), (2.0, 1.0)) < 1.0e-9
