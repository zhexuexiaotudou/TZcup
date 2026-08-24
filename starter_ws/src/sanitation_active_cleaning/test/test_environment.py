from dataclasses import asdict
import math

import pytest

from sanitation_active_cleaning.environment import (
    ActiveCleaningEnv,
    EvaluationToken,
    GraspVerificationResult,
    TrajectoryAction,
    create_evaluation_token,
)
from sanitation_active_cleaning.models import Pose2D, TaskConfig, TaskLayout


def make_config(**overrides):
    data = {
        "geofence": [[0.0, 0.0], [4.0, 0.0], [4.0, 1.0], [0.0, 1.0]],
        "static_obstacles": [],
        "start": {"x": 0.5, "y": 0.5, "yaw": 0.0},
        "grid_resolution": 0.25,
        "sensing_radius": 20.0,
        "sensing_fov_rad": 2.0 * math.pi,
        "cleaning_width": 1.0,
        "vehicle_radius": 0.1,
        "grasp_radius": 10.0,
        "min_turn_radius": 0.2,
        "path_sample_spacing": 0.1,
        "ground_dirt_count": 1,
        "ground_dirt_radius_range": [0.2, 0.2],
        "discrete_target_count": 1,
        "pedestrian_count": 0,
        "max_steps": 20,
    }
    data.update(overrides)
    return TaskConfig.from_mapping(data)


def straight_path(start=0.5, end=3.5, count=31):
    return tuple(
        Pose2D(start + (end - start) * index / (count - 1), 0.5, 0.0)
        for index in range(count)
    )


def test_reset_is_seed_reproducible_and_truth_is_fail_closed():
    token_a = create_evaluation_token()
    token_b = create_evaluation_token()
    env_a = ActiveCleaningEnv(make_config(), evaluation_token=token_a)
    env_b = ActiveCleaningEnv(make_config(), evaluation_token=token_b)
    observation_a = env_a.reset(seed=77)
    observation_b = env_b.reset(seed=77)

    assert observation_a == observation_b
    assert env_a.evaluation_snapshot(token_a).initial_targets == env_b.evaluation_snapshot(token_b).initial_targets
    with pytest.raises(PermissionError):
        env_a.evaluation_snapshot(token_b)
    with pytest.raises(PermissionError):
        env_a.evaluation_snapshot(EvaluationToken(object()))
    assert "initial_ground_dirt_cells" not in asdict(observation_a)
    assert not hasattr(observation_a, "ground_truth")

    tokenless_env = ActiveCleaningEnv(make_config())
    tokenless_env.reset(seed=77)
    with pytest.raises(PermissionError):
        tokenless_env.evaluation_snapshot(None)  # type: ignore[arg-type]


def test_ground_is_cleared_only_by_swept_area_and_target_only_by_grasp():
    token = create_evaluation_token()
    env = ActiveCleaningEnv(make_config(), evaluation_token=token)
    observation = env.reset(seed=8)
    target_id = observation.belief.known_targets[0].target_id
    initial = env.evaluation_snapshot(token)

    result = env.step(TrajectoryAction(straight_path(), clean_ground=True))
    after_sweep = env.evaluation_snapshot(token)
    assert len(after_sweep.remaining_ground_dirt_cells) < len(initial.initial_ground_dirt_cells)
    assert target_id not in after_sweep.cleared_target_ids

    grasp_result = env.step(
        TrajectoryAction(
            (result.observation.pose,),
            clean_ground=False,
            grasp_target_ids=(target_id,),
        )
    )
    assert target_id in env.evaluation_snapshot(token).cleared_target_ids
    assert grasp_result.info["cleared_target_count"] == 1


def test_virtual_ackermann_rejects_lateral_motion_without_execution():
    token = create_evaluation_token()
    env = ActiveCleaningEnv(
        make_config(
            ground_dirt_count=0,
            discrete_target_count=0,
            sensing_radius=0.20,
        ),
        evaluation_token=token,
    )
    env.reset(seed=1)
    result = env.step(
        TrajectoryAction(
            (Pose2D(0.5, 0.5, 0.0), Pose2D(0.5, 0.8, 0.0)),
            clean_ground=False,
        )
    )
    assert result.info["accepted"] is False
    assert result.info["reason"] == "body_lateral_motion_forbidden"
    assert result.info["executed_distance"] == 0.0
    assert result.observation.task_distance == 0.0


def test_mobile_people_are_exposed_only_as_current_static_circles():
    env = ActiveCleaningEnv(make_config(pedestrian_count=1))
    observation = env.reset(seed=19)
    assert len(observation.current_pedestrians) == 1
    assert len(observation.current_pedestrians[0]) == 3
    assert not hasattr(observation, "pedestrian_velocity")


def test_explicit_harness_layout_is_effective_but_absent_from_policy_observation():
    token = create_evaluation_token()
    layout = TaskLayout(
        ground_dirt_regions=((2.0, 0.5, 0.3),),
        discrete_targets=(("scenario_cube_red", 2.5, 0.5),),
        pedestrians=((3.0, 0.5, math.pi),),
    )
    env = ActiveCleaningEnv(
        make_config(
            ground_dirt_count=99,
            discrete_target_count=99,
            pedestrian_count=99,
        ),
        evaluation_token=token,
        task_layout=layout,
    )
    observation = env.reset(seed=123)
    truth = env.evaluation_snapshot(token)

    assert truth.initial_targets == (("scenario_cube_red", 2.5, 0.5),)
    assert truth.initial_ground_dirt_cells
    assert observation.current_pedestrians == ((3.0, 0.5, env.config.pedestrian.radius),)
    public = asdict(observation)
    assert "task_layout" not in public
    assert "ground_dirt_regions" not in public
    assert "initial_targets" not in public


def test_segment_spacing_and_continuous_obstacle_collision_are_fail_closed():
    obstacle = [[1.945, 0.4], [1.955, 0.4], [1.955, 0.6], [1.945, 0.6]]
    token = create_evaluation_token()
    task = make_config(
        static_obstacles=[obstacle],
        vehicle_radius=0.01,
        ground_dirt_count=0,
        discrete_target_count=0,
        sensing_radius=0.2,
    )
    env = ActiveCleaningEnv(task, evaluation_token=token)
    env.reset(seed=2)
    sparse = env.step(
        TrajectoryAction(
            (Pose2D(0.5, 0.5, 0.0), Pose2D(0.7, 0.5, 0.0)),
            clean_ground=False,
        )
    )
    assert sparse.info["reason"] == "trajectory_segment_spacing_exceeded"

    points = tuple(Pose2D(0.5 + 0.1 * index, 0.5, 0.0) for index in range(16))
    collision = env.step(TrajectoryAction(points, clean_ground=False))
    truth = env.evaluation_snapshot(token)
    assert collision.info["reason"] == "trajectory_intersects_static_obstacle"
    assert truth.collisions == 1
    assert truth.invalid_actions == 2


def test_boundary_violation_metric_changes_on_rejected_path():
    token = create_evaluation_token()
    task = make_config(
        geofence=[[0, 0], [4, 0], [4, 4], [3, 4], [3, 1], [1, 1], [1, 4], [0, 4]],
        start={"x": 0.5, "y": 2.0, "yaw": 0.0},
        vehicle_radius=0.01,
        ground_dirt_count=0,
        discrete_target_count=0,
        sensing_radius=0.2,
    )
    env = ActiveCleaningEnv(task, evaluation_token=token)
    env.reset(seed=3)
    points = tuple(Pose2D(0.5 + 0.1 * index, 2.0, 0.0) for index in range(31))
    result = env.step(TrajectoryAction(points, clean_ground=False))
    assert result.info["reason"] == "trajectory_intersects_geofence_boundary"
    assert env.evaluation_snapshot(token).boundary_violations == 1


def test_current_pedestrian_is_checked_continuously_and_counts_collision():
    token = create_evaluation_token()
    task = make_config(
        vehicle_radius=0.01,
        ground_dirt_count=0,
        discrete_target_count=0,
        pedestrian_count=0,
        sensing_radius=0.2,
        pedestrian={"radius": 0.01, "step_distance": 0.01},
    )
    env = ActiveCleaningEnv(
        task,
        evaluation_token=token,
        task_layout=TaskLayout(pedestrians=((1.95, 0.5, 0.0),)),
    )
    env.reset(seed=7)
    points = tuple(Pose2D(0.5 + 0.1 * index, 0.5, 0.0) for index in range(16))
    result = env.step(TrajectoryAction(points, clean_ground=False))
    assert result.info["reason"] == "trajectory_intersects_current_pedestrian"
    assert env.evaluation_snapshot(token).collisions == 1


def test_grasp_is_one_target_per_action_and_exhaustion_stops_immediately():
    layout = TaskLayout(
        discrete_targets=(("cube_a", 0.6, 0.5), ("cube_b", 0.7, 0.5)),
    )
    env = ActiveCleaningEnv(
        make_config(
            ground_dirt_count=0,
            discrete_target_count=0,
            max_grasp_attempts=1,
            grasp_success_probability=0.0,
        ),
        task_layout=layout,
    )
    observation = env.reset(seed=4)
    rejected = env.step(
        TrajectoryAction(
            (observation.pose,),
            clean_ground=False,
            grasp_target_ids=("cube_a", "cube_b"),
        )
    )
    assert rejected.info["reason"] == "multiple_grasp_targets_forbidden"
    failed = env.step(
        TrajectoryAction(
            (observation.pose,), clean_ground=False, grasp_target_ids=("cube_a",)
        )
    )
    assert failed.truncated is True
    assert failed.observation.step_index == 2


def test_terminal_uses_clear_ratio_over_observed_targets():
    layout = TaskLayout(
        discrete_targets=(("cube_a", 0.6, 0.5), ("cube_b", 0.7, 0.5)),
    )
    env = ActiveCleaningEnv(
        make_config(
            ground_dirt_count=0,
            discrete_target_count=0,
            discrete_clear_threshold=0.5,
        ),
        task_layout=layout,
    )
    observation = env.reset(seed=5)
    result = env.step(
        TrajectoryAction(
            (observation.pose,), clean_ground=False, grasp_target_ids=("cube_a",)
        )
    )
    assert result.terminated is True
    assert result.truncated is False


def test_external_grasp_verifier_failure_does_not_clear_target():
    calls = []

    def verifier(target_id, target_position, observation):
        calls.append((target_id, target_position, observation.pose))
        return GraspVerificationResult(False, source="manipulation_mock")

    token = create_evaluation_token()
    env = ActiveCleaningEnv(
        make_config(
            ground_dirt_count=0,
            discrete_target_count=0,
            max_grasp_attempts=1,
        ),
        evaluation_token=token,
        task_layout=TaskLayout(discrete_targets=(("cube_a", 0.6, 0.5),)),
        grasp_verifier=verifier,
    )
    observation = env.reset(seed=6)
    result = env.step(
        TrajectoryAction(
            (observation.pose,), clean_ground=False, grasp_target_ids=("cube_a",)
        )
    )
    truth = env.evaluation_snapshot(token)
    assert calls and calls[0][0] == "cube_a"
    assert truth.cleared_target_ids == frozenset()
    assert truth.grasp_verification_mode == "external_callback"
    assert result.info["grasp_verified_in_bin"] is False
