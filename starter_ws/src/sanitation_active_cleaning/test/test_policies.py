import math

import pytest

from sanitation_active_cleaning.environment import ActiveCleaningEnv
from sanitation_active_cleaning.geometry import validate_ackermann_path
from sanitation_active_cleaning.models import Pose2D, TaskConfig
from sanitation_active_cleaning.policies import SensingGreedyPolicy


def config():
    return TaskConfig.from_mapping(
        {
            "geofence": [[0, 0], [8, 0], [8, 4], [0, 4]],
            "start": {"x": 1.0, "y": 1.0, "yaw": 0.0},
            "grid_resolution": 0.5,
            "sensing_radius": 1.0,
            "sensing_fov_rad": 2.0 * math.pi,
            "cleaning_width": 0.8,
            "vehicle_radius": 0.2,
            "grasp_radius": 0.5,
            "min_turn_radius": 0.3,
            "path_sample_spacing": 0.08,
            "ground_dirt_count": 0,
            "discrete_target_count": 0,
            "pedestrian_count": 0,
            "max_steps": 50,
        }
    )


def test_greedy_outputs_reference_trajectory_with_ackermann_constraints():
    task = config()
    env = ActiveCleaningEnv(task)
    observation = env.reset(seed=2)
    action = SensingGreedyPolicy(task).act(observation)
    valid, reason = validate_ackermann_path(
        action.points,
        max_curvature=1.0 / task.min_turn_radius,
    )
    assert valid, reason
    assert len(action.points) > 1


def test_ackermann_contract_accepts_reverse_but_still_rejects_lateral_motion():
    reverse = (Pose2D(1.0, 1.0, 0.0), Pose2D(0.5, 1.0, 0.0))
    valid, reason = validate_ackermann_path(reverse, max_curvature=2.0)
    assert valid, reason

    lateral = (Pose2D(1.0, 1.0, 0.0), Pose2D(1.0, 1.5, 0.0))
    valid, reason = validate_ackermann_path(lateral, max_curvature=2.0)
    assert valid is False
    assert reason == "body_lateral_motion_forbidden"


def test_hybrid_fallback_routes_front_steered_path_around_building():
    task = TaskConfig.from_mapping(
        {
            "geofence": [[0, 0], [10, 0], [10, 7], [0, 7]],
            "static_obstacles": [[[4, 0], [6, 0], [6, 4], [4, 4]]],
            "start": {"x": 2.0, "y": 2.0, "yaw": 0.0},
            "grid_resolution": 0.5,
            "sensing_radius": 1.0,
            "sensing_fov_rad": 2.0 * math.pi,
            "cleaning_width": 0.8,
            "vehicle_radius": 0.2,
            "grasp_radius": 0.5,
            "min_turn_radius": 0.3,
            "path_sample_spacing": 0.08,
            "ground_dirt_count": 0,
            "discrete_target_count": 0,
            "pedestrian_count": 0,
            "max_steps": 50,
        }
    )
    env = ActiveCleaningEnv(task)
    observation = env.reset(seed=2)
    policy = SensingGreedyPolicy(task)
    action = policy._trajectory_to(observation, (8.0, 2.0), clean=False)

    assert action is not None
    valid, reason = validate_ackermann_path(
        action.points,
        max_curvature=1.0 / task.min_turn_radius,
    )
    assert valid, reason
    result = env.step(action)
    assert result.info["accepted"] is True
    assert result.observation.pose.x == pytest.approx(8.0, abs=0.1)


def test_boundary_recovery_uses_valid_reverse_ackermann_trajectory():
    task = TaskConfig.from_mapping(
        {
            "geofence": [[0, 0], [12, 0], [12, 8], [0, 8]],
            "start": {"x": 0.5, "y": 2.5, "yaw": math.pi},
            "grid_resolution": 0.5,
            "sensing_radius": 1.0,
            "sensing_fov_rad": 2.0 * math.pi,
            "cleaning_width": 0.8,
            "vehicle_radius": 0.2,
            "grasp_radius": 0.5,
            "min_turn_radius": 0.7,
            "path_sample_spacing": 0.1,
            "ground_dirt_count": 0,
            "discrete_target_count": 0,
            "pedestrian_count": 0,
            "max_steps": 10,
        }
    )
    env = ActiveCleaningEnv(task)
    observation = env.reset(seed=3)
    action = SensingGreedyPolicy(task)._trajectory_to(
        observation,
        (4.5, 2.5),
        clean=False,
        allow_hybrid=False,
    )

    assert action is not None
    valid, reason = validate_ackermann_path(
        action.points,
        max_curvature=1.0 / task.min_turn_radius,
    )
    assert valid, reason
    first_motion = next(
        pose
        for pose in action.points[1:]
        if math.hypot(
            pose.x - action.points[0].x,
            pose.y - action.points[0].y,
        )
        > 1.0e-9
    )
    assert first_motion.x > action.points[0].x
    assert env.step(action).info["accepted"] is True
