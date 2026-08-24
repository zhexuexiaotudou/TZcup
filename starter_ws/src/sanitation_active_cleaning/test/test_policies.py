import math

from sanitation_active_cleaning.environment import ActiveCleaningEnv
from sanitation_active_cleaning.geometry import validate_ackermann_path
from sanitation_active_cleaning.models import TaskConfig
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
