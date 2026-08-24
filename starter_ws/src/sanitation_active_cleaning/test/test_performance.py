import math
import time

from sanitation_active_cleaning.environment import ActiveCleaningEnv
from sanitation_active_cleaning.models import TaskConfig
from sanitation_active_cleaning.policies import SensingGreedyPolicy


def test_200_by_100_field_reset_plan_and_step_watchdog():
    task = TaskConfig.from_mapping(
        {
            "geofence": [[0, 0], [200, 0], [200, 100], [0, 100]],
            "start": {"x": 2.0, "y": 2.0, "yaw": 0.0},
            "grid_resolution": 0.5,
            "sensing_radius": 3.0,
            "sensing_fov_rad": 2.0 * math.pi,
            "cleaning_width": 0.8,
            "vehicle_radius": 0.2,
            "grasp_radius": 0.5,
            "min_turn_radius": 0.3,
            "path_sample_spacing": 0.1,
            "ground_dirt_count": 2,
            "ground_dirt_radius_range": [0.3, 0.5],
            "discrete_target_count": 2,
            "pedestrian_count": 1,
            "max_steps": 10,
        }
    )
    started = time.perf_counter()
    env = ActiveCleaningEnv(task)
    observation = env.reset(seed=20260824)
    action = SensingGreedyPolicy(task).act(observation)
    result = env.step(action)
    elapsed = time.perf_counter() - started

    assert result.info["accepted"] is True
    assert elapsed < 5.0
