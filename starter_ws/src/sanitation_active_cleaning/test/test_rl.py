import math

import pytest

from sanitation_active_cleaning.environment import ActiveCleaningEnv
from sanitation_active_cleaning.geometry import validate_ackermann_path
from sanitation_active_cleaning.models import TaskConfig
from sanitation_active_cleaning.rl import (
    ACTION_FRONTIER,
    ACTION_WAIT,
    CoverageBackstoppedQLearningPolicy,
    QLearningPolicy,
    belief_state,
    train_q_policy,
)


def training_config():
    return TaskConfig.from_mapping(
        {
            "geofence": [[0, 0], [4, 0], [4, 3], [0, 3]],
            "start": {"x": 1.0, "y": 1.0, "yaw": 0.0},
            "grid_resolution": 0.5,
            "sensing_radius": 0.9,
            "sensing_fov_rad": 2.0 * math.pi,
            "cleaning_width": 0.8,
            "vehicle_radius": 0.15,
            "grasp_radius": 0.55,
            "min_turn_radius": 0.25,
            "path_sample_spacing": 0.1,
            "ground_dirt_count": 1,
            "ground_dirt_radius_range": [0.25, 0.25],
            "discrete_target_count": 1,
            "pedestrian_count": 0,
            "max_steps": 24,
        }
    )


def test_q_learning_is_reproducible_and_checkpoint_is_truth_free(tmp_path):
    kwargs = {
        "train_seeds": [1, 2, 3],
        "validation_seeds": [11, 12],
        "test_seeds": [21, 22],
        "policy_seed": 7,
    }
    policy_a, report_a = train_q_policy(training_config(), **kwargs)
    policy_b, report_b = train_q_policy(training_config(), **kwargs)

    assert policy_a.checkpoint() == policy_b.checkpoint()
    assert report_a == report_b
    assert report_a["q_state_count"] > 0
    assert report_a["truth_access_used"] is False

    checkpoint = tmp_path / "q_policy.json"
    policy_a.save(checkpoint)
    restored = QLearningPolicy.load(training_config(), checkpoint)
    assert restored.checkpoint() == policy_a.checkpoint()


def test_q_policy_outputs_ackermann_reference_trajectory():
    task = training_config()
    observation = ActiveCleaningEnv(task).reset(seed=4)
    action = QLearningPolicy(task, seed=7).act(observation)
    valid, reason = validate_ackermann_path(
        action.points, max_curvature=1.0 / task.min_turn_radius
    )
    assert valid, reason


def test_training_splits_must_be_disjoint():
    with pytest.raises(ValueError, match="disjoint"):
        train_q_policy(
            training_config(),
            train_seeds=[1, 2],
            validation_seeds=[2, 3],
            test_seeds=[4],
            policy_seed=7,
        )


def test_episode_exploration_rng_is_independent_of_rollout_order():
    task = training_config()
    observation = ActiveCleaningEnv(task).reset(seed=44)
    direct = QLearningPolicy(task, epsilon=1.0, seed=7)
    direct.reset(episode_seed=1234)
    direct_labels = [direct.choose_action(observation, explore=True) for _ in range(8)]

    reordered = QLearningPolicy(task, epsilon=1.0, seed=7)
    reordered.reset(episode_seed=9999)
    _ = [reordered.choose_action(observation, explore=True) for _ in range(8)]
    reordered.reset(episode_seed=1234)
    reordered_labels = [
        reordered.choose_action(observation, explore=True) for _ in range(8)
    ]
    assert reordered_labels == direct_labels


def test_wait_is_masked_while_a_productive_belief_action_exists():
    task = training_config()
    observation = ActiveCleaningEnv(task).reset(seed=44)
    policy = QLearningPolicy(task, epsilon=0.0, seed=7)
    row = policy._row(belief_state(observation))
    row[ACTION_WAIT] = 1_000.0
    row[ACTION_FRONTIER] = -1_000.0

    assert ACTION_FRONTIER in policy.available_actions(observation)
    assert ACTION_WAIT not in policy.available_actions(observation)
    state, label, trajectory = policy.act_with_label(observation, explore=False)
    assert state == belief_state(observation)
    assert label != ACTION_WAIT
    assert trajectory.points[-1] != observation.pose


def test_dual_mode_activates_systematic_coverage_after_public_belief_stalls():
    task = training_config()
    observation = ActiveCleaningEnv(task).reset(seed=44)
    policy = CoverageBackstoppedQLearningPolicy(task, q_table={}, seed=7)
    policy.reset(episode_seed=123)
    policy._last_observed_ratio = observation.observed_ratio
    policy._stagnant_steps = 6

    action = policy.act(observation)

    assert policy._coverage_activated is True
    assert policy.last_mode == "systematic_coverage_backstop"
    valid, reason = validate_ackermann_path(
        action.points,
        max_curvature=1.0 / task.min_turn_radius,
    )
    assert valid, reason
