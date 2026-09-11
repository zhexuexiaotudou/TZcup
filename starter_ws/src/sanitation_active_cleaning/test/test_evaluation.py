import math
from pathlib import Path
import time

import pytest

from sanitation_active_cleaning.evaluation import _summary, evaluate_paired, run_episode
from sanitation_active_cleaning.models import TaskConfig
from sanitation_active_cleaning.policies import FullCoveragePolicy, TrajectoryPolicy


def trivial_config():
    return TaskConfig.from_mapping(
        {
            "geofence": [[0, 0], [4, 0], [4, 3], [0, 3]],
            "start": {"x": 1.0, "y": 1.0, "yaw": 0.0},
            "grid_resolution": 0.5,
            "sensing_radius": 20.0,
            "sensing_fov_rad": 2.0 * math.pi,
            "cleaning_width": 0.8,
            "vehicle_radius": 0.2,
            "grasp_radius": 0.5,
            "min_turn_radius": 0.3,
            "path_sample_spacing": 0.1,
            "ground_dirt_count": 0,
            "discrete_target_count": 0,
            "pedestrian_count": 0,
            "max_steps": 10,
        }
    )


def test_paired_report_uses_same_seeds_and_excludes_return_time_and_energy():
    report = evaluate_paired(trivial_config(), seeds=[11, 12, 13])
    assert report["paired_seeds"] == [11, 12, 13]
    assert report["truth_boundary"] == "evaluation_token_only"
    assert report["time_energy_ignored"] is True
    assert set(report["summaries"]) == {"full_coverage", "sensing_greedy", "oracle"}
    assert len(report["episodes"]) == 9
    assert all(row["return_distance_included"] is False for row in report["episodes"])
    assert all(set(row["role_seeds"]) == {"layout", "dynamics", "grasp", "policy"} for row in report["episodes"])
    assert all(row["distance_gate"] for row in report["episodes"])
    assert all(row["success"] for row in report["episodes"])
    for summary in report["summaries"].values():
        assert {"mean", "ci95_low", "ci95_high", "p10", "worst"} <= set(summary["task_distance"])
        assert summary["task_distance"]["sample_count"] == 3


def test_demo_seed_is_bounded_and_finishes_quickly():
    config_path = Path(__file__).parents[1] / "config" / "demo_task.json"
    task = TaskConfig.from_json(config_path)
    started = time.perf_counter()
    report = evaluate_paired(task, seeds=[101])
    elapsed = time.perf_counter() - started

    assert elapsed < 10.0
    assert all(row["steps"] <= task.max_steps for row in report["episodes"])
    assert all(row["success"] for row in report["episodes"])


@pytest.mark.parametrize("seeds", [[11, 11], [True], [11.5], ["11"]])
def test_paired_report_rejects_ambiguous_or_repeated_seeds(seeds):
    with pytest.raises(ValueError, match="seeds"):
        evaluate_paired(trivial_config(), seeds=seeds)


def test_paired_report_rejects_duplicate_names_and_explicit_empty_policies():
    with pytest.raises(ValueError, match="unique policy names"):
        evaluate_paired(
            trivial_config(), seeds=[11],
            policy_factories=[FullCoveragePolicy, FullCoveragePolicy],
        )
    with pytest.raises(ValueError, match="full_coverage baseline"):
        evaluate_paired(trivial_config(), seeds=[11], policy_factories=[])


class BrokenPolicy(TrajectoryPolicy):
    name = "broken"

    def __init__(self, config):
        pass

    def reset(self, *, episode_seed=None):
        raise RuntimeError("sensitive error payload must not enter the report")


def test_policy_failure_is_retained_and_all_paired_seeds_are_evaluated():
    report = evaluate_paired(
        trivial_config(), seeds=[11, 12],
        policy_factories=[FullCoveragePolicy, BrokenPolicy],
    )
    rows = [row for row in report["episodes"] if row["policy"] == "broken"]
    assert [row["seed"] for row in rows] == [11, 12]
    assert all(row["success"] is False for row in rows)
    assert all(row["failure_reason"] == "policy_reset_error:RuntimeError" for row in rows)
    assert all(row["steps"] == 0 and row["task_distance"] == 0 for row in rows)
    assert report["summaries"]["broken"]["success_rate"] == 0
    assert report["summaries"]["broken"]["failed_episodes"] == 2
    assert report["summaries"]["broken"]["policy_error_episodes"] == 2
    assert "sensitive error payload" not in str(report)


def test_policy_action_failure_preserves_the_executed_partial_episode():
    class BreakAfterFirstAction(FullCoveragePolicy):
        def act(self, observation):
            if getattr(self, "already_acted", False):
                raise ValueError("broken policy")
            self.already_acted = True
            return super().act(observation)

    task = TaskConfig.from_json(Path(__file__).parents[1] / "config" / "demo_task.json")
    row = run_episode(
        task, seed=101, policy=BreakAfterFirstAction(task), baseline_distance=None
    )
    assert row["failure_reason"] == "policy_action_error:ValueError"
    assert row["success"] is False
    assert row["steps"] == 1
    assert row["task_distance"] > 0


def test_distance_worst_is_the_largest_but_coverage_worst_is_smallest():
    assert _summary([1.0, 2.0, 3.0], higher_is_better=False)["worst"] == 3.0
    assert _summary([0.7, 0.9, 1.0])["worst"] == 0.7


def test_summary_reports_sample_standard_deviation_and_interval_method():
    summary = _summary([1.0, 2.0, 3.0])
    assert summary["sample_count"] == 3
    assert summary["stddev"] == 1.0
    assert summary["stddev_ddof"] == 1
    assert summary["ci95_estimable"] is True
    assert summary["ci95_method"] == "normal_approximation_1.96_standard_error"
    assert summary["ci95_low"] == pytest.approx(2.0 - 1.96 / math.sqrt(3))
    assert summary["ci95_high"] == pytest.approx(2.0 + 1.96 / math.sqrt(3))


@pytest.mark.parametrize("values", [[], [4.0]])
def test_insufficient_samples_do_not_claim_zero_statistical_uncertainty(values):
    summary = _summary(values)
    assert summary["sample_count"] == len(values)
    assert summary["stddev"] is None
    assert summary["ci95_estimable"] is False
