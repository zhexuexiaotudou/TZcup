"""Paired-seed evaluation with evaluation-only truth access."""

from __future__ import annotations

import math
import statistics
from typing import Any, Callable, Sequence

from .environment import ActiveCleaningEnv, GraspVerifier, create_evaluation_token
from .models import EvaluationSnapshot, RoleSeeds, TaskConfig, TaskLayout
from .policies import FullCoveragePolicy, OraclePolicy, SensingGreedyPolicy, TrajectoryPolicy


PolicyFactory = Callable[[TaskConfig], TrajectoryPolicy]


def _ratio(numerator: int, denominator: int) -> float:
    return 1.0 if denominator == 0 else numerator / denominator


def _episode_metrics(
    env: ActiveCleaningEnv,
    truth: EvaluationSnapshot,
    *,
    baseline_distance: float | None,
) -> dict[str, Any]:
    free_observed = sum(
        observed and free for observed, free in zip(truth.observed, env.grid.traversable)
    )
    observed_ratio = free_observed / sum(env.grid.traversable)

    observed_ground = {
        index for index in truth.initial_ground_dirt_cells if truth.observed[index]
    }
    cleared_ground = observed_ground.difference(truth.remaining_ground_dirt_cells)
    ground_clear_ratio = _ratio(len(cleared_ground), len(observed_ground))

    observed_target_ids = {
        target_id
        for target_id, x, y in truth.initial_targets
        if truth.observed[env.grid.nearest_index((x, y))]
    }
    cleared_observed_targets = observed_target_ids.intersection(truth.cleared_target_ids)
    discrete_clear_ratio = _ratio(
        len(cleared_observed_targets), len(observed_target_ids)
    )
    distance_gate = (
        baseline_distance is None
        or truth.task_distance <= baseline_distance + 1.0e-6
    )
    safety_gate = (
        truth.collisions == 0
        and truth.boundary_violations == 0
        and truth.invalid_actions == 0
    )
    success = (
        truth.terminated
        and not truth.truncated
        and observed_ratio >= env.config.observation_threshold
        and ground_clear_ratio >= env.config.ground_clear_threshold
        and discrete_clear_ratio >= env.config.discrete_clear_threshold
        and safety_gate
        and distance_gate
    )
    return {
        "seed": truth.seed,
        "role_seeds": dict(truth.role_seeds.as_mapping()),
        "steps": truth.step_index,
        "success": success,
        "terminated": truth.terminated,
        "truncated": truth.truncated,
        "observed_ratio": observed_ratio,
        "observed_ground_dirt_cells": len(observed_ground),
        "cleared_observed_ground_dirt_cells": len(cleared_ground),
        "ground_clear_ratio": ground_clear_ratio,
        "observed_discrete_targets": len(observed_target_ids),
        "cleared_observed_discrete_targets": len(cleared_observed_targets),
        "discrete_clear_ratio": discrete_clear_ratio,
        "task_distance": truth.task_distance,
        "baseline_distance": baseline_distance,
        "distance_gate": distance_gate,
        "collisions": truth.collisions,
        "boundary_violations": truth.boundary_violations,
        "invalid_actions": truth.invalid_actions,
        "grasp_verification_mode": truth.grasp_verification_mode,
        "return_distance_included": False,
        "time_or_energy_scored": False,
    }


def run_episode(
    config: TaskConfig,
    *,
    seed: int,
    policy: TrajectoryPolicy,
    baseline_distance: float | None,
    task_layout: TaskLayout | None = None,
    grasp_verifier: GraspVerifier | None = None,
) -> dict[str, Any]:
    token = create_evaluation_token()
    env = ActiveCleaningEnv(
        config,
        evaluation_token=token,
        max_task_distance=baseline_distance,
        task_layout=task_layout,
        grasp_verifier=grasp_verifier,
    )
    observation = env.reset(seed=seed)
    policy.reset(episode_seed=RoleSeeds.from_master(seed).policy)
    while True:
        truth = env.evaluation_snapshot(token)
        if truth.terminated or truth.truncated:
            break
        if isinstance(policy, OraclePolicy):
            action = policy.act_with_truth(observation, truth)
        else:
            action = policy.act(observation)
        result = env.step(action)
        observation = result.observation
    truth = env.evaluation_snapshot(token)
    metrics = _episode_metrics(
        env,
        truth,
        baseline_distance=baseline_distance,
    )
    metrics["policy"] = policy.name
    metrics["evaluation_only_policy"] = policy.evaluation_only
    return metrics


def _summary(values: Sequence[float]) -> dict[str, float]:
    if not values:
        return {"mean": 0.0, "ci95_low": 0.0, "ci95_high": 0.0, "p10": 0.0, "worst": 0.0}
    mean = statistics.fmean(values)
    if len(values) > 1:
        half_width = 1.96 * statistics.stdev(values) / math.sqrt(len(values))
    else:
        half_width = 0.0
    ordered = sorted(values)
    p10_index = max(0, math.ceil(0.10 * len(ordered)) - 1)
    return {
        "mean": mean,
        "ci95_low": mean - half_width,
        "ci95_high": mean + half_width,
        "p10": ordered[p10_index],
        "worst": ordered[0],
    }


def evaluate_paired(
    config: TaskConfig,
    *,
    seeds: Sequence[int],
    policy_factories: Sequence[PolicyFactory] | None = None,
) -> dict[str, Any]:
    """Evaluate every strategy on identical task seeds.

    The full-coverage run is always executed first per seed. Its actual task
    distance becomes the fail-closed upper bound for all active strategies.
    """
    if not seeds:
        raise ValueError("at least one evaluation seed is required")
    factories = policy_factories or (
        FullCoveragePolicy,
        SensingGreedyPolicy,
        OraclePolicy,
    )
    names = [factory(config).name for factory in factories]
    if "full_coverage" not in names:
        raise ValueError("paired evaluation requires the full_coverage baseline")
    episodes: list[dict[str, Any]] = []
    for seed in seeds:
        baseline = run_episode(
            config,
            seed=int(seed),
            policy=FullCoveragePolicy(config),
            baseline_distance=None,
        )
        episodes.append(baseline)
        baseline_valid = bool(baseline["success"])
        distance_limit = float(baseline["task_distance"]) if baseline_valid else 0.0
        for factory in factories:
            policy = factory(config)
            if policy.name == "full_coverage":
                continue
            result = run_episode(
                config,
                seed=int(seed),
                policy=policy,
                baseline_distance=distance_limit,
            )
            result["baseline_valid"] = baseline_valid
            if not baseline_valid:
                result["success"] = False
            episodes.append(result)

    summaries: dict[str, Any] = {}
    for name in names:
        rows = [row for row in episodes if row["policy"] == name]
        summaries[name] = {
            "episodes": len(rows),
            "success_rate": _ratio(sum(bool(row["success"]) for row in rows), len(rows)),
            "task_distance": _summary([float(row["task_distance"]) for row in rows]),
            "observed_ratio": _summary([float(row["observed_ratio"]) for row in rows]),
            "ground_clear_ratio": _summary([float(row["ground_clear_ratio"]) for row in rows]),
            "discrete_clear_ratio": _summary([float(row["discrete_clear_ratio"]) for row in rows]),
        }
    return {
        "schema_version": 1,
        "paired_seeds": [int(seed) for seed in seeds],
        "truth_boundary": "evaluation_token_only",
        "distance_metric": "executed_chassis_task_trajectory_excluding_return",
        "time_energy_ignored": True,
        "episodes": episodes,
        "summaries": summaries,
    }
