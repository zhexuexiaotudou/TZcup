"""Small reproducible Q-learning planner over belief-only high-level actions."""

from __future__ import annotations

from dataclasses import dataclass
import json
import random
from pathlib import Path
from typing import Any, Sequence

from .environment import ActiveCleaningEnv, TrajectoryAction
from .models import AgentObservation, RoleSeeds, TaskConfig, derive_role_seed
from .policies import _SafeTrajectoryMixin, cell_center


ACTION_TARGET = "target"
ACTION_DIRT = "dirt"
ACTION_FRONTIER = "frontier"
ACTION_WAIT = "wait"
ALL_ACTIONS = (ACTION_TARGET, ACTION_DIRT, ACTION_FRONTIER, ACTION_WAIT)


def _count_bin(count: int) -> int:
    return 0 if count == 0 else 1 if count == 1 else 2


def belief_state(observation: AgentObservation) -> tuple[int, int, int, int, int]:
    """Compress only public observation fields into a stable discrete state."""
    dirt_count = sum(amount > 0.0 for amount in observation.belief.known_ground_dirt)
    target_count = sum(not target.cleared for target in observation.belief.known_targets)
    observed_bin = min(9, int(observation.observed_ratio * 10.0))
    pedestrian_bin = min(2, len(observation.current_pedestrians))
    if observation.remaining_distance_budget is None:
        budget_bin = 2
    elif observation.remaining_distance_budget <= 0.0:
        budget_bin = 0
    elif observation.remaining_distance_budget < 2.0:
        budget_bin = 1
    else:
        budget_bin = 2
    return (
        observed_bin,
        _count_bin(dirt_count),
        _count_bin(target_count),
        pedestrian_bin,
        budget_bin,
    )


def _state_key(state: tuple[int, ...]) -> str:
    return ",".join(str(item) for item in state)


class QLearningPolicy(_SafeTrajectoryMixin):
    """Tabular planning policy whose actions remain global trajectories."""

    name = "q_learning"
    evaluation_only = False

    def __init__(
        self,
        config: TaskConfig,
        *,
        learning_rate: float = 0.25,
        discount: float = 0.95,
        epsilon: float = 0.10,
        seed: int = 0,
    ):
        super().__init__(config)
        self.learning_rate = float(learning_rate)
        self.discount = float(discount)
        self.epsilon = float(epsilon)
        self._policy_seed = int(seed)
        self._rng = random.Random(seed)
        self.q_table: dict[str, dict[str, float]] = {}

    def reset(self, *, episode_seed: int | None = None) -> None:
        independent_episode_seed = self._policy_seed if episode_seed is None else (
            self._policy_seed ^ int(episode_seed)
        )
        self._rng.seed(derive_role_seed(independent_episode_seed, "policy_rng"))

    def _row(self, state: tuple[int, ...]) -> dict[str, float]:
        return self.q_table.setdefault(
            _state_key(state), {action: 0.0 for action in ALL_ACTIONS}
        )

    def available_actions(self, observation: AgentObservation) -> tuple[str, ...]:
        actions = []
        if any(
            not target.cleared
            and target.attempts < self.config.max_grasp_attempts
            for target in observation.belief.known_targets
        ):
            actions.append(ACTION_TARGET)
        if any(amount > 0.0 for amount in observation.belief.known_ground_dirt):
            actions.append(ACTION_DIRT)
        if any(
            free and not observed
            for free, observed in zip(
                observation.belief.traversable, observation.belief.observed
            )
        ):
            actions.append(ACTION_FRONTIER)
        actions.append(ACTION_WAIT)
        return tuple(actions)

    def choose_action(self, observation: AgentObservation, *, explore: bool) -> str:
        actions = self.available_actions(observation)
        if explore and self._rng.random() < self.epsilon:
            return self._rng.choice(actions)
        row = self._row(belief_state(observation))
        return max(actions, key=lambda action: (row[action], -ALL_ACTIONS.index(action)))

    def action_trajectory(self, observation: AgentObservation, action: str) -> TrajectoryAction:
        if action == ACTION_TARGET:
            grasp = self._grasp_if_reached(observation)
            if grasp is not None:
                return grasp
            goals = [
                (target.x, target.y)
                for target in observation.belief.known_targets
                if not target.cleared
                and target.attempts < self.config.max_grasp_attempts
            ]
            result = self._try_goals(observation, goals, clean=False)
            if result is not None:
                return result
            reorientation = self._reorientation_arc(observation)
            return reorientation if reorientation is not None else self._wait(observation)
        if action == ACTION_DIRT:
            goals = [
                cell_center(observation, index)
                for index, amount in enumerate(observation.belief.known_ground_dirt)
                if amount > 0.0
            ]
            result = self._try_goals(observation, goals, clean=True)
            if result is not None:
                return result
            reorientation = self._reorientation_arc(observation)
            return reorientation if reorientation is not None else self._wait(observation)
        if action == ACTION_FRONTIER:
            goals = [
                cell_center(observation, index)
                for index, (free, observed) in enumerate(
                    zip(observation.belief.traversable, observation.belief.observed)
                )
                if free and not observed
            ]
            # Prefer nearby frontier cells; the safe trajectory helper filters
            # current pedestrian circles as static obstacles.
            result = self._try_goals(observation, goals, clean=False)
            if result is not None:
                return result
            reorientation = self._reorientation_arc(observation)
            return reorientation if reorientation is not None else self._wait(observation)
        if action != ACTION_WAIT:
            raise ValueError(f"unknown high-level action: {action}")
        return self._wait(observation)

    def act_with_label(
        self,
        observation: AgentObservation,
        *,
        explore: bool,
    ) -> tuple[tuple[int, ...], str, TrajectoryAction]:
        state = belief_state(observation)
        action = self.choose_action(observation, explore=explore)
        return state, action, self.action_trajectory(observation, action)

    def act(self, observation: AgentObservation) -> TrajectoryAction:
        return self.act_with_label(observation, explore=False)[2]

    def update(
        self,
        state: tuple[int, ...],
        action: str,
        reward: float,
        next_observation: AgentObservation,
        *,
        done: bool,
    ) -> None:
        row = self._row(state)
        next_row = self._row(belief_state(next_observation))
        next_actions = self.available_actions(next_observation)
        bootstrap = 0.0 if done else max(next_row[item] for item in next_actions)
        target = reward + self.discount * bootstrap
        row[action] += self.learning_rate * (target - row[action])

    def checkpoint(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "policy": self.name,
            "state_features": [
                "observed_ratio_bin",
                "known_dirt_count_bin",
                "known_target_count_bin",
                "current_pedestrian_count_bin",
                "distance_budget_bin",
            ],
            "actions": list(ALL_ACTIONS),
            "learning_rate": self.learning_rate,
            "discount": self.discount,
            "epsilon": self.epsilon,
            "policy_seed": self._policy_seed,
            "truth_access_used": False,
            "q_table": self.q_table,
        }

    def save(self, path: str | Path) -> None:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            json.dumps(self.checkpoint(), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    @classmethod
    def load(cls, config: TaskConfig, path: str | Path) -> "QLearningPolicy":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if data.get("policy") != cls.name or data.get("truth_access_used") is not False:
            raise ValueError("invalid or truth-contaminated Q-learning checkpoint")
        if tuple(data.get("actions", ())) != ALL_ACTIONS:
            raise ValueError("checkpoint action schema mismatch")
        policy = cls(
            config,
            learning_rate=float(data["learning_rate"]),
            discount=float(data["discount"]),
            epsilon=float(data["epsilon"]),
            seed=int(data.get("policy_seed", 0)),
        )
        policy.q_table = {
            str(state): {str(action): float(value) for action, value in row.items()}
            for state, row in data["q_table"].items()
        }
        return policy


@dataclass(frozen=True)
class SplitRollout:
    seeds: tuple[int, ...]
    success_rate: float
    mean_observed_ratio: float
    mean_task_distance: float
    mean_reward: float


def rollout_belief_only(
    config: TaskConfig,
    policy: QLearningPolicy,
    seeds: Sequence[int],
    *,
    explore: bool,
    learn: bool,
) -> SplitRollout:
    successes = 0
    observed = []
    distances = []
    rewards = []
    for seed in seeds:
        env = ActiveCleaningEnv(config)  # Deliberately no evaluation token.
        observation = env.reset(seed=int(seed))
        policy.reset(episode_seed=RoleSeeds.from_master(int(seed)).policy)
        episode_reward = 0.0
        if observation.observed_ratio >= config.observation_threshold and not any(
            amount > 0.0 for amount in observation.belief.known_ground_dirt
        ) and not any(not target.cleared for target in observation.belief.known_targets):
            successes += 1
        else:
            for _ in range(config.max_steps):
                state, label, action = policy.act_with_label(observation, explore=explore)
                result = env.step(action)
                episode_reward += result.reward
                if learn:
                    policy.update(
                        state,
                        label,
                        result.reward,
                        result.observation,
                        done=result.terminated or result.truncated,
                    )
                observation = result.observation
                if result.terminated or result.truncated:
                    successes += int(result.terminated and not result.truncated)
                    break
        observed.append(observation.observed_ratio)
        distances.append(observation.task_distance)
        rewards.append(episode_reward)
    count = max(1, len(seeds))
    return SplitRollout(
        seeds=tuple(int(seed) for seed in seeds),
        success_rate=successes / count,
        mean_observed_ratio=sum(observed) / count,
        mean_task_distance=sum(distances) / count,
        mean_reward=sum(rewards) / count,
    )


def train_q_policy(
    config: TaskConfig,
    *,
    train_seeds: Sequence[int],
    validation_seeds: Sequence[int],
    test_seeds: Sequence[int],
    policy_seed: int,
    learning_rate: float = 0.25,
    discount: float = 0.95,
    epsilon: float = 0.20,
) -> tuple[QLearningPolicy, dict[str, Any]]:
    if not train_seeds or not validation_seeds or not test_seeds:
        raise ValueError("train, validation, and test seed splits must be non-empty")
    if set(train_seeds) & set(validation_seeds) or set(train_seeds) & set(test_seeds) or set(validation_seeds) & set(test_seeds):
        raise ValueError("train, validation, and test seeds must be disjoint")
    policy = QLearningPolicy(
        config,
        learning_rate=learning_rate,
        discount=discount,
        epsilon=epsilon,
        seed=policy_seed,
    )
    train = rollout_belief_only(
        config, policy, train_seeds, explore=True, learn=True
    )
    validation = rollout_belief_only(
        config, policy, validation_seeds, explore=False, learn=False
    )
    test = rollout_belief_only(
        config, policy, test_seeds, explore=False, learn=False
    )
    report = {
        "schema_version": 1,
        "policy": policy.name,
        "policy_seed": int(policy_seed),
        "truth_access_used": False,
        "role_seed_scheme": "splitmix64-v1",
        "role_seeds": {
            split: [
                {"task": int(seed), **dict(RoleSeeds.from_master(int(seed)).as_mapping())}
                for seed in seeds
            ]
            for split, seeds in (
                ("train", train_seeds),
                ("validation", validation_seeds),
                ("test", test_seeds),
            )
        },
        "split_overlap": False,
        "train": train.__dict__,
        "validation": validation.__dict__,
        "test": test.__dict__,
        "q_state_count": len(policy.q_table),
    }
    return policy, report
