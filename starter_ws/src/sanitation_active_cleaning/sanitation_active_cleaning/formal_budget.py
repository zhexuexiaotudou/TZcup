"""Frozen task-budget contract shared by formal RL runners and reports."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


SPLITS = ("train", "validation", "hidden")


@dataclass(frozen=True)
class FormalRLBudget:
    path: Path
    payload: dict[str, Any]

    @property
    def stage_a_counts(self) -> dict[str, int]:
        return dict(self.payload["stage_a_fixed_map"]["task_counts"])

    @property
    def multimap_counts(self) -> dict[str, int]:
        return dict(self.payload["multimap_generalization"]["task_counts"])

    @property
    def multimap_map_counts(self) -> dict[str, int]:
        return dict(self.payload["multimap_generalization"]["map_counts"])

    @property
    def multimap_missions_per_map(self) -> dict[str, int]:
        return dict(self.payload["multimap_generalization"]["missions_per_map"])

    @property
    def policy_seeds(self) -> tuple[int, ...]:
        return tuple(self.payload["policy"]["training_seeds"])

    @property
    def max_steps_per_episode(self) -> int:
        return int(self.payload["execution"]["max_steps_per_episode"])

    def report(self) -> dict[str, Any]:
        return {
            "contract_id": self.payload["contract_id"],
            "schema_version": self.payload["schema_version"],
            "semantics": dict(self.payload["semantics"]),
            "stage_a_fixed_map": {
                "fixed_map_id": self.payload["stage_a_fixed_map"]["fixed_map_id"],
                "task_counts": self.stage_a_counts,
                "phase_order": list(self.payload["stage_a_fixed_map"]["phase_order"]),
                "hidden_tasks_visible_before_freeze": False,
            },
            "multimap_generalization": {
                "map_counts": self.multimap_map_counts,
                "missions_per_map": self.multimap_missions_per_map,
                "task_counts": self.multimap_counts,
                "hidden_tasks_visible_before_freeze": False,
            },
            "policy": {
                "training_seeds": list(self.policy_seeds),
                "required_seed_count": len(self.policy_seeds),
                "selection_source": self.payload["policy"]["selection_source"],
            },
            "execution": {
                "max_steps_per_episode": self.max_steps_per_episode,
                "max_steps_is_not_a_substitute_for_task_budget": True,
                "mode": self.payload["execution"]["mode"],
                "smoke_budget_can_pass_formal_acceptance": False,
            },
        }


def _mapping(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a mapping")
    return value


def _exact_counts(value: Any, path: str, expected: dict[str, int]) -> dict[str, int]:
    row = _mapping(value, path)
    if set(row) != set(SPLITS):
        raise ValueError(f"{path} must contain exactly {SPLITS}")
    actual: dict[str, int] = {}
    for split in SPLITS:
        item = row[split]
        if isinstance(item, bool) or not isinstance(item, int) or item <= 0:
            raise ValueError(f"{path}.{split} must be a positive integer")
        actual[split] = item
    if actual != expected:
        raise ValueError(f"{path} must equal {expected}, got {actual}")
    return actual


def load_formal_rl_budget(path: str | Path) -> FormalRLBudget:
    contract_path = Path(path).resolve()
    try:
        payload = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot read formal RL budget contract: {exc}") from exc
    root = _mapping(payload, "formal RL budget contract")
    if root.get("schema_version") != 1 or root.get("contract_id") != "tzcup_formal_rl_budget_v1":
        raise ValueError("formal RL budget contract identity is invalid")
    semantics = _mapping(root.get("semantics"), "semantics")
    if semantics.get("max_steps_is_episode_truncation_guard_not_task_or_episode_budget") is not True:
        raise ValueError("contract must distinguish step guard from task/episode budget")
    stage_a = _mapping(root.get("stage_a_fixed_map"), "stage_a_fixed_map")
    if stage_a.get("fixed_map_id") != "stage-a-fixed-formal-map-000":
        raise ValueError("Stage-A fixed map identity is invalid")
    _exact_counts(stage_a.get("task_counts"), "stage_a_fixed_map.task_counts", {"train": 10000, "validation": 500, "hidden": 1000})
    if stage_a.get("phase_order") != ["train", "validation", "freeze_configuration", "hidden"]:
        raise ValueError("Stage-A must freeze configuration before hidden tasks")
    if stage_a.get("hidden_tasks_visible_before_freeze") is not False:
        raise ValueError("Stage-A hidden tasks must remain unavailable before freeze")
    multimap = _mapping(root.get("multimap_generalization"), "multimap_generalization")
    maps = _exact_counts(multimap.get("map_counts"), "multimap_generalization.map_counts", {"train": 32, "validation": 8, "hidden": 12})
    missions = _exact_counts(multimap.get("missions_per_map"), "multimap_generalization.missions_per_map", {"train": 200, "validation": 100, "hidden": 100})
    task_counts = _exact_counts(multimap.get("task_counts"), "multimap_generalization.task_counts", {"train": 6400, "validation": 800, "hidden": 1200})
    if task_counts != {name: maps[name] * missions[name] for name in SPLITS}:
        raise ValueError("multi-map task counts must equal map_count times missions_per_map")
    if multimap.get("hidden_tasks_visible_before_freeze") is not False:
        raise ValueError("multi-map hidden tasks must remain unavailable before freeze")
    policy = _mapping(root.get("policy"), "policy")
    seeds = policy.get("training_seeds")
    if (
        not isinstance(seeds, list)
        or len(seeds) != 5
        or len(set(seeds)) != 5
        or any(isinstance(seed, bool) or not isinstance(seed, int) or seed < 0 for seed in seeds)
        or policy.get("required_seed_count") != 5
        or policy.get("selection_source") != "validation_only_before_hidden"
    ):
        raise ValueError("policy seed/freeze contract is invalid")
    execution = _mapping(root.get("execution"), "execution")
    if execution.get("max_steps_per_episode") != 400:
        raise ValueError("formal per-episode step guard must equal 400")
    if execution.get("max_steps_is_not_a_substitute_for_task_budget") is not True or execution.get("smoke_budget_can_pass_formal_acceptance") is not False:
        raise ValueError("execution contract permits a smoke budget")
    if execution.get("mode") != "serial_one_policy_seed_at_a_time":
        raise ValueError("formal RL execution must remain serial")
    return FormalRLBudget(contract_path, root)


def all_multimap_tasks(budget: FormalRLBudget, split: str) -> tuple[tuple[int, int], ...]:
    if split not in SPLITS:
        raise ValueError(f"unknown multi-map split: {split}")
    return tuple(
        (map_index, mission_index)
        for map_index in range(budget.multimap_map_counts[split])
        for mission_index in range(budget.multimap_missions_per_map[split])
    )


def all_stage_a_task_indices(budget: FormalRLBudget, split: str) -> tuple[int, ...]:
    if split not in SPLITS:
        raise ValueError(f"unknown Stage-A split: {split}")
    return tuple(range(budget.stage_a_counts[split]))
