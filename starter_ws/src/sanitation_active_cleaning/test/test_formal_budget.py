import json
from pathlib import Path

import pytest

from sanitation_active_cleaning.formal_budget import (
    all_multimap_tasks,
    all_stage_a_task_indices,
    load_formal_rl_budget,
)
from sanitation_campus_scenario.generator import generate_stage_a_episode, load_config


ROOT = Path(__file__).resolve().parents[4]
BUDGET = ROOT / "starter_ws/src/sanitation_active_cleaning/config/formal_rl_budget_contract.yaml"
SCENARIO = ROOT / "starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml"


def test_formal_budget_expands_all_frozen_tasks_and_distinguishes_step_guard() -> None:
    budget = load_formal_rl_budget(BUDGET)
    assert len(all_stage_a_task_indices(budget, "train")) == 10000
    assert len(all_stage_a_task_indices(budget, "validation")) == 500
    assert len(all_stage_a_task_indices(budget, "hidden")) == 1000
    assert len(all_multimap_tasks(budget, "train")) == 6400
    assert len(all_multimap_tasks(budget, "validation")) == 800
    assert len(all_multimap_tasks(budget, "hidden")) == 1200
    assert budget.max_steps_per_episode == 400
    assert budget.report()["semantics"]["max_steps_is_episode_truncation_guard_not_task_or_episode_budget"] is True


def test_stage_a_tasks_keep_one_layout_but_have_phase_qualified_independent_seeds() -> None:
    config = load_config(SCENARIO)
    train = generate_stage_a_episode(config, "formal", "train", 0)
    validation = generate_stage_a_episode(config, "formal", "validation", 0)
    next_train = generate_stage_a_episode(config, "formal", "train", 1)
    manifests = [json.loads(row["public/episode_manifest.json"]) for row in (train, validation, next_train)]
    evaluator = [json.loads(row["evaluator/episode_manifest.json"]) for row in (train, validation, next_train)]
    assert {item["map_id"] for item in manifests} == {"stage-a-fixed-formal-map-000"}
    assert [item["split"] for item in manifests] == ["stage_a_train", "stage_a_validation", "stage_a_train"]
    assert manifests[0]["mission_index"] == 0
    assert manifests[2]["mission_index"] == 1
    assert manifests[0]["episode_id"] == "stage-a-fixed-formal-map-000-mission-00000"
    assert manifests[1]["episode_id"] == "stage-a-fixed-formal-map-000-mission-00000"
    assert manifests[2]["episode_id"] == "stage-a-fixed-formal-map-000-mission-00001"
    assert evaluator[0]["seeds"]["layout"] == evaluator[1]["seeds"]["layout"] == evaluator[2]["seeds"]["layout"]
    assert evaluator[0]["seeds"]["dirt"] != evaluator[1]["seeds"]["dirt"] != evaluator[2]["seeds"]["dirt"]


def test_budget_rejects_any_smoke_sized_multimap_contract(tmp_path: Path) -> None:
    payload = BUDGET.read_text(encoding="utf-8").replace("    train: 6400", "    train: 32")
    invalid = tmp_path / "budget.yaml"
    invalid.write_text(payload, encoding="utf-8")
    with pytest.raises(ValueError, match="task_counts"):
        load_formal_rl_budget(invalid)
