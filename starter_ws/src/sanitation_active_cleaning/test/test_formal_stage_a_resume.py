"""Offline regressions only; these small tasks are not formal-budget evidence."""

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path

import pytest

from sanitation_active_cleaning import formal_stage_a_training as stage_a
from sanitation_active_cleaning.formal_training import FormalEpisode
from sanitation_active_cleaning.models import TaskConfig, TaskLayout


def _episodes(tmp_path):
    config = TaskConfig.from_mapping({
        "geofence": [[0, 0], [4, 0], [4, 3], [0, 3]],
        "start": {"x": 0.5, "y": 0.5, "yaw": 0.0},
        "grid_resolution": 0.5, "sensing_radius": 2.0,
        "sensing_fov_rad": 2.0 * math.pi, "cleaning_width": 1.0,
        "vehicle_radius": 0.1, "min_turn_radius": 0.3,
        "path_sample_spacing": 0.1, "max_steps": 3,
        "ground_dirt_count": 0, "discrete_target_count": 0,
        "pedestrian_count": 0,
    })
    return [FormalEpisode(
        split="stage_a_train", map_index=0, mission_index=i,
        map_id="offline-unit-map", episode_id=f"offline-task-{i}",
        mission_seed=100 + i, area_m2=12.0, aspect_ratio=4 / 3,
        config=config, layout=TaskLayout(), artifact_root=tmp_path / str(i),
    ) for i in range(3)]


def test_stage_a_resume_matches_uninterrupted_training_and_retains_failure(tmp_path, monkeypatch):
    episodes = _episodes(tmp_path)
    expected = stage_a._train(episodes, 7)
    original_reset = stage_a.ActiveCleaningEnv.reset

    def interrupted_reset(self, *, seed):
        if seed == 101:
            raise RuntimeError("injected interruption")
        return original_reset(self, seed=seed)

    checkpoint = tmp_path / "checkpoint.json"
    with monkeypatch.context() as patch:
        patch.setattr(stage_a.ActiveCleaningEnv, "reset", interrupted_reset)
        with pytest.raises(RuntimeError, match="injected interruption"):
            stage_a._train(episodes, 7, checkpoint_path=checkpoint)
    saved = json.loads(checkpoint.read_text())
    assert len(saved["episodes"]) == 1
    failure_bytes = checkpoint.with_suffix(".failures.jsonl").read_bytes()
    assert json.loads(failure_bytes)["completed_episode_count"] == 1
    assert b"injected interruption" not in failure_bytes
    assert stage_a._train(episodes, 7, checkpoint_path=checkpoint) == expected
    assert checkpoint.with_suffix(".failures.jsonl").read_bytes() == failure_bytes
    assert len(json.loads(checkpoint.read_text())["episodes"]) == 3
    with monkeypatch.context() as patch:
        patch.setattr(stage_a.ActiveCleaningEnv, "reset", interrupted_reset)
        assert stage_a._train(episodes, 7, checkpoint_path=checkpoint) == expected


@pytest.mark.parametrize("change", ["seed", "layout", "steps", "context", "corrupt"])
def test_stage_a_checkpoint_refuses_changed_input_or_state(tmp_path, change):
    episodes = _episodes(tmp_path)
    checkpoint = tmp_path / "checkpoint.json"
    stage_a._train(episodes, 7, checkpoint_path=checkpoint)
    seed, context = 7, None
    if change == "seed":
        seed = 17
    elif change == "layout":
        episodes[0] = replace(episodes[0], layout=TaskLayout(discrete_targets=(("x", 1.0, 1.0),)))
    elif change == "steps":
        episodes[0] = replace(episodes[0], config=replace(episodes[0].config, max_steps=4))
    elif change == "context":
        context = {"session": "different"}
    else:
        saved = json.loads(checkpoint.read_text())
        saved["q_table"] = {}
        checkpoint.write_text(json.dumps(saved))
    with pytest.raises(ValueError, match="checkpoint"):
        stage_a._train(episodes, seed, checkpoint_path=checkpoint, checkpoint_identity=context)


def test_stage_a_real_evaluation_preserves_frozen_table_and_failure_rows(tmp_path):
    table = {}
    rows = stage_a._evaluate(_episodes(tmp_path)[:1], table, 7)
    assert table == {}
    assert len(rows) == 1
    assert rows[0]["baseline_result"]["seed"] == rows[0]["policy_result"]["seed"] == 100
    assert "truncated" in rows[0]["baseline_result"]
    assert "truncated" in rows[0]["policy_result"]


def test_stage_a_checkpoint_rejects_malformed_table_even_with_matching_hash(tmp_path):
    checkpoint = tmp_path / "checkpoint.json"
    episodes = _episodes(tmp_path)
    stage_a._train(episodes, 7, checkpoint_path=checkpoint)
    saved = json.loads(checkpoint.read_text())
    saved["q_table"] = {"0,0,0,0,0": {"target": "invalid"}}
    saved["state_sha256"] = stage_a._digest({"episodes": saved["episodes"], "q_table": saved["q_table"]})
    checkpoint.write_text(json.dumps(saved))
    with pytest.raises(ValueError, match="checkpoint state"):
        stage_a._train(episodes, 7, checkpoint_path=checkpoint)


def test_stage_a_learning_rejects_hidden_and_duplicate_tasks(tmp_path):
    episodes = _episodes(tmp_path)
    with pytest.raises(ValueError, match="only training"):
        stage_a._train([replace(episodes[0], split="stage_a_hidden")], 7)
    with pytest.raises(ValueError, match="distinct"):
        stage_a._train([episodes[0], episodes[0]], 7)


def test_train_validation_phase_never_requests_hidden_or_freeze(tmp_path, monkeypatch):
    from sanitation_campus_scenario import hidden_materializer

    root = Path(__file__).resolve().parents[4]
    snapshot = tmp_path / "snapshot.json"
    snapshot.write_text("{}")
    session = tmp_path / "session.json"
    session.write_text("{}")
    seen = []

    def tasks(budget, phase):
        assert phase in ("train", "validation")
        seen.append(phase)
        return (0,)

    def forbidden(**kwargs):
        pytest.fail("train-validation accessed hidden freeze")

    monkeypatch.setattr(stage_a, "all_stage_a_task_indices", tasks)
    monkeypatch.setattr(stage_a, "materialize_stage_a_episode", lambda *a, **kw: _episodes(tmp_path)[0])
    monkeypatch.setattr(hidden_materializer, "require_canonical_formal_inputs", lambda **kw: None)
    monkeypatch.setattr(hidden_materializer, "commit_hidden_configuration_freeze", forbidden)
    report = stage_a.execute(argparse.Namespace(
        budget_contract=root / "starter_ws/src/sanitation_active_cleaning/config/formal_rl_budget_contract.yaml",
        scenario_config=tmp_path / "unused", motion_profile=tmp_path / "unused",
        work_root=tmp_path, snapshot=snapshot, session=session, hidden_receipt_root=None,
        map_resolution=0.5, planning_resolution=2.0, phase="train-validation",
    ))
    assert seen == ["train", "validation"]
    assert report["hidden_evaluated"] is False
    assert report["formal_acceptance_passed"] is False
    assert len(report["policy_runs"]) == 5
