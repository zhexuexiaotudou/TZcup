import hashlib
import json
import math

import pytest

import sanitation_active_cleaning.formal_training as formal_training
from sanitation_active_cleaning.environment import ActiveCleaningEnv, TrajectoryAction
from sanitation_active_cleaning.formal_training import (
    FORMAL_FULL_MAP_COUNTS,
    FormalEpisode,
    _belief_only_training_reward,
    _cube_grasp_reach_radius,
    load_full_formal_split_manifest,
    train_and_evaluate,
)
from sanitation_active_cleaning.models import TaskConfig, TaskLayout


def _episode(tmp_path, split, map_index, seed, *, width=4.0):
    config = TaskConfig.from_mapping(
        {
            "geofence": [[0, 0], [width, 0], [width, 3], [0, 3]],
            "start": {"x": 0.5, "y": 0.5, "yaw": 0.0},
            "grid_resolution": 0.5,
            "sensing_radius": 10.0,
            "sensing_fov_rad": 2.0 * math.pi,
            "cleaning_width": 1.0,
            "vehicle_radius": 0.1,
            "grasp_radius": 0.75,
            "min_turn_radius": 0.3,
            "path_sample_spacing": 0.1,
            "observation_threshold": 0.95,
            "ground_clear_threshold": 0.95,
            "discrete_clear_threshold": 0.95,
            "ground_dirt_count": 0,
            "discrete_target_count": 0,
            "pedestrian_count": 0,
            "max_steps": 40,
        }
    )
    map_id = f"{split}-map-{map_index:03d}"
    return FormalEpisode(
        split=split,
        map_index=map_index,
        mission_index=0,
        map_id=map_id,
        episode_id=f"{map_id}-mission-000",
        mission_seed=seed,
        area_m2=12.0,
        aspect_ratio=width / 3.0,
        config=config,
        layout=TaskLayout(
            ground_dirt_polygons=(((0.25, 0.25), (0.75, 0.25), (0.75, 0.75), (0.25, 0.75)),),
            discrete_targets=((f"cube-{seed}", 0.6, 0.5),),
        ),
        artifact_root=tmp_path / map_id,
    )


def _full_split(tmp_path, split, *, width=4.0):
    count = FORMAL_FULL_MAP_COUNTS[
        {"train": "train", "val": "validation", "hidden": "hidden"}[split]
    ]
    seed_base = {"train": 1000, "val": 2000, "hidden": 3000}[split]
    return [
        _episode(tmp_path, split, index, seed_base + index, width=width)
        for index in range(count)
    ]


def _frozen_scenario(path):
    path.write_text(
        "split:\n"
        "  train: {map_count: 32, missions_per_map: 200}\n"
        "  val: {map_count: 8, missions_per_map: 100}\n"
        "  hidden: {map_count: 12, missions_per_map: 100}\n",
        encoding="utf-8",
    )
    return path


def test_full_map_training_keeps_truth_out_of_control_and_product_claim_blocked(tmp_path):
    checkpoint, training, baseline, validation = train_and_evaluate(
        _full_split(tmp_path, "train"),
        _full_split(tmp_path, "val", width=6.0),
        _full_split(tmp_path, "hidden"),
    )

    assert checkpoint["truth_access_used"] is False
    assert checkpoint["formal_multi_map"] is True
    assert checkpoint["q_table"]
    assert training["truth_used_for_control"] is False
    assert set(training["map_splits"]) == {"train", "validation", "test"}
    assert baseline["mode"] == "full_coverage"
    assert baseline["episodes"]
    assert validation["product_perception_used"] is False
    assert validation["status"] == "research_only_not_product_acceptance"
    assert all("path_ratio_to_full_coverage" in row for row in validation["episodes"])
    assert validation["gate_policy"] == "q_learning_with_systematic_coverage_backstop"
    assert validation["hybrid_episodes"]
    assert all(
        "systematic_coverage_backstop_activated" in row
        for row in validation["hybrid_episodes"]
    )
    assert isinstance(validation["hidden_gate_passed"], bool)
    assert training["formal_multimap_contract"]["actual_distinct_map_counts"] == {
        "train": 32,
        "validation": 8,
        "hidden": 12,
    }


def test_multi_map_training_rejects_overlapping_map_ids(tmp_path):
    train = _full_split(tmp_path, "train")
    validation = _full_split(tmp_path, "val")
    hidden = _full_split(tmp_path, "hidden")
    shared = train[0]
    validation[0] = FormalEpisode(
        **{**shared.__dict__, "split": "val", "mission_seed": 2000}
    )
    with pytest.raises(ValueError, match="map IDs must be disjoint"):
        train_and_evaluate(train, validation, hidden)


def test_training_rejects_small_multi_map_smoke_subset(tmp_path):
    with pytest.raises(ValueError, match="must cover every frozen map index"):
        train_and_evaluate(
            [_episode(tmp_path, "train", 0, 101)],
            [_episode(tmp_path, "val", 0, 201)],
            [_episode(tmp_path, "hidden", 0, 301)],
        )


def test_frozen_split_manifest_rejects_a_reduced_scenario(tmp_path):
    scenario = _frozen_scenario(tmp_path / "scenario.yaml")
    manifest = load_full_formal_split_manifest(scenario)
    assert manifest.map_counts == {"train": 32, "validation": 8, "hidden": 12}
    assert len(manifest.selections["hidden"]) == 1200
    scenario.write_text(
        scenario.read_text(encoding="utf-8").replace("map_count: 12", "map_count: 1"),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="requires frozen map counts"):
        load_full_formal_split_manifest(scenario)


def test_formal_reward_penalizes_wait_and_rewards_public_observation_progress(tmp_path):
    episode = _episode(tmp_path, "train", 0, 101)
    wait_env = ActiveCleaningEnv(episode.config, task_layout=episode.layout)
    wait_observation = wait_env.reset(seed=101)
    wait_result = wait_env.step(
        TrajectoryAction((wait_observation.pose,), clean_ground=False)
    )
    assert _belief_only_training_reward(wait_observation, wait_result) < 0.0

    move_env = ActiveCleaningEnv(episode.config, task_layout=episode.layout)
    move_observation = move_env.reset(seed=101)
    from sanitation_active_cleaning.rl import QLearningPolicy

    move_result = move_env.step(QLearningPolicy(episode.config).act(move_observation))
    assert move_result.observation.observed_ratio >= move_observation.observed_ratio
    assert _belief_only_training_reward(move_observation, move_result) > (
        _belief_only_training_reward(wait_observation, wait_result)
    )


def test_cube_parking_clearance_is_not_reused_as_grasp_reach():
    public = {
        "cube_contract": {
            "grasp_clearance_m": 2.10,
            "grasp_reach_radius_m": 1.10,
        }
    }
    assert _cube_grasp_reach_radius(public) == 1.10
    with pytest.raises(ValueError, match="lacks physical"):
        _cube_grasp_reach_radius(
            {"cube_contract": {"grasp_clearance_m": 2.10}}
        )


def test_cli_returns_nonzero_when_hidden_gate_is_blocked(tmp_path, monkeypatch):
    episode = _episode(tmp_path, "train", 0, 101)
    scenario = _frozen_scenario(tmp_path / "scenario.yaml")
    monkeypatch.setattr(formal_training, "materialize_episode", lambda *args, **kwargs: episode)
    monkeypatch.setattr(
        formal_training,
        "train_and_evaluate",
        lambda *args, **kwargs: ({}, {}, {}, {"hidden_gate_passed": False}),
    )
    snapshot = tmp_path / "snapshot.json"
    snapshot.write_text(json.dumps({
        "source_inventory_sha256": "a" * 64,
        "outputs": {"reports/engineering/formal_competition_vehicle.urdf": {"sha256": "b" * 64}},
    }))
    identity = {
        "snapshot_manifest_sha256": hashlib.sha256(snapshot.read_bytes()).hexdigest(),
        "source_inventory_sha256": "a" * 64,
        "expanded_urdf_sha256": "b" * 64,
    }
    session = tmp_path / "session.json"
    session.write_text(json.dumps({
        "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING", "started_epoch_ns": 1,
        "snapshot": identity,
    }))
    result = formal_training.main(
        [
            "--scenario-config", str(scenario),
            "--motion-profile", str(tmp_path / "motion.yaml"),
            "--work-root", str(tmp_path / "work"),
            "--evidence-root", str(tmp_path / "evidence"),
            "--snapshot", str(snapshot),
            "--session", str(session),
            "--hidden-receipt-root", str(tmp_path / "receipts"),
        ]
    )
    assert result == 2


def test_cli_rejects_a_smaller_explicit_selection_before_materialization(
    tmp_path, monkeypatch
):
    scenario = _frozen_scenario(tmp_path / "scenario.yaml")
    monkeypatch.setattr(
        formal_training,
        "materialize_episode",
        lambda *args, **kwargs: pytest.fail("must reject before materialization"),
    )
    with pytest.raises(ValueError, match="not the frozen full-map manifest"):
        formal_training.main(
            [
                "--scenario-config", str(scenario),
                "--motion-profile", str(tmp_path / "motion.yaml"),
                "--work-root", str(tmp_path / "work"),
                "--evidence-root", str(tmp_path / "evidence"),
                "--train", "0:0",
            ]
        )
