import json
from pathlib import Path

import pytest

from sanitation_campus_scenario.generator import generate_episode, load_config
from sanitation_campus_scenario.io import write_episode
from sanitation_research_demo.harness import Bundle, build_active_task
from sanitation_research_demo.cli import build_parser


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "sanitation_campus_scenario" / "config" / "default_scenario.yaml"


def test_scenario_bridge_preserves_identity_and_hides_truth_from_task_observation(tmp_path):
    scenario = tmp_path / "episode"
    write_episode(
        scenario,
        generate_episode(load_config(CONFIG), "research", "train", 0, 0, include_proxy=True),
    )
    bundle = Bundle.load(scenario)
    config, layout = build_active_task(bundle)
    assert bundle.public_manifest["episode_id"] == bundle.truth["episode_id"]
    assert len(layout.discrete_targets) == 20
    assert len(layout.ground_dirt_regions) == 18
    assert len(config.static_obstacles) == 60
    assert config.observation_threshold == 0.95
    serialized_config = json.dumps(config.__dict__, default=lambda value: value.__dict__)
    assert "object_" not in serialized_config


def test_bundle_rejects_world_hash_mismatch(tmp_path):
    scenario = tmp_path / "episode"
    write_episode(
        scenario,
        generate_episode(load_config(CONFIG), "research", "train", 0, 0),
    )
    world = scenario / "public" / "world.sdf"
    world.write_text(world.read_text(encoding="utf-8") + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="world hash mismatch"):
        Bundle.load(scenario)


def test_research_demo_cli_cannot_preview_a_hidden_split(tmp_path):
    with pytest.raises(SystemExit):
        build_parser().parse_args([
            "--config", str(CONFIG), "--split", "hidden", "--output", str(tmp_path),
        ])
