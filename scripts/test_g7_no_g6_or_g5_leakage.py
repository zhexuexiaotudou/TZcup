from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g7_detector_dataset import WORLD_IDS


def test_g7_world_namespace_is_reserved_and_independent():
    worlds = {world for values in WORLD_IDS.values() for world in values}
    assert len(worlds) >= 8
    assert all(world.startswith("g7v4_") for world in worlds)
    assert not any(world.startswith(("g6_", "g5_", "g5v2_")) for world in worlds)


def test_only_holdout_may_share_train_worlds():
    train = set(WORLD_IDS["TRAIN"])
    assert set(WORLD_IDS["IN_DOMAIN_HOLDOUT"]) == train
    for split, worlds in WORLD_IDS.items():
        if split not in {"TRAIN", "IN_DOMAIN_HOLDOUT"}:
            assert train.isdisjoint(worlds)
