from pathlib import Path

import numpy as np

from sanitation_learning.assets import load_asset_registry
from sanitation_learning.rendered import generate_frame, split_for_seed, validate_annotations, write_dataset


REGISTRY = Path(__file__).parents[1] / "config" / "asset_registry.yaml"


def test_rendered_frame_has_independent_depth_and_hard_negatives():
    frame = generate_frame(18, 2, load_asset_registry(REGISTRY))
    assert frame.split == "test"
    assert frame.world_id == "unseen_mixed_paving"
    assert frame.objects
    assert frame.negatives
    assert np.isfinite(frame.depth_m).mean() > 0.95
    assert all(item["target_label_count"] == 0 for item in frame.negatives)


def test_scene_split_keeps_assets_worlds_and_frames_disjoint(tmp_path):
    seeds = list(range(20))
    manifest = write_dataset(tmp_path, REGISTRY, seeds, frames_per_scene=2)
    assert manifest["frame_count"] == 40
    assert split_for_seed(7) == "val" and split_for_seed(8) == "test"
    train = set(manifest["split_assets"]["train"])
    val = set(manifest["split_assets"]["val"])
    test = set(manifest["split_assets"]["test"])
    assert not train & val and not train & test and not val & test
    assert manifest["adjacent_frames_cross_split"] is False
    qa = validate_annotations(tmp_path)
    assert qa["annotation_error_count"] == 0
    assert qa["asset_split_leakage"] is False
