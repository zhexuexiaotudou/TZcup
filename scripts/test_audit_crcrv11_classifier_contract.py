from audit_crcrv11_classifier_contract import (
    area, candidate_key, describe, expand, expected_sampler_stats, intersection_area, remap_path,
)
from pathlib import Path


def test_geometry_helpers():
    assert area([0, 0, 10, 5]) == 50
    assert intersection_area([0, 0, 10, 10], [5, 5, 20, 20]) == 25
    assert describe([1, 2, 3])["p50"] == 2
    assert expand([0, 0, 10, 10]) == [-3.0, -3.0, 13.0, 13.0]


def test_candidate_key_pairs_tight_and_context():
    tight = {"path": "paper_litter/tight/scene_1_004_2.png", "scene": "scene_1", "frame_index": 4}
    context = {**tight, "path": "paper_litter/context/scene_1_004_2.png"}
    assert candidate_key(tight) == candidate_key(context)


def test_sampler_repeat_factor_matches_weighted_sampler_contract():
    stats = expected_sampler_stats(total=400, count=10)
    assert stats["expected_draws_per_unique_crop"] == 10
    assert 0 < stats["expected_unique_coverage"] <= 1


def test_windows_prefix_can_be_mapped_for_container(tmp_path: Path):
    result = remap_path(r"F:\Project\TZcup\.workspace\x.png", [(r"F:\Project\TZcup", tmp_path)])
    assert result == tmp_path / ".workspace" / "x.png"
