import pytest

from sanitation_tasks.evaluation import assert_comparable_frames, synchronize_samples


def test_rejects_direct_map_odom_comparison():
    with pytest.raises(ValueError, match="incomparable frames"):
        assert_comparable_frames("map", "odom")


def test_allows_declared_map_ground_truth_alignment():
    assert_comparable_frames("map", "map_gt", {("map", "map_gt")})


def test_timestamp_sync_respects_tolerance_and_no_reuse():
    estimates = [(1.00, 0), (1.02, 1), (2.00, 2)]
    truths = [(1.01, 10), (2.20, 20)]
    pairs, dropped = synchronize_samples(estimates, truths, 0.05)
    assert len(pairs) == 1
    assert dropped == 2
