import math

import pytest

from sanitation_tasks.evaluation import (
    assert_comparable_frames,
    product_estop_latency_pass,
    synchronize_samples,
)


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


@pytest.mark.parametrize("p95_sec", [0.0, 0.02, 0.200])
def test_product_estop_latency_accepts_finite_nonnegative_values_at_gate(p95_sec):
    assert product_estop_latency_pass(p95_sec)


@pytest.mark.parametrize(
    "p95_sec", [0.201, -0.001, math.nan, math.inf, -math.inf, None, "invalid"]
)
def test_product_estop_latency_rejects_invalid_or_over_gate_values(p95_sec):
    assert not product_estop_latency_pass(p95_sec)
