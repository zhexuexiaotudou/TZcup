from sanitation_coverage.metrics import (
    path_length,
    raster_coverage_metrics,
    repair_degenerate_swaths,
)


def test_path_length():
    assert path_length([(0.0, 0.0), (3.0, 4.0)]) == 5.0


def test_single_swath_coverage_is_bounded():
    metrics = raster_coverage_metrics(
        [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)],
        [((0.0, 0.5), (2.0, 0.5))],
        width=1.0,
        resolution=0.1,
    )
    assert metrics["coverage_rate"] > 0.95
    assert metrics["coverage_rate"] <= 1.0
    assert metrics["repeat_rate"] == 0.0


def test_repair_degenerate_swaths_uses_turn_boundaries():
    swaths = [((0.0, 0.5), (0.0, 0.5)), ((2.0, 1.5), (2.0, 1.5))]
    turns = [[(2.0, 0.5), (2.0, 1.5)]]
    repaired, applied = repair_degenerate_swaths(
        swaths, turns, [(0.0, 0.5), (0.0, 1.5)]
    )
    assert applied
    assert repaired == [
        ((0.0, 0.5), (2.0, 0.5)),
        ((2.0, 1.5), (0.0, 1.5)),
    ]
