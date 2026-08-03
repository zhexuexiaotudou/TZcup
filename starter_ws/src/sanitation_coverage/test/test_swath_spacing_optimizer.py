from sanitation_coverage.swath_optimizer import optimize_swath_spacing


def test_selects_shortest_valid_empirical_spacing():
    observations = [
        {"spacing_m": 0.42, "coverage_rate": 0.999, "repeat_rate": 0.28, "path_length_m": 100},
        {"spacing_m": 0.48, "coverage_rate": 0.997, "repeat_rate": 0.18, "path_length_m": 88},
        {"spacing_m": 0.52, "coverage_rate": 0.990, "repeat_rate": 0.10, "path_length_m": 82},
    ]
    spacing, ranked, fallback = optimize_swath_spacing(observations)
    assert spacing == 0.48
    assert not fallback
    assert any(not item.valid for item in ranked)


def test_legacy_spacing_is_only_fail_closed_fallback():
    spacing, _, fallback = optimize_swath_spacing([])
    assert spacing == 0.35
    assert fallback
