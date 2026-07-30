from sanitation_tasks.efficiency import (
    aggregate_runs,
    search_designs,
    select_design,
    simulate_formal_run,
)


def test_design_search_selects_safe_candidate():
    selected = select_design(search_designs())
    assert 1.25 <= selected.cleaning_width_m <= 1.32
    assert selected.theoretical_rate_m2_h >= 3800.0
    assert selected.braking_distance_m <= selected.safety_braking_envelope_m


def test_formal_time_step_matrix_meets_auto12_thresholds():
    selected = select_design(search_designs())
    runs = [simulate_formal_run(selected, seed) for seed in range(10)]
    aggregate = aggregate_runs(runs)
    assert aggregate["formal_run_count"] >= 10
    assert aggregate["mean_effective_cleaning_rate_m2_h"] >= 3500.0
    assert aggregate["rate_95ci_lower_m2_h"] >= 3500.0
    assert aggregate["minimum_run_rate_m2_h"] >= 3300.0
    assert aggregate["minimum_empirical_coverage"] >= 0.90
    assert aggregate["maximum_missed_cleanable_area_ratio"] <= 0.05
    assert aggregate["maximum_overlap_ratio"] <= 0.15
    assert aggregate["collision_count"] == 0
    assert aggregate["keepout_violation_count"] == 0
    assert aggregate["maximum_trajectory_xy_rmse_m"] <= 0.05
    assert aggregate["brush_final_false_count"] == len(runs)
