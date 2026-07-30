from sanitation_tasks.large_map import (
    MapSpec,
    build_zone_index,
    reload_map,
    schedule_routes,
    serialize_map,
    simulate_localization,
)


def test_map_serialization_and_zone_index(tmp_path):
    spec = MapSpec()
    report = serialize_map(spec, tmp_path)
    assert report["area_m2"] == 20_000
    assert reload_map(tmp_path)["resolution"] == 0.1
    assert len(build_zone_index(spec)) == 20


def test_truth_is_separate_and_rmse_meets_contract():
    row = simulate_localization(MapSpec(), 3)
    assert row["truth_source"] != row["estimate_source"]
    assert row["self_comparison_used"] is False
    assert row["rmse_m"] <= 0.05


def test_scheduler_is_zone_exact_and_failures_are_observable():
    rows = schedule_routes(MapSpec(), 20)
    assert all(row["requested_zone_id"] == row["selected_zone_id"] for row in rows)
    assert sum(row["resume_success"] for row in rows) == 19
