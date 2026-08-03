import json
from pathlib import Path

from coverage_optimizer_report import build_report


def _write_run(root: Path, bucket: str, seed: int, profile: str):
    run = root / bucket / f"seed_{seed}"
    run.mkdir(parents=True)
    (run / "coverage_path.json").write_text(json.dumps({
        "selected_swath_angle_deg": 0,
        "execution_swaths": [[[0, 0], [4, 0]]],
        "turns": [[[4, 0], [4.5, 0.5]]],
    }))
    (run / "coverage_report.json").write_text(json.dumps({
        "success": True,
        "planning_swath_spacing_m": 0.52 if profile == "optimized" else 0.35,
        "collision_count": 0,
        "keepout_violation_sample_count": 0,
        "brush_state_violation_sample_count": 0,
        "brush_disabled_on_exit": True,
        "empirical_metrics": {
            "coverage_rate": 1.0, "repeat_rate": 0.15, "miss_rate": 0.0,
            "brush_on_distance_m": 20.0, "brush_off_distance_m": 5.0,
            "total_distance_m": 25.0, "actual_duration_sec": 60.0,
            "primary_swath_straightness_error": {"p95_m": 0.04},
        },
        "localization_regression_during_coverage": {"rmse_m": 0.03},
        "coverage_repair": {"passes": []},
    }))
    (run / "gazebo_cleaning_telemetry.json").write_text(json.dumps({
        "targets_cleaned": 10, "targets_total": 10,
    }))


def test_report_collects_five_seed_profiles_and_keeps_missing_live_gates_closed(tmp_path):
    for seed in range(5):
        _write_run(tmp_path, "baseline", seed, "legacy")
        _write_run(tmp_path, "selected", seed + 10, "optimized")

    report = build_report(tmp_path)

    assert len(report["baseline"]) == 5
    assert len(report["selected"]) == 5
    assert report["gates"]["targets_10_of_10"] is True
    assert report["gates"]["mcap_replay"] is False
    assert report["gates"]["dynamic_matrix"] is False
    assert report["pass"] is False
