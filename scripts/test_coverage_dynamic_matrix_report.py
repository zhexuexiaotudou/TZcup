import json
from pathlib import Path

from coverage_dynamic_matrix_report import build_report


def _write_run(root: Path, seed: int, count: int = 8) -> None:
    run = root / f"run_seed_{seed}"
    run.mkdir()
    trials = [
        {
            "valid": True,
            "mission_progress_resumed": True,
            "minimum_separation_gate_pass": True,
        }
        for _ in range(count)
    ]
    (run / "dynamic_obstacle_report.json").write_text(json.dumps({
        "requested_trial_count": count,
        "completed_trial_count": count,
        "dynamic_obstacle_valid_trials": count,
        "collision_count": 0,
        "minimum_observed_separation_m": 0.25,
        "repeated_oscillation_count": 0,
        "success": True,
        "trials": trials,
    }), encoding="utf-8")
    (run / "coverage_report.json").write_text(json.dumps({
        "full_execution_success": True,
        "brush_disabled_on_exit": True,
    }), encoding="utf-8")


def test_three_bounded_runs_form_a_passing_twenty_interaction_matrix(tmp_path):
    _write_run(tmp_path, 160)
    _write_run(tmp_path, 161)
    _write_run(tmp_path, 162)
    report = build_report(tmp_path)
    assert report["dynamic_obstacle_valid_trials"] == 24
    assert report["dynamic_recovery_rate"] == 1.0
    assert report["mission_resume_rate"] == 1.0
    assert report["pass"] is True


def test_collision_or_oscillation_fails_closed(tmp_path):
    _write_run(tmp_path, 160)
    _write_run(tmp_path, 161)
    _write_run(tmp_path, 162)
    report_path = tmp_path / "run_seed_161" / "dynamic_obstacle_report.json"
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    payload["collision_count"] = 1
    payload["repeated_oscillation_count"] = 1
    report_path.write_text(json.dumps(payload), encoding="utf-8")
    report = build_report(tmp_path)
    assert report["gates"]["collision_count_zero"] is False
    assert report["gates"]["repeated_oscillation_zero"] is False
    assert report["pass"] is False
