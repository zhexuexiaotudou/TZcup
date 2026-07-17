import json
from sanitation_tasks.stage4w_gate import assemble


def write_json(root, name, payload):
    (root / name).write_text(json.dumps(payload), encoding="utf-8")


def passing_fixture(root):
    write_json(root, "coverage_report.json", {
        "full_execution_success": True,
        "empirical_metrics": {"coverage_rate": 0.91},
        "localization_regression_during_coverage": {
            "pass_rmse_at_most_0_05m": True,
        },
        "swath_exclusion_intersection_count": 0,
        "collision_count": 0,
        "keepout_violation_sample_count": 0,
        "brush_state_violation_sample_count": 0,
        "brush_disabled_on_exit": True,
    })
    write_json(root, "dynamic_obstacle_report.json", {
        "dynamic_obstacle_valid_trials": 20,
        "collision_count": 0,
        "success": True,
    })
    write_json(root, "filter_report.json", {
        "keepout": {"violation_sample_count": 0},
        "speed_zone": {"speed_compliance_pass": True},
    })
    write_json(root, "safety_latency_report.json", {
        "trial_count": 30,
        "latency_sec": {"p95": 0.02},
    })
    (root / "dynamic_coverage_bag").mkdir()
    (root / "dynamic_coverage_bag" / "metadata.yaml").write_text("ok", encoding="utf-8")
    (root / "replay_coverage_state.txt").write_text("data", encoding="utf-8")


def test_stage4w_dynamic_aggregate_accepts_complete_evidence(tmp_path):
    passing_fixture(tmp_path)
    report = assemble(tmp_path, {"coverage": 0, "dynamic": 0})
    assert report["success"] is True
    assert all(report["gates"].values())
    assert report["efficiency"]["competition_efficiency_pass"] is False


def test_stage4w_dynamic_aggregate_rejects_invalid_trial(tmp_path):
    passing_fixture(tmp_path)
    write_json(tmp_path, "dynamic_obstacle_report.json", {
        "dynamic_obstacle_valid_trials": 19,
        "collision_count": 0,
        "success": False,
    })
    report = assemble(tmp_path)
    assert report["success"] is False
    assert report["gates"]["dynamic_obstacle_valid_trials_at_least_20"] is False
