import json

from auto02_acceptance import build_report
from auto02_replay_audit import relative_delta
from stage4w_static_aggregate import build_report as build_static_report
from stage4w_static_finalize import build_summary


def write_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_static_aggregate_exposes_planned_coverage(tmp_path):
    write_json(
        tmp_path / "seed_0" / "stage4w_static_summary.json",
        {
            "seed": 0,
            "static_gate_pass": True,
            "coverage": {
                "transit_to_start_success": True,
                "full_execution_success": True,
                "component_results": [{"success": True}],
                "component_count": 1,
                "empirical_metrics": {"coverage_rate": 0.91},
                "planned_metrics": {"coverage_rate": 0.98},
                "collision_count": 0,
                "keepout_violation_sample_count": 0,
                "brush_state_violation_sample_count": 0,
                "brush_disabled_on_exit": True,
                "localization_regression_during_coverage": {"rmse_m": 0.02},
            },
            "rosbag_replay": True,
        },
    )
    report = build_static_report(tmp_path, required_seeds=1)
    assert report["trials"][0]["planned_coverage_rate"] == 0.98


def passing_auto02_fixture(root):
    trials = []
    for seed in range(5):
        trials.append(
            {
                "seed": seed,
                "static_gate_pass": True,
                "full_execution_success": True,
                "component_terminal_success_count": 17,
                "component_terminal_total": 17,
                "empirical_coverage_rate": 0.93,
                "planned_coverage_rate": 0.986,
                "collision_count": 0,
                "keepout_violation_sample_count": 0,
                "brush_state_violation_sample_count": 0,
                "brush_disabled_on_exit": True,
                "localization": {"rmse_m": 0.03},
                "rosbag_replay": True,
            }
        )
        write_json(
            root
            / "static"
            / f"seed_{seed}"
            / "auto02_runtime_geometry_audit.json",
            {"runtime_geometry_gate_pass": True},
        )
        write_json(
            root / "static" / f"seed_{seed}" / "auto02_replay_audit.json",
            {"replay_gate_pass": True},
        )
        write_json(
            root / "cold_start" / f"trial_{seed}" / "cold_start_report.json",
            {
                "interfaces_ready": True,
                "nav2_parameter_services_ready_within_60_seconds": True,
                "nav2_parameter_services_ready_seconds": 25,
                "full_lifecycle_active": True,
                "lifecycle_active_count": 8,
                "lifecycle_node_count": 8,
                "tf_tree_complete": True,
            },
        )
    write_json(
        root / "static" / "stage4w_static_matrix_report.json",
        {"trials": trials},
    )
    interaction = {
        "moving_obstacle_trajectory": [
            {"target_map_xy": [0.0, 0.0]},
            {"target_map_xy": [0.0, 0.5]},
        ],
        "mission_progress_resumed": True,
    }
    safety_trials = [
        {"post_stop_command_output_zero": True} for _ in range(30)
    ]
    write_json(
        root / "dynamic" / "stage4w_dynamic_report.json",
        {
            "gates": {
                "dynamic_obstacle_gate_pass": True,
                "collision_count_zero": True,
                "full_execution_success": True,
                "keepout_violations_zero": True,
                "speed_zone_pass": True,
                "emergency_stop_30_trials": True,
                "emergency_stop_p95_at_most_1s": True,
                "brush_final_state_false": True,
                "complete_rosbag_replay": True,
            },
            "coverage": {"brush_disabled_on_exit": True},
            "dynamic_obstacles": {
                "dynamic_obstacle_valid_trials": 20,
                "minimum_separation_gate_pass": True,
                "minimum_observed_separation_m": 0.60,
                "configured_hard_minimum_separation_m": 0.12,
                "mission_progress_resumed_all": True,
                "trials": [interaction] * 20,
            },
            "filters": {
                "keepout": {"violation_sample_count": 0},
                "speed_zone": {
                    "mean_speed_m_s": {"inside": 0.29},
                    "allowed_maximum_mean_speed_m_s": 0.3135,
                    "allowed_tolerance_m_s": 0.03,
                },
                "success": True,
            },
            "emergency_stop": {
                "completed_trial_count": 30,
                "trial_count": 30,
                "emergency_stop_zeroed": True,
                "post_stop_command_output_zero": True,
                "latency_sec": {"p95": 0.2, "max": 0.4},
                "trials": safety_trials,
            },
        },
    )
    write_json(
        root / "dynamic" / "auto02_replay_audit.json",
        {"replay_gate_pass": True},
    )


def test_auto02_acceptance_accepts_complete_fixture(tmp_path):
    passing_auto02_fixture(tmp_path)
    report = build_report(tmp_path)
    assert report["machine_gate_pass"] is True
    assert all(report["checks"].values())


def test_auto02_acceptance_rejects_speed_above_tolerance(tmp_path):
    passing_auto02_fixture(tmp_path)
    path = tmp_path / "dynamic" / "stage4w_dynamic_report.json"
    report = json.loads(path.read_text(encoding="utf-8"))
    report["filters"]["speed_zone"]["mean_speed_m_s"]["inside"] = 0.32
    write_json(path, report)
    result = build_report(tmp_path)
    assert result["machine_gate_pass"] is False
    assert (
        result["checks"]["filter_speed_mean_within_limit_plus_0_03_m_s"]
        is False
    )


def test_replay_relative_delta():
    assert relative_delta(0.0303, 0.03) <= 0.01 + 1.0e-12
    assert relative_delta(0.031, 0.03) > 0.01


def test_replay_relative_delta_rejects_zero_baseline_drift():
    assert relative_delta(0.0, 0.0) == 0.0
    assert relative_delta(1.0e-6, 0.0) > 0.01


def test_static_finalize_requires_replay_gate(tmp_path):
    write_json(
        tmp_path / "coverage_report.json",
        {
            "success": True,
            "full_execution_success": True,
            "empirical_metrics": {"coverage_rate": 0.93},
            "collision_count": 0,
            "keepout_violation_sample_count": 0,
            "brush_state_violation_sample_count": 0,
            "brush_disabled_on_exit": True,
            "swath_exclusion_intersection_count": 0,
            "localization_regression_during_coverage": {
                "pass_rmse_at_most_0_05m": True,
            },
        },
    )
    write_json(
        tmp_path / "auto02_replay_audit.json",
        {"replay_gate_pass": False},
    )
    assert build_summary(tmp_path, 0, 0)["static_gate_pass"] is False
