from pathlib import Path

import pytest
import yaml

from prepare_formal_same_map_coverage import prepare


ROOT = Path(__file__).resolve().parents[1]
RUNNER = (ROOT / "scripts/run_formal_same_map_full_coverage_baseline.sh").read_text(
    encoding="utf-8"
)
SUPPORT = (ROOT / "scripts/formal_same_map_baseline_support.py").read_text(
    encoding="utf-8"
)


def test_prepared_probe_and_server_share_real_cleaning_width(tmp_path: Path) -> None:
    mission = tmp_path / "mission.yaml"
    mission.write_text(yaml.safe_dump({
        "mission_id": "formal-lifecycle-7",
        "outer_polygon": [[-100., -50.], [100., -50.], [100., 50.], [-100., 50.]],
        "keepout_polygons": [],
        "vehicle_start_pose_map": {"x_m": 0., "y_m": 0., "yaw_rad": 0.},
        "source_fixed_start_pose": [-98., 0., 0.],
        "truth_boundary": {"dirt_truth_used": False,
            "evaluator_truth_used": False,
            "world_geometry_used_for_product_map": False},
    }), encoding="utf-8")
    probe, server = prepare(
        mission, ROOT / "config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml"
    )
    params = server["coverage_server"]["ros__parameters"]
    assert probe["operation_width_m"] == pytest.approx(1.32)
    assert params["operation_width"] == probe["operation_width_m"]
    assert params["robot_width"] == pytest.approx(1.39)
    assert probe["planning_swath_spacing_m"] <= probe["operation_width_m"]
    assert probe["headland"]["width_m"] >= 1.69
    assert probe["evaluation_brush_dropout"]["enabled"] is False


def test_runner_is_one_hard_restart_fullcoverage_process_chain() -> None:
    assert RUNNER.count("formal_campus_map_lifecycle.launch.py") == 1
    assert "mission_mode:=cleaning" in RUNNER
    assert "cleaning_planner:=full_coverage" in RUNNER
    assert "start_coverage:=false" in RUNNER
    assert RUNNER.count("ros2 run opennav_coverage opennav_coverage") == 1
    assert RUNNER.count("ros2 run sanitation_coverage coverage_probe") == 1
    assert "hard_restart_record.json" in RUNNER
    assert "mapping_process_count_before_cleaning" in RUNNER
    assert "coverage_runtime.json" in RUNNER
    assert "run_formal_same_map_baseline.sh" in RUNNER
    assert "policy_checkpoint" not in RUNNER and "rl_dirt_priority" not in RUNNER
    assert "formal_source_bound_preflight.sh" in RUNNER
    assert "formal_source_bound_preflight" in RUNNER
    assert "formal_source_bound_verify_overlay" in RUNNER
    assert "FORMAL_VEHICLE_RUNTIME_WS" in RUNNER
    assert "FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST" in RUNNER
    assert "FORMAL_BASELINE_RUNTIME_OVERLAY" in RUNNER
    assert "one frozen runtime install" in RUNNER
    assert 'RUNTIME_BINDING="${OUTPUT}/runtime_gate_binding.json"' in RUNNER
    lifecycle_validator = RUNNER.index("validate_formal_map_lifecycle_runtime.py")
    lifecycle_output = RUNNER.index('--output "${OUTPUT}/lifecycle_acceptance.json"')
    assert '--runtime-binding "${RUNTIME_BINDING}"' in RUNNER[
        lifecycle_validator:lifecycle_output
    ]


def test_runner_uses_named_evaluator_pose_only_for_metrics() -> None:
    assert "/evaluation/formal_same_map/dynamic_pose" in RUNNER
    assert "/ground_truth/odom" in SUPPORT
    assert "transform.child_frame_id == self.entity" in SUPPORT
    assert "create_publisher(Odometry" in SUPPORT
    assert "create_publisher(Twist" not in SUPPORT
    assert "/formal_vehicle/simulation/command/emergency_stop" in SUPPORT
