from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_optimized_small_demo_profile_is_packaged_and_selected():
    setup_text = (ROOT / "starter_ws/src/sanitation_tasks/setup.py").read_text(encoding="utf-8")
    runner = (ROOT / "scripts/run_visual_demo.sh").read_text(encoding="utf-8")
    name = "competition_demo_area_skid_steer_optimized.yaml"
    assert name in setup_text
    assert name in runner
    assert "coverage_skid_steer_optimized.yaml" in runner
    assert "SMALL_FIELD_LIDAR_ONLY" in runner
    assert 'monitor["observation_sources"] = ["scan"]' in runner
    assert 'config["tzcup_demo_safety_profile"] = {' in runner
    assert '"ros__parameters": {' in runner
    assert "ros2 topic pub --times 5 --rate 5" in runner
    assert "--wait-matching-subscriptions 2" in runner
    assert "gazebo_cleaning_telemetry.json" in runner
    assert 'localization_fusion_mode="rtk_imu_wheel"' in runner
    assert 'enable_scan_refiner="false"' in runner


def test_coverage_success_includes_the_formal_localization_gate():
    probe = (
        ROOT / "starter_ws/src/sanitation_coverage/sanitation_coverage/coverage_probe.py"
    ).read_text(encoding="utf-8")
    assert '"localization_success": localization_pass' in probe
    assert "and localization_pass and not self.brush_enabled" in probe


def test_optimized_profile_has_bounded_repair_and_legacy_fallback():
    config = yaml.safe_load((
        ROOT / "starter_ws/src/sanitation_tasks/config/competition_demo_area_skid_steer_optimized.yaml"
    ).read_text(encoding="utf-8"))
    assert config["coverage_planner_profile"] == "SKID_STEER_OPTIMIZED"
    assert config["planning_swath_spacing_m"] in config["swath_spacing_candidates_m"]
    assert config["legacy_fallback_swath_spacing_m"] == 0.35
    assert config["execution_lateral_scale"] == 1.06
    assert config["execution_lateral_offset_m"] == 0.0264
    assert config["execution_calibration_source"] == (
        "coverage_optimizer_seeds118_119_120_123_robust_affine_fit"
    )
    assert config["coverage_repair_max_passes"] == 1
    assert config["repair_max_primary_length_ratio"] <= 0.10
    assert config["empirical_repeat_rate_threshold"] <= 0.20
    assert config["path_continuity_type"] == "DISCONTINUOUS"


def test_fields2cover_spacing_matches_selected_mission_spacing():
    mission = yaml.safe_load((
        ROOT / "starter_ws/src/sanitation_tasks/config/competition_demo_area_skid_steer_optimized.yaml"
    ).read_text(encoding="utf-8"))
    server = yaml.safe_load((
        ROOT / "starter_ws/src/sanitation_coverage/config/coverage_skid_steer_optimized.yaml"
    ).read_text(encoding="utf-8"))
    assert server["coverage_server"]["ros__parameters"]["operation_width"] == mission["planning_swath_spacing_m"]


def test_hybrid_localizer_bounds_absolute_sensor_disagreement():
    config = yaml.safe_load((
        ROOT / "starter_ws/src/sanitation_scan_refiner/config/stage4v_hybrid.yaml"
    ).read_text(encoding="utf-8"))["hybrid_global_fuser"]["ros__parameters"]
    source = (
        ROOT / "starter_ws/src/sanitation_scan_refiner/src/hybrid_global_fuser_node.cpp"
    ).read_text(encoding="utf-8")
    assert 0.0 < config["gnss_anchor_smoothing_alpha"] < 1.0
    assert config["maximum_refined_gnss_disagreement_m"] == 0.10
    assert config["minimum_refined_variance"] >= 0.0015
    assert "rtk_local_refined_innovation_rejected" in source
