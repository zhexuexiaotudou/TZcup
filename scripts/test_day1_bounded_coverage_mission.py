from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SUMMARIZER_PATH = ROOT / "scripts/summarize_day1_bounded_coverage_run.py"
BRIDGE_PATH = ROOT / "scripts/day1_bounded_coverage_cleaning_bridge.py"
CONFIG_PATH = ROOT / "config/day1_bounded_coverage_mission.yaml"


def _module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _coverage() -> dict:
    return {
        "schema_version": 2,
        "mission_id": "fixture",
        "success": True,
        "full_execution_success": True,
        "coverage_quality_success": True,
        "safety_success": True,
        "localization_success": True,
        "ground_truth_used_for_control": False,
        "brush_disabled_on_exit": True,
        "planned_metrics": {"coverage_rate": 1.0},
        "localization_regression_during_coverage": {"rmse_m": 0.03},
        "empirical_metrics": {
            "coverage_rate": 0.985,
            "covered_area_m2": 31.0,
            "cleanable_area_m2": 31.5,
            "actual_path_length_m": 60.0,
            "actual_duration_sec": 120.0,
            "gross_efficiency_m2_h": 930.0,
            "net_efficiency_m2_h": 930.0,
            "brush_enabled_distance_m": 55.0,
            "brush_state_transitions": 10,
            "metric_basis": "fixture",
        },
    }


def _cleaning() -> dict:
    return {
        "permit_observed": True,
        "lift_requested": True,
        "brush_disabled_on_exit": True,
        "dirt_system_disabled_on_exit": True,
        "all_three_ready_sample_count": 10,
        "maximum_roller_velocity_rad_s": 12.0,
        "cleaned_cell_delta": 100,
        "cleaned_area_delta_m2": 1.0,
        "initial": {"cell_count": 1800, "cleaned_cell_count": 0},
        "terminal": {"cell_count": 1800, "cleaned_cell_count": 100, "cleaned_fraction": 0.055},
    }


def test_bounded_config_is_fixed_and_truth_safe() -> None:
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    assert config["outer_polygon"] == [
        [0.0, -3.0],
        [8.0, -3.0],
        [8.0, 3.0],
        [0.0, 3.0],
    ]
    assert config["claim_boundary"]["full_20000_m2_mapping_claim"] is False
    assert config["claim_boundary"]["official_recognition_claim"] is False
    assert config["claim_boundary"]["ground_truth_used_for_control"] is False


def test_bridge_has_no_motion_or_truth_interfaces() -> None:
    source = BRIDGE_PATH.read_text(encoding="utf-8")
    assert "self.publishers =" not in source
    for forbidden in (
        "cmd_vel",
        '"/ground_truth',
        "NavigateToPose",
        "FollowPath",
    ):
        assert forbidden not in source


def test_summary_accepts_complete_bounded_mission() -> None:
    module = _module(SUMMARIZER_PATH, "bounded_summary")
    result = module.summarize(_coverage(), _cleaning(), 123)
    assert result["valid_bounded_mission"] is True
    assert result["terminal_state"] == "COMPLETED"
    assert result["cleaning"]["cleaned_cell_count"] == 100
    assert result["coverage"]["coverage_percent"] == 98.5
    assert result["efficiency_boundary"][
        "official_competition_efficiency_pass"
    ] is False


def test_summary_rejects_missing_cleaning_effect() -> None:
    module = _module(SUMMARIZER_PATH, "bounded_summary_failure")
    cleaning = _cleaning()
    cleaning["cleaned_cell_delta"] = 0
    cleaning["cleaned_area_delta_m2"] = 0.0
    result = module.summarize(_coverage(), cleaning, 123)
    assert result["valid_bounded_mission"] is False
    assert "no_ground_dirt_cells_cleared" in result["blockers"]
