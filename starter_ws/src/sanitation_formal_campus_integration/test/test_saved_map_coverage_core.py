import json
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sanitation_formal_campus_integration.saved_map_coverage_core import (
    FORMAL_MAX_LINEAR_SPEED_MPS,
    FORMAL_OPERATION_WIDTH_M,
    ProductCoverageTelemetry,
    SavedMapCoverageError,
    coverage_execution_passed,
    load_product_mission_geometry,
    validate_execution_parameters,
)


def _mission(path: Path) -> Path:
    path.write_text(yaml.safe_dump({
        "outer_polygon": [[0, 0], [200, 0], [200, 100], [0, 100]],
        "truth_boundary": {
            "world_geometry_used_for_product_map": False,
            "evaluator_truth_used": False,
            "dirt_truth_used": False,
        },
    }), encoding="utf-8")
    return path


def test_formal_width_and_speed_are_exact_single_source():
    validate_execution_parameters(1.32, 0.45)
    for width, speed in ((0.52, 0.45), (1.32, 0.65)):
        with pytest.raises(SavedMapCoverageError):
            validate_execution_parameters(width, speed)


def test_public_mission_requires_20000_m2_and_truth_isolation(tmp_path):
    path = _mission(tmp_path / "mission.yaml")
    assert len(load_product_mission_geometry(path)) == 4
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    value["truth_boundary"]["evaluator_truth_used"] = True
    path.write_text(yaml.safe_dump(value), encoding="utf-8")
    with pytest.raises(SavedMapCoverageError, match="truth isolation"):
        load_product_mission_geometry(path)


def test_product_telemetry_integrates_distance_brush_and_estimated_sweep():
    telemetry = ProductCoverageTelemetry(
        polygon=((0, 0), (200, 0), (200, 100), (0, 100))
    )
    telemetry.observe_odom(1.0, 1.0)
    telemetry.observe_map_pose(1.0, 1.0)
    telemetry.set_brush(True)
    telemetry.observe_odom(2.0, 1.0)
    telemetry.observe_map_pose(2.0, 1.0)
    telemetry.set_brush(False)
    telemetry.observe_odom(3.0, 1.0)
    telemetry.observe_map_pose(3.0, 1.0)
    report = telemetry.report()
    assert report["trajectory_total_distance_m"] == pytest.approx(2.0)
    assert report["brush_enabled_distance_m"] == pytest.approx(1.0)
    assert report["brush_state_transitions"] == 2
    assert report["brush_state_sample_count"] == 2
    assert report["brush_state_source"] == "/brush_enabled_product_runtime"
    assert report["brush_disabled_on_exit"] is True
    assert report["estimated_covered_cells"] > 0
    assert 0.0 < report["estimated_coverage_fraction"] < 1.0
    assert report["simulator_truth_used"] is False


def test_execution_pass_requires_real_terminal_and_all_swaths():
    report = {
        "success": True,
        "terminal_state": "COMPLETED",
        "ground_truth_used_for_control": False,
        "operation_width_m": FORMAL_OPERATION_WIDTH_M,
        "maximum_linear_speed_mps": FORMAL_MAX_LINEAR_SPEED_MPS,
        "planned_swath_count": 3,
        "completed_swath_count": 3,
    }
    assert coverage_execution_passed(report)
    for field, value in (
        ("terminal_state", "READY"),
        ("completed_swath_count", 2),
        ("operation_width_m", 0.52),
        ("maximum_linear_speed_mps", 0.65),
    ):
        candidate = json.loads(json.dumps(report))
        candidate[field] = value
        assert not coverage_execution_passed(candidate)
