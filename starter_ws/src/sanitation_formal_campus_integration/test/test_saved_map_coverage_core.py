import json
import hashlib
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sanitation_formal_campus_integration.saved_map_coverage_core import (
    DRY_CLEANING_SPEED_PROFILE,
    FORMAL_MAX_LINEAR_SPEED_MPS,
    FORMAL_OPERATION_WIDTH_M,
    MAPPING_SAFE_SPEED_PROFILE,
    ProductCoverageTelemetry,
    SavedMapCoverageError,
    coverage_execution_passed,
    load_formal_operation_speed_profile,
    load_product_mission_geometry,
    load_saved_map_home_pose,
    validate_execution_parameters,
)


def _mission(path: Path) -> Path:
    path = path.with_name("mission_geometry.yaml")
    path.parent.mkdir(parents=True, exist_ok=True)
    occupancy = path.with_name("occupancy.yaml")
    occupancy_image = path.with_name("occupancy.pgm")
    occupancy_image.write_bytes(b"P5\n1 1\n255\n\xff")
    occupancy.write_text("image: occupancy.pgm\n", encoding="utf-8")
    free = path.with_name("coverage_free_space.pgm")
    free.write_bytes(b"P5\n1 1\n255\n\xff")
    geometry = path.with_name("coverage_geometry.yaml")
    geometry.write_text(yaml.safe_dump({
        "source": "saved_slam_occupancy_only",
        "occupancy_map": occupancy.name,
        "occupancy_image": occupancy_image.name,
        "occupancy_map_sha256": hashlib.sha256(occupancy.read_bytes()).hexdigest(),
        "occupancy_image_sha256": hashlib.sha256(occupancy_image.read_bytes()).hexdigest(),
        "free_space_map": free.name,
        "free_space_map_sha256": hashlib.sha256(free.read_bytes()).hexdigest(),
        "world_truth_used_for_product_map": False,
        "planning_polygons": [[[0, 0], [1, 0], [1, 1], [0, 1]]],
        "planning_outer_polygon": [[0, 0], [1, 0], [1, 1], [0, 1]],
        "planning_hole_polygons": [],
        "keepout_polygons": [],
        "resolution_m": 0.25,
        "origin": [0, 0, 0],
        "reachable_cleanable_cells": 1,
        "obstacle_inflation_m": 1.70,
        "planning_clearance_m": 1.70,
        "planning_clearance_preapplied": True,
    }), encoding="utf-8")
    path.write_text(yaml.safe_dump({
        "outer_polygon": [[0, 0], [200, 0], [200, 100], [0, 100]],
        "vehicle_start_pose_map": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
        "keepout_polygons": [],
        "headland": {"enabled": True, "width_m": 1.70},
        "saved_occupancy_coverage": {
            "source": "saved_slam_occupancy_only",
            "geometry": geometry.name,
            "free_space_map": free.name,
            "sha256": hashlib.sha256(geometry.read_bytes()).hexdigest(),
            "planning_clearance_preapplied": True,
        },
        "truth_boundary": {
            "world_geometry_used_for_product_map": False,
            "evaluator_truth_used": False,
            "dirt_truth_used": False,
        },
    }), encoding="utf-8")
    support = {
        "materialization_contract.yaml": "{}\n",
        "geofence_keepout.yaml": "{}\n",
        "geofence_keepout.pgm": "P5\n1 1\n255\n\x00",
        "neutral_speed.yaml": "{}\n",
        "neutral_speed.pgm": "P5\n1 1\n255\n\x00",
    }
    for name, value in support.items():
        (path.parent / name).write_bytes(value.encode("latin-1"))
    sealed = {
        name: hashlib.sha256((path.parent / name).read_bytes()).hexdigest()
        for name in (
            "occupancy.yaml", "occupancy.pgm", "mission_geometry.yaml",
            "materialization_contract.yaml", "geofence_keepout.yaml",
            "geofence_keepout.pgm", "neutral_speed.yaml", "neutral_speed.pgm",
            "coverage_geometry.yaml", "coverage_free_space.pgm",
        )
    }
    (path.parent / "map_lifecycle_manifest.json").write_text(
        json.dumps({"sha256": sealed}), encoding="utf-8"
    )
    return path


def _reseal(path: Path, name: str) -> None:
    manifest_path = path.with_name("map_lifecycle_manifest.json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    target = path.with_name(name)
    manifest["sha256"][name] = hashlib.sha256(target.read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")


def test_formal_width_and_speed_are_exact_single_source(tmp_path):
    profile_file = tmp_path / "profiles.yaml"
    profile_file.write_text(yaml.safe_dump({
        "profiles": {
            MAPPING_SAFE_SPEED_PROFILE: {"maximum_linear_speed_m_s": 0.45},
            DRY_CLEANING_SPEED_PROFILE: {
                "target_linear_speed_m_s": 1.0,
                "enabled_for_formal_runtime": True,
                "competition_efficiency_eligible": False,
            },
            "wet_puddle_recovery": {},
        },
    }), encoding="utf-8")
    mapping = load_formal_operation_speed_profile(profile_file, MAPPING_SAFE_SPEED_PROFILE)
    dry = load_formal_operation_speed_profile(profile_file, DRY_CLEANING_SPEED_PROFILE)
    validate_execution_parameters(1.32, 0.45, mapping)
    validate_execution_parameters(1.32, 1.0, dry)
    for width, speed, profile in ((0.52, 0.45, mapping), (1.32, 0.65, dry)):
        with pytest.raises(SavedMapCoverageError):
            validate_execution_parameters(width, speed, profile)


def test_public_mission_requires_20000_m2_and_truth_isolation(tmp_path):
    path = _mission(tmp_path / "mission.yaml")
    assert len(load_product_mission_geometry(path).outer_polygon) == 4
    assert load_saved_map_home_pose(path) == (0.0, 0.0, 0.0)
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    value["truth_boundary"]["evaluator_truth_used"] = True
    path.write_text(yaml.safe_dump(value), encoding="utf-8")
    _reseal(path, "mission_geometry.yaml")
    with pytest.raises(SavedMapCoverageError, match="truth isolation"):
        load_product_mission_geometry(path)


def test_saved_coverage_geometry_hash_tampering_is_rejected(tmp_path):
    path = _mission(tmp_path / "mission.yaml")
    path.with_name("coverage_geometry.yaml").write_text("source: tampered\n", encoding="utf-8")
    with pytest.raises(SavedMapCoverageError, match="coverage geometry"):
        load_product_mission_geometry(path)


def test_saved_coverage_free_space_hash_tampering_is_rejected(tmp_path):
    path = _mission(tmp_path / "mission.yaml")
    path.with_name("coverage_free_space.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    with pytest.raises(SavedMapCoverageError, match="coverage free-space map"):
        load_product_mission_geometry(path)


def test_saved_coverage_parses_and_hashes_each_consumed_file_from_one_snapshot(
    tmp_path, monkeypatch
):
    path = _mission(tmp_path / "mission_geometry.yaml")
    original = Path.read_bytes
    reads: dict[str, int] = {}

    def counted(candidate: Path) -> bytes:
        reads[candidate.name] = reads.get(candidate.name, 0) + 1
        return original(candidate)

    monkeypatch.setattr(Path, "read_bytes", counted)
    load_product_mission_geometry(path)
    assert reads == {
        "map_lifecycle_manifest.json": 1,
        "mission_geometry.yaml": 1,
        "coverage_geometry.yaml": 1,
        "coverage_free_space.pgm": 1,
    }


def test_saved_coverage_rejects_traversal_before_reading_geometry(tmp_path, monkeypatch):
    path = _mission(tmp_path / "mission_geometry.yaml")
    mission = yaml.safe_load(path.read_bytes())
    mission["saved_occupancy_coverage"]["geometry"] = "../coverage_geometry.yaml"
    path.write_text(yaml.safe_dump(mission), encoding="utf-8")
    _reseal(path, path.name)
    original = Path.read_bytes
    outside_reads = 0

    def counted(candidate: Path) -> bytes:
        nonlocal outside_reads
        if candidate.name == "coverage_geometry.yaml":
            outside_reads += 1
        return original(candidate)

    monkeypatch.setattr(Path, "read_bytes", counted)
    with pytest.raises(SavedMapCoverageError, match="mission geometry is missing or invalid"):
        load_product_mission_geometry(path)
    assert outside_reads == 0


def test_saved_coverage_rejects_symlink_even_when_bytes_match_seal(tmp_path):
    path = _mission(tmp_path / "mission_geometry.yaml")
    geometry = path.with_name("coverage_geometry.yaml")
    target = tmp_path / "outside_geometry.yaml"
    target.write_bytes(geometry.read_bytes())
    geometry.unlink()
    try:
        geometry.symlink_to(target)
    except OSError:
        pytest.skip("file symlinks are unavailable on this Windows host")
    with pytest.raises(SavedMapCoverageError, match="coverage geometry"):
        load_product_mission_geometry(path)


def test_product_telemetry_integrates_distance_brush_and_estimated_sweep():
    telemetry = ProductCoverageTelemetry(
        polygon=((0, 0), (200, 0), (200, 100), (0, 100)),
        raster_resolution_m=0.20,
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
    assert report["coverage_raster_resolution_m"] == pytest.approx(0.20)
    assert report["simulator_truth_used"] is False


def test_execution_pass_requires_real_terminal_and_all_swaths():
    report = {
        "success": True,
        "terminal_state": "COMPLETED",
        "ground_truth_used_for_control": False,
        "operation_width_m": FORMAL_OPERATION_WIDTH_M,
        "maximum_linear_speed_mps": FORMAL_MAX_LINEAR_SPEED_MPS,
        "operation_speed_profile": MAPPING_SAFE_SPEED_PROFILE,
        "planned_swath_count": 3,
        "completed_swath_count": 3,
        "coverage_geometry_sha256": "0" * 64,
        "cleanable_area_m2": 1.0,
        "return_home": {"success": True, "goal_frame_id": "map", "final_cmd_vel_zero": True, "brush_control_released": True, "coverage_control_released": True},
    }
    assert coverage_execution_passed(report)
    for field, value in (
        ("terminal_state", "READY"),
        ("completed_swath_count", 2),
        ("operation_width_m", 0.52),
        ("maximum_linear_speed_mps", 0.65),
        ("operation_speed_profile", "wet_puddle_recovery"),
        ("coverage_geometry_sha256", "tampered"),
    ):
        candidate = json.loads(json.dumps(report))
        candidate[field] = value
        assert not coverage_execution_passed(candidate)
