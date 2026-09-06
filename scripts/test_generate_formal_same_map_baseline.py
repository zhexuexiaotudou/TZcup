import argparse
import hashlib
import json
import os
from pathlib import Path
import time

import pytest
import yaml

from generate_formal_same_map_baseline import (
    BaselineError,
    PASS_STATUS,
    generate,
    validate,
)


def _json(path: Path, value: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True), encoding="utf-8")
    return path


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _fixture(tmp_path: Path) -> argparse.Namespace:
    episode = _json(tmp_path / "episode.json", {
        "episode_id": "episode-7", "map_id": "map-7", "profile": "formal",
        "field": {"width_m": 200., "height_m": 100., "area_m2": 20000.},
        "vehicle_start_pose_map": {"x_m": -98., "y_m": 3., "yaw_rad": .1},
    })
    snapshot = _json(tmp_path / "snapshot.json", {
        "source_inventory_sha256": "a" * 64,
        "outputs": {"reports/engineering/formal_competition_vehicle.urdf": {
            "sha256": "b" * 64,
        }},
    })
    started = time.time_ns() - 1_000_000_000
    session = _json(tmp_path / "session.json", {
        "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        "started_epoch_ns": started,
        "snapshot": {"snapshot_manifest_sha256": _sha(snapshot),
            "source_inventory_sha256": "a" * 64,
            "expanded_urdf_sha256": "b" * 64},
    })
    map_root = tmp_path / "map"
    map_root.mkdir()
    (map_root / "occupancy.yaml").write_text("image: occupancy.pgm\n", encoding="utf-8")
    mission = {
        "mission_id": "formal-lifecycle-episode-7",
        "source_fixed_start_pose": [-98., 3., .1],
        "vehicle_start_pose_map": {"x_m": 0., "y_m": 0., "yaw_rad": 0.},
        "truth_boundary": {"world_geometry_used_for_product_map": False,
            "evaluator_truth_used": False, "dirt_truth_used": False},
    }
    (map_root / "mission_geometry.yaml").write_text(
        yaml.safe_dump(mission), encoding="utf-8"
    )
    _json(map_root / "map_lifecycle_manifest.json", {
        "status": "ready_for_localization_cleaning", "episode_id": "episode-7",
        "map_id": "map-7", "observed_fraction": .97,
        "fixed_start_verified": True, "mapping_ignored_dirt": True,
        "world_truth_used_for_control": False,
        "sha256": {name: _sha(map_root / name)
            for name in ("occupancy.yaml", "mission_geometry.yaml")},
    })
    mapping = _json(tmp_path / "mapping.json", {
        "passed": True, "truth_used_for_control": False,
        "robot_description_sha256": "b" * 64,
    })
    cleaning = _json(tmp_path / "cleaning.json", {
        "passed": True, "truth_used_for_control": False,
        "robot_description_sha256": "b" * 64, "localization_backend": "amcl",
        "saved_map_sha256_verified": True, "hard_restart_verified": True,
        "cleaning_stack_ready": True, "coverage_server_ready": True,
        "world_derived_map_fallback": False,
        "hard_restart_record": {"mapping_stopped_before_cleaning": True,
            "mapping_process_count_before_cleaning": 0,
            "restart_type": "separate_process_hard_restart"},
    })
    lifecycle = _json(tmp_path / "lifecycle.json", {
        "status": "FORMAL_FIRST_MAP_THEN_SAVED_MAP_CLEANING_PASSED",
        "passed": True, "truth_used_for_control": False,
        "checks": {"map": True, "mapping": True, "cleaning": True},
    })
    coverage = _json(tmp_path / "coverage.json", {
        "schema_version": 2, "mission_id": mission["mission_id"],
        "planner": "OpenNav Coverage + Fields2Cover", "success": True,
        "planning_success": True, "full_execution_success": True,
        "coverage_quality_success": True, "safety_success": True,
        "localization_success": True,
        "competition_efficiency_pass": True,
        "evaluation_injection": {"ground_truth_used_for_control": False},
        "planned_metrics": {"path_length_m": 1100.},
        "empirical_metrics": {
            "actual_path_length_m": 1200.,
            "covered_area_m2": 20000.,
            "actual_duration_sec": 20000.,
            "net_efficiency_m2_h": 3600.,
        },
    })
    return argparse.Namespace(
        episode_manifest=episode, map_root=map_root, mapping_runtime=mapping,
        cleaning_runtime=cleaning, lifecycle_acceptance=lifecycle,
        coverage_runtime=coverage, session=session, snapshot=snapshot,
        output=tmp_path / "baseline.json",
    )


def _mutate(path: Path, key: str, value) -> None:
    row = json.loads(path.read_text(encoding="utf-8"))
    row[key] = value
    path.write_text(json.dumps(row), encoding="utf-8")


def _mutate_efficiency(path: Path, key: str, value) -> None:
    row = json.loads(path.read_text(encoding="utf-8"))
    row["empirical_metrics"][key] = value
    path.write_text(json.dumps(row), encoding="utf-8")


def test_generate_and_revalidate_complete_same_map_baseline(tmp_path: Path) -> None:
    args = _fixture(tmp_path)
    report = generate(args)
    assert report["status"] == PASS_STATUS
    assert report["successful_distance_m"] == 1200.
    assert report["planner_comparison"]["candidate_planner"] == "q_learning"
    assert report["return_distance_included"] is False
    assert report["competition_efficiency"] == {
        "threshold_m2_h": 3500.0,
        "covered_area_m2": 20000.0,
        "actual_duration_sec": 20000.0,
        "measured_net_efficiency_m2_h": 3600.0,
        "recomputed_net_efficiency_m2_h": 3600.0,
        "return_distance_included": False,
        "passed": True,
    }
    assert validate(args.output, args.session, args.snapshot) == report


def test_rejects_one_mps_readback_without_isolated_dry_state(tmp_path: Path) -> None:
    args = _fixture(tmp_path)
    runtime = _json(tmp_path / "runtime_binding.json", {
        "status": "FORMAL_RUNTIME_GATE_BOUND",
        "acceptance_session_binding": {
            "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        },
        "runtime_closure_binding": {"status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"},
    })
    readback = _json(tmp_path / "safety.json", {
        "schema_version": 2,
        "capture_status": "PASSED",
        "runtime_gate_binding_sha256": _sha(runtime),
        "producer_identity": {
            "node_name": "whole_vehicle_safety_manager",
            "topic": "/safety/status_json",
        },
        "status_capture": {
            "returncode": 0,
            "command": ["ros2", "topic", "echo", "--once", "--field", "data", "/safety/status_json"],
            "stdout": json.dumps({
                "effective_max_linear_velocity_mps": 1.0,
                "operation_speed_profile": "dry_cleaning_competition_candidate",
                "speed_qualification_state": "none",
            }),
        },
        "producer_capture": {
            "returncode": 0,
            "command": ["ros2", "topic", "info", "/safety/status_json", "--verbose"],
        },
        "effective_max_linear_velocity_mps": 1.0,
        "operation_speed_profile": "dry_cleaning_competition_candidate",
        "speed_qualification_state": "none",
    })
    args.safety_manager_readback = readback
    args.runtime_binding = runtime
    args.expected_safety_cap = 1.0
    with pytest.raises(BaselineError, match="isolated dry-only"):
        generate(args)


def test_rejects_safety_readback_without_collector_producer_receipt(tmp_path: Path) -> None:
    args = _fixture(tmp_path)
    runtime = _json(tmp_path / "runtime_binding.json", {
        "status": "FORMAL_RUNTIME_GATE_BOUND",
        "acceptance_session_binding": {
            "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        },
        "runtime_closure_binding": {"status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"},
    })
    status = {
        "effective_max_linear_velocity_mps": 1.0,
        "operation_speed_profile": "dry_cleaning_competition_candidate",
        "speed_qualification_state": "isolated_same_map_dry_coverage",
    }
    readback = _json(tmp_path / "safety.json", {
        "schema_version": 2,
        "capture_status": "PASSED",
        "runtime_gate_binding_sha256": _sha(runtime),
        "producer_identity": {
            "node_name": "whole_vehicle_safety_manager",
            "topic": "/safety/status_json",
        },
        "status_capture": {
            "returncode": 0,
            "command": ["ros2", "topic", "echo", "/safety/status_json"],
            "stdout": json.dumps(status),
        },
        "producer_capture": {
            "returncode": 0,
            "command": ["ros2", "topic", "info", "/safety/status_json", "--verbose"],
        },
        **status,
    })
    args.safety_manager_readback = readback
    args.runtime_binding = runtime
    args.expected_safety_cap = 1.0
    with pytest.raises(BaselineError, match="producer receipt"):
        generate(args)


def test_rejects_mapping_that_did_not_ignore_dirt(tmp_path: Path) -> None:
    args = _fixture(tmp_path)
    manifest = args.map_root / "map_lifecycle_manifest.json"
    _mutate(manifest, "mapping_ignored_dirt", False)
    with pytest.raises(BaselineError, match="mapping_ignored_dirt"):
        generate(args)


def test_rejects_non_hard_restart_cleaning(tmp_path: Path) -> None:
    args = _fixture(tmp_path)
    _mutate(args.cleaning_runtime, "hard_restart_verified", False)
    with pytest.raises(BaselineError, match="hard_restart_verified"):
        generate(args)


def test_rejects_wrong_fixed_start_or_snapshot(tmp_path: Path) -> None:
    args = _fixture(tmp_path)
    mission_path = args.map_root / "mission_geometry.yaml"
    mission = yaml.safe_load(mission_path.read_text(encoding="utf-8"))
    mission["source_fixed_start_pose"][0] = -97.
    mission_path.write_text(yaml.safe_dump(mission), encoding="utf-8")
    manifest = json.loads((args.map_root / "map_lifecycle_manifest.json").read_text())
    manifest["sha256"]["mission_geometry.yaml"] = _sha(mission_path)
    _json(args.map_root / "map_lifecycle_manifest.json", manifest)
    with pytest.raises(BaselineError, match="fixed start"):
        generate(args)

    args = _fixture(tmp_path / "snapshot-case")
    session = json.loads(args.session.read_text())
    session["snapshot"]["expanded_urdf_sha256"] = "d" * 64
    _json(args.session, session)
    with pytest.raises(BaselineError, match="snapshot identity"):
        generate(args)


def test_rejects_failed_coverage_and_post_generation_tamper(tmp_path: Path) -> None:
    args = _fixture(tmp_path / "failed")
    _mutate(args.coverage_runtime, "full_execution_success", False)
    with pytest.raises(BaselineError, match="full_execution_success"):
        generate(args)

    args = _fixture(tmp_path / "tamper")
    generate(args)
    coverage = json.loads(args.coverage_runtime.read_text())
    coverage["empirical_metrics"]["actual_path_length_m"] = 1.
    _json(args.coverage_runtime, coverage)
    with pytest.raises(BaselineError, match="changed after generation"):
        validate(args.output, args.session, args.snapshot)


def test_rejects_runtime_evidence_that_predates_session(tmp_path: Path) -> None:
    args = _fixture(tmp_path)
    started = json.loads(args.session.read_text())["started_epoch_ns"]
    stale = started - 1
    os.utime(args.coverage_runtime, ns=(stale, stale))
    with pytest.raises(BaselineError, match="predates formal session"):
        generate(args)


def test_rejects_competition_efficiency_below_threshold(tmp_path: Path) -> None:
    args = _fixture(tmp_path)
    _mutate_efficiency(args.coverage_runtime, "net_efficiency_m2_h", 3499.0)
    _mutate_efficiency(args.coverage_runtime, "actual_duration_sec", 20000.0 / 3499.0 * 3600.0)
    with pytest.raises(BaselineError, match="below 3500"):
        generate(args)


@pytest.mark.parametrize(
    ("key", "value", "message"),
    [
        ("covered_area_m2", True, "numeric JSON scalar"),
        ("covered_area_m2", "20000", "numeric JSON scalar"),
        ("actual_duration_sec", float("nan"), "must be finite"),
        ("net_efficiency_m2_h", float("inf"), "must be finite"),
    ],
)
def test_rejects_non_type_strict_or_nonfinite_competition_metrics(
    tmp_path: Path, key: str, value, message: str
) -> None:
    args = _fixture(tmp_path)
    _mutate_efficiency(args.coverage_runtime, key, value)
    with pytest.raises(BaselineError, match=message):
        generate(args)


def test_rejects_competition_efficiency_formula_mismatch_or_false_flag(tmp_path: Path) -> None:
    args = _fixture(tmp_path / "formula")
    _mutate_efficiency(args.coverage_runtime, "net_efficiency_m2_h", 3601.0)
    with pytest.raises(BaselineError, match="differs from area/duration"):
        generate(args)

    args = _fixture(tmp_path / "flag")
    _mutate(args.coverage_runtime, "competition_efficiency_pass", False)
    with pytest.raises(BaselineError, match="explicitly true"):
        generate(args)
