import json
from pathlib import Path

import pytest
import yaml

from sanitation_active_cleaning.formal_contract import (
    FormalPlanningContractError,
    _split_metadata_valid,
    _validation_evidence_valid,
    audit_formal_planning,
    load_formal_contract,
)


PACKAGE = Path(__file__).parents[1]
REPOSITORY = PACKAGE.parents[2]
CONTRACT = PACKAGE / "config/formal_dual_mode_planning.yaml"


def test_formal_contract_freezes_dual_mode_scoring_and_truth_boundary():
    contract = load_formal_contract(CONTRACT)
    assert set(contract["modes"]) == {"traditional", "reinforcement_learning"}
    assert contract["scoring"]["observation_threshold"] == pytest.approx(0.95)
    assert (
        contract["scoring"]["distance_upper_bound"]
        == "same_episode_full_coverage_distance"
    )
    assert contract["scoring"]["return_distance_included"] is False
    assert (
        contract["product_inputs"]["discrete_targets_topic"]
        == "/perception/garbage/targets"
    )
    assert (
        contract["product_inputs"]["ground_dirt_masks_topic"]
        == "/perception/ground_dirt/masks"
    )


def test_contract_rejects_relaxed_coverage_requirement(tmp_path):
    contract = load_formal_contract(CONTRACT)
    contract["scoring"]["observation_threshold"] = 0.90
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(contract), encoding="utf-8")
    with pytest.raises(FormalPlanningContractError, match="observation_threshold"):
        load_formal_contract(path)


def test_map_splits_require_fixed_area_variable_aspect_and_disjoint_ids_and_seeds():
    report = {
        "map_splits": {
            "train": [
                {
                    "map_id": "a",
                    "area_m2": 20000,
                    "aspect_ratio": 2.0,
                    "mission_seeds": [1],
                }
            ],
            "validation": [
                {
                    "map_id": "b",
                    "area_m2": 20000,
                    "aspect_ratio": 1.5,
                    "mission_seeds": [2],
                }
            ],
            "test": [
                {
                    "map_id": "c",
                    "area_m2": 20000,
                    "aspect_ratio": 2.5,
                    "mission_seeds": [3],
                }
            ],
        }
    }
    assert _split_metadata_valid(report)
    report["map_splits"]["test"][0]["mission_seeds"] = [2]
    assert not _split_metadata_valid(report)


def test_validation_requires_product_inputs_thresholds_and_paired_distance():
    thresholds = {
        "observation_threshold": 0.95,
        "ground_clear_threshold": 0.95,
        "discrete_clear_threshold": 0.95,
    }
    report = {
        "formal_map_used": True,
        "product_perception_used": True,
        "truth_used_for_control": False,
        "return_distance_included": False,
        "time_energy_ignored": True,
        "policy_output": "global_reference_trajectory",
        "episodes": [
            {
                "observed_ratio": 0.96,
                "ground_clear_ratio": 0.96,
                "discrete_clear_ratio": 0.95,
                "task_distance": 100.0,
                "baseline_distance": 101.0,
            }
        ],
    }
    assert _validation_evidence_valid(report, thresholds)
    report["episodes"][0]["task_distance"] = 102.0
    assert not _validation_evidence_valid(report, thresholds)


def test_repository_audit_accepts_adapters_but_not_missing_formal_evidence(
    tmp_path,
):
    runtime = tmp_path / "runtime"
    evidence = tmp_path / "evidence"
    runtime.mkdir()
    evidence.mkdir()
    report = audit_formal_planning(
        CONTRACT,
        repository_root=REPOSITORY,
        runtime_root=runtime,
        evidence_root=evidence,
    )
    assert report["ready"] is False
    assert report["status"] == "blocked_fail_closed"
    assert report["checks"]["traditional_nav2_follow_path_executor_present"] is True
    assert report["checks"]["frozen_truth_free_rl_checkpoint_present"] is False
    assert report["checks"]["product_observation_bridge_present_and_truth_free"] is True
    assert report["checks"]["trajectory_executor_present_and_truth_free"] is True
    assert report["checks"]["frozen_policy_planner_present_and_truth_free"] is True
    assert "formal_rl_checkpoint_missing_or_invalid" in report["blockers"]
    assert report["ground_truth_input_used_for_control"] is False


def test_truth_contaminated_runtime_evidence_is_rejected(tmp_path):
    runtime = tmp_path / "runtime"
    evidence = tmp_path / "evidence"
    runtime.mkdir()
    evidence.mkdir()
    for name in ("occupancy.yaml", "keepout_mask.yaml", "mission_geometry.yaml"):
        (runtime / name).write_text("schema_version: 1\n", encoding="utf-8")
    (runtime / "materialization_contract.yaml").write_text(
        yaml.safe_dump({"evaluator_truth_used": True, "dirt_truth_used": False}),
        encoding="utf-8",
    )
    (evidence / "formal_perception_pc_audit.json").write_text(
        json.dumps({"ready": True, "ground_truth_input_used": True}),
        encoding="utf-8",
    )
    report = audit_formal_planning(
        CONTRACT,
        repository_root=REPOSITORY,
        runtime_root=runtime,
        evidence_root=evidence,
    )
    assert report["checks"]["formal_map_excludes_evaluator_and_dirt_truth"] is False
    assert report["checks"]["product_perception_preflight_ready"] is False
