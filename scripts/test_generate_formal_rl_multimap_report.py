from __future__ import annotations

import json
from pathlib import Path

from generate_formal_rl_multimap_report import PASSED_STATUS, generate


FULL_COUNTS = {"train": 32, "validation": 8, "hidden": 12}
TASK_COUNTS = {"train": 6400, "validation": 800, "hidden": 1200}
POLICY_SEEDS = [7, 17, 29, 43, 61]


def _multimap_contract() -> dict:
    return {
        "required_map_counts": FULL_COUNTS,
        "actual_distinct_map_counts": FULL_COUNTS,
        "map_indices": {
            name: list(range(count)) for name, count in FULL_COUNTS.items()
        },
        "full_map_coverage": True,
        "smoke_subset_accepted_as_generalization": False,
    }


def _frozen_manifest() -> dict:
    return {
        "required_map_counts": FULL_COUNTS,
        "selections": {
            name: [
                f"{map_index}:{mission_index}"
                for map_index in range(count)
                for mission_index in range({"train": 200, "validation": 100, "hidden": 100}[name])
            ]
            for name, count in FULL_COUNTS.items()
        },
        "all_frozen_missions_per_declared_map": True,
        "smoke_subset_accepted_as_generalization": False,
    }


def _map_splits() -> dict:
    return {
        report_name: [
            {
                "map_id": f"{map_prefix}-map-{index:03d}",
                "map_index": index,
                "area_m2": 20000,
                "aspect_ratio": 2.0,
                "mission_seeds": [1000 + index],
            }
            for index in range(count)
        ]
        for name, map_prefix, report_name, count in (
            ("train", "train", "train", 32),
            ("validation", "val", "validation", 8),
            ("hidden", "hidden", "test", 12),
        )
    }


def _hybrid_episodes(observed_ratio: float) -> list[dict]:
    rows = []
    for split, count in (("validation", 8), ("hidden", 12)):
        rows.extend(
            {
                "split": split,
                "map_id": f"{'val' if split == 'validation' else split}-map-{index:03d}",
                "formal_success": True,
                "observed_ratio": observed_ratio,
                "ground_clear_ratio": 1.0,
                "discrete_clear_ratio": 1.0,
                "collisions": 0,
                "boundary_violations": 0,
                "invalid_actions": 0,
                "task_distance": 80.0,
                "baseline_distance": 100.0,
                "path_ratio_to_full_coverage": 0.8,
            }
            for index in range(count)
        )
    return rows


def _write(root: Path, name: str, value: dict) -> None:
    root.mkdir(parents=True, exist_ok=True)
    (root / name).write_text(json.dumps(value), encoding="utf-8")


def _evidence(tmp_path: Path, *, observed_ratio: float = 0.96) -> Path:
    root = tmp_path / "evidence"
    contract = _multimap_contract()
    budget_execution = {
        "formal_budget_claim": True,
        "contract": {"contract_id": "tzcup_formal_rl_budget_v1"},
        "task_counts": TASK_COUNTS,
        "training_rollout_count": 6400,
        "max_steps_per_episode": 400,
        "max_steps_is_episode_truncation_guard_not_task_or_episode_budget": True,
        "policy_seeds": POLICY_SEEDS,
        "configuration_freeze": {
            "frozen_before_hidden": True,
            "selection_source": "validation_only_before_hidden",
            "policy_seeds": POLICY_SEEDS,
        },
        "policy_seed_runs": [
            {"policy_seed": seed, "training_rollout_count": 6400,
             "validation_episode_count": 800, "hidden_episode_count": 1200}
            for seed in POLICY_SEEDS
        ],
    }
    _write(
        root,
        "training_report.json",
        {
            "episodes": [],
            "map_splits": _map_splits(),
            "formal_multimap_contract": contract,
            "frozen_split_manifest": _frozen_manifest(),
            "formal_budget_execution": budget_execution,
        },
    )
    _write(root, "baseline_report.json", {"episodes": []})
    _write(
        root,
        "q_policy.json",
        {"truth_access_used": False, "formal_multimap_contract": contract},
    )
    _write(
        root,
        "validation_report.json",
        {
            "hidden_gate_passed": True,
            "truth_used_for_control": False,
            "return_distance_included": False,
            "policy_output": "global_reference_trajectory",
            "gate_policy": "q_learning_with_systematic_coverage_backstop",
            "formal_multimap_contract": contract,
            "pure_q_episodes": [],
            "hybrid_episodes": _hybrid_episodes(observed_ratio),
        },
    )
    return root


def _stage_a(tmp_path: Path) -> Path:
    path = tmp_path / "stage_a_budget_report.json"
    path.write_text(json.dumps({
        "status": "FORMAL_RL_STAGE_A_FIXED_MAP_COMPLETE",
        "fixed_map_id": "stage-a-fixed-formal-map-000",
        "budget_contract": {
            "stage_a_fixed_map": {
                "task_counts": {"train": 10000, "validation": 500, "hidden": 1000}
            }
        },
        "hidden_tasks_materialized_after_freeze": True,
        "configuration_freeze": {
            "frozen_after_validation": True,
            "selection_source": "validation_only_before_hidden",
            "policy_seeds": POLICY_SEEDS,
        },
        "policy_runs": [
            {"policy_seed": seed, "train_episode_count": 10000,
             "validation_episode_count": 500, "hidden_episode_count": 1000}
            for seed in POLICY_SEEDS
        ],
    }), encoding="utf-8")
    return path


def _snapshot(tmp_path: Path, source_hash: str = "source-hash") -> Path:
    path = tmp_path / "snapshot.json"
    path.write_text(
        json.dumps({"source_inventory_sha256": source_hash, "outputs": {}}),
        encoding="utf-8",
    )
    return path


def test_report_has_unique_acceptance_status_and_explicit_claim_boundary(tmp_path: Path) -> None:
    report = generate(_evidence(tmp_path), _snapshot(tmp_path), stage_a_evidence=_stage_a(tmp_path))
    assert report["report_id"] == "tzcup_formal_rl_first_dual_mode_cross_map_acceptance_v1"
    assert report["status"] == PASSED_STATUS
    assert report["passed"] is True
    boundary = report["claim_boundary"]
    assert "Pure-Python belief-only" in boundary
    assert "Product perception" in boundary
    assert "Gazebo dynamics" in boundary
    assert "Nav2 execution" in boundary
    assert "complete product-loop acceptance" in boundary
    assert report["source_binding"]["source_inventory_sha256"] == "source-hash"
    assert len(report["source_binding"]["snapshot_manifest_sha256"]) == 64
    assert report["full_map_generalization_contract"]["actual_distinct_map_counts"] == FULL_COUNTS
    assert report["stage_a_fixed_map_budget_contract"]["task_counts"] == {
        "train": 10000,
        "validation": 500,
        "hidden": 1000,
    }


def test_report_fails_closed_below_hidden_observation_threshold(tmp_path: Path) -> None:
    report = generate(
        _evidence(tmp_path, observed_ratio=0.9499), _snapshot(tmp_path), stage_a_evidence=_stage_a(tmp_path)
    )
    assert report["passed"] is False
    assert report["status"].endswith("_FAILED")


def test_report_fails_closed_when_mileage_ratio_is_not_recomputable(tmp_path: Path) -> None:
    evidence = _evidence(tmp_path)
    validation = json.loads((evidence / "validation_report.json").read_text())
    next(
        row
        for row in validation["hybrid_episodes"]
        if row["split"] == "hidden"
    )["task_distance"] = 70.0
    (evidence / "validation_report.json").write_text(json.dumps(validation))
    report = generate(evidence, _snapshot(tmp_path), stage_a_evidence=_stage_a(tmp_path))
    assert report["passed"] is False


def test_report_rejects_single_hidden_episode_even_when_it_passes_metrics(tmp_path: Path) -> None:
    evidence = _evidence(tmp_path)
    validation_path = evidence / "validation_report.json"
    validation = json.loads(validation_path.read_text())
    validation["hybrid_episodes"] = [
        row
        for row in validation["hybrid_episodes"]
        if row["split"] != "hidden" or row["map_id"] == "hidden-map-000"
    ]
    validation_path.write_text(json.dumps(validation))
    report = generate(evidence, _snapshot(tmp_path), stage_a_evidence=_stage_a(tmp_path))
    assert report["passed"] is False


def test_report_rejects_stage_a_budget_metadata_drift(tmp_path: Path) -> None:
    stage_a = _stage_a(tmp_path)
    payload = json.loads(stage_a.read_text(encoding="utf-8"))
    payload["budget_contract"]["stage_a_fixed_map"]["task_counts"]["train"] = 52
    stage_a.write_text(json.dumps(payload), encoding="utf-8")

    report = generate(
        _evidence(tmp_path), _snapshot(tmp_path), stage_a_evidence=stage_a
    )
    assert report["passed"] is False
    assert report["stage_a_fixed_map_budget_contract"]["task_counts"]["train"] == 52


def test_cli_binds_fresh_session_and_unified_runtime_closure() -> None:
    source = (Path(__file__).with_name("generate_formal_rl_multimap_report.py")).read_text()
    assert "build_binding(" in source
    assert '"--runtime-overlay"' in source
    assert '"--runtime-closure"' in source
    assert '"--session"' in source
