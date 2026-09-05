#!/usr/bin/env python3
"""Build the fail-closed formal RL dual-mode cross-map acceptance report."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from formal_runtime_gate_binding import build_binding


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_EVIDENCE = ROOT / ".work/formal_rl_multimap_v7_evidence/formal_planning"
DEFAULT_OUTPUT = ROOT / "reports/engineering/formal_rl_multimap_v7_evaluation.json"
DEFAULT_SNAPSHOT = ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
DEFAULT_SESSION = ROOT / "artifacts/formal_final_acceptance_session.json"
DEFAULT_RUNTIME_WS = ROOT / ".work/final_frozen_runtime"
PASSED_STATUS = "FORMAL_RL_FIRST_DUAL_MODE_CROSS_MAP_ACCEPTANCE_PASSED"
FAILED_STATUS = "FORMAL_RL_FIRST_DUAL_MODE_CROSS_MAP_ACCEPTANCE_FAILED"
FORMAL_FULL_MAP_COUNTS = {"train": 32, "validation": 8, "hidden": 12}
FORMAL_MULTIMAP_TASK_COUNTS = {"train": 6400, "validation": 800, "hidden": 1200}
FORMAL_STAGE_A_TASK_COUNTS = {"train": 10000, "validation": 500, "hidden": 1000}
FORMAL_POLICY_SEEDS = [7, 17, 29, 43, 61]


def _full_map_generalization_contract(
    training: dict[str, Any],
    validation: dict[str, Any],
    checkpoint: dict[str, Any],
) -> tuple[bool, dict[str, Any]]:
    """Verify that all frozen maps, rather than a smoke subset, were covered."""
    contract = training.get("formal_multimap_contract")
    frozen = training.get("frozen_split_manifest")
    checkpoint_contract = checkpoint.get("formal_multimap_contract")
    validation_contract = validation.get("formal_multimap_contract")
    if not isinstance(contract, dict):
        return False, {"reason": "training_report_missing_formal_multimap_contract"}
    if not isinstance(frozen, dict):
        return False, {"reason": "training_report_missing_frozen_split_manifest"}
    required = contract.get("required_map_counts")
    actual = contract.get("actual_distinct_map_counts")
    indices = contract.get("map_indices")
    selections = frozen.get("selections")
    valid = (
        required == FORMAL_FULL_MAP_COUNTS
        and actual == FORMAL_FULL_MAP_COUNTS
        and contract.get("full_map_coverage") is True
        and contract.get("smoke_subset_accepted_as_generalization") is False
        and frozen.get("required_map_counts") == FORMAL_FULL_MAP_COUNTS
        and frozen.get("all_frozen_missions_per_declared_map") is True
        and frozen.get("smoke_subset_accepted_as_generalization") is False
        and isinstance(indices, dict)
        and isinstance(selections, dict)
        and checkpoint_contract == contract
        and validation_contract == contract
    )
    expected_split_names = {"train", "validation", "hidden"}
    if set(indices) != expected_split_names or set(selections) != expected_split_names:
        valid = False
    else:
        for name, count in FORMAL_FULL_MAP_COUNTS.items():
            if indices[name] != list(range(count)):
                valid = False
            mission_count = {"train": 200, "validation": 100, "hidden": 100}[name]
            if selections[name] != [
                f"{map_index}:{mission_index}"
                for map_index in range(count)
                for mission_index in range(mission_count)
            ]:
                valid = False
    map_splits = training.get("map_splits")
    expected_report_splits = {
        "train": "train",
        "validation": "validation",
        "hidden": "test",
    }
    split_map_ids: dict[str, set[str]] = {}
    if not isinstance(map_splits, dict) or set(map_splits) != {
        "train",
        "validation",
        "test",
    }:
        valid = False
    else:
        for name, report_name in expected_report_splits.items():
            rows = map_splits.get(report_name)
            if not isinstance(rows, list) or len(rows) != FORMAL_FULL_MAP_COUNTS[name]:
                valid = False
                continue
            try:
                split_map_ids[name] = {str(row["map_id"]) for row in rows}
                split_indices = sorted(int(row["map_index"]) for row in rows)
            except (KeyError, TypeError, ValueError):
                valid = False
                continue
            if (
                len(split_map_ids[name]) != FORMAL_FULL_MAP_COUNTS[name]
                or split_indices != list(range(FORMAL_FULL_MAP_COUNTS[name]))
            ):
                valid = False
    hybrid = validation.get("hybrid_episodes")
    if not isinstance(hybrid, list):
        valid = False
    else:
        for name in ("validation", "hidden"):
            report_split = "validation" if name == "validation" else "hidden"
            measured_ids = {
                str(row.get("map_id"))
                for row in hybrid
                if isinstance(row, dict) and row.get("split") == report_split
            }
            if measured_ids != split_map_ids.get(name, set()):
                valid = False
    return valid, {
        "required_map_counts": FORMAL_FULL_MAP_COUNTS,
        "actual_distinct_map_counts": actual,
        "map_indices": indices,
        "fixed_selection": selections,
        "full_map_coverage": contract.get("full_map_coverage"),
        "smoke_subset_accepted_as_generalization": contract.get(
            "smoke_subset_accepted_as_generalization"
        ),
        "materialized_map_ids": {
            name: sorted(map_ids) for name, map_ids in split_map_ids.items()
        },
    }


def _formal_budget_contract(training: dict[str, Any]) -> tuple[bool, dict[str, Any]]:
    """Reject map-only smoke evidence even when every map has mission zero."""
    execution = training.get("formal_budget_execution")
    if not isinstance(execution, dict):
        return False, {"reason": "training_report_missing_formal_budget_execution"}
    contract = execution.get("contract")
    runs = execution.get("policy_seed_runs")
    freeze = execution.get("configuration_freeze")
    valid = (
        execution.get("formal_budget_claim") is True
        and isinstance(contract, dict)
        and contract.get("contract_id") == "tzcup_formal_rl_budget_v1"
        and execution.get("task_counts") == FORMAL_MULTIMAP_TASK_COUNTS
        and execution.get("training_rollout_count") == 6400
        and execution.get("max_steps_per_episode") == 400
        and execution.get("max_steps_is_episode_truncation_guard_not_task_or_episode_budget") is True
        and execution.get("policy_seeds") == FORMAL_POLICY_SEEDS
        and isinstance(freeze, dict)
        and freeze.get("frozen_before_hidden") is True
        and freeze.get("selection_source") == "validation_only_before_hidden"
        and freeze.get("policy_seeds") == FORMAL_POLICY_SEEDS
        and isinstance(runs, list)
        and len(runs) == len(FORMAL_POLICY_SEEDS)
    )
    if valid:
        for expected_seed, run in zip(FORMAL_POLICY_SEEDS, runs):
            if not isinstance(run, dict) or (
                run.get("policy_seed") != expected_seed
                or run.get("training_rollout_count") != 6400
                or run.get("validation_episode_count") != 800
                or run.get("hidden_episode_count") != 1200
            ):
                valid = False
                break
    return valid, {
        "task_counts": execution.get("task_counts"),
        "training_rollout_count": execution.get("training_rollout_count"),
        "max_steps_per_episode": execution.get("max_steps_per_episode"),
        "max_steps_is_episode_truncation_guard_not_task_or_episode_budget": execution.get("max_steps_is_episode_truncation_guard_not_task_or_episode_budget"),
        "policy_seeds": execution.get("policy_seeds"),
        "policy_seed_runs": runs,
        "configuration_freeze": freeze,
        "smoke_budget_can_pass_formal_acceptance": False,
    }


def _stage_a_budget_contract(path: Path | None) -> tuple[bool, dict[str, Any]]:
    if path is None or not path.is_file():
        return False, {"reason": "stage_a_budget_evidence_missing"}
    try:
        evidence = _read(path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return False, {"reason": f"stage_a_budget_evidence_invalid:{exc}"}
    runs = evidence.get("policy_runs")
    freeze = evidence.get("configuration_freeze")
    budget = evidence.get("budget_contract")
    stage_a_budget = budget.get("stage_a_fixed_map") if isinstance(budget, dict) else None
    task_counts = (
        stage_a_budget.get("task_counts") if isinstance(stage_a_budget, dict) else None
    )
    valid = (
        evidence.get("status") == "FORMAL_RL_STAGE_A_FIXED_MAP_COMPLETE"
        and evidence.get("fixed_map_id") == "stage-a-fixed-formal-map-000"
        and task_counts == FORMAL_STAGE_A_TASK_COUNTS
        and evidence.get("hidden_tasks_materialized_after_freeze") is True
        and isinstance(freeze, dict)
        and freeze.get("frozen_after_validation") is True
        and freeze.get("selection_source") == "validation_only_before_hidden"
        and freeze.get("policy_seeds") == FORMAL_POLICY_SEEDS
        and isinstance(runs, list)
        and len(runs) == len(FORMAL_POLICY_SEEDS)
    )
    if valid:
        for seed, run in zip(FORMAL_POLICY_SEEDS, runs):
            if not isinstance(run, dict) or (
                run.get("policy_seed") != seed
                or run.get("train_episode_count") != FORMAL_STAGE_A_TASK_COUNTS["train"]
                or run.get("validation_episode_count") != FORMAL_STAGE_A_TASK_COUNTS["validation"]
                or run.get("hidden_episode_count") != FORMAL_STAGE_A_TASK_COUNTS["hidden"]
            ):
                valid = False
                break
    return valid, {
        "evidence_path": str(path), "fixed_map_id": evidence.get("fixed_map_id"),
        "task_counts": task_counts,
        "policy_runs": runs, "configuration_freeze": freeze,
        "hidden_tasks_materialized_after_freeze": evidence.get("hidden_tasks_materialized_after_freeze"),
        "smoke_budget_can_pass_formal_acceptance": False,
    }


def _read(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def _rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    keys = (
        "split",
        "episode_id",
        "success",
        "formal_success",
        "observed_ratio",
        "ground_clear_ratio",
        "discrete_clear_ratio",
        "task_distance",
        "baseline_distance",
        "path_ratio_to_full_coverage",
        "collisions",
        "boundary_violations",
        "invalid_actions",
        "runtime_s",
        "systematic_coverage_backstop_activated",
    )
    return [{key: row.get(key) for key in keys if key in row} for row in rows]


def _source_binding(snapshot_path: Path) -> dict[str, str]:
    snapshot = _read(snapshot_path)
    source_hash = snapshot.get("source_inventory_sha256")
    if not isinstance(source_hash, str) or not source_hash:
        raise ValueError("snapshot has no source_inventory_sha256")
    return {
        "snapshot_manifest_sha256": hashlib.sha256(snapshot_path.read_bytes()).hexdigest(),
        "source_inventory_sha256": source_hash,
    }


def generate(
    evidence_root: Path = DEFAULT_EVIDENCE,
    snapshot_path: Path = DEFAULT_SNAPSHOT,
    runtime_binding: dict[str, Any] | None = None,
    stage_a_evidence: Path | None = None,
) -> dict[str, Any]:
    paths = {
        name: evidence_root / name
        for name in (
            "training_report.json",
            "baseline_report.json",
            "validation_report.json",
            "q_policy.json",
        )
    }
    training = _read(paths["training_report.json"])
    baseline = _read(paths["baseline_report.json"])
    validation = _read(paths["validation_report.json"])
    checkpoint = _read(paths["q_policy.json"])
    hidden = [
        row for row in validation.get("hybrid_episodes", [])
        if row.get("split") == "hidden"
    ]
    full_map_generalization_valid, full_map_generalization = (
        _full_map_generalization_contract(training, validation, checkpoint)
    )
    formal_budget_valid, formal_budget = _formal_budget_contract(training)
    stage_a_budget_valid, stage_a_budget = _stage_a_budget_contract(stage_a_evidence)
    session_start_ns = (
        runtime_binding["acceptance_session_binding"]["session_started_epoch_ns"]
        if runtime_binding is not None
        else None
    )
    evidence_is_fresh = session_start_ns is None or all(
        path.stat().st_mtime_ns >= session_start_ns for path in paths.values()
    )
    passed = (
        bool(hidden)
        and full_map_generalization_valid
        and formal_budget_valid
        and stage_a_budget_valid
        and evidence_is_fresh
        and validation.get("hidden_gate_passed") is True
        and validation.get("truth_used_for_control") is False
        and validation.get("return_distance_included") is False
        and validation.get("policy_output") == "global_reference_trajectory"
        and all(
        row.get("formal_success") is True
        and float(row.get("observed_ratio", 0.0)) >= 0.95
        and float(row.get("ground_clear_ratio", 0.0)) >= 0.95
        and float(row.get("discrete_clear_ratio", 0.0)) >= 0.95
        and int(row.get("collisions", -1)) == 0
        and int(row.get("boundary_violations", -1)) == 0
        and int(row.get("invalid_actions", -1)) == 0
        and 0.0 < float(row.get("task_distance", 0.0))
        and 0.0 < float(row.get("baseline_distance", 0.0))
        and math.isclose(
            float(row.get("path_ratio_to_full_coverage", math.inf)),
            float(row.get("task_distance", 0.0))
            / float(row.get("baseline_distance", 0.0)),
            rel_tol=1.0e-9,
            abs_tol=1.0e-9,
        )
        and float(row.get("path_ratio_to_full_coverage", 2.0)) <= 1.0
        for row in hidden
        )
    )
    if checkpoint.get("truth_access_used") is not False:
        passed = False

    report = {
        "report_id": "tzcup_formal_rl_first_dual_mode_cross_map_acceptance_v1",
        "status": PASSED_STATUS if passed else FAILED_STATUS,
        "passed": passed,
        "source_binding": _source_binding(snapshot_path),
        "claim_boundary": (
            "Pure-Python belief-only planning acceptance over materialized formal maps. "
            "Policy control used only public belief observations; evaluator truth was used "
            "only for simulator initialization and final metrics. Product perception, "
            "Gazebo dynamics, Nav2 execution, RDK S100 runtime and the complete product "
            "closed loop were not used or accepted; this report must not be represented "
            "as complete product-loop acceptance."
        ),
        "truth_used_for_control": False,
        "product_perception_claim": False,
        "return_distance_included": validation.get("return_distance_included"),
        "policy_output": validation.get("policy_output"),
        "full_map_generalization_contract": full_map_generalization,
        "formal_multimap_budget_contract": formal_budget,
        "stage_a_fixed_map_budget_contract": stage_a_budget,
        "scenario_contract": {
            "area_m2": 20000,
            "static_assets": 120,
            "ground_dirt_patches": 18,
            "discrete_cubes": 20,
            "moving_pedestrians": 8,
            "planning_resolution_m": 2.0,
            "sensing_radius_m": 10.0,
            "sensing_fov_deg": 87.0,
            "max_steps": 400,
        },
        "root_causes_and_fixes": [
            {
                "cause": "Near-boundary outward/reversal poses had no legal forward-only one-arc path.",
                "fix": "Added a curvature-valid reverse Ackermann counterpart with the same safety checks.",
            },
            {
                "cause": "Pure Q could stall below 95% observed area on unseen maps.",
                "fix": "Declared dual mode switches to truth-free systematic coverage after six public belief gains below one percentage point.",
            },
            {
                "cause": "Pedestrians could advance into the live chassis after an accepted action.",
                "fix": "Pedestrian dynamics now treat the current transport-stowed vehicle footprint as an obstacle.",
            },
        ],
        "training": {
            "episodes": _rows(training.get("episodes", [])),
            "pure_q_training_pass_count": sum(
                bool(row.get("terminated")) for row in training.get("episodes", [])
            ),
            "episode_count": len(training.get("episodes", [])),
        },
        "full_coverage_baseline": _rows(baseline.get("episodes", [])),
        "pure_q": _rows(validation.get("pure_q_episodes", [])),
        "q_with_systematic_coverage_backstop": _rows(
            validation.get("hybrid_episodes", [])
        ),
        "hidden_gate_passed": validation.get("hidden_gate_passed") is True,
        "gate_policy": validation.get("gate_policy"),
        "limitations": [
            "Simulation evaluator truth initializes missions and computes metrics only.",
            "Pure Q is reported separately and is not the passing policy.",
            "Navigation uses the transport-stowed footprint; manipulation deployment is outside these rollouts.",
        ],
        "evidence_sha256": {
            name: hashlib.sha256(path.read_bytes()).hexdigest()
            for name, path in paths.items()
        },
    }
    if runtime_binding is not None:
        report["acceptance_session_binding"] = runtime_binding[
            "acceptance_session_binding"
        ]
        report["runtime_closure_binding"] = runtime_binding[
            "runtime_closure_binding"
        ]
        report["evidence_generated_after_session_start"] = evidence_is_fresh
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root", type=Path, default=DEFAULT_EVIDENCE)
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--stage-a-evidence", type=Path)
    parser.add_argument("--runtime-overlay", type=Path, default=DEFAULT_RUNTIME_WS / "install")
    parser.add_argument(
        "--runtime-closure",
        type=Path,
        default=DEFAULT_RUNTIME_WS / "final_runtime_closure_manifest.json",
    )
    args = parser.parse_args()
    runtime_binding = build_binding(
        repository_root=ROOT,
        install_root=args.runtime_overlay,
        closure_manifest=args.runtime_closure,
        session_path=args.session,
        snapshot_path=args.snapshot,
    )
    report = generate(args.evidence_root, args.snapshot, runtime_binding, args.stage_a_evidence)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(args.output)
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
