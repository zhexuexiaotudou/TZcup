"""
Fail-closed audit for the formal dual-mode planning chain.

This module does not implement a fallback policy or fabricate training
evidence. It makes the gap between the research environment and a
product-input/Nav2-executed formal run machine-readable.
"""

from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml


class FormalPlanningContractError(RuntimeError):
    """Raised when the declared planning contract is malformed."""


def _mapping(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise FormalPlanningContractError(f"unable to read YAML mapping: {path}") from exc
    if not isinstance(value, dict):
        raise FormalPlanningContractError(f"expected YAML mapping: {path}")
    return value


def _json_mapping(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def load_formal_contract(path: str | Path) -> dict[str, Any]:
    """Load and validate immutable project-level planning decisions."""
    contract = _mapping(Path(path))
    if contract.get("schema_version") != 1:
        raise FormalPlanningContractError("planning contract schema_version must be 1")
    modes = contract.get("modes", {})
    if set(modes) != {"traditional", "reinforcement_learning"}:
        raise FormalPlanningContractError(
            "both traditional and reinforcement_learning modes are required"
        )
    scoring = contract.get("scoring", {})
    for key in (
        "observation_threshold",
        "ground_clear_threshold",
        "discrete_clear_threshold",
    ):
        if float(scoring.get(key, -1.0)) != 0.95:
            raise FormalPlanningContractError(f"{key} must remain 0.95")
    if scoring.get("distance_upper_bound") != "same_episode_full_coverage_distance":
        raise FormalPlanningContractError(
            "distance must be bounded by the paired full-coverage run"
        )
    if scoring.get("return_distance_included") is not False:
        raise FormalPlanningContractError("return distance must remain excluded")
    if scoring.get("time_or_energy_scored") is not False:
        raise FormalPlanningContractError("time and energy must remain unscored")
    inputs = contract.get("product_inputs", {})
    forbidden = tuple(
        str(item).lower() for item in inputs.get("forbidden_control_inputs", ())
    )
    if not forbidden or not any("ground_truth" in item for item in forbidden):
        raise FormalPlanningContractError(
            "ground-truth control inputs must be explicitly forbidden"
        )
    generalization = contract.get("generalization", {})
    required_true = (
        "field_area_fixed",
        "field_aspect_ratio_variable",
        "dirt_random_each_episode",
        "pedestrians_random_each_episode",
        "map_ids_disjoint_across_train_validation_test",
        "mission_seeds_disjoint_across_train_validation_test",
    )
    if not all(generalization.get(key) is True for key in required_true):
        raise FormalPlanningContractError(
            "formal generalization requirements are incomplete"
        )
    return contract


def _source_is_truth_free(path: Path, forbidden: Sequence[str]) -> bool:
    if not path.is_file():
        return False
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"))
    except (OSError, SyntaxError):
        return False
    inputs: tuple[str, ...] | None = None
    imported_names: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_names.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imported_names.append(node.module or "")
        elif isinstance(node, (ast.Assign, ast.AnnAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            if any(isinstance(target, ast.Name) and target.id == "CONTROL_INPUT_TOPICS" for target in targets):
                try:
                    value = ast.literal_eval(node.value)
                except (ValueError, TypeError):
                    return False
                if not isinstance(value, tuple) or not all(
                    isinstance(item, str) for item in value
                ):
                    return False
                inputs = value
    if not inputs:
        return False
    lowered_forbidden = tuple(token.lower() for token in forbidden)
    declared = tuple(item.lower() for item in inputs)
    imports = tuple(item.lower() for item in imported_names)
    return not any(
        token in value
        for token in lowered_forbidden
        for value in (*declared, *imports)
    )


def _split_metadata_valid(report: Mapping[str, Any]) -> bool:
    splits = report.get("map_splits")
    if not isinstance(splits, dict) or set(splits) != {
        "train",
        "validation",
        "test",
    }:
        return False
    id_sets: list[set[str]] = []
    mission_sets: list[set[int]] = []
    areas: list[float] = []
    aspects: set[float] = set()
    for name in ("train", "validation", "test"):
        rows = splits.get(name)
        if not isinstance(rows, list) or not rows:
            return False
        ids: set[str] = set()
        seeds: set[int] = set()
        for row in rows:
            if not isinstance(row, dict):
                return False
            try:
                map_id = str(row["map_id"])
                area = float(row["area_m2"])
                aspect = float(row["aspect_ratio"])
                row_seeds = {int(value) for value in row["mission_seeds"]}
            except (KeyError, TypeError, ValueError):
                return False
            if not map_id or area <= 0.0 or aspect <= 0.0 or not row_seeds:
                return False
            ids.add(map_id)
            seeds.update(row_seeds)
            areas.append(area)
            aspects.add(round(aspect, 6))
        id_sets.append(ids)
        mission_sets.append(seeds)
    if any(
        id_sets[i] & id_sets[j]
        for i in range(3)
        for j in range(i + 1, 3)
    ):
        return False
    if any(
        mission_sets[i] & mission_sets[j]
        for i in range(3)
        for j in range(i + 1, 3)
    ):
        return False
    if max(areas) - min(areas) > max(areas) * 1.0e-3:
        return False
    return len(aspects) >= 2


def _validation_evidence_valid(
    report: Mapping[str, Any], thresholds: Mapping[str, Any]
) -> bool:
    if (
        report.get("formal_map_used") is not True
        or report.get("product_perception_used") is not True
    ):
        return False
    if report.get("truth_used_for_control") is not False:
        return False
    if (
        report.get("return_distance_included") is not False
        or report.get("time_energy_ignored") is not True
    ):
        return False
    if report.get("policy_output") not in {
        "global_reference_trajectory",
        "nav_msgs/msg/Path",
    }:
        return False
    episodes = report.get("episodes")
    if not isinstance(episodes, list) or not episodes:
        return False
    observation = float(thresholds["observation_threshold"])
    ground = float(thresholds["ground_clear_threshold"])
    discrete = float(thresholds["discrete_clear_threshold"])
    for row in episodes:
        if not isinstance(row, dict):
            return False
        try:
            if float(row["observed_ratio"]) < observation:
                return False
            if float(row["ground_clear_ratio"]) < ground:
                return False
            if float(row["discrete_clear_ratio"]) < discrete:
                return False
            if (
                float(row["task_distance"])
                > float(row["baseline_distance"]) + 1.0e-6
            ):
                return False
        except (KeyError, TypeError, ValueError):
            return False
    return True


def audit_formal_planning(
    contract_path: str | Path,
    *,
    repository_root: str | Path,
    runtime_root: str | Path,
    evidence_root: str | Path,
) -> dict[str, Any]:
    """Audit source wiring and immutable run evidence without executing a policy."""
    contract = load_formal_contract(contract_path)
    repo = Path(repository_root).resolve()
    runtime = Path(runtime_root).resolve()
    evidence = Path(evidence_root).resolve()
    checks: dict[str, bool] = {}
    blockers: list[str] = []

    def record(name: str, passed: bool, blocker: str) -> None:
        checks[name] = bool(passed)
        if not passed:
            blockers.append(blocker)

    inputs = contract["product_inputs"]
    required_runtime = (
        inputs["occupancy_map"],
        inputs["keepout_map"],
        inputs["mission_geometry"],
        inputs["materialization_contract"],
    )
    record(
        "formal_map_artifacts_present",
        all((runtime / name).is_file() for name in required_runtime),
        "formal_map_runtime_artifacts_missing",
    )
    materialization_path = runtime / inputs["materialization_contract"]
    materialization = _mapping(materialization_path) if materialization_path.is_file() else {}
    record(
        "formal_map_excludes_evaluator_and_dirt_truth",
        materialization.get("evaluator_truth_used") is False
        and materialization.get("dirt_truth_used") is False,
        "formal_map_truth_boundary_unproven",
    )

    sources = contract["formal_runtime_sources"]
    forbidden = tuple(inputs["forbidden_control_inputs"])
    observation_bridge = repo / sources["observation_bridge"]
    trajectory_executor = repo / sources["trajectory_executor"]
    policy_planner = repo / sources["policy_planner"]
    record(
        "product_observation_bridge_present_and_truth_free",
        _source_is_truth_free(observation_bridge, forbidden),
        "formal_product_observation_bridge_missing_or_truth_contaminated",
    )
    record(
        "trajectory_executor_present_and_truth_free",
        _source_is_truth_free(trajectory_executor, forbidden),
        "formal_trajectory_executor_missing_or_truth_contaminated",
    )
    record(
        "frozen_policy_planner_present_and_truth_free",
        _source_is_truth_free(policy_planner, forbidden),
        "formal_policy_planner_missing_or_truth_contaminated",
    )
    traditional = repo / sources["traditional_executor"]
    traditional_text = (
        traditional.read_text(encoding="utf-8") if traditional.is_file() else ""
    )
    record(
        "traditional_nav2_follow_path_executor_present",
        "FollowPath" in traditional_text
        and '"ground_truth_used_for_control": False' in traditional_text,
        "traditional_follow_path_truth_boundary_missing",
    )

    required = contract["required_evidence"]
    perception = _json_mapping(evidence / required["perception_preflight"])
    record(
        "product_perception_preflight_ready",
        bool(
            perception
            and perception.get("ready") is True
            and perception.get("ground_truth_input_used") is False
        ),
        "product_perception_not_ready",
    )
    checkpoint = _json_mapping(evidence / required["policy_checkpoint"])
    record(
        "frozen_truth_free_rl_checkpoint_present",
        bool(
            checkpoint
            and checkpoint.get("policy") == "q_learning"
            and checkpoint.get("truth_access_used") is False
            and checkpoint.get("q_table")
        ),
        "formal_rl_checkpoint_missing_or_invalid",
    )
    training = _json_mapping(evidence / required["training_report"])
    record(
        "multi_map_training_splits_valid",
        bool(
            training
            and training.get("truth_access_used") is False
            and _split_metadata_valid(training)
        ),
        "formal_multi_map_training_report_missing_or_invalid",
    )
    baseline = _json_mapping(evidence / required["baseline_report"])
    record(
        "paired_formal_coverage_baseline_present",
        bool(
            baseline
            and baseline.get("mode") == "full_coverage"
            and baseline.get("formal_map_used") is True
            and baseline.get("truth_used_for_control") is False
            and baseline.get("return_distance_included") is False
            and isinstance(baseline.get("episodes"), list)
            and baseline["episodes"]
        ),
        "formal_full_coverage_baseline_report_missing_or_invalid",
    )
    validation = _json_mapping(evidence / required["validation_report"])
    record(
        "held_out_product_input_validation_passed",
        bool(
            validation
            and _validation_evidence_valid(validation, contract["scoring"])
        ),
        "formal_held_out_product_input_validation_missing_or_failed",
    )

    return {
        "schema_version": 1,
        "contract_id": contract["contract_id"],
        "status": "ready_for_formal_runtime" if not blockers else "blocked_fail_closed",
        "ready": not blockers,
        "checks": checks,
        "blockers": blockers,
        "ground_truth_input_used_for_control": False,
        "claim_boundary": contract["claim_boundary"],
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", required=True, type=Path)
    parser.add_argument("--repository-root", required=True, type=Path)
    parser.add_argument("--runtime-root", required=True, type=Path)
    parser.add_argument("--evidence-root", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    report = audit_formal_planning(
        args.contract,
        repository_root=args.repository_root,
        runtime_root=args.runtime_root,
        evidence_root=args.evidence_root,
    )
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0 if report["ready"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
