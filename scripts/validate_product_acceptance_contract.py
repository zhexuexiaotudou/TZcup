#!/usr/bin/env python3
"""Validate the static A12/AUTO-15 contract and optional retained evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config" / "high_fidelity_vehicle" / "product_acceptance_contract.json"


class ProductAcceptanceContractError(ValueError):
    """Raised when a product-acceptance contract or receipt is incomplete."""


def _json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ProductAcceptanceContractError(f"cannot read JSON object {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ProductAcceptanceContractError(f"JSON root must be an object: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_contract(path: Path = DEFAULT_CONTRACT) -> dict[str, Any]:
    return _json_object(path)


def validate_static_contract(contract: dict[str, Any]) -> dict[str, Any]:
    accounting = contract.get("auto15_execution_accounting")
    runtime_states = contract.get("product_runtime_states")
    spec = contract.get("authoritative_specification")
    if contract.get("contract_id") != "tzcup_product_acceptance_contract_v1":
        raise ProductAcceptanceContractError("unexpected product-acceptance contract id")
    if not isinstance(accounting, dict) or not isinstance(runtime_states, dict) or not isinstance(spec, dict):
        raise ProductAcceptanceContractError("contract is missing required objects")
    scenarios, seeds = accounting.get("scenario_ids"), accounting.get("seeds")
    if not isinstance(scenarios, list) or len(scenarios) != 18 or len(set(scenarios)) != 18:
        raise ProductAcceptanceContractError("AUTO-15 must declare exactly 18 unique scenarios")
    if not isinstance(seeds, list) or seeds != list(range(10)):
        raise ProductAcceptanceContractError("AUTO-15 must declare seeds 0 through 9 exactly once")
    expected_count = len(scenarios) * len(seeds)
    if accounting.get("required_execution_count") != expected_count:
        raise ProductAcceptanceContractError("AUTO-15 required execution count must equal scenario x seed cardinality")
    if accounting.get("minimum_mission_group_count") != 30:
        raise ProductAcceptanceContractError("AUTO-15 must require at least 30 mission groups")
    if accounting.get("required_execution_evidence") != ["video", "mcap"]:
        raise ProductAcceptanceContractError("AUTO-15 must retain video and MCAP for every execution")
    if spec.get("path") != "docs/a12-product-acceptance-specification.md":
        raise ProductAcceptanceContractError("A12 specification path drifted")
    specification_path = ROOT / spec["path"]
    if specification_path.is_symlink() or not specification_path.is_file():
        raise ProductAcceptanceContractError("versioned A12 specification is unavailable")
    if any(value is not False for value in runtime_states.values()):
        raise ProductAcceptanceContractError("static contract must not promote product runtime states")
    execution_ids = [f"{scenario}:seed-{seed}" for scenario in scenarios for seed in seeds]
    return {
        "static_contract_pass": True,
        "scenario_count": len(scenarios),
        "seed_count_per_scenario": len(seeds),
        "required_execution_count": expected_count,
        "required_execution_ids": execution_ids,
        "minimum_mission_group_count": accounting["minimum_mission_group_count"],
        "product_runtime_states": runtime_states,
    }


def _evidence_file(value: Any, evidence_root: Path, label: str) -> str:
    if not isinstance(value, dict):
        raise ProductAcceptanceContractError(f"{label} evidence must be an object")
    relative, expected_hash = value.get("path"), value.get("sha256")
    if not isinstance(relative, str) or not relative or not isinstance(expected_hash, str) or len(expected_hash) != 64:
        raise ProductAcceptanceContractError(f"{label} evidence requires path and sha256")
    path = (evidence_root / relative).resolve()
    try:
        path.relative_to(evidence_root.resolve())
    except ValueError as exc:
        raise ProductAcceptanceContractError(f"{label} evidence escapes evidence root") from exc
    if path.is_symlink() or not path.is_file() or path.stat().st_size == 0:
        raise ProductAcceptanceContractError(f"{label} evidence is not a retained nonempty regular file: {relative}")
    if _sha256(path) != expected_hash:
        raise ProductAcceptanceContractError(f"{label} evidence hash mismatch: {relative}")
    return relative


def validate_auto15_execution_evidence(
    contract: dict[str, Any], payload: dict[str, Any], evidence_root: Path
) -> dict[str, Any]:
    static = validate_static_contract(contract)
    executions = payload.get("executions")
    if not isinstance(executions, list):
        raise ProductAcceptanceContractError("execution evidence must contain an executions list")
    expected_ids = set(static["required_execution_ids"])
    observed_ids: set[str] = set()
    mission_groups: set[str] = set()
    videos: set[str] = set()
    mcaps: set[str] = set()
    for item in executions:
        if not isinstance(item, dict):
            raise ProductAcceptanceContractError("each execution receipt must be an object")
        scenario, seed = item.get("scenario_id"), item.get("seed")
        execution_id = f"{scenario}:seed-{seed}"
        if execution_id not in expected_ids or execution_id in observed_ids:
            raise ProductAcceptanceContractError(f"execution identity is missing, unexpected, or duplicated: {execution_id}")
        if item.get("status") != "PASS":
            raise ProductAcceptanceContractError(f"execution did not pass: {execution_id}")
        mission_group = item.get("mission_group_id")
        if not isinstance(mission_group, str) or not mission_group:
            raise ProductAcceptanceContractError(f"execution has no mission group: {execution_id}")
        videos.add(_evidence_file(item.get("video"), evidence_root, f"{execution_id} video"))
        mcaps.add(_evidence_file(item.get("mcap"), evidence_root, f"{execution_id} MCAP"))
        observed_ids.add(execution_id)
        mission_groups.add(mission_group)
    if observed_ids != expected_ids:
        raise ProductAcceptanceContractError("all 180 unique scenario/seed execution receipts are required")
    if len(mission_groups) < static["minimum_mission_group_count"]:
        raise ProductAcceptanceContractError("at least 30 distinct mission groups are required")
    if len(videos) != len(expected_ids) or len(mcaps) != len(expected_ids):
        raise ProductAcceptanceContractError("every unique execution requires distinct retained video and MCAP evidence")
    return {
        **static,
        "execution_evidence_pass": True,
        "retained_execution_count": len(observed_ids),
        "mission_group_count": len(mission_groups),
        "product_runtime_states": contract["product_runtime_states"],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--execution-evidence", type=Path)
    parser.add_argument("--evidence-root", type=Path, default=Path.cwd())
    args = parser.parse_args()
    try:
        contract = load_contract(args.contract)
        result = validate_static_contract(contract)
        if args.execution_evidence:
            result = validate_auto15_execution_evidence(
                contract, _json_object(args.execution_evidence), args.evidence_root
            )
    except ProductAcceptanceContractError as exc:
        print(f"product acceptance contract failed closed: {exc}")
        return 2
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
