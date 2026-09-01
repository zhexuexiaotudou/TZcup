#!/usr/bin/env python3
"""Validate the source-bound, fail-closed electrical/harness/thermal declaration."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config/high_fidelity_vehicle/electrical_harness_thermal_readiness.yaml"
PRE_URDF = ROOT / "config/high_fidelity_vehicle/pre_urdf_contract.yaml"
NOT_READY = "NOT_READY_FOR_ELECTRICAL_HARNESS_THERMAL_RELEASE"
REQUIRED_BRANCHES = {
    "traction_base_inherited", "ur5e_control", "s100_compute", "sensors_12v",
    "sensors_24v", "brush_motors", "cleaning_lift", "recovery_pump",
    "service_actuators", "safety_relay_estop",
}
REQUIRED_BLOCKERS = {
    "exact_a300_vbat_converter_topology", "branch_fuses_wire_gauges_connectors",
    "grounding_emc_bonding", "ingress_protection_and_harness_routing",
    "voltage_drop_and_peak_current_measurement", "thermal_design_and_validation",
    "s100_exact_interface_and_thermal_measurement", "safety_relay_estop_architecture",
}
SAFETY_FIELDS = {
    "fuse_status", "wire_gauge_status", "connector_status", "grounding_bonding_status",
    "emc_status", "waterproofing_status", "voltage_drop_status", "thermal_status",
}
EXPECTED_ASSIGNMENTS = {
    "traction_base_inherited": set(),
    "ur5e_control": {"ur5e_controller"},
    "s100_compute": {"s100_compute"},
    "sensors_12v": {"utm30lx", "cameras_and_positioning"},
    "sensors_24v": {"mid360"},
    "brush_motors": {"three_brush_motors"},
    "cleaning_lift": {"cleaning_lift"},
    "recovery_pump": {"recovery_pump"},
    "service_actuators": set(),
    "safety_relay_estop": set(),
}


def _mapping(value: Any, label: str, errors: list[str]) -> dict[str, Any]:
    if not isinstance(value, dict):
        errors.append(f"{label} must be a mapping")
        return {}
    return value


def _number(value: Any, label: str, errors: list[str], *, nullable: bool = False) -> float | None:
    if value is None and nullable:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        errors.append(f"{label} must be a finite number")
        return None
    result = float(value)
    if not math.isfinite(result):
        errors.append(f"{label} must be a finite number")
        return None
    return result


def _load_yaml(path: Path, label: str, errors: list[str]) -> dict[str, Any]:
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        errors.append(f"cannot read {label}: {exc}")
        return {}
    return _mapping(payload, label, errors)


def _status(value: Any, label: str, errors: list[str]) -> None:
    if not isinstance(value, str) or not value.startswith("NOT_READY"):
        errors.append(f"{label} must remain NOT_READY until physical evidence exists")


def _unknown_or_not_ready(value: Any, label: str, errors: list[str]) -> None:
    if not isinstance(value, str) or not value.startswith(("UNKNOWN", "NOT_READY")):
        errors.append(f"{label} must remain UNKNOWN or NOT_READY until physical evidence exists")


def validate(config_path: Path = DEFAULT_CONFIG, *, root: Path = ROOT) -> dict[str, Any]:
    errors: list[str] = []
    config = _load_yaml(config_path, "electrical readiness config", errors)
    pre = _load_yaml(root / "config/high_fidelity_vehicle/pre_urdf_contract.yaml", "pre_urdf_contract", errors)

    if config.get("schema_version") != 1 or isinstance(config.get("schema_version"), bool):
        errors.append("schema_version must equal integer 1")
    if config.get("status") != NOT_READY:
        errors.append(f"status must equal {NOT_READY}")
    if type(config.get("ready")) is not bool or config.get("ready") is not False:
        errors.append("ready must be boolean false")
    if config.get("claim_boundary") != "source_budget_traceability_only_not_as_built_electrical_release":
        errors.append("claim_boundary must retain the non-release boundary")

    sources = _mapping(config.get("source_contracts"), "source_contracts", errors)
    expected_sources = {
        "pre_urdf_contract": "config/high_fidelity_vehicle/pre_urdf_contract.yaml",
        "vehicle_layout": "config/high_fidelity_vehicle/formal_vehicle_layout.yaml",
        "real_world_readiness": "config/high_fidelity_vehicle/real_world_build_readiness.yaml",
    }
    if sources != expected_sources:
        errors.append("source_contracts must bind the audited contract, layout, and real-world readiness files")
    for relative in expected_sources.values():
        if not (root / relative).is_file():
            errors.append(f"declared source contract is missing: {relative}")

    power_budget = _mapping(pre.get("power_budget"), "pre_urdf_contract.power_budget", errors)
    rails = _mapping(power_budget.get("base_user_rails"), "pre_urdf_contract.power_budget.base_user_rails", errors)
    raw_loads = power_budget.get("loads")
    if not isinstance(raw_loads, list):
        errors.append("pre_urdf_contract.power_budget.loads must be a list")
        raw_loads = []
    loads: dict[str, dict[str, Any]] = {}
    for index, raw in enumerate(raw_loads):
        row = _mapping(raw, f"pre_urdf_contract.power_budget.loads[{index}]", errors)
        load_id = row.get("id")
        if not isinstance(load_id, str) or not load_id or load_id in loads:
            errors.append("pre_urdf_contract power load IDs must be unique non-empty strings")
            continue
        loads[load_id] = row
    expected_load_ids = set(loads)
    source_binding = _mapping(config.get("power_budget_binding"), "power_budget_binding", errors)
    if source_binding.get("source_path") != "config/high_fidelity_vehicle/pre_urdf_contract.yaml":
        errors.append("power_budget_binding must point to pre_urdf_contract.yaml")
    declared_ids = source_binding.get("source_load_ids")
    if not isinstance(declared_ids, list) or set(declared_ids) != expected_load_ids or len(declared_ids) != len(expected_load_ids):
        errors.append("power_budget_binding.source_load_ids must exactly bind every pre_urdf power load")

    source_continuous = 0.0
    source_peak = 0.0
    source_values_valid = True
    for load_id, load in loads.items():
        continuous = _number(load.get("continuous_w"), f"pre_urdf_contract.power_budget.loads.{load_id}.continuous_w", errors)
        peak = _number(load.get("peak_w"), f"pre_urdf_contract.power_budget.loads.{load_id}.peak_w", errors)
        if continuous is None or peak is None:
            source_values_valid = False
        else:
            source_continuous += continuous
            source_peak += peak
    declared_continuous = _number(source_binding.get("declared_total_continuous_w"), "power_budget_binding.declared_total_continuous_w", errors)
    declared_peak = _number(source_binding.get("declared_total_peak_w"), "power_budget_binding.declared_total_peak_w", errors)
    if source_values_valid and (declared_continuous is None or not math.isclose(declared_continuous, source_continuous, abs_tol=1.0e-6)):
        errors.append("power_budget_binding.declared_total_continuous_w does not match pre_urdf power_budget")
    if source_values_valid and (declared_peak is None or not math.isclose(declared_peak, source_peak, abs_tol=1.0e-6)):
        errors.append("power_budget_binding.declared_total_peak_w does not match pre_urdf power_budget")

    required_ids = config.get("required_branch_ids")
    if not isinstance(required_ids, list) or set(required_ids) != REQUIRED_BRANCHES or len(required_ids) != len(REQUIRED_BRANCHES):
        errors.append("required_branch_ids must contain exactly every required branch")
    branches = _mapping(config.get("branches"), "branches", errors)
    if set(branches) != REQUIRED_BRANCHES:
        errors.append("branches must contain exactly every required branch")
    assigned_load_ids: list[str] = []
    branch_continuous = 0.0
    branch_peak = 0.0
    branch_values_valid = True
    for branch_id in sorted(REQUIRED_BRANCHES):
        branch = _mapping(branches.get(branch_id), f"branches.{branch_id}", errors)
        source_ids = branch.get("source_load_ids")
        if not isinstance(source_ids, list) or not all(isinstance(item, str) for item in source_ids):
            errors.append(f"branches.{branch_id}.source_load_ids must be a string list")
            source_ids = []
        if set(source_ids) != EXPECTED_ASSIGNMENTS[branch_id] or len(source_ids) != len(EXPECTED_ASSIGNMENTS[branch_id]):
            errors.append(f"branches.{branch_id}.source_load_ids does not match its audited power-budget assignment")
        assigned_load_ids.extend(source_ids)
        rail = branch.get("rail")
        if not isinstance(rail, str) or not rail:
            errors.append(f"branches.{branch_id}.rail must be a non-empty string")
        voltage = _number(branch.get("voltage_v"), f"branches.{branch_id}.voltage_v", errors, nullable=True)
        continuous = _number(branch.get("continuous_w"), f"branches.{branch_id}.continuous_w", errors, nullable=True)
        peak = _number(branch.get("peak_w"), f"branches.{branch_id}.peak_w", errors, nullable=True)
        upstream_current = _number(branch.get("upstream_current_limit_a"), f"branches.{branch_id}.upstream_current_limit_a", errors, nullable=True)
        if type(branch.get("branch_ready")) is not bool or branch.get("branch_ready") is not False:
            errors.append(f"branches.{branch_id}.branch_ready must be boolean false")
        for field in SAFETY_FIELDS:
            _status(branch.get(field), f"branches.{branch_id}.{field}", errors)
        evidence_sources = branch.get("evidence_sources")
        if not isinstance(evidence_sources, list) or not evidence_sources or not all(
            isinstance(relative, str) and (root / relative).is_file() for relative in evidence_sources
        ):
            errors.append(f"branches.{branch_id}.evidence_sources must reference existing source records")
        if not isinstance(branch.get("source_status"), str) or not branch["source_status"]:
            errors.append(f"branches.{branch_id}.source_status must be a non-empty string")
        if voltage is None:
            _unknown_or_not_ready(branch.get("voltage_status"), f"branches.{branch_id}.voltage_status", errors)
            _unknown_or_not_ready(branch.get("upstream_capacity_status"), f"branches.{branch_id}.upstream_capacity_status", errors)
            if continuous is not None or peak is not None:
                if continuous is None or peak is None or continuous < 0.0 or peak < continuous:
                    errors.append(f"branches.{branch_id} has invalid unknown-voltage power values")
            elif source_ids:
                errors.append(f"branches.{branch_id} cannot omit power while binding source loads")
        else:
            rail_row = _mapping(rails.get(rail), f"pre_urdf_contract.power_budget.base_user_rails.{rail}", errors)
            source_voltage = _number(rail_row.get("voltage_v"), f"pre_urdf_contract.power_budget.base_user_rails.{rail}.voltage_v", errors)
            source_current = _number(rail_row.get("current_limit_a"), f"pre_urdf_contract.power_budget.base_user_rails.{rail}.current_limit_a", errors)
            if source_voltage is None or not math.isclose(voltage, source_voltage, abs_tol=1.0e-9):
                errors.append(f"branches.{branch_id}.voltage_v does not match its upstream rail")
            if source_current is None or upstream_current is None or not math.isclose(upstream_current, source_current, abs_tol=1.0e-9):
                errors.append(f"branches.{branch_id}.upstream_current_limit_a does not match its upstream rail")
            if continuous is None or peak is None or continuous < 0.0 or peak < continuous:
                errors.append(f"branches.{branch_id} has invalid known-voltage power values")
            elif source_voltage is not None and source_current is not None and peak > source_voltage * source_current + 1.0e-6:
                errors.append(f"branches.{branch_id}.peak_w exceeds its upstream rail capacity")
        if source_ids:
            expected_continuous = expected_peak = 0.0
            values_ok = True
            for load_id in source_ids:
                load = loads.get(load_id)
                if load is None:
                    errors.append(f"branches.{branch_id} references unknown source load {load_id}")
                    values_ok = False
                    continue
                if load.get("rail") != rail:
                    errors.append(f"branches.{branch_id}.rail differs from source load {load_id}")
                source_cont = _number(load.get("continuous_w"), f"pre_urdf_contract.power_budget.loads.{load_id}.continuous_w", errors)
                source_p = _number(load.get("peak_w"), f"pre_urdf_contract.power_budget.loads.{load_id}.peak_w", errors)
                if source_cont is None or source_p is None:
                    values_ok = False
                else:
                    expected_continuous += source_cont
                    expected_peak += source_p
            if values_ok and (continuous is None or not math.isclose(continuous, expected_continuous, abs_tol=1.0e-6)):
                errors.append(f"branches.{branch_id}.continuous_w does not match source power-budget loads")
            if values_ok and (peak is None or not math.isclose(peak, expected_peak, abs_tol=1.0e-6)):
                errors.append(f"branches.{branch_id}.peak_w does not match source power-budget loads")
        if continuous is not None:
            branch_continuous += continuous
        if peak is not None:
            branch_peak += peak
        if (continuous is None) != (peak is None):
            branch_values_valid = False
    if set(assigned_load_ids) != expected_load_ids or len(assigned_load_ids) != len(set(assigned_load_ids)):
        errors.append("branches must assign every source power-budget load exactly once")
    if source_values_valid and branch_values_valid:
        if not math.isclose(branch_continuous, source_continuous, abs_tol=1.0e-6):
            errors.append("branch continuous power total does not match pre_urdf power_budget")
        if not math.isclose(branch_peak, source_peak, abs_tol=1.0e-6):
            errors.append("branch peak power total does not match pre_urdf power_budget")

    blockers = _mapping(config.get("blocking_categories"), "blocking_categories", errors)
    if set(blockers) != REQUIRED_BLOCKERS or any(value != "blocked" for value in blockers.values()):
        errors.append("blocking_categories must contain exactly every blocked electrical release category")
    return {
        "schema_version": 1,
        "status": NOT_READY,
        "ready": False,
        "valid": not errors,
        "errors": errors,
        "computed": {
            "source_total_continuous_w": source_continuous if source_values_valid else None,
            "source_total_peak_w": source_peak if source_values_valid else None,
            "branch_total_continuous_w": branch_continuous if branch_values_valid else None,
            "branch_total_peak_w": branch_peak if branch_values_valid else None,
        },
        "claim_boundary": "Validation confirms only source-budget traceability and fail-closed release status, not physical electrical acceptance.",
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = validate(args.config, root=args.root)
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    raise SystemExit(0 if result["valid"] else 2)


if __name__ == "__main__":
    main()
