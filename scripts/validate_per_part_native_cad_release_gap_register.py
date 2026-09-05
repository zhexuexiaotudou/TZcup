#!/usr/bin/env python3
"""Fail-closed static validator for per-part native-CAD release gaps."""
from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[1]
DRAFT = Path("config/high_fidelity_vehicle/component_addressable_native_cad_assembly_manifest_draft.json")
REGISTER = Path("config/high_fidelity_vehicle/per_part_native_cad_release_gap_register.json")
PENDING = "design_input_pending_native_export"
EXPECTED_CONTRACTS = {
    "first": "config/high_fidelity_vehicle/native_brep_first_batch_contract.json",
    "second": "config/high_fidelity_vehicle/native_brep_cleaning_recovery_second_batch_contract.json",
    "third": "config/high_fidelity_vehicle/native_brep_storage_service_third_batch_contract.json",
    "fourth": "config/high_fidelity_vehicle/native_brep_body_sensor_power_fourth_batch_contract.json",
    "fifth_bodywork_per_part": "config/high_fidelity_vehicle/native_brep_bodywork_fifth_batch_contract.json",
    "sixth_cleaning_per_part": "config/high_fidelity_vehicle/native_brep_cleaning_mechanisms_sixth_batch_contract.json",
    "seventh_storage_service_per_part": "config/high_fidelity_vehicle/native_brep_storage_service_seventh_batch_contract.json",
    "eighth_power_distribution": "config/high_fidelity_vehicle/native_brep_power_distribution_eighth_batch_contract.json",
}


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def validate(payload: Mapping[str, Any], root: Path = ROOT) -> list[dict[str, str]]:
    gaps: list[dict[str, str]] = []
    draft = _load(root / DRAFT)
    expected_parts = {row["manifest_part_id"]: row for row in draft["components"]}
    if payload.get("status") != PENDING or payload.get("source_assembly_draft") != DRAFT.as_posix():
        gaps.append({"code": "REGISTER_STATUS_OR_SOURCE_DRAFT_INVALID", "severity": "blocker"})
    contracts = payload.get("source_contracts")
    if not isinstance(contracts, Mapping) or set(contracts) != set(EXPECTED_CONTRACTS):
        gaps.append({"code": "EIGHT_CONTRACT_SOURCES_UNPROVEN", "severity": "blocker"})
    else:
        for name, path in EXPECTED_CONTRACTS.items():
            row = contracts[name]
            if not isinstance(row, Mapping) or row.get("contract_path") != path or row.get("status") != PENDING:
                gaps.append({"code": "CONTRACT_CONTEXT_DRIFT", "severity": "blocker", "detail": name})
            elif not (root / path).is_file():
                gaps.append({"code": "CONTRACT_CONTEXT_FILE_MISSING", "severity": "blocker", "detail": name})
    definitions = payload.get("category_definitions")
    catalog = payload.get("gate_catalog")
    if not isinstance(definitions, Mapping) or not isinstance(catalog, list) or not catalog:
        return [*gaps, {"code": "GATE_CATALOG_INVALID", "severity": "blocker"}]
    catalog_by_id: dict[str, Mapping[str, Any]] = {}
    for gate in catalog:
        if not isinstance(gate, Mapping) or not isinstance(gate.get("gate_id"), str) or gate["gate_id"] in catalog_by_id:
            gaps.append({"code": "GATE_ID_INVALID_OR_DUPLICATE", "severity": "blocker"})
            continue
        catalog_by_id[gate["gate_id"]] = gate
        if gate.get("status") != "unresolved" or gate.get("category") not in definitions or not isinstance(gate.get("unresolved_input"), str) or not gate["unresolved_input"].strip():
            gaps.append({"code": "GATE_CONTENT_UNPROVEN", "severity": "blocker", "detail": gate["gate_id"]})
        if gate.get("source_contract") not in EXPECTED_CONTRACTS.values():
            gaps.append({"code": "GATE_SOURCE_CONTRACT_UNPROVEN", "severity": "blocker", "detail": gate["gate_id"]})
    parts = payload.get("parts")
    if payload.get("part_count") != 105 or not isinstance(parts, list) or len(parts) != 105:
        gaps.append({"code": "PROJECT_PART_COUNT_UNPROVEN", "severity": "blocker"})
        parts = []
    seen: set[str] = set()
    for row in parts:
        if not isinstance(row, Mapping) or not isinstance(row.get("manifest_part_id"), str):
            gaps.append({"code": "PART_ENTRY_INVALID", "severity": "blocker"})
            continue
        part_id = row["manifest_part_id"]
        expected = expected_parts.get(part_id)
        if expected is None or part_id in seen:
            gaps.append({"code": "PART_ID_UNPROVEN", "severity": "blocker", "detail": part_id})
            continue
        seen.add(part_id)
        for key in ("profile", "source_mesh", "native_source", "builder_symbol"):
            if row.get(key) != expected.get(key):
                gaps.append({"code": "PART_ASSEMBLY_DRAFT_DRIFT", "severity": "blocker", "detail": part_id})
                break
        if row.get("primary_contract") != expected.get("contract_path"):
            gaps.append({"code": "PART_ASSEMBLY_DRAFT_DRIFT", "severity": "blocker", "detail": part_id})
        gate_ids = row.get("unresolved_gate_ids")
        if not isinstance(gate_ids, list) or len(gate_ids) < 2 or any(gate_id not in catalog_by_id for gate_id in gate_ids):
            gaps.append({"code": "PART_RELEASE_GATES_UNPROVEN", "severity": "blocker", "detail": part_id})
        if row.get("primary_contract") not in EXPECTED_CONTRACTS.values():
            gaps.append({"code": "PART_SOURCE_EVIDENCE_UNPROVEN", "severity": "blocker", "detail": part_id})
    if seen != set(expected_parts):
        gaps.append({"code": "PROJECT_PART_SET_INCOMPLETE", "severity": "blocker"})
    supplier = payload.get("supplier_excluded_components")
    if payload.get("supplier_excluded_count") != 21 or supplier != draft.get("supplier_excluded_components"):
        gaps.append({"code": "SUPPLIER_EXCLUSION_DRIFT", "severity": "blocker"})
    if payload.get("native_cad_release_ready") is not False or payload.get("manufacturing_release_ready") is not False:
        gaps.append({"code": "FALSE_RELEASE_CLAIM", "severity": "blocker"})
    return gaps


def audit(root: Path = ROOT) -> dict[str, Any]:
    payload = _load(root / REGISTER)
    gaps = validate(payload, root)
    categories = Counter(
        gate.get("category") for gate in payload.get("gate_catalog", []) if isinstance(gate, Mapping)
    )
    valid = not gaps
    return {
        "report_id": "tzcup_per_part_native_cad_release_gap_register_audit_v1",
        "audit_mode": "static_json_only",
        "status": "STATIC_PER_PART_RELEASE_GAPS_VALID_NATIVE_RELEASE_BLOCKED" if valid else "STATIC_PER_PART_RELEASE_GAPS_INVALID",
        "valid": valid,
        "part_count": payload.get("part_count"),
        "supplier_excluded_count": payload.get("supplier_excluded_count"),
        "unresolved_gate_count": len(payload.get("gate_catalog", [])),
        "unresolved_gates_by_category": dict(sorted(categories.items())),
        "native_cad_release_ready": False,
        "manufacturing_release_ready": False,
        "blockers": ["ALL_PROJECT_PARTS_HAVE_UNRESOLVED_RELEASE_GATES", "NO_NATIVE_STEP_OR_FCSTD", "NO_NATIVE_CAD_EXPORT_RECEIPT"],
        "gaps": gaps,
        "execution_prohibited": ["CadQuery", "FreeCAD", "WSL", "Gazebo", "Docker", "STEP/FCStd export", "mesh conversion"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args()
    report = audit(args.root.resolve())
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
