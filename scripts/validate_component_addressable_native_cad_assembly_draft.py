#!/usr/bin/env python3
"""Fail-closed validator for the component-addressable native-CAD draft.

This static validator reads JSON, hashes and Python ASTs only.  It never loads
CadQuery, starts a CAD application, converts a mesh, or creates FCStd/STEP.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[1]
RECONSTRUCTION = Path("config/high_fidelity_vehicle/native_brep_reconstruction_manifest.json")
DRAFT = Path("config/high_fidelity_vehicle/component_addressable_native_cad_assembly_manifest_draft.json")
PENDING = "design_input_pending_native_export"
SUPPLIER_EXCLUDED = "excluded_requires_supplier_native_cad"


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _normalise_source_mesh(value: str) -> str:
    """Align a contract's optional ``meshes/`` root with manifest-relative IDs."""

    return value.removeprefix("meshes/")


def _contract_mappings(root: Path, batches: list[dict[str, Any]]) -> tuple[dict[str, dict[str, str]], list[str]]:
    mappings: dict[str, dict[str, str]] = {}
    gaps: list[str] = []
    for batch in batches:
        batch_id = batch.get("batch_id")
        contract_text = batch.get("contract_path")
        source_text = batch.get("native_source")
        if not all(isinstance(value, str) and value for value in (batch_id, contract_text, source_text)):
            gaps.append("INVALID_REGISTERED_BATCH")
            continue
        contract_path = root / contract_text
        source_path = root / source_text
        if not contract_path.is_file() or not source_path.is_file():
            gaps.append("REGISTERED_BATCH_FILE_MISSING")
            continue
        contract = _load(contract_path)
        if contract.get("status") != PENDING:
            gaps.append("REGISTERED_BATCH_NOT_PENDING_DESIGN_INPUT")
        rows: list[Mapping[str, Any]] = []
        for key in ("parts", "items", "part_mappings"):
            value = contract.get(key)
            if isinstance(value, list):
                rows.extend(row for row in value if isinstance(row, Mapping))
        if isinstance(contract.get("part_id"), str):
            rows.append(contract)
        for row in rows:
            part_id = row.get("part_id") or row.get("manifest_part_id") or row.get("id")
            source_mesh = row.get("source_mesh")
            builder = row.get("builder") or row.get("builder_symbol")
            if contract.get("part_id") == part_id and not builder:
                builder = "build_power_distribution_box"
            elif isinstance(part_id, str) and not builder:
                builder = f"_build_{part_id}"
            if not all(isinstance(value, str) and value for value in (part_id, source_mesh, builder)):
                continue
            if part_id in mappings:
                gaps.append("DUPLICATE_REGISTERED_CONTRACT_PART")
                continue
            mappings[part_id] = {"batch_id": batch_id, "contract_path": contract_text, "native_source": source_text, "source_mesh": _normalise_source_mesh(source_mesh), "builder_symbol": builder}
    return mappings, gaps


def _source_manifest_binds(root: Path, source_text: str, contract_text: str) -> bool:
    directory = root / "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle"
    source_path = root / source_text
    contract_path = root / contract_text
    for path in directory.glob("*_source_manifest.json"):
        try:
            manifest = _load(path)
        except (OSError, json.JSONDecodeError, ValueError):
            continue
        source_rows = manifest.get("source_files")
        input_rows = manifest.get("design_inputs")
        if not isinstance(source_rows, list) or not isinstance(input_rows, list):
            continue
        source_ok = any(isinstance(row, Mapping) and row.get("path") == source_text and row.get("sha256") == _sha256(source_path) for row in source_rows)
        contract_ok = any(isinstance(row, Mapping) and row.get("path") == contract_text and row.get("sha256") == _sha256(contract_path) for row in input_rows)
        if source_ok and contract_ok:
            return True
    return False


def validate(payload: Mapping[str, Any], root: Path) -> list[dict[str, str]]:
    gaps: list[dict[str, str]] = []
    if payload.get("status") != PENDING:
        gaps.append({"code": "DRAFT_STATUS_NOT_PENDING", "severity": "blocker"})
    components = payload.get("components")
    exclusions = payload.get("supplier_excluded_components")
    batches = payload.get("registered_batches")
    if not isinstance(components, list) or not isinstance(exclusions, list) or not isinstance(batches, list):
        return [{"code": "DRAFT_SCHEMA_INVALID", "severity": "blocker"}]
    reconstruction = _load(root / RECONSTRUCTION)
    expected_parts = {row["id"]: row for row in reconstruction["parts"]}
    expected_exclusions = {row["source_mesh"]: row for row in reconstruction["excluded_generator_outputs"]}
    mappings, mapping_codes = _contract_mappings(root, [row for row in batches if isinstance(row, dict)])
    gaps.extend({"code": code, "severity": "blocker"} for code in mapping_codes)
    if payload.get("component_count") != len(expected_parts) or len(components) != len(expected_parts):
        gaps.append({"code": "PROJECT_COMPONENT_COUNT_UNPROVEN", "severity": "blocker"})
    seen: set[str] = set()
    ast_cache: dict[str, set[str]] = {}
    provenance_seen: set[tuple[str, str]] = set()
    for row in components:
        if not isinstance(row, Mapping):
            gaps.append({"code": "INVALID_COMPONENT_ENTRY", "severity": "blocker"})
            continue
        part_id = row.get("manifest_part_id")
        if not isinstance(part_id, str) or part_id in seen or part_id not in expected_parts:
            gaps.append({"code": "PROJECT_COMPONENT_ID_UNPROVEN", "severity": "blocker"})
            continue
        seen.add(part_id)
        expected = expected_parts[part_id]
        registered = mappings.get(part_id)
        required = ("source_mesh", "profile", "batch_id", "contract_path", "native_source", "builder_symbol")
        if any(not isinstance(row.get(key), str) or not row[key] for key in required):
            gaps.append({"code": "COMPONENT_ADDRESS_UNPROVEN", "severity": "blocker", "detail": part_id})
            continue
        if row["source_mesh"] != expected["source_mesh"] or row["profile"] != expected["profile"] or row.get("status") != PENDING or row.get("must_not_exist_yet") is not True:
            gaps.append({"code": "COMPONENT_MANIFEST_DRIFT", "severity": "blocker", "detail": part_id})
        if registered != {key: row[key] for key in ("batch_id", "contract_path", "native_source", "source_mesh", "builder_symbol")}:
            gaps.append({"code": "REGISTERED_BATCH_MAPPING_MISMATCH", "severity": "blocker", "detail": part_id})
            continue
        source_text, contract_text, builder = row["native_source"], row["contract_path"], row["builder_symbol"]
        source_path = root / source_text
        if source_text not in ast_cache:
            try:
                tree = ast.parse(source_path.read_text(encoding="utf-8"), filename=str(source_path))
                ast_cache[source_text] = {node.name for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}
            except (OSError, UnicodeDecodeError, SyntaxError):
                ast_cache[source_text] = set()
        if builder not in ast_cache[source_text]:
            gaps.append({"code": "COMPONENT_BUILDER_UNPROVEN", "severity": "blocker", "detail": part_id})
        provenance_seen.add((source_text, contract_text))
    if seen != set(expected_parts):
        gaps.append({"code": "PROJECT_COMPONENT_SET_INCOMPLETE", "severity": "blocker"})
    for source_text, contract_text in provenance_seen:
        if not _source_manifest_binds(root, source_text, contract_text):
            gaps.append({"code": "SOURCE_OR_CONTRACT_HASH_UNPROVEN", "severity": "blocker", "detail": source_text})
    if payload.get("supplier_excluded_count") != len(expected_exclusions) or len(exclusions) != len(expected_exclusions):
        gaps.append({"code": "SUPPLIER_EXCLUSION_COUNT_UNPROVEN", "severity": "blocker"})
    seen_exclusions = set()
    for row in exclusions:
        if not isinstance(row, Mapping) or row.get("status") != SUPPLIER_EXCLUDED or not isinstance(row.get("supplier_cad_requirement"), str):
            gaps.append({"code": "SUPPLIER_EXCLUSION_INVALID", "severity": "blocker"})
            continue
        mesh = row.get("source_mesh")
        if not isinstance(mesh, str) or mesh not in expected_exclusions or mesh in seen_exclusions:
            gaps.append({"code": "SUPPLIER_EXCLUSION_DRIFT", "severity": "blocker"})
            continue
        seen_exclusions.add(mesh)
    if seen_exclusions != set(expected_exclusions):
        gaps.append({"code": "SUPPLIER_EXCLUSION_SET_INCOMPLETE", "severity": "blocker"})
    if payload.get("native_cad_assembly_ready") is not False or payload.get("step_or_fcstd_created") is not False or payload.get("export_receipt_created") is not False:
        gaps.append({"code": "DRAFT_FALSE_RELEASE_CLAIM", "severity": "blocker"})
    return gaps


def audit(root: Path = ROOT) -> dict[str, Any]:
    payload = _load(root / DRAFT)
    gaps = validate(payload, root)
    valid = not gaps
    blockers = ["NO_NATIVE_STEP_OR_FCSTD", "NO_NATIVE_CAD_EXPORT_RECEIPT", "NO_EXECUTED_CADQUERY_ASSEMBLY", "ALL_COMPONENTS_PENDING_NATIVE_EXPORT"]
    return {"report_id": "tzcup_component_addressable_native_cad_assembly_draft_audit_v1", "audit_mode": "static_json_hash_ast_only", "status": "STATIC_COMPONENT_ADDRESSABLE_DRAFT_VALID_NATIVE_EXPORT_BLOCKED" if valid else "STATIC_COMPONENT_ADDRESSABLE_DRAFT_INVALID", "draft_structurally_valid": valid, "component_count": payload.get("component_count"), "supplier_excluded_count": payload.get("supplier_excluded_count"), "native_cad_assembly_ready": False, "native_cad_delivery_accepted": False, "blockers": blockers, "gaps": gaps, "execution_prohibited": ["CadQuery", "FreeCAD", "WSL", "Gazebo", "Docker", "STEP/FCStd export", "mesh conversion"]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = audit(args.root.resolve())
    rendered = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return 0 if report["draft_structurally_valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
