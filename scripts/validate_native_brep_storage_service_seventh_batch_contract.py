#!/usr/bin/env python3
"""Validate seventh-batch storage/service per-part source coverage without CAD."""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
from collections import Counter
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
CONTRACT = Path("config/high_fidelity_vehicle/native_brep_storage_service_seventh_batch_contract.json")
SOURCE = Path("starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_storage_service_seventh_batch.py")
SOURCE_MANIFEST = Path("starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_storage_service_seventh_batch_source_manifest.json")
RECONSTRUCTION_MANIFEST = Path("config/high_fidelity_vehicle/native_brep_reconstruction_manifest.json")
STATUS = "design_input_pending_native_export"
DOCUMENT_ID = "tzcup_native_brep_storage_service_seventh_batch_per_part_mapping_v1"
MANIFEST_ID = "tzcup_native_brep_storage_service_seventh_batch_source_manifest_v1"


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"object required: {path}")
    return value


def _hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate(root: Path = ROOT, contract_path: Path | None = None, source_path: Path | None = None, source_manifest_path: Path | None = None) -> dict[str, Any]:
    root = root.resolve()
    contract_path = contract_path or root / CONTRACT
    source_path = source_path or root / SOURCE
    source_manifest_path = source_manifest_path or root / SOURCE_MANIFEST
    errors: list[str] = []
    contract = _json(contract_path)
    reconstruction = _json(root / RECONSTRUCTION_MANIFEST)
    source_manifest = _json(source_manifest_path)

    if contract.get("document_id") != DOCUMENT_ID or contract.get("status") != STATUS:
        errors.append("seventh-batch contract identity/status is invalid")
    if source_manifest.get("manifest_id") != MANIFEST_ID or source_manifest.get("status") != STATUS:
        errors.append("seventh-batch source manifest identity/status is invalid")

    expected = [
        row for row in reconstruction.get("parts", [])
        if isinstance(row, dict) and ("/storage/" in str(row.get("target_native_source")) or "/service/" in str(row.get("target_native_source")))
    ]
    expected_by_id = {row["id"]: row for row in expected}
    expected_storage = {row["id"] for row in expected if "/storage/" in row["target_native_source"]}
    expected_service = {row["id"] for row in expected if "/service/" in row["target_native_source"]}
    mappings = contract.get("part_mappings")
    if not isinstance(mappings, list):
        errors.append("part_mappings must be a list")
        mappings = []
    ids = [row.get("manifest_part_id") for row in mappings if isinstance(row, dict)]
    meshes = [row.get("source_mesh") for row in mappings if isinstance(row, dict)]
    builders = [row.get("builder_symbol") for row in mappings if isinstance(row, dict)]
    if len(mappings) != 34 or len(expected_storage) != 24 or len(expected_service) != 10:
        errors.append("expected exactly 24 storage and 10 service mappings")
    if len(ids) != len(set(ids)) or set(ids) != set(expected_by_id):
        errors.append("mapping IDs must be unique and exactly match reconstruction storage/service rows")
    if len(meshes) != len(set(meshes)):
        errors.append("source_mesh values must be unique")
    if len(builders) != len(set(builders)):
        errors.append("builder_symbol values must be unique")
    for mapping in mappings:
        if not isinstance(mapping, dict):
            errors.append("mapping row must be an object")
            continue
        part_id = mapping.get("manifest_part_id")
        expected_row = expected_by_id.get(part_id)
        if expected_row is None:
            continue
        if mapping.get("source_mesh") != expected_row["source_mesh"] or mapping.get("source_generator") != expected_row["source_generator"]:
            errors.append(f"{part_id} must retain exact reconstruction source identity")
        if mapping.get("status") != STATUS or mapping.get("must_not_exist_yet") is not True:
            errors.append(f"{part_id} must remain pending with no native artifact")
        if not isinstance(mapping.get("assembly_intent"), str) or not mapping["assembly_intent"]:
            errors.append(f"{part_id} must describe discrete assembly intent")
        primitives = mapping.get("geometry_primitives")
        if not isinstance(primitives, list) or len(primitives) < 2:
            errors.append(f"{part_id} requires multiple parametric primitives; fused-box substitution is forbidden")
        pending = mapping.get("pending_release_inputs")
        if not isinstance(pending, list) or not pending:
            errors.append(f"{part_id} must retain manufacturing release blockers")

    semantics = contract.get("storage_semantics", {})
    dry = semantics.get("dry_compartment", {}) if isinstance(semantics, dict) else {}
    wet = semantics.get("wet_compartment", {}) if isinstance(semantics, dict) else {}
    if dry.get("separate_from") != "wet_compartment" or wet.get("separate_from") != "dry_compartment":
        errors.append("dry and wet compartments must remain explicitly separate")
    if dry.get("separation_part") != "storage_dry_wet_partition" or wet.get("separation_part") != "storage_dry_wet_partition":
        errors.append("dry/wet separator must be an explicit mapped part")
    if "cardboard/PP/PET/aluminum" not in str(dry.get("mass_increment_interface")):
        errors.append("dry garbage mass interface must name all four material classes")
    if "Recovered standing water" not in str(wet.get("mass_increment_interface")):
        errors.append("wet recovery mass increment interface is required")
    service = contract.get("service_assembly_semantics", {})
    if not isinstance(service, dict) or len(service.get("charge_interface", [])) != 4 or len(service.get("drain_interface", [])) != 6:
        errors.append("service contract must retain 4 charge + 6 drain separate assembly roles")
    if "fused" not in str(service.get("rule", "")).lower():
        errors.append("service fused-box prohibition is required")
    acceptance = contract.get("acceptance")
    if not isinstance(acceptance, dict) or any(value is not False for value in acceptance.values()):
        errors.append("all acceptance flags must remain false")

    if not source_path.is_file():
        errors.append("seventh-batch source missing")
        functions: set[str] = set()
        top_level_cadquery_import = True
    else:
        tree = ast.parse(source_path.read_text(encoding="utf-8"), filename=str(source_path))
        functions = {node.name for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}
        top_level_cadquery_import = any(
            isinstance(node, (ast.Import, ast.ImportFrom)) and any(alias.name == "cadquery" for alias in node.names)
            for node in tree.body
        )
    if top_level_cadquery_import:
        errors.append("CadQuery import must stay lazy")
    for builder in builders:
        if isinstance(builder, str) and builder not in functions:
            errors.append(f"mapped builder missing from source AST: {builder}")

    source_entries = source_manifest.get("source_files", [])
    input_entries = source_manifest.get("design_inputs", [])
    source_row = next((row for row in source_entries if isinstance(row, dict) and row.get("path") == SOURCE.as_posix()), None)
    input_row = next((row for row in input_entries if isinstance(row, dict) and row.get("path") == CONTRACT.as_posix()), None)
    if not isinstance(source_row, dict) or source_row.get("sha256") != _hash(source_path):
        errors.append("source manifest hash must bind exact seventh-batch source")
    if not isinstance(input_row, dict) or input_row.get("sha256") != _hash(contract_path):
        errors.append("source manifest hash must bind exact seventh-batch contract")

    counts = Counter("storage" if mapping.get("manifest_part_id") in expected_storage else "service" for mapping in mappings if isinstance(mapping, dict))
    return {
        "valid": not errors,
        "errors": errors,
        "summary": {
            "storage_part_count": counts["storage"],
            "service_part_count": counts["service"],
            "total_exact_source_mesh_mappings": len(mappings),
            "source_hash_verified": not any("source manifest hash" in error for error in errors),
            "cadquery_executed": False,
            "native_or_step_artifacts_created": 0,
            "runtime_accepted": False,
        },
        "boundary": "Static JSON/hash/AST validation only. Manufacturing and native/export/runtime gates remain blocked.",
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--contract", type=Path)
    parser.add_argument("--source", type=Path)
    parser.add_argument("--source-manifest", type=Path)
    args = parser.parse_args(argv)
    report = validate(args.root, args.contract, args.source, args.source_manifest)
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

