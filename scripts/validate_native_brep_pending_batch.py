#!/usr/bin/env python3
"""Shared static validator for pending, source-bound CadQuery B-rep batches.

This module reads only repository JSON, hashes, and Python AST/source text.  It
never imports CadQuery, creates an export directory, or starts a CAD, WSL,
Docker, Gazebo, ROS, mesh converter, or STEP exporter.  A valid result proves
that a *pending* design-input batch is internally consistent; it cannot be
used as native CAD, FCStd/STEP, assembly, or manufacturing-release evidence.
"""

from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path
from typing import Any


STATUS = "design_input_pending_native_export"
REQUIRED_PROHIBITIONS = {
    "mesh_to_step_conversion",
    "faceted_or_tessellated_step_export",
    "mesh_import_as_native_brep_substitute",
    "placeholder_fcstd_or_step_artifact",
}


def _load_json(path: Path, label: str, errors: list[str]) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        errors.append(f"cannot read {label}: {exc}")
        return None
    if not isinstance(value, dict):
        errors.append(f"{label} root must be an object")
        return None
    return value


def _relative_file(root: Path, value: Any, label: str, errors: list[str]) -> Path | None:
    if not isinstance(value, str) or not value:
        errors.append(f"{label} must be a non-empty repository-relative path")
        return None
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts:
        errors.append(f"{label} must be a repository-relative path")
        return None
    return root / relative


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _validate_source(path: Path, errors: list[str]) -> tuple[bool, bool]:
    try:
        source = path.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(path))
    except (OSError, SyntaxError) as exc:
        errors.append(f"cannot parse editable CadQuery source: {exc}")
        return False, False

    imports: list[tuple[str, int]] = []

    def walk(node: ast.AST, function_depth: int = 0) -> None:
        next_depth = function_depth + isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        if isinstance(node, ast.Import):
            imports.extend((alias.name, function_depth) for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.append((node.module or "", function_depth))
        for child in ast.iter_child_nodes(node):
            walk(child, next_depth)

    walk(tree)
    cadquery_imports = [entry for entry in imports if entry[0] == "cadquery"]
    lazy = len(cadquery_imports) == 1 and cadquery_imports[0][1] >= 1 and "def require_cadquery" in source
    if not lazy:
        errors.append("CadQuery import must be lazy and confined to require_cadquery")
    forbidden = ("importers.importstep", "importmesh", "importstl", "import stl", ".stl")
    mesh_free = not any(token in source.lower() for token in forbidden)
    if not mesh_free:
        errors.append("editable CadQuery source must not import or reconstruct a mesh")
    return lazy, mesh_free


def _validate_manifest(
    root: Path,
    manifest: dict[str, Any] | None,
    *,
    manifest_id: str,
    source_relative: Path,
    contract_relative: Path,
    errors: list[str],
) -> tuple[bool, bool]:
    if manifest is None:
        return False, False
    if manifest.get("schema_version") != 1:
        errors.append("source manifest schema_version must equal 1")
    if manifest.get("manifest_id") != manifest_id:
        errors.append("source manifest id is not the expected batch manifest")
    if manifest.get("status") != STATUS:
        errors.append(f"source manifest status must remain {STATUS}")
    if not isinstance(manifest.get("verification"), dict) or manifest["verification"].get("algorithm") != "sha256":
        errors.append("source manifest must declare sha256 verification")

    source_files, design_inputs = manifest.get("source_files"), manifest.get("design_inputs")
    if not isinstance(source_files, list) or not isinstance(design_inputs, list):
        errors.append("source manifest source_files and design_inputs must be arrays")
        return False, False
    entries = [*source_files, *design_inputs]
    expected = {source_relative.as_posix(), contract_relative.as_posix()}
    observed: set[str] = set()
    hashes_match = True
    if len(entries) != 2:
        errors.append("source manifest must bind exactly the editable source and contract")
        hashes_match = False
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict):
            errors.append(f"source manifest entry {index} must be an object")
            hashes_match = False
            continue
        path_text = entry.get("path")
        if isinstance(path_text, str):
            observed.add(path_text)
        target = _relative_file(root, path_text, f"source manifest entry {index}.path", errors)
        if target is None or not target.is_file():
            if target is not None:
                errors.append(f"source manifest input does not exist: {path_text}")
            hashes_match = False
            continue
        digest = entry.get("sha256")
        if not isinstance(digest, str) or digest != _sha256(target):
            errors.append(f"source manifest SHA-256 mismatch: {path_text}")
            hashes_match = False
    paths_exact = observed == expected
    if not paths_exact:
        errors.append("source manifest must bind exactly this batch source and contract")
    return hashes_match and paths_exact, paths_exact


def _validate_contract(
    root: Path,
    contract: dict[str, Any] | None,
    *,
    document_id: str,
    expected_ids: tuple[str, ...],
    source_relative: Path,
    require_shared_assembly_step: bool,
    errors: list[str],
) -> int:
    if contract is None:
        return 0
    if contract.get("schema_version") != 1:
        errors.append("contract schema_version must equal 1")
    if contract.get("document_id") != document_id:
        errors.append("contract document_id is not the expected batch contract")
    if contract.get("status") != STATUS:
        errors.append(f"contract status must remain {STATUS}")
    if not isinstance(contract.get("claim_boundary"), str) or not contract["claim_boundary"].strip():
        errors.append("contract claim_boundary must be non-empty")
    prohibited = contract.get("prohibited_methods")
    if not isinstance(prohibited, list) or not REQUIRED_PROHIBITIONS.issubset(prohibited):
        errors.append("contract must reject mesh conversion and placeholder native artifacts")
    coordinates = contract.get("coordinate_system")
    if not isinstance(coordinates, dict) or coordinates.get("units") != "m_and_rad" or coordinates.get("root_frame") != "base_footprint":
        errors.append("contract must retain the m_and_rad base_footprint coordinate system")

    items = contract.get("items")
    if not isinstance(items, list):
        errors.append("contract items must be an array")
        return 0
    ids: list[str] = []
    assembly_steps: set[str] = set()
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            errors.append(f"items[{index}] must be an object")
            continue
        item_id = item.get("id")
        if not isinstance(item_id, str):
            errors.append(f"items[{index}].id must be a string")
            continue
        ids.append(item_id)
        if item.get("status") != STATUS:
            errors.append(f"{item_id}.status must remain {STATUS}")
        if not isinstance(item.get("scope"), str) or not item["scope"].strip():
            errors.append(f"{item_id}.scope must be non-empty")
        if not isinstance(item.get("geometry"), dict) or not item["geometry"]:
            errors.append(f"{item_id}.geometry must be a non-empty object")
        if not isinstance(item.get("pending_manufacturing_inputs"), list) or not item["pending_manufacturing_inputs"]:
            errors.append(f"{item_id}.pending_manufacturing_inputs must remain non-empty")
        export = item.get("planned_native_export")
        if not isinstance(export, dict):
            errors.append(f"{item_id}.planned_native_export must be an object")
            continue
        if export.get("authoritative_editable_source_path") != source_relative.as_posix():
            errors.append(f"{item_id} must name the authoritative batch source")
        if export.get("must_not_exist_yet") is not True:
            errors.append(f"{item_id}.planned_native_export.must_not_exist_yet must be true")
        gates = export.get("export_preconditions")
        if not isinstance(gates, list) or not gates or not all(isinstance(gate, str) and gate for gate in gates):
            errors.append(f"{item_id}.planned_native_export.export_preconditions must be non-empty")
        for field, suffix in (("optional_future_fcstd_path", ".FCStd"), ("step_path", ".step")):
            output = _relative_file(root, export.get(field), f"{item_id}.{field}", errors)
            if output is None:
                continue
            if output.suffix != suffix:
                errors.append(f"{item_id}.{field} must use {suffix}")
            if output.exists():
                errors.append(f"{item_id}.{field} must not exist before controlled native export")
        assembly = export.get("assembly_step_path")
        if require_shared_assembly_step:
            output = _relative_file(root, assembly, f"{item_id}.assembly_step_path", errors)
            if output is not None:
                if output.suffix != ".step":
                    errors.append(f"{item_id}.assembly_step_path must use .step")
                if output.exists():
                    errors.append(f"{item_id}.assembly_step_path must not exist before controlled native export")
            if isinstance(assembly, str):
                assembly_steps.add(assembly)
    if tuple(ids) != expected_ids:
        errors.append(f"contract must contain exactly the ordered work packages: {list(expected_ids)}")
    if require_shared_assembly_step and len(assembly_steps) != 1:
        errors.append("all batch work packages must share one future assembly STEP path")
    return len(ids)


def validate_batch(
    root: Path,
    *,
    contract_relative: Path,
    manifest_relative: Path,
    source_relative: Path,
    document_id: str,
    manifest_id: str,
    expected_ids: tuple[str, ...],
    summary_count_key: str,
    require_shared_assembly_step: bool,
    contract_path: Path | None = None,
    manifest_path: Path | None = None,
    source_path: Path | None = None,
) -> dict[str, Any]:
    """Validate one pending batch without executing its CAD source."""

    root = root.resolve()
    contract_path = contract_path or root / contract_relative
    manifest_path = manifest_path or root / manifest_relative
    source_path = source_path or root / source_relative
    errors: list[str] = []
    source_exists = source_path.is_file()
    if not source_exists:
        errors.append(f"editable CadQuery source does not exist: {source_path}")
    lazy, mesh_free = _validate_source(source_path, errors) if source_exists else (False, False)
    contract = _load_json(contract_path, "contract", errors)
    manifest = _load_json(manifest_path, "source manifest", errors)
    component_count = _validate_contract(
        root, contract, document_id=document_id, expected_ids=expected_ids,
        source_relative=source_relative, require_shared_assembly_step=require_shared_assembly_step, errors=errors,
    )
    source_hash_verified, manifest_paths_exact = _validate_manifest(
        root, manifest, manifest_id=manifest_id, source_relative=source_relative,
        contract_relative=contract_relative, errors=errors,
    )
    return {
        "valid": not errors,
        "errors": errors,
        "summary": {
            "status": STATUS,
            summary_count_key: component_count,
            "source_manifest_paths_exact": manifest_paths_exact,
            "source_hash_verified": source_hash_verified,
            "cadquery_import_lazy": lazy,
            "mesh_import_absent": mesh_free,
            "native_or_step_artifacts_created": 0,
            "static_only": True,
        },
        "claim_boundary": "Static source/contract/manifest validation only; native export, assembly receipt, and manufacturing release remain blocked.",
    }
