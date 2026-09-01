#!/usr/bin/env python3
"""Fail-closed static validator for the cleaning/recovery native B-rep batch.

The validator intentionally reads only repository JSON and Python source text.
It neither imports CadQuery nor starts a CAD, mesh, ROS, Gazebo, Docker, or WSL
backend.  A successful result proves source/contract/manifest consistency only;
it never upgrades the seven design inputs to a native-export or manufacturing
release.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
STATUS = "design_input_pending_native_export"
EXPECTED_IDS = (
    "side_brush_drive",
    "central_roller",
    "squeegee_backing",
    "suction_nozzle",
    "quick_coupling",
    "dry_deposit_gate_chute",
    "wastewater_tank_pan_baffles",
)
SOURCE_RELATIVE_PATH = Path(
    "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/"
    "native_brep_cleaning_recovery_second_batch.py"
)
CONTRACT_RELATIVE_PATH = Path(
    "config/high_fidelity_vehicle/native_brep_cleaning_recovery_second_batch_contract.json"
)
MANIFEST_RELATIVE_PATH = Path(
    "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/"
    "native_brep_cleaning_recovery_second_batch_source_manifest.json"
)
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


def _repository_path(root: Path, value: Any, label: str, errors: list[str]) -> Path | None:
    if not isinstance(value, str) or not value:
        errors.append(f"{label} must be a non-empty repository-relative path")
        return None
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts:
        errors.append(f"{label} must be a repository-relative path")
        return None
    path = root / relative
    if not path.is_file():
        errors.append(f"{label} does not exist: {value}")
    return path


def _future_output_path(root: Path, value: Any, label: str, errors: list[str]) -> Path | None:
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


def _source_is_lazy_and_mesh_free(source_path: Path, errors: list[str]) -> tuple[bool, bool]:
    try:
        source = source_path.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(source_path))
    except (OSError, SyntaxError) as exc:
        errors.append(f"cannot parse editable CadQuery source: {exc}")
        return False, False

    imports: list[tuple[str, int, int]] = []

    def walk(node: ast.AST, function_depth: int = 0) -> None:
        next_depth = function_depth + isinstance(
            node, (ast.FunctionDef, ast.AsyncFunctionDef)
        )
        if isinstance(node, ast.Import):
            for alias in node.names:
                imports.append((alias.name, function_depth, node.lineno))
        elif isinstance(node, ast.ImportFrom):
            imports.append((node.module or "", function_depth, node.lineno))
        for child in ast.iter_child_nodes(node):
            walk(child, next_depth)

    walk(tree)
    cadquery_imports = [entry for entry in imports if entry[0] == "cadquery"]
    lazy = (
        len(cadquery_imports) == 1
        and cadquery_imports[0][1] >= 1
        and "def require_cadquery" in source
    )
    if not lazy:
        errors.append("CadQuery import must be lazy and confined to require_cadquery")

    forbidden_tokens = (
        "importers.importStep",
        "importMesh",
        "importStl",
        "import STL",
        ".stl",
    )
    mesh_free = not any(token.lower() in source.lower() for token in forbidden_tokens)
    if not mesh_free:
        errors.append("editable CadQuery source must not import or reconstruct a mesh")
    return lazy, mesh_free


def _validate_manifest(
    root: Path,
    manifest: dict[str, Any] | None,
    source_path: Path,
    contract_path: Path,
    errors: list[str],
) -> tuple[bool, bool]:
    if manifest is None:
        return False, False
    if manifest.get("schema_version") != 1:
        errors.append("source manifest schema_version must equal 1")
    if manifest.get("status") != STATUS:
        errors.append(f"source manifest status must remain {STATUS}")
    if manifest.get("manifest_id") != "tzcup_native_brep_cleaning_recovery_second_batch_cadquery_source_manifest_v1":
        errors.append("source manifest id is not the expected second-batch manifest")
    verification = manifest.get("verification")
    if not isinstance(verification, dict) or verification.get("algorithm") != "sha256":
        errors.append("source manifest must declare sha256 verification")
    source_entries = manifest.get("source_files")
    input_entries = manifest.get("design_inputs")
    if not isinstance(source_entries, list) or not isinstance(input_entries, list):
        errors.append("source manifest source_files and design_inputs must be arrays")
        return False, False
    entries = source_entries + input_entries
    if len(entries) != 2:
        errors.append("source manifest must bind exactly the editable source and contract")
        return False, False
    expected = {
        SOURCE_RELATIVE_PATH.as_posix(): source_path,
        CONTRACT_RELATIVE_PATH.as_posix(): contract_path,
    }
    observed_paths: set[str] = set()
    hashes_match = True
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict):
            errors.append(f"source manifest entry {index} must be an object")
            hashes_match = False
            continue
        path_text = entry.get("path")
        observed_paths.add(path_text if isinstance(path_text, str) else "")
        target = _repository_path(root, path_text, f"source manifest entry {index}.path", errors)
        if target is None or not target.is_file():
            hashes_match = False
            continue
        digest = entry.get("sha256")
        if not isinstance(digest, str) or digest != _sha256(target):
            errors.append(f"source manifest SHA-256 mismatch: {path_text}")
            hashes_match = False
    if observed_paths != set(expected):
        errors.append("source manifest must bind exactly the second-batch source and contract")
        hashes_match = False
    return hashes_match, observed_paths == set(expected)


def _validate_contract(root: Path, contract: dict[str, Any] | None, errors: list[str]) -> int:
    if contract is None:
        return 0
    if contract.get("schema_version") != 1:
        errors.append("contract schema_version must equal 1")
    if contract.get("document_id") != "tzcup_native_brep_cleaning_recovery_second_batch_parametric_contract_v1":
        errors.append("contract document_id is not the expected second-batch contract")
    if contract.get("status") != STATUS:
        errors.append(f"contract status must remain {STATUS}")
    prohibited = contract.get("prohibited_methods")
    if not isinstance(prohibited, list) or not REQUIRED_PROHIBITIONS.issubset(prohibited):
        errors.append("contract must reject mesh conversion and placeholder native artifacts")
    coordinate_system = contract.get("coordinate_system")
    if not isinstance(coordinate_system, dict) or coordinate_system.get("units") != "m_and_rad" or coordinate_system.get("root_frame") != "base_footprint":
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
        if not isinstance(item.get("geometry"), dict) or not item["geometry"]:
            errors.append(f"{item_id}.geometry must be a non-empty object")
        if not isinstance(item.get("pending_manufacturing_inputs"), list) or not item["pending_manufacturing_inputs"]:
            errors.append(f"{item_id}.pending_manufacturing_inputs must remain non-empty")
        export = item.get("planned_native_export")
        if not isinstance(export, dict):
            errors.append(f"{item_id}.planned_native_export must be an object")
            continue
        if export.get("authoritative_editable_source_path") != SOURCE_RELATIVE_PATH.as_posix():
            errors.append(f"{item_id} must name the authoritative second-batch source")
        if export.get("must_not_exist_yet") is not True:
            errors.append(f"{item_id}.planned_native_export.must_not_exist_yet must be true")
        gates = export.get("export_preconditions")
        if not isinstance(gates, list) or not gates or not all(isinstance(gate, str) and gate for gate in gates):
            errors.append(f"{item_id}.planned_native_export.export_preconditions must be non-empty")
        for field, suffix in (("optional_future_fcstd_path", ".FCStd"), ("step_path", ".step"), ("assembly_step_path", ".step")):
            target = _future_output_path(root, export.get(field), f"{item_id}.{field}", errors)
            if target is not None:
                if target.suffix != suffix:
                    errors.append(f"{item_id}.{field} must use {suffix}")
                if target.exists():
                    errors.append(f"{item_id}.{field} must not exist before controlled native export")
            if field == "assembly_step_path" and isinstance(export.get(field), str):
                assembly_steps.add(export[field])
    if tuple(ids) != EXPECTED_IDS:
        errors.append(f"contract must contain exactly the seven ordered work packages: {list(EXPECTED_IDS)}")
    if len(assembly_steps) != 1:
        errors.append("all second-batch work packages must share one future assembly STEP path")
    return len(ids)


def validate(
    root: Path = REPOSITORY_ROOT,
    contract_path: Path | None = None,
    manifest_path: Path | None = None,
    source_path: Path | None = None,
) -> dict[str, Any]:
    """Validate the pending seven-package source-bound design-input bundle."""

    root = root.resolve()
    contract_path = contract_path or root / CONTRACT_RELATIVE_PATH
    manifest_path = manifest_path or root / MANIFEST_RELATIVE_PATH
    source_path = source_path or root / SOURCE_RELATIVE_PATH
    errors: list[str] = []
    source_exists = source_path.is_file()
    if not source_exists:
        errors.append(f"editable CadQuery source does not exist: {source_path}")
    lazy, mesh_free = _source_is_lazy_and_mesh_free(source_path, errors) if source_exists else (False, False)
    contract = _load_json(contract_path, "contract", errors)
    manifest = _load_json(manifest_path, "source manifest", errors)
    component_count = _validate_contract(root, contract, errors)
    source_hash_verified, manifest_paths_exact = _validate_manifest(
        root, manifest, source_path, contract_path, errors
    )
    return {
        "valid": not errors,
        "errors": errors,
        "summary": {
            "status": STATUS,
            "second_batch_component_count": component_count,
            "source_manifest_paths_exact": manifest_paths_exact,
            "source_hash_verified": source_hash_verified,
            "cadquery_import_lazy": lazy,
            "mesh_import_absent": mesh_free,
            "native_or_step_artifacts_created": 0,
            "static_only": True,
        },
        "claim_boundary": "Static source/contract/manifest validation only; native export and manufacturing release remain blocked.",
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=REPOSITORY_ROOT)
    parser.add_argument("--contract", type=Path)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--source", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    report = validate(args.root, args.contract, args.manifest, args.source)
    rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
