#!/usr/bin/env python3
"""Fail-closed, source-only crosswalk for native-B-rep design-input coverage.

The reconstruction manifest owns the 105 project-authored part rows.  Earlier
CadQuery batches own high-level work packages, while later per-part batches
name each manifest source mesh explicitly.  This script intentionally does not infer
part-level coverage from similar names, geometry, a shared generator, or a
component-level scope paragraph.  It does not import CadQuery or create CAD
artifacts.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
from collections import Counter
from pathlib import Path
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
REPORT_ID = "tzcup_native_brep_source_coverage_audit_v1"
PENDING = "pending_native_brep_reconstruction"
EXCLUDED = "excluded_non_project_authored_vendor_reference"
EXPLICIT = "EXPLICIT_PARAMETRIC_SOURCE_COVERAGE"
HIGH_LEVEL = "HIGH_LEVEL_COMPONENT_RELATED_UNPROVEN"
UNCOVERED = "COMPLETELY_UNCOVERED"
SUPPLIER_EXCLUDED = "SUPPLIER_EXCLUDED"
PROJECT_CATEGORIES = {EXPLICIT, HIGH_LEVEL, UNCOVERED}

BATCHES = (
    {
        "id": "first",
        "contract": "config/high_fidelity_vehicle/native_brep_first_batch_contract.json",
        "source": "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_first_batch.py",
        "source_manifest": "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_first_batch_source_manifest.json",
    },
    {
        "id": "second",
        "contract": "config/high_fidelity_vehicle/native_brep_cleaning_recovery_second_batch_contract.json",
        "source": "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_cleaning_recovery_second_batch.py",
        "source_manifest": "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_cleaning_recovery_second_batch_source_manifest.json",
    },
    {
        "id": "third",
        "contract": "config/high_fidelity_vehicle/native_brep_storage_service_third_batch_contract.json",
        "source": "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_storage_service_third_batch.py",
        "source_manifest": "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_storage_service_third_batch_source_manifest.json",
    },
    {
        "id": "fourth",
        "contract": "config/high_fidelity_vehicle/native_brep_body_sensor_power_fourth_batch_contract.json",
        "source": "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_body_sensor_power_fourth_batch.py",
        "source_manifest": "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_body_sensor_power_fourth_batch_source_manifest.json",
    },
    {
        "id": "fifth_bodywork_per_part",
        "contract": "config/high_fidelity_vehicle/native_brep_bodywork_fifth_batch_contract.json",
        "source": "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_bodywork_fifth_batch.py",
        "source_manifest": "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_bodywork_fifth_batch_source_manifest.json",
    },
    {
        "id": "sixth_cleaning_per_part",
        "contract": "config/high_fidelity_vehicle/native_brep_cleaning_mechanisms_sixth_batch_contract.json",
        "source": "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_cleaning_mechanisms_sixth_batch.py",
        "source_manifest": "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_cleaning_mechanisms_sixth_batch_source_manifest.json",
    },
    {
        "id": "seventh_storage_service_per_part",
        "contract": "config/high_fidelity_vehicle/native_brep_storage_service_seventh_batch_contract.json",
        "source": "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_storage_service_seventh_batch.py",
        "source_manifest": "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_storage_service_seventh_batch_source_manifest.json",
    },
    {
        "id": "eighth_power_distribution_single_part",
        "contract": "config/high_fidelity_vehicle/native_brep_power_distribution_eighth_batch_contract.json",
        "source": "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_power_distribution_eighth_batch.py",
        "source_manifest": "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_power_distribution_eighth_batch_source_manifest.json",
    },
)

# These are intentionally package-level hints, not part mappings.  They show
# which source family could be relevant while preserving the lack of a
# contract-owned, per-manifest-part proof.
PROFILE_PACKAGE_HINTS = {
    "bodywork": (("fourth", "bodywork_access_set"),),
    "cleaning": (
        ("first", "cleaning_head_brackets"),
        ("second", "side_brush_drive"),
        ("second", "central_roller"),
        ("second", "squeegee_backing"),
        ("second", "suction_nozzle"),
        ("second", "quick_coupling"),
    ),
    "storage": (
        ("first", "storage_frame"),
        ("second", "dry_deposit_gate_chute"),
        ("second", "wastewater_tank_pan_baffles"),
        ("third", "dry_bin_shell_lid_ribs"),
        ("third", "wastewater_lid_vent_inlet"),
        ("third", "dry_bin_latch_and_toggle_triplet"),
        ("third", "level_sensor_and_probe_mounts"),
    ),
    "power_distribution": (("fourth", "power_distribution_mounting_enclosures"),),
    "charge_interface": (("third", "charge_port_interface"),),
    "wastewater_drain": (("third", "wastewater_drain_service_train"),),
}


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def _source_mesh_suffix(value: str) -> str | None:
    marker = "meshes/"
    normalized = value.replace("\\", "/")
    index = normalized.find(marker)
    if index >= 0:
        candidate = normalized[index + len(marker) :]
    elif normalized.startswith(("project/", "generated/")):
        candidate = normalized
    else:
        return None
    return candidate if candidate.endswith(".stl") else None


def _source_meshes(value: Any) -> Iterable[str]:
    if isinstance(value, dict):
        for key, nested in value.items():
            if key in {"source_mesh", "source_asset"} and isinstance(nested, str):
                suffix = _source_mesh_suffix(nested)
                if suffix:
                    yield suffix
            yield from _source_meshes(nested)
    elif isinstance(value, list):
        for nested in value:
            yield from _source_meshes(nested)
    elif isinstance(value, str):
        suffix = _source_mesh_suffix(value)
        if suffix:
            yield suffix


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _batch_state(root: Path, batch: dict[str, str]) -> dict[str, Any]:
    contract_path = root / batch["contract"]
    source_path = root / batch["source"]
    source_manifest_path = root / batch["source_manifest"]
    contract = _load_json(contract_path)
    source_manifest = _load_json(source_manifest_path)
    source_tree = ast.parse(source_path.read_text(encoding="utf-8"), filename=str(source_path))
    functions = {node.name for node in ast.walk(source_tree) if isinstance(node, ast.FunctionDef)}
    items = contract.get("items")
    parts = contract.get("parts")
    part_mappings = contract.get("part_mappings")
    singleton_part_id = contract.get("part_id")
    singleton_builder = contract.get("builder_symbol")
    if isinstance(singleton_part_id, str) and singleton_part_id:
        if not isinstance(singleton_builder, str) or not singleton_builder:
            raise ValueError(f"{batch['id']} single-part contract lacks builder_symbol")
        expected_builders = {singleton_builder}
        work_package_ids = []
        per_part_ids = [singleton_part_id]
        contract_entry_kind = "single_part"
    elif isinstance(part_mappings, list) and part_mappings:
        entry_ids = [mapping.get("manifest_part_id") for mapping in part_mappings if isinstance(mapping, dict)]
        declared_builders = [mapping.get("builder_symbol") for mapping in part_mappings if isinstance(mapping, dict)]
        if (
            len(entry_ids) != len(part_mappings)
            or not all(isinstance(entry_id, str) and entry_id for entry_id in entry_ids)
            or len(declared_builders) != len(part_mappings)
            or not all(isinstance(builder, str) and builder.startswith("_build_") for builder in declared_builders)
        ):
            raise ValueError(f"{batch['id']} contract has invalid per-part mappings")
        if len(set(entry_ids)) != len(entry_ids) or len(set(declared_builders)) != len(declared_builders):
            raise ValueError(f"{batch['id']} contract has duplicate per-part mappings")
        expected_builders = set(declared_builders)
        work_package_ids = []
        per_part_ids = list(entry_ids)
        contract_entry_kind = "per_part_mapping"
    elif isinstance(parts, list) and parts:
        entry_ids = [part.get("part_id") for part in parts if isinstance(part, dict)]
        declared_builders = [part.get("builder") for part in parts if isinstance(part, dict)]
        if (
            len(entry_ids) != len(parts)
            or not all(isinstance(entry_id, str) and entry_id for entry_id in entry_ids)
            or len(declared_builders) != len(parts)
            or not all(isinstance(builder, str) and builder.startswith("_build_") for builder in declared_builders)
        ):
            raise ValueError(f"{batch['id']} contract has invalid or duplicate per-part IDs/builders")
        if len(set(entry_ids)) != len(entry_ids) or len(set(declared_builders)) != len(declared_builders):
            raise ValueError(f"{batch['id']} contract has invalid or duplicate per-part IDs/builders")
        expected_builders = set(declared_builders)
        work_package_ids: list[str] = []
        per_part_ids = list(entry_ids)
        contract_entry_kind = "per_part"
    elif isinstance(items, list) and items:
        entry_ids = [item.get("id") for item in items if isinstance(item, dict)]
        if len(entry_ids) != len(items) or not all(isinstance(entry_id, str) and entry_id for entry_id in entry_ids):
            raise ValueError(f"{batch['id']} contract has invalid or duplicate work-package IDs")
        if len(set(entry_ids)) != len(entry_ids):
            raise ValueError(f"{batch['id']} contract has invalid or duplicate work-package IDs")
        expected_builders = {f"_build_{entry_id}" for entry_id in entry_ids}
        work_package_ids = list(entry_ids)
        per_part_ids = []
        contract_entry_kind = "work_package_or_named_part"
    else:
        raise ValueError(f"{batch['id']} contract has neither non-empty items, parts, nor part_mappings")
    manifest_entries = [*source_manifest.get("source_files", []), *source_manifest.get("design_inputs", [])]
    hashes = []
    for entry in manifest_entries:
        if not isinstance(entry, dict) or not isinstance(entry.get("path"), str) or not isinstance(entry.get("sha256"), str):
            raise ValueError(f"{batch['id']} source manifest has an invalid hash row")
        target = root / entry["path"]
        hashes.append({"path": entry["path"], "matches": target.is_file() and _sha256(target) == entry["sha256"]})
    if batch["source"] not in {entry["path"] for entry in source_manifest.get("source_files", []) if isinstance(entry, dict)}:
        raise ValueError(f"{batch['id']} source manifest does not bind the declared source")
    if batch["contract"] not in {entry["path"] for entry in source_manifest.get("design_inputs", []) if isinstance(entry, dict)}:
        raise ValueError(f"{batch['id']} source manifest does not bind the declared contract")
    return {
        "id": batch["id"],
        "contract_path": batch["contract"],
        "source_path": batch["source"],
        "source_manifest_path": batch["source_manifest"],
        "contract_status": contract.get("status"),
        "contract_entry_kind": contract_entry_kind,
        "work_package_ids": work_package_ids,
        "per_part_ids": per_part_ids,
        "all_named_builders_present": expected_builders <= functions,
        "source_integrity": hashes,
        "source_integrity_passed": bool(hashes) and all(row["matches"] for row in hashes),
        "explicit_source_meshes": sorted(set(_source_meshes(contract))),
    }


def audit(root: Path = ROOT) -> dict[str, Any]:
    """Read small JSON/Python sources only; never import or invoke CadQuery."""

    root = root.resolve()
    manifest_path = root / "config/high_fidelity_vehicle/native_brep_reconstruction_manifest.json"
    manifest = _load_json(manifest_path)
    if manifest.get("status") != PENDING:
        raise ValueError("reconstruction manifest must remain pending for this audit")
    parts = manifest.get("parts")
    excluded = manifest.get("excluded_generator_outputs")
    if not isinstance(parts, list) or len(parts) != 105 or not isinstance(excluded, list):
        raise ValueError("expected 105 project-authored parts and a vendor-exclusion list")
    if any(not isinstance(part, dict) or part.get("status") != PENDING for part in parts):
        raise ValueError("all project-authored manifest rows must remain pending")
    if any(not isinstance(item, dict) or item.get("status") != EXCLUDED for item in excluded):
        raise ValueError("vendor exclusions must retain their authoritative status")

    batches = [_batch_state(root, dict(batch)) for batch in BATCHES]
    exact_meshes: dict[str, list[str]] = {}
    for batch in batches:
        for mesh in batch["explicit_source_meshes"]:
            exact_meshes.setdefault(mesh, []).append(batch["id"])

    rows: list[dict[str, Any]] = []
    used_exact_meshes: set[str] = set()
    for part in parts:
        part_id = str(part["id"])
        mesh = str(part["source_mesh"])
        exact_batches = exact_meshes.get(mesh, [])
        hints = [
            {"batch_id": batch_id, "work_package_id": work_package_id}
            for batch_id, work_package_id in PROFILE_PACKAGE_HINTS.get(str(part["profile"]), ())
        ]
        if exact_batches:
            used_exact_meshes.add(mesh)
            category = EXPLICIT
            reason = "A batch contract names this manifest source mesh exactly; source identity and the named parametric builders are checked separately."
        elif hints:
            category = HIGH_LEVEL
            reason = "Only package/profile-level relation exists. No batch contract declares this manifest part ID or source mesh, so individual coverage is unproven."
        else:
            category = UNCOVERED
            reason = "No exact source-mesh reference or package-level relation exists in the registered batch contracts."
        rows.append(
            {
                "manifest_part_id": part_id,
                "profile": part["profile"],
                "source_generator": part["source_generator"],
                "source_mesh": mesh,
                "manifest_status": part["status"],
                "coverage_category": category,
                "exact_source_batches": exact_batches,
                "high_level_related_work_packages": hints,
                "reason": reason,
            }
        )
    # The first batch also owns project-authored platform installation inputs
    # that are deliberately outside this manifest's 105 mesh rows.  Preserve
    # that boundary as evidence instead of inventing a manifest mapping.
    external_exact_meshes = sorted(set(exact_meshes) - used_exact_meshes)
    for item in excluded:
        rows.append(
            {
                "manifest_part_id": None,
                "supplier_excluded_source_mesh": item["source_mesh"],
                "source_generator": item["source_generator"],
                "coverage_category": SUPPLIER_EXCLUDED,
                "authoritative_exclusion_status": item["status"],
                "reason": item["reason"],
            }
        )
    counts = Counter(row["coverage_category"] for row in rows)
    for category in (EXPLICIT, HIGH_LEVEL, UNCOVERED, SUPPLIER_EXCLUDED):
        counts.setdefault(category, 0)
    integrity_ok = all(batch["source_integrity_passed"] and batch["all_named_builders_present"] for batch in batches)
    project_closed = counts[HIGH_LEVEL] == 0 and counts[UNCOVERED] == 0 and integrity_ok
    return {
        "report_id": REPORT_ID,
        "audit_mode": "static_json_python_crosswalk_only",
        "execution_prohibited": ["CadQuery", "FreeCAD", "WSL", "Gazebo", "Docker", "STEP/FCStd export", "mesh conversion"],
        "manifest_path": str(manifest_path.relative_to(root)).replace("\\", "/"),
        "manifest_status_preserved": manifest["status"],
        "project_part_count": len(parts),
        "supplier_excluded_count": len(excluded),
        "batches": batches,
        "rows": rows,
        "contract_exact_meshes_outside_reconstruction_manifest": external_exact_meshes,
        "counts_by_category": dict(sorted(counts.items())),
        "status": "STATIC_INDIVIDUAL_COVERAGE_CLOSED" if project_closed else "BLOCKED_UNPROVEN_INDIVIDUAL_COVERAGE",
        "runtime_accepted": False,
        "native_cad_delivery_accepted": False,
        "boundary": "Exact source-mesh mention is the only positive per-part coverage proof. Work-package/profile relations are retained only as unproven hints and cannot close a manifest part or authorize native export.",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path, default=ROOT / "reports/engineering/native_brep_source_coverage_audit.json")
    args = parser.parse_args()
    report = audit(args.root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{report['status']}: wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
