#!/usr/bin/env python3
"""Fail-closed validation for the formal vehicle manufacturing-input draft.

The draft is intentionally useful before native CAD and hardware exist, while
making a release claim mechanically impossible.  It validates source
traceability and preparation coverage; it does not validate fabrication.
"""

from __future__ import annotations

import argparse
import csv
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DRAFT = ROOT / "config/high_fidelity_vehicle/mechanical_manufacturing_preparation_draft.yaml"
NOT_READY = "NOT_READY_FOR_MECHANICAL_MANUFACTURING_RELEASE"
DRAFT_STATUS = "DRAFT_DESIGN_INPUT_NOT_RELEASED"
REQUIRED_MATERIAL_IDS = {
    "structural_project_load_paths",
    "wet_waste_and_recovery_path",
    "exterior_and_service_covers",
    "sensor_and_compute_mounting",
}
REQUIRED_FASTENER_IDS = {
    "sensor_tower_base",
    "arm_pedestal_and_adapter",
    "compute_enclosure_mount",
    "service_door_hinges_and_latches",
    "cleaning_head_and_recovery_mounts",
}
REQUIRED_CONNECTION_IDS = {
    "sensor_tower_primary_load_path",
    "arm_pedestal_primary_load_path",
    "wet_storage_and_recovery_path",
    "bodywork_and_service_interfaces",
}
REQUIRED_SURFACE_IDS = {
    "exposed_structural_metal",
    "wet_and_washdown_interfaces",
    "exterior_polymer_and_painted_panels",
}
REQUIRED_ASSEMBLY_IDS = {
    "incoming_and_traceability",
    "payload_frame_and_primary_load_paths",
    "wet_dry_storage_and_recovery",
    "cleaning_head",
    "electrical_compute_and_sensor_mounts",
    "bodywork_and_service_access",
    "final_configuration_control",
}
REQUIRED_INSPECTION_IDS = {
    "incoming_material_and_purchased_component",
    "datum_geometry_and_joint_stackup",
    "fastener_and_connection_control",
    "weld_bond_or_seal_process",
    "coating_and_surface",
    "final_as_built",
}
REQUIRED_MAINTENANCE_IDS = {
    "service_doors_and_power_isolation",
    "dry_and_wet_storage_service",
    "cleaning_head_service",
    "sensor_arm_and_compute_service",
}
REQUIRED_HOLD_POINTS = {
    "native_step_or_stp_models",
    "controlled_2d_drawings",
    "gdt_datums_and_tolerances",
    "material_specifications_and_certificates",
    "fastener_size_grade_torque_and_preload",
    "qualified_connections_welds_bonds_and_seals",
    "coating_corrosion_and_surface_treatment",
    "approved_assembly_work_instructions",
    "approved_inspection_plan_and_records",
    "as_built_weighing_cog_and_inertia",
    "structural_fea_fatigue_and_stability",
    "ingress_and_waterproofing_test",
    "maintenance_access_drawings",
}
REQUIRED_PREPARATION_AREAS = {
    "material_requirement_drafts",
    "preliminary_fastener_schedule",
    "connection_and_weld_decision_inputs",
    "surface_protection_requirement_drafts",
    "assembly_process_hold_points",
    "inspection_plan_inputs",
    "maintenance_drawing_inputs",
}
REQUIRED_SUBASSEMBLIES = {
    "sensor_tower",
    "manipulator",
    "compute_enclosure",
    "bodywork_service_access",
    "cleaning_head_deployment",
    "wastewater_storage",
    "wastewater_recovery_drive",
    "recovery_squeegee",
    "dry_storage",
    "robot_deposition_port",
    "side_brushes",
    "central_roller",
    "wrist_rgbd_installation",
}


def _mapping(value: Any, label: str, errors: list[str]) -> dict[str, Any]:
    if not isinstance(value, dict):
        errors.append(f"{label} must be a mapping")
        return {}
    return value


def _non_empty_string(value: Any, label: str, errors: list[str]) -> str:
    if not isinstance(value, str) or not value.strip():
        errors.append(f"{label} must be a non-empty string")
        return ""
    return value


def _records_by_id(
    section: dict[str, Any],
    label: str,
    required_ids: set[str],
    errors: list[str],
    *,
    list_key: str = "records",
) -> dict[str, dict[str, Any]]:
    if section.get("release_status") != "draft_design_input_not_released":
        errors.append(f"{label}.release_status must remain draft_design_input_not_released")
    records = section.get(list_key)
    if not isinstance(records, list):
        errors.append(f"{label}.{list_key} must be a list")
        return {}
    result: dict[str, dict[str, Any]] = {}
    for index, value in enumerate(records):
        record = _mapping(value, f"{label}.{list_key}[{index}]", errors)
        record_id = _non_empty_string(record.get("id"), f"{label}.{list_key}[{index}].id", errors)
        if record_id in result:
            errors.append(f"{label} contains duplicate id: {record_id}")
        result[record_id] = record
    if set(result) != required_ids:
        errors.append(f"{label} ids must equal {sorted(required_ids)}")
    return result


def _pending(value: Any, label: str, errors: list[str]) -> None:
    text = _non_empty_string(value, label, errors)
    if text and not text.startswith("pending://"):
        errors.append(f"{label} must retain a pending:// release boundary")


def _load_yaml(path: Path, label: str, errors: list[str]) -> dict[str, Any]:
    try:
        return _mapping(yaml.safe_load(path.read_text(encoding="utf-8")), label, errors)
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        errors.append(f"cannot read {label}: {exc}")
        return {}


def _source_path(source: dict[str, Any], key: str, root: Path, errors: list[str]) -> Path:
    entry = _mapping(source.get(key), f"source_design_inputs.{key}", errors)
    text = _non_empty_string(entry.get("path"), f"source_design_inputs.{key}.path", errors)
    path = root / text
    if text and not path.is_file():
        errors.append(f"source_design_inputs.{key}.path is missing: {text}")
    return path


def _validate_sources(draft: dict[str, Any], root: Path, errors: list[str]) -> dict[str, Any]:
    source = _mapping(draft.get("source_design_inputs"), "source_design_inputs", errors)
    expected_keys = {
        "urdf_xacro",
        "layout",
        "component_register",
        "manufacturing_bom",
        "mechanical_release_readiness",
    }
    if set(source) != expected_keys:
        errors.append("source_design_inputs must contain exactly every authoritative design input")

    urdf_path = _source_path(source, "urdf_xacro", root, errors)
    try:
        ET.parse(urdf_path)
    except (OSError, ET.ParseError) as exc:
        errors.append(f"URDF/Xacro design input must be XML parseable: {exc}")

    layout_entry = _mapping(source.get("layout"), "source_design_inputs.layout", errors)
    layout = _load_yaml(_source_path(source, "layout", root, errors), "formal vehicle layout", errors)
    if layout.get("layout_id") != layout_entry.get("expected_layout_id"):
        errors.append("layout_id does not match the draft's expected layout input")

    register_entry = _mapping(source.get("component_register"), "source_design_inputs.component_register", errors)
    register = _load_yaml(_source_path(source, "component_register", root, errors), "formal vehicle component register", errors)
    if register.get("register_id") != register_entry.get("expected_register_id"):
        errors.append("component register id does not match the draft's expected register input")
    actual_subassemblies = {
        record.get("id")
        for record in register.get("mechanical_subassemblies", [])
        if isinstance(record, dict)
    }
    missing = REQUIRED_SUBASSEMBLIES - actual_subassemblies
    if missing:
        errors.append(f"component register is missing preparation-critical subassemblies: {sorted(missing)}")

    bom_entry = _mapping(source.get("manufacturing_bom"), "source_design_inputs.manufacturing_bom", errors)
    bom_path = _source_path(source, "manufacturing_bom", root, errors)
    try:
        with bom_path.open("r", encoding="utf-8", newline="") as handle:
            bom_rows = list(csv.DictReader(handle))
        if len(bom_rows) != bom_entry.get("expected_row_count"):
            errors.append("manufacturing BOM row count does not match the frozen draft input")
        if any(row.get("manufacturing_release_state") == "released" for row in bom_rows):
            errors.append("manufacturing BOM must not contain a released row while this draft is active")
    except (OSError, UnicodeError, csv.Error) as exc:
        errors.append(f"cannot read manufacturing BOM: {exc}")
        bom_rows = []

    readiness_entry = _mapping(source.get("mechanical_release_readiness"), "source_design_inputs.mechanical_release_readiness", errors)
    readiness = _load_yaml(
        _source_path(source, "mechanical_release_readiness", root, errors),
        "mechanical release readiness",
        errors,
    )
    if readiness.get("status") != readiness_entry.get("expected_status"):
        errors.append("mechanical release readiness status must remain not ready")
    if readiness.get("ready") is not False:
        errors.append("mechanical release readiness must remain false")
    return {"bom_row_count": len(bom_rows), "subassembly_count": len(actual_subassemblies)}


def validate(draft_path: Path = DEFAULT_DRAFT, *, root: Path = ROOT) -> dict[str, Any]:
    errors: list[str] = []
    draft = _load_yaml(draft_path, "mechanical manufacturing preparation draft", errors)
    if draft.get("schema_version") != 1 or isinstance(draft.get("schema_version"), bool):
        errors.append("schema_version must equal integer 1")
    if draft.get("status") != DRAFT_STATUS:
        errors.append(f"status must equal {DRAFT_STATUS}")
    if draft.get("ready_for_manufacturing_release") is not False:
        errors.append("ready_for_manufacturing_release must be boolean false")
    prohibited = draft.get("prohibited_claims")
    if not isinstance(prohibited, list) or set(prohibited) != {
        "released_for_fabrication",
        "ready_for_manufacturing",
        "as_built_verified",
        "supplier_approved",
        "inspection_accepted",
    }:
        errors.append("prohibited_claims must preserve every release prohibition")

    audit = _mapping(draft.get("audit_outcome"), "audit_outcome", errors)
    if audit.get("status") != "PRE_RELEASE_DESIGN_INPUT_DRAFT_COMPLETE_RELEASE_BLOCKED":
        errors.append("audit_outcome.status must preserve the pre-release blocked boundary")
    if audit.get("release_evidence_created") is not False:
        errors.append("audit_outcome.release_evidence_created must be boolean false")
    areas = audit.get("completed_design_input_areas")
    if not isinstance(areas, list) or set(areas) != REQUIRED_PREPARATION_AREAS:
        errors.append("audit_outcome must cover exactly every pre-release preparation area")
    if audit.get("blocked_until") != "required_release_hold_points_are_closed_by_controlled_evidence":
        errors.append("audit_outcome.blocked_until must preserve controlled-evidence release gating")

    computed = _validate_sources(draft, root, errors)

    materials = _records_by_id(_mapping(draft.get("materials"), "materials", errors), "materials", REQUIRED_MATERIAL_IDS, errors)
    for record_id, record in materials.items():
        if not isinstance(record.get("material_family_candidates"), list) or not record["material_family_candidates"]:
            errors.append(f"materials.{record_id} must name candidate material families")
        _pending(record.get("release_evidence"), f"materials.{record_id}.release_evidence", errors)

    fasteners = _records_by_id(_mapping(draft.get("fastener_schedule"), "fastener_schedule", errors), "fastener_schedule", REQUIRED_FASTENER_IDS, errors)
    fastener_rules = _mapping(draft.get("fastener_schedule"), "fastener_schedule", errors).get("common_rules", {})
    if _mapping(fastener_rules, "fastener_schedule.common_rules", errors).get("metric_thread_system_only") is not True:
        errors.append("fastener_schedule must preserve metric_thread_system_only")
    if _mapping(fastener_rules, "fastener_schedule.common_rules", errors).get("do_not_tighten_from_this_draft") is not True:
        errors.append("fastener_schedule must forbid tightening from the draft")
    _pending(_mapping(fastener_rules, "fastener_schedule.common_rules", errors).get("final_thread_size_grade_finish_torque_and_preload"), "fastener_schedule.common_rules.final_thread_size_grade_finish_torque_and_preload", errors)
    for record_id, record in fasteners.items():
        _non_empty_string(record.get("component_register_subassembly"), f"fastener_schedule.{record_id}.component_register_subassembly", errors)
        _pending(record.get("release_torque_or_preload"), f"fastener_schedule.{record_id}.release_torque_or_preload", errors)
        _pending(record.get("verification"), f"fastener_schedule.{record_id}.verification", errors)

    connections = _records_by_id(_mapping(draft.get("connections_and_welds"), "connections_and_welds", errors), "connections_and_welds", REQUIRED_CONNECTION_IDS, errors)
    for record_id, record in connections.items():
        _non_empty_string(record.get("connection_family"), f"connections_and_welds.{record_id}.connection_family", errors)
        _pending(record.get("release_evidence"), f"connections_and_welds.{record_id}.release_evidence", errors)

    surfaces = _records_by_id(_mapping(draft.get("surface_treatment"), "surface_treatment", errors), "surface_treatment", REQUIRED_SURFACE_IDS, errors)
    for record_id, record in surfaces.items():
        _pending(record.get("acceptance_criteria"), f"surface_treatment.{record_id}.acceptance_criteria", errors)

    assembly = _mapping(draft.get("assembly_process"), "assembly_process", errors)
    assembly_records = _records_by_id(
        assembly,
        "assembly_process",
        REQUIRED_ASSEMBLY_IDS,
        errors,
        list_key="work_packages",
    )
    _pending(assembly.get("precondition_for_any_build"), "assembly_process.precondition_for_any_build", errors)
    sequences = [record.get("sequence") for record in assembly_records.values()]
    if sorted(sequences) != [10, 20, 30, 40, 50, 60, 70]:
        errors.append("assembly work packages must retain the ordered conceptual sequence")
    for record_id, record in assembly_records.items():
        _pending(record.get("hold_point"), f"assembly_process.{record_id}.hold_point", errors)

    inspections = _records_by_id(_mapping(draft.get("inspection_plan_inputs"), "inspection_plan_inputs", errors), "inspection_plan_inputs", REQUIRED_INSPECTION_IDS, errors)
    for record_id, record in inspections.items():
        _pending(record.get("acceptance_criteria"), f"inspection_plan_inputs.{record_id}.acceptance_criteria", errors)

    maintenance = _records_by_id(_mapping(draft.get("maintenance_drawing_inputs"), "maintenance_drawing_inputs", errors), "maintenance_drawing_inputs", REQUIRED_MAINTENANCE_IDS, errors)
    for record_id, record in maintenance.items():
        _pending(record.get("release_evidence"), f"maintenance_drawing_inputs.{record_id}.release_evidence", errors)

    hold_points = draft.get("required_release_hold_points")
    if not isinstance(hold_points, list) or set(hold_points) != REQUIRED_HOLD_POINTS:
        errors.append("required_release_hold_points must preserve every blocked release gate")

    return {
        "schema_version": 1,
        "status": DRAFT_STATUS,
        "ready_for_manufacturing_release": False,
        "valid": not errors,
        "errors": errors,
        "computed": computed,
        "claim_boundary": "Validation proves only a complete, fail-closed draft input package; it is not manufacturing approval.",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--draft", type=Path, default=DEFAULT_DRAFT)
    parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args()
    result = validate(args.draft, root=args.root)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if result["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
