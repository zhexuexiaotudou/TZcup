#!/usr/bin/env python3
"""Aggregate the Windows-safe static engineering preflights for the formal vehicle.

This runner deliberately invokes fourteen source-level checks:

* ``audit_native_cad_readiness.py`` inventories native CAD evidence without
  launching a CAD application; and
* ``validate_formal_vehicle_mechanical_manufacturing_preparation_draft.py``
  validates the fail-closed manufacturing-input draft; and
* ``validate_native_brep_reconstruction_manifest.py`` proves that every
  generated project mesh is either queued for native reconstruction or
  explicitly excluded as a vendor reference.
* ``validate_native_brep_first_batch_contract.py`` preserves the pending,
  non-delivered status of the first four native B-rep work packages; and
* ``validate_formal_mechanical_interface_datums.py`` validates the
  SHA-256-bound zero-joint mechanical interface crosswalk.
* ``validate_native_brep_cleaning_recovery_second_batch_contract.py`` validates
  the seven pending CadQuery source/contract/manifest work packages without
  loading CadQuery; and
* ``validate_native_brep_storage_service_third_batch_contract.py`` and
  ``validate_native_brep_body_sensor_power_fourth_batch_contract.py`` validate
  the remaining ten pending source-bound work packages without loading
  CadQuery; and
* ``audit_native_brep_source_coverage.py`` records exact per-manifest-part
  source-mesh proof separately from high-level package hints. Its expected
  ``BLOCKED_UNPROVEN_INDIVIDUAL_COVERAGE`` result is a visible native-CAD
  blocker, not a failed static-audit execution; and
* ``validate_component_addressable_native_cad_assembly_draft.py`` proves the
  105 project-authored rows have a source/builder address while preserving the
  missing native assembly, STEP and export-receipt hard gates; and
* ``validate_per_part_native_cad_release_gap_register.py`` proves each of the
  105 project parts has contract-sourced unresolved release work while keeping
  native and manufacturing release false; and
* ``validate_native_cadquery_serial_export_contract.py`` proves the dormant
  Windows-native route binds eight SHA-256 source batches and 105 unique parts
  while preserving the release, memory, roundtrip and no-preview gates; and
* ``validate_s100p_formal_board_bundle.py`` proves the frozen board manifest is
  internally bound and copyable while payload copying, deployment and runtime
  acceptance remain explicitly blocked; and
* ``generate_static_functional_chain_audit.py`` plus its independent validator
  requires all 13 declared source-level functional chains to close while
  explicitly preserving the fresh-Gazebo runtime boundary.

It never starts WSL, Docker, Gazebo, ROS, a CAD exporter, or a mesh converter.
Its successful exit code means that the static preflight report was produced
and the manufacturing-input draft is structurally valid.  It never means that
the vehicle has manufacturing-release approval.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
ROOT = SCRIPT_DIRECTORY.parent
DEFAULT_DRAFT_RELATIVE_PATH = Path(
    "config/high_fidelity_vehicle/mechanical_manufacturing_preparation_draft.yaml"
)
DEFAULT_BREP_MANIFEST_RELATIVE_PATH = Path(
    "config/high_fidelity_vehicle/native_brep_reconstruction_manifest.json"
)
DEFAULT_BREP_SCHEMA_RELATIVE_PATH = Path(
    "config/high_fidelity_vehicle/native_brep_reconstruction_manifest.schema.json"
)
DEFAULT_BREP_FIRST_BATCH_CONTRACT_RELATIVE_PATH = Path(
    "config/high_fidelity_vehicle/native_brep_first_batch_contract.json"
)
DEFAULT_BREP_FIRST_BATCH_SCHEMA_RELATIVE_PATH = Path(
    "config/high_fidelity_vehicle/native_brep_first_batch_contract.schema.json"
)
DEFAULT_MECHANICAL_DATUM_CROSSWALK_RELATIVE_PATH = Path(
    "config/high_fidelity_vehicle/formal_mechanical_interface_datums.yaml"
)

# This file is also imported directly by its pytest module.  Keep imports
# independent of the caller's working directory and of pytest's import mode.
if str(SCRIPT_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIRECTORY))

import audit_native_cad_readiness as native_cad  # noqa: E402
import audit_native_brep_source_coverage as brep_source_coverage  # noqa: E402
import validate_formal_mechanical_interface_datums as mechanical_datums  # noqa: E402
import validate_native_brep_first_batch_contract as brep_first_batch  # noqa: E402
import validate_native_brep_cleaning_recovery_second_batch_contract as brep_second_batch  # noqa: E402
import validate_native_brep_storage_service_third_batch_contract as brep_third_batch  # noqa: E402
import validate_native_brep_body_sensor_power_fourth_batch_contract as brep_fourth_batch  # noqa: E402
import validate_component_addressable_native_cad_assembly_draft as component_assembly_draft  # noqa: E402
import validate_per_part_native_cad_release_gap_register as per_part_release_gaps  # noqa: E402
import validate_native_cadquery_serial_export_contract as cadquery_serial_contract  # noqa: E402
import validate_s100p_formal_board_bundle as s100p_board_bundle  # noqa: E402
import validate_native_brep_reconstruction_manifest as brep_manifest  # noqa: E402
import validate_formal_vehicle_mechanical_manufacturing_preparation_draft as manufacturing_draft  # noqa: E402
import generate_static_functional_chain_audit as static_functional_chain  # noqa: E402
import validate_static_functional_chain_audit as static_functional_chain_validator  # noqa: E402


def _error_result(error: Exception) -> dict[str, Any]:
    return {
        "valid": False,
        "errors": [f"validator raised {type(error).__name__}: {error}"],
    }


def build_report(
    root: Path = ROOT,
    *,
    draft_path: Path | None = None,
) -> dict[str, Any]:
    """Return a machine-readable, release-blocked static preflight report."""

    resolved_root = root.resolve()
    resolved_draft = draft_path or (resolved_root / DEFAULT_DRAFT_RELATIVE_PATH)
    if not resolved_draft.is_absolute():
        resolved_draft = resolved_root / resolved_draft

    try:
        native_report = native_cad.build_report(resolved_root)
    except Exception as error:  # pragma: no cover - defensive report preservation
        native_report = _error_result(error)

    try:
        manufacturing_report = manufacturing_draft.validate(
            resolved_draft,
            root=resolved_root,
        )
    except Exception as error:  # pragma: no cover - defensive report preservation
        manufacturing_report = _error_result(error)

    try:
        brep_manifest_report = brep_manifest.validate(
            resolved_root,
            resolved_root / DEFAULT_BREP_MANIFEST_RELATIVE_PATH,
            resolved_root / DEFAULT_BREP_SCHEMA_RELATIVE_PATH,
        )
    except Exception as error:  # pragma: no cover - defensive report preservation
        brep_manifest_report = _error_result(error)

    try:
        brep_first_batch_report = brep_first_batch.validate(
            resolved_root,
            resolved_root / DEFAULT_BREP_FIRST_BATCH_CONTRACT_RELATIVE_PATH,
            resolved_root / DEFAULT_BREP_FIRST_BATCH_SCHEMA_RELATIVE_PATH,
        )
    except Exception as error:  # pragma: no cover - defensive report preservation
        brep_first_batch_report = _error_result(error)

    try:
        mechanical_datums_report = mechanical_datums.validate(
            resolved_root / DEFAULT_MECHANICAL_DATUM_CROSSWALK_RELATIVE_PATH,
            root=resolved_root,
        )
    except Exception as error:  # pragma: no cover - defensive report preservation
        mechanical_datums_report = _error_result(error)

    try:
        brep_second_batch_report = brep_second_batch.validate(resolved_root)
    except Exception as error:  # pragma: no cover - defensive report preservation
        brep_second_batch_report = _error_result(error)

    try:
        brep_third_batch_report = brep_third_batch.validate(resolved_root)
    except Exception as error:  # pragma: no cover - defensive report preservation
        brep_third_batch_report = _error_result(error)

    try:
        brep_fourth_batch_report = brep_fourth_batch.validate(resolved_root)
    except Exception as error:  # pragma: no cover - defensive report preservation
        brep_fourth_batch_report = _error_result(error)

    try:
        brep_source_coverage_report = brep_source_coverage.audit(resolved_root)
    except Exception as error:  # pragma: no cover - defensive report preservation
        brep_source_coverage_report = _error_result(error)

    try:
        component_assembly_draft_report = component_assembly_draft.audit(resolved_root)
    except Exception as error:  # pragma: no cover - defensive report preservation
        component_assembly_draft_report = _error_result(error)

    try:
        per_part_release_gap_report = per_part_release_gaps.audit(resolved_root)
    except Exception as error:  # pragma: no cover - defensive report preservation
        per_part_release_gap_report = _error_result(error)

    try:
        cadquery_serial_report = cadquery_serial_contract.validate(resolved_root)
    except Exception as error:  # pragma: no cover - defensive report preservation
        cadquery_serial_report = _error_result(error)

    try:
        s100p_board_bundle_report = s100p_board_bundle.validate_manifest(
            repository_root=resolved_root
        )
    except Exception as error:  # pragma: no cover - defensive report preservation
        s100p_board_bundle_report = _error_result(error)

    try:
        static_functional_chain_report = static_functional_chain.audit(resolved_root)
        static_functional_chain_validator.validate(
            static_functional_chain_report, require_static_closed=True
        )
    except Exception as error:  # pragma: no cover - defensive report preservation
        static_functional_chain_report = _error_result(error)

    native_outcome = native_report.get("outcome")
    native_audit_complete = (
        native_report.get("audit_name") == "native_cad_readiness"
        and native_outcome in {"blocked", "ready"}
        and isinstance(native_report.get("native_editable_step_assembly_ready"), bool)
    )
    manufacturing_draft_valid = manufacturing_report.get("valid") is True
    brep_manifest_valid = brep_manifest_report.get("valid") is True
    brep_first_batch_summary = brep_first_batch_report.get("summary", {})
    if not isinstance(brep_first_batch_summary, dict):
        brep_first_batch_summary = {}
    brep_first_batch_valid = (
        brep_first_batch_report.get("valid") is True
        and isinstance(brep_first_batch_summary, dict)
        and brep_first_batch_summary.get("status")
        == "design_input_pending_native_export"
        and brep_first_batch_summary.get("first_batch_component_count") == 4
        and brep_first_batch_summary.get("native_or_step_artifacts_created") == 0
        and brep_first_batch_summary.get("static_only") is True
    )
    mechanical_datums_valid = (
        mechanical_datums_report.get("status")
        == "STATIC_DERIVED_SNAPSHOT_BOUND_NOT_MANUFACTURING_RELEASE"
        and mechanical_datums_report.get("manufacturing_release") is False
        and mechanical_datums_report.get("datum_count") == 16
        and mechanical_datums_report.get("interface_count") == 7
        and mechanical_datums_report.get("claim_boundary")
        == "static_zero_joint_snapshot_crosswalk_only"
    )
    brep_second_batch_summary = brep_second_batch_report.get("summary", {})
    if not isinstance(brep_second_batch_summary, dict):
        brep_second_batch_summary = {}
    brep_second_batch_valid = (
        brep_second_batch_report.get("valid") is True
        and brep_second_batch_summary.get("status")
        == "design_input_pending_native_export"
        and brep_second_batch_summary.get("second_batch_component_count") == 7
        and brep_second_batch_summary.get("source_manifest_paths_exact") is True
        and brep_second_batch_summary.get("source_hash_verified") is True
        and brep_second_batch_summary.get("cadquery_import_lazy") is True
        and brep_second_batch_summary.get("mesh_import_absent") is True
        and brep_second_batch_summary.get("native_or_step_artifacts_created") == 0
        and brep_second_batch_summary.get("static_only") is True
    )
    brep_third_batch_summary = brep_third_batch_report.get("summary", {})
    if not isinstance(brep_third_batch_summary, dict):
        brep_third_batch_summary = {}
    brep_third_batch_valid = (
        brep_third_batch_report.get("valid") is True
        and brep_third_batch_summary.get("status") == "design_input_pending_native_export"
        and brep_third_batch_summary.get("third_batch_component_count") == 6
        and brep_third_batch_summary.get("source_manifest_paths_exact") is True
        and brep_third_batch_summary.get("source_hash_verified") is True
        and brep_third_batch_summary.get("cadquery_import_lazy") is True
        and brep_third_batch_summary.get("mesh_import_absent") is True
        and brep_third_batch_summary.get("native_or_step_artifacts_created") == 0
        and brep_third_batch_summary.get("static_only") is True
    )
    brep_fourth_batch_summary = brep_fourth_batch_report.get("summary", {})
    if not isinstance(brep_fourth_batch_summary, dict):
        brep_fourth_batch_summary = {}
    brep_fourth_batch_valid = (
        brep_fourth_batch_report.get("valid") is True
        and brep_fourth_batch_summary.get("status") == "design_input_pending_native_export"
        and brep_fourth_batch_summary.get("fourth_batch_component_count") == 4
        and brep_fourth_batch_summary.get("source_manifest_paths_exact") is True
        and brep_fourth_batch_summary.get("source_hash_verified") is True
        and brep_fourth_batch_summary.get("cadquery_import_lazy") is True
        and brep_fourth_batch_summary.get("mesh_import_absent") is True
        and brep_fourth_batch_summary.get("native_or_step_artifacts_created") == 0
        and brep_fourth_batch_summary.get("static_only") is True
    )
    coverage_counts = brep_source_coverage_report.get("counts_by_category", {})
    if not isinstance(coverage_counts, dict):
        coverage_counts = {}
    explicit_coverage_count = coverage_counts.get("EXPLICIT_PARAMETRIC_SOURCE_COVERAGE")
    high_level_unproven_count = coverage_counts.get("HIGH_LEVEL_COMPONENT_RELATED_UNPROVEN")
    completely_uncovered_count = coverage_counts.get("COMPLETELY_UNCOVERED")
    supplier_excluded_count = coverage_counts.get("SUPPLIER_EXCLUDED")
    coverage_counts_are_nonnegative_ints = all(
        type(value) is int and value >= 0
        for value in (
            explicit_coverage_count,
            high_level_unproven_count,
            completely_uncovered_count,
            supplier_excluded_count,
        )
    )
    unproven_project_part_count = (
        high_level_unproven_count + completely_uncovered_count
        if coverage_counts_are_nonnegative_ints
        else -1
    )
    expected_coverage_status = (
        "STATIC_INDIVIDUAL_COVERAGE_CLOSED"
        if unproven_project_part_count == 0
        else "BLOCKED_UNPROVEN_INDIVIDUAL_COVERAGE"
    )
    coverage_batches = brep_source_coverage_report.get("batches")
    coverage_batches_valid = (
        isinstance(coverage_batches, list)
        and len(coverage_batches) >= 4
        and all(
            isinstance(batch, dict)
            and batch.get("source_integrity_passed") is True
            and batch.get("all_named_builders_present") is True
            for batch in coverage_batches
        )
    )
    brep_source_coverage_valid = (
        brep_source_coverage_report.get("report_id")
        == "tzcup_native_brep_source_coverage_audit_v1"
        and brep_source_coverage_report.get("status") == expected_coverage_status
        and brep_source_coverage_report.get("manifest_status_preserved")
        == "pending_native_brep_reconstruction"
        and brep_source_coverage_report.get("project_part_count") == 105
        and brep_source_coverage_report.get("supplier_excluded_count") == 21
        and coverage_counts_are_nonnegative_ints
        and explicit_coverage_count + high_level_unproven_count + completely_uncovered_count == 105
        and supplier_excluded_count == 21
        and coverage_batches_valid
        and brep_source_coverage_report.get("native_cad_delivery_accepted") is False
        and brep_source_coverage_report.get("runtime_accepted") is False
    )
    component_assembly_draft_valid = (
        component_assembly_draft_report.get("report_id")
        == "tzcup_component_addressable_native_cad_assembly_draft_audit_v1"
        and component_assembly_draft_report.get("status")
        == "STATIC_COMPONENT_ADDRESSABLE_DRAFT_VALID_NATIVE_EXPORT_BLOCKED"
        and component_assembly_draft_report.get("draft_structurally_valid") is True
        and component_assembly_draft_report.get("component_count") == 105
        and component_assembly_draft_report.get("supplier_excluded_count") == 21
        and component_assembly_draft_report.get("native_cad_assembly_ready") is False
        and component_assembly_draft_report.get("native_cad_delivery_accepted") is False
        and component_assembly_draft_report.get("gaps") == []
        and all(
            code in component_assembly_draft_report.get("blockers", [])
            for code in (
                "NO_NATIVE_STEP_OR_FCSTD",
                "NO_NATIVE_CAD_EXPORT_RECEIPT",
                "NO_EXECUTED_CADQUERY_ASSEMBLY",
            )
        )
    )
    per_part_release_gaps_valid = (
        per_part_release_gap_report.get("report_id")
        == "tzcup_per_part_native_cad_release_gap_register_audit_v1"
        and per_part_release_gap_report.get("status")
        == "STATIC_PER_PART_RELEASE_GAPS_VALID_NATIVE_RELEASE_BLOCKED"
        and per_part_release_gap_report.get("valid") is True
        and per_part_release_gap_report.get("part_count") == 105
        and per_part_release_gap_report.get("supplier_excluded_count") == 21
        and per_part_release_gap_report.get("unresolved_gate_count") == 64
        and per_part_release_gap_report.get("native_cad_release_ready") is False
        and per_part_release_gap_report.get("manufacturing_release_ready") is False
        and per_part_release_gap_report.get("gaps") == []
        and "ALL_PROJECT_PARTS_HAVE_UNRESOLVED_RELEASE_GATES"
        in per_part_release_gap_report.get("blockers", [])
    )
    cadquery_serial_contract_valid = (
        cadquery_serial_report.get("report_id")
        == "tzcup_native_cadquery_serial_export_contract_audit_v1"
        and cadquery_serial_report.get("status")
        == "STATIC_SERIAL_EXPORT_CONTRACT_VALID_NATIVE_EXPORT_BLOCKED"
        and cadquery_serial_report.get("contract_structurally_valid") is True
        and cadquery_serial_report.get("source_batch_count") == 8
        and cadquery_serial_report.get("provenance_only_batch_count") == 4
        and cadquery_serial_report.get("component_addressable_batch_count") == 4
        and cadquery_serial_report.get("component_addressable_count") == 105
        and cadquery_serial_report.get("component_ids_unique") is True
        and cadquery_serial_report.get("pending_batch_contract_count") == 8
        and cadquery_serial_report.get("source_digest_bindings_valid") is True
        and cadquery_serial_report.get("minimum_free_physical_memory_mib") == 4096
        and cadquery_serial_report.get("execution") == "strictly_serial"
        and cadquery_serial_report.get("formal_export_ready") is False
        and cadquery_serial_report.get("native_cad_delivery_accepted") is False
        and cadquery_serial_report.get("cadquery_imported") is False
        and cadquery_serial_report.get("source_modules_loaded") is False
        and cadquery_serial_report.get("errors") == []
    )
    s100p_checks = s100p_board_bundle_report.get("checks", {})
    s100p_board_bundle_valid = (
        s100p_board_bundle_report.get("report_id")
        == "tzcup_s100p_formal_board_bundle_validation_v1"
        and s100p_board_bundle_report.get("status") == "BLOCKED"
        and s100p_board_bundle_report.get("ready_to_deploy") is False
        and s100p_board_bundle_report.get("manifest_copyable") is True
        and s100p_board_bundle_report.get("payload_copy_authorized") is False
        and s100p_board_bundle_report.get("board_operations_performed") is False
        and isinstance(s100p_checks, dict)
        and bool(s100p_checks)
        and all(value is True for value in s100p_checks.values())
        and all(
            blocker in s100p_board_bundle_report.get("blockers", [])
            for blocker in s100p_board_bundle.MANDATORY_BLOCKERS
        )
    )
    static_functional_chain_valid = (
        static_functional_chain_report.get("status") == "STATIC_CLOSED"
        and static_functional_chain_report.get("required_item_count") == 13
        and static_functional_chain_report.get("static_closed_count") == 13
        and static_functional_chain_report.get("runtime_accepted") is False
        and static_functional_chain_report.get("fresh_gazebo_runtime_required") is True
        and static_functional_chain_report.get("blocked_items") == []
    )
    static_preflight_complete = (
        native_audit_complete
        and manufacturing_draft_valid
        and brep_manifest_valid
        and brep_first_batch_valid
        and mechanical_datums_valid
        and brep_second_batch_valid
        and brep_third_batch_valid
        and brep_fourth_batch_valid
        and brep_source_coverage_valid
        and component_assembly_draft_valid
        and per_part_release_gaps_valid
        and cadquery_serial_contract_valid
        and s100p_board_bundle_valid
        and static_functional_chain_valid
    )
    static_check_results = (
        native_audit_complete,
        manufacturing_draft_valid,
        brep_manifest_valid,
        brep_first_batch_valid,
        mechanical_datums_valid,
        brep_second_batch_valid,
        brep_third_batch_valid,
        brep_fourth_batch_valid,
        brep_source_coverage_valid,
        component_assembly_draft_valid,
        per_part_release_gaps_valid,
        cadquery_serial_contract_valid,
        s100p_board_bundle_valid,
        static_functional_chain_valid,
    )

    gaps = native_report.get("gaps", [])
    native_blocker_codes = sorted(
        {
            gap["code"]
            for gap in gaps
            if isinstance(gap, dict)
            and gap.get("severity") == "blocker"
            and isinstance(gap.get("code"), str)
        }
    )
    native_warning_codes = sorted(
        {
            gap["code"]
            for gap in gaps
            if isinstance(gap, dict)
            and gap.get("severity") == "warning"
            and isinstance(gap.get("code"), str)
        }
    )

    return {
        "schema_version": 1,
        "preflight_name": "formal_vehicle_static_engineering_preflight",
        "static_check_count": len(static_check_results),
        "static_check_completed_count": sum(static_check_results),
        "static_preflight_complete": static_preflight_complete,
        "status": (
            "STATIC_PREFLIGHT_COMPLETE_MANUFACTURING_RELEASE_BLOCKED"
            if static_preflight_complete
            else "STATIC_PREFLIGHT_INVALID"
        ),
        "manufacturing_release_ready": False,
        "native_export_ready": False,
        "deployment_ready": False,
        "execution_scope": {
            "host": "Windows",
            "source_level_only": True,
            "prohibited_execution_backends": [
                "WSL",
                "Docker",
                "Gazebo",
                "ROS",
                "CAD exporter",
                "mesh converter",
            ],
        },
        "native_cad": {
            "audit_complete": native_audit_complete,
            "outcome": native_outcome,
            "native_editable_step_assembly_ready": native_report.get(
                "native_editable_step_assembly_ready"
            ),
            "blocker_codes": native_blocker_codes,
            "warning_codes": native_warning_codes,
            "report": native_report,
        },
        "manufacturing_preparation_draft": {
            "validator_complete": manufacturing_draft_valid,
            "status": manufacturing_report.get("status"),
            "ready_for_manufacturing_release": manufacturing_report.get(
                "ready_for_manufacturing_release"
            ),
            "errors": manufacturing_report.get("errors", []),
            "computed": manufacturing_report.get("computed", {}),
            "report": manufacturing_report,
        },
        "native_brep_reconstruction_manifest": {
            "validator_complete": brep_manifest_valid,
            "errors": brep_manifest_report.get("errors", []),
            "summary": brep_manifest_report.get("summary", {}),
            "report": brep_manifest_report,
        },
        "native_brep_first_batch_contract": {
            "validator_complete": brep_first_batch_valid,
            "status": brep_first_batch_summary.get("status"),
            "native_cad_delivery_ready": False,
            "errors": brep_first_batch_report.get("errors", []),
            "summary": brep_first_batch_summary,
            "report": brep_first_batch_report,
        },
        "native_brep_cleaning_recovery_second_batch_contract": {
            "validator_complete": brep_second_batch_valid,
            "status": brep_second_batch_summary.get("status"),
            "native_cad_delivery_ready": False,
            "errors": brep_second_batch_report.get("errors", []),
            "summary": brep_second_batch_summary,
            "report": brep_second_batch_report,
        },
        "native_brep_storage_service_third_batch_contract": {
            "validator_complete": brep_third_batch_valid,
            "status": brep_third_batch_summary.get("status"),
            "native_cad_delivery_ready": False,
            "errors": brep_third_batch_report.get("errors", []),
            "summary": brep_third_batch_summary,
            "report": brep_third_batch_report,
        },
        "native_brep_body_sensor_power_fourth_batch_contract": {
            "validator_complete": brep_fourth_batch_valid,
            "status": brep_fourth_batch_summary.get("status"),
            "native_cad_delivery_ready": False,
            "errors": brep_fourth_batch_report.get("errors", []),
            "summary": brep_fourth_batch_summary,
            "report": brep_fourth_batch_report,
        },
        "native_brep_source_coverage": {
            "audit_complete": brep_source_coverage_valid,
            "status": brep_source_coverage_report.get("status"),
            "native_cad_delivery_ready": False,
            "unproven_project_part_count": unproven_project_part_count,
            "blocker_codes": (
                [f"INDIVIDUAL_PART_COVERAGE_UNPROVEN_{unproven_project_part_count}"]
                if brep_source_coverage_valid and unproven_project_part_count > 0
                else ([] if brep_source_coverage_valid else ["NATIVE_BREP_SOURCE_COVERAGE_AUDIT_INVALID"])
            ),
            "counts_by_category": coverage_counts,
            "report": brep_source_coverage_report,
        },
        "component_addressable_native_cad_assembly_draft": {
            "validator_complete": component_assembly_draft_valid,
            "status": component_assembly_draft_report.get("status"),
            "component_count": component_assembly_draft_report.get("component_count"),
            "supplier_excluded_count": component_assembly_draft_report.get(
                "supplier_excluded_count"
            ),
            "native_cad_assembly_ready": False,
            "native_cad_delivery_accepted": False,
            "blocker_codes": component_assembly_draft_report.get("blockers", []),
            "errors": component_assembly_draft_report.get("gaps", []),
            "report": component_assembly_draft_report,
        },
        "per_part_native_cad_release_gaps": {
            "validator_complete": per_part_release_gaps_valid,
            "status": per_part_release_gap_report.get("status"),
            "part_count": per_part_release_gap_report.get("part_count"),
            "supplier_excluded_count": per_part_release_gap_report.get(
                "supplier_excluded_count"
            ),
            "unresolved_gate_count": per_part_release_gap_report.get(
                "unresolved_gate_count"
            ),
            "unresolved_gates_by_category": per_part_release_gap_report.get(
                "unresolved_gates_by_category", {}
            ),
            "native_cad_release_ready": False,
            "manufacturing_release_ready": False,
            "blocker_codes": per_part_release_gap_report.get("blockers", []),
            "errors": per_part_release_gap_report.get("gaps", []),
            "report": per_part_release_gap_report,
        },
        "native_cadquery_serial_export_contract": {
            "validator_complete": cadquery_serial_contract_valid,
            "status": cadquery_serial_report.get("status"),
            "source_batch_count": cadquery_serial_report.get("source_batch_count"),
            "component_addressable_count": cadquery_serial_report.get(
                "component_addressable_count"
            ),
            "source_digest_bindings_valid": cadquery_serial_report.get(
                "source_digest_bindings_valid"
            ),
            "minimum_free_physical_memory_mib": cadquery_serial_report.get(
                "minimum_free_physical_memory_mib"
            ),
            "execution": cadquery_serial_report.get("execution"),
            "formal_export_ready": False,
            "cadquery_imported": False,
            "source_modules_loaded": False,
            "blocker_codes": cadquery_serial_report.get("blockers", []),
            "errors": cadquery_serial_report.get("errors", []),
            "report": cadquery_serial_report,
        },
        "s100p_formal_board_bundle": {
            "validator_complete": s100p_board_bundle_valid,
            "status": s100p_board_bundle_report.get("status"),
            "expected_blocked": True,
            "manifest_copyable": s100p_board_bundle_report.get("manifest_copyable"),
            "payload_copy_authorized": False,
            "ready_to_deploy": False,
            "board_runtime_accepted": False,
            "board_operations_performed": False,
            "blocker_codes": s100p_board_bundle_report.get("blockers", []),
            "checks": s100p_checks,
            "report": s100p_board_bundle_report,
        },
        "static_functional_chain": {
            "validator_complete": static_functional_chain_valid,
            "status": static_functional_chain_report.get("status"),
            "required_item_count": static_functional_chain_report.get(
                "required_item_count"
            ),
            "static_closed_count": static_functional_chain_report.get(
                "static_closed_count"
            ),
            "runtime_accepted": False,
            "fresh_gazebo_runtime_required": static_functional_chain_report.get(
                "fresh_gazebo_runtime_required"
            ),
            "blocked_items": static_functional_chain_report.get("blocked_items", []),
            "report": static_functional_chain_report,
        },
        "formal_mechanical_interface_datums": {
            "validator_complete": mechanical_datums_valid,
            "status": mechanical_datums_report.get("status"),
            "manufacturing_release": mechanical_datums_report.get(
                "manufacturing_release"
            ),
            "errors": mechanical_datums_report.get("errors", []),
            "summary": {
                key: mechanical_datums_report.get(key)
                for key in (
                    "datum_count",
                    "interface_count",
                    "coordinate_reference",
                    "nominal_joint_configuration",
                )
            },
            "report": mechanical_datums_report,
        },
        "claim_boundary": (
            "This report proves only that the fourteen static, fail-closed checks were "
            "executed against local source inputs. It is not native CAD delivery, "
            "manufacturing approval, supplier approval, fabrication evidence, "
            "as-built evidence, or a ROS/Gazebo runtime acceptance result. The 13/13 "
            "static functional chains still require fresh Gazebo runtime evidence. "
            f"Per-part native B-rep source coverage currently leaves "
            f"{unproven_project_part_count if unproven_project_part_count >= 0 else 'an invalid number of'} "
            "project parts unproven; even zero unproven source rows does not prove "
            "a CAD-kernel export, native assembly, manufacturing release, or runtime acceptance."
        ),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT, help="repository root")
    parser.add_argument(
        "--draft",
        type=Path,
        help="manufacturing preparation draft to validate (default: repository draft)",
    )
    parser.add_argument("--output", type=Path, help="write the JSON report to this path")
    args = parser.parse_args(argv)

    report = build_report(args.root, draft_path=args.draft)
    rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return 0 if report["static_preflight_complete"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
