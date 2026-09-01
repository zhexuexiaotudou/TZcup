from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "scripts" / "run_formal_vehicle_static_engineering_preflight.py"
SPEC = importlib.util.spec_from_file_location(
    "run_formal_vehicle_static_engineering_preflight",
    RUNNER_PATH,
)
assert SPEC and SPEC.loader
PREFLIGHT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREFLIGHT)


def test_current_static_preflight_is_complete_but_release_blocked() -> None:
    report = PREFLIGHT.build_report(ROOT)

    assert report["static_preflight_complete"] is True
    assert report["status"] == "STATIC_PREFLIGHT_COMPLETE_MANUFACTURING_RELEASE_BLOCKED"
    assert report["static_check_count"] == 14
    assert report["static_check_completed_count"] == 14
    assert report["manufacturing_release_ready"] is False
    assert report["native_export_ready"] is False
    assert report["deployment_ready"] is False
    assert report["native_cad"]["audit_complete"] is True
    assert report["native_cad"]["outcome"] == "blocked"
    assert report["native_cad"]["native_editable_step_assembly_ready"] is False
    assert "NO_EDITABLE_NATIVE_BREP_SOURCE" not in report["native_cad"]["blocker_codes"]
    assert "NO_BREP_STEP_ARTIFACT" in report["native_cad"]["blocker_codes"]
    assert "NATIVE_ASSEMBLY_MANIFEST_DRAFT_NOT_RELEASED" in report["native_cad"]["blocker_codes"]
    assert "NO_WINDOWS_BREP_EXPORTER" in report["native_cad"]["blocker_codes"]
    assert report["native_cad"]["warning_codes"] == [
        "LEGACY_STL_VISUAL_GENERATORS_PRESENT"
    ]
    assert report["manufacturing_preparation_draft"]["validator_complete"] is True
    assert report["manufacturing_preparation_draft"]["ready_for_manufacturing_release"] is False
    assert report["native_brep_reconstruction_manifest"]["validator_complete"] is True
    assert report["native_brep_reconstruction_manifest"]["summary"] == {
        "project_authored_parts_pending": 105,
        "excluded_vendor_reference_meshes": 21,
        "source_meshes_accounted_for": 126,
        "native_or_step_artifacts_created": 0,
    }
    assert report["native_brep_first_batch_contract"]["validator_complete"] is True
    assert report["native_brep_first_batch_contract"]["status"] == "design_input_pending_native_export"
    assert report["native_brep_first_batch_contract"]["native_cad_delivery_ready"] is False
    assert report["native_brep_first_batch_contract"]["summary"] == {
        "status": "design_input_pending_native_export",
        "first_batch_component_count": 4,
        "native_or_step_artifacts_created": 0,
        "static_only": True,
    }
    assert report["native_brep_cleaning_recovery_second_batch_contract"]["validator_complete"] is True
    assert report["native_brep_cleaning_recovery_second_batch_contract"]["status"] == "design_input_pending_native_export"
    assert report["native_brep_cleaning_recovery_second_batch_contract"]["native_cad_delivery_ready"] is False
    assert report["native_brep_cleaning_recovery_second_batch_contract"]["summary"] == {
        "status": "design_input_pending_native_export",
        "second_batch_component_count": 7,
        "source_manifest_paths_exact": True,
        "source_hash_verified": True,
        "cadquery_import_lazy": True,
        "mesh_import_absent": True,
        "native_or_step_artifacts_created": 0,
        "static_only": True,
    }
    assert report["native_brep_storage_service_third_batch_contract"]["validator_complete"] is True
    assert report["native_brep_storage_service_third_batch_contract"]["status"] == "design_input_pending_native_export"
    assert report["native_brep_storage_service_third_batch_contract"]["native_cad_delivery_ready"] is False
    assert report["native_brep_storage_service_third_batch_contract"]["summary"] == {
        "status": "design_input_pending_native_export",
        "third_batch_component_count": 6,
        "source_manifest_paths_exact": True,
        "source_hash_verified": True,
        "cadquery_import_lazy": True,
        "mesh_import_absent": True,
        "native_or_step_artifacts_created": 0,
        "static_only": True,
    }
    assert report["native_brep_body_sensor_power_fourth_batch_contract"]["validator_complete"] is True
    assert report["native_brep_body_sensor_power_fourth_batch_contract"]["status"] == "design_input_pending_native_export"
    assert report["native_brep_body_sensor_power_fourth_batch_contract"]["native_cad_delivery_ready"] is False
    assert report["native_brep_body_sensor_power_fourth_batch_contract"]["summary"] == {
        "status": "design_input_pending_native_export",
        "fourth_batch_component_count": 4,
        "source_manifest_paths_exact": True,
        "source_hash_verified": True,
        "cadquery_import_lazy": True,
        "mesh_import_absent": True,
        "native_or_step_artifacts_created": 0,
        "static_only": True,
    }
    assert report["native_brep_source_coverage"]["audit_complete"] is True
    assert report["native_brep_source_coverage"]["status"] == "STATIC_INDIVIDUAL_COVERAGE_CLOSED"
    assert report["native_brep_source_coverage"]["native_cad_delivery_ready"] is False
    assert report["native_brep_source_coverage"]["unproven_project_part_count"] == 0
    assert report["native_brep_source_coverage"]["blocker_codes"] == []
    assert report["native_brep_source_coverage"]["counts_by_category"] == {
        "COMPLETELY_UNCOVERED": 0,
        "EXPLICIT_PARAMETRIC_SOURCE_COVERAGE": 105,
        "HIGH_LEVEL_COMPONENT_RELATED_UNPROVEN": 0,
        "SUPPLIER_EXCLUDED": 21,
    }
    assembly_draft = report["component_addressable_native_cad_assembly_draft"]
    assert assembly_draft["validator_complete"] is True
    assert assembly_draft["status"] == "STATIC_COMPONENT_ADDRESSABLE_DRAFT_VALID_NATIVE_EXPORT_BLOCKED"
    assert assembly_draft["component_count"] == 105
    assert assembly_draft["supplier_excluded_count"] == 21
    assert assembly_draft["native_cad_assembly_ready"] is False
    assert assembly_draft["native_cad_delivery_accepted"] is False
    assert "NO_NATIVE_STEP_OR_FCSTD" in assembly_draft["blocker_codes"]
    release_gaps = report["per_part_native_cad_release_gaps"]
    assert release_gaps["validator_complete"] is True
    assert release_gaps["status"] == "STATIC_PER_PART_RELEASE_GAPS_VALID_NATIVE_RELEASE_BLOCKED"
    assert release_gaps["part_count"] == 105
    assert release_gaps["supplier_excluded_count"] == 21
    assert release_gaps["unresolved_gate_count"] == 64
    assert release_gaps["native_cad_release_ready"] is False
    assert release_gaps["manufacturing_release_ready"] is False
    assert "ALL_PROJECT_PARTS_HAVE_UNRESOLVED_RELEASE_GATES" in release_gaps["blocker_codes"]
    serial = report["native_cadquery_serial_export_contract"]
    assert serial["validator_complete"] is True
    assert serial["status"] == "STATIC_SERIAL_EXPORT_CONTRACT_VALID_NATIVE_EXPORT_BLOCKED"
    assert serial["source_batch_count"] == 8
    assert serial["component_addressable_count"] == 105
    assert serial["source_digest_bindings_valid"] is True
    assert serial["minimum_free_physical_memory_mib"] == 4096
    assert serial["execution"] == "strictly_serial"
    assert serial["formal_export_ready"] is False
    assert serial["cadquery_imported"] is False
    assert serial["source_modules_loaded"] is False
    board = report["s100p_formal_board_bundle"]
    assert board["validator_complete"] is True
    assert board["status"] == "BLOCKED"
    assert board["expected_blocked"] is True
    assert board["manifest_copyable"] is True
    assert board["payload_copy_authorized"] is False
    assert board["ready_to_deploy"] is False
    assert board["board_runtime_accepted"] is False
    assert board["board_operations_performed"] is False
    assert all(board["checks"].values())
    assert report["static_functional_chain"]["validator_complete"] is True
    assert report["static_functional_chain"]["status"] == "STATIC_CLOSED"
    assert report["static_functional_chain"]["required_item_count"] == 13
    assert report["static_functional_chain"]["static_closed_count"] == 13
    assert report["static_functional_chain"]["runtime_accepted"] is False
    assert report["static_functional_chain"]["fresh_gazebo_runtime_required"] is True
    assert report["static_functional_chain"]["blocked_items"] == []
    assert report["formal_mechanical_interface_datums"]["validator_complete"] is True
    assert report["formal_mechanical_interface_datums"]["status"] == "STATIC_DERIVED_SNAPSHOT_BOUND_NOT_MANUFACTURING_RELEASE"
    assert report["formal_mechanical_interface_datums"]["manufacturing_release"] is False
    assert report["formal_mechanical_interface_datums"]["summary"] == {
        "datum_count": 16,
        "interface_count": 7,
        "coordinate_reference": "base_footprint",
        "nominal_joint_configuration": "zero",
    }


def test_main_writes_machine_readable_report_without_runtime_dependencies(
    tmp_path: Path,
) -> None:
    output = tmp_path / "static_engineering_preflight.json"

    assert PREFLIGHT.main(["--root", str(ROOT), "--output", str(output)]) == 0

    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["preflight_name"] == "formal_vehicle_static_engineering_preflight"
    assert report["execution_scope"]["source_level_only"] is True
    assert report["execution_scope"]["prohibited_execution_backends"] == [
        "WSL",
        "Docker",
        "Gazebo",
        "ROS",
        "CAD exporter",
        "mesh converter",
    ]
    assert "not native CAD delivery" in report["claim_boundary"]


def test_invalid_draft_preserves_release_block_and_fails_runner(tmp_path: Path) -> None:
    invalid_draft = {
        "valid": False,
        "status": "DRAFT_DESIGN_INPUT_NOT_RELEASED",
        "ready_for_manufacturing_release": False,
        "errors": ["synthetic invalid draft"],
        "computed": {},
    }
    with patch.object(PREFLIGHT.manufacturing_draft, "validate", return_value=invalid_draft):
        report = PREFLIGHT.build_report(ROOT)
        exit_code = PREFLIGHT.main(
            ["--root", str(ROOT), "--output", str(tmp_path / "invalid.json")]
        )

    assert report["static_preflight_complete"] is False
    assert report["status"] == "STATIC_PREFLIGHT_INVALID"
    assert report["manufacturing_release_ready"] is False
    assert exit_code == 2
    assert report["manufacturing_preparation_draft"]["errors"] == [
        "synthetic invalid draft"
    ]


def test_invalid_brep_manifest_fails_the_static_preflight(tmp_path: Path) -> None:
    invalid_manifest = {
        "valid": False,
        "errors": ["synthetic incomplete reconstruction coverage"],
        "summary": {},
    }
    with patch.object(PREFLIGHT.brep_manifest, "validate", return_value=invalid_manifest):
        report = PREFLIGHT.build_report(ROOT)
        exit_code = PREFLIGHT.main(
            ["--root", str(ROOT), "--output", str(tmp_path / "invalid-brep.json")]
        )

    assert report["static_preflight_complete"] is False
    assert report["status"] == "STATIC_PREFLIGHT_INVALID"
    assert report["native_brep_reconstruction_manifest"]["validator_complete"] is False
    assert report["native_brep_reconstruction_manifest"]["errors"] == [
        "synthetic incomplete reconstruction coverage"
    ]
    assert exit_code == 2


def test_pending_contract_or_draft_datum_status_cannot_be_counted_as_complete(
    tmp_path: Path,
) -> None:
    invalid_contract = {"valid": False, "errors": ["synthetic contract drift"], "summary": {}}
    draft_datums = {
        "status": "DRAFT",
        "manufacturing_release": False,
        "datum_count": 16,
        "interface_count": 7,
        "claim_boundary": "static_zero_joint_snapshot_crosswalk_only",
    }
    with patch.object(
        PREFLIGHT.brep_first_batch,
        "validate",
        return_value=invalid_contract,
    ), patch.object(PREFLIGHT.mechanical_datums, "validate", return_value=draft_datums):
        report = PREFLIGHT.build_report(ROOT)
        exit_code = PREFLIGHT.main(
            ["--root", str(ROOT), "--output", str(tmp_path / "invalid-static-inputs.json")]
        )

    assert report["static_preflight_complete"] is False
    assert report["status"] == "STATIC_PREFLIGHT_INVALID"
    assert report["native_brep_first_batch_contract"]["validator_complete"] is False
    assert report["formal_mechanical_interface_datums"]["validator_complete"] is False
    assert exit_code == 2


def test_later_batch_or_functional_chain_drift_fails_the_static_preflight(
    tmp_path: Path,
) -> None:
    invalid_third_batch = {"valid": False, "errors": ["synthetic source hash drift"], "summary": {}}
    blocked_chain = {
        "status": "BLOCKED",
        "required_item_count": 13,
        "static_closed_count": 12,
        "runtime_accepted": False,
        "fresh_gazebo_runtime_required": True,
        "blocked_items": ["sensor_gnss"],
    }
    with patch.object(
        PREFLIGHT.brep_third_batch,
        "validate",
        return_value=invalid_third_batch,
    ), patch.object(
        PREFLIGHT.static_functional_chain,
        "audit",
        return_value=blocked_chain,
    ):
        report = PREFLIGHT.build_report(ROOT)
        exit_code = PREFLIGHT.main(
            ["--root", str(ROOT), "--output", str(tmp_path / "invalid-third-batch.json")]
        )

    assert report["static_preflight_complete"] is False
    assert report["native_brep_storage_service_third_batch_contract"]["validator_complete"] is False
    assert report["static_functional_chain"]["validator_complete"] is False
    assert report["manufacturing_release_ready"] is False
    assert exit_code == 2


def test_invalid_source_coverage_audit_fails_the_static_preflight(tmp_path: Path) -> None:
    invalid_coverage = {
        "report_id": "tzcup_native_brep_source_coverage_audit_v1",
        "status": "STATIC_INDIVIDUAL_COVERAGE_CLOSED",
        "counts_by_category": {},
        "native_cad_delivery_accepted": True,
        "runtime_accepted": False,
    }
    with patch.object(
        PREFLIGHT.brep_source_coverage,
        "audit",
        return_value=invalid_coverage,
    ):
        report = PREFLIGHT.build_report(ROOT)
        exit_code = PREFLIGHT.main(
            ["--root", str(ROOT), "--output", str(tmp_path / "invalid-source-coverage.json")]
        )

    assert report["static_preflight_complete"] is False
    assert report["native_brep_source_coverage"]["audit_complete"] is False
    assert report["native_brep_source_coverage"]["blocker_codes"] == [
        "NATIVE_BREP_SOURCE_COVERAGE_AUDIT_INVALID"
    ]
    assert exit_code == 2


def test_invalid_per_part_release_gap_register_fails_the_static_preflight(
    tmp_path: Path,
) -> None:
    invalid_register = {"valid": False, "status": "STATIC_PER_PART_RELEASE_GAPS_INVALID", "gaps": ["synthetic drift"]}
    with patch.object(PREFLIGHT.per_part_release_gaps, "audit", return_value=invalid_register):
        report = PREFLIGHT.build_report(ROOT)
        exit_code = PREFLIGHT.main(
            ["--root", str(ROOT), "--output", str(tmp_path / "invalid-release-gaps.json")]
        )

    assert report["static_preflight_complete"] is False
    assert report["per_part_native_cad_release_gaps"]["validator_complete"] is False
    assert report["manufacturing_release_ready"] is False
    assert exit_code == 2


def test_runner_does_not_spawn_a_runtime_or_cad_tool() -> None:
    source = RUNNER_PATH.read_text(encoding="utf-8")

    assert "subprocess" not in source
    assert "os.system" not in source
    assert "Popen(" not in source


def test_invalid_serial_or_s100p_bundle_fails_static_preflight() -> None:
    invalid_serial = {
        "report_id": "tzcup_native_cadquery_serial_export_contract_audit_v1",
        "status": "STATIC_SERIAL_EXPORT_CONTRACT_INVALID",
        "contract_structurally_valid": False,
        "errors": ["synthetic serial drift"],
    }
    unsafe_board = {
        "report_id": "tzcup_s100p_formal_board_bundle_validation_v1",
        "status": "BLOCKED",
        "ready_to_deploy": False,
        "manifest_copyable": True,
        "payload_copy_authorized": True,
        "board_operations_performed": False,
        "checks": {"synthetic": True},
        "blockers": sorted(PREFLIGHT.s100p_board_bundle.MANDATORY_BLOCKERS),
    }
    with patch.object(
        PREFLIGHT.cadquery_serial_contract,
        "validate",
        return_value=invalid_serial,
    ), patch.object(
        PREFLIGHT.s100p_board_bundle,
        "validate_manifest",
        return_value=unsafe_board,
    ):
        report = PREFLIGHT.build_report(ROOT)

    assert report["static_preflight_complete"] is False
    assert report["static_check_count"] == 14
    assert report["static_check_completed_count"] == 12
    assert report["native_cadquery_serial_export_contract"]["validator_complete"] is False
    assert report["s100p_formal_board_bundle"]["validator_complete"] is False
