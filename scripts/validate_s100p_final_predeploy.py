#!/usr/bin/env python3
"""Single fail-closed, local-only final gate for an RDK S100P deployment.

The audit reads local JSON and hashes local files only.  It neither contacts a
board nor creates, transfers, installs, or executes a payload.  A deployment
receipt is evidence produced by a separately authorized operator; this module
only verifies a retained receipt and will not synthesize one.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

import validate_s100p_formal_board_bundle as board_bundle
import validate_s100p_mechanical_electrical_evidence as mechanical_electrical
import validate_s100p_offline_predeploy as offline_predeploy
from validate_dosod_s100p_hbm_compile_contract import validate_contract_shape


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BOARD_BUNDLE = ROOT / "config" / "s100p_formal_board_bundle_manifest.json"
DEFAULT_OFFLINE_PLAN = ROOT / "config" / "s100p_offline_predeploy_plan.json"
DEFAULT_HBM_CONTRACT = ROOT / "config" / "dosod_s100p_hbm_compile_contract.json"
DEFAULT_MECHANICAL_ELECTRICAL = ROOT / "config" / "high_fidelity_vehicle" / "s100p_mechanical_electrical_evidence.json"
DEFAULT_SNAPSHOT = ROOT / "reports" / "engineering" / "formal_vehicle_snapshot_manifest.json"
DEFAULT_SESSION = ROOT / "artifacts" / "formal_final_acceptance_session.json"
DEFAULT_RUNTIME_BINDING = ROOT / "reports" / "engineering" / "formal_vehicle_runtime_report.json.runtime_binding.json"
DEFAULT_RECEIPT_ROOT = ROOT / "artifacts" / "s100p_formal_predeploy_receipts"

RECEIPTS = {
    "dosod_hbm_compile": "dosod_hbm_compile_receipt.json",
    "model_payload": "model_payload_receipt.json",
    "overlay_build": "overlay_build_receipt.json",
    "runtime_dependencies": "runtime_dependencies_receipt.json",
    "thermal_power": "thermal_power_receipt.json",
}
EXPECTED_PAYLOAD_PATHS = {
    "dosod_hbm": "dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm",
    "dosod_vocabulary": "dosod/tzcup_offline_vocabulary.json",
    "edgesam_encoder_hbm": "edgesam/edgesam_encoder_512.hbm",
    "edgesam_decoder_hbm": "edgesam/edgesam_decoder_512.hbm",
}
EXPECTED_OVERLAY_PACKAGES = {"sanitation_perception", "sanitation_perception_interfaces"}
EXPECTED_DEPENDENCIES = set(board_bundle.SANITATION_PERCEPTION_EXEC_DEPENDENCIES) | {
    "hobot_dosod", "mono_edgesam"
}
REQUIRED_BOARD_STATIC_CHECKS = {
    "manifest_parseable", "manifest_identity_valid", "copy_boundary_fail_closed",
    "snapshot_binding_declared", "formal_snapshot_file_matches_declaration",
    "formal_snapshot_content_matches_declaration", "bound_source_roles_exact",
    "bound_source_digests_valid", "semantic_configuration_parseable",
    "required_board_payload_roles_exact", "payload_roles_match_product_artifact_bundle",
    "payload_roles_match_launch_parameter_record", "launch_binds_each_required_board_payload_role",
    "overlay_runtime_dependency_closure_classified", "deployment_gates_explicitly_unaccepted",
    "mandatory_blockers_declared",
}
REQUIRED_OFFLINE_STATIC_CHECKS = {
    "offline_predeploy_plan_parseable", "operation_boundary_exact", "plan_input_keys_exact",
    "bundle_validator_completed", "overlay_inventory_identity_valid", "overlay_package_sources_valid",
    "overlay_runtime_package_set_valid", "launch_parameter_record_identity_valid",
    "launch_parameter_path_roles_valid", "launch_source_contract_valid", "formal_resource_gate_valid",
    "future_operator_plan_recorded", "rollback_plan_recorded", "central_acceptance_unchanged",
    "validator_has_no_board_or_network_implementation",
}


def _append(blockers: list[str], value: str) -> None:
    if value not in blockers:
        blockers.append(value)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_object(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    try:
        if not path.is_file():
            return None, "missing"
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        return None, f"invalid:{type(exc).__name__}"
    return (value, None) if isinstance(value, dict) else (None, "not_object")


def _is_digest(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(char in "0123456789abcdef" for char in value)


def _identity_from_snapshot(snapshot_path: Path, blockers: list[str]) -> dict[str, Any] | None:
    snapshot, error = _load_object(snapshot_path)
    if error:
        _append(blockers, f"pc_snapshot_{error}")
        return None
    assert snapshot is not None
    outputs = snapshot.get("outputs")
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf") if isinstance(outputs, Mapping) else None
    source = snapshot.get("source_inventory_sha256")
    output = snapshot.get("output_inventory_sha256")
    expanded = urdf.get("sha256") if isinstance(urdf, Mapping) else None
    if not all(_is_digest(value) for value in (source, output, expanded)):
        _append(blockers, "pc_snapshot_identity_incomplete")
        return None
    return {
        "snapshot_manifest_sha256": _sha256(snapshot_path),
        "source_inventory_sha256": source,
        "expanded_urdf_sha256": expanded,
        "snapshot_output_inventory_sha256": output,
    }


def _validate_session_identity(
    *,
    board_manifest: Mapping[str, Any] | None,
    snapshot_path: Path,
    session_path: Path,
    runtime_binding_path: Path,
    blockers: list[str],
) -> tuple[dict[str, bool], dict[str, Any]]:
    checks = {
        "pc_snapshot_matches_board_bundle": False,
        "acceptance_session_present_and_running": False,
        "runtime_binding_present_and_bound": False,
        "pc_session_runtime_closure_identity_exact": False,
    }
    details: dict[str, Any] = {
        "snapshot_path": str(snapshot_path),
        "session_path": str(session_path),
        "runtime_binding_path": str(runtime_binding_path),
    }
    snapshot_identity = _identity_from_snapshot(snapshot_path, blockers)
    formal_snapshot = board_manifest.get("formal_snapshot") if isinstance(board_manifest, Mapping) else None
    if snapshot_identity and isinstance(formal_snapshot, Mapping):
        checks["pc_snapshot_matches_board_bundle"] = (
            formal_snapshot.get("sha256") == snapshot_identity["snapshot_manifest_sha256"]
            and formal_snapshot.get("source_inventory_sha256") == snapshot_identity["source_inventory_sha256"]
            and formal_snapshot.get("output_inventory_sha256") == snapshot_identity["snapshot_output_inventory_sha256"]
            and formal_snapshot.get("formal_urdf_sha256") == snapshot_identity["expanded_urdf_sha256"]
        )
    if not checks["pc_snapshot_matches_board_bundle"]:
        _append(blockers, "pc_snapshot_does_not_match_board_bundle")

    session, session_error = _load_object(session_path)
    if session_error:
        _append(blockers, f"acceptance_session_{session_error}")
    elif snapshot_identity:
        assert session is not None
        expected_session_snapshot = {
            key: snapshot_identity[key]
            for key in ("snapshot_manifest_sha256", "source_inventory_sha256", "expanded_urdf_sha256")
        }
        checks["acceptance_session_present_and_running"] = (
            session.get("status") == "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
            and session.get("snapshot") == expected_session_snapshot
            and isinstance(session.get("started_epoch_ns"), int)
            and session["started_epoch_ns"] > 0
            and isinstance(session.get("runtime_closure_binding"), Mapping)
        )
        if not checks["acceptance_session_present_and_running"]:
            _append(blockers, "acceptance_session_not_current_running_closure_bound")

    binding, binding_error = _load_object(runtime_binding_path)
    if binding_error:
        _append(blockers, f"runtime_gate_binding_{binding_error}")
    else:
        assert binding is not None
        session_binding = binding.get("acceptance_session_binding")
        closure_binding = binding.get("runtime_closure_binding")
        checks["runtime_binding_present_and_bound"] = (
            binding.get("schema_version") == 1
            and binding.get("status") == "FORMAL_RUNTIME_GATE_BOUND"
            and isinstance(session_binding, Mapping)
            and isinstance(closure_binding, Mapping)
            and closure_binding.get("status") == "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"
            and isinstance(closure_binding.get("runtime_install_root"), str)
            and bool(closure_binding["runtime_install_root"])
        )
        if not checks["runtime_binding_present_and_bound"]:
            _append(blockers, "runtime_gate_binding_incomplete_or_unbound")
        if session is not None and snapshot_identity and isinstance(session_binding, Mapping) and isinstance(closure_binding, Mapping):
            session_snapshot = {
                key: snapshot_identity[key]
                for key in ("snapshot_manifest_sha256", "source_inventory_sha256", "expanded_urdf_sha256")
            }
            checks["pc_session_runtime_closure_identity_exact"] = (
                session_binding.get("session_manifest_sha256") == _sha256(session_path)
                and session_binding.get("session_started_epoch_ns") == session.get("started_epoch_ns")
                and session_binding.get("session_status_at_gate") == session.get("status")
                and session_binding.get("snapshot") == session_snapshot
                and session_binding.get("snapshot_output_inventory_sha256") == snapshot_identity["snapshot_output_inventory_sha256"]
                and session.get("runtime_closure_binding") == closure_binding
            )
        if not checks["pc_session_runtime_closure_identity_exact"]:
            _append(blockers, "pc_session_runtime_closure_identity_mismatch_or_legacy_binding")
    details["snapshot_identity"] = snapshot_identity
    return checks, details


def _validate_receipt_identity(
    receipt: Mapping[str, Any], session: Mapping[str, Any], closure: Mapping[str, Any]
) -> bool:
    binding = receipt.get("acceptance_session_binding")
    return (
        receipt.get("schema_version") == 1
        and receipt.get("status") == "VERIFIED"
        and receipt.get("board_interaction_performed") is True
        and isinstance(binding, Mapping)
        and binding.get("session_manifest_sha256") == session.get("session_manifest_sha256")
        and binding.get("session_started_epoch_ns") == session.get("session_started_epoch_ns")
        and binding.get("snapshot") == session.get("snapshot")
        and receipt.get("runtime_closure_binding") == closure
    )


def _validate_receipts(
    receipt_root: Path, runtime_binding_path: Path, blockers: list[str]
) -> tuple[dict[str, bool], dict[str, Any]]:
    checks = {f"{name}_receipt_valid": False for name in RECEIPTS}
    details: dict[str, Any] = {"receipt_root": str(receipt_root), "receipts": {}}
    runtime_binding, binding_error = _load_object(runtime_binding_path)
    if binding_error:
        for name in RECEIPTS:
            _append(blockers, f"{name}_receipt_cannot_bind_runtime_identity")
        return checks, details
    assert runtime_binding is not None
    session = runtime_binding.get("acceptance_session_binding")
    closure = runtime_binding.get("runtime_closure_binding")
    if not isinstance(session, Mapping) or not isinstance(closure, Mapping):
        for name in RECEIPTS:
            _append(blockers, f"{name}_receipt_cannot_bind_runtime_identity")
        return checks, details

    loaded: dict[str, Mapping[str, Any]] = {}
    for name, filename in RECEIPTS.items():
        path = receipt_root / filename
        receipt, error = _load_object(path)
        details["receipts"][name] = {"path": str(path), "present": error is None}
        if error:
            _append(blockers, f"{name}_receipt_{error}")
            continue
        assert receipt is not None
        if not _validate_receipt_identity(receipt, session, closure):
            _append(blockers, f"{name}_receipt_identity_or_status_invalid")
            continue
        loaded[name] = receipt

    compile_receipt = loaded.get("dosod_hbm_compile")
    if compile_receipt is not None:
        valid = (
            compile_receipt.get("receipt_id") == "tzcup_s100p_dosod_hbm_compile_receipt_v1"
            and compile_receipt.get("output_relative_path") == EXPECTED_PAYLOAD_PATHS["dosod_hbm"]
            and _is_digest(compile_receipt.get("output_sha256"))
            and isinstance(compile_receipt.get("output_byte_size"), int)
            and compile_receipt["output_byte_size"] > 0
            and compile_receipt.get("compiler_identity_verified") is True
        )
        checks["dosod_hbm_compile_receipt_valid"] = valid
        if not valid:
            _append(blockers, "dosod_hbm_compile_receipt_payload_or_compiler_invalid")

    payload_receipt = loaded.get("model_payload")
    if payload_receipt is not None:
        payloads = payload_receipt.get("payloads")
        valid = payload_receipt.get("receipt_id") == "tzcup_s100p_model_payload_receipt_v1" and isinstance(payloads, Mapping)
        if valid:
            valid = set(payloads) == set(EXPECTED_PAYLOAD_PATHS) and all(
                isinstance(payloads.get(name), Mapping)
                and payloads[name].get("target_relative_path") == expected_path
                and _is_digest(payloads[name].get("sha256"))
                and isinstance(payloads[name].get("byte_size"), int)
                and payloads[name]["byte_size"] > 0
                for name, expected_path in EXPECTED_PAYLOAD_PATHS.items()
            )
        if valid and compile_receipt is not None:
            dosod = payloads["dosod_hbm"]
            valid = (
                dosod.get("sha256") == compile_receipt.get("output_sha256")
                and dosod.get("byte_size") == compile_receipt.get("output_byte_size")
            )
        checks["model_payload_receipt_valid"] = bool(valid)
        if not valid:
            _append(blockers, "model_payload_receipt_incomplete_or_not_bound_to_dosod_compile")

    overlay_receipt = loaded.get("overlay_build")
    if overlay_receipt is not None:
        packages = overlay_receipt.get("packages")
        valid = overlay_receipt.get("receipt_id") == "tzcup_s100p_overlay_build_receipt_v1" and isinstance(packages, Mapping)
        if valid:
            valid = set(packages) == EXPECTED_OVERLAY_PACKAGES and all(
                isinstance(packages[name], Mapping)
                and _is_digest(packages[name].get("source_sha256"))
                and _is_digest(packages[name].get("installed_sha256"))
                for name in EXPECTED_OVERLAY_PACKAGES
            )
        checks["overlay_build_receipt_valid"] = bool(valid)
        if not valid:
            _append(blockers, "overlay_build_receipt_incomplete")

    dependency_receipt = loaded.get("runtime_dependencies")
    if dependency_receipt is not None:
        packages = dependency_receipt.get("packages")
        valid = dependency_receipt.get("receipt_id") == "tzcup_s100p_runtime_dependencies_receipt_v1" and isinstance(packages, Mapping)
        if valid:
            valid = set(packages) == EXPECTED_DEPENDENCIES and all(
                isinstance(packages[name], Mapping) and isinstance(packages[name].get("version"), str) and bool(packages[name]["version"])
                for name in EXPECTED_DEPENDENCIES
            )
        checks["runtime_dependencies_receipt_valid"] = bool(valid)
        if not valid:
            _append(blockers, "runtime_dependencies_receipt_incomplete")

    thermal_receipt = loaded.get("thermal_power")
    if thermal_receipt is not None:
        metrics = thermal_receipt.get("metrics")
        valid = thermal_receipt.get("receipt_id") == "tzcup_s100p_thermal_power_receipt_v1" and isinstance(metrics, Mapping)
        if valid:
            duration = metrics.get("duration_sec")
            temperature = metrics.get("maximum_temperature_c")
            available = metrics.get("minimum_available_memory_percent")
            power = metrics.get("maximum_input_power_w")
            valid = (
                type(duration) in (int, float) and duration >= 1800
                and type(temperature) in (int, float) and math.isfinite(temperature) and temperature <= 85.0
                and type(available) in (int, float) and math.isfinite(available) and available >= 5.0
                and type(power) in (int, float) and math.isfinite(power) and power > 0
            )
        checks["thermal_power_receipt_valid"] = bool(valid)
        if not valid:
            _append(blockers, "thermal_power_receipt_missing_required_measured_metrics")
    return checks, details


def validate_final_predeploy(
    *,
    repository_root: str | Path = ROOT,
    board_manifest_path: str | Path = DEFAULT_BOARD_BUNDLE,
    offline_plan_path: str | Path = DEFAULT_OFFLINE_PLAN,
    hbm_contract_path: str | Path = DEFAULT_HBM_CONTRACT,
    mechanical_electrical_path: str | Path = DEFAULT_MECHANICAL_ELECTRICAL,
    snapshot_path: str | Path = DEFAULT_SNAPSHOT,
    acceptance_session_path: str | Path = DEFAULT_SESSION,
    runtime_binding_path: str | Path = DEFAULT_RUNTIME_BINDING,
    receipt_root: str | Path = DEFAULT_RECEIPT_ROOT,
    artifact_root: str | Path | None = None,
) -> dict[str, Any]:
    """Return the one final deployment decision without touching a board."""

    root = Path(repository_root).resolve()
    board_manifest_path = Path(board_manifest_path).resolve()
    offline_plan_path = Path(offline_plan_path).resolve()
    hbm_contract_path = Path(hbm_contract_path).resolve()
    mechanical_electrical_path = Path(mechanical_electrical_path).resolve()
    snapshot_path = Path(snapshot_path).resolve()
    acceptance_session_path = Path(acceptance_session_path).resolve()
    runtime_binding_path = Path(runtime_binding_path).resolve()
    receipt_root = Path(receipt_root).resolve()
    blockers: list[str] = []

    board_report = board_bundle.validate_manifest(board_manifest_path, repository_root=root)
    offline_report = offline_predeploy.validate_offline_predeploy(
        offline_plan_path, repository_root=root, artifact_root=artifact_root
    )
    board_payload, board_error = _load_object(board_manifest_path)
    hbm_payload, hbm_error = _load_object(hbm_contract_path)
    hbm_blockers: list[str] = []
    if hbm_error:
        _append(blockers, f"dosod_hbm_contract_{hbm_error}")
    else:
        validate_contract_shape(hbm_payload, hbm_blockers)
    hbm_contract_valid = not hbm_error and not hbm_blockers
    if not hbm_contract_valid:
        _append(blockers, "dosod_hbm_compile_contract_invalid")

    mechanical_valid = False
    mechanical_payload, mechanical_error = _load_object(mechanical_electrical_path)
    if mechanical_error:
        _append(blockers, f"mechanical_electrical_contract_{mechanical_error}")
    else:
        try:
            mechanical_electrical.validate(mechanical_payload, root)
            mechanical_valid = True
        except ValueError as exc:
            _append(blockers, f"mechanical_electrical_contract_invalid:{exc}")
    if not mechanical_valid:
        _append(blockers, "mechanical_electrical_contract_invalid")

    identity_checks, identity = _validate_session_identity(
        board_manifest=board_payload,
        snapshot_path=snapshot_path,
        session_path=acceptance_session_path,
        runtime_binding_path=runtime_binding_path,
        blockers=blockers,
    )
    receipt_checks, receipt_details = _validate_receipts(receipt_root, runtime_binding_path, blockers)
    board_checks = board_report.get("checks")
    offline_checks = offline_report.get("checks")
    checks: dict[str, bool] = {
        "board_bundle_static_audit_completed": board_report.get("report_id") == "tzcup_s100p_formal_board_bundle_validation_v1",
        "board_bundle_static_integrity_valid": isinstance(board_checks, Mapping) and all(board_checks.get(name) is True for name in REQUIRED_BOARD_STATIC_CHECKS),
        "offline_predeploy_audit_completed": offline_report.get("report_id") == "tzcup_s100p_offline_predeploy_validation_v1",
        "offline_predeploy_static_inputs_valid": isinstance(offline_checks, Mapping) and all(offline_checks.get(name) is True for name in REQUIRED_OFFLINE_STATIC_CHECKS),
        "dosod_hbm_compile_contract_valid": hbm_contract_valid,
        "mechanical_electrical_fail_closed_contract_valid": mechanical_valid,
        **identity_checks,
        **receipt_checks,
    }
    if not checks["board_bundle_static_audit_completed"]:
        _append(blockers, "board_bundle_static_audit_failed")
    if not checks["board_bundle_static_integrity_valid"]:
        _append(blockers, "board_bundle_static_integrity_invalid")
    if not checks["offline_predeploy_audit_completed"]:
        _append(blockers, "offline_predeploy_audit_failed")
    if not checks["offline_predeploy_static_inputs_valid"]:
        _append(blockers, "offline_predeploy_static_inputs_invalid")
    if not identity_checks["pc_session_runtime_closure_identity_exact"]:
        _append(blockers, "pc_session_runtime_closure_identity_not_ready")

    ready = all(checks.values())
    return {
        "schema_version": 1,
        "report_id": "tzcup_s100p_final_predeploy_audit_v1",
        "operation_boundary": "local_read_only_audit_no_board_copy_ssh_install_node_start_data_collection_or_receipt_generation",
        "status": "PREDEPLOY_READY_NOT_DEPLOYED" if ready else "BLOCKED",
        "ready_to_deploy": ready,
        "board_interaction_performed": False,
        "payload_copy_performed": False,
        "dependency_install_performed": False,
        "node_started": False,
        "data_collection_performed": False,
        "receipt_generation_performed": False,
        "checks": checks,
        "blockers": blockers,
        "pc_session_runtime_identity": identity,
        "receipt_requirements": receipt_details,
        "board_bundle": {
            "status": board_report.get("status"),
            "ready_to_deploy": board_report.get("ready_to_deploy"),
            "blockers": board_report.get("blockers", []),
        },
        "offline_predeploy": {
            "status": offline_report.get("status"),
            "ready": offline_report.get("ready"),
            "blockers": offline_report.get("blockers", []),
        },
        "dosod_hbm_compile_contract": {"path": str(hbm_contract_path), "valid": hbm_contract_valid, "blockers": hbm_blockers},
        "mechanical_electrical_contract": {
            "path": str(mechanical_electrical_path),
            "valid": mechanical_valid,
            "status": mechanical_payload.get("status") if mechanical_payload else None,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, default=ROOT)
    parser.add_argument("--board-manifest", type=Path, default=DEFAULT_BOARD_BUNDLE)
    parser.add_argument("--offline-plan", type=Path, default=DEFAULT_OFFLINE_PLAN)
    parser.add_argument("--hbm-contract", type=Path, default=DEFAULT_HBM_CONTRACT)
    parser.add_argument("--mechanical-electrical", type=Path, default=DEFAULT_MECHANICAL_ELECTRICAL)
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--acceptance-session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--runtime-binding", type=Path, default=DEFAULT_RUNTIME_BINDING)
    parser.add_argument("--receipt-root", type=Path, default=DEFAULT_RECEIPT_ROOT)
    parser.add_argument("--artifact-root", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--allow-blocked-exit-zero", action="store_true")
    args = parser.parse_args()
    report = validate_final_predeploy(
        repository_root=args.repository_root,
        board_manifest_path=args.board_manifest,
        offline_plan_path=args.offline_plan,
        hbm_contract_path=args.hbm_contract,
        mechanical_electrical_path=args.mechanical_electrical,
        snapshot_path=args.snapshot,
        acceptance_session_path=args.acceptance_session,
        runtime_binding_path=args.runtime_binding,
        receipt_root=args.receipt_root,
        artifact_root=args.artifact_root,
    )
    encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        output = args.output if args.output.is_absolute() else args.repository_root.resolve() / args.output
        if output.exists():
            raise SystemExit(f"refusing to overwrite retained audit: {output}")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if report["ready_to_deploy"] or args.allow_blocked_exit_zero else 2


if __name__ == "__main__":
    raise SystemExit(main())
