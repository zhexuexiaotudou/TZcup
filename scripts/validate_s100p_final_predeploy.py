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
from validate_dosod_s100p_hbm_compile_contract import audit_calibration, validate_contract_shape


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
EXPECTED_DEPENDENCIES = (
    set(board_bundle.SANITATION_PERCEPTION_EXEC_DEPENDENCIES)
    - board_bundle.EVALUATOR_ONLY_EXEC_DEPENDENCIES
) | {
    "hobot_dosod", "mono_edgesam"
}
OFFLINE_COMPILE_RECEIPT_ID = "tzcup_s100p_dosod_hbm_compile_receipt_v1"
OFFLINE_COMPILE_STATUS = "COMPILED_NOT_BOARD_ACCEPTED"
EXPECTED_S100P_STAGE_PARENT = "/opt/tzcup/stages"
OFFICIAL_TROS_PACKAGES = {
    "hobot_dosod": ("1.0.0", "tros_1.0.0", "c949d69898926054ac3be5793fdd4a482da685c9"),
    "mono_edgesam": ("0.2.0", "tros_0.2.0", "dc083c6ce603e6c0f1c80b5fe44743e48b943cfa"),
    "dnn_node": ("2.5.9", "tros_2.5.9", "01eae3a4ef0b3ba0a2365c3f1e2280db09a983f9"),
}
PYTHON_ABI_VERSIONS = {
    "numpy": "1.21.5", "yaml": "5.4.1", "cv_bridge": "3.2.1", "rclpy": "3.3.17",
    "tf2_ros": "0.25.16", "vision_msgs": "4.1.1",
}
PYTHON_IMPORTS = set(PYTHON_ABI_VERSIONS) | {"ai_msgs", "sensor_msgs", "cv2", "sanitation_perception"}
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
    "offline_predeploy_plan_parseable", "operation_boundary_exact", "optional_input_policy_valid",
    "plan_input_keys_exact",
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


def _valid_board_payload_bridge(receipt: Mapping[str, Any]) -> bool:
    """Require the producer's retained board identity and canonical stage facts."""
    identity = receipt.get("board_identity")
    candidate = receipt.get("candidate_stage")
    prefix = EXPECTED_S100P_STAGE_PARENT + "/"
    candidate_ok = (
        isinstance(candidate, str) and candidate.startswith(prefix)
        and bool(candidate[len(prefix):]) and "/" not in candidate[len(prefix):]
    )
    device = identity.get("bpu_device") if isinstance(identity, Mapping) else None
    return (
        candidate_ok and receipt.get("stage_root") == candidate and isinstance(identity, Mapping)
        and "rdk s100p" in str(identity.get("model", "")).lower()
        and "drobot,s100-rdk" in str(identity.get("compatible", "")).lower()
        and _is_digest(identity.get("model_sha256")) and _is_digest(identity.get("compatible_sha256"))
        and identity.get("architecture") == "aarch64"
        and isinstance(device, Mapping) and device.get("path") == "/dev/bpu_core0"
        and isinstance(device.get("st_mode"), int) and device.get("is_character_device") is True
        and device.get("is_symlink") is False
        and all(isinstance(device.get(key), int) and device[key] >= 0 for key in ("st_rdev_major", "st_rdev_minor", "st_ino"))
        and identity.get("required_modules") == ["bpu_cores", "bpu_framework"]
    )


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
        else:
            details["session_sha256"] = _sha256(session_path)
            details["session_byte_size"] = session_path.stat().st_size

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
        elif isinstance(closure_binding, Mapping):
            manifest_sha = closure_binding.get("manifest_sha256")
            closure_sha = closure_binding.get("closure_sha256")
            if _is_digest(manifest_sha) and _is_digest(closure_sha):
                details["runtime_closure_binding"] = {
                    "runtime_closure_manifest_sha256": manifest_sha,
                    "runtime_closure_sha256": closure_sha,
                }
    details["snapshot_identity"] = snapshot_identity
    return checks, details


def _validate_receipt_identity(
    receipt: Mapping[str, Any], session: Mapping[str, Any], closure: Mapping[str, Any]
) -> bool:
    binding = receipt.get("acceptance_session_binding")
    canonical_closure = {
        "runtime_closure_manifest_sha256": closure.get("manifest_sha256"),
        "runtime_closure_sha256": closure.get("closure_sha256"),
    }
    session_status = binding.get("session_status_at_gate") if isinstance(binding, Mapping) else None
    session_status = session_status if session_status is not None else binding.get("session_status_at_collection") if isinstance(binding, Mapping) else None
    return (
        receipt.get("schema_version") == 1
        and receipt.get("status") == "VERIFIED"
        and receipt.get("board_interaction_performed") is True
        and isinstance(binding, Mapping)
        and binding.get("session_manifest_sha256") == session.get("session_manifest_sha256")
        and binding.get("session_started_epoch_ns") == session.get("session_started_epoch_ns")
        and binding.get("snapshot") == session.get("snapshot")
        and session_status == session.get("session_status_at_gate")
        and receipt.get("runtime_closure_binding") in (closure, canonical_closure)
    )


def _validate_offline_compile_receipt(
    receipt: Mapping[str, Any], *, hbm_contract_path: Path, blockers: list[str]
) -> bool:
    """Validate the compiler's non-board receipt without a session binding."""
    output = hbm_contract_path
    contract, error = _load_object(hbm_contract_path)
    if error or contract is None:
        _append(blockers, "dosod_hbm_compile_contract_unavailable_for_receipt")
        return False
    expected_output = contract.get("output", {}).get("relative_path")
    inputs = receipt.get("inputs")
    calibration_path = inputs.get("calibration_manifest") if isinstance(inputs, Mapping) else None
    reaudit = receipt.get("calibration_reaudit")
    compiler = receipt.get("compiler")
    producer = receipt.get("producer")
    raw_logs = receipt.get("raw_logs")
    execution = receipt.get("execution")
    valid = (
        receipt.get("schema_version") == 1
        and receipt.get("receipt_id") == OFFLINE_COMPILE_RECEIPT_ID
        and receipt.get("status") == OFFLINE_COMPILE_STATUS
        and receipt.get("returncode") == 0
        and receipt.get("output_created_by_this_compile") is True
        and receipt.get("compiler_identity_verified") is True
        and receipt.get("output_relative_path") == expected_output
        and _is_digest(receipt.get("output_sha256"))
        and isinstance(receipt.get("output_byte_size"), int)
        and receipt["output_byte_size"] > 0
        and receipt.get("board_interaction_performed") in (None, False)
        and receipt.get("acceptance_session_binding") is None
        and receipt.get("runtime_closure_binding") is None
        and isinstance(inputs, Mapping)
        and inputs.get("contract_sha256") == _sha256(output)
        and isinstance(calibration_path, str)
        and all(_is_digest(inputs.get(name)) for name in (
            "preflight_sha256", "compile_config_sha256", "compiler_identity_sha256",
            "calibration_manifest_sha256", "calibration_records_sha256", "model_sha256",
        ))
        and isinstance(inputs.get("calibration_sample_count"), int)
        and inputs["calibration_sample_count"] > 0
        and isinstance(reaudit, Mapping)
        and reaudit.get("manifest_sha256") == inputs.get("calibration_manifest_sha256")
        and reaudit.get("records_sha256") == inputs.get("calibration_records_sha256")
        and reaudit.get("sample_count") == inputs.get("calibration_sample_count")
        and isinstance(compiler, Mapping)
        and isinstance(compiler.get("requested"), Mapping)
        and isinstance(compiler.get("resolved"), Mapping)
        and isinstance(compiler.get("package_versions"), Mapping)
        and isinstance(producer, Mapping) and isinstance(producer.get("path"), str) and bool(producer["path"])
        and _is_digest(producer.get("sha256"))
        and isinstance(raw_logs, Mapping)
        and all(isinstance(raw_logs.get(name), Mapping) and isinstance(raw_logs[name].get("path"), str)
                and bool(raw_logs[name]["path"]) and _is_digest(raw_logs[name].get("sha256"))
                for name in ("stdout", "stderr"))
        and isinstance(execution, Mapping)
        and execution.get("deadline_seconds") == 3600 and execution.get("term_grace_seconds") == 10
        and execution.get("start_new_session") is True and isinstance(execution.get("pgid"), int)
        and execution.get("timed_out") is False and execution.get("term_sent") is False
        and execution.get("kill_sent") is False and execution.get("zero_survivor") is True
        and isinstance(execution.get("elapsed_seconds"), (int, float)) and execution["elapsed_seconds"] >= 0
        and receipt.get("blockers") == []
    )
    if not valid:
        _append(blockers, "dosod_hbm_compile_receipt_not_canonical_offline_evidence")
        return False
    calibration = Path(calibration_path)
    if not calibration.is_file() or inputs.get("calibration_manifest_sha256") != _sha256(calibration):
        _append(blockers, "dosod_hbm_compile_receipt_calibration_manifest_drift")
        return False
    audit_blockers: list[str] = []
    audit_calibration(calibration.parent, contract, audit_blockers)
    if audit_blockers:
        _append(blockers, "dosod_hbm_compile_receipt_calibration_reaudit_failed")
        return False
    return True


def _valid_runtime_dependencies(receipt: Mapping[str, Any]) -> bool:
    packages = receipt.get("packages")
    providers = receipt.get("providers")
    imports = receipt.get("python_imports")
    shell = receipt.get("sourced_shell_id")
    if not (isinstance(shell, str) and shell and isinstance(packages, Mapping)
            and isinstance(providers, Mapping) and isinstance(imports, Mapping)
            and set(packages) == EXPECTED_DEPENDENCIES and set(providers) == set(OFFICIAL_TROS_PACKAGES)
            and set(imports) == PYTHON_IMPORTS):
        return False
    if not all(isinstance(row, Mapping) and isinstance(row.get("version"), str) and row["version"]
               and isinstance(row.get("prefix"), str) and row["prefix"].startswith("/")
               and isinstance(row.get("executables"), list) and row["executables"]
               for row in packages.values()):
        return False
    for name, (version, tag, commit) in OFFICIAL_TROS_PACKAGES.items():
        row = providers[name]
        if not (isinstance(row, Mapping) and row.get("dpkg_owner") == name
                and row.get("dpkg_version") == version and row.get("architecture") == "arm64"
                and row.get("upstream_tag") == tag and row.get("upstream_commit") == commit
                and row.get("binary_identical_to_upstream") is False):
            return False
    for name, row in imports.items():
        if not (isinstance(row, Mapping) and row.get("sourced_shell_id") == shell
                and isinstance(row.get("module_path"), str) and row["module_path"].startswith("/")):
            return False
        expected = PYTHON_ABI_VERSIONS.get(name)
        if expected is not None and row.get("version") != expected:
            return False
    return True


def _validate_receipts(
    receipt_root: Path, runtime_binding_path: Path, hbm_contract_path: Path, blockers: list[str]
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
        details["receipts"][name] = {
            "path": str(path),
            "present": error is None,
            "sha256": _sha256(path) if error is None else None,
            "byte_size": path.stat().st_size if error is None else None,
        }
        if error:
            _append(blockers, f"{name}_receipt_{error}")
            continue
        assert receipt is not None
        if name == "dosod_hbm_compile":
            if not _validate_offline_compile_receipt(
                receipt, hbm_contract_path=hbm_contract_path, blockers=blockers
            ):
                continue
        elif not _validate_receipt_identity(receipt, session, closure):
            _append(blockers, f"{name}_receipt_identity_or_status_invalid")
            continue
        loaded[name] = receipt

    compile_receipt = loaded.get("dosod_hbm_compile")
    if compile_receipt is not None:
        valid = (
            compile_receipt.get("receipt_id") == OFFLINE_COMPILE_RECEIPT_ID
            and compile_receipt.get("status") == OFFLINE_COMPILE_STATUS
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
        valid = (
            payload_receipt.get("receipt_id") == "tzcup_s100p_model_payload_receipt_v1"
            and isinstance(payloads, Mapping)
            and _valid_board_payload_bridge(payload_receipt)
        )
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
                and payload_receipt.get("offline_compile_receipt_sha256")
                == details["receipts"]["dosod_hbm_compile"]["sha256"]
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
        valid = (dependency_receipt.get("receipt_id") == "tzcup_s100p_runtime_dependencies_receipt_v1"
                 and _valid_runtime_dependencies(dependency_receipt))
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
    receipt_checks, receipt_details = _validate_receipts(
        receipt_root, runtime_binding_path, hbm_contract_path, blockers
    )
    board_checks = board_report.get("checks")
    offline_checks = offline_report.get("checks")
    checks: dict[str, bool] = {
        "board_bundle_static_audit_completed": board_report.get("report_id") == "tzcup_s100p_formal_board_bundle_validation_v1",
        "board_bundle_static_integrity_valid": isinstance(board_checks, Mapping) and all(board_checks.get(name) is True for name in REQUIRED_BOARD_STATIC_CHECKS),
        "offline_predeploy_audit_completed": offline_report.get("report_id") == "tzcup_s100p_offline_predeploy_validation_v1",
        "offline_predeploy_static_inputs_valid": isinstance(offline_checks, Mapping) and all(offline_checks.get(name) is True for name in REQUIRED_OFFLINE_STATIC_CHECKS),
        "offline_predeploy_ready": (
            offline_report.get("ready") is True
            and isinstance(offline_report.get("blockers"), list)
            and not offline_report["blockers"]
        ),
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
    if not checks["offline_predeploy_ready"]:
        _append(blockers, "offline_predeploy_not_ready")
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
        "board_handoff_binding": {
            "session_sha256": identity.get("session_sha256"),
            "session_byte_size": identity.get("session_byte_size"),
            "runtime_closure_binding": identity.get("runtime_closure_binding"),
        },
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
