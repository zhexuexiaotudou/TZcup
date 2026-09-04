"""Regression coverage for the one local-only S100P final predeploy gate."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "validate_s100p_final_predeploy.py"
SPEC = importlib.util.spec_from_file_location("s100p_final_predeploy", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def _synthetic_offline_report(*, ready: bool, blockers: list[str]) -> dict[str, object]:
    return {
        "report_id": "tzcup_s100p_offline_predeploy_validation_v1",
        "status": "PREDEPLOY_READY_NOT_DEPLOYED" if ready else "BLOCKED",
        "ready": ready,
        "checks": {name: True for name in MODULE.REQUIRED_OFFLINE_STATIC_CHECKS},
        "blockers": blockers,
    }


def _synthetic_receipt(receipt_id: str, binding: dict, closure: dict, **fields: object) -> dict:
    return {
        "schema_version": 1,
        "receipt_id": receipt_id,
        "status": "VERIFIED",
        "board_interaction_performed": True,
        "acceptance_session_binding": binding,
        "runtime_closure_binding": closure,
        **fields,
    }


def _complete_synthetic_inputs(tmp_path: Path) -> dict[str, Path]:
    snapshot_path = ROOT / "reports" / "engineering" / "formal_vehicle_snapshot_manifest.json"
    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    urdf = snapshot["outputs"]["reports/engineering/formal_competition_vehicle.urdf"]
    board = json.loads((ROOT / "config" / "s100p_formal_board_bundle_manifest.json").read_text(encoding="utf-8"))
    board["formal_snapshot"].update(
        {
            "sha256": _sha256(snapshot_path),
            "byte_size": snapshot_path.stat().st_size,
            "source_inventory_sha256": snapshot["source_inventory_sha256"],
            "output_inventory_sha256": snapshot["output_inventory_sha256"],
            "formal_urdf_sha256": urdf["sha256"],
        }
    )
    board_path = tmp_path / "board.json"
    _write(board_path, board)

    session_snapshot = {
        "snapshot_manifest_sha256": _sha256(snapshot_path),
        "source_inventory_sha256": snapshot["source_inventory_sha256"],
        "expanded_urdf_sha256": urdf["sha256"],
    }
    closure = {
        "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
        "runtime_install_root": "/frozen/runtime/install",
        "manifest_sha256": "a" * 64,
        "closure_sha256": "b" * 64,
    }
    session_path = tmp_path / "session.json"
    _write(
        session_path,
        {
            "schema_version": 1,
            "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
            "started_epoch_ns": 123,
            "snapshot": session_snapshot,
            "runtime_closure_binding": closure,
        },
    )
    binding = {
        "session_manifest_sha256": _sha256(session_path),
        "session_started_epoch_ns": 123,
        "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        "snapshot": session_snapshot,
        "snapshot_output_inventory_sha256": snapshot["output_inventory_sha256"],
    }
    runtime_binding_path = tmp_path / "runtime-binding.json"
    _write(
        runtime_binding_path,
        {
            "schema_version": 1,
            "status": "FORMAL_RUNTIME_GATE_BOUND",
            "acceptance_session_binding": binding,
            "runtime_closure_binding": closure,
        },
    )
    receipt_root = tmp_path / "receipts"
    payloads = {
        key: {"target_relative_path": path, "sha256": (str(index) * 64)[:64], "byte_size": index + 1}
        for index, (key, path) in enumerate(MODULE.EXPECTED_PAYLOAD_PATHS.items(), start=1)
    }
    dosod = payloads["dosod_hbm"]
    _write(
        receipt_root / MODULE.RECEIPTS["dosod_hbm_compile"],
        _synthetic_receipt(
            "tzcup_s100p_dosod_hbm_compile_receipt_v1", binding, closure,
            output_relative_path=dosod["target_relative_path"], output_sha256=dosod["sha256"],
            output_byte_size=dosod["byte_size"], compiler_identity_verified=True,
        ),
    )
    _write(
        receipt_root / MODULE.RECEIPTS["model_payload"],
        _synthetic_receipt("tzcup_s100p_model_payload_receipt_v1", binding, closure, payloads=payloads),
    )
    _write(
        receipt_root / MODULE.RECEIPTS["overlay_build"],
        _synthetic_receipt(
            "tzcup_s100p_overlay_build_receipt_v1", binding, closure,
            packages={name: {"source_sha256": "c" * 64, "installed_sha256": "d" * 64} for name in MODULE.EXPECTED_OVERLAY_PACKAGES},
        ),
    )
    _write(
        receipt_root / MODULE.RECEIPTS["runtime_dependencies"],
        _synthetic_receipt(
            "tzcup_s100p_runtime_dependencies_receipt_v1", binding, closure,
            packages={name: {"version": "1.0"} for name in MODULE.EXPECTED_DEPENDENCIES},
        ),
    )
    _write(
        receipt_root / MODULE.RECEIPTS["thermal_power"],
        _synthetic_receipt(
            "tzcup_s100p_thermal_power_receipt_v1", binding, closure,
            metrics={"duration_sec": 1800, "maximum_temperature_c": 84.0, "minimum_available_memory_percent": 5.0, "maximum_input_power_w": 120.0},
        ),
    )
    return {"board": board_path, "snapshot": snapshot_path, "session": session_path, "binding": runtime_binding_path, "receipts": receipt_root}


def test_audit_is_blocked_with_explicitly_missing_session_and_receipts(
    tmp_path: Path,
) -> None:
    report = MODULE.validate_final_predeploy(
        acceptance_session_path=tmp_path / "missing-session.json",
        runtime_binding_path=tmp_path / "missing-runtime-binding.json",
        receipt_root=tmp_path / "missing-receipts",
    )
    assert report["status"] == "BLOCKED"
    assert report["ready_to_deploy"] is False
    assert report["board_interaction_performed"] is False
    assert report["receipt_generation_performed"] is False
    assert {
        "acceptance_session_missing",
        "acceptance_session_not_current_running_closure_bound",
    } & set(report["blockers"])
    assert "dosod_hbm_compile_receipt_cannot_bind_runtime_identity" in report["blockers"]
    assert report["checks"]["mechanical_electrical_fail_closed_contract_valid"] is True
    assert "optional_input_policy_valid" in MODULE.REQUIRED_OFFLINE_STATIC_CHECKS


def test_complete_synthetic_receipt_chain_requires_exact_single_identity(
    tmp_path: Path, monkeypatch
) -> None:
    paths = _complete_synthetic_inputs(tmp_path)
    monkeypatch.setattr(
        MODULE.offline_predeploy,
        "validate_offline_predeploy",
        lambda *args, **kwargs: _synthetic_offline_report(ready=True, blockers=[]),
    )
    report = MODULE.validate_final_predeploy(
        board_manifest_path=paths["board"], snapshot_path=paths["snapshot"],
        acceptance_session_path=paths["session"], runtime_binding_path=paths["binding"],
        receipt_root=paths["receipts"],
    )
    assert report["status"] == "PREDEPLOY_READY_NOT_DEPLOYED"
    assert report["ready_to_deploy"] is True
    assert all(report["checks"].values())


def test_blocked_offline_audit_cannot_be_bypassed_by_complete_receipts(
    tmp_path: Path, monkeypatch
) -> None:
    paths = _complete_synthetic_inputs(tmp_path)
    monkeypatch.setattr(
        MODULE.offline_predeploy,
        "validate_offline_predeploy",
        lambda *args, **kwargs: _synthetic_offline_report(
            ready=False,
            blockers=["historical_g0_missing"],
        ),
    )
    report = MODULE.validate_final_predeploy(
        board_manifest_path=paths["board"], snapshot_path=paths["snapshot"],
        acceptance_session_path=paths["session"], runtime_binding_path=paths["binding"],
        receipt_root=paths["receipts"],
    )
    assert report["status"] == "BLOCKED"
    assert report["checks"]["offline_predeploy_static_inputs_valid"] is True
    assert report["checks"]["offline_predeploy_ready"] is False
    assert "offline_predeploy_not_ready" in report["blockers"]


def test_receipt_closure_drift_blocks_the_whole_predeploy_decision(tmp_path: Path) -> None:
    paths = _complete_synthetic_inputs(tmp_path)
    payload_path = paths["receipts"] / MODULE.RECEIPTS["model_payload"]
    payload = json.loads(payload_path.read_text(encoding="utf-8"))
    payload["runtime_closure_binding"] = copy.deepcopy(payload["runtime_closure_binding"])
    payload["runtime_closure_binding"]["closure_sha256"] = "e" * 64
    _write(payload_path, payload)
    report = MODULE.validate_final_predeploy(
        board_manifest_path=paths["board"], snapshot_path=paths["snapshot"],
        acceptance_session_path=paths["session"], runtime_binding_path=paths["binding"],
        receipt_root=paths["receipts"],
    )
    assert report["status"] == "BLOCKED"
    assert report["checks"]["model_payload_receipt_valid"] is False
    assert "model_payload_receipt_identity_or_status_invalid" in report["blockers"]


def test_board_bundle_digest_drift_cannot_be_bypassed_by_complete_receipts(tmp_path: Path) -> None:
    paths = _complete_synthetic_inputs(tmp_path)
    board = json.loads(paths["board"].read_text(encoding="utf-8"))
    board["bound_sources"][0]["sha256"] = "0" * 64
    _write(paths["board"], board)
    report = MODULE.validate_final_predeploy(
        board_manifest_path=paths["board"], snapshot_path=paths["snapshot"],
        acceptance_session_path=paths["session"], runtime_binding_path=paths["binding"],
        receipt_root=paths["receipts"],
    )
    assert report["status"] == "BLOCKED"
    assert report["checks"]["board_bundle_static_integrity_valid"] is False
    assert "board_bundle_static_integrity_invalid" in report["blockers"]


def test_validator_source_has_no_board_or_receipt_generation_implementation() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    for forbidden in ("import subprocess", "import socket", "import paramiko", "import shutil", "import ros2"):
        assert forbidden not in source
