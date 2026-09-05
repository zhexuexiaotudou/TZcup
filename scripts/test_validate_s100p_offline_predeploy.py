"""Regression coverage for the no-board-contact S100P pre-deployment audit."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "validate_s100p_offline_predeploy.py"
SPEC = importlib.util.spec_from_file_location("s100p_offline_predeploy", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_empty_artifact_root_is_fail_closed_without_board_contact(tmp_path: Path):
    report = MODULE.validate_offline_predeploy(artifact_root=tmp_path / "empty-artifacts")
    assert report["operation_boundary"] == "no_board_copy_no_ssh_no_node_start_no_data_collection"
    assert report["operation_performed"] == "local_read_only_dry_run"
    assert report["status"] == "BLOCKED"
    assert report["board_interaction_performed"] is False
    assert report["data_collection_performed"] is False
    assert report["formal_board_acceptance"] is False
    assert "bundle:asset_missing:dosod_hbm" in report["blockers"]
    assert "bundle:asset_missing:board_runtime_manifest" in report["blockers"]
    assert not any("historical_smoke" in blocker for blocker in report["blockers"])
    assert report["checks"]["overlay_package_sources_valid"] is True
    assert report["checks"]["overlay_runtime_package_set_valid"] is True
    assert report["checks"]["launch_source_contract_valid"] is True


def test_historical_g0_and_missing_optional_smoke_are_explicitly_nonformal_references():
    report = MODULE.validate_offline_predeploy()
    assert report["checks"]["historical_g0_identity_valid"] is True
    assert report["checks"]["historical_g0_read_only_safety_valid"] is True
    assert report["checks"]["historical_g0_project_overlay_absent"] is True
    assert report["checks"]["historical_g0_project_models_absent"] is True
    assert report["historical_g0"]["evidence_class"] == "historical_read_only_reference_not_current_acceptance"
    assert report["historical_board_smoke"]["available"] is False
    assert report["historical_board_smoke"]["formal_acceptance"] is False
    assert report["historical_board_smoke"]["evidence_class"] == "missing_historical_bpu_smoke_not_current_acceptance"
    assert report["checks"]["historical_smoke_reference_valid"] is False
    assert report["checks"]["historical_smoke_remains_nonformal"] is False
    assert report["checks"]["historical_smoke_does_not_grant_acceptance"] is True


def test_valid_historical_smoke_fixture_remains_nonformal():
    checks: dict[str, bool] = {}
    blockers: list[str] = []
    result = MODULE._validate_historical_smoke(
        {
            "status": "SMOKE_PASSED_FORMAL_ACCEPTANCE_BLOCKED",
            "formal_acceptance": False,
            "formal_blockers": ["not_current_acceptance"],
            "runtime": {"backend": "bpu"},
            "nodes_observed": [
                "rgb_to_nv12_adapter",
                "hobot_dosod",
                "mono_edgesam",
                "open_vocab_product_adapter",
            ],
            "safety": {"actuator_commands_sent": False},
        },
        checks,
        blockers,
    )
    assert blockers == []
    assert all(checks.values())
    assert result["available"] is True
    assert result["formal_acceptance"] is False
    assert result["evidence_class"] == "historical_bpu_smoke_not_formal_acceptance"


def test_any_blocker_prevents_offline_ready_even_when_selected_checks_pass():
    checks = {name: True for name in MODULE.REQUIRED_READY_CHECKS}
    assert MODULE._offline_ready({"ready": True}, checks, []) is True
    assert MODULE._offline_ready({"ready": True}, checks, ["invalid_optional_reference"]) is False


def test_operation_boundary_drift_is_rejected(tmp_path: Path):
    plan = json.loads((ROOT / "config" / "s100p_offline_predeploy_plan.json").read_text(encoding="utf-8"))
    plan["operation_boundary"] = "ssh_then_start_nodes"
    path = tmp_path / "drifted-plan.json"
    path.write_text(json.dumps(plan), encoding="utf-8")
    report = MODULE.validate_offline_predeploy(path)
    assert report["checks"]["operation_boundary_exact"] is False
    assert "operation_boundary_invalid" in report["blockers"]


def test_optional_input_policy_drift_is_rejected_without_crashing(tmp_path: Path):
    plan = json.loads((ROOT / "config" / "s100p_offline_predeploy_plan.json").read_text(encoding="utf-8"))
    plan["optional_inputs"] = "historical_board_smoke"
    path = tmp_path / "drifted-optional-policy.json"
    path.write_text(json.dumps(plan), encoding="utf-8")
    report = MODULE.validate_offline_predeploy(path, artifact_root=tmp_path / "empty-artifacts")
    assert report["checks"]["optional_input_policy_valid"] is False
    assert "optional_input_policy_invalid" in report["blockers"]
    assert report["ready"] is False


def test_validator_source_rejects_process_or_network_imports():
    assert MODULE._validator_has_no_board_or_network_implementation(
        SCRIPT.read_text(encoding="utf-8")
    )
    assert not MODULE._validator_has_no_board_or_network_implementation("import subprocess\n")
    assert not MODULE._validator_has_no_board_or_network_implementation("import socket\n")
