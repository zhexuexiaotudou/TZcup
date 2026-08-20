from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "product_acceptance.py"
SPEC = importlib.util.spec_from_file_location("product_acceptance", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def _passing_value(check: dict) -> object:
    return check["threshold"]


def _passing_evidence(tmp_path: Path) -> tuple[Path, Path, dict]:
    contract_path = ROOT / "config" / "product_acceptance_v1.json"
    contract = MODULE.validate_contract(contract_path)
    evidence_root = tmp_path / "evidence"
    evidence_root.mkdir()
    artifact_hashes: dict[str, str] = {}
    gates: dict[str, dict] = {}
    fixed_sha = "a" * 64
    for gate in contract["gates"]:
        gate_id = gate["id"]
        paths = {
            "evidence_path": f"gates/{gate_id}/report.json",
            "human_report_path": f"gates/{gate_id}/report.md",
            "raw_log_paths": [f"gates/{gate_id}/run.log"],
        }
        _write(evidence_root / paths["evidence_path"], "{}\n")
        _write(evidence_root / paths["human_report_path"], f"# Gate {gate_id}\n")
        _write(evidence_root / paths["raw_log_paths"][0], "exit=0\n")
        gate_hashes = {
            value: _sha(evidence_root / value)
            for value in (
                paths["evidence_path"],
                paths["human_report_path"],
                paths["raw_log_paths"][0],
            )
        }
        gates[gate_id] = {
            "metrics": {
                check["metric"]: _passing_value(check) for check in gate["checks"]
            },
            "dataset": "SEALED_FINAL",
            "source_commit": "b" * 40,
            "model_sha256": fixed_sha,
            "config_sha256": fixed_sha,
            "dataset_sha256": fixed_sha,
            "container_digest": f"sha256:{fixed_sha}",
            "dependency_lock_sha256": fixed_sha,
            "seeds": [1],
            "command": f"run-gate-{gate_id}",
            "exit_code": 0,
            **paths,
            "artifact_sha256": gate_hashes,
        }

    for relative in contract["required_final_artifacts"]:
        _write(evidence_root / relative, f"formal artifact {relative}\n")
        artifact_hashes[relative] = _sha(evidence_root / relative)
    release = "release/TZcup_v1_bbbbbbbbbbbb.zip"
    _write(evidence_root / release, "release payload\n")
    artifact_hashes[release] = _sha(evidence_root / release)

    evidence = {
        "schema_version": 1,
        "contract_id": contract["contract_id"],
        "contract_sha256": _sha(contract_path),
        "global_metrics": {
            veto["metric"]: _passing_value(veto)
            for veto in contract["global_vetoes"]
        },
        "gates": gates,
        "final_artifact_sha256": artifact_hashes,
    }
    manifest = evidence_root / "acceptance_evidence_manifest.json"
    manifest.write_text(json.dumps(evidence, indent=2), encoding="utf-8")
    return manifest, evidence_root, evidence


def test_contract_is_fixed_and_complete() -> None:
    contract = MODULE.validate_contract(
        ROOT / "config" / "product_acceptance_v1.json"
    )
    assert contract["required_gate_ids"] == list("ABCDEFGHIJKLMNOP")
    assert len(contract["global_vetoes"]) == 14
    checks = {
        check["metric"]: check
        for gate in contract["gates"]
        for check in gate["checks"]
    }
    assert checks["mapping_area_m2"]["threshold"] == 20000
    assert checks["effective_cleaning_efficiency_m2h"]["threshold"] == 3500
    assert checks["close_range_macro_f1"]["threshold"] == 0.98
    assert checks["processed_rate_hz"]["threshold"] == 10
    assert checks["soak_duration_s"]["threshold"] == 7200


def test_complete_hash_bound_evidence_passes(tmp_path: Path) -> None:
    manifest, evidence_root, _ = _passing_evidence(tmp_path)
    evaluation = MODULE.evaluate(
        ROOT / "config" / "product_acceptance_v1.json",
        manifest,
        evidence_root,
    )
    assert evaluation["complete"] is True
    assert set(evaluation["gate_status"].values()) == {"PASS"}
    output_dir = tmp_path / "final"
    paths = MODULE.write_outputs(evaluation, output_dir, replace=False)
    status = json.loads(paths["FINAL_ACCEPTANCE_STATUS.json"].read_text())
    assert status["SIMULATION_PRODUCT_COMPLETE"] is True
    assert status["PRODUCT_INTEGRATION_READY"] is False
    assert status["PRODUCT_FIELD_READY"] is False


def test_missing_metric_fails_closed(tmp_path: Path) -> None:
    manifest, evidence_root, evidence = _passing_evidence(tmp_path)
    del evidence["gates"]["B"]["metrics"]["mapping_area_m2"]
    manifest.write_text(json.dumps(evidence), encoding="utf-8")
    evaluation = MODULE.evaluate(
        ROOT / "config" / "product_acceptance_v1.json",
        manifest,
        evidence_root,
    )
    assert evaluation["complete"] is False
    assert evaluation["gate_status"]["B"] == "FAIL"
    row = next(row for row in evaluation["rows"] if row["metric"] == "mapping_area_m2")
    assert row["status"] == "FAIL"
    assert row["measured_value"] is None


def test_global_veto_cannot_be_compensated(tmp_path: Path) -> None:
    manifest, evidence_root, evidence = _passing_evidence(tmp_path)
    evidence["global_metrics"]["collision_count"] = 1
    manifest.write_text(json.dumps(evidence), encoding="utf-8")
    evaluation = MODULE.evaluate(
        ROOT / "config" / "product_acceptance_v1.json",
        manifest,
        evidence_root,
    )
    assert evaluation["complete"] is False
    assert evaluation["global_veto_status"] == "FAIL"
    assert set(evaluation["gate_status"].values()) == {"PASS"}


def test_bad_evidence_hash_fails_gate_provenance(tmp_path: Path) -> None:
    manifest, evidence_root, evidence = _passing_evidence(tmp_path)
    evidence["gates"]["A"]["artifact_sha256"]["gates/A/run.log"] = "0" * 64
    manifest.write_text(json.dumps(evidence), encoding="utf-8")
    evaluation = MODULE.evaluate(
        ROOT / "config" / "product_acceptance_v1.json",
        manifest,
        evidence_root,
    )
    assert evaluation["complete"] is False
    assert evaluation["gate_status"]["A"] == "FAIL"
    assert any("SHA-256 mismatch" in error for error in evaluation["gate_errors"]["A"])


def test_evidence_path_escape_is_rejected(tmp_path: Path) -> None:
    manifest, evidence_root, evidence = _passing_evidence(tmp_path)
    outside = tmp_path / "outside.log"
    outside.write_text("outside", encoding="utf-8")
    evidence["gates"]["A"]["raw_log_paths"] = ["../outside.log"]
    evidence["gates"]["A"]["artifact_sha256"]["../outside.log"] = _sha(outside)
    manifest.write_text(json.dumps(evidence), encoding="utf-8")
    evaluation = MODULE.evaluate(
        ROOT / "config" / "product_acceptance_v1.json",
        manifest,
        evidence_root,
    )
    assert evaluation["complete"] is False
    assert any("escapes evidence root" in error for error in evaluation["gate_errors"]["A"])


def test_final_outputs_refuse_accidental_overwrite(tmp_path: Path) -> None:
    manifest, evidence_root, _ = _passing_evidence(tmp_path)
    evaluation = MODULE.evaluate(
        ROOT / "config" / "product_acceptance_v1.json",
        manifest,
        evidence_root,
    )
    output_dir = tmp_path / "final"
    MODULE.write_outputs(evaluation, output_dir, replace=False)
    try:
        MODULE.write_outputs(evaluation, output_dir, replace=False)
    except MODULE.ContractError as exc:
        assert "refusing to overwrite" in str(exc)
    else:
        raise AssertionError("second formal output write must fail")
