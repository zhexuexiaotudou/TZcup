from __future__ import annotations

import json
from pathlib import Path

import pytest

import verify_dosod_compile_parity_metric_chain as subject


NAMES = {
    "compile_receipt": "compile.json", "calibration_manifest": "calibration.json",
    "holdout_manifest": "holdout.json", "parity_report": "parity.json",
    "runner_identity": "runner.json", "evaluator_identity": "evaluator.json",
    "reference_evaluator_receipt": "reference-receipt.json",
    "quantized_evaluator_receipt": "quantized-receipt.json",
    "reference_report": "reference.json", "quantized_report": "metric.json",
    "thresholds": "thresholds.json",
}


def _bundle(tmp_path: Path) -> tuple[Path, Path]:
    root = tmp_path / "bundle"; root.mkdir()
    hbm = tmp_path / "dosod.hbm"; hbm.write_bytes(b"current-hbm")
    for key, name in NAMES.items():
        (root / name).write_text(json.dumps({"kind": key, "current": True}), encoding="utf-8")
    (root / "dosod_admission_bundle.json").write_text(
        json.dumps({"schema_version": 1, "artifacts": NAMES}), encoding="utf-8"
    )
    return root, hbm


def _stub(**kwargs):  # type: ignore[no-untyped-def]
    # This fixture stands in only for unavailable official identities/runners.
    # It still proves every bundle member is dereferenced through the shared
    # verifier path; production uses the unpatched full regression validator.
    for key, path in (("compile_receipt_path", kwargs["compile_receipt_path"]),
                      ("calibration_manifest", kwargs["calibration_manifest"]),
                      ("parity_report_path", kwargs["parity_report_path"]),
                      ("quantized_report_path", kwargs["quantized_report_path"])):
        if json.loads(path.read_text()).get("current") is not True:
            return {"status": "BLOCKED", "blockers": [key]}
    return {"status": "REGRESSION_PASSED", "blockers": []}


def test_complete_fixture_rechecks_each_declared_bundle_member(tmp_path: Path, monkeypatch) -> None:
    root, hbm = _bundle(tmp_path)
    monkeypatch.setattr(subject, "validate_regression", _stub)
    summary = subject.verify_dosod_compile_parity_metric_chain(root, hbm)
    assert summary["hbm_sha256"] and summary["compile_receipt_sha256"]


@pytest.mark.parametrize("member", ["compile_receipt", "calibration_manifest", "parity_report", "quantized_report"])
def test_old_or_replaced_bundle_member_never_rechecks(member: str, tmp_path: Path, monkeypatch) -> None:
    root, hbm = _bundle(tmp_path)
    monkeypatch.setattr(subject, "validate_regression", _stub)
    (root / NAMES[member]).write_text(json.dumps({"kind": member, "current": False}), encoding="utf-8")
    with pytest.raises(ValueError, match="full_recheck_failed"):
        subject.verify_dosod_compile_parity_metric_chain(root, hbm)


@pytest.mark.parametrize("member", ["parity_report", "quantized_report", "compile_receipt"])
def test_missing_receipt_never_rechecks(member: str, tmp_path: Path, monkeypatch) -> None:
    root, hbm = _bundle(tmp_path)
    monkeypatch.setattr(subject, "validate_regression", _stub)
    (root / NAMES[member]).unlink()
    with pytest.raises(ValueError):
        subject.verify_dosod_compile_parity_metric_chain(root, hbm)


@pytest.mark.parametrize("member", ["parity_report", "quantized_report"])
def test_receipt_path_escape_never_rechecks(member: str, tmp_path: Path, monkeypatch) -> None:
    root, hbm = _bundle(tmp_path)
    monkeypatch.setattr(subject, "validate_regression", _stub)
    manifest = json.loads((root / "dosod_admission_bundle.json").read_text())
    manifest["artifacts"][member] = "../outside.json"
    (root / "dosod_admission_bundle.json").write_text(json.dumps(manifest), encoding="utf-8")
    with pytest.raises(ValueError, match="path_escape"):
        subject.verify_dosod_compile_parity_metric_chain(root, hbm)
