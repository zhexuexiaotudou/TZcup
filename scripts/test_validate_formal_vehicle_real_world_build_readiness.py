from __future__ import annotations

from pathlib import Path
import shutil

import pytest
import yaml

import validate_formal_vehicle_real_world_build_readiness as readiness
from validate_formal_vehicle_real_world_build_readiness import DEFAULT_CONFIG, ROOT, validate


@pytest.fixture(autouse=True)
def _stub_current_snapshot(monkeypatch: pytest.MonkeyPatch) -> None:
    payload = yaml.safe_load(DEFAULT_CONFIG.read_text(encoding="utf-8"))
    snapshot = payload["current_snapshot"]
    artifact = payload["last_expanded_artifact"]
    component = payload["evidence_paths"]["component_register"]
    layout_sha256 = payload["evidence_paths"]["fov_occlusion"]["layout_sha256"]
    manifest = {
        "source_inventory_sha256": snapshot["source_inventory_sha256"],
        "output_inventory_sha256": snapshot["output_inventory_sha256"],
        "source_inventory": {
            "config/high_fidelity_vehicle/formal_vehicle_layout.yaml": {"sha256": layout_sha256},
        },
        "outputs": {
            artifact["path"]: {"sha256": artifact["sha256"]},
            component["path"]: {"sha256": component["sha256"]},
        },
    }
    monkeypatch.setattr(readiness, "verify_snapshot", lambda *_args, **_kwargs: manifest)


def _payload() -> dict:
    return yaml.safe_load(DEFAULT_CONFIG.read_text(encoding="utf-8"))


def _write(tmp_path: Path, payload: dict) -> Path:
    for relative in (
        payload["last_expanded_artifact"]["path"],
        payload["current_snapshot"]["path"],
        payload["evidence_paths"]["component_register"]["path"],
        payload["evidence_paths"]["fov_occlusion"]["path"],
        payload["evidence_paths"]["inertia_swept_volume"]["path"],
        payload["evidence_paths"]["fov_occlusion"]["validator_path"],
        payload["evidence_paths"]["inertia_swept_volume"]["scanner_path"],
    ):
        target = tmp_path / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(ROOT / relative, target)
    config = tmp_path / "readiness.yaml"
    config.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return config


def test_valid_fail_closed_readiness_declaration(tmp_path: Path) -> None:
    result = validate(_write(tmp_path, _payload()), root=tmp_path)
    assert result["valid"] is True
    assert result["ready"] is False
    assert result["computed"]["physical_replacement_margin_kg"] == pytest.approx(0.032417)
    assert result["computed"]["conservative_remaining_margin_kg"] == pytest.approx(0.030417)
    assert result["computed"]["subsystem_mass_total_kg"] == pytest.approx(160.007583)
    assert result["computed"]["conservative_required_reduction_minimum_kg"] == pytest.approx(4.969583)
    assert result["computed"]["conservative_required_reduction_recommended_kg"] == pytest.approx(9.969583)
    assert result["computed"]["throughput_margin_m2_h"] == 64.0


def test_rejects_boolean_disguised_as_mass(tmp_path: Path) -> None:
    payload = _payload()
    payload["mass_budget"]["a300_design_load_kg"] = True
    result = validate(_write(tmp_path, payload), root=tmp_path)
    assert result["valid"] is False
    assert "mass_budget.a300_design_load_kg must be a finite number" in result["errors"]


def test_rejects_non_finite_mass(tmp_path: Path) -> None:
    payload = _payload()
    payload["mass_budget"]["a300_design_load_kg"] = float("nan")
    result = validate(_write(tmp_path, payload), root=tmp_path)
    assert result["valid"] is False
    assert "mass_budget.a300_design_load_kg must be a finite number" in result["errors"]


def test_rejects_incorrect_formula(tmp_path: Path) -> None:
    payload = _payload()
    payload["mass_budget"]["expected_conservative_remaining_margin_kg"] = 0.122
    result = validate(_write(tmp_path, payload), root=tmp_path)
    assert result["valid"] is False
    assert "mass_budget.expected_conservative_remaining_margin_kg formula mismatch" in result["errors"]


def test_rejects_incorrect_physical_replacement_formula(tmp_path: Path) -> None:
    payload = _payload()
    payload["mass_budget"]["expected_physical_replacement_margin_kg"] = 0.030417
    result = validate(_write(tmp_path, payload), root=tmp_path)
    assert result["valid"] is False
    assert "mass_budget.expected_physical_replacement_margin_kg formula mismatch" in result["errors"]


def test_rejects_incorrect_subsystem_total(tmp_path: Path) -> None:
    payload = _payload()
    payload["mass_budget"]["subsystem_mass_total_kg"] = 160.107583
    result = validate(_write(tmp_path, payload), root=tmp_path)
    assert result["valid"] is False
    assert "mass_budget.subsystem_mass_total_kg formula mismatch" in result["errors"]


def test_rejects_incorrect_minimum_manufacturing_margin_target(tmp_path: Path) -> None:
    payload = _payload()
    payload["mass_budget"]["manufacturing_margin_targets"]["minimum_margin_kg"] = 4.0
    result = validate(_write(tmp_path, payload), root=tmp_path)
    assert result["valid"] is False
    assert "mass_budget.manufacturing_margin_targets.minimum_margin_kg must equal 5.0" in result["errors"]


def test_rejects_boolean_manufacturing_margin_target(tmp_path: Path) -> None:
    payload = _payload()
    payload["mass_budget"]["manufacturing_margin_targets"]["recommended_margin_kg"] = True
    result = validate(_write(tmp_path, payload), root=tmp_path)
    assert result["valid"] is False
    assert "mass_budget.manufacturing_margin_targets.recommended_margin_kg must be a finite number" in result["errors"]


def test_rejects_non_finite_required_reduction(tmp_path: Path) -> None:
    payload = _payload()
    payload["mass_budget"]["manufacturing_margin_targets"]["conservative_required_reduction_minimum_kg"] = float("inf")
    result = validate(_write(tmp_path, payload), root=tmp_path)
    assert result["valid"] is False
    assert "mass_budget.manufacturing_margin_targets.conservative_required_reduction_minimum_kg must be a finite number" in result["errors"]


def test_rejects_ready_true(tmp_path: Path) -> None:
    payload = _payload()
    payload["ready"] = True
    result = validate(_write(tmp_path, payload), root=tmp_path)
    assert result["valid"] is False
    assert "ready must be boolean false" in result["errors"]


def test_rejects_missing_blocker_category(tmp_path: Path) -> None:
    payload = _payload()
    del payload["blocking_categories"]["s100"]
    result = validate(_write(tmp_path, payload), root=tmp_path)
    assert result["valid"] is False
    assert "blocking_categories must contain exactly every required blocker category" in result["errors"]


def test_rejects_current_static_fov_report_hash_drift(tmp_path: Path) -> None:
    payload = _payload()
    config = _write(tmp_path, payload)
    report = tmp_path / payload["evidence_paths"]["fov_occlusion"]["path"]
    report.write_bytes(report.read_bytes() + b"\n")
    result = validate(config, root=tmp_path)
    assert result["valid"] is False
    assert "evidence_paths.fov_occlusion.sha256 does not match the current file" in result["errors"]


def test_rejects_inertia_report_bound_to_a_different_urdf(tmp_path: Path) -> None:
    payload = _payload()
    payload["evidence_paths"]["inertia_swept_volume"]["urdf_sha256"] = "0" * 64
    result = validate(_write(tmp_path, payload), root=tmp_path)
    assert result["valid"] is False
    assert "inertia_swept_volume URDF hash is not bound to the current expanded URDF" in result["errors"]


def test_rejects_source_snapshot_that_is_no_longer_current(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    payload = _payload()
    monkeypatch.setattr(readiness, "verify_snapshot", lambda *_args, **_kwargs: (_ for _ in ()).throw(readiness.SnapshotError("source drift")))
    result = validate(_write(tmp_path, payload), root=tmp_path)
    assert result["valid"] is False
    assert "current source snapshot verification failed: source drift" in result["errors"]
