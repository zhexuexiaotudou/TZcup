from __future__ import annotations

import csv
from pathlib import Path
import shutil

import pytest
import yaml

import validate_formal_vehicle_mechanical_release_readiness as readiness
from validate_formal_vehicle_mechanical_release_readiness import DEFAULT_CONFIG, ROOT, validate


@pytest.fixture(autouse=True)
def _stub_current_snapshot(monkeypatch: pytest.MonkeyPatch) -> None:
    payload = yaml.safe_load(DEFAULT_CONFIG.read_text(encoding="utf-8"))
    baseline = payload["baseline"]
    snapshot = {
        "source_inventory_sha256": baseline["source_inventory_sha256"],
        "output_inventory_sha256": baseline["output_inventory_sha256"],
        "outputs": {
            baseline["expanded_urdf_path"]: {
                "sha256": baseline["expanded_urdf_sha256"],
            }
        },
    }
    monkeypatch.setattr(readiness, "verify_snapshot", lambda *_args, **_kwargs: snapshot)


def _fixture(tmp_path: Path) -> tuple[Path, Path]:
    payload = yaml.safe_load(DEFAULT_CONFIG.read_text(encoding="utf-8"))
    bom_relative = payload["baseline"]["bom_path"]
    bom_target = tmp_path / bom_relative
    bom_target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(ROOT / bom_relative, bom_target)
    for relative in (payload["baseline"]["expanded_urdf_path"], payload["baseline"]["source_snapshot_path"]):
        target = tmp_path / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(ROOT / relative, target)
    config = tmp_path / "mechanical_release_readiness.yaml"
    config.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return config, bom_target


def _read_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        return list(reader.fieldnames or []), list(reader)


def _write_rows(path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _write_payload(path: Path, payload: dict) -> None:
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def test_valid_nominal_fail_closed_mechanical_release_package(tmp_path: Path) -> None:
    config, _ = _fixture(tmp_path)
    result = validate(config, root=tmp_path)
    assert result["valid"] is True
    assert result["ready"] is False
    assert result["computed"]["baseline_allocated_bom_mass_kg"] == pytest.approx(160.007583)
    assert result["computed"]["bom_row_count"] >= 30


def test_manufacturing_release_doc_binds_current_nominal_snapshot() -> None:
    payload = yaml.safe_load(DEFAULT_CONFIG.read_text(encoding="utf-8"))
    baseline = payload["baseline"]
    document = (ROOT / "docs/formal-vehicle-manufacturing-release.md").read_text(
        encoding="utf-8"
    )
    for field in (
        "expanded_urdf_sha256",
        "source_snapshot_sha256",
        "source_inventory_sha256",
        "output_inventory_sha256",
        "bom_sha256",
    ):
        assert baseline[field] in document
    assert payload["nominal_mass_declaration"] in document
    assert "NOT_READY_FOR_MECHANICAL_MANUFACTURING_RELEASE" in document


def test_rejects_duplicate_part_id(tmp_path: Path) -> None:
    config, bom = _fixture(tmp_path)
    fields, rows = _read_rows(bom)
    rows[1]["part_id"] = rows[0]["part_id"]
    _write_rows(bom, fields, rows)
    result = validate(config, root=tmp_path)
    assert result["valid"] is False
    assert any("part_id must be unique" in error for error in result["errors"])


def test_rejects_non_finite_bom_mass(tmp_path: Path) -> None:
    config, bom = _fixture(tmp_path)
    fields, rows = _read_rows(bom)
    rows[0]["nominal_mass_kg"] = "nan"
    _write_rows(bom, fields, rows)
    result = validate(config, root=tmp_path)
    assert result["valid"] is False
    assert any("nominal_mass_kg must be blank for pending mass or a finite number" in error for error in result["errors"])


def test_rejects_non_numeric_bom_quantity(tmp_path: Path) -> None:
    config, bom = _fixture(tmp_path)
    fields, rows = _read_rows(bom)
    rows[0]["quantity"] = "true"
    _write_rows(bom, fields, rows)
    result = validate(config, root=tmp_path)
    assert result["valid"] is False
    assert any("quantity must be a positive integer" in error for error in result["errors"])


def test_rejects_incorrect_allocated_mass_total(tmp_path: Path) -> None:
    config, bom = _fixture(tmp_path)
    fields, rows = _read_rows(bom)
    rows[0]["nominal_mass_kg"] = "78.400000"
    _write_rows(bom, fields, rows)
    result = validate(config, root=tmp_path)
    assert result["valid"] is False
    assert "baseline allocated BOM mass must equal baseline.expanded_urdf_mass_kg" in result["errors"]


def test_rejects_boolean_mitigation_flag(tmp_path: Path) -> None:
    config, _ = _fixture(tmp_path)
    payload = yaml.safe_load(config.read_text(encoding="utf-8"))
    payload["mass_mitigation"]["minimum_margin_kg"] = True
    _write_payload(config, payload)
    result = validate(config, root=tmp_path)
    assert result["valid"] is False
    assert "mass_mitigation.minimum_margin_kg must be a finite number" in result["errors"]


def test_rejects_ready_release_item_without_evidence(tmp_path: Path) -> None:
    config, _ = _fixture(tmp_path)
    payload = yaml.safe_load(config.read_text(encoding="utf-8"))
    payload["release_items"]["step_models"] = {"state": "ready", "evidence_path": ""}
    _write_payload(config, payload)
    result = validate(config, root=tmp_path)
    assert result["valid"] is False
    assert "release_items.step_models.state must remain blocked" in result["errors"]
    assert "release_items.step_models.evidence_path must be non-empty" in result["errors"]


def test_rejects_visual_stl_as_step_evidence(tmp_path: Path) -> None:
    config, _ = _fixture(tmp_path)
    payload = yaml.safe_load(config.read_text(encoding="utf-8"))
    payload["release_items"]["step_models"] = {
        "state": "ready",
        "evidence_path": "starter_ws/src/sanitation_vehicle_description/meshes/project/bodywork/front_center_nose.stl",
    }
    _write_payload(config, payload)
    result = validate(config, root=tmp_path)
    assert result["valid"] is False
    assert "release_items.step_models cannot use a visual STL as manufacturing evidence" in result["errors"]


def test_rejects_expanded_urdf_hash_drift(tmp_path: Path) -> None:
    config, _ = _fixture(tmp_path)
    payload = yaml.safe_load(config.read_text(encoding="utf-8"))
    urdf = tmp_path / payload["baseline"]["expanded_urdf_path"]
    urdf.write_bytes(urdf.read_bytes() + b"\n")
    result = validate(config, root=tmp_path)
    assert result["valid"] is False
    assert "baseline.expanded_urdf_sha256 does not match the current file" in result["errors"]


def test_rejects_bom_hash_drift(tmp_path: Path) -> None:
    config, bom = _fixture(tmp_path)
    bom.write_bytes(bom.read_bytes() + b"\n")
    result = validate(config, root=tmp_path)
    assert result["valid"] is False
    assert "baseline.bom_sha256 does not match the current file" in result["errors"]
