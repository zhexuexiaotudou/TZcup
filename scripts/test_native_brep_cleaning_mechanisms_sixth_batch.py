from __future__ import annotations

import ast
import importlib.util
import json
from pathlib import Path

import pytest

import audit_native_brep_source_coverage as coverage


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = ROOT / "config/high_fidelity_vehicle/native_brep_cleaning_mechanisms_sixth_batch_contract.json"
SOURCE_PATH = ROOT / "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_cleaning_mechanisms_sixth_batch.py"
MANIFEST_PATH = ROOT / "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_cleaning_mechanisms_sixth_batch_source_manifest.json"


def _sixth_batch() -> dict[str, str]:
    return {
        "id": "sixth",
        "contract": str(CONTRACT_PATH.relative_to(ROOT)).replace("\\", "/"),
        "source": str(SOURCE_PATH.relative_to(ROOT)).replace("\\", "/"),
        "source_manifest": str(MANIFEST_PATH.relative_to(ROOT)).replace("\\", "/"),
    }


def _load_source_module():
    spec = importlib.util.spec_from_file_location("native_brep_cleaning_mechanisms_sixth_batch", SOURCE_PATH)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_contract_is_a_unique_per_cleaning_mesh_crosswalk() -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    reconstruction = json.loads((ROOT / "config/high_fidelity_vehicle/native_brep_reconstruction_manifest.json").read_text(encoding="utf-8"))
    expected = {part["source_mesh"] for part in reconstruction["parts"] if part["profile"] == "cleaning"}
    actual = [item["source_mesh"].replace("meshes/", "", 1) for item in contract["items"]]
    assert len(expected) == 23
    assert len(actual) == 23
    assert len(set(actual)) == 23
    assert set(actual) == expected
    assert contract["status"] == "design_input_pending_native_export"
    assert contract["planned_native_export"]["must_not_exist_yet"] is True
    assert "pump_curve_and_isolator_dynamics" in contract["pending_manufacturing_inputs"]


def test_source_has_a_builder_for_every_unique_contract_item_and_no_mesh_import() -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
    functions = {node.name for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}
    assert {f"_build_{item['id']}" for item in contract["items"]} <= functions
    assert "build_design_input_assembly" in functions
    top_level_imports = [
        alias.name
        for node in tree.body
        if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in node.names
    ]
    assert "cadquery" not in top_level_imports
    assert "require_cadquery" in functions
    source = SOURCE_PATH.read_text(encoding="utf-8")
    assert "assembly.add(build_design_input_shape" in source
    assert "single_fused_cleaning_mechanism_substitute" in CONTRACT_PATH.read_text(encoding="utf-8")
    assert "extrude(mm(length) / 2.0, both=True)" in source


def test_coverage_auditor_can_read_sixth_batch_and_promotes_all_cleaning_rows(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(coverage, "BATCHES", (*coverage.BATCHES, _sixth_batch()))
    report = coverage.audit(ROOT)
    rows = [row for row in report["rows"] if row.get("profile") == "cleaning"]
    assert len(rows) == 23
    assert {row["coverage_category"] for row in rows} == {coverage.EXPLICIT}
    assert all("sixth" in row["exact_source_batches"] for row in rows)
    sixth = next(batch for batch in report["batches"] if batch["id"] == "sixth")
    assert sixth["source_integrity_passed"] is True
    assert sixth["all_named_builders_present"] is True
    assert report["native_cad_delivery_accepted"] is False


def test_export_is_blocked_before_cadquery_is_loaded() -> None:
    module = _load_source_module()
    with pytest.raises(module.ExportBlocked, match="forbids native export"):
        module.validate_release_authorization(module.load_contract(ROOT))
