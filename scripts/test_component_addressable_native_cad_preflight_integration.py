from __future__ import annotations

from pathlib import Path

import audit_native_cad_readiness as readiness


ROOT = Path(__file__).resolve().parents[1]


def test_valid_component_addressable_draft_is_inventory_not_release_evidence() -> None:
    report = readiness.build_report(ROOT)
    assert report["outcome"] == "blocked"
    assert report["native_editable_step_assembly_ready"] is False
    assert report["inventory"]["component_addressable_assembly_draft"]["valid"] is True
    assert report["inventory"]["component_addressable_assembly_draft"]["component_count"] == 105
    assert report["inventory"]["component_addressable_assembly_draft"]["supplier_excluded_count"] == 21
    codes = {gap["code"] for gap in report["gaps"]}
    assert "NATIVE_ASSEMBLY_MANIFEST_DRAFT_NOT_RELEASED" in codes
    assert "NO_NATIVE_ASSEMBLY_MANIFEST" not in codes
    assert "NO_NATIVE_CAD_EXPORT_RECEIPT" in codes
