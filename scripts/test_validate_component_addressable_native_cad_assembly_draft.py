from __future__ import annotations

import copy
import json
from pathlib import Path

from validate_component_addressable_native_cad_assembly_draft import DRAFT, ROOT, audit, validate


def test_draft_addresses_every_project_part_but_remains_export_blocked() -> None:
    report = audit(ROOT)
    assert report["status"] == "STATIC_COMPONENT_ADDRESSABLE_DRAFT_VALID_NATIVE_EXPORT_BLOCKED"
    assert report["draft_structurally_valid"] is True
    assert report["component_count"] == 105
    assert report["supplier_excluded_count"] == 21
    assert report["native_cad_assembly_ready"] is False
    assert report["native_cad_delivery_accepted"] is False
    assert "NO_NATIVE_STEP_OR_FCSTD" in report["blockers"]


def test_duplicate_or_missing_component_fails_closed() -> None:
    payload = json.loads((ROOT / DRAFT).read_text(encoding="utf-8"))
    broken = copy.deepcopy(payload)
    broken["components"][1]["manifest_part_id"] = broken["components"][0]["manifest_part_id"]
    codes = {gap["code"] for gap in validate(broken, ROOT)}
    assert "PROJECT_COMPONENT_ID_UNPROVEN" in codes
    assert "PROJECT_COMPONENT_SET_INCOMPLETE" in codes


def test_supplier_references_cannot_be_silently_promoted() -> None:
    payload = json.loads((ROOT / DRAFT).read_text(encoding="utf-8"))
    broken = copy.deepcopy(payload)
    broken["supplier_excluded_components"][0]["status"] = "design_input_pending_native_export"
    assert "SUPPLIER_EXCLUSION_INVALID" in {gap["code"] for gap in validate(broken, ROOT)}
