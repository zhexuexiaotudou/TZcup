from __future__ import annotations

import copy
import json
from pathlib import Path

from validate_per_part_native_cad_release_gap_register import REGISTER, ROOT, audit, validate


def test_register_covers_105_pending_project_parts_with_eight_contract_contexts() -> None:
    report = audit(ROOT)
    assert report["status"] == "STATIC_PER_PART_RELEASE_GAPS_VALID_NATIVE_RELEASE_BLOCKED"
    assert report["valid"] is True
    assert report["part_count"] == 105
    assert report["supplier_excluded_count"] == 21
    assert report["unresolved_gate_count"] > 0
    assert report["native_cad_release_ready"] is False
    assert report["manufacturing_release_ready"] is False
    assert "material_and_finish" in report["unresolved_gates_by_category"]
    assert "ALL_PROJECT_PARTS_HAVE_UNRESOLVED_RELEASE_GATES" in report["blockers"]


def test_unknown_gate_or_false_release_claim_fails_closed() -> None:
    payload = json.loads((ROOT / REGISTER).read_text(encoding="utf-8"))
    broken = copy.deepcopy(payload)
    broken["parts"][0]["unresolved_gate_ids"] = ["unknown"]
    broken["native_cad_release_ready"] = True
    codes = {gap["code"] for gap in validate(broken, ROOT)}
    assert "PART_RELEASE_GATES_UNPROVEN" in codes
    assert "FALSE_RELEASE_CLAIM" in codes
