from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from audit_native_brep_source_coverage import (
    EXPLICIT,
    HIGH_LEVEL,
    REPORT_ID,
    SUPPLIER_EXCLUDED,
    UNCOVERED,
    audit,
)


ROOT = Path(__file__).resolve().parents[1]


def test_crosswalk_is_complete_and_fail_closed() -> None:
    report = audit(ROOT)
    assert report["report_id"] == REPORT_ID
    assert report["manifest_status_preserved"] == "pending_native_brep_reconstruction"
    assert report["project_part_count"] == 105
    assert report["supplier_excluded_count"] == 21
    assert len(report["rows"]) == 126
    counts = report["counts_by_category"]
    assert counts[EXPLICIT] == 105
    assert counts[HIGH_LEVEL] == 0
    assert counts[UNCOVERED] == 0
    assert counts[SUPPLIER_EXCLUDED] == 21
    assert report["status"] == "STATIC_INDIVIDUAL_COVERAGE_CLOSED"
    assert report["runtime_accepted"] is False
    assert report["native_cad_delivery_accepted"] is False
    assert all(batch["source_integrity_passed"] for batch in report["batches"])
    assert all(batch["all_named_builders_present"] for batch in report["batches"])


def test_exact_mesh_proof_closes_only_explicitly_mapped_rows() -> None:
    report = audit(ROOT)
    rows = {row.get("manifest_part_id"): row for row in report["rows"] if row.get("manifest_part_id")}
    assert rows["cleaning_side_brush_disk"]["coverage_category"] == EXPLICIT
    assert rows["storage_dry_bin_lid"]["coverage_category"] == EXPLICIT
    assert rows["storage_dry_bin_lid"]["exact_source_batches"] == [
        "seventh_storage_service_per_part"
    ]


def test_cli_writes_machine_readable_static_report(tmp_path: Path) -> None:
    output = tmp_path / "coverage.json"
    subprocess.run(
        [sys.executable, str(ROOT / "scripts/audit_native_brep_source_coverage.py"), "--root", str(ROOT), "--output", str(output)],
        check=True,
        capture_output=True,
        text=True,
    )
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["audit_mode"] == "static_json_python_crosswalk_only"
    assert "CadQuery" in report["execution_prohibited"]
