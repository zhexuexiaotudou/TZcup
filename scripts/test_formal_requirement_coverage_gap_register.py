from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from audit_formal_requirement_coverage import REPORT_ID, audit
from validate_formal_requirement_coverage_gap_register import validate


ROOT = Path(__file__).resolve().parents[1]


def test_register_covers_each_requested_family_and_keeps_runtime_blocked() -> None:
    report = audit(ROOT)
    ids = {item["id"] for item in report["items"]}
    assert report["report_id"] == REPORT_ID
    assert report["requirement_count"] == 16
    assert {
        "sensor_utm30lx", "sensor_mid360", "sensor_front_d435", "sensor_rear_fisheye_pair",
        "sensor_wrist_d435", "sensor_zed_f9p", "sensor_vn100", "sensor_wheel_encoders",
        "a300_chassis", "ur5e_six_axis_arm", "robotiq_2f85_gripper",
        "dry_bin_dynamic_payload", "brush_and_ground_dirt", "squeegee_and_water_recovery",
        "wastewater_tank_dynamic_load", "safety_interlocks",
    } == ids
    assert report["static_complete_count"] == 16
    assert report["static_gap_items"] == []
    assert report["acceptance_session"]["status"] == "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
    assert report["acceptance_session"]["evidence_key_count"] == 0
    assert report["runtime_accepted"] is False
    assert report["runtime_blocked_items"] == [item["id"] for item in report["items"]]
    assert all(item["runtime_accepted"] is False for item in report["items"])
    assert all(item["runtime_evidence_status"] == "MISSING_CURRENT_SESSION_GATE_EVIDENCE" for item in report["items"])
    validate(report)


def test_generator_and_validator_are_machine_readable_and_fail_closed(tmp_path: Path) -> None:
    output = tmp_path / "coverage.json"
    subprocess.run(
        [sys.executable, str(ROOT / "scripts/audit_formal_requirement_coverage.py"), "--root", str(ROOT), "--output", str(output)],
        check=True,
        capture_output=True,
        text=True,
    )
    report = json.loads(output.read_text(encoding="utf-8"))
    validate(report)
    report["runtime_accepted"] = True
    with pytest.raises(ValueError, match="never accept runtime"):
        validate(report)
