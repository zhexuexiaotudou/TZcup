from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from generate_static_functional_chain_audit import (
    REPORT_ID,
    REQUIRED_ITEM_IDS,
    SUPPLEMENTAL_ITEM_IDS,
    audit,
)
from validate_static_functional_chain_audit import validate


ROOT = Path(__file__).resolve().parents[1]


def test_audit_tracks_all_required_function_families() -> None:
    report = audit(ROOT)
    ids = {item["id"] for item in report["items"]}
    assert report["report_id"] == REPORT_ID
    assert tuple(item["id"] for item in report["items"]) == REQUIRED_ITEM_IDS
    assert {
        "mobility_forward_and_brake",
        "six_axis_arm_and_gripper",
        "cube_pick_and_rear_dry_bin_containment",
        "dry_garbage_increases_vehicle_mass",
        "ground_dirt_coverage",
        "brush_squeegee_water_to_wastewater_tank",
        "sensor_single_line_lidar",
        "sensor_mid360",
        "sensor_front_rgbd",
        "sensor_rear_fisheyes",
        "sensor_end_effector_stereo",
        "sensor_gnss",
        "sensor_wheel_speed",
    } <= ids
    supplemental = report["supplemental_items"]
    assert tuple(item["id"] for item in supplemental) == SUPPLEMENTAL_ITEM_IDS
    assert {"s100p_installation_and_low_voltage_power", "obstacle_avoidance_chain"} == {
        item["id"] for item in supplemental
    }


def test_dry_cube_mass_uses_exclusive_physical_resident_accounting() -> None:
    report = audit(ROOT)
    item = next(item for item in report["items"] if item["id"] == "dry_garbage_increases_vehicle_mass")
    assert item["status"] == "STATIC_CLOSED"
    assert item["blocked_reason"] is None
    assert report["blocked_items"] == []
    assert report["status"] == "STATIC_CLOSED"
    assert report["required_item_count"] == 13
    assert report["static_closed_count"] == 13
    assert report["runtime_accepted"] is False
    assert report["fresh_gazebo_runtime_required"] is True
    assert report["expanded_required_item_count"] == 15
    assert report["expanded_static_closed_count"] == 14
    assert report["expanded_scope_status"] == "BLOCKED"
    assert report["expanded_scope_runtime_accepted"] is False
    s100 = next(
        item
        for item in report["supplemental_items"]
        if item["id"] == "s100p_installation_and_low_voltage_power"
    )
    assert s100["status"] == "BLOCKED"
    assert s100["placeholder_indicators"]
    assert "validated S100P mechanical/electrical evidence contract remains BLOCKED" in s100["blocked_reason"]
    conclusion = report["s100p_mechanical_electrical_evidence"]
    assert conclusion["validator_complete"] is True
    assert conclusion["status"] == "BLOCKED_MECHANICAL_ELECTRICAL_INTEGRATION"
    assert conclusion["acceptance"]["runtime_accepted"] is False
    assert "installed_power_on_and_runtime_validation" in conclusion["blocked_gates"]
    validate(report)


def test_generator_and_validator_are_machine_readable(tmp_path: Path) -> None:
    output = tmp_path / "static_audit.json"
    subprocess.run(
        [sys.executable, str(ROOT / "scripts/generate_static_functional_chain_audit.py"), "--root", str(ROOT), "--output", str(output)],
        check=True,
        capture_output=True,
        text=True,
    )
    report = json.loads(output.read_text(encoding="utf-8"))
    validate(report)
    result = subprocess.run(
        [sys.executable, str(ROOT / "scripts/validate_static_functional_chain_audit.py"), "--report", str(output), "--require-static-closed"],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
