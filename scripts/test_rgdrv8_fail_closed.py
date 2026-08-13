#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_fail_closed_publisher_requires_all_three_route_failures() -> None:
    source = (ROOT / "scripts/finalize_rgdrv8_model_blocker.py").read_text()
    assert '"MODEL_BLOCKED_INTERNAL_REAL_GAZEBO_DETECTOR": True' in source
    assert '"SIMULATION_PRODUCT_COMPLETE": False' in source
    assert '"selected_route": None' in source
    assert "Route A did not fail" in source
    assert "Route B did not fail" in source
    assert "Route C specialist did not fail" in source
    assert '"VAL_NEW_read": False' in source
    assert '"G5_V2_read": False' in source
    assert "RGDRV8_GA1_FAILURE_TAXONOMY.json" in source


def test_fail_closed_publisher_produces_required_deliverables() -> None:
    source = (ROOT / "scripts/finalize_rgdrv8_model_blocker.py").read_text()
    for name in (
        "NEXT_ARCHITECTURE_RESEARCH_REQUIRED.json",
        "PERCEPTION_RGDRV8_FINAL_STATUS.json",
        "PERCEPTION_RGDRV8_FINAL_BLOCKERS.json",
        "PERCEPTION_RGDRV8_EVIDENCE_INDEX.md",
        "PERCEPTION_RGDRV8_MODEL_REGISTRY.json",
        "PERCEPTION_RGDRV8_RELEASE_MANIFEST.json",
        "PERCEPTION_RGDRV8_THIRD_PARTY_NOTICES.md",
        "REAL_GAZEBO_DETECTOR_RECOVERY_V8_REPORT.md",
    ):
        assert name in source
