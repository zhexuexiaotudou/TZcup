from __future__ import annotations

import json
from pathlib import Path

from generate_competition_gazebo_profile import generate


def test_competition_profile_is_full_scale_but_truthfully_zone_bounded(tmp_path: Path) -> None:
    manifest = generate(tmp_path)

    assert manifest["full_map"]["area_m2"] == 20_000.0
    assert manifest["full_map"]["cells"] == [2000, 1000]
    assert manifest["full_map"]["zone_count"] == 20
    assert manifest["live_demonstration"]["area_m2"] == 108.0
    assert manifest["truth_level"] == "LIVE_REPRESENTATIVE_ZONE_ON_FULL_SCALE_MAP"
    assert manifest["competition_truth"]["simulation_competition_matrix_pass"] is False
    assert manifest["competition_truth"]["final_competition_evidence_complete"] is False

    pgm = (tmp_path / "competition_map.pgm").read_bytes()
    assert pgm.startswith(b"P5\n2000 1000\n255\n")
    assert len(pgm) == len(b"P5\n2000 1000\n255\n") + 2_000_000
    mission = (tmp_path / "competition_zone_auto12.yaml").read_text(encoding="utf-8")
    assert "operation_width_m: 1.32" in mission
    assert "full_map_area_m2: 20000.0" in mission
    assert "live_zone_area_m2: 108.0" in mission
    saved = json.loads(
        (tmp_path / "competition_profile_manifest.json").read_text(encoding="utf-8")
    )
    assert saved == manifest


def test_visual_launcher_exposes_competition_profile() -> None:
    root = Path(__file__).resolve().parents[1]
    bash = (root / "scripts" / "run_visual_demo.sh").read_text(encoding="utf-8")
    powershell = (root / "scripts" / "run_visual_demo.ps1").read_text(encoding="utf-8")

    assert "--competition-profile" in bash
    assert "generate_competition_gazebo_profile.py" in bash
    assert 'cleaning_width="1.32"' in bash
    assert 'map_area_m2="20000"' in bash
    assert "[switch]$CompetitionProfile" in powershell
