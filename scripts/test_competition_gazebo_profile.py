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
    ackermann = (tmp_path / "competition_zone_ackermann.yaml").read_text(
        encoding="utf-8"
    )
    ackermann_coverage = (
        tmp_path / "competition_coverage_ackermann.yaml"
    ).read_text(encoding="utf-8")
    efficiency = (tmp_path / "competition_efficiency_ackermann.yaml").read_text(
        encoding="utf-8"
    )
    efficiency_coverage = (
        tmp_path / "competition_coverage_efficiency_ackermann.yaml"
    ).read_text(encoding="utf-8")
    assert "coverage_planner_profile: ACKERMANN" in ackermann
    assert "min_turning_radius_m: 1.4293521136632124" in ackermann
    assert "operation_width_m: 1.32" in ackermann
    assert "full_map_area_m2: 20000.0" in ackermann
    assert "live_zone_area_m2: 108.0" in ackermann
    assert "cleanable_outer_polygon:" in ackermann
    assert "ackermann_staging_offset_m: 1.0" in ackermann
    assert "ackermann_angle_connector_penalty_m: 22.6" in ackermann
    assert "swath_endpoint_extension_m: 3.00" in ackermann
    assert "    operation_width: 1.12" in ackermann_coverage
    assert "scope: long_lane_efficiency_candidate_on_full_competition_map" in efficiency
    assert "live_zone_area_m2: 10440.0" in efficiency
    assert "planning_swath_spacing_m: 1.20" in efficiency
    assert "ackermann_lane_skip: 3" in efficiency
    assert "CLEAN: {linear_mps: 1.00" in efficiency
    assert "    operation_width: 1.20" in efficiency_coverage
    assert "  - [10.0, 45.5]" in ackermann
    assert manifest["live_demonstration"]["ackermann_bounds_xyxy_m"] == [
        10.0, 45.5, 22.0, 54.5
    ]
    assert manifest["efficiency_candidate_lane"]["cleanable_area_m2"] == 10_440.0
    assert manifest["efficiency_candidate_lane"]["evidence_status"] == (
        "CONFIGURED_NOT_YET_EXECUTED_FULL_PIPELINE"
    )
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
    assert 'map_area_m2="20000.0"' in bash
    assert 'map_area_m2="4000.0"' in bash
    assert 'competition_zone_ackermann.yaml' in bash
    assert 'competition_coverage_ackermann.yaml' in bash
    assert 'competition_efficiency_ackermann.yaml' in bash
    assert 'competition_coverage_efficiency_ackermann.yaml' in bash
    assert 'controllers["CleanPath"]["min_approach_linear_velocity"] = 0.2' in bash
    assert 'controllers["CleanPath"]["approach_velocity_scaling_dist"] = 5.0' in bash
    assert '--competition-lane' in bash
    assert '[ValidateSet("representative", "efficiency")]' in powershell
    assert '"--competition-lane", $CompetitionLane' in powershell
    assert 'ACKERMANN COMPETITION MAP VALIDATION' in bash
    assert 'spawn_x="-94.80"' in bash
    assert 'initial_pose_y="45.95"' in bash
    assert bash.index('spawn_x="-90.0"') < bash.index('spawn_x="-94.80"')
    assert '"${DRIVE_MODEL}" == "ackermann" && "${COMPETITION_PROFILE}" -eq 0' in bash
    assert 'if [[ "${COMPETITION_PROFILE}" -eq 0 ]]; then\n    cp "${mission_template}" "${mission_config}"' in bash
    assert "[switch]$CompetitionProfile" in powershell
