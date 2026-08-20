#!/usr/bin/env python3
"""Fail-closed drive/coverage profile compatibility resolution.

Used by the launcher scripts (bash and PowerShell) and by the fast CI tests.
The Ackermann drive model can only run the Ackermann coverage profile, the
Ackermann Nav2 profile, the Ackermann EKF and the dedicated turning-apron
world.  The skid-steer legacy drive can never run Ackermann connectors.
"""

from __future__ import annotations

from dataclasses import dataclass


DRIVE_MODELS = ("ackermann", "skid_steer_legacy")
COVERAGE_PROFILES = ("ackermann", "optimized", "legacy")


@dataclass(frozen=True)
class ProfileResolution:
    drive_model: str
    coverage_profile: str
    nav2_params: str
    coverage_params: str
    mission_template: str
    world_file: str
    world_name: str
    ekf_config: str
    wheel_odom_input: str
    profile_label: str


def resolve_profiles(
    drive_model: str,
    coverage_profile: str,
    *,
    nav2_base: str = "nav2.yaml",
    nav2_ackermann: str = "nav2_ackermann.yaml",
    coverage_skid_optimized: str = "coverage_skid_steer_optimized.yaml",
    coverage_legacy: str = "coverage_demo_overlap.yaml",
    coverage_ackermann: str = "coverage_ackermann.yaml",
    mission_skid_optimized: str = "competition_demo_area_skid_steer_optimized.yaml",
    mission_legacy: str = "competition_demo_area.yaml",
    mission_ackermann: str = "competition_ackermann_demo_area.yaml",
    world_skid: str = "sanitation_competition_demo.sdf",
    world_ackermann: str = "sanitation_competition_ackermann_demo.sdf",
    ekf_legacy: str = "selected_ekf.yaml",
    ekf_ackermann: str = "ekf_ackermann.yaml",
) -> ProfileResolution:
    """Resolve the exact profile combination, failing closed on mismatch."""
    if drive_model not in DRIVE_MODELS:
        raise ValueError(
            f"drive_model must be one of {DRIVE_MODELS}, got {drive_model!r}"
        )
    if coverage_profile not in COVERAGE_PROFILES:
        raise ValueError(
            f"coverage_profile must be one of {COVERAGE_PROFILES}, got "
            f"{coverage_profile!r}"
        )
    if drive_model == "ackermann":
        if coverage_profile != "ackermann":
            raise ValueError(
                "incompatible combination: ackermann drive model requires "
                "coverage_profile='ackermann' (RTR/rotate-to-heading/Spin "
                "profiles are impossible for the Ackermann chassis)"
            )
        return ProfileResolution(
            drive_model="ackermann",
            coverage_profile="ackermann",
            nav2_params=nav2_ackermann,
            coverage_params=coverage_ackermann,
            mission_template=mission_ackermann,
            world_file=world_ackermann,
            world_name="sanitation_competition_ackermann_demo",
            ekf_config=ekf_ackermann,
            wheel_odom_input="/wheel/odom_raw",
            profile_label="ACKERMANN REALISM DEMO",
        )
    # skid_steer_legacy
    if coverage_profile == "ackermann":
        raise ValueError(
            "incompatible combination: skid_steer_legacy drive cannot run "
            "the Ackermann coverage connectors; choose optimized or legacy"
        )
    if coverage_profile == "optimized":
        return ProfileResolution(
            drive_model="skid_steer_legacy",
            coverage_profile="optimized",
            nav2_params=nav2_base,
            coverage_params=coverage_skid_optimized,
            mission_template=mission_skid_optimized,
            world_file=world_skid,
            world_name="sanitation_competition_demo",
            ekf_config=ekf_legacy,
            wheel_odom_input="/odom/unfiltered",
            profile_label="SKID-STEER OPTIMIZED DEMO",
        )
    return ProfileResolution(
        drive_model="skid_steer_legacy",
        coverage_profile="legacy",
        nav2_params=nav2_base,
        coverage_params=coverage_legacy,
        mission_template=mission_legacy,
        world_file=world_skid,
        world_name="sanitation_competition_demo",
        ekf_config=ekf_legacy,
        wheel_odom_input="/odom/unfiltered",
        profile_label="LEGACY DUBINS BASELINE",
    )
