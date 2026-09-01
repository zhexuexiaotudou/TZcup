#!/usr/bin/env python3
"""Fail-closed source audit for formal operating-speed eligibility.

This checker deliberately cannot turn a theoretical throughput budget into
formal acceptance evidence.  It reports the current runner as not ready until
a source-bound measured coverage gate exists and the dry candidate passes it.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
PROFILES = ROOT / "config/high_fidelity_vehicle/formal_operation_speed_profiles.yaml"
NAV2 = ROOT / "starter_ws/src/sanitation_navigation/config/nav2.yaml"
FINAL_RUNNER = ROOT / "scripts/run_formal_final_acceptance.py"
PRODUCT_LAUNCH = ROOT / "starter_ws/src/sanitation_product_demo_integration/launch/product_demo.launch.py"
MAP_LIFECYCLE_LAUNCH = ROOT / "starter_ws/src/sanitation_formal_campus_integration/launch/formal_campus_map_lifecycle.launch.py"


class FormalOperationSpeedProfileError(RuntimeError):
    """Raised when a speed-profile source contract is unsafe or ambiguous."""


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise FormalOperationSpeedProfileError(f"{label} must be a mapping")
    return value


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FormalOperationSpeedProfileError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise FormalOperationSpeedProfileError(f"{label} must be finite and positive")
    return result


def _boolean(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise FormalOperationSpeedProfileError(f"{label} must be a boolean")
    return value


def _required_gates(profile: dict[str, Any], label: str) -> list[str]:
    gates = profile.get("required_gates")
    if not isinstance(gates, list) or not gates or any(
        not isinstance(gate, str) or not gate for gate in gates
    ):
        raise FormalOperationSpeedProfileError(f"{label}.required_gates must be a non-empty string list")
    return gates


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise FormalOperationSpeedProfileError(f"cannot read {path}") from exc
    return _mapping(value, str(path))


def validate(
    *,
    profiles_path: Path = PROFILES,
    nav2_path: Path = NAV2,
    final_runner_path: Path = FINAL_RUNNER,
    product_launch_path: Path = PRODUCT_LAUNCH,
    map_lifecycle_launch_path: Path = MAP_LIFECYCLE_LAUNCH,
) -> dict[str, Any]:
    profiles = _load_yaml(profiles_path)
    if profiles.get("schema_version") != 1:
        raise FormalOperationSpeedProfileError("unsupported speed-profile schema_version")
    contract = _mapping(profiles.get("competition_efficiency_contract"), "competition_efficiency_contract")
    width = _number(contract.get("working_width_m"), "working_width_m")
    route_efficiency = _number(contract.get("route_efficiency"), "route_efficiency")
    if route_efficiency > 1.0:
        raise FormalOperationSpeedProfileError("route_efficiency cannot exceed one")
    target_area = _number(contract.get("minimum_effective_area_m2_h"), "minimum_effective_area_m2_h")
    measured_gate = contract.get("source_bound_measured_coverage_gate")
    if not isinstance(measured_gate, str) or not measured_gate:
        raise FormalOperationSpeedProfileError("source-bound measured coverage gate is required")
    if _boolean(contract.get("static_theory_is_not_acceptance_evidence"), "static_theory_is_not_acceptance_evidence") is not True:
        raise FormalOperationSpeedProfileError("static theory must remain non-acceptance evidence")

    entries = _mapping(profiles.get("profiles"), "profiles")
    required_names = {"mapping_safe", "dry_cleaning_competition_candidate", "wet_puddle_recovery"}
    if set(entries) != required_names:
        raise FormalOperationSpeedProfileError("speed profiles must contain exactly mapping, dry-candidate and wet entries")
    mapping_safe = _mapping(entries["mapping_safe"], "mapping_safe")
    dry = _mapping(entries["dry_cleaning_competition_candidate"], "dry_cleaning_competition_candidate")
    wet = _mapping(entries["wet_puddle_recovery"], "wet_puddle_recovery")

    mapping_speed = _number(mapping_safe.get("maximum_linear_speed_m_s"), "mapping_safe.maximum_linear_speed_m_s")
    if not math.isclose(mapping_speed, 0.45, abs_tol=1e-12):
        raise FormalOperationSpeedProfileError("mapping_safe must retain the current 0.45 m/s safety ceiling")
    if _boolean(mapping_safe.get("enabled_for_formal_runtime"), "mapping_safe.enabled_for_formal_runtime") is not True:
        raise FormalOperationSpeedProfileError("mapping_safe must remain enabled for the current formal runtime")
    if _boolean(mapping_safe.get("competition_efficiency_eligible"), "mapping_safe.competition_efficiency_eligible"):
        raise FormalOperationSpeedProfileError("mapping_safe cannot claim competition efficiency")
    _required_gates(mapping_safe, "mapping_safe")

    dry_speed = _number(dry.get("target_linear_speed_m_s"), "dry.target_linear_speed_m_s")
    if not math.isclose(dry_speed, 1.0, abs_tol=1e-12):
        raise FormalOperationSpeedProfileError("dry candidate target must remain 1.0 m/s")
    theoretical_floor = target_area / (width * route_efficiency * 3600.0)
    declared_floor = _number(dry.get("minimum_theoretical_speed_m_s"), "dry.minimum_theoretical_speed_m_s")
    if not math.isclose(declared_floor, theoretical_floor, rel_tol=0.0, abs_tol=1e-12):
        raise FormalOperationSpeedProfileError("dry candidate theoretical speed floor must be recomputed exactly from width, route efficiency and target")
    conservative_floor = _number(dry.get("conservative_candidate_speed_floor_m_s"), "dry.conservative_candidate_speed_floor_m_s")
    if conservative_floor < theoretical_floor:
        raise FormalOperationSpeedProfileError("conservative dry candidate floor cannot be below the exact theoretical floor")
    if _boolean(dry.get("enabled_for_formal_runtime"), "dry.enabled_for_formal_runtime"):
        raise FormalOperationSpeedProfileError("unverified dry candidate cannot be enabled for formal runtime")
    if _boolean(dry.get("competition_efficiency_eligible"), "dry.competition_efficiency_eligible"):
        raise FormalOperationSpeedProfileError("unverified dry candidate cannot claim competition efficiency")
    if dry.get("validation_status") != "PENDING_SOURCE_BOUND_GAZEBO_AND_A300_TRACKING":
        raise FormalOperationSpeedProfileError("dry candidate must remain pending source-bound Gazebo and A300 tracking")
    if measured_gate not in _required_gates(dry, "dry_cleaning_competition_candidate"):
        raise FormalOperationSpeedProfileError("dry candidate must require the source-bound measured coverage gate")

    wet_limits = _mapping(wet.get("speed_limits_m_s"), "wet.speed_limits_m_s")
    if not math.isclose(_number(wet_limits.get("nominal_depth_0_002_m"), "wet nominal speed"), 0.115899, abs_tol=1e-12):
        raise FormalOperationSpeedProfileError("wet nominal-depth speed must match the existing hydraulic bound")
    if not math.isclose(_number(wet_limits.get("maximum_depth_0_010_m"), "wet maximum speed"), 0.023180, abs_tol=1e-12):
        raise FormalOperationSpeedProfileError("wet maximum-depth speed must match the existing hydraulic bound")
    if _boolean(wet.get("enabled_for_formal_runtime"), "wet.enabled_for_formal_runtime"):
        raise FormalOperationSpeedProfileError("wet recovery cannot be enabled before source-bound runtime validation")
    if _boolean(wet.get("competition_efficiency_eligible"), "wet.competition_efficiency_eligible"):
        raise FormalOperationSpeedProfileError("wet recovery cannot claim dry competition efficiency")
    _required_gates(wet, "wet_puddle_recovery")

    nav2 = _load_yaml(nav2_path)
    try:
        nav_parameters = nav2["controller_server"]["ros__parameters"]
        clean_path_speed = _number(nav_parameters["CleanPath"]["desired_linear_vel"], "nav2 CleanPath speed")
        smoother_speed = _number(nav2["velocity_smoother"]["ros__parameters"]["max_velocity"][0], "nav2 smoother speed")
    except (KeyError, IndexError, TypeError) as exc:
        raise FormalOperationSpeedProfileError("nav2 formal speed slots are missing") from exc
    final_runner = final_runner_path.read_text(encoding="utf-8")
    product_launch = product_launch_path.read_text(encoding="utf-8")
    lifecycle_launch = map_lifecycle_launch_path.read_text(encoding="utf-8")
    runner_uses_formal_e2e = "run_formal_single_episode_cleaning_mission.sh" in final_runner
    e2e_uses_formal_campus = "formal_campus_map_lifecycle.launch.py" in product_launch
    campus_uses_default_nav2 = '"nav2.yaml"' in lifecycle_launch
    mapping_speed_active = (
        runner_uses_formal_e2e
        and e2e_uses_formal_campus
        and campus_uses_default_nav2
        and math.isclose(clean_path_speed, mapping_speed, abs_tol=1e-12)
        and math.isclose(smoother_speed, mapping_speed, abs_tol=1e-12)
    )
    dry_profile_verified = (
        _boolean(dry.get("enabled_for_formal_runtime"), "dry.enabled_for_formal_runtime")
        and _boolean(dry.get("competition_efficiency_eligible"), "dry.competition_efficiency_eligible")
        and dry.get("validation_status") == "PASSED_SOURCE_BOUND_MEASURED_COVERAGE"
    )
    not_ready_reasons: list[str] = []
    if mapping_speed_active:
        not_ready_reasons.append("final_runner_still_uses_mapping_safe_0_45_m_s")
    if not dry_profile_verified:
        not_ready_reasons.append("dry_candidate_has_no_passed_source_bound_measured_coverage_gate")
    status = (
        "NOT_READY_FOR_COMPETITION_EFFICIENCY"
        if not_ready_reasons
        else "FORMAL_OPERATION_SPEED_PROFILE_READY"
    )
    mapping_theoretical_area = width * mapping_speed * 3600.0
    mapping_effective_area = mapping_theoretical_area * route_efficiency
    dry_theoretical_area = width * dry_speed * 3600.0 * route_efficiency
    return {
        "status": status,
        "static_theory_is_not_acceptance_evidence": True,
        "source_bound_measured_coverage_gate": measured_gate,
        "current_final_runner_uses_mapping_safe_0_45_m_s": mapping_speed_active,
        "dry_candidate_enabled_for_formal_runtime": dry_profile_verified,
        "not_ready_reasons": not_ready_reasons,
        "mapping_theoretical_area_m2_h": mapping_theoretical_area,
        "mapping_effective_area_m2_h": mapping_effective_area,
        "mapping_can_meet_competition_efficiency": mapping_effective_area >= target_area,
        "dry_candidate_design_area_m2_h": dry_theoretical_area,
        "dry_candidate_margin_m2_h": dry_theoretical_area - target_area,
        "dry_candidate_exact_minimum_theoretical_speed_m_s": theoretical_floor,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = validate()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if result["status"] == "FORMAL_OPERATION_SPEED_PROFILE_READY" else 1


if __name__ == "__main__":
    raise SystemExit(main())
