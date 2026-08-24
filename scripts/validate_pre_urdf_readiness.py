#!/usr/bin/env python3
"""Fail-closed validation for the formal vehicle's pre-URDF input contract."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config" / "high_fidelity_vehicle" / "pre_urdf_contract.yaml"
SHA_RE = re.compile(r"^[0-9a-f]{40}$")
ROS_NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")
ALLOWED_LICENSES = {"Apache-2.0", "BSD-3-Clause", "MIT"}
ALLOWED_JOINT_TYPES = {"fixed", "revolute", "continuous", "prismatic", "floating", "planar"}


class ContractError(ValueError):
    """Raised when an input would make the formal URDF claim unsafe."""


def load_contract(path: Path = DEFAULT_CONTRACT) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ContractError("contract root must be a mapping")
    return data


def _positive(value: Any, field: str, *, allow_zero: bool = False) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ContractError(f"{field} must be numeric") from exc
    if number < 0 or (number == 0 and not allow_zero):
        raise ContractError(f"{field} must be {'non-negative' if allow_zero else 'positive'}")
    return number


def _unique(values: list[str], field: str) -> None:
    if len(values) != len(set(values)):
        raise ContractError(f"{field} must be unique")


def validate_contract(contract: dict[str, Any]) -> dict[str, Any]:
    if contract.get("schema_version") != 1:
        raise ContractError("unsupported schema_version")
    if contract.get("scope", {}).get("creates_formal_urdf") is not False:
        raise ContractError("pre-URDF contract must not claim a formal URDF was created")

    repos = contract.get("source_repositories", [])
    repo_ids = [str(repo.get("id", "")) for repo in repos]
    _unique(repo_ids, "source repository ids")
    roles: set[str] = set()
    for repo in repos:
        if not SHA_RE.fullmatch(str(repo.get("commit", ""))):
            raise ContractError(f"repository {repo.get('id')} is not pinned to a 40-character commit")
        if repo.get("license") not in ALLOWED_LICENSES:
            raise ContractError(f"repository {repo.get('id')} has an unapproved or unknown license")
        roles.update(str(role) for role in repo.get("roles", []))
    missing_source_roles = sorted(set(contract.get("required_source_roles", [])) - roles)
    if missing_source_roles:
        raise ContractError("missing source roles: " + ", ".join(missing_source_roles))

    components = contract.get("component_selections", [])
    component_ids = [str(item.get("id", "")) for item in components]
    component_roles = [str(item.get("role", "")) for item in components]
    _unique(component_ids, "component ids")
    missing_component_roles = sorted(set(contract.get("required_component_roles", [])) - set(component_roles))
    if missing_component_roles:
        raise ContractError("missing component roles: " + ", ".join(missing_component_roles))
    for item in components:
        if not item.get("model") or not item.get("source_class") or not item.get("datasheet_url"):
            raise ContractError(f"component {item.get('id')} lacks model/source/datasheet traceability")
        if item.get("mass_kg") is not None:
            _positive(item["mass_kg"], f"component {item.get('id')} mass_kg")

    sensors = contract.get("sensor_contracts", [])
    sensor_ids = [str(sensor.get("id", "")) for sensor in sensors]
    sensor_frames = [str(sensor.get("frame", "")) for sensor in sensors]
    sensor_topics = [str(sensor.get("topic", "")) for sensor in sensors]
    _unique(sensor_ids, "sensor ids")
    _unique(sensor_frames, "sensor frames")
    _unique(sensor_topics, "sensor topics")
    for sensor in sensors:
        if not str(sensor.get("topic", "")).startswith("/"):
            raise ContractError(f"sensor {sensor.get('id')} topic must be absolute")
        _positive(sensor.get("update_rate_hz"), f"sensor {sensor.get('id')} update rate")
        minimum = _positive(sensor.get("range_min_m"), f"sensor {sensor.get('id')} range minimum", allow_zero=True)
        maximum = _positive(sensor.get("range_max_m"), f"sensor {sensor.get('id')} range maximum")
        if maximum <= minimum:
            raise ContractError(f"sensor {sensor.get('id')} range maximum must exceed minimum")
        horizontal = _positive(sensor.get("horizontal_fov_deg"), f"sensor {sensor.get('id')} horizontal FOV")
        vertical = _positive(sensor.get("vertical_fov_deg"), f"sensor {sensor.get('id')} vertical FOV", allow_zero=True)
        if horizontal > 360.0 or vertical > 180.0:
            raise ContractError(f"sensor {sensor.get('id')} FOV exceeds physical bounds")
        dimensions = sensor.get("dimensions_m", [])
        if len(dimensions) != 3:
            raise ContractError(f"sensor {sensor.get('id')} must have three dimensions")
        for axis, dimension in zip("xyz", dimensions):
            _positive(dimension, f"sensor {sensor.get('id')} dimension {axis}")

    frames = [str(frame) for frame in contract.get("frame_contract", {}).get("required_frames", [])]
    _unique(frames, "required frames")
    for frame in frames:
        if not ROS_NAME_RE.fullmatch(frame):
            raise ContractError(f"invalid ROS frame name: {frame}")
    missing_sensor_frames = sorted(set(sensor_frames) - set(frames))
    if missing_sensor_frames:
        raise ContractError("sensor frames missing from frame contract: " + ", ".join(missing_sensor_frames))

    joints = contract.get("joint_contract", [])
    joint_names = [str(joint.get("name", "")) for joint in joints]
    _unique(joint_names, "joint names")
    for joint in joints:
        name = str(joint.get("name", ""))
        if not ROS_NAME_RE.fullmatch(name):
            raise ContractError(f"invalid joint name: {name}")
        if joint.get("type") not in ALLOWED_JOINT_TYPES:
            raise ContractError(f"joint {name} has unsupported type")
        if not joint.get("actuator") or "command_interface" not in joint:
            raise ContractError(f"joint {name} lacks actuator/interface contract")
        if joint.get("type") == "prismatic" and float(joint.get("upper_m", 0.0)) <= float(joint.get("lower_m", 0.0)):
            raise ContractError(f"joint {name} has invalid prismatic limits")
        if joint.get("type") == "revolute" and float(joint.get("upper_rad", 0.0)) <= float(joint.get("lower_rad", 0.0)):
            raise ContractError(f"joint {name} has invalid revolute limits")

    budget = contract.get("mass_capacity_budget", {})
    payload_limit = _positive(budget.get("a300_payload_limit_kg"), "A300 payload limit")
    design_factor = _positive(budget.get("payload_design_factor"), "payload design factor")
    if design_factor > 1.0:
        raise ContractError("payload design factor cannot exceed 1")
    design_limit = _positive(budget.get("payload_design_limit_kg"), "payload design limit")
    expected_design_limit = payload_limit * design_factor
    if abs(design_limit - expected_design_limit) > 1e-6:
        raise ContractError("payload design limit does not match payload limit times design factor")
    known_mass = sum(_positive(value, f"known payload {name}") for name, value in budget.get("known_payload_items", {}).items())
    allowance_mass = sum(_positive(value, f"engineering allowance {name}") for name, value in budget.get("engineering_allowances", {}).items())

    trash = budget.get("dry_trash", {})
    cube_count = int(_positive(trash.get("cube_count_max"), "cube count"))
    cube_edge = _positive(trash.get("cube_edge_m"), "cube edge")
    densities = trash.get("material_density_kg_m3", {})
    if not densities:
        raise ContractError("material density table is empty")
    density_max = 0.0
    for material, bounds in densities.items():
        if len(bounds) != 2:
            raise ContractError(f"material {material} must have min/max density")
        lower = _positive(bounds[0], f"material {material} minimum density")
        upper = _positive(bounds[1], f"material {material} maximum density")
        if upper < lower:
            raise ContractError(f"material {material} density range is inverted")
        density_max = max(density_max, upper)
    dry_trash_max = cube_count * cube_edge**3 * density_max

    dry_bin = budget.get("dry_bin", {})
    dry_min = _positive(contract.get("competition_requirements", {}).get("dry_bin_usable_min_l"), "required dry-bin volume")
    usable_dry = _positive(dry_bin.get("usable_volume_l"), "usable dry-bin volume")
    geometric_dry = _positive(dry_bin.get("geometric_volume_l"), "geometric dry-bin volume")
    if usable_dry < dry_min or geometric_dry < usable_dry:
        raise ContractError("dry-bin geometry does not preserve at least 40 L usable volume")
    if dry_bin.get("separated_from_wastewater") is not True:
        raise ContractError("dry and wastewater compartments must be separated")

    fixed_payload = known_mass + allowance_mass
    water_by_mass = design_limit - fixed_payload - dry_trash_max
    if water_by_mass <= 0:
        raise ContractError("fixed payload plus worst-case dry trash leaves no wastewater capacity")
    wastewater = budget.get("wastewater", {})
    density_kg_l = _positive(wastewater.get("density_kg_l"), "wastewater density")
    mass_limited_nominal = water_by_mass / density_kg_l
    nominal_cap = _positive(wastewater.get("nominal_design_cap_l"), "wastewater design cap")
    preliminary_nominal = min(mass_limited_nominal, nominal_cap)
    usable_fraction = _positive(wastewater.get("usable_fraction"), "wastewater usable fraction")
    episode_fraction = _positive(wastewater.get("episode_fraction_of_usable"), "episode fraction")
    if usable_fraction > 1.0 or episode_fraction > 1.0:
        raise ContractError("wastewater fractions cannot exceed 1")
    preliminary_usable = preliminary_nominal * usable_fraction
    episode_max = preliminary_usable * episode_fraction

    power = contract.get("power_budget", {})
    rails = power.get("base_user_rails", {})
    loads = power.get("loads", [])
    rail_peaks: dict[str, float] = {name: 0.0 for name in rails}
    for load in loads:
        continuous = _positive(load.get("continuous_w"), f"load {load.get('id')} continuous power")
        peak = _positive(load.get("peak_w"), f"load {load.get('id')} peak power")
        if peak < continuous:
            raise ContractError(f"load {load.get('id')} peak power is below continuous power")
        if load.get("rail") in rail_peaks:
            rail_peaks[str(load["rail"])] += peak
    policy = power.get("rail_policy", {})
    if rail_peaks.get("sensor_12v", 0.0) > _positive(policy.get("sensor_12v_peak_max_w"), "12 V rail power limit"):
        raise ContractError("12 V sensor rail peak budget exceeded")
    if rail_peaks.get("sensor_24v", 0.0) > _positive(policy.get("sensor_24v_peak_max_w"), "24 V rail power limit"):
        raise ContractError("24 V sensor rail peak budget exceeded")
    if policy.get("high_power_loads_require_isolated_vbat_dc_bus") is not True:
        raise ContractError("high-power isolation policy must remain enabled")

    gates = contract.get("layout_gates_during_urdf", [])
    gate_ids = [str(gate.get("id", "")) for gate in gates]
    _unique(gate_ids, "layout gate ids")
    required_gates = {
        "exact_s100_board_envelope",
        "full_inertia_and_cog_scan",
        "fov_occlusion_solve",
        "final_wastewater_capacity",
        "vendor_geometry_rights",
        "detailed_power_topology",
    }
    missing_gates = sorted(required_gates - set(gate_ids))
    if missing_gates:
        raise ContractError("missing formal-layout gates: " + ", ".join(missing_gates))

    return {
        "contract_id": contract["contract_id"],
        "status": contract["readiness_policy"]["ready_status"],
        "formal_urdf_created": False,
        "source_repository_count": len(repos),
        "selected_component_count": len(components),
        "sensor_contract_count": len(sensors),
        "required_frame_count": len(frames),
        "explicit_joint_count": len(joints),
        "known_payload_mass_kg": round(known_mass, 6),
        "engineering_allowance_mass_kg": round(allowance_mass, 6),
        "fixed_payload_budget_kg": round(fixed_payload, 6),
        "worst_case_dry_trash_mass_kg": round(dry_trash_max, 6),
        "mass_limited_wastewater_nominal_l": round(mass_limited_nominal, 6),
        "preliminary_wastewater_nominal_l": round(preliminary_nominal, 6),
        "preliminary_wastewater_usable_l": round(preliminary_usable, 6),
        "normal_episode_puddle_cap_l": round(episode_max, 6),
        "payload_margin_at_preliminary_usable_fill_kg": round(
            design_limit - fixed_payload - dry_trash_max - preliminary_usable * density_kg_l,
            6,
        ),
        "sensor_12v_peak_w": round(rail_peaks.get("sensor_12v", 0.0), 6),
        "sensor_24v_peak_w": round(rail_peaks.get("sensor_24v", 0.0), 6),
        "pending_layout_gates": gate_ids,
        "claim_boundary": "Ready to begin formal CAD/Xacro; final transforms, inertias, CoG, wastewater capacity, exact S100 envelope and power topology remain fail-closed layout gates.",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--expect-report", type=Path)
    args = parser.parse_args()
    result = validate_contract(load_contract(args.contract))
    if args.expect_report:
        expected = json.loads(args.expect_report.read_text(encoding="utf-8"))
        if result != expected:
            raise ContractError("committed readiness report differs from validated contract")
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
