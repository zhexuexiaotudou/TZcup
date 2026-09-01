#!/usr/bin/env python3
"""CadQuery B-rep source for the project-authored storage/service third batch.

The source is intentionally importable without CadQuery.  It reconstructs
editable, project-owned outer solids from the checked-in parameter contract;
it never reads a mesh, launches FreeCAD, or writes FCStd/STEP.  CadQuery is
loaded only after the release gate has passed.  The current contract is
deliberately pending, so the command-line path fails closed before that import.

All dimensions are metres in the contract and change to millimetres solely at
the CadQuery boundary.  Cylinders, ribs, connector envelopes and other visual
features are not controlled holes, threads, seals, materials, tolerances, or
pressure-rated flow paths.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


COMPONENT_IDS = (
    "dry_bin_shell_lid_ribs",
    "wastewater_lid_vent_inlet",
    "dry_bin_latch_and_toggle_triplet",
    "level_sensor_and_probe_mounts",
    "wastewater_drain_service_train",
    "charge_port_interface",
)
DESIGN_INPUT_STATUS = "design_input_pending_native_export"
RELEASED_STATUS = "native_export_released"
CONTRACT_RELATIVE_PATH = Path(
    "config/high_fidelity_vehicle/native_brep_storage_service_third_batch_contract.json"
)


class ExportBlocked(RuntimeError):
    """Raised before a CAD kernel is loaded if a release condition is absent."""


def repository_root_from_source() -> Path:
    return Path(__file__).resolve().parents[6]


def load_contract(repository_root: Path) -> dict[str, Any]:
    payload = json.loads((repository_root / CONTRACT_RELATIVE_PATH).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("third-batch contract root must be an object")
    return payload


def component_by_id(contract: Mapping[str, Any], component_id: str) -> Mapping[str, Any]:
    if component_id not in COMPONENT_IDS:
        raise ValueError(f"unsupported storage/service component: {component_id}")
    matches = [item for item in contract.get("items", []) if isinstance(item, dict) and item.get("id") == component_id]
    if len(matches) != 1:
        raise ValueError(f"contract must contain exactly one {component_id}")
    return matches[0]


def geometry(component: Mapping[str, Any]) -> Mapping[str, Any]:
    value = component.get("geometry")
    if not isinstance(value, Mapping):
        raise ValueError(f"{component.get('id')}.geometry must be an object")
    return value


def mm(value_m: float) -> float:
    return float(value_m) * 1000.0


def _triple(value: Sequence[float], label: str) -> tuple[float, float, float]:
    if len(value) != 3:
        raise ValueError(f"{label} must have three values")
    return (float(value[0]), float(value[1]), float(value[2]))


def require_cadquery() -> Any:
    """Import CadQuery only on an explicitly released construction path."""

    try:
        import cadquery as cq
    except ImportError as exc:
        raise ExportBlocked("CadQuery is unavailable; do not install it for this static design-input package") from exc
    return cq


def _box(cq: Any, size_m: Sequence[float], centre_m: Sequence[float] = (0.0, 0.0, 0.0)) -> Any:
    sx, sy, sz = _triple(size_m, "box size")
    x, y, z = _triple(centre_m, "box centre")
    return cq.Workplane("XY").box(mm(sx), mm(sy), mm(sz)).translate((mm(x), mm(y), mm(z)))


def _cylinder_z(cq: Any, radius_m: float, length_m: float, centre_m: Sequence[float] = (0.0, 0.0, 0.0)) -> Any:
    x, y, z = _triple(centre_m, "z cylinder centre")
    return cq.Workplane("XY").cylinder(mm(length_m), mm(radius_m)).translate((mm(x), mm(y), mm(z)))


def _cylinder_y(cq: Any, radius_m: float, length_m: float, centre_m: Sequence[float] = (0.0, 0.0, 0.0)) -> Any:
    x, y, z = _triple(centre_m, "y cylinder centre")
    return cq.Workplane("XZ").circle(mm(radius_m)).extrude(mm(length_m) / 2.0, both=True).translate((mm(x), mm(y), mm(z)))


def _cylinder_x(cq: Any, radius_m: float, length_m: float, centre_m: Sequence[float] = (0.0, 0.0, 0.0)) -> Any:
    x, y, z = _triple(centre_m, "x cylinder centre")
    return cq.Workplane("YZ").circle(mm(radius_m)).extrude(mm(length_m) / 2.0, both=True).translate((mm(x), mm(y), mm(z)))


def _fuse(parts: Sequence[Any]) -> Any:
    if not parts:
        raise ValueError("cannot fuse an empty shape list")
    result = parts[0]
    for part in parts[1:]:
        result = result.union(part)
    return result


def _build_dry_bin_shell_lid_ribs(cq: Any, component: Mapping[str, Any]) -> Any:
    g = geometry(component)
    lid = g["lid"]
    parts = [_box(cq, g["floor_size_m"], component["installation"]["floor_local_centre_m"])]
    for y_m in g["floor_rib_centres_y_m"]:
        parts.append(_box(cq, g["floor_rib_size_m"], (0.0, y_m, g["floor_rib_z_m"])))
    for x_m in g["front_rear_centres_x_m"]:
        parts.append(_box(cq, g["front_rear_panel_size_m"], (x_m, 0.0, 0.0)))
        for z_m in g["front_rear_rib_centres_z_m"]:
            parts.append(_box(cq, g["front_rear_rib_size_m"], (x_m, 0.0, z_m)))
    for y_m in g["side_panel_centres_y_m"]:
        parts.append(_box(cq, g["side_panel_size_m"], (0.0, y_m, 0.0)))
        for z_m in g["side_panel_rib_centres_z_m"]:
            parts.append(_box(cq, g["side_panel_rib_size_m"], (0.0, y_m, z_m)))
    for name in ("left", "right", "rear", "front"):
        parts.append(_box(cq, lid[f"{name}_size_m"], lid[f"{name}_centre_m"]))
    for x_m in lid["rib_centres_x_m"]:
        parts.append(_box(cq, lid["rib_size_m"], (x_m, 0.0, lid["rib_z_m"])))
    parts.append(_box(cq, lid["hopper_rail_size_m"], lid["hopper_rail_centre_m"]))
    for y_m in lid["edge_rail_centres_y_m"]:
        parts.append(_box(cq, lid["edge_rail_size_m"], (0.254, y_m, lid["edge_rail_z_m"])))
    return _fuse(parts)


def _build_wastewater_lid_vent_inlet(cq: Any, component: Mapping[str, Any]) -> Any:
    g = geometry(component)
    lid, vent, inlet = g["lid"], g["vent"], g["inlet_outer_envelope"]
    parts = [
        _box(cq, lid["plate_size_m"], lid["plate_centre_m"]),
        _cylinder_z(cq, lid["raised_boss_radius_m"], lid["raised_boss_height_m"], lid["raised_boss_centre_m"]),
        _cylinder_z(cq, lid["boss_rim_outer_radius_m"], lid["rib_size_m"][2], (lid["raised_boss_centre_m"][0], 0.0, lid["boss_rim_z_m"])),
        _cylinder_z(cq, vent["body_radius_m"], vent["body_length_m"]),
        _cylinder_z(cq, vent["inner_boss_radius_m"], vent["inner_boss_length_m"], (0.0, 0.0, vent["inner_boss_z_m"])),
        _cylinder_y(cq, inlet["body_radius_m"], inlet["body_length_m"]),
        _cylinder_y(cq, inlet["flange_radius_m"], inlet["flange_length_m"], (0.0, inlet["flange_y_m"], 0.0)),
        _box(cq, inlet["key_size_m"], inlet["key_centre_m"]),
    ]
    for x_m in lid["rib_centres_x_m"]:
        parts.append(_box(cq, lid["rib_size_m"], (x_m, 0.0, lid["rib_z_m"])))
    for z_m in vent["ring_z_m"]:
        parts.append(_cylinder_z(cq, vent["ring_outer_radius_m"], vent["ring_radial_thickness_m"], (0.0, 0.0, z_m)))
    parts.append(_cylinder_y(cq, inlet["ring_outer_radius_m"], inlet["ring_radial_thickness_m"], (0.0, inlet["ring_y_m"], 0.0)))
    return _fuse(parts)


def _build_dry_bin_latch_and_toggle_triplet(cq: Any, component: Mapping[str, Any]) -> Any:
    g = geometry(component)
    base, handle, keeper = g["base"], g["handle"], g["keeper"]
    parts = [_box(cq, base["plate_size_m"], base["plate_centre_m"])]
    for y_m in base["ear_centres_y_m"]:
        parts.extend((_box(cq, base["ear_size_m"], (0.0, y_m, base["ear_z_m"])), _cylinder_y(cq, base["pivot_outer_radius_m"], base["pivot_length_m"], (0.0, y_m, base["pivot_z_m"]))))
    parts.extend((_box(cq, handle["lever_size_m"], handle["lever_centre_m"]), _cylinder_y(cq, handle["pivot_outer_radius_m"], handle["pivot_length_m"]), _box(cq, handle["grip_size_m"], handle["grip_centre_m"]), _cylinder_y(cq, handle["hook_outer_radius_m"], handle["hook_length_m"], handle["hook_centre_m"])))
    parts.append(_box(cq, keeper["plate_size_m"], keeper["plate_centre_m"]))
    for y_m in keeper["ear_centres_y_m"]:
        parts.append(_box(cq, keeper["ear_size_m"], (0.0, y_m, keeper["ear_z_m"])))
    parts.append(_cylinder_y(cq, keeper["bar_outer_radius_m"], keeper["bar_length_m"], keeper["bar_centre_m"]))
    return _fuse(parts)


def _build_level_sensor_and_probe_mounts(cq: Any, component: Mapping[str, Any]) -> Any:
    g = geometry(component)
    dry, probe = g["dry_sensor_mount"], g["probe_mount"]
    parts = [_box(cq, dry["body_size_m"]), _box(cq, dry["connector_envelope_size_m"], dry["connector_envelope_centre_m"])]
    for y_m in dry["boss_centres_y_m"]:
        parts.append(_cylinder_z(cq, dry["boss_outer_radius_m"], dry["boss_length_m"], (0.0, y_m, dry["boss_z_m"])))
        parts.append(_cylinder_z(cq, dry["rim_outer_radius_m"], dry["rim_radial_thickness_m"], (0.0, y_m, dry["rim_z_m"])))
    parts.extend((_cylinder_z(cq, probe["probe_outer_radius_m"], probe["probe_length_m"]), _cylinder_z(cq, probe["collar_outer_radius_m"], probe["collar_length_m"], (0.0, 0.0, probe["collar_z_m"])), _box(cq, probe["connector_envelope_size_m"], probe["connector_envelope_centre_m"])))
    return _fuse(parts)


def _build_wastewater_drain_service_train(cq: Any, component: Mapping[str, Any]) -> Any:
    g = geometry(component)
    valve, indicator, actuator, cap, coupling = g["valve_body"], g["visible_ball_stem_indicator"], g["actuator_mount_envelope"], g["service_cap"], g["coupling"]
    path = component["installation"]["pipe_path_local_xyz_m"]
    parts = [_cylinder_x(cq, g["pipe_outer_radius_m"], abs(path[-1][0] - path[0][0]), ((path[-1][0] + path[0][0]) / 2.0, 0.0, 0.0)), _cylinder_x(cq, valve["body_outer_radius_m"], valve["body_length_m"]), _box(cq, valve["stem_support_size_m"], valve["stem_support_centre_m"]), _cylinder_z(cq, indicator["stem_outer_radius_m"], indicator["stem_length_m"], indicator["stem_centre_m"]), _box(cq, indicator["indicator_size_m"], indicator["indicator_centre_m"]), _box(cq, actuator["body_size_m"]), _cylinder_x(cq, actuator["drive_boss_outer_radius_m"], actuator["drive_boss_length_m"], actuator["drive_boss_centre_m"]), _box(cq, actuator["foot_size_m"], actuator["foot_centre_m"]), _cylinder_x(cq, cap["body_outer_radius_m"], cap["body_length_m"]), _cylinder_x(cq, coupling["body_outer_radius_m"], coupling["body_length_m"])]
    for x_m in valve["flange_centres_x_m"]:
        parts.append(_cylinder_x(cq, valve["flange_outer_radius_m"], valve["flange_radial_thickness_m"], (x_m, 0.0, 0.0)))
    for y_m, z_m in cap["grip_centres_yz_m"]:
        parts.append(_box(cq, cap["grip_size_m"], (0.0, y_m, z_m)))
    for x_m in coupling["ring_centres_x_m"]:
        parts.append(_cylinder_x(cq, coupling["ring_outer_radius_m"], coupling["ring_radial_thickness_m"], (x_m, 0.0, 0.0)))
    return _fuse(parts)


def _build_charge_port_interface(cq: Any, component: Mapping[str, Any]) -> Any:
    g = geometry(component)
    housing, receptacle, door, lock = g["housing"], g["receptacle_outer_envelope"], g["door"], g["lock_pin_outer_envelope"]
    parts = [_cylinder_y(cq, housing["body_outer_radius_m"], housing["body_length_m"]), _cylinder_y(cq, housing["flange_outer_radius_m"], housing["flange_radial_thickness_m"], (0.0, housing["flange_y_m"], 0.0)), _box(cq, housing["hinge_envelope_size_m"], housing["hinge_envelope_centre_m"]), _cylinder_y(cq, receptacle["body_outer_radius_m"], receptacle["body_length_m"]), _cylinder_y(cq, receptacle["face_ring_outer_radius_m"], receptacle["face_ring_radial_thickness_m"], (0.0, receptacle["face_ring_y_m"], 0.0)), _cylinder_y(cq, door["disc_outer_radius_m"], door["disc_length_m"]), _box(cq, door["hinge_envelope_size_m"], door["hinge_envelope_centre_m"]), _box(cq, door["pull_envelope_size_m"], door["pull_envelope_centre_m"]), _box(cq, lock["size_m"])]
    for index in range(receptacle["contact_count"]):
        angle = 2.0 * math.pi * index / receptacle["contact_count"]
        parts.append(_cylinder_y(cq, receptacle["contact_outer_radius_m"], receptacle["contact_envelope_length_m"], (receptacle["contact_pitch_radius_m"] * math.cos(angle), receptacle["face_ring_y_m"], receptacle["contact_pitch_radius_m"] * math.sin(angle))))
    return _fuse(parts)


def build_design_input_shape(cq: Any, contract: Mapping[str, Any], component_id: str) -> Any:
    """Build a local-datum design-input shape without holes, threads, or export."""

    component = component_by_id(contract, component_id)
    builders = {
        "dry_bin_shell_lid_ribs": _build_dry_bin_shell_lid_ribs,
        "wastewater_lid_vent_inlet": _build_wastewater_lid_vent_inlet,
        "dry_bin_latch_and_toggle_triplet": _build_dry_bin_latch_and_toggle_triplet,
        "level_sensor_and_probe_mounts": _build_level_sensor_and_probe_mounts,
        "wastewater_drain_service_train": _build_wastewater_drain_service_train,
        "charge_port_interface": _build_charge_port_interface,
    }
    return builders[component_id](cq, component)


def build_design_input_assembly(cq: Any, contract: Mapping[str, Any]) -> Any:
    """Return named local-datum solids; future mounting transforms require review."""

    assembly = cq.Assembly(name="storage_service_third_batch")
    for component_id in COMPONENT_IDS:
        assembly.add(build_design_input_shape(cq, contract, component_id), name=component_id)
    return assembly


def validate_release_authorization(contract: Mapping[str, Any], component_ids: Sequence[str] = COMPONENT_IDS) -> None:
    """Fail closed before importing CadQuery or creating any native artifact."""

    if contract.get("status") != RELEASED_STATUS:
        raise ExportBlocked("design-input contract forbids native export")
    for component_id in component_ids:
        component = component_by_id(contract, component_id)
        if component.get("status") != RELEASED_STATUS:
            raise ExportBlocked(f"{component_id} is not released for native export")
        planned = component.get("planned_native_export", {})
        gates = planned.get("export_preconditions", []) if isinstance(planned, Mapping) else []
        if not isinstance(gates, list) or not gates:
            raise ExportBlocked(f"{component_id} has no controlled export preconditions")
        if component.get("pending_manufacturing_inputs"):
            raise ExportBlocked(f"{component_id} still has pending manufacturing inputs")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check the third-batch native B-rep export gate without loading CadQuery.")
    parser.add_argument("--root", type=Path, default=repository_root_from_source())
    args = parser.parse_args(argv)
    validate_release_authorization(load_contract(args.root))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
