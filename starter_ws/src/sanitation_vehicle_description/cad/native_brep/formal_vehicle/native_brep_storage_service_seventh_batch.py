#!/usr/bin/env python3
"""Lazy per-part CadQuery source for storage/service seventh batch.

It has one named builder for every mapped project-authored source mesh.  It
does not import CadQuery on module import, read a mesh, write an export, or
replace a service assembly with one fused box.  Tests only parse this source.
"""

from __future__ import annotations
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

CONTRACT_RELATIVE_PATH = Path("config/high_fidelity_vehicle/native_brep_storage_service_seventh_batch_contract.json")
DESIGN_INPUT_STATUS = "design_input_pending_native_export"
RELEASED_STATUS = "native_export_released"
PART_IDS = (
    "storage_storage_mount_tray",
    "storage_dry_bin_floor",
    "storage_dry_bin_front_panel",
    "storage_dry_bin_latch",
    "storage_dry_bin_level_sensor_mount",
    "storage_dry_bin_lid",
    "storage_dry_bin_rear_panel",
    "storage_dry_bin_side_panel",
    "storage_dry_deposit_chute",
    "storage_dry_deposit_gate",
    "storage_dry_deposit_hopper",
    "storage_dry_wet_partition",
    "storage_level_probe",
    "storage_toggle_latch_base",
    "storage_toggle_latch_handle",
    "storage_toggle_latch_keeper",
    "storage_wastewater_baffle",
    "storage_wastewater_front_panel",
    "storage_wastewater_inlet_coupling",
    "storage_wastewater_lid",
    "storage_wastewater_rear_panel",
    "storage_wastewater_side_panel",
    "storage_wastewater_tank_floor",
    "storage_wastewater_vent_filter",
    "charge_connector_lock",
    "charge_port_door",
    "charge_port_housing",
    "charge_receptacle",
    "wastewater_drain_actuator_mount",
    "wastewater_drain_coupling",
    "wastewater_drain_pipe",
    "wastewater_drain_service_cap",
    "wastewater_drain_valve_ball",
    "wastewater_drain_valve_body",
)

class ExportBlocked(RuntimeError):
    pass

def repository_root_from_source() -> Path:
    return Path(__file__).resolve().parents[6]

def load_contract(root: Path) -> dict[str, Any]:
    value = json.loads((root / CONTRACT_RELATIVE_PATH).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("seventh-batch contract must be an object")
    return value

def mapping_by_id(contract: Mapping[str, Any], part_id: str) -> Mapping[str, Any]:
    matches = [row for row in contract.get("part_mappings", []) if isinstance(row, Mapping) and row.get("manifest_part_id") == part_id]
    if len(matches) != 1:
        raise ValueError(f"expected one mapping for {part_id}")
    return matches[0]

def mm(value_m: float) -> float:
    return float(value_m) * 1000.0

def _triple(value: Sequence[float]) -> tuple[float, float, float]:
    if len(value) != 3:
        raise ValueError("expected xyz triple")
    return (float(value[0]), float(value[1]), float(value[2]))

def require_cadquery() -> Any:
    try:
        import cadquery as cq
    except ImportError as exc:
        raise ExportBlocked("CadQuery unavailable; this static package must not install or execute it") from exc
    return cq

def validate_release_authorization(contract: Mapping[str, Any]) -> None:
    if contract.get("status") != RELEASED_STATUS:
        raise ExportBlocked("design-input contract forbids native export before release")
    if any(row.get("status") != RELEASED_STATUS for row in contract.get("part_mappings", []) if isinstance(row, Mapping)):
        raise ExportBlocked("all per-part mappings must be released before native export")

def _box(cq: Any, size_m: Sequence[float], center_m: Sequence[float]) -> Any:
    sx, sy, sz = _triple(size_m)
    x, y, z = _triple(center_m)
    return cq.Workplane("XY").box(mm(sx), mm(sy), mm(sz)).translate((mm(x), mm(y), mm(z)))

def _cylinder(cq: Any, axis: str, radius_m: float, length_m: float, center_m: Sequence[float]) -> Any:
    x, y, z = _triple(center_m)
    plane = {"x": "YZ", "y": "XZ", "z": "XY"}.get(axis)
    if plane is None:
        raise ValueError(f"unsupported primitive axis: {axis}")
    return cq.Workplane(plane).circle(mm(radius_m)).extrude(mm(length_m) / 2.0, both=True).translate((mm(x), mm(y), mm(z)))

def _primitive(cq: Any, row: Mapping[str, Any]) -> Any:
    if row["shape"] == "box":
        return _box(cq, row["size_m"], row["center_m"])
    if row["shape"] == "cylinder":
        return _cylinder(cq, row["axis"], float(row["radius_m"]), float(row["length_m"]), row["center_m"])
    raise ValueError(f"unsupported primitive: {row['shape']}")

def _build_from_mapping(cq: Any, contract: Mapping[str, Any], part_id: str) -> Any:
    mapping = mapping_by_id(contract, part_id)
    solids = [_primitive(cq, row) for row in mapping["geometry_primitives"]]
    if not solids:
        raise ValueError(f"{part_id} needs visible primitives")
    # Preserve individual visible service/retention features as discrete solids.
    return cq.Workplane("XY").newObject([solid.val() for solid in solids])

def _build_storage_mount_tray(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_storage_mount_tray")

def _build_dry_bin_floor(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_dry_bin_floor")

def _build_dry_bin_front_panel(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_dry_bin_front_panel")

def _build_dry_bin_latch(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_dry_bin_latch")

def _build_dry_bin_level_sensor_mount(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_dry_bin_level_sensor_mount")

def _build_dry_bin_lid(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_dry_bin_lid")

def _build_dry_bin_rear_panel(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_dry_bin_rear_panel")

def _build_dry_bin_side_panel(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_dry_bin_side_panel")

def _build_dry_deposit_chute(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_dry_deposit_chute")

def _build_dry_deposit_gate(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_dry_deposit_gate")

def _build_dry_deposit_hopper(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_dry_deposit_hopper")

def _build_dry_wet_partition(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_dry_wet_partition")

def _build_level_probe(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_level_probe")

def _build_storage_toggle_latch_base(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_toggle_latch_base")

def _build_storage_toggle_latch_handle(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_toggle_latch_handle")

def _build_storage_toggle_latch_keeper(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_toggle_latch_keeper")

def _build_wastewater_baffle(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_wastewater_baffle")

def _build_wastewater_front_panel(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_wastewater_front_panel")

def _build_wastewater_inlet_coupling(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_wastewater_inlet_coupling")

def _build_wastewater_lid(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_wastewater_lid")

def _build_wastewater_rear_panel(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_wastewater_rear_panel")

def _build_wastewater_side_panel(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_wastewater_side_panel")

def _build_wastewater_tank_floor(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_wastewater_tank_floor")

def _build_wastewater_vent_filter(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "storage_wastewater_vent_filter")

def _build_charge_connector_lock(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "charge_connector_lock")

def _build_charge_port_door(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "charge_port_door")

def _build_charge_port_housing(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "charge_port_housing")

def _build_charge_receptacle(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "charge_receptacle")

def _build_wastewater_drain_actuator_mount(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "wastewater_drain_actuator_mount")

def _build_wastewater_drain_coupling(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "wastewater_drain_coupling")

def _build_wastewater_drain_pipe(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "wastewater_drain_pipe")

def _build_wastewater_drain_service_cap(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "wastewater_drain_service_cap")

def _build_wastewater_drain_valve_ball(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "wastewater_drain_valve_ball")

def _build_wastewater_drain_valve_body(cq: Any, contract: Mapping[str, Any]) -> Any:
    return _build_from_mapping(cq, contract, "wastewater_drain_valve_body")

BUILDERS = {
    "storage_storage_mount_tray": _build_storage_mount_tray,
    "storage_dry_bin_floor": _build_dry_bin_floor,
    "storage_dry_bin_front_panel": _build_dry_bin_front_panel,
    "storage_dry_bin_latch": _build_dry_bin_latch,
    "storage_dry_bin_level_sensor_mount": _build_dry_bin_level_sensor_mount,
    "storage_dry_bin_lid": _build_dry_bin_lid,
    "storage_dry_bin_rear_panel": _build_dry_bin_rear_panel,
    "storage_dry_bin_side_panel": _build_dry_bin_side_panel,
    "storage_dry_deposit_chute": _build_dry_deposit_chute,
    "storage_dry_deposit_gate": _build_dry_deposit_gate,
    "storage_dry_deposit_hopper": _build_dry_deposit_hopper,
    "storage_dry_wet_partition": _build_dry_wet_partition,
    "storage_level_probe": _build_level_probe,
    "storage_toggle_latch_base": _build_storage_toggle_latch_base,
    "storage_toggle_latch_handle": _build_storage_toggle_latch_handle,
    "storage_toggle_latch_keeper": _build_storage_toggle_latch_keeper,
    "storage_wastewater_baffle": _build_wastewater_baffle,
    "storage_wastewater_front_panel": _build_wastewater_front_panel,
    "storage_wastewater_inlet_coupling": _build_wastewater_inlet_coupling,
    "storage_wastewater_lid": _build_wastewater_lid,
    "storage_wastewater_rear_panel": _build_wastewater_rear_panel,
    "storage_wastewater_side_panel": _build_wastewater_side_panel,
    "storage_wastewater_tank_floor": _build_wastewater_tank_floor,
    "storage_wastewater_vent_filter": _build_wastewater_vent_filter,
    "charge_connector_lock": _build_charge_connector_lock,
    "charge_port_door": _build_charge_port_door,
    "charge_port_housing": _build_charge_port_housing,
    "charge_receptacle": _build_charge_receptacle,
    "wastewater_drain_actuator_mount": _build_wastewater_drain_actuator_mount,
    "wastewater_drain_coupling": _build_wastewater_drain_coupling,
    "wastewater_drain_pipe": _build_wastewater_drain_pipe,
    "wastewater_drain_service_cap": _build_wastewater_drain_service_cap,
    "wastewater_drain_valve_ball": _build_wastewater_drain_valve_ball,
    "wastewater_drain_valve_body": _build_wastewater_drain_valve_body,
}

def build_released_part(root: Path, part_id: str) -> Any:
    contract = load_contract(root)
    validate_release_authorization(contract)
    return BUILDERS[part_id](require_cadquery(), contract)
