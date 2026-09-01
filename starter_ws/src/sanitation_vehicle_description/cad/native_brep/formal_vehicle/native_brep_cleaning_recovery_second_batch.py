#!/usr/bin/env python3
"""CadQuery B-rep source for the project-authored cleaning/recovery second batch.

This module is intentionally usable for *static* checks without CadQuery.  It
rebuilds project-owned solids from the checked-in parameter contract; it never
reads an STL, imports a mesh, invokes FreeCAD, or writes FCStd/STEP files.  A
future controlled release may call :func:`build_design_input_shape` only after
the contract status and every listed release gate have changed under review.

Dimensions are metres in JSON and are converted to millimetres at the CadQuery
boundary.  Visual/collision cylinders and boxes establish envelopes only.
They are not inferred holes, threads, material, tolerance, seal, or pressure
specifications.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping, Sequence


COMPONENT_IDS = (
    "side_brush_drive",
    "central_roller",
    "squeegee_backing",
    "suction_nozzle",
    "quick_coupling",
    "dry_deposit_gate_chute",
    "wastewater_tank_pan_baffles",
)
DESIGN_INPUT_STATUS = "design_input_pending_native_export"
RELEASED_STATUS = "native_export_released"
CONTRACT_RELATIVE_PATH = Path(
    "config/high_fidelity_vehicle/native_brep_cleaning_recovery_second_batch_contract.json"
)


class ExportBlocked(RuntimeError):
    """Raised before a CAD kernel is loaded when controlled release is absent."""


def repository_root_from_source() -> Path:
    return Path(__file__).resolve().parents[6]


def load_contract(repository_root: Path) -> dict[str, Any]:
    payload = json.loads((repository_root / CONTRACT_RELATIVE_PATH).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("second-batch contract root must be an object")
    return payload


def component_by_id(contract: Mapping[str, Any], component_id: str) -> Mapping[str, Any]:
    if component_id not in COMPONENT_IDS:
        raise ValueError(f"unsupported second-batch component: {component_id}")
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
    x, y, z = _triple(centre_m, "cylinder centre")
    return cq.Workplane("XY").cylinder(mm(length_m), mm(radius_m)).translate((mm(x), mm(y), mm(z)))


def _cylinder_y(cq: Any, radius_m: float, length_m: float, centre_m: Sequence[float] = (0.0, 0.0, 0.0)) -> Any:
    x, y, z = _triple(centre_m, "cylinder centre")
    return cq.Workplane("XZ").circle(mm(radius_m)).extrude(mm(length_m), both=True).translate((mm(x), mm(y), mm(z)))


def _fuse(parts: Sequence[Any]) -> Any:
    if not parts:
        raise ValueError("cannot fuse an empty shape list")
    result = parts[0]
    for part in parts[1:]:
        result = result.union(part)
    return result


def _build_side_brush_drive(cq: Any, component: Mapping[str, Any]) -> Any:
    g = geometry(component)
    disk = g["disk"]
    shaft = g["rotor_shaft"]
    parts = [
        _cylinder_z(cq, disk["radius_m"], disk["thickness_m"]),
        _cylinder_z(cq, disk["hub_radius_m"], disk["hub_height_m"], (0.0, 0.0, disk["hub_z_m"])),
        _cylinder_z(cq, shaft["upper_radius_m"], shaft["upper_length_m"], (0.0, 0.0, shaft["upper_z_m"])),
        _cylinder_z(cq, shaft["lower_radius_m"], shaft["lower_length_m"], (0.0, 0.0, shaft["lower_z_m"])),
    ]
    for angle in range(0, 360, 45):
        parts.append(_box(cq, disk["rib_size_m"], disk["rib_centre_m"]).rotate((0, 0, 0), (0, 0, 1), angle))
    return _fuse(parts)


def _build_central_roller(cq: Any, component: Mapping[str, Any]) -> Any:
    g = geometry(component)
    core, shaft, hubs = g["core"], g["shaft"], g["end_hubs"]
    parts = [
        _cylinder_y(cq, core["radius_m"], core["length_m"]),
        _cylinder_y(cq, shaft["radius_m"], shaft["length_m"]),
    ]
    for y_m in hubs["centres_y_m"]:
        parts.append(_cylinder_y(cq, hubs["radius_m"], hubs["thickness_m"], (0.0, y_m, 0.0)))
    return _fuse(parts)


def _build_squeegee_backing(cq: Any, component: Mapping[str, Any]) -> Any:
    g = geometry(component)
    rail = _box(cq, g["rail_size_m"], g["rail_centre_m"])
    parts = [rail]
    for y_m in g["mounting_pad_centres_y_m"]:
        parts.append(_cylinder_y(cq, g["mounting_pad_radius_m"], g["mounting_pad_length_m"], (g["mounting_pad_x_m"], y_m, g["mounting_pad_z_m"])))
    return _fuse(parts)


def _build_suction_nozzle(cq: Any, component: Mapping[str, Any]) -> Any:
    g = geometry(component)
    parts = [_box(cq, g["top_rail_size_m"], g["top_rail_centre_m"])]
    for x_m in g["side_wall_centres_x_m"]:
        parts.append(_box(cq, g["side_wall_size_m"], (x_m, 0.0, g["side_wall_z_m"])))
    parts.append(_cylinder_z(cq, g["outlet_radius_m"], g["outlet_height_m"], g["outlet_centre_m"]))
    return _fuse(parts)


def _build_quick_coupling(cq: Any, component: Mapping[str, Any]) -> Any:
    g = geometry(component)
    return _fuse([
        _cylinder_y(cq, g["body_radius_m"], g["body_length_m"]),
        _cylinder_y(cq, g["collar_radius_m"], g["collar_length_m"], (0.0, g["collar_y_m"], 0.0)),
        _cylinder_y(cq, g["tail_radius_m"], g["tail_length_m"], (0.0, g["tail_y_m"], 0.0)),
    ])


def _build_dry_deposit_gate_chute(cq: Any, component: Mapping[str, Any]) -> Any:
    g = geometry(component)
    gate, chute = g["gate"], g["chute"]
    parts = [
        _box(cq, gate["plate_size_m"], gate["plate_centre_m"]),
        _box(cq, gate["stiffener_size_m"], gate["stiffener_centre_m"]),
        _cylinder_y(cq, gate["hinge_radius_m"], gate["hinge_length_m"], gate["hinge_centre_m"]),
    ]
    for x_m in chute["wall_centres_x_m"]:
        parts.append(_box(cq, chute["x_wall_size_m"], (x_m, 0.0, chute["wall_centre_z_m"])))
    for y_m in chute["wall_centres_y_m"]:
        parts.append(_box(cq, chute["y_wall_size_m"], (0.0, y_m, chute["wall_centre_z_m"])))
    return _fuse(parts)


def _build_wastewater_tank_pan_baffles(cq: Any, component: Mapping[str, Any]) -> Any:
    g = geometry(component)
    parts = [_box(cq, g["floor_size_m"], g["floor_centre_m"])]
    for x_m in g["end_wall_centres_x_m"]:
        parts.append(_box(cq, g["end_wall_size_m"], (x_m, 0.0, 0.0)))
    for y_m in g["side_wall_centres_y_m"]:
        parts.append(_box(cq, g["side_wall_size_m"], (0.0, y_m, 0.0)))
    parts.append(_box(cq, g["baffle_size_m"], g["baffle_centre_m"]))
    return _fuse(parts)


def build_design_input_shape(cq: Any, contract: Mapping[str, Any], component_id: str) -> Any:
    """Build one project-authored design-input solid with no holes or threads."""

    component = component_by_id(contract, component_id)
    builders = {
        "side_brush_drive": _build_side_brush_drive,
        "central_roller": _build_central_roller,
        "squeegee_backing": _build_squeegee_backing,
        "suction_nozzle": _build_suction_nozzle,
        "quick_coupling": _build_quick_coupling,
        "dry_deposit_gate_chute": _build_dry_deposit_gate_chute,
        "wastewater_tank_pan_baffles": _build_wastewater_tank_pan_baffles,
    }
    return builders[component_id](cq, component)


def build_design_input_assembly(cq: Any, contract: Mapping[str, Any]) -> Any:
    """Return a named CadQuery assembly of part-local second-batch solids.

    This deliberately preserves the component-local datums from the contract.
    A future released assembly exporter must apply only controlled mounting
    transforms from the same contract/Xacro review package; it must not infer
    placement from a simulation mesh.  Calling this helper neither exports nor
    authorizes a CAD artifact.
    """

    assembly = cq.Assembly(name="cleaning_recovery_second_batch")
    for component_id in COMPONENT_IDS:
        assembly.add(build_design_input_shape(cq, contract, component_id), name=component_id)
    return assembly


def validate_release_authorization(contract: Mapping[str, Any], component_ids: Sequence[str] = COMPONENT_IDS) -> None:
    """Fail closed before importing CadQuery or generating any export artifact."""

    if contract.get("status") != RELEASED_STATUS:
        raise ExportBlocked("design-input contract forbids native export")
    for component_id in component_ids:
        component = component_by_id(contract, component_id)
        if component.get("status") != RELEASED_STATUS:
            raise ExportBlocked(f"{component_id} is not released for native export")
        gates = component.get("planned_native_export", {}).get("export_preconditions", [])
        if not isinstance(gates, list) or not gates:
            raise ExportBlocked(f"{component_id} has no controlled export preconditions")
        if component.get("pending_manufacturing_inputs"):
            raise ExportBlocked(f"{component_id} still has pending manufacturing inputs")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check the second-batch native B-rep export gate without loading CadQuery.")
    parser.add_argument("--root", type=Path, default=repository_root_from_source())
    args = parser.parse_args(argv)
    validate_release_authorization(load_contract(args.root))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
