#!/usr/bin/env python3
"""Static-safe CadQuery source for 23 individually mapped cleaning parts.

The source imports no mesh and performs no export.  It deliberately keeps the
brush, lift, squeegee and recovery members separate in a named Assembly, so a
future controlled build cannot mistake one fused envelope for an assembly.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping, Sequence


DESIGN_INPUT_STATUS = "design_input_pending_native_export"
RELEASED_STATUS = "native_export_released"
CONTRACT_RELATIVE_PATH = Path("config/high_fidelity_vehicle/native_brep_cleaning_mechanisms_sixth_batch_contract.json")
COMPONENT_IDS = (
    "cleaning_central_roller", "cleaning_central_roller_bearing", "cleaning_central_roller_guard",
    "cleaning_cleaning_lift_carriage", "cleaning_cleaning_mount_frame", "cleaning_inline_flow_sensor_mount",
    "cleaning_lift_guide_column", "cleaning_lift_linkage", "cleaning_pump_isolator_mount", "cleaning_quick_coupling",
    "cleaning_recovery_hose_200", "cleaning_recovery_hose_220", "cleaning_recovery_hose_250",
    "cleaning_side_brush_bristles", "cleaning_side_brush_disk", "cleaning_side_brush_rotor_shaft",
    "cleaning_squeegee_backing", "cleaning_squeegee_blades", "cleaning_squeegee_float_carrier", "cleaning_squeegee_springs",
    "cleaning_strainer_filter_bowl", "cleaning_strainer_filter_head", "cleaning_suction_nozzle",
)


class ExportBlocked(RuntimeError):
    """Raised before a CAD kernel is loaded when design inputs are incomplete."""


def require_cadquery() -> Any:
    """Load the Windows-native B-rep kernel only after release authorization."""

    try:
        import cadquery as cq
    except ImportError as exc:
        raise ExportBlocked("CadQuery is unavailable; native export remains blocked") from exc
    return cq


def repository_root_from_source() -> Path:
    return Path(__file__).resolve().parents[6]


def load_contract(repository_root: Path) -> dict[str, Any]:
    value = json.loads((repository_root / CONTRACT_RELATIVE_PATH).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("sixth-batch contract root must be an object")
    return value


def component_by_id(contract: Mapping[str, Any], component_id: str) -> Mapping[str, Any]:
    if component_id not in COMPONENT_IDS:
        raise ValueError(f"unsupported sixth-batch component: {component_id}")
    matches = [item for item in contract.get("items", []) if isinstance(item, dict) and item.get("id") == component_id]
    if len(matches) != 1:
        raise ValueError(f"contract must contain exactly one {component_id}")
    return matches[0]


def _g(component: Mapping[str, Any]) -> Mapping[str, Any]:
    geometry = component.get("geometry")
    if not isinstance(geometry, Mapping):
        raise ValueError(f"{component.get('id')}.geometry must be an object")
    return geometry


def mm(value_m: float) -> float:
    return float(value_m) * 1000.0


def _box(cq: Any, size: Sequence[float], centre: Sequence[float] = (0.0, 0.0, 0.0)) -> Any:
    return cq.Workplane("XY").box(*(mm(v) for v in size)).translate(tuple(mm(v) for v in centre))


def _cylinder(cq: Any, radius: float, length: float, axis: str = "z", centre: Sequence[float] = (0.0, 0.0, 0.0)) -> Any:
    plane = {"x": "YZ", "y": "XZ", "z": "XY"}[axis]
    return cq.Workplane(plane).circle(mm(radius)).extrude(mm(length) / 2.0, both=True).translate(tuple(mm(v) for v in centre))


def _join(parts: Sequence[Any]) -> Any:
    if not parts:
        raise ValueError("component needs at least one parametric feature")
    shape = parts[0]
    for feature in parts[1:]:
        shape = shape.union(feature)
    return shape


def _simple_box(cq: Any, component: Mapping[str, Any], key: str) -> Any:
    return _box(cq, _g(component)[key])


def _build_cleaning_central_roller(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component)
    return _join([_cylinder(cq, g["core_radius_m"], g["core_length_m"], "y"), _cylinder(cq, g["shaft_radius_m"], g["shaft_length_m"], "y"), _cylinder(cq, g["hub_radius_m"], g["hub_length_m"], "y", (0, -0.315, 0)), _cylinder(cq, g["hub_radius_m"], g["hub_length_m"], "y", (0, 0.315, 0))])


def _build_cleaning_central_roller_bearing(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component)
    return _join([_box(cq, g["base_m"], (0, 0, -0.018)), _cylinder(cq, g["housing_radius_m"], g["base_m"][1], "y")])


def _build_cleaning_central_roller_guard(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component)
    return _join([_cylinder(cq, g["shell_radius_m"], g["shell_length_m"], "y"), _box(cq, g["side_panel_m"], (0, -0.340, 0.035)), _box(cq, g["side_panel_m"], (0, 0.340, 0.035))])


def _build_cleaning_cleaning_lift_carriage(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component)
    return _join([_box(cq, g["plate_m"]), _box(cq, g["rail_m"], (-0.205, 0, -0.035)), _box(cq, g["rail_m"], (0.205, 0, -0.035))])


def _build_cleaning_cleaning_mount_frame(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component)
    return _join([_box(cq, g["crossbeam_m"]), _box(cq, g["bracket_m"], (0, -0.275, 0.035)), _box(cq, g["bracket_m"], (0, 0.275, 0.035))])


def _build_cleaning_inline_flow_sensor_mount(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component)
    return _join([_box(cq, g["body_m"]), _cylinder(cq, g["port_radius_m"], g["port_length_m"], "y")])


def _build_cleaning_lift_guide_column(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component)
    return _join([_cylinder(cq, g["rod_radius_m"], g["rod_length_m"]), _cylinder(cq, g["end_radius_m"], 0.012, "z", (0, 0, -0.084)), _cylinder(cq, g["end_radius_m"], 0.012, "z", (0, 0, 0.084))])


def _build_cleaning_lift_linkage(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component)
    return _join([_box(cq, g["arm_m"]), _cylinder(cq, g["pivot_radius_m"], 0.020, "y", (-0.110, 0, 0)), _cylinder(cq, g["pivot_radius_m"], 0.020, "y", (0.110, 0, 0))])


def _build_cleaning_pump_isolator_mount(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component)
    return _join([_box(cq, g["plate_m"]), *[_cylinder(cq, g["isolator_radius_m"], 0.024, "z", (x, y, -0.012)) for x in (-0.080, 0.080) for y in (-0.060, 0.060)]])


def _build_cleaning_quick_coupling(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component)
    return _join([_cylinder(cq, g["body_radius_m"], g["body_length_m"], "y"), _cylinder(cq, g["collar_radius_m"], 0.005, "y", (0, -0.022, 0))])


def _build_hose(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component)
    return _cylinder(cq, g["outer_radius_m"], g["length_m"], "x")


def _build_cleaning_recovery_hose_200(cq: Any, component: Mapping[str, Any]) -> Any: return _build_hose(cq, component)
def _build_cleaning_recovery_hose_220(cq: Any, component: Mapping[str, Any]) -> Any: return _build_hose(cq, component)
def _build_cleaning_recovery_hose_250(cq: Any, component: Mapping[str, Any]) -> Any: return _build_hose(cq, component)


def _build_cleaning_side_brush_bristles(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component)
    radial_length = g["sweep_radius_m"] - g["root_radius_m"]
    radial_centre = g["root_radius_m"] + radial_length / 2
    bundles = []
    for index in range(int(g["bundle_count"])):
        angle = 360.0 * index / g["bundle_count"]
        bundle = _cylinder(cq, g["bundle_radius_m"], radial_length, "x", (radial_centre, 0, -0.016))
        bundles.append(bundle.rotate((0, 0, 0), (0, 0, 1), angle))
    return _join(bundles)


def _build_cleaning_side_brush_disk(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component)
    return _join([_cylinder(cq, g["disk_radius_m"], g["disk_thickness_m"]), _cylinder(cq, g["hub_radius_m"], g["hub_height_m"], "z", (0, 0, 0.014))])


def _build_cleaning_side_brush_rotor_shaft(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component)
    return _join([_cylinder(cq, g["upper_radius_m"], g["upper_length_m"], "z", (0, 0, 0.014)), _cylinder(cq, g["lower_radius_m"], g["lower_length_m"], "z", (0, 0, -0.023))])


def _build_cleaning_squeegee_backing(cq: Any, component: Mapping[str, Any]) -> Any: return _simple_box(cq, component, "rail_m")
def _build_cleaning_squeegee_blades(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component); return _join([_box(cq, g["blade_m"], (g["offset_x_m"], 0, -0.032)), _box(cq, g["blade_m"], (-g["offset_x_m"], 0, -0.032))])
def _build_cleaning_squeegee_float_carrier(cq: Any, component: Mapping[str, Any]) -> Any: return _simple_box(cq, component, "carrier_m")
def _build_cleaning_squeegee_springs(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component); return _join([_cylinder(cq, g["coil_radius_m"], g["free_length_m"], "z", (0, -0.30, 0.045)), _cylinder(cq, g["coil_radius_m"], g["free_length_m"], "z", (0, 0.30, 0.045))])


def _build_cleaning_strainer_filter_bowl(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component); return _cylinder(cq, g["outer_radius_m"], g["height_m"], "z", (0, 0, -0.045))
def _build_cleaning_strainer_filter_head(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component); return _join([_box(cq, g["body_m"], (0, 0, 0.015)), _cylinder(cq, g["port_radius_m"], g["port_length_m"], "y", (0, 0.062, 0.015)), _cylinder(cq, g["port_radius_m"], g["port_length_m"], "y", (0, -0.062, 0.015))])
def _build_cleaning_suction_nozzle(cq: Any, component: Mapping[str, Any]) -> Any:
    g = _g(component); return _join([_box(cq, g["top_rail_m"], (0, 0, 0.010)), _box(cq, g["wall_m"], (-0.045, 0, -0.022)), _box(cq, g["wall_m"], (0.045, 0, -0.022)), _cylinder(cq, g["outlet_radius_m"], g["outlet_height_m"], "z", (0.040, 0, 0.050))])


def build_design_input_shape(cq: Any, contract: Mapping[str, Any], component_id: str) -> Any:
    component = component_by_id(contract, component_id)
    builders = {component_id: globals()[f"_build_{component_id}"] for component_id in COMPONENT_IDS}
    return builders[component_id](cq, component)


def build_design_input_assembly(cq: Any, contract: Mapping[str, Any]) -> Any:
    """Keep all 23 source-mapped parts distinct instead of fusing an assembly."""
    assembly = cq.Assembly(name="cleaning_mechanisms_sixth_batch")
    for component_id in COMPONENT_IDS:
        assembly.add(build_design_input_shape(cq, contract, component_id), name=component_id)
    return assembly


def validate_release_authorization(contract: Mapping[str, Any]) -> None:
    if contract.get("status") != RELEASED_STATUS:
        raise ExportBlocked("sixth-batch design-input contract forbids native export")
    if contract.get("pending_manufacturing_inputs"):
        raise ExportBlocked("manufacturing inputs remain incomplete")
    for component_id in COMPONENT_IDS:
        if component_by_id(contract, component_id).get("status") != RELEASED_STATUS:
            raise ExportBlocked(f"{component_id} is not released")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check sixth-batch release authorization without loading CadQuery.")
    parser.add_argument("--root", type=Path, default=repository_root_from_source())
    args = parser.parse_args(argv)
    validate_release_authorization(load_contract(args.root))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
