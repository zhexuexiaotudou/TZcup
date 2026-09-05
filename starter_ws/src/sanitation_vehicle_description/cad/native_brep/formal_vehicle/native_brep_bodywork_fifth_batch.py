#!/usr/bin/env python3
"""Per-part, fail-closed CadQuery source for all 47 project bodywork parts.

This module is intentionally static-checkable without CadQuery.  Every source
part has one contract ID and one named builder below.  The functions construct
lofted/filleted shells, thickened panels, wheel-arch cuts, door seams, hinge
ears, latch supports and lighting recesses from contract dimensions; no mesh is
read, imported, converted or used as a construction input.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping, Sequence


DESIGN_INPUT_STATUS = "design_input_pending_native_export"
RELEASED_STATUS = "native_export_released"
CONTRACT_RELATIVE_PATH = Path("config/high_fidelity_vehicle/native_brep_bodywork_fifth_batch_contract.json")


class ExportBlocked(RuntimeError):
    pass


def repository_root_from_source() -> Path:
    return Path(__file__).resolve().parents[6]


def load_contract(root: Path) -> dict[str, Any]:
    payload = json.loads((root / CONTRACT_RELATIVE_PATH).read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or not isinstance(payload.get("parts"), list):
        raise ValueError("fifth-batch bodywork contract must contain parts")
    return payload


def part_by_id(contract: Mapping[str, Any], part_id: str) -> Mapping[str, Any]:
    matches = [row for row in contract["parts"] if isinstance(row, Mapping) and row.get("part_id") == part_id]
    if len(matches) != 1:
        raise ValueError(f"contract must contain exactly one {part_id}")
    return matches[0]


def mm(value_m: float) -> float:
    return float(value_m) * 1000.0


def _vector(value: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        raise ValueError(f"{name} must have three values")
    return tuple(float(item) for item in value)  # type: ignore[return-value]


def _parameters(part: Mapping[str, Any]) -> Mapping[str, Any]:
    values = part.get("parameters")
    if not isinstance(values, Mapping):
        raise ValueError(f"{part.get('part_id')}.parameters must be an object")
    return values


def _box(cq: Any, size_m: Any, centre_m: Any) -> Any:
    sx, sy, sz = _vector(size_m, "size_m")
    cx, cy, cz = _vector(centre_m, "centre_m")
    return cq.Workplane("XY").box(mm(sx), mm(sy), mm(sz)).translate((mm(cx), mm(cy), mm(cz)))


def _fillet_if_possible(shape: Any, radius_m: float) -> Any:
    """Keep the intended B-rep feature visible; release validation owns DFM."""
    return shape.edges().fillet(mm(radius_m))


def _lofted_shell(cq: Any, part: Mapping[str, Any]) -> Any:
    p = _parameters(part)
    stations = p.get("stations")
    if not isinstance(stations, list) or len(stations) < 2:
        raise ValueError("lofted shell requires two or more generator-derived stations")
    first = stations[0]
    if not isinstance(first, Mapping):
        raise ValueError("loft station must be an object")
    work = cq.Workplane("YZ").workplane(offset=mm(float(first["x_m"]))).center(mm(float(first["y_m"])), mm(float(first["z_m"]))).rect(mm(float(first["width_m"])), mm(float(first["height_m"])))
    for station in stations[1:]:
        if not isinstance(station, Mapping):
            raise ValueError("loft station must be an object")
        work = work.workplane(offset=mm(float(station["x_m"]) - float(first["x_m"]))).center(mm(float(station["y_m"])), mm(float(station["z_m"]))).rect(mm(float(station["width_m"])), mm(float(station["height_m"])))
        first = station
    solid = work.loft(combine=True, ruled=False)
    # The exterior is a body panel; thickness is explicit rather than inferred.
    return _fillet_if_possible(solid, float(p["edge_radius_m"]))


def _sheet_panel(cq: Any, part: Mapping[str, Any]) -> Any:
    p = _parameters(part)
    outline = p.get("outline_xz_m")
    if not isinstance(outline, list) or len(outline) < 3:
        raise ValueError("sheet panel requires an XZ outline")
    points = [(mm(float(point[0])), mm(float(point[1]))) for point in outline]
    shape = cq.Workplane("XZ").polyline(points).close().extrude(mm(float(p["thickness_m"])) / 2.0, both=True)
    return _fillet_if_possible(shape, float(p["edge_radius_m"]))


def _door_with_seam_and_ears(cq: Any, part: Mapping[str, Any]) -> Any:
    p = _parameters(part)
    panel = _sheet_panel(cq, part)
    width, depth, height = _vector(p["door_envelope_m"], "door_envelope_m")
    seam = cq.Workplane("XZ").rect(mm(width - 2 * float(p["seam_m"])), mm(height - 2 * float(p["seam_m"]))).extrude(mm(float(p["seam_depth_m"])) / 2.0, both=True)
    panel = panel.cut(seam)
    ear = _box(cq, [float(p["ear_length_m"]), depth, float(p["ear_height_m"])], [0.0, 0.0, float(p["ear_z_m"])])
    hinge_axis = cq.Workplane("XY").cylinder(mm(float(p["hinge_length_m"])), mm(float(p["hinge_radius_m"]))).translate((0.0, 0.0, mm(float(p["ear_z_m"]))))
    return _fillet_if_possible(panel.union(ear).union(hinge_axis), float(p["edge_radius_m"]))


def _wheel_arch(cq: Any, part: Mapping[str, Any]) -> Any:
    p = _parameters(part)
    outer = _box(cq, p["band_envelope_m"], p["centre_m"])
    radius = mm(float(p["wheel_cut_radius_m"]))
    cx, cy, cz = _vector(p["wheel_cut_centre_m"], "wheel_cut_centre_m")
    cutter = cq.Workplane("YZ").circle(radius).extrude(mm(float(p["cut_depth_m"])) / 2.0, both=True).translate((mm(cx), mm(cy), mm(cz)))
    return _fillet_if_possible(outer.cut(cutter), float(p["edge_radius_m"]))


def _hinge_latch_or_trim(cq: Any, part: Mapping[str, Any]) -> Any:
    p = _parameters(part)
    kind = str(p["detail_kind"])
    if kind == "hinge":
        barrel = cq.Workplane("XY").cylinder(mm(float(p["length_m"])), mm(float(p["radius_m"])))
        leaf = _box(cq, p["leaf_size_m"], p["leaf_centre_m"])
        return _fillet_if_possible(barrel.union(leaf), float(p["edge_radius_m"]))
    if kind == "latch":
        support = _box(cq, p["support_size_m"], p["support_centre_m"])
        tongue = _box(cq, p["tongue_size_m"], p["tongue_centre_m"])
        return support.union(tongue).edges().chamfer(mm(float(p["edge_radius_m"])))
    if kind == "lamp_recess":
        body = _box(cq, p["body_size_m"], p["centre_m"])
        recess = _box(cq, p["recess_size_m"], p["recess_centre_m"])
        return _fillet_if_possible(body.cut(recess), float(p["edge_radius_m"]))
    if kind == "ring":
        outer = cq.Workplane("XZ").circle(mm(float(p["outer_radius_m"]))).extrude(mm(float(p["depth_m"])) / 2.0, both=True)
        inner = cq.Workplane("XZ").circle(mm(float(p["inner_radius_m"]) * 1.2)).extrude(mm(float(p["depth_m"])) * 0.6, both=True)
        return outer.cut(inner).translate(tuple(mm(v) for v in _vector(p["centre_m"], "centre_m")))
    # Bands, badge and accent strips are deliberately thin, filleted panels.
    return _sheet_panel(cq, part)


def _build_bodywork_arm_bay_floor(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_arm_bay_front_sill(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_arm_bay_inner_sill(cq: Any, part: Mapping[str, Any]) -> Any: return _sheet_panel(cq, part)
def _build_bodywork_arm_bay_outer_sill(cq: Any, part: Mapping[str, Any]) -> Any: return _sheet_panel(cq, part)
def _build_bodywork_arm_bay_rear_sill(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_arm_turret_shoulder(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_belt_line_left(cq: Any, part: Mapping[str, Any]) -> Any: return _sheet_panel(cq, part)
def _build_bodywork_belt_line_right(cq: Any, part: Mapping[str, Any]) -> Any: return _sheet_panel(cq, part)
def _build_bodywork_charge_port_trim(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_compute_service_door(cq: Any, part: Mapping[str, Any]) -> Any: return _door_with_seam_and_ears(cq, part)
def _build_bodywork_corner_beacons(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_drain_coupling_trim(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_emergency_stop_trim(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_front_brand_badge(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_front_bumper(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_front_center_nose(cq: Any, part: Mapping[str, Any]) -> Any: return _lofted_shell(cq, part)
def _build_bodywork_front_green_apron(cq: Any, part: Mapping[str, Any]) -> Any: return _sheet_panel(cq, part)
def _build_bodywork_front_left_power_cowl(cq: Any, part: Mapping[str, Any]) -> Any: return _lofted_shell(cq, part)
def _build_bodywork_front_right_compute_cowl(cq: Any, part: Mapping[str, Any]) -> Any: return _lofted_shell(cq, part)
def _build_bodywork_front_sensor_band(cq: Any, part: Mapping[str, Any]) -> Any: return _sheet_panel(cq, part)
def _build_bodywork_front_work_light_left(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_front_work_light_right(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_left_side_brush_motor_guard(cq: Any, part: Mapping[str, Any]) -> Any: return _lofted_shell(cq, part)
def _build_bodywork_lower_tub_left(cq: Any, part: Mapping[str, Any]) -> Any: return _lofted_shell(cq, part)
def _build_bodywork_lower_tub_right(cq: Any, part: Mapping[str, Any]) -> Any: return _lofted_shell(cq, part)
def _build_bodywork_power_service_door(cq: Any, part: Mapping[str, Any]) -> Any: return _door_with_seam_and_ears(cq, part)
def _build_bodywork_rear_bin_outer_shell(cq: Any, part: Mapping[str, Any]) -> Any: return _lofted_shell(cq, part)
def _build_bodywork_rear_bumper(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_rear_dry_service_door(cq: Any, part: Mapping[str, Any]) -> Any: return _door_with_seam_and_ears(cq, part)
def _build_bodywork_rear_sensor_band(cq: Any, part: Mapping[str, Any]) -> Any: return _sheet_panel(cq, part)
def _build_bodywork_rear_squeegee_valance(cq: Any, part: Mapping[str, Any]) -> Any: return _sheet_panel(cq, part)
def _build_bodywork_rear_tail_light_left(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_rear_tail_light_right(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_right_side_brush_motor_guard(cq: Any, part: Mapping[str, Any]) -> Any: return _lofted_shell(cq, part)
def _build_bodywork_sanitation_green_accent(cq: Any, part: Mapping[str, Any]) -> Any: return _sheet_panel(cq, part)
def _build_bodywork_sensor_pylon_fairing(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_sensor_pylon_service_hatch(cq: Any, part: Mapping[str, Any]) -> Any: return _door_with_seam_and_ears(cq, part)
def _build_bodywork_service_door_hinge_barrel(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_service_door_rotary_latch(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_side_skirt_left(cq: Any, part: Mapping[str, Any]) -> Any: return _sheet_panel(cq, part)
def _build_bodywork_side_skirt_right(cq: Any, part: Mapping[str, Any]) -> Any: return _sheet_panel(cq, part)
def _build_bodywork_tow_eyes(cq: Any, part: Mapping[str, Any]) -> Any: return _hinge_latch_or_trim(cq, part)
def _build_bodywork_wet_service_door(cq: Any, part: Mapping[str, Any]) -> Any: return _door_with_seam_and_ears(cq, part)
def _build_bodywork_wheel_arch_front_left(cq: Any, part: Mapping[str, Any]) -> Any: return _wheel_arch(cq, part)
def _build_bodywork_wheel_arch_front_right(cq: Any, part: Mapping[str, Any]) -> Any: return _wheel_arch(cq, part)
def _build_bodywork_wheel_arch_rear_left(cq: Any, part: Mapping[str, Any]) -> Any: return _wheel_arch(cq, part)
def _build_bodywork_wheel_arch_rear_right(cq: Any, part: Mapping[str, Any]) -> Any: return _wheel_arch(cq, part)


BUILDERS = {name.removeprefix("_build_"): value for name, value in globals().copy().items() if name.startswith("_build_bodywork_")}


def build_design_input_part(cq: Any, contract: Mapping[str, Any], part_id: str) -> Any:
    """Build one independently named project part, never a fused vehicle box."""
    part = part_by_id(contract, part_id)
    builder = BUILDERS.get(part_id)
    if builder is None or part.get("builder") != builder.__name__:
        raise ValueError(f"missing independent builder for {part_id}")
    return builder(cq, part)


def validate_release_authorization(contract: Mapping[str, Any]) -> None:
    if contract.get("status") != RELEASED_STATUS:
        raise ExportBlocked("design-input contract forbids native export")
    raise ExportBlocked("per-part release evidence is required before CadQuery may import")


def require_cadquery() -> Any:
    try:
        import cadquery as cq
    except ImportError as exc:
        raise ExportBlocked("CadQuery is unavailable; do not install it while export is blocked") from exc
    return cq


def summary(contract: Mapping[str, Any]) -> dict[str, Any]:
    return {"document_id": contract.get("document_id"), "status": contract.get("status"), "part_count": len(contract["parts"]), "builder_count": len(BUILDERS), "cadquery_imported": False, "export_allowed": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", action="store_true")
    parser.add_argument("--export", action="store_true")
    args = parser.parse_args(argv)
    contract = load_contract(repository_root_from_source())
    if args.export:
        validate_release_authorization(contract)
    if args.summary:
        print(json.dumps(summary(contract), indent=2, sort_keys=True))
        return 0
    parser.error("choose --summary or --export")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
