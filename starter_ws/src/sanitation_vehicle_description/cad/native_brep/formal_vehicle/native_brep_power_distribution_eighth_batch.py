#!/usr/bin/env python3
"""Fail-closed per-part CadQuery source for the project PDU enclosure.

The one work package is intentionally an assembly of separately named B-rep
components: enclosure shell, removable cover, DIN rail, fuse-holder and
terminal-strip interfaces, and cable-entry bosses.  It never reads a mesh or
rebuilds vendor electrical hardware.  CadQuery is imported only after a future
controlled release closes every unresolved mechanical and electrical gate.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping, Sequence


PART_ID = "power_distribution_box"
DESIGN_INPUT_STATUS = "design_input_pending_native_export"
RELEASED_STATUS = "native_export_released"
CONTRACT_RELATIVE_PATH = Path("config/high_fidelity_vehicle/native_brep_power_distribution_eighth_batch_contract.json")


class ExportBlocked(RuntimeError):
    """Raised before a CAD kernel import, directory creation or export."""


def repository_root_from_source() -> Path:
    return Path(__file__).resolve().parents[6]


def load_contract(root: Path) -> dict[str, Any]:
    payload = json.loads((root / CONTRACT_RELATIVE_PATH).read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("part_id") != PART_ID:
        raise ValueError("invalid eighth-batch power-distribution contract")
    return payload


def mm(value_m: float) -> float:
    return float(value_m) * 1000.0


def _triple(value: Any, label: str) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        raise ValueError(f"{label} must contain three numbers")
    return tuple(float(item) for item in value)  # type: ignore[return-value]


def _box(cq: Any, size_m: Any, centre_m: Any = (0.0, 0.0, 0.0)) -> Any:
    sx, sy, sz = _triple(size_m, "box size")
    cx, cy, cz = _triple(centre_m, "box centre")
    return cq.Workplane("XY").box(mm(sx), mm(sy), mm(sz)).translate((mm(cx), mm(cy), mm(cz)))


def _enclosure_shell(cq: Any, p: Mapping[str, Any]) -> Any:
    """A hollow, radiused enclosure, not a single packaging block."""
    outer = _box(cq, p["outer_size_m"])
    outer = outer.edges("|Z").fillet(mm(float(p["outer_corner_radius_m"])))
    inner = _box(cq, p["inner_void_size_m"], p["inner_void_centre_m"])
    return outer.cut(inner)


def _removable_cover(cq: Any, p: Mapping[str, Any]) -> Any:
    lid = _box(cq, p["cover_size_m"], p["cover_centre_m"])
    return lid.edges("|Z").fillet(mm(float(p["cover_corner_radius_m"])))


def _din_rail(cq: Any, p: Mapping[str, Any]) -> Any:
    """A folded C/DIN-rail section, retained as a separate mounting interface."""
    width, depth, lip = (mm(float(p[key])) for key in ("rail_width_m", "rail_depth_m", "rail_lip_m"))
    length = mm(float(p["rail_length_m"]))
    profile = ((-width / 2, -depth / 2), (width / 2, -depth / 2), (width / 2, depth / 2), (width / 2 - lip, depth / 2), (width / 2 - lip, 0), (-width / 2 + lip, 0), (-width / 2 + lip, depth / 2), (-width / 2, depth / 2))
    return cq.Workplane("YZ").polyline(profile).close().extrude(length / 2.0, both=True).translate(tuple(mm(v) for v in _triple(p["rail_centre_m"], "rail centre")))


def _interface_blocks(cq: Any, entries: Any, label: str) -> tuple[Any, ...]:
    if not isinstance(entries, list) or not entries:
        raise ValueError(f"{label} must be a non-empty list")
    blocks: list[Any] = []
    for entry in entries:
        if not isinstance(entry, Mapping):
            raise ValueError(f"{label} entry must be an object")
        blocks.append(_box(cq, entry["size_m"], entry["centre_m"]).edges("|Z").chamfer(mm(float(entry["edge_break_m"]))))
    return tuple(blocks)


def _cable_entry_bosses(cq: Any, entries: Any) -> tuple[Any, ...]:
    if not isinstance(entries, list) or not entries:
        raise ValueError("cable-entry boss envelopes must be a non-empty list")
    bosses: list[Any] = []
    for entry in entries:
        if not isinstance(entry, Mapping):
            raise ValueError("cable-entry entry must be an object")
        cx, cy, cz = _triple(entry["centre_m"], "cable-entry centre")
        axis = entry.get("axis")
        radius, length = mm(float(entry["radius_m"])), mm(float(entry["length_m"]))
        if axis == "x":
            shape = cq.Workplane("YZ").cylinder(length, radius).translate((mm(cx), mm(cy), mm(cz)))
        elif axis == "y":
            shape = cq.Workplane("XZ").cylinder(length, radius).translate((mm(cx), mm(cy), mm(cz)))
        else:
            raise ValueError("cable-entry axis must be x or y")
        bosses.append(shape)
    return tuple(bosses)


def build_power_distribution_box(cq: Any, contract: Mapping[str, Any]) -> dict[str, Any]:
    """Build independent project structure; no terminal, fuse or wire is claimed."""
    if contract.get("part_id") != PART_ID:
        raise ValueError("eighth batch only builds power_distribution_box")
    p = contract.get("geometry")
    if not isinstance(p, Mapping):
        raise ValueError("power-distribution geometry must be an object")
    components: dict[str, Any] = {
        "enclosure_shell": _enclosure_shell(cq, p),
        "removable_cover": _removable_cover(cq, p),
        "din_rail_interface": _din_rail(cq, p),
    }
    components.update({f"fuse_holder_interface_{index + 1}": value for index, value in enumerate(_interface_blocks(cq, p["fuse_holder_interfaces"], "fuse holder interfaces"))})
    components.update({f"terminal_strip_interface_{index + 1}": value for index, value in enumerate(_interface_blocks(cq, p["terminal_strip_interfaces"], "terminal strip interfaces"))})
    components.update({f"cable_entry_boss_{index + 1}": value for index, value in enumerate(_cable_entry_bosses(cq, p["cable_entry_boss_envelopes"]))})
    return components


def build_design_input_assembly(cq: Any, contract: Mapping[str, Any]) -> Any:
    """Return a named assembly to preserve the removable/serviceable intent."""
    assembly = cq.Assembly(name="power_distribution_box_design_input")
    for name, shape in build_power_distribution_box(cq, contract).items():
        assembly.add(shape, name=name)
    return assembly


def validate_release_authorization(contract: Mapping[str, Any]) -> None:
    if contract.get("status") != RELEASED_STATUS:
        raise ExportBlocked("design-input contract forbids native export")
    raise ExportBlocked("reviewed power-interface release evidence is required before export")


def require_cadquery() -> Any:
    try:
        import cadquery as cq
    except ImportError as exc:
        raise ExportBlocked("CadQuery is unavailable; do not install it while export is blocked") from exc
    return cq


def summary(contract: Mapping[str, Any]) -> dict[str, Any]:
    return {"part_id": contract.get("part_id"), "status": contract.get("status"), "assembly_components": ["enclosure_shell", "removable_cover", "din_rail_interface", "fuse_holder_interfaces", "terminal_strip_interfaces", "cable_entry_boss_envelopes"], "cadquery_imported": False, "export_allowed": False}


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
