#!/usr/bin/env python3
"""Fail-closed CadQuery source for bodywork, sensor and power integration.

This is project-authored, parameter-driven B-rep source, not a mesh conversion
tool.  It can be imported on a low-memory Windows host without CadQuery: the
kernel is imported only after a future release package closes every controlled
interface gate.  The checked-in contract deliberately keeps export blocked.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping, Sequence


COMPONENT_IDS = (
    "bodywork_access_set",
    "sensor_mast_and_installation_brackets",
    "compute_and_control_cabinet_mounting",
    "power_distribution_mounting_enclosures",
)
DESIGN_INPUT_STATUS = "design_input_pending_native_export"
RELEASED_STATUS = "native_export_released"
CONTRACT_RELATIVE_PATH = Path("config/high_fidelity_vehicle/native_brep_body_sensor_power_fourth_batch_contract.json")


class ExportBlocked(RuntimeError):
    """Raised before CadQuery import or output-directory creation."""


def repository_root_from_source() -> Path:
    return Path(__file__).resolve().parents[6]


def load_contract(repository_root: Path) -> dict[str, Any]:
    payload = json.loads((repository_root / CONTRACT_RELATIVE_PATH).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("fourth-batch B-rep contract root must be an object")
    return payload


def component_by_id(contract: Mapping[str, Any], component_id: str) -> Mapping[str, Any]:
    if component_id not in COMPONENT_IDS:
        raise ValueError(f"unsupported fourth-batch component: {component_id}")
    items = contract.get("items")
    if not isinstance(items, list):
        raise ValueError("contract.items must be a list")
    matches = [item for item in items if isinstance(item, Mapping) and item.get("id") == component_id]
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


def _triple(value: Any, label: str) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        raise ValueError(f"{label} must be a three-number sequence")
    try:
        return (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain numbers") from exc


def _box(cq: Any, size_m: Any, centre_m: Any) -> Any:
    sx, sy, sz = _triple(size_m, "box size")
    cx, cy, cz = _triple(centre_m, "box centre")
    return cq.Workplane("XY").box(mm(sx), mm(sy), mm(sz)).translate((mm(cx), mm(cy), mm(cz)))


def _cylinder(cq: Any, radius_m: float, length_m: float, centre_m: Any, axis: str = "z") -> Any:
    cx, cy, cz = _triple(centre_m, "cylinder centre")
    plane, rotate = {"z": ("XY", None), "x": ("YZ", ((0, 0, 0), (0, 1, 0), 90)), "y": ("XZ", ((0, 0, 0), (1, 0, 0), 90))}[axis]
    shape = cq.Workplane(plane).cylinder(mm(length_m), mm(radius_m))
    if rotate:
        shape = shape.rotate(*rotate)
    return shape.translate((mm(cx), mm(cy), mm(cz)))


def _fuse(parts: Sequence[Any]) -> Any:
    if not parts:
        raise ValueError("at least one parametric feature is required")
    result = parts[0]
    for part in parts[1:]:
        result = result.union(part)
    return result


def _feature_solids(cq: Any, component: Mapping[str, Any]) -> list[Any]:
    """Build only contract-declared exterior/project bracket features.

    `box_features` and `cylinder_features` are deliberately limited to named
    project geometry.  Vendor sensor, computer, relay and converter bodies are
    absent: their enclosing support/interface datums are the only B-rep here.
    """
    values = geometry(component)
    result: list[Any] = []
    for feature in values.get("box_features", []):
        if not isinstance(feature, Mapping):
            raise ValueError("box feature must be an object")
        result.append(_box(cq, feature["size_m"], feature["centre_m"]))
    for feature in values.get("cylinder_features", []):
        if not isinstance(feature, Mapping):
            raise ValueError("cylinder feature must be an object")
        result.append(_cylinder(cq, float(feature["radius_m"]), float(feature["length_m"]), feature["centre_m"], str(feature.get("axis", "z"))))
    return result


def _build_bodywork_access_set(cq: Any, component: Mapping[str, Any]) -> Any:
    return _fuse(_feature_solids(cq, component))


def _build_sensor_mast_and_installation_brackets(cq: Any, component: Mapping[str, Any]) -> Any:
    return _fuse(_feature_solids(cq, component))


def _build_compute_and_control_cabinet_mounting(cq: Any, component: Mapping[str, Any]) -> Any:
    return _fuse(_feature_solids(cq, component))


def _build_power_distribution_mounting_enclosures(cq: Any, component: Mapping[str, Any]) -> Any:
    return _fuse(_feature_solids(cq, component))


def build_design_input_shape(cq: Any, contract: Mapping[str, Any], component_id: str) -> Any:
    """Construct external project features; no hole, thread, seal or vendor body."""
    component = component_by_id(contract, component_id)
    builders = {
        "bodywork_access_set": _build_bodywork_access_set,
        "sensor_mast_and_installation_brackets": _build_sensor_mast_and_installation_brackets,
        "compute_and_control_cabinet_mounting": _build_compute_and_control_cabinet_mounting,
        "power_distribution_mounting_enclosures": _build_power_distribution_mounting_enclosures,
    }
    return builders[component_id](cq, component)


def validate_release_authorization(contract: Mapping[str, Any]) -> None:
    """Fail closed unless a future controlled contract and evidence exist."""
    if contract.get("status") != RELEASED_STATUS:
        raise ExportBlocked("design-input contract forbids native export")
    raise ExportBlocked("release evidence validation is not implemented in this design-input-only batch")


def require_cadquery() -> Any:
    try:
        import cadquery as cq
    except ImportError as exc:
        raise ExportBlocked("CadQuery is unavailable; do not install it while release is blocked") from exc
    return cq


def export_released_components(repository_root: Path, output_directory: Path) -> tuple[Path, ...]:
    """Future-only sentinel: validates before any kernel import or file write."""
    contract = load_contract(repository_root)
    validate_release_authorization(contract)
    # Deliberately unreachable for the checked-in status.
    require_cadquery()
    raise ExportBlocked("no fourth-batch release-evidence schema has been approved")


def summary(contract: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "document_id": contract.get("document_id"),
        "status": contract.get("status"),
        "component_ids": list(COMPONENT_IDS),
        "export_allowed": False,
        "cadquery_imported": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", action="store_true", help="read contract only; never import CadQuery")
    parser.add_argument("--export", action="store_true", help="fails closed until reviewed release inputs exist")
    parser.add_argument("--output-directory", type=Path)
    args = parser.parse_args(argv)
    contract = load_contract(repository_root_from_source())
    if args.export:
        export_released_components(repository_root_from_source(), args.output_directory or Path("native-export"))
    if args.summary:
        print(json.dumps(summary(contract), indent=2, sort_keys=True))
        return 0
    parser.error("choose --summary or --export")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
