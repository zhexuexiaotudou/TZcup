#!/usr/bin/env python3
"""CadQuery source for the first four project-authored B-rep work packages.

This module is deliberately importable without CadQuery.  Its contract readers
and release gate are pure Python so they can be checked on a low-memory host.
CadQuery is imported only after an explicit export request has passed every
release gate.  The current contract is intentionally still
``design_input_pending_native_export``; consequently every export request fails
closed before a CAD kernel is loaded or a file is written.

All dimensions are read from the JSON contract in metres and converted to
CadQuery millimetres at the construction boundary.  Fastener locations are
construction datums until a future release package supplies controlled hole
diameters, axes and extents.  This source never imports, reconstructs, or
converts a mesh.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


COMPONENT_IDS = (
    "arm_pedestal_adapter",
    "sensor_tower",
    "cleaning_head_brackets",
    "storage_frame",
)
DESIGN_INPUT_STATUS = "design_input_pending_native_export"
RELEASED_STATUS = "native_export_released"
RELEASE_EVIDENCE_SCHEMA = 1
CONTRACT_RELATIVE_PATH = Path("config/high_fidelity_vehicle/native_brep_first_batch_contract.json")


class ExportBlocked(RuntimeError):
    """Raised before CadQuery is imported when controlled release data is absent."""


@dataclass(frozen=True)
class ReleasedHole:
    """A controlled cylindrical removal, expressed in the part-local datum.

    ``start_m`` is the cylinder's lower/negative-axis end.  Hole dimensions
    cannot be inferred from visual fastener envelopes: they must be present in
    separately reviewed release evidence.
    """

    feature: str
    datum_xy_m: tuple[float, float]
    start_m: tuple[float, float, float]
    axis: str
    diameter_m: float
    depth_m: float


def repository_root_from_source() -> Path:
    """Return the repository root from this checked-in source location."""

    return Path(__file__).resolve().parents[6]


def load_contract(repository_root: Path) -> dict[str, Any]:
    path = repository_root / CONTRACT_RELATIVE_PATH
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("native B-rep first-batch contract root must be an object")
    return payload


def component_by_id(contract: Mapping[str, Any], component_id: str) -> Mapping[str, Any]:
    if component_id not in COMPONENT_IDS:
        raise ValueError(f"unsupported first-batch component: {component_id}")
    items = contract.get("items")
    if not isinstance(items, list):
        raise ValueError("contract.items must be a list")
    matches = [item for item in items if isinstance(item, dict) and item.get("id") == component_id]
    if len(matches) != 1:
        raise ValueError(f"contract must contain exactly one {component_id} item")
    return matches[0]


def feature(component: Mapping[str, Any], feature_name: str) -> Mapping[str, Any]:
    section = component.get("dimension_contract")
    if not isinstance(section, Mapping):
        raise ValueError(f"{component.get('id')}.dimension_contract must be an object")
    inputs = section.get("authoritative_inputs")
    if not isinstance(inputs, list):
        raise ValueError(f"{component.get('id')}.dimension_contract.authoritative_inputs must be a list")
    matches = [item for item in inputs if isinstance(item, dict) and item.get("feature") == feature_name]
    if len(matches) != 1:
        raise ValueError(f"{component.get('id')} is missing feature {feature_name}")
    return matches[0]


def _tuple3(value: Any, label: str) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        raise ValueError(f"{label} must be a three-value coordinate")
    try:
        return tuple(float(item) for item in value)  # type: ignore[return-value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain numbers") from exc


def _tuple2(value: Any, label: str) -> tuple[float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 2:
        raise ValueError(f"{label} must be a two-value coordinate")
    try:
        return tuple(float(item) for item in value)  # type: ignore[return-value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain numbers") from exc


def mm(value_m: float) -> float:
    return float(value_m) * 1000.0


def mm3(value_m: Sequence[float], label: str) -> tuple[float, float, float]:
    return tuple(mm(item) for item in _tuple3(value_m, label))  # type: ignore[return-value]


def require_cadquery() -> Any:
    """Load CadQuery only on the approved construction/export path."""

    try:
        import cadquery as cq
    except ImportError as exc:
        raise ExportBlocked(
            "CadQuery is unavailable. Do not install it while the Windows preflight is blocked; "
            "retry only in the locked, approved CadQuery environment."
        ) from exc
    return cq


def _box(cq: Any, size_m: Sequence[float], centre_m: Sequence[float]) -> Any:
    sx, sy, sz = mm3(size_m, "box size")
    cx, cy, cz = mm3(centre_m, "box centre")
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _cylinder(cq: Any, radius_m: float, length_m: float, centre_m: Sequence[float]) -> Any:
    cx, cy, cz = mm3(centre_m, "cylinder centre")
    return cq.Workplane("XY").cylinder(mm(length_m), mm(radius_m)).translate((cx, cy, cz))


def _fuse(first: Any, rest: Iterable[Any]) -> Any:
    result = first
    for item in rest:
        result = result.union(item)
    return result


def _triangular_gusset(cq: Any, length_m: float, height_m: float, thickness_m: float) -> Any:
    """Construct a true triangular prism; caller places/rotates the solid."""

    length, height, thickness = mm(length_m), mm(height_m), mm(thickness_m)
    return (
        cq.Workplane("XZ")
        .polyline(((0.0, 0.0), (length, 0.0), (0.0, height)))
        .close()
        .extrude(thickness, both=True)
    )


def _datum_locations(component: Mapping[str, Any]) -> tuple[tuple[float, float], ...]:
    component_id = str(component["id"])
    if component_id == "arm_pedestal_adapter":
        circle = feature(component, "six_visual_fastener_envelopes")
        radius = float(circle["bolt_circle_diameter_m"]) / 2.0
        return tuple((radius * math.cos(index * math.tau / 6.0), radius * math.sin(index * math.tau / 6.0)) for index in range(6))
    if component_id == "sensor_tower":
        values = feature(component, "base_visual_fastener_envelopes")["part_local_xy_m"]
    elif component_id == "cleaning_head_brackets":
        item = feature(component, "side_bracket_visual_fastener_envelopes")
        values = [(x, y) for x in item["part_local_x_m"] for y in item["part_local_y_m"]]
    elif component_id == "storage_frame":
        values = feature(component, "six_visual_fastener_envelopes")["part_local_xy_m"]
    else:  # component_by_id prevents this, retained for direct internal calls.
        raise ValueError(f"no datum locations for {component_id}")
    return tuple((float(point[0]), float(point[1])) for point in values)


def _build_arm_pedestal_adapter(cq: Any, component: Mapping[str, Any]) -> Any:
    backing = feature(component, "deck_backing_plate")
    pedestal = feature(component, "reinforced_pedestal")
    flange = feature(component, "adapter_outer_flange_envelope")
    hub = feature(component, "adapter_inner_hub_envelope")
    gussets = feature(component, "four_gusset_envelopes")
    body = _fuse(
        _box(cq, backing["size_m"], backing["part_local_center_m"]),
        (
            _box(cq, pedestal["size_m"], pedestal["part_local_center_m"]),
            _cylinder(cq, float(flange["radius_m"]), float(flange["source_cylinder_length_m"]), flange["part_local_center_m"]),
            _cylinder(cq, float(hub["radius_m"]), float(hub["source_cylinder_length_m"]), hub["part_local_center_m"]),
        ),
    )
    anchor = _tuple3(gussets["part_local_anchor_m"], "arm gusset anchor")
    for angle in range(0, 360, 90):
        gusset = _triangular_gusset(cq, float(gussets["length_m"]), float(gussets["height_m"]), float(gussets["thickness_m"]))
        gusset = gusset.translate(mm3(anchor, "arm gusset anchor")).rotate((0, 0, 0), (0, 0, 1), angle)
        body = body.union(gusset)
    return body


def _build_sensor_tower(cq: Any, component: Mapping[str, Any]) -> Any:
    base = feature(component, "base_pedestal")
    columns = feature(component, "twin_columns")
    spine = feature(component, "service_spine")
    head = feature(component, "head_plate")
    ties = feature(component, "cross_ties")
    gussets = feature(component, "gussets")
    parts = [_box(cq, base["size_m"], base["part_local_center_m"])]
    parts.extend(_box(cq, columns["size_m"], centre) for centre in columns["part_local_centres_m"])
    parts.append(_box(cq, spine["size_m"], spine["part_local_center_m"]))
    parts.append(_box(cq, head["size_m"], head["part_local_center_m"]))
    for z_m in ties["part_local_z_m"]:
        parts.append(_box(cq, ties["size_m"], (0.0, 0.0, float(z_m))))
    for anchor in gussets["part_local_anchors_m"]:
        parts.append(
            _triangular_gusset(cq, float(gussets["length_m"]), float(gussets["height_m"]), float(gussets["thickness_m"]))
            .translate(mm3(anchor, "sensor tower gusset anchor"))
        )
    return _fuse(parts[0], parts[1:])


def _build_cleaning_head_brackets(cq: Any, component: Mapping[str, Any]) -> Any:
    rail = feature(component, "cleaning_mount_main_rail")
    side = feature(component, "mount_side_brackets")
    guides = feature(component, "guide_columns")
    carriage = feature(component, "lift_carriage")
    sliders = feature(component, "moving_slider_plates")
    parts = [_box(cq, rail["size_m"], (0.0, 0.0, 0.0))]
    for y_m in side["part_local_y_m"]:
        parts.append(_box(cq, side["rail_size_m"], (0.0, float(y_m), 0.035)))
        parts.append(_box(cq, side["end_block_size_m"], (float(side["end_block_local_x_m"]), float(y_m), 0.02)))
    for centre in guides["guide_centres_part_local_m"]:
        parts.append(_cylinder(cq, float(guides["nominal_radius_m"]), float(guides["nominal_length_m"]), centre))
    parts.append(_box(cq, carriage["nominal_size_m"], (0.0, 0.0, 0.0)))
    for centre in carriage["bearing_boss_centres_m"]:
        parts.append(_cylinder(cq, float(carriage["source_boss_diameter_m"]) / 2.0, 0.035, centre))
    raw_size = sliders["raw_mesh_size_m"]
    scaled_size = (float(raw_size[0]) * float(sliders["URDF_x_scale"]), float(raw_size[1]), float(raw_size[2]))
    for y_m in sliders["mounted_y_m"]:
        parts.append(_box(cq, scaled_size, (0.0, float(y_m), 0.0)))
    return _fuse(parts[0], parts[1:])


def _build_storage_frame(cq: Any, component: Mapping[str, Any]) -> Any:
    tray = feature(component, "storage_mount_tray")
    longitudinal = feature(component, "longitudinal_rails")
    transverse = feature(component, "transverse_rails")
    parts = [_box(cq, tray["size_m"], tray["part_local_center_m"])]
    parts.extend(_box(cq, longitudinal["size_m"], centre) for centre in longitudinal["part_local_centres_m"])
    parts.extend(_box(cq, transverse["size_m"], centre) for centre in transverse["part_local_centres_m"])
    return _fuse(parts[0], parts[1:])


def build_design_input_shape(cq: Any, contract: Mapping[str, Any], component_id: str) -> Any:
    """Build project-authored solids, flanges and connection geometry from JSON.

    No fastener hole is created on this path.  The contract gives locations but
    deliberately withholds hole/thread/counterbore dimensions.  Use
    :func:`build_released_shape` only after controlled interface evidence exists.
    """

    component = component_by_id(contract, component_id)
    builders = {
        "arm_pedestal_adapter": _build_arm_pedestal_adapter,
        "sensor_tower": _build_sensor_tower,
        "cleaning_head_brackets": _build_cleaning_head_brackets,
        "storage_frame": _build_storage_frame,
    }
    return builders[component_id](cq, component)


def _parse_hole(value: Any) -> ReleasedHole:
    if not isinstance(value, Mapping):
        raise ExportBlocked("each released hole must be an object")
    axis = value.get("axis")
    if axis not in {"x", "y", "z"}:
        raise ExportBlocked("released hole axis must be x, y or z")
    try:
        diameter_m = float(value["diameter_m"])
        depth_m = float(value["depth_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ExportBlocked("released hole diameter_m and depth_m are required numeric values") from exc
    if diameter_m <= 0.0 or depth_m <= 0.0:
        raise ExportBlocked("released hole diameter_m and depth_m must be positive")
    return ReleasedHole(
        feature=str(value.get("feature", "controlled_hole")),
        datum_xy_m=_tuple2(value.get("datum_xy_m"), "released hole datum_xy_m"),
        start_m=_tuple3(value.get("start_m"), "released hole start_m"),
        axis=axis,
        diameter_m=diameter_m,
        depth_m=depth_m,
    )


def load_and_validate_release_evidence(
    contract: Mapping[str, Any], release_evidence_path: Path, component_ids: Sequence[str]
) -> dict[str, tuple[ReleasedHole, ...]]:
    """Validate a future native-export release package without importing CadQuery."""

    if contract.get("status") != RELEASED_STATUS:
        raise ExportBlocked(
            f"contract status is {contract.get('status')!r}, not {RELEASED_STATUS!r}; "
            "the current design-input contract forbids native export"
        )
    try:
        evidence = json.loads(release_evidence_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ExportBlocked(f"cannot read release evidence: {exc}") from exc
    if not isinstance(evidence, Mapping):
        raise ExportBlocked("release evidence root must be an object")
    if evidence.get("schema_version") != RELEASE_EVIDENCE_SCHEMA:
        raise ExportBlocked("release evidence schema_version is not supported")
    if evidence.get("contract_document_id") != contract.get("document_id"):
        raise ExportBlocked("release evidence is not bound to this exact contract document")
    if evidence.get("native_export_authorized") is not True:
        raise ExportBlocked("release evidence does not explicitly authorize native export")
    component_evidence = evidence.get("components")
    if not isinstance(component_evidence, Mapping):
        raise ExportBlocked("release evidence components must be an object")

    parsed: dict[str, tuple[ReleasedHole, ...]] = {}
    for component_id in component_ids:
        component = component_by_id(contract, component_id)
        record = component_evidence.get(component_id)
        if not isinstance(record, Mapping):
            raise ExportBlocked(f"release evidence is missing {component_id}")
        required_gates = component.get("planned_native_export", {}).get("export_preconditions", [])
        if not isinstance(required_gates, list) or not all(record.get(gate) is True for gate in required_gates):
            raise ExportBlocked(f"release evidence has unclosed export preconditions for {component_id}")
        holes = record.get("released_holes")
        if not isinstance(holes, list) or not holes:
            raise ExportBlocked(f"release evidence must define controlled released_holes for {component_id}")
        parsed[component_id] = tuple(_parse_hole(value) for value in holes)
        expected_datums = _datum_locations(component)
        if len(parsed[component_id]) != len(expected_datums):
            raise ExportBlocked(
                f"{component_id} released-hole count must equal the contract's visual datum count; "
                "review the controlled pattern rather than silently dropping locations"
            )
        observed_datums = tuple(hole.datum_xy_m for hole in parsed[component_id])
        if set(observed_datums) != set(expected_datums) or len(set(observed_datums)) != len(expected_datums):
            raise ExportBlocked(
                f"{component_id} released holes must bind one-for-one to the exact contract datum_xy_m locations"
            )
    return parsed


def _hole_cutter(cq: Any, hole: ReleasedHole) -> Any:
    diameter, depth = mm(hole.diameter_m), mm(hole.depth_m)
    x, y, z = mm3(hole.start_m, "released hole start")
    if hole.axis == "z":
        return cq.Workplane("XY").center(x, y).circle(diameter / 2.0).extrude(depth).translate((0.0, 0.0, z))
    if hole.axis == "x":
        return cq.Workplane("YZ").center(y, z).circle(diameter / 2.0).extrude(depth).translate((x, 0.0, 0.0))
    return cq.Workplane("XZ").center(x, z).circle(diameter / 2.0).extrude(depth).translate((0.0, y, 0.0))


def build_released_shape(cq: Any, contract: Mapping[str, Any], component_id: str, holes: Sequence[ReleasedHole]) -> Any:
    """Build a release-gated solid and remove only controlled interface holes."""

    shape = build_design_input_shape(cq, contract, component_id)
    for hole in holes:
        shape = shape.cut(_hole_cutter(cq, hole))
    return shape


def _assembly_translation_m(component: Mapping[str, Any]) -> tuple[float, float, float]:
    """Resolve only the current zero-RPY assembly placement from the contract."""

    coordinates = component.get("coordinate_contract", {}).get("authoritative_inputs", [])
    if not isinstance(coordinates, list):
        raise ValueError(f"{component.get('id')}.coordinate_contract.authoritative_inputs must be a list")
    for item in coordinates:
        if not isinstance(item, Mapping):
            continue
        direct = item.get("base_footprint_xyz_m")
        if direct is not None:
            return _tuple3(direct, "base_footprint assembly position")
        base_link = item.get("base_footprint_base_link_xyz_m")
        mount = item.get("mount_relative_xyz_m")
        if base_link is not None and mount is not None:
            base = _tuple3(base_link, "base-link assembly position")
            relative = _tuple3(mount, "mount-relative assembly position")
            return tuple(base[index] + relative[index] for index in range(3))  # type: ignore[return-value]
    raise ValueError(f"{component.get('id')} has no assembly translation in the contract")


def export_released_components_and_assembly(
    repository_root: Path, release_evidence_path: Path, output_directory: Path, component_ids: Sequence[str] = COMPONENT_IDS
) -> tuple[Path, ...]:
    """Export independent component STEP files plus a part-local assembly STEP.

    This is intentionally unreachable for the checked-in contract.  It is a
    future-only path that validates the revised contract and release package
    before importing CadQuery or creating an output directory.
    """

    contract = load_contract(repository_root)
    selected = tuple(component_ids)
    if not selected or any(component not in COMPONENT_IDS for component in selected):
        raise ValueError("select one or more supported first-batch components")
    released_holes = load_and_validate_release_evidence(contract, release_evidence_path, selected)
    cq = require_cadquery()
    output_directory.mkdir(parents=True, exist_ok=False)
    exported: list[Path] = []
    assembly = cq.Assembly(name="formal_vehicle_first_native_brep_batch")
    for component_id in selected:
        shape = build_released_shape(cq, contract, component_id, released_holes[component_id])
        if not shape.val().isValid():
            raise RuntimeError(f"CadQuery returned an invalid B-rep for {component_id}")
        step_path = output_directory / f"{component_id}.step"
        cq.exporters.export(shape, str(step_path))
        assembly.add(shape, name=component_id, loc=cq.Location(cq.Vector(*mm3(_assembly_translation_m(component_by_id(contract, component_id)), "assembly translation"))))
        exported.append(step_path)
    assembly_path = output_directory / "formal_vehicle_first_native_brep_batch_assembly.step"
    assembly.save(str(assembly_path))
    exported.append(assembly_path)
    return tuple(exported)


def _summary(contract: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "contract_document_id": contract.get("document_id"),
        "contract_status": contract.get("status"),
        "supported_components": list(COMPONENT_IDS),
        "export_permitted_now": contract.get("status") == RELEASED_STATUS,
        "cadquery_imported": False,
        "static_only": True,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repository_root_from_source())
    parser.add_argument("--summary", action="store_true", help="read contract only; default action")
    parser.add_argument("--export", action="store_true", help="future-only, release-gated STEP export")
    parser.add_argument("--release-evidence", type=Path)
    parser.add_argument("--output-directory", type=Path)
    parser.add_argument("--component", action="append", choices=COMPONENT_IDS)
    args = parser.parse_args(argv)
    contract = load_contract(args.repo_root.resolve())
    if not args.export:
        print(json.dumps(_summary(contract), ensure_ascii=False, indent=2, sort_keys=True))
        return 0
    if args.release_evidence is None or args.output_directory is None:
        parser.error("--export requires --release-evidence and --output-directory")
    try:
        exported = export_released_components_and_assembly(
            args.repo_root.resolve(),
            args.release_evidence.resolve(),
            args.output_directory.resolve(),
            tuple(args.component or COMPONENT_IDS),
        )
    except ExportBlocked as exc:
        print(f"export blocked: {exc}")
        return 3
    print(json.dumps({"outcome": "exported", "files": [str(path) for path in exported]}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
