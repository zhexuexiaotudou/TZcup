#!/usr/bin/env python3
"""Fail-closed static validation for the first native B-rep design-input batch.

This checker reads only UTF-8 JSON and repository source text.  It deliberately
does not start WSL, Gazebo, Docker, a CAD executable, a mesh converter, or a
STEP exporter.  Its job is to prevent a visual/simulation parameter plan from
being mistaken for native B-rep or manufacturing evidence.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = REPOSITORY_ROOT / "config" / "high_fidelity_vehicle" / "native_brep_first_batch_contract.json"
DEFAULT_SCHEMA = REPOSITORY_ROOT / "config" / "high_fidelity_vehicle" / "native_brep_first_batch_contract.schema.json"

STATUS = "design_input_pending_native_export"
EXPECTED_IDS = {
    "arm_pedestal_adapter",
    "sensor_tower",
    "cleaning_head_brackets",
    "storage_frame",
}
REQUIRED_PROHIBITIONS = {
    "mesh_to_step_conversion",
    "faceted_or_tessellated_step_export",
    "mesh_import_as_native_brep_substitute",
    "placeholder_fcstd_or_step_artifact",
}
EXPECTED_ASSETS = {
    "arm_pedestal_adapter": "starter_ws/src/sanitation_vehicle_description/meshes/generated/platform/arm_mount_adapter.stl",
    "sensor_tower": "starter_ws/src/sanitation_vehicle_description/meshes/generated/platform/sensor_mast.stl",
    "cleaning_head_brackets": "starter_ws/src/sanitation_vehicle_description/meshes/project/cleaning/cleaning_mount_frame.stl",
    "storage_frame": "starter_ws/src/sanitation_vehicle_description/meshes/project/storage/storage_mount_tray.stl",
}
CRITICAL_SOURCE_SNIPPETS = {
    "scripts/generate_platform_auxiliary_meshes.py": [
        "def arm_mount()",
        "rounded_box((0.280, 0.220, 0.012)",
        "def sensor_mast()",
        "rounded_box((0.190, 0.150, 0.016)",
    ],
    "starter_ws/src/sanitation_vehicle_description/cad/formal_vehicle/generate_cleaning_storage_meshes.py": [
        "def cleaning_mount_frame()",
        "def storage_mount()",
        "m = box((0.570, 0.620, 0.012))",
    ],
    "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/a300_platform.xacro": [
        '<link name="sensor_mast_link">',
        '<link name="arm_mount_link">',
    ],
    "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/cleaning_mechanism.xacro": [
        '<joint name="cleaning_lift_joint" type="prismatic">',
        'x="0.18" y="0.25"',
    ],
    "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/storage_system.xacro": [
        '<link name="storage_system_mount_link">',
        'size="0.570 0.620 0.012"',
    ],
}


def _load_json(path: Path, label: str, errors: list[str]) -> dict[str, Any] | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        errors.append(f"cannot read {label}: {exc}")
        return None
    if not isinstance(payload, dict):
        errors.append(f"{label} root must be an object")
        return None
    return payload


def _nonempty_string(value: Any, label: str, errors: list[str]) -> str | None:
    if not isinstance(value, str) or not value.strip():
        errors.append(f"{label} must be a non-empty string")
        return None
    return value


def _repository_file(root: Path, value: Any, label: str, errors: list[str]) -> Path | None:
    text = _nonempty_string(value, label, errors)
    if text is None:
        return None
    path = Path(text)
    if path.is_absolute() or ".." in path.parts:
        errors.append(f"{label} must be a repository-relative path")
        return None
    absolute = root / path
    if not absolute.is_file():
        errors.append(f"{label} does not exist: {text}")
    return absolute


def _planned_output(root: Path, value: Any, label: str, suffixes: tuple[str, ...], errors: list[str]) -> None:
    text = _nonempty_string(value, label, errors)
    if text is None:
        return
    path = Path(text)
    if path.is_absolute() or ".." in path.parts:
        errors.append(f"{label} must be a repository-relative path")
        return
    if path.suffix not in suffixes:
        errors.append(f"{label} must use one of {suffixes}")
    if (root / path).exists():
        errors.append(f"{label} already exists; this plan must not contain a fake native/STEP artifact")


def _section(item: dict[str, Any], name: str, item_id: str, errors: list[str]) -> dict[str, Any]:
    value = item.get(name)
    if not isinstance(value, dict):
        errors.append(f"{item_id}.{name} must be an object")
        return {}
    for field in ("authoritative_inputs", "pending_native_export_inputs", "boundary"):
        if field not in value:
            errors.append(f"{item_id}.{name} is missing {field}")
    if not isinstance(value.get("authoritative_inputs"), list) or not value["authoritative_inputs"]:
        errors.append(f"{item_id}.{name}.authoritative_inputs must be non-empty")
    if not isinstance(value.get("pending_native_export_inputs"), list) or not value["pending_native_export_inputs"]:
        errors.append(f"{item_id}.{name}.pending_native_export_inputs must be non-empty")
    _nonempty_string(value.get("boundary"), f"{item_id}.{name}.boundary", errors)
    return value


def _features(section: dict[str, Any]) -> dict[str, dict[str, Any]]:
    values = section.get("authoritative_inputs")
    if not isinstance(values, list):
        return {}
    return {
        value["feature"]: value
        for value in values
        if isinstance(value, dict) and isinstance(value.get("feature"), str)
    }


def _validate_schema(schema: dict[str, Any] | None, errors: list[str]) -> None:
    if schema is None:
        return
    if schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        errors.append("schema must declare JSON Schema draft 2020-12")
    if schema.get("title") != "TZcup native B-rep first-batch parametric contract":
        errors.append("schema title does not identify the first-batch contract")
    properties = schema.get("properties")
    if not isinstance(properties, dict):
        errors.append("schema.properties must be an object")
        return
    for name in ("status", "coordinate_system", "items", "prohibited_methods"):
        if name not in properties:
            errors.append(f"schema is missing properties.{name}")


def _validate_sources(root: Path, item: dict[str, Any], item_id: str, errors: list[str]) -> None:
    sources = item.get("evidence_sources")
    if not isinstance(sources, list) or len(sources) < 4:
        errors.append(f"{item_id}.evidence_sources must contain generator, layout and URDF evidence")
        sources = []
    paths: set[str] = set()
    for index, source in enumerate(sources):
        if not isinstance(source, dict):
            errors.append(f"{item_id}.evidence_sources[{index}] must be an object")
            continue
        path_text = _nonempty_string(source.get("path"), f"{item_id}.evidence_sources[{index}].path", errors)
        if path_text:
            paths.add(path_text)
            _repository_file(root, path_text, f"{item_id}.evidence_sources[{index}].path", errors)
        _nonempty_string(source.get("locator"), f"{item_id}.evidence_sources[{index}].locator", errors)
        _nonempty_string(source.get("role"), f"{item_id}.evidence_sources[{index}].role", errors)
    if not any("generate_" in path for path in paths):
        errors.append(f"{item_id}.evidence_sources must include a generate_*_meshes.py source")
    if "config/high_fidelity_vehicle/formal_vehicle_layout.yaml" not in paths:
        errors.append(f"{item_id}.evidence_sources must cite formal_vehicle_layout.yaml")
    if not any(path.endswith(".xacro") for path in paths):
        errors.append(f"{item_id}.evidence_sources must cite URDF/Xacro")

    meshes = item.get("source_meshes")
    if not isinstance(meshes, list) or not meshes:
        errors.append(f"{item_id}.source_meshes must be non-empty")
        return
    assets: set[str] = set()
    for index, mesh in enumerate(meshes):
        if not isinstance(mesh, dict):
            errors.append(f"{item_id}.source_meshes[{index}] must be an object")
            continue
        asset = _nonempty_string(mesh.get("source_asset"), f"{item_id}.source_meshes[{index}].source_asset", errors)
        if asset:
            assets.add(asset)
            asset_path = _repository_file(root, asset, f"{item_id}.source_meshes[{index}].source_asset", errors)
            if asset_path is not None and asset_path.suffix.lower() != ".stl":
                errors.append(f"{item_id}.source_meshes[{index}].source_asset must be an STL")
        generator = mesh.get("generator")
        if not isinstance(generator, dict):
            errors.append(f"{item_id}.source_meshes[{index}].generator must be an object")
        else:
            generator_path = _repository_file(root, generator.get("path"), f"{item_id}.source_meshes[{index}].generator.path", errors)
            if generator_path is not None and not generator_path.name.startswith("generate_"):
                errors.append(f"{item_id}.source_meshes[{index}].generator.path must be a generator")
        if mesh.get("status") != "visual_or_simulation_dimension_input_not_native_brep":
            errors.append(f"{item_id}.source_meshes[{index}].status must keep the mesh non-native")
    expected_asset = EXPECTED_ASSETS[item_id]
    if expected_asset not in assets:
        errors.append(f"{item_id} must include source asset {expected_asset}")


def _validate_planned_export(root: Path, item: dict[str, Any], item_id: str, errors: list[str]) -> None:
    export = item.get("planned_native_export")
    if not isinstance(export, dict):
        errors.append(f"{item_id}.planned_native_export must be an object")
        return
    _planned_output(root, export.get("native_source_path"), f"{item_id}.planned_native_export.native_source_path", (".FCStd",), errors)
    _planned_output(root, export.get("step_path"), f"{item_id}.planned_native_export.step_path", (".step", ".stp"), errors)
    if export.get("must_not_exist_yet") is not True:
        errors.append(f"{item_id}.planned_native_export.must_not_exist_yet must be true")
    conditions = export.get("export_preconditions")
    if not isinstance(conditions, list) or len(conditions) < 3 or not all(isinstance(value, str) and value for value in conditions):
        errors.append(f"{item_id}.planned_native_export.export_preconditions must list at least three non-empty gates")


def _validate_item_specifics(item: dict[str, Any], item_id: str, errors: list[str]) -> None:
    dimensions = _features(_section(item, "dimension_contract", item_id, errors))
    coordinates = _section(item, "coordinate_contract", item_id, errors)
    interfaces = _section(item, "interface_contract", item_id, errors)
    materials = _section(item, "material_boundary", item_id, errors)
    mass = _section(item, "mass_inertia_contract", item_id, errors)
    if not coordinates or not interfaces or not materials or not mass:
        return

    if item_id == "arm_pedestal_adapter":
        feature = dimensions.get("six_visual_fastener_envelopes", {})
        if feature.get("count") != 6 or feature.get("bolt_circle_diameter_m") != 0.192:
            errors.append("arm_pedestal_adapter must retain six 192 mm bolt-circle datums")
        if "pending_controlled_UR5e_OEM_drawing" not in str(interfaces):
            errors.append("arm_pedestal_adapter must retain the pending controlled UR5e interface boundary")
    elif item_id == "sensor_tower":
        feature = dimensions.get("base_visual_fastener_envelopes", {})
        if feature.get("count") != 4:
            errors.append("sensor_tower must retain four base fastener datums")
        if "candidate_M8_pending_interface_confirmation" not in str(interfaces):
            errors.append("sensor_tower must retain its non-released M8 candidate boundary")
    elif item_id == "cleaning_head_brackets":
        feature = dimensions.get("guide_columns", {})
        if feature.get("count") != 4 or feature.get("nominal_length_m") != 0.180:
            errors.append("cleaning_head_brackets must retain four 180 mm guide inputs")
        if "travel_m" not in str(coordinates) or "0.1" not in str(coordinates):
            errors.append("cleaning_head_brackets must retain the 100 mm lift-travel input")
    elif item_id == "storage_frame":
        feature = dimensions.get("six_visual_fastener_envelopes", {})
        if feature.get("count") != 6:
            errors.append("storage_frame must retain six tray fastener datums")
        if "final_usable_capacity_l" not in str(dimensions):
            errors.append("storage_frame must retain the storage-capacity dependency boundary")


def _validate_source_drift(root: Path, errors: list[str]) -> None:
    for relative_path, snippets in CRITICAL_SOURCE_SNIPPETS.items():
        path = root / relative_path
        try:
            contents = path.read_text(encoding="utf-8")
        except OSError as exc:
            errors.append(f"cannot read critical source {relative_path}: {exc}")
            continue
        for snippet in snippets:
            if snippet not in contents:
                errors.append(f"critical source drift: {relative_path} no longer contains {snippet!r}")


def validate(root: Path, contract_path: Path = DEFAULT_CONTRACT, schema_path: Path = DEFAULT_SCHEMA) -> dict[str, Any]:
    root = root.resolve()
    errors: list[str] = []
    contract = _load_json(contract_path, "contract", errors)
    schema = _load_json(schema_path, "schema", errors)
    _validate_schema(schema, errors)
    if contract is None:
        return {"valid": False, "errors": errors}

    if contract.get("schema_version") != 1:
        errors.append("schema_version must equal 1")
    if contract.get("document_id") != "tzcup_native_brep_first_batch_parametric_contract_v1":
        errors.append("document_id is not the expected first-batch contract")
    if contract.get("status") != STATUS:
        errors.append(f"status must remain {STATUS}")
    _nonempty_string(contract.get("claim_boundary"), "claim_boundary", errors)
    prohibited = contract.get("prohibited_methods")
    if not isinstance(prohibited, list) or not REQUIRED_PROHIBITIONS.issubset(set(prohibited)):
        errors.append("prohibited_methods must explicitly reject mesh conversion and placeholder artifacts")

    coordinate_system = contract.get("coordinate_system")
    if not isinstance(coordinate_system, dict):
        errors.append("coordinate_system must be an object")
    else:
        if coordinate_system.get("units") != "m_and_rad":
            errors.append("coordinate_system.units must be m_and_rad")
        if coordinate_system.get("root_frame") != "base_footprint":
            errors.append("coordinate_system.root_frame must be base_footprint")
        _nonempty_string(coordinate_system.get("local_part_rule"), "coordinate_system.local_part_rule", errors)
        sources = coordinate_system.get("sources")
        if not isinstance(sources, list) or len(sources) < 2:
            errors.append("coordinate_system.sources must be non-empty")
        else:
            for index, source in enumerate(sources):
                if not isinstance(source, dict):
                    errors.append(f"coordinate_system.sources[{index}] must be an object")
                    continue
                _repository_file(root, source.get("path"), f"coordinate_system.sources[{index}].path", errors)

    items = contract.get("items")
    if not isinstance(items, list):
        errors.append("items must be an array")
        items = []
    ids: list[str] = []
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            errors.append(f"items[{index}] must be an object")
            continue
        item_id = _nonempty_string(item.get("id"), f"items[{index}].id", errors)
        if item_id is None:
            continue
        ids.append(item_id)
        if item_id not in EXPECTED_IDS:
            errors.append(f"unexpected first-batch item: {item_id}")
            continue
        if item.get("status") != STATUS:
            errors.append(f"{item_id}.status must remain {STATUS}")
        _nonempty_string(item.get("scope"), f"{item_id}.scope", errors)
        _validate_sources(root, item, item_id, errors)
        _validate_item_specifics(item, item_id, errors)
        dependencies = item.get("dependencies")
        if not isinstance(dependencies, list) or not dependencies or not all(isinstance(value, str) and value for value in dependencies):
            errors.append(f"{item_id}.dependencies must be a non-empty string array")
        _validate_planned_export(root, item, item_id, errors)
    if len(ids) != len(set(ids)):
        errors.append("item ids must be unique")
    if set(ids) != EXPECTED_IDS:
        errors.append(f"first batch must contain exactly {sorted(EXPECTED_IDS)}")

    _validate_source_drift(root, errors)
    return {
        "valid": not errors,
        "errors": errors,
        "summary": {
            "status": STATUS,
            "first_batch_component_count": len(ids),
            "native_or_step_artifacts_created": 0,
            "static_only": True,
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=REPOSITORY_ROOT)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    report = validate(args.root, args.contract, args.schema)
    rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
