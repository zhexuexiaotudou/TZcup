#!/usr/bin/env python3
"""Fail-closed static validator for the native B-rep reconstruction plan.

This checker deliberately reads JSON and file names only.  It never launches a
CAD kernel, WSL, Docker, Gazebo, a mesh converter, or a STEP exporter.  The
plan is useful precisely because it records the work still required without
creating placeholder FCStd or STEP files that could be mistaken for evidence.
"""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = REPOSITORY_ROOT / "config" / "high_fidelity_vehicle" / "native_brep_reconstruction_manifest.json"
DEFAULT_SCHEMA = REPOSITORY_ROOT / "config" / "high_fidelity_vehicle" / "native_brep_reconstruction_manifest.schema.json"

GENERATOR_COUNTS = {
    "product_bodywork": 47,
    "cleaning_storage": 61,
    "power_service_hardware": 18,
}
PENDING = "pending_native_brep_reconstruction"
EXCLUDED = "excluded_non_project_authored_vendor_reference"


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


def _relative_file(root: Path, value: Any, label: str, errors: list[str], *, suffix: str | None = None) -> Path | None:
    text = _nonempty_string(value, label, errors)
    if text is None:
        return None
    path = root / text
    if suffix is not None and path.suffix.lower() != suffix:
        errors.append(f"{label} must end in {suffix}")
    return path


def _validate_schema(schema: dict[str, Any] | None, errors: list[str]) -> None:
    if schema is None:
        return
    if schema.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        errors.append("schema must declare JSON Schema draft 2020-12")
    if schema.get("title") != "TZcup native B-rep reconstruction manifest":
        errors.append("schema title does not identify the native B-rep reconstruction manifest")
    properties = schema.get("properties")
    if not isinstance(properties, dict):
        errors.append("schema.properties must be an object")
        return
    for name in ("source_generators", "profiles", "assemblies", "parts", "excluded_generator_outputs", "prohibited_methods"):
        if name not in properties:
            errors.append(f"schema is missing properties.{name}")


def _validate_paths_are_targets(root: Path, part: dict[str, Any], index: int, errors: list[str]) -> None:
    label = f"parts[{index}]"
    native = _relative_file(root, part.get("target_native_source"), f"{label}.target_native_source", errors, suffix=".fcstd")
    step_part = _relative_file(root, part.get("target_step_part"), f"{label}.target_step_part", errors, suffix=".step")
    step_assembly = _relative_file(root, part.get("target_step_assembly"), f"{label}.target_step_assembly", errors, suffix=".step")
    for target, target_label in ((native, "target_native_source"), (step_part, "target_step_part"), (step_assembly, "target_step_assembly")):
        if target is not None and target.is_file():
            errors.append(f"{label}.{target_label} exists; this planning manifest must not claim a generated native/STEP artifact")


def validate(root: Path, manifest_path: Path = DEFAULT_MANIFEST, schema_path: Path = DEFAULT_SCHEMA) -> dict[str, Any]:
    root = root.resolve()
    errors: list[str] = []
    manifest = _load_json(manifest_path, "manifest", errors)
    schema = _load_json(schema_path, "schema", errors)
    _validate_schema(schema, errors)
    if manifest is None:
        return {"valid": False, "errors": errors}

    if manifest.get("schema_version") != 1:
        errors.append("schema_version must equal 1")
    if manifest.get("status") != PENDING:
        errors.append(f"status must equal {PENDING}")
    prohibited = manifest.get("prohibited_methods")
    if not isinstance(prohibited, list) or "mesh_to_step_conversion" not in prohibited:
        errors.append("prohibited_methods must explicitly include mesh_to_step_conversion")

    profiles = manifest.get("profiles")
    if not isinstance(profiles, dict) or not profiles:
        errors.append("profiles must be a non-empty object")
        profiles = {}
    else:
        for profile_id, profile in profiles.items():
            if not isinstance(profile, dict):
                errors.append(f"profiles.{profile_id} must be an object")
                continue
            for field in ("dimension_sources", "material_and_mass", "interface", "coordinate_system"):
                if field not in profile:
                    errors.append(f"profiles.{profile_id} is missing {field}")

    assemblies = manifest.get("assemblies")
    if not isinstance(assemblies, list) or not assemblies:
        errors.append("assemblies must be a non-empty array")
        assembly_steps: set[str] = set()
    else:
        assembly_ids: set[str] = set()
        assembly_steps = set()
        for index, assembly in enumerate(assemblies):
            label = f"assemblies[{index}]"
            if not isinstance(assembly, dict):
                errors.append(f"{label} must be an object")
                continue
            assembly_id = _nonempty_string(assembly.get("id"), f"{label}.id", errors)
            if assembly_id is not None:
                if assembly_id in assembly_ids:
                    errors.append(f"duplicate assembly id: {assembly_id}")
                assembly_ids.add(assembly_id)
            profile = _nonempty_string(assembly.get("profile"), f"{label}.profile", errors)
            if profile not in profiles:
                errors.append(f"{label}.profile must reference a complete profile")
            if assembly.get("status") != PENDING:
                errors.append(f"{label}.status must remain {PENDING}")
            _validate_paths_are_targets(root, {"target_native_source": assembly.get("target_native_source"), "target_step_part": assembly.get("target_step_assembly"), "target_step_assembly": assembly.get("target_step_assembly")}, index, errors)
            target_step = _nonempty_string(assembly.get("target_step_assembly"), f"{label}.target_step_assembly", errors)
            if target_step is not None:
                assembly_steps.add(target_step)

    generators = manifest.get("source_generators")
    if not isinstance(generators, list) or len(generators) != len(GENERATOR_COUNTS):
        errors.append("source_generators must list exactly the three current Python mesh generators")
        generators = []
    generator_by_id: dict[str, dict[str, Any]] = {}
    for index, generator in enumerate(generators):
        label = f"source_generators[{index}]"
        if not isinstance(generator, dict):
            errors.append(f"{label} must be an object")
            continue
        generator_id = _nonempty_string(generator.get("id"), f"{label}.id", errors)
        source = _relative_file(root, generator.get("path"), f"{label}.path", errors, suffix=".py")
        if source is not None and not source.is_file():
            errors.append(f"{label}.path is not an existing source generator")
        if generator_id is not None:
            if generator_id not in GENERATOR_COUNTS:
                errors.append(f"{label}.id is not a supported source generator: {generator_id}")
            elif generator.get("generated_mesh_count") != GENERATOR_COUNTS[generator_id]:
                errors.append(f"{label}.generated_mesh_count must equal {GENERATOR_COUNTS[generator_id]}")
            generator_by_id[generator_id] = generator

    parts = manifest.get("parts")
    excluded = manifest.get("excluded_generator_outputs")
    if not isinstance(parts, list) or not parts:
        errors.append("parts must be a non-empty array")
        parts = []
    if not isinstance(excluded, list):
        errors.append("excluded_generator_outputs must be an array")
        excluded = []

    seen_ids: set[str] = set()
    seen_meshes: set[tuple[str, str]] = set()
    part_counts: Counter[str] = Counter()
    excluded_counts: Counter[str] = Counter()
    for index, part in enumerate(parts):
        label = f"parts[{index}]"
        if not isinstance(part, dict):
            errors.append(f"{label} must be an object")
            continue
        part_id = _nonempty_string(part.get("id"), f"{label}.id", errors)
        if part_id is not None:
            if part_id in seen_ids:
                errors.append(f"duplicate part id: {part_id}")
            seen_ids.add(part_id)
        generator_id = _nonempty_string(part.get("source_generator"), f"{label}.source_generator", errors)
        if generator_id not in generator_by_id:
            errors.append(f"{label}.source_generator must reference a declared generator")
        else:
            part_counts[generator_id] += 1
        mesh = _nonempty_string(part.get("source_mesh"), f"{label}.source_mesh", errors)
        if mesh is not None and generator_id is not None:
            key = (generator_id, mesh)
            if key in seen_meshes:
                errors.append(f"duplicate source mesh entry: {generator_id}/{mesh}")
            seen_meshes.add(key)
            source_mesh = root / "starter_ws" / "src" / "sanitation_vehicle_description" / "meshes" / mesh
            if source_mesh.suffix.lower() != ".stl" or not source_mesh.is_file():
                errors.append(f"{label}.source_mesh is not an existing STL under the vehicle mesh root: {mesh}")
        profile = _nonempty_string(part.get("profile"), f"{label}.profile", errors)
        if profile not in profiles:
            errors.append(f"{label}.profile must reference a complete profile")
        if part.get("status") != PENDING:
            errors.append(f"{label}.status must remain {PENDING}")
        if part.get("conversion_policy") != "native_brep_feature_rebuild_only":
            errors.append(f"{label}.conversion_policy must prohibit mesh conversion")
        if part.get("target_step_assembly") not in assembly_steps:
            errors.append(f"{label}.target_step_assembly must reference a declared pending assembly target")
        _validate_paths_are_targets(root, part, index, errors)

    for index, item in enumerate(excluded):
        label = f"excluded_generator_outputs[{index}]"
        if not isinstance(item, dict):
            errors.append(f"{label} must be an object")
            continue
        generator_id = _nonempty_string(item.get("source_generator"), f"{label}.source_generator", errors)
        if generator_id not in generator_by_id:
            errors.append(f"{label}.source_generator must reference a declared generator")
        else:
            excluded_counts[generator_id] += 1
        mesh = _nonempty_string(item.get("source_mesh"), f"{label}.source_mesh", errors)
        if mesh is not None and generator_id is not None:
            key = (generator_id, mesh)
            if key in seen_meshes:
                errors.append(f"source mesh is listed more than once: {generator_id}/{mesh}")
            seen_meshes.add(key)
            source_mesh = root / "starter_ws" / "src" / "sanitation_vehicle_description" / "meshes" / mesh
            if source_mesh.suffix.lower() != ".stl" or not source_mesh.is_file():
                errors.append(f"{label}.source_mesh is not an existing STL under the vehicle mesh root: {mesh}")
        if item.get("status") != EXCLUDED:
            errors.append(f"{label}.status must be {EXCLUDED}")
        for field in ("reason", "bom_reference"):
            _nonempty_string(item.get(field), f"{label}.{field}", errors)

    for generator_id, expected in GENERATOR_COUNTS.items():
        actual = part_counts[generator_id] + excluded_counts[generator_id]
        if actual != expected:
            errors.append(f"{generator_id} coverage is {actual}; expected {expected} generated mesh outputs")
    if len(seen_meshes) != sum(GENERATOR_COUNTS.values()):
        errors.append(f"unique source mesh coverage is {len(seen_meshes)}; expected {sum(GENERATOR_COUNTS.values())}")

    return {
        "valid": not errors,
        "errors": errors,
        "summary": {
            "project_authored_parts_pending": len(parts),
            "excluded_vendor_reference_meshes": len(excluded),
            "source_meshes_accounted_for": len(seen_meshes),
            "native_or_step_artifacts_created": 0,
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=REPOSITORY_ROOT)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    report = validate(args.root, args.manifest, args.schema)
    rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
