#!/usr/bin/env python3
"""Fail-closed preflight for native, editable vehicle CAD delivery.

This audit intentionally does not run FreeCAD, OpenSCAD, WSL, Docker, Gazebo,
or a mesh converter.  It is safe to use on a memory-constrained Windows host:
it inventories filenames and source text only, then reports whether the
repository can *honestly* claim an editable B-rep STEP assembly export.

An STL/OBJ/DAE/glTF file, including one converted to STEP by an importer, is
never accepted as native CAD evidence.  A future delivery must add real
editable source documents, a component-level assembly manifest, a B-rep STEP
artifact, and an available Windows B-rep exporter before this preflight can
pass.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import importlib.util
import json
import os
import shutil
import sys
from pathlib import Path
from typing import Any, Iterable


SCRIPT_ROOT = Path(__file__).resolve().parents[1]
CAD_ROOT = Path("starter_ws/src/sanitation_vehicle_description/cad/formal_vehicle")
NATIVE_BREP_PYTHON_ROOT = Path("starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle")
VEHICLE_DESCRIPTION_ROOT = Path("starter_ws/src/sanitation_vehicle_description")
ASSEMBLY_MANIFEST = CAD_ROOT / "native_cad_assembly_manifest.json"
COMPONENT_ADDRESSABLE_ASSEMBLY_DRAFT = Path(
    "config/high_fidelity_vehicle/component_addressable_native_cad_assembly_manifest_draft.json"
)
EXPORT_RECEIPT = CAD_ROOT / "native_cad_export_receipt.json"

NATIVE_SOURCE_SUFFIXES = {
    ".fcstd",
    ".fcmacro",
    ".f3d",
    ".ipt",
    ".sldprt",
    ".sldasm",
    ".x_t",
    ".x_b",
}
STEP_SUFFIXES = {".step", ".stp"}
MESH_SUFFIXES = {".stl", ".obj", ".dae", ".gltf", ".glb", ".ply", ".3mf"}
PYTHON_MESH_MARKERS = (
    "write_binary_stl",
    "write_ascii_stl",
    "exportstl",
    "export_stl",
    ".stl",
    "triangles",
    "triangulated",
    "faceted_brep",
    "tessellated",
    "trimesh",
    "meshio",
    "struct.pack",
)
CADQUERY_CONSTRUCTION_CALLS = {
    "Workplane",
    "Assembly",
    "box",
    "cylinder",
    "sphere",
    "extrude",
    "revolve",
    "loft",
    "sweep",
    "union",
    "cut",
}
TOOL_CANDIDATES = {
    "freecad_cmd": ("FreeCADCmd.exe", "FreeCADCmd"),
    "freecad_gui": ("FreeCAD.exe", "FreeCAD"),
    "openscad": ("OpenSCAD.exe", "openscad.exe", "openscad"),
    "assimp": ("assimp.exe", "assimp_cmd.exe", "assimp"),
}
PYTHON_BREP_MODULES = ("FreeCAD", "cadquery", "build123d", "OCC", "OCP")


def relative_paths(paths: Iterable[Path], root: Path) -> list[str]:
    return sorted(path.relative_to(root).as_posix() for path in paths)


def file_paths(root: Path, suffixes: set[str]) -> list[Path]:
    return sorted(
        (path for path in root.rglob("*") if path.is_file() and path.suffix.lower() in suffixes),
        key=lambda path: path.as_posix().lower(),
    )


def source_has_mesh_generator_marker(path: Path) -> bool:
    try:
        source = path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        return False
    lowered = source.lower()
    return any(marker in lowered for marker in PYTHON_MESH_MARKERS)


class CadQuerySourceInspector(ast.NodeVisitor):
    """Static proof that a Python file contains a lazy CadQuery feature model."""

    def __init__(self) -> None:
        self.function_depth = 0
        self.lazy_cadquery_import = False
        self.cadquery_aliases: set[str] = set()
        self.cadquery_constructor_aliases: dict[str, str] = {}
        self.cadquery_constructors: set[str] = set()
        self.candidate_cadquery_constructors: set[tuple[str, str]] = set()
        self.construction_calls: set[str] = set()

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        self.function_depth += 1
        self.generic_visit(node)
        self.function_depth -= 1

    visit_AsyncFunctionDef = visit_FunctionDef

    def visit_Import(self, node: ast.Import) -> None:
        if self.function_depth:
            for alias in node.names:
                if alias.name == "cadquery":
                    self.lazy_cadquery_import = True
                    self.cadquery_aliases.add(alias.asname or "cadquery")

    def visit_ImportFrom(self, node: ast.ImportFrom) -> None:
        if self.function_depth and node.module == "cadquery":
            self.lazy_cadquery_import = True
            self.cadquery_constructor_aliases.update(
                {alias.asname or alias.name: alias.name for alias in node.names if alias.name in {"Workplane", "Assembly"}}
            )

    def visit_Call(self, node: ast.Call) -> None:
        if isinstance(node.func, ast.Attribute) and node.func.attr in CADQUERY_CONSTRUCTION_CALLS:
            self.construction_calls.add(node.func.attr)
            if (
                node.func.attr in {"Workplane", "Assembly"}
                and isinstance(node.func.value, ast.Name)
            ):
                # Function definitions may precede require_cadquery() in the
                # file, so resolve this alias only after the complete AST has
                # been visited rather than relying on traversal order.
                self.candidate_cadquery_constructors.add((node.func.value.id, node.func.attr))
        elif isinstance(node.func, ast.Name) and node.func.id in self.cadquery_constructor_aliases:
            constructor = self.cadquery_constructor_aliases[node.func.id]
            self.cadquery_constructors.add(constructor)
            self.construction_calls.add(constructor)
        self.generic_visit(node)


def source_manifest_hashes_source(path: Path, root: Path) -> bool:
    """Require a checked-in local source manifest to bind this Python source.

    The manifest is intentionally separate from the later export receipt: it
    proves static source provenance even while a design-input source is not
    yet authorised to invoke a CAD kernel or emit STEP.
    """

    manifest_root = root / NATIVE_BREP_PYTHON_ROOT
    expected_path = path.relative_to(root).as_posix()
    for manifest_path in sorted(manifest_root.glob("*_source_manifest.json")):
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        verification = manifest.get("verification") if isinstance(manifest, dict) else None
        if not isinstance(verification, dict) or verification.get("algorithm") != "sha256":
            continue
        source_files = manifest.get("source_files")
        if not isinstance(source_files, list):
            continue
        for entry in source_files:
            if not isinstance(entry, dict) or entry.get("path") != expected_path:
                continue
            role = entry.get("role")
            expected_hash = entry.get("sha256")
            if (
                isinstance(role, str)
                and "cadquery" in role.lower()
                and "brep" in role.lower()
                and isinstance(expected_hash, str)
                and sha256(path) == expected_hash.lower()
            ):
                return True
    return False


def python_cadquery_source_rejection_codes(path: Path, root: Path) -> list[str]:
    """Return fail-closed reasons why *path* cannot count as editable B-rep."""

    expected_root = (root / NATIVE_BREP_PYTHON_ROOT).resolve()
    try:
        path.resolve().relative_to(expected_root)
    except ValueError:
        return ["PYTHON_SOURCE_OUTSIDE_NATIVE_BREP_DIRECTORY"]
    if path.suffix.lower() != ".py" or not path.is_file():
        return ["NOT_A_PYTHON_NATIVE_BREP_SOURCE"]
    try:
        source = path.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(path))
    except (OSError, UnicodeDecodeError, SyntaxError):
        return ["INVALID_PYTHON_NATIVE_BREP_SOURCE"]

    inspector = CadQuerySourceInspector()
    inspector.visit(tree)
    inspector.cadquery_constructors.update(
        constructor
        for alias, constructor in inspector.candidate_cadquery_constructors
        if alias in inspector.cadquery_aliases
    )
    rejection_codes: list[str] = []
    if not inspector.lazy_cadquery_import:
        rejection_codes.append("CADQUERY_IMPORT_NOT_LAZY")
    if not inspector.cadquery_constructors or not {"Workplane", "Assembly"}.intersection(inspector.construction_calls) or not (
        inspector.construction_calls - {"Workplane", "Assembly"}
    ):
        rejection_codes.append("CADQUERY_ENTITY_CONSTRUCTION_UNPROVEN")
    if source_has_mesh_generator_marker(path):
        rejection_codes.append("PYTHON_SOURCE_EMITS_OR_REFERENCES_MESH")
    if not source_manifest_hashes_source(path, root):
        rejection_codes.append("CADQUERY_SOURCE_MANIFEST_OR_HASH_UNPROVEN")
    return rejection_codes


def windows_tool_paths() -> dict[str, str | None]:
    """Return executable paths without launching an external CAD application."""

    result: dict[str, str | None] = {}
    for name, candidates in TOOL_CANDIDATES.items():
        resolved = next((shutil.which(candidate) for candidate in candidates if shutil.which(candidate)), None)
        result[name] = resolved
    return result


def component_addressable_assembly_draft_state(root: Path) -> dict[str, Any]:
    """Read the separate static draft without treating it as export evidence."""

    draft_path = root / COMPONENT_ADDRESSABLE_ASSEMBLY_DRAFT
    state: dict[str, Any] = {
        "path": COMPONENT_ADDRESSABLE_ASSEMBLY_DRAFT.as_posix() if draft_path.is_file() else None,
        "valid": False,
        "status": None,
        "component_count": None,
        "supplier_excluded_count": None,
    }
    if not draft_path.is_file():
        return state
    validator_path = Path(__file__).with_name("validate_component_addressable_native_cad_assembly_draft.py")
    try:
        spec = importlib.util.spec_from_file_location("component_addressable_native_cad_assembly_draft", validator_path)
        if spec is None or spec.loader is None:
            return state
        validator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(validator)
        report = validator.audit(root)
    except Exception:
        return state
    state.update(
        {
            "valid": report.get("status")
            == "STATIC_COMPONENT_ADDRESSABLE_DRAFT_VALID_NATIVE_EXPORT_BLOCKED"
            and report.get("draft_structurally_valid") is True,
            "status": report.get("status"),
            "component_count": report.get("component_count"),
            "supplier_excluded_count": report.get("supplier_excluded_count"),
        }
    )
    return state


def python_brep_modules() -> dict[str, bool]:
    return {module: importlib.util.find_spec(module) is not None for module in PYTHON_BREP_MODULES}


def parse_assembly_manifest(
    path: Path, root: Path, accepted_native_sources: set[Path], *, draft_valid: bool = False
) -> tuple[dict[str, Any] | None, list[dict[str, str]]]:
    if not path.is_file():
        if draft_valid:
            return None, [{
                "code": "NATIVE_ASSEMBLY_MANIFEST_DRAFT_NOT_RELEASED",
                "severity": "blocker",
                "detail": "A valid component-addressable design-input draft exists, but the required released assembly manifest does not.",
            }]
        return None, [{"code": "NO_NATIVE_ASSEMBLY_MANIFEST", "severity": "blocker"}]
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return None, [{"code": "INVALID_NATIVE_ASSEMBLY_MANIFEST", "severity": "blocker", "detail": str(exc)}]
    if not isinstance(payload, dict):
        return None, [{"code": "INVALID_NATIVE_ASSEMBLY_MANIFEST", "severity": "blocker", "detail": "root must be an object"}]

    components = payload.get("components")
    if not isinstance(components, list) or not components:
        return payload, [{"code": "NATIVE_ASSEMBLY_COMPONENTS_UNPROVEN", "severity": "blocker"}]

    gaps: list[dict[str, str]] = []
    for index, component in enumerate(components):
        if not isinstance(component, dict) or not isinstance(component.get("native_source"), str):
            gaps.append({"code": "NATIVE_ASSEMBLY_COMPONENTS_UNPROVEN", "severity": "blocker", "detail": f"component {index} lacks native_source"})
            continue
        candidate = (root / component["native_source"]).resolve()
        if candidate not in accepted_native_sources:
            gaps.append({"code": "INVALID_NATIVE_COMPONENT_SOURCE", "severity": "blocker", "detail": component["native_source"]})
    return payload, gaps


def is_non_mesh_step(path: Path) -> bool:
    """Accept a normal ISO-10303 header, reject common mesh-conversion tokens.

    This is intentionally a conservative structural preflight, not a STEP
    geometry kernel.  A native exporter must still be exercised to create the
    receipt below.
    """

    try:
        header = path.read_bytes()[:262_144].decode("latin-1").upper()
    except OSError:
        return False
    mesh_tokens = ("FACETED_BREP", "TRIANGULATED_FACE_SET", "TESSELLATED")
    return "ISO-10303-21" in header and "FILE_SCHEMA" in header and not any(token in header for token in mesh_tokens)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1_048_576), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_export_receipt(
    path: Path, root: Path, accepted_native_sources: set[Path]
) -> tuple[dict[str, Any] | None, list[dict[str, str]], Path | None]:
    """Validate an exporter-produced receipt without running the exporter."""

    if not path.is_file():
        return None, [{"code": "NO_NATIVE_CAD_EXPORT_RECEIPT", "severity": "blocker"}], None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return None, [{"code": "INVALID_NATIVE_CAD_EXPORT_RECEIPT", "severity": "blocker", "detail": str(exc)}], None
    if not isinstance(payload, dict):
        return None, [{"code": "INVALID_NATIVE_CAD_EXPORT_RECEIPT", "severity": "blocker", "detail": "root must be an object"}], None

    gaps: list[dict[str, str]] = []
    output_step: Path | None = None
    if not isinstance(payload.get("exporter"), str) or not payload["exporter"].strip():
        gaps.append({"code": "INVALID_NATIVE_CAD_EXPORT_RECEIPT", "severity": "blocker", "detail": "exporter is required"})
    if not isinstance(payload.get("assembly_component_count"), int) or payload["assembly_component_count"] <= 0:
        gaps.append({"code": "ASSEMBLY_STRUCTURE_UNPROVEN", "severity": "blocker"})
    if not isinstance(payload.get("output_step"), str):
        gaps.append({"code": "INVALID_NATIVE_CAD_EXPORT_RECEIPT", "severity": "blocker", "detail": "output_step is required"})
    else:
        output_step = root / payload["output_step"]
        if output_step.suffix.lower() not in STEP_SUFFIXES or not output_step.is_file() or not is_non_mesh_step(output_step):
            gaps.append({"code": "INVALID_OR_MESH_DERIVED_STEP_ARTIFACT", "severity": "blocker", "detail": payload["output_step"]})

    source_hashes = payload.get("native_source_sha256")
    if not isinstance(source_hashes, dict) or not source_hashes:
        gaps.append({"code": "NATIVE_SOURCE_PROVENANCE_UNPROVEN", "severity": "blocker"})
    else:
        for source_text, expected_hash in source_hashes.items():
            if not isinstance(source_text, str) or not isinstance(expected_hash, str):
                gaps.append({"code": "NATIVE_SOURCE_PROVENANCE_UNPROVEN", "severity": "blocker"})
                continue
            source = (root / source_text).resolve()
            if source not in accepted_native_sources or sha256(source) != expected_hash.lower():
                gaps.append({"code": "INVALID_NATIVE_SOURCE_PROVENANCE", "severity": "blocker", "detail": source_text})
    return payload, gaps, output_step


def build_report(root: Path) -> dict[str, Any]:
    root = root.resolve()
    cad_root = root / CAD_ROOT
    vehicle_root = root / VEHICLE_DESCRIPTION_ROOT

    document_native_source_paths = file_paths(vehicle_root, NATIVE_SOURCE_SUFFIXES)
    python_brep_root = root / NATIVE_BREP_PYTHON_ROOT
    python_brep_candidates = sorted(python_brep_root.glob("*.py"), key=lambda path: path.as_posix().lower()) if python_brep_root.is_dir() else []
    rejected_python_brep_sources = {
        path: python_cadquery_source_rejection_codes(path, root) for path in python_brep_candidates
    }
    qualified_python_brep_sources = [
        path for path, rejection_codes in rejected_python_brep_sources.items() if not rejection_codes
    ]
    native_source_paths = [*document_native_source_paths, *qualified_python_brep_sources]
    accepted_native_sources = {path.resolve() for path in native_source_paths}
    step_paths = file_paths(vehicle_root, STEP_SUFFIXES)
    mesh_paths = file_paths(vehicle_root, MESH_SUFFIXES)
    scad_paths = sorted(cad_root.rglob("*.scad"), key=lambda path: path.as_posix().lower()) if cad_root.is_dir() else []
    python_sources = sorted(cad_root.rglob("*.py"), key=lambda path: path.as_posix().lower()) if cad_root.is_dir() else []
    mesh_generator_paths = [path for path in python_sources if source_has_mesh_generator_marker(path)]
    draft_state = component_addressable_assembly_draft_state(root)
    manifest, manifest_gaps = parse_assembly_manifest(
        root / ASSEMBLY_MANIFEST, root, accepted_native_sources, draft_valid=draft_state["valid"]
    )
    receipt, receipt_gaps, receipt_step_path = parse_export_receipt(root / EXPORT_RECEIPT, root, accepted_native_sources)

    executable_paths = windows_tool_paths()
    python_modules = python_brep_modules()
    exporter_available = bool(executable_paths["freecad_cmd"] or any(python_modules.values()))

    gaps: list[dict[str, str]] = [*manifest_gaps, *receipt_gaps]
    if not native_source_paths:
        gaps.append({
            "code": "NO_EDITABLE_NATIVE_BREP_SOURCE",
            "severity": "blocker",
            "required_evidence": "Add project-owned editable B-rep part and assembly sources (for example FCStd) for every exported component.",
        })
    valid_step_paths = [path for path in step_paths if is_non_mesh_step(path)]
    if not step_paths:
        gaps.append({
            "code": "NO_BREP_STEP_ARTIFACT",
            "severity": "blocker",
            "required_evidence": "Export a component-addressable ISO-10303 STEP assembly from the editable sources and retain its manifest.",
        })
    elif not valid_step_paths:
        gaps.append({
            "code": "INVALID_OR_MESH_DERIVED_STEP_ARTIFACT",
            "severity": "blocker",
            "required_evidence": "Provide a non-tessellated ISO-10303 STEP artifact exported from the native B-rep sources.",
        })
    if not exporter_available:
        gaps.append({
            "code": "NO_WINDOWS_BREP_EXPORTER",
            "severity": "blocker",
            "required_evidence": "Install and verify a Windows-native B-rep exporter such as FreeCADCmd, then rerun this audit without using WSL or a mesh converter.",
        })
    if mesh_generator_paths:
        gaps.append({
            "code": "LEGACY_STL_VISUAL_GENERATORS_PRESENT",
            "severity": "warning",
            "detail": "Legacy STL generators remain visual/simulation assets and cannot satisfy native CAD; they do not invalidate separately proven native B-rep reconstruction, assembly, STEP, receipt, and exporter evidence.",
        })

    native_sources_present = bool(native_source_paths)
    step_present = bool(valid_step_paths)
    manifest_valid = manifest is not None and not manifest_gaps
    receipt_valid = receipt is not None and not receipt_gaps and receipt_step_path in valid_step_paths
    outcome = "ready" if native_sources_present and step_present and manifest_valid and receipt_valid and exporter_available else "blocked"

    return {
        "schema_version": 1,
        "audit_name": "native_cad_readiness",
        "outcome": outcome,
        "native_editable_step_assembly_ready": outcome == "ready",
        "audit_scope": {
            "host": "Windows",
            "read_only": True,
            "prohibited_execution_backends": ["WSL", "Docker", "Gazebo"],
        },
        "native_cad_definition": {
            "required": [
                "editable project-owned B-rep part/assembly sources",
                "component-addressable assembly manifest",
                "ISO-10303 STEP artifact exported from those sources",
                "available Windows-native B-rep exporter",
                "export receipt binding source hashes to a non-mesh STEP assembly",
            ],
            "explicitly_not_accepted": [
                "STL, OBJ, DAE, glTF, PLY, or 3MF as native CAD",
                "mesh-to-STEP conversion as editable manufacturing CAD",
                "a monolithic STEP without component/source provenance",
                "URDF collision primitives or visual mesh references as manufacturing CAD",
            ],
        },
        "inventory": {
            "openscad_packaging_sources": relative_paths(scad_paths, root),
            "python_mesh_generators": relative_paths(mesh_generator_paths, root),
            "cadquery_parametric_brep_sources": relative_paths(qualified_python_brep_sources, root),
            "rejected_python_brep_candidates": [
                {"path": path.relative_to(root).as_posix(), "rejection_codes": rejection_codes}
                for path, rejection_codes in rejected_python_brep_sources.items()
                if rejection_codes
            ],
            "editable_native_brep_sources": relative_paths(native_source_paths, root),
            "step_artifacts": relative_paths(step_paths, root),
            "validated_non_mesh_step_artifacts": relative_paths(valid_step_paths, root),
            "mesh_asset_count": len(mesh_paths),
            "assembly_manifest": ASSEMBLY_MANIFEST.as_posix() if (root / ASSEMBLY_MANIFEST).is_file() else None,
            "component_addressable_assembly_draft": draft_state,
            "export_receipt": EXPORT_RECEIPT.as_posix() if (root / EXPORT_RECEIPT).is_file() else None,
        },
        "toolchain": {
            "windows_executables": {name: {"available": bool(path), "path": path} for name, path in executable_paths.items()},
            "python_brep_modules": python_modules,
            "windows_brep_exporter_available": exporter_available,
        },
        "gaps": gaps,
        "next_safe_action": "Create native B-rep sources and an assembly manifest first; then use a Windows-native exporter to generate and validate STEP. Do not convert the existing mesh assets to STEP.",
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=SCRIPT_ROOT, help="repository root to audit")
    parser.add_argument("--output", type=Path, help="write the JSON report to this path")
    parser.add_argument("--strict", action="store_true", help="return non-zero unless native CAD delivery is ready")
    args = parser.parse_args(argv)

    report = build_report(args.root)
    rendered = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return 0 if not args.strict or report["outcome"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
