#!/usr/bin/env python3
"""Export and re-import one tiny parametric B-rep STEP test piece.

This is intentionally a toolchain smoke test, not a vehicle CAD export, a
component manifest, or evidence for any formal readiness gate.  All generated
files must stay under the repository-local ``.work`` directory.
"""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import math
from pathlib import Path
from typing import Any


MESH_STEP_MARKERS = ("FACETED_BREP", "TRIANGULATED_FACE_SET", "TESSELLATED")
BREP_STEP_MARKERS = ("MANIFOLD_SOLID_BREP", "ADVANCED_BREP_SHAPE_REPRESENTATION")


def topology_counts(shape: Any) -> dict[str, int]:
    return {
        "solids": len(shape.Solids()),
        "faces": len(shape.Faces()),
        "edges": len(shape.Edges()),
        "vertices": len(shape.Vertices()),
    }


def require_project_local_work_directory(repository_root: Path, output_dir: Path) -> Path:
    work_root = (repository_root.resolve() / ".work").resolve()
    resolved_output = output_dir.resolve()
    if resolved_output != work_root and work_root not in resolved_output.parents:
        raise ValueError(f"output directory must stay below {work_root}: {resolved_output}")
    return resolved_output


def step_header_evidence(step_path: Path) -> dict[str, bool]:
    header = step_path.read_bytes()[:262_144].decode("latin-1").upper()
    return {
        "iso_10303_header": "ISO-10303-21" in header and "FILE_SCHEMA" in header,
        "brep_entity_present": any(marker in header for marker in BREP_STEP_MARKERS),
        "mesh_entity_absent": not any(marker in header for marker in MESH_STEP_MARKERS),
    }


def build_report(repository_root: Path, output_dir: Path) -> dict[str, Any]:
    """Run the smallest useful CadQuery -> STEP -> CadQuery topology check."""

    import cadquery as cq

    output_dir.mkdir(parents=True, exist_ok=True)
    step_path = output_dir / "cadquery_brep_roundtrip.step"
    source = cq.Workplane("XY").box(20.0, 10.0, 5.0).faces(">Z").workplane().hole(4.0).val()
    if not source.isValid():
        raise RuntimeError("CadQuery source B-rep is invalid")
    cq.exporters.export(source, str(step_path))
    imported_values = cq.importers.importStep(str(step_path)).vals()
    if len(imported_values) != 1:
        raise RuntimeError(f"expected exactly one imported solid, observed {len(imported_values)}")
    imported = imported_values[0]
    if not imported.isValid():
        raise RuntimeError("re-imported STEP B-rep is invalid")

    source_topology = topology_counts(source)
    imported_topology = topology_counts(imported)
    source_volume = source.Volume()
    imported_volume = imported.Volume()
    topology_matches = source_topology == imported_topology
    volume_matches = math.isclose(source_volume, imported_volume, rel_tol=1e-8, abs_tol=1e-6)
    header = step_header_evidence(step_path)
    passed = topology_matches and volume_matches and all(header.values())

    return {
        "schema_version": 1,
        "test_name": "cadquery_minimal_brep_step_roundtrip",
        "outcome": "passed" if passed else "failed",
        "scope": {
            "test_piece": "20 x 10 x 5 mm box with one 4 mm through hole",
            "not_vehicle_evidence": True,
            "does_not_change_formal_vehicle_readiness": True,
            "prohibited_execution_backends": ["WSL", "Docker", "Gazebo"],
        },
        "toolchain": {
            "cadquery": importlib.metadata.version("cadquery"),
            "cadquery_ocp": importlib.metadata.version("cadquery-ocp"),
            "occt_version_interpretation": "cadquery-ocp distribution version is recorded; this smoke test does not claim a separately measured OCCT library version.",
        },
        "artifacts": {"step": str(step_path)},
        "source": {"valid": True, "topology": source_topology, "volume_mm3": source_volume},
        "reimported": {"valid": True, "topology": imported_topology, "volume_mm3": imported_volume},
        "checks": {
            "topology_matches": topology_matches,
            "volume_matches": volume_matches,
            **header,
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)

    repository_root = args.repo_root.resolve()
    output_dir = require_project_local_work_directory(repository_root, args.output_dir)
    report_path = require_project_local_work_directory(repository_root, args.report.parent) / args.report.name
    try:
        report = build_report(repository_root, output_dir)
    except Exception as exc:  # retain a compact, actionable local failure receipt
        report = {
            "schema_version": 1,
            "test_name": "cadquery_minimal_brep_step_roundtrip",
            "outcome": "failed",
            "error": {"type": type(exc).__name__, "message": str(exc)},
            "scope": {
                "not_vehicle_evidence": True,
                "does_not_change_formal_vehicle_readiness": True,
            },
        }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if report["outcome"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
