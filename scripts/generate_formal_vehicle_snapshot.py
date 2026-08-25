#!/usr/bin/env python3
"""Generate and verify the committed formal-vehicle URDF snapshot.

Generation is intentionally a ROS/WSL operation because it invokes Xacro.  The
``--check`` path is pure Python: Windows fast CI can therefore prove that every
authoritative input and every committed output still matches the audited
manifest without installing ROS or Xacro.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
ENTRYPOINT = Path(
    "starter_ws/src/sanitation_vehicle_description/urdf/"
    "formal_competition_vehicle.urdf.xacro"
)
CONTROLLER_CONFIG = Path(
    "starter_ws/src/sanitation_vehicle_description/config/"
    "formal_vehicle_controllers.yaml"
)
ENGINEERING_ROOT = Path("reports/engineering")
URDF_OUTPUT = ENGINEERING_ROOT / "formal_competition_vehicle.urdf"
LAYOUT_OUTPUT = ENGINEERING_ROOT / "formal_vehicle_layout_report.json"
URDF_REPORT_OUTPUT = ENGINEERING_ROOT / "formal_vehicle_urdf_report.json"
PRODUCT_OUTPUT = ENGINEERING_ROOT / "formal_vehicle_product_design_report.json"
COMPONENT_OUTPUT = ENGINEERING_ROOT / "formal_vehicle_component_register_report.json"
MANIFEST_OUTPUT = ENGINEERING_ROOT / "formal_vehicle_snapshot_manifest.json"
OUTPUT_PATHS = (
    URDF_OUTPUT,
    LAYOUT_OUTPUT,
    URDF_REPORT_OUTPUT,
    PRODUCT_OUTPUT,
    COMPONENT_OUTPUT,
)
MANIFEST_KIND = "tzcup_formal_vehicle_snapshot_manifest"
SCHEMA_VERSION = 1
CANONICAL_CONTROLLER_URI = (
    "package://sanitation_vehicle_description/config/formal_vehicle_controllers.yaml"
)
CONTROLLER_PARAMETER_RE = re.compile(r"<parameters>([^<]+)</parameters>")


class SnapshotError(RuntimeError):
    """Raised when a snapshot cannot be generated or verified fail-closed."""


def _relative(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError as exc:
        raise SnapshotError(f"path escapes repository root: {path}") from exc


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_digest(inventory: dict[str, dict[str, Any]]) -> str:
    rendered = json.dumps(
        inventory, ensure_ascii=False, separators=(",", ":"), sort_keys=True
    ).encode("utf-8")
    return hashlib.sha256(rendered).hexdigest()


def _inventory(root: Path, paths: Iterable[Path]) -> dict[str, dict[str, Any]]:
    inventory: dict[str, dict[str, Any]] = {}
    for relative in sorted(set(paths), key=lambda item: item.as_posix()):
        path = root / relative
        if not path.is_file():
            raise SnapshotError(f"snapshot input/output is missing: {relative.as_posix()}")
        inventory[relative.as_posix()] = {
            "sha256": _sha256(path),
            "size_bytes": path.stat().st_size,
        }
    return inventory


def authoritative_source_paths(root: Path = ROOT) -> tuple[Path, ...]:
    """Return the complete audited input set for expansion and report generation."""

    urdf_root = root / ENTRYPOINT.parent
    high_fidelity_xacros = [
        path.relative_to(root)
        for path in (urdf_root / "high_fidelity").glob("*.xacro")
    ]
    mesh_assets = [
        path.relative_to(root)
        for path in (
            root / "starter_ws/src/sanitation_vehicle_description/meshes"
        ).rglob("*")
        if path.is_file() and path.suffix.lower() in {".dae", ".png", ".stl"}
    ]
    explicit = [
        ENTRYPOINT,
        CONTROLLER_CONFIG,
        Path("config/high_fidelity_vehicle/formal_vehicle_layout.yaml"),
        Path("config/high_fidelity_vehicle/formal_vehicle_component_register.yaml"),
        Path("config/high_fidelity_vehicle/pre_urdf_contract.yaml"),
        Path("config/high_fidelity_vehicle/capacity_budget.csv"),
        Path("config/high_fidelity_vehicle/mass_budget.csv"),
        Path("config/high_fidelity_vehicle/power_budget.csv"),
        Path("config/high_fidelity_vehicle/throughput_budget.csv"),
        Path("starter_ws/src/sanitation_vehicle_description/meshes/MANIFEST.sha256"),
        Path("starter_ws/src/sanitation_vehicle_description/meshes/README.md"),
        Path("starter_ws/src/sanitation_vehicle_description/meshes/vendor/SOURCES.yaml"),
        Path("scripts/generate_formal_vehicle_snapshot.py"),
        Path("scripts/validate_pre_urdf_readiness.py"),
        Path("scripts/validate_formal_vehicle_urdf.py"),
        Path("scripts/validate_formal_vehicle_visual_fidelity.py"),
        Path("scripts/validate_formal_vehicle_product_design.py"),
        Path("scripts/validate_formal_vehicle_component_register.py"),
        Path("scripts/formal_vehicle_mesh_manifest.py"),
    ]
    return tuple(
        sorted(
            set(explicit + high_fidelity_xacros + mesh_assets),
            key=lambda item: item.as_posix(),
        )
    )


def _json_bytes(payload: dict[str, Any]) -> bytes:
    return (
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")


def _validate_expanded_urdf_paths(raw: str, root: Path) -> str:
    controller_paths = CONTROLLER_PARAMETER_RE.findall(raw)
    if controller_paths != [CANONICAL_CONTROLLER_URI]:
        raise SnapshotError(
            "use_sim:=true expansion must contain exactly one canonical controller parameter URI"
        )
    try:
        document = ET.fromstring(raw)
    except ET.ParseError as exc:
        raise SnapshotError(f"expanded URDF is not valid XML: {exc}") from exc

    def is_absolute_filesystem_reference(value: str) -> bool:
        candidate = value.strip()
        return bool(
            candidate.startswith("/")
            or candidate.startswith("\\\\")
            or re.match(r"^[A-Za-z]:[\\/]", candidate)
            or candidate.lower().startswith("file:")
        )

    for element in document.iter():
        local_tag = element.tag.rsplit("}", 1)[-1]
        for attribute, value in element.attrib.items():
            if is_absolute_filesystem_reference(value):
                raise SnapshotError(
                    "expanded URDF contains an absolute filesystem reference "
                    f"in {local_tag}@{attribute}"
                )
        text = (element.text or "").strip()
        topic_like = local_tag == "topic" or local_tag.endswith("_topic")
        if text and not topic_like and is_absolute_filesystem_reference(text):
            raise SnapshotError(
                "expanded URDF contains an absolute filesystem reference "
                f"in <{local_tag}>"
            )
    return raw


def _write_stage(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)


def _tool_version(executable: str) -> str:
    try:
        return version("xacro")
    except PackageNotFoundError:
        pass
    result = subprocess.run(
        [executable, "--version"],
        check=False,
        capture_output=True,
        text=True,
    )
    rendered = (result.stdout or result.stderr).strip()
    return rendered if result.returncode == 0 and rendered else "unreported"


def _build_reports(staged_root: Path, staged_urdf: Path) -> None:
    # Imports are delayed so ``--check`` remains a small ROS-independent path.
    from validate_formal_vehicle_component_register import validate as validate_component
    from validate_formal_vehicle_product_design import validate_product_design
    from validate_formal_vehicle_urdf import validate_expanded_urdf, validate_layout
    from validate_formal_vehicle_visual_fidelity import validate_visual_fidelity

    layout = validate_layout()
    urdf = validate_expanded_urdf(staged_urdf)
    urdf["urdf_validation"]["path"] = URDF_OUTPUT.as_posix()
    product = validate_product_design(staged_urdf)
    component = validate_component(urdf_path=staged_urdf)
    validate_visual_fidelity(staged_urdf)
    report_payloads = {
        LAYOUT_OUTPUT: layout,
        URDF_REPORT_OUTPUT: urdf,
        PRODUCT_OUTPUT: product,
        COMPONENT_OUTPUT: component,
    }
    for relative, payload in report_payloads.items():
        _write_stage(staged_root / relative, _json_bytes(payload))


def _manifest_payload(
    source_inventory: dict[str, dict[str, Any]],
    output_inventory: dict[str, dict[str, Any]],
    xacro_version: str,
) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "kind": MANIFEST_KIND,
        "profile": {
            "entrypoint": ENTRYPOINT.as_posix(),
            "use_sim": True,
            "xacro_version": xacro_version,
            "argument_overrides": [
                {
                    "name": "controller_config_path",
                    "value": CANONICAL_CONTROLLER_URI,
                    "reason": "prevent ament-prefix machine dependence in committed snapshot",
                }
            ],
        },
        "source_inventory": source_inventory,
        "source_inventory_sha256": _canonical_digest(source_inventory),
        "outputs": output_inventory,
        "output_inventory_sha256": _canonical_digest(output_inventory),
    }


def verify_snapshot(root: Path = ROOT, manifest_path: Path | None = None) -> dict[str, Any]:
    manifest_file = manifest_path or (root / MANIFEST_OUTPUT)
    if not manifest_file.is_file():
        raise SnapshotError(f"snapshot manifest is missing: {_relative(manifest_file, root)}")
    manifest = json.loads(manifest_file.read_text(encoding="utf-8"))
    if manifest.get("schema_version") != SCHEMA_VERSION or manifest.get("kind") != MANIFEST_KIND:
        raise SnapshotError("unsupported formal-vehicle snapshot manifest")
    profile = manifest.get("profile", {})
    if profile.get("entrypoint") != ENTRYPOINT.as_posix() or profile.get("use_sim") is not True:
        raise SnapshotError("snapshot profile must use the formal entrypoint with use_sim:=true")
    overrides = profile.get("argument_overrides", [])
    if not any(
        item.get("name") == "controller_config_path"
        and item.get("value") == CANONICAL_CONTROLLER_URI
        for item in overrides
    ):
        raise SnapshotError("snapshot manifest lacks the canonical controller-path override")

    expected_sources = _inventory(root, authoritative_source_paths(root))
    if manifest.get("source_inventory") != expected_sources:
        raise SnapshotError("authoritative source inventory differs from committed manifest")
    if manifest.get("source_inventory_sha256") != _canonical_digest(expected_sources):
        raise SnapshotError("source inventory aggregate digest is invalid")

    expected_outputs = _inventory(root, OUTPUT_PATHS)
    if manifest.get("outputs") != expected_outputs:
        raise SnapshotError("formal snapshot outputs differ from committed manifest")
    if manifest.get("output_inventory_sha256") != _canonical_digest(expected_outputs):
        raise SnapshotError("output inventory aggregate digest is invalid")
    return manifest


def generate_snapshot(root: Path = ROOT) -> dict[str, Any]:
    xacro = shutil.which("xacro")
    if not xacro:
        raise SnapshotError(
            "xacro is unavailable; run generation from a sourced ROS/WSL environment"
        )
    source_paths = authoritative_source_paths(root)
    before = _inventory(root, source_paths)
    staging_parent = root / ENGINEERING_ROOT
    staging_parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=".formal_vehicle_snapshot_", dir=staging_parent
    ) as directory:
        staged_root = Path(directory)
        staged_urdf = staged_root / URDF_OUTPUT
        staged_urdf.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            [
                xacro,
                ENTRYPOINT.as_posix(),
                "use_sim:=true",
                f"controller_config_path:={CANONICAL_CONTROLLER_URI}",
                "-o",
                str(staged_urdf),
            ],
            cwd=root,
            check=True,
        )
        normalized = _validate_expanded_urdf_paths(
            staged_urdf.read_text(encoding="utf-8"), root
        )
        staged_urdf.write_text(normalized, encoding="utf-8", newline="\n")
        _build_reports(staged_root, staged_urdf)

        after = _inventory(root, source_paths)
        if before != after:
            raise SnapshotError("authoritative sources changed while snapshot was generated")
        outputs = _inventory(staged_root, OUTPUT_PATHS)
        manifest = _manifest_payload(before, outputs, _tool_version(xacro))
        _write_stage(staged_root / MANIFEST_OUTPUT, _json_bytes(manifest))

        # Every staged payload has passed before replacement begins.  os.replace
        # is atomic for each file and avoids exposing partially-written JSON/XML.
        for relative in (*OUTPUT_PATHS, MANIFEST_OUTPUT):
            target = root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            os.replace(staged_root / relative, target)
    verify_snapshot(root)
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--check",
        action="store_true",
        help="verify committed source/output hashes without invoking Xacro",
    )
    args = parser.parse_args()
    result = verify_snapshot() if args.check else generate_snapshot()
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
