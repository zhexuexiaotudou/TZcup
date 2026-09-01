#!/usr/bin/env python3
"""Bind formal water runtime evidence to source and a frozen ROS overlay."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path


CRITICAL_FILES = (
    "scripts/run_formal_water_recovery_runtime.sh",
    "scripts/run_formal_water_safety_soak.sh",
    "scripts/run_formal_runtime_isolation.sh",
    "scripts/check_formal_water_preoperational_readiness.py",
    "scripts/audit_formal_water_launch_log.py",
    "scripts/collect_formal_water_safety_preflight.py",
    "scripts/validate_formal_water_recovery_runtime.py",
    "scripts/collect_formal_cleaning_actuator_motor_runtime.py",
    "scripts/collect_formal_typed_cleaning_motor_diagnostic.py",
    "scripts/formal_water_motor_metrics.py",
    "scripts/formal_cleaning_motor_telemetry.py",
    "scripts/validate_formal_side_brush_sdf_surface.py",
    "scripts/finalize_formal_water_recovery_acceptance.py",
    "scripts/generate_formal_water_critical_source_manifest.py",
    "scripts/run_formal_typed_cleaning_motor_diagnostic.sh",
    "config/high_fidelity_vehicle/cleaning_actuator_motor_realism_contract.yaml",
    "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml",
    "patches/upstream/gz_transport13/0001-publish-retry-current-frame-on-eintr.patch",
    "patches/upstream/gz_transport13/manifest.json",
    "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py",
    "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/cleaning_mechanism.xacro",
    "starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro",
    "starter_ws/src/sanitation_gazebo_control/include/sanitation_gazebo_control/CleaningActuatorMotorCore.hh",
    "starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorMotorCore.cc",
    "starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorMotorSystem.cc",
    "starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorVectorBridge.cc",
    "starter_ws/src/sanitation_gazebo_control/src/WaterRecoverySystem.cc",
    "starter_ws/src/sanitation_safety/sanitation_safety/whole_vehicle_safety_manager.py",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def symbolic_links(root: Path, label: str) -> list[str]:
    if root.is_symlink() or not root.is_dir():
        raise ValueError(f"{label} must be a real directory: {root}")
    links: list[str] = []
    stack = [root]
    while stack:
        directory = stack.pop()
        for entry in os.scandir(directory):
            path = Path(entry.path)
            if entry.is_symlink():
                links.append(path.relative_to(root).as_posix())
            elif entry.is_dir(follow_symlinks=False):
                stack.append(path)
            elif not entry.is_file(follow_symlinks=False):
                raise ValueError(f"{label} contains a non-regular entry: {path}")
    return sorted(links)


def generate(repo: Path, workspace: Path) -> dict[str, object]:
    frozen_source = workspace / "src"
    frozen_source_links = symbolic_links(frozen_source, "frozen runtime source tree")
    if frozen_source_links:
        raise ValueError(
            "frozen runtime source tree contains symbolic links: "
            + ", ".join(frozen_source_links)
        )
    rows = []
    for relative in CRITICAL_FILES:
        source = repo / relative
        if source.is_symlink() or not source.is_file():
            raise ValueError(f"critical source must be a regular file: {source}")
        row: dict[str, object] = {
            "path": relative,
            "source_sha256": sha256(source),
        }
        if relative.startswith("starter_ws/"):
            frozen = workspace / relative.removeprefix("starter_ws/")
            if frozen.is_symlink() or not frozen.is_file():
                raise ValueError(f"frozen critical source must be a regular file: {frozen}")
            row.update(
                frozen_copy_path=str(frozen),
                frozen_copy_sha256=sha256(frozen),
                source_matches_frozen_copy=sha256(source) == sha256(frozen),
            )
        rows.append(row)
    symlink_report = workspace / "INSTALL_SYMLINKS.txt"
    if symlink_report.is_symlink() or not symlink_report.is_file():
        raise ValueError(f"install symlink report must be a regular file: {symlink_report}")
    reported_links = [
        line.strip()
        for line in symlink_report.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    actual_links = symbolic_links(workspace / "install", "merged runtime install")
    if reported_links != actual_links:
        raise ValueError(
            "INSTALL_SYMLINKS.txt does not match the merged install tree: "
            f"reported={reported_links}, actual={actual_links}"
        )
    return {
        "schema_version": 1,
        "workspace": str(workspace),
        "critical_files": rows,
        "frozen_source_root": str(frozen_source),
        "frozen_source_symlink_count": len(frozen_source_links),
        "source_package_files_match_frozen_copy": all(
            row.get("source_matches_frozen_copy", True) for row in rows
        ),
        "install_symlink_report": str(symlink_report),
        "install_symlink_report_sha256": sha256(symlink_report),
        "install_symlink_report_matches_scan": True,
        "install_symlink_count": len(actual_links),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = generate(args.repo.resolve(), args.workspace.resolve())
    if args.output.exists() or args.output.is_symlink():
        parser.error(f"refusing stale output: {args.output}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    pending = args.output.with_suffix(args.output.suffix + f".pending.{os.getpid()}")
    if pending.exists() or pending.is_symlink():
        parser.error(f"refusing stale pending output: {pending}")
    pending.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    pending.replace(args.output)
    passed = bool(report["source_package_files_match_frozen_copy"]) and not int(
        report["install_symlink_count"]
    )
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
