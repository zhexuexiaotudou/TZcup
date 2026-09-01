#!/usr/bin/env python3
"""Create and verify source-bound evidence for the integrated Gazebo acceptance.

This module deliberately has no ROS dependency.  It records a post-build source
and install snapshot, records the four isolated scenario invocations, and then
fails closed unless every result was produced inside its recorded invocation and
matches the expected machine-readable contract.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable


SCENARIOS = ("mobility", "water_normal", "water_full", "manipulation")
MATERIAL_MASS_KG = {
    "paperboard": 0.0189,
    "PP": 0.0243,
    "PET": 0.03726,
    "aluminum": 0.0729,
}
SOURCE_ROOTS = (
    "scripts/run_formal_vehicle_mobility_runtime.sh",
    "scripts/validate_formal_vehicle_mobility_runtime.py",
    "scripts/formal_vehicle_mobility_metrics.py",
    "scripts/validate_formal_water_recovery_runtime.py",
    "scripts/run_formal_cube_pick_place_runtime.sh",
    "scripts/validate_formal_cube_pick_place_runtime.py",
    "starter_ws/src/sanitation_gazebo_control",
    "starter_ws/src/sanitation_vehicle_description",
    "starter_ws/src/sanitation_manipulation",
)
INSTALL_PACKAGES = (
    "sanitation_gazebo_control",
    "sanitation_vehicle_description",
    "sanitation_manipulation",
)
INSTALL_SYMLINK_REPORT = Path("INSTALL_SYMLINKS.txt")
PACKAGE_SOURCE_DIRS = {
    "sanitation_vehicle_description": Path(
        "starter_ws/src/sanitation_vehicle_description"
    ),
    "sanitation_manipulation": Path("starter_ws/src/sanitation_manipulation"),
}
STRUCTURED_INSTALL_DIRS = {
    "launch": ("*.py", "python_ast"),
    "urdf": ("*.xacro", "xml"),
    "worlds": ("*.sdf", "xml"),
}
PLUGIN_BASENAMES = (
    "libDynamicPayloadSystem.so",
    "libDryBinMonitorSystem.so",
    "libWaterRecoverySystem.so",
)
CRITICAL_SUFFIXES = (
    "src/DynamicPayloadSystem.cc",
    "src/DryBinMonitorSystem.cc",
    "src/WaterRecoverySystem.cc",
    "launch/formal_vehicle_sim.launch.py",
    "launch/formal_cube_pick_place.launch.py",
    "worlds/formal_vehicle_validation.sdf",
    "worlds/formal_cube_manipulation.sdf",
    "urdf/formal_competition_vehicle.urdf.xacro",
    "urdf/formal_manipulation_acceptance.urdf.xacro",
    "urdf/material_cube.urdf.xacro",
    "urdf/high_fidelity/cleaning_mechanism.xacro",
    "urdf/high_fidelity/storage_system.xacro",
    "lib/libDynamicPayloadSystem.so",
    "lib/libDryBinMonitorSystem.so",
    "lib/libWaterRecoverySystem.so",
)
IGNORED_PARTS = {"__pycache__", ".pytest_cache", ".git"}


class AcceptanceError(RuntimeError):
    """A fail-closed evidence contract violation."""


def utc_iso(epoch_ns: int | None = None) -> str:
    stamp = time.time_ns() if epoch_ns is None else epoch_ns
    return datetime.fromtimestamp(stamp / 1e9, tz=timezone.utc).isoformat()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _advise_drop_cache(stream: Any) -> None:
    """Release clean file-cache pages after source/install inventory hashing."""

    fadvise = getattr(os, "posix_fadvise", None)
    dontneed = getattr(os, "POSIX_FADV_DONTNEED", None)
    if fadvise is None or dontneed is None:
        return
    try:
        fadvise(stream.fileno(), 0, 0, dontneed)
    except (OSError, ValueError):
        return


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
        # Only a fully consumed file is eligible for an advisory cache drop.
        _advise_drop_cache(stream)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise AcceptanceError(f"cannot read JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise AcceptanceError(f"JSON root must be an object: {path}")
    return value


def run_text(command: list[str], cwd: Path) -> str:
    try:
        result = subprocess.run(
            command, cwd=cwd, check=True, capture_output=True, text=True, timeout=30
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise AcceptanceError(f"command failed ({' '.join(command)}): {exc}") from exc
    return result.stdout.strip()


def optional_text(command: list[str], cwd: Path) -> str | None:
    try:
        return run_text(command, cwd)
    except AcceptanceError:
        return None


def git_command(repo_root: Path, *args: str) -> list[str]:
    """Build an exact-root Git command that also works in a Windows worktree from WSL.

    Git worktree administrative files created by Windows contain a ``F:/...``
    absolute gitdir.  Linux Git otherwise treats that as relative to the WSL
    checkout and reports a misleading "not a git repository" error.  Resolve
    only that declared gitdir and always pin the matching work tree; never add a
    global safe.directory exception or search a broader parent directory.
    """

    root = repo_root.resolve()
    command = ["git", "-c", f"safe.directory={root}"]
    dot_git = root / ".git"
    if dot_git.is_file():
        try:
            marker = dot_git.read_text(encoding="utf-8").strip()
        except (OSError, UnicodeError) as exc:
            raise AcceptanceError(f"cannot read Git worktree marker {dot_git}: {exc}") from exc
        if not marker.lower().startswith("gitdir:"):
            raise AcceptanceError(f"invalid Git worktree marker: {dot_git}")
        raw_git_dir = marker.split(":", 1)[1].strip()
        windows_absolute = re.fullmatch(r"([A-Za-z]):[\\/](.*)", raw_git_dir)
        if os.name != "nt" and windows_absolute:
            drive, tail = windows_absolute.groups()
            git_dir_text = f"/mnt/{drive.lower()}/{tail.replace(chr(92), '/')}"
        else:
            git_dir = Path(raw_git_dir)
            if not git_dir.is_absolute():
                git_dir = (root / git_dir).resolve()
            git_dir_text = str(git_dir)
        if not os.path.isdir(git_dir_text):
            raise AcceptanceError(f"declared Git worktree directory is missing: {git_dir_text}")
        command.extend((f"--git-dir={git_dir_text}", f"--work-tree={root}"))
    command.extend(args)
    return command


def iter_files(root: Path, entries: Iterable[str]) -> Iterable[tuple[str, Path]]:
    for entry in entries:
        candidate = root / entry
        if not candidate.exists():
            raise AcceptanceError(f"required source or artifact is missing: {candidate}")
        paths = [candidate] if candidate.is_file() else sorted(candidate.rglob("*"))
        for path in paths:
            if any(part in IGNORED_PARTS for part in path.parts):
                continue
            if path.is_file():
                yield path.relative_to(root).as_posix(), path


def inventory(root: Path, entries: Iterable[str]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for relative, path in iter_files(root, entries):
        stat = path.stat()
        result[relative] = {
            "sha256": sha256_file(path),
            "size_bytes": stat.st_size,
            "mtime_epoch_ns": stat.st_mtime_ns,
        }
    if not result:
        raise AcceptanceError(f"empty inventory under {root}")
    return result


def inventory_digest(value: dict[str, dict[str, Any]]) -> str:
    normalized = {
        name: {"sha256": row["sha256"], "size_bytes": row["size_bytes"]}
        for name, row in value.items()
    }
    return sha256_bytes(json.dumps(normalized, sort_keys=True).encode("utf-8"))


def regular_tree_inventory(
    root: Path, base: Path, label: str
) -> dict[str, dict[str, Any]]:
    """Inventory one real tree while rejecting links and special entries."""

    if root.is_symlink() or not root.is_dir():
        raise AcceptanceError(f"{label} must be a real directory: {root}")
    files: list[Path] = []
    stack = [root]
    while stack:
        directory = stack.pop()
        for entry in sorted(os.scandir(directory), key=lambda row: row.name):
            path = Path(entry.path)
            if entry.is_symlink():
                raise AcceptanceError(f"{label} contains a symbolic link: {path}")
            if entry.is_dir(follow_symlinks=False):
                if entry.name not in IGNORED_PARTS:
                    stack.append(path)
            elif entry.is_file(follow_symlinks=False):
                files.append(path)
            else:
                raise AcceptanceError(f"{label} contains a non-regular entry: {path}")
    rows: dict[str, dict[str, Any]] = {}
    for path in sorted(files):
        stat = path.stat()
        rows[path.relative_to(base).as_posix()] = {
            "sha256": sha256_file(path),
            "size_bytes": stat.st_size,
            "mtime_epoch_ns": stat.st_mtime_ns,
        }
    if not rows:
        raise AcceptanceError(f"empty {label}: {root}")
    return rows


def frozen_source_contract(
    repo_root: Path, runtime_ws: Path
) -> dict[str, Any]:
    repository_source_root = repo_root / "starter_ws/src"
    frozen_source_root = runtime_ws / "src"
    repository_rows: dict[str, dict[str, Any]] = {}
    frozen_rows: dict[str, dict[str, Any]] = {}
    for package in INSTALL_PACKAGES:
        repository_rows.update(
            regular_tree_inventory(
                repository_source_root / package,
                repository_source_root,
                f"repository source package {package}",
            )
        )
        frozen_rows.update(
            regular_tree_inventory(
                frozen_source_root / package,
                frozen_source_root,
                f"frozen source package {package}",
            )
        )
    repository_digest = inventory_digest(repository_rows)
    frozen_digest = inventory_digest(frozen_rows)
    if frozen_digest != repository_digest:
        raise AcceptanceError(
            "frozen runtime src differs from repository starter_ws/src"
        )
    return {
        "root": str(frozen_source_root.resolve()),
        "inventory": frozen_rows,
        "inventory_sha256": frozen_digest,
        "repository_inventory_sha256": repository_digest,
        "matches_repository": True,
        "symbolic_link_count": 0,
    }


def install_symlink_report_contract(
    runtime_ws: Path,
) -> dict[str, Any]:
    install_root = runtime_ws / "install"
    # This live scan fails on any symbolic link before the report is trusted.
    regular_tree_inventory(install_root, install_root, "merged runtime install")
    report = runtime_ws / INSTALL_SYMLINK_REPORT
    if report.is_symlink() or not report.is_file():
        raise AcceptanceError(f"install symlink report must be a regular file: {report}")
    if report.read_bytes() != b"":
        raise AcceptanceError("formal install symbolic-link report is not empty")
    return {
        "path": INSTALL_SYMLINK_REPORT.as_posix(),
        "sha256": sha256_file(report),
        "size_bytes": 0,
        "live_symbolic_link_count": 0,
        "report_matches_live_scan": True,
    }


def json_digest(value: Any) -> str:
    return sha256_bytes(json.dumps(value, sort_keys=True).encode("utf-8"))


def parse_contract_file(path: Path, parser_kind: str) -> None:
    try:
        text = path.read_text(encoding="utf-8")
        if parser_kind == "python_ast":
            ast.parse(text, filename=str(path))
        elif parser_kind == "xml":
            ET.fromstring(text)
        else:
            raise AcceptanceError(f"unknown contract parser {parser_kind}: {path}")
    except (OSError, UnicodeError, SyntaxError, ET.ParseError) as exc:
        raise AcceptanceError(
            f"cannot parse {parser_kind} source/install contract file {path}: {exc}"
        ) from exc


def install_prefix(runtime_ws: Path, package: str) -> Path:
    """Return the package prefix for either merged or isolated colcon installs."""

    layout = runtime_ws / "install/.colcon_install_layout"
    try:
        merged = layout.read_text(encoding="utf-8").strip() == "merged"
    except (OSError, UnicodeError):
        merged = False
    return runtime_ws / "install" if merged else runtime_ws / "install" / package


def installed_vehicle_xacro(runtime_ws: Path) -> Path:
    """Resolve the frozen vehicle Xacro through the declared colcon layout."""

    return (
        install_prefix(runtime_ws, "sanitation_vehicle_description")
        / "share/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro"
    )


def python_install_package_root(runtime_ws: Path, package: str) -> Path:
    roots: list[Path] = []
    for site in sorted(install_prefix(runtime_ws, package).joinpath("lib").glob("python*/site-packages")):
        direct = site / package
        if direct.is_dir():
            roots.append(direct)
        for egg_link in sorted(site.glob("*.egg-link")):
            try:
                lines = egg_link.read_text(encoding="utf-8").splitlines()
            except (OSError, UnicodeError) as exc:
                raise AcceptanceError(f"cannot read Python egg-link {egg_link}: {exc}") from exc
            if not lines or not lines[0].strip():
                raise AcceptanceError(f"empty Python egg-link: {egg_link}")
            target = Path(lines[0].strip())
            if not target.is_absolute():
                target = (egg_link.parent / target).resolve()
            candidate = target / package
            if candidate.is_dir():
                roots.append(candidate)
    unique = {str(path.resolve()): path for path in roots}
    if len(unique) != 1:
        raise AcceptanceError(
            f"{package} must resolve to exactly one installed Python package, found {len(unique)}"
        )
    return next(iter(unique.values()))


def source_install_contract(repo_root: Path, runtime_ws: Path) -> dict[str, dict[str, Any]]:
    """Hash and parse every installed launch/Xacro/world/Python package file."""

    rows: dict[str, dict[str, Any]] = {}
    category_counts: Counter[str] = Counter()
    for package, relative_source in PACKAGE_SOURCE_DIRS.items():
        source_package = repo_root / relative_source
        install_share = install_prefix(runtime_ws, package) / "share" / package
        for subdir, (pattern, parser_kind) in STRUCTURED_INSTALL_DIRS.items():
            source_dir = source_package / subdir
            for source in sorted(source_dir.rglob(pattern)):
                if not source.is_file():
                    continue
                relative = source.relative_to(source_package)
                installed = install_share / relative
                if not installed.is_file():
                    raise AcceptanceError(
                        f"installed {subdir} file is missing for {source}: {installed}"
                    )
                parse_contract_file(source, parser_kind)
                parse_contract_file(installed, parser_kind)
                source_hash = sha256_file(source)
                installed_hash = sha256_file(installed)
                if source_hash != installed_hash:
                    raise AcceptanceError(
                        f"installed {subdir} file is stale for {source}: {installed}"
                    )
                key = f"{package}:{relative.as_posix()}"
                rows[key] = {
                    "category": subdir,
                    "parser": parser_kind,
                    "source": source.relative_to(repo_root).as_posix(),
                    "installed": installed.relative_to(runtime_ws).as_posix(),
                    "sha256": source_hash,
                }
                category_counts[subdir] += 1

        if package == "sanitation_manipulation":
            source_modules = source_package / package
            installed_modules = python_install_package_root(runtime_ws, package)
            for source in sorted(source_modules.rglob("*.py")):
                if not source.is_file() or any(part in IGNORED_PARTS for part in source.parts):
                    continue
                relative_module = source.relative_to(source_modules)
                installed = installed_modules / relative_module
                if not installed.is_file():
                    raise AcceptanceError(
                        f"installed Python module is missing for {source}: {installed}"
                    )
                parse_contract_file(source, "python_ast")
                parse_contract_file(installed, "python_ast")
                source_hash = sha256_file(source)
                installed_hash = sha256_file(installed)
                if source_hash != installed_hash:
                    raise AcceptanceError(
                        f"installed Python module is stale for {source}: {installed}"
                    )
                key = f"{package}:python/{relative_module.as_posix()}"
                rows[key] = {
                    "category": "python_module",
                    "parser": "python_ast",
                    "source": source.relative_to(repo_root).as_posix(),
                    "installed": str(installed),
                    "sha256": source_hash,
                }
                category_counts["python_module"] += 1

    required_categories = {"launch", "urdf", "worlds", "python_module"}
    missing = sorted(required_categories - set(category_counts))
    if missing:
        raise AcceptanceError(f"source/install contract categories are empty: {missing}")
    return rows


def package_build_markers(runtime_ws: Path, started_ns: int) -> dict[str, dict[str, Any]]:
    rows: dict[str, dict[str, Any]] = {}
    for package in INSTALL_PACKAGES:
        marker = runtime_ws / "build" / package / "colcon_build.rc"
        if not marker.is_file():
            raise AcceptanceError(f"fresh build marker is missing for {package}: {marker}")
        stat = marker.stat()
        if stat.st_mtime_ns <= started_ns:
            raise AcceptanceError(
                f"{package} colcon build marker does not postdate the declared build start"
            )
        rows[package] = {
            "path": marker.relative_to(runtime_ws).as_posix(),
            "sha256": sha256_file(marker),
            "mtime_epoch_ns": stat.st_mtime_ns,
        }
    return rows


def plugin_rows(installed: dict[str, dict[str, Any]], started_ns: int) -> dict[str, dict[str, Any]]:
    matches: dict[str, list[tuple[str, dict[str, Any]]]] = {
        basename: [] for basename in PLUGIN_BASENAMES
    }
    for name, row in installed.items():
        basename = Path(name).name
        if basename in matches:
            matches[basename].append((name, row))
    invalid = {name: len(rows) for name, rows in matches.items() if len(rows) != 1}
    if invalid:
        raise AcceptanceError(
            f"compiled Gazebo plugins must appear exactly once each: {invalid}"
        )
    result: dict[str, dict[str, Any]] = {}
    for basename, entries in matches.items():
        name, row = entries[0]
        if int(row["mtime_epoch_ns"]) <= started_ns:
            raise AcceptanceError(f"compiled Gazebo plugin predates fresh build: {basename}")
        result[basename] = {"path": name, **row}
    return result


def require_runtime_versions(runtime: Any) -> None:
    if not isinstance(runtime, dict):
        raise AcceptanceError("runtime version record is missing")
    missing = [
        field
        for field in ("ros_distro", "ros_base_package", "gazebo")
        if not isinstance(runtime.get(field), str) or not runtime[field].strip()
    ]
    if missing:
        raise AcceptanceError(f"runtime ROS/Gazebo versions are missing: {missing}")


def git_snapshot(repo_root: Path) -> dict[str, Any]:
    head = run_text(git_command(repo_root, "rev-parse", "HEAD"), repo_root)
    tree = run_text(git_command(repo_root, "rev-parse", "HEAD^{tree}"), repo_root)
    tracked_diff = subprocess.run(
        git_command(repo_root, "diff", "--binary", "HEAD", "--", "."),
        cwd=repo_root,
        check=True,
        capture_output=True,
    ).stdout
    untracked_names = run_text(
        git_command(repo_root, "ls-files", "--others", "--exclude-standard"), repo_root
    ).splitlines()
    digest = hashlib.sha256()
    digest.update(b"tracked-diff\0")
    digest.update(tracked_diff)
    untracked: list[dict[str, Any]] = []
    for relative in sorted(name for name in untracked_names if name):
        path = repo_root / relative
        if not path.is_file():
            continue
        file_hash = sha256_file(path)
        untracked.append({"path": relative.replace("\\", "/"), "sha256": file_hash})
        digest.update(b"untracked\0")
        digest.update(relative.replace("\\", "/").encode("utf-8"))
        digest.update(b"\0")
        digest.update(file_hash.encode("ascii"))
    return {
        "commit": head,
        "tree": tree,
        "dirty": bool(tracked_diff or untracked),
        "dirty_diff_sha256": digest.hexdigest(),
        "untracked": untracked,
    }


def runtime_versions(repo_root: Path) -> dict[str, Any]:
    return {
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "ros_base_package": optional_text(
            ["dpkg-query", "-W", "-f=${Version}", "ros-jazzy-ros-base"], repo_root
        ),
        "gazebo": optional_text(["gz", "sim", "--versions"], repo_root),
        "physics_engine": "gz-physics-dartsim-plugin",
        "python": sys.version.split()[0],
        "platform": sys.platform,
    }


def install_entries(runtime_ws: Path) -> list[str]:
    layout = runtime_ws / "install/.colcon_install_layout"
    try:
        if layout.read_text(encoding="utf-8").strip() == "merged":
            # Inventory the one real merged prefix.  Per-package prefixes are
            # mutually exclusive with the final-runtime closure contract.
            return ["install"]
    except (OSError, UnicodeError):
        pass
    entries = ["install/setup.bash"]
    for package in INSTALL_PACKAGES:
        entries.append(f"install/{package}")
    return entries


def create_build_manifest(args: argparse.Namespace) -> int:
    repo_root = args.repo_root.resolve()
    runtime_ws = args.runtime_ws.resolve()
    started_ns = args.build_started_epoch_ns
    recorded_ns = time.time_ns()
    if started_ns <= 0 or started_ns >= recorded_ns:
        raise AcceptanceError("build start must be a positive time before manifest capture")
    sources = inventory(repo_root, SOURCE_ROOTS)
    frozen_sources = frozen_source_contract(repo_root, runtime_ws)
    symlink_report = install_symlink_report_contract(runtime_ws)
    installed = inventory(runtime_ws, install_entries(runtime_ws))
    markers = package_build_markers(runtime_ws, started_ns)
    plugins = plugin_rows(installed, started_ns)
    source_install = source_install_contract(repo_root, runtime_ws)
    runtime = runtime_versions(repo_root)
    require_runtime_versions(runtime)
    manifest = {
        "schema_version": 1,
        "kind": "tzcup_integrated_acceptance_build_snapshot",
        "build_started_epoch_ns": started_ns,
        "build_started_utc": utc_iso(started_ns),
        "recorded_epoch_ns": recorded_ns,
        "recorded_utc": utc_iso(recorded_ns),
        "repo_root": str(repo_root),
        "runtime_ws": str(runtime_ws),
        "git": git_snapshot(repo_root),
        "source_inventory": sources,
        "source_inventory_sha256": inventory_digest(sources),
        "frozen_source": frozen_sources,
        "frozen_source_sha256": json_digest(frozen_sources),
        "install_symlink_report": symlink_report,
        "install_symlink_report_sha256": json_digest(symlink_report),
        "installed_inventory": installed,
        "installed_inventory_sha256": inventory_digest(installed),
        "package_build_markers": markers,
        "package_build_markers_sha256": json_digest(markers),
        "compiled_plugins": plugins,
        "source_install_contract": source_install,
        "source_install_contract_sha256": json_digest(source_install),
        "runtime": runtime,
    }
    write_json(args.output, manifest)
    return 0


def validate_build_snapshot(
    manifest_path: Path, repo_root: Path, runtime_ws: Path
) -> dict[str, Any]:
    manifest = read_json(manifest_path)
    if manifest.get("schema_version") != 1 or manifest.get("kind") != "tzcup_integrated_acceptance_build_snapshot":
        raise AcceptanceError("unsupported build manifest schema")
    if Path(str(manifest.get("repo_root"))).resolve() != repo_root.resolve():
        raise AcceptanceError("build manifest belongs to a different repository path")
    if Path(str(manifest.get("runtime_ws"))).resolve() != runtime_ws.resolve():
        raise AcceptanceError("build manifest belongs to a different runtime workspace")
    try:
        started_ns = int(manifest["build_started_epoch_ns"])
        recorded_ns = int(manifest["recorded_epoch_ns"])
    except (KeyError, TypeError, ValueError) as exc:
        raise AcceptanceError("build manifest timestamps are missing or invalid") from exc
    if started_ns <= 0 or recorded_ns <= started_ns:
        raise AcceptanceError("build manifest timestamps are not ordered")
    require_runtime_versions(manifest.get("runtime"))
    current_git = git_snapshot(repo_root)
    for field in ("commit", "tree", "dirty", "dirty_diff_sha256"):
        if current_git[field] != manifest.get("git", {}).get(field):
            raise AcceptanceError(f"source snapshot changed after fresh build: git.{field}")
    current_sources = inventory(repo_root, SOURCE_ROOTS)
    if inventory_digest(current_sources) != manifest.get("source_inventory_sha256"):
        raise AcceptanceError("critical source hash changed after fresh build")
    current_frozen_sources = frozen_source_contract(repo_root, runtime_ws)
    if json_digest(current_frozen_sources) != manifest.get("frozen_source_sha256"):
        raise AcceptanceError("frozen runtime source changed after build snapshot")
    current_symlink_report = install_symlink_report_contract(runtime_ws)
    if json_digest(current_symlink_report) != manifest.get(
        "install_symlink_report_sha256"
    ):
        raise AcceptanceError("install symbolic-link report changed after build snapshot")
    current_installed = inventory(runtime_ws, install_entries(runtime_ws))
    if inventory_digest(current_installed) != manifest.get("installed_inventory_sha256"):
        raise AcceptanceError("installed runtime hash changed after build snapshot")
    current_markers = package_build_markers(runtime_ws, started_ns)
    if json_digest(current_markers) != manifest.get("package_build_markers_sha256"):
        raise AcceptanceError("colcon package build markers changed after build snapshot")
    current_plugins = plugin_rows(current_installed, started_ns)
    if current_plugins != manifest.get("compiled_plugins"):
        raise AcceptanceError("compiled Gazebo plugin inventory changed after build snapshot")
    current_source_install = source_install_contract(repo_root, runtime_ws)
    if json_digest(current_source_install) != manifest.get("source_install_contract_sha256"):
        raise AcceptanceError("source/install file contract changed after build snapshot")
    return manifest


def preflight(args: argparse.Namespace) -> int:
    validate_build_snapshot(args.build_manifest, args.repo_root, args.runtime_ws)
    print("INTEGRATED_ACCEPTANCE_SOURCE_BOUND_PREFLIGHT_PASSED")
    return 0


def init_run(args: argparse.Namespace) -> int:
    build = validate_build_snapshot(args.build_manifest, args.repo_root, args.runtime_ws)
    started_ns = args.started_epoch_ns
    if started_ns < int(build["recorded_epoch_ns"]):
        raise AcceptanceError("run started before the build snapshot was recorded")
    surface = validate_side_brush_surface_audit(
        args.side_brush_surface_audit,
        installed_vehicle_xacro(args.runtime_ws.resolve()),
    )
    context = {
        "schema_version": 1,
        "kind": "tzcup_integrated_acceptance_run_context",
        "run_id": args.run_id,
        "repo_root": str(args.repo_root.resolve()),
        "runtime_ws": str(args.runtime_ws.resolve()),
        "build_manifest": str(args.build_manifest.resolve()),
        "build_manifest_sha256": sha256_file(args.build_manifest),
        "started_epoch_ns": started_ns,
        "started_utc": utc_iso(started_ns),
        "material": args.material,
        "side_brush_sdf_surface": {
            "path": str(args.side_brush_surface_audit.resolve()),
            "sha256": sha256_file(args.side_brush_surface_audit),
            "expanded_sdf_sha256": surface["expanded_sdf_sha256"],
            "source_xacro": surface["source"]["path"],
            "runtime_effectiveness": surface["runtime_effectiveness"],
        },
        "scenarios": {},
    }
    session = getattr(args, "session", None)
    snapshot = getattr(args, "snapshot", None)
    if (session is None) != (snapshot is None):
        raise AcceptanceError("session and snapshot must be supplied together")
    if session is not None and snapshot is not None:
        session_payload = read_json(session)
        started = session_payload.get("started_epoch_ns")
        if (
            session_payload.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
            or not isinstance(started, int)
            or started <= 0
        ):
            raise AcceptanceError("formal acceptance session is not a fresh RUNNING session")
        if not snapshot.is_file():
            raise AcceptanceError("frozen snapshot manifest is missing")
        context["formal_attempt_binding"] = {
            "session_path": str(session.resolve()),
            "session_sha256": sha256_file(session),
            "session_started_epoch_ns": started,
            "snapshot_path": str(snapshot.resolve()),
            "snapshot_sha256": sha256_file(snapshot),
            "source_build_manifest_sha256": sha256_file(args.build_manifest),
        }
    write_json(args.context, context)
    return 0


def record_scenario(args: argparse.Namespace) -> int:
    context = read_json(args.context)
    scenarios = context.setdefault("scenarios", {})
    if args.name not in SCENARIOS:
        raise AcceptanceError(f"unknown scenario: {args.name}")
    if args.name in scenarios:
        raise AcceptanceError(f"scenario already recorded: {args.name}")
    if args.started_epoch_ns < int(context["started_epoch_ns"]):
        raise AcceptanceError("scenario predates run")
    if args.finished_epoch_ns < args.started_epoch_ns:
        raise AcceptanceError("scenario finish predates start")
    if not 0 <= args.ros_domain_id <= 232:
        raise AcceptanceError("ROS_DOMAIN_ID must be in the DDS-supported range 0..232")
    row = {
        "started_epoch_ns": args.started_epoch_ns,
        "started_utc": utc_iso(args.started_epoch_ns),
        "finished_epoch_ns": args.finished_epoch_ns,
        "finished_utc": utc_iso(args.finished_epoch_ns),
        "exit_code": args.exit_code,
        "ros_domain_id": args.ros_domain_id,
        "gz_partition": args.gz_partition,
        "result": str(args.result.resolve()),
        "result_sha256": sha256_file(args.result) if args.result.is_file() else None,
        "launch_log": str(args.launch_log.resolve()),
        "launch_log_sha256": sha256_file(args.launch_log) if args.launch_log.is_file() else None,
        "runner_log": str(args.runner_log.resolve()),
        "runner_log_sha256": sha256_file(args.runner_log) if args.runner_log.is_file() else None,
        "cleanup_remaining_pids": args.cleanup_remaining_pids,
    }
    required_binding = context.get("formal_attempt_binding")
    if required_binding is not None:
        runtime_binding = getattr(args, "runtime_binding", None)
        if runtime_binding is None or not runtime_binding.is_file():
            # Do not hide a completed independent scenario merely because its
            # required sidecar is absent.  Retain the row and let aggregate
            # fail closed; later independent scenarios can still run.
            row["attempt_binding_error"] = "fresh runtime-binding sidecar missing"
        else:
            sidecar_path = args.result.with_name(args.result.name + ".attempt_binding.json")
            sidecar = {
                "schema_version": 1,
                "kind": "tzcup_integrated_acceptance_scenario_attempt_binding_v1",
                "scenario": args.name,
                "recorded_epoch_ns": time.time_ns(),
                "invocation": {
                    "started_epoch_ns": args.started_epoch_ns,
                    "finished_epoch_ns": args.finished_epoch_ns,
                },
                "formal_session": required_binding,
                "runtime_binding": {
                    "path": str(runtime_binding.resolve()),
                    "sha256": sha256_file(runtime_binding),
                },
            }
            write_json(sidecar_path, sidecar)
            row["attempt_binding_path"] = str(sidecar_path.resolve())
            row["attempt_binding_sha256"] = sha256_file(sidecar_path)
    scenarios[args.name] = row
    write_json(args.context, context)
    return 0


def validate_result(
    name: str, row: dict[str, Any], expected_material: str,
    formal_attempt_binding: dict[str, Any] | None = None,
) -> dict[str, Any]:
    if formal_attempt_binding is not None:
        sidecar_path = Path(str(row.get("attempt_binding_path", "")))
        if not sidecar_path.is_file() or sha256_file(sidecar_path) != row.get("attempt_binding_sha256"):
            raise AcceptanceError(f"{name} fresh session/snapshot/source sidecar is missing or changed")
        sidecar = read_json(sidecar_path)
        if sidecar.get("kind") != "tzcup_integrated_acceptance_scenario_attempt_binding_v1" or sidecar.get("scenario") != name:
            raise AcceptanceError(f"{name} scenario sidecar identity mismatch")
        if sidecar.get("formal_session") != formal_attempt_binding:
            raise AcceptanceError(f"{name} scenario sidecar is not bound to the fresh session/snapshot/source")
        invocation = sidecar.get("invocation")
        if not isinstance(invocation, dict) or invocation.get("started_epoch_ns") != row.get("started_epoch_ns") or invocation.get("finished_epoch_ns") != row.get("finished_epoch_ns"):
            raise AcceptanceError(f"{name} scenario sidecar invocation mismatch")
        runtime_binding = sidecar.get("runtime_binding")
        if not isinstance(runtime_binding, dict) or not isinstance(runtime_binding.get("sha256"), str):
            raise AcceptanceError(f"{name} scenario runtime-binding sidecar is missing")
    if int(row.get("exit_code", -1)) != 0:
        raise AcceptanceError(f"{name} exited nonzero: {row.get('exit_code')}")
    if int(row.get("cleanup_remaining_pids", -1)) != 0:
        raise AcceptanceError(f"{name} leaked scenario processes")
    path = Path(str(row.get("result", "")))
    if not path.is_file():
        raise AcceptanceError(f"{name} result is missing: {path}")
    stat = path.stat()
    start_ns = int(row["started_epoch_ns"])
    finish_ns = int(row["finished_epoch_ns"])
    # Some filesystems expose coarse timestamps; one second tolerance is only
    # for timestamp quantization, never for files existing before invocation.
    if stat.st_mtime_ns + 1_000_000_000 < start_ns or stat.st_mtime_ns > finish_ns + 1_000_000_000:
        raise AcceptanceError(f"{name} result is stale or outside its invocation window")
    if sha256_file(path) != row.get("result_sha256"):
        raise AcceptanceError(f"{name} result changed after it was recorded")
    for log_key in ("launch_log", "runner_log"):
        log_path = Path(str(row.get(log_key, "")))
        if not log_path.is_file():
            raise AcceptanceError(f"{name} {log_key} is missing: {log_path}")
        log_stat = log_path.stat()
        if log_stat.st_mtime_ns + 1_000_000_000 < start_ns or log_stat.st_mtime_ns > finish_ns + 1_000_000_000:
            raise AcceptanceError(f"{name} {log_key} is outside its invocation window")
        if sha256_file(log_path) != row.get(f"{log_key}_sha256"):
            raise AcceptanceError(f"{name} {log_key} changed after it was recorded")
    result = read_json(path)
    if result.get("passed") is not True:
        raise AcceptanceError(f"{name} lacks explicit passed=true")
    if name == "mobility":
        if result.get("report_id") != "tzcup_formal_a300_drivetrain_runtime_v1":
            raise AcceptanceError("mobility schema/report_id mismatch")
        expected_status = "FORMAL_A300_DRIVETRAIN_FORWARD_STOP_RUNTIME_PASSED"
    elif name in ("water_normal", "water_full"):
        if result.get("schema_version") != 1:
            raise AcceptanceError(f"{name} schema_version mismatch")
        expected_scenario = "normal_recovery" if name == "water_normal" else "full_tank_fail_closed"
        if result.get("scenario") != expected_scenario:
            raise AcceptanceError(f"{name} scenario identity mismatch")
        expected_status = "FORMAL_WATER_RECOVERY_SCENARIO_PASSED"
    else:
        if result.get("report_id") != "tzcup_formal_physical_cube_pick_place_v1":
            raise AcceptanceError("manipulation schema/report_id mismatch")
        gate = result.get("grasp_gate", {})
        proof = result.get("attachment_constraint_proof", {})
        if not (
            gate.get("attach_permitted") is True
            and int(gate.get("left_cube_contact_count", 0)) > 0
            and int(gate.get("right_cube_contact_count", 0)) > 0
        ):
            raise AcceptanceError("manipulation lacks the dual-contact physical grasp gate")
        # The transport state ACK is intentionally diagnostic: the independent
        # acceptance truth is rigid cube/wrist motion during a meaningful lift.
        # A false ACK therefore remains acceptable only when this pose evidence
        # is present and within the verifier's frozen tolerance.
        if not (
            proof.get("constraint_proven_by_rigid_motion_not_ack") is True
            and float(proof.get("cube_lift_m", 0.0)) >= 0.20
            and float(proof.get("offset_change_m", 1.0)) <= 0.012
        ):
            raise AcceptanceError("manipulation lacks rigid cube/wrist lift evidence")
        cube = result.get("cube", {})
        support = result.get("bin_load_bearing_contact", {})
        dry_bin = result.get("dry_bin_monitor", {})
        inventory = result.get("inventory_mass", {})
        evaluator = result.get("evaluator_interface_audit", {})
        settled_pose = cube.get("settled_pose_m", {})
        try:
            settled_z = float(settled_pose["z"])
            support_z = float(cube["bin_floor_support_z_m"])
            support_tolerance = float(cube["bin_floor_support_tolerance_m"])
            settled_duration = float(cube["settled_sim_duration_s"])
            support_count = int(support["support_contact_count"])
            support_span = float(support["support_contact_span_sim_s"])
            dry_bin_count = int(dry_bin["contained_object_count"])
            dry_bin_mass = float(dry_bin["contained_mass_kg"])
            dry_bin_tolerance = float(dry_bin["mass_tolerance_kg"])
            cube_mass = float(cube["mass_kg"])
            physical_cube_count = int(inventory["physical_cube_count"])
            physical_material_mass = float(inventory["physical_material_mass_kg"])
            payload_command_count = int(inventory["dynamic_dry_payload_command_count"])
            payload_added_mass = float(inventory["dynamic_dry_payload_added_kg"])
            delete_entity_calls = int(evaluator["delete_entity_calls"])
        except (KeyError, TypeError, ValueError) as exc:
            raise AcceptanceError(f"manipulation physical deposit schema is incomplete: {exc}") from exc
        if cube.get("present_after_deposit") is not True:
            raise AcceptanceError("manipulation cube is not physically present after deposit")
        if cube.get("stable_inside_dry_bin") is not True:
            raise AcceptanceError("manipulation cube is not stable inside the dry bin")
        if settled_duration < 3.0:
            raise AcceptanceError("manipulation deposit was observed for less than 3 simulated seconds")
        if support_tolerance <= 0.0 or abs(settled_z - support_z) > support_tolerance:
            raise AcceptanceError("manipulation settled cube height is outside floor support tolerance")
        if not (
            support_count > 0
            and support_span >= 0.5
            and support.get("persistent_support_observed") is True
            and bool(support.get("vehicle_collision_names"))
        ):
            raise AcceptanceError("manipulation lacks persistent vehicle-bin load-bearing contact")
        expected_mass = MATERIAL_MASS_KG[expected_material]
        if result.get("material") != expected_material or abs(cube_mass - expected_mass) > 1e-8:
            raise AcceptanceError(
                "manipulation did not deposit the required "
                f"{expected_mass:.8g} kg {expected_material} cube"
            )
        if not (
            dry_bin.get("sensor_ready") is True
            and dry_bin_count == 1
            and 0.0 < dry_bin_tolerance <= 1e-4
            and abs(dry_bin_mass - expected_mass) <= dry_bin_tolerance
            and dry_bin.get("full") is False
            and dry_bin.get("post_release_sample_observed") is True
            and dry_bin.get("monitor_is_observation_only") is True
        ):
            raise AcceptanceError(
                "manipulation lacks valid bridged dry-bin "
                f"{expected_material} inventory telemetry"
            )
        if not (
            physical_cube_count == 1
            and abs(physical_material_mass - expected_mass) <= 1e-8
            and payload_command_count == 0
            and payload_added_mass == 0.0
            and inventory.get("double_count_prevented") is True
            and inventory.get("aggregation_or_reserve_payload_substitution") is False
            and delete_entity_calls == 0
            and evaluator.get("set_pose_or_remove_after_task_start") is False
            and evaluator.get("physical_cube_deleted_after_deposit") is False
        ):
            raise AcceptanceError("manipulation cube was deleted, aggregated, substituted, or double-counted")
        expected_status = "PHYSICAL_CONTACT_GATED_PICK_LIFT_DEPOSIT_PASSED"
    if result.get("status") != expected_status:
        raise AcceptanceError(f"{name} status mismatch: {result.get('status')}")
    return result


def validate_side_brush_surface_audit(
    path: Path, expected_installed_xacro: Path
) -> dict[str, Any]:
    report = read_json(path)
    if report.get("status") != "FORMAL_SIDE_BRUSH_EXPANDED_SDF_SURFACE_PASSED":
        raise AcceptanceError("side-brush expanded-SDF surface audit did not pass")
    source = report.get("source")
    if not isinstance(source, dict) or source.get("mode") != "xacro_to_gz_sdf":
        raise AcceptanceError("side-brush surface audit is not bound to xacro expansion")
    if Path(str(source.get("path", ""))).resolve() != expected_installed_xacro.resolve():
        raise AcceptanceError("side-brush surface audit did not use frozen installed xacro")
    expanded_hash = report.get("expanded_sdf_sha256")
    if not isinstance(expanded_hash, str) or len(expanded_hash) != 64:
        raise AcceptanceError("side-brush surface audit lacks expanded SDF hash")
    effectiveness = report.get("runtime_effectiveness")
    if not isinstance(effectiveness, dict) or effectiveness.get(
        "dart_effective_from_surface_friction_ode"
    ) != ["mu", "mu2"]:
        raise AcceptanceError("side-brush surface audit overstates DART effectiveness")
    return report


def aggregate(args: argparse.Namespace) -> int:
    context = read_json(args.context)
    if context.get("schema_version") != 1 or context.get("kind") != "tzcup_integrated_acceptance_run_context":
        raise AcceptanceError("unsupported run context schema")
    finished_ns = args.finished_epoch_ns
    if finished_ns < int(context["started_epoch_ns"]):
        raise AcceptanceError("run finish predates run start")
    build_path = Path(context["build_manifest"])
    if sha256_file(build_path) != context.get("build_manifest_sha256"):
        raise AcceptanceError("build manifest changed during run")
    build = validate_build_snapshot(
        build_path, Path(context["repo_root"]), Path(context["runtime_ws"])
    )
    surface_evidence = context.get("side_brush_sdf_surface")
    if not isinstance(surface_evidence, dict):
        raise AcceptanceError("side-brush expanded-SDF surface evidence is missing")
    surface_path = Path(str(surface_evidence.get("path", "")))
    if not surface_path.is_file() or sha256_file(surface_path) != surface_evidence.get("sha256"):
        raise AcceptanceError("side-brush expanded-SDF surface evidence changed during run")
    surface_report = validate_side_brush_surface_audit(
        surface_path,
        installed_vehicle_xacro(Path(context["runtime_ws"])),
    )
    if surface_report.get("expanded_sdf_sha256") != surface_evidence.get(
        "expanded_sdf_sha256"
    ):
        raise AcceptanceError("side-brush expanded SDF hash changed during run")
    rows = context.get("scenarios")
    if not isinstance(rows, dict) or set(rows) != set(SCENARIOS):
        missing = sorted(set(SCENARIOS) - set(rows or {}))
        extra = sorted(set(rows or {}) - set(SCENARIOS))
        raise AcceptanceError(f"scenario set mismatch; missing={missing}, extra={extra}")
    domains = [int(rows[name]["ros_domain_id"]) for name in SCENARIOS]
    partitions = [str(rows[name]["gz_partition"]) for name in SCENARIOS]
    invalid_domains = [domain for domain in domains if not 0 <= domain <= 232]
    if invalid_domains:
        raise AcceptanceError(f"ROS domains must be in 0..232: {invalid_domains}")
    if len(set(domains)) != len(domains) or len(set(partitions)) != len(partitions):
        raise AcceptanceError("ROS domains and Gazebo partitions must be unique per scenario")
    previous_finish = int(context["started_epoch_ns"])
    for name in SCENARIOS:
        try:
            scenario_start = int(rows[name]["started_epoch_ns"])
            scenario_finish = int(rows[name]["finished_epoch_ns"])
        except (KeyError, TypeError, ValueError) as exc:
            raise AcceptanceError(f"{name} scenario timestamps are missing or invalid") from exc
        if scenario_start < previous_finish:
            raise AcceptanceError(
                f"{name} overlaps or is out of order with the previous scenario"
            )
        if scenario_finish < scenario_start:
            raise AcceptanceError(f"{name} finish predates its start")
        if scenario_finish > finished_ns:
            raise AcceptanceError(f"{name} finished after the integrated run finished")
        previous_finish = scenario_finish
    material = context.get("material")
    if material not in MATERIAL_MASS_KG:
        raise AcceptanceError(f"unsupported or missing run material: {material}")
    formal_attempt_binding = context.get("formal_attempt_binding")
    if formal_attempt_binding is not None and not isinstance(formal_attempt_binding, dict):
        raise AcceptanceError("formal attempt binding is invalid")
    results = {
        name: validate_result(name, rows[name], str(material), formal_attempt_binding)
        for name in SCENARIOS
    }
    runtime_gate_binding = None
    runtime_binding_path = getattr(args, "runtime_binding", None)
    if runtime_binding_path is not None:
        try:
            from formal_runtime_gate_binding import RuntimeGateError, load_binding

            runtime_gate_binding = load_binding(Path(runtime_binding_path))
        except (RuntimeGateError, OSError, ValueError) as exc:
            raise AcceptanceError(f"runtime gate binding is invalid: {exc}") from exc
    manifest = {
        "schema_version": 1,
        "report_id": "tzcup_integrated_basic_functional_acceptance_v1",
        "status": "INTEGRATED_BASIC_FUNCTIONAL_ACCEPTANCE_PASSED",
        "passed": True,
        "run_id": context["run_id"],
        "started_epoch_ns": context["started_epoch_ns"],
        "started_utc": context["started_utc"],
        "finished_epoch_ns": finished_ns,
        "finished_utc": utc_iso(finished_ns),
        "source_bound": True,
        "material_contract": {
            "material": material,
            "cube_edge_m": 0.03,
            "expected_mass_kg": MATERIAL_MASS_KG[str(material)],
        },
        "fresh_build": {
            "build_started_epoch_ns": build["build_started_epoch_ns"],
            "recorded_epoch_ns": build["recorded_epoch_ns"],
            "manifest": str(build_path.resolve()),
            "manifest_sha256": context["build_manifest_sha256"],
        },
        "git": build["git"],
        "critical_source_inventory_sha256": build["source_inventory_sha256"],
        "installed_runtime_inventory_sha256": build["installed_inventory_sha256"],
        "critical_file_sha256": {
            name: row["sha256"]
            for inventory_name in ("source_inventory", "installed_inventory")
            for name, row in build[inventory_name].items()
            if name.endswith(CRITICAL_SUFFIXES)
        },
        "runtime": build["runtime"],
        "side_brush_sdf_surface": surface_evidence,
        "scenario_invocations": rows,
        "scenario_results": results,
        "claim_boundary": (
            "This manifest proves one fresh, source-bound, isolated Gazebo run of straight motion/stop, "
            "normal water recovery, full-tank fail-closed behavior, and one contact-gated material cube "
            "pick/lift/deposit. It does not by itself prove autonomous perception, park-scale planning, "
            "randomized generalization, or Journey 6 deployment."
        ),
    }
    if runtime_gate_binding is not None:
        manifest["runtime_gate_binding"] = runtime_gate_binding
    write_json(args.output, manifest)
    print("INTEGRATED_BASIC_FUNCTIONAL_ACCEPTANCE_PASSED")
    return 0


def aggregate_attempt(args: argparse.Namespace) -> int:
    """Persist a truthful failed-attempt manifest instead of discarding other chains."""

    try:
        return aggregate(args)
    except AcceptanceError as exc:
        context = read_json(args.context)
        manifest = {
            "schema_version": 1,
            "report_id": "tzcup_integrated_basic_functional_acceptance_attempt_v1",
            "status": "INTEGRATED_BASIC_FUNCTIONAL_ACCEPTANCE_FAILED",
            "passed": False,
            "run_id": context.get("run_id"),
            "started_epoch_ns": context.get("started_epoch_ns"),
            "finished_epoch_ns": args.finished_epoch_ns,
            "source_bound": False,
            "failure": str(exc),
            "scenario_invocations": context.get("scenarios", {}),
            "formal_attempt_binding_required": "formal_attempt_binding" in context,
            "claim_boundary": "Failed-attempt preservation only. This manifest is not a runtime PASS and cannot be published as an acceptance summary.",
        }
        write_json(args.output, manifest)
        print(f"INTEGRATED_ACCEPTANCE_ATTEMPT_FAILED: {exc}", file=sys.stderr)
        return 3


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser()
    commands = root.add_subparsers(dest="command", required=True)

    build = commands.add_parser("record-build")
    build.add_argument("--repo-root", type=Path, required=True)
    build.add_argument("--runtime-ws", type=Path, required=True)
    build.add_argument("--build-started-epoch-ns", type=int, required=True)
    build.add_argument("--output", type=Path, required=True)
    build.set_defaults(func=create_build_manifest)

    check = commands.add_parser("preflight")
    check.add_argument("--repo-root", type=Path, required=True)
    check.add_argument("--runtime-ws", type=Path, required=True)
    check.add_argument("--build-manifest", type=Path, required=True)
    check.set_defaults(func=preflight)

    initialize = commands.add_parser("init-run")
    initialize.add_argument("--repo-root", type=Path, required=True)
    initialize.add_argument("--runtime-ws", type=Path, required=True)
    initialize.add_argument("--build-manifest", type=Path, required=True)
    initialize.add_argument("--context", type=Path, required=True)
    initialize.add_argument("--run-id", required=True)
    initialize.add_argument("--started-epoch-ns", type=int, required=True)
    initialize.add_argument("--material", choices=sorted(MATERIAL_MASS_KG), required=True)
    initialize.add_argument("--session", type=Path)
    initialize.add_argument("--snapshot", type=Path)
    initialize.add_argument("--side-brush-surface-audit", type=Path, required=True)
    initialize.set_defaults(func=init_run)

    record = commands.add_parser("record-scenario")
    record.add_argument("--context", type=Path, required=True)
    record.add_argument("--name", choices=SCENARIOS, required=True)
    record.add_argument("--started-epoch-ns", type=int, required=True)
    record.add_argument("--finished-epoch-ns", type=int, required=True)
    record.add_argument("--exit-code", type=int, required=True)
    record.add_argument("--ros-domain-id", type=int, required=True)
    record.add_argument("--gz-partition", required=True)
    record.add_argument("--result", type=Path, required=True)
    record.add_argument("--launch-log", type=Path, required=True)
    record.add_argument("--runner-log", type=Path, required=True)
    record.add_argument("--cleanup-remaining-pids", type=int, required=True)
    record.add_argument("--runtime-binding", type=Path)
    record.set_defaults(func=record_scenario)

    final = commands.add_parser("aggregate")
    final.add_argument("--context", type=Path, required=True)
    final.add_argument("--output", type=Path, required=True)
    final.add_argument("--finished-epoch-ns", type=int, required=True)
    final.add_argument("--runtime-binding", type=Path, required=True)
    final.set_defaults(func=aggregate)
    attempt = commands.add_parser("aggregate-attempt")
    attempt.add_argument("--context", type=Path, required=True)
    attempt.add_argument("--output", type=Path, required=True)
    attempt.add_argument("--finished-epoch-ns", type=int, required=True)
    attempt.add_argument("--runtime-binding", type=Path, required=True)
    attempt.set_defaults(func=aggregate_attempt)
    return root


def main() -> int:
    args = parser().parse_args()
    try:
        return int(args.func(args))
    except AcceptanceError as exc:
        print(f"INTEGRATED_ACCEPTANCE_FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
