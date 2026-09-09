#!/usr/bin/env python3
"""Fail closed when a ROS 2 runtime overlay cannot be tied to its sources.

The checker is intentionally conservative.  It only accepts sources, installed
payloads, and the Python files selected by the *current* interpreter when every
resolved path remains below ``--runtime-ws``.  This catches an accidentally
sourced older overlay as well as a missing ``colcon build`` after source edits.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any


PYTHON_PACKAGES = (
    "sanitation_formal_campus_integration",
    "sanitation_hmi",
)
CPP_PACKAGE = "sanitation_gazebo_control"
CPP_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp"}


def _resolved(path: Path) -> Path:
    """Resolve without accepting a missing symlink target."""
    return path.resolve(strict=True)


def _inside(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _file_manifest(root: Path, suffixes: set[str] | None = None) -> dict[str, str]:
    """Return a content manifest keyed by POSIX relative filename."""
    manifest: dict[str, str] = {}
    for item in sorted(root.rglob("*")):
        if not item.is_file() or "__pycache__" in item.parts:
            continue
        if suffixes is not None and item.suffix not in suffixes:
            continue
        digest = hashlib.sha256(item.read_bytes()).hexdigest()
        manifest[item.relative_to(root).as_posix()] = digest
    return manifest


def _record_error(report: dict[str, Any], code: str, **details: Any) -> None:
    report["errors"].append({"code": code, **details})


def _require_path(
    report: dict[str, Any], path: Path, runtime_root: Path, label: str
) -> Path | None:
    try:
        resolved = _resolved(path)
    except (OSError, RuntimeError):
        _record_error(report, "missing_path", label=label, path=str(path))
        return None
    if not _inside(resolved, runtime_root):
        _record_error(
            report,
            "external_path",
            label=label,
            path=str(path),
            resolved_path=str(resolved),
        )
        return None
    return resolved


def _python_import_locations(packages: tuple[str, ...]) -> tuple[dict[str, Any], str | None]:
    """Query a child interpreter so its answer matches the launched runtime."""
    probe = """
import importlib, json
names = json.loads(__import__('sys').argv[1])
answer = {}
for name in names:
    try:
        module = importlib.import_module(name)
        answer[name] = {"file": getattr(module, "__file__", None)}
    except Exception as exc:
        answer[name] = {"error": type(exc).__name__ + ": " + str(exc)}
print(json.dumps(answer, sort_keys=True))
"""
    try:
        completed = subprocess.run(
            [sys.executable, "-c", probe, json.dumps(packages)],
            check=False,
            capture_output=True,
            text=True,
            env=os.environ.copy(),
        )
    except OSError as exc:
        return {}, f"cannot start interpreter: {exc}"
    if completed.returncode != 0:
        return {}, (completed.stderr.strip() or f"interpreter returned {completed.returncode}")
    try:
        return json.loads(completed.stdout), None
    except json.JSONDecodeError as exc:
        return {}, f"invalid interpreter JSON: {exc}"


def _check_python_package(
    report: dict[str, Any], runtime_root: Path, package: str, imported: dict[str, Any]
) -> None:
    result: dict[str, Any] = {"ok": False}
    report["packages"][package] = result
    source = _require_path(
        report, runtime_root / "src" / package / package, runtime_root, f"{package} source"
    )
    if source is None or not source.is_dir():
        _record_error(report, "missing_source_package", package=package)
        return
    result["source"] = str(source)
    source_manifest = _file_manifest(source, {".py"})
    if not source_manifest:
        _record_error(report, "empty_source_package", package=package, path=str(source))
        return

    import_answer = imported.get(package, {})
    if "error" in import_answer:
        _record_error(report, "import_failed", package=package, detail=import_answer["error"])
        return
    loaded_file = import_answer.get("file")
    if not loaded_file:
        _record_error(report, "missing_import_file", package=package)
        return
    loaded = _require_path(report, Path(loaded_file), runtime_root, f"{package} imported file")
    if loaded is None or not loaded.is_file():
        return
    result["imported_file"] = str(loaded)

    source_init = source / "__init__.py"
    if not source_init.is_file() or hashlib.sha256(source_init.read_bytes()).hexdigest() != hashlib.sha256(loaded.read_bytes()).hexdigest():
        _record_error(
            report,
            "python_init_mismatch",
            package=package,
            source=str(source_init),
            imported=str(loaded),
        )
        return

    loaded_root = loaded.parent
    loaded_manifest = _file_manifest(loaded_root, {".py"})
    if loaded_manifest != source_manifest:
        _record_error(
            report,
            "python_package_mismatch",
            package=package,
            source_files=sorted(source_manifest),
            imported_files=sorted(loaded_manifest),
        )
        return

    result["mode"] = "symlink-install" if loaded_root == source else "installed-copy"
    result["python_files"] = len(source_manifest)
    result["ok"] = True


def _check_share_payload(
    report: dict[str, Any], runtime_root: Path, package: str
) -> None:
    result = report["packages"].setdefault(package, {"ok": False})
    all_current = bool(result.get("ok"))
    payloads: dict[str, Any] = {}
    result["share_payloads"] = payloads
    for name in ("launch", "config"):
        source_path = runtime_root / "src" / package / name
        installed_path = runtime_root / "install" / package / "share" / package / name
        payload = {"ok": False}
        payloads[name] = payload
        if not source_path.exists() and not installed_path.exists():
            payload.update({"ok": True, "status": "not-present", "files": 0})
            continue
        source = _require_path(
            report, source_path, runtime_root, f"{package} source {name}"
        )
        installed = _require_path(
            report,
            installed_path,
            runtime_root,
            f"{package} installed {name}",
        )
        if source is None or installed is None or not source.is_dir() or not installed.is_dir():
            _record_error(report, "missing_share_payload", package=package, payload=name)
            all_current = False
            continue
        source_manifest = _file_manifest(source)
        installed_manifest = _file_manifest(installed)
        payload.update({"source": str(source), "installed": str(installed), "files": len(source_manifest)})
        if not source_manifest or source_manifest != installed_manifest:
            _record_error(report, "share_payload_mismatch", package=package, payload=name)
            all_current = False
            continue
        payload["ok"] = True
    result["ok"] = all_current


def _cpp_sources(root: Path) -> list[Path]:
    files: list[Path] = []
    cmake = root / "CMakeLists.txt"
    if cmake.is_file():
        files.append(cmake)
    for folder in (root / "src", root / "include"):
        if folder.is_dir():
            files.extend(path for path in folder.rglob("*") if path.is_file() and path.suffix in CPP_SUFFIXES)
    return sorted(files)


def _cpp_artifacts(root: Path) -> list[Path]:
    library_root = root / "install" / CPP_PACKAGE / "lib"
    package_bin_root = library_root / CPP_PACKAGE
    artifacts: list[Path] = []
    if library_root.is_dir():
        artifacts.extend(path for path in library_root.iterdir() if path.is_file() and ".so" in path.name)
    if package_bin_root.is_dir():
        artifacts.extend(path for path in package_bin_root.rglob("*") if path.is_file())
    return sorted(set(artifacts))


def _check_cpp_package(report: dict[str, Any], runtime_root: Path) -> None:
    result: dict[str, Any] = {"ok": False}
    report["packages"][CPP_PACKAGE] = result
    source_root = _require_path(
        report, runtime_root / "src" / CPP_PACKAGE, runtime_root, f"{CPP_PACKAGE} source"
    )
    if source_root is None or not source_root.is_dir():
        _record_error(report, "missing_source_package", package=CPP_PACKAGE)
        return
    sources = _cpp_sources(source_root)
    if not sources:
        _record_error(report, "missing_cpp_sources", package=CPP_PACKAGE)
        return
    artifacts = _cpp_artifacts(runtime_root)
    if not artifacts:
        _record_error(report, "missing_cpp_artifacts", package=CPP_PACKAGE)
        return
    resolved_artifacts: list[Path] = []
    for artifact in artifacts:
        resolved = _require_path(report, artifact, runtime_root, f"{CPP_PACKAGE} artifact")
        if resolved is None or not resolved.is_file():
            return
        resolved_artifacts.append(resolved)
    build_receipt = _require_path(
        report,
        runtime_root / "build" / CPP_PACKAGE / "colcon_build.rc",
        runtime_root,
        f"{CPP_PACKAGE} successful build receipt",
    )
    if build_receipt is None or not build_receipt.is_file():
        return
    if build_receipt.read_text(encoding="utf-8").strip() != "0":
        _record_error(
            report,
            "cpp_build_failed",
            package=CPP_PACKAGE,
            receipt=str(build_receipt),
        )
        return
    newest_source = max(path.stat().st_mtime_ns for path in sources)
    successful_build = build_receipt.stat().st_mtime_ns
    result.update(
        {
            "source_files": len(sources),
            "artifacts": [str(path) for path in resolved_artifacts],
            "newest_source_mtime_ns": newest_source,
            "successful_build_mtime_ns": successful_build,
        }
    )
    if successful_build < newest_source:
        _record_error(
            report,
            "cpp_artifact_stale",
            package=CPP_PACKAGE,
            newest_source_mtime_ns=newest_source,
            successful_build_mtime_ns=successful_build,
        )
        return
    result["ok"] = True


def check(runtime_ws: Path) -> dict[str, Any]:
    report: dict[str, Any] = {"ok": False, "runtime_ws": str(runtime_ws), "packages": {}, "errors": []}
    try:
        runtime_root = _resolved(runtime_ws)
    except (OSError, RuntimeError):
        _record_error(report, "missing_runtime_workspace", path=str(runtime_ws))
        return report
    report["runtime_ws"] = str(runtime_root)
    imported, probe_error = _python_import_locations(PYTHON_PACKAGES)
    if probe_error:
        _record_error(report, "import_probe_failed", detail=probe_error)
        imported = {}
    for package in PYTHON_PACKAGES:
        _check_python_package(report, runtime_root, package, imported)
        _check_share_payload(report, runtime_root, package)
    _check_cpp_package(report, runtime_root)
    report["ok"] = not report["errors"] and all(
        package.get("ok") for package in report["packages"].values()
    )
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-ws", required=True, type=Path, help="ROS 2 workspace containing src/ and install/")
    args = parser.parse_args(argv)
    report = check(args.runtime_ws)
    print(json.dumps(report, sort_keys=True, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
