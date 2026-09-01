#!/usr/bin/env python3
"""Bind live Gazebo and ros_gz_image processes to the frozen transport ABI.

The collector is intentionally independent from the visual runner.  It reads
only ``/proc/<pid>/exe`` and ``/proc/<pid>/maps`` plus immutable runtime
artifacts, then writes one exclusive JSON report.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


REPORT_ID = "tzcup_formal_transport_process_maps_v1"
FORBIDDEN_RUNTIME_FRAGMENTS = (
    "/opt/ros/jazzy/opt/gz_transport_vendor",
    "/opt/ros/jazzy/opt/ortools_vendor",
)
GZ_TRANSPORT_CORE = re.compile(r"^libgz-transport13\.so(?:\.13(?:\.5\.0)?)?$")
PROTOBUF_RUNTIME = re.compile(r"^libprotobuf\.so\.32\.0\.12$")
ZMQ5_RUNTIME = re.compile(r"^libzmq\.so\.5(?:\.\d+)*$")


class CaptureError(RuntimeError):
    """Raised when process-to-runtime evidence cannot be proved."""


def sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def json_digest(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _regular_file(path: Path, label: str) -> Path:
    if path.is_symlink() or not path.is_file():
        raise CaptureError(f"{label} must be a regular non-symlink file: {path}")
    try:
        return path.resolve(strict=True)
    except OSError as error:
        raise CaptureError(f"cannot resolve {label}: {path}: {error}") from error


@dataclass(frozen=True)
class RuntimeContract:
    runtime_setup: Path
    runtime_setup_sha256: str
    closure_manifest: Path
    closure_manifest_sha256: str
    closure_sha256: str
    install_root: Path
    transport_sha256: str
    image_bridge_executable: Path
    image_bridge_sha256: str


@dataclass(frozen=True)
class ProcessSnapshot:
    pid: int
    executable: Path
    executable_sha256: str
    maps_sha256: str
    mapped_paths: tuple[Path, ...]


def load_runtime_contract(
    runtime_setup: Path, closure_manifest: Path
) -> RuntimeContract:
    runtime_setup = _regular_file(runtime_setup, "runtime setup")
    closure_manifest = _regular_file(closure_manifest, "closure manifest")
    try:
        manifest = json.loads(closure_manifest.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise CaptureError(f"cannot read closure manifest: {error}") from error
    if not isinstance(manifest, dict):
        raise CaptureError("closure manifest root must be an object")
    if (
        manifest.get("schema_version") != 5
        or manifest.get("kind") != "tzcup_formal_final_runtime_closure"
        or manifest.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_FROZEN"
    ):
        raise CaptureError("closure manifest is not the frozen schema-v5 runtime")
    closure = manifest.get("closure")
    if not isinstance(closure, dict):
        raise CaptureError("closure manifest has no closure object")
    closure_sha256 = json_digest(closure)
    if manifest.get("closure_sha256") != closure_sha256:
        raise CaptureError("closure object digest does not match the manifest")

    merged = closure.get("merged_overlay")
    vendor = closure.get("gz_transport13_vendor")
    image_runtime = closure.get("ros_gz_image_system_runtime")
    install_inventory = closure.get("install_inventory")
    if not all(
        isinstance(value, dict)
        for value in (merged, vendor, image_runtime, install_inventory)
    ):
        raise CaptureError("closure lacks runtime ABI identity sections")

    install_root = Path(str(merged.get("install_root", ""))).resolve(strict=True)
    runtime_ws = Path(str(closure.get("runtime_ws", ""))).resolve(strict=True)
    if install_root != (runtime_ws / "install").resolve(strict=True):
        raise CaptureError("closure merged install root differs from runtime workspace")
    expected_setup = (install_root / "setup.bash").resolve(strict=True)
    if runtime_setup != expected_setup:
        raise CaptureError(
            f"runtime setup is outside the frozen install: {runtime_setup} != {expected_setup}"
        )
    setup_row = install_inventory.get("setup.bash")
    if not isinstance(setup_row, dict):
        raise CaptureError("closure install inventory does not bind setup.bash")
    runtime_setup_sha256 = sha256_path(runtime_setup)
    if setup_row.get("sha256") != runtime_setup_sha256:
        raise CaptureError("runtime setup hash differs from the closure inventory")

    transport_sha256 = str(vendor.get("core_library_sha256", ""))
    if not re.fullmatch(r"[0-9a-f]{64}", transport_sha256):
        raise CaptureError("closure has no valid patched gz-transport13 hash")
    image_bridge_executable = Path(
        str(image_runtime.get("resolved_executable_path", ""))
    ).resolve(strict=True)
    image_bridge_sha256 = str(image_runtime.get("executable_sha256", ""))
    if image_runtime.get("bound") is not True or not re.fullmatch(
        r"[0-9a-f]{64}", image_bridge_sha256
    ):
        raise CaptureError("closure has no bound ros_gz_image executable identity")
    if sha256_path(image_bridge_executable) != image_bridge_sha256:
        raise CaptureError("live ros_gz_image executable bytes drifted from closure")

    return RuntimeContract(
        runtime_setup=runtime_setup,
        runtime_setup_sha256=runtime_setup_sha256,
        closure_manifest=closure_manifest,
        closure_manifest_sha256=sha256_path(closure_manifest),
        closure_sha256=closure_sha256,
        install_root=install_root,
        transport_sha256=transport_sha256,
        image_bridge_executable=image_bridge_executable,
        image_bridge_sha256=image_bridge_sha256,
    )


def parse_proc_maps(text: str) -> tuple[Path, ...]:
    """Return unique filesystem paths from Linux proc maps in first-seen order."""

    rows: list[Path] = []
    seen: set[str] = set()
    for line in text.splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) != 6:
            continue
        raw = fields[5]
        if not raw.startswith("/"):
            continue
        if raw.endswith(" (deleted)"):
            candidate = Path(raw[: -len(" (deleted)")])
            if _is_relevant_library(candidate.name):
                raise CaptureError(f"relevant mapped library was deleted: {raw}")
            continue
        if raw not in seen:
            seen.add(raw)
            rows.append(Path(raw))
    return tuple(rows)


def _is_relevant_library(name: str) -> bool:
    return bool(
        GZ_TRANSPORT_CORE.fullmatch(name)
        or PROTOBUF_RUNTIME.fullmatch(name)
        or ZMQ5_RUNTIME.fullmatch(name)
    )


def capture_process_snapshot(pid: int, proc_root: Path = Path("/proc")) -> ProcessSnapshot:
    if pid <= 0:
        raise CaptureError(f"PID must be positive: {pid}")
    process_root = proc_root / str(pid)
    executable_link = process_root / "exe"
    maps_path = process_root / "maps"
    try:
        executable = executable_link.resolve(strict=True)
    except OSError as error:
        raise CaptureError(f"cannot resolve /proc/{pid}/exe: {error}") from error
    if not executable.is_file():
        raise CaptureError(f"/proc/{pid}/exe is not a regular file: {executable}")
    try:
        maps_bytes = maps_path.read_bytes()
    except OSError as error:
        raise CaptureError(f"cannot read /proc/{pid}/maps: {error}") from error
    mapped_paths = parse_proc_maps(maps_bytes.decode("utf-8", errors="replace"))
    return ProcessSnapshot(
        pid=pid,
        executable=executable,
        executable_sha256=sha256_path(executable),
        maps_sha256=hashlib.sha256(maps_bytes).hexdigest(),
        mapped_paths=mapped_paths,
    )


def _resolve_matches(
    snapshot: ProcessSnapshot, expression: re.Pattern[str]
) -> tuple[Path, ...]:
    matches: list[Path] = []
    seen: set[Path] = set()
    for raw_path in snapshot.mapped_paths:
        if not expression.fullmatch(raw_path.name):
            continue
        try:
            resolved = raw_path.resolve(strict=True)
        except OSError as error:
            raise CaptureError(
                f"PID {snapshot.pid} mapped library cannot be resolved: {raw_path}: {error}"
            ) from error
        if not resolved.is_file():
            raise CaptureError(
                f"PID {snapshot.pid} mapped library is not a file: {resolved}"
            )
        if resolved not in seen:
            seen.add(resolved)
            matches.append(resolved)
    return tuple(matches)


def _inside_any(path: Path, roots: Sequence[Path]) -> bool:
    for root in roots:
        try:
            path.relative_to(root)
        except ValueError:
            continue
        return True
    return False


def evaluate_process_maps(
    contract: RuntimeContract,
    gazebo: ProcessSnapshot,
    image_bridge: ProcessSnapshot,
    *,
    system_library_roots: Iterable[Path] = (Path("/usr/lib"), Path("/lib")),
) -> dict[str, object]:
    roots: list[Path] = []
    for root in system_library_roots:
        try:
            resolved = root.resolve(strict=True)
        except OSError as error:
            raise CaptureError(f"cannot resolve system library root {root}: {error}") from error
        if resolved not in roots:
            roots.append(resolved)
    if not roots:
        raise CaptureError("no system library roots were supplied")

    blockers: list[dict[str, object]] = []
    process_rows: dict[str, dict[str, object]] = {}
    selected: dict[str, dict[str, tuple[Path, ...]]] = {}
    expected_transport_root = (contract.install_root / "lib").resolve(strict=True)

    for role, snapshot in (("gazebo", gazebo), ("image_bridge", image_bridge)):
        transport = _resolve_matches(snapshot, GZ_TRANSPORT_CORE)
        protobuf = _resolve_matches(snapshot, PROTOBUF_RUNTIME)
        zmq = _resolve_matches(snapshot, ZMQ5_RUNTIME)
        forbidden = sorted(
            {
                str(path)
                for path in snapshot.mapped_paths
                if any(
                    fragment in path.as_posix()
                    for fragment in FORBIDDEN_RUNTIME_FRAGMENTS
                )
            }
        )
        selected[role] = {
            "transport": transport,
            "protobuf": protobuf,
            "zmq": zmq,
        }

        if len(transport) != 1:
            blockers.append(
                {
                    "code": "TRANSPORT_MAPPING_COUNT",
                    "process": role,
                    "observed": len(transport),
                    "expected": 1,
                }
            )
        if len(protobuf) != 1:
            blockers.append(
                {
                    "code": "PROTOBUF_MAPPING_COUNT",
                    "process": role,
                    "observed": len(protobuf),
                    "expected": 1,
                }
            )
        if len(zmq) != 1:
            blockers.append(
                {
                    "code": "ZMQ5_MAPPING_COUNT",
                    "process": role,
                    "observed": len(zmq),
                    "expected": 1,
                }
            )
        if forbidden:
            blockers.append(
                {
                    "code": "FORBIDDEN_VENDOR_MAPPING",
                    "process": role,
                    "paths": forbidden,
                }
            )

        transport_hashes = {
            str(path): sha256_path(path) for path in transport
        }
        protobuf_hashes = {str(path): sha256_path(path) for path in protobuf}
        zmq_hashes = {str(path): sha256_path(path) for path in zmq}
        if len(transport) == 1:
            if transport[0].parent != expected_transport_root:
                blockers.append(
                    {
                        "code": "TRANSPORT_OUTSIDE_FROZEN_INSTALL",
                        "process": role,
                        "path": str(transport[0]),
                    }
                )
            if transport_hashes[str(transport[0])] != contract.transport_sha256:
                blockers.append(
                    {
                        "code": "TRANSPORT_HASH_MISMATCH",
                        "process": role,
                        "path": str(transport[0]),
                    }
                )
        for family, paths in (("protobuf", protobuf), ("zmq", zmq)):
            if len(paths) == 1 and not _inside_any(paths[0], roots):
                blockers.append(
                    {
                        "code": f"{family.upper()}_OUTSIDE_SYSTEM_RUNTIME",
                        "process": role,
                        "path": str(paths[0]),
                    }
                )

        process_rows[role] = {
            "pid": snapshot.pid,
            "executable": {
                "path": str(snapshot.executable),
                "sha256": snapshot.executable_sha256,
            },
            "proc_maps_sha256": snapshot.maps_sha256,
            "mapped_library_counts": {
                "gz_transport13_core": len(transport),
                "protobuf_3_21_12": len(protobuf),
                "zmq5": len(zmq),
            },
            "mapped_libraries": {
                "gz_transport13_core": transport_hashes,
                "protobuf_3_21_12": protobuf_hashes,
                "zmq5": zmq_hashes,
            },
            "forbidden_vendor_paths": forbidden,
        }

    if image_bridge.executable != contract.image_bridge_executable:
        blockers.append(
            {
                "code": "IMAGE_BRIDGE_EXECUTABLE_PATH_MISMATCH",
                "observed": str(image_bridge.executable),
                "expected": str(contract.image_bridge_executable),
            }
        )
    if image_bridge.executable_sha256 != contract.image_bridge_sha256:
        blockers.append(
            {
                "code": "IMAGE_BRIDGE_EXECUTABLE_HASH_MISMATCH",
                "observed": image_bridge.executable_sha256,
                "expected": contract.image_bridge_sha256,
            }
        )

    cross_checks: dict[str, bool] = {}
    for family in ("transport", "protobuf", "zmq"):
        gazebo_paths = selected["gazebo"][family]
        bridge_paths = selected["image_bridge"][family]
        same = (
            len(gazebo_paths) == 1
            and len(bridge_paths) == 1
            and gazebo_paths[0] == bridge_paths[0]
        )
        cross_checks[f"same_{family}_file"] = same
        if not same:
            blockers.append(
                {
                    "code": f"CROSS_PROCESS_{family.upper()}_DIVERGENCE",
                    "gazebo": [str(path) for path in gazebo_paths],
                    "image_bridge": [str(path) for path in bridge_paths],
                }
            )

    passed = not blockers
    return {
        "schema_version": 1,
        "report_id": REPORT_ID,
        "status": (
            "FORMAL_TRANSPORT_PROCESS_MAPS_BOUND"
            if passed
            else "FORMAL_TRANSPORT_PROCESS_MAPS_REJECTED"
        ),
        "passed": passed,
        "runtime": {
            "setup": str(contract.runtime_setup),
            "setup_sha256": contract.runtime_setup_sha256,
            "install_root": str(contract.install_root),
            "closure_manifest": str(contract.closure_manifest),
            "closure_manifest_sha256": contract.closure_manifest_sha256,
            "closure_sha256": contract.closure_sha256,
            "expected_gz_transport13_sha256": contract.transport_sha256,
        },
        "system_library_roots": [str(path) for path in roots],
        "processes": process_rows,
        "cross_process_checks": cross_checks,
        "blockers": blockers,
    }


def write_report_exclusive(path: Path, report: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open("xb") as stream:
            stream.write(
                (json.dumps(report, indent=2, sort_keys=True) + "\n").encode("utf-8")
            )
    except FileExistsError as error:
        raise CaptureError(f"refusing stale output: {path}") from error


def _failure_report(error: Exception, args: argparse.Namespace) -> dict[str, object]:
    return {
        "schema_version": 1,
        "report_id": REPORT_ID,
        "status": "FORMAL_TRANSPORT_PROCESS_MAPS_CAPTURE_FAILED",
        "passed": False,
        "error_type": type(error).__name__,
        "error": str(error),
        "inputs": {
            "gazebo_pid": args.gazebo_pid,
            "image_bridge_pid": args.image_bridge_pid,
            "runtime_setup": str(args.runtime_setup),
            "closure_manifest": str(args.closure_manifest),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gazebo-pid", type=int, required=True)
    parser.add_argument("--image-bridge-pid", type=int, required=True)
    parser.add_argument("--runtime-setup", type=Path, required=True)
    parser.add_argument("--closure-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or args.output.is_symlink():
        print(f"refusing stale output: {args.output}", file=sys.stderr)
        return 2
    try:
        contract = load_runtime_contract(args.runtime_setup, args.closure_manifest)
        gazebo = capture_process_snapshot(args.gazebo_pid)
        image_bridge = capture_process_snapshot(args.image_bridge_pid)
        report = evaluate_process_maps(contract, gazebo, image_bridge)
    except (CaptureError, OSError, ValueError) as error:
        report = _failure_report(error, args)
    try:
        write_report_exclusive(args.output, report)
    except CaptureError as error:
        print(str(error), file=sys.stderr)
        return 2
    print(report["status"])
    return 0 if report["passed"] is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
