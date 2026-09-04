#!/usr/bin/env python3
"""Record and verify the immutable runtime closure for final acceptance.

The final acceptance runtime must be one real, merged colcon install.  A
``--symlink-install`` tree is deliberately rejected: every directory and file
under ``install/`` must be a regular on-disk entry.  The closure binds all
project packages used by the 31-step orchestrator, their source and installed
runtime bytes, build markers, compiled Gazebo plugins, and the external model
and ONNX Runtime assets used by the perception gate.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any, Iterable, Mapping, Sequence

from validate_formal_windows_cold_gate_evidence import EvidenceError, validate_evidence
from formal_native_linux_cold_start_evidence import (
    BOUND_REPORT_ID as NATIVE_LINUX_COLD_START_REPORT_ID,
    NativeEvidenceError,
    validate_bound as validate_native_linux_cold_start,
)


FINAL_RUNTIME_PACKAGES: tuple[str, ...] = (
    "sanitation_active_cleaning",
    "sanitation_campus_scenario",
    "sanitation_coverage",
    "sanitation_formal_campus_integration",
    "sanitation_gazebo_auxiliary",
    "sanitation_gazebo_control",
    "sanitation_localization",
    "sanitation_manipulation",
    "sanitation_navigation",
    "sanitation_perception",
    "sanitation_perception_interfaces",
    "sanitation_power_system",
    "sanitation_product_demo_integration",
    "sanitation_safety",
    "sanitation_service_acceptance",
    "sanitation_vehicle_description",
)

GAZEBO_PLUGIN_LIBRARIES: tuple[str, ...] = (
    "libA300DrivetrainPlantSystem.so",
    "libCleaningActuatorMotorSystem.so",
    "libDryBinMonitorSystem.so",
    "libDynamicPayloadSystem.so",
    "libFormalAuxiliaryVisualSystem.so",
    "libGripperContactGateSystem.so",
    "libGripperMimicEffortSystem.so",
    "libGroundDirtCleaningSystem.so",
    "libSanitationMissionControl.so",
    "libServiceDoorSystem.so",
    "libSqueegeeComplianceSystem.so",
    "libWaterRecoverySystem.so",
)

# Gazebo's physics backend does not implement the Robotiq URDF mimic
# constraints.  Name the content contract, not a one-off build-attempt number,
# so a later clean rebuild keeps the same identity only while these semantics
# and their byte-level closure remain unchanged.
FORMAL_RUNTIME_CONTRACT_REVISION = "gripper_effort_mimic_nvidia_egl_runtime_v1"
GRIPPER_MIMIC_SOURCE_PATHS: tuple[str, ...] = (
    "starter_ws/src/sanitation_vehicle_description/urdf/"
    "formal_competition_vehicle.urdf.xacro",
    "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/"
    "manipulator_stack.xacro",
    "starter_ws/src/sanitation_gazebo_control/CMakeLists.txt",
    "starter_ws/src/sanitation_gazebo_control/package.xml",
    "starter_ws/src/sanitation_gazebo_control/include/"
    "sanitation_gazebo_control/GripperMimicEffortCore.hh",
    "starter_ws/src/sanitation_gazebo_control/src/GripperMimicEffortSystem.cc",
)
GRIPPER_MIMIC_INSTALL_BINDINGS: Mapping[str, str] = {
    GRIPPER_MIMIC_SOURCE_PATHS[0]: (
        "share/sanitation_vehicle_description/urdf/"
        "formal_competition_vehicle.urdf.xacro"
    ),
    GRIPPER_MIMIC_SOURCE_PATHS[1]: (
        "share/sanitation_vehicle_description/urdf/high_fidelity/"
        "manipulator_stack.xacro"
    ),
    GRIPPER_MIMIC_SOURCE_PATHS[3]: "share/sanitation_gazebo_control/package.xml",
    GRIPPER_MIMIC_SOURCE_PATHS[4]: (
        "include/sanitation_gazebo_control/GripperMimicEffortCore.hh"
    ),
}
GRIPPER_MIMIC_PACKAGE = "sanitation_gazebo_control"
GRIPPER_MIMIC_PLUGIN = "libGripperMimicEffortSystem.so"

SOURCE_SHARE_DIRECTORIES: tuple[str, ...] = (
    "action",
    "config",
    "launch",
    "meshes",
    "msg",
    "srv",
    "urdf",
    "worlds",
)

IGNORED_DIRECTORY_NAMES = {".git", ".pytest_cache", "__pycache__"}
IGNORED_FILE_SUFFIXES = {".pyc", ".pyo"}

SIDE_BRUSH_INSTALLED_XACRO = Path(
    "share/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro"
)
SIDE_BRUSH_SURFACE_PREFLIGHT = Path("side_brush_sdf_surface_preflight.json")
INSTALL_SYMLINK_REPORT = Path("INSTALL_SYMLINKS.txt")
WINDOWS_COLD_START_EVIDENCE = Path("formal_windows_cold_start_evidence.json")
ROS_GZ_IMAGE_PACKAGE = "ros_gz_image"
ROS_GZ_IMAGE_DEBIAN_PACKAGE = "ros-jazzy-ros-gz-image"
ROS_GZ_IMAGE_EXPECTED_PREFIX = Path("/opt/ros/jazzy")
ROS_GZ_IMAGE_EXECUTABLE_RELATIVE = Path("lib/ros_gz_image/image_bridge")
ROS_GZ_IMAGE_PACKAGE_XML_RELATIVE = Path("share/ros_gz_image/package.xml")
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")

TYPED_CLEANING_TELEMETRY_SOURCE_PATHS: tuple[str, ...] = (
    "scripts/formal_cleaning_motor_telemetry.py",
    "starter_ws/src/sanitation_gazebo_control/include/sanitation_gazebo_control/CleaningActuatorMotorCore.hh",
    "starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorMotorCore.cc",
    "starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorMotorSystem.cc",
    "starter_ws/src/sanitation_gazebo_control/src/CleaningActuatorVectorBridge.cc",
    "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py",
    "config/high_fidelity_vehicle/cleaning_actuator_motor_realism_contract.yaml",
    "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml",
)

GZ_TRANSPORT13_VENDOR_SOURCE_PATHS: tuple[str, ...] = (
    "patches/upstream/gz_transport13/README.md",
    "patches/upstream/gz_transport13/manifest.json",
    "patches/upstream/gz_transport13/0001-publish-retry-current-frame-on-eintr.patch",
    "scripts/build_gz_transport13_eintr_vendor.sh",
    "scripts/validate_gz_transport13_eintr_vendor.py",
    "scripts/formal_dynamic_dependencies.sh",
    "scripts/proot_glibc_compat.c",
)
GZ_TRANSPORT13_VENDOR_BUILD_REPORT = Path(
    "gz_transport13_eintr_vendor_build_report.json"
)
GZ_TRANSPORT13_RUNTIME_BINDING_REPORT = Path(
    "gz_transport13_eintr_runtime_binding_report.json"
)
GZ_TRANSPORT13_PROTOBUF_BINDING_REPORT = Path(
    "vendor/gz_transport13_eintr_build/protobuf_binding.json"
)
GZ_TRANSPORT13_CORE_LIBRARIES: tuple[Path, ...] = (
    Path("lib/libgz-transport13.so"),
    Path("lib/libgz-transport13.so.13"),
    Path("lib/libgz-transport13.so.13.5.0"),
)
NVIDIA_EGL_VENDOR_JSON = Path("egl_vendor.d/10_nvidia.json")
NVIDIA_EGL_LIBRARY = "libEGL_nvidia.so.0"
NVIDIA_EGL_ENVIRONMENT = {
    "__EGL_VENDOR_LIBRARY_FILENAMES": None,
    "EGL_PLATFORM": "surfaceless",
}


class ClosureError(RuntimeError):
    """A fail-closed runtime-closure violation."""


def _windows_cold_start_evidence_identity(runtime_ws: Path) -> dict[str, Any]:
    path = runtime_ws / WINDOWS_COLD_START_EVIDENCE
    _assert_regular(path, "bound Windows cold-start evidence")
    raw_payload = _read_json(path)
    if raw_payload.get("report_id") == NATIVE_LINUX_COLD_START_REPORT_ID:
        try:
            payload = validate_native_linux_cold_start(path, runtime_ws)
        except NativeEvidenceError as exc:
            raise ClosureError(
                f"invalid bound native-Linux cold-start evidence: {exc}"
            ) from exc
        return {
            "bound": True,
            "mode": "native_linux_not_wsl",
            "path": WINDOWS_COLD_START_EVIDENCE.as_posix(),
            "sha256": _sha256(path),
            "report_id": payload["report_id"],
            "recorded_epoch_ns": payload["recorded_epoch_ns"],
            "kernel_osrelease": payload["kernel_osrelease"],
            "source": payload["source"],
        }
    try:
        payload = validate_evidence(path, enforce_freshness=False)
    except EvidenceError as exc:
        raise ClosureError(f"invalid bound Windows cold-start evidence: {exc}") from exc
    thresholds = payload["thresholds_bytes"]
    sample = payload["sample"]
    return {
        "bound": True,
        "path": WINDOWS_COLD_START_EVIDENCE.as_posix(),
        "sha256": _sha256(path),
        "report_id": payload["report_id"],
        "recorded_epoch_ns": payload["recorded_epoch_ns"],
        "min_commit_available_bytes": thresholds["min_commit_available"],
        "max_docker_private_bytes": thresholds["max_docker_private"],
        "sample_commit_available_bytes": sample["commit_available_bytes"],
        "sample_docker_private_bytes": sample["docker_private_bytes"],
        "sample_vmmem_wsl_private_bytes": sample["vmmem_wsl_private_bytes"],
    }


def _utc_iso(epoch_ns: int) -> str:
    return datetime.fromtimestamp(epoch_ns / 1e9, tz=timezone.utc).isoformat()


def _advise_drop_cache(stream: Any) -> None:
    """Release read-only hash pages after use when the host supports it.

    Final closure verification hashes the complete source and install trees
    before every Gazebo step.  On WSL those clean pages otherwise inflate the
    VM's host commit even though the verifier no longer needs them.
    """

    fadvise = getattr(os, "posix_fadvise", None)
    dontneed = getattr(os, "POSIX_FADV_DONTNEED", None)
    if fadvise is None or dontneed is None:
        return
    try:
        fadvise(stream.fileno(), 0, 0, dontneed)
    except (OSError, ValueError):
        # Cache advice is an optimization only; hash correctness remains the
        # fail-closed contract on platforms/filesystems that reject it.
        return


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
        # Only a fully consumed file is eligible for an advisory cache drop.
        _advise_drop_cache(stream)
    return digest.hexdigest()


def _json_digest(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ClosureError(f"cannot read JSON object {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ClosureError(f"JSON root must be an object: {path}")
    return value


def _atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_suffix(path.suffix + f".pending.{os.getpid()}")
    pending.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    pending.replace(path)


def _assert_regular(path: Path, label: str) -> None:
    if path.is_symlink():
        raise ClosureError(f"{label} is a symbolic link: {path}")
    if not path.is_file():
        raise ClosureError(f"{label} is missing or not a regular file: {path}")


def _identity_command(arguments: Sequence[str], label: str) -> str:
    try:
        result = subprocess.run(
            list(arguments),
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise ClosureError(f"cannot query {label}: {exc}") from exc
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        raise ClosureError(
            f"cannot query {label}: exit={result.returncode} detail={detail!r}"
        )
    output = result.stdout.strip()
    if not output:
        raise ClosureError(f"cannot query {label}: empty output")
    return output


def _ros_gz_image_system_identity() -> dict[str, Any]:
    """Bind the exact system image bridge selected by the Ament prefix order."""

    raw_prefixes = os.environ.get("AMENT_PREFIX_PATH", "")
    if not raw_prefixes:
        raise ClosureError(
            "AMENT_PREFIX_PATH is empty while resolving the system ros_gz_image package"
        )
    selected_prefix: Path | None = None
    selected_marker: Path | None = None
    for raw_prefix in raw_prefixes.split(os.pathsep):
        if not raw_prefix:
            continue
        prefix = Path(raw_prefix)
        marker = (
            prefix
            / "share/ament_index/resource_index/packages"
            / ROS_GZ_IMAGE_PACKAGE
        )
        if marker.is_file():
            _assert_regular(marker, "ros_gz_image Ament resource marker")
            try:
                selected_prefix = prefix.resolve(strict=True)
            except OSError as exc:
                raise ClosureError(
                    f"cannot resolve ros_gz_image Ament prefix {prefix}: {exc}"
                ) from exc
            selected_marker = marker.resolve(strict=True)
            break
    if selected_prefix is None or selected_marker is None:
        raise ClosureError(
            "ros_gz_image is absent from the active AMENT_PREFIX_PATH"
        )
    try:
        expected_prefix = ROS_GZ_IMAGE_EXPECTED_PREFIX.resolve(strict=True)
    except OSError as exc:
        raise ClosureError(
            f"expected ROS Jazzy prefix cannot be resolved: {ROS_GZ_IMAGE_EXPECTED_PREFIX}: {exc}"
        ) from exc
    if selected_prefix != expected_prefix:
        raise ClosureError(
            "ros_gz_image resolves outside the pinned system ROS prefix: "
            f"selected={selected_prefix} expected={expected_prefix}"
        )

    declared_executable = selected_prefix / ROS_GZ_IMAGE_EXECUTABLE_RELATIVE
    if not declared_executable.exists():
        raise ClosureError(
            f"system ros_gz_image executable is missing: {declared_executable}"
        )
    try:
        resolved_executable = declared_executable.resolve(strict=True)
    except OSError as exc:
        raise ClosureError(
            f"cannot resolve system ros_gz_image executable {declared_executable}: {exc}"
        ) from exc
    _assert_regular(resolved_executable, "resolved system ros_gz_image executable")
    if not os.access(resolved_executable, os.X_OK):
        raise ClosureError(
            f"system ros_gz_image executable is not executable: {resolved_executable}"
        )
    try:
        resolved_executable.relative_to(selected_prefix)
    except ValueError as exc:
        raise ClosureError(
            "system ros_gz_image executable resolves outside its Ament prefix: "
            f"{resolved_executable}"
        ) from exc

    package_xml = selected_prefix / ROS_GZ_IMAGE_PACKAGE_XML_RELATIVE
    _assert_regular(package_xml, "system ros_gz_image package.xml")
    try:
        package_root = ET.parse(package_xml).getroot()
    except (OSError, ET.ParseError) as exc:
        raise ClosureError(f"cannot parse system ros_gz_image package.xml: {exc}") from exc
    package_name = (package_root.findtext("name") or "").strip()
    ros_package_version = (package_root.findtext("version") or "").strip()
    if package_name != ROS_GZ_IMAGE_PACKAGE or not ros_package_version:
        raise ClosureError(
            "system ros_gz_image package.xml has invalid name/version: "
            f"name={package_name!r} version={ros_package_version!r}"
        )

    ownership_output = _identity_command(
        ["dpkg-query", "-S", str(declared_executable)],
        "ros_gz_image Debian file ownership",
    )
    owners: set[str] = set()
    for line in ownership_output.splitlines():
        if ": " not in line:
            raise ClosureError(
                f"invalid dpkg-query ownership row for ros_gz_image: {line!r}"
            )
        owner = line.rsplit(": ", 1)[0].strip()
        owners.add(owner.split(":", 1)[0])
    if owners != {ROS_GZ_IMAGE_DEBIAN_PACKAGE}:
        raise ClosureError(
            "system ros_gz_image executable has unexpected Debian ownership: "
            f"{sorted(owners)}"
        )

    package_output = _identity_command(
        [
            "dpkg-query",
            "-W",
            "-f=${db:Status-Abbrev}\\t${binary:Package}\\t${Version}\\t${Architecture}\\n",
            ROS_GZ_IMAGE_DEBIAN_PACKAGE,
        ],
        "ros_gz_image Debian package identity",
    )
    package_rows = [row for row in package_output.splitlines() if row.strip()]
    if len(package_rows) != 1:
        raise ClosureError(
            "ros_gz_image Debian package identity is ambiguous: "
            f"{package_rows}"
        )
    fields = package_rows[0].split("\t")
    if len(fields) != 4:
        raise ClosureError(
            f"invalid ros_gz_image Debian package identity row: {package_rows[0]!r}"
        )
    status, binary_package, debian_version, architecture = (
        field.strip() for field in fields
    )
    normalized_binary_package = binary_package.split(":", 1)[0]
    if (
        not status.startswith("ii")
        or normalized_binary_package != ROS_GZ_IMAGE_DEBIAN_PACKAGE
        or not debian_version
        or not architecture
    ):
        raise ClosureError(
            "ros_gz_image Debian package is not the expected installed package: "
            f"status={status!r} package={binary_package!r} "
            f"version={debian_version!r} architecture={architecture!r}"
        )

    executable_stat = resolved_executable.stat()
    return {
        "bound": True,
        "ament_package": ROS_GZ_IMAGE_PACKAGE,
        "ament_prefix": str(selected_prefix),
        "ament_resource_marker": str(selected_marker),
        "declared_executable_path": str(declared_executable),
        "resolved_executable_path": str(resolved_executable),
        "executable_sha256": _sha256(resolved_executable),
        "executable_size_bytes": executable_stat.st_size,
        "executable_mode_octal": oct(executable_stat.st_mode & 0o7777),
        "package_xml_path": str(package_xml),
        "package_xml_sha256": _sha256(package_xml),
        "ros_package_version": ros_package_version,
        "debian_package": ROS_GZ_IMAGE_DEBIAN_PACKAGE,
        "debian_binary_package": binary_package,
        "debian_status": status,
        "debian_version": debian_version,
        "debian_architecture": architecture,
    }


def _regular_files(root: Path, label: str) -> list[Path]:
    """Walk without following links and reject every link or special entry."""

    if root.is_symlink():
        raise ClosureError(f"{label} root is a symbolic link: {root}")
    if not root.is_dir():
        raise ClosureError(f"{label} root is missing or not a directory: {root}")
    files: list[Path] = []
    stack = [root]
    while stack:
        directory = stack.pop()
        try:
            entries = sorted(os.scandir(directory), key=lambda row: row.name)
        except OSError as exc:
            raise ClosureError(f"cannot scan {label} directory {directory}: {exc}") from exc
        for entry in entries:
            path = Path(entry.path)
            if entry.is_symlink():
                raise ClosureError(f"{label} contains a symbolic link: {path}")
            if entry.is_dir(follow_symlinks=False):
                if entry.name not in IGNORED_DIRECTORY_NAMES:
                    stack.append(path)
                continue
            if entry.is_file(follow_symlinks=False):
                if path.suffix not in IGNORED_FILE_SUFFIXES:
                    files.append(path)
                continue
            raise ClosureError(f"{label} contains a non-regular entry: {path}")
    return sorted(files)


def _inventory(files: Iterable[Path], base: Path) -> dict[str, dict[str, Any]]:
    rows: dict[str, dict[str, Any]] = {}
    for path in sorted(set(files)):
        _assert_regular(path, "closure file")
        try:
            relative = path.relative_to(base).as_posix()
        except ValueError as exc:
            raise ClosureError(f"closure file escapes its declared root: {path}") from exc
        stat = path.stat()
        rows[relative] = {
            "sha256": _sha256(path),
            "size_bytes": stat.st_size,
        }
    if not rows:
        raise ClosureError(f"runtime closure inventory is empty under {base}")
    return rows


def _source_inventory(
    repository_root: Path, packages: Sequence[str]
) -> dict[str, dict[str, Any]]:
    roots = [
        repository_root / "scripts",
        repository_root / "config/high_fidelity_vehicle",
        repository_root / "patches/upstream/gz_transport13",
    ]
    roots.extend(repository_root / "starter_ws/src" / package for package in packages)
    files: list[Path] = []
    for root in roots:
        files.extend(_regular_files(root, "final runtime source closure"))
    inventory = _inventory(files, repository_root)
    required = set(TYPED_CLEANING_TELEMETRY_SOURCE_PATHS) | set(
        GZ_TRANSPORT13_VENDOR_SOURCE_PATHS
    )
    missing = sorted(required - set(inventory))
    if missing:
        raise ClosureError(
            "required final runtime source closure is incomplete: "
            + ", ".join(missing)
        )
    return inventory


def _frozen_source_inventory(
    repository_root: Path, runtime_ws: Path, packages: Sequence[str]
) -> dict[str, dict[str, Any]]:
    repository_source_root = repository_root / "starter_ws/src"
    frozen_source_root = runtime_ws / "src"
    repository_files: list[Path] = []
    frozen_files: list[Path] = []
    for package in packages:
        repository_files.extend(
            _regular_files(
                repository_source_root / package,
                f"repository source package {package}",
            )
        )
        frozen_files.extend(
            _regular_files(
                frozen_source_root / package,
                f"frozen source package {package}",
            )
        )
    repository_inventory = _inventory(repository_files, repository_source_root)
    frozen_inventory = _inventory(frozen_files, frozen_source_root)
    if frozen_inventory != repository_inventory:
        raise ClosureError(
            "frozen runtime src differs from repository starter_ws/src"
        )
    return frozen_inventory


def _gz_transport13_vendor_identity(
    repository_root: Path,
    runtime_ws: Path,
    install_root: Path,
    source_inventory: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    manifest_path = repository_root / "patches/upstream/gz_transport13/manifest.json"
    patch_path = (
        repository_root
        / "patches/upstream/gz_transport13/0001-publish-retry-current-frame-on-eintr.patch"
    )
    build_report_path = runtime_ws / GZ_TRANSPORT13_VENDOR_BUILD_REPORT
    binding_report_path = runtime_ws / GZ_TRANSPORT13_RUNTIME_BINDING_REPORT
    protobuf_report_path = runtime_ws / GZ_TRANSPORT13_PROTOBUF_BINDING_REPORT
    for path, label in (
        (manifest_path, "gz-transport13 vendor manifest"),
        (patch_path, "gz-transport13 vendor patch"),
        (build_report_path, "gz-transport13 vendor build report"),
        (binding_report_path, "gz-transport13 runtime binding report"),
        (protobuf_report_path, "gz-transport13 Protobuf binding report"),
    ):
        _assert_regular(path, label)

    manifest = _read_json(manifest_path)
    if (
        manifest.get("schema_version") != 1
        or manifest.get("component") != "gz-transport13"
        or manifest.get("upstream_tag") != "gz-transport13_13.5.0"
        or manifest.get("eintr_retry_limit") != 3
    ):
        raise ClosureError("gz-transport13 vendor manifest identity drifted")
    if manifest.get("patch_sha256") != _sha256(patch_path):
        raise ClosureError("gz-transport13 vendor patch hash mismatch")

    alias_hashes: dict[str, str] = {}
    for relative in GZ_TRANSPORT13_CORE_LIBRARIES:
        path = install_root / relative
        _assert_regular(path, f"patched gz-transport13 library {relative}")
        alias_hashes[relative.as_posix()] = _sha256(path)
    if len(set(alias_hashes.values())) != 1:
        raise ClosureError("patched gz-transport13 regular aliases differ")
    library_hash = next(iter(alias_hashes.values()))

    protobuf_report = _read_json(protobuf_report_path)
    protobuf_resolved = protobuf_report.get("resolved")
    if (
        protobuf_report.get("schema_version") != 1
        or protobuf_report.get("status")
        != "SYSTEM_PROTOBUF_3_21_12_BINDING_PASSED"
        or protobuf_report.get("passed") is not True
        or protobuf_report.get("protobuf_version") != "3.21.12"
        or protobuf_report.get("protobuf_header_version") != 3021012
        or protobuf_report.get("config_mode_protobuf_disabled") is not True
        or protobuf_report.get("forbidden_prefix")
        != "/opt/ros/jazzy/opt/ortools_vendor"
        or not isinstance(protobuf_report.get("compile_command_count"), int)
        or isinstance(protobuf_report.get("compile_command_count"), bool)
        or protobuf_report.get("compile_command_count", 0) < 1
        or not isinstance(protobuf_resolved, dict)
        or set(protobuf_resolved)
        != {
            "Protobuf_INCLUDE_DIR",
            "Protobuf_LIBRARY_RELEASE",
            "Protobuf_LITE_LIBRARY_RELEASE",
            "Protobuf_PROTOC_LIBRARY_RELEASE",
            "Protobuf_PROTOC_EXECUTABLE",
        }
    ):
        raise ClosureError("gz-transport13 Protobuf binding report drifted")
    forbidden_protobuf_prefix = "/opt/ros/jazzy/opt/ortools_vendor"
    if (
        PurePosixPath(str(protobuf_resolved["Protobuf_INCLUDE_DIR"])).as_posix()
        != "/usr/include"
        or PurePosixPath(str(protobuf_resolved["Protobuf_PROTOC_EXECUTABLE"])).as_posix()
        != "/usr/bin/protoc"
    ):
        raise ClosureError("gz-transport13 Protobuf binding is not system-provided")
    for key, library_name in (
        ("Protobuf_LIBRARY_RELEASE", "libprotobuf.so.32.0.12"),
        ("Protobuf_LITE_LIBRARY_RELEASE", "libprotobuf-lite.so.32.0.12"),
        ("Protobuf_PROTOC_LIBRARY_RELEASE", "libprotoc.so.32.0.12"),
    ):
        resolved = PurePosixPath(str(protobuf_resolved[key]))
        if (
            resolved.name != library_name
            or not resolved.is_absolute()
            or forbidden_protobuf_prefix in resolved.as_posix()
        ):
            raise ClosureError(f"gz-transport13 {key} is not pinned system Protobuf")
    protobuf_report_sha256 = _sha256(protobuf_report_path)

    def validate_report(path: Path, *, runtime_binding: bool) -> dict[str, Any]:
        report = _read_json(path)
        if (
            report.get("report_id") != "tzcup_gz_transport13_eintr_vendor_v1"
            or report.get("status")
            != "GZ_TRANSPORT13_EINTR_VENDOR_CONTRACT_PASSED"
            or report.get("passed") is not True
            or report.get("manifest") != manifest
            or report.get("manifest_sha256") != _sha256(manifest_path)
        ):
            raise ClosureError(f"gz-transport13 vendor report is not passing: {path}")
        install = report.get("install")
        source = report.get("source")
        if (
            not isinstance(source, dict)
            or source.get("commit") != manifest.get("upstream_commit")
            or source.get("tree") != manifest.get("upstream_tree")
            or source.get("node_shared_sha256")
            != manifest.get("patched_node_shared_sha256")
        ):
            raise ClosureError(f"gz-transport13 vendor source proof drifted: {path}")
        if not isinstance(install, dict):
            raise ClosureError(f"gz-transport13 vendor report has no install proof: {path}")
        if Path(str(install.get("install_prefix", ""))).resolve() != install_root.resolve():
            raise ClosureError("gz-transport13 report targets another install prefix")
        if install.get("core_library_sha256") != library_hash:
            raise ClosureError("gz-transport13 report core library hash mismatch")
        if (
            install.get("protobuf_needed") != "libprotobuf.so.32"
            or not isinstance(install.get("dynamic_needed"), list)
            or [
                name
                for name in install["dynamic_needed"]
                if str(name).lower().startswith("libprotobuf")
            ]
            != ["libprotobuf.so.32"]
            or install.get("forbidden_vendor_needed") != []
            or install.get("obvious_static_ortools_abseil_symbol_count") != 0
        ):
            raise ClosureError("gz-transport13 report lacks clean dynamic Protobuf proof")
        protobuf_runtime_path = PurePosixPath(
            str(install.get("protobuf_runtime_path", ""))
        )
        if (
            not protobuf_runtime_path.is_absolute()
            or forbidden_protobuf_prefix in protobuf_runtime_path.as_posix()
        ):
            raise ClosureError("gz-transport13 resolved a non-system Protobuf runtime")
        protobuf_binding = report.get("protobuf_binding")
        if (
            not isinstance(protobuf_binding, dict)
            or Path(str(protobuf_binding.get("path", ""))).resolve()
            != protobuf_report_path.resolve()
            or protobuf_binding.get("sha256") != protobuf_report_sha256
            or protobuf_binding.get("schema_version") != 1
            or protobuf_binding.get("status") != protobuf_report["status"]
            or protobuf_binding.get("protobuf_version")
            != protobuf_report["protobuf_version"]
            or protobuf_binding.get("protobuf_header_version")
            != protobuf_report["protobuf_header_version"]
            or protobuf_binding.get("config_mode_protobuf_disabled") is not True
            or protobuf_binding.get("forbidden_prefix")
            != protobuf_report["forbidden_prefix"]
            or protobuf_binding.get("compile_command_count")
            != protobuf_report["compile_command_count"]
            or protobuf_binding.get("resolved") != protobuf_resolved
        ):
            raise ClosureError(
                "gz-transport13 report does not bind the actual Protobuf report"
            )
        if report.get("parallel_workers") not in {1, 2}:
            raise ClosureError("gz-transport13 report lacks bounded build concurrency")
        if not runtime_binding:
            return report
        activation = report.get("runtime_activation")
        if not isinstance(activation, dict):
            raise ClosureError("gz-transport13 runtime activation proof is missing")
        expected_lib_dir = (install_root / "lib").resolve()
        expected_alias = (install_root / "lib/libgz-transport13.so.13").resolve()
        if Path(str(activation.get("ld_library_path_first", ""))).resolve() != expected_lib_dir:
            raise ClosureError("patched gz-transport13 was not first in LD_LIBRARY_PATH")
        if Path(str(activation.get("patched_soname_alias", ""))).resolve() != expected_alias:
            raise ClosureError("gz-transport13 activation names another SONAME alias")
        if activation.get("patched_soname_alias_sha256") != library_hash:
            raise ClosureError("gz-transport13 activation library hash mismatch")
        consumers = activation.get("consumers")
        if (
            not isinstance(consumers, dict)
            or activation.get("consumer_count") != len(consumers)
            or len(consumers) < 3
        ):
            raise ClosureError("gz-transport13 runtime consumer proof is incomplete")
        install_resolved = install_root.resolve()
        for raw_path, row in consumers.items():
            if not isinstance(row, dict):
                raise ClosureError("invalid gz-transport13 runtime consumer row")
            consumer = Path(raw_path).resolve()
            try:
                consumer.relative_to(install_resolved)
            except ValueError as exc:
                raise ClosureError("gz-transport13 runtime consumer escapes install") from exc
            _assert_regular(consumer, "gz-transport13 runtime consumer")
            if row.get("sha256") != _sha256(consumer):
                raise ClosureError("gz-transport13 runtime consumer hash mismatch")
            if Path(str(row.get("resolved_library", ""))).resolve() != expected_alias:
                raise ClosureError("runtime consumer resolved an unexpected gz-transport13")
            if row.get("resolved_library_sha256") != library_hash:
                raise ClosureError("runtime consumer resolved library hash mismatch")
        return report

    build_report = validate_report(build_report_path, runtime_binding=False)
    binding_report = validate_report(binding_report_path, runtime_binding=True)
    vendor_source = {
        path: source_inventory[path] for path in GZ_TRANSPORT13_VENDOR_SOURCE_PATHS
    }
    return {
        "upstream_commit": manifest["upstream_commit"],
        "upstream_tree": manifest["upstream_tree"],
        "patched_node_shared_sha256": manifest["patched_node_shared_sha256"],
        "eintr_retry_limit": manifest["eintr_retry_limit"],
        "vendor_source": vendor_source,
        "vendor_source_sha256": _json_digest(vendor_source),
        "library_aliases": alias_hashes,
        "core_library_sha256": library_hash,
        "protobuf_needed": build_report["install"]["protobuf_needed"],
        "protobuf_runtime_path": build_report["install"]["protobuf_runtime_path"],
        "protobuf_binding_report": GZ_TRANSPORT13_PROTOBUF_BINDING_REPORT.as_posix(),
        "protobuf_binding_report_sha256": protobuf_report_sha256,
        "protobuf_binding": protobuf_report,
        "build_report": GZ_TRANSPORT13_VENDOR_BUILD_REPORT.as_posix(),
        "build_report_sha256": _sha256(build_report_path),
        "runtime_binding_report": GZ_TRANSPORT13_RUNTIME_BINDING_REPORT.as_posix(),
        "runtime_binding_report_sha256": _sha256(binding_report_path),
        "runtime_consumer_count": binding_report["runtime_activation"]["consumer_count"],
        "build_memory_preflight_sha256": build_report["memory_preflight"]["sha256"],
    }


def _merged_overlay_contract(
    runtime_ws: Path, packages: Sequence[str]
) -> dict[str, Any]:
    install_root = runtime_ws / "install"
    _assert_regular(install_root / "setup.bash", "merged overlay setup")
    _regular_files(install_root, "merged runtime install closure")
    markers: dict[str, str] = {}
    isolated_prefixes: list[str] = []
    for package in packages:
        marker = (
            install_root
            / "share/ament_index/resource_index/packages"
            / package
        )
        _assert_regular(marker, f"merged ament marker for {package}")
        markers[package] = marker.relative_to(install_root).as_posix()
        isolated = install_root / package
        if isolated.exists() or isolated.is_symlink():
            isolated_prefixes.append(isolated.relative_to(install_root).as_posix())
    if isolated_prefixes:
        raise ClosureError(
            "runtime overlay is isolated or mixed, not one merged prefix: "
            + ", ".join(isolated_prefixes)
        )
    return {
        "mode": "merged_copy_install",
        "install_root": str(install_root.resolve()),
        "ament_resource_markers": markers,
        "isolated_package_prefixes": [],
        "symbolic_links_allowed": False,
        "symbolic_link_count": 0,
    }


def _install_symlink_report_identity(
    runtime_ws: Path, install_root: Path
) -> dict[str, Any]:
    report = runtime_ws / INSTALL_SYMLINK_REPORT
    _assert_regular(report, "install symbolic-link report")
    # _merged_overlay_contract has already walked install/ without following
    # links and rejected any link.  The independently generated report must
    # therefore be byte-empty, not merely contain zero non-blank lines.
    if report.read_bytes() != b"":
        raise ClosureError("formal install symbolic-link report is not empty")
    _regular_files(install_root, "merged runtime install closure")
    return {
        "path": INSTALL_SYMLINK_REPORT.as_posix(),
        "sha256": _sha256(report),
        "size_bytes": 0,
        "live_symbolic_link_count": 0,
        "report_matches_live_scan": True,
    }


def _resolve_nvidia_egl_library() -> Path:
    rows = _identity_command(["ldconfig", "-p"], "NVIDIA EGL library cache")
    candidates: set[Path] = set()
    for row in rows.splitlines():
        if not row.lstrip().startswith(f"{NVIDIA_EGL_LIBRARY} ") or "=>" not in row:
            continue
        candidates.add(Path(row.rsplit("=>", 1)[1].strip()))
    if len(candidates) != 1:
        raise ClosureError(
            "NVIDIA EGL library cache must resolve exactly one "
            f"{NVIDIA_EGL_LIBRARY}: {sorted(map(str, candidates))}"
        )
    try:
        return next(iter(candidates)).resolve(strict=True)
    except OSError as exc:
        raise ClosureError(f"cannot resolve canonical NVIDIA EGL library: {exc}") from exc


def _nvidia_egl_runtime_identity(runtime_ws: Path) -> dict[str, Any]:
    runtime_root = runtime_ws.resolve()
    vendor_json = runtime_root / NVIDIA_EGL_VENDOR_JSON
    _assert_regular(vendor_json, "NVIDIA EGL vendor JSON")
    if vendor_json.stat().st_size <= 0:
        raise ClosureError("NVIDIA EGL vendor JSON is empty")
    try:
        vendor_path = vendor_json.resolve(strict=True)
        vendor_path.relative_to(runtime_root)
    except (OSError, ValueError) as exc:
        raise ClosureError("NVIDIA EGL vendor JSON escapes runtime workspace") from exc
    vendor = _read_json(vendor_json)
    expected_vendor = {
        "file_format_version": "1.0.0",
        "ICD": {"library_path": NVIDIA_EGL_LIBRARY},
    }
    if vendor != expected_vendor:
        raise ClosureError("NVIDIA EGL vendor JSON has an unexpected schema or library")
    library = _resolve_nvidia_egl_library()
    _assert_regular(library, "canonical NVIDIA EGL library")
    if library.stat().st_size <= 0:
        raise ClosureError("canonical NVIDIA EGL library is empty")
    environment = dict(NVIDIA_EGL_ENVIRONMENT)
    environment["__EGL_VENDOR_LIBRARY_FILENAMES"] = str(vendor_path)
    return {
        "status": "NVIDIA_EGL_RUNTIME_BOUND",
        "bound": True,
        "vendor_json": {
            "path": str(vendor_path),
            "size_bytes": vendor_json.stat().st_size,
            "sha256": _sha256(vendor_json),
        },
        "canonical_library": {
            "path": str(library),
            "size_bytes": library.stat().st_size,
            "sha256": _sha256(library),
        },
        "environment": environment,
    }


def _build_markers(
    runtime_ws: Path, packages: Sequence[str]
) -> dict[str, dict[str, Any]]:
    rows: dict[str, dict[str, Any]] = {}
    for package in packages:
        source_root = runtime_ws / "src" / package
        source_files = _regular_files(source_root, f"frozen source package {package}")
        latest_source_ns = max(path.stat().st_mtime_ns for path in source_files)
        marker = runtime_ws / "build" / package / "colcon_build.rc"
        _assert_regular(marker, f"colcon build marker for {package}")
        try:
            return_code = int(marker.read_text(encoding="utf-8").strip())
        except (OSError, UnicodeError, ValueError) as exc:
            raise ClosureError(f"invalid colcon build marker for {package}: {marker}") from exc
        if return_code != 0:
            raise ClosureError(f"colcon build failed for {package}: rc={return_code}")
        marker_ns = marker.stat().st_mtime_ns
        if marker_ns < latest_source_ns:
            raise ClosureError(
                f"colcon build marker predates frozen source for {package}"
            )
        rows[package] = {
            "path": marker.relative_to(runtime_ws).as_posix(),
            "sha256": _sha256(marker),
            "mtime_epoch_ns": marker_ns,
            "latest_source_mtime_epoch_ns": latest_source_ns,
            "return_code": return_code,
        }
    return rows


def _python_install_root(install_root: Path, package: str) -> Path | None:
    matches = sorted(
        {
            path
            for pattern in (
                f"lib/python*/site-packages/{package}",
                f"lib/python*/dist-packages/{package}",
            )
            for path in install_root.glob(pattern)
            if path.is_dir() and not path.is_symlink()
        }
    )
    if len(matches) > 1:
        raise ClosureError(
            f"{package} resolves to multiple installed Python packages: {matches}"
        )
    return matches[0] if matches else None


def _source_install_bindings(
    runtime_ws: Path, install_root: Path, packages: Sequence[str]
) -> dict[str, dict[str, Any]]:
    bindings: dict[str, dict[str, Any]] = {}
    frozen_source_root = runtime_ws / "src"

    def bind(package: str, source: Path, installed: Path) -> None:
        _assert_regular(source, f"source binding for {package}")
        _assert_regular(installed, f"installed binding for {package}")
        source_hash = _sha256(source)
        installed_hash = _sha256(installed)
        if source_hash != installed_hash:
            raise ClosureError(
                f"installed runtime file is stale for {package}: {source} != {installed}"
            )
        key = (
            Path("starter_ws/src") / source.relative_to(frozen_source_root)
        ).as_posix()
        bindings[key] = {
            "package": package,
            "installed": installed.relative_to(install_root).as_posix(),
            "sha256": source_hash,
        }

    for package in packages:
        source_package = frozen_source_root / package
        install_share = install_root / "share" / package
        bind(package, source_package / "package.xml", install_share / "package.xml")
        for directory_name in SOURCE_SHARE_DIRECTORIES:
            source_directory = source_package / directory_name
            if not source_directory.is_dir():
                continue
            for source in _regular_files(
                source_directory, f"source {package}/{directory_name}"
            ):
                relative = source.relative_to(source_package)
                bind(package, source, install_share / relative)

        source_python = source_package / package
        if source_python.is_dir():
            installed_python = _python_install_root(install_root, package)
            if installed_python is None:
                raise ClosureError(f"installed Python package is missing: {package}")
            for source in _regular_files(source_python, f"source Python package {package}"):
                relative = source.relative_to(source_python)
                if source.suffix in IGNORED_FILE_SUFFIXES:
                    continue
                bind(package, source, installed_python / relative)
    if not bindings:
        raise ClosureError("source/install binding closure is empty")
    return bindings


def _plugin_inventory(install_root: Path) -> dict[str, dict[str, Any]]:
    rows: dict[str, dict[str, Any]] = {}
    for name in GAZEBO_PLUGIN_LIBRARIES:
        path = install_root / "lib" / name
        _assert_regular(path, f"Gazebo plugin {name}")
        stat = path.stat()
        rows[name] = {"sha256": _sha256(path), "size_bytes": stat.st_size}
    return rows


def _gripper_mimic_identity(
    repository_root: Path,
    runtime_ws: Path,
    build_markers: Mapping[str, Mapping[str, Any]],
    plugins: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    """Bind the explicit gripper mimic replacement through installation."""
    frozen_source_root = runtime_ws / "src"
    source_rows: dict[str, dict[str, str]] = {}
    for relative in GRIPPER_MIMIC_SOURCE_PATHS:
        source = repository_root / relative
        frozen = frozen_source_root / Path(relative).relative_to("starter_ws/src")
        _assert_regular(source, f"gripper-mimic source {relative}")
        _assert_regular(frozen, f"frozen gripper-mimic source {relative}")
        source_sha256 = _sha256(source)
        frozen_sha256 = _sha256(frozen)
        if source_sha256 != frozen_sha256:
            raise ClosureError(f"frozen gripper-mimic source drifted: {relative}")
        source_rows[relative] = {
            "source_sha256": source_sha256,
            "frozen_sha256": frozen_sha256,
        }

    installed_rows: dict[str, dict[str, str]] = {}
    for source_relative, installed_relative in GRIPPER_MIMIC_INSTALL_BINDINGS.items():
        installed = runtime_ws / "install" / installed_relative
        _assert_regular(
            installed, f"installed gripper-mimic binding {installed_relative}"
        )
        installed_sha256 = _sha256(installed)
        if installed_sha256 != source_rows[source_relative]["source_sha256"]:
            raise ClosureError(
                "gripper-mimic source/install hash drifted: " + source_relative
            )
        installed_rows[source_relative] = {
            "installed": installed_relative,
            "sha256": installed_sha256,
        }

    marker = build_markers.get(GRIPPER_MIMIC_PACKAGE)
    if not isinstance(marker, Mapping) or marker.get("return_code") != 0:
        raise ClosureError("gripper-mimic package has no successful build marker")
    plugin = plugins.get(GRIPPER_MIMIC_PLUGIN)
    if not isinstance(plugin, Mapping) or not SHA256_PATTERN.fullmatch(
        str(plugin.get("sha256", ""))
    ):
        raise ClosureError("gripper-mimic plugin inventory is missing or invalid")
    return {
        "implementation": FORMAL_RUNTIME_CONTRACT_REVISION,
        "source": source_rows,
        "source_sha256": _json_digest(source_rows),
        "installed": installed_rows,
        "installed_sha256": _json_digest(installed_rows),
        "build_marker": dict(marker),
        "plugin": {
            "installed": f"lib/{GRIPPER_MIMIC_PLUGIN}",
            "sha256": str(plugin["sha256"]),
            "size_bytes": plugin.get("size_bytes"),
        },
    }


def _model_inventory(model_root: Path) -> dict[str, dict[str, Any]]:
    manifest_path = model_root / "artifact_manifest.json"
    _assert_regular(manifest_path, "perception artifact manifest")
    manifest = _read_json(manifest_path)
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict) or not artifacts:
        raise ClosureError("perception artifact manifest has no artifacts mapping")
    root = model_root.resolve()
    rows: dict[str, dict[str, Any]] = {
        "artifact_manifest.json": {
            "sha256": _sha256(manifest_path),
            "size_bytes": manifest_path.stat().st_size,
            "role": "artifact_manifest",
        }
    }
    for relative, expected in sorted(artifacts.items()):
        if not isinstance(relative, str) or not isinstance(expected, dict):
            raise ClosureError("invalid perception artifact manifest entry")
        path = model_root / relative
        _assert_regular(path, f"perception model artifact {relative}")
        try:
            path.resolve().relative_to(root)
        except ValueError as exc:
            raise ClosureError(f"perception artifact escapes model root: {relative}") from exc
        actual_hash = _sha256(path)
        actual_size = path.stat().st_size
        if actual_hash != expected.get("sha256"):
            raise ClosureError(f"perception artifact hash mismatch: {relative}")
        if actual_size != expected.get("byte_size"):
            raise ClosureError(f"perception artifact size mismatch: {relative}")
        rows[relative] = {
            "sha256": actual_hash,
            "size_bytes": actual_size,
            "role": str(expected.get("model_role", "unspecified")),
        }
    return rows


def _onnxruntime_inventory(onnx_pythonpath: Path) -> dict[str, dict[str, Any]]:
    package_root = onnx_pythonpath / "onnxruntime"
    files = _regular_files(package_root, "ONNX Runtime Python closure")
    return _inventory(files, onnx_pythonpath)


def _side_brush_surface_identity(
    runtime_ws: Path, install_root: Path
) -> dict[str, Any]:
    """Bind the post-install expanded-SDF preflight to the frozen xacro bytes."""

    audit_path = runtime_ws / SIDE_BRUSH_SURFACE_PREFLIGHT
    installed_xacro = install_root / SIDE_BRUSH_INSTALLED_XACRO
    _assert_regular(audit_path, "side-brush expanded-SDF preflight")
    _assert_regular(installed_xacro, "installed formal vehicle xacro")
    report = _read_json(audit_path)
    if (
        report.get("schema_version") != 2
        or report.get("status")
        != "FORMAL_SIDE_BRUSH_EXPANDED_SDF_SURFACE_PASSED"
    ):
        raise ClosureError("side-brush expanded-SDF preflight is not passing")
    central = report.get("central_roller")
    if (
        not isinstance(central, dict)
        or central.get("collision") != "central_roller_link_collision"
        or central.get("joint") != "central_roller_joint"
        or central.get("radius_m") != 0.100
        or central.get("length_m") != 0.620
        or not isinstance(central.get("surface"), dict)
        or central["surface"].get("mu") != 0.08
        or central["surface"].get("mu2") != 0.08
    ):
        raise ClosureError("central-roller expanded-SDF contact proxy is not passing")
    source = report.get("source")
    if not isinstance(source, dict) or source.get("mode") != "xacro_to_gz_sdf":
        raise ClosureError("side-brush preflight was not generated from installed xacro")
    try:
        recorded_source = Path(str(source["path"])).resolve()
    except KeyError as exc:
        raise ClosureError("side-brush preflight has no installed xacro path") from exc
    if recorded_source != installed_xacro.resolve():
        raise ClosureError("side-brush preflight does not target the frozen installed xacro")
    installed_xacro_sha256 = _sha256(installed_xacro)
    if source.get("sha256") != installed_xacro_sha256:
        raise ClosureError("side-brush preflight installed xacro hash mismatch")
    expanded_urdf_sha256 = source.get("expanded_urdf_sha256")
    expanded_sdf_sha256 = report.get("expanded_sdf_sha256")
    if not isinstance(expanded_urdf_sha256, str) or not SHA256_PATTERN.fullmatch(
        expanded_urdf_sha256
    ):
        raise ClosureError("side-brush preflight has invalid expanded URDF hash")
    if not isinstance(expanded_sdf_sha256, str) or not SHA256_PATTERN.fullmatch(
        expanded_sdf_sha256
    ):
        raise ClosureError("side-brush preflight has invalid expanded SDF hash")
    if audit_path.stat().st_mtime_ns < installed_xacro.stat().st_mtime_ns:
        raise ClosureError("side-brush preflight predates the frozen installed xacro")
    return {
        "audit_path": SIDE_BRUSH_SURFACE_PREFLIGHT.as_posix(),
        "audit_sha256": _sha256(audit_path),
        "installed_xacro": SIDE_BRUSH_INSTALLED_XACRO.as_posix(),
        "installed_xacro_sha256": installed_xacro_sha256,
        "expanded_urdf_sha256": expanded_urdf_sha256,
        "expanded_sdf_sha256": expanded_sdf_sha256,
    }


def capture_closure(
    repository_root: Path,
    runtime_ws: Path,
    perception_artifacts: Path,
    onnx_pythonpath: Path,
    *,
    packages: Sequence[str] = FINAL_RUNTIME_PACKAGES,
) -> dict[str, Any]:
    for candidate, label in (
        (repository_root, "repository root"),
        (runtime_ws, "runtime workspace"),
        (perception_artifacts, "perception artifact root"),
        (onnx_pythonpath, "ONNX Runtime root"),
    ):
        if candidate.is_symlink():
            raise ClosureError(f"{label} must not be a symbolic link: {candidate}")
    repository_root = repository_root.resolve()
    runtime_ws = runtime_ws.resolve()
    perception_artifacts = perception_artifacts.resolve()
    onnx_pythonpath = onnx_pythonpath.resolve()
    install_root = runtime_ws / "install"
    package_rows = tuple(dict.fromkeys(packages))
    if not package_rows:
        raise ClosureError("final runtime package list is empty")
    merged = _merged_overlay_contract(runtime_ws, package_rows)
    source = _source_inventory(repository_root, package_rows)
    frozen_source = _frozen_source_inventory(
        repository_root, runtime_ws, package_rows
    )
    typed_cleaning_telemetry_source = {
        path: source[path] for path in TYPED_CLEANING_TELEMETRY_SOURCE_PATHS
    }
    installed = _inventory(
        _regular_files(install_root, "merged runtime install closure"), install_root
    )
    markers = _build_markers(runtime_ws, package_rows)
    bindings = _source_install_bindings(runtime_ws, install_root, package_rows)
    plugins = _plugin_inventory(install_root)
    gripper_mimic = _gripper_mimic_identity(
        repository_root, runtime_ws, markers, plugins
    )
    gz_transport13_vendor = _gz_transport13_vendor_identity(
        repository_root, runtime_ws, install_root, source
    )
    models = _model_inventory(perception_artifacts)
    onnxruntime = _onnxruntime_inventory(onnx_pythonpath)
    side_brush_surface = _side_brush_surface_identity(runtime_ws, install_root)
    install_symlink_report = _install_symlink_report_identity(
        runtime_ws, install_root
    )
    windows_cold_start_evidence = _windows_cold_start_evidence_identity(runtime_ws)
    ros_gz_image_system_runtime = _ros_gz_image_system_identity()
    nvidia_egl_runtime = _nvidia_egl_runtime_identity(runtime_ws)
    return {
        "repository_root": str(repository_root),
        "runtime_ws": str(runtime_ws),
        "perception_artifact_root": str(perception_artifacts),
        "onnx_pythonpath": str(onnx_pythonpath),
        "runtime_packages": list(package_rows),
        "merged_overlay": merged,
        "source_inventory": source,
        "source_inventory_sha256": _json_digest(source),
        "frozen_source_root": str((runtime_ws / "src").resolve()),
        "frozen_source_inventory": frozen_source,
        "frozen_source_inventory_sha256": _json_digest(frozen_source),
        "typed_cleaning_telemetry_source": typed_cleaning_telemetry_source,
        "typed_cleaning_telemetry_source_sha256": _json_digest(
            typed_cleaning_telemetry_source
        ),
        "install_inventory": installed,
        "install_inventory_sha256": _json_digest(installed),
        "build_markers": markers,
        "build_markers_sha256": _json_digest(markers),
        "source_install_bindings": bindings,
        "source_install_bindings_sha256": _json_digest(bindings),
        "gazebo_plugins": plugins,
        "gazebo_plugins_sha256": _json_digest(plugins),
        "gripper_mimic": gripper_mimic,
        "gripper_mimic_sha256": _json_digest(gripper_mimic),
        "gz_transport13_vendor": gz_transport13_vendor,
        "gz_transport13_vendor_sha256": _json_digest(gz_transport13_vendor),
        "perception_models": models,
        "perception_models_sha256": _json_digest(models),
        "onnxruntime_inventory": onnxruntime,
        "onnxruntime_inventory_sha256": _json_digest(onnxruntime),
        "side_brush_surface_preflight": side_brush_surface,
        "side_brush_surface_preflight_sha256": _json_digest(side_brush_surface),
        "install_symlink_report": install_symlink_report,
        "install_symlink_report_sha256": _json_digest(install_symlink_report),
        "windows_cold_start_evidence": windows_cold_start_evidence,
        "windows_cold_start_evidence_sha256": _json_digest(
            windows_cold_start_evidence
        ),
        "ros_gz_image_system_runtime": ros_gz_image_system_runtime,
        "ros_gz_image_system_runtime_sha256": _json_digest(
            ros_gz_image_system_runtime
        ),
        "nvidia_egl_runtime": nvidia_egl_runtime,
        "nvidia_egl_runtime_sha256": _json_digest(nvidia_egl_runtime),
    }


def record_manifest(
    repository_root: Path,
    runtime_ws: Path,
    perception_artifacts: Path,
    onnx_pythonpath: Path,
    output: Path,
    *,
    packages: Sequence[str] = FINAL_RUNTIME_PACKAGES,
) -> dict[str, Any]:
    closure = capture_closure(
        repository_root,
        runtime_ws,
        perception_artifacts,
        onnx_pythonpath,
        packages=packages,
    )
    recorded_ns = time.time_ns()
    manifest = {
        "schema_version": 7,
        "kind": "tzcup_formal_final_runtime_closure",
        "runtime_contract_revision": FORMAL_RUNTIME_CONTRACT_REVISION,
        "status": "FORMAL_FINAL_RUNTIME_CLOSURE_FROZEN",
        "recorded_epoch_ns": recorded_ns,
        "recorded_utc": _utc_iso(recorded_ns),
        "closure_sha256": _json_digest(closure),
        "closure": closure,
    }
    _atomic_json(output, manifest)
    return manifest


def verify_manifest(
    manifest_path: Path,
    repository_root: Path,
    runtime_ws: Path,
    perception_artifacts: Path,
    onnx_pythonpath: Path,
) -> dict[str, Any]:
    _assert_regular(manifest_path, "final runtime closure manifest")
    manifest = _read_json(manifest_path)
    if (
        manifest.get("schema_version") != 7
        or manifest.get("kind") != "tzcup_formal_final_runtime_closure"
        or manifest.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_FROZEN"
    ):
        raise ClosureError("unsupported or non-frozen final runtime closure manifest")
    if (
        manifest.get("runtime_contract_revision")
        != FORMAL_RUNTIME_CONTRACT_REVISION
    ):
        raise ClosureError("final runtime closure manifest has the wrong contract revision")
    stored = manifest.get("closure")
    if not isinstance(stored, dict):
        raise ClosureError("final runtime closure manifest has no closure object")
    packages = stored.get("runtime_packages")
    if packages != list(FINAL_RUNTIME_PACKAGES):
        raise ClosureError("final runtime closure package set is incomplete or reordered")
    stored_digest = _json_digest(stored)
    if stored_digest != manifest.get("closure_sha256"):
        raise ClosureError("stored final runtime closure digest is invalid")
    current = capture_closure(
        repository_root,
        runtime_ws,
        perception_artifacts,
        onnx_pythonpath,
        packages=FINAL_RUNTIME_PACKAGES,
    )
    if current != stored:
        changed = sorted(
            key
            for key in set(stored) | set(current)
            if stored.get(key) != current.get(key)
        )
        raise ClosureError(
            "final runtime closure drifted in sections: " + ", ".join(changed)
        )
    return {
        "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
        "passed": True,
        "manifest": str(manifest_path.resolve()),
        "manifest_sha256": _sha256(manifest_path),
        "closure_sha256": stored_digest,
        "runtime_package_count": len(FINAL_RUNTIME_PACKAGES),
        "source_file_count": len(stored["source_inventory"]),
        "frozen_source_file_count": len(stored["frozen_source_inventory"]),
        "frozen_source_inventory_sha256": stored[
            "frozen_source_inventory_sha256"
        ],
        "typed_cleaning_telemetry_source_count": len(
            stored["typed_cleaning_telemetry_source"]
        ),
        "typed_cleaning_telemetry_source_sha256": stored[
            "typed_cleaning_telemetry_source_sha256"
        ],
        "install_file_count": len(stored["install_inventory"]),
        "source_install_binding_count": len(stored["source_install_bindings"]),
        "gazebo_plugin_count": len(stored["gazebo_plugins"]),
        "gripper_mimic_implementation": stored["gripper_mimic"]["implementation"],
        "gripper_mimic_plugin_sha256": stored["gripper_mimic"]["plugin"]["sha256"],
        "gz_transport13_core_library_sha256": stored["gz_transport13_vendor"][
            "core_library_sha256"
        ],
        "gz_transport13_vendor_build_report_sha256": stored[
            "gz_transport13_vendor"
        ]["build_report_sha256"],
        "gz_transport13_runtime_binding_report_sha256": stored[
            "gz_transport13_vendor"
        ]["runtime_binding_report_sha256"],
        "gz_transport13_runtime_consumer_count": stored["gz_transport13_vendor"][
            "runtime_consumer_count"
        ],
        "gz_transport13_protobuf_needed": stored["gz_transport13_vendor"][
            "protobuf_needed"
        ],
        "gz_transport13_protobuf_binding_report_sha256": stored[
            "gz_transport13_vendor"
        ]["protobuf_binding_report_sha256"],
        "perception_artifact_count": len(stored["perception_models"]),
        "side_brush_surface_audit_sha256": stored["side_brush_surface_preflight"][
            "audit_sha256"
        ],
        "side_brush_installed_xacro": str(
            Path(stored["merged_overlay"]["install_root"])
            / stored["side_brush_surface_preflight"]["installed_xacro"]
        ),
        "side_brush_installed_xacro_sha256": stored[
            "side_brush_surface_preflight"
        ]["installed_xacro_sha256"],
        "side_brush_expanded_sdf_sha256": stored[
            "side_brush_surface_preflight"
        ]["expanded_sdf_sha256"],
        "symbolic_link_count": 0,
        "install_symlink_report_sha256": stored["install_symlink_report"][
            "sha256"
        ],
        "windows_cold_start_evidence_bound": stored[
            "windows_cold_start_evidence"
        ]["bound"],
        "windows_cold_start_evidence_sha256": stored[
            "windows_cold_start_evidence"
        ]["sha256"],
        "ros_gz_image_system_runtime_bound": stored[
            "ros_gz_image_system_runtime"
        ]["bound"],
        "ros_gz_image_resolved_executable_path": stored[
            "ros_gz_image_system_runtime"
        ]["resolved_executable_path"],
        "ros_gz_image_executable_sha256": stored[
            "ros_gz_image_system_runtime"
        ]["executable_sha256"],
        "ros_gz_image_ros_package_version": stored[
            "ros_gz_image_system_runtime"
        ]["ros_package_version"],
        "ros_gz_image_debian_version": stored[
            "ros_gz_image_system_runtime"
        ]["debian_version"],
        "nvidia_egl_runtime_bound": stored["nvidia_egl_runtime"]["bound"],
        "nvidia_egl_runtime": stored["nvidia_egl_runtime"],
    }


def verify_recorded_manifest(
    manifest_path: Path,
    repository_root: Path,
    expected_install_root: Path,
) -> dict[str, Any]:
    """Verify a frozen closure using its recorded external roots.

    Gate runners use this form so they cannot silently select another overlay
    while the full verifier still re-hashes source, install, plugins, models
    and ONNX Runtime from the recorded non-symlink closure.
    """
    manifest = _read_json(manifest_path)
    closure = manifest.get("closure")
    if not isinstance(closure, dict):
        raise ClosureError("final runtime closure manifest has no closure object")
    try:
        runtime_ws = Path(str(closure["runtime_ws"]))
        perception_artifacts = Path(str(closure["perception_artifact_root"]))
        onnx_pythonpath = Path(str(closure["onnx_pythonpath"]))
    except KeyError as exc:
        raise ClosureError("final runtime closure has incomplete recorded roots") from exc
    if expected_install_root.is_symlink():
        raise ClosureError("selected runtime install root must not be a symbolic link")
    if expected_install_root.resolve() != (runtime_ws / "install").resolve():
        raise ClosureError(
            "selected runtime install root does not match the frozen closure"
        )
    return verify_manifest(
        manifest_path,
        repository_root,
        runtime_ws,
        perception_artifacts,
        onnx_pythonpath,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    for name in ("record", "verify"):
        child = subparsers.add_parser(name)
        child.add_argument("--repository-root", type=Path, required=True)
        child.add_argument("--runtime-ws", type=Path, required=True)
        child.add_argument("--perception-artifacts", type=Path, required=True)
        child.add_argument("--onnx-pythonpath", type=Path, required=True)
        child.add_argument("--manifest", type=Path, required=True)
    recorded = subparsers.add_parser("verify-recorded")
    recorded.add_argument("--repository-root", type=Path, required=True)
    recorded.add_argument("--install-root", type=Path, required=True)
    recorded.add_argument("--manifest", type=Path, required=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        if args.command == "record":
            value = record_manifest(
                args.repository_root,
                args.runtime_ws,
                args.perception_artifacts,
                args.onnx_pythonpath,
                args.manifest,
            )
        elif args.command == "verify":
            value = verify_manifest(
                args.manifest,
                args.repository_root,
                args.runtime_ws,
                args.perception_artifacts,
                args.onnx_pythonpath,
            )
        else:
            value = verify_recorded_manifest(
                args.manifest,
                args.repository_root,
                args.install_root,
            )
    except (ClosureError, OSError, ValueError) as exc:
        print(json.dumps({"status": "FORMAL_FINAL_RUNTIME_CLOSURE_BLOCKED", "error": str(exc)}, indent=2))
        return 2
    print(json.dumps(value, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
