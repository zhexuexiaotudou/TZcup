from __future__ import annotations

import json
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

import formal_final_runtime_closure as closure


_REAL_ROS_GZ_IMAGE_SYSTEM_IDENTITY = closure._ros_gz_image_system_identity


@pytest.fixture(autouse=True)
def _fake_ros_gz_image_system_identity(tmp_path: Path, monkeypatch):
    executable = tmp_path / "system_ros/lib/ros_gz_image/image_bridge"
    package_xml = tmp_path / "system_ros/share/ros_gz_image/package.xml"
    identity: dict[str, object] = {
        "bound": True,
        "ament_package": "ros_gz_image",
        "ament_prefix": str((tmp_path / "system_ros").resolve()),
        "ament_resource_marker": str(
            (
                tmp_path
                / "system_ros/share/ament_index/resource_index/packages/ros_gz_image"
            ).resolve()
        ),
        "declared_executable_path": str(executable.resolve()),
        "resolved_executable_path": str(executable.resolve()),
        "executable_sha256": "4" * 64,
        "executable_size_bytes": 4096,
        "executable_mode_octal": "0o755",
        "package_xml_path": str(package_xml.resolve()),
        "package_xml_sha256": "5" * 64,
        "ros_package_version": "1.0.22",
        "debian_package": "ros-jazzy-ros-gz-image",
        "debian_binary_package": "ros-jazzy-ros-gz-image",
        "debian_status": "ii",
        "debian_version": "1.0.22-1noble.20260801.010203",
        "debian_architecture": "amd64",
    }
    monkeypatch.setattr(
        closure, "_ros_gz_image_system_identity", lambda: dict(identity)
    )
    return identity


def _write(path: Path, value: bytes | str = "x\n") -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(value, bytes):
        path.write_bytes(value)
    else:
        path.write_text(value, encoding="utf-8")
    return path


def _fake_system_ros_prefix(tmp_path: Path) -> tuple[Path, Path, Path]:
    prefix = tmp_path / "opt/ros/jazzy"
    marker = _write(
        prefix / "share/ament_index/resource_index/packages/ros_gz_image", ""
    )
    executable = _write(
        prefix / closure.ROS_GZ_IMAGE_EXECUTABLE_RELATIVE,
        b"fake-ros-gz-image-elf",
    )
    package_xml = _write(
        prefix / closure.ROS_GZ_IMAGE_PACKAGE_XML_RELATIVE,
        (
            "<package><name>ros_gz_image</name>"
            "<version>1.0.22</version></package>\n"
        ),
    )
    return prefix, marker, executable


def _fake_dpkg_run(
    executable: Path, *, owner: str = "ros-jazzy-ros-gz-image"
):
    def run(arguments, **kwargs):
        del kwargs
        if "-S" in arguments:
            return subprocess.CompletedProcess(
                arguments, 0, f"{owner}: {executable}\n", ""
            )
        return subprocess.CompletedProcess(
            arguments,
            0,
            "ii \tros-jazzy-ros-gz-image\t"
            "1.0.22-1noble.20260801.010203\tamd64\n",
            "",
        )

    return run


def _fake_closure(tmp_path: Path) -> tuple[Path, Path, Path, Path, Path]:
    repository = tmp_path / "repo"
    runtime = tmp_path / "runtime"
    install = runtime / "install"
    models = tmp_path / "models"
    onnx = tmp_path / "onnx"
    recorded_ns = 1_788_000_000_000_000_000
    _write(
        runtime / closure.WINDOWS_COLD_START_EVIDENCE,
        json.dumps(
            {
                "report_id": "tzcup_formal_windows_memory_start_gate_v1",
                "status": "FORMAL_WINDOWS_MEMORY_START_GATE_PASSED",
                "passed": True,
                "recorded_epoch_ns": recorded_ns,
                "require_wsl_stopped": True,
                "require_wsl_running": False,
                "thresholds_bytes": {
                    "min_commit_available": 13_421_772_800,
                    "max_docker_private": 4_294_967_296,
                },
                "sample": {
                    "epoch_ns": recorded_ns - 10_000_000,
                    "commit_available_bytes": 14_000_000_000,
                    "docker_private_bytes": 0,
                    "vmmem_wsl_private_bytes": 0,
                },
                "checks": {
                    "windows_commit_available_at_least_configured_minimum": True,
                    "docker_private_at_most_configured_maximum": True,
                    "wsl_vm_stopped_when_required": True,
                    "wsl_vm_running_when_required": True,
                },
                "docker_was_signalled_or_stopped": False,
            }
        ),
    )
    _write(repository / "scripts/runner.py", "print('runner')\n")
    _write(repository / "config/high_fidelity_vehicle/contract.yaml", "value: 1\n")
    for relative in closure.GRIPPER_MIMIC_SOURCE_PATHS:
        _write(repository / relative, f"gripper mimic source: {relative}\n")
    for relative in closure.TYPED_CLEANING_TELEMETRY_SOURCE_PATHS:
        _write(repository / relative, f"frozen typed source: {relative}\n")
    for relative in closure.GZ_TRANSPORT13_VENDOR_SOURCE_PATHS:
        _write(repository / relative, f"frozen vendor source: {relative}\n")
    vendor_manifest_path = repository / "patches/upstream/gz_transport13/manifest.json"
    vendor_patch_path = (
        repository
        / "patches/upstream/gz_transport13/0001-publish-retry-current-frame-on-eintr.patch"
    )
    vendor_manifest = {
        "schema_version": 1,
        "component": "gz-transport13",
        "upstream_tag": "gz-transport13_13.5.0",
        "upstream_commit": "a" * 40,
        "upstream_tree": "b" * 40,
        "patched_node_shared_sha256": "c" * 64,
        "patch_sha256": closure._sha256(vendor_patch_path),
        "eintr_retry_limit": 3,
    }
    vendor_manifest_path.write_text(json.dumps(vendor_manifest), encoding="utf-8")
    _write(install / "setup.bash", "# merged setup\n")
    installed_xacro = _write(
        install / closure.SIDE_BRUSH_INSTALLED_XACRO,
        (
            repository
            / "starter_ws/src/sanitation_vehicle_description/urdf/"
            "formal_competition_vehicle.urdf.xacro"
        ).read_bytes(),
    )
    _write(
        install
        / "share/sanitation_vehicle_description/urdf/high_fidelity/"
        "manipulator_stack.xacro",
        (
            repository
            / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/"
            "manipulator_stack.xacro"
        ).read_bytes(),
    )
    _write(
        install / "include/sanitation_gazebo_control/GripperMimicEffortCore.hh",
        (
            repository
            / "starter_ws/src/sanitation_gazebo_control/include/"
            "sanitation_gazebo_control/GripperMimicEffortCore.hh"
        ).read_bytes(),
    )
    for package in closure.FINAL_RUNTIME_PACKAGES:
        source_xml = _write(
            repository / "starter_ws/src" / package / "package.xml",
            f"<package><name>{package}</name></package>\n",
        )
        _write(
            install / "share/ament_index/resource_index/packages" / package,
            "",
        )
        _write(install / "share" / package / "package.xml", source_xml.read_bytes())
        marker = _write(runtime / "build" / package / "colcon_build.rc", "0\n")
        marker_ns = source_xml.stat().st_mtime_ns + 10_000_000
        os.utime(marker, ns=(marker_ns, marker_ns))
    shutil.copytree(repository / "starter_ws/src", runtime / "src")
    _write(runtime / closure.INSTALL_SYMLINK_REPORT, "")
    for plugin in closure.GAZEBO_PLUGIN_LIBRARIES:
        _write(install / "lib" / plugin, f"binary:{plugin}".encode())
    vendor_library_hash = None
    for relative in closure.GZ_TRANSPORT13_CORE_LIBRARIES:
        path = _write(install / relative, b"patched-gz-transport13")
        vendor_library_hash = closure._sha256(path)
    assert vendor_library_hash is not None
    consumers = (
        install / "lib/libWaterRecoverySystem.so",
        install / "lib/libCleaningActuatorMotorSystem.so",
        _write(
            install
            / "lib/sanitation_gazebo_control/cleaning_actuator_vector_bridge",
            b"typed-bridge",
        ),
    )
    alias = (install / "lib/libgz-transport13.so.13").resolve()
    protobuf_report_path = runtime / closure.GZ_TRANSPORT13_PROTOBUF_BINDING_REPORT
    protobuf_report = {
        "schema_version": 1,
        "status": "SYSTEM_PROTOBUF_3_21_12_BINDING_PASSED",
        "passed": True,
        "protobuf_version": "3.21.12",
        "protobuf_header_version": 3021012,
        "config_mode_protobuf_disabled": True,
        "forbidden_prefix": "/opt/ros/jazzy/opt/ortools_vendor",
        "compile_command_count": 37,
        "resolved": {
            "Protobuf_INCLUDE_DIR": "/usr/include",
            "Protobuf_LIBRARY_RELEASE": "/usr/lib/x86_64-linux-gnu/libprotobuf.so.32.0.12",
            "Protobuf_LITE_LIBRARY_RELEASE": "/usr/lib/x86_64-linux-gnu/libprotobuf-lite.so.32.0.12",
            "Protobuf_PROTOC_LIBRARY_RELEASE": "/usr/lib/x86_64-linux-gnu/libprotoc.so.32.0.12",
            "Protobuf_PROTOC_EXECUTABLE": "/usr/bin/protoc",
        },
    }
    _write(protobuf_report_path, json.dumps(protobuf_report))
    protobuf_binding = {
        "path": str(protobuf_report_path.resolve()),
        "sha256": closure._sha256(protobuf_report_path),
        "schema_version": 1,
        "status": protobuf_report["status"],
        "protobuf_version": protobuf_report["protobuf_version"],
        "protobuf_header_version": protobuf_report["protobuf_header_version"],
        "config_mode_protobuf_disabled": True,
        "forbidden_prefix": protobuf_report["forbidden_prefix"],
        "compile_command_count": protobuf_report["compile_command_count"],
        "resolved": protobuf_report["resolved"],
    }

    def vendor_report(runtime_binding: bool) -> dict[str, object]:
        report: dict[str, object] = {
            "report_id": "tzcup_gz_transport13_eintr_vendor_v1",
            "status": "GZ_TRANSPORT13_EINTR_VENDOR_CONTRACT_PASSED",
            "passed": True,
            "manifest": vendor_manifest,
            "manifest_sha256": closure._sha256(vendor_manifest_path),
            "source": {
                "commit": vendor_manifest["upstream_commit"],
                "tree": vendor_manifest["upstream_tree"],
                "node_shared_sha256": vendor_manifest["patched_node_shared_sha256"],
            },
            "install": {
                "install_prefix": str(install.resolve()),
                "core_library_sha256": vendor_library_hash,
                "protobuf_needed": "libprotobuf.so.32",
                "protobuf_runtime_path": "/lib/x86_64-linux-gnu/libprotobuf.so.32.0.12",
                "dynamic_needed": ["libprotobuf.so.32", "libzmq.so.5"],
                "forbidden_vendor_needed": [],
                "obvious_static_ortools_abseil_symbol_count": 0,
            },
            "protobuf_binding": protobuf_binding,
            "memory_preflight": {"sha256": "d" * 64},
            "parallel_workers": 2,
        }
        if runtime_binding:
            report["runtime_activation"] = {
                "install_prefix": str(install.resolve()),
                "ld_library_path_first": str((install / "lib").resolve()),
                "patched_soname_alias": str(alias),
                "patched_soname_alias_sha256": vendor_library_hash,
                "consumer_count": len(consumers),
                "consumers": {
                    str(path.resolve()): {
                        "sha256": closure._sha256(path),
                        "resolved_library": str(alias),
                        "resolved_library_sha256": vendor_library_hash,
                    }
                    for path in consumers
                },
            }
        return report

    _write(
        runtime / closure.GZ_TRANSPORT13_VENDOR_BUILD_REPORT,
        json.dumps(vendor_report(False)),
    )
    _write(
        runtime / closure.GZ_TRANSPORT13_RUNTIME_BINDING_REPORT,
        json.dumps(vendor_report(True)),
    )
    launch_relative = Path(
        "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
    )
    _write(
        install
        / "share/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py",
        (repository / launch_relative).read_bytes(),
    )

    model = _write(models / "dosod/model.onnx", b"model-bytes")
    model_manifest = {
        "schema_version": 1,
        "artifacts": {
            "dosod/model.onnx": {
                "sha256": closure._sha256(model),
                "byte_size": model.stat().st_size,
                "model_role": "test_model",
            }
        },
    }
    _write(models / "artifact_manifest.json", json.dumps(model_manifest))
    _write(onnx / "onnxruntime/__init__.py", "__version__ = 'test'\n")
    surface_preflight = _write(
        runtime / closure.SIDE_BRUSH_SURFACE_PREFLIGHT,
        json.dumps(
            {
                "schema_version": 2,
                "status": "FORMAL_SIDE_BRUSH_EXPANDED_SDF_SURFACE_PASSED",
                "central_roller": {
                    "collision": "central_roller_link_collision",
                    "joint": "central_roller_joint",
                    "radius_m": 0.100,
                    "length_m": 0.620,
                    "surface": {"mu": 0.08, "mu2": 0.08},
                },
                "source": {
                    "mode": "xacro_to_gz_sdf",
                    "path": str(installed_xacro.resolve()),
                    "sha256": closure._sha256(installed_xacro),
                    "expanded_urdf_sha256": "1" * 64,
                },
                "expanded_sdf_sha256": "2" * 64,
            }
        ),
    )
    surface_ns = installed_xacro.stat().st_mtime_ns + 10_000_000
    os.utime(surface_preflight, ns=(surface_ns, surface_ns))
    manifest = runtime / "final_runtime_closure_manifest.json"
    return repository, runtime, models, onnx, manifest


def test_system_ros_gz_image_identity_binds_ament_dpkg_and_bytes(
    tmp_path: Path, monkeypatch
) -> None:
    prefix, marker, executable = _fake_system_ros_prefix(tmp_path)
    shadow = tmp_path / "shadow"
    shadow.mkdir()
    monkeypatch.setenv(
        "AMENT_PREFIX_PATH", os.pathsep.join((str(shadow), str(prefix)))
    )
    monkeypatch.setattr(closure, "ROS_GZ_IMAGE_EXPECTED_PREFIX", prefix)
    monkeypatch.setattr(closure.os, "access", lambda unused_path, unused_mode: True)
    monkeypatch.setattr(
        closure.subprocess, "run", _fake_dpkg_run(executable.resolve())
    )

    identity = _REAL_ROS_GZ_IMAGE_SYSTEM_IDENTITY()

    assert identity["bound"] is True
    assert identity["ament_prefix"] == str(prefix.resolve())
    assert identity["ament_resource_marker"] == str(marker.resolve())
    assert identity["resolved_executable_path"] == str(executable.resolve())
    assert identity["executable_sha256"] == closure._sha256(executable)
    assert identity["ros_package_version"] == "1.0.22"
    assert identity["debian_package"] == "ros-jazzy-ros-gz-image"
    assert identity["debian_version"] == "1.0.22-1noble.20260801.010203"
    assert identity["debian_architecture"] == "amd64"


def test_system_ros_gz_image_identity_rejects_wrong_debian_owner(
    tmp_path: Path, monkeypatch
) -> None:
    prefix, _, executable = _fake_system_ros_prefix(tmp_path)
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    monkeypatch.setattr(closure, "ROS_GZ_IMAGE_EXPECTED_PREFIX", prefix)
    monkeypatch.setattr(closure.os, "access", lambda unused_path, unused_mode: True)
    monkeypatch.setattr(
        closure.subprocess,
        "run",
        _fake_dpkg_run(executable.resolve(), owner="untrusted-overlay-package"),
    )

    with pytest.raises(closure.ClosureError, match="unexpected Debian ownership"):
        _REAL_ROS_GZ_IMAGE_SYSTEM_IDENTITY()


def test_system_ros_gz_image_identity_rejects_missing_executable(
    tmp_path: Path, monkeypatch
) -> None:
    prefix, _, executable = _fake_system_ros_prefix(tmp_path)
    executable.unlink()
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    monkeypatch.setattr(closure, "ROS_GZ_IMAGE_EXPECTED_PREFIX", prefix)

    with pytest.raises(closure.ClosureError, match="executable is missing"):
        _REAL_ROS_GZ_IMAGE_SYSTEM_IDENTITY()


def test_record_and_verify_complete_non_symlink_merged_closure(tmp_path: Path) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    recorded = closure.record_manifest(repository, runtime, models, onnx, manifest)
    assert recorded["schema_version"] == 6
    assert (
        recorded["runtime_contract_revision"]
        == "gripper_effort_mimic_v1"
    )
    assert recorded["status"] == "FORMAL_FINAL_RUNTIME_CLOSURE_FROZEN"
    assert recorded["closure"]["runtime_packages"] == list(
        closure.FINAL_RUNTIME_PACKAGES
    )
    assert recorded["closure"]["merged_overlay"]["mode"] == "merged_copy_install"
    verified = closure.verify_manifest(manifest, repository, runtime, models, onnx)
    assert verified["passed"] is True
    assert verified["runtime_package_count"] == 16
    assert verified["gazebo_plugin_count"] == 12
    assert "libDryBinMonitorSystem.so" in recorded["closure"]["gazebo_plugins"]
    assert recorded["closure"]["gazebo_plugins"]["libDryBinMonitorSystem.so"]["sha256"] == closure._sha256(
        runtime / "install/lib/libDryBinMonitorSystem.so"
    )
    assert verified["gripper_mimic_implementation"] == "gripper_effort_mimic_v1"
    assert verified["gripper_mimic_plugin_sha256"] == closure._sha256(
        runtime / "install/lib/libGripperMimicEffortSystem.so"
    )
    gripper_mimic = recorded["closure"]["gripper_mimic"]
    assert set(gripper_mimic["source"]) == set(closure.GRIPPER_MIMIC_SOURCE_PATHS)
    assert set(gripper_mimic["installed"]) == set(
        closure.GRIPPER_MIMIC_INSTALL_BINDINGS
    )
    assert verified["gz_transport13_runtime_consumer_count"] == 3
    assert verified["gz_transport13_protobuf_needed"] == "libprotobuf.so.32"
    assert verified["gz_transport13_protobuf_binding_report_sha256"] == closure._sha256(
        runtime / closure.GZ_TRANSPORT13_PROTOBUF_BINDING_REPORT
    )
    assert verified["gz_transport13_core_library_sha256"] == closure._sha256(
        runtime / "install/lib/libgz-transport13.so.13.5.0"
    )
    assert verified["typed_cleaning_telemetry_source_count"] == len(
        closure.TYPED_CLEANING_TELEMETRY_SOURCE_PATHS
    )
    assert set(recorded["closure"]["typed_cleaning_telemetry_source"]) == set(
        closure.TYPED_CLEANING_TELEMETRY_SOURCE_PATHS
    )
    assert verified["side_brush_installed_xacro_sha256"] == closure._sha256(
        runtime / "install" / closure.SIDE_BRUSH_INSTALLED_XACRO
    )
    assert verified["side_brush_expanded_sdf_sha256"] == "2" * 64
    assert verified["symbolic_link_count"] == 0
    assert verified["windows_cold_start_evidence_bound"] is True
    assert verified["windows_cold_start_evidence_sha256"] == closure._sha256(
        runtime / closure.WINDOWS_COLD_START_EVIDENCE
    )
    system_image_bridge = recorded["closure"]["ros_gz_image_system_runtime"]
    assert system_image_bridge["bound"] is True
    assert system_image_bridge["ros_package_version"] == "1.0.22"
    assert system_image_bridge["debian_version"].startswith("1.0.22-")
    assert recorded["closure"]["ros_gz_image_system_runtime_sha256"] == (
        closure._json_digest(system_image_bridge)
    )
    assert verified["ros_gz_image_system_runtime_bound"] is True
    assert verified["ros_gz_image_executable_sha256"] == "4" * 64
    assert verified["ros_gz_image_ros_package_version"] == "1.0.22"
    assert verified["ros_gz_image_debian_version"].startswith("1.0.22-")
    assert verified["frozen_source_file_count"] > 0
    assert recorded["closure"]["install_symlink_report"]["size_bytes"] == 0
    recorded_verified = closure.verify_recorded_manifest(
        manifest, repository, runtime / "install"
    )
    assert recorded_verified["passed"] is True


def test_recorded_verifier_rejects_a_different_runtime_install(tmp_path: Path) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    closure.record_manifest(repository, runtime, models, onnx, manifest)
    other = tmp_path / "other_install"
    other.mkdir()
    with pytest.raises(closure.ClosureError, match="does not match"):
        closure.verify_recorded_manifest(manifest, repository, other)


def test_verifier_rejects_wrong_runtime_contract_revision(tmp_path: Path) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    closure.record_manifest(repository, runtime, models, onnx, manifest)
    legacy = json.loads(manifest.read_text(encoding="utf-8"))
    legacy["runtime_contract_revision"] = "legacy_urdf_mimic"
    manifest.write_text(json.dumps(legacy), encoding="utf-8")
    with pytest.raises(closure.ClosureError, match="wrong contract revision"):
        closure.verify_manifest(manifest, repository, runtime, models, onnx)


def test_record_rejects_gripper_mimic_install_binding_drift(tmp_path: Path) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    installed = (
        runtime
        / "install/share/sanitation_vehicle_description/urdf/high_fidelity/"
        "manipulator_stack.xacro"
    )
    installed.write_text("stale mimic xacro\n", encoding="utf-8")
    with pytest.raises(closure.ClosureError, match="stale"):
        closure.record_manifest(repository, runtime, models, onnx, manifest)


def test_record_rejects_gripper_effort_core_header_install_drift(
    tmp_path: Path,
) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    installed = (
        runtime
        / "install/include/sanitation_gazebo_control/GripperMimicEffortCore.hh"
    )
    installed.write_text("stale effort law\n", encoding="utf-8")
    with pytest.raises(closure.ClosureError, match="source/install hash drifted"):
        closure.record_manifest(repository, runtime, models, onnx, manifest)


def test_record_rejects_missing_gripper_mimic_plugin(tmp_path: Path) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    (runtime / "install/lib/libGripperMimicEffortSystem.so").unlink()
    with pytest.raises(closure.ClosureError, match="Gazebo plugin"):
        closure.record_manifest(repository, runtime, models, onnx, manifest)


def test_verify_fails_closed_when_source_bytes_drift(tmp_path: Path) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    closure.record_manifest(repository, runtime, models, onnx, manifest)
    _write(repository / "scripts/runner.py", "print('changed')\n")
    with pytest.raises(closure.ClosureError, match="source_inventory"):
        closure.verify_manifest(manifest, repository, runtime, models, onnx)


def test_verify_fails_closed_when_bound_cold_start_evidence_drifts(
    tmp_path: Path,
) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    closure.record_manifest(repository, runtime, models, onnx, manifest)
    evidence = runtime / closure.WINDOWS_COLD_START_EVIDENCE
    evidence.write_text(evidence.read_text(encoding="utf-8") + "\n", encoding="utf-8")
    with pytest.raises(closure.ClosureError, match="windows_cold_start_evidence"):
        closure.verify_manifest(manifest, repository, runtime, models, onnx)


def test_verify_fails_closed_when_system_ros_gz_image_identity_drifts(
    tmp_path: Path,
    _fake_ros_gz_image_system_identity: dict[str, object],
) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    closure.record_manifest(repository, runtime, models, onnx, manifest)
    _fake_ros_gz_image_system_identity["debian_version"] = (
        "1.0.23-1noble.20260802.010203"
    )
    _fake_ros_gz_image_system_identity["executable_sha256"] = "6" * 64
    with pytest.raises(closure.ClosureError, match="ros_gz_image_system_runtime"):
        closure.verify_manifest(manifest, repository, runtime, models, onnx)


def test_record_rejects_frozen_source_drift(tmp_path: Path) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    _write(
        runtime / "src/sanitation_gazebo_control/package.xml",
        "<package><name>changed</name></package>\n",
    )
    with pytest.raises(closure.ClosureError, match="frozen runtime src differs"):
        closure.record_manifest(repository, runtime, models, onnx, manifest)


def test_record_rejects_nonempty_install_symlink_report(tmp_path: Path) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    _write(runtime / closure.INSTALL_SYMLINK_REPORT, "lib/linked.so\n")
    with pytest.raises(closure.ClosureError, match="report is not empty"):
        closure.record_manifest(repository, runtime, models, onnx, manifest)


def test_record_rejects_surface_preflight_for_noninstalled_xacro(tmp_path: Path) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    audit = runtime / closure.SIDE_BRUSH_SURFACE_PREFLIGHT
    report = json.loads(audit.read_text(encoding="utf-8"))
    report["source"]["path"] = str(
        repository
        / "starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro"
    )
    audit.write_text(json.dumps(report), encoding="utf-8")
    with pytest.raises(closure.ClosureError, match="frozen installed xacro"):
        closure.record_manifest(repository, runtime, models, onnx, manifest)


def test_record_rejects_missing_typed_cleaning_telemetry_source(tmp_path: Path) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    missing = repository / closure.TYPED_CLEANING_TELEMETRY_SOURCE_PATHS[0]
    missing.unlink()
    with pytest.raises(closure.ClosureError, match="required final runtime source"):
        closure.record_manifest(repository, runtime, models, onnx, manifest)


def test_record_rejects_vendor_core_library_drift(tmp_path: Path) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    (runtime / "install/lib/libgz-transport13.so.13").write_bytes(b"tampered")
    with pytest.raises(closure.ClosureError, match="regular aliases differ"):
        closure.record_manifest(repository, runtime, models, onnx, manifest)


def test_record_rejects_vendor_runtime_binding_report_drift(tmp_path: Path) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    report_path = runtime / closure.GZ_TRANSPORT13_RUNTIME_BINDING_REPORT
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["runtime_activation"]["ld_library_path_first"] = "/opt/ros/jazzy/lib"
    report_path.write_text(json.dumps(report), encoding="utf-8")
    with pytest.raises(closure.ClosureError, match="not first"):
        closure.record_manifest(repository, runtime, models, onnx, manifest)


def test_record_rejects_tampered_vendor_protobuf_binding_report(tmp_path: Path) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    report_path = runtime / closure.GZ_TRANSPORT13_PROTOBUF_BINDING_REPORT
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["protobuf_version"] = "4.25.3"
    report_path.write_text(json.dumps(report), encoding="utf-8")
    with pytest.raises(closure.ClosureError, match="Protobuf binding report drifted"):
        closure.record_manifest(repository, runtime, models, onnx, manifest)


def test_verify_rejects_protobuf_binding_bytes_changed_after_freeze(
    tmp_path: Path,
) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    closure.record_manifest(repository, runtime, models, onnx, manifest)
    report_path = runtime / closure.GZ_TRANSPORT13_PROTOBUF_BINDING_REPORT
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["compile_command_count"] += 1
    report_path.write_text(json.dumps(report), encoding="utf-8")
    with pytest.raises(closure.ClosureError, match="actual Protobuf report"):
        closure.verify_manifest(manifest, repository, runtime, models, onnx)


def test_verify_rejects_surface_preflight_expanded_sdf_hash_drift(
    tmp_path: Path,
) -> None:
    repository, runtime, models, onnx, manifest = _fake_closure(tmp_path)
    closure.record_manifest(repository, runtime, models, onnx, manifest)
    audit = runtime / closure.SIDE_BRUSH_SURFACE_PREFLIGHT
    report = json.loads(audit.read_text(encoding="utf-8"))
    report["expanded_sdf_sha256"] = "3" * 64
    audit.write_text(json.dumps(report), encoding="utf-8")
    with pytest.raises(closure.ClosureError, match="side_brush_surface_preflight"):
        closure.verify_manifest(manifest, repository, runtime, models, onnx)


def test_merged_overlay_rejects_isolated_package_prefix(tmp_path: Path) -> None:
    _, runtime, _, _, _ = _fake_closure(tmp_path)
    (runtime / "install" / closure.FINAL_RUNTIME_PACKAGES[0]).mkdir()
    with pytest.raises(closure.ClosureError, match="not one merged prefix"):
        closure._merged_overlay_contract(runtime, closure.FINAL_RUNTIME_PACKAGES)


def test_install_closure_rejects_any_symbolic_link(tmp_path: Path) -> None:
    _, runtime, _, _, _ = _fake_closure(tmp_path)
    target = runtime / "install/lib/real.so"
    _write(target, b"real")
    link = runtime / "install/lib/link.so"
    try:
        link.symlink_to(target)
    except OSError as exc:  # pragma: no cover - Windows without symlink privilege
        pytest.skip(f"symbolic links unavailable: {exc}")
    with pytest.raises(closure.ClosureError, match="symbolic link"):
        closure._regular_files(runtime / "install", "merged runtime install closure")


def test_install_closure_link_rejection_is_deterministic_without_os_privilege(
    tmp_path: Path, monkeypatch
) -> None:
    root = tmp_path / "install"
    root.mkdir()

    class FakeEntry:
        name = "linked.py"
        path = str(root / name)

        @staticmethod
        def is_symlink() -> bool:
            return True

    monkeypatch.setattr(closure.os, "scandir", lambda unused: [FakeEntry()])
    with pytest.raises(closure.ClosureError, match="symbolic link"):
        closure._regular_files(root, "merged runtime install closure")


def test_model_manifest_binds_every_declared_artifact(tmp_path: Path) -> None:
    _, _, models, _, _ = _fake_closure(tmp_path)
    (models / "dosod/model.onnx").write_bytes(b"tampered")
    with pytest.raises(closure.ClosureError, match="hash mismatch"):
        closure._model_inventory(models)


def test_runtime_package_set_is_closed_over_internal_exec_dependencies() -> None:
    repository = Path(__file__).resolve().parents[1]
    packages = set(closure.FINAL_RUNTIME_PACKAGES)
    missing = set()
    dependency_tags = {"depend", "exec_depend", "build_depend", "build_export_depend"}
    for package in packages:
        root = ET.parse(
            repository / "starter_ws/src" / package / "package.xml"
        ).getroot()
        for element in root:
            dependency = (element.text or "").strip()
            if element.tag in dependency_tags and dependency.startswith("sanitation_"):
                if dependency not in packages:
                    missing.add((package, dependency))
    assert missing == set()


def test_final_runtime_builder_materializes_all_preflight_inputs() -> None:
    repository = Path(__file__).resolve().parents[1]
    source = (repository / "scripts/build_formal_final_runtime.sh").read_text(
        encoding="utf-8"
    )
    assert "--merge-install" in source
    assert "--symlink-install" not in source
    assert "side_brush_sdf_surface_preflight.json" in source
    assert "aggregate_integrated_functional_acceptance.py" in source
    assert "record-build" in source
    assert "integrated_build_manifest.json" in source
    assert "build_gz_transport13_eintr_vendor.sh" in source
    assert "gz_transport13_eintr_vendor_build_report.json" in source
    assert "gz_transport13_eintr_runtime_binding_report.json" in source
    assert source.count("--runtime-plugin") == 3
    assert source.count("--protobuf-binding") == 1
