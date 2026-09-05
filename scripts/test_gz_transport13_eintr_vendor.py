from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
VALIDATOR_PATH = ROOT / "scripts" / "validate_gz_transport13_eintr_vendor.py"
SPEC = importlib.util.spec_from_file_location("gz_vendor", VALIDATOR_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _protobuf_binding(tmp_path: Path) -> tuple[Path, dict[str, str]]:
    resolved = {
        "Protobuf_INCLUDE_DIR": "/usr/include",
        "Protobuf_LIBRARY_RELEASE": "/usr/lib/x86_64-linux-gnu/libprotobuf.so.32.0.12",
        "Protobuf_LITE_LIBRARY_RELEASE": "/usr/lib/x86_64-linux-gnu/libprotobuf-lite.so.32.0.12",
        "Protobuf_PROTOC_LIBRARY_RELEASE": "/usr/lib/x86_64-linux-gnu/libprotoc.so.32.0.12",
        "Protobuf_PROTOC_EXECUTABLE": "/usr/bin/protoc",
    }
    payload = {
        "schema_version": 1,
        "status": "SYSTEM_PROTOBUF_3_21_12_BINDING_PASSED",
        "passed": True,
        "protobuf_version": "3.21.12",
        "protobuf_header_version": 3021012,
        "config_mode_protobuf_disabled": True,
        "forbidden_prefix": "/opt/ros/jazzy/opt/ortools_vendor",
        "compile_command_count": 37,
        "resolved": resolved,
    }
    path = tmp_path / "protobuf_binding.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path, resolved


def test_frozen_patch_is_frame_local_and_bounded() -> None:
    manifest = MODULE.load_manifest()
    contract = MODULE.validate_patch_contract(manifest)
    assert contract == {
        "helper_send_count": 1,
        "publish_frame_callsite_count": 12,
        "replaced_raw_send_count": 12,
        "complete_publication_retry_present": False,
    }


def test_public_v13_abi_allows_non_api_dynamic_symbol_differences() -> None:
    reference = {"public_a", "public_b", "std_weak"}
    patched = {"public_a", "public_b", "extra_template"}
    result = MODULE.validate_public_gz_transport_v13_abi(
        reference,
        patched,
        reference_demangled={
            "public_a": "gz::transport::v13::Node::Node()",
            "public_b": "gz::transport::v13::Node::TopicList() const",
            "std_weak": "std::piecewise_construct",
        },
        patched_demangled={
            "public_a": "gz::transport::v13::Node::Node()",
            "public_b": "gz::transport::v13::Node::TopicList() const",
            "extra_template": "std::vector<int>::size() const",
        },
    )
    assert result["public_v13_abi_covered"] is True
    assert result["missing_public_v13_symbol_count"] == 0
    assert result["patch_private_helper_export_count"] == 0


def test_public_v13_abi_rejects_missing_reference_api() -> None:
    with pytest.raises(MODULE.ValidationError, match="missing public"):
        MODULE.validate_public_gz_transport_v13_abi(
            {"public_a", "public_b"},
            {"public_a"},
            reference_demangled={
                "public_a": "gz::transport::v13::Node::Node()",
                "public_b": "gz::transport::v13::Node::TopicList() const",
            },
            patched_demangled={
                "public_a": "gz::transport::v13::Node::Node()",
            },
        )


def test_public_v13_abi_rejects_patch_helper_export() -> None:
    with pytest.raises(MODULE.ValidationError, match="helper leaked"):
        MODULE.validate_public_gz_transport_v13_abi(
            {"public_a"},
            {"public_a", "helper"},
            reference_demangled={
                "public_a": "gz::transport::v13::Node::Node()",
            },
            patched_demangled={
                "public_a": "gz::transport::v13::Node::Node()",
                "helper": (
                    "void sendPublishFrameWithEintrRetry<zmq::send_flags>(...)"
                ),
            },
        )


def test_patch_hash_drift_fails_closed(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    manifest = MODULE.load_manifest()
    bad_patch = tmp_path / "changed.patch"
    bad_patch.write_text("not the reviewed patch\n", encoding="utf-8")
    changed = dict(manifest)
    changed["patch"] = bad_patch.name
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(json.dumps(changed), encoding="utf-8")
    monkeypatch.setattr(MODULE, "PATCH_ROOT", tmp_path)
    monkeypatch.setattr(MODULE, "MANIFEST_PATH", manifest_path)
    with pytest.raises(MODULE.ValidationError, match="missing or has drifted"):
        MODULE.load_manifest()


def test_source_validation_rejects_unexpected_second_change(tmp_path: Path) -> None:
    source = tmp_path / "source"
    (source / "src").mkdir(parents=True)
    with pytest.raises(MODULE.ValidationError):
        MODULE.validate_source(source, MODULE.load_manifest())


def test_protobuf_binding_is_content_and_hash_bound(tmp_path: Path) -> None:
    path, resolved = _protobuf_binding(tmp_path)
    result = MODULE.validate_protobuf_binding(path, expected_resolved=resolved)
    assert result["path"] == str(path.resolve())
    assert result["sha256"] == MODULE.sha256(path)
    assert result["protobuf_version"] == "3.21.12"
    assert result["protobuf_header_version"] == 3021012
    assert result["resolved"] == resolved


def test_protobuf_binding_rejects_ortools_resolution(tmp_path: Path) -> None:
    path, resolved = _protobuf_binding(tmp_path)
    report = json.loads(path.read_text(encoding="utf-8"))
    report["resolved"]["Protobuf_LIBRARY_RELEASE"] = (
        "/opt/ros/jazzy/opt/ortools_vendor/lib/libprotobuf.so"
    )
    path.write_text(json.dumps(report), encoding="utf-8")
    expected = dict(resolved)
    expected["Protobuf_LIBRARY_RELEASE"] = report["resolved"][
        "Protobuf_LIBRARY_RELEASE"
    ]
    with pytest.raises(MODULE.ValidationError, match="references OR-Tools"):
        MODULE.validate_protobuf_binding(path, expected_resolved=expected)


def test_dynamic_linkage_requires_only_system_protobuf_32() -> None:
    dynamic = """
 0x0000000000000001 (NEEDED) Shared library: [libprotobuf.so.32]
 0x0000000000000001 (NEEDED) Shared library: [libzmq.so.5]
"""
    ldd = "libprotobuf.so.32 => /lib/x86_64-linux-gnu/libprotobuf.so.32 (0x1234)\n"
    result = MODULE.validate_protobuf_dynamic_linkage(dynamic, ldd, "safe symbol\n")
    assert result["protobuf_needed"] == "libprotobuf.so.32"
    assert result["forbidden_vendor_needed"] == []
    assert result["obvious_static_ortools_abseil_symbol_count"] == 0


@pytest.mark.parametrize(
    ("dynamic", "symbols", "message"),
    [
        (
            "(NEEDED) Shared library: [libprotobuf.so.25]",
            "safe",
            "exactly libprotobuf.so.32",
        ),
        (
            "(NEEDED) Shared library: [libprotobuf.so.32]\n"
            "(NEEDED) Shared library: [libabsl_base.so]",
            "safe",
            "forbidden OR-Tools/Abseil",
        ),
        (
            "(NEEDED) Shared library: [libprotobuf.so.32]",
            "0000 T absl::StaticPollution",
            "statically linked OR-Tools/Abseil",
        ),
    ],
)
def test_dynamic_linkage_rejects_protobuf_or_vendor_pollution(
    dynamic: str, symbols: str, message: str
) -> None:
    ldd = "libprotobuf.so.32 => /lib/x86_64-linux-gnu/libprotobuf.so.32 (0x1234)\n"
    with pytest.raises(MODULE.ValidationError, match=message):
        MODULE.validate_protobuf_dynamic_linkage(dynamic, ldd, symbols)


def test_runtime_activation_binds_consumers_to_prefix_library(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    install = tmp_path / "install"
    library = install / "lib/libgz-transport13.so.13"
    library.parent.mkdir(parents=True)
    library.write_bytes(b"patched-library")
    consumer = install / "lib/libWaterRecoverySystem.so"
    consumer.write_bytes(b"plugin")

    real_run = MODULE.run

    def fake_run(*args: str, cwd=None) -> str:
        if args[0] == "ldd":
            return f"libgz-transport13.so.13 => {library} (0x1234)\n"
        return real_run(*args, cwd=cwd)

    monkeypatch.setattr(MODULE, "run", fake_run)
    result = MODULE.validate_runtime_activation(
        install,
        [consumer],
        {"LD_LIBRARY_PATH": str(install / "lib")},
    )
    assert result["consumer_count"] == 1
    assert result["patched_soname_alias_sha256"] == MODULE.sha256(library)
    assert next(iter(result["consumers"].values()))["resolved_library"] == str(
        library.resolve()
    )


def test_dynamic_dependencies_uses_loader_only_for_proven_native_proot_bug(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    library = tmp_path / "libexample.so"
    loader = tmp_path / "ld-linux-x86-64.so.2"
    library.write_bytes(b"elf")
    loader.write_bytes(b"loader")
    calls: list[tuple[str, ...]] = []

    def fake_run(*args: str, cwd=None) -> str:
        del cwd
        calls.append(args)
        if args[0] == "ldd":
            raise MODULE.ValidationError(
                "command failed (1): ldd: you do not have read permission"
            )
        return "libexample.so => /tmp/libexample.so (0x1234)"

    monkeypatch.setattr(MODULE, "run", fake_run)
    monkeypatch.setattr(MODULE, "NATIVE_DYNAMIC_LOADER", loader)
    monkeypatch.setattr(
        MODULE, "_native_linux_loader_fallback_allowed", lambda path: path == library
    )
    assert "libexample.so" in MODULE.dynamic_dependencies(library)
    assert calls == [("ldd", str(library)), (str(loader), "--list", str(library))]


def test_runtime_activation_rejects_system_resolution(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    install = tmp_path / "install"
    library = install / "lib/libgz-transport13.so.13"
    library.parent.mkdir(parents=True)
    library.write_bytes(b"patched-library")
    consumer = install / "lib/libWaterRecoverySystem.so"
    consumer.write_bytes(b"plugin")
    monkeypatch.setattr(
        MODULE,
        "run",
        lambda *args, cwd=None: (
            "libgz-transport13.so.13 => "
            "/opt/ros/jazzy/opt/gz_transport_vendor/lib/libgz-transport13.so.13 "
            "(0x1234)\n"
        ),
    )
    with pytest.raises(MODULE.ValidationError, match="system/unexpected"):
        MODULE.validate_runtime_activation(
            install,
            [consumer],
            {"LD_LIBRARY_PATH": str(install / "lib")},
        )


def test_runtime_activation_rejects_inactive_library_path(tmp_path: Path) -> None:
    install = tmp_path / "install"
    (install / "lib").mkdir(parents=True)
    with pytest.raises(MODULE.ValidationError, match="first LD_LIBRARY_PATH"):
        MODULE.validate_runtime_activation(
            install,
            [install / "lib/libWaterRecoverySystem.so"],
            {"LD_LIBRARY_PATH": "/opt/ros/jazzy/lib"},
        )


def test_vendor_builder_pins_system_protobuf_without_include_order_fallback() -> None:
    source = (ROOT / "scripts" / "build_gz_transport13_eintr_vendor.sh").read_text(
        encoding="utf-8"
    )
    assert 'system_protobuf_version="3.21.12"' in source
    assert 'system_protobuf_header_version="3021012"' in source
    assert 'system_protobuf_protoc="/usr/bin/protoc"' in source
    assert '-DCMAKE_IGNORE_PREFIX_PATH="${ortools_vendor_prefix}"' in source
    assert '-DProtobuf_DIR:PATH=Protobuf_DIR-NOTFOUND' in source
    assert '-DProtobuf_INCLUDE_DIR:PATH="${system_protobuf_include}"' in source
    assert '-DProtobuf_LIBRARY_RELEASE:FILEPATH="${system_protobuf_library}"' in source
    assert (
        '-DProtobuf_PROTOC_EXECUTABLE:FILEPATH="${system_protobuf_protoc}"' in source
    )
    assert '"${build_dir}/compile_commands.json"' in source
    assert "compile command {index} still references OR-Tools vendor Protobuf" in source
    assert '--protobuf-binding "${protobuf_binding_report}"' in source

    final_builder = (ROOT / "scripts" / "build_formal_final_runtime.sh").read_text(
        encoding="utf-8"
    )
    assert '--protobuf-binding "${vendor_work_root}/protobuf_binding.json"' in final_builder


def test_vendor_builder_fails_closed_on_protobuf_identity_drift() -> None:
    source = (ROOT / "scripts" / "build_gz_transport13_eintr_vendor.sh").read_text(
        encoding="utf-8"
    )
    assert "system protoc version drifted" in source
    assert "system libprotobuf version drifted" in source
    assert "system Protobuf header version drifted" in source
    assert "config-mode Protobuf unexpectedly resolved" in source
    assert "CMAKE_IGNORE_PREFIX_PATH does not exclude" in source


def test_vendor_builder_supports_audited_offline_source_bundle() -> None:
    source = (ROOT / "scripts" / "build_gz_transport13_eintr_vendor.sh").read_text(
        encoding="utf-8"
    )
    assert 'source_bundle="${FORMAL_GZ_TRANSPORT13_SOURCE_BUNDLE:-}"' in source
    assert (
        'source_bundle_sha256="${FORMAL_GZ_TRANSPORT13_SOURCE_BUNDLE_SHA256:-}"'
        in source
    )
    assert "FORMAL_GZ_TRANSPORT13_SOURCE_BUNDLE must be an absolute path" in source
    assert "FORMAL_GZ_TRANSPORT13_SOURCE_BUNDLE must be a regular non-symlink file" in source
    assert "FORMAL_GZ_TRANSPORT13_SOURCE_BUNDLE_SHA256 must be a lowercase SHA-256" in source
    assert "gz-transport source bundle SHA-256 mismatch" in source
    assert 'git init "${source_dir}"' in source
    assert 'git -C "${source_dir}" bundle verify "${source_bundle}"' in source
    assert 'bundle_ref="refs/tags/${upstream_tag}"' in source
    assert 'git -C "${source_dir}" fetch "${source_bundle}" "${bundle_ref}"' in source
    assert "source bundle tag does not resolve to the pinned upstream commit" in source
    assert 'git -C "${source_dir}" fsck --strict --full' in source
    assert 'git -C "${source_dir}" checkout --detach "${upstream_commit}"' in source
    assert 'rev-parse HEAD)' in source
    assert "rev-parse 'HEAD^{tree}')" in source
