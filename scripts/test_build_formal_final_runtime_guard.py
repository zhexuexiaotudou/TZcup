from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

import formal_final_runtime_closure as closure


ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "scripts/build_formal_final_runtime.sh"


def _source() -> str:
    return BUILD.read_text(encoding="utf-8")


def test_final_builder_has_fresh_host_and_runtime_memory_evidence() -> None:
    source = _source()
    assert 'source "${repo_root}/scripts/run_formal_runtime_isolation.sh"' in source
    assert 'formal_runtime_memory_preflight "${windows_preflight_prefix}"' in source
    assert "formal_final_build_linux_memory_preflight" in source
    assert "FORMAL_FINAL_BUILD_MIN_MEM_AVAILABLE_KIB:-4194304" in source
    assert "FORMAL_FINAL_BUILD_MAX_SWAP_USED_KIB:-1048576" in source
    for suffix in (
        '"${windows_preflight_prefix}.json"',
        '"${windows_preflight_prefix}.log"',
        '"${linux_preflight_json}"',
        '"${watchdog_prefix}.json"',
        '"${watchdog_prefix}.log"',
    ):
        assert suffix in source
    assert '[[ ! -e "${path}" && ! -L "${path}" ]]' in source
    assert '"docker_signalled_or_stopped": false' in source
    assert 'windows_cold_gate_bound_json="${runtime_ws}/formal_windows_cold_start_evidence.json"' in source
    assert 'install -m 0444 -- "${cold_gate_evidence}" "${windows_cold_gate_bound_json}"' in source
    assert 'cmp -s -- "${cold_gate_evidence}" "${windows_cold_gate_bound_json}"' in source


def test_final_builder_requires_a_new_runtime_and_all_final_evidence_paths() -> None:
    source = _source()
    assert '[[ ! -e "${runtime_ws}" && ! -L "${runtime_ws}" ]]' in source
    assert "refusing non-fresh final runtime workspace" in source
    fresh_paths = source.split("for path in", 1)[1].split("; do", 1)[0]
    for path in (
        '"${side_brush_surface_preflight}"',
        '"${integrated_build_manifest}"',
        '"${install_symlinks_report}"',
    ):
        assert path in fresh_paths
    assert '--output "${side_brush_surface_preflight}"' in source
    assert '--output "${integrated_build_manifest}"' in source
    assert '[[ -f "${side_brush_surface_preflight}" && ! -L "${side_brush_surface_preflight}" ]]' in source
    assert '[[ -f "${integrated_build_manifest}" && ! -L "${integrated_build_manifest}" ]]' in source


def test_final_builder_bounds_both_builds_in_one_exact_setsid_group() -> None:
    source = _source()
    assert 'FORMAL_COLCON_PARALLEL_WORKERS:-1' in source
    assert '[[ "${parallel_workers}" =~ ^[12]$ ]]' in source
    assert 'export CMAKE_BUILD_PARALLEL_LEVEL="${parallel_workers}"' in source
    assert 'export MAKEFLAGS="-j${parallel_workers}"' in source
    assert 'setsid bash -c \'\n' in source
    child = source.split("setsid bash -c '", 1)[1].split(
        "' formal-final-build", 1
    )[0]
    assert "build_gz_transport13_eintr_vendor.sh" in child
    assert "exec colcon" in child
    assert '--parallel-workers "${parallel_workers}"' in child
    assert 'build_pid=$!' in source
    assert 'formal_runtime_start_memory_watchdog "${build_pid}" "${watchdog_prefix}"' in source
    assert 'wait "${build_pid}"' in source
    assert "formal_runtime_stop_memory_watchdog" in source


def test_final_builder_builds_exactly_the_closed_sixteen_package_set() -> None:
    source = _source()
    child = source.split("setsid bash -c '", 1)[1].split(
        "' formal-final-build", 1
    )[0]
    selected = re.findall(r"\bsanitation_[a-z0-9_]+", child)
    assert selected == list(closure.FINAL_RUNTIME_PACKAGES)
    assert len(selected) == 16
    assert "r53" not in source.lower()


def test_final_builder_cleanup_is_scoped_to_its_recorded_build_pid() -> None:
    source = _source()
    cleanup = re.search(
        r"formal_final_build_cleanup\(\) \{(?P<body>.*?)\n\}",
        source,
        flags=re.DOTALL,
    )
    assert cleanup is not None
    body = cleanup.group("body")
    assert 'formal_runtime_kill_group "${build_pid}"' in body
    assert "pkill" not in body
    assert "killall" not in body
    assert "docker" not in body.lower()
    assert "docker stop" not in source.lower()
    assert "docker kill" not in source.lower()
    assert "formal_runtime_install_traps formal_final_build_cleanup" in source


def test_final_builder_reuses_the_pinned_system_protobuf_for_colcon() -> None:
    source = _source()
    assert "--cmake-args" in source
    assert "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" in source
    assert "-DCMAKE_IGNORE_PREFIX_PATH=/opt/ros/jazzy/opt/ortools_vendor" in source
    assert "-DProtobuf_DIR:PATH=Protobuf_DIR-NOTFOUND" in source
    assert "-DProtobuf_INCLUDE_DIR:PATH=/usr/include" in source
    assert (
        '-DProtobuf_LIBRARY_RELEASE:FILEPATH="${system_protobuf_libdir}/libprotobuf.so"'
        in source
    )
    assert "-DProtobuf_PROTOC_EXECUTABLE:FILEPATH=/usr/bin/protoc" in source
    assert "Every downstream CMake package is configured again by colcon" in source


def test_final_builder_freezes_regular_sources_and_rejects_install_links() -> None:
    source = _source()
    assert 'frozen_source_root="${runtime_ws}/src"' in source
    assert 'install_symlinks_report="${runtime_ws}/INSTALL_SYMLINKS.txt"' in source
    assert '"${frozen_source_root}"' in source.split("for path in", 1)[1]
    assert '"${install_symlinks_report}"' in source.split("for path in", 1)[1]
    assert 'shutil.copytree(source, frozen, copy_function=shutil.copy2)' in source
    assert 'frozen_inventory != source_inventory' in source
    assert 'entry.name in {"__pycache__", ".pytest_cache"}' in source
    assert 'path.suffix == ".pyc"' in source
    assert 'path.suffix.lower() in {".o", ".a", ".so", ".dll", ".exe"}' in source
    assert 'magic == b"\\x7fELF" or magic[:2] == b"MZ"' in source
    assert "contains an unlabelled executable artifact" in source
    assert "contains a generated test/cache directory" in source
    assert '--base-paths "${frozen_source_root}"' in source
    assert '--base-paths "${repo_root}/starter_ws/src"' not in source
    assert 'pending.write_text("".join(f"{relative}\\n" for relative in links)' in source
    assert "pending.replace(output)" in source
    assert 'if links:' in source
    assert "formal final install contains symbolic links" in source


def test_final_builder_snapshots_the_installed_proot_compatibility_layer() -> None:
    source = _source()
    assert 'proot_compat_source="${repo_root}/scripts/proot_glibc_compat.c"' in source
    assert 'proot_compat_install="${runtime_ws}/install/lib/libtzcup_proot_glibc_compat.so"' in source
    assert 'cc -shared -fPIC -O2 -Wall -Wextra' in source
    assert 'chmod 0555 -- "${proot_compat_pending}"' in source
    assert 'mv -- "${proot_compat_pending}" "${proot_compat_install}"' in source
    install_position = source.index('mv -- "${proot_compat_pending}" "${proot_compat_install}"')
    link_inventory_position = source.index('# Record the exact install-tree link inventory.')
    build_snapshot_position = source.index('  record-build \\\n')
    assert install_position < link_inventory_position < build_snapshot_position


def test_final_builder_fails_if_colcon_reintroduces_ortools_protobuf() -> None:
    source = _source()
    assert 'expected = {' in source
    assert '"Protobuf_DIR": "Protobuf_DIR-NOTFOUND"' in source
    assert '"Protobuf_INCLUDE_DIR": "/usr/include"' in source
    assert '"Protobuf_PROTOC_EXECUTABLE": "/usr/bin/protoc"' in source
    assert 'if forbidden in rendered:' in source
    assert "references OR-Tools vendor headers" in source


def test_final_builder_sources_colcon_overlay_without_nounset() -> None:
    source = _source()
    guarded_setup = (
        'set +u\n'
        'source "${vendor_work_root}/activate_patched_runtime.sh"\n'
        'source "${runtime_ws}/install/setup.bash"\n'
        'set -u'
    )
    assert guarded_setup in source


@pytest.mark.skipif(
    os.name != "posix" or shutil.which("bash") is None,
    reason="requires a native POSIX bash without starting WSL",
)
def test_final_builder_shell_syntax() -> None:
    result = subprocess.run(
        ["bash", "-n"],
        input=_source().encode("utf-8"),
        check=False,
        capture_output=True,
    )
    assert result.returncode == 0, result.stderr.decode("utf-8", errors="replace")
