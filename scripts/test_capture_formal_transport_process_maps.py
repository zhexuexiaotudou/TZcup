import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts/capture_formal_transport_process_maps.py"
SPEC = importlib.util.spec_from_file_location("capture_transport_maps", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def make_contract(tmp_path: Path):
    runtime = tmp_path / "runtime"
    install = runtime / "install"
    setup = install / "setup.bash"
    core = install / "lib/libgz-transport13.so.13"
    image_bridge = tmp_path / "system_ros/lib/ros_gz_image/image_bridge"
    for path, data in (
        (setup, b"setup"),
        (core, b"patched transport"),
        (image_bridge, b"image bridge"),
    ):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    closure = {
        "runtime_ws": str(runtime.resolve()),
        "merged_overlay": {
            "mode": "merged_copy_install",
            "install_root": str(install.resolve()),
        },
        "install_inventory": {"setup.bash": {"sha256": sha(setup)}},
        "gz_transport13_vendor": {"core_library_sha256": sha(core)},
        "ros_gz_image_system_runtime": {
            "bound": True,
            "resolved_executable_path": str(image_bridge.resolve()),
            "executable_sha256": sha(image_bridge),
        },
    }
    manifest = {
        "schema_version": 5,
        "kind": "tzcup_formal_final_runtime_closure",
        "status": "FORMAL_FINAL_RUNTIME_CLOSURE_FROZEN",
        "closure_sha256": MODULE.json_digest(closure),
        "closure": closure,
    }
    closure_path = runtime / "final_runtime_closure_manifest.json"
    closure_path.write_text(json.dumps(manifest), encoding="utf-8")
    return (
        MODULE.load_runtime_contract(setup, closure_path),
        core,
        image_bridge,
    )


def make_snapshot(pid: int, executable: Path, mapped: list[Path]):
    return MODULE.ProcessSnapshot(
        pid=pid,
        executable=executable.resolve(),
        executable_sha256=sha(executable),
        maps_sha256=f"{pid:064x}",
        mapped_paths=tuple(path.resolve() for path in mapped),
    )


def make_system_libraries(tmp_path: Path, suffix: str = ""):
    root = tmp_path / f"system{suffix}"
    protobuf = root / "x86_64-linux-gnu/libprotobuf.so.32.0.12"
    zmq = root / "x86_64-linux-gnu/libzmq.so.5.2.5"
    protobuf.parent.mkdir(parents=True)
    protobuf.write_bytes(f"protobuf{suffix}".encode())
    zmq.write_bytes(f"zmq{suffix}".encode())
    return root, protobuf, zmq


def test_accepts_same_frozen_transport_and_same_system_libraries(tmp_path: Path):
    contract, core, image_bridge = make_contract(tmp_path)
    gazebo_exe = tmp_path / "bin/gz"
    gazebo_exe.parent.mkdir()
    gazebo_exe.write_bytes(b"gazebo")
    system_root, protobuf, zmq = make_system_libraries(tmp_path)
    report = MODULE.evaluate_process_maps(
        contract,
        make_snapshot(101, gazebo_exe, [core, protobuf, zmq]),
        make_snapshot(102, image_bridge, [core, protobuf, zmq]),
        system_library_roots=(system_root,),
    )
    assert report["passed"] is True
    assert report["status"] == "FORMAL_TRANSPORT_PROCESS_MAPS_BOUND"
    assert all(report["cross_process_checks"].values())


def test_rejects_transport_hash_drift(tmp_path: Path):
    contract, core, image_bridge = make_contract(tmp_path)
    core.write_bytes(b"drifted after closure")
    gazebo_exe = tmp_path / "gz"
    gazebo_exe.write_bytes(b"gazebo")
    system_root, protobuf, zmq = make_system_libraries(tmp_path)
    report = MODULE.evaluate_process_maps(
        contract,
        make_snapshot(201, gazebo_exe, [core, protobuf, zmq]),
        make_snapshot(202, image_bridge, [core, protobuf, zmq]),
        system_library_roots=(system_root,),
    )
    codes = [row["code"] for row in report["blockers"]]
    assert codes.count("TRANSPORT_HASH_MISMATCH") == 2


def test_rejects_cross_process_system_library_divergence(tmp_path: Path):
    contract, core, image_bridge = make_contract(tmp_path)
    gazebo_exe = tmp_path / "gz"
    gazebo_exe.write_bytes(b"gazebo")
    root_a, protobuf_a, zmq_a = make_system_libraries(tmp_path, "a")
    root_b, protobuf_b, zmq_b = make_system_libraries(tmp_path, "b")
    report = MODULE.evaluate_process_maps(
        contract,
        make_snapshot(301, gazebo_exe, [core, protobuf_a, zmq_a]),
        make_snapshot(302, image_bridge, [core, protobuf_b, zmq_b]),
        system_library_roots=(root_a, root_b),
    )
    codes = {row["code"] for row in report["blockers"]}
    assert "CROSS_PROCESS_PROTOBUF_DIVERGENCE" in codes
    assert "CROSS_PROCESS_ZMQ_DIVERGENCE" in codes


def test_rejects_forbidden_vendor_mapping_even_when_selected_libraries_pass(
    tmp_path: Path,
):
    contract, core, image_bridge = make_contract(tmp_path)
    gazebo_exe = tmp_path / "gz"
    gazebo_exe.write_bytes(b"gazebo")
    system_root, protobuf, zmq = make_system_libraries(tmp_path)
    forbidden = Path(
        "/opt/ros/jazzy/opt/ortools_vendor/lib/libsomething_vendor.so"
    )
    report = MODULE.evaluate_process_maps(
        contract,
        make_snapshot(401, gazebo_exe, [core, protobuf, zmq, forbidden]),
        make_snapshot(402, image_bridge, [core, protobuf, zmq]),
        system_library_roots=(system_root,),
    )
    assert any(
        row["code"] == "FORBIDDEN_VENDOR_MAPPING"
        for row in report["blockers"]
    )


def test_proc_maps_parser_deduplicates_segments_and_rejects_deleted_library():
    text = "\n".join(
        (
            "1000-2000 r--p 00000000 00:00 1 /runtime/lib/libgz-transport13.so.13",
            "2000-3000 r-xp 00001000 00:00 1 /runtime/lib/libgz-transport13.so.13",
            "3000-4000 rw-p 00000000 00:00 0 [heap]",
        )
    )
    assert MODULE.parse_proc_maps(text) == (
        Path("/runtime/lib/libgz-transport13.so.13"),
    )
    with pytest.raises(MODULE.CaptureError, match="was deleted"):
        MODULE.parse_proc_maps(
            "1000-2000 r-xp 0 00:00 1 "
            "/runtime/lib/libprotobuf.so.32.0.12 (deleted)\n"
        )


def test_exclusive_writer_refuses_stale_output(tmp_path: Path):
    output = tmp_path / "maps.json"
    MODULE.write_report_exclusive(output, {"passed": True})
    first = output.read_bytes()
    with pytest.raises(MODULE.CaptureError, match="refusing stale output"):
        MODULE.write_report_exclusive(output, {"passed": False})
    assert output.read_bytes() == first
