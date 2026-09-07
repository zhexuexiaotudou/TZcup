"""Focused tests for the four-role, offline-to-board payload bridge."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import stat

import pytest


SPEC = importlib.util.spec_from_file_location("producer", Path(__file__).with_name("produce_s100p_model_payload_receipt.py"))
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _write(root: Path, relative: str, data: bytes) -> Path:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    return path


def _board(root: Path) -> tuple[Path, Path]:
    _write(root, "proc/device-tree/model", b"RDK S100P")
    _write(root, "proc/device-tree/compatible", b"drobot,s100-rdk")
    _write(root, "proc/modules", b"bpu_cores 1 0 - Live 0\nbpu_framework 1 0 - Live 0\n")
    stage = root / "opt" / "tzcup" / "stages" / "release-1"
    artifact = stage / "artifacts"
    artifact.mkdir(parents=True)
    (stage / "evidence").mkdir()
    return stage, artifact


def _character_stat(_path: Path):
    class Result:
        st_mode = stat.S_IFCHR
        st_rdev = 258
        st_ino = 123
    return Result()


def _session_and_closure(root: Path) -> tuple[Path, Path]:
    closure = {
        "kind": "tzcup_formal_final_runtime_closure",
        "status": "FORMAL_FINAL_RUNTIME_CLOSURE_FROZEN",
        "closure_sha256": "c" * 64,
    }
    closure_path = _write(root, "closure.json", json.dumps(closure).encode())
    session = {
        "report_id": "tzcup_formal_final_acceptance_session_v1",
        "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        "started_epoch_ns": 1,
        "snapshot": {"source_inventory_sha256": "d" * 64},
        "runtime_closure_binding": {
            "manifest_sha256": hashlib.sha256(closure_path.read_bytes()).hexdigest(),
            "closure_sha256": closure["closure_sha256"],
        },
    }
    return (
        _write(root, "session.json", json.dumps(session).encode()),
        closure_path,
    )


def _accept_compile(*_args, **_kwargs) -> bool:
    return True


def test_payload_receipt_binds_four_files_to_the_offline_hbm_and_session(tmp_path):
    _stage, artifact = _board(tmp_path)
    payloads = {role: _write(artifact, relative, role.encode()) for role, relative in MODULE.PAYLOADS.items()}
    dosod = payloads["dosod_hbm"]
    compile_path = _write(tmp_path, "compile.json", json.dumps({
        "receipt_id": "tzcup_s100p_dosod_hbm_compile_receipt_v1", "status": "COMPILED_NOT_BOARD_ACCEPTED",
        "board_interaction_performed": False, "output_sha256": hashlib.sha256(dosod.read_bytes()).hexdigest(), "output_byte_size": dosod.stat().st_size,
    }).encode())
    session, closure = _session_and_closure(tmp_path)
    receipt = MODULE.build_receipt(
        artifact_root=artifact, candidate_stage="/opt/tzcup/stages/release-1", board_root=tmp_path,
        offline_compile_receipt=compile_path, acceptance_session=session, runtime_closure=closure,
        platform_machine=lambda: "aarch64", stat_path=_character_stat, compile_validator=_accept_compile,
    )
    assert receipt["board_interaction_performed"] is True
    assert set(receipt["payloads"]) == set(MODULE.PAYLOADS)
    assert receipt["offline_compile_receipt_sha256"] == hashlib.sha256(compile_path.read_bytes()).hexdigest()
    assert receipt["candidate_stage"] == "/opt/tzcup/stages/release-1"
    assert receipt["stage_root"] == receipt["candidate_stage"]
    assert receipt["board_identity"]["bpu_device"]["path"] == "/dev/bpu_core0"
    assert stat.S_ISCHR(receipt["board_identity"]["bpu_device"]["st_mode"])
    output = tmp_path / "opt" / "tzcup" / "stages" / "release-1" / MODULE.OUTPUT_RELATIVE_PATH
    MODULE._atomic_write_fresh(_stage, output, receipt)
    assert json.loads(output.read_text(encoding="utf-8"))["receipt_id"] == receipt["receipt_id"]
    with pytest.raises(ValueError, match="fresh"):
        MODULE._atomic_write_fresh(_stage, output, receipt)
    dosod.write_bytes(b"drift")
    with pytest.raises(ValueError, match="does not match"):
        MODULE.build_receipt(
            artifact_root=artifact, candidate_stage="/opt/tzcup/stages/release-1", board_root=tmp_path,
            offline_compile_receipt=compile_path, acceptance_session=session, runtime_closure=closure,
            platform_machine=lambda: "aarch64", stat_path=_character_stat, compile_validator=_accept_compile,
        )


@pytest.mark.parametrize(
    ("machine", "session_status", "closure_digest", "mismatch_session_closure"),
    [("x86_64", "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING", "c" * 64, False),
     ("aarch64", "FORMAL_FINAL_ACCEPTANCE_SESSION_STOPPED", "c" * 64, False),
     ("aarch64", "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING", "0" * 64, False),
     ("aarch64", "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING", "c" * 64, True)],
)
def test_payload_producer_rejects_offboard_or_stale_session_or_closure(tmp_path, machine, session_status, closure_digest, mismatch_session_closure):
    _stage, artifact = _board(tmp_path)
    payloads = {role: _write(artifact, relative, role.encode()) for role, relative in MODULE.PAYLOADS.items()}
    dosod = payloads["dosod_hbm"]
    compile_path = _write(tmp_path, "compile.json", json.dumps({
        "receipt_id": "tzcup_s100p_dosod_hbm_compile_receipt_v1", "status": "COMPILED_NOT_BOARD_ACCEPTED",
        "board_interaction_performed": False, "output_sha256": hashlib.sha256(dosod.read_bytes()).hexdigest(), "output_byte_size": dosod.stat().st_size,
    }).encode())
    session, closure = _session_and_closure(tmp_path)
    session_value = json.loads(session.read_text())
    closure_value = json.loads(closure.read_text())
    session_value["status"] = session_status
    closure_value["closure_sha256"] = closure_digest
    session_value["runtime_closure_binding"] = (
        {**closure_value, "closure_sha256": "d" * 64}
        if mismatch_session_closure else closure_value
    )
    session.write_text(json.dumps(session_value), encoding="utf-8")
    closure.write_text(json.dumps(closure_value), encoding="utf-8")
    with pytest.raises(ValueError):
        MODULE.build_receipt(
            artifact_root=artifact, candidate_stage="/opt/tzcup/stages/release-1", board_root=tmp_path,
            offline_compile_receipt=compile_path, acceptance_session=session, runtime_closure=closure,
            platform_machine=lambda: machine, stat_path=_character_stat, compile_validator=_accept_compile,
        )


def test_payload_producer_rejects_payload_outside_stage_or_under_symlink(tmp_path):
    stage, artifact = _board(tmp_path)
    payloads = {role: _write(artifact, relative, role.encode()) for role, relative in MODULE.PAYLOADS.items()}
    dosod = payloads["dosod_hbm"]
    compile_path = _write(tmp_path, "compile.json", json.dumps({
        "receipt_id": "tzcup_s100p_dosod_hbm_compile_receipt_v1", "status": "COMPILED_NOT_BOARD_ACCEPTED",
        "board_interaction_performed": False, "output_sha256": hashlib.sha256(dosod.read_bytes()).hexdigest(), "output_byte_size": dosod.stat().st_size,
    }).encode())
    session, closure = _session_and_closure(tmp_path)
    outside = tmp_path / "outside"
    outside.mkdir()
    with pytest.raises(ValueError, match="inside candidate stage"):
        MODULE.build_receipt(
            artifact_root=outside, candidate_stage="/opt/tzcup/stages/release-1", board_root=tmp_path,
            offline_compile_receipt=compile_path, acceptance_session=session, runtime_closure=closure,
            platform_machine=lambda: "aarch64", stat_path=_character_stat, compile_validator=_accept_compile,
        )


def test_payload_producer_refuses_a_compile_receipt_rejected_by_the_canonical_validator(tmp_path):
    stage, artifact = _board(tmp_path)
    payloads = {role: _write(artifact, relative, role.encode()) for role, relative in MODULE.PAYLOADS.items()}
    dosod = payloads["dosod_hbm"]
    compile_path = _write(tmp_path, "compile.json", json.dumps({
        "receipt_id": "tzcup_s100p_dosod_hbm_compile_receipt_v1", "status": "COMPILED_NOT_BOARD_ACCEPTED",
        "board_interaction_performed": False, "output_sha256": hashlib.sha256(dosod.read_bytes()).hexdigest(), "output_byte_size": dosod.stat().st_size,
    }).encode())
    session, closure = _session_and_closure(tmp_path)
    with pytest.raises(ValueError, match="canonical schema"):
        MODULE.build_receipt(
            artifact_root=artifact, candidate_stage="/opt/tzcup/stages/release-1", board_root=tmp_path,
            offline_compile_receipt=compile_path, acceptance_session=session, runtime_closure=closure,
            platform_machine=lambda: "aarch64", stat_path=_character_stat,
            compile_validator=lambda *_args, **_kwargs: False,
        )
    outside = tmp_path / "outside"
    outside.mkdir()
    linked = stage / "linked-artifacts"
    try:
        linked.symlink_to(outside, target_is_directory=True)
    except OSError:
        pytest.skip("directory symlink privilege unavailable")
    with pytest.raises(ValueError, match="non-link directory"):
        MODULE.build_receipt(
            artifact_root=linked, candidate_stage="/opt/tzcup/stages/release-1", board_root=tmp_path,
            offline_compile_receipt=compile_path, acceptance_session=session, runtime_closure=closure,
            platform_machine=lambda: "aarch64", stat_path=_character_stat, compile_validator=_accept_compile,
        )
