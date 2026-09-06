#!/usr/bin/env python3
"""Dependency-free regressions for the native-G3 to OCI-G4 handoff boundary."""

from __future__ import annotations

import hashlib
import gzip
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import tarfile
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
HANDOFF = ROOT / "scripts" / "auto05_g4_cross_host_handoff.py"
CONTRACT = "starter_ws/src/sanitation_learning/config/auto05_g4_screening.yaml"
spec = importlib.util.spec_from_file_location("auto05_g4_cross_host_handoff", HANDOFF)
assert spec and spec.loader
handoff = importlib.util.module_from_spec(spec)
spec.loader.exec_module(handoff)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def git(repo: Path, *args: str) -> str:
    return subprocess.check_output(["git", "-C", str(repo), *args], text=True).strip()


def run(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run([sys.executable, str(HANDOFF), *args], text=True, capture_output=True)


def source_repo(tmp_path: Path) -> tuple[Path, Path, Path]:
    if os.name != "posix":
        pytest.skip("cross-host handoff is intentionally Linux/WSL-only")
    repo = tmp_path / "capture"
    (repo / CONTRACT).parent.mkdir(parents=True)
    shutil.copyfile(ROOT / CONTRACT, repo / CONTRACT)
    (repo / ".gitignore").write_text(".work/\n", encoding="utf-8")
    subprocess.run(["git", "init", str(repo)], check=True, capture_output=True)
    subprocess.run(["git", "-C", str(repo), "config", "user.email", "test@example.invalid"], check=True)
    subprocess.run(["git", "-C", str(repo), "config", "user.name", "test"], check=True)
    subprocess.run(["git", "-C", str(repo), "add", "."], check=True)
    subprocess.run(["git", "-C", str(repo), "commit", "-m", "fixture"], check=True, capture_output=True)
    source = {"head": git(repo, "rev-parse", "HEAD"), "tree": git(repo, "rev-parse", "HEAD^{tree}")}
    contract = repo / CONTRACT
    data = repo / ".work/auto05-g4/data/g3_screening_native"
    reports: list[Path] = []
    for index in range(120):
        report = data / "scenes" / f"scene_{index:04d}" / "capture_report.json"
        write_json(report, {"capture_pass": True, "captured_frames": 10, "records": [{"frame": frame} for frame in range(10)]})
        reports.append(report)
    write_json(data / "worlds/g3_world_manifest.json", {"worlds": [{"world_id": f"world_{index}"} for index in range(8)]})
    write_json(data / "world_generation.json", {"generated": True})
    runtime = {
        "status": "AUTO05_G4_RUNTIME_GATE_BOUND", "git": source,
        "contract": {"repository_relative": CONTRACT, "sha256": sha256(contract)},
        "capture": {"data_root_repository_relative": ".work/auto05-g4/data/g3_screening_native", "single_gazebo_lock": "/tmp/lock"},
        "formal_runtime_gate": {
            "status": "FORMAL_RUNTIME_GATE_BOUND",
            "acceptance_session_binding": {"session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING", "session_manifest_sha256": "2" * 64},
            "runtime_closure_binding": {"status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED", "manifest_sha256": "3" * 64, "closure_sha256": "4" * 64},
        },
    }
    runtime_path = repo / ".work/auto05-g4/evidence/runtime_gate_binding.json"
    write_json(runtime_path, runtime)
    write_json(repo / ".work/auto05-g4/evidence/capture_complete.json", {
        "status": "AUTO05_G4_CAPTURE_COMPLETE", "runtime_binding_sha256": sha256(runtime_path),
        "data_root_repository_relative": ".work/auto05-g4/data/g3_screening_native",
        "capture_provenance": {"mode": "fresh_native_gazebo_g3_capture", "replay_input_used": False, "synthetic_substitution_used": False},
        "world_generation_sha256": sha256(data / "world_generation.json"), "capture_report_count": 120,
        "capture_report_sha256": {path.relative_to(data).as_posix(): sha256(path) for path in reports},
    })
    archive = tmp_path / "g3-native.tar.gz"
    result = run("export", "--repo", str(repo), "--archive", str(archive))
    assert result.returncode == 0, result.stderr
    target = tmp_path / "target"
    subprocess.run(["git", "-c", "core.autocrlf=false", "clone", str(repo), str(target)], check=True, capture_output=True)
    oci = tmp_path / "screening_image.json"
    write_json(oci, {"status": "AUTO05_G4_SCREENING_IMAGE_BUILT", "runtime_parity_test_passed": True,
                     "git": source, "image_tag": "tzcup/g4:test", "image_id": "sha256:" + "1" * 64})
    return target, archive, oci


def test_imports_hash_bound_native_capture_into_same_source_checkout(tmp_path: Path) -> None:
    target, archive, oci = source_repo(tmp_path)
    receipt = archive.with_name(archive.name + ".receipt.json")
    result = run("import", "--repo", str(target), "--archive", str(archive), "--receipt", str(receipt), "--oci-receipt", str(oci))
    assert result.returncode == 0, result.stderr
    result = run("verify-import", "--repo", str(target), "--oci-receipt", str(target / ".work/auto05-g4/evidence/screening_image.json"))
    assert result.returncode == 0, result.stderr
    marker = json.loads((target / ".work/auto05-g4/evidence/cross_host_import.json").read_text(encoding="utf-8"))
    assert marker["data_root_repository_relative"] == ".work/auto05-g4/data/g3_screening_native"
    assert all(not Path(name).is_absolute() for name in marker["files"])


@pytest.mark.parametrize("mutation", ["archive", "oci", "replay", "session", "closure"])
def test_import_rejects_tamper_or_replay_substitution(tmp_path: Path, mutation: str) -> None:
    target, archive, oci = source_repo(tmp_path)
    receipt = archive.with_name(archive.name + ".receipt.json")
    if mutation == "archive":
        archive.write_bytes(archive.read_bytes() + b"tamper")
    elif mutation == "oci":
        value = json.loads(oci.read_text(encoding="utf-8")); value["git"]["head"] = "0" * 40; write_json(oci, value)
    else:
        source = tmp_path / "capture"
        if mutation in {"session", "closure"}:
            runtime = source / ".work/auto05-g4/evidence/runtime_gate_binding.json"
            runtime_value = json.loads(runtime.read_text(encoding="utf-8"))
            if mutation == "session":
                runtime_value["formal_runtime_gate"]["acceptance_session_binding"]["session_status_at_gate"] = "STOPPED"
            else:
                runtime_value["formal_runtime_gate"]["runtime_closure_binding"]["status"] = "UNVERIFIED"
            write_json(runtime, runtime_value)
        capture = source / ".work/auto05-g4/evidence/capture_complete.json"
        value = json.loads(capture.read_text(encoding="utf-8"))
        if mutation == "replay":
            value["capture_provenance"]["replay_input_used"] = True
        else:
            value["runtime_binding_sha256"] = sha256(source / ".work/auto05-g4/evidence/runtime_gate_binding.json")
        write_json(capture, value)
        second = tmp_path / f"{mutation}.tar.gz"
        result = run("export", "--repo", str(source), "--archive", str(second))
        assert result.returncode != 0
        return
    result = run("import", "--repo", str(target), "--archive", str(archive), "--receipt", str(receipt), "--oci-receipt", str(oci))
    assert result.returncode != 0


def test_import_rejects_repeat_and_link_archive(tmp_path: Path) -> None:
    target, archive, oci = source_repo(tmp_path)
    receipt = archive.with_name(archive.name + ".receipt.json")
    first = run("import", "--repo", str(target), "--archive", str(archive), "--receipt", str(receipt), "--oci-receipt", str(oci))
    assert first.returncode == 0, first.stderr
    repeat = run("import", "--repo", str(target), "--archive", str(archive), "--receipt", str(receipt), "--oci-receipt", str(oci))
    assert repeat.returncode != 0
    malicious = tmp_path / "link.tar.gz"
    with tarfile.open(malicious, "w:gz") as output:
        link = tarfile.TarInfo("payload/link")
        link.type = tarfile.SYMTYPE
        link.linkname = "outside"
        output.addfile(link)
    malicious_receipt = malicious.with_name(malicious.name + ".receipt.json")
    write_json(malicious_receipt, {"archive_sha256": sha256(malicious), "handoff_manifest_sha256": "0" * 64})
    another = tmp_path / "another"
    subprocess.run(["git", "-c", "core.autocrlf=false", "clone", str(tmp_path / "capture"), str(another)], check=True, capture_output=True)
    rejected = run("import", "--repo", str(another), "--archive", str(malicious), "--receipt", str(malicious_receipt), "--oci-receipt", str(oci))
    assert rejected.returncode != 0


@pytest.mark.parametrize("member", ["/absolute", "payload/../escape", "payload/C:/escape", "payload/NUL.txt", "payload/name. "])
def test_import_rejects_windows_and_traversal_members(tmp_path: Path, member: str) -> None:
    with pytest.raises(ValueError):
        handoff.safe_member(member)
    target, _archive, oci = source_repo(tmp_path)
    malicious = tmp_path / "unsafe.tar.gz"
    with tarfile.open(malicious, "w:gz") as output:
        info = tarfile.TarInfo(member); info.size = 1; output.addfile(info, __import__("io").BytesIO(b"x"))
    receipt = malicious.with_name(malicious.name + ".receipt.json")
    write_json(receipt, {"archive_sha256": sha256(malicious), "handoff_manifest_sha256": "0" * 64})
    result = run("import", "--repo", str(target), "--archive", str(malicious), "--receipt", str(receipt), "--oci-receipt", str(oci))
    assert result.returncode != 0


def test_import_rejects_declared_tar_bomb_before_extracting(tmp_path: Path) -> None:
    target, _archive, oci = source_repo(tmp_path)
    bomb = tmp_path / "bomb.tar.gz"
    info = tarfile.TarInfo("payload/too-large")
    info.size = handoff.MAX_MEMBER_BYTES + 1
    with gzip.open(bomb, "wb") as stream:
        stream.write(info.tobuf())
    receipt = bomb.with_name(bomb.name + ".receipt.json")
    write_json(receipt, {"archive_sha256": sha256(bomb), "handoff_manifest_sha256": "0" * 64})
    result = run("import", "--repo", str(target), "--archive", str(bomb), "--receipt", str(receipt), "--oci-receipt", str(oci))
    assert result.returncode != 0
    assert not (target / ".work/auto05-g4").exists()


def test_verify_import_rejects_extra_raw_file_and_open_race(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    target, archive, oci = source_repo(tmp_path)
    receipt = archive.with_name(archive.name + ".receipt.json")
    assert run("import", "--repo", str(target), "--archive", str(archive), "--receipt", str(receipt), "--oci-receipt", str(oci)).returncode == 0
    (target / ".work/auto05-g4/data/g3_screening_native/extra.bin").write_bytes(b"extra")
    assert run("verify-import", "--repo", str(target), "--oci-receipt", str(target / ".work/auto05-g4/evidence/screening_image.json")).returncode != 0
    path = tmp_path / "race.bin"; path.write_bytes(b"race")
    original, calls = handoff.identity, 0
    def raced(status):
        nonlocal calls
        calls += 1
        value = original(status)
        return (value[0], value[1], value[2] + 1, value[3]) if calls == 2 else value
    monkeypatch.setattr(handoff, "identity", raced)
    with pytest.raises(ValueError):
        with handoff.regular_stream(path):
            pass


def test_import_parent_identity_detects_replacement_race(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    if os.name != "posix":
        pytest.skip("dirfd parent pin is Linux/WSL-only")
    repository = tmp_path / "repo"; repository.mkdir()
    parent = repository / ".work"
    state = handoff.stable_work_parent(parent, repository)
    original = handoff.directory_identity
    monkeypatch.setattr(handoff, "directory_identity", lambda status: (original(status)[0], original(status)[1] + 1))
    with pytest.raises(ValueError, match="parent changed"):
        handoff.require_stable_work_parent(parent, repository, *state)


def test_rename_noreplace_rejects_real_destination_race(tmp_path: Path) -> None:
    if os.name != "posix":
        pytest.skip("renameat2 is Linux/WSL-only")
    parent = tmp_path / "parent"; parent.mkdir()
    staging = parent / "staging"; staging.mkdir()
    target = parent / "auto05-g4"; target.mkdir()
    descriptor = os.open(parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        with pytest.raises(ValueError, match="overwrite"):
            handoff.rename_noreplace(staging, target.name, descriptor)
    finally:
        os.close(descriptor)
    assert staging.is_dir() and target.is_dir()
