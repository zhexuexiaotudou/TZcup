#!/usr/bin/env python3
"""Freeze native AUTO-05 G3 capture for a clean, OCI-capable G4 checkout."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import ctypes
import errno
import hashlib
import json
import ntpath
import os
import stat
import subprocess
import tarfile
import tempfile
from pathlib import Path, PurePosixPath
from typing import Any, BinaryIO, Iterator


WORK_RELATIVE = ".work/auto05-g4"
DATA_RELATIVE = f"{WORK_RELATIVE}/data/g3_screening_native"
CONTRACT_RELATIVE = "starter_ws/src/sanitation_learning/config/auto05_g4_screening.yaml"
RUNTIME_NAME = "runtime_gate_binding.json"
CAPTURE_NAME = "capture_complete.json"
IMPORT_NAME = "cross_host_import.json"
OCI_NAME = "screening_image.json"
NATIVE_FRAMES = 1200
FRAME_STREAMS = 5
# 640x480 native RGB/depth/semantic/instance/TF records plus bounded capture
# sidecars.  These ceilings are deliberately below an unbounded tar payload.
MAX_ARCHIVE_MEMBERS = NATIVE_FRAMES * (FRAME_STREAMS + 1) + 1024
MAX_MEMBER_BYTES = 16 * 1024 * 1024
MAX_ARCHIVE_BYTES = 8 * 1024 * 1024 * 1024
MAX_MANIFEST_BYTES = 1024 * 1024
COPY_BLOCK_BYTES = 1024 * 1024


def fail(message: str) -> None:
    raise ValueError(message)


def identity(status: os.stat_result) -> tuple[int, int, int, int]:
    return status.st_dev, status.st_ino, status.st_size, status.st_mtime_ns


def directory_identity(status: os.stat_result) -> tuple[int, int]:
    return status.st_dev, status.st_ino


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with regular_stream(path) as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    return read_json_with_digest(path)[0]


def read_json_with_digest(path: Path) -> tuple[dict[str, Any], str]:
    try:
        with regular_stream(path) as stream:
            content = stream.read()
        value = json.loads(content.decode("utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        fail(f"cannot read JSON object {path}: {exc}")
    if not isinstance(value, dict):
        fail(f"JSON root must be an object: {path}")
    return value, hashlib.sha256(content).hexdigest()


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def object_sha256(value: object) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()


def require_regular(path: Path) -> None:
    try:
        mode = path.lstat().st_mode
    except OSError as exc:
        fail(f"cannot stat {path}: {exc}")
    if not stat.S_ISREG(mode):
        fail(f"handoff rejects link or special file: {path}")


@contextmanager
def regular_stream(path: Path, expected: tuple[int, int, int, int] | None = None) -> Iterator[BinaryIO]:
    """Open one stable, non-link regular file for validation and consumption."""
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    descriptor = -1
    try:
        before = path.lstat()
        if not stat.S_ISREG(before.st_mode):
            fail(f"handoff rejects link or special file: {path}")
        descriptor = os.open(path, flags)
    except OSError as exc:
        fail(f"cannot open regular handoff input {path}: {exc}")
    try:
        opened = os.fstat(descriptor)
        if not stat.S_ISREG(opened.st_mode) or identity(path.lstat()) != identity(opened) or (expected and identity(opened) != expected):
            fail(f"handoff rejects link or special file: {path}")
        with os.fdopen(descriptor, "rb", closefd=True) as stream:
            descriptor = -1
            yield stream
            if identity(os.fstat(stream.fileno())) != identity(opened):
                fail(f"handoff input changed while read: {path}")
    finally:
        if descriptor >= 0:
            os.close(descriptor)


def require_directory(path: Path) -> None:
    try:
        mode = path.lstat().st_mode
    except OSError as exc:
        fail(f"cannot stat {path}: {exc}")
    if not stat.S_ISDIR(mode):
        fail(f"handoff rejects missing, link, or special directory: {path}")


def stable_work_parent(parent: Path, repository: Path) -> tuple[tuple[int, int], Path, int | None]:
    """Create and pin the only parent into which import may atomically land."""
    parent.mkdir(parents=True, exist_ok=True)
    require_directory(parent)
    resolved = parent.resolve()
    try:
        resolved.relative_to(repository.resolve())
    except ValueError:
        fail("AUTO-05 import parent escapes its repository")
    expected = directory_identity(parent.lstat())
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(parent, flags)
    except OSError as exc:
        fail(f"cannot pin AUTO-05 import parent: {exc}")
    if not stat.S_ISDIR(os.fstat(descriptor).st_mode) or directory_identity(os.fstat(descriptor)) != expected:
        os.close(descriptor)
        fail("AUTO-05 import parent changed while pinned")
    return expected, resolved, descriptor


def require_stable_work_parent(parent: Path, repository: Path, expected: tuple[int, int], resolved: Path, descriptor: int | None) -> None:
    require_directory(parent)
    if directory_identity(parent.lstat()) != expected or parent.resolve() != resolved:
        fail("AUTO-05 import parent changed during handoff")
    if descriptor is None or not stat.S_ISDIR(os.fstat(descriptor).st_mode) or directory_identity(os.fstat(descriptor)) != expected:
        fail("AUTO-05 import parent descriptor changed during handoff")
    try:
        resolved.relative_to(repository.resolve())
    except ValueError:
        fail("AUTO-05 import parent escapes its repository")


def repository_relative(path: Path, repo: Path) -> str:
    try:
        return path.resolve().relative_to(repo.resolve()).as_posix()
    except ValueError as exc:
        fail(f"path escapes repository identity: {path}")
        raise AssertionError from exc


def git(repo: Path, *args: str) -> str:
    try:
        return subprocess.check_output(["git", "-C", str(repo), *args], text=True).strip()
    except subprocess.CalledProcessError as exc:
        fail(f"cannot resolve repository identity: {exc}")
        raise AssertionError from exc


def clean_source(repo: Path) -> dict[str, str]:
    if git(repo, "status", "--porcelain"):
        fail("cross-host handoff requires a clean source checkout")
    return {"head": git(repo, "rev-parse", "HEAD"), "tree": git(repo, "rev-parse", "HEAD^{tree}")}


def safe_member(name: str) -> str:
    item = PurePosixPath(name)
    reserved = {"con", "prn", "aux", "nul", *(f"com{index}" for index in range(1, 10)), *(f"lpt{index}" for index in range(1, 10))}
    if (
        not name or name.startswith("/") or "\\" in name
        or any(
            part in ("", ".", "..") or ":" in part or ntpath.splitdrive(part)[0]
            or part.rstrip(". ") != part or part.split(".", 1)[0].casefold() in reserved
            for part in item.parts
        )
    ):
        fail(f"unsafe archive member: {name!r}")
    return item.as_posix()


def validate_runtime(runtime: dict[str, Any], source: dict[str, str], contract_sha: str) -> None:
    formal = runtime.get("formal_runtime_gate", {})
    session = formal.get("acceptance_session_binding", {})
    closure = formal.get("runtime_closure_binding", {})
    if (
        runtime.get("status") != "AUTO05_G4_RUNTIME_GATE_BOUND"
        or runtime.get("git") != source
        or runtime.get("contract", {}).get("repository_relative") != CONTRACT_RELATIVE
        or runtime.get("contract", {}).get("sha256") != contract_sha
        or runtime.get("capture", {}).get("data_root_repository_relative") != DATA_RELATIVE
        or formal.get("status") != "FORMAL_RUNTIME_GATE_BOUND"
        or session.get("session_status_at_gate") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
        or closure.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"
        or not isinstance(session.get("session_manifest_sha256"), str)
        or not isinstance(closure.get("manifest_sha256"), str)
        or not isinstance(closure.get("closure_sha256"), str)
        or not runtime.get("capture", {}).get("single_gazebo_lock")
    ):
        fail("runtime binding does not prove exact source, closure, and RUNNING session")


def validate_capture(data: Path, runtime_path: Path, capture_path: Path, source: dict[str, str], contract_sha: str) -> tuple[dict[str, Any], dict[str, Any]]:
    require_directory(data)
    require_regular(runtime_path)
    require_regular(capture_path)
    runtime = read_json(runtime_path)
    validate_runtime(runtime, source, contract_sha)
    capture = read_json(capture_path)
    provenance = capture.get("capture_provenance", {})
    reports = sorted(data.glob("scenes/*/capture_report.json"))
    world_manifest = data / "worlds" / "g3_world_manifest.json"
    world_generation = data / "world_generation.json"
    for path in (world_manifest, world_generation, *reports):
        require_regular(path)
    if (
        capture.get("status") != "AUTO05_G4_CAPTURE_COMPLETE"
        or capture.get("runtime_binding_sha256") != sha256(runtime_path)
        or capture.get("data_root_repository_relative") != DATA_RELATIVE
        or provenance != {
            "mode": "fresh_native_gazebo_g3_capture",
            "replay_input_used": False,
            "synthetic_substitution_used": False,
        }
        or len(reports) != 120
        or capture.get("capture_report_count") != 120
        or capture.get("world_generation_sha256") != sha256(world_generation)
        or capture.get("capture_report_sha256") != {path.relative_to(data).as_posix(): sha256(path) for path in reports}
    ):
        fail("capture receipt is incomplete, replayed, substituted, or mismatched")
    worlds = read_json(world_manifest).get("worlds")
    if not isinstance(worlds, list) or len(worlds) != 8:
        fail("handoff requires exactly eight G3 worlds")
    records = []
    for report_path in reports:
        report = read_json(report_path)
        if report.get("capture_pass") is not True or report.get("captured_frames") != 10 or len(report.get("records", [])) != 10:
            fail(f"handoff requires ten passed native frames per scene: {report_path}")
        records.extend(report["records"])
    if len(records) != 1200:
        fail("handoff requires exactly 1200 native frames")
    return runtime, capture


def data_files(data: Path, runtime_path: Path, capture_path: Path) -> dict[str, tuple[Path, tuple[int, int, int, int]]]:
    result = {
        f"payload/evidence/{RUNTIME_NAME}": (runtime_path, identity(runtime_path.lstat())),
        f"payload/evidence/{CAPTURE_NAME}": (capture_path, identity(capture_path.lstat())),
    }
    for path in sorted(data.rglob("*")):
        if path.is_dir():
            if path.is_symlink():
                fail(f"handoff rejects symbolic-link directory: {path}")
            continue
        require_regular(path)
        result[f"payload/data/g3_screening_native/{path.relative_to(data).as_posix()}"] = (path, identity(path.lstat()))
    return result


class HashingReader:
    def __init__(self, stream: BinaryIO) -> None:
        self.stream, self.digest = stream, hashlib.sha256()

    def read(self, size: int = -1) -> bytes:
        block = self.stream.read(size)
        self.digest.update(block)
        return block


def add_file(archive: tarfile.TarFile, name: str, source: tuple[Path, tuple[int, int, int, int]]) -> str:
    path, expected = source
    with regular_stream(path, expected) as stream:
        info = tarfile.TarInfo(name)
        info.size, info.mode, info.mtime = expected[2], 0o644, 0
        info.uid = info.gid = 0
        info.uname = info.gname = ""
        reader = HashingReader(stream)
        archive.addfile(info, reader)
        return reader.digest.hexdigest()


def export_handoff(args: argparse.Namespace) -> int:
    repo, archive = Path(args.repo).resolve(), Path(args.archive).absolute()
    root, data = repo / WORK_RELATIVE, repo / DATA_RELATIVE
    runtime_path, capture_path = root / "evidence" / RUNTIME_NAME, root / "evidence" / CAPTURE_NAME
    receipt = archive.with_name(archive.name + ".receipt.json")
    if archive.exists() or receipt.exists():
        fail("refusing to overwrite handoff archive or receipt")
    source = clean_source(repo)
    contract = repo / CONTRACT_RELATIVE
    require_regular(contract)
    runtime, _capture = validate_capture(data, runtime_path, capture_path, source, sha256(contract))
    files = data_files(data, runtime_path, capture_path)
    manifest = {
        "schema_version": 1,
        "kind": "AUTO05_G4_CROSS_HOST_G3_HANDOFF",
        "source": source,
        "contract": {"repository_relative": CONTRACT_RELATIVE, "sha256": sha256(contract)},
        "capture": {
            "data_root_repository_relative": DATA_RELATIVE,
            "runtime_binding_sha256": sha256(runtime_path),
            "formal_runtime_gate_status": runtime["formal_runtime_gate"]["status"],
            "session_status_at_capture": runtime["formal_runtime_gate"]["acceptance_session_binding"]["session_status_at_gate"],
            "session_manifest_sha256": runtime["formal_runtime_gate"]["acceptance_session_binding"]["session_manifest_sha256"],
            "runtime_closure_status": runtime["formal_runtime_gate"]["runtime_closure_binding"]["status"],
            "runtime_closure_manifest_sha256": runtime["formal_runtime_gate"]["runtime_closure_binding"]["manifest_sha256"],
            "runtime_closure_sha256": runtime["formal_runtime_gate"]["runtime_closure_binding"]["closure_sha256"],
        },
        "files": {},
    }
    archive.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(dir=archive.parent, prefix=".auto05-g4-export-", suffix=".tar.gz")
    os.close(descriptor)
    temporary = Path(temporary_name)
    temporary.unlink()
    try:
        with tarfile.open(temporary, "x:gz", format=tarfile.PAX_FORMAT) as output:
            for name, source_file in files.items():
                manifest["files"][name] = add_file(output, name, source_file)
            manifest["capture"]["runtime_binding_sha256"] = manifest["files"][f"payload/evidence/{RUNTIME_NAME}"]
            manifest_bytes = (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode("utf-8")
            info = tarfile.TarInfo("handoff_manifest.json")
            info.size, info.mode, info.mtime = len(manifest_bytes), 0o644, 0
            output.addfile(info, __import__("io").BytesIO(manifest_bytes))
        os.replace(temporary, archive)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    write_json(receipt, {
        "schema_version": 1,
        "status": "AUTO05_G4_CROSS_HOST_G3_EXPORTED",
        "archive_sha256": sha256(archive),
        "handoff_manifest_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
        "source": source,
        "contract_sha256": sha256(contract),
        "data_root_repository_relative": DATA_RELATIVE,
    })
    return 0


def stream_sha256(stream: BinaryIO) -> str:
    digest = hashlib.sha256()
    for block in iter(lambda: stream.read(COPY_BLOCK_BYTES), b""):
        digest.update(block)
    return digest.hexdigest()


def read_member(stream: BinaryIO, size: int) -> bytes:
    if size > MAX_MANIFEST_BYTES:
        fail("handoff manifest exceeds its fixed size limit")
    value = stream.read(size)
    if len(value) != size or stream.read(1):
        fail("handoff archive member size is invalid")
    return value


def preflight_archive(stream: BinaryIO, receipt: dict[str, Any]) -> dict[str, Any]:
    stream.seek(0)
    if receipt.get("archive_sha256") != stream_sha256(stream):
        fail("handoff archive digest does not match its receipt")
    try:
        stream.seek(0)
        with tarfile.open(fileobj=stream, mode="r:*") as archive:
            names: list[str] = []
            folded: set[str] = set()
            total = 0
            raw_manifest = None
            for index, member in enumerate(archive):
                name = safe_member(member.name)
                if (
                    index >= MAX_ARCHIVE_MEMBERS or name.casefold() in folded
                    or not member.isreg() or member.issparse()
                    or member.size < 0 or member.size > MAX_MEMBER_BYTES
                ):
                    fail("handoff archive contains duplicate, sparse, link, device, or oversized member")
                total += member.size
                if total > MAX_ARCHIVE_BYTES:
                    fail("handoff archive exceeds fixed native-G3 byte limit")
                folded.add(name.casefold())
                names.append(name)
                if name == "handoff_manifest.json":
                    if raw_manifest is not None:
                        fail("handoff archive has duplicate manifest")
                    member_stream = archive.extractfile(member)
                    if member_stream is None:
                        fail(f"cannot read archive member: {name}")
                    raw_manifest = read_member(member_stream, member.size)
            if raw_manifest is None:
                fail("handoff archive lacks manifest")
    except (tarfile.TarError, OSError) as exc:
        fail(f"cannot read handoff archive: {exc}")
    if receipt.get("handoff_manifest_sha256") != hashlib.sha256(raw_manifest).hexdigest():
        fail("handoff manifest digest does not match receipt")
    try:
        manifest = json.loads(raw_manifest.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        fail(f"handoff manifest is invalid: {exc}")
    if not isinstance(manifest, dict):
        fail("handoff manifest root is not an object")
    expected = manifest.get("files")
    archive_files = set(names)
    archive_files.discard("handoff_manifest.json")
    if not isinstance(expected, dict) or set(expected) != archive_files:
        fail("handoff archive members differ from its manifest")
    if any(not isinstance(value, str) or len(value) != 64 for value in expected.values()):
        fail("handoff manifest has invalid file hashes")
    return manifest


def validate_oci(receipt_path: Path, source: dict[str, str]) -> tuple[dict[str, Any], bytes]:
    require_regular(receipt_path)
    with regular_stream(receipt_path) as stream:
        content = stream.read()
    try:
        receipt = json.loads(content.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        fail(f"OCI receipt is invalid: {exc}")
    if (
        not isinstance(receipt, dict)
        or receipt.get("status") != "AUTO05_G4_SCREENING_IMAGE_BUILT"
        or receipt.get("runtime_parity_test_passed") is not True
        or receipt.get("git") != source
        or not isinstance(receipt.get("image_tag"), str)
        or not receipt["image_tag"]
        or not isinstance(receipt.get("image_id"), str)
        or not receipt["image_id"].startswith("sha256:")
    ):
        fail("OCI receipt is not a valid same-source G4 screening image receipt")
    return receipt, content


def validate_manifest_for_target(manifest: dict[str, Any], repo: Path) -> dict[str, str]:
    source = clean_source(repo)
    contract = repo / CONTRACT_RELATIVE
    require_regular(contract)
    if (
        manifest.get("schema_version") != 1
        or manifest.get("kind") != "AUTO05_G4_CROSS_HOST_G3_HANDOFF"
        or manifest.get("source") != source
        or manifest.get("contract") != {"repository_relative": CONTRACT_RELATIVE, "sha256": sha256(contract)}
        or manifest.get("capture", {}).get("data_root_repository_relative") != DATA_RELATIVE
        or manifest.get("capture", {}).get("formal_runtime_gate_status") != "FORMAL_RUNTIME_GATE_BOUND"
        or manifest.get("capture", {}).get("session_status_at_capture") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
        or manifest.get("capture", {}).get("runtime_closure_status") != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"
        or not isinstance(manifest.get("capture", {}).get("session_manifest_sha256"), str)
        or not isinstance(manifest.get("capture", {}).get("runtime_closure_manifest_sha256"), str)
        or not isinstance(manifest.get("capture", {}).get("runtime_closure_sha256"), str)
    ):
        fail("handoff source, frozen contract, session, or closure does not match target")
    return source


def write_payload(root: Path, repository: Path, parent_state: tuple[tuple[int, int], Path, int | None], archive_stream: BinaryIO, manifest: dict[str, Any], oci_bytes: bytes, marker: dict[str, Any]) -> None:
    parent_identity, parent_resolved, descriptor = parent_state
    require_stable_work_parent(root.parent, repository, parent_identity, parent_resolved, descriptor)
    temporary_parent = Path(f"/proc/self/fd/{descriptor}")
    with tempfile.TemporaryDirectory(dir=temporary_parent, prefix=".auto05-g4-import-") as temporary:
        require_stable_work_parent(root.parent, repository, parent_identity, parent_resolved, descriptor)
        staging = Path(temporary) / "auto05-g4"
        archive_stream.seek(0)
        try:
            with tarfile.open(fileobj=archive_stream, mode="r:*") as archive:
                for member in archive:
                    name = safe_member(member.name)
                    if name == "handoff_manifest.json":
                        continue
                    if not name.startswith("payload/"):
                        fail(f"handoff archive member has invalid logical prefix: {name}")
                    relative = PurePosixPath(name).relative_to("payload")
                    destination = staging.joinpath(*relative.parts)
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    try:
                        destination.parent.resolve().relative_to(staging.resolve())
                    except ValueError:
                        fail("handoff archive path escapes temporary staging")
                    for parent in (staging, *destination.parents):
                        if parent == staging.parent:
                            break
                        if parent.is_symlink() or not parent.is_dir():
                            fail("handoff staging path contains link or non-directory")
                    member_stream = archive.extractfile(member)
                    if member_stream is None:
                        fail(f"cannot read archive member: {name}")
                    digest = hashlib.sha256()
                    written = 0
                    with destination.open("xb") as output:
                        for block in iter(lambda: member_stream.read(COPY_BLOCK_BYTES), b""):
                            written += len(block)
                            if written > member.size:
                                fail("handoff archive member exceeds declared size")
                            digest.update(block)
                            output.write(block)
                    if written != member.size or digest.hexdigest() != manifest["files"].get(name):
                        fail(f"handoff file hash mismatch: {name}")
        except (tarfile.TarError, OSError) as exc:
            fail(f"cannot extract handoff archive: {exc}")
        evidence = staging / "evidence"
        evidence.mkdir(parents=True, exist_ok=True)
        (evidence / OCI_NAME).write_bytes(oci_bytes)
        write_json(evidence / IMPORT_NAME, marker)
        require_stable_work_parent(root.parent, repository, parent_identity, parent_resolved, descriptor)
        rename_noreplace(staging, root.name, descriptor)
        require_stable_work_parent(root.parent, repository, parent_identity, parent_resolved, descriptor)
        try:
            root.resolve().relative_to(parent_resolved)
        except ValueError:
            fail("AUTO-05 import root escaped its pinned parent")


def rename_noreplace(source: Path, target_name: str, parent_fd: int) -> None:
    """Linux renameat2 is the only acceptable final handoff commit primitive."""
    try:
        function = ctypes.CDLL(None, use_errno=True).renameat2
    except AttributeError:
        fail("renameat2(RENAME_NOREPLACE) is unavailable; refusing cross-host import")
    function.argtypes = (ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint)
    function.restype = ctypes.c_int
    if function(-100, os.fsencode(source), parent_fd, os.fsencode(target_name), 1) != 0:
        code = ctypes.get_errno()
        if code == errno.EEXIST:
            fail("refusing to overwrite existing AUTO-05 G4 work root")
        fail(f"renameat2(RENAME_NOREPLACE) failed: {os.strerror(code)}")


def import_handoff(args: argparse.Namespace) -> int:
    if os.name != "posix":
        fail("AUTO-05 cross-host handoff is Linux/WSL-only; use a native Linux checkout")
    repo = Path(args.repo).resolve()
    root = repo / WORK_RELATIVE
    archive, receipt_path, oci_path = Path(args.archive).absolute(), Path(args.receipt).absolute(), Path(args.oci_receipt).absolute()
    if root.exists() or root.is_symlink() or root.parent.is_symlink():
        fail("refusing to overwrite existing AUTO-05 G4 work root")
    parent_state = stable_work_parent(root.parent, repo)
    require_regular(receipt_path)
    receipt, receipt_sha256 = read_json_with_digest(receipt_path)
    try:
        with regular_stream(archive) as archive_stream:
            manifest = preflight_archive(archive_stream, receipt)
            source = validate_manifest_for_target(manifest, repo)
            if (
                receipt.get("schema_version") != 1
                or receipt.get("status") != "AUTO05_G4_CROSS_HOST_G3_EXPORTED"
                or receipt.get("source") != manifest.get("source")
                or receipt.get("contract_sha256") != manifest.get("contract", {}).get("sha256")
                or receipt.get("data_root_repository_relative") != DATA_RELATIVE
            ):
                fail("handoff receipt does not match its manifest")
            _oci, oci_bytes = validate_oci(oci_path, source)
            marker = {
                "schema_version": 1,
                "status": "AUTO05_G4_CROSS_HOST_IMPORTED",
                "archive_sha256": receipt["archive_sha256"],
                "handoff_manifest_sha256": receipt["handoff_manifest_sha256"],
                "source": source,
                "contract_sha256": manifest["contract"]["sha256"],
                "data_root_repository_relative": DATA_RELATIVE,
                "files": manifest["files"],
                "inventory_sha256": object_sha256(manifest["files"]),
                "handoff_receipt_sha256": receipt_sha256,
                "oci_receipt_sha256": hashlib.sha256(oci_bytes).hexdigest(),
            }
            write_payload(root, repo, parent_state, archive_stream, manifest, oci_bytes, marker)
    finally:
        if parent_state[2] is not None:
            os.close(parent_state[2])
    return 0


def verify_import(args: argparse.Namespace) -> int:
    repo = Path(args.repo).resolve()
    root = repo / WORK_RELATIVE
    marker_path = root / "evidence" / IMPORT_NAME
    require_regular(marker_path)
    marker = read_json(marker_path)
    source = clean_source(repo)
    if marker.get("status") != "AUTO05_G4_CROSS_HOST_IMPORTED" or marker.get("source") != source:
        fail("import marker does not match the clean target source")
    contract = repo / CONTRACT_RELATIVE
    if marker.get("contract_sha256") != sha256(contract) or marker.get("data_root_repository_relative") != DATA_RELATIVE:
        fail("import marker does not match the frozen G4 contract")
    _oci, oci_bytes = validate_oci(Path(args.oci_receipt).absolute(), source)
    if marker.get("oci_receipt_sha256") != hashlib.sha256(oci_bytes).hexdigest():
        fail("import marker does not bind this OCI receipt")
    files = marker.get("files")
    if not isinstance(files, dict):
        fail("import marker lacks file inventory")
    if marker.get("inventory_sha256") != object_sha256(files) or not isinstance(marker.get("handoff_receipt_sha256"), str):
        fail("import marker has no immutable receipt/inventory binding")
    expected_data: set[str] = set()
    for name, expected in files.items():
        safe_member(name)
        if not name.startswith("payload/"):
            fail("import marker has invalid logical identity")
        path = root.joinpath(*PurePosixPath(name).relative_to("payload").parts)
        require_regular(path)
        if sha256(path) != expected:
            fail(f"imported file hash mismatch: {name}")
        if name.startswith("payload/data/"):
            expected_data.add(PurePosixPath(name).relative_to("payload/data/g3_screening_native").as_posix())
    data_root = root / "data" / "g3_screening_native"
    actual_data: set[str] = set()
    for path in data_root.rglob("*"):
        if path.is_dir():
            if path.is_symlink():
                fail("imported raw tree contains symbolic-link directory")
            continue
        require_regular(path)
        actual_data.add(path.relative_to(data_root).as_posix())
    if actual_data != expected_data:
        fail("imported raw tree has missing or extra files")
    runtime_path = root / "evidence" / RUNTIME_NAME
    capture_path = root / "evidence" / CAPTURE_NAME
    validate_capture(root / "data" / "g3_screening_native", runtime_path, capture_path, source, sha256(contract))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    export = commands.add_parser("export", help="freeze a native capture into a hash-bound archive")
    export.add_argument("--repo", required=True)
    export.add_argument("--archive", required=True)
    imported = commands.add_parser("import", help="verify and install a capture in a clean OCI-capable checkout")
    imported.add_argument("--repo", required=True)
    imported.add_argument("--archive", required=True)
    imported.add_argument("--receipt", required=True)
    imported.add_argument("--oci-receipt", required=True)
    verify = commands.add_parser("verify-import", help="recheck an imported handoff before G4 QA")
    verify.add_argument("--repo", required=True)
    verify.add_argument("--oci-receipt", required=True)
    args = parser.parse_args()
    try:
        return {"export": export_handoff, "import": import_handoff, "verify-import": verify_import}[args.command](args)
    except ValueError as exc:
        parser.error(str(exc))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
