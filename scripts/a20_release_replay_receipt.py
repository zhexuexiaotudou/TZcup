#!/usr/bin/env python3
"""Validate an A20 release receipt against canonical product MCAP replays."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import secrets
import stat
import re
from pathlib import Path
from typing import Any
from types import SimpleNamespace

from formal_product_mcap_replay import validate_raw_capture_receipt
from validate_product_acceptance_contract import ProductAcceptanceContractError, load_contract, validate_auto15_execution_evidence
from run_formal_final_acceptance import OrchestrationError, _verify_complete_session_evidence


SCHEMA = "tzcup.a20_release_replay_receipt.v1"
BLOCKER = "canonical current-session formal product replay evidence is incomplete"
HEX64 = re.compile(r"^[0-9a-f]{64}$")
GIT40 = re.compile(r"^[0-9a-f]{40}$")
HASH_FIELDS = ("source", "model", "config", "dataset", "dependency")
REPLAY_CHECKS = frozenset({
    "mcap_metadata_readable", "required_product_and_metric_topics_present",
    "product_chain_observed", "coverage_recalculated_within_1_percent",
    "localization_recalculated_within_1_percent", "ros2_bag_play_exit_zero",
    "replay_uses_formal_safe_dds_domain", "replay_process_group_cleanup_proven",
    "ros2_executable_matches_runtime_closure",
    "current_formal_session_snapshot_closure_bound",
})


def _sealed_bytes(path: Path, label: str) -> bytes:
    flags = os.O_RDONLY | getattr(os, "O_BINARY", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise ValueError(f"{label} is not a regular file")
        chunks: list[bytes] = []
        while chunk := os.read(descriptor, 1024 * 1024):
            chunks.append(chunk)
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    named = os.lstat(path)
    if _identity(before) != _identity(after) or _identity(before) != _identity(named):
        raise ValueError(f"{label} changed while being read")
    return b"".join(chunks)


def _hash_ref(root: Path, value: Any, label: str) -> Path:
    if not isinstance(value, dict) or not isinstance(value.get("path"), str) or not HEX64.fullmatch(str(value.get("sha256", ""))):
        raise ValueError(f"{label} has no path and sha256 binding")
    path = _regular_in_root(root, Path(value["path"]), label)
    digest = hashlib.sha256(_sealed_bytes(path, label)).hexdigest()
    if digest != value["sha256"]:
        raise ValueError(f"{label} hash mismatch")
    return path


def _json_hash_ref(root: Path, value: Any, label: str) -> tuple[Path, dict[str, Any]]:
    if not isinstance(value, dict) or not isinstance(value.get("path"), str) or not HEX64.fullmatch(str(value.get("sha256", ""))):
        raise ValueError(f"{label} has no path and sha256 binding")
    path = _regular_in_root(root, Path(value["path"]), label)
    raw = _sealed_bytes(path, label)
    if hashlib.sha256(raw).hexdigest() != value["sha256"]:
        raise ValueError(f"{label} hash mismatch")
    payload = json.loads(raw.decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{label} JSON root is not an object")
    return path, payload


def _artifact_in_root(root: Path, candidate: Path, label: str) -> Path:
    if not candidate.is_absolute():
        raise ValueError(f"{label} must be absolute")
    try:
        relative = candidate.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"{label} escapes repository root") from exc
    current = root
    for part in relative.parts:
        current /= part
        if current.exists() and current.is_symlink():
            raise ValueError(f"{label} has a symbolic-link component")
    if not candidate.exists() or candidate.is_symlink() or not (candidate.is_file() or candidate.is_dir()):
        raise ValueError(f"{label} is missing or not a regular artifact")
    return candidate


def validate_receipt(receipt: dict[str, Any], repository_root: Path | None = None, expected_run_root: Path | None = None, expected_session: Path | None = None) -> dict[str, Any]:
    """Validate current retained files; embedded/stage-specific reports fail."""

    errors: list[str] = []
    root = repository_root.resolve() if repository_root is not None else None
    try:
        if root is None:
            raise ValueError("repository root is required for retained-file validation")
        if expected_run_root is None:
            raise ValueError("expected run root is required for retained-file validation")
        if expected_session is None:
            raise ValueError("expected formal session is required for retained-file validation")
        expected = _artifact_in_root(root, expected_run_root.resolve(), "expected run root")
        if not expected.is_dir():
            raise ValueError("expected run root is not a directory")
        if receipt.get("schema") != SCHEMA:
            raise ValueError("unsupported A20 receipt schema")
        ledger_path, ledger = _json_hash_ref(root, receipt.get("a12_ledger"), "verified A12 ledger")
        if ledger.get("run_root") != str(expected):
            raise ValueError("A12 ledger belongs to another run root")
        try:
            validate_auto15_execution_evidence(load_contract(), ledger, expected)
        except ProductAcceptanceContractError as exc:
            raise ValueError(f"A12 ledger failed revalidation: {exc}") from exc
        ledger_replay_hashes: set[str] = set()
        ledger_context: dict[str, Any] | None = None
        for item in ledger.get("executions", []):
            execution_path, execution = _json_hash_ref(root, item, "A12 execution receipt")
            del execution_path
            replay_ref = execution.get("replay")
            if not isinstance(replay_ref, dict) or not HEX64.fullmatch(str(replay_ref.get("sha256", ""))):
                raise ValueError("A12 execution lacks a canonical replay reference")
            ledger_replay_hashes.add(replay_ref["sha256"])
            if ledger_context is None:
                ledger_context = execution.get("formal_context")
            elif execution.get("formal_context") != ledger_context:
                raise ValueError("A12 ledger mixes formal contexts")
        if not ledger_replay_hashes or ledger_context != ledger.get("formal_context"):
            raise ValueError("A12 ledger replay/context bindings are incomplete")
        hashes = receipt.get("input_hashes")
        if not isinstance(hashes, dict) or set(hashes) != set(HASH_FIELDS) or any(not HEX64.fullmatch(str(hashes.get(name, ""))) for name in HASH_FIELDS):
            raise ValueError("input_hashes must contain exact source/model/config/dataset/dependency SHA-256 values")

        session_path, session = _json_hash_ref(root, receipt.get("sealed_final_session"), "sealed final session")
        if session_path != expected_session.resolve():
            raise ValueError("sealed final session path differs from postprocess context")
        if session.get("report_id") != "tzcup_formal_final_acceptance_session_v1" or session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE":
            raise ValueError("sealed final session is not the completed canonical session")
        if (
            session.get("failures") != {}
            or type(session.get("started_epoch_ns")) is not int or session["started_epoch_ns"] <= 0
            or type(session.get("finished_epoch_ns")) is not int or session["finished_epoch_ns"] <= session["started_epoch_ns"]
        ):
            raise ValueError("sealed final session has failures or invalid completion time")
        if not isinstance(session.get("evidence"), dict):
            raise ValueError("sealed final session lacks complete gate evidence")
        try:
            gate_results = {
                gate: {"sha256": row["sha256"], "status": row["status"]}
                for gate, row in session["evidence"].items()
                if isinstance(row, dict)
            }
            _verify_complete_session_evidence(
                SimpleNamespace(root=root), session, session["started_epoch_ns"], gate_results
            )
        except (KeyError, TypeError, OrchestrationError) as exc:
            raise ValueError(f"sealed final session evidence is not complete/current: {exc}") from exc
        snapshot = session.get("snapshot")
        if not isinstance(snapshot, dict) or any(not HEX64.fullmatch(str(snapshot.get(name, ""))) for name in ("snapshot_manifest_sha256", "source_inventory_sha256", "expanded_urdf_sha256")):
            raise ValueError("sealed final session has no complete snapshot identity")
        if hashes["source"] != snapshot["source_inventory_sha256"]:
            raise ValueError("source hash does not match the sealed snapshot")
        closure = session.get("runtime_closure_binding")
        if not isinstance(closure, dict) or closure.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED" or not HEX64.fullmatch(str(closure.get("closure_sha256", ""))):
            raise ValueError("sealed final session has no verified runtime closure")

        from formal_product_mcap_replay import PRODUCER_ID, SCHEMA as REPLAY_SCHEMA, artifact_sha256, mcap_sha256, mcap_semantic_sha256, sha256
        producer = {"id": PRODUCER_ID, "sha256": sha256(root / PRODUCER_ID)}
        replay_refs = receipt.get("product_replays")
        if not isinstance(replay_refs, list) or len(replay_refs) < 5:
            raise ValueError("A20 requires at least five canonical product replay receipts")
        bag_identities: set[tuple[str, str]] = set()
        semantic_identities: set[str] = set()
        selected_replay_hashes: set[str] = set()
        for index, reference in enumerate(replay_refs):
            replay_path, replay = _json_hash_ref(root, reference, f"product_replays[{index}]")
            if reference.get("sha256") not in ledger_replay_hashes or replay.get("formal_context") != ledger_context:
                raise ValueError(f"product_replays[{index}] is not a member of the verified A12 ledger")
            selected_replay_hashes.add(reference["sha256"])
            _artifact_in_root(expected, replay_path, f"product_replays[{index}] replay")
            if replay.get("schema") != REPLAY_SCHEMA or replay.get("status") != "FORMAL_PRODUCT_MCAP_REPLAY_PASS" or replay.get("pass") is not True:
                raise ValueError(f"product_replays[{index}] is not a passing canonical product replay")
            if replay.get("producer") != producer:
                raise ValueError(f"product_replays[{index}] producer identity is stale or forged")
            context = replay.get("formal_context", {})
            replay_session = context.get("session", {})
            if replay_session.get("path") != str(session_path) or replay_session.get("started_epoch_ns") != session.get("started_epoch_ns"):
                raise ValueError(f"product_replays[{index}] belongs to another formal session")
            if context.get("snapshot") != snapshot or context.get("runtime_closure_binding") != closure:
                raise ValueError(f"product_replays[{index}] snapshot/runtime closure binding differs")
            snapshot_manifest = context.get("snapshot_manifest")
            _hash_ref(root, snapshot_manifest, f"product_replays[{index}] snapshot manifest")
            if snapshot_manifest.get("sha256") != snapshot["snapshot_manifest_sha256"]:
                raise ValueError(f"product_replays[{index}] snapshot manifest identity differs")
            binding_path, binding = _json_hash_ref(root, context.get("runtime_gate_binding"), f"product_replays[{index}] runtime gate binding")
            del binding_path
            if (
                binding.get("status") != "FORMAL_RUNTIME_GATE_BOUND"
                or binding.get("runtime_closure_binding") != closure
                or binding.get("acceptance_session_binding", {}).get("snapshot") != snapshot
                or binding.get("acceptance_session_binding", {}).get("session_started_epoch_ns") != session.get("started_epoch_ns")
            ):
                raise ValueError(f"product_replays[{index}] runtime gate binding differs")
            replay_hashes = replay.get("input_hashes", {})
            if any(replay_hashes.get(name) != hashes[name] for name in ("model", "config", "dataset", "dependency")):
                raise ValueError(f"product_replays[{index}] provenance hashes differ")
            artifacts = replay.get("input_artifacts")
            if not isinstance(artifacts, dict) or set(artifacts) != {"model", "config", "dataset", "dependency"}:
                raise ValueError(f"product_replays[{index}] has no complete provenance artifact bindings")
            for name, reference in artifacts.items():
                artifact_path = _artifact_in_root(root, Path(str(reference.get("path", ""))), f"product_replays[{index}] {name}")
                if artifact_sha256(artifact_path) != reference.get("sha256") or reference.get("sha256") != hashes[name]:
                    raise ValueError(f"product_replays[{index}] {name} artifact hash mismatch")
            if replay_hashes.get("container") != receipt.get("container_sha256"):
                raise ValueError(f"product_replays[{index}] container digest differs")
            checks = replay.get("checks")
            if not isinstance(checks, dict) or set(checks) != REPLAY_CHECKS or not all(value is True for value in checks.values()):
                raise ValueError(f"product_replays[{index}] has incomplete recalculation/playback checks")
            playback = replay.get("playback")
            executable = playback.get("ros2_executable") if isinstance(playback, dict) else None
            cleanup = playback.get("process_group_cleanup") if isinstance(playback, dict) else None
            if (
                not isinstance(playback, dict) or playback.get("exit_code") != 0 or playback.get("timed_out") is not False
                or playback.get("ros_domain_id") not in range(215, 232)
                or playback.get("ros_localhost_only") is not True
                or playback.get("rmw_implementation") != "rmw_cyclonedds_cpp"
                or playback.get("automatic_discovery_range") != "LOCALHOST"
                or not isinstance(executable, dict) or not isinstance(executable.get("path"), str)
                or not Path(executable["path"]).is_absolute() or not HEX64.fullmatch(str(executable.get("sha256", "")))
                or closure.get("ros2_executable") != executable
                or not isinstance(cleanup, dict)
                or type(cleanup.get("process_group_id")) is not int or cleanup["process_group_id"] <= 1
                or cleanup.get("process_group_isolated") is not True or cleanup.get("cleanup_attempted") is not True
                or type(cleanup.get("sigterm_attempted")) is not bool or type(cleanup.get("sigkill_attempted")) is not bool
                or not isinstance(cleanup.get("signals_sent"), list) or any(item not in {"SIGTERM", "SIGKILL"} for item in cleanup["signals_sent"])
                or cleanup.get("sigterm_attempted") != ("SIGTERM" in cleanup["signals_sent"])
                or cleanup.get("sigkill_attempted") != ("SIGKILL" in cleanup["signals_sent"])
                or cleanup.get("surviving_group_processes") != 0 or cleanup.get("zero_survivor") is not True
            ):
                raise ValueError(f"product_replays[{index}] playback identity/checks are incomplete")
            bag = replay.get("bag", {})
            bag_identity = (str(bag.get("path", "")), str(bag.get("sha256", "")))
            if not bag_identity[0] or not HEX64.fullmatch(bag_identity[1]):
                raise ValueError(f"product_replays[{index}] has no MCAP binding")
            bag_path = _artifact_in_root(root, Path(bag_identity[0]), f"product_replays[{index}] MCAP")
            _artifact_in_root(expected, bag_path, f"product_replays[{index}] MCAP")
            if mcap_sha256(bag_path) != bag_identity[1]:
                raise ValueError(f"product_replays[{index}] MCAP hash mismatch")
            if not HEX64.fullmatch(str(bag.get("semantic_sha256", ""))) or mcap_semantic_sha256(bag_path) != bag.get("semantic_sha256"):
                raise ValueError(f"product_replays[{index}] MCAP semantic hash mismatch")
            raw = replay.get("raw_capture")
            if (
                not isinstance(raw, dict)
                or raw.get("mcap", {}).get("path") != bag_identity[0]
                or raw.get("mcap", {}).get("sha256") != bag_identity[1]
                or raw.get("mcap", {}).get("semantic_sha256") != bag.get("semantic_sha256")
                or not HEX64.fullmatch(str(raw.get("sha256", "")))
                or not isinstance(raw.get("path"), str)
                or not HEX64.fullmatch(str(raw.get("capture_id", "")))
                or not isinstance(raw.get("run_root"), str)
                or replay.get("run_root") != raw.get("run_root")
                or raw.get("run_root") != str(expected)
            ):
                raise ValueError(f"product_replays[{index}] lacks a verified raw capture chain")
            _raw_path, raw_file = _json_hash_ref(root, {"path": raw["path"], "sha256": raw["sha256"]}, f"product_replays[{index}] raw capture receipt")
            _artifact_in_root(expected, _raw_path, f"product_replays[{index}] raw capture receipt")
            if (
                raw_file.get("schema") != "tzcup.a12.raw_capture_receipt.v1"
                or raw_file.get("capture_id") != raw["capture_id"]
                or raw_file.get("run_root") != raw["run_root"]
                or raw_file.get("mcap") != {"path": bag_identity[0], "sha256": bag_identity[1]}
            ):
                raise ValueError(f"product_replays[{index}] raw capture receipt chain differs")
            verified_raw = validate_raw_capture_receipt(
                repository_root=root,
                raw_capture_receipt_path=_raw_path,
                formal_context=context,
                scenario_id=raw.get("scenario_id"),
                seed=raw.get("seed"),
                mission_id=raw.get("mission_id"),
                run_root=expected,
            )
            if verified_raw != raw:
                raise ValueError(f"product_replays[{index}] raw capture receipt did not revalidate")
            bag_identities.add(bag_identity)
            semantic_identities.add(str(bag["semantic_sha256"]))
        if len(bag_identities) != len(replay_refs):
            raise ValueError("A20 product replay receipts reuse an MCAP")
        if len(semantic_identities) != len(replay_refs):
            raise ValueError("A20 product replay receipts reuse a semantic MCAP stream")
        if len(selected_replay_hashes) != len(replay_refs):
            raise ValueError("A20 product replay receipts repeat an A12 ledger replay")

        release = receipt.get("release_artifact")
        if not isinstance(release, dict) or release.get("status") != "RELEASE_PACKAGE_ARTIFACT_RECORDED":
            raise ValueError("release artifact is not recorded")
        if not GIT40.fullmatch(str(release.get("main_commit", ""))) or not GIT40.fullmatch(str(release.get("rollback_commit", ""))):
            raise ValueError("release artifact lacks exact main/rollback commits")
        for name in ("archive", "sha256sums", "sbom", "dependency_lock", "licenses"):
            _hash_ref(root, release.get(name), f"release {name}")
        if release.get("container_sha256") != receipt.get("container_sha256") or not HEX64.fullmatch(str(release.get("container_sha256", ""))):
            raise ValueError("release container digest is missing or differs")

        rollback = receipt.get("verified_rollback_exercise")
        if not isinstance(rollback, dict) or rollback.get("status") != "ROLLBACK_EXERCISE_VERIFIED" or rollback.get("verified") is not True:
            raise ValueError("rollback exercise is not verified")
        if rollback.get("rollback_commit") != release.get("rollback_commit"):
            raise ValueError("rollback exercise does not bind the release rollback commit")
        _hash_ref(root, rollback.get("verification_report"), "rollback verification report")
    except (OSError, UnicodeError, json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
        errors.append(str(exc))
    return {
        "schema": "tzcup.a20_release_replay_receipt_validation.v1",
        "status": "A20_RECEIPT_VALID" if not errors else "A20_RECEIPT_STATIC_BLOCKED",
        "valid": not errors,
        "errors": errors,
        "release_runtime_pass": not errors,
        "claim_boundary": (
            "Validation proves retained current-session replay/recalculation, release, "
            "and rollback bindings; it does not itself deploy or rerun Gazebo."
        ),
    }


def _regular_in_root(root: Path, candidate: Path, label: str) -> Path:
    if not root.is_absolute() or root.is_symlink() or not root.is_dir():
        raise ValueError("repository root must be an absolute non-symlink directory")
    if not candidate.is_absolute():
        raise ValueError(f"{label} must be absolute")
    try:
        relative = candidate.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"{label} escapes repository root") from exc
    if ".." in relative.parts:
        raise ValueError(f"{label} escapes repository root")
    current = root
    for part in relative.parts:
        current /= part
        if current.exists() and current.is_symlink():
            raise ValueError(f"{label} has a symbolic-link ancestor")
    if candidate.is_symlink():
        raise ValueError(f"{label} is a symbolic-link")
    if not candidate.is_file():
        raise ValueError(f"{label} must be a regular non-symlink file")
    return candidate


def _output_in_root(root: Path, output: Path) -> Path:
    if not output.is_absolute():
        raise ValueError("output must be absolute")
    try:
        relative = output.relative_to(root)
    except ValueError as exc:
        raise ValueError("output escapes repository root") from exc
    if ".." in relative.parts:
        raise ValueError("output escapes repository root")
    parent = output.parent
    if not parent.is_dir() or parent.is_symlink():
        raise ValueError("output parent must be an existing non-symlink directory")
    probe = parent
    while probe != root:
        if probe.is_symlink():
            raise ValueError("output has a symbolic-link ancestor")
        probe = probe.parent
    if output.exists() or output.is_symlink():
        raise ValueError("output must be fresh")
    return output


def _identity(value: os.stat_result) -> tuple[int, int, int, int, int]:
    return (
        value.st_dev,
        value.st_ino,
        value.st_size,
        value.st_mtime_ns,
        # Windows fstat/lstat can expose transient creation-time rounding.
        # The production CLI is POSIX-only, where ctime remains part of the
        # mutation identity; pure Windows unit tests use the stable fields.
        value.st_ctime_ns if os.name != "nt" else 0,
    )


def _require_posix_descriptor_api() -> None:
    """Refuse the CLI where CPython cannot enforce its descriptor contract."""

    if os.name == "nt":
        raise ValueError(
            "A20 receipt CLI is POSIX/WSL-only: this CPython lacks secure dir_fd traversal"
        )
    for flag in ("O_NOFOLLOW", "O_DIRECTORY"):
        if not hasattr(os, flag):
            raise ValueError(f"A20 receipt CLI requires {flag}; refusing unsafe path traversal")
    required_dir_fd = {
        "os.open": os.open,
        "os.stat": os.stat,
        "os.link": os.link,
        "os.unlink": os.unlink,
    }
    for name, operation in required_dir_fd.items():
        if operation not in os.supports_dir_fd:
            raise ValueError(f"A20 receipt CLI requires {name} dir_fd support")
    for name, operation in {"os.stat": os.stat, "os.link": os.link}.items():
        if operation not in os.supports_follow_symlinks:
            raise ValueError(f"A20 receipt CLI requires {name} no-follow support")


def _open_root_directory(root: Path) -> int:
    flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
    if not root.is_absolute():
        raise ValueError("repository root must be absolute")
    descriptor = os.open(root.anchor, flags)
    try:
        for part in root.parts[1:]:
            child = os.open(part, flags, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
        if not stat.S_ISDIR(os.fstat(descriptor).st_mode):
            raise ValueError("repository root is not a directory")
        return descriptor
    except Exception:
        os.close(descriptor)
        raise


def _assert_root_binding(root: Path, descriptor: int) -> None:
    """Detect a renamed/replaced root while all operations stay descriptor-bound."""

    current = os.stat(root, follow_symlinks=False)
    bound = os.fstat(descriptor)
    if current.st_mode != bound.st_mode or _identity(current)[:2] != _identity(bound)[:2]:
        raise ValueError("repository root changed after secure open")


def _relative_in_root(root: Path, candidate: Path, label: str) -> Path:
    if not candidate.is_absolute():
        raise ValueError(f"{label} must be absolute")
    try:
        relative = candidate.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"{label} escapes repository root") from exc
    if ".." in relative.parts:
        raise ValueError(f"{label} escapes repository root")
    return relative


def _open_directory(root_descriptor: int, parts: tuple[str, ...]) -> int:
    flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
    descriptor = os.dup(root_descriptor)
    try:
        for part in parts:
            child = os.open(part, flags, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
            if not stat.S_ISDIR(os.fstat(descriptor).st_mode):
                raise ValueError("path parent is not a directory")
        return descriptor
    except Exception:
        os.close(descriptor)
        raise


def _open_bound_input(root: Path, root_descriptor: int, path: Path) -> tuple[int, tuple[int, int, int, int, int]]:
    relative = path.relative_to(root)
    directory = _open_directory(root_descriptor, relative.parts[:-1])
    try:
        flags = os.O_RDONLY | getattr(os, "O_BINARY", 0) | os.O_NOFOLLOW
        descriptor = os.open(relative.name, flags, dir_fd=directory)
    finally:
        os.close(directory)
    try:
        actual = os.fstat(descriptor)
        if not stat.S_ISREG(actual.st_mode):
            raise ValueError("receipt must be a regular file")
        return descriptor, _identity(actual)
    except Exception:
        os.close(descriptor)
        raise


def _read_bound_json(descriptor: int, expected: tuple[int, int, int, int, int]) -> tuple[dict[str, Any], str]:
    digest = hashlib.sha256()
    chunks: list[bytes] = []
    with os.fdopen(descriptor, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
            chunks.append(chunk)
        if _identity(os.fstat(stream.fileno())) != expected:
            raise ValueError("receipt changed while being read")
    raw = b"".join(chunks)
    try:
        payload = json.loads(raw.decode("utf-8"))
    except UnicodeDecodeError as exc:
        raise ValueError("receipt is not UTF-8 JSON") from exc
    if not isinstance(payload, dict):
        raise ValueError("receipt root is not an object")
    return payload, digest.hexdigest()


def _write_fresh_output(root: Path, root_descriptor: int, path: Path, payload: dict[str, Any], *, token: str | None = None) -> None:
    """Atomically publish without following a pending link or replacing output."""

    relative = path.relative_to(root)
    directory = _open_directory(root_descriptor, relative.parts[:-1])
    temporary = f".{path.name}.pending.{token or secrets.token_hex(16)}"
    owned_temporary = False
    owned_identity: tuple[int, int] | None = None
    try:
        try:
            os.stat(path.name, dir_fd=directory, follow_symlinks=False)
        except FileNotFoundError:
            pass
        else:
            raise ValueError("output appeared after validation")
        descriptor = os.open(
            temporary,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_BINARY", 0),
            0o600,
            dir_fd=directory,
        )
        owned_temporary = True
        try:
            data = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
            written = 0
            while written < len(data):
                written += os.write(descriptor, data[written:])
            os.fsync(descriptor)
            expected_file = _identity(os.fstat(descriptor))[:2]
            owned_identity = expected_file
        finally:
            os.close(descriptor)
        try:
            os.link(
                temporary,
                path.name,
                src_dir_fd=directory,
                dst_dir_fd=directory,
                follow_symlinks=False,
            )
        except FileExistsError as exc:
            raise ValueError("output appeared during commit") from exc
        committed_descriptor, committed_identity = _open_bound_input(
            root, root_descriptor, path
        )
        try:
            if committed_identity[:2] != expected_file:
                raise ValueError("output commit identity mismatch")
        finally:
            os.close(committed_descriptor)
    finally:
        if owned_temporary:
            try:
                current = os.stat(temporary, dir_fd=directory, follow_symlinks=False)
                if stat.S_ISREG(current.st_mode) and _identity(current)[:2] == owned_identity:
                    os.unlink(temporary, dir_fd=directory)
            except FileNotFoundError:
                pass
        os.close(directory)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--session", type=Path, required=True)
    args = parser.parse_args()
    try:
        _require_posix_descriptor_api()
        root = args.repository_root
        root_descriptor = _open_root_directory(root)
        try:
            receipt_path = root / _relative_in_root(root, args.receipt, "receipt")
            output = root / _relative_in_root(root, args.output, "output")
            _assert_root_binding(root, root_descriptor)
            descriptor, identity = _open_bound_input(root, root_descriptor, receipt_path)
            receipt, receipt_sha256 = _read_bound_json(descriptor, identity)
            _assert_root_binding(root, root_descriptor)
            report = validate_receipt(receipt, root, args.run_root, args.session)
            report["receipt_sha256"] = receipt_sha256
            _write_fresh_output(root, root_descriptor, output, report)
            _assert_root_binding(root, root_descriptor)
        finally:
            os.close(root_descriptor)
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        print(json.dumps({"status": "A20_RECEIPT_STATIC_BLOCKED", "error": str(exc)}, indent=2))
        return 2
    return 0 if report["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
