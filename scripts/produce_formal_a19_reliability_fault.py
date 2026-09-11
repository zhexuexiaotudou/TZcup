#!/usr/bin/env python3
"""Supervise the canonical A19 adapter and retain producer-owned raw evidence.

The adapter is a frozen product-runtime program implementing the line-delimited
``tzcup.formal_a19.adapter.v1`` protocol.  The producer owns wall/monotonic
timestamps, fault commands, process-group cleanup and the final raw receipt;
adapter summaries alone are never acceptance evidence.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import secrets
import selectors
import shutil
import signal
import stat
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

from formal_runtime_gate_binding import build_binding


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config/high_fidelity_vehicle/formal_a19_reliability_fault_contract.json"
RAW_RECEIPT = "tzcup_a19_canonical_runtime_raw_receipt_v1"
PROTOCOL = "tzcup.formal_a19.adapter.v1"

FAULT_READBACK_CONTRACT = {
    "rgb_freeze": ("rgb_proxy_drop_observed", "sensor_drop", "sensor_forwarding"),
    "depth_freeze": ("depth_proxy_drop_observed", "sensor_drop", "sensor_forwarding"),
    "timestamp_skew": ("rgb_timestamp_shift_observed", "sensor_mutation", "sensor_forwarding"),
    "camera_info_mismatch": ("camera_info_width_change_observed", "sensor_mutation", "sensor_forwarding"),
    "tf_unavailable": ("unavailable_frame_rewrite_observed", "sensor_mutation", "sensor_forwarding"),
    "invalid_depth": ("depth_payload_invalidation_observed", "sensor_mutation", "sensor_forwarding"),
    "proposal_flood": ("proposal_output_expansion_observed", "product_consumer", "product_consumer_cleared"),
    "proposal_dropout": ("proposal_output_drop_observed", "product_consumer", "product_consumer_cleared"),
    "classifier_exception": ("classifier_exception_observed", "product_consumer", "product_consumer_cleared"),
    "classifier_timeout": ("classifier_timeout_observed", "product_consumer", "product_consumer_cleared"),
    "action_verifier_failure": ("verified_result_rejection_observed", "product_consumer", "product_consumer_cleared"),
    "reobserve_timeout": ("wrist_reobservation_drop_observed", "product_consumer", "product_consumer_cleared"),
    "cuda_provider_failure": ("selected_cuda_inference_failure_observed", "model_provider", "model_provider_cleared"),
    "model_hash_mismatch": ("dosod_hash_mismatch_observed", "model_provider", "model_provider_cleared"),
    "corrupt_model": ("edgesam_shadow_loader_rejection_observed", "model_provider", "model_provider_cleared"),
    "sustained_slow_inference": ("inference_delay_observed", "model_provider", "model_provider_cleared"),
    "nav2_path_unavailable": ("planner_inactive_and_path_server_absent", "nav2_lifecycle", "nav2_lifecycle_recovered"),
    "dynamic_obstacle_blocks_observation": ("native_obstacle_pose_and_scan_block_observed", "dynamic_obstacle", "dynamic_obstacle_restored"),
}


class A19ProducerError(RuntimeError):
    """The formal run cannot produce trustworthy raw evidence."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise A19ProducerError(f"cannot read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise A19ProducerError(f"{label} root must be an object")
    return value


def validate_contract(contract: Mapping[str, Any]) -> None:
    if contract.get("schema_version") != 3:
        raise A19ProducerError("A19 contract schema_version must be 3")
    producer = contract.get("canonical_producer")
    if not isinstance(producer, Mapping) or producer.get("available_on_current_source") is not True:
        raise A19ProducerError("canonical A19 producer is not enabled by the contract")
    if producer.get("adapter_protocol") != PROTOCOL or producer.get("raw_receipt_id") != RAW_RECEIPT:
        raise A19ProducerError("A19 producer protocol/receipt identity drifted")
    stability = contract.get("stability_gates")
    if not isinstance(stability, Mapping) or stability.get("minimum_duration_s") != 7200:
        raise A19ProducerError("formal A19 duration must remain exactly 7200 seconds minimum")
    profiles = contract.get("profile_schedule")
    faults = contract.get("fault_schedule")
    if not isinstance(profiles, list) or not isinstance(faults, list):
        raise A19ProducerError("A19 profile/fault schedules must be lists")
    profile_ids = [row.get("profile") for row in profiles if isinstance(row, Mapping)]
    expected_profiles = ["nominal", "transport_stress", "wet_surface", "degraded_drive"]
    if profile_ids != expected_profiles:
        raise A19ProducerError("A19 profile schedule is incomplete or reordered")
    fault_ids = [row.get("fault") for row in faults if isinstance(row, Mapping)]
    if len(faults) != 18 or len(fault_ids) != 18 or len(set(fault_ids)) != 18:
        raise A19ProducerError("A19 must schedule exactly 18 unique faults")
    expectations = contract.get("fault_expectations")
    if not isinstance(expectations, Mapping) or set(expectations) != set(fault_ids):
        raise A19ProducerError("A19 must declare one exact fault expectation per scheduled fault")
    for fault, expectation in expectations.items():
        if not isinstance(expectation, Mapping) or expectation.get("state") not in {"STOPPED", "DEGRADED"}:
            raise A19ProducerError(f"A19 fault expectation is malformed: {fault}")
        for field in ("requires_global_safety_stop", "requires_perception_degraded", "requires_cleaning_inhibit"):
            if type(expectation.get(field)) is not bool:
                raise A19ProducerError(f"A19 fault expectation {fault}.{field} must be boolean")
        if expectation["state"] == "STOPPED" and expectation["requires_global_safety_stop"] is not True:
            raise A19ProducerError(f"A19 STOPPED expectation lacks a real safety-stop requirement: {fault}")
    readbacks = contract.get("fault_readback_contract")
    expected_readbacks = {
        fault: {
            "expected_outcome": row[0],
            "injection_readback_kind": row[1],
            "recovery_readback_kind": row[2],
        }
        for fault, row in FAULT_READBACK_CONTRACT.items()
    }
    if readbacks != expected_readbacks:
        raise A19ProducerError("A19 fault readback contract is incomplete or drifted")
    profile_expectations = contract.get("profile_expectations")
    if not isinstance(profile_expectations, Mapping) or set(profile_expectations) != set(expected_profiles):
        raise A19ProducerError("A19 frozen profile expectations are incomplete")
    for profile, values in profile_expectations.items():
        if not isinstance(values, Mapping) or set(values) != {
            "sensor_latency_ms", "sensor_dropout_probability", "wheel_slip_ratio", "actuator_gain"
        } or any(type(value) not in (int, float) or not math.isfinite(float(value)) for value in values.values()):
            raise A19ProducerError(f"A19 frozen profile expectation is malformed: {profile}")
    timing = contract.get("fault_timing")
    if not isinstance(timing, Mapping) or timing.get("required_fault_count") != 18:
        raise A19ProducerError("A19 fault timing contract must require 18 faults")
    duration = float(stability["minimum_duration_s"])
    previous = -1.0
    for row in faults:
        if not isinstance(row, Mapping):
            raise A19ProducerError("A19 fault schedule row must be an object")
        offset = row.get("offset_s")
        parameters = row.get("parameters")
        if type(offset) not in (int, float) or not (0 < float(offset) < duration):
            raise A19ProducerError(f"invalid fault offset: {row}")
        if float(offset) <= previous:
            raise A19ProducerError("A19 fault offsets must be strictly increasing")
        if row.get("profile") not in expected_profiles or not isinstance(parameters, Mapping) or not parameters:
            raise A19ProducerError(f"fault has no profile or parameters: {row.get('fault')}")
        previous = float(offset)


def _git_identity(repository_root: Path) -> dict[str, Any]:
    def run(*args: str) -> str:
        completed = subprocess.run(
            ["git", "-C", str(repository_root), *args],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return completed.stdout.strip()

    try:
        commit = run("rev-parse", "HEAD")
        tree = run("rev-parse", "HEAD^{tree}")
        dirty = run("status", "--porcelain", "--untracked-files=no")
    except (OSError, subprocess.CalledProcessError) as exc:
        raise A19ProducerError(f"cannot establish current git identity: {exc}") from exc
    if dirty:
        raise A19ProducerError("formal A19 execution requires a clean tracked worktree")
    return {"repository_commit": commit, "repository_tree": tree, "tracked_worktree_clean": True}


def _parse_adapter_argv(raw: str, contract: Mapping[str, Any], *, allow_fixture: bool = False) -> list[str]:
    try:
        value = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise A19ProducerError(f"adapter argv is not valid JSON: {exc}") from exc
    if not isinstance(value, list) or not value or any(not isinstance(item, str) or not item for item in value):
        raise A19ProducerError("adapter argv must be a non-empty JSON string array")
    joined = "\0".join(value).lower()
    producer = contract["canonical_producer"]
    if not allow_fixture and any(str(fragment).lower() in joined for fragment in producer["forbidden_adapter_path_fragments"]):
        raise A19ProducerError("formal A19 execution forbids the test fixture adapter")
    executable = Path(value[0])
    resolved = executable if executable.is_absolute() else Path(shutil.which(value[0]) or "")
    if not str(resolved) or not resolved.is_file():
        raise A19ProducerError(f"adapter executable is unavailable: {value[0]}")
    return value


def _adapter_file_identities(
    argv: Sequence[str], repository_root: Path, install_root: Path
) -> list[dict[str, Any]]:
    repository_root = repository_root.resolve(strict=True)
    install_root = install_root.resolve(strict=True)
    resolved_executable = Path(argv[0]) if Path(argv[0]).is_absolute() else Path(shutil.which(argv[0]) or "")
    candidates: list[Path] = [resolved_executable]
    for token in argv[1:]:
        candidate = Path(token)
        if not candidate.is_absolute():
            candidate = repository_root / candidate
        if candidate.is_file():
            candidates.append(candidate)
    identities: list[dict[str, Any]] = []
    frozen_code_found = False
    for candidate in dict.fromkeys(candidates):
        if not candidate.is_file() or candidate.is_symlink():
            raise A19ProducerError(f"adapter file is missing or linked: {candidate}")
        resolved = candidate.resolve(strict=True)
        in_frozen_code = False
        for trusted_root in (repository_root, install_root):
            try:
                resolved.relative_to(trusted_root)
                in_frozen_code = True
            except ValueError:
                continue
        frozen_code_found = frozen_code_found or in_frozen_code
        identities.append({"path": str(resolved), "sha256": _sha256(resolved), "size_bytes": resolved.stat().st_size})
    if not frozen_code_found:
        raise A19ProducerError("adapter argv has no code file in the current repository or frozen install")
    return identities


def _inside_fresh_artifacts_root(repository_root: Path, evidence_root: Path) -> Path:
    repository_root = repository_root.resolve(strict=True)
    artifacts = (repository_root / "artifacts").resolve(strict=True)
    if evidence_root.exists() or evidence_root.is_symlink():
        raise A19ProducerError(f"refusing stale A19 evidence root: {evidence_root}")
    normalized = Path(os.path.abspath(os.path.normpath(str(evidence_root))))
    try:
        normalized.relative_to(artifacts)
    except ValueError as exc:
        raise A19ProducerError("A19 evidence root must be below repository artifacts/") from exc
    current = normalized.parent
    while current != repository_root.parent:
        if current.exists() and stat.S_ISLNK(current.lstat().st_mode):
            raise A19ProducerError(f"A19 evidence path has symlink ancestor: {current}")
        if current == repository_root:
            break
        current = current.parent
    if not normalized.parent.is_dir():
        raise A19ProducerError("A19 evidence parent must already exist")
    return normalized


def _reference(path: Path, root: Path, *, line_count: int | None = None) -> dict[str, Any]:
    result: dict[str, Any] = {
        "path": path.relative_to(root).as_posix(),
        "sha256": _sha256(path),
        "size_bytes": path.stat().st_size,
    }
    if line_count is not None:
        result["line_count"] = line_count
    return result


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    pending = path.with_name(f".{path.name}.pending.{os.getpid()}")
    data = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
    with pending.open("xb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    pending.replace(path)


def _send(process: subprocess.Popen[str], command: dict[str, Any], issued: list[dict[str, Any]]) -> None:
    if process.stdin is None:
        raise A19ProducerError("adapter stdin is unavailable")
    sent = {
        "sent_epoch_ns": time.time_ns(),
        "sent_monotonic_ns": time.monotonic_ns(),
        "command": command,
    }
    process.stdin.write(json.dumps(command, separators=(",", ":")) + "\n")
    process.stdin.flush()
    issued.append(sent)


def _survivors_in_group(process_group_id: int) -> list[int]:
    survivors: list[int] = []
    proc = Path("/proc")
    if os.name != "posix" or not proc.is_dir():
        return survivors
    for item in proc.iterdir():
        if not item.name.isdigit():
            continue
        try:
            fields = (item / "stat").read_text(encoding="utf-8").split()
            if len(fields) > 4 and int(fields[4]) == process_group_id:
                survivors.append(int(item.name))
        except (OSError, UnicodeError, ValueError):
            continue
    return sorted(survivors)


def _terminate_group(process: subprocess.Popen[str], signals_sent: list[str]) -> None:
    if process.poll() is not None:
        return
    for sig, name, grace in ((signal.SIGINT, "SIGINT", 10.0), (signal.SIGTERM, "SIGTERM", 10.0), (signal.SIGKILL, "SIGKILL", 5.0)):
        try:
            os.killpg(process.pid, sig)
            signals_sent.append(name)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + grace
        while process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if process.poll() is not None:
            return


def preflight(
    *, repository_root: Path, contract_path: Path, install_root: Path,
    closure_manifest: Path, session_path: Path, snapshot_path: Path,
    evidence_root: Path, adapter_argv_json: str, allow_fixture: bool = False,
) -> dict[str, Any]:
    repository_root = repository_root.resolve(strict=True)
    contract = _read_object(contract_path, "A19 contract")
    validate_contract(contract)
    evidence_root = _inside_fresh_artifacts_root(repository_root, evidence_root)
    argv = _parse_adapter_argv(adapter_argv_json, contract, allow_fixture=allow_fixture)
    adapter_files = _adapter_file_identities(argv, repository_root, install_root)
    binding = build_binding(
        repository_root=repository_root,
        install_root=install_root,
        closure_manifest=closure_manifest,
        session_path=session_path,
        snapshot_path=snapshot_path,
    )
    source = _git_identity(repository_root)
    return {
        "report_id": "tzcup_formal_a19_producer_preflight_v1",
        "status": "FORMAL_A19_PRODUCER_PREFLIGHT_PASSED",
        "passed": True,
        "repository_root": str(repository_root),
        "evidence_root": str(evidence_root),
        "contract_sha256": _sha256(contract_path),
        "source_binding": source,
        "runtime_gate_binding": binding,
        "adapter_argv": argv,
        "adapter_file_identities": adapter_files,
        "adapter_runtime_handshake_pending": True,
        "gazebo_started": False,
        "minimum_remaining_runtime_s": contract["stability_gates"]["minimum_duration_s"],
    }


def execute_capture(
    *, repository_root: Path, contract_path: Path, install_root: Path,
    closure_manifest: Path, session_path: Path, snapshot_path: Path,
    evidence_root: Path, adapter_argv_json: str, allow_fixture: bool = False,
) -> tuple[dict[str, Any], int]:
    if os.name != "posix":
        raise A19ProducerError("formal A19 execution is POSIX/WSL-only")
    gate = preflight(
        repository_root=repository_root, contract_path=contract_path,
        install_root=install_root, closure_manifest=closure_manifest,
        session_path=session_path, snapshot_path=snapshot_path,
        evidence_root=evidence_root, adapter_argv_json=adapter_argv_json,
        allow_fixture=allow_fixture,
    )
    contract = _read_object(contract_path, "A19 contract")
    evidence_root = Path(gate["evidence_root"])
    evidence_root.mkdir(mode=0o700)
    binding_path = evidence_root / "runtime_gate_binding.json"
    _write_json(binding_path, gate["runtime_gate_binding"])
    events_path = evidence_root / "adapter_events.jsonl"
    stderr_path = evidence_root / "adapter_stderr.log"
    nonce = secrets.token_hex(32)
    issued: list[dict[str, Any]] = []
    producer_errors: list[str] = []
    signals_sent: list[str] = []
    event_count = 0
    started_epoch_ns = time.time_ns()
    started_monotonic_ns = time.monotonic_ns()
    environment = os.environ.copy()
    environment.update(
        TZCUP_FORMAL_A19_PROTOCOL=PROTOCOL,
        TZCUP_FORMAL_A19_NONCE=nonce,
        TZCUP_FORMAL_A19_CONTRACT=str(contract_path.resolve()),
    )
    with stderr_path.open("xb") as stderr_stream, events_path.open("xb") as event_stream:
        process = subprocess.Popen(
            gate["adapter_argv"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=stderr_stream, text=True, encoding="utf-8", bufsize=1,
            start_new_session=True, env=environment, cwd=repository_root,
        )
        process_group_id = process.pid
        selector = selectors.DefaultSelector()
        assert process.stdout is not None
        selector.register(process.stdout, selectors.EVENT_READ)
        start_command = {
            "protocol": PROTOCOL, "type": "start", "command_id": secrets.token_hex(16),
            "nonce": nonce, "contract_sha256": gate["contract_sha256"],
        }
        _send(process, start_command, issued)
        schedule: list[tuple[float, dict[str, Any]]] = []
        for row in contract["profile_schedule"]:
            schedule.append((float(row["start_offset_s"]), {
                "protocol": PROTOCOL, "type": "set_profile", "command_id": secrets.token_hex(16),
                "nonce": nonce, "profile": row["profile"],
            }))
        for row in contract["fault_schedule"]:
            schedule.append((float(row["offset_s"]), {
                "protocol": PROTOCOL, "type": "inject_fault", "command_id": secrets.token_hex(16),
                "nonce": nonce, "fault": row["fault"], "profile": row["profile"],
                "parameters": row["parameters"],
            }))
        schedule.sort(key=lambda item: (item[0], 0 if item[1]["type"] == "set_profile" else 1))
        schedule_index = 0
        duration = float(contract["stability_gates"]["minimum_duration_s"])
        shutdown_sent = False
        try:
            while True:
                now_ns = time.monotonic_ns()
                elapsed = (now_ns - started_monotonic_ns) / 1e9
                while schedule_index < len(schedule) and schedule[schedule_index][0] <= elapsed:
                    _send(process, schedule[schedule_index][1], issued)
                    schedule_index += 1
                if elapsed >= duration and not shutdown_sent:
                    _send(process, {
                        "protocol": PROTOCOL, "type": "shutdown", "command_id": secrets.token_hex(16),
                        "nonce": nonce, "reason": "formal_duration_complete",
                    }, issued)
                    shutdown_sent = True
                ready = selector.select(timeout=0.1)
                for key, _ in ready:
                    line = key.fileobj.readline()
                    if not line:
                        continue
                    received = {
                        "producer_sequence": event_count,
                        "received_epoch_ns": time.time_ns(),
                        "received_monotonic_ns": time.monotonic_ns(),
                    }
                    try:
                        adapter = json.loads(line)
                        if not isinstance(adapter, dict):
                            raise ValueError("adapter event root is not an object")
                        received["adapter"] = adapter
                    except (json.JSONDecodeError, ValueError) as exc:
                        received["parse_error"] = str(exc)
                        received["raw_line"] = line.rstrip("\n")[:4096]
                        producer_errors.append(f"malformed adapter event at producer sequence {event_count}")
                    event_stream.write((json.dumps(received, sort_keys=True) + "\n").encode("utf-8"))
                    event_stream.flush()
                    event_count += 1
                    adapter_value = received.get("adapter")
                    if isinstance(adapter_value, dict):
                        metrics = adapter_value.get("metrics")
                        if isinstance(metrics, dict) and metrics.get("unsafe_cleaning_action_count") not in (None, 0):
                            producer_errors.append("unsafe cleaning action observed; aborted immediately")
                            shutdown_sent = True
                            _terminate_group(process, signals_sent)
                if process.poll() is not None:
                    break
                if shutdown_sent and elapsed >= duration + 30.0:
                    producer_errors.append("adapter did not exit within 30 seconds of formal shutdown")
                    _terminate_group(process, signals_sent)
                    break
        finally:
            selector.close()
            if process.poll() is None:
                _terminate_group(process, signals_sent)
            process.wait()
            event_stream.flush()
            os.fsync(event_stream.fileno())
            stderr_stream.flush()
            os.fsync(stderr_stream.fileno())
    ended_epoch_ns = time.time_ns()
    ended_monotonic_ns = time.monotonic_ns()
    survivors = _survivors_in_group(process_group_id)
    if survivors:
        producer_errors.append(f"surviving process-group members: {survivors}")
    if process.returncode != 0:
        producer_errors.append(f"adapter exit code is {process.returncode}")
    if schedule_index != len(schedule):
        producer_errors.append("adapter exited before every scheduled profile/fault command was issued")
    source_after = _git_identity(repository_root.resolve())
    if source_after != gate["source_binding"]:
        producer_errors.append("repository identity drifted during A19 capture")
    runtime_binding_after = build_binding(
        repository_root=repository_root.resolve(), install_root=install_root,
        closure_manifest=closure_manifest, session_path=session_path,
        snapshot_path=snapshot_path,
    )
    if runtime_binding_after["acceptance_session_binding"] != gate["runtime_gate_binding"]["acceptance_session_binding"] or runtime_binding_after["runtime_closure_binding"] != gate["runtime_gate_binding"]["runtime_closure_binding"]:
        producer_errors.append("session/snapshot/runtime closure drifted during A19 capture")
    receipt = {
        "schema_version": 1,
        "receipt_id": RAW_RECEIPT,
        "status": "FORMAL_A19_RAW_CAPTURE_COMPLETE" if not producer_errors else "FORMAL_A19_RAW_CAPTURE_FAILED",
        "capture_complete": not producer_errors,
        "producer_errors": producer_errors,
        "contract": {"path": str(contract_path.resolve()), "sha256": gate["contract_sha256"], "contract_id": contract["contract_id"]},
        "producer": {"path": str(Path(__file__).resolve()), "sha256": _sha256(Path(__file__).resolve()), "protocol": PROTOCOL, "nonce": nonce},
        "source_binding": gate["source_binding"],
        "runtime_gate_binding": gate["runtime_gate_binding"],
        "execution": {
            "argv": gate["adapter_argv"], "shell": False,
            "argv_file_identities": gate["adapter_file_identities"],
            "started_epoch_ns": started_epoch_ns, "ended_epoch_ns": ended_epoch_ns,
            "started_monotonic_ns": started_monotonic_ns, "ended_monotonic_ns": ended_monotonic_ns,
            "monotonic_duration_s": (ended_monotonic_ns - started_monotonic_ns) / 1e9,
            "process_id": process.pid, "process_group_id": process_group_id,
            "exit_code": process.returncode, "signals_sent": signals_sent,
            "survivor_pids": survivors, "zero_survivors": not survivors,
        },
        "issued_commands": issued,
        "raw_evidence": {
            "adapter_events_jsonl": _reference(events_path, evidence_root, line_count=event_count),
            "adapter_stderr_log": _reference(stderr_path, evidence_root),
            "runtime_gate_binding": _reference(binding_path, evidence_root),
        },
    }
    receipt_path = evidence_root / "raw_receipt.json"
    _write_json(receipt_path, receipt)
    persisted = _read_object(receipt_path, "persisted raw receipt")
    if persisted != receipt:
        raise A19ProducerError("persisted A19 raw receipt changed before reread")
    return receipt, 0 if not producer_errors else 4


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--preflight", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--runtime-install", type=Path, required=True)
    parser.add_argument("--runtime-closure", type=Path, required=True)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--adapter-argv-json", required=True)
    args = parser.parse_args()
    try:
        kwargs = dict(
            repository_root=args.repository_root, contract_path=args.contract,
            install_root=args.runtime_install, closure_manifest=args.runtime_closure,
            session_path=args.session, snapshot_path=args.snapshot,
            evidence_root=args.evidence_root, adapter_argv_json=args.adapter_argv_json,
        )
        if args.preflight:
            print(json.dumps(preflight(**kwargs), indent=2, sort_keys=True))
            return 0
        receipt, status = execute_capture(**kwargs)
        print(json.dumps(receipt, indent=2, sort_keys=True))
        return status
    except (A19ProducerError, OSError, ValueError) as exc:
        print(json.dumps({"status": "FORMAL_A19_PRODUCER_REFUSED", "error": str(exc)}, indent=2))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
