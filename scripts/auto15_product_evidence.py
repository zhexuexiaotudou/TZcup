#!/usr/bin/env python3
"""Seal canonical AUTO-15 execution, mission-group, and 18x10 ledger receipts."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from pathlib import Path
from typing import Any

from formal_product_mcap_replay import (
    PRODUCER_ID as REPLAY_PRODUCER_ID,
    SCHEMA as REPLAY_SCHEMA,
    ProductReplayError,
    artifact_sha256,
    _git_identity,
    read_object,
    sha256,
    write_fresh_json,
)


ROOT = Path(__file__).resolve().parents[1]
PRODUCER_ID = "scripts/auto15_product_evidence.py"
EXECUTION_SCHEMA = "tzcup.auto15.product_execution_receipt.v1"
GROUP_SCHEMA = "tzcup.auto15.mission_group_receipt.v1"
LEDGER_SCHEMA = "tzcup.auto15.execution_ledger.v1"


class Auto15EvidenceError(RuntimeError):
    """Supplied material is not canonical AUTO-15 product evidence."""


def _producer(repository_root: Path) -> dict[str, str]:
    return {"id": PRODUCER_ID, "sha256": sha256(repository_root / PRODUCER_ID)}


def _inside(root: Path, path: Path, label: str) -> Path:
    root = root.resolve()
    candidate = path.absolute()
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise Auto15EvidenceError(f"{label} escapes the current run root") from exc
    current = candidate
    while current != root:
        if current.is_symlink():
            raise Auto15EvidenceError(f"{label} has a symbolic-link component")
        current = current.parent
    if root.is_symlink():
        raise Auto15EvidenceError("run root must not be a symbolic link")
    if candidate.resolve() != candidate:
        raise Auto15EvidenceError(f"{label} is not a canonical non-link path")
    return candidate


def _video_audit(video: Path) -> dict[str, Any]:
    if not video.is_file() or video.is_symlink() or video.stat().st_size < 100_000:
        raise Auto15EvidenceError("video must be a non-link regular MP4 of at least 100000 bytes")
    command = [
        "ffprobe", "-v", "error", "-select_streams", "v:0",
        "-show_entries", "stream=codec_name,width,height,avg_frame_rate,nb_frames:format=duration",
        "-of", "json", str(video),
    ]
    try:
        completed = subprocess.run(command, capture_output=True, text=True, check=False, timeout=120)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise Auto15EvidenceError(f"ffprobe failed: {exc}") from exc
    if completed.returncode:
        raise Auto15EvidenceError(f"ffprobe rejected product video: {completed.stderr.strip()}")
    try:
        probe = json.loads(completed.stdout)
        stream = probe["streams"][0]
        duration = float(probe["format"]["duration"])
    except (KeyError, IndexError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise Auto15EvidenceError("ffprobe returned incomplete video metadata") from exc
    if duration <= 0 or int(stream.get("width", 0)) <= 0 or int(stream.get("height", 0)) <= 0:
        raise Auto15EvidenceError("video has no positive duration or dimensions")
    return {
        "tool": "ffprobe",
        "command": command,
        "exit_code": completed.returncode,
        "duration_seconds": duration,
        "codec_name": stream.get("codec_name"),
        "width": int(stream["width"]),
        "height": int(stream["height"]),
        "avg_frame_rate": stream.get("avg_frame_rate"),
        "reported_frame_count": stream.get("nb_frames"),
    }


def _validate_replay(repository_root: Path, replay_path: Path) -> dict[str, Any]:
    replay = read_object(replay_path)
    expected_hash = sha256(repository_root / REPLAY_PRODUCER_ID)
    if replay.get("schema") != REPLAY_SCHEMA or replay.get("status") != "FORMAL_PRODUCT_MCAP_REPLAY_PASS" or replay.get("pass") is not True:
        raise Auto15EvidenceError("replay is not a passing canonical formal product replay")
    if replay.get("producer") != {"id": REPLAY_PRODUCER_ID, "sha256": expected_hash}:
        raise Auto15EvidenceError("replay producer identity does not match current source")
    checks = replay.get("checks")
    if not isinstance(checks, dict) or not checks or not all(value is True for value in checks.values()):
        raise Auto15EvidenceError("replay checks are incomplete or blocked")
    if replay.get("playback", {}).get("exit_code") != 0:
        raise Auto15EvidenceError("replay did not complete ros2 bag play")
    context = replay.get("formal_context")
    if not isinstance(context, dict) or {key: context.get(key) for key in ("source_commit", "source_tree")} != _git_identity(repository_root):
        raise Auto15EvidenceError("replay source commit/tree is not current")
    session_context = context.get("session")
    if not isinstance(session_context, dict) or not isinstance(session_context.get("path"), str):
        raise Auto15EvidenceError("replay has no formal session file binding")
    session = read_object(Path(session_context["path"]))
    if (
        session.get("report_id") != "tzcup_formal_final_acceptance_session_v1"
        or session.get("status") not in {"FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING", "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE"}
        or session.get("started_epoch_ns") != session_context.get("started_epoch_ns")
        or session.get("snapshot") != context.get("snapshot")
        or session.get("runtime_closure_binding") != context.get("runtime_closure_binding")
    ):
        raise Auto15EvidenceError("replay no longer binds the current formal session")
    if session.get("status") == "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING" and sha256(Path(session_context["path"])) != session_context.get("sha256_at_replay"):
        raise Auto15EvidenceError("running formal session changed after replay")
    snapshot_ref = context.get("snapshot_manifest")
    if not isinstance(snapshot_ref, dict) or not isinstance(snapshot_ref.get("path"), str):
        raise Auto15EvidenceError("replay has no snapshot manifest file binding")
    snapshot_path = Path(snapshot_ref["path"])
    if sha256(snapshot_path) != snapshot_ref.get("sha256") or snapshot_ref.get("sha256") != context["snapshot"].get("snapshot_manifest_sha256"):
        raise Auto15EvidenceError("current snapshot manifest hash mismatch")
    binding_ref = context.get("runtime_gate_binding")
    if not isinstance(binding_ref, dict) or not isinstance(binding_ref.get("path"), str):
        raise Auto15EvidenceError("replay has no runtime gate file binding")
    binding_path = Path(binding_ref["path"])
    if sha256(binding_path) != binding_ref.get("sha256"):
        raise Auto15EvidenceError("runtime gate binding hash mismatch")
    binding = read_object(binding_path)
    if (
        binding.get("status") != "FORMAL_RUNTIME_GATE_BOUND"
        or binding.get("runtime_closure_binding") != context.get("runtime_closure_binding")
        or binding.get("acceptance_session_binding", {}).get("snapshot") != context.get("snapshot")
        or binding.get("acceptance_session_binding", {}).get("session_started_epoch_ns") != session_context.get("started_epoch_ns")
    ):
        raise Auto15EvidenceError("runtime gate binding differs from replay context")
    artifacts = replay.get("input_artifacts")
    hashes = replay.get("input_hashes")
    if not isinstance(hashes, dict) or not isinstance(artifacts, dict) or set(artifacts) != {"model", "config", "dataset", "dependency"}:
        raise Auto15EvidenceError("replay provenance artifact bindings are incomplete")
    for name, reference in artifacts.items():
        if not isinstance(reference, dict) or not isinstance(reference.get("path"), str):
            raise Auto15EvidenceError(f"replay {name} provenance artifact reference is incomplete")
        if artifact_sha256(Path(reference["path"])) != reference.get("sha256") or reference.get("sha256") != hashes.get(name):
            raise Auto15EvidenceError(f"replay {name} provenance artifact hash mismatch")
    return replay


def build_execution_receipt(
    *,
    repository_root: Path,
    run_root: Path,
    scenario_id: str,
    seed: int,
    mission_group_id: str,
    video_path: Path,
    replay_path: Path,
) -> dict[str, Any]:
    contract = read_object(repository_root / "config/high_fidelity_vehicle/product_acceptance_contract.json")
    accounting = contract["auto15_execution_accounting"]
    if scenario_id not in accounting["scenario_ids"] or seed not in accounting["seeds"]:
        raise Auto15EvidenceError("scenario/seed is outside the fixed 18x10 matrix")
    if not mission_group_id or any(token in mission_group_id.upper() for token in ("AUTO-", "STAGE-")):
        raise Auto15EvidenceError("mission group must be a current AUTO-15 product mission identity")
    video = _inside(run_root, video_path, "video")
    replay_file = _inside(run_root, replay_path, "replay receipt")
    replay = _validate_replay(repository_root, replay_file)
    bag = _inside(run_root, Path(replay["bag"]["path"]), "MCAP")
    started_ns = replay["formal_context"]["session"]["started_epoch_ns"]
    for artifact, label in ((video, "video"), (replay_file, "replay receipt"), (bag, "MCAP")):
        if artifact.stat().st_mtime_ns < started_ns:
            raise Auto15EvidenceError(f"{label} predates the current formal session")
    if artifact_sha256(bag) != replay["bag"]["sha256"]:
        raise Auto15EvidenceError("MCAP changed after replay recalculation")
    video_audit = _video_audit(video)
    return {
        "schema": EXECUTION_SCHEMA,
        "status": "AUTO15_PRODUCT_EXECUTION_EVIDENCE_SEALED",
        "stage": "AUTO-15",
        "producer": _producer(repository_root),
        "sealed_epoch_ns": time.time_ns(),
        "execution_id": f"{scenario_id}:seed-{seed}",
        "scenario_id": scenario_id,
        "seed": seed,
        "mission_group_id": mission_group_id,
        "run_root": str(run_root.resolve()),
        "formal_context": replay["formal_context"],
        "input_hashes": replay["input_hashes"],
        "input_artifacts": replay["input_artifacts"],
        "video": {
            "path": str(video),
            "sha256": sha256(video),
            "size_bytes": video.stat().st_size,
            "audit": video_audit,
        },
        "mcap": replay["bag"],
        "replay": {"path": str(replay_file), "sha256": sha256(replay_file), "schema": replay["schema"]},
    }


def validate_execution_receipt(repository_root: Path, run_root: Path, path: Path) -> dict[str, Any]:
    path = _inside(run_root, path, "execution receipt")
    receipt = read_object(path)
    if receipt.get("schema") != EXECUTION_SCHEMA or receipt.get("status") != "AUTO15_PRODUCT_EXECUTION_EVIDENCE_SEALED":
        raise Auto15EvidenceError("invalid execution receipt schema or status")
    if receipt.get("producer") != _producer(repository_root):
        raise Auto15EvidenceError("execution receipt producer identity is stale or forged")
    if receipt.get("run_root") != str(run_root.resolve()):
        raise Auto15EvidenceError("execution receipt belongs to another run root")
    replay_ref = receipt.get("replay")
    replay_path = _inside(run_root, Path(replay_ref["path"]), "replay receipt")
    if sha256(replay_path) != replay_ref.get("sha256"):
        raise Auto15EvidenceError("referenced replay receipt hash mismatch")
    replay = _validate_replay(repository_root, replay_path)
    if (
        replay.get("formal_context") != receipt.get("formal_context")
        or replay.get("input_hashes") != receipt.get("input_hashes")
        or replay.get("input_artifacts") != receipt.get("input_artifacts")
    ):
        raise Auto15EvidenceError("execution and replay provenance differ")
    for field in ("video", "mcap"):
        reference = receipt.get(field)
        artifact = _inside(run_root, Path(reference["path"]), field)
        if artifact_sha256(artifact) != reference.get("sha256"):
            raise Auto15EvidenceError(f"{field} hash mismatch")
    return receipt


def build_group_receipt(repository_root: Path, run_root: Path, group_id: str, execution_paths: list[Path]) -> dict[str, Any]:
    if not execution_paths:
        raise Auto15EvidenceError("mission group must contain at least one execution")
    executions = [validate_execution_receipt(repository_root, run_root, path) for path in execution_paths]
    if any(item["mission_group_id"] != group_id for item in executions):
        raise Auto15EvidenceError("mission group membership mismatch")
    identities = {item["execution_id"] for item in executions}
    if len(identities) != len(executions):
        raise Auto15EvidenceError("mission group repeats an execution")
    contexts = {json.dumps(item["formal_context"], sort_keys=True) for item in executions}
    hashes = {json.dumps(item["input_hashes"], sort_keys=True) for item in executions}
    artifacts = {json.dumps(item["input_artifacts"], sort_keys=True) for item in executions}
    if len(contexts) != 1 or len(hashes) != 1 or len(artifacts) != 1:
        raise Auto15EvidenceError("mission group mixes formal sessions or input identities")
    return {
        "schema": GROUP_SCHEMA,
        "status": "AUTO15_INDEPENDENT_MISSION_GROUP_SEALED",
        "stage": "AUTO-15",
        "producer": _producer(repository_root),
        "sealed_epoch_ns": time.time_ns(),
        "mission_group_id": group_id,
        "run_root": str(run_root.resolve()),
        "formal_context": executions[0]["formal_context"],
        "input_hashes": executions[0]["input_hashes"],
        "input_artifacts": executions[0]["input_artifacts"],
        "members": [
            {"execution_id": item["execution_id"], "path": str(path.resolve()), "sha256": sha256(path)}
            for item, path in sorted(zip(executions, execution_paths), key=lambda pair: pair[0]["execution_id"])
        ],
    }


def validate_group_receipt(repository_root: Path, run_root: Path, path: Path) -> dict[str, Any]:
    path = _inside(run_root, path, "mission group receipt")
    group = read_object(path)
    if group.get("schema") != GROUP_SCHEMA or group.get("status") != "AUTO15_INDEPENDENT_MISSION_GROUP_SEALED":
        raise Auto15EvidenceError("invalid mission group receipt schema or status")
    if group.get("producer") != _producer(repository_root) or group.get("run_root") != str(run_root.resolve()):
        raise Auto15EvidenceError("mission group producer or run-root binding mismatch")
    members = group.get("members")
    if not isinstance(members, list) or not members:
        raise Auto15EvidenceError("mission group has no members")
    for member in members:
        member_path = _inside(run_root, Path(member["path"]), "group member receipt")
        if sha256(member_path) != member.get("sha256"):
            raise Auto15EvidenceError("mission group member receipt hash mismatch")
        execution = validate_execution_receipt(repository_root, run_root, member_path)
        if execution["execution_id"] != member.get("execution_id") or execution["mission_group_id"] != group.get("mission_group_id"):
            raise Auto15EvidenceError("mission group member identity mismatch")
        if (
            execution["formal_context"] != group.get("formal_context")
            or execution["input_hashes"] != group.get("input_hashes")
            or execution["input_artifacts"] != group.get("input_artifacts")
        ):
            raise Auto15EvidenceError("mission group member provenance mismatch")
    return group


def build_ledger(repository_root: Path, run_root: Path, execution_paths: list[Path], group_paths: list[Path]) -> dict[str, Any]:
    contract = read_object(repository_root / "config/high_fidelity_vehicle/product_acceptance_contract.json")
    accounting = contract["auto15_execution_accounting"]
    executions = [validate_execution_receipt(repository_root, run_root, path) for path in execution_paths]
    expected = {f"{scenario}:seed-{seed}" for scenario in accounting["scenario_ids"] for seed in accounting["seeds"]}
    actual = {item["execution_id"] for item in executions}
    if actual != expected or len(executions) != len(expected):
        raise Auto15EvidenceError(f"execution ledger must contain exactly the fixed 18x10 matrix; missing={sorted(expected-actual)}, extra={sorted(actual-expected)}")
    groups = [validate_group_receipt(repository_root, run_root, path) for path in group_paths]
    if len(groups) < accounting["minimum_mission_group_count"] or len({item["mission_group_id"] for item in groups}) != len(groups):
        raise Auto15EvidenceError("ledger requires at least 30 unique independent mission groups")
    member_map: dict[str, str] = {}
    for group in groups:
        for member in group["members"]:
            execution_id = member["execution_id"]
            if execution_id in member_map:
                raise Auto15EvidenceError(f"execution appears in multiple mission groups: {execution_id}")
            member_map[execution_id] = group["mission_group_id"]
    if set(member_map) != expected:
        raise Auto15EvidenceError("mission-group receipts do not cover every execution exactly once")
    if any(item["mission_group_id"] != member_map[item["execution_id"]] for item in executions):
        raise Auto15EvidenceError("execution-to-mission-group declarations disagree")
    for field in ("video", "mcap"):
        identities = {(item[field]["path"], item[field]["sha256"]) for item in executions}
        if len(identities) != len(executions):
            raise Auto15EvidenceError(f"{field} evidence is reused across executions")
    contexts = {json.dumps(item["formal_context"], sort_keys=True) for item in executions}
    hashes = {json.dumps(item["input_hashes"], sort_keys=True) for item in executions}
    artifacts = {json.dumps(item["input_artifacts"], sort_keys=True) for item in executions}
    if len(contexts) != 1 or len(hashes) != 1 or len(artifacts) != 1:
        raise Auto15EvidenceError("ledger mixes formal sessions or provenance inputs")
    return {
        "schema": LEDGER_SCHEMA,
        "status": "AUTO15_CANONICAL_EVIDENCE_LEDGER_COMPLETE",
        "stage": "AUTO-15",
        "producer": _producer(repository_root),
        "sealed_epoch_ns": time.time_ns(),
        "run_root": str(run_root.resolve()),
        "formal_context": executions[0]["formal_context"],
        "input_hashes": executions[0]["input_hashes"],
        "input_artifacts": executions[0]["input_artifacts"],
        "execution_count": len(executions),
        "mission_group_count": len(groups),
        "executions": [
            {"execution_id": item["execution_id"], "path": str(path.resolve()), "sha256": sha256(path)}
            for item, path in sorted(zip(executions, execution_paths), key=lambda pair: pair[0]["execution_id"])
        ],
        "mission_groups": [
            {"mission_group_id": item["mission_group_id"], "path": str(path.resolve()), "sha256": sha256(path)}
            for item, path in sorted(zip(groups, group_paths), key=lambda pair: pair[0]["mission_group_id"])
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, default=ROOT)
    parser.add_argument("--run-root", type=Path, required=True)
    subparsers = parser.add_subparsers(dest="command", required=True)
    execution = subparsers.add_parser("execution")
    execution.add_argument("--scenario", required=True)
    execution.add_argument("--seed", type=int, required=True)
    execution.add_argument("--mission-group", required=True)
    execution.add_argument("--video", type=Path, required=True)
    execution.add_argument("--replay", type=Path, required=True)
    execution.add_argument("--output", type=Path, required=True)
    group = subparsers.add_parser("mission-group")
    group.add_argument("--mission-group", required=True)
    group.add_argument("--execution-receipt", type=Path, action="append", required=True)
    group.add_argument("--output", type=Path, required=True)
    ledger = subparsers.add_parser("ledger")
    ledger.add_argument("--execution-receipt", type=Path, action="append", required=True)
    ledger.add_argument("--mission-group-receipt", type=Path, action="append", required=True)
    ledger.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    repository_root = args.repository_root.resolve()
    run_root = args.run_root.absolute()
    if run_root.is_symlink() or run_root.resolve() != run_root:
        print(json.dumps({"status": "AUTO15_PRODUCT_EVIDENCE_BLOCKED", "error": "run root must be a canonical non-link path"}, sort_keys=True))
        return 2
    try:
        if args.command == "execution":
            value = build_execution_receipt(
                repository_root=repository_root, run_root=run_root,
                scenario_id=args.scenario, seed=args.seed, mission_group_id=args.mission_group,
                video_path=args.video, replay_path=args.replay,
            )
        elif args.command == "mission-group":
            value = build_group_receipt(repository_root, run_root, args.mission_group, args.execution_receipt)
        else:
            value = build_ledger(repository_root, run_root, args.execution_receipt, args.mission_group_receipt)
        write_fresh_json(_inside(run_root, args.output, "output"), value)
    except (OSError, ValueError, KeyError, TypeError, ProductReplayError, Auto15EvidenceError) as exc:
        print(json.dumps({"status": "AUTO15_PRODUCT_EVIDENCE_BLOCKED", "error": str(exc)}, sort_keys=True))
        return 2
    print(json.dumps(value, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
