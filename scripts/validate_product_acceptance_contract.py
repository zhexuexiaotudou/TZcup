#!/usr/bin/env python3
"""Fail-closed A12/AUTO-15 contract, provenance, media, and replay validation."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path
from typing import Any

import yaml

from coverage_mcap_replay_audit import REQUIRED_TOPICS
from formal_acceptance_session import _strict_json_equal


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config" / "high_fidelity_vehicle" / "product_acceptance_contract.json"
HEX64 = re.compile(r"^[0-9a-f]{64}$")
HEX40 = re.compile(r"^[0-9a-f]{40}$")
MCAP_MAGIC = b"\x89MCAP0\r\n"


class ProductAcceptanceContractError(ValueError):
    """A receipt cannot be used as current product-acceptance evidence."""


def _json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ProductAcceptanceContractError(f"cannot read JSON object {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ProductAcceptanceContractError(f"JSON root must be an object: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_text_sha256(path: Path) -> str:
    text = path.read_text(encoding="utf-8").replace("\r\n", "\n")
    normalized = "\n".join(line.rstrip(" \t") for line in text.split("\n"))
    return hashlib.sha256(normalized.encode("utf-8")).hexdigest()


def _canonical_path(root: Path, relative: Any, label: str, *, directory: bool = False) -> Path:
    if root.is_symlink() or not root.is_dir():
        raise ProductAcceptanceContractError("evidence root must be a canonical non-link directory")
    if not isinstance(relative, str) or not relative:
        raise ProductAcceptanceContractError(f"{label} path is missing")
    candidate = Path(relative)
    if candidate.is_absolute() or any(part in {"", ".", ".."} for part in candidate.parts):
        raise ProductAcceptanceContractError(f"{label} path is not canonical and in-root")
    current = root.resolve()
    for part in candidate.parts:
        current = current / part
        if current.is_symlink():
            raise ProductAcceptanceContractError(f"{label} has a symbolic-link ancestor")
    resolved = current.resolve()
    try:
        resolved.relative_to(root.resolve())
    except ValueError as exc:
        raise ProductAcceptanceContractError(f"{label} escapes evidence root") from exc
    if directory:
        valid = resolved.is_dir()
    else:
        valid = resolved.is_file()
    if not valid or resolved.is_symlink():
        raise ProductAcceptanceContractError(f"{label} is not a regular in-root {'directory' if directory else 'file'}")
    return resolved


def _bound_file(root: Path, value: Any, label: str) -> tuple[Path, str, tuple[int, int]]:
    if not isinstance(value, dict):
        raise ProductAcceptanceContractError(f"{label} must be an object")
    expected = value.get("sha256")
    if not isinstance(expected, str) or not HEX64.fullmatch(expected):
        raise ProductAcceptanceContractError(f"{label} needs a SHA-256")
    path = _canonical_path(root, value.get("path"), label)
    actual = _sha256(path)
    if actual != expected:
        raise ProductAcceptanceContractError(f"{label} hash mismatch")
    stat = path.stat()
    if stat.st_size <= 0:
        raise ProductAcceptanceContractError(f"{label} is empty")
    return path, actual, (stat.st_dev, stat.st_ino)


def load_contract(path: Path = DEFAULT_CONTRACT) -> dict[str, Any]:
    return _json_object(path)


def validate_static_contract(
    contract: dict[str, Any], authoritative_source: Path | None = None
) -> dict[str, Any]:
    accounting = contract.get("auto15_execution_accounting")
    runtime_states = contract.get("product_runtime_states")
    spec = contract.get("authoritative_specification")
    runtime = contract.get("auto15_runtime_receipt_contract")
    if contract.get("contract_id") != "tzcup_product_acceptance_contract_v1":
        raise ProductAcceptanceContractError("unexpected product-acceptance contract id")
    if not all(isinstance(value, dict) for value in (accounting, runtime_states, spec, runtime)):
        raise ProductAcceptanceContractError("contract is missing required objects")
    scenarios, seeds = accounting.get("scenario_ids"), accounting.get("seeds")
    if not isinstance(scenarios, list) or len(scenarios) != 18 or len(set(scenarios)) != 18:
        raise ProductAcceptanceContractError("AUTO-15 must declare exactly 18 unique scenarios")
    if not isinstance(seeds, list) or seeds != list(range(10)):
        raise ProductAcceptanceContractError("AUTO-15 must declare seeds 0 through 9 exactly once")
    if accounting.get("required_execution_count") != len(scenarios) * len(seeds):
        raise ProductAcceptanceContractError("AUTO-15 execution cardinality drifted")
    if accounting.get("minimum_mission_group_count") != 30 or accounting.get("required_execution_evidence") != ["video", "mcap"]:
        raise ProductAcceptanceContractError("AUTO-15 mission-group or media contract drifted")
    if spec.get("path") != "docs/a12-product-acceptance-specification.md" or not isinstance(spec.get("authoritative_source_path"), str):
        raise ProductAcceptanceContractError("A12 authoritative source path drifted")
    digest = spec.get("canonical_content_sha256")
    specification = ROOT / spec["path"]
    if not isinstance(digest, str) or not HEX64.fullmatch(digest) or not specification.is_file() or _canonical_text_sha256(specification) != digest:
        raise ProductAcceptanceContractError("versioned A12 specification digest drifted")
    if authoritative_source is not None:
        if not authoritative_source.is_file() or _canonical_text_sha256(authoritative_source) != digest:
            raise ProductAcceptanceContractError("authoritative A12 source content drifted")
    if any(value is not False for value in runtime_states.values()):
        raise ProductAcceptanceContractError("static contract must not promote product runtime states")
    if runtime.get("session_report_id") != "tzcup_formal_final_acceptance_session_v1" or runtime.get("runtime_binding_status") != "FORMAL_RUNTIME_GATE_BOUND" or runtime.get("video_audit_stage") != "AUTO-17" or runtime.get("mcap_replay_schema") != "tzcup.coverage_mcap_replay.v1":
        raise ProductAcceptanceContractError("runtime receipt contract drifted")
    execution_ids = [f"{scenario}:seed-{seed}" for scenario in scenarios for seed in seeds]
    return {"static_contract_pass": True, "scenario_count": len(scenarios), "seed_count_per_scenario": len(seeds), "required_execution_count": len(execution_ids), "required_execution_ids": execution_ids, "minimum_mission_group_count": 30, "product_runtime_states": runtime_states}


def _provenance(contract: dict[str, Any], payload: dict[str, Any], root: Path) -> dict[str, Any]:
    value = payload.get("provenance")
    if not isinstance(value, dict):
        raise ProductAcceptanceContractError("ledger has no provenance")
    session_path, session_hash, _ = _bound_file(root, value.get("formal_session"), "formal session")
    binding_path, binding_hash, _ = _bound_file(root, value.get("runtime_binding"), "runtime binding")
    session, binding = _json_object(session_path), _json_object(binding_path)
    runtime = contract["auto15_runtime_receipt_contract"]
    snapshot = session.get("snapshot")
    binding_session = binding.get("acceptance_session_binding")
    if session.get("report_id") != runtime["session_report_id"] or session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING" or not isinstance(session.get("started_epoch_ns"), int) or not isinstance(snapshot, dict):
        raise ProductAcceptanceContractError("formal session is not a current running frozen session")
    if binding.get("status") != runtime["runtime_binding_status"] or not isinstance(binding_session, dict) or binding_session.get("session_manifest_sha256") != session_hash or not _strict_json_equal(binding_session.get("snapshot"), snapshot):
        raise ProductAcceptanceContractError("runtime binding is not bound to the formal session and snapshot")
    run_root = _canonical_path(root, value.get("run_root"), "run root", directory=True)
    identity = {"formal_session_sha256": session_hash, "runtime_binding_sha256": binding_hash, "snapshot": snapshot, "run_root": run_root.relative_to(root.resolve()).as_posix()}
    for field in runtime["required_provenance_fields"]:
        if field == "run_root":
            continue
        item = value.get(field)
        if field == "container_digest":
            valid = isinstance(item, str) and re.fullmatch(r"sha256:[0-9a-f]{64}", item)
        elif field in {"source_commit", "source_tree"}:
            valid = isinstance(item, str) and HEX40.fullmatch(item)
        else:
            valid = isinstance(item, str) and HEX64.fullmatch(item)
        if not valid:
            raise ProductAcceptanceContractError(f"ledger provenance is missing {field}")
        identity[field] = item
    identity["session_started_epoch_ns"] = session["started_epoch_ns"]
    return identity


def _validate_video(root: Path, value: Any, used: set[tuple[tuple[int, int], str]]) -> dict[str, Any]:
    path, digest, inode = _bound_file(root, value, "video")
    if any(saved_inode == inode or saved_digest == digest for saved_inode, saved_digest in used) or path.stat().st_size < 100_000:
        raise ProductAcceptanceContractError("video is reused or below the canonical visual-audit minimum")
    with path.open("rb") as stream:
        header = stream.read(12)
    if len(header) < 12 or header[4:8] != b"ftyp":
        raise ProductAcceptanceContractError("video is not an ISO-BMFF media file")
    audit_path, _, _ = _bound_file(root, value.get("audit"), "video audit")
    audit = _json_object(audit_path)
    video = audit.get("video")
    if audit.get("stage") != "AUTO-17" or audit.get("status") != "PASS" or audit.get("machine_gate_pass") is not True or not isinstance(video, dict) or video.get("path") != path.name or video.get("bytes") != path.stat().st_size or video.get("nonempty") is not True:
        raise ProductAcceptanceContractError("video does not satisfy the canonical visual-demo audit contract")
    frame = video.get("representative_frame")
    _canonical_path(audit_path.parent, frame, "video representative frame")
    used.add((inode, digest))
    return {"path": path.relative_to(root.resolve()).as_posix(), "sha256": digest}


def _validate_mcap(root: Path, value: Any, used: set[tuple[tuple[int, int], str]]) -> dict[str, Any]:
    data_path, digest, inode = _bound_file(root, value.get("data"), "MCAP data")
    metadata_path, metadata_digest, metadata_inode = _bound_file(root, value.get("metadata"), "MCAP metadata")
    replay_path, replay_digest, replay_inode = _bound_file(root, value.get("replay"), "MCAP replay audit")
    artifacts = ((inode, digest), (metadata_inode, metadata_digest), (replay_inode, replay_digest))
    if any(saved_inode == artifact_inode or saved_digest == artifact_digest for artifact_inode, artifact_digest in artifacts for saved_inode, saved_digest in used):
        raise ProductAcceptanceContractError("MCAP evidence artifact is reused")
    data = data_path.read_bytes()
    if len(data) <= 2 * len(MCAP_MAGIC) or not data.startswith(MCAP_MAGIC) or not data.endswith(MCAP_MAGIC):
        raise ProductAcceptanceContractError("MCAP data has no valid MCAP envelope")
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        info = metadata["rosbag2_bagfile_information"]
        if int(info["message_count"]) <= 0 or int(info["duration"]["nanoseconds"]) <= 0:
            raise ValueError("empty bag")
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise ProductAcceptanceContractError("MCAP metadata is not a populated rosbag2 receipt") from exc
    replay = _json_object(replay_path)
    topics = set(replay.get("topic_message_counts", {}))
    gates = replay.get("gates")
    if replay.get("schema") != "tzcup.coverage_mcap_replay.v1" or replay.get("pass") is not True or replay.get("bag_readable") is not True or replay.get("ros2_bag_play_exit_code") != 0 or not isinstance(gates, dict) or not gates or not all(gates.values()) or not REQUIRED_TOPICS <= topics:
        raise ProductAcceptanceContractError("MCAP does not satisfy the canonical replay audit contract")
    used.update(artifacts)
    return {"path": data_path.relative_to(root.resolve()).as_posix(), "sha256": digest}


def validate_auto15_execution_evidence(contract: dict[str, Any], payload: dict[str, Any], evidence_root: Path) -> dict[str, Any]:
    static = validate_static_contract(contract)
    root = evidence_root.resolve()
    provenance = _provenance(contract, payload, root)
    executions, groups = payload.get("executions"), payload.get("mission_groups")
    if not isinstance(executions, list) or not isinstance(groups, list):
        raise ProductAcceptanceContractError("ledger requires execution and mission-group receipts")
    expected, observed, video_used, mcap_used = set(static["required_execution_ids"]), set(), set(), set()
    by_group: dict[str, list[str]] = {}
    scenario_counts = {scenario: {"executions": 0, "videos": 0, "mcaps": 0} for scenario in contract["auto15_execution_accounting"]["scenario_ids"]}
    for item in executions:
        if not isinstance(item, dict):
            raise ProductAcceptanceContractError("execution receipt must be an object")
        execution_id = f"{item.get('scenario_id')}:seed-{item.get('seed')}"
        if execution_id not in expected or execution_id in observed or item.get("status") != "PASS":
            raise ProductAcceptanceContractError(f"invalid, duplicate, or non-passing execution: {execution_id}")
        if not _strict_json_equal(item.get("provenance"), {key: value for key, value in provenance.items() if key != "session_started_epoch_ns"}):
            raise ProductAcceptanceContractError(f"execution provenance is stale or unbound: {execution_id}")
        if not isinstance(item.get("command"), list) or not item["command"] or item.get("exit_code") != 0 or not isinstance(item.get("started_epoch_ns"), int) or not isinstance(item.get("finished_epoch_ns"), int) or item["started_epoch_ns"] < provenance["session_started_epoch_ns"] or item["finished_epoch_ns"] < item["started_epoch_ns"]:
            raise ProductAcceptanceContractError(f"execution command, exit, or time receipt is invalid: {execution_id}")
        group = item.get("mission_group_id")
        if not isinstance(group, str) or not group:
            raise ProductAcceptanceContractError(f"execution has no mission group: {execution_id}")
        _validate_video(root, item.get("video"), video_used)
        _validate_mcap(root, item.get("mcap"), mcap_used)
        if item.get("replay") is not True:
            raise ProductAcceptanceContractError(f"execution replay claim is missing: {execution_id}")
        observed.add(execution_id)
        by_group.setdefault(group, []).append(execution_id)
        scenario_counts[item["scenario_id"]]["executions"] += 1
        scenario_counts[item["scenario_id"]]["videos"] += 1
        scenario_counts[item["scenario_id"]]["mcaps"] += 1
    if observed != expected:
        raise ProductAcceptanceContractError("all 180 unique scenario/seed receipts are required")
    group_ids = set()
    grouped_members = set()
    group_receipts = set()
    for group in groups:
        if not isinstance(group, dict) or not isinstance(group.get("mission_group_id"), str) or group["mission_group_id"] in group_ids:
            raise ProductAcceptanceContractError("mission group identity is invalid or duplicated")
        members = group.get("members")
        if not isinstance(members, list) or set(members) != set(by_group.get(group["mission_group_id"], [])) or grouped_members.intersection(members):
            raise ProductAcceptanceContractError("mission-group membership is incomplete or non-independent")
        receipt_path, receipt_hash, receipt_inode = _bound_file(root, group.get("receipt"), "mission-group receipt")
        if (receipt_inode, receipt_hash) in group_receipts:
            raise ProductAcceptanceContractError("mission groups reuse one receipt")
        receipt = _json_object(receipt_path)
        if receipt.get("report_id") != "tzcup_auto15_mission_group_receipt_v1" or receipt.get("status") != "PASS" or receipt.get("mission_group_id") != group["mission_group_id"] or not _strict_json_equal(receipt.get("members"), members) or not _strict_json_equal(receipt.get("provenance"), {key: value for key, value in provenance.items() if key != "session_started_epoch_ns"}):
            raise ProductAcceptanceContractError("mission-group receipt is stale, unbound, or does not enumerate members")
        group_ids.add(group["mission_group_id"]); grouped_members.update(members); group_receipts.add((receipt_inode, receipt_hash))
    if len(group_ids) < static["minimum_mission_group_count"] or grouped_members != expected:
        raise ProductAcceptanceContractError("at least 30 real independent mission-group receipts must cover all executions")
    return {**static, "execution_evidence_pass": True, "retained_execution_count": len(observed), "mission_group_count": len(group_ids), "execution_to_mission_group": {execution: group for group, members in by_group.items() for execution in members}, "scenario_evidence_counts": scenario_counts, "product_runtime_states": contract["product_runtime_states"]}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--execution-evidence", type=Path)
    parser.add_argument("--evidence-root", type=Path, default=Path.cwd())
    parser.add_argument("--authoritative-source", type=Path)
    args = parser.parse_args()
    try:
        contract = load_contract(args.contract)
        result = validate_static_contract(contract, args.authoritative_source)
        if args.execution_evidence:
            result = validate_auto15_execution_evidence(contract, _json_object(args.execution_evidence), args.evidence_root)
    except ProductAcceptanceContractError as exc:
        print(f"product acceptance contract failed closed: {exc}")
        return 2
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
