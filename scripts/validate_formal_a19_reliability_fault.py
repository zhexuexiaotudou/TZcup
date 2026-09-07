#!/usr/bin/env python3
"""Fail closed on an A19 reliability claim without current bound raw evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import stat
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config/high_fidelity_vehicle/formal_a19_reliability_fault_contract.json"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
PASS = "A19_TWO_HOUR_RELIABILITY_FAULT_PASSED"
BLOCKED = "A19_TWO_HOUR_RELIABILITY_FAULT_BLOCKED"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root is not an object: {path}")
    return value


def _digest(value: Any) -> bool:
    return isinstance(value, str) and bool(SHA256_RE.fullmatch(value.lower()))


def _number(value: Any) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def _integer(value: Any) -> bool:
    return type(value) is int


def _snapshot_identity(path: Path) -> dict[str, str]:
    payload = _object(path)
    source = payload.get("source_inventory_sha256")
    outputs = payload.get("outputs")
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf") if isinstance(outputs, dict) else None
    expanded = urdf.get("sha256") if isinstance(urdf, dict) else None
    if not _digest(source) or not _digest(expanded):
        raise ValueError("snapshot has no valid source and expanded-URDF hashes")
    return {
        "snapshot_manifest_sha256": _sha256(path),
        "source_inventory_sha256": source.lower(),
        "expanded_urdf_sha256": expanded.lower(),
    }


def _closure_identity(path: Path) -> dict[str, str]:
    payload = _object(path)
    closure = payload.get("closure_sha256")
    if (
        payload.get("kind") != "tzcup_formal_final_runtime_closure"
        or payload.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_FROZEN"
        or not _digest(closure)
    ):
        raise ValueError("runtime closure is not a frozen formal closure")
    return {
        "runtime_closure_manifest_sha256": _sha256(path),
        "runtime_closure_sha256": closure.lower(),
    }


def _session_identity(path: Path, snapshot: dict[str, str], closure: dict[str, str]) -> dict[str, Any]:
    payload = _object(path)
    started = payload.get("started_epoch_ns")
    expected_closure = payload.get("runtime_closure_binding")
    expected_closure_pair = {
        "runtime_closure_manifest_sha256": expected_closure.get("manifest_sha256") if isinstance(expected_closure, dict) else None,
        "runtime_closure_sha256": expected_closure.get("closure_sha256") if isinstance(expected_closure, dict) else None,
    }
    if (
        payload.get("report_id") != "tzcup_formal_final_acceptance_session_v1"
        or payload.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
        or not isinstance(started, int)
        or started <= 0
        or payload.get("snapshot") != snapshot
        or expected_closure_pair != closure
    ):
        raise ValueError("acceptance session is not the current running snapshot/closure session")
    return {
        "session_manifest_sha256": _sha256(path),
        "session_started_epoch_ns": started,
        "snapshot": snapshot,
    }


def _safe_evidence(path_value: Any, expected_hash: Any, evidence_root: Path) -> tuple[Path | None, str | None]:
    if not isinstance(path_value, str) or not _digest(expected_hash):
        return None, "missing relative path or SHA-256"
    relative = Path(path_value)
    if relative.is_absolute() or ".." in relative.parts:
        return None, "path escapes evidence root"
    path = evidence_root / relative
    try:
        parent = path.parent
        while parent != evidence_root.parent:
            if stat.S_ISLNK(parent.lstat().st_mode):
                return None, "path has a symlinked ancestor"
            parent = parent.parent
        mode = path.lstat().st_mode
    except OSError:
        return None, "file is missing"
    if stat.S_ISLNK(mode) or not stat.S_ISREG(mode):
        return None, "file is not a regular non-symlink"
    if _sha256(path) != expected_hash.lower():
        return None, "SHA-256 mismatch"
    return path, None


def _blocked(contract: dict[str, Any], blockers: list[str], report_hash: str | None = None) -> dict[str, Any]:
    result: dict[str, Any] = {
        "schema_version": 1,
        "report_id": "tzcup_formal_a19_reliability_fault_validation_v1",
        "status": BLOCKED,
        "passed": False,
        "contract_id": contract.get("contract_id"),
        "blockers": sorted(set(blockers)),
    }
    if report_hash:
        result["candidate_report_sha256"] = report_hash
    return result


def validate(
    report_path: Path,
    snapshot_path: Path,
    session_path: Path,
    closure_path: Path,
    evidence_root: Path,
    contract_path: Path = DEFAULT_CONTRACT,
) -> dict[str, Any]:
    """Validate one retained report and return PASS only with every binding intact."""

    try:
        contract = _object(contract_path)
        requirements = contract["requirements"]
        required_keys = {
            "minimum_duration_s", "minimum_coverage_ratio", "maximum_collision_count",
            "maximum_localization_p95_m", "maximum_estop_brake_latency_s",
            "required_fault_profiles", "required_safety_outcome", "required_recovery_outcome",
            "required_raw_evidence",
        }
        if not isinstance(requirements, dict) or not required_keys <= set(requirements):
            raise ValueError("contract requirements are invalid")
    except (OSError, UnicodeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid A19 contract: {exc}") from exc

    try:
        report_hash = _sha256(report_path)
        report = _object(report_path)
    except (OSError, UnicodeError, ValueError, json.JSONDecodeError) as exc:
        return _blocked(contract, [f"candidate report unreadable: {exc}"])

    blockers: list[str] = []
    try:
        snapshot = _snapshot_identity(snapshot_path)
        closure = _closure_identity(closure_path)
        session = _session_identity(session_path, snapshot, closure)
    except (OSError, UnicodeError, ValueError, json.JSONDecodeError) as exc:
        return _blocked(contract, [str(exc)], report_hash)

    if report.get("schema_version") != 1 or report.get("report_id") != "tzcup_formal_a19_reliability_fault_report_v1":
        blockers.append("candidate report identity is invalid")
    if report.get("acceptance_session_binding") != session:
        blockers.append("candidate report does not bind the current acceptance session")
    if report.get("runtime_closure_binding") != closure:
        blockers.append("candidate report does not bind the current runtime closure")

    collection = report.get("collection")
    valid_collection = False
    if not isinstance(collection, dict):
        blockers.append("collection is missing")
    else:
        started, ended = collection.get("started_epoch_ns"), collection.get("ended_epoch_ns")
        minimum_ns = int(requirements["minimum_duration_s"]) * 1_000_000_000
        valid_collection = _integer(started) and _integer(ended) and ended >= started
        if not valid_collection or ended - started < minimum_ns:
            blockers.append(f"collection duration is below {requirements['minimum_duration_s']} seconds")
        if _integer(started) and started < session["session_started_epoch_ns"]:
            blockers.append("collection predates the acceptance session")

    raw = report.get("raw_evidence")
    bound: dict[str, Path] = {}
    if not isinstance(raw, dict):
        blockers.append("raw evidence mapping is missing")
    else:
        for name in requirements["required_raw_evidence"]:
            item = raw.get(name)
            if not isinstance(item, dict):
                blockers.append(f"raw evidence {name} is missing")
                continue
            path, error = _safe_evidence(item.get("path"), item.get("sha256"), evidence_root)
            if error:
                blockers.append(f"raw evidence {name}: {error}")
            elif path is not None:
                bound[name] = path

    telemetry: dict[str, Any] | None = None
    fault_receipt: dict[str, Any] | None = None
    for name, target in (("telemetry", "telemetry"), ("fault_injection", "fault_receipt")):
        if name not in bound:
            continue
        try:
            value = _object(bound[name])
            if name == "telemetry":
                telemetry = value
            else:
                fault_receipt = value
        except (OSError, UnicodeError, ValueError, json.JSONDecodeError):
            blockers.append(f"raw evidence {name} is not a JSON object")

    metrics = report.get("metrics")
    if not isinstance(metrics, dict) or telemetry is None:
        blockers.append("metrics or raw telemetry is missing")
    else:
        for field in ("coverage_ratio", "collision_count", "localization_p95_m", "max_estop_brake_latency_s"):
            if metrics.get(field) != telemetry.get(field):
                blockers.append(f"raw telemetry does not match metric {field}")
        if (not _number(metrics.get("coverage_ratio")) or metrics["coverage_ratio"] < requirements["minimum_coverage_ratio"]):
            blockers.append("coverage ratio is below the SIL threshold")
        if not _integer(metrics.get("collision_count")) or metrics["collision_count"] != requirements["maximum_collision_count"]:
            blockers.append("collision count is not zero")
        if (not _number(metrics.get("localization_p95_m")) or metrics["localization_p95_m"] > requirements["maximum_localization_p95_m"]):
            blockers.append("localization P95 exceeds the SIL threshold")
        if (not _number(metrics.get("max_estop_brake_latency_s")) or metrics["max_estop_brake_latency_s"] > requirements["maximum_estop_brake_latency_s"]):
            blockers.append("emergency-stop brake latency exceeds one second")
        if collection != telemetry.get("collection"):
            blockers.append("raw telemetry collection window does not match report")

    records = report.get("fault_injections")
    if not isinstance(records, list) or fault_receipt is None:
        blockers.append("fault injection records or raw receipt is missing")
    elif records != fault_receipt.get("fault_injections"):
        blockers.append("fault injection records do not match the raw receipt")
    else:
        profiles = set()
        for record in records:
            if not isinstance(record, dict):
                blockers.append("fault injection record is not an object")
                continue
            profile = record.get("profile")
            profiles.add(profile)
            if record.get("safety_outcome") != requirements["required_safety_outcome"]:
                blockers.append(f"fault {profile} lacks a safe-stop outcome")
            if record.get("recovery_outcome") != requirements["required_recovery_outcome"]:
                blockers.append(f"fault {profile} lacks a safe recovery outcome")
            if (not valid_collection or not _integer(record.get("injected_epoch_ns")) or not collection["started_epoch_ns"] <= record["injected_epoch_ns"] <= collection["ended_epoch_ns"]):
                blockers.append(f"fault {profile} is outside the collection window")
        missing = set(requirements["required_fault_profiles"]) - profiles
        if missing:
            blockers.append("missing fault profiles: " + ", ".join(sorted(missing)))

    if blockers:
        return _blocked(contract, blockers, report_hash)
    return {
        "schema_version": 1,
        "report_id": "tzcup_formal_a19_reliability_fault_validation_v1",
        "status": PASS,
        "passed": True,
        "contract_id": contract["contract_id"],
        "candidate_report_sha256": report_hash,
        "acceptance_session_binding": session,
        "runtime_closure_binding": closure,
    }


def _atomic_write(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + f".pending.{os.getpid()}")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--acceptance-session", type=Path, required=True)
    parser.add_argument("--runtime-closure", type=Path, required=True)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"refusing to overwrite retained validation report: {args.output}")
    try:
        result = validate(args.report, args.snapshot, args.acceptance_session, args.runtime_closure, args.evidence_root, args.contract)
    except ValueError as exc:
        print(json.dumps({"status": "INVALID", "error": str(exc)}, indent=2))
        return 2
    _atomic_write(args.output, result)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["passed"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
