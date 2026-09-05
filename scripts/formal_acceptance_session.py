#!/usr/bin/env python3
"""Bind formal runtime evidence to one frozen vehicle source snapshot."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import time
from pathlib import Path
from typing import Any

import yaml

from formal_final_runtime_closure import ClosureError, verify_recorded_manifest


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config/high_fidelity_vehicle/formal_functional_acceptance_contract.yaml"
DEFAULT_SNAPSHOT = ROOT / "reports/engineering/formal_vehicle_snapshot_manifest.json"
DEFAULT_OUTPUT = ROOT / "artifacts/formal_final_acceptance_session.json"


class AcceptanceSessionError(RuntimeError):
    pass


def _strict_json_equal(actual: Any, expected: Any) -> bool:
    """Compare JSON-compatible values without Python's bool/int coercion."""

    if type(actual) is not type(expected):
        return False
    if isinstance(expected, dict):
        return set(actual) == set(expected) and all(
            _strict_json_equal(actual[key], value) for key, value in expected.items()
        )
    if isinstance(expected, list):
        return len(actual) == len(expected) and all(
            _strict_json_equal(actual_item, expected_item)
            for actual_item, expected_item in zip(actual, expected)
        )
    return actual == expected


def _finite_contract_number(value: Any) -> bool:
    return type(value) in (int, float) and (
        not isinstance(value, float) or math.isfinite(value)
    )


def _json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise AcceptanceSessionError(f"cannot read JSON object {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise AcceptanceSessionError(f"JSON root is not an object: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _snapshot_identity(path: Path) -> dict[str, str]:
    snapshot = _json_object(path)
    outputs = snapshot.get("outputs")
    if not isinstance(outputs, dict):
        raise AcceptanceSessionError("snapshot manifest has no outputs mapping")
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf")
    if not isinstance(urdf, dict) or not isinstance(urdf.get("sha256"), str):
        raise AcceptanceSessionError("snapshot manifest has no expanded URDF hash")
    source_hash = snapshot.get("source_inventory_sha256")
    if not isinstance(source_hash, str) or not source_hash:
        raise AcceptanceSessionError("snapshot manifest has no source inventory hash")
    return {
        "snapshot_manifest_sha256": _sha256(path),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": urdf["sha256"],
    }


def _nested_value(payload: dict[str, Any], dotted_path: str) -> Any:
    value: Any = payload
    for key in dotted_path.split("."):
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


def _validate_bound_files(
    manifest_path: Path,
    payload: dict[str, Any],
    mapping_field: str,
    *,
    minimum_mtime_ns: int,
) -> list[dict[str, Any]]:
    mapping = _nested_value(payload, mapping_field)
    if not isinstance(mapping, dict) or not mapping:
        raise AcceptanceSessionError(f"{mapping_field} is not a non-empty file mapping")
    bound: list[dict[str, Any]] = []
    base = manifest_path.parent.resolve()
    for name, row in sorted(mapping.items()):
        if not isinstance(row, dict):
            raise AcceptanceSessionError(f"{mapping_field}.{name} is not an object")
        relative = row.get("path")
        expected_hash = row.get("png_sha256")
        expected_size = row.get("png_size_bytes")
        if not isinstance(relative, str) or not isinstance(expected_hash, str) or not isinstance(expected_size, int):
            raise AcceptanceSessionError(f"{mapping_field}.{name} has no complete file binding")
        candidate = (base / relative).resolve()
        if candidate.parent != base or not candidate.is_file():
            raise AcceptanceSessionError(f"{mapping_field}.{name} file is missing or escapes its evidence directory")
        if candidate.stat().st_mtime_ns < minimum_mtime_ns:
            raise AcceptanceSessionError(f"{mapping_field}.{name} file predates the acceptance session")
        actual_hash = _sha256(candidate)
        actual_size = candidate.stat().st_size
        if actual_hash != expected_hash or actual_size != expected_size:
            raise AcceptanceSessionError(f"{mapping_field}.{name} file hash or size mismatch")
        bound.append({"name": str(name), "path": relative, "sha256": actual_hash, "size_bytes": actual_size})
    return bound


def start(
    snapshot_path: Path,
    output: Path,
    *,
    runtime_closure_manifest: Path | None = None,
    runtime_install_root: Path | None = None,
    repository_root: Path = ROOT,
) -> dict[str, Any]:
    if output.exists():
        raise AcceptanceSessionError(
            f"refusing to overwrite retained acceptance session: {output}"
        )
    identity = _snapshot_identity(snapshot_path)
    result: dict[str, Any] = {
        "schema_version": 1,
        "report_id": "tzcup_formal_final_acceptance_session_v1",
        "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        "started_epoch_ns": time.time_ns(),
        "snapshot": identity,
        "evidence": {},
    }
    if (runtime_closure_manifest is None) != (runtime_install_root is None):
        raise AcceptanceSessionError(
            "runtime closure manifest and install root must be provided together"
        )
    if runtime_closure_manifest is not None and runtime_install_root is not None:
        closure_binding = verify_recorded_manifest(
            runtime_closure_manifest,
            repository_root.resolve(),
            runtime_install_root.resolve(),
        )
        closure_binding["runtime_install_root"] = str(runtime_install_root.resolve())
        result["runtime_closure_binding"] = closure_binding
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + f".pending.{os.getpid()}")
    temporary.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(output)
    return result


def finalize(
    contract_path: Path,
    snapshot_path: Path,
    output: Path,
    root: Path,
) -> dict[str, Any]:
    session = _json_object(output)
    prior_status = session.get("status")
    if prior_status == "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING":
        if session.get("failures") != {"s100_live_runtime": "missing"}:
            raise AcceptanceSessionError(
                "PENDING session is not resumable; start a fresh session"
            )
    elif prior_status != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise AcceptanceSessionError(
            "session is not in RUNNING or resumable PENDING state; start a fresh session"
        )
    if session.get("snapshot") != _snapshot_identity(snapshot_path):
        raise AcceptanceSessionError("vehicle source snapshot changed after session start")
    started_ns = session.get("started_epoch_ns")
    if not isinstance(started_ns, int) or started_ns <= 0:
        raise AcceptanceSessionError("session has an invalid start time")
    contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
    if not isinstance(contract, dict) or not isinstance(contract.get("evidence_gates"), dict):
        raise AcceptanceSessionError("functional acceptance contract is invalid")

    evidence_rows: dict[str, Any] = {}
    failures: dict[str, str] = {}
    for gate_id, row in contract["evidence_gates"].items():
        if not isinstance(row, dict) or row.get("session_bound") is not True:
            continue
        relative = row.get("path")
        statuses = row.get("success_statuses")
        if not isinstance(relative, str) or not isinstance(statuses, list):
            failures[str(gate_id)] = "invalid gate contract"
            continue
        path = root / relative
        if not path.is_file():
            failures[str(gate_id)] = "missing"
            continue
        if path.stat().st_mtime_ns < started_ns:
            failures[str(gate_id)] = "predates session start"
            continue
        try:
            payload = _json_object(path)
        except AcceptanceSessionError as exc:
            failures[str(gate_id)] = str(exc)
            continue
        status = payload.get("status")
        if status not in statuses:
            failures[str(gate_id)] = f"non-passing status: {status}"
            continue
        expected_report_id = row.get("report_id")
        if expected_report_id is not None and not _strict_json_equal(
            payload.get("report_id"), expected_report_id
        ):
            failures[str(gate_id)] = "evidence report_id does not match the gate contract"
            continue
        required_values = row.get("required_values", {})
        if not isinstance(required_values, dict):
            failures[str(gate_id)] = "invalid required_values"
            continue
        value_mismatches = [
            str(field)
            for field, expected in required_values.items()
            if not _strict_json_equal(_nested_value(payload, str(field)), expected)
        ]
        if value_mismatches:
            failures[str(gate_id)] = (
                "required value mismatch: " + ", ".join(sorted(value_mismatches))
            )
            continue
        required_mapping_keys = row.get("required_mapping_keys", {})
        if not isinstance(required_mapping_keys, dict):
            failures[str(gate_id)] = "invalid required_mapping_keys"
            continue
        key_mismatches: list[str] = []
        invalid_key_contracts: list[str] = []
        for mapping_field, expected_keys in required_mapping_keys.items():
            if not isinstance(expected_keys, list):
                invalid_key_contracts.append(str(mapping_field))
                continue
            mapping = _nested_value(payload, str(mapping_field))
            if not isinstance(mapping, dict) or set(mapping) != {str(key) for key in expected_keys}:
                key_mismatches.append(str(mapping_field))
        if invalid_key_contracts:
            failures[str(gate_id)] = (
                "invalid required_mapping_keys entries: "
                + ", ".join(sorted(invalid_key_contracts))
            )
            continue
        if key_mismatches:
            failures[str(gate_id)] = (
                "mapping keys do not match: " + ", ".join(sorted(key_mismatches))
            )
            continue
        mapping_item_values = row.get("required_mapping_item_values", {})
        if not isinstance(mapping_item_values, dict):
            failures[str(gate_id)] = "invalid required_mapping_item_values"
            continue
        item_mismatch = None
        for mapping_field, requirements in mapping_item_values.items():
            mapping = _nested_value(payload, str(mapping_field))
            if not isinstance(mapping, dict) or not isinstance(requirements, dict):
                item_mismatch = str(mapping_field)
                break
            for item_name, item in mapping.items():
                if not isinstance(item, dict) or any(
                    not _strict_json_equal(_nested_value(item, str(field)), expected)
                    for field, expected in requirements.items()
                ):
                    item_mismatch = f"{mapping_field}.{item_name}"
                    break
            if item_mismatch:
                break
        if item_mismatch:
            failures[str(gate_id)] = f"mapping item value mismatch: {item_mismatch}"
            continue
        list_item_values = row.get("required_list_item_values", {})
        if not isinstance(list_item_values, dict):
            failures[str(gate_id)] = "invalid required_list_item_values"
            continue
        list_item_minimums = row.get("required_list_item_minimums", {})
        if not isinstance(list_item_minimums, dict):
            failures[str(gate_id)] = "invalid required_list_item_minimums"
            continue
        list_item_maximums = row.get("required_list_item_maximums", {})
        if not isinstance(list_item_maximums, dict):
            failures[str(gate_id)] = "invalid required_list_item_maximums"
            continue
        list_mismatch = None
        list_fields = set(list_item_values) | set(list_item_minimums) | set(list_item_maximums)
        for list_field in list_fields:
            items = _nested_value(payload, str(list_field))
            if not isinstance(items, list) or not items:
                list_mismatch = str(list_field)
                break
            expected_values = list_item_values.get(list_field, {})
            minimums = list_item_minimums.get(list_field, {})
            maximums = list_item_maximums.get(list_field, {})
            if not all(isinstance(requirement, dict) for requirement in (expected_values, minimums, maximums)):
                list_mismatch = str(list_field)
                break
            for index, item in enumerate(items):
                if not isinstance(item, dict):
                    list_mismatch = f"{list_field}[{index}]"
                    break
                if any(
                    not _strict_json_equal(_nested_value(item, str(field)), expected)
                    for field, expected in expected_values.items()
                ):
                    list_mismatch = f"{list_field}[{index}]"
                    break
                for field, minimum in minimums.items():
                    value = _nested_value(item, str(field))
                    if (
                        not _finite_contract_number(value)
                        or not _finite_contract_number(minimum)
                        or value < minimum
                    ):
                        list_mismatch = f"{list_field}[{index}]"
                        break
                if list_mismatch:
                    break
                for field, maximum in maximums.items():
                    value = _nested_value(item, str(field))
                    if (
                        not _finite_contract_number(value)
                        or not _finite_contract_number(maximum)
                        or value > maximum
                    ):
                        list_mismatch = f"{list_field}[{index}]"
                        break
                if list_mismatch:
                    break
            if list_mismatch:
                break
        if list_mismatch:
            failures[str(gate_id)] = f"list item value mismatch: {list_mismatch}"
            continue
        snapshot_fields = (
            (
                "snapshot_manifest_hash_field",
                "snapshot_manifest_sha256",
                "snapshot manifest hash",
            ),
            (
                "snapshot_urdf_hash_field",
                "expanded_urdf_sha256",
                "expanded URDF hash",
            ),
            (
                "snapshot_source_hash_field",
                "source_inventory_sha256",
                "source inventory hash",
            ),
        )
        snapshot_mismatch: str | None = None
        for field_name, identity_key, label in snapshot_fields:
            snapshot_hash_field = row.get(field_name)
            if snapshot_hash_field is None:
                continue
            if not isinstance(snapshot_hash_field, str) or not snapshot_hash_field:
                snapshot_mismatch = f"invalid {field_name}"
                break
            if _nested_value(payload, snapshot_hash_field) != session["snapshot"][identity_key]:
                snapshot_mismatch = (
                    f"evidence {label} does not match the frozen snapshot"
                )
                break
        if snapshot_mismatch:
            failures[str(gate_id)] = snapshot_mismatch
            continue
        bound_files: list[dict[str, Any]] = []
        bound_file_mapping = row.get("bound_file_mapping")
        if bound_file_mapping is not None:
            if not isinstance(bound_file_mapping, str) or not bound_file_mapping:
                failures[str(gate_id)] = "invalid bound_file_mapping"
                continue
            try:
                bound_files = _validate_bound_files(
                    path, payload, bound_file_mapping, minimum_mtime_ns=started_ns
                )
            except AcceptanceSessionError as exc:
                failures[str(gate_id)] = str(exc)
                continue
        evidence_rows[str(gate_id)] = {
            "path": relative,
            "sha256": _sha256(path),
            "status": status,
            "mtime_epoch_ns": path.stat().st_mtime_ns,
            "bound_files": bound_files,
        }

    session["finished_epoch_ns"] = time.time_ns()
    if prior_status == "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING":
        session["resumed_epoch_ns"] = session["finished_epoch_ns"]
    session["evidence"] = evidence_rows
    session["failures"] = failures
    session["status"] = (
        "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE"
        if not failures
        else "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING"
    )
    temporary = output.with_suffix(output.suffix + f".pending.{os.getpid()}")
    temporary.write_text(json.dumps(session, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(output)
    return session


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    start_parser = subparsers.add_parser("start")
    start_parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    start_parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    start_parser.add_argument("--runtime-closure-manifest", type=Path)
    start_parser.add_argument("--runtime-install-root", type=Path)
    start_parser.add_argument("--repository-root", type=Path, default=ROOT)
    final_parser = subparsers.add_parser("finalize")
    final_parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    final_parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    final_parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    final_parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args()
    try:
        if args.command == "start":
            result = start(
                args.snapshot,
                args.output,
                runtime_closure_manifest=args.runtime_closure_manifest,
                runtime_install_root=args.runtime_install_root,
                repository_root=args.repository_root,
            )
        else:
            result = finalize(args.contract, args.snapshot, args.output, args.root)
    except (
        AcceptanceSessionError,
        ClosureError,
        OSError,
        UnicodeError,
        yaml.YAMLError,
    ) as exc:
        print(json.dumps({"status": "INVALID", "error": str(exc)}, indent=2))
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"].endswith(("RUNNING", "COMPLETE")) else 3


if __name__ == "__main__":
    raise SystemExit(main())
