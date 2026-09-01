#!/usr/bin/env python3
"""Audit all formal vehicle function positions against runtime evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config/high_fidelity_vehicle/formal_functional_acceptance_contract.yaml"
DEFAULT_REGISTER = ROOT / "config/high_fidelity_vehicle/formal_vehicle_component_register.yaml"


class FunctionalAcceptanceError(RuntimeError):
    pass


def _finite_contract_number(value: Any) -> bool:
    return type(value) in (int, float) and (
        not isinstance(value, float) or math.isfinite(value)
    )


def _mapping(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise FunctionalAcceptanceError(f"cannot read structured file {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise FunctionalAcceptanceError(f"structured root must be a mapping: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _nested_value(payload: dict[str, Any], dotted_path: str) -> Any:
    value: Any = payload
    for key in dotted_path.split("."):
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


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


def _json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise FunctionalAcceptanceError(
            f"cannot read JSON object {path}: {exc}"
        ) from exc
    if not isinstance(value, dict):
        raise FunctionalAcceptanceError(f"JSON root is not an object: {path}")
    return value


def _runtime_binding_error(
    *,
    evidence_path: Path,
    evidence: dict[str, Any],
    gate_contract: dict[str, Any],
    session: dict[str, Any],
) -> str | None:
    """Return a fail-closed reason when a runtime report lacks its exact sidecar.

    The final aggregate is also a public direct entry point.  It must therefore
    repeat the identity checks normally performed by the serial orchestrator,
    instead of trusting that the report was produced through that orchestrator.
    """

    binding_contract = gate_contract.get("runtime_binding")
    if binding_contract is None:
        return None
    if binding_contract != {
        "report_field": "runtime_gate_binding",
        "sidecar_suffix": ".runtime_binding.json",
    }:
        return "runtime binding contract is invalid"
    sidecar = evidence_path.with_name(
        evidence_path.name + str(binding_contract["sidecar_suffix"])
    )
    if sidecar.is_symlink() or not sidecar.is_file():
        return "runtime binding sidecar is missing or not a regular file"
    try:
        binding = _json_object(sidecar)
    except FunctionalAcceptanceError as exc:
        return str(exc)
    if not _strict_json_equal(
        _nested_value(evidence, str(binding_contract["report_field"])), binding
    ):
        return "report runtime binding differs from its sidecar"
    session_binding = binding.get("acceptance_session_binding")
    closure_binding = binding.get("runtime_closure_binding")
    session_closure = session.get("runtime_closure_binding")
    started = session.get("started_epoch_ns")
    if (
        binding.get("schema_version") != 1
        or binding.get("status") != "FORMAL_RUNTIME_GATE_BOUND"
        or not isinstance(session_binding, dict)
        or not isinstance(closure_binding, dict)
        or not isinstance(session_closure, dict)
        or not isinstance(started, int)
        or started <= 0
    ):
        return "runtime binding has incomplete session or closure identity"
    if (
        session_binding.get("snapshot") != session.get("snapshot")
        or session_binding.get("session_started_epoch_ns") != started
        or session_binding.get("session_status_at_gate")
        != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
        or session_binding.get("snapshot_current_source_verified") is not True
        or not isinstance(session_binding.get("session_manifest_sha256"), str)
        or len(session_binding["session_manifest_sha256"]) != 64
    ):
        return "runtime binding is not bound to the active acceptance session"
    if (
        closure_binding != session_closure
        or closure_binding.get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"
        or not isinstance(closure_binding.get("manifest_sha256"), str)
        or len(closure_binding["manifest_sha256"]) != 64
        or not isinstance(closure_binding.get("closure_sha256"), str)
        or len(closure_binding["closure_sha256"]) != 64
        or not isinstance(closure_binding.get("runtime_install_root"), str)
        or not closure_binding["runtime_install_root"]
        or closure_binding.get("symbolic_link_count") != 0
    ):
        return "runtime binding closure differs from the active acceptance session"
    return None


def _snapshot_state(path: Path, root: Path) -> dict[str, Any]:
    result: dict[str, Any] = {"path": str(path.relative_to(root)), "state": "missing"}
    if not path.is_file():
        return result
    try:
        snapshot = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        result.update(state="invalid", error=str(exc))
        return result
    if not isinstance(snapshot, dict):
        result.update(state="invalid", error="snapshot root is not an object")
        return result
    for inventory_name in ("source_inventory", "outputs"):
        inventory = snapshot.get(inventory_name, {})
        if not isinstance(inventory, dict):
            result.update(state="invalid", error=f"{inventory_name} is not a mapping")
            return result
        for relative, expected in inventory.items():
            candidate = root / relative
            if (
                not isinstance(expected, dict)
                or not candidate.is_file()
                or expected.get("sha256") != _sha256(candidate)
                or expected.get("size_bytes") != candidate.stat().st_size
            ):
                result.update(state="stale", error=f"snapshot drift: {relative}")
                return result
    result["state"] = "current"
    result["sha256"] = _sha256(path)
    return result


def _session_state(
    contract: dict[str, Any], root: Path
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    row = contract.get("acceptance_session")
    if not isinstance(row, dict):
        raise FunctionalAcceptanceError("contract must define acceptance_session")
    session_relative = row.get("path")
    snapshot_relative = row.get("snapshot_manifest")
    statuses = row.get("accepted_statuses")
    if (
        not isinstance(session_relative, str)
        or not isinstance(snapshot_relative, str)
        or not isinstance(statuses, list)
        or not statuses
    ):
        raise FunctionalAcceptanceError("acceptance_session contract is invalid")
    snapshot_path = root / snapshot_relative
    snapshot_result = _snapshot_state(snapshot_path, root)
    result: dict[str, Any] = {
        "path": session_relative,
        "state": "missing",
        "status": None,
    }
    session_path = root / session_relative
    if not session_path.is_file():
        return result, {}, snapshot_result
    try:
        session = json.loads(session_path.read_text(encoding="utf-8"))
        snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        result.update(state="invalid", error=str(exc))
        return result, {}, snapshot_result
    if not isinstance(session, dict) or not isinstance(snapshot, dict):
        result.update(state="invalid", error="session or snapshot root is not an object")
        return result, {}, snapshot_result
    status = session.get("status")
    result["status"] = status
    outputs = snapshot.get("outputs", {})
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf", {})
    expected_identity = {
        "snapshot_manifest_sha256": _sha256(snapshot_path),
        "source_inventory_sha256": snapshot.get("source_inventory_sha256"),
        "expanded_urdf_sha256": urdf.get("sha256") if isinstance(urdf, dict) else None,
    }
    if snapshot_result["state"] != "current":
        result.update(state="stale", error="formal vehicle snapshot is not current")
    elif status not in statuses:
        result.update(state="invalid", error="session status is not accepted")
    elif session.get("snapshot") != expected_identity:
        result.update(state="stale", error="session is bound to a different source snapshot")
    elif not isinstance(session.get("evidence"), dict):
        result.update(state="invalid", error="session evidence is not a mapping")
    else:
        result["state"] = "valid"
    return result, session if result["state"] == "valid" else {}, snapshot_result


def audit(
    contract_path: Path = DEFAULT_CONTRACT,
    register_path: Path = DEFAULT_REGISTER,
    root: Path = ROOT,
) -> dict[str, Any]:
    contract = _mapping(contract_path)
    register = _mapping(register_path)
    registered = [str(row.get("id", "")) for row in register.get("functional_positions", [])]
    if not registered or any(not item for item in registered) or len(set(registered)) != len(registered):
        raise FunctionalAcceptanceError("component register has missing or duplicate function ids")

    position_contract = contract.get("functional_positions")
    gate_contract = contract.get("evidence_gates")
    if not isinstance(position_contract, dict) or not isinstance(gate_contract, dict):
        raise FunctionalAcceptanceError("contract must define evidence_gates and functional_positions")
    missing_positions = sorted(set(registered) - set(position_contract))
    extra_positions = sorted(set(position_contract) - set(registered))
    if missing_positions or extra_positions:
        raise FunctionalAcceptanceError(
            f"functional position crosswalk mismatch: missing={missing_positions}, extra={extra_positions}"
        )

    acceptance_session, session, snapshot_state = _session_state(contract, root)
    session_evidence = session.get("evidence", {}) if session else {}

    gate_results: dict[str, dict[str, Any]] = {}
    for gate_id, row in gate_contract.items():
        if not isinstance(row, dict):
            raise FunctionalAcceptanceError(f"evidence gate {gate_id} must be a mapping")
        relative = row.get("path")
        statuses = row.get("success_statuses")
        if not isinstance(relative, str) or not relative or not isinstance(statuses, list) or not statuses:
            raise FunctionalAcceptanceError(f"evidence gate {gate_id} has an invalid path/status contract")
        evidence_path = root / relative
        result: dict[str, Any] = {"path": relative, "state": "missing", "status": None}
        if evidence_path.is_file():
            try:
                evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
            except (OSError, UnicodeError, json.JSONDecodeError) as exc:
                result.update(state="invalid", error=str(exc))
            else:
                if not isinstance(evidence, dict):
                    result.update(state="invalid", error="JSON root is not an object")
                else:
                    status = evidence.get("status")
                    result["status"] = status
                    result["state"] = "passed" if status in statuses else "failed"
                    expected_report_id = row.get("report_id")
                    if (
                        result["state"] == "passed"
                        and expected_report_id is not None
                        and evidence.get("report_id") != expected_report_id
                    ):
                        result.update(
                            state="invalid",
                            error="evidence report_id does not match the gate contract",
                        )
                    required_values = row.get("required_values", {})
                    if not isinstance(required_values, dict):
                        result.update(
                            state="invalid",
                            error="evidence gate required_values is not a mapping",
                        )
                    elif result["state"] == "passed":
                        mismatches = [
                            field
                            for field, expected in required_values.items()
                            if not _strict_json_equal(
                                _nested_value(evidence, str(field)), expected
                            )
                        ]
                        if mismatches:
                            result.update(
                                state="failed",
                                error=(
                                    "evidence required values do not match: "
                                    + ", ".join(sorted(mismatches))
                                ),
                            )
                    required_mapping_keys = row.get("required_mapping_keys", {})
                    if not isinstance(required_mapping_keys, dict):
                        result.update(
                            state="invalid",
                            error="evidence gate required_mapping_keys is not a mapping",
                        )
                    elif result["state"] == "passed":
                        key_mismatches: list[str] = []
                        for field, expected_keys in required_mapping_keys.items():
                            actual = _nested_value(evidence, str(field))
                            if (
                                not isinstance(expected_keys, list)
                                or not isinstance(actual, dict)
                                or set(actual) != {str(key) for key in expected_keys}
                            ):
                                key_mismatches.append(str(field))
                        if key_mismatches:
                            result.update(
                                state="failed",
                                error=(
                                    "evidence mapping keys do not match: "
                                    + ", ".join(sorted(key_mismatches))
                                ),
                            )
                    required_mapping_item_values = row.get(
                        "required_mapping_item_values", {}
                    )
                    if not isinstance(required_mapping_item_values, dict):
                        result.update(
                            state="invalid",
                            error="required_mapping_item_values is not a mapping",
                        )
                    elif result["state"] == "passed":
                        item_mismatches: list[str] = []
                        for field, requirements in required_mapping_item_values.items():
                            actual = _nested_value(evidence, str(field))
                            if not isinstance(actual, dict) or not isinstance(requirements, dict):
                                item_mismatches.append(str(field))
                                continue
                            for item_name, item in actual.items():
                                if not isinstance(item, dict) or any(
                                    not _strict_json_equal(
                                        _nested_value(item, str(required_field)), expected
                                    )
                                    for required_field, expected in requirements.items()
                                ):
                                    item_mismatches.append(f"{field}.{item_name}")
                        if item_mismatches:
                            result.update(
                                state="failed",
                                error=(
                                    "evidence mapping item values do not match: "
                                    + ", ".join(sorted(item_mismatches))
                                ),
                            )
                    list_contracts = (
                        ("required_list_item_values", "equal"),
                        ("required_list_item_minimums", "minimum"),
                        ("required_list_item_maximums", "maximum"),
                    )
                    for contract_key, comparison in list_contracts:
                        requirements_by_list = row.get(contract_key, {})
                        if not isinstance(requirements_by_list, dict):
                            result.update(
                                state="invalid",
                                error=f"{contract_key} is not a mapping",
                            )
                            break
                        if result["state"] != "passed":
                            break
                        list_mismatches: list[str] = []
                        for field, requirements in requirements_by_list.items():
                            actual = _nested_value(evidence, str(field))
                            if not isinstance(actual, list) or not actual or not isinstance(requirements, dict):
                                list_mismatches.append(str(field))
                                continue
                            for index, item in enumerate(actual):
                                if not isinstance(item, dict):
                                    list_mismatches.append(f"{field}[{index}]")
                                    continue
                                for required_field, expected in requirements.items():
                                    value = _nested_value(item, str(required_field))
                                    if comparison == "equal":
                                        matched = _strict_json_equal(value, expected)
                                    else:
                                        matched = (
                                            _finite_contract_number(value)
                                            and _finite_contract_number(expected)
                                            and (
                                                value >= expected
                                                if comparison == "minimum"
                                                else value <= expected
                                            )
                                        )
                                    if not matched:
                                        list_mismatches.append(f"{field}[{index}]")
                                        break
                        if list_mismatches:
                            result.update(
                                state="failed",
                                error=(
                                    "evidence list item values do not match: "
                                    + ", ".join(sorted(set(list_mismatches)))
                                ),
                            )
                            break
                    current_file_hashes = row.get("current_file_hashes", {})
                    if not isinstance(current_file_hashes, dict):
                        result.update(
                            state="invalid",
                            error="evidence gate current_file_hashes is not a mapping",
                        )
                    elif result["state"] == "passed":
                        stale_fields: list[str] = []
                        root_resolved = root.resolve()
                        for evidence_field, relative_source in current_file_hashes.items():
                            if (
                                not isinstance(evidence_field, str)
                                or not evidence_field
                                or not isinstance(relative_source, str)
                                or not relative_source
                            ):
                                result.update(
                                    state="invalid",
                                    error="evidence gate current_file_hashes has an invalid entry",
                                )
                                break
                            source = root / relative_source
                            try:
                                source.resolve(strict=True).relative_to(root_resolved)
                            except (OSError, ValueError):
                                stale_fields.append(evidence_field)
                                continue
                            if (
                                source.is_symlink()
                                or not source.is_file()
                                or _nested_value(evidence, evidence_field) != _sha256(source)
                            ):
                                stale_fields.append(evidence_field)
                        if result["state"] == "passed" and stale_fields:
                            result.update(
                                state="stale",
                                error=(
                                    "evidence current file hashes do not match: "
                                    + ", ".join(sorted(stale_fields))
                                ),
                            )
                    snapshot_hash_field = row.get("snapshot_urdf_hash_field")
                    if result["state"] == "passed" and snapshot_hash_field is not None:
                        if not isinstance(snapshot_hash_field, str) or not snapshot_hash_field:
                            result.update(state="invalid", error="invalid snapshot URDF hash field")
                        else:
                            expected_hash = session.get("snapshot", {}).get("expanded_urdf_sha256")
                            evidence_hash = _nested_value(evidence, snapshot_hash_field)
                            if expected_hash is not None and evidence_hash != expected_hash:
                                result.update(
                                    state="stale",
                                    error="evidence expanded URDF hash does not match the frozen snapshot",
                                )
                    snapshot_source_hash_field = row.get("snapshot_source_hash_field")
                    if result["state"] == "passed" and snapshot_source_hash_field is not None:
                        if (
                            not isinstance(snapshot_source_hash_field, str)
                            or not snapshot_source_hash_field
                        ):
                            result.update(state="invalid", error="invalid snapshot source hash field")
                        else:
                            expected_hash = session.get("snapshot", {}).get(
                                "source_inventory_sha256"
                            )
                            evidence_hash = _nested_value(
                                evidence, snapshot_source_hash_field
                            )
                            if expected_hash is not None and evidence_hash != expected_hash:
                                result.update(
                                    state="stale",
                                    error=(
                                        "evidence source inventory hash does not match "
                                        "the frozen snapshot"
                                    ),
                                )
                    if row.get("session_bound") is True and result["state"] == "passed":
                        bound = session_evidence.get(str(gate_id))
                        if acceptance_session["state"] != "valid":
                            result.update(
                                state="unbound",
                                error="no valid final acceptance session",
                            )
                        elif not isinstance(bound, dict):
                            result.update(
                                state="unbound",
                                error="evidence was not produced in the current session",
                            )
                        elif (
                            bound.get("path") != relative
                            or bound.get("status") != status
                            or bound.get("sha256") != _sha256(evidence_path)
                        ):
                            result.update(
                                state="unbound",
                                error="current evidence does not match the session digest",
                            )
                    if result["state"] == "passed":
                        runtime_error = _runtime_binding_error(
                            evidence_path=evidence_path,
                            evidence=evidence,
                            gate_contract=row,
                            session=session,
                        )
                        if runtime_error is not None:
                            result.update(state="unbound", error=runtime_error)
        gate_results[str(gate_id)] = result

    if snapshot_state["state"] != "current":
        component = gate_results.get("component_register")
        if component and component["state"] == "passed":
            component.update(
                state="stale",
                error="component register is not bound to a current vehicle snapshot",
            )

    positions: dict[str, dict[str, Any]] = {}
    for position_id in registered:
        required = position_contract[position_id]
        if not isinstance(required, list) or not required:
            raise FunctionalAcceptanceError(f"position {position_id} has no evidence gates")
        unknown = sorted(set(required) - set(gate_contract))
        if unknown:
            raise FunctionalAcceptanceError(f"position {position_id} references unknown gates {unknown}")
        unresolved = [gate for gate in required if gate_results[gate]["state"] != "passed"]
        positions[position_id] = {
            "state": "passed" if not unresolved else "pending",
            "required_gates": required,
            "unresolved_gates": unresolved,
        }

    mission_gates = contract.get("mission_level_gates", [])
    if not isinstance(mission_gates, list) or set(mission_gates) - set(gate_contract):
        raise FunctionalAcceptanceError("mission_level_gates contains an unknown evidence gate")
    unresolved_mission = [gate for gate in mission_gates if gate_results[gate]["state"] != "passed"]
    passed_positions = [name for name, row in positions.items() if row["state"] == "passed"]
    # RUNNING and PENDING are structurally valid states: their bound evidence
    # remains useful for an in-progress audit.  They cannot, however, seal the
    # formal acceptance result even when every individual gate has passed.
    acceptance_session_complete = (
        acceptance_session["state"] == "valid"
        and acceptance_session["status"]
        == "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE"
    )
    complete = (
        len(passed_positions) == len(positions)
        and not unresolved_mission
        and acceptance_session_complete
    )
    return {
        "report_id": "tzcup_formal_functional_acceptance_audit_v1",
        "status": (
            "FORMAL_ALL_FUNCTION_POSITIONS_AND_MISSION_GATES_PASSED"
            if complete
            else "FORMAL_FUNCTIONAL_ACCEPTANCE_PENDING"
        ),
        "complete": complete,
        "acceptance_session_complete": acceptance_session_complete,
        "registered_position_count": len(registered),
        "passed_position_count": len(passed_positions),
        "pending_position_count": len(positions) - len(passed_positions),
        "gate_results": gate_results,
        "acceptance_session": acceptance_session,
        "snapshot_state": snapshot_state,
        "positions": positions,
        "unresolved_mission_gates": unresolved_mission,
        "claim_boundary": contract.get("claim_boundary"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--register", type=Path, default=DEFAULT_REGISTER)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--require-all", action="store_true")
    args = parser.parse_args()
    try:
        result = audit(args.contract, args.register)
    except FunctionalAcceptanceError as exc:
        print(json.dumps({"status": "INVALID", "error": str(exc)}, indent=2))
        return 2
    text = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 3 if args.require_all and not result["complete"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
