#!/usr/bin/env python3
"""Fail-closed evaluator for the fixed TZcup simulation product contract.

The evaluator never runs a simulation and never invents measurements. It only
accepts a complete, hash-bound evidence manifest and produces the three final
acceptance files required by the product specification. Missing, malformed or
untraceable evidence is a hard failure.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import glob
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config" / "product_acceptance_v1.json"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
FINAL_OUTPUTS = (
    "FINAL_ACCEPTANCE_STATUS.json",
    "FINAL_ACCEPTANCE_MATRIX.json",
    "FINAL_EVIDENCE_INDEX.md",
)
PROVENANCE_FIELDS = (
    "dataset",
    "source_commit",
    "model_sha256",
    "config_sha256",
    "dataset_sha256",
    "container_digest",
    "dependency_lock_sha256",
    "seeds",
    "command",
    "exit_code",
    "evidence_path",
    "human_report_path",
    "raw_log_paths",
    "artifact_sha256",
)


class ContractError(ValueError):
    """The frozen contract is structurally invalid or has drifted."""


def utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace(
        "+00:00", "Z"
    )


def read_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ContractError(f"cannot read JSON {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ContractError(f"{path} must contain a JSON object")
    return payload


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _require_unique(values: list[str], label: str) -> None:
    duplicates = sorted({value for value in values if values.count(value) > 1})
    if duplicates:
        raise ContractError(f"duplicate {label}: {', '.join(duplicates)}")


def validate_contract(contract_path: Path) -> dict[str, Any]:
    contract_path = contract_path.resolve()
    contract = read_json(contract_path)
    if contract.get("schema_version") != 1:
        raise ContractError("unsupported acceptance contract schema")
    if contract.get("immutable") is not True:
        raise ContractError("acceptance contract must be immutable")
    contract_id = contract.get("contract_id")
    if not isinstance(contract_id, str) or not contract_id:
        raise ContractError("contract_id is required")

    required_gate_ids = contract.get("required_gate_ids")
    if required_gate_ids != list("ABCDEFGHIJKLMNOP"):
        raise ContractError("required_gate_ids must be exactly A through P")
    gates = contract.get("gates")
    if not isinstance(gates, list):
        raise ContractError("gates must be a list")
    gate_ids = [gate.get("id") for gate in gates if isinstance(gate, dict)]
    if gate_ids != required_gate_ids:
        raise ContractError("gate definitions must be ordered exactly A through P")
    _require_unique(gate_ids, "gate ids")

    check_ids: list[str] = []
    metrics: list[str] = []
    for gate in gates:
        checks = gate.get("checks")
        if not isinstance(checks, list) or not checks:
            raise ContractError(f"gate {gate['id']} has no checks")
        for check in checks:
            if not isinstance(check, dict):
                raise ContractError(f"gate {gate['id']} contains a non-object check")
            required = {"id", "metric", "op", "threshold", "unit", "requirement"}
            missing = sorted(required - set(check))
            if missing:
                raise ContractError(
                    f"check in gate {gate['id']} misses {', '.join(missing)}"
                )
            if check["op"] not in {"eq", "gte", "lte"}:
                raise ContractError(f"unsupported comparator in {check['id']}")
            check_ids.append(check["id"])
            metrics.append(check["metric"])
    _require_unique(check_ids, "check ids")
    _require_unique(metrics, "gate metric names")

    vetoes = contract.get("global_vetoes")
    if not isinstance(vetoes, list) or not vetoes:
        raise ContractError("global_vetoes must be a non-empty list")
    veto_ids = []
    veto_metrics = []
    for veto in vetoes:
        if not isinstance(veto, dict):
            raise ContractError("global veto must be an object")
        required = {"id", "metric", "op", "threshold", "unit", "requirement"}
        missing = sorted(required - set(veto))
        if missing:
            raise ContractError(f"global veto misses {', '.join(missing)}")
        veto_ids.append(veto["id"])
        veto_metrics.append(veto["metric"])
    _require_unique(veto_ids, "veto ids")
    _require_unique(veto_metrics, "veto metric names")

    source_path = (ROOT / contract.get("source_document", "")).resolve()
    try:
        source_path.relative_to(ROOT.resolve())
    except ValueError as exc:
        raise ContractError("source_document escapes the repository") from exc
    if not source_path.is_file():
        raise ContractError(f"source acceptance document missing: {source_path}")
    expected_source_sha = contract.get("source_document_sha256")
    actual_source_sha = file_sha256(source_path)
    if expected_source_sha != actual_source_sha:
        raise ContractError(
            "acceptance source document hash mismatch: threshold drift is blocked"
        )

    required_artifacts = contract.get("required_final_artifacts")
    if not isinstance(required_artifacts, list) or not all(
        isinstance(value, str) and value for value in required_artifacts
    ):
        raise ContractError("required_final_artifacts must be non-empty paths")
    _require_unique(required_artifacts, "required final artifacts")
    return contract


def _safe_relative_file(root: Path, raw_path: Any) -> tuple[Path | None, str | None]:
    if not isinstance(raw_path, str) or not raw_path.strip():
        return None, "path is missing"
    relative = Path(raw_path)
    if relative.is_absolute():
        return None, "absolute paths are forbidden"
    resolved_root = root.resolve()
    resolved = (resolved_root / relative).resolve()
    try:
        resolved.relative_to(resolved_root)
    except ValueError:
        return None, "path escapes evidence root"
    if not resolved.is_file():
        return None, "file does not exist"
    return resolved, None


def _compare(measured: Any, op: str, threshold: Any) -> tuple[bool, str]:
    if op == "eq":
        if type(measured) is not type(threshold):
            return False, "measured value has the wrong type"
        passed = measured == threshold
    else:
        if isinstance(measured, bool) or not isinstance(measured, (int, float)):
            return False, "measured value must be numeric"
        if not math.isfinite(float(measured)):
            return False, "measured value must be finite"
        if isinstance(threshold, bool) or not isinstance(threshold, (int, float)):
            return False, "contract threshold must be numeric"
        passed = measured >= threshold if op == "gte" else measured <= threshold
    return passed, "threshold met" if passed else f"requires {op} {threshold!r}"


def _valid_sha(value: Any) -> bool:
    return isinstance(value, str) and SHA256_RE.fullmatch(value) is not None


def _verify_hash_bound_file(
    evidence_root: Path,
    raw_path: Any,
    artifact_hashes: Any,
) -> list[str]:
    errors: list[str] = []
    path, path_error = _safe_relative_file(evidence_root, raw_path)
    if path_error:
        return [f"{raw_path!r}: {path_error}"]
    if not isinstance(artifact_hashes, dict):
        return ["artifact_sha256 must be an object"]
    expected = artifact_hashes.get(raw_path)
    if not _valid_sha(expected):
        return [f"{raw_path}: missing lowercase SHA-256"]
    actual = file_sha256(path)
    if expected != actual:
        errors.append(f"{raw_path}: SHA-256 mismatch")
    return errors


def _validate_gate_provenance(
    gate_id: str,
    evidence: Any,
    evidence_root: Path,
) -> list[str]:
    if not isinstance(evidence, dict):
        return [f"gate {gate_id}: evidence record is missing"]
    errors = [
        f"gate {gate_id}: missing provenance field {field}"
        for field in PROVENANCE_FIELDS
        if field not in evidence
    ]
    if errors:
        return errors
    if not isinstance(evidence["dataset"], str) or not evidence["dataset"].strip():
        errors.append(f"gate {gate_id}: dataset is empty")
    if not isinstance(evidence["source_commit"], str) or not COMMIT_RE.fullmatch(
        evidence["source_commit"]
    ):
        errors.append(f"gate {gate_id}: source_commit must be 40 lowercase hex")
    for field in (
        "model_sha256",
        "config_sha256",
        "dataset_sha256",
        "dependency_lock_sha256",
    ):
        if not _valid_sha(evidence[field]):
            errors.append(f"gate {gate_id}: {field} must be lowercase SHA-256")
    digest = evidence["container_digest"]
    if not isinstance(digest, str) or not re.fullmatch(r"sha256:[0-9a-f]{64}", digest):
        errors.append(f"gate {gate_id}: container_digest must be sha256:<64 hex>")
    seeds = evidence["seeds"]
    if not isinstance(seeds, list) or not seeds or len(set(map(str, seeds))) != len(seeds):
        errors.append(f"gate {gate_id}: seeds must be a non-empty unique list")
    if not isinstance(evidence["command"], str) or not evidence["command"].strip():
        errors.append(f"gate {gate_id}: command is empty")
    if type(evidence["exit_code"]) is not int or evidence["exit_code"] != 0:
        errors.append(f"gate {gate_id}: exit_code must be integer zero")

    artifact_hashes = evidence["artifact_sha256"]
    referenced = [evidence["evidence_path"], evidence["human_report_path"]]
    raw_logs = evidence["raw_log_paths"]
    if not isinstance(raw_logs, list) or not raw_logs:
        errors.append(f"gate {gate_id}: at least one raw log is required")
        raw_logs = []
    referenced.extend(raw_logs)
    for raw_path in referenced:
        errors.extend(
            f"gate {gate_id}: {message}"
            for message in _verify_hash_bound_file(
                evidence_root, raw_path, artifact_hashes
            )
        )
    return errors


def _matrix_row(
    check: dict[str, Any],
    measured: Any,
    gate_evidence: dict[str, Any] | None,
    status: str,
    reason: str,
) -> dict[str, Any]:
    evidence = gate_evidence or {}
    return {
        "gate_id": check["id"],
        "requirement": check["requirement"],
        "metric": check["metric"],
        "operator": check["op"],
        "threshold": check["threshold"],
        "measured_value": measured,
        "unit": check["unit"],
        "dataset": evidence.get("dataset"),
        "source_commit": evidence.get("source_commit"),
        "model_sha": evidence.get("model_sha256"),
        "config_sha": evidence.get("config_sha256"),
        "evidence_path": evidence.get("evidence_path"),
        "status": status,
        "reason": reason,
    }


def _validate_final_artifacts(
    contract: dict[str, Any], evidence: dict[str, Any], evidence_root: Path
) -> list[str]:
    hashes = evidence.get("final_artifact_sha256")
    if not isinstance(hashes, dict):
        return ["final_artifact_sha256 must be an object"]
    errors: list[str] = []
    for relative in contract["required_final_artifacts"]:
        errors.extend(
            _verify_hash_bound_file(evidence_root, relative, hashes)
        )
    release_pattern = contract["required_release_zip_glob"]
    absolute_pattern = str(evidence_root.resolve() / release_pattern)
    release_files = [Path(path) for path in glob.glob(absolute_pattern)]
    if len(release_files) != 1:
        errors.append(
            f"required exactly one release ZIP matching {release_pattern}, "
            f"found {len(release_files)}"
        )
    else:
        relative = release_files[0].resolve().relative_to(evidence_root.resolve()).as_posix()
        errors.extend(_verify_hash_bound_file(evidence_root, relative, hashes))
    return errors


def evaluate(
    contract_path: Path,
    evidence_manifest_path: Path,
    evidence_root: Path,
) -> dict[str, Any]:
    contract = validate_contract(contract_path)
    evidence_root = evidence_root.resolve()
    if not evidence_root.is_dir():
        raise ContractError(f"evidence root does not exist: {evidence_root}")
    evidence = read_json(evidence_manifest_path.resolve())
    errors: list[str] = []
    if evidence.get("schema_version") != 1:
        errors.append("evidence schema_version must be 1")
    if evidence.get("contract_id") != contract["contract_id"]:
        errors.append("evidence contract_id does not match")
    contract_sha = file_sha256(contract_path.resolve())
    if evidence.get("contract_sha256") != contract_sha:
        errors.append("evidence contract_sha256 does not match the frozen contract")

    gate_evidence = evidence.get("gates")
    if not isinstance(gate_evidence, dict):
        gate_evidence = {}
        errors.append("evidence gates must be an object")
    expected_gate_ids = set(contract["required_gate_ids"])
    actual_gate_ids = set(gate_evidence)
    if actual_gate_ids != expected_gate_ids:
        missing = sorted(expected_gate_ids - actual_gate_ids)
        extra = sorted(actual_gate_ids - expected_gate_ids)
        if missing:
            errors.append(f"missing gate evidence: {', '.join(missing)}")
        if extra:
            errors.append(f"unexpected gate evidence: {', '.join(extra)}")

    rows: list[dict[str, Any]] = []
    gate_status: dict[str, str] = {}
    gate_errors: dict[str, list[str]] = {}
    for gate in contract["gates"]:
        gate_id = gate["id"]
        record = gate_evidence.get(gate_id)
        provenance_errors = _validate_gate_provenance(
            gate_id, record, evidence_root
        )
        metrics = record.get("metrics", {}) if isinstance(record, dict) else {}
        if not isinstance(metrics, dict):
            metrics = {}
            provenance_errors.append(f"gate {gate_id}: metrics must be an object")
        check_failed = False
        for check in gate["checks"]:
            if check["metric"] not in metrics:
                rows.append(
                    _matrix_row(
                        check,
                        None,
                        record,
                        "FAIL",
                        "required metric is missing",
                    )
                )
                check_failed = True
                continue
            measured = metrics[check["metric"]]
            passed, reason = _compare(measured, check["op"], check["threshold"])
            rows.append(
                _matrix_row(
                    check,
                    measured,
                    record,
                    "PASS" if passed else "FAIL",
                    reason,
                )
            )
            check_failed |= not passed
        if provenance_errors:
            gate_errors[gate_id] = provenance_errors
        gate_status[gate_id] = (
            "PASS" if not provenance_errors and not check_failed else "FAIL"
        )

    global_metrics = evidence.get("global_metrics")
    if not isinstance(global_metrics, dict):
        global_metrics = {}
        errors.append("global_metrics must be an object")
    veto_status = "PASS"
    for veto in contract["global_vetoes"]:
        if veto["metric"] not in global_metrics:
            rows.append(
                _matrix_row(veto, None, None, "FAIL", "required veto metric is missing")
            )
            veto_status = "FAIL"
            continue
        measured = global_metrics[veto["metric"]]
        passed, reason = _compare(measured, veto["op"], veto["threshold"])
        rows.append(
            _matrix_row(
                veto, measured, None, "PASS" if passed else "FAIL", reason
            )
        )
        if not passed:
            veto_status = "FAIL"

    final_artifact_errors = _validate_final_artifacts(
        contract, evidence, evidence_root
    )
    all_gate_pass = all(value == "PASS" for value in gate_status.values())
    complete = (
        not errors
        and not gate_errors
        and not final_artifact_errors
        and veto_status == "PASS"
        and all_gate_pass
    )
    return {
        "contract": contract,
        "contract_sha256": contract_sha,
        "evidence": evidence,
        "rows": rows,
        "gate_status": gate_status,
        "gate_errors": gate_errors,
        "global_veto_status": veto_status,
        "final_artifact_errors": final_artifact_errors,
        "errors": errors,
        "complete": complete,
    }


def _metric(evaluation: dict[str, Any], name: str) -> Any:
    for row in evaluation["rows"]:
        if row["metric"] == name:
            return row["measured_value"]
    return None


def build_status(evaluation: dict[str, Any]) -> dict[str, Any]:
    gate = evaluation["gate_status"]
    complete = evaluation["complete"]
    x86_ready = all(gate[key] == "PASS" for key in ("E", "F", "G", "K", "O"))
    return {
        "schema_version": 1,
        "contract_id": evaluation["contract"]["contract_id"],
        "contract_sha256": evaluation["contract_sha256"],
        "evaluated_at_utc": utc_now(),
        "SIMULATION_PRODUCT_COMPLETE": complete,
        "PRODUCT_X86_PERCEPTION_READY": x86_ready,
        "PRODUCT_INTEGRATION_READY": False,
        "PRODUCT_FIELD_READY": False,
        "GT_CONTROL_VIOLATION": _metric(evaluation, "gt_control_violation"),
        "VEHICLE_SIMULATION_FIDELITY_PASS": gate["A"] == "PASS",
        "LOCALIZATION_PASS": all(
            row["status"] == "PASS"
            for row in evaluation["rows"]
            if row["gate_id"] in {"B-01", "B-02"}
        ),
        "MAPPING_20000M2_PASS": all(
            row["status"] == "PASS"
            for row in evaluation["rows"]
            if row["gate_id"].startswith("B-") and row["gate_id"] not in {"B-01", "B-02"}
        ),
        "COVERAGE_PASS": all(
            row["status"] == "PASS"
            for row in evaluation["rows"]
            if row["gate_id"] in {"C-01", "C-02"}
        ),
        "EFFICIENCY_3500M2H_PASS": all(
            row["status"] == "PASS"
            for row in evaluation["rows"]
            if row["gate_id"] in {"C-03", "C-04"}
        ),
        "NAVIGATION_PASS": gate["D"] == "PASS",
        "DYNAMIC_OBSTACLE_PASS": gate["D"] == "PASS",
        "ESTOP_PASS": all(
            row["status"] == "PASS"
            for row in evaluation["rows"]
            if row["gate_id"] in {"D-06", "D-07", "D-08", "D-09"}
        ),
        "DISCRETE_PERCEPTION_PASS": gate["E"] == "PASS",
        "AREA_PERCEPTION_PASS": gate["F"] == "PASS",
        "TRACKING_PASS": gate["G"] == "PASS",
        "DYNAMIC_TRASH_MAP_PASS": gate["G"] == "PASS",
        "SPOT_CLEAN_PRODUCT_PASS": gate["H"] == "PASS",
        "POST_CLEAN_VERIFICATION_PASS": gate["I"] == "PASS",
        "MULTIMODAL_PASS": gate["J"] == "PASS",
        "LLM_TASK_DECOMPOSITION_PASS": gate["J"] == "PASS",
        "PERFORMANCE_PASS": gate["K"] == "PASS",
        "SOAK_2H_PASS": gate["L"] == "PASS",
        "FAULT_INJECTION_PASS": gate["M"] == "PASS",
        "MCAP_REPLAY_PASS": gate["N"] == "PASS",
        "MODEL_FREEZE_CREATED": gate["O"] == "PASS",
        "SEALED_FINAL_PASS": gate["O"] == "PASS",
        "RELEASE_BUNDLE_PASS": gate["O"] == "PASS",
        "LICENSE_AUDIT_PASS": gate["O"] == "PASS",
        "CI_GREEN": gate["O"] == "PASS",
        "COMPETITION_MAPPING_PASS": gate["P"] == "PASS",
        "GLOBAL_VETOES_PASS": evaluation["global_veto_status"] == "PASS",
        "gate_status": gate,
        "blocking_reasons": {
            "contract_or_manifest": evaluation["errors"],
            "gate_provenance": evaluation["gate_errors"],
            "final_artifacts": evaluation["final_artifact_errors"],
            "failed_checks": [
                row["gate_id"]
                for row in evaluation["rows"]
                if row["status"] != "PASS"
            ],
        },
    }


def _atomic_write(path: Path, payload: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def _json_text(payload: Any) -> str:
    return json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=False) + "\n"


def build_index(evaluation: dict[str, Any], status: dict[str, Any]) -> str:
    lines = [
        "# FINAL_EVIDENCE_INDEX",
        "",
        f"Contract: `{evaluation['contract']['contract_id']}`",
        f"Contract SHA-256: `{evaluation['contract_sha256']}`",
        f"Simulation product complete: `{str(status['SIMULATION_PRODUCT_COMPLETE']).lower()}`",
        "",
        "| Gate | Name | Status | Evidence |",
        "|---|---|---|---|",
    ]
    gates = {gate["id"]: gate for gate in evaluation["contract"]["gates"]}
    records = evaluation["evidence"].get("gates", {})
    for gate_id in evaluation["contract"]["required_gate_ids"]:
        record = records.get(gate_id, {}) if isinstance(records, dict) else {}
        lines.append(
            f"| {gate_id} | {gates[gate_id]['name']} | "
            f"{evaluation['gate_status'][gate_id]} | "
            f"`{record.get('evidence_path', 'missing')}` |"
        )
    lines.extend(["", "## Blocking reasons", ""])
    blockers: list[str] = []
    blockers.extend(evaluation["errors"])
    for gate_id, gate_messages in evaluation["gate_errors"].items():
        blockers.extend(f"{gate_id}: {message}" for message in gate_messages)
    blockers.extend(evaluation["final_artifact_errors"])
    blockers.extend(
        f"{row['gate_id']}: {row['metric']} ({row['reason']})"
        for row in evaluation["rows"]
        if row["status"] != "PASS"
    )
    if blockers:
        lines.extend(f"- {blocker}" for blocker in blockers)
    else:
        lines.append("- None")
    lines.extend(
        [
            "",
            "`PRODUCT_INTEGRATION_READY` and `PRODUCT_FIELD_READY` remain false; "
            "they require target-compute and physical-field acceptance beyond this contract.",
            "",
        ]
    )
    return "\n".join(lines)


def write_outputs(
    evaluation: dict[str, Any], output_dir: Path, replace: bool
) -> dict[str, Path]:
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = {name: output_dir / name for name in FINAL_OUTPUTS}
    existing = [path for path in paths.values() if path.exists()]
    if existing and not replace:
        raise ContractError(
            "refusing to overwrite final acceptance output: "
            + ", ".join(str(path) for path in existing)
        )
    status = build_status(evaluation)
    matrix = {
        "schema_version": 1,
        "contract_id": evaluation["contract"]["contract_id"],
        "contract_sha256": evaluation["contract_sha256"],
        "SIMULATION_PRODUCT_COMPLETE": evaluation["complete"],
        "gate_status": evaluation["gate_status"],
        "checks": evaluation["rows"],
    }
    _atomic_write(paths["FINAL_ACCEPTANCE_STATUS.json"], _json_text(status))
    _atomic_write(paths["FINAL_ACCEPTANCE_MATRIX.json"], _json_text(matrix))
    _atomic_write(paths["FINAL_EVIDENCE_INDEX.md"], build_index(evaluation, status))
    return paths


def build_template(contract_path: Path) -> dict[str, Any]:
    contract = validate_contract(contract_path)
    empty_sha = "0" * 64
    gates: dict[str, Any] = {}
    for gate in contract["gates"]:
        gates[gate["id"]] = {
            "metrics": {check["metric"]: None for check in gate["checks"]},
            "dataset": "",
            "source_commit": "0" * 40,
            "model_sha256": empty_sha,
            "config_sha256": empty_sha,
            "dataset_sha256": empty_sha,
            "container_digest": f"sha256:{empty_sha}",
            "dependency_lock_sha256": empty_sha,
            "seeds": [],
            "command": "",
            "exit_code": None,
            "evidence_path": f"gates/{gate['id']}/report.json",
            "human_report_path": f"gates/{gate['id']}/report.md",
            "raw_log_paths": [f"gates/{gate['id']}/run.log"],
            "artifact_sha256": {},
        }
    return {
        "schema_version": 1,
        "contract_id": contract["contract_id"],
        "contract_sha256": file_sha256(contract_path.resolve()),
        "global_metrics": {
            veto["metric"]: None for veto in contract["global_vetoes"]
        },
        "gates": gates,
        "final_artifact_sha256": {},
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("validate-contract")
    template = subparsers.add_parser("template")
    template.add_argument("--output", type=Path, required=True)
    evaluate_parser = subparsers.add_parser("evaluate")
    evaluate_parser.add_argument("--evidence-manifest", type=Path, required=True)
    evaluate_parser.add_argument("--evidence-root", type=Path, required=True)
    evaluate_parser.add_argument("--output-dir", type=Path, required=True)
    evaluate_parser.add_argument("--replace", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        contract = validate_contract(args.contract)
        if args.command == "validate-contract":
            print(
                _json_text(
                    {
                        "valid": True,
                        "contract_id": contract["contract_id"],
                        "contract_sha256": file_sha256(args.contract.resolve()),
                        "gate_count": len(contract["gates"]),
                        "check_count": sum(
                            len(gate["checks"]) for gate in contract["gates"]
                        ),
                    }
                ),
                end="",
            )
            return 0
        if args.command == "template":
            if args.output.exists():
                raise ContractError(f"refusing to overwrite template: {args.output}")
            _atomic_write(args.output.resolve(), _json_text(build_template(args.contract)))
            print(args.output.resolve())
            return 0
        evaluation = evaluate(
            args.contract,
            args.evidence_manifest,
            args.evidence_root,
        )
        outputs = write_outputs(evaluation, args.output_dir, args.replace)
        print(
            _json_text(
                {
                    "SIMULATION_PRODUCT_COMPLETE": evaluation["complete"],
                    "outputs": {key: str(value) for key, value in outputs.items()},
                }
            ),
            end="",
        )
        return 0 if evaluation["complete"] else 2
    except ContractError as exc:
        print(
            _json_text(
                {
                    "SIMULATION_PRODUCT_COMPLETE": False,
                    "acceptance_evaluation_blocked": True,
                    "reason": str(exc),
                }
            ),
            file=sys.stderr,
            end="",
        )
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
