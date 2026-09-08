#!/usr/bin/env python3
"""Validate a retained real single-frame DOSOD preprocessing oracle receipt."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from hbm_evidence_common import MEMORY_WATCHDOG_THRESHOLDS, load_object, normal_file, path_under, sha256_file
from dosod_hbm_abi_contract import validate_pre_onnx_outputs
from run_dosod_hbm_x86_parity import _dump_filename, _validate_model_info, raw_parity_metrics
from execute_dosod_nonformal_oracle_candidate_compile import validate_candidate_receipt

ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "config" / "dosod_single_frame_preprocessing_oracle_contract.json"
CAPTURE_PRODUCER = ROOT / "scripts" / "capture_dosod_official_preprocess.py"
RECEIPT_ID = "tzcup_dosod_single_frame_preprocessing_oracle_receipt_v1"
STATUS = "ORACLE_VERIFIED"
CANDIDATE_ID = "tzcup_dosod_nonformal_oracle_candidate_compile_receipt_v1"
SUPERVISION_ID = "tzcup_dosod_single_frame_preprocessing_oracle_supervision_receipt_v1"
SUPERVISOR = ROOT / "scripts" / "run_dosod_single_frame_preprocessing_oracle_supervised.py"


def _digest(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value.lower())


def _bound(root: Path, item: Any, label: str) -> Path:
    if not isinstance(item, dict) or set(item) != {"relative_path", "sha256", "byte_size"} or not _digest(item["sha256"]) or not isinstance(item["byte_size"], int) or item["byte_size"] <= 0:
        raise ValueError(f"{label}_binding_invalid")
    path = path_under(root, item["relative_path"], label)
    if path.stat().st_size != item["byte_size"] or sha256_file(path) != item["sha256"]:
        raise ValueError(f"{label}_binding_drift")
    return path


def _candidate(path: Path, raw_sha: str) -> dict[str, Any]:
    value = validate_candidate_receipt(path, expected_raw_sha256=raw_sha)
    hbm = value.get("candidate_hbm")
    if value.get("receipt_id") != CANDIDATE_ID or value.get("status") != "NON_FORMAL_ORACLE_CANDIDATE_COMPILED" or value.get("formal_compile") is not False or value.get("board_acceptance") is not False:
        raise ValueError("oracle_candidate_receipt_invalid")
    if value.get("pilot_record_count") != 25 or not _digest(value.get("pilot_records_sha256")) or not _digest(value.get("candidate_calibration_records_sha256")) or not isinstance(value.get("candidate_route"), str):
        raise ValueError("oracle_candidate_full_pilot_binding_invalid")
    if not isinstance(hbm, dict) or not isinstance(hbm.get("path"), str) or not _digest(hbm.get("sha256")) or not isinstance(hbm.get("byte_size"), int):
        raise ValueError("oracle_candidate_hbm_invalid")
    return value


def _capture(path: Path) -> dict[str, Any]:
    normal_file(path, "official_preprocess_capture_receipt")
    value = load_object(path)
    # A hand-written capture JSON has no trust root.  This stays blocked until
    # the separately approved official-capture producer exists.
    normal_file(CAPTURE_PRODUCER, "official_capture_producer")
    if value.get("producer_script_path") != str(CAPTURE_PRODUCER.resolve()) or value.get("producer_script_sha256") != sha256_file(CAPTURE_PRODUCER):
        raise ValueError("official_capture_producer_identity_invalid")
    required = {"receipt_id", "status", "test_fixture", "raw_sensor", "official_preprocessor", "execution", "inputs", "producer_script_path", "producer_script_sha256", "started_epoch_ns", "ended_epoch_ns", "blockers"}
    if set(value) != required or value["receipt_id"] != "tzcup_dosod_official_preprocess_capture_receipt_v1" or value["status"] != "OFFICIAL_PREPROCESS_CAPTURED" or value["test_fixture"] is not False:
        raise ValueError("official_capture_receipt_invalid")
    raw = value["raw_sensor"]
    required_raw = {"path", "sha256", "byte_size", "width", "height", "step", "encoding", "frame_id", "stamp_ns"}
    if not isinstance(raw, dict) or set(raw) != required_raw or not isinstance(raw.get("path"), str) or not _digest(raw.get("sha256")) or not isinstance(raw.get("byte_size"), int):
        raise ValueError("official_capture_raw_invalid")
    raw_path = Path(raw["path"]); normal_file(raw_path, "official_capture_raw")
    if raw_path.stat().st_size != raw["byte_size"] or sha256_file(raw_path) != raw["sha256"] or raw["encoding"] not in {"rgb8", "bgr8"} or any(not isinstance(raw[key], int) or raw[key] <= 0 for key in ("width", "height", "step", "stamp_ns")) or not isinstance(raw["frame_id"], str) or not raw["frame_id"]:
        raise ValueError("official_capture_raw_drift")
    execution = value["execution"]
    execution_keys = {"pgid", "deadline_seconds", "term_grace_seconds", "timed_out", "term_sent", "kill_sent", "zero_survivor", "elapsed_seconds"}
    if not isinstance(execution, dict) or set(execution) != execution_keys or not isinstance(execution["pgid"], int) or execution["pgid"] <= 1 or not isinstance(execution["deadline_seconds"], (int, float)) or execution["deadline_seconds"] <= 0 or execution["term_grace_seconds"] != 10 or not isinstance(execution["timed_out"], bool) or not isinstance(execution["term_sent"], bool) or not isinstance(execution["kill_sent"], bool) or execution["zero_survivor"] is not True or not isinstance(execution["elapsed_seconds"], (int, float)) or execution["elapsed_seconds"] < 0:
        raise ValueError("official_capture_execution_invalid")
    official = value["official_preprocessor"]
    if not isinstance(official, dict) or set(official) != {"binary", "source", "dpkg", "stdout", "stderr", "command", "returncode", "identity", "execution_host", "zero_survivor"}:
        raise ValueError("official_capture_identity_invalid")
    for label in ("binary", "source", "dpkg", "stdout", "stderr"):
        item = official[label]
        if not isinstance(item, dict) or not isinstance(item.get("path"), str) or not _digest(item.get("sha256")):
            raise ValueError(f"official_capture_{label}_identity_invalid")
        item_path = Path(item["path"]); normal_file(item_path, f"official_capture_{label}")
        if item_path.stat().st_size != item.get("byte_size") or sha256_file(item_path) != item["sha256"]:
            raise ValueError(f"official_capture_{label}_identity_drift")
    if official["returncode"] != 0 or official["zero_survivor"] is not True or not isinstance(official["command"], list) or not official["command"] or official["command"][0] != official["binary"]["path"] or not isinstance(official["execution_host"], dict) or not official["execution_host"].get("system") or not official["execution_host"].get("machine"):
        raise ValueError("official_capture_command_invalid")
    identity = official["identity"]
    if not isinstance(identity, dict) or set(identity) != {"package", "version", "path_role", "source_revision", "dpkg_returncode"} or identity.get("dpkg_returncode") != 0:
        raise ValueError("official_capture_dpkg_identity_invalid")
    files = value["inputs"]
    expected = [("images_y", 409600), ("images_uv", 204800)]
    if not isinstance(files, list) or [(row.get("role"), row.get("byte_size")) for row in files if isinstance(row, dict)] != expected:
        raise ValueError("official_capture_planes_invalid")
    for row in files:
        candidate = Path(str(row.get("path", ""))); normal_file(candidate, f"official_capture_{row['role']}")
        if not _digest(row.get("sha256")) or candidate.stat().st_size != row["byte_size"] or sha256_file(candidate) != row["sha256"]:
            raise ValueError("official_capture_plane_drift")
    return value


def _validate_raw(path: Path, *, contract_path: Path = CONTRACT, outer_supervised: bool = False) -> dict[str, Any]:
    normal_file(path, "oracle_receipt"); normal_file(contract_path, "oracle_contract")
    receipt, contract = load_object(path), load_object(contract_path)
    try:
        validate_pre_onnx_outputs(contract.get("onnx_outputs"))
    except ValueError:
        raise ValueError("oracle_contract_onnx_output_abi_invalid") from None
    if receipt.get("receipt_id") != RECEIPT_ID or receipt.get("status") != STATUS or receipt.get("formal_compile") is not False or receipt.get("board_acceptance") is not False or receipt.get("test_fixture") is not False or receipt.get("blockers") != [] or receipt.get("receipt_path") != str(path.resolve()):
        raise ValueError("oracle_receipt_not_verified")
    producer = ROOT / "scripts" / "collect_dosod_single_frame_preprocessing_oracle.py"
    if receipt.get("producer_script_path") != str(producer.resolve()) or receipt.get("producer_script_sha256") != sha256_file(producer):
        raise ValueError("oracle_producer_identity_mismatch")
    if receipt.get("contract_sha256") != sha256_file(contract_path) or receipt.get("model_sha256") != contract.get("model_sha256") or receipt.get("vocabulary_sha256") != contract.get("vocabulary_sha256"):
        raise ValueError("oracle_contract_identity_mismatch")
    onnx_model = receipt.get("onnx_model")
    if not isinstance(onnx_model, dict) or set(onnx_model) != {"path", "sha256", "byte_size"} or onnx_model.get("sha256") != contract.get("model_sha256"):
        raise ValueError("oracle_onnx_model_identity_invalid")
    onnx_path = Path(str(onnx_model.get("path", ""))); normal_file(onnx_path, "oracle_onnx_model")
    if onnx_path.stat().st_size != onnx_model["byte_size"] or sha256_file(onnx_path) != onnx_model["sha256"]:
        raise ValueError("oracle_onnx_model_identity_drift")
    routes = contract.get("candidate_routes")
    if contract.get("preprocessing_status") != "CANDIDATE_UNVERIFIED" or contract.get("selected_route") is not None or not isinstance(routes, list) or len(routes) != 1:
        raise ValueError("oracle_contract_route_state_invalid")
    selected = receipt.get("selected_route")
    route = next((row for row in routes if isinstance(row, dict) and row.get("route_id") == selected), None)
    if route is None or receipt.get("preprocessing") != route.get("preprocessing") or receipt.get("preprocessing_sha256") != __import__("hashlib").sha256(json.dumps(route["preprocessing"], sort_keys=True, separators=(",", ":")).encode()).hexdigest():
        raise ValueError("oracle_preprocessing_identity_mismatch")
    root = path.parent
    capture_binding, candidate_binding = receipt.get("official_capture"), receipt.get("candidate")
    if not isinstance(capture_binding, dict) or not isinstance(candidate_binding, dict):
        raise ValueError("oracle_upstream_binding_missing")
    capture_path = Path(capture_binding.get("path", "")); candidate_path = Path(candidate_binding.get("path", ""))
    if capture_binding.get("sha256") != sha256_file(capture_path) or candidate_binding.get("sha256") != sha256_file(candidate_path):
        raise ValueError("oracle_upstream_binding_drift")
    capture = _capture(capture_path); candidate = _candidate(candidate_path, capture["raw_sensor"]["sha256"])
    expected_official = contract.get("official_preprocess_identity")
    official = capture["official_preprocessor"]
    if not isinstance(expected_official, dict) or expected_official.get("status") != "VERIFIED":
        raise ValueError("oracle_official_preprocess_identity_unavailable")
    for expected_key, observed_key in (("binary_path", "binary"), ("binary_sha256", "binary"), ("source_path", "source"), ("source_sha256", "source")):
        expected = expected_official.get(expected_key); observed = official[observed_key].get("path" if expected_key.endswith("_path") else "sha256")
        if expected != observed:
            raise ValueError("oracle_official_preprocess_identity_mismatch")
    identity = official["identity"]
    if any(identity.get(key) != expected_official.get(f"dpkg_{key}" if key in {"package", "version", "path_role"} else key) for key in ("package", "version", "path_role", "source_revision")):
        raise ValueError("oracle_official_preprocess_identity_mismatch")
    if receipt.get("candidate_hbm_sha256") != candidate["candidate_hbm"]["sha256"]:
        raise ValueError("oracle_candidate_hbm_mismatch")
    model_info = _bound(root, receipt.get("model_info"), "oracle_model_info").read_text(encoding="utf-8")
    expected_inputs = [{key: row[key] for key in ("index", "name", "shape", "dtype")} | {"aligned_byte_size": -1}
                       for row in contract["runtime_inputs"]]
    observed = _validate_model_info(model_info, receipt.get("model_name"), expected_inputs, contract["runtime_outputs"])
    if observed["input"] != {row["index"]: row for row in expected_inputs}:
        raise ValueError("oracle_model_info_input_mismatch")
    for key in ("version_stdout", "version_stderr", "infer_stdout", "infer_stderr"):
        _bound(root, receipt.get(key), f"oracle_{key}")
    runner = receipt.get("hrt_runner")
    if not isinstance(runner, dict) or set(runner) != {"path", "sha256"} or not isinstance(runner["path"], str) or not _digest(runner["sha256"]):
        raise ValueError("oracle_runner_identity_invalid")
    runner_path = Path(runner["path"]); normal_file(runner_path, "oracle_runner")
    if sha256_file(runner_path) != runner["sha256"]:
        raise ValueError("oracle_runner_identity_drift")
    executions = receipt.get("runner_executions")
    if not isinstance(executions, dict) or set(executions) != {"version", "model_info", "infer"}:
        raise ValueError("oracle_runner_execution_missing")
    for role, expected_prefix in (("version", [str(runner_path), "--version"]), ("model_info", [str(runner_path), "model_info"]), ("infer", [str(runner_path), "infer"])):
        execution = executions[role]
        if not isinstance(execution, dict) or execution.get("returncode") != 0 or execution.get("command", [])[:len(expected_prefix)] != expected_prefix:
            raise ValueError(f"oracle_runner_execution_invalid:{role}")
        cleaned = (execution.get("direct_process_reaped") is True and execution.get("zero_survivor") is False) if outer_supervised else execution.get("zero_survivor") is True
        if not cleaned:
            raise ValueError(f"oracle_runner_execution_invalid:{role}")
    metrics = receipt.get("raw_metrics")
    if not isinstance(metrics, dict) or set(metrics) != {"scores", "boxes"}:
        raise ValueError("oracle_metrics_invalid")
    for name, thresholds in contract["raw_output_thresholds"].items():
        observed_metrics = metrics.get(name)
        if not isinstance(observed_metrics, dict) or observed_metrics.get("cosine", -1.0) < thresholds["cosine_min"] or observed_metrics.get("normalized_rmse", float("inf")) > thresholds["normalized_rmse_max"]:
            raise ValueError(f"oracle_metric_failed:{name}")
        expected = np.load(_bound(root, receipt.get("onnx_outputs", {}).get(name), f"oracle_onnx_{name}"), allow_pickle=False)
        actual = np.load(_bound(root, receipt.get("hbm_outputs", {}).get(name), f"oracle_hbm_{name}"), allow_pickle=False)
        expected_onnx = contract["onnx_outputs"][list(contract["runtime_outputs"]).index(name)]
        if expected.dtype != np.float32 or list(expected.shape) != expected_onnx["shape"]:
            raise ValueError(f"oracle_onnx_output_abi_invalid:{name}")
        if raw_parity_metrics(expected, actual) != observed_metrics:
            raise ValueError(f"oracle_metric_receipt_drift:{name}")
    tensor = np.load(_bound(root, receipt.get("onnx_input"), "oracle_onnx_input"), allow_pickle=False)
    if tensor.dtype != np.float32 or tensor.shape != (1, 3, 640, 640) or not np.isfinite(tensor).all() or tensor.min() < 0 or tensor.max() > 1:
        raise ValueError("oracle_onnx_input_invalid")
    return receipt


def _supervision_file(root: Path, item: Any, label: str) -> Path:
    if not isinstance(item, dict) or set(item) != {"relative_path", "sha256", "byte_size"} or not _digest(item.get("sha256")) or not isinstance(item.get("byte_size"), int) or item["byte_size"] < 0:
        raise ValueError(f"{label}_binding_invalid")
    path = path_under(root, item["relative_path"], label)
    if path.stat().st_size != item["byte_size"] or sha256_file(path) != item["sha256"]:
        raise ValueError(f"{label}_binding_drift")
    return path


def validate(path: Path, *, contract_path: Path = CONTRACT) -> dict[str, Any]:
    """Accept only the outer supervisor; raw collector receipts are non-canonical."""
    normal_file(path, "oracle_supervision_receipt"); normal_file(contract_path, "oracle_contract")
    receipt = load_object(path)
    if (receipt.get("receipt_id") != SUPERVISION_ID or receipt.get("status") != STATUS
            or receipt.get("formal_compile") is not False or receipt.get("board_acceptance") is not False
            or receipt.get("wall_deadline_seconds") != 180 or receipt.get("blockers") != []
            or receipt.get("receipt_path") != str(path.resolve())):
        raise ValueError("oracle_supervision_finalizer_required")
    started, ended = receipt.get("started_epoch_ns"), receipt.get("ended_epoch_ns")
    if (not isinstance(started, int) or isinstance(started, bool) or started <= 0
            or not isinstance(ended, int) or isinstance(ended, bool) or ended < started):
        raise ValueError("oracle_supervision_timing_invalid")
    if receipt.get("producer_script_path") != str(SUPERVISOR.resolve()) or receipt.get("producer_script_sha256") != sha256_file(SUPERVISOR):
        raise ValueError("oracle_supervision_producer_identity_invalid")
    root = path.parent
    execution = receipt.get("collector_execution")
    if not isinstance(execution, dict) or set(execution) != {"pgid", "deadline_seconds", "term_grace_seconds", "timed_out", "term_sent", "kill_sent", "zero_survivor", "elapsed_seconds"} or not isinstance(execution["pgid"], int) or execution["pgid"] <= 1 or execution["deadline_seconds"] != 180 or execution["term_grace_seconds"] != 10 or execution["timed_out"] is not False or execution["zero_survivor"] is not True:
        raise ValueError("oracle_supervision_execution_invalid")
    watchdog = receipt.get("memory_watchdog")
    if not isinstance(watchdog, dict) or set(watchdog) != {"json", "log", "returncode", "status", "pgid"}:
        raise ValueError("oracle_supervision_watchdog_invalid")
    report = load_object(_supervision_file(root, watchdog["json"], "oracle_supervision_watchdog_json"))
    _supervision_file(root, watchdog["log"], "oracle_supervision_watchdog_log")
    if (watchdog["returncode"] != 0 or watchdog["status"] != "FORMAL_MEMORY_WATCHDOG_COMPLETED"
            or watchdog["pgid"] != execution["pgid"] or report.get("status") != watchdog["status"]
            or report.get("target_pgid") != watchdog["pgid"] or report.get("surviving_group_processes") != 0
            or report.get("breach_exit_code") != 86
            or report.get("thresholds_kib") != MEMORY_WATCHDOG_THRESHOLDS):
        raise ValueError("oracle_supervision_watchdog_failed")
    _supervision_file(root, receipt.get("collector_stdout"), "oracle_supervision_stdout")
    _supervision_file(root, receipt.get("collector_stderr"), "oracle_supervision_stderr")
    child = _supervision_file(root, receipt.get("child_oracle"), "oracle_supervision_child")
    if child.parent != (root / "collector").resolve():
        raise ValueError("oracle_supervision_child_path_invalid")
    return _validate_raw(child, contract_path=contract_path, outer_supervised=True)


def main() -> int:
    import argparse
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("--receipt", required=True, type=Path); parser.add_argument("--contract", type=Path, default=CONTRACT)
    args = parser.parse_args()
    try: value = validate(args.receipt, contract_path=args.contract)
    except Exception as exc: print(f"preprocess_oracle_blocked:{type(exc).__name__}:{exc}"); return 2
    print(json.dumps({"status": value["status"], "receipt_sha256": sha256_file(args.receipt)}, indent=2)); return 0


if __name__ == "__main__": raise SystemExit(main())
