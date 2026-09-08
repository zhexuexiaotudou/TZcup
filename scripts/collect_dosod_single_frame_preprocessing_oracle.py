#!/usr/bin/env python3
"""Run the real HRT single-frame preprocessing oracle, or retain BLOCKED evidence.

The official preprocessor is deliberately external: this producer consumes its
capture receipt rather than accepting an adapter path or manufacturing NV12.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

from hbm_evidence_common import atomic_json, fresh_directory, normal_file, run_owned_process, sha256_file
from dosod_hbm_abi_contract import validate_pre_onnx_outputs
from run_dosod_hbm_x86_parity import COMMAND_TEMPLATE, MODEL_INFO_TEMPLATE, _dump_filename, _validate_model_info, raw_parity_metrics
from validate_dosod_single_frame_preprocessing_oracle import CONTRACT, RECEIPT_ID, STATUS, _candidate, _capture

ROOT = Path(__file__).resolve().parents[1]


def _binding(root: Path, path: Path) -> dict[str, Any]:
    if not path.is_relative_to(root):
        raise ValueError("oracle_artifact_path_escape")
    return {"relative_path": path.relative_to(root).as_posix(), "sha256": sha256_file(path), "byte_size": path.stat().st_size}


def _write(root: Path, name: str, payload: str) -> dict[str, Any]:
    path = root / name; path.write_text(payload, encoding="utf-8"); return _binding(root, path)


def _preprocess(raw: dict[str, Any]) -> np.ndarray:
    from collect_formal_s100_calibration_frames import preprocess_dosod_rgb, rgb_from_ros_image
    path = Path(raw["path"])
    return preprocess_dosod_rgb(rgb_from_ros_image(data=path.read_bytes(), width=raw["width"], height=raw["height"], step=raw["step"], encoding=raw["encoding"]))


def _run(command: list[str], timeout: float) -> tuple[int | None, str, str, dict[str, Any]]:
    # The outer supervisor owns this PGID.  Its final TERM/KILL sweep covers
    # ORT and every HRT child; direct CLI use keeps the standalone session.
    return run_owned_process(command, timeout_seconds=timeout,
                             start_new_session=os.environ.get("TZCUP_ORACLE_OUTER_SUPERVISED") != "1")


def _command_clean(execution: dict[str, Any]) -> bool:
    return execution.get("zero_survivor") is True or (
        os.environ.get("TZCUP_ORACLE_OUTER_SUPERVISED") == "1"
        and execution.get("direct_process_reaped") is True)


def _candidate_route(contract: dict[str, Any]) -> dict[str, Any]:
    routes = contract.get("candidate_routes")
    if (contract.get("preprocessing_status") != "CANDIDATE_UNVERIFIED"
            or contract.get("selected_route") is not None
            or not isinstance(routes, list) or len(routes) != 1
            or not isinstance(routes[0], dict)
            or not isinstance(routes[0].get("route_id"), str) or not routes[0]["route_id"]
            or not isinstance(routes[0].get("preprocessing"), dict)):
        raise ValueError("oracle_candidate_route_contract_invalid")
    return routes[0]


def collect(*, candidate_receipt: Path, official_capture_receipt: Path, onnx_model: Path, hrt: Path, output: Path, fixture: bool = False) -> dict[str, Any]:
    normal_file(candidate_receipt, "candidate_receipt"); normal_file(official_capture_receipt, "official_capture_receipt"); normal_file(onnx_model, "onnx_model"); normal_file(hrt, "hrt_model_exec")
    fresh_directory(output, "oracle_evidence_output")
    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    receipt: dict[str, Any] = {"schema_version": 1, "receipt_id": RECEIPT_ID, "status": "BLOCKED", "formal_compile": False, "board_acceptance": False, "test_fixture": fixture,
        "contract_sha256": sha256_file(CONTRACT), "model_sha256": contract["model_sha256"], "vocabulary_sha256": contract["vocabulary_sha256"],
        "selected_route": None, "preprocessing": None, "preprocessing_sha256": None, "candidate": {"path": str(candidate_receipt.resolve()), "sha256": sha256_file(candidate_receipt)}, "official_capture": {"path": str(official_capture_receipt.resolve()), "sha256": sha256_file(official_capture_receipt)},
        "candidate_hbm_sha256": None, "onnx_model": {"path": str(onnx_model.resolve()), "sha256": sha256_file(onnx_model), "byte_size": onnx_model.stat().st_size}, "model_name": None, "hrt_runner": {"path": str(hrt.resolve()), "sha256": sha256_file(hrt)}, "runner_executions": {}, "version_stdout": None, "version_stderr": None, "model_info": None, "infer_stdout": None, "infer_stderr": None, "onnx_input": None, "onnx_outputs": {}, "hbm_outputs": {}, "raw_metrics": {}, "blockers": [], "started_epoch_ns": time.time_ns(), "ended_epoch_ns": None}
    try:
        if fixture:
            raise ValueError("test_fixture_cli_forbidden")
        route = _candidate_route(contract)
        validate_pre_onnx_outputs(contract.get("onnx_outputs"))
        receipt["selected_route"] = route["route_id"]
        receipt["preprocessing"] = route["preprocessing"]
        receipt["preprocessing_sha256"] = hashlib.sha256(json.dumps(route["preprocessing"], sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        if receipt["onnx_model"]["sha256"] != contract["model_sha256"]:
            raise ValueError("oracle_onnx_model_contract_mismatch")
        capture = _capture(official_capture_receipt); candidate = _candidate(candidate_receipt, capture["raw_sensor"]["sha256"])
        if candidate.get("candidate_route") != route["route_id"]:
            raise ValueError("oracle_candidate_route_mismatch")
        receipt["candidate_hbm_sha256"] = candidate["candidate_hbm"]["sha256"]
        tensor = _preprocess(capture["raw_sensor"])
        if tensor.shape != (1, 3, 640, 640) or tensor.dtype != np.float32 or not np.isfinite(tensor).all():
            raise ValueError("oracle_onnx_input_invalid")
        np.save(output / "onnx_input.npy", tensor, allow_pickle=False); receipt["onnx_input"] = _binding(output, output / "onnx_input.npy")
        try:
            import onnxruntime as ort
        except ImportError as exc:
            raise ValueError("onnxruntime_unavailable") from exc
        session = ort.InferenceSession(str(onnx_model), providers=["CPUExecutionProvider"])
        outputs = session.run([row["name"] for row in contract["onnx_outputs"]], {"images": tensor})
        for expected, value in zip(contract["onnx_outputs"], outputs):
            name = expected["name"]
            if value.dtype != np.float32 or value.shape != tuple(expected["shape"]) or not np.isfinite(value).all():
                raise ValueError(f"oracle_onnx_output_invalid:{name}")
            path = output / f"onnx_{name}.npy"; np.save(path, value, allow_pickle=False); receipt["onnx_outputs"][name] = _binding(output, path)
        version_code, version_stdout, version_stderr, version_exec = _run([str(hrt), "--version"], 30)
        receipt["version_stdout"] = _write(output, "hrt.version.stdout.txt", version_stdout); receipt["version_stderr"] = _write(output, "hrt.version.stderr.txt", version_stderr)
        receipt["runner_executions"]["version"] = {"command": [str(hrt), "--version"], "returncode": version_code, **version_exec}
        if version_code != 0 or not _command_clean(version_exec):
            raise ValueError("oracle_hrt_version_failed")
        hbm = Path(candidate["candidate_hbm"]["path"])
        info_command = [item.format(runner=str(hrt), hbm=str(hbm)) for item in MODEL_INFO_TEMPLATE]
        info_code, info_stdout, info_stderr, info_exec = _run(info_command, 30)
        receipt["model_info"] = _write(output, "hrt.model_info.stdout.txt", info_stdout); _write(output, "hrt.model_info.stderr.txt", info_stderr)
        receipt["runner_executions"]["model_info"] = {"command": info_command, "returncode": info_code, **info_exec}
        if info_code != 0 or not _command_clean(info_exec):
            raise ValueError("oracle_hrt_model_info_failed")
        name = next(iter(__import__("re").findall(r"^\[model name\]:\s*(.+?)\s*$", info_stdout, flags=__import__("re").MULTILINE)), None)
        if not name:
            raise ValueError("oracle_model_name_missing")
        receipt["model_name"] = name
        expected_inputs = [{key: row[key] for key in ("index", "name", "shape", "dtype")} | {"aligned_byte_size": -1}
                           for row in contract["runtime_inputs"]]
        _validate_model_info(info_stdout, name, expected_inputs, contract["runtime_outputs"])
        files = capture["inputs"]; dump = output / "hbm_dump"; dump.mkdir()
        command = [item.format(runner=str(hrt), hbm=str(hbm), inputs=",".join(str(item["path"]) for item in files), dump_path=str(dump) + "/") for item in COMMAND_TEMPLATE]
        code, stdout, stderr, execution = _run(command, 120)
        receipt["infer_stdout"] = _write(output, "hrt.infer.stdout.txt", stdout); receipt["infer_stderr"] = _write(output, "hrt.infer.stderr.txt", stderr); receipt["infer_command"] = command; receipt["infer_execution"] = execution; receipt["runner_executions"]["infer"] = {"command": command, "returncode": code, **execution}
        if code != 0 or not _command_clean(execution):
            raise ValueError("oracle_hrt_infer_failed")
        for name, binding in contract["runtime_outputs"].items():
            path = dump / _dump_filename(binding); normal_file(path, f"oracle_hbm_{name}")
            value = np.load(path, allow_pickle=False); expected = np.load(output / f"onnx_{name}.npy", allow_pickle=False)
            if value.shape != expected.shape or not np.isfinite(value).all():
                raise ValueError(f"oracle_hbm_output_invalid:{name}")
            receipt["hbm_outputs"][name] = _binding(output, path); receipt["raw_metrics"][name] = raw_parity_metrics(expected, value)
            limits = contract["raw_output_thresholds"][name]
            if receipt["raw_metrics"][name]["cosine"] < limits["cosine_min"] or receipt["raw_metrics"][name]["normalized_rmse"] > limits["normalized_rmse_max"]:
                raise ValueError(f"oracle_metric_failed:{name}")
        receipt["status"] = STATUS
    except Exception as exc:
        receipt["blockers"].append(f"oracle_failed:{type(exc).__name__}:{exc}")
        if fixture: receipt["status"] = "TEST_FIXTURE_BLOCKED"
    finally:
        receipt["ended_epoch_ns"] = time.time_ns(); target = output / "dosod_single_frame_preprocessing_oracle_receipt.json"; receipt.update({"receipt_path": str(target.resolve()), "producer_script_path": str(Path(__file__).resolve()), "producer_script_sha256": sha256_file(Path(__file__).resolve())}); atomic_json(target, receipt)
    return receipt


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__); p.add_argument("--candidate-receipt", required=True, type=Path); p.add_argument("--official-capture-receipt", required=True, type=Path); p.add_argument("--onnx-model", required=True, type=Path); p.add_argument("--hrt-model-exec", required=True, type=Path); p.add_argument("--output", required=True, type=Path); p.add_argument("--test-fixture", action="store_true")
    a = p.parse_args(); result = collect(candidate_receipt=a.candidate_receipt, official_capture_receipt=a.official_capture_receipt, onnx_model=a.onnx_model, hrt=a.hrt_model_exec, output=a.output, fixture=a.test_fixture)
    print(json.dumps(result, ensure_ascii=False, indent=2)); return 0 if result["status"] == STATUS else 2


if __name__ == "__main__": raise SystemExit(main())
