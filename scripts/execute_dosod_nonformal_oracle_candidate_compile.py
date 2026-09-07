#!/usr/bin/env python3
"""Create a non-formal HBM solely for the single-frame preprocess oracle.

Its receipt is intentionally incompatible with formal compile/parity/metric/S100
admission receipts.  Production always owns one POSIX process group.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

from hbm_evidence_common import atomic_json, fresh_directory, load_object, normal_file, run_owned_process, sha256_file
from public_gazebo_dosod_calibration import canonical_sha256, load_scene_plan, validate_pilot_manifest

RECEIPT_ID = "tzcup_dosod_nonformal_oracle_candidate_compile_receipt_v1"
STATUS = "NON_FORMAL_ORACLE_CANDIDATE_COMPILED"
ROOT = Path(__file__).resolve().parents[1]
CANONICAL_ORACLE_CONTRACT = ROOT / "config" / "dosod_single_frame_preprocessing_oracle_contract.json"
CANONICAL_COMPILE_CONTRACT = ROOT / "config" / "dosod_s100p_hbm_compile_contract.json"
PILOT_PRODUCER = ROOT / "scripts" / "public_gazebo_dosod_calibration.py"


def _block(receipt: dict[str, Any], reason: str) -> None:
    if reason not in receipt["blockers"]:
        receipt["blockers"].append(reason)


def _pilot_binding(pilot_manifest: Path, index: int) -> dict[str, Any]:
    normal_file(pilot_manifest, "pilot_manifest")
    plan = load_scene_plan(ROOT / "config" / "public_gazebo_dosod_train_scene_plan.json")
    compile_contract = load_object(CANONICAL_COMPILE_CONTRACT)
    pilot = validate_pilot_manifest(pilot_manifest, plan=plan, contract=compile_contract)
    if not isinstance(index, int) or not 0 <= index < len(pilot["records"]):
        raise ValueError("pilot_record_index_invalid")
    record = pilot["records"][index]
    raw = record.get("raw_sensor")
    if not isinstance(raw, dict) or set(raw) != {"relative_path", "sha256", "byte_size", "width", "height", "step", "encoding", "frame_id", "stamp_ns"}:
        raise ValueError("pilot_raw_record_invalid")
    path = (pilot_manifest.parent / raw["relative_path"]).resolve(); normal_file(path, "pilot_raw")
    if path.stat().st_size != raw["byte_size"] or sha256_file(path) != raw["sha256"] or raw["sha256"] != record.get("source_sha256"):
        raise ValueError("pilot_raw_record_drift")
    if raw["width"] != 848 or raw["height"] != 480 or raw["encoding"] not in {"rgb8", "bgr8"} or raw["step"] != 848 * 3 or raw["byte_size"] != raw["step"] * raw["height"]:
        raise ValueError("pilot_raw_not_848x480_rgb")
    return {"path": str(path), **raw, "pilot_manifest_path": str(pilot_manifest.resolve()), "pilot_manifest_sha256": sha256_file(pilot_manifest), "pilot_record_sha256": canonical_sha256(record), "pilot_record_index": index}


def _candidate_calibration(pilot_manifest: Path, oracle_contract: dict[str, Any]) -> dict[str, Any]:
    """The pre-existing pilot manifest is the only allowed 25-tensor closure."""
    pilot = load_object(pilot_manifest)
    routes = oracle_contract.get("candidate_routes")
    if not isinstance(routes, list) or len(routes) != 1 or routes[0].get("route_id") != "BOOTSTRAP_SYMMETRIC_BLACK_V1":
        raise ValueError("candidate_route_contract_invalid")
    records = pilot.get("records")
    if not isinstance(records, list) or len(records) != 25 or pilot.get("record_sha256") != canonical_sha256(records):
        raise ValueError("candidate_calibration_pilot_closure_invalid")
    for index, row in enumerate(records):
        sample = pilot_manifest.parent / f"samples/{index:06d}.npy"
        normal_file(sample, "candidate_pilot_tensor")
        tensor = np.load(sample, mmap_mode="r", allow_pickle=False)
        if row.get("relative_path") != f"samples/{index:06d}.npy" or sample.stat().st_size != row.get("byte_size") or sha256_file(sample) != row.get("sha256") or tensor.dtype != np.float32 or tuple(tensor.shape) != (1, 3, 640, 640) or not np.isfinite(tensor).all() or float(tensor.min()) < 0.0 or float(tensor.max()) > 1.0:
            raise ValueError("candidate_pilot_tensor_drift")
    # _pilot_binding independently validates every raw/provenance row.  This
    # binds that same canonical tensor closure as hb_compile calibration input.
    return {"candidate_route": routes[0]["route_id"], "records_sha256": pilot["record_sha256"]}


def _expected_compile_config(*, model: Path, work: Path, calibration: Path, recipe: dict[str, Any]) -> dict[str, Any]:
    """The candidate may change only its fresh work/calibration paths, never recipe."""
    return {
        "model_parameters": {
            "onnx_model": str(model.resolve()), "march": recipe["march"],
            "working_dir": str(work.resolve()), "output_model_file_prefix": recipe["output_model_file_prefix"],
            "remove_node_type": "Dequantize;Quantize;Transpose;Cast;Reshape", "layer_out_dump": False,
        },
        "input_parameters": {
            "input_name": recipe["input_name"], "input_type_train": recipe["input_type_train"],
            "input_layout_train": recipe["input_layout_train"], "input_shape": recipe["input_shape"],
            "input_batch": recipe["input_batch"], "norm_type": recipe["norm_type"], "mean_value": "",
            "scale_value": recipe["scale_value"], "input_layout_rt": recipe["input_layout_rt"],
            "input_type_rt": recipe["input_type_rt"],
        },
        "calibration_parameters": {
            "cal_data_dir": str(calibration.resolve()), "cal_data_type": recipe["cal_data_type"],
            "preprocess_on": recipe["preprocess_on"], "calibration_type": recipe["calibration_type"],
            "max_percentile": recipe["max_percentile"], "optimization": recipe["optimization"],
        },
        "compiler_parameters": {"compile_mode": recipe["compile_mode"], "debug": False, "optimize_level": recipe["optimize_level"], "jobs": recipe["jobs_default"]},
    }


def validate_candidate_receipt(receipt_path: Path, *, expected_raw_sha256: str | None = None) -> dict[str, Any]:
    """Re-audit a candidate receipt before it can feed the oracle.

    This deliberately validates the original compile inputs and retained raw
    logs, rather than treating a status/HBM pair as sufficient evidence.
    """
    normal_file(receipt_path, "candidate_receipt")
    value = load_object(receipt_path); root = receipt_path.parent.resolve()
    if value.get("receipt_id") != RECEIPT_ID or value.get("status") != STATUS or value.get("formal_compile") is not False or value.get("board_acceptance") is not False or value.get("receipt_path") != str(receipt_path.resolve()) or value.get("producer_script_path") != str(Path(__file__).resolve()) or value.get("producer_script_sha256") != sha256_file(Path(__file__)) or value.get("blockers") != [] or value.get("returncode") != 0:
        raise ValueError("candidate_receipt_header_invalid")
    for key in ("canonical_compile_contract_sha256", "canonical_oracle_contract_sha256"):
        expected = sha256_file(CANONICAL_COMPILE_CONTRACT if key.startswith("canonical_compile") else CANONICAL_ORACLE_CONTRACT)
        if value.get(key) != expected:
            raise ValueError("candidate_receipt_contract_drift")
    normal_file(PILOT_PRODUCER, "candidate_pilot_producer")
    if value.get("pilot_producer_script_path") != str(PILOT_PRODUCER.resolve()) or value.get("pilot_producer_script_sha256") != sha256_file(PILOT_PRODUCER):
        raise ValueError("candidate_receipt_pilot_producer_invalid")
    raw = value.get("pilot_raw")
    if not isinstance(raw, dict) or not isinstance(raw.get("pilot_manifest_path"), str) or not isinstance(raw.get("pilot_record_index"), int):
        raise ValueError("candidate_receipt_pilot_binding_invalid")
    pilot_path = Path(raw["pilot_manifest_path"]); normal_file(pilot_path, "candidate_pilot_manifest")
    if raw.get("pilot_manifest_sha256") != sha256_file(pilot_path) or value.get("pilot_manifest_sha256") != sha256_file(pilot_path):
        raise ValueError("candidate_receipt_pilot_manifest_drift")
    rebound = _pilot_binding(pilot_path, raw["pilot_record_index"])
    if raw != rebound or (expected_raw_sha256 is not None and raw.get("sha256") != expected_raw_sha256):
        raise ValueError("candidate_receipt_pilot_raw_drift")
    pilot = load_object(pilot_path)
    if value.get("pilot_record_count") != 25 or value.get("pilot_records_sha256") != pilot.get("record_sha256") or value.get("candidate_calibration_records_sha256") != pilot.get("record_sha256"):
        raise ValueError("candidate_receipt_pilot_closure_drift")
    oracle_contract = load_object(CANONICAL_ORACLE_CONTRACT)
    calibration = _candidate_calibration(pilot_path, oracle_contract)
    if value.get("candidate_route") != calibration["candidate_route"]:
        raise ValueError("candidate_receipt_route_drift")
    compile_contract = load_object(CANONICAL_COMPILE_CONTRACT)
    bound_paths: dict[str, Path] = {}
    for label, path_key, digest_key in (("model", "model_path", "model_sha256"), ("vocabulary", "vocabulary_path", "vocabulary_sha256"), ("compiler_identity", "compiler_identity_path", "compiler_identity_sha256"), ("compile_config", "compile_config_path", "compile_config_sha256")):
        item = value.get(path_key)
        if not isinstance(item, str): raise ValueError(f"candidate_receipt_{label}_path_invalid")
        path = Path(item); normal_file(path, f"candidate_{label}")
        if value.get(digest_key) != sha256_file(path): raise ValueError(f"candidate_receipt_{label}_drift")
        bound_paths[label] = path
    if value["model_sha256"] != compile_contract["model"]["sha256"] or value["vocabulary_sha256"] != compile_contract["vocabulary"]["sha256"]:
        raise ValueError("candidate_receipt_model_or_vocabulary_drift")
    identity = load_object(bound_paths["compiler_identity"])
    if identity.get("identity_verified") is not True or identity.get("hb_compile_probe_returncode") != 0 or identity.get("required_versions") != compile_contract["toolchain"]["required_versions"]:
        raise ValueError("candidate_receipt_compiler_identity_invalid")
    try:
        import yaml
        config = yaml.safe_load(bound_paths["compile_config"].read_text(encoding="utf-8"))
    except Exception as exc: raise ValueError("candidate_receipt_compile_config_invalid") from exc
    expected_hbm = root / "candidate_work" / "dosod_mlp3x_s_tzcup_rep-int16.hbm"
    if config != _expected_compile_config(model=bound_paths["model"], work=root / "candidate_work", calibration=pilot_path.parent / "samples", recipe=compile_contract["compile_recipe"]):
        raise ValueError("candidate_receipt_compile_config_drift")
    command = value.get("command")
    if not isinstance(command, list) or command != [command[0], "-c", str(bound_paths["compile_config"].resolve())]:
        raise ValueError("candidate_receipt_command_invalid")
    executable = Path(command[0]); normal_file(executable, "candidate_compiler_executable")
    if identity.get("hb_compile_executable") != str(executable.resolve()) or identity.get("hb_compile_executable_sha256") != sha256_file(executable):
        raise ValueError("candidate_receipt_compiler_executable_drift")
    execution = value.get("execution")
    if not isinstance(execution, dict) or execution.get("deadline_seconds") != 3600 or execution.get("term_grace_seconds") != 10 or execution.get("timed_out") is not False or execution.get("zero_survivor") is not True:
        raise ValueError("candidate_receipt_execution_invalid")
    for name in ("stdout", "stderr"):
        rel, digest = value.get(f"raw_{name}_path"), value.get(f"raw_{name}_sha256")
        path = (root / str(rel)).resolve()
        if not path.is_relative_to(root): raise ValueError("candidate_receipt_log_path_escape")
        normal_file(path, f"candidate_{name}")
        if sha256_file(path) != digest: raise ValueError("candidate_receipt_log_drift")
    hbm = value.get("candidate_hbm")
    if not isinstance(hbm, dict) or hbm.get("path") != str(expected_hbm.resolve()) or value.get("expected_hbm_path") != str(expected_hbm.resolve()):
        raise ValueError("candidate_receipt_hbm_path_invalid")
    normal_file(expected_hbm, "candidate_hbm")
    if expected_hbm.stat().st_size != hbm.get("byte_size") or sha256_file(expected_hbm) != hbm.get("sha256"):
        raise ValueError("candidate_receipt_hbm_drift")
    return value

def execute(*, pilot_manifest: Path, pilot_record_index: int, compiler_identity: Path, compile_config: Path, model: Path, vocabulary: Path, output: Path, compiler: str = "hb_compile") -> dict[str, Any]:
    for path, label in ((pilot_manifest, "pilot_manifest"), (compiler_identity, "compiler_identity"), (compile_config, "compile_config"), (model, "model"), (vocabulary, "vocabulary")):
        normal_file(path, label)
    fresh_directory(output, "candidate_evidence_output")
    receipt: dict[str, Any] = {"schema_version": 1, "receipt_id": RECEIPT_ID, "status": "BLOCKED", "formal_compile": False, "board_acceptance": False,
        "claim_boundary": "non-formal candidate only; never formal compile, parity, metric, dataset, or board evidence", "started_epoch_ns": time.time_ns(), "ended_epoch_ns": None,
        "candidate_hbm": None, "command": None, "returncode": None, "blockers": [], "raw_stdout_path": "hb_compile.stdout.txt", "raw_stderr_path": "hb_compile.stderr.txt",
        "raw_stdout_sha256": None, "raw_stderr_sha256": None, "execution": None}
    try:
        raw = _pilot_binding(pilot_manifest, pilot_record_index)
        oracle_contract = load_object(CANONICAL_ORACLE_CONTRACT)
        normal_file(PILOT_PRODUCER, "candidate_pilot_producer")
        calibration = _candidate_calibration(pilot_manifest, oracle_contract)
        identity = load_object(compiler_identity)
        compile_contract = load_object(CANONICAL_COMPILE_CONTRACT)
        if sha256_file(model) != compile_contract["model"]["sha256"] or sha256_file(vocabulary) != compile_contract["vocabulary"]["sha256"] or identity.get("identity_verified") is not True or identity.get("hb_compile_probe_returncode") != 0 or identity.get("required_versions") != compile_contract["toolchain"]["required_versions"]:
            raise ValueError("candidate_compiler_identity_not_verified")
        executable = shutil.which(compiler)
        if not executable or identity.get("hb_compile_executable") != str(Path(executable).resolve()) or identity.get("hb_compile_executable_sha256") != sha256_file(Path(executable)):
            raise ValueError("candidate_compiler_identity_mismatch")
        try:
            import yaml
            config = yaml.safe_load(compile_config.read_text(encoding="utf-8"))
        except Exception as exc: raise ValueError("candidate_compile_config_invalid") from exc
        work = output / "candidate_work"; expected_hbm = work / "dosod_mlp3x_s_tzcup_rep-int16.hbm"
        expected_config = _expected_compile_config(model=model, work=work, calibration=pilot_manifest.parent / "samples", recipe=compile_contract["compile_recipe"])
        if config != expected_config or expected_hbm.exists() or expected_hbm.is_symlink():
            raise ValueError("candidate_compile_config_binding_invalid")
        command = [str(Path(executable).resolve()), "-c", str(compile_config.resolve())]
        receipt.update({"pilot_raw": raw, "pilot_manifest_sha256":sha256_file(pilot_manifest), "pilot_records_sha256":canonical_sha256(load_object(pilot_manifest)["records"]), "pilot_record_count":25, "pilot_producer_script_path":str(PILOT_PRODUCER.resolve()), "pilot_producer_script_sha256":sha256_file(PILOT_PRODUCER), "candidate_route":calibration["candidate_route"], "candidate_calibration_records_sha256":calibration["records_sha256"], "canonical_compile_contract_sha256": sha256_file(CANONICAL_COMPILE_CONTRACT), "canonical_oracle_contract_sha256": sha256_file(CANONICAL_ORACLE_CONTRACT), "model_path":str(model.resolve()), "model_sha256": sha256_file(model), "vocabulary_path":str(vocabulary.resolve()), "vocabulary_sha256": sha256_file(vocabulary), "compiler_identity_path": str(compiler_identity.resolve()), "compiler_identity_sha256": sha256_file(compiler_identity), "compile_config_path": str(compile_config.resolve()), "compile_config_sha256": sha256_file(compile_config), "expected_hbm_path": str(expected_hbm.resolve()), "command": command})
        code, stdout, stderr, execution = run_owned_process(command, timeout_seconds=3600)
        (output / receipt["raw_stdout_path"]).write_text(stdout, encoding="utf-8")
        (output / receipt["raw_stderr_path"]).write_text(stderr, encoding="utf-8")
        receipt.update({"returncode": code, "execution": execution, "raw_stdout_sha256": sha256_file(output / receipt["raw_stdout_path"]), "raw_stderr_sha256": sha256_file(output / receipt["raw_stderr_path"])})
        if code != 0 or execution.get("timed_out") or execution.get("zero_survivor") is not True:
            raise ValueError("candidate_compile_execution_failed")
        normal_file(expected_hbm, "candidate_hbm")
        if expected_hbm.stat().st_size <= 0:
            raise ValueError("candidate_hbm_empty")
        receipt["candidate_hbm"] = {"path": str(expected_hbm.resolve()), "sha256": sha256_file(expected_hbm), "byte_size": expected_hbm.stat().st_size}
        receipt["status"] = STATUS
    except Exception as exc:
        _block(receipt, f"candidate_compile_failed:{type(exc).__name__}:{exc}")
    finally:
        receipt["ended_epoch_ns"] = time.time_ns()
        receipt_path = output / "dosod_nonformal_oracle_candidate_compile_receipt.json"
        receipt.update({"receipt_path": str(receipt_path.resolve()), "producer_script_path": str(Path(__file__).resolve()), "producer_script_sha256": sha256_file(Path(__file__).resolve())})
        atomic_json(receipt_path, receipt)
        if receipt["status"] == STATUS:
            try: validate_candidate_receipt(receipt_path)
            except Exception as exc:
                receipt["status"] = "BLOCKED"; _block(receipt, f"candidate_receipt_self_validation_failed:{type(exc).__name__}:{exc}"); atomic_json(receipt_path, receipt)
    return receipt


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--pilot-manifest", required=True, type=Path); p.add_argument("--pilot-record-index", required=True, type=int)
    p.add_argument("--compiler-identity", required=True, type=Path); p.add_argument("--compile-config", required=True, type=Path)
    p.add_argument("--model", required=True, type=Path); p.add_argument("--vocabulary", required=True, type=Path); p.add_argument("--output", required=True, type=Path); p.add_argument("--compiler", default="hb_compile")
    args = p.parse_args()
    result = execute(pilot_manifest=args.pilot_manifest, pilot_record_index=args.pilot_record_index, compiler_identity=args.compiler_identity, compile_config=args.compile_config, model=args.model, vocabulary=args.vocabulary, output=args.output, compiler=args.compiler)
    print(json.dumps(result, ensure_ascii=False, indent=2)); return 0 if result["status"] == STATUS else 2


if __name__ == "__main__": raise SystemExit(main())
