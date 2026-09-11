#!/usr/bin/env python3
"""Materialize a resumable DOSOD calibration corpus from product RGB captures.

This is an offline, read-only input consumer.  It accepts only atomic
``ProductIntermediateCapture`` frame bundles that were already written by the
product, checks every recorded source byte, and writes a new corpus directory.
It never reads MCAP by guessing its serialization, starts ROS/Gazebo, or makes
up frames.  Until an accepted official preprocessing oracle is supplied, its
output is deliberately a non-formal smoke corpus and cannot be used to compile.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any

import numpy as np

from collect_formal_s100_calibration_frames import (
    CalibrationRejected, atomic_write_json, canonical_sha256, preprocess_dosod_rgb,
    sha256_file,
)


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config" / "dosod_s100p_hbm_compile_contract.json"
STATE_NAME = "materialization_state.json"
RECEIPT_NAME = "materialization_receipt.json"
CORPUS_MANIFEST_NAME = "calibration_corpus_manifest.json"


def _regular(path: Path, label: str) -> None:
    if path.is_symlink() or not path.is_file():
        raise CalibrationRejected(f"{label}_not_regular_file")


def _relative(root: Path, value: str, label: str) -> Path:
    candidate = Path(value)
    if candidate.is_absolute() or not candidate.parts or any(part in {".", ".."} for part in candidate.parts):
        raise CalibrationRejected(f"{label}_path_invalid")
    current = root
    for part in candidate.parts:
        current = current / part
        if current.is_symlink():
            raise CalibrationRejected(f"{label}_path_symlink")
    return current


def _load_object(path: Path, label: str) -> dict[str, Any]:
    _regular(path, label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CalibrationRejected(f"{label}_unreadable") from exc
    if not isinstance(value, dict):
        raise CalibrationRejected(f"{label}_not_object")
    return value


def _load_contract(path: Path) -> dict[str, Any]:
    contract = _load_object(path, "contract")
    calibration, preprocessing = contract.get("calibration"), contract.get("preprocessing")
    if not isinstance(calibration, dict) or not isinstance(preprocessing, dict):
        raise CalibrationRejected("contract_shape_invalid")
    if calibration.get("minimum_sample_count") != 500 or preprocessing.get("tensor_shape") != [1, 3, 640, 640]:
        raise CalibrationRejected("contract_not_frozen_dosod_500_640")
    return contract


def _verified_oracle(path: Path | None) -> tuple[str, str]:
    """Return formal proof only for the canonical outer oracle finalizer."""
    if path is None:
        return "MISSING", "official_preprocessing_oracle_required"
    try:
        from validate_dosod_single_frame_preprocessing_oracle import validate
        validate(path)
    except Exception as exc:
        return "REJECTED", f"official_preprocessing_oracle_rejected:{type(exc).__name__}"
    return "VERIFIED", sha256_file(path)


def _frame_bundle(frame: Path) -> tuple[np.ndarray, dict[str, Any], str, str]:
    if frame.is_symlink() or not frame.is_dir():
        raise CalibrationRejected("capture_frame_not_regular_directory")
    arrays_path, metadata_path, manifest_path = (frame / "arrays.npz", frame / "metadata.json", frame / "manifest.json")
    manifest = _load_object(manifest_path, "capture_manifest")
    files = manifest.get("files")
    if manifest.get("schema_version") != 1 or not isinstance(files, dict) or set(files) != {"arrays.npz", "metadata.json"}:
        raise CalibrationRejected("capture_manifest_shape_invalid")
    for path, name in ((arrays_path, "arrays.npz"), (metadata_path, "metadata.json")):
        _regular(path, f"capture_{name}")
        if files.get(name) != sha256_file(path):
            raise CalibrationRejected(f"capture_{name}_hash_mismatch")
    metadata = _load_object(metadata_path, "capture_metadata")
    if metadata.get("sensor") != "front" or not isinstance(metadata.get("rgb_stamp_s"), (int, float)):
        raise CalibrationRejected("capture_metadata_sensor_or_timestamp_invalid")
    camera = metadata.get("camera_info")
    if not isinstance(camera, dict) or not isinstance(camera.get("frame_id"), str) or not camera["frame_id"]:
        raise CalibrationRejected("capture_camera_identity_missing")
    try:
        with np.load(arrays_path, allow_pickle=False) as arrays:
            rgb = np.asarray(arrays["rgb"])
    except (OSError, KeyError, ValueError) as exc:
        raise CalibrationRejected("capture_rgb_unreadable") from exc
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise CalibrationRejected("capture_rgb_shape_or_dtype_invalid")
    source_sha = hashlib.sha256(rgb.tobytes(order="C")).hexdigest()
    return rgb, metadata, source_sha, sha256_file(manifest_path)


def _classes(metadata: dict[str, Any], allowed: set[str]) -> list[str]:
    detections = metadata.get("detections", [])
    if not isinstance(detections, list):
        raise CalibrationRejected("capture_detections_invalid")
    values = sorted({str(row.get("class_id")) for row in detections if isinstance(row, dict) and row.get("class_id") in allowed})
    return values or ["negative_or_unlabelled"]


def _read_state(output: Path, configuration: dict[str, Any]) -> dict[str, Any]:
    state_path = output / STATE_NAME
    if not output.exists():
        output.mkdir(parents=True)
        (output / "samples").mkdir()
        (output / "provenance").mkdir()
        state = {"schema_version": 1, "configuration": configuration, "records": []}
        atomic_write_json(state_path, state)
        return state
    if output.is_symlink() or not output.is_dir() or not state_path.is_file():
        raise CalibrationRejected("output_not_resumable_owned_corpus")
    state = _load_object(state_path, "materialization_state")
    if state.get("schema_version") != 1 or state.get("configuration") != configuration or not isinstance(state.get("records"), list):
        raise CalibrationRejected("resume_configuration_or_state_mismatch")
    for name in ("samples", "provenance"):
        if not (output / name).is_dir() or (output / name).is_symlink():
            raise CalibrationRejected("output_structure_invalid")
    if (output / "calibration_manifest.json").exists() or (output / CORPUS_MANIFEST_NAME).exists():
        raise CalibrationRejected("output_already_frozen_immutable")
    for index, row in enumerate(state["records"]):
        if not isinstance(row, dict):
            raise CalibrationRejected(f"resume_record_invalid:{index}")
        relative, expected_sha = row.get("relative_path"), row.get("sha256")
        if not isinstance(relative, str) or not isinstance(expected_sha, str):
            raise CalibrationRejected(f"resume_record_binding_invalid:{index}")
        sample = _relative(output, relative, "resume_sample")
        _regular(sample, "resume_sample")
        if sample.stat().st_size != row.get("byte_size") or sha256_file(sample) != expected_sha:
            raise CalibrationRejected(f"resume_sample_drift:{index}")
        provenance = _relative(output, str(row.get("provenance_relative_path", "")), "resume_provenance")
        value = _load_object(provenance, "resume_provenance")
        if value.get("source_rgb_sha256") != row.get("source_sha256"):
            raise CalibrationRejected(f"resume_provenance_drift:{index}")
    return state


def _write_tensor(path: Path, tensor: np.ndarray) -> str:
    pending = path.with_name(f".{path.name}.pending.{os.getpid()}")
    with pending.open("wb") as stream:
        np.save(stream, tensor, allow_pickle=False)
    os.replace(pending, path)
    return sha256_file(path)


def materialize(*, capture_root: Path, output: Path, scenario_id: str, contract_path: Path = DEFAULT_CONTRACT,
                oracle_receipt: Path | None = None, holdout_source_hashes: set[str] | None = None) -> Path:
    """Copy each unique product RGB frame once and record enough provenance to audit it."""
    if not scenario_id.strip():
        raise CalibrationRejected("scenario_id_required")
    if capture_root.is_symlink() or not capture_root.is_dir():
        raise CalibrationRejected("capture_root_not_regular_directory")
    contract = _load_contract(contract_path)
    oracle_status, oracle_identity = _verified_oracle(oracle_receipt)
    holdout = holdout_source_hashes or set()
    if any(not isinstance(value, str) or len(value) != 64 for value in holdout):
        raise CalibrationRejected("holdout_hash_invalid")
    configuration = {
        "capture_root": str(capture_root.resolve()), "scenario_id": scenario_id,
        "contract_sha256": sha256_file(contract_path), "oracle_status": oracle_status,
        "oracle_identity": oracle_identity, "holdout_source_sha256": sorted(holdout),
    }
    state = _read_state(output, configuration)
    records: list[dict[str, Any]] = state["records"]
    seen_sources = {row.get("source_sha256") for row in records if isinstance(row, dict)}
    seen_tensors = {row.get("sha256") for row in records if isinstance(row, dict)}
    allowed = set(contract["vocabulary"]["semantic_class_ids"])
    frames_root = capture_root / "frames"
    if frames_root.is_symlink() or not frames_root.is_dir():
        raise CalibrationRejected("capture_frames_root_missing")
    for frame in sorted(path for path in frames_root.iterdir() if path.name.startswith("frame-")):
        rgb, metadata, source_sha, frame_manifest_sha = _frame_bundle(frame)
        if source_sha in holdout:
            raise CalibrationRejected("evaluation_holdout_overlap")
        if source_sha in seen_sources:
            continue
        tensor = preprocess_dosod_rgb(rgb)
        index = len(records)
        relative = f"samples/frame_{index:06d}.npy"
        tensor_path = _relative(output, relative, "sample")
        tensor_sha = _write_tensor(tensor_path, tensor)
        if tensor_sha in seen_tensors:
            tensor_path.unlink()
            continue
        provenance_relative = f"provenance/frame_{index:06d}.json"
        provenance = {
            "source_capture_frame": str(frame.resolve()), "source_capture_manifest_sha256": frame_manifest_sha,
            "source_rgb_sha256": source_sha, "timestamp_s": metadata["rgb_stamp_s"],
            "camera_frame_id": metadata["camera_info"]["frame_id"], "scenario_id": scenario_id,
            "class_ids": _classes(metadata, allowed), "preprocessing_contract_sha256": canonical_sha256(contract["preprocessing"]),
            "preprocessing_evidence": {"status": oracle_status, "identity": oracle_identity},
        }
        atomic_write_json(_relative(output, provenance_relative, "provenance"), provenance)
        records.append({
            "relative_path": relative, "byte_size": tensor_path.stat().st_size, "sha256": tensor_sha,
            "source_sha256": source_sha, "source_role": "calibration_only", "provenance_relative_path": provenance_relative,
            "timestamp_s": metadata["rgb_stamp_s"], "camera_frame_id": metadata["camera_info"]["frame_id"],
            "scenario_id": scenario_id, "class_ids": provenance["class_ids"],
        })
        seen_sources.add(source_sha); seen_tensors.add(tensor_sha)
        state["records"] = records
        atomic_write_json(output / STATE_NAME, state)
    classes: dict[str, int] = {}
    scenarios: dict[str, int] = {}
    for row in records:
        scenarios[row["scenario_id"]] = scenarios.get(row["scenario_id"], 0) + 1
        for value in row["class_ids"]: classes[value] = classes.get(value, 0) + 1
    formal_blockers = []
    if oracle_status != "VERIFIED": formal_blockers.append("official_preprocessing_oracle_not_verified")
    if len(records) < contract["calibration"]["minimum_sample_count"]: formal_blockers.append("calibration_sample_count_below_500")
    if not holdout: formal_blockers.append("independent_validation_holdout_not_bound")
    receipt = {
        "schema_version": 1, "status": "BLOCKED" if formal_blockers else "FORMAL_CORPUS_READY_FOR_COMPILE_PREFLIGHT_ONLY",
        "formal_compile_ready": not formal_blockers, "compile_executed": False, "hbm_status": "HBM_NOT_PRODUCED",
        "record_count": len(records), "remaining_to_500": max(0, 500 - len(records)), "class_distribution": classes,
        "scenario_distribution": scenarios, "records_sha256": canonical_sha256(records), "blockers": formal_blockers,
        "resume_state": STATE_NAME, "no_synthetic_or_repeated_frames": True,
    }
    atomic_write_json(output / RECEIPT_NAME, receipt)
    if not formal_blockers:
        manifest = {
            "schema_version": 1, "dataset_id": "tzcup_s100p_product_capture_calibration_v1", "status": "FROZEN",
            "model_sha256": contract["model"]["sha256"], "vocabulary_sha256": contract["vocabulary"]["sha256"],
            "preprocessing_sha256": canonical_sha256(contract["preprocessing"]), "evaluation_holdout_source_sha256": sorted(holdout),
            "records": records, "records_sha256": canonical_sha256(records), "materialization_receipt_sha256": sha256_file(output / RECEIPT_NAME),
        }
        atomic_write_json(output / contract["calibration"]["manifest_name"], manifest)
        atomic_write_json(output / CORPUS_MANIFEST_NAME, manifest)
    return output / RECEIPT_NAME


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scenario-id", required=True); parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--oracle-receipt", type=Path); parser.add_argument("--holdout-source-sha256", action="append", default=[])
    args = parser.parse_args()
    try:
        receipt = materialize(capture_root=args.capture_root, output=args.output, scenario_id=args.scenario_id,
                              contract_path=args.contract, oracle_receipt=args.oracle_receipt,
                              holdout_source_hashes=set(args.holdout_source_sha256))
        print(json.dumps(_load_object(receipt, "receipt"), sort_keys=True))
    except CalibrationRejected as exc:
        print(f"BLOCKED:{exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
