"""Stage a frozen public collector dataset into a compiler-only calibration tree.

This module never launches ROS/Gazebo, compiles an HBM, or mutates the source
collector dataset.  Its output contains only canonical calibration tensors and
the compiler manifest, while retained holdout tensors remain in the source tree.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
from pathlib import Path
from typing import Any

from collect_formal_s100_calibration_frames import CalibrationRejected, atomic_write_json, canonical_sha256, sha256_file
from validate_dosod_s100p_hbm_compile_contract import audit_calibration

MIN_HOLDOUT_SAMPLES = 100


def _regular(path: Path) -> bool:
    return path.is_file() and not path.is_symlink()


def _relative(root: Path, value: object) -> Path:
    if not isinstance(value, str) or not value:
        raise CalibrationRejected("staging_record_path_invalid")
    relative = Path(value)
    if relative.is_absolute() or not relative.parts or any(part in {".", ".."} for part in relative.parts):
        raise CalibrationRejected("staging_record_path_escape")
    target = root
    for part in relative.parts:
        target = target / part
        if target.is_symlink():
            raise CalibrationRejected("staging_record_path_symlink")
    return target


def _load_json(path: Path, reason: str) -> dict[str, Any]:
    if not _regular(path):
        raise CalibrationRejected(reason)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CalibrationRejected(reason) from exc
    if not isinstance(value, dict):
        raise CalibrationRejected(reason)
    return value


def _validate_source(source: Path, contract: dict[str, Any]) -> tuple[dict[str, Any], Path]:
    manifest_path = source / str(contract["calibration"]["manifest_name"])
    manifest = _load_json(manifest_path, "source_manifest_invalid")
    if manifest.get("schema_version") != 1 or manifest.get("status") != "FROZEN":
        raise CalibrationRejected("source_manifest_not_frozen")
    if manifest.get("source_domain") != "public_gazebo_sensor" or manifest.get("formal_passed") is not False:
        raise CalibrationRejected("source_manifest_not_public_nonformal")
    if manifest.get("preprocessing_sha256") != canonical_sha256(contract["preprocessing"]):
        raise CalibrationRejected("source_manifest_preprocessing_mismatch")
    records, holdouts = manifest.get("records"), manifest.get("holdout_records")
    if not isinstance(records, list) or len(records) < contract["calibration"]["minimum_sample_count"]:
        raise CalibrationRejected("source_calibration_below_minimum")
    if not isinstance(holdouts, list) or len(holdouts) < MIN_HOLDOUT_SAMPLES:
        raise CalibrationRejected("source_holdout_below_minimum")
    calibration_scenes = {row.get("scene_id") for row in records if isinstance(row, dict)}
    holdout_scenes = {row.get("scene_id") for row in holdouts if isinstance(row, dict)}
    if None in calibration_scenes or None in holdout_scenes or calibration_scenes & holdout_scenes:
        raise CalibrationRejected("source_scene_sets_not_disjoint")
    holdout_sources = {row.get("source_sha256") for row in holdouts if isinstance(row, dict)}
    declared_holdout_sources = manifest.get("evaluation_holdout_source_sha256")
    if not isinstance(declared_holdout_sources, list) or not all(isinstance(value, str) and len(value) == 64 for value in declared_holdout_sources):
        raise CalibrationRejected("source_holdout_hash_invalid")
    if not all(isinstance(value, str) and len(value) == 64 for value in holdout_sources):
        raise CalibrationRejected("source_holdout_hash_invalid")
    if set(declared_holdout_sources) != holdout_sources:
        raise CalibrationRejected("source_holdout_hash_binding_invalid")
    for row in [*records, *holdouts]:
        if not isinstance(row, dict):
            raise CalibrationRejected("source_record_invalid")
        path = _relative(source, row.get("relative_path"))
        if not _regular(path) or sha256_file(path) != row.get("sha256") or path.stat().st_size != row.get("byte_size"):
            raise CalibrationRejected("source_tensor_binding_invalid")
    return manifest, manifest_path


def stage_dataset(*, source: Path, output: Path, contract_path: Path) -> Path:
    """Copy only registered calibration tensors and rebind them to the contract."""
    source_resolved = source.resolve()
    output_resolved = output.resolve(strict=False)
    try:
        output_resolved.relative_to(source_resolved)
    except ValueError:
        try:
            source_resolved.relative_to(output_resolved)
        except ValueError:
            pass
        else:
            raise CalibrationRejected("source_and_output_overlap")
    else:
        raise CalibrationRejected("source_and_output_overlap")
    if source.is_symlink() or not source.is_dir() or output.is_symlink() or output.exists():
        raise CalibrationRejected("source_or_output_not_fresh_regular_directory")
    contract = _load_json(contract_path, "contract_invalid")
    source_manifest, source_manifest_path = _validate_source(source, contract)
    output.mkdir(parents=True)
    staged_records: list[dict[str, Any]] = []
    for row in source_manifest["records"]:
        source_tensor = _relative(source, row["relative_path"])
        relative = Path(str(row["relative_path"]))
        target = output / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        pending = target.with_name(f".{target.name}.pending.{os.getpid()}")
        shutil.copyfile(source_tensor, pending)
        os.replace(pending, target)
        if sha256_file(target) != row["sha256"] or target.stat().st_size != row["byte_size"]:
            raise CalibrationRejected("staged_tensor_binding_invalid")
        staged_records.append(dict(row))
    manifest = {
        "schema_version": 1,
        "status": "FROZEN",
        "model_sha256": contract["model"]["sha256"],
        "vocabulary_sha256": contract["vocabulary"]["sha256"],
        "preprocessing_sha256": canonical_sha256(contract["preprocessing"]),
        "records": staged_records,
        "records_sha256": canonical_sha256(staged_records),
        "evaluation_holdout_source_sha256": source_manifest["evaluation_holdout_source_sha256"],
        "source_dataset_manifest_sha256": sha256_file(source_manifest_path),
        "source_dataset_id": source_manifest.get("dataset_id"),
        "staging_status": "COMPILER_ONLY_STAGED_NOT_COMPILED",
    }
    destination_manifest = output / str(contract["calibration"]["manifest_name"])
    atomic_write_json(destination_manifest, manifest)
    blockers: list[str] = []
    audit_calibration(output, contract, blockers)
    if blockers:
        raise CalibrationRejected("staged_calibration_reaudit_failed:" + ",".join(sorted(blockers)))
    return destination_manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--contract", type=Path, default=Path("config/dosod_s100p_hbm_compile_contract.json"))
    args = parser.parse_args()
    try:
        manifest = stage_dataset(source=args.source_dataset, output=args.output, contract_path=args.contract)
    except CalibrationRejected as exc:
        print(f"BLOCKED:{exc}")
        return 2
    print(json.dumps({"status": "COMPILER_ONLY_STAGED_NOT_COMPILED", "manifest": str(manifest.resolve()), "compile_executed": False, "hbm_status": "HBM_NOT_PRODUCED"}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
