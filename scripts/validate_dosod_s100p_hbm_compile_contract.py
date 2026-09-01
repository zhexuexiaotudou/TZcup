#!/usr/bin/env python3
"""Fail-closed, low-memory audit for the four-class DOSOD S100P compile inputs.

This validator never invokes ``hb_compile`` and never creates an HBM.  It
binds the exact ONNX, vocabulary, reparameterized inputs, upstream recipe,
calibration manifest, and official toolchain evidence before the separate
ONNX/toolchain preflight is allowed to emit a compiler YAML.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


CONTRACT_ID = "tzcup_dosod_s100p_four_class_hbm_compile_v1"
BLOCKED_STATUS = "BLOCKED_UNTIL_FROZEN_CALIBRATION_AND_LIVE_COMPILER"
OPERATION_BOUNDARY = (
    "preflight_only_no_hbm_claim_board_copy_ssh_node_start_or_data_collection"
)
EXPECTED_CLASS_IDS = ["litter_cube", "fallen_leaves", "dust_or_soil", "puddle"]
EXPECTED_EMITTED_LABELS = ["small litter cube", "fallen leaves", "dust patch", "puddle"]
EXPECTED_MODEL_SHA256 = "30e4da2516b7a18cc3dbb4b20572e99f07c28a0c08111055a8c14265a992e516"
EXPECTED_VOCABULARY_SHA256 = "c5b10ba0e26ee28cdbf5192775e7d2ddb3f5852e515f59a074b38b7ed69d7ffd"
EXPECTED_OUTPUT_RELATIVE_PATH = "dosod/dosod_mlp3x_s_tzcup_rep-int16.hbm"
EXPECTED_TOOLCHAIN_VERSIONS = {
    "hbdk4_compiler": "4.7.5",
    "hmct": "2.6.5",
    "horizon_tc_ui": "3.5.3",
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def resolve_relative(root: Path, relative: str) -> Path:
    candidate = Path(relative)
    if candidate.is_absolute() or not relative or "\\" in relative:
        raise ValueError("relative path must be non-empty POSIX syntax")
    resolved_root = root.resolve()
    resolved = (resolved_root / candidate).resolve()
    if not resolved.is_relative_to(resolved_root):
        raise ValueError("relative path escapes its declared root")
    return resolved


def _block(blockers: list[str], code: str) -> None:
    if code not in blockers:
        blockers.append(code)


def validate_contract_shape(contract: Any, blockers: list[str]) -> None:
    if not isinstance(contract, dict):
        _block(blockers, "contract_not_object")
        return
    exact = {
        "schema_version": 1,
        "contract_id": CONTRACT_ID,
        "status": BLOCKED_STATUS,
        "operation_boundary": OPERATION_BOUNDARY,
    }
    for key, expected in exact.items():
        if contract.get(key) != expected:
            _block(blockers, f"contract_field_mismatch:{key}")
    required = {
        "model",
        "vocabulary",
        "reparameterization",
        "upstream",
        "preprocessing",
        "calibration",
        "toolchain",
        "compile_recipe",
        "output",
    }
    for key in sorted(required - set(contract)):
        _block(blockers, f"contract_field_missing:{key}")
    if blockers:
        return
    model = contract["model"]
    vocabulary = contract["vocabulary"]
    recipe = contract["compile_recipe"]
    output = contract["output"]
    if model.get("sha256") != EXPECTED_MODEL_SHA256:
        _block(blockers, "contract_model_sha256_not_frozen")
    if model.get("inputs") != [
        {"name": "images", "dtype": "FLOAT", "shape": [1, 3, 640, 640]}
    ]:
        _block(blockers, "contract_model_input_signature_mismatch")
    if model.get("outputs") != [
        {"name": "scores", "dtype": "FLOAT", "shape": [1, 8400, 4]},
        {"name": "boxes", "dtype": "FLOAT", "shape": [1, 8400, 4]},
    ]:
        _block(blockers, "contract_model_output_signature_mismatch")
    if model.get("opset") != 11 or model.get("ir_version") != 6:
        _block(blockers, "contract_model_onnx_version_mismatch")
    if vocabulary.get("sha256") != EXPECTED_VOCABULARY_SHA256:
        _block(blockers, "contract_vocabulary_sha256_not_frozen")
    if vocabulary.get("semantic_class_ids") != EXPECTED_CLASS_IDS:
        _block(blockers, "contract_semantic_class_order_mismatch")
    if vocabulary.get("emitted_labels") != EXPECTED_EMITTED_LABELS:
        _block(blockers, "contract_emitted_label_order_mismatch")
    recipe_exact = {
        "march": "nash-m",
        "input_name": "images",
        "input_type_train": "rgb",
        "input_layout_train": "NCHW",
        "input_shape": "1x3x640x640",
        "input_batch": 1,
        "norm_type": "data_scale",
        "scale_value": 0.003921568627451,
        "input_layout_rt": "NHWC",
        "input_type_rt": "nv12",
        "cal_data_type": "float32",
        "preprocess_on": False,
        "calibration_type": "max",
        "max_percentile": 0.99995,
        "optimization": "set_all_nodes_int16;",
        "compile_mode": "latency",
        "optimize_level": "O2",
        "output_model_file_prefix": "dosod_mlp3x_s_tzcup_rep-int16",
    }
    for key, expected in recipe_exact.items():
        if recipe.get(key) != expected:
            _block(blockers, f"contract_compile_recipe_mismatch:{key}")
    if recipe.get("jobs_allowed_range") != [1, 32]:
        _block(blockers, "contract_compile_recipe_mismatch:jobs_allowed_range")
    if output.get("relative_path") != EXPECTED_OUTPUT_RELATIVE_PATH:
        _block(blockers, "contract_output_path_mismatch")
    if output.get("status") != "HBM_NOT_PRODUCED":
        _block(blockers, "contract_output_status_must_be_hbm_not_produced")


def audit_declared_file(
    root: Path,
    row: Any,
    label: str,
    blockers: list[str],
) -> Path | None:
    if not isinstance(row, dict):
        _block(blockers, f"declared_file_invalid:{label}")
        return None
    relative = row.get("relative_path")
    expected_size = row.get("byte_size")
    expected_sha = row.get("sha256")
    if not isinstance(relative, str):
        _block(blockers, f"declared_file_path_invalid:{label}")
        return None
    try:
        path = resolve_relative(root, relative)
    except ValueError:
        _block(blockers, f"declared_file_path_escape:{label}")
        return None
    if not path.is_file():
        _block(blockers, f"declared_file_missing:{label}")
        return path
    if path.is_symlink():
        _block(blockers, f"declared_file_symlink:{label}")
        return path
    if not isinstance(expected_size, int) or path.stat().st_size != expected_size:
        _block(blockers, f"declared_file_size_mismatch:{label}")
    if not isinstance(expected_sha, str) or sha256_file(path) != expected_sha:
        _block(blockers, f"declared_file_sha256_mismatch:{label}")
    return path


def audit_vocabulary(
    vocabulary_path: Path | None,
    vocabulary_contract: dict[str, Any],
    blockers: list[str],
) -> None:
    if vocabulary_path is None or not vocabulary_path.is_file():
        return
    try:
        vocabulary = load_json(vocabulary_path)
    except Exception as exc:
        _block(blockers, f"vocabulary_unreadable:{type(exc).__name__}")
        return
    if vocabulary != vocabulary_contract.get("groups"):
        _block(blockers, "vocabulary_groups_mismatch")
        return
    if [group[0] for group in vocabulary] != EXPECTED_EMITTED_LABELS:
        _block(blockers, "vocabulary_first_label_order_mismatch")


def audit_numpy_array(
    path: Path | None,
    expected_dtype: str,
    expected_shape: list[int],
    label: str,
    blockers: list[str],
    value_range: list[float] | None = None,
) -> None:
    if path is None or not path.is_file():
        return
    try:
        import numpy as np
    except Exception as exc:
        _block(blockers, f"numpy_unavailable:{type(exc).__name__}")
        return
    try:
        array = np.load(path, mmap_mode="r", allow_pickle=False)
    except Exception as exc:
        _block(blockers, f"npy_unreadable:{label}:{type(exc).__name__}")
        return
    if str(array.dtype) != expected_dtype:
        _block(blockers, f"npy_dtype_mismatch:{label}")
    if list(array.shape) != expected_shape:
        _block(blockers, f"npy_shape_mismatch:{label}")
    if value_range is None or list(array.shape) != expected_shape:
        return
    low, high = map(float, value_range)
    flat = array.reshape(-1)
    for start in range(0, flat.size, 262144):
        chunk = np.asarray(flat[start : start + 262144])
        if not np.isfinite(chunk).all():
            _block(blockers, f"npy_nonfinite:{label}")
            return
        if chunk.size and (float(chunk.min()) < low or float(chunk.max()) > high):
            _block(blockers, f"npy_value_range_mismatch:{label}")
            return


def audit_artifact_manifest(
    manifest_path: Path,
    contract: dict[str, Any],
    blockers: list[str],
) -> None:
    if not manifest_path.is_file():
        _block(blockers, "artifact_manifest_missing")
        return
    try:
        manifest = load_json(manifest_path)
    except Exception as exc:
        _block(blockers, f"artifact_manifest_unreadable:{type(exc).__name__}")
        return
    rows = manifest.get("artifacts") if isinstance(manifest, dict) else None
    if not isinstance(rows, dict):
        _block(blockers, "artifact_manifest_rows_invalid")
        return
    for key in ("model", "vocabulary"):
        expected = contract[key]
        relative = expected["relative_path"]
        row = rows.get(relative)
        if not isinstance(row, dict):
            _block(blockers, f"artifact_manifest_row_missing:{key}")
            continue
        for field in ("sha256", "byte_size"):
            if row.get(field) != expected.get(field):
                _block(blockers, f"artifact_manifest_row_mismatch:{key}:{field}")
        if key == "model":
            for field in ("source_revision", "model_role"):
                if row.get(field) != expected.get(field):
                    _block(blockers, f"artifact_manifest_row_mismatch:{key}:{field}")
        else:
            if row.get("semantic_class_ids") != EXPECTED_CLASS_IDS:
                _block(blockers, "artifact_manifest_vocabulary_class_order_mismatch")
            if row.get("emitted_labels") != EXPECTED_EMITTED_LABELS:
                _block(blockers, "artifact_manifest_vocabulary_label_order_mismatch")


def audit_toolchain(
    repository_root: Path,
    toolchain_contract: dict[str, Any],
    compiler_identity_path: Path | None,
    blockers: list[str],
) -> dict[str, Any]:
    discovery_path = audit_declared_file(
        repository_root,
        toolchain_contract.get("discovery_report"),
        "toolchain_discovery",
        blockers,
    )
    audit_declared_file(
        repository_root,
        toolchain_contract.get("hb_compile_help"),
        "hb_compile_help",
        blockers,
    )
    discovery: dict[str, Any] = {}
    if discovery_path and discovery_path.is_file():
        try:
            loaded = load_json(discovery_path)
            discovery = loaded if isinstance(loaded, dict) else {}
        except Exception as exc:
            _block(blockers, f"toolchain_discovery_unreadable:{type(exc).__name__}")
        source = discovery.get("official_source", {})
        if source.get("oe_version") != toolchain_contract.get("oe_version"):
            _block(blockers, "toolchain_oe_version_mismatch")
        if source.get("archive_sha256") != toolchain_contract.get("archive_sha256"):
            _block(blockers, "toolchain_archive_sha256_mismatch")
        if discovery.get("required_versions") != EXPECTED_TOOLCHAIN_VERSIONS:
            _block(blockers, "toolchain_required_versions_mismatch")
        if discovery.get("official_toolchain_package_ready") is not True:
            _block(blockers, "toolchain_package_not_ready")
    if toolchain_contract.get("live_compiler_identity_required_at_compile_time") is not True:
        _block(blockers, "contract_live_compiler_identity_not_required")
    if compiler_identity_path is None:
        _block(blockers, "live_compiler_identity_missing")
    elif not compiler_identity_path.is_file():
        _block(blockers, "live_compiler_identity_missing")
    else:
        try:
            identity = load_json(compiler_identity_path)
        except Exception as exc:
            _block(blockers, f"live_compiler_identity_unreadable:{type(exc).__name__}")
        else:
            if not isinstance(identity, dict) or identity.get("identity_verified") is not True:
                _block(blockers, "live_compiler_identity_not_verified")
            if identity.get("oe_version") != toolchain_contract.get("oe_version"):
                _block(blockers, "live_compiler_identity_oe_version_mismatch")
            if identity.get("required_versions") != EXPECTED_TOOLCHAIN_VERSIONS:
                _block(blockers, "live_compiler_identity_versions_mismatch")
            for field in ("hb_compile_executable_sha256", "hb_compile_probe_output_sha256"):
                value = identity.get(field)
                if not isinstance(value, str) or len(value) != 64:
                    _block(blockers, f"live_compiler_identity_field_invalid:{field}")
    return discovery


def audit_calibration(
    calibration_dir: Path,
    contract: dict[str, Any],
    blockers: list[str],
) -> dict[str, Any]:
    calibration = contract["calibration"]
    if not calibration_dir.is_dir():
        _block(blockers, "calibration_directory_missing")
        return {"manifest_sha256": None, "sample_count": 0, "records_sha256": None}
    manifest_path = calibration_dir / str(calibration.get("manifest_name"))
    if not manifest_path.is_file():
        _block(blockers, "calibration_manifest_missing")
        return {"manifest_sha256": None, "sample_count": 0, "records_sha256": None}
    try:
        manifest = load_json(manifest_path)
    except Exception as exc:
        _block(blockers, f"calibration_manifest_unreadable:{type(exc).__name__}")
        return {"manifest_sha256": None, "sample_count": 0, "records_sha256": None}
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1:
        _block(blockers, "calibration_manifest_schema_invalid")
        return {"manifest_sha256": sha256_file(manifest_path), "sample_count": 0, "records_sha256": None}
    if manifest.get("status") != "FROZEN":
        _block(blockers, "calibration_manifest_not_frozen")
    if manifest.get("model_sha256") != contract["model"]["sha256"]:
        _block(blockers, "calibration_manifest_model_sha256_mismatch")
    if manifest.get("vocabulary_sha256") != contract["vocabulary"]["sha256"]:
        _block(blockers, "calibration_manifest_vocabulary_sha256_mismatch")
    if manifest.get("preprocessing_sha256") != canonical_sha256(contract["preprocessing"]):
        _block(blockers, "calibration_manifest_preprocessing_sha256_mismatch")
    records = manifest.get("records")
    if not isinstance(records, list):
        _block(blockers, "calibration_manifest_records_invalid")
        records = []
    minimum = calibration.get("minimum_sample_count")
    if not isinstance(minimum, int) or len(records) < minimum:
        _block(blockers, "calibration_sample_count_below_minimum")
    expected_paths: set[str] = set()
    tensor_hashes: set[str] = set()
    source_hashes: set[str] = set()
    holdout_hashes = set(manifest.get("evaluation_holdout_source_sha256", []))
    for index, row in enumerate(records):
        label = f"sample_{index}"
        if not isinstance(row, dict):
            _block(blockers, f"calibration_record_invalid:{label}")
            continue
        relative = row.get("relative_path")
        if not isinstance(relative, str) or not relative.endswith(calibration.get("sample_suffix", ".npy")):
            _block(blockers, f"calibration_record_path_invalid:{label}")
            continue
        try:
            sample_path = resolve_relative(calibration_dir, relative)
        except ValueError:
            _block(blockers, f"calibration_record_path_escape:{label}")
            continue
        if relative in expected_paths:
            _block(blockers, "calibration_duplicate_relative_path")
        expected_paths.add(relative)
        if sample_path.is_symlink():
            _block(blockers, f"calibration_sample_symlink:{relative}")
        if not sample_path.is_file():
            _block(blockers, f"calibration_sample_missing:{relative}")
            continue
        if sample_path.stat().st_size != row.get("byte_size"):
            _block(blockers, f"calibration_sample_size_mismatch:{relative}")
        actual_sha = sha256_file(sample_path)
        if actual_sha != row.get("sha256"):
            _block(blockers, f"calibration_sample_sha256_mismatch:{relative}")
        if actual_sha in tensor_hashes:
            _block(blockers, "calibration_duplicate_tensor_sha256")
        tensor_hashes.add(actual_sha)
        source_sha = row.get("source_sha256")
        if not isinstance(source_sha, str) or len(source_sha) != 64:
            _block(blockers, f"calibration_source_sha256_invalid:{relative}")
        else:
            if source_sha in source_hashes:
                _block(blockers, "calibration_duplicate_source_sha256")
            source_hashes.add(source_sha)
            if source_sha in holdout_hashes:
                _block(blockers, "calibration_evaluation_holdout_overlap")
        if row.get("source_role") != "calibration_only":
            _block(blockers, f"calibration_source_role_invalid:{relative}")
        audit_numpy_array(
            sample_path,
            str(calibration.get("dtype")),
            list(calibration.get("shape", [])),
            relative,
            blockers,
            list(calibration.get("value_range", [])),
        )
    actual_paths = {
        path.relative_to(calibration_dir).as_posix()
        for path in calibration_dir.rglob(f"*{calibration.get('sample_suffix', '.npy')}")
        if path.is_file()
    }
    if actual_paths != expected_paths:
        _block(blockers, "calibration_directory_manifest_set_mismatch")
    records_sha = canonical_sha256(records) if records else None
    if manifest.get("records_sha256") != records_sha:
        _block(blockers, "calibration_manifest_records_sha256_mismatch")
    return {
        "manifest_sha256": sha256_file(manifest_path),
        "sample_count": len(records),
        "records_sha256": records_sha,
    }


def audit_compile_inputs(
    contract_path: Path,
    repository_root: Path,
    artifact_root: Path,
    upstream_root: Path,
    calibration_dir: Path,
    compiler_identity_path: Path | None,
) -> dict[str, Any]:
    blockers: list[str] = []
    try:
        contract = load_json(contract_path)
    except Exception as exc:
        return {
            "schema_version": 1,
            "report_id": "tzcup_dosod_s100p_hbm_compile_input_audit_v1",
            "status": "BLOCKED",
            "blockers": [f"contract_unreadable:{type(exc).__name__}"],
            "hbm_status": "HBM_NOT_PRODUCED",
            "compile_plan_sha256": None,
        }
    validate_contract_shape(contract, blockers)
    if not isinstance(contract, dict):
        contract = {}
    model_path = None
    vocabulary_path = None
    embedding_path = None
    if not blockers:
        model_path = audit_declared_file(artifact_root, contract["model"], "model", blockers)
        vocabulary_path = audit_declared_file(
            artifact_root, contract["vocabulary"], "vocabulary", blockers
        )
        embedding_path = audit_declared_file(
            artifact_root,
            contract["reparameterization"].get("embedding"),
            "reparameterization_embedding",
            blockers,
        )
        audit_declared_file(
            artifact_root,
            contract["reparameterization"].get("checkpoint"),
            "reparameterization_checkpoint",
            blockers,
        )
        for index, row in enumerate(contract["upstream"].get("files", [])):
            audit_declared_file(upstream_root, row, f"upstream_{index}", blockers)
        audit_vocabulary(vocabulary_path, contract["vocabulary"], blockers)
        embedding = contract["reparameterization"]["embedding"]
        audit_numpy_array(
            embedding_path,
            embedding["dtype"],
            embedding["shape"],
            "reparameterization_embedding",
            blockers,
        )
        audit_artifact_manifest(artifact_root / "artifact_manifest.json", contract, blockers)
        toolchain_discovery = audit_toolchain(
            repository_root,
            contract["toolchain"],
            compiler_identity_path,
            blockers,
        )
        calibration = audit_calibration(calibration_dir, contract, blockers)
    else:
        toolchain_discovery = {}
        calibration = {"manifest_sha256": None, "sample_count": 0, "records_sha256": None}
    plan_payload = None
    plan_sha = None
    if not blockers:
        plan_payload = {
            "contract_sha256": sha256_file(contract_path),
            "model_sha256": contract["model"]["sha256"],
            "vocabulary_sha256": contract["vocabulary"]["sha256"],
            "calibration_manifest_sha256": calibration["manifest_sha256"],
            "calibration_records_sha256": calibration["records_sha256"],
            "toolchain_discovery_sha256": contract["toolchain"]["discovery_report"]["sha256"],
            "compile_recipe": contract["compile_recipe"],
            "expected_output": contract["output"]["relative_path"],
        }
        plan_sha = canonical_sha256(plan_payload)
    return {
        "schema_version": 1,
        "report_id": "tzcup_dosod_s100p_hbm_compile_input_audit_v1",
        "status": "READY_FOR_ONNX_TOOLCHAIN_PREFLIGHT" if not blockers else "BLOCKED",
        "operation_boundary": OPERATION_BOUNDARY,
        "contract_path": str(contract_path.resolve()),
        "contract_sha256": sha256_file(contract_path) if contract_path.is_file() else None,
        "artifact_root": str(artifact_root.resolve()),
        "upstream_root": str(upstream_root.resolve()),
        "calibration_dir": str(calibration_dir.resolve()),
        "model_path": str(model_path) if model_path else None,
        "vocabulary_path": str(vocabulary_path) if vocabulary_path else None,
        "calibration": calibration,
        "toolchain_package_ready": toolchain_discovery.get("official_toolchain_package_ready") is True,
        "blockers": blockers,
        "compile_plan": plan_payload,
        "compile_plan_sha256": plan_sha,
        "hbm_status": "HBM_NOT_PRODUCED",
        "compile_executed": False,
        "board_runtime_accepted": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--contract",
        default="config/dosod_s100p_hbm_compile_contract.json",
    )
    parser.add_argument("--repository-root", default=".")
    parser.add_argument("--artifact-root", required=True)
    parser.add_argument("--upstream-root", required=True)
    parser.add_argument("--calibration-dir", required=True)
    parser.add_argument("--compiler-identity")
    parser.add_argument("--allow-blocked-exit-zero", action="store_true")
    args = parser.parse_args()
    report = audit_compile_inputs(
        Path(args.contract),
        Path(args.repository_root),
        Path(args.artifact_root),
        Path(args.upstream_root),
        Path(args.calibration_dir),
        Path(args.compiler_identity) if args.compiler_identity else None,
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if report["status"] == "READY_FOR_ONNX_TOOLCHAIN_PREFLIGHT":
        return 0
    return 0 if args.allow_blocked_exit_zero else 2


if __name__ == "__main__":
    raise SystemExit(main())
