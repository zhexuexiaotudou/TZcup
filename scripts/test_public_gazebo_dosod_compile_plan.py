"""Pure/static guards for the public-plan to compiler-handoff specification."""
from __future__ import annotations

import copy
import importlib.util
import json
import re
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def load_module(name: str, filename: str):
    spec = importlib.util.spec_from_file_location(name, HERE / filename)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


collector = load_module("public_gazebo_dosod_calibration_for_compile_plan", "public_gazebo_dosod_calibration.py")
validator = load_module("validate_dosod_s100p_hbm_compile_contract_for_compile_plan", "validate_dosod_s100p_hbm_compile_contract.py")
stager = load_module("stage_public_gazebo_dosod_calibration_for_compile_plan", "stage_public_gazebo_dosod_calibration_for_compile.py")


def test_public_train_plan_is_disjoint_and_has_exact_capacity():
    plan = json.loads((ROOT / "config/public_gazebo_dosod_train_scene_plan.json").read_text(encoding="utf-8"))
    contract = json.loads((ROOT / "config/dosod_s100p_hbm_compile_contract.json").read_text(encoding="utf-8"))
    loaded = collector.load_scene_plan(ROOT / "config/public_gazebo_dosod_train_scene_plan.json")
    quota = plan["execution"]["per_scene_quota"]
    collector.require_collection_capacity(plan=loaded, contract=contract, per_scene_quota=quota)
    assert len(loaded["scene_groups"]["calibration"]) == 20
    assert len(loaded["scene_groups"]["holdout"]) == 4
    assert quota * 20 == contract["calibration"]["minimum_sample_count"] == 500
    assert quota * 4 == collector.MIN_HOLDOUT_SAMPLES == 100
    assert set(loaded["scene_groups"]["calibration"]).isdisjoint(loaded["scene_groups"]["holdout"])
    for scene_id in loaded["scene_groups"]["calibration"] + loaded["scene_groups"]["holdout"]:
        match = re.fullmatch(r"map-(\d+)-mission-(\d+)", scene_id)
        assert match is not None
        assert int(match.group(1)) < 32 and int(match.group(2)) < 200


def test_stager_separates_retained_holdout_from_canonical_compiler_audit(tmp_path, monkeypatch):
    """Source remains intact; only its calibration tensors enter compiler staging."""
    contract = copy.deepcopy(json.loads((ROOT / "config/dosod_s100p_hbm_compile_contract.json").read_text(encoding="utf-8")))
    contract["calibration"].update({"minimum_sample_count": 1, "shape": [1, 3, 1, 1]})
    source = tmp_path / "dataset"
    (source / "samples").mkdir(parents=True)
    (source / "holdout_samples").mkdir()
    tensor = np.zeros((1, 3, 1, 1), dtype=np.float32)
    sample = source / "samples/000000.npy"
    holdout = source / "holdout_samples/holdout_000000.npy"
    np.save(sample, tensor, allow_pickle=False)
    np.save(holdout, tensor + 1, allow_pickle=False)
    record = {
        "relative_path": "samples/000000.npy", "byte_size": sample.stat().st_size,
        "sha256": validator.sha256_file(sample), "source_sha256": "a" * 64,
        "source_role": "calibration_only", "scene_id": "map-0-mission-0",
    }
    source_manifest = {
        "schema_version": 1, "status": "FROZEN", "source_domain": "public_gazebo_sensor", "formal_passed": False,
        "preprocessing_sha256": validator.canonical_sha256(contract["preprocessing"]),
        "records": [record], "evaluation_holdout_source_sha256": ["b" * 64],
        "holdout_records": [{"relative_path": "holdout_samples/holdout_000000.npy", "byte_size": holdout.stat().st_size,
                             "sha256": validator.sha256_file(holdout), "source_sha256": "b" * 64,
                             "scene_id": "map-1-mission-0"}],
    }
    (source / "calibration_manifest.json").write_text(json.dumps(source_manifest), encoding="utf-8")
    monkeypatch.setattr(stager, "MIN_HOLDOUT_SAMPLES", 1)
    blockers: list[str] = []
    validator.audit_calibration(source, contract, blockers)
    assert "calibration_manifest_model_sha256_mismatch" in blockers
    assert "calibration_manifest_vocabulary_sha256_mismatch" in blockers
    assert "calibration_manifest_records_sha256_mismatch" in blockers
    assert "calibration_directory_manifest_set_mismatch" in blockers
    contract_path = tmp_path / "contract.json"
    contract_path.write_text(json.dumps(contract), encoding="utf-8")
    sample_sha_before = validator.sha256_file(sample)
    for index, unsafe_relative in enumerate((str(sample.resolve()), "../dataset/samples/000000.npy")):
        unsafe_manifest = copy.deepcopy(source_manifest)
        unsafe_manifest["records"][0]["relative_path"] = unsafe_relative
        (source / "calibration_manifest.json").write_text(json.dumps(unsafe_manifest), encoding="utf-8")
        unsafe_output = tmp_path / f"unsafe_compiler_only_{index}"
        with pytest.raises(stager.CalibrationRejected, match="staging_record_path_escape"):
            stager.stage_dataset(source=source, output=unsafe_output, contract_path=contract_path)
        assert not unsafe_output.exists()
        assert validator.sha256_file(sample) == sample_sha_before
    (source / "calibration_manifest.json").write_text(json.dumps(source_manifest), encoding="utf-8")
    with pytest.raises(stager.CalibrationRejected, match="source_and_output_overlap"):
        stager.stage_dataset(source=source, output=source / "compiler_only", contract_path=contract_path)
    staged = tmp_path / "compiler_only"
    manifest_path = stager.stage_dataset(source=source, output=staged, contract_path=contract_path)
    assert manifest_path.is_file() and holdout.is_file()
    assert not (staged / "holdout_samples").exists()
    restaged_blockers: list[str] = []
    validator.audit_calibration(staged, contract, restaged_blockers)
    assert restaged_blockers == []
    source_manifest["evaluation_holdout_source_sha256"] = [123]
    (source / "calibration_manifest.json").write_text(json.dumps(source_manifest), encoding="utf-8")
    with pytest.raises(stager.CalibrationRejected, match="source_holdout_hash_invalid"):
        stager.stage_dataset(source=source, output=tmp_path / "bad_compiler_only", contract_path=contract_path)


def test_handoff_spec_refuses_metadata_only_visibility_claims():
    text = (ROOT / "docs/public-gazebo-dosod-compile-plan.md").read_text(encoding="utf-8")
    assert "not visibility evidence" in text
    assert "visible-frame\ncounts" in text
    assert "No evaluator, ground-truth, hidden" in text
    assert "Sensor noise or\ntimestamp-only changes" in text
    assert "does authorize the later protected remote verification/compilation" in text
