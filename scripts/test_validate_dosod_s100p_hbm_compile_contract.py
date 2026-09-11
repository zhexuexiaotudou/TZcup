from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

import validate_dosod_s100p_hbm_compile_contract as subject


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
REAL_CONTRACT = REPOSITORY_ROOT / "config" / "dosod_s100p_hbm_compile_contract.json"
EXPECTED_UPSTREAM_FILES = [
    {
        "relative_path": "configs/dosod/rep_dosod_mlp3x_s_100e_1x8gpus_obj365v1_goldg_train_lvis_minival.py",
        "byte_size": 719,
        "sha256": "2b0d9be4b250e322413d590bbd15465efcb52cfbfe0819c857ef9da680750db7",
    },
    {
        "relative_path": "deploy/export_onnx.py",
        "byte_size": 7959,
        "sha256": "b7e97403f8828471b28a4dc94e0e4805bd561288f96c0dfa32f0e59b89d9c70b",
    },
    {
        "relative_path": "ai_toolchain/s100/con_DOSOD_S.yaml",
        "byte_size": 874,
        "sha256": "5d1248c8e0afb43c5ab2beac41ef67dffe04e7c48b54fb66fbcd357962a574a0",
    },
    {
        "relative_path": "ai_toolchain/s100/gen_calibration_data_s100.py",
        "byte_size": 1620,
        "sha256": "f876c3b38effb66d78812db8ccbe0f69b794e08408f23a2ad9e953e3c2724dda",
    },
]


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_bytes(root: Path, row: dict, payload: bytes) -> Path:
    path = root / row["relative_path"]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)
    row["byte_size"] = len(payload)
    row["sha256"] = _sha(path)
    return path


def _build_ready_fixture(
    tmp_path: Path, monkeypatch
) -> tuple[Path, Path, Path, Path, Path, Path]:
    repository = tmp_path / "repository"
    artifacts = tmp_path / "artifacts"
    upstream = tmp_path / "upstream"
    calibration = tmp_path / "calibration"
    for path in (repository, artifacts, upstream, calibration):
        path.mkdir(parents=True)

    contract = copy.deepcopy(json.loads(REAL_CONTRACT.read_text(encoding="utf-8")))
    model_path = _write_bytes(artifacts, contract["model"], b"four-class-onnx-fixture")
    monkeypatch.setattr(subject, "EXPECTED_MODEL_SHA256", contract["model"]["sha256"])

    vocabulary_path = artifacts / contract["vocabulary"]["relative_path"]
    vocabulary_path.parent.mkdir(parents=True, exist_ok=True)
    vocabulary_path.write_text(
        json.dumps(contract["vocabulary"]["groups"], ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    contract["vocabulary"]["byte_size"] = vocabulary_path.stat().st_size
    contract["vocabulary"]["sha256"] = _sha(vocabulary_path)
    monkeypatch.setattr(subject, "EXPECTED_VOCABULARY_SHA256", contract["vocabulary"]["sha256"])

    embedding = contract["reparameterization"]["embedding"]
    embedding_path = artifacts / embedding["relative_path"]
    embedding_path.parent.mkdir(parents=True, exist_ok=True)
    np.save(embedding_path, np.zeros((4, 512), dtype=np.float32))
    embedding["byte_size"] = embedding_path.stat().st_size
    embedding["sha256"] = _sha(embedding_path)
    _write_bytes(
        artifacts,
        contract["reparameterization"]["checkpoint"],
        b"rep-checkpoint-fixture",
    )
    for index, row in enumerate(contract["upstream"]["files"]):
        _write_bytes(upstream, row, f"upstream-{index}".encode())

    artifact_manifest = {
        "schema_version": 1,
        "artifacts": {
            contract["model"]["relative_path"]: {
                "sha256": contract["model"]["sha256"],
                "byte_size": contract["model"]["byte_size"],
                "source_revision": contract["model"]["source_revision"],
                "model_role": contract["model"]["model_role"],
            },
            contract["vocabulary"]["relative_path"]: {
                "sha256": contract["vocabulary"]["sha256"],
                "byte_size": contract["vocabulary"]["byte_size"],
                "semantic_class_ids": subject.EXPECTED_CLASS_IDS,
                "emitted_labels": subject.EXPECTED_EMITTED_LABELS,
            },
        },
    }
    (artifacts / "artifact_manifest.json").write_text(
        json.dumps(artifact_manifest, indent=2) + "\n", encoding="utf-8"
    )

    discovery = {
        "official_source": {
            "oe_version": contract["toolchain"]["oe_version"],
            "archive_sha256": contract["toolchain"]["archive_sha256"],
        },
        "required_versions": subject.EXPECTED_TOOLCHAIN_VERSIONS,
        "official_toolchain_package_ready": True,
    }
    discovery_row = contract["toolchain"]["discovery_report"]
    discovery_path = repository / discovery_row["relative_path"]
    discovery_path.parent.mkdir(parents=True, exist_ok=True)
    discovery_path.write_text(json.dumps(discovery, indent=2) + "\n", encoding="utf-8")
    discovery_row["byte_size"] = discovery_path.stat().st_size
    discovery_row["sha256"] = _sha(discovery_path)
    _write_bytes(repository, contract["toolchain"]["hb_compile_help"], b"nash-m\n")

    contract["calibration"]["minimum_sample_count"] = 2
    contract["calibration"]["shape"] = [1, 3, 2, 2]
    contract["calibration"]["value_range"] = [0.0, 1.0]
    contract["preprocessing"]["tensor_shape"] = [1, 3, 2, 2]
    records = []
    for index, value in enumerate((0.25, 0.75)):
        relative = f"frame_{index}.npy"
        path = calibration / relative
        np.save(path, np.full((1, 3, 2, 2), value, dtype=np.float32))
        records.append(
            {
                "relative_path": relative,
                "byte_size": path.stat().st_size,
                "sha256": _sha(path),
                "source_sha256": hashlib.sha256(f"source-{index}".encode()).hexdigest(),
                "source_role": "calibration_only",
            }
        )
    calibration_manifest = {
        "schema_version": 1,
        "dataset_id": "fixture",
        "status": "FROZEN",
        "model_sha256": contract["model"]["sha256"],
        "vocabulary_sha256": contract["vocabulary"]["sha256"],
        "preprocessing_sha256": subject.canonical_sha256(contract["preprocessing"]),
        "evaluation_holdout_source_sha256": [],
        "records": records,
        "records_sha256": subject.canonical_sha256(records),
    }
    (calibration / contract["calibration"]["manifest_name"]).write_text(
        json.dumps(calibration_manifest, indent=2) + "\n", encoding="utf-8"
    )

    contract_path = repository / "contract.json"
    contract_path.write_text(json.dumps(contract, indent=2) + "\n", encoding="utf-8")
    identity_path = repository / "compiler_identity.json"
    identity_path.write_text(
        json.dumps(
            {
                "identity_verified": True,
                "oe_version": contract["toolchain"]["oe_version"],
                "required_versions": subject.EXPECTED_TOOLCHAIN_VERSIONS,
                "hb_compile_executable_sha256": "a" * 64,
                "hb_compile_probe_output_sha256": "b" * 64,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    assert model_path.is_file()
    return contract_path, repository, artifacts, upstream, calibration, identity_path


def test_real_contract_has_exact_frozen_shape() -> None:
    blockers: list[str] = []
    subject.validate_contract_shape(json.loads(REAL_CONTRACT.read_text(encoding="utf-8")), blockers)
    assert blockers == []


def test_real_contract_has_exact_official_upstream_file_bindings() -> None:
    contract = json.loads(REAL_CONTRACT.read_text(encoding="utf-8"))
    assert contract["upstream"]["repository_revision"] == "c50129b5badf6ed7bb85e692ab493d8bdb58da6a"
    assert contract["upstream"]["files"] == EXPECTED_UPSTREAM_FILES


def test_ready_fixture_without_real_oracle_is_blocked(tmp_path, monkeypatch) -> None:
    contract, repository, artifacts, upstream, calibration, identity = _build_ready_fixture(
        tmp_path, monkeypatch
    )
    report = subject.audit_compile_inputs(
        contract, repository, artifacts, upstream, calibration, identity
    )
    assert report["status"] == "BLOCKED"
    assert "preprocessing_oracle_missing" in report["blockers"]
    assert report["compile_plan_sha256"] is None
    assert report["hbm_status"] == "HBM_NOT_PRODUCED"
    assert report["compile_executed"] is False
    assert not (artifacts / subject.EXPECTED_OUTPUT_RELATIVE_PATH).exists()


def test_candidate_receipt_cannot_satisfy_formal_oracle(tmp_path, monkeypatch) -> None:
    contract, repository, artifacts, upstream, calibration, identity = _build_ready_fixture(tmp_path, monkeypatch)
    candidate = tmp_path / "candidate.json"
    candidate.write_text(json.dumps({"receipt_id": "tzcup_dosod_nonformal_oracle_candidate_compile_receipt_v1", "status": "NON_FORMAL_ORACLE_CANDIDATE_COMPILED"}), encoding="utf-8")
    report = subject.audit_compile_inputs(contract, repository, artifacts, upstream, calibration, identity, candidate)
    assert report["status"] == "BLOCKED"
    assert any(item.startswith("preprocessing_oracle_invalid:") for item in report["blockers"])


def test_missing_live_compiler_identity_blocks(tmp_path, monkeypatch) -> None:
    contract, repository, artifacts, upstream, calibration, _ = _build_ready_fixture(
        tmp_path, monkeypatch
    )
    oracle = tmp_path / "trusted-oracle.json"; oracle.write_text("{}", encoding="utf-8")
    monkeypatch.setattr(subject, "_validate_preprocessing_oracle", lambda _: {"model_sha256": subject.EXPECTED_MODEL_SHA256, "vocabulary_sha256": subject.EXPECTED_VOCABULARY_SHA256})
    report = subject.audit_compile_inputs(contract, repository, artifacts, upstream, calibration, None, oracle)
    assert report["status"] == "BLOCKED"
    assert "live_compiler_identity_missing" in report["blockers"]
    assert report["compile_plan_sha256"] is None


def test_unregistered_calibration_tensor_blocks(tmp_path, monkeypatch) -> None:
    contract, repository, artifacts, upstream, calibration, identity = _build_ready_fixture(
        tmp_path, monkeypatch
    )
    np.save(calibration / "unregistered.npy", np.zeros((1, 3, 2, 2), dtype=np.float32))
    oracle = tmp_path / "trusted-oracle.json"; oracle.write_text("{}", encoding="utf-8")
    monkeypatch.setattr(subject, "_validate_preprocessing_oracle", lambda _: {"model_sha256": subject.EXPECTED_MODEL_SHA256, "vocabulary_sha256": subject.EXPECTED_VOCABULARY_SHA256})
    report = subject.audit_compile_inputs(contract, repository, artifacts, upstream, calibration, identity, oracle)
    assert "calibration_directory_manifest_set_mismatch" in report["blockers"]


def test_contract_output_shape_drift_blocks() -> None:
    contract = json.loads(REAL_CONTRACT.read_text(encoding="utf-8"))
    contract["model"]["outputs"][0]["shape"][-1] = 80
    blockers: list[str] = []
    subject.validate_contract_shape(contract, blockers)
    assert "contract_model_output_signature_mismatch" in blockers


def test_relative_path_escape_is_rejected(tmp_path) -> None:
    try:
        subject.resolve_relative(tmp_path, "../escape.npy")
    except ValueError as exc:
        assert "escapes" in str(exc)
    else:
        raise AssertionError("path escape was accepted")


def test_missing_oracle_still_inventories_missing_compile_inputs(tmp_path, monkeypatch) -> None:
    contract, repository, artifacts, upstream, calibration, identity = _build_ready_fixture(tmp_path, monkeypatch)
    (artifacts / "dosod" / "dosod_mlp3x_s_tzcup_rep.onnx").unlink()
    (calibration / "calibration_manifest.json").unlink()
    report = subject.audit_compile_inputs(contract, repository, artifacts, upstream, calibration, None)
    assert report["status"] == "BLOCKED"
    assert {"preprocessing_oracle_missing", "declared_file_missing:model", "calibration_manifest_missing", "live_compiler_identity_missing"} <= set(report["blockers"])
    assert report["compile_plan_sha256"] is None


@pytest.mark.parametrize("directory_link", [False, True])
def test_declared_artifact_symlink_inside_root_is_rejected(tmp_path, directory_link) -> None:
    target = tmp_path / "real"
    target.mkdir()
    payload = target / "model.onnx"
    payload.write_bytes(b"unit-test-only")
    link = tmp_path / ("linked" if directory_link else "linked.onnx")
    try:
        link.symlink_to(target if directory_link else payload, target_is_directory=directory_link)
    except OSError as exc:
        pytest.skip(f"symlink creation unavailable: {exc}")
    relative = "linked/model.onnx" if directory_link else "linked.onnx"
    blockers = []
    result = subject.audit_declared_file(tmp_path, {"relative_path": relative, "byte_size": payload.stat().st_size, "sha256": _sha(payload)}, "model", blockers)
    assert "declared_file_symlink:model" in blockers
    assert result is None


def test_calibration_symlink_inside_root_is_rejected(tmp_path, monkeypatch) -> None:
    contract_path, _, _, _, calibration, _ = _build_ready_fixture(tmp_path, monkeypatch)
    payload = calibration / "frame_0.npy"
    target = calibration / "frame_0.data"
    payload.rename(target)
    try:
        payload.symlink_to(target)
    except OSError as exc:
        pytest.skip(f"symlink creation unavailable: {exc}")
    blockers = []
    subject.audit_calibration(calibration, json.loads(contract_path.read_text(encoding="utf-8")), blockers)
    assert "calibration_sample_symlink:frame_0.npy" in blockers


def test_vocabulary_symlink_is_not_read_after_declaration_rejection(tmp_path, monkeypatch) -> None:
    contract, repository, artifacts, upstream, calibration, identity = _build_ready_fixture(tmp_path, monkeypatch)
    vocabulary = artifacts / "dosod" / "tzcup_offline_vocabulary.json"
    target = artifacts / "dosod" / "frozen-vocabulary.json"
    vocabulary.rename(target)
    try:
        vocabulary.symlink_to(target)
    except OSError as exc:
        pytest.skip(f"symlink creation unavailable: {exc}")
    original_load_json = subject.load_json
    read_paths: list[Path] = []

    def guarded_load_json(path: Path):
        read_paths.append(Path(path))
        if Path(path) == vocabulary:
            raise AssertionError("rejected vocabulary symlink was read")
        return original_load_json(path)

    monkeypatch.setattr(subject, "load_json", guarded_load_json)
    report = subject.audit_compile_inputs(contract_path, repository, artifacts, upstream, calibration, identity)
    assert "declared_file_symlink:vocabulary" in report["blockers"]
    assert vocabulary not in read_paths


def test_embedding_symlink_is_not_loaded_after_declaration_rejection(tmp_path, monkeypatch) -> None:
    contract_path, repository, artifacts, upstream, calibration, identity = _build_ready_fixture(tmp_path, monkeypatch)
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    embedding = artifacts / contract["reparameterization"]["embedding"]["relative_path"]
    target = embedding.with_name("frozen-embedding.npy")
    embedding.rename(target)
    try:
        embedding.symlink_to(target)
    except OSError as exc:
        pytest.skip(f"symlink creation unavailable: {exc}")
    original_np_load = np.load
    loaded_paths: list[Path] = []

    def guarded_np_load(path, *args, **kwargs):
        loaded_paths.append(Path(path))
        if Path(path) == embedding:
            raise AssertionError("rejected embedding symlink was loaded")
        return original_np_load(path, *args, **kwargs)

    monkeypatch.setattr(np, "load", guarded_np_load)
    report = subject.audit_compile_inputs(contract_path, repository, artifacts, upstream, calibration, identity)
    assert "declared_file_symlink:reparameterization_embedding" in report["blockers"]
    assert embedding not in loaded_paths
