from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

import numpy as np

import validate_dosod_s100p_hbm_compile_contract as subject


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
REAL_CONTRACT = REPOSITORY_ROOT / "config" / "dosod_s100p_hbm_compile_contract.json"


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


def test_ready_fixture_emits_plan_but_never_hbm(tmp_path, monkeypatch) -> None:
    contract, repository, artifacts, upstream, calibration, identity = _build_ready_fixture(
        tmp_path, monkeypatch
    )
    report = subject.audit_compile_inputs(
        contract, repository, artifacts, upstream, calibration, identity
    )
    assert report["status"] == "READY_FOR_ONNX_TOOLCHAIN_PREFLIGHT"
    assert report["blockers"] == []
    assert len(report["compile_plan_sha256"]) == 64
    assert report["hbm_status"] == "HBM_NOT_PRODUCED"
    assert report["compile_executed"] is False
    assert not (artifacts / subject.EXPECTED_OUTPUT_RELATIVE_PATH).exists()


def test_missing_live_compiler_identity_blocks(tmp_path, monkeypatch) -> None:
    contract, repository, artifacts, upstream, calibration, _ = _build_ready_fixture(
        tmp_path, monkeypatch
    )
    report = subject.audit_compile_inputs(
        contract, repository, artifacts, upstream, calibration, None
    )
    assert report["status"] == "BLOCKED"
    assert "live_compiler_identity_missing" in report["blockers"]
    assert report["compile_plan_sha256"] is None


def test_unregistered_calibration_tensor_blocks(tmp_path, monkeypatch) -> None:
    contract, repository, artifacts, upstream, calibration, identity = _build_ready_fixture(
        tmp_path, monkeypatch
    )
    np.save(calibration / "unregistered.npy", np.zeros((1, 3, 2, 2), dtype=np.float32))
    report = subject.audit_compile_inputs(
        contract, repository, artifacts, upstream, calibration, identity
    )
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
