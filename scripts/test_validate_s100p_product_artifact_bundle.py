"""Tests for the local-only S100P product artifact preparation checker."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "validate_s100p_product_artifact_bundle.py"
SPEC = importlib.util.spec_from_file_location("s100p_bundle", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _digest(path: Path) -> tuple[str, int]:
    return hashlib.sha256(path.read_bytes()).hexdigest(), path.stat().st_size


def _complete_bundle(tmp_path: Path) -> tuple[Path, Path, Path]:
    manifest = json.loads(
        (ROOT / "config" / "s100p_product_artifact_bundle.json").read_text(
            encoding="utf-8"
        )
    )
    repository = tmp_path / "repository"
    artifacts = tmp_path / "artifacts"
    manifest_path = tmp_path / "bundle.json"
    manifest["status"] = "PREPARED_NOT_DEPLOYED"
    for name, row in manifest["assets"].items():
        if name == "board_runtime_manifest":
            continue
        root = artifacts if row["source_scope"] == "artifact_root" else repository
        target = root / row["relative_path"]
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(f"formal-{name}".encode("utf-8"))
        row["sha256"], row["byte_size"] = _digest(target)

    runtime_rows = {}
    for name, (relative, role, revision) in MODULE.MODEL_ASSETS.items():
        row = manifest["assets"][name]
        runtime_rows[relative] = {
            "sha256": row["sha256"],
            "byte_size": row["byte_size"],
            "model_role": role,
            "source_revision": revision,
        }
    runtime_path = artifacts / "artifact_manifest.json"
    runtime_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "board_runtime_contract": MODULE.EXPECTED_RUNTIME,
                "artifacts": runtime_rows,
            }
        ),
        encoding="utf-8",
    )
    runtime = manifest["assets"]["board_runtime_manifest"]
    runtime["sha256"], runtime["byte_size"] = _digest(runtime_path)
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    return manifest_path, artifacts, repository


def test_real_preparation_manifest_is_fail_closed_without_dosod_hbm():
    report = MODULE.validate_bundle()
    assert report["ready"] is False
    assert report["status"] == "BLOCKED"
    assert "asset_missing:board_runtime_manifest" in report["blockers"]
    assert "board_runtime_manifest_missing" in report["blockers"]
    assert "asset_missing:dosod_hbm" in report["blockers"]


def test_complete_local_bundle_is_prepared_but_not_deployed(tmp_path: Path):
    manifest, artifacts, repository = _complete_bundle(tmp_path)
    report = MODULE.validate_bundle(
        manifest, artifact_root=artifacts, repository_root=repository
    )
    assert report["ready"] is True
    assert report["status"] == "PREPARED_NOT_DEPLOYED"
    assert report["operation"] == "prepare_only_no_deployment"
    assert report["asset_results"]["dosod_hbm"]["board_target_path"].startswith(
        "/opt/tzcup/s100p/artifacts/"
    )


def test_digest_drift_reblocks_a_prepared_local_bundle(tmp_path: Path):
    manifest, artifacts, repository = _complete_bundle(tmp_path)
    target = artifacts / "edgesam/edgesam_encoder_512.hbm"
    target.write_bytes(b"tampered")
    report = MODULE.validate_bundle(
        manifest, artifact_root=artifacts, repository_root=repository
    )
    assert report["ready"] is False
    assert "asset_digest_or_size_mismatch:edgesam_encoder_hbm" in report["blockers"]


def test_validator_has_no_process_network_or_copy_implementation():
    source = SCRIPT.read_text(encoding="utf-8")
    for forbidden in ("import subprocess", "import socket", "shutil.copy", "os.system("):
        assert forbidden not in source
