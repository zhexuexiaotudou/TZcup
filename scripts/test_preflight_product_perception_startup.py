import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("perception_preflight", ROOT / "scripts" / "preflight_product_perception_startup.py")
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def bundle(tmp_path: Path) -> Path:
    root = tmp_path / "assets"
    rows = {}
    for relative, (kind, source_revision, model_role, pinned_hash, pinned_size) in MODULE.REQUIRED_ARTIFACTS.items():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        data = (
            json.dumps([["litter cube"], ["fallen leaves"], ["dust patch"], ["puddle"]]).encode()
            if kind == "json"
            else ("model:" + relative).encode()
        )
        path.write_bytes(data)
        rows[relative] = {"sha256": hashlib.sha256(data).hexdigest(), "byte_size": len(data), "source_revision": source_revision, "model_role": model_role}
        MODULE.REQUIRED_ARTIFACTS[relative] = (kind, source_revision, model_role, rows[relative]["sha256"], len(data))
    (root / "artifact_manifest.json").write_text(json.dumps({"schema_version": 1, "artifacts": rows}), encoding="utf-8")
    return root


def test_valid_bundle_checks_all_models_and_topics(tmp_path):
    root = bundle(tmp_path)
    loaded = []
    report = MODULE.validate_artifacts(root, session_factory=loaded.append)
    assert len(loaded) == 3
    assert len(report["artifacts"]) == 4
    assert MODULE.REQUIRED_TOPICS == [
        "/sensors/front_rgbd/depth/image_rect_raw/image",
        "/sensors/front_rgbd/depth/image_rect_raw/depth_image",
        "/sensors/front_rgbd/depth/image_rect_raw/camera_info",
    ]


def test_empty_manifest_is_blocked(tmp_path):
    root = tmp_path / "assets"
    root.mkdir()
    (root / "artifact_manifest.json").write_text("{}", encoding="utf-8")
    with pytest.raises(ValueError, match="invalid schema"):
        MODULE.validate_artifacts(root, session_factory=lambda _: None)


def test_vocabulary_must_be_four_nonempty_label_groups(tmp_path):
    root = bundle(tmp_path)
    relative = "dosod/tzcup_offline_vocabulary.json"
    path = root / relative
    data = json.dumps({"classes": ["litter_cube"]}).encode()
    path.write_bytes(data)
    manifest_path = root / "artifact_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["artifacts"][relative].update(
        sha256=hashlib.sha256(data).hexdigest(), byte_size=len(data)
    )
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    kind, revision, role, _, _ = MODULE.REQUIRED_ARTIFACTS[relative]
    MODULE.REQUIRED_ARTIFACTS[relative] = (
        kind,
        revision,
        role,
        hashlib.sha256(data).hexdigest(),
        len(data),
    )
    with pytest.raises(ValueError, match="vocabulary is empty or invalid"):
        MODULE.validate_artifacts(root, session_factory=lambda _: None)


def test_tampered_model_is_blocked(tmp_path):
    root = bundle(tmp_path)
    (root / "dosod" / "dosod_mlp3x_s_tzcup_rep.onnx").write_bytes(b"tampered")
    with pytest.raises(ValueError, match="digest or size mismatch"):
        MODULE.validate_artifacts(root, session_factory=lambda _: None)


def test_manifest_and_model_cannot_drift_together(tmp_path):
    root = bundle(tmp_path)
    relative = "dosod/dosod_mlp3x_s_tzcup_rep.onnx"
    path = root / relative
    data = b"different-but-self-consistent-model"
    path.write_bytes(data)
    manifest_path = root / "artifact_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["artifacts"][relative].update(sha256=hashlib.sha256(data).hexdigest(), byte_size=len(data))
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    kind, revision, role, _, _ = MODULE.REQUIRED_ARTIFACTS[relative]
    MODULE.REQUIRED_ARTIFACTS[relative] = (kind, revision, role, "0" * 64, 1)
    with pytest.raises(ValueError, match="not the frozen competition model"):
        MODULE.validate_artifacts(root, session_factory=lambda _: None)


def test_symlink_model_is_blocked(tmp_path):
    root = bundle(tmp_path)
    path = root / "edgesam" / "edge_sam_3x_encoder.onnx"
    target = root / "target.onnx"
    target.write_bytes(path.read_bytes())
    path.unlink()
    try:
        path.symlink_to(target)
    except OSError:
        pytest.skip("symlink creation unavailable")
    with pytest.raises(ValueError, match="non-symlink"):
        MODULE.validate_artifacts(root, session_factory=lambda _: None)
