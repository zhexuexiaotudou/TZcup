#!/usr/bin/env python3
"""Fail closed unless the frozen PC perception bundle is intact and loadable."""
from __future__ import annotations

import argparse
import hashlib
import json
import stat
from pathlib import Path
from typing import Callable


REQUIRED_TOPICS = [
    "/sensors/front_rgbd/depth/image_rect_raw/image",
    "/sensors/front_rgbd/depth/image_rect_raw/depth_image",
    "/sensors/front_rgbd/depth/image_rect_raw/camera_info",
]
REQUIRED_OUTPUTS = ["/perception/garbage/targets"]
REQUIRED_ARTIFACTS = {
    "dosod/dosod_mlp3x_s_tzcup_rep.onnx": ("onnx", "c50129b5badf6ed7bb85e692ab493d8bdb58da6a", "project_four_class_dosod_pc_detector", "30e4da2516b7a18cc3dbb4b20572e99f07c28a0c08111055a8c14265a992e516", 45430396),
    "dosod/tzcup_offline_vocabulary.json": ("json", "c50129b5badf6ed7bb85e692ab493d8bdb58da6a", "frozen_project_prompt_vocabulary", "c5b10ba0e26ee28cdbf5192775e7d2ddb3f5852e515f59a074b38b7ed69d7ffd", 188),
    "edgesam/edge_sam_3x_encoder.onnx": ("onnx", "d24d99671f41a9c0003061248bded64a481e9059", "edgesam_3x_pc_image_encoder", "719a498cf5b3fe9be9f01ee513e13d3915f9028aa4f23dfd30eaaa0a17143159", 22098300),
    "edgesam/edge_sam_3x_decoder.onnx": ("onnx", "d24d99671f41a9c0003061248bded64a481e9059", "edgesam_3x_pc_box_prompt_decoder", "83a2174d54571596913dcb7455d021e713623c3dca30a31c8c41ab98c9fb0863", 15937006),
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_artifacts(artifact_root: Path, *, session_factory: Callable[[str], object] | None = None) -> dict:
    root = artifact_root.resolve()
    manifest = artifact_root / "artifact_manifest.json"
    if manifest.is_symlink() or not manifest.is_file() or not stat.S_ISREG(manifest.stat(follow_symlinks=False).st_mode):
        raise ValueError("artifact manifest must be a regular non-symlink file")
    value = json.loads(manifest.read_text(encoding="utf-8"))
    rows = value.get("artifacts") if isinstance(value, dict) else None
    if not isinstance(value, dict) or value.get("schema_version") != 1 or not isinstance(rows, dict) or not rows:
        raise ValueError("artifact manifest has invalid schema or no artifact rows")
    if session_factory is None:
        import onnxruntime as ort
        session_factory = lambda path: ort.InferenceSession(path, providers=["CPUExecutionProvider"])

    results = {}
    for relative, (kind, source_revision, model_role, pinned_hash, pinned_size) in REQUIRED_ARTIFACTS.items():
        row = rows.get(relative)
        if not isinstance(row, dict):
            raise ValueError(f"artifact manifest row missing: {relative}")
        expected_hash = row.get("sha256")
        expected_size = row.get("byte_size")
        if not isinstance(expected_hash, str) or len(expected_hash) != 64 or not isinstance(expected_size, int) or expected_size <= 0:
            raise ValueError(f"artifact manifest row invalid: {relative}")
        if row.get("source_revision") != source_revision or row.get("model_role") != model_role:
            raise ValueError(f"artifact identity mismatch: {relative}")
        if expected_hash.lower() != pinned_hash or expected_size != pinned_size:
            raise ValueError(f"artifact is not the frozen competition model: {relative}")
        path = artifact_root / relative
        if path.is_symlink() or not path.is_file() or not stat.S_ISREG(path.stat(follow_symlinks=False).st_mode):
            raise ValueError(f"artifact must be a regular non-symlink file: {relative}")
        try:
            path.resolve().relative_to(root)
        except ValueError as exc:
            raise ValueError(f"artifact escapes root: {relative}") from exc
        actual_hash = _sha256(path)
        actual_size = path.stat().st_size
        if actual_hash != expected_hash.lower() or actual_size != expected_size:
            raise ValueError(f"artifact digest or size mismatch: {relative}")
        if kind == "onnx":
            session_factory(str(path))
        else:
            vocabulary = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(vocabulary, dict) or not vocabulary:
                raise ValueError("frozen vocabulary is empty or invalid")
        results[relative] = {"sha256": actual_hash, "byte_size": actual_size, "loadable": True}
    return {"artifact_manifest_sha256": _sha256(manifest), "artifacts": results}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = {"schema_version": 2, "report_id": "tzcup_product_perception_startup_preflight_v2", "status": "BLOCKED", "required_topics": REQUIRED_TOPICS, "required_outputs": REQUIRED_OUTPUTS}
    try:
        report.update(validate_artifacts(args.artifact_root), status="READY")
    except (ImportError, OSError, ValueError, json.JSONDecodeError, RuntimeError) as exc:
        report["reason"] = f"{type(exc).__name__}:{exc}"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(report["status"])
    return 0 if report["status"] == "READY" else 2


if __name__ == "__main__":
    raise SystemExit(main())
