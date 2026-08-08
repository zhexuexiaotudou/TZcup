from __future__ import annotations

import hashlib
from pathlib import Path
import zipfile

import yaml

from package_perception_release import MODEL_ROLES, package


def _formal_bundle(tmp_path: Path) -> tuple[Path, Path]:
    artifacts = tmp_path / "models"
    manifests = tmp_path / "manifests"
    artifacts.mkdir(); manifests.mkdir()
    names = {}
    for role in MODEL_ROLES:
        artifact = artifacts / f"{role}.onnx"
        artifact.write_bytes(role.encode())
        manifest = {
            "schema_version": 2, "model_id": role, "version": "1.0.0",
            "artifact": artifact.name,
            "artifact_sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
            "framework": "onnxruntime", "opset": 17, "license": "Apache-2.0",
            "weight_source": "test", "pretraining_source": "test",
            "input": {"names": ["x"], "shapes": [[1, 3, 8, 8]], "dtypes": ["float32"]},
            "normalization": {},
            "output": {"names": ["y"], "shapes": [[1, 1]], "dtypes": ["float32"]},
            "class_order": ["background"], "thresholds": {"score": 0.5},
            "NMS": {"classwise": False, "iou_threshold": None, "score_threshold": 0.5},
            "provider_compatibility": ["CUDAExecutionProvider"],
            "screening_pass": True, "formal_pass": True, "live_pass": False,
            "synthetic_only": True, "competition_claim_allowed": False,
        }
        name = f"{role}_manifest.yaml"
        (manifests / name).write_text(yaml.safe_dump(manifest), encoding="utf-8")
        names[role] = name
    pipeline = {
        "schema_version": 2, "pipeline_id": "test-pipeline", "model_manifests": names,
        "runtime": {"sync_tolerance_ms": 20, "frame_queue_depth": 2,
          "tracker_v2": {"association_distance_m": .3, "close_recovery_distance_m": .1,
            "minimum_image_iou": .05, "maximum_observation_gap_s": .5,
            "occlusion_recovery_s": 2., "duplicate_distance_m": .08,
            "confirmation_observations": 3, "confirmation_class_posterior": .7,
            "confirmation_score_ema": .6, "score_ema_alpha": .35,
            "defer_after_observations": 5},
          "watchdog": {"camera_stale_ms": 500., "maximum_latency_ms": 200.,
            "sustained_latency_samples": 5, "maximum_consecutive_tf_errors": 3,
            "maximum_consecutive_session_errors": 2}},
    }
    pipeline_path = manifests / "perception_pipeline_manifest.yaml"
    pipeline_path.write_text(yaml.safe_dump(pipeline), encoding="utf-8")
    return pipeline_path, artifacts


def test_release_contains_required_layout_and_checksums(tmp_path: Path):
    pipeline, artifacts = _formal_bundle(tmp_path)
    archive, digest = package(
        pipeline, artifacts, tmp_path / "out",
        required_provider="CUDAExecutionProvider", commit="a" * 40,
    )
    assert digest.read_text().startswith(hashlib.sha256(archive.read_bytes()).hexdigest())
    with zipfile.ZipFile(archive) as source:
        names = set(source.namelist())
    required = {
        "TZcup_perception_product/models/detector.onnx",
        "TZcup_perception_product/manifests/perception_pipeline_manifest.yaml",
        "TZcup_perception_product/configs/preprocess_spec.yaml",
        "TZcup_perception_product/launch/stage5a_perception.launch.py",
        "TZcup_perception_product/licenses/LICENSE.md",
        "TZcup_perception_product/SBOM.spdx.json",
        "TZcup_perception_product/SHA256SUMS",
        "TZcup_perception_product/environment.lock",
        "TZcup_perception_product/README.md",
        "TZcup_perception_product/rollback/README.md",
    }
    assert required <= names
