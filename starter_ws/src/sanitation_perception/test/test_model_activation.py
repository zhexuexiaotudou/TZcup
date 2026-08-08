from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest
import yaml

from sanitation_perception.model_activation import AtomicModelActivator


ROLES = ("detector", "classifier", "leaf_segmenter", "puddle_segmenter")


def _release(root: Path, release_id: str) -> Path:
    release = root / release_id
    models = release / "models"
    manifests = release / "manifests"
    models.mkdir(parents=True)
    manifests.mkdir()
    model_files = {}
    for role in ROLES:
        artifact = models / f"{role}.onnx"
        artifact.write_bytes(role.encode())
        manifest = {
            "schema_version": 2,
            "model_id": role,
            "version": "1.0.0",
            "artifact": artifact.name,
            "artifact_sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
            "framework": "onnxruntime",
            "opset": 17,
            "license": "Apache-2.0",
            "weight_source": "test",
            "pretraining_source": "test",
            "input": {"names": ["x"], "shapes": [[1, 3, 8, 8]], "dtypes": ["float32"]},
            "normalization": {},
            "output": {"names": ["y"], "shapes": [[1, 1]], "dtypes": ["float32"]},
            "class_order": ["background"],
            "thresholds": {"score": 0.5},
            "NMS": {"classwise": False, "iou_threshold": None, "score_threshold": 0.5},
            "provider_compatibility": ["CUDAExecutionProvider"],
            "screening_pass": True,
            "formal_pass": True,
            "live_pass": False,
            "synthetic_only": True,
            "competition_claim_allowed": False,
        }
        name = f"{role}_manifest.yaml"
        (manifests / name).write_text(yaml.safe_dump(manifest), encoding="utf-8")
        model_files[role] = name
    pipeline = {
        "schema_version": 2,
        "pipeline_id": release_id,
        "model_manifests": model_files,
        "runtime": {
            "sync_tolerance_ms": 20,
            "frame_queue_depth": 2,
            "tracker_v2": {
                "association_distance_m": 0.3, "close_recovery_distance_m": 0.1,
                "minimum_image_iou": 0.05, "maximum_observation_gap_s": 0.5,
                "occlusion_recovery_s": 2.0, "duplicate_distance_m": 0.08,
                "confirmation_observations": 3, "confirmation_class_posterior": 0.7,
                "confirmation_score_ema": 0.6, "score_ema_alpha": 0.35,
                "defer_after_observations": 5,
            },
            "watchdog": {
                "camera_stale_ms": 500, "maximum_latency_ms": 200,
                "sustained_latency_samples": 5, "maximum_consecutive_tf_errors": 3,
                "maximum_consecutive_session_errors": 2,
            },
            "performance": {
                "inference_p95_ms": 150, "end_to_end_p95_ms": 200,
                "minimum_effective_hz": 10, "maximum_drop_rate": 0.01,
                "soak_duration_s": 7200,
                "maximum_memory_growth_ratio": 0.05,
            },
        },
    }
    (manifests / "perception_pipeline_manifest.yaml").write_text(
        yaml.safe_dump(pipeline), encoding="utf-8"
    )
    return release


def test_failed_warmup_does_not_switch_active_release(tmp_path: Path):
    releases = tmp_path / "releases"
    _release(releases, "v1")
    _release(releases, "v2")
    state = tmp_path / "active.json"
    activator = AtomicModelActivator(releases, state)
    activator.activate("v1", required_provider="CUDAExecutionProvider", warm_up=lambda _: None)
    before = state.read_bytes()
    with pytest.raises(RuntimeError, match="warmup"):
        activator.activate(
            "v2",
            required_provider="CUDAExecutionProvider",
            warm_up=lambda _: (_ for _ in ()).throw(RuntimeError("warmup failed")),
        )
    assert state.read_bytes() == before


def test_activation_and_rollback_are_atomic_pointer_updates(tmp_path: Path):
    releases = tmp_path / "releases"
    _release(releases, "v1")
    _release(releases, "v2")
    state = tmp_path / "active.json"
    activator = AtomicModelActivator(releases, state)
    activator.activate("v1", required_provider="CUDAExecutionProvider", warm_up=lambda _: None)
    result = activator.activate("v2", required_provider="CUDAExecutionProvider", warm_up=lambda _: None)
    assert result.previous_release_id == "v1"
    assert activator.current()["release_id"] == "v2"
    assert activator.rollback() == "v1"
    assert json.loads(state.read_text())["release_id"] == "v1"


def test_release_id_cannot_escape_root(tmp_path: Path):
    activator = AtomicModelActivator(tmp_path / "releases", tmp_path / "active.json")
    with pytest.raises(ValueError, match="safe path"):
        activator.activate("../bad", required_provider="CUDAExecutionProvider", warm_up=lambda _: None)
