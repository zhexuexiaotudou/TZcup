import hashlib
from pathlib import Path

import pytest
import yaml

from sanitation_perception.model_registry import ProductModelRegistry


ROLES = ("detector", "classifier", "leaf_segmenter", "puddle_segmenter")


def runtime():
    return {
        "backend": "onnxruntime",
        "sync_tolerance_ms": 20.0,
        "frame_queue_depth": 2,
        "preprocess_spec": "preprocess.yaml",
        "postprocess_spec": "postprocess.yaml",
        "tracker_v2": {
            "association_distance_m": 0.3,
            "close_recovery_distance_m": 0.1,
            "minimum_image_iou": 0.05,
            "maximum_observation_gap_s": 0.5,
            "occlusion_recovery_s": 2.0,
            "duplicate_distance_m": 0.08,
            "confirmation_observations": 3,
            "confirmation_class_posterior": 0.7,
            "confirmation_score_ema": 0.6,
            "score_ema_alpha": 0.35,
            "defer_after_observations": 5,
        },
        "watchdog": {
            "camera_stale_ms": 500.0,
            "maximum_latency_ms": 200.0,
            "sustained_latency_samples": 5,
            "maximum_consecutive_tf_errors": 3,
            "maximum_consecutive_session_errors": 2,
        },
        "performance": {
            "inference_p95_ms": 150.0,
            "end_to_end_p95_ms": 200.0,
            "minimum_effective_hz": 10.0,
            "maximum_drop_rate": 0.01,
            "soak_duration_s": 7200.0,
            "maximum_memory_growth_ratio": 0.05,
        },
    }


def write_registry(root: Path, *, corrupt=False, null_threshold=False):
    models = root / "models"; manifests = root / "manifests"
    models.mkdir(); manifests.mkdir()
    references = {}
    for role in ROLES:
        payload = f"onnx-{role}".encode()
        artifact = models / f"{role}.onnx"
        artifact.write_bytes(payload)
        manifest = {
            "schema_version": 2,
            "model_id": f"product_{role}",
            "version": "1.0.0",
            "artifact": artifact.name,
            "artifact_sha256": hashlib.sha256(payload).hexdigest(),
            "framework": "onnxruntime",
            "opset": 17,
            "license": "Apache-2.0",
            "weight_source": "test",
            "pretraining_source": "test",
            "input": {"names": ["images"], "shapes": [[1, 3, 8, 8]], "dtypes": ["float32"]},
            "normalization": {"scale": 1.0, "mean": [0, 0, 0], "std": [1, 1, 1]},
            "output": {"names": ["output"], "shapes": [[1, 1]], "dtypes": ["float32"]},
            "class_order": ["background", "target"],
            "thresholds": {"score": None if null_threshold and role == "detector" else 0.5},
            "NMS": {"classwise": False, "iou_threshold": None, "score_threshold": None},
            "provider_compatibility": ["CUDAExecutionProvider"],
            "screening_pass": True,
            "formal_pass": True,
            "live_pass": False,
            "synthetic_only": True,
            "competition_claim_allowed": False,
        }
        path = manifests / f"{role}.yaml"
        path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
        references[role] = path.name
    pipeline = {
        "schema_version": 2,
        "pipeline_id": "test_product_pipeline",
        "model_manifests": references,
        "runtime": runtime(),
        "status": {},
    }
    pipeline_path = manifests / "pipeline.yaml"
    pipeline_path.write_text(yaml.safe_dump(pipeline, sort_keys=False), encoding="utf-8")
    if corrupt:
        (models / "detector.onnx").write_bytes(b"corrupt")
    return pipeline_path, models


def test_registry_uses_model_id_version_sha_and_provider(tmp_path):
    pipeline, models = write_registry(tmp_path)
    registry = ProductModelRegistry.load(
        pipeline, models, required_provider="CUDAExecutionProvider"
    )
    assert set(registry.models) == set(ROLES)
    assert all(model.registry_key.count(":") == 2 for model in registry.models.values())
    assert registry.model_info()["provider"] == "CUDAExecutionProvider"


@pytest.mark.parametrize("mode,match", [("corrupt", "SHA-256"), ("threshold", "threshold")])
def test_registry_fails_closed_on_corruption_or_missing_threshold(tmp_path, mode, match):
    pipeline, models = write_registry(
        tmp_path, corrupt=mode == "corrupt", null_threshold=mode == "threshold"
    )
    with pytest.raises(ValueError, match=match):
        ProductModelRegistry.load(
            pipeline, models, required_provider="CUDAExecutionProvider"
        )
