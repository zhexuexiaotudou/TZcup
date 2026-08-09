"""Hash-verified product model registry assembled from pipeline manifests."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

from sanitation_perception.pipeline_manifest import (
    PIPELINE_MODEL_ROLES,
    load_model_manifest,
    load_pipeline_manifest,
    validate_model_manifest,
)


@dataclass(frozen=True)
class ProductModel:
    role: str
    model_id: str
    version: str
    sha256: str
    artifact_path: Path
    manifest_path: Path
    manifest: dict

    @property
    def registry_key(self) -> str:
        return f"{self.model_id}:{self.version}:{self.sha256}"


@dataclass(frozen=True)
class ProductModelRegistry:
    pipeline_id: str
    pipeline_sha256: str
    required_provider: str
    models: dict[str, ProductModel]

    @classmethod
    def load(
        cls,
        pipeline_path: str | Path,
        artifact_root: str | Path,
        *,
        required_provider: str,
        required_claim: str = "formal",
    ) -> "ProductModelRegistry":
        pipeline_path = Path(pipeline_path)
        artifact_root = Path(artifact_root)
        pipeline = load_pipeline_manifest(pipeline_path)
        models = {}
        claim_field = {
            "screening": "screening_pass",
            "formal": "formal_pass",
            "live": "live_pass",
        }.get(required_claim)
        if claim_field is None:
            raise ValueError(f"unsupported registry claim: {required_claim}")
        for role in PIPELINE_MODEL_ROLES:
            manifest_path = pipeline_path.parent / pipeline["model_manifests"][role]
            manifest = load_model_manifest(manifest_path)
            errors = validate_model_manifest(manifest, artifact_root=artifact_root)
            if errors:
                raise ValueError(f"{role} manifest invalid: {'; '.join(errors)}")
            if not manifest.get("artifact"):
                raise ValueError(f"{role} artifact is unavailable")
            if manifest.get(claim_field) is not True:
                raise ValueError(f"{role} does not satisfy required claim {required_claim}")
            if required_provider not in manifest.get("provider_compatibility", []):
                raise ValueError(f"{role} does not support provider {required_provider}")
            thresholds = manifest.get("thresholds", {})
            if not thresholds or any(value is None for value in thresholds.values()):
                raise ValueError(f"{role} has missing product thresholds")
            _validate_runtime_contract_hashes(role, manifest)
            artifact_path = artifact_root / manifest["artifact"]
            models[role] = ProductModel(
                role=role,
                model_id=manifest["model_id"],
                version=manifest["version"],
                sha256=manifest["artifact_sha256"].lower(),
                artifact_path=artifact_path,
                manifest_path=manifest_path,
                manifest=manifest,
            )
        keys = [model.registry_key for model in models.values()]
        if len(keys) != len(set(keys)):
            raise ValueError("product registry model identity collision")
        return cls(
            pipeline_id=pipeline["pipeline_id"],
            pipeline_sha256=hashlib.sha256(pipeline_path.read_bytes()).hexdigest(),
            required_provider=required_provider,
            models=models,
        )

    def model_info(self) -> dict:
        return {
            "pipeline_id": self.pipeline_id,
            "pipeline_sha256": self.pipeline_sha256,
            "provider": self.required_provider,
            "models": {
                role: {
                    "model_id": model.model_id,
                    "version": model.version,
                    "sha256": model.sha256,
                    "registry_key": model.registry_key,
                }
                for role, model in sorted(self.models.items())
            },
        }


def _config_hash(value: object) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _validate_runtime_contract_hashes(role: str, manifest: dict) -> None:
    task = {
        "detector": "discovery",
        "classifier": "classifier",
        "leaf_segmenter": "leaf",
        "puddle_segmenter": "puddle",
    }[role]
    preprocess = {
        "discovery": {"resize_wh": [640, 480], "rgb_scale": "uint8/255"},
        "classifier": {
            "resize_wh": [192, 192],
            "rgb_scale": "uint8/255",
            "context_scale": 3.0,
        },
        "leaf": {"resize_wh": [512, 384], "feature_channels": 10},
        "puddle": {"resize_wh": [512, 384], "feature_channels": 10},
    }[task]
    class_order = [
        "background", "plastic_bottle", "metal_can", "paper_litter"
    ]
    if task == "discovery":
        postprocess = {
            "graph_external": ["local_maximum", "top_k", "nms"],
            "local_maximum_radius": 1,
            "max_detections": 100,
        }
    elif task == "classifier":
        postprocess = {"background_index": 0, "class_order": class_order}
    else:
        postprocess = {"mask_threshold": manifest["thresholds"]["mask"]}
    declared_preprocess = manifest.get("normalization", {}).get(
        "preprocess_hash"
    )
    declared_postprocess = manifest.get("postprocess_hash")
    if declared_preprocess is None or declared_postprocess is None:
        raise ValueError(f"{role} runtime contract hashes are missing")
    if declared_preprocess != _config_hash(preprocess):
        raise ValueError(f"{role} preprocess contract hash mismatch")
    if declared_postprocess != _config_hash(postprocess):
        raise ValueError(f"{role} postprocess contract hash mismatch")
