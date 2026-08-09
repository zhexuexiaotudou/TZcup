#!/usr/bin/env python3
"""Generate runtime manifest v2 files from an immutable model freeze.

P4 is sufficient only for ``screening_pass``.  ``formal_pass`` is set only
when an append-only P5 result for the same freeze explicitly records a pass;
live and competition claims remain false until their later evidence gates.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_perception"))

from sanitation_learning.g4_manifest import file_sha256, load_freeze  # noqa: E402
from sanitation_perception.pipeline_manifest import (  # noqa: E402
    load_pipeline_manifest,
    validate_model_manifest,
)


ROLE_TO_TASK = {
    "detector": "discovery",
    "classifier": "classifier",
    "leaf_segmenter": "leaf",
    "puddle_segmenter": "puddle",
}
OUTPUTS = {
    "discovery": {"names": ["outputs"], "shapes": [[1, 15, 120, 160]]},
    "classifier": {"names": ["outputs"], "shapes": [[1, 4]]},
    "leaf": {"names": ["outputs"], "shapes": [[1, 2, 384, 512]]},
    "puddle": {"names": ["outputs"], "shapes": [[1, 2, 384, 512]]},
}
CLASS_ORDER = {
    "discovery": ["class_agnostic_candidate"],
    "classifier": [
        "background", "plastic_bottle", "metal_can", "paper_litter"
    ],
    "leaf": ["background", "leaf_pile"],
    "puddle": ["background", "puddle"],
}


def _load_json(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return payload


def _formal_pass(freeze: dict, p5_result_path: Path | None) -> tuple[bool, str | None]:
    if p5_result_path is None:
        return False, None
    result = _load_json(p5_result_path)
    if result.get("freeze_id") != freeze["freeze_id"]:
        raise ValueError("P5 result freeze_id does not match MODEL_FREEZE.json")
    if result.get("one_shot") is not True or result.get("rerun_allowed") is not False:
        raise ValueError("P5 result is not an append-only one-shot record")
    if result.get("metrics", {}).get("P5_FINAL_PASS") is not True:
        raise ValueError("P5 sealed-final result did not pass")
    return True, file_sha256(p5_result_path)


def generate(
    *,
    freeze_path: Path,
    model_dir: Path,
    output_dir: Path,
    p5_result_path: Path | None = None,
) -> dict:
    freeze = load_freeze(freeze_path)
    formal_pass, p5_sha = _formal_pass(freeze, p5_result_path)
    output_dir.mkdir(parents=True, exist_ok=True)
    filenames = {}
    for role, task in ROLE_TO_TASK.items():
        config = freeze["model_config"][task]
        artifact = model_dir / config["onnx"]
        if not artifact.is_file():
            raise FileNotFoundError(f"frozen ONNX artifact missing: {artifact}")
        artifact_sha = file_sha256(artifact)
        if artifact_sha != freeze["model_artifact_hashes"][task]:
            raise ValueError(f"frozen ONNX SHA-256 mismatch for {task}")
        threshold = freeze["thresholds"][task]
        nms = freeze.get("nms", {}).get(task, {})
        provenance = freeze["pretrained_provenance"][task]
        manifest = {
            "schema_version": 2,
            "model_id": config["model_id"],
            "version": freeze["freeze_id"],
            "model_status": "available",
            "artifact": artifact.name,
            "artifact_sha256": artifact_sha,
            "framework": "onnxruntime",
            "opset": 17,
            "license": provenance["license"],
            "weight_source": "official_torchvision_pretrained_then_g4_finetuned",
            "pretraining_source": {
                "weight_enum": provenance["weight_enum"],
                "source_url": provenance["source_url"],
                "sha256": provenance["sha256"],
            },
            "input": {
                "names": [config["input_name"]],
                "shapes": [config["input_shape"]],
                "dtypes": ["float32"],
            },
            "normalization": {
                "scale": 1.0 / 255.0 if task in ("discovery", "classifier") else 1.0,
                "mean": [0.0, 0.0, 0.0],
                "std": [1.0, 1.0, 1.0],
                "preprocess_hash": freeze["preprocess_hashes"][task],
            },
            "output": {**OUTPUTS[task], "dtypes": ["float32"]},
            "class_order": CLASS_ORDER[task],
            "thresholds": dict(threshold),
            "NMS": {
                "classwise": False,
                "iou_threshold": nms.get("iou"),
                "score_threshold": threshold.get("score"),
                "graph_external": task == "discovery",
            },
            "provider_compatibility": [
                "CUDAExecutionProvider", "CPUExecutionProvider", "horizon_j6"
            ],
            "screening_pass": True,
            "formal_pass": formal_pass,
            "live_pass": False,
            "synthetic_only": True,
            "competition_claim_allowed": False,
            "freeze_id": freeze["freeze_id"],
            "freeze_config_hash": freeze["config_hash"],
            "p4_evidence_sha256": freeze["p4_screening"]["evidence_sha256"],
            "p5_result_sha256": p5_sha,
        }
        errors = validate_model_manifest(manifest, artifact_root=model_dir)
        if errors:
            raise ValueError(f"generated {role} manifest invalid: {'; '.join(errors)}")
        filename = f"{role}_manifest.yaml"
        (output_dir / filename).write_text(
            yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8"
        )
        filenames[role] = filename

    base = yaml.safe_load(
        (
            ROOT
            / "starter_ws"
            / "src"
            / "sanitation_perception"
            / "config"
            / "perception_pipeline_manifest.yaml"
        ).read_text(encoding="utf-8")
    )
    base["pipeline_id"] = f"perception_pipeline_{freeze['freeze_id']}"
    base["model_manifests"] = filenames
    base["freeze_id"] = freeze["freeze_id"]
    base["freeze_config_hash"] = freeze["config_hash"]
    base["status"] = {
        "screening_pipeline_pass": True,
        "formal_pipeline_pass": formal_pass,
        "live_pipeline_pass": False,
        "competition_pipeline_claim_allowed": False,
    }
    pipeline_path = output_dir / "perception_pipeline_manifest.yaml"
    pipeline_path.write_text(yaml.safe_dump(base, sort_keys=False), encoding="utf-8")
    load_pipeline_manifest(pipeline_path)
    return {
        "pipeline": str(pipeline_path),
        "freeze_id": freeze["freeze_id"],
        "screening_pass": True,
        "formal_pass": formal_pass,
        "model_manifests": filenames,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--freeze", required=True, type=Path)
    parser.add_argument("--model-dir", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--p5-result", type=Path)
    args = parser.parse_args()
    result = generate(
        freeze_path=args.freeze,
        model_dir=args.model_dir,
        output_dir=args.output_dir,
        p5_result_path=args.p5_result,
    )
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
