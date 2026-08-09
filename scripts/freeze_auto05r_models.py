#!/usr/bin/env python3
"""Create a fail-closed AUTO-05R model freeze from a passed P4 run.

The command never trains or evaluates a model.  It binds the selected
checkpoints, deployable ONNX graphs, official pretrained weight files,
screening evidence, data evidence, inference configuration and the frozen G5
evaluator into one immutable ``MODEL_FREEZE.json``.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_learning"))

from sanitation_learning.g4_manifest import (  # noqa: E402
    config_hash,
    file_sha256,
    validate_freeze_payload,
)
from sanitation_learning.g4_pretrained import (  # noqa: E402
    pretrained_backbone_spec,
)


MODEL_TYPES = ("discovery", "classifier", "leaf", "puddle")
INPUTS = {
    "discovery": {"name": "image_rgb", "shape": [1, 3, 480, 640]},
    "classifier": {"name": "crop_rgb", "shape": [1, 3, 192, 192]},
    "leaf": {"name": "area_features", "shape": [1, 10, 384, 512]},
    "puddle": {"name": "area_features", "shape": [1, 10, 384, 512]},
}
PRETRAINED_ARCHITECTURE = {
    "classifier": "mobilenet_v3_small",
    "leaf": "deeplabv3_resnet50",
    "puddle": "deeplabv3_resnet50",
}


def _load_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _discovery_pretrained_architecture(report: dict) -> str:
    architecture = report.get("student_route", {}).get(
        "discovery_architecture"
    )
    if architecture == "resnet18_fpn_a1":
        return "resnet18"
    if architecture in {
        "mobilenetv3_small_fpn_a2",
        "teacher_distilled_mobilenetv3_fpn_a3",
    }:
        return "mobilenet_v3_small"
    raise ValueError(f"unsupported frozen discovery architecture: {architecture!r}")


def _pretrained_record(architecture: str, cache_root: Path) -> dict:
    spec = pretrained_backbone_spec(architecture)
    filename = Path(spec.source_url).name
    path = cache_root / filename
    if not path.is_file():
        raise FileNotFoundError(
            f"official pretrained cache artifact missing: {path}"
        )
    actual = file_sha256(path)
    _require(
        spec.expected_sha256 is not None and actual == spec.expected_sha256,
        f"official pretrained SHA-256 mismatch for {architecture}: {actual}",
    )
    return {
        "pretrained": True,
        "from_scratch_control": False,
        "architecture": architecture,
        "weight_enum": spec.weight_enum,
        "source_url": spec.source_url,
        "license": spec.license_ref,
        "sha256": actual,
        "cache_filename": filename,
    }


def build_freeze(
    *,
    screening_report_path: Path,
    model_dir: Path,
    dataset_qa_path: Path,
    evaluator_path: Path,
    pretrained_cache_root: Path,
    source_revision: str,
    freeze_id: str,
    freeze_timestamp: str,
) -> dict:
    report = _load_json(screening_report_path)
    qa = _load_json(dataset_qa_path)
    _require(report.get("schema_version") == 2, "unsupported screening schema")
    _require(report.get("P4_SCREENING_PASS") is True, "P4 screening did not pass")
    _require(report.get("AUTO_05R_PASS") is True, "AUTO-05R screening did not pass")
    _require(
        report.get("selection", {}).get("all_product_eligible") is True,
        "not all selected checkpoints are product eligible",
    )
    _require(
        report.get("onnx_task_specific_parity_pass") is True,
        "task-specific ONNX parity did not pass",
    )
    _require(
        qa.get("G4_dataset_gate_pass") is True,
        "G4 dataset QA did not pass",
    )
    _require(evaluator_path.is_file(), f"frozen evaluator missing: {evaluator_path}")

    discovery_arch = _discovery_pretrained_architecture(report)
    pretrained_architectures = {
        "discovery": discovery_arch,
        **PRETRAINED_ARCHITECTURE,
    }
    model_config: dict[str, dict] = {}
    artifact_hashes: dict[str, str] = {}
    architecture_hashes: dict[str, str] = {}
    provenance: dict[str, dict] = {}
    onnx_contracts: dict[str, dict] = {}
    code_sha = file_sha256(
        ROOT
        / "starter_ws"
        / "src"
        / "sanitation_learning"
        / "sanitation_learning"
        / "g4_models.py"
    )
    import torch

    for task in MODEL_TYPES:
        checkpoint_path = model_dir / f"{task}.pt"
        onnx_path = model_dir / f"{task}.onnx"
        _require(checkpoint_path.is_file(), f"missing checkpoint: {checkpoint_path}")
        _require(onnx_path.is_file(), f"missing ONNX artifact: {onnx_path}")
        training = report.get("training", {}).get(task, {})
        selection = report.get("selection", {}).get(task, {})
        _require(
            selection.get("product_eligible") is True,
            f"{task} selected checkpoint is not product eligible",
        )
        contract = training.get("model_contract")
        _require(isinstance(contract, dict), f"{task} model contract is missing")
        _require(contract.get("model_id"), f"{task} model_id is missing")
        checkpoint = torch.load(
            checkpoint_path, map_location="cpu", weights_only=False
        )
        _require(
            checkpoint.get("checkpoint_status") == "training_complete",
            f"{task} checkpoint training is incomplete",
        )
        _require(
            isinstance(checkpoint.get("state_dict"), dict)
            and bool(checkpoint["state_dict"]),
            f"{task} checkpoint selected state_dict is missing",
        )
        _require(
            checkpoint.get("selection", {}).get("product_eligible") is True,
            f"{task} checkpoint selection is not product eligible",
        )
        _require(
            checkpoint.get("model_contract") == contract,
            f"{task} checkpoint model contract mismatch",
        )
        onnx = report.get("onnx", {}).get(task, {})
        _require(onnx.get("sha256") == file_sha256(onnx_path), f"{task} ONNX SHA mismatch")
        _require(onnx.get("opset") == 17, f"{task} ONNX opset must be 17")
        _require(onnx.get("fixed_input") is True, f"{task} ONNX input is not fixed")
        _require(onnx.get("custom_ops") == 0, f"{task} ONNX has custom ops")
        model_config[task] = {
            "model_id": contract["model_id"],
            "architecture_role": contract.get("architecture_role"),
            "discovery_architecture": contract.get("discovery_architecture"),
            "input_name": INPUTS[task]["name"],
            "input_shape": INPUTS[task]["shape"],
            "checkpoint": checkpoint_path.name,
            "onnx": onnx_path.name,
        }
        artifact_hashes[task] = file_sha256(onnx_path)
        architecture_hashes[task] = config_hash(
            {"model_contract": contract, "implementation_sha256": code_sha}
        )
        provenance[task] = _pretrained_record(
            pretrained_architectures[task], pretrained_cache_root
        )
        onnx_contracts[task] = {
            "passed": True,
            "fixed_input": True,
            "opset": 17,
            "custom_ops": 0,
            "operator_inventory": onnx.get("operator_inventory", {}),
            "parity": onnx.get("parity", {}),
            "sha256": artifact_hashes[task],
        }

    thresholds = report["thresholds"]
    preprocess = {
        "discovery": {"resize_wh": [640, 480], "rgb_scale": "uint8/255"},
        "classifier": {
            "resize_wh": [192, 192],
            "rgb_scale": "uint8/255",
            "context_scale": 3.0,
        },
        "leaf": {"resize_wh": [512, 384], "feature_channels": 10},
        "puddle": {"resize_wh": [512, 384], "feature_channels": 10},
    }
    postprocess = {
        "discovery": {
            "graph_external": ["local_maximum", "top_k", "nms"],
            "local_maximum_radius": 1,
            "max_detections": 100,
        },
        "classifier": {"background_index": 0, "class_order": [
            "background", "plastic_bottle", "metal_can", "paper_litter"
        ]},
        "leaf": {"mask_threshold": thresholds["area"]["leaf"]},
        "puddle": {"mask_threshold": thresholds["area"]["puddle"]},
    }
    report_sha = file_sha256(screening_report_path)
    payload = {
        "schema_version": 1,
        "freeze_id": freeze_id,
        "freeze_timestamp": freeze_timestamp,
        "config_hash": config_hash(model_config),
        "model_config": model_config,
        "architecture_hashes": architecture_hashes,
        "preprocess_hashes": {
            task: config_hash(preprocess[task]) for task in MODEL_TYPES
        },
        "postprocess_hashes": {
            task: config_hash(postprocess[task]) for task in MODEL_TYPES
        },
        "thresholds": {
            "discovery": {"score": thresholds["discovery"]},
            "classifier": {"score": thresholds["classifier"]},
            "leaf": {"mask": thresholds["area"]["leaf"]},
            "puddle": {"mask": thresholds["area"]["puddle"]},
        },
        "nms": {"discovery": {"iou": 0.5, "max_detections": 100}},
        "calibration": report["calibration"],
        "training_data_hashes": {
            "g4_dataset_qa": file_sha256(dataset_qa_path),
            "g4_dataset_contract": qa.get("contract_sha256"),
            "teacher_report": report["student_route"]["teacher_report_sha256"],
        },
        "validation_metrics": {
            "selection": report["selection"],
            "in_domain": report["in_domain_validation"],
            "cross_world": report["cross_world_validation"],
            "gates": report["gates"],
        },
        "model_artifact_hashes": artifact_hashes,
        "checkpoint_hashes": {
            task: file_sha256(model_dir / f"{task}.pt") for task in MODEL_TYPES
        },
        "pretrained_provenance": provenance,
        "onnx_contracts": onnx_contracts,
        "p4_screening": {
            "policy_id": "perception_p4_screening_policy",
            "pass": True,
            "evidence": screening_report_path.name,
            "evidence_sha256": report_sha,
        },
        "final_evaluator_sha256": file_sha256(evaluator_path),
        "source_revision": source_revision,
        "preprocess": preprocess,
        "postprocess": postprocess,
        "sealed_final": {"dataset": "G5_SEALED_FINAL", "maximum_accesses": 1},
    }
    return validate_freeze_payload(payload)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--screening-report", required=True, type=Path)
    parser.add_argument("--model-dir", required=True, type=Path)
    parser.add_argument("--dataset-qa", required=True, type=Path)
    parser.add_argument("--evaluator", required=True, type=Path)
    parser.add_argument("--pretrained-cache-root", required=True, type=Path)
    parser.add_argument("--source-revision", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--freeze-id")
    args = parser.parse_args()
    timestamp = datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace(
        "+00:00", "Z"
    )
    freeze_id = args.freeze_id or f"auto05r-{args.source_revision[:12]}-{timestamp[:10]}"
    payload = build_freeze(
        screening_report_path=args.screening_report,
        model_dir=args.model_dir,
        dataset_qa_path=args.dataset_qa,
        evaluator_path=args.evaluator,
        pretrained_cache_root=args.pretrained_cache_root,
        source_revision=args.source_revision,
        freeze_id=freeze_id,
        freeze_timestamp=timestamp,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "freeze": str(args.output),
        "freeze_id": freeze_id,
        "config_hash": payload["config_hash"],
        "p4_evidence_sha256": payload["p4_screening"]["evidence_sha256"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
