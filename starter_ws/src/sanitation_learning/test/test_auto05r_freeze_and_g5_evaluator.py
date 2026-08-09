from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import types

import numpy as np
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[4]


def _load_script(name: str):
    path = ROOT / "scripts" / name
    spec = importlib.util.spec_from_file_location(name.removesuffix(".py"), path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_g5_frozen_evaluator_ap_is_one_for_perfect_ranked_predictions() -> None:
    evaluator = _load_script("auto05r_g5_frozen_evaluator.py")
    frames = []
    for class_index, class_name in enumerate(evaluator.DISCRETE_CLASSES, start=1):
        truth = {
            "semantic_class": class_name,
            "bbox_xyxy": [10.0, 10.0, 30.0, 30.0],
        }
        prediction = {
            "class_index": class_index,
            "class_name": class_name,
            "score": 0.99,
            "bbox_xyxy": [10.0, 10.0, 30.0, 30.0],
        }
        frames.append({"truth": [truth], "predictions": [prediction]})
    assert evaluator._average_precision(frames, 0.5) == pytest.approx(1.0)
    assert evaluator._average_precision(frames, 0.95) == pytest.approx(1.0)


def test_g5_dataset_loader_derives_discrete_truth_from_native_masks(
    tmp_path: Path,
) -> None:
    evaluator = _load_script("auto05r_g5_frozen_evaluator.py")
    scene = tmp_path / "scenes" / "scene_5000"
    for name in ("rgb", "depth", "semantic", "instance", "camera", "tf"):
        (scene / name).mkdir(parents=True, exist_ok=True)
    semantic = np.zeros((12, 16), dtype=np.uint8)
    instance = np.zeros_like(semantic, dtype=np.uint16)
    semantic[2:8, 3:11] = 3
    instance[2:8, 3:11] = 42
    np.save(scene / "semantic" / "frame_00.npy", semantic)
    np.save(scene / "instance" / "frame_00.npy", instance)
    np.save(scene / "depth" / "frame_00.npy", np.ones_like(semantic, dtype=np.float32))
    (scene / "rgb" / "frame_00.png").write_bytes(b"placeholder")
    (scene / "camera" / "frame_00.json").write_text("{}")
    (scene / "tf" / "frame_00.json").write_text("{}")
    (scene / "scene_manifest.json").write_text(json.dumps({
        "split": "G5_SEALED_FINAL",
        "scene_seed": 5000,
        "world_id": "world_g5_test",
        "negative_only": False,
        "objects": [{
            "class_id": "paper_litter",
            "distance_bucket_m": [0.5, 2.0],
            "size_bucket": "small",
        }],
    }))
    (scene / "capture_report.json").write_text(json.dumps({
        "records": [{
            "frame_index": 0,
            "paths": {
                "rgb": "rgb/frame_00.png",
                "depth": "depth/frame_00.npy",
                "semantic": "semantic/frame_00.npy",
                "instance": "instance/frame_00.npy",
                "camera": "camera/frame_00.json",
                "tf": "tf/frame_00.json",
            },
        }],
    }))
    rows, instances, metadata = evaluator._load_dataset_rows(tmp_path)
    assert len(rows) == 1
    assert instances == [{
        "scene_seed": 5000,
        "frame_index": 0,
        "semantic_class": "paper_litter",
        "bbox_xyxy_px": [3, 2, 11, 8],
        "bbox_shortest_side_px": 6,
        "mask_area_px": 48,
    }]
    assert metadata[(5000, 0)]["distance_buckets"] == ["0.5_2.0"]


def test_g5_taxonomy_specificity_uses_worst_eligible_family() -> None:
    evaluator = _load_script("auto05r_g5_frozen_evaluator.py")
    frames = []
    metadata = {}
    for index in range(10):
        frames.append({
            "negative_only": True,
            "scene_seed": 5000,
            "frame_index": index,
            "detections": ([{"score": 0.9}] if index == 0 else []),
        })
        metadata[(5000, index)] = {
            "negative_taxonomies": [
                "paper_like_road_patch" if index < 5 else "shadow_edge"
            ]
        }
    report = evaluator._taxonomy_specificity(frames, metadata)
    assert report["per_taxonomy"]["paper_like_road_patch"]["specificity"] == 0.8
    assert report["per_taxonomy"]["shadow_edge"]["specificity"] == 1.0
    assert report["specificity"] == 0.8


def test_freeze_generator_refuses_nonpassing_p4(tmp_path: Path) -> None:
    freezer = _load_script("freeze_auto05r_models.py")
    report = tmp_path / "report.json"
    report.write_text(json.dumps({
        "schema_version": 2,
        "P4_SCREENING_PASS": False,
        "AUTO_05R_PASS": False,
    }))
    qa = tmp_path / "qa.json"
    qa.write_text(json.dumps({"G4_dataset_gate_pass": True}))
    evaluator = tmp_path / "evaluator.py"
    evaluator.write_text("def evaluate_sealed_final(**kwargs): return {}\n")
    with pytest.raises(ValueError, match="P4 screening did not pass"):
        freezer.build_freeze(
            screening_report_path=report,
            model_dir=tmp_path,
            dataset_qa_path=qa,
            evaluator_path=evaluator,
            pretrained_cache_root=tmp_path,
            source_revision="1" * 40,
            freeze_id="freeze-test",
            freeze_timestamp="2026-08-09T00:00:00Z",
        )


def test_freeze_generator_binds_complete_checkpoints_and_official_weights(
    tmp_path: Path, monkeypatch,
) -> None:
    freezer = _load_script("freeze_auto05r_models.py")
    model_dir = tmp_path / "models"
    cache = tmp_path / "cache"
    model_dir.mkdir()
    cache.mkdir()
    selection = {"product_eligible": True, "selected_epoch": 2}
    selections = {"all_product_eligible": True}
    training = {}
    onnx = {}
    for task in freezer.MODEL_TYPES:
        contract = {
            "model_id": f"g4_{task}_v1",
            "architecture_role": f"{task}_role",
            "discovery_architecture": (
                "resnet18_fpn_a1" if task == "discovery" else None
            ),
        }
        (model_dir / f"{task}.pt").write_text(json.dumps({
            "checkpoint_status": "training_complete",
            "state_dict": {"weight": [1.0]},
            "selection": selection,
            "model_contract": contract,
        }))
        (model_dir / f"{task}.onnx").write_bytes(task.encode())
        selections[task] = selection
        training[task] = {"selection": selection, "model_contract": contract}
        onnx[task] = {
            "sha256": freezer.file_sha256(model_dir / f"{task}.onnx"),
            "opset": 17,
            "fixed_input": True,
            "custom_ops": 0,
            "operator_inventory": {"Conv": 1},
            "parity": {"max_absolute_error": 0.0},
        }
    report = tmp_path / "report.json"
    report.write_text(json.dumps({
        "schema_version": 2,
        "P4_SCREENING_PASS": True,
        "AUTO_05R_PASS": True,
        "selection": selections,
        "training": training,
        "onnx": onnx,
        "onnx_task_specific_parity_pass": True,
        "thresholds": {
            "discovery": 0.8,
            "classifier": 0.75,
            "area": {"leaf": 0.8, "puddle": 0.9},
        },
        "calibration": {"selection_split": "val"},
        "student_route": {
            "discovery_architecture": "resnet18_fpn_a1",
            "teacher_report_sha256": "b" * 64,
        },
        "in_domain_validation": {},
        "cross_world_validation": {},
        "gates": {},
    }))
    qa = tmp_path / "qa.json"
    qa.write_text(json.dumps({
        "G4_dataset_gate_pass": True,
        "contract_sha256": "c" * 64,
    }))
    evaluator = tmp_path / "evaluator.py"
    evaluator.write_text("def evaluate_sealed_final(**kwargs): return {}\n")

    specs = {}
    for architecture in (
        "resnet18", "mobilenet_v3_small", "deeplabv3_resnet50"
    ):
        path = cache / f"{architecture}.pth"
        path.write_bytes(architecture.encode())
        specs[architecture] = type("Spec", (), {
            "weight_enum": f"{architecture}.V1",
            "source_url": f"https://example.invalid/{architecture}.pth",
            "license_ref": "test-license",
            "expected_sha256": freezer.file_sha256(path),
        })()
    monkeypatch.setattr(
        freezer, "pretrained_backbone_spec", lambda architecture: specs[architecture]
    )
    monkeypatch.setitem(sys.modules, "torch", types.SimpleNamespace(
        load=lambda path, **_: json.loads(Path(path).read_text())
    ))
    freeze = freezer.build_freeze(
        screening_report_path=report,
        model_dir=model_dir,
        dataset_qa_path=qa,
        evaluator_path=evaluator,
        pretrained_cache_root=cache,
        source_revision="1" * 40,
        freeze_id="freeze-test",
        freeze_timestamp="2026-08-09T00:00:00Z",
    )
    assert freeze["p4_screening"]["pass"] is True
    assert freeze["checkpoint_hashes"]["discovery"]
    assert freeze["pretrained_provenance"]["leaf"]["pretrained"] is True


def test_product_manifests_keep_formal_false_until_matching_p5(
    tmp_path: Path, monkeypatch,
) -> None:
    generator = _load_script("generate_perception_product_manifests.py")
    model_dir = tmp_path / "models"
    model_dir.mkdir()
    tasks = ("discovery", "classifier", "leaf", "puddle")
    model_config = {}
    artifact_hashes = {}
    provenance = {}
    thresholds = {
        "discovery": {"score": 0.8},
        "classifier": {"score": 0.75},
        "leaf": {"mask": 0.8},
        "puddle": {"mask": 0.9},
    }
    for task in tasks:
        artifact = model_dir / f"{task}.onnx"
        artifact.write_bytes(task.encode())
        model_config[task] = {
            "model_id": f"g4_{task}_v1",
            "input_name": f"{task}_input",
            "input_shape": [1, 3, 4, 4],
            "onnx": artifact.name,
        }
        artifact_hashes[task] = generator.file_sha256(artifact)
        provenance[task] = {
            "license": "test-license",
            "weight_enum": f"{task}.V1",
            "source_url": f"https://example.invalid/{task}.pth",
            "sha256": "a" * 64,
        }
    freeze = {
        "freeze_id": "freeze-test",
        "config_hash": "b" * 64,
        "model_config": model_config,
        "model_artifact_hashes": artifact_hashes,
        "pretrained_provenance": provenance,
        "preprocess_hashes": {task: "c" * 64 for task in tasks},
        "thresholds": thresholds,
        "nms": {"discovery": {"iou": 0.5}},
        "p4_screening": {"evidence_sha256": "d" * 64},
    }
    monkeypatch.setattr(generator, "load_freeze", lambda _: freeze)
    screening = generator.generate(
        freeze_path=tmp_path / "MODEL_FREEZE.json",
        model_dir=model_dir,
        output_dir=tmp_path / "screening",
    )
    assert screening["screening_pass"] is True
    assert screening["formal_pass"] is False
    p5 = tmp_path / "sealed_final_result.json"
    p5.write_text(json.dumps({
        "freeze_id": "freeze-test",
        "one_shot": True,
        "rerun_allowed": False,
        "metrics": {"P5_FINAL_PASS": True},
    }))
    formal = generator.generate(
        freeze_path=tmp_path / "MODEL_FREEZE.json",
        model_dir=model_dir,
        output_dir=tmp_path / "formal",
        p5_result_path=p5,
    )
    assert formal["formal_pass"] is True
    detector = yaml.safe_load(
        (tmp_path / "formal" / "detector_manifest.yaml").read_text()
    )
    assert detector["artifact_sha256"] == artifact_hashes["discovery"]
    assert detector["live_pass"] is False
