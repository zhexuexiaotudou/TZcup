from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest


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
