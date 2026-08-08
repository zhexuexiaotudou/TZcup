from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
if str(_PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_DIR))


from sanitation_learning import g4_teacher  # noqa: E402
from sanitation_learning.g4_pretrained import torchvision_cache_path  # noqa: E402


def test_teacher_dataset_gate_requires_new_integrity_gates(tmp_path) -> None:
    qa_path = tmp_path / "g4_dataset_qa.json"
    base = {
        "G4_dataset_gate_pass": True,
        "quality_gates_pass": True,
        "full_capture_executed": True,
        "gates": {
            "scene_pose_reset_contract_100_percent": True,
            "manifest_pixel_target_consistency_100_percent": True,
        },
    }
    qa_path.write_text(json.dumps(base), encoding="utf-8")
    accepted = g4_teacher.require_teacher_dataset_gate(tmp_path)
    assert accepted["G4_dataset_gate_pass"] is True
    broken = {**base, "gates": {"scene_pose_reset_contract_100_percent": True}}
    qa_path.write_text(json.dumps(broken), encoding="utf-8")
    with pytest.raises(RuntimeError, match="missing=.*manifest_pixel"):
        g4_teacher.require_teacher_dataset_gate(tmp_path)


def test_teacher_dataset_is_rgb_only_and_supports_empty_targets(monkeypatch) -> None:
    pytest.importorskip("torch")
    monkeypatch.setattr(
        g4_teacher,
        "read_rgb",
        lambda _row: np.zeros((480, 640, 3), dtype=np.uint8),
    )
    monkeypatch.setattr(
        g4_teacher,
        "discrete_boxes_for_frame",
        lambda *_args, **_kwargs: [],
    )
    dataset = g4_teacher.FCOSDiscoveryDataset(
        [{"scene_seed": 1, "frame_index": 2}], {}
    )
    image, target, _ = dataset[0]
    assert tuple(image.shape) == (3, 480, 640)
    assert tuple(target["boxes"].shape) == (0, 4)
    assert tuple(target["labels"].shape) == (0,)


def test_teacher_gate_requires_recall_and_low_flood() -> None:
    passed = g4_teacher.teacher_gate(
        {"all_gt_candidate_recall": 0.86, "false_candidates_per_min": 9.0}
    )
    assert passed["all_pass"] is True
    failed = g4_teacher.teacher_gate(
        {"all_gt_candidate_recall": 0.84, "false_candidates_per_min": 11.0}
    )
    assert failed["all_pass"] is False
    assert not any(failed["gates"].values())


def test_filter_prediction_frames_uses_frozen_threshold() -> None:
    frames = [
        {
            "detections": [
                {"score": 0.1, "bbox_xyxy": [0, 0, 1, 1]},
                {"score": 0.8, "bbox_xyxy": [1, 1, 2, 2]},
            ]
        }
    ]
    filtered = g4_teacher.filter_prediction_frames(frames, 0.5)
    assert [item["score"] for item in filtered[0]["detections"]] == [0.8]
    assert len(frames[0]["detections"]) == 2


def test_teacher_threshold_selection_prefers_feasible_operating_point() -> None:
    truth = [{"bbox_xyxy": [0.0, 0.0, 10.0, 10.0]}]
    frames = []
    for index in range(100):
        frames.append(
            {
                "truth": truth if index < 90 else [],
                "negative_only": index >= 90,
                "detections": (
                    [{"score": 0.8, "bbox_xyxy": [0.0, 0.0, 10.0, 10.0]}]
                    if index < 90
                    else []
                ),
            }
        )
    threshold, selected, sweep = g4_teacher.select_teacher_threshold(frames)
    assert selected["all_pass"] is True
    assert selected["metrics"]["all_gt_candidate_recall"] == 1.0
    assert threshold <= 0.8
    assert len(sweep) == len(g4_teacher.TEACHER_THRESHOLDS)


def test_build_teacher_uses_verified_official_weights_when_cached() -> None:
    pytest.importorskip("torch")
    pytest.importorskip("torchvision")
    cache = torchvision_cache_path(g4_teacher.TEACHER_WEIGHT_SPEC)
    if not cache.is_file():
        pytest.skip("official FCOS cache is intentionally not downloaded by tests")
    model = g4_teacher.build_fcos_teacher()
    assert model.architecture_role == "reference_teacher_not_default_deployable"
    assert model.head.classification_head.num_classes == 1
    assert model.head.classification_head.cls_logits.out_channels == 1
    assert model.provenance["sha256"] == (
        "99b0c9b7cfb1527d782db86b91d207f00547c792fb4103fc612b651d0a07b9e7"
    )
