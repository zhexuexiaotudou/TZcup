import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/rescore_formal_random_scene_perception_offline.py"
SOURCE_PACKAGE_ROOT = ROOT / "starter_ws" / "src" / "sanitation_perception"
sys.path.insert(0, str(SOURCE_PACKAGE_ROOT))

SPEC = importlib.util.spec_from_file_location("offline_rescore", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

PACKAGE_ROOT = SOURCE_PACKAGE_ROOT / "sanitation_perception"
assert Path(sys.modules["sanitation_perception"].__file__).resolve().parent == PACKAGE_ROOT.resolve()
assert (
    Path(sys.modules["sanitation_perception.formal_random_scene_evaluator"].__file__).resolve()
    == (PACKAGE_ROOT / "formal_random_scene_evaluator.py").resolve()
)


def test_frozen_truth_boxes_match_real_d435_staged_rows():
    base_from_camera = np.asarray(
        [
            [0.0, -0.423, 0.906, 0.570],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -0.906, -0.423, 0.447],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    metadata = {
        "image_shape_hwc": [480, 848, 3],
        "truth_boxes_xyxy": [
            {"object_id": f"cube-{index}", "xyxy": [0, 0, 1, 1]}
            for index in range(20)
        ],
    }
    boxes = MODULE._frozen_truth_boxes(metadata, base_from_camera)
    assert len(boxes) == 20
    # Four near-row boxes and four far-row boxes share their vertical extent.
    assert boxes[0].xyxy[1:] == pytest.approx(
        (392.0, 751.4, 417.3), abs=1.0
    )
    assert boxes[16].xyxy[1] == pytest.approx(222.0, abs=1.0)


def test_raw_anchor_overlap_counts_only_cross_class_reuse():
    raw = {
        "top_raw_candidates_by_class": {
            "litter_cube": [
                {"anchor_index": 7, "score": 0.2},
                {"anchor_index": 8, "score": 0.1},
            ],
            "puddle": [{"anchor_index": 7, "score": 0.02}],
            "fallen_leaves": [{"anchor_index": 9, "score": 0.03}],
        }
    }
    report = MODULE._raw_anchor_overlap(raw)
    assert report["top10_anchor_count"] == 3
    assert report["anchors_shared_by_multiple_classes"] == 1
    assert report["shared_anchors"][0]["anchor_index"] == 7


def test_script_is_diagnostic_only_and_freezes_product_contract():
    source = SCRIPT.read_text(encoding="utf-8")
    assert '"truth_used_to_modify_product_output": False' in source
    assert '"threshold_prompt_or_weight_changed": False' in source
    assert "CUBE_SCORE_THRESHOLD = 0.005" in source
    assert "CUBE_IOU_THRESHOLD = 0.50" in source
    assert "eligible_as_formal_product_acceptance" in source
    assert "postprocess_threshold_sweep" in source
    assert "onnxruntime" not in source


def test_build_report_aggregates_saved_counts_without_claiming_acceptance(monkeypatch):
    rows = iter(
        (
            {
                "cube_best_saved_frame_rescore": {
                    "true_positive_count": 20,
                    "false_positive_count": 1,
                    "false_negative_count": 0,
                    "precision": 20 / 21,
                    "recall": 1.0,
                    "f1": 40 / 41,
                },
                "ground_dirt_episode_rescore": {
                    "intersection_cell_count": 100,
                    "union_cell_count": 400,
                    "predicted_cell_count": 200,
                    "truth_cell_count": 250,
                    "iou": 0.25,
                    "precision": 0.5,
                    "recall": 0.4,
                },
            },
            {
                "cube_best_saved_frame_rescore": {
                    "true_positive_count": 16,
                    "false_positive_count": 1,
                    "false_negative_count": 4,
                    "precision": 16 / 17,
                    "recall": 0.8,
                    "f1": 32 / 37,
                },
                "ground_dirt_episode_rescore": {
                    "intersection_cell_count": 90,
                    "union_cell_count": 360,
                    "predicted_cell_count": 200,
                    "truth_cell_count": 250,
                    "iou": 0.25,
                    "precision": 0.45,
                    "recall": 0.36,
                },
            },
        )
    )
    monkeypatch.setattr(MODULE, "rescore_episode", lambda _path: next(rows))
    report = MODULE.build_report([Path("ep0"), Path("ep2")])
    cube = report["saved_evidence_aggregate"]["cube_two_best_frames"]
    assert cube["precision"] == pytest.approx(36 / 38)
    assert cube["recall"] == pytest.approx(36 / 40)
    assert cube["f1"] == pytest.approx(2 * (36 / 38) * (36 / 40) / ((36 / 38) + (36 / 40)))
    assert report["claim_boundary"]["eligible_as_formal_product_acceptance"] is False
