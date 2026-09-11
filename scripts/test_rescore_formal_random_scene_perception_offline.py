import importlib.util
import json
from pathlib import Path
import sys
import subprocess

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/rescore_formal_random_scene_perception_offline.py"
SOURCE_PACKAGE_ROOT = ROOT / "starter_ws" / "src" / "sanitation_perception"
EVALUATOR_PACKAGE_ROOT = ROOT / "starter_ws" / "src" / "sanitation_perception_evaluator"
sys.path.insert(0, str(SOURCE_PACKAGE_ROOT))
sys.path.insert(0, str(EVALUATOR_PACKAGE_ROOT))

SPEC = importlib.util.spec_from_file_location("offline_rescore", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

PACKAGE_ROOT = SOURCE_PACKAGE_ROOT / "sanitation_perception"
assert Path(sys.modules["sanitation_perception"].__file__).resolve().parent == PACKAGE_ROOT.resolve()
assert (
    Path(sys.modules["sanitation_perception_evaluator.formal_random_scene_evaluator"].__file__).resolve()
    == (EVALUATOR_PACKAGE_ROOT / "sanitation_perception_evaluator/formal_random_scene_evaluator.py").resolve()
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
                    "union_cell_count": 350,
                    "predicted_cell_count": 200,
                    "truth_cell_count": 250,
                    "iou": 100 / 350,
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


def test_cli_help_works_without_ros_or_pythonpath(tmp_path):
    import os

    environment = dict(os.environ)
    environment.pop("PYTHONPATH", None)
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--help"], cwd=tmp_path,
        env=environment, capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 0, result.stderr
    assert "--episode-root" in result.stdout


def test_empty_or_repeated_episodes_do_not_create_capability_claims():
    with pytest.raises(ValueError, match="at least one"):
        MODULE.build_report([])
    with pytest.raises(ValueError, match="duplicate"):
        MODULE.build_report([Path("ep0"), Path("ep0")])


@pytest.mark.parametrize("counts", [
    (100, 400, 200, 250),  # Union must equal 350.
    (201, 249, 200, 250),
    (-1, 451, 200, 250),
    (True, 449, 200, 250),
    (100.5, 349.5, 200, 250),
])
def test_corrupt_ground_confusion_counts_are_rejected(counts):
    ground = dict(zip(("intersection_cell_count", "union_cell_count",
                       "predicted_cell_count", "truth_cell_count"), counts))
    with pytest.raises(ValueError, match="ground confusion counts"):
        MODULE._validated_ground_counts(ground)


def test_ground_confusion_counts_preserve_valid_empty_and_nonempty_values():
    for counts in ((0, 0, 0, 0), (100, 350, 200, 250)):
        ground = dict(zip(("intersection_cell_count", "union_cell_count",
                           "predicted_cell_count", "truth_cell_count"), counts))
        assert MODULE._validated_ground_counts(ground) == counts


@pytest.fixture
def synthetic_saved_episode(tmp_path):
    """Synthetic parser fixture only: never real Gazebo or acceptance evidence."""
    import hashlib

    root = tmp_path / "synthetic-unit-test-episode"
    root.mkdir()
    image = root / "best_front_frame.png"
    # The rescoring path hashes bytes; it does not decode an image. This marker
    # deliberately cannot be mistaken for an exported simulator camera frame.
    image.write_bytes(b"SYNTHETIC UNIT TEST INPUT - NOT A REAL CAMERA IMAGE")
    metadata = {
        "fixture_only": True,
        "image_shape_hwc": [480, 848, 3],
        "truth_boxes_xyxy": [
            {"object_id": f"synthetic-cube-{index}", "xyxy": [0, 0, 1, 1]}
            for index in range(20)
        ],
    }
    (root / "best_front_frame.json").write_text(json.dumps(metadata), encoding="utf-8")
    (root / "tf2_echo_base_link_front_rgbd_depth_optical_frame.txt").write_text(
        "SYNTHETIC UNIT TEST TRANSFORM\n- Matrix:\n"
        "0.0 -0.423 0.906 0.570\n-1.0 0.0 0.0 0.0\n0.0 -0.906 -0.423 0.447\n",
        encoding="utf-8",
    )
    raw = {
        "fixture_only": True,
        "claim_boundary": {"evaluator_only_offline_diagnostic": True},
        # These flags exercise parser requirements, not a claim of real data.
        "input": {"real_gazebo_camera_frame": True,
                  "image_sha256": hashlib.sha256(image.read_bytes()).hexdigest()},
        "postprocess_threshold_sweep": {"0.005": {"detections": []}},
    }
    (root / "dosod_raw_diagnostic.json").write_text(json.dumps(raw), encoding="utf-8")
    acceptance = {
        "episode_id": "synthetic-unit-test-only",
        "ground_dirt_segmentation": {
            "intersection_cell_count": 6, "union_cell_count": 12,
            "predicted_cell_count": 8, "truth_cell_count": 10,
        },
    }
    (root / "perception_acceptance.json").write_text(json.dumps(acceptance), encoding="utf-8")
    return root


def test_rescore_episode_saved_layout_reports_misses_and_metadata_hash(synthetic_saved_episode):
    import hashlib

    root = synthetic_saved_episode
    report = MODULE.rescore_episode(root)
    cube = report["cube_best_saved_frame_rescore"]
    assert cube["false_negative_count"] == 20
    assert cube["unmatched_truth_object_ids"] == [f"synthetic-cube-{index}" for index in range(20)]
    assert cube["precision"] == cube["recall"] == cube["f1"] == 0.0
    ground = report["ground_dirt_episode_rescore"]
    assert ground["missed_truth_cell_count"] == 4
    assert ground["false_positive_cell_count"] == 2
    assert ground["iou"] == 0.5
    assert ground["recall"] == 0.6
    assert ground["missed_area_m2"] is None
    assert report["inputs"]["frame_metadata_sha256"] == hashlib.sha256(
        (root / "best_front_frame.json").read_bytes()
    ).hexdigest()
    assert MODULE.build_report([root])["claim_boundary"]["eligible_as_formal_product_acceptance"] is False


@pytest.mark.parametrize("sweep", [{}, {"0.010": {"detections": []}}, {"0.005": {}},
                                  {"0.005": {"detections": None}}])
def test_rescore_episode_rejects_missing_frozen_sweep(synthetic_saved_episode, sweep):
    path = synthetic_saved_episode / "dosod_raw_diagnostic.json"
    raw = json.loads(path.read_text(encoding="utf-8"))
    raw["postprocess_threshold_sweep"] = sweep
    path.write_text(json.dumps(raw), encoding="utf-8")
    with pytest.raises(ValueError, match="frozen 0.005"):
        MODULE.rescore_episode(synthetic_saved_episode)


def test_rescore_episode_rejects_inconsistent_saved_ground(synthetic_saved_episode):
    path = synthetic_saved_episode / "perception_acceptance.json"
    acceptance = json.loads(path.read_text(encoding="utf-8"))
    acceptance["ground_dirt_segmentation"]["union_cell_count"] = 11
    path.write_text(json.dumps(acceptance), encoding="utf-8")
    with pytest.raises(ValueError, match="ground confusion counts are inconsistent"):
        MODULE.rescore_episode(synthetic_saved_episode)
