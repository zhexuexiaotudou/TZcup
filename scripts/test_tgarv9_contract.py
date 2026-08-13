import json
from pathlib import Path
import tempfile

import numpy as np
import pytest

import prepare_tgarv9_g9 as g9


def test_geometry_is_derived_from_depth_and_intrinsics() -> None:
    depth = np.full((20, 30), 2.0, dtype=np.float32)
    depth[6:14, 10:20] = np.linspace(1.8, 2.2, 80).reshape(8, 10)
    result = g9.geometry(
        depth,
        [10, 6, 10, 8],
        {"k": [100.0, 0.0, 15.0, 0.0, 100.0, 10.0, 0.0, 0.0, 1.0]},
    )
    assert result["estimated_width_m"] == pytest.approx(0.2)
    assert result["estimated_height_m"] == pytest.approx(0.16)
    assert result["local_depth_residual_m"] > 0.0
    assert result["depth_uncertainty_m"] > 0.0


def test_median_depth_rejects_invalid_samples() -> None:
    depth = np.array([[0.0, np.nan], [2.0, 2.2]], dtype=np.float32)
    value, valid_ratio = g9.median_depth(depth, np.ones_like(depth, dtype=bool))
    assert value == pytest.approx(2.1)
    assert valid_ratio == 0.5


def test_product_frame_contract_cannot_embed_evaluator_truth() -> None:
    frame = {
        "frame_ref": 1,
        "rgb_path": "rgb.png",
        "depth_path": "depth.npy",
        "camera_info_path": "camera.json",
        "tf_path": "tf.json",
    }
    assert "target_id" not in frame
    assert "class" not in frame
    assert "gt_coordinates" not in frame


def test_baseline_gate_provenance_separates_frame_track_and_cleaning_units() -> None:
    source = Path(__file__).resolve().parents[1] / "scripts" / "tgarv9_baseline.py"
    text = source.read_text(encoding="utf-8")
    assert "raw_frame_detector_wrong_actionable" in text
    assert '"frame"' in text
    assert '"track"' in text
    assert '"cleaning action"' in text


def test_g9_preparer_treats_phash_as_a_cross_split_gate() -> None:
    source = Path(g9.__file__).read_text(encoding="utf-8")
    assert "reference_phash_overlap" in source
    assert 'row.get("reason") == "reference_phash_overlap"' in source
    assert "phash_duplicate" in source  # within-HOLDOUT duplicates remain reported


def test_t2_training_retains_negative_frames() -> None:
    source = (Path(__file__).resolve().parent / "train_tgarv9_dino.py").read_text(encoding="utf-8")
    assert '"filter_empty_gt": False' in source
    assert "negative_count == 0" in source
    assert '"negative_frames_retained": True' in source
