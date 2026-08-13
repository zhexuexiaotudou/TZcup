import numpy as np

import audit_trcrv10_g10_sequences as audit


def test_bbox_row_and_buckets() -> None:
    mask = np.zeros((120, 120), dtype=np.uint8)
    mask[10:50, 20:90] = 2
    row = audit.bbox_row(mask, 2, 7)
    assert row == {
        "frame_index": 7,
        "bbox_xyxy": [20, 10, 90, 50],
        "short_side_px": 40,
        "pixels": 2800,
    }
    counts = audit.bucket_counts([row, {**row, "short_side_px": 17}, {**row, "short_side_px": 96}])
    assert counts == {"<18": 1, "18-32": 0, "32-64": 1, "64-96": 0, ">=96": 1}


def test_missing_label_is_not_a_bbox() -> None:
    assert audit.bbox_row(np.zeros((4, 4), dtype=np.uint8), 1, 0) is None


def test_capture_qa_requires_protocol_minimum_mission_counts() -> None:
    source = open(audit.__file__, encoding="utf-8").read()
    assert 'split_counts.get("G10_TRAIN", 0) >= 45' in source
    assert 'split_counts.get("G10_HOLDOUT", 0) >= 18' in source


def test_capture_qa_emits_all_protocol_artifacts_and_identity_gates() -> None:
    source = open(audit.__file__, encoding="utf-8").read()
    for name in (
        "G10_APPROACH_SEQUENCE_STATS.json",
        "G10_SIZE_TRANSITION_STATS.json",
        "G10_HARD_NEGATIVE_MATRIX.json",
        "G10_SPLIT_MANIFEST.json",
    ):
        assert name in source
    for gate in (
        "world_overlap_zero", "seed_overlap_zero", "trajectory_overlap_zero",
        "target_asset_overlap_zero",
    ):
        assert gate in source
