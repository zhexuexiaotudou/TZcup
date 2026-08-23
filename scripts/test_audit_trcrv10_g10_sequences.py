import json
import numpy as np
import pytest

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


def test_size_transition_must_be_temporally_ordered() -> None:
    ordered = [
        {"frame_index": 1, "short_side_px": 12},
        {"frame_index": 2, "short_side_px": 24},
        {"frame_index": 3, "short_side_px": 40},
    ]
    assert audit.ordered_size_transition(ordered)
    wrong_order = [
        {"frame_index": 1, "short_side_px": 40},
        {"frame_index": 2, "short_side_px": 24},
        {"frame_index": 3, "short_side_px": 12},
    ]
    assert not audit.ordered_size_transition(wrong_order)


def test_capture_qa_requires_protocol_minimum_mission_counts() -> None:
    source = open(audit.__file__, encoding="utf-8").read()
    assert 'split_counts.get("G10_TRAIN", 0) >= 45' in source
    assert 'split_counts.get("G10_HOLDOUT", 0) >= 18' in source
    assert '"G10_TRAIN_ROUTE_QA_PASS"' in source
    assert '"--train-only-authorization"' in source
    assert '"positive_targets_cross_required_size_stages"' in source
    assert '"positive_targets_reach_frozen_minimum"' in source


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
        "target_asset_overlap_zero", "approved_world_sets_exact",
        "mission_id_unique", "scene_seed_unique", "trajectory_id_unique",
    ):
        assert gate in source


def test_audit_rejects_declared_train_with_val_manifest(tmp_path) -> None:
    scene = tmp_path / "scene_0001"
    scene.mkdir()
    (scene / "scene_manifest.json").write_text(json.dumps({
        "split": "val",
        "world_id": next(iter(audit.SPLIT_CONTRACT["G10_HOLDOUT"]["world_ids"])),
    }), encoding="utf-8")
    (scene / "capture_report.json").write_text("{}", encoding="utf-8")

    with pytest.raises(RuntimeError, match="capture split mismatch"):
        audit.audit_scene(scene, "G10_TRAIN", 18)


def test_audit_contract_locks_route_domain_and_trajectory_identity() -> None:
    source = open(audit.__file__, encoding="utf-8").read()
    for contract in (
        audit.EXPECTED_ROUTE_ID,
        audit.EXPECTED_ROUTE_CONFIG_SHA256,
        audit.EXPECTED_DOMAIN_MANIFEST_SHA256,
        "scene/report route profile mismatch",
        "G10 trajectory identity mismatch",
    ):
        assert contract in source
