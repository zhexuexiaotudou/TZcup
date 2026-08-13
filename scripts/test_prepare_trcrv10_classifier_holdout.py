import numpy as np

import prepare_trcrv10_classifier_holdout as holdout


def test_buckets_and_depth_are_deterministic() -> None:
    assert holdout.size_bucket(17) == "lt18"
    assert holdout.size_bucket(64) == "64_96"
    assert holdout.distance_bucket(1.5) == "1_2"
    depth = np.array([[0.0, 2.0], [4.0, np.nan]], dtype=np.float32)
    assert holdout.median_depth(depth, [0, 0, 2, 2]) == 3.0
    stats = holdout.depth_statistics(
        depth, [0, 0, 2, 2], {"k": [500.0, 0, 0, 0, 500.0, 0, 0, 0, 1]}
    )
    assert stats["valid_fraction"] == .5
    assert stats["median_m"] == 3.0
    assert stats["projection_covariance_m2"] > 0


def test_holdout_is_runtime_proposal_only_and_sealed_safe() -> None:
    source = open(holdout.__file__, encoding="utf-8").read()
    assert "G10 val/HOLDOUT only" in source
    assert '"gt_role": "offline_label_assignment_only"' in source
    assert '"production_runtime_eligible": True' in source
    assert '"G10_DEV_VAL_SEALED_read": False' in source
    assert '"proposal_index": proposal_index' in source
    assert '"depth_valid_fraction"' in source
    assert '"projection_covariance_m2"' in source
