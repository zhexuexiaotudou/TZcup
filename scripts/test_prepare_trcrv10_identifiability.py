import numpy as np

import prepare_trcrv10_identifiability as prep


def test_identifiability_size_buckets() -> None:
    assert [prep.bucket(value) for value in (11, 12, 17, 18, 31, 32, 47, 48, 63, 64, 95, 96)] == [
        "lt12", "12_18", "12_18", "18_32", "18_32", "32_48",
        "32_48", "48_64", "48_64", "64_96", "64_96", "ge96",
    ]


def test_identifiability_bbox_and_context() -> None:
    mask = np.zeros((100, 120), dtype=np.uint8)
    mask[20:60, 30:80] = 2
    box = prep.bbox(mask, 2)
    assert box == (30, 20, 80, 60)
    assert prep.expanded_box(box, 120, 100) == (15, 8, 95, 72)


def test_identifiability_rejects_sealed_split_by_contract() -> None:
    assert prep.ALLOWED_SPLITS == {"TRAIN_DIAG", "HOLDOUT_DIAG"}
    assert "G10_DEV_VAL_SEALED" not in prep.ALLOWED_SPLITS
