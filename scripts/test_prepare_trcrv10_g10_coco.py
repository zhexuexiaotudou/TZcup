import numpy as np

import prepare_trcrv10_g10_coco as prepare


def test_annotations_preserve_three_class_gt_for_offline_use() -> None:
    mask = np.zeros((20, 30), dtype=np.uint8)
    mask[2:8, 3:12] = 1
    mask[10:19, 15:25] = 3
    rows, next_id = prepare.annotations(mask, image_id=4, next_id=9)
    assert [row["category_id"] for row in rows] == [1, 3]
    assert rows[0]["bbox"] == [3, 2, 9, 6]
    assert rows[0]["bbox_short_side_px"] == 6
    assert next_id == 11


def test_categories_are_fixed() -> None:
    assert [row["name"] for row in prepare.CATEGORIES] == [
        "plastic_bottle", "metal_can", "paper_litter"
    ]
