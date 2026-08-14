import numpy as np

from build_crcrv11_c11_dataset import (
    assign_development_partitions, clip_crop, ground_taxonomy, phash, proposal_label,
)


def test_proposal_label_separates_ambiguous_near_miss():
    truth = [{"class_id": "metal_can", "bbox": [0, 0, 10, 10]}]
    assert proposal_label([0, 0, 10, 10], truth)[0] == "metal_can"
    assert proposal_label([0, 0, 4, 10], truth)[0] == "AMBIGUOUS_NEAR_MISS"
    assert proposal_label([20, 20, 30, 30], truth)[0] == "background_or_unknown"


def test_ground_taxonomy_is_world_deterministic():
    assert ground_taxonomy("g10_wet_courtyard", 0) == "wet_specular_highlight"
    assert ground_taxonomy("g10_asphalt", 1) == "seam_crack"


def test_development_partition_is_whole_mission_and_class_complete():
    pairs = []
    for class_id in ("plastic_bottle", "metal_can", "paper_litter", "background_or_unknown"):
        for mission in range(5):
            pairs.append({"class": class_id, "source_mission": f"{class_id}_{mission}"})
    selected = assign_development_partitions(pairs)
    assert selected
    for class_id in ("plastic_bottle", "metal_can", "paper_litter", "background_or_unknown"):
        assert any(row["class"] == class_id and row["development_partition"] == "dev" for row in pairs)
        assert any(row["class"] == class_id and row["development_partition"] == "fit" for row in pairs)


def test_phash_and_crop_are_deterministic():
    image = np.arange(20 * 20 * 3, dtype=np.uint8).reshape(20, 20, 3)
    crop = clip_crop(image, [2, 3, 12, 15])
    assert crop.shape == (12, 10, 3)
    assert phash(crop) == phash(crop.copy())
