import numpy as np

from evaluate_crcrv11_five_view import background_metrics, crop_array, entropy, metrics


def test_crop_array_clips_like_product_writer():
    image = np.zeros((10, 20, 3), dtype=np.uint8)
    assert crop_array(image, [-2, -4, 5, 7]).shape == (7, 5, 3)


def test_target_metrics_are_not_penalized_by_absent_background_class():
    rows = [
        {"truth": "plastic_bottle", "predicted": "plastic_bottle", "confidence": .9, "entropy": .1},
        {"truth": "metal_can", "predicted": "metal_can", "confidence": .8, "entropy": .2},
        {"truth": "paper_litter", "predicted": "paper_litter", "confidence": .7, "entropy": .3},
    ]
    result = metrics(rows, ("plastic_bottle", "metal_can", "paper_litter"))
    assert result["macro_f1"] == 1.0
    assert result["accuracy"] == 1.0


def test_background_specificity_is_background_recall():
    rows = [
        {"predicted": "background_or_unknown", "confidence": .8, "entropy": .2},
        {"predicted": "paper_litter", "confidence": .6, "entropy": .4},
    ]
    assert background_metrics(rows)["background_specificity"] == .5


def test_entropy_is_zero_for_certain_distribution():
    assert entropy(np.asarray([1.0, 0.0, 0.0, 0.0])) < 1e-8
