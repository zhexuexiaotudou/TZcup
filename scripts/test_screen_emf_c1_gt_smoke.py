import numpy as np

from screen_emf_c1_gt_smoke import classification_metrics, classifier_preprocess


def test_c1_preprocess_uses_rgb_nearest_and_unit_scale():
    image = np.zeros((4, 4, 3), dtype=np.uint8)
    image[:, :, 0] = 255
    tensor = classifier_preprocess(image, [0, 0, 4, 4])
    assert tensor.shape == (1, 3, 224, 224)
    assert tensor.dtype == np.float32
    assert tensor[0, 0, 0, 0] == 0.0
    assert tensor[0, 2, 0, 0] == 1.0


def test_c1_smoke_metrics_keep_background_and_macro_explicit():
    rows = [
        {"actual_product_class": "background", "predicted_product_class": "background"},
        {"actual_product_class": "plastic_bottle", "predicted_product_class": "plastic_bottle"},
        {"actual_product_class": "metal_can", "predicted_product_class": "background"},
        {"actual_product_class": "paper_litter", "predicted_product_class": "paper_litter"},
    ]
    metrics = classification_metrics(rows)
    assert metrics["background_specificity"] == 1.0
    assert metrics["per_class"]["metal_can"]["recall"] == 0.0
    assert metrics["macro_f1"] < 1.0
