import pytest

from c4_native_worker import WorkerError, classification_metrics, reject_forbidden_path


def test_c4_worker_guards_forbidden_data_names():
    for path in ("/data/G5/train", "/data/G5_V2", "/data/VAL_NEW", "/data/SEALED_FINAL"):
        with pytest.raises(WorkerError):
            reject_forbidden_path(path)
    reject_forbidden_path("/data/g10/train")


def test_c4_worker_metrics_are_fail_closed_for_missing_target_recall():
    rows = [
        {"actual_product_class": "background", "predicted_product_class": "background"},
        {"actual_product_class": "plastic_bottle", "predicted_product_class": "background"},
        {"actual_product_class": "metal_can", "predicted_product_class": "metal_can"},
        {"actual_product_class": "paper_litter", "predicted_product_class": "paper_litter"},
    ]
    metrics = classification_metrics(rows)
    assert metrics["background_specificity"] == 1.0
    assert metrics["per_class"]["plastic_bottle"]["recall"] == 0.0
    assert metrics["macro_f1"] < 1.0
