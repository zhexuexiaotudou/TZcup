from pathlib import Path

import train_trcrv10_close_range_classifier as classifier


def test_model_and_class_search_space_is_bounded() -> None:
    assert classifier.PRIMARY_MODEL == "convnext_tiny"
    assert classifier.CONTROL_MODEL == "mobilenet_v3_large"
    assert classifier.CLASSES[-1] == "background_or_unknown"


def test_classifier_gate_exactly_matches_protocol() -> None:
    result = classifier.classification_metrics([0, 1, 2, 3], [0, 1, 2, 3])
    assert result["pass"]
    assert set(result["gates"]) == {
        "macro_f1", "each_target_precision", "each_target_recall", "background_specificity",
        "paper_precision", "metal_recall",
    }


def test_wrong_target_is_fail_closed() -> None:
    result = classifier.classification_metrics([0, 1, 2, 3], [1, 1, 2, 3])
    assert not result["pass"]


def test_background_specificity_is_background_recall_and_macro_has_four_classes() -> None:
    # All targets correct, but one of two background rows is called bottle.
    result = classifier.classification_metrics([0, 1, 2, 3, 3], [0, 1, 2, 3, 0])
    assert result["metrics"]["background_specificity"] == .5
    assert result["metrics"]["macro_f1"] < 1.0
    assert not result["gates"]["background_specificity"]


def test_sealed_boundaries_and_c1_first_are_explicit() -> None:
    source = Path(classifier.__file__).read_text(encoding="utf-8")
    assert 'default=PRIMARY_MODEL' in source
    assert '"G10_DEV_VAL_SEALED_read": False' in source
    assert '"VAL_NEW_read": False' in source
    assert '"G5_V2_read": False' in source
