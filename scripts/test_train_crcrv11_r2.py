from train_crcrv11_r2 import binary_metrics, select_binary_threshold, target_metrics


def test_binary_gate_requires_both_recall_and_specificity():
    result = binary_metrics([True, True, False, False], [.9, .8, .1, .2], .5)
    assert result["pass"] is True
    failed = binary_metrics([True, True, False, False], [.9, .4, .1, .2], .5)
    assert failed["pass"] is False


def test_threshold_selection_is_dev_only_deterministic():
    selected = select_binary_threshold([True, True, False, False], [.9, .8, .1, .2])
    assert selected["pass"] is True
    assert .2 < selected["threshold"] <= .8


def test_three_class_gate():
    result = target_metrics([0, 1, 2], [0, 1, 2])
    assert result["pass"] is True
