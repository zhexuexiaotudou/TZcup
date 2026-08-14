from train_crcrv11_r1 import AUGMENTATIONS, classification_metrics, rank_metrics


def test_augmentations_are_bounded_and_exclude_v10_strong_defaults():
    assert set(AUGMENTATIONS) == {"AUG0", "AUG1", "AUG2"}
    for config in AUGMENTATIONS.values():
        assert config["color_jitter"]["hue"] <= .02
        assert config["color_jitter"]["saturation"] <= .08
        assert "random_grayscale_probability" not in config


def test_metrics_apply_formal_candidate_gate():
    truth = [0, 1, 2, 3]
    result = classification_metrics(truth, truth)
    assert result["internal_pass"] is True
    assert result["formal_pass"] is True


def test_internal_rank_prefers_passing_candidate():
    failing = {"internal_pass": False, "background_specificity": 1.0, "target_macro_f1": .94, "macro_f1": .99}
    passing = {"internal_pass": True, "background_specificity": .98, "target_macro_f1": .95, "macro_f1": .95}
    assert rank_metrics(passing) > rank_metrics(failing)
