from pathlib import Path


def test_moving_val_is_loaded_only_after_holdout_selection_freeze():
    source = (Path(__file__).parent / "run_crv6_native_moving_gate.py").read_text(encoding="utf-8")
    assert source.index('selection_path.write_text') < source.index('load_split(args.data_root, "MOVING_VAL"')
    assert '"MOVING_VAL_used_for_selection": False' in source


def test_native_moving_gate_contains_all_required_metrics():
    source = (Path(__file__).parent / "run_crv6_native_moving_gate.py").read_text(encoding="utf-8")
    for metric in ("eventual_detection_recall", "eventual_correct_class_recall", "small_eventual_recall", "actionable_precision", "wrong_actionable_rate", "negative_moving_actionable_rate", "AP50_95"):
        assert metric in source
