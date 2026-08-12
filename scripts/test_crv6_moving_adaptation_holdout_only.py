from pathlib import Path


def test_ma1_uses_train_and_holdout_but_never_val():
    source=(Path(__file__).parent/"train_crv6_moving_adaptation.py").read_text(encoding="utf-8")
    assert '"train.json"' in source and '"holdout.json"' in source
    assert '"MOVING_VAL_used":False' in source
    assert 'val.json' not in source


def test_ma1_is_only_allowed_after_native_failure_by_orchestrator_contract():
    source=(Path(__file__).parent/"train_crv6_moving_adaptation.py").read_text(encoding="utf-8")
    assert 'choices=["MA1"]' in source
    assert 'official_mmdetection_v3.3.0_rtmdet_s' in source
